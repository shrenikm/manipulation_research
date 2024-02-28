from enum import Enum
from typing import Optional, Sequence

import attr
import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.class_utils import StrEnum
from python.common.control.constructs import PIDGains
from python.common.model_utils import ObjectModelConfig
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    Lite6PliantType,
    add_gripper_positions_and_velocities_to_lite6_state,
    create_lite6_state,
    get_gripper_status_from_lite6_state,
    get_joint_positions_from_lite6_state,
    get_joint_velocities_from_lite6_state,
    get_lite6_num_positions,
    get_lite6_num_states,
    get_lite6_num_velocities,
)

LITE6_PLIANT_DEFAULT_MESHCAT_PORT = 7000

LITE6_PLIANT_SUPPORTED_MODEL_TYPES = (
    Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
    Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
)


# Positions desired
LITE6_PLIANT_PD_IP_NAME = "pd_input"
# Velocities desired
LITE6_PLIANT_VD_IP_NAME = "vd_input"
# Gripper status desired
LITE6_PLIANT_GSD_IP_NAME = "gsd_input"
# State estimated
LITE6_PLIANT_SE_IP_NAME = "se_input"

LITE6_PLIANT_PD_OP_NAME = "pd_output"
LITE6_PLIANT_VD_OP_NAME = "vd_output"
LITE6_PLIANT_GSD_OP_NAME = "gsd_output"

LITE6_PLIANT_SD_OP_NAME = "sd_output"
LITE6_PLIANT_PE_OP_NAME = "pe_output"
LITE6_PLIANT_VE_OP_NAME = "ve_output"
LITE6_PLIANT_GSE_OP_NAME = "gse_output"

LITE6_PLIANT_MULTIPLEXER_PREFIX = "mult_"
LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_PD_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VD_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GSD_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_SE_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_SD_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_PE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_PE_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VE_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GSE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GSE_OP_NAME
)


@attr.frozen
class Lite6PliantConfig:
    lite6_model_type: Lite6ModelType
    lite6_control_type: Lite6ControlType
    lite6_pliant_type: Lite6PliantType
    inverse_dynamics_pid_gains: PIDGains
    plant_config: MultibodyPlantConfig
    object_model_configs: Optional[Sequence[ObjectModelConfig]] = None


class Lite6PliantMultiplexer(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self.num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )
        self.pd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME,
            size=LITE6_DOF,
        )
        self.vd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME,
            size=LITE6_DOF,
        )
        self.gsd_input_port = self.DeclareAbstractInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME,
            model_value=Value(Lite6GripperStatus.CLOSED),
        )

        self.sd_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME,
            size=self.num_total_states,
            calc=self._compute_state_desired_output,
        )

    def _compute_state_desired_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_output_vector = np.zeros(self.num_total_states, dtype=np.float64)

        state_estimated_vector = self.se_input_port.Eval(context)
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        if self.config.lite6_control_type == Lite6ControlType.STATE:
            # For full state control, we need to route the desired positions and
            # velocities into the non gripper state values of the output.
            # The gripper values are set according to the desired flag. Desired gripper
            # velocity will be 0 with the positions being open/closed according to the
            # GSD port input.
            state_output_vector = create_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                positions_vector=positions_desired_vector,
                velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            # For velocity control, we route the desired velocities into the output
            # state's velocities. The output's positions are obtained from the input
            # estimated positions state. This is the only case where we use the SE input
            # port value.
            positions_estimated_vector = get_joint_positions_from_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                state_vector=state_estimated_vector,
            )
            state_output_vector = create_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                positions_vector=positions_estimated_vector,
                velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        else:
            raise NotImplementedError

        # If the desired gripper position is neutral,
        #state_output_vector = add_gripper_positions_and_velocities_to_lite6_state(
        #    lite6_model_type=self.config.lite6_model_type,
        #    state_vector=state_output_vector,
        #    gripper_positions=
        #)

        output_vector.SetFromVector(
            value=state_output_vector,
        )


class Lite6PliantDeMultiplexer(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self.num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )

        self.pe_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_positions_estimated_output,
        )
        self.ve_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_velocities_estimated_output,
        )
        self.gse_output_port = self.DeclareAbstractOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GSE_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_estimated_output,
        )

    def _compute_positions_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        positions_output_vector = get_joint_positions_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=positions_output_vector,
        )

    def _compute_velocities_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        velocities_output_vector = get_joint_velocities_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=velocities_output_vector,
        )

    def _compute_gripper_status_estimated_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        lite6_gripper_status = get_gripper_status_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )

        output_value.set_value(lite6_gripper_status)
