from enum import Enum
from typing import Optional

import attr
import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.class_utils import StrEnum
from python.common.control.constructs import PIDGains
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    create_lite6_state,
    get_gripper_status_from_lite6_state,
    get_joint_positions_from_lite6_state,
    get_joint_velocities_from_lite6_state,
    get_lite6_num_positions,
    get_lite6_num_states,
    get_lite6_num_velocities,
)

LITE6_PLIANT_SUPPORTED_MODEL_TYPES = (
    Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
    Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
)


LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME = "positions_desired_input"
LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME = "velocities_desired_input"
LITE6_PLIANT_GRIPPER_STATUS_DESIRED_IP_NAME = "gripper_status_desired_input"

LITE6_PLIANT_STATE_ESTIMATED_IP_NAME = "state_estimated_input"
LITE6_PLIANT_STATE_DESIRED_OP_NAME = "state_desired_output"
LITE6_PLIANT_POSITIONS_ESTIMATED_OP_NAME = "positions_estimated_output"
LITE6_PLIANT_VELOCITIES_ESTIMATED_OP_NAME = "velocities_estimated_output"
LITE6_PLIANT_GRIPPER_STATUS_ESTIMATED_OP_NAME = "gripper_status_estimated_output"

LITE6_PLIANT_MULTIPLEXER_PREFIX = "mult_"
LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GRIPPER_STATUS_DESIRED_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_STATE_ESTIMATED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GRIPPER_STATUS_DESIRED_IP_NAME
)


@attr.frozen
class Lite6PliantConfig:
    lite6_model_type: Lite6ModelType
    lite6_control_type: Lite6ControlType
    run_on_hardware: bool
    time_step_s: float
    inverse_dynamics_pid_gains: PIDGains
    plant_config: Optional[MultibodyPlantConfig] = None


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

        self._se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )
        self._pd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME,
            size=LITE6_DOF,
        )
        self._vd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME,
            size=LITE6_DOF,
        )
        self._gsd_input_port = self.DeclareAbstractInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME,
            model_value=Value(False),
        )

        self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_STATE_DESIRED_OP_NAME,
            size=self.num_total_states,
            calc=self.compute_state_desired_output,
        )

    def compute_state_desired_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_output_vector = np.zeros(self.num_total_states, dtype=np.float64)

        state_estimated_vector = self._se_input_port.Eval(context)
        positions_desired_vector = self._pd_input_port.Eval(context)
        velocities_desired_vector = self._vd_input_port.Eval(context)
        gripper_status_desired = self._gsd_input_port.Eval(context)

        # TODO: Remove
        assert isinstance(positions_desired_vector, np.ndarray)
        assert isinstance(gripper_status_desired_bool, bool)

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

        self._se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )

        self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_POSITIONS_ESTIMATED_OP_NAME,
            size=LITE6_DOF,
            calc=self.compute_positions_estimated_output,
        )
        self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_VELOCITIES_ESTIMATED_OP_NAME,
            size=LITE6_DOF,
            calc=self.compute_velocities_estimated_output,
        )
        self.DeclareAbstractOutputPort(
            name=LITE6_PLIANT_GRIPPER_STATUS_ESTIMATED_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.CLOSED),
            calc=self.compute_gripper_status_estimated_output,
        )

    def compute_positions_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self._se_input_port.Eval(context)

        # TODO: Remove
        assert isinstance(state_estimated_vector, np.ndarray)

        positions_output_vector = get_joint_positions_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=positions_output_vector,
        )

    def compute_velocities_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self._se_input_port.Eval(context)

        # TODO: Remove
        assert isinstance(state_estimated_vector, np.ndarray)

        velocities_output_vector = get_joint_velocities_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=velocities_output_vector,
        )

    def compute_gripper_status_estimated_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:
        state_estimated_vector = self._se_input_port.Eval(context)

        # TODO: Remove
        assert isinstance(state_estimated_vector, np.ndarray)

        lite6_gripper_status = get_gripper_status_from_lite6_state(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )

        output_value.set_value(lite6_gripper_status)
