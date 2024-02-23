from enum import Enum
from typing import Optional

import attr
from pydrake.common.value import Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.class_utils import StrEnum
from python.common.control.constructs import PIDGains
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ModelType,
    get_lite6_num_states,
)

LITE6_PLIANT_SUPPORTED_MODEL_TYPES = (
    Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
    Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
)


LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME = "positions_desired_input"
LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME = "velocities_desired_input"
LITE6_PLIANT_GRIPPER_CLOSED_STATUS_IP_NAME = "gripper_closed_status_input"

LITE6_PLIANT_STATE_ESTIMATED_IP_NAME = "state_estimated_input"
LITE6_PLIANT_STATE_DESIRED_OP_NAME = "state_desired_output"

LITE6_PLIANT_MULTIPLEXER_PREFIX = "mult_"
LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GCS_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GRIPPER_CLOSED_STATUS_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_STATE_ESTIMATED_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GRIPPER_CLOSED_STATUS_IP_NAME
)


class Lite6ControlType(StrEnum):
    POSTITION_AND_VELOCITY = "position_and_velocity"
    VELOCITY_ONLY = "velocity_only"


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

        num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=num_total_states,
        )
        self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME,
            size=LITE6_DOF,
        )
        self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME,
            size=LITE6_DOF,
        )
        self.DeclareAbstractInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GCS_IP_NAME,
            model_value=Value(False),
        )

        self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_STATE_DESIRED_OP_NAME,
            size=num_total_states,
            calc=None,
        )

    def compute_state_desired_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        ...
