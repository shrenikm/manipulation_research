from enum import Enum
from typing import Optional

import attr
from pydrake.multibody.plant import MultibodyPlantConfig

from python.common.class_utils import StrEnum
from python.common.control.constructs import PIDGains
from python.lite6.utils.lite6_model_utils import Lite6ModelType

LITE6_PLIANT_SUPPORTED_MODEL_TYPES = (
    Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
    Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
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
