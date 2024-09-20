from enum import StrEnum

import attr

from manr.common.custom_types import (
    GripperPositionsVector,
    GripperVelocitiesVector,
    JointPositionsVector,
    JointVelocitiesVector,
)


class ManipulatorGripperStatus(StrEnum):
    """
    Enum to represent the status of a manipulator's gripper.
    """

    OPENING = "opening"
    CLOSING = "closing"
    NEUTRAL = "neutral"
    OPEN = "open"
    CLOSED = "closed"


@attr.frozen
class ManipulatorState:
    """
    Class to represent the state of a manipulator.
    """

    joint_positions: JointPositionsVector
    gripper_positions: GripperPositionsVector

    joint_velocities: JointVelocitiesVector
    gripper_velocities: GripperVelocitiesVector
