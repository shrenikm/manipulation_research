from __future__ import annotations

from enum import StrEnum

import attr
import numpy as np
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
    joint_velocities: JointVelocitiesVector

    gripper_positions: GripperPositionsVector
    gripper_velocities: GripperVelocitiesVector
    gripper_status: ManipulatorGripperStatus

    @classmethod
    def create_dummy(cls) -> ManipulatorState:
        """
        Create a dummy manipulator state.
        Useful for declaring abstract states, etc.
        """
        return cls(
            joint_positions=np.empty(0),
            joint_velocities=np.empty(0),
            gripper_positions=np.empty(0),
            gripper_velocities=np.empty(0),
            gripper_status=ManipulatorGripperStatus.NEUTRAL,
        )
