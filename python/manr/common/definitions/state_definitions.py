import attr

from manr.common.custom_types import (
    GripperPositionsVector,
    GripperVelocitiesVector,
    JointPositionsVector,
    JointVelocitiesVector,
)


@attr.frozen
class ManipulatorState:
    """
    Class to represent the state of a manipulator.
    """

    joint_positions: JointPositionsVector
    gripper_positions: GripperPositionsVector

    joint_velocities: JointVelocitiesVector
    gripper_velocities: GripperVelocitiesVector
