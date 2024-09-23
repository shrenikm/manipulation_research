from enum import StrEnum


class ManipulatorControlTarget(StrEnum):
    """
    Enum to represent the control target of a manipulator.
    """

    JOINT = "joint"
    EEF = "eef"


class ManipulatorControlTyupe(StrEnum):
    # Position control. Can be for any control target.
    POSITION = "position"
    # Velocity control. Can be for any control target.
    VELOCITY = "velocity"
    # Control both position and velocity.
    STATE = "state"


class Lite6GripperStatus(Enum):
    """
    Parallel gripper status. Either open or closed.
    """

    OPEN = auto()
    CLOSED = auto()
    NEUTRAL = auto()
