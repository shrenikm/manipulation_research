import os
from typing import Optional, Tuple

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex

from python.common.class_utils import StrEnum
from python.common.custom_types import (
    FileName,
    FilePath,
    PositionsVector,
    StateVector,
    VelocitiesVector,
)
from python.common.robot_model_utils import (
    MODELS_ENVIRONMENT_DIRNAME,
    ROBOT_MODELS_DRAKE_URDF_DIRNAME,
    add_robot_models_to_package_map,
    get_models_directory_path,
    get_robot_models_directory_path,
)

LITE6_TABLE_FILENAME = "lite6_table.urdf"
LITE6_DESCRIPTION_DIRNAME = "lite6_description"
LITE6_GRIPPER_URDF_FILENAME_PREFIXES = (
    "lite6_normal_",
    "lite6_reverse",
    "lite6_vacuum",
)
LITE6_ROBOT_WITH_GRIPPER_URDF_FILENAME_PREFIX = "lite6_robot_with_"
LITE6_ROBOT_WITHOUT_GRIPPER_URDF_FILENAME_PREFIX = "lite6_robot_without_"
LITE6_GRIPPER_SUBDIR = "gripper"
LITE6_ROBOT_WITH_GRIPPER_SUBDIR = "robot_with_gripper"
LITE6_ROBOT_WITHOUT_GRIPPER_SUBDIR = "robot_without_gripper"

LITE6_DOF = 6
LITE6_GRIPPER_DOF = 2

# Gripper open and closed positions from the URDF
LITE6_NP_GRIPPER_OPEN_POSITIONS = (0.008, -0.008)
LITE6_NP_GRIPPER_CLOSED_POSITIONS = (0.0, 0.0)

LITE6_RP_GRIPPER_OPEN_POSITIONS = (0.008, -0.008)
LITE6_RP_GRIPPER_CLOSED_POSITIONS = (0.0, 0.0)


class Lite6ModelType(StrEnum):
    NP_GRIPPER = "lite6_normal_parallel_gripper.urdf"
    RP_GRIPPER = "lite6_reverse_parallel_gripper.urdf"
    V_GRIPPER = "lite6_vacuum_gripper.urdf"
    ROBOT_WITHOUT_GRIPPER = "lite6_robot_without_gripper.urdf"
    ROBOT_WITH_NP_GRIPPER = "lite6_robot_with_normal_parallel_gripper.urdf"
    ROBOT_WITH_RP_GRIPPER = "lite6_robot_with_reverse_parallel_gripper.urdf"
    ROBOT_WITH_V_GRIPPER = "lite6_robot_with_vacuum_gripper.urdf"


class Lite6ModelGroups:
    LITE6_GRIPPER_MODELS = (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    )

    LITE6_ROBOT_MODELS = (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
    )

    LITE6_ROBOT_WITH_GRIPPER_MODELS = (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
    )

    LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS = (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
    )

    LITE6_ROBOT_WITH_VACUUM_GRIPPER_MODELS = (Lite6ModelType.ROBOT_WITH_V_GRIPPER,)


def get_drake_lite6_urdf_path(lite6_model_type: Lite6ModelType) -> FilePath:

    lite6_urdf_filename = lite6_model_type.value

    if LITE6_ROBOT_WITH_GRIPPER_URDF_FILENAME_PREFIX in lite6_urdf_filename:
        subdir = LITE6_ROBOT_WITH_GRIPPER_SUBDIR
    elif LITE6_ROBOT_WITHOUT_GRIPPER_URDF_FILENAME_PREFIX in lite6_urdf_filename:
        subdir = LITE6_ROBOT_WITHOUT_GRIPPER_SUBDIR
    elif any(
        gripper_prefix in lite6_urdf_filename
        for gripper_prefix in LITE6_GRIPPER_URDF_FILENAME_PREFIXES
    ):
        subdir = LITE6_GRIPPER_SUBDIR
    else:
        raise NotImplementedError("Invalid lite6 urdf filename")

    return os.path.join(
        get_robot_models_directory_path(),
        LITE6_DESCRIPTION_DIRNAME,
        ROBOT_MODELS_DRAKE_URDF_DIRNAME,
        subdir,
        lite6_urdf_filename,
    )


def get_lite6_table_urdf_path() -> FilePath:
    return os.path.join(
        get_models_directory_path(),
        MODELS_ENVIRONMENT_DIRNAME,
        LITE6_TABLE_FILENAME,
    )


def get_lite6_urdf_base_frame_name(lite6_model_type: Lite6ModelType) -> str:
    return {
        Lite6ModelType.NP_GRIPPER: "lite6_normal_parallel_gripper_link",
        Lite6ModelType.RP_GRIPPER: "lite6_reverse_parallel_gripper_link",
        Lite6ModelType.V_GRIPPER: "lite6_vacuum_gripper_link",
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER: "link_base",
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER: "link_base",
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER: "link_base",
        Lite6ModelType.ROBOT_WITH_V_GRIPPER: "link_base",
    }[lite6_model_type]


def get_lite6_table_urdf_lite6_position_frame_name() -> str:
    return "link_lite6_position"


def get_lite6_num_actuators(lite6_model_type: Lite6ModelType) -> int:
    return {
        Lite6ModelType.NP_GRIPPER: 2,
        Lite6ModelType.RP_GRIPPER: 2,
        Lite6ModelType.V_GRIPPER: 0,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER: LITE6_DOF,
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER: LITE6_DOF,
    }[lite6_model_type]


def get_lite6_num_positions(lite6_model_type: Lite6ModelType) -> int:
    return {
        Lite6ModelType.NP_GRIPPER: 2,
        Lite6ModelType.RP_GRIPPER: 2,
        Lite6ModelType.V_GRIPPER: 0,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER: LITE6_DOF,
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER: LITE6_DOF,
    }[lite6_model_type]


def get_lite6_num_velocities(lite6_model_type: Lite6ModelType) -> int:
    return {
        Lite6ModelType.NP_GRIPPER: 2,
        Lite6ModelType.RP_GRIPPER: 2,
        Lite6ModelType.V_GRIPPER: 0,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER: LITE6_DOF,
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER: LITE6_DOF + 2,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER: LITE6_DOF,
    }[lite6_model_type]


def get_lite6_num_states(lite6_model_type: Lite6ModelType) -> int:
    return get_lite6_num_positions(
        lite6_model_type=lite6_model_type,
    ) + get_lite6_num_velocities(
        lite6_model_type=lite6_model_type,
    )


def add_joint_positions_to_lite6_state(
    lite6_model_type: Lite6ModelType,
    state_vector: StateVector,
    positions_vector: PositionsVector,
) -> StateVector:
    """
    Returns a new state vector with the positions at the correct indices.
    """
    assert lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert positions_vector.size == LITE6_DOF

    state_vector_with_added_positions = np.copy(state_vector)
    state_vector_with_added_positions[:LITE6_DOF] = positions_vector
    return state_vector_with_added_positions


def add_joint_velocities_to_lite6_state(
    lite6_model_type: Lite6ModelType,
    state_vector: StateVector,
    velocities_vector: VelocitiesVector,
) -> VelocitiesVector:
    """
    Returns a new state vector with the velocities at the correct indices.
    """
    assert lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert velocities_vector.size == LITE6_DOF

    state_vector_with_added_velocities = np.copy(state_vector)

    if lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS:
        state_vector_with_added_velocities[
            LITE6_DOF + LITE6_GRIPPER_DOF : 2 * LITE6_DOF + LITE6_GRIPPER_DOF
        ] = velocities_vector
        return state_vector_with_added_velocities
    else:
        state_vector_with_added_velocities[LITE6_DOF:] = velocities_vector
        return state_vector_with_added_velocities


def get_parallel_gripper_positions(
    lite6_model_type: Lite6ModelType,
    gripper_closed_desired: bool,
) -> Tuple[float, float]:
    assert lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    if (
        lite6_model_type == Lite6ModelType.ROBOT_WITH_NP_GRIPPER
        and not gripper_closed_desired
    ):
        return LITE6_NP_GRIPPER_OPEN_POSITIONS
    elif (
        lite6_model_type == Lite6ModelType.ROBOT_WITH_NP_GRIPPER
        and gripper_closed_desired
    ):
        return LITE6_NP_GRIPPER_CLOSED_POSITIONS
    elif (
        lite6_model_type == Lite6ModelType.ROBOT_WITH_RP_GRIPPER
        and not gripper_closed_desired
    ):
        return LITE6_RP_GRIPPER_OPEN_POSITIONS
    elif (
        lite6_model_type == Lite6ModelType.ROBOT_WITH_RP_GRIPPER
        and gripper_closed_desired
    ):
        return LITE6_RP_GRIPPER_CLOSED_POSITIONS
    else:
        raise NotImplementedError


def add_parallel_gripper_state_to_lite6_state(
    lite6_model_type: Lite6ModelType,
    state_vector: StateVector,
    gripper_closed_desired: bool,
) -> StateVector:
    """
    Returns a new state vector with the gripper positions and velocities using
    the desired gripper closed status. (Note that this is only the lite6 robot
    models with parallel grippers).
    If the gripper needs to be closed, we set the positions of the grippers as
    such and the velocities to zero. Similary for the open position with the
    velocities still being set to zero.
    """
    parallel_gripper_positions = get_parallel_gripper_positions(
        lite6_model_type=lite6_model_type,
        gripper_closed_desired=gripper_closed_desired,
    )
    state_vector_with_added_gripper_state = np.copy(state_vector)
    state_vector_with_added_gripper_state[
        LITE6_DOF : LITE6_DOF + LITE6_GRIPPER_DOF
    ] = parallel_gripper_positions
    state_vector_with_added_gripper_state[
        2 * LITE6_DOF + LITE6_GRIPPER_DOF : 2 * LITE6_DOF + 2 * LITE6_GRIPPER_DOF
    ] = 0.0
    return state_vector_with_added_gripper_state


def create_lite6_state(
    lite6_model_type: Lite6ModelType,
    positions_vector: PositionsVector,
    velocities_vector: VelocitiesVector,
    gripper_closed_desired: bool,
) -> StateVector:
    """
    Creates the lite6 state from the given positions, velocities and gripper closed status.
    """
    state_vector = np.zeros(get_lite6_num_states(lite6_model_type), dtype=np.float64)
    state_vector = add_joint_positions_to_lite6_state(
        lite6_model_type=lite6_model_type,
        state_vector=state_vector,
        positions_vector=positions_vector,
    )
    state_vector = add_joint_velocities_to_lite6_state(
        lite6_model_type=lite6_model_type,
        state_vector=state_vector,
        velocities_vector=velocities_vector,
    )
    state_vector = add_parallel_gripper_state_to_lite6_state(
        lite6_model_type=lite6_model_type,
        state_vector=state_vector,
        gripper_closed_desired=gripper_closed_desired,
    )
    return state_vector


def get_joint_positions_from_lite6_state(
    lite6_model_type: Lite6ModelType,
    state_vector: StateVector,
) -> PositionsVector:
    assert lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_MODELS
    return state_vector[:LITE6_DOF]


def add_lite6_model_to_plant(
    plant: MultibodyPlant,
    lite6_model_type: Lite6ModelType,
    parser: Optional[Parser] = None,
    place_on_table: bool = False,
) -> ModelInstanceIndex:
    """
    Loads and adds the required lite6 model (given by the model type) to the
    plant.
    If the parser is not given, it is created from the plant. If it is given,
    it is required that its package map be updated to include the model paths.
    If place_on_table is True, the table URDF is also loaded and the robot's
    frame is welded to the taable. If False, the robot is welded to the world
    frame.
    """
    if parser is None:
        parser = Parser(plant)
        package_map = parser.package_map()
        add_robot_models_to_package_map(package_map=package_map)
    else:
        assert parser.package_map().Contains(LITE6_DESCRIPTION_DIRNAME)

    if place_on_table:
        # Add the table and the robot model.
        lite6_model = parser.AddModels(
            get_drake_lite6_urdf_path(lite6_model_type=lite6_model_type),
        )[0]
        parser.AddModels(get_lite6_table_urdf_path())

        # Weld the robot to the table.
        lite6_position_frame_name = get_lite6_table_urdf_lite6_position_frame_name()
        base_frame_name = get_lite6_urdf_base_frame_name(
            lite6_model_type=lite6_model_type,
        )
        plant.WeldFrames(
            plant.GetFrameByName(lite6_position_frame_name),
            plant.GetFrameByName(base_frame_name),
        )
        return lite6_model
    else:
        # Add only the robot model.
        lite6_model = parser.AddModels(
            get_drake_lite6_urdf_path(lite6_model_type=lite6_model_type),
        )[0]

        base_frame_name = get_lite6_urdf_base_frame_name(
            lite6_model_type=lite6_model_type,
        )
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(base_frame_name))
        return lite6_model
