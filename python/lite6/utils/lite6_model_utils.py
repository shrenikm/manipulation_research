import os
from enum import Enum

from python.common.custom_types import FileName, FilePath
from python.common.robot_model_utils import (
    ROBOT_MODELS_DRAKE_URDF_DIRNAME,
    get_robot_models_directory_path,
)

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


class Lite6ModelType(str, Enum):
    NP_GRIPPER = "lite6_normal_parallel_gripper.urdf"
    RP_GRIPPER = "lite6_reverse_parallel_gripper.urdf"
    V_GRIPPER = "lite6_vacuum_gripper.urdf"
    ROBOT_WITHOUT_GRIPPER = "lite6_robot_without_gripper.urdf"
    ROBOT_WITH_NP_GRIPPER = "lite6_robot_with_normal_parallel_gripper.urdf"
    ROBOT_WITH_RP_GRIPPER = "lite6_robot_with_reverse_parallel_gripper.urdf"
    ROBOT_WITH_V_GRIPPER = "lite6_robot_with_vacuum_gripper.urdf"


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
