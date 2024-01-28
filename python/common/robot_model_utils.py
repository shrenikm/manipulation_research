import os

from pydrake.multibody.parsing import PackageMap

from python.common.custom_types import DirPath, FileName, FilePath

ROBOT_MODELS_DIRNAME = "robot_models"
LITE6_DESCRIPTION_DIRNAME = "lite6_description"
ROBOT_MODELS_DRAKE_URDF_DIRNAME = "drake_urdf"

LITE6_GRIPPER_URDF_FILENAME_PREFIXES = (
    "lite6_normal_",
    "lite6_reverse",
    "lite6_vaccum",
)
LITE6_ROBOT_WITH_GRIPPER_URDF_FILENAME_PREFIX = "lite6_robot_with_"
LITE6_ROBOT_WITHOUT_GRIPPER_URDF_FILENAME_PREFIX = "lite6_robot_without_"


def get_robot_models_directory_path() -> DirPath:
    current_directory_path = os.path.dirname(
        os.path.expanduser(os.path.realpath(__file__))
    )
    robot_models_directory_path = os.path.join(
        current_directory_path,
        "..",
        "..",
        ROBOT_MODELS_DIRNAME,
    )
    return os.path.realpath(robot_models_directory_path)


def add_robot_models_to_package_map(package_map: PackageMap) -> None:
    """
    Add all the robot models/descriptions inside robot_models/ into the package map.
    """
    robot_models_directory_path = get_robot_models_directory_path()

    all_robot_model_dirnames_and_paths = [
        (dirname, dirpath)
        for dirname in os.listdir(robot_models_directory_path)
        if os.path.isdir(dirpath := os.path.join(robot_models_directory_path, dirname))
    ]

    for package_name, package_path in all_robot_model_dirnames_and_paths:
        package_map.Add(
            package_name=package_name,
            package_path=package_path,
        )


def get_drake_lite6_urdf_path(lite6_urdf_filename: FileName) -> FilePath:

    if LITE6_ROBOT_WITH_GRIPPER_URDF_FILENAME_PREFIX in lite6_urdf_filename:
        subdir = "robot_with_gripper"
    elif LITE6_ROBOT_WITHOUT_GRIPPER_URDF_FILENAME_PREFIX in lite6_urdf_filename:
        subdir = "robot_without_gripper"
    elif any(
        gripper_prefix in lite6_urdf_filename
        for gripper_prefix in LITE6_GRIPPER_URDF_FILENAME_PREFIXES
    ):
        subdir = "gripper"
    else:
        raise NotImplementedError("Invalid lite6 urdf filename")

    return os.path.join(
        get_robot_models_directory_path(),
        LITE6_DESCRIPTION_DIRNAME,
        ROBOT_MODELS_DRAKE_URDF_DIRNAME,
        subdir,
        lite6_urdf_filename,
    )
