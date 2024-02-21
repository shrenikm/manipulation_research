import os

from pydrake.multibody.parsing import PackageMap

from python.common.custom_types import DirPath, FileName, FilePath

MODELS_DIRNAME = "models"
ROBOT_MODELS_DIRNAME = "robot_models"
ROBOT_MODELS_DRAKE_URDF_DIRNAME = "drake_urdf"


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
