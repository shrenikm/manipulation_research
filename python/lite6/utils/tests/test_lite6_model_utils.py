import os

import pytest

from python.common.testing_utils import execute_pytest_file
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    get_drake_lite6_urdf_path,
    get_lite6_num_actuators,
    get_lite6_num_positions,
    get_lite6_num_states,
    get_lite6_num_velocities,
    get_lite6_table_urdf_lite6_position_frame_name,
    get_lite6_table_urdf_path,
    get_lite6_urdf_base_frame_name,
)


def test_get_drake_lite6_urdf_path() -> None:
    """
    Test that each a valid and existing model path can be obtained
    for each lite6 model type.
    """
    for lite6_model_type in Lite6ModelType.get_all_fields():
        urdf_path = get_drake_lite6_urdf_path(
            lite6_model_type=lite6_model_type,
        )
        assert os.path.isfile(urdf_path)


def test_get_lite6_table_urdf_path() -> None:
    """
    Test that we can retrieve the path to the lite6 table urdf.
    """
    assert os.path.isfile(get_lite6_table_urdf_path())


def test_lite6_urdf_base_frame_name() -> None:
    for lite6_model_type in Lite6ModelType.get_all_fields():
        base_frame_name = get_lite6_urdf_base_frame_name(
            lite6_model_type=lite6_model_type,
        )
        assert isinstance(base_frame_name, str)


def test_get_lite6_table_urdf_lite6_position_frame_name() -> None:
    assert get_lite6_table_urdf_lite6_position_frame_name() == "link_lite6_position"


def test_get_lite6_num_actuators() -> None:
    assert get_lite6_num_actuators(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_actuators(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_actuators(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_NP_GRIPPER) == 8
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_RP_GRIPPER) == 8
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_positions() -> None:
    assert get_lite6_num_positions(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_positions(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_positions(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_NP_GRIPPER) == 8
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_RP_GRIPPER) == 8
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_velocities() -> None:
    assert get_lite6_num_velocities(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_velocities(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_velocities(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_NP_GRIPPER) == 8
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_RP_GRIPPER) == 8
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_states() -> None:
    assert get_lite6_num_states(Lite6ModelType.NP_GRIPPER) == 4
    assert get_lite6_num_states(Lite6ModelType.RP_GRIPPER) == 4
    assert get_lite6_num_states(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 12
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_NP_GRIPPER) == 16
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_RP_GRIPPER) == 16
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 12


if __name__ == "__main__":
    execute_pytest_file()
