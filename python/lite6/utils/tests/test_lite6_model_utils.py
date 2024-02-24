import os

import numpy as np
import pytest

from python.common.testing_utils import execute_pytest_file
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ModelGroups,
    Lite6ModelType,
    add_joint_positions_to_lite6_state,
    add_joint_velocities_to_lite6_state,
    add_parallel_gripper_state_to_lite6_state,
    create_lite6_state,
    get_drake_lite6_urdf_path,
    get_lite6_num_actuators,
    get_lite6_num_positions,
    get_lite6_num_states,
    get_lite6_num_velocities,
    get_lite6_table_urdf_lite6_position_frame_name,
    get_lite6_table_urdf_path,
    get_lite6_urdf_base_frame_name,
)


def test_lite6_model_groups() -> None:
    assert Lite6ModelType.NP_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS
    assert Lite6ModelType.RP_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS
    assert Lite6ModelType.V_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_NP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_RP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_V_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITHOUT_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS

    assert (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    )
    assert (
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    )
    assert (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    )

    assert (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    )
    assert (
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    )

    assert (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER
        in Lite6ModelGroups.LITE6_ROBOT_WITH_VACUUM_GRIPPER_MODELS
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


def test_add_joint_positions_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_joint_positions_to_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                positions_vector=np.zeros(6),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
    ):
        state_vector_initial = np.ones(
            16,
            dtype=np.float64,
        )
        state_vector = np.copy(state_vector_initial)
        positions_vector = 2 * np.ones(6, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_positions_to_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                positions_vector=np.zeros(5),
            )

        new_state_vector = add_joint_positions_to_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            positions_vector=positions_vector,
        )
        # No mutation test
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
            ],
        )


def test_add_joint_velocities_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_joint_velocities_to_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                velocities_vector=np.zeros(6),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
    ):
        state_vector_initial = np.ones(
            16,
            dtype=np.float64,
        )
        state_vector = np.copy(state_vector_initial)
        velocities_vector = 3 * np.ones(6, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_velocities_to_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                velocities_vector=np.zeros(5),
            )

        new_state_vector = add_joint_velocities_to_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            velocities_vector=velocities_vector,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                1.0,
                1.0,
            ],
        )


def test_add_parallel_gripper_state_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_parallel_gripper_state_to_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                gripper_closed_desired=True,
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
    ):
        state_vector_initial = np.ones(
            16,
            dtype=np.float64,
        )
        state_vector = np.copy(state_vector_initial)

        gripper_closed_desired = True
        new_state_vector = add_parallel_gripper_state_to_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            gripper_closed_desired=gripper_closed_desired,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                0.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                0.0,
            ],
        )

        gripper_closed_desired = False
        new_state_vector = add_parallel_gripper_state_to_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            gripper_closed_desired=gripper_closed_desired,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                0.008,
                -0.008,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                0.0,
            ],
        )


def test_create_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            create_lite6_state(
                lite6_model_type=lite6_model_type,
                positions_vector=np.zeros(6),
                velocities_vector=np.zeros(6),
                gripper_closed_desired=True,
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_NP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_RP_GRIPPER,
    ):
        positions_vector = 2 * np.ones(6, dtype=np.float64)
        velocities_vector = 3 * np.ones(6, dtype=np.float64)

        gripper_closed_desired = True
        state_vector = create_lite6_state(
            lite6_model_type=lite6_model_type,
            positions_vector=positions_vector,
            velocities_vector=velocities_vector,
            gripper_closed_desired=gripper_closed_desired,
        )
        np.testing.assert_array_equal(
            state_vector,
            [
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                0.0,
                0.0,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                0.0,
                0.0,
            ],
        )

        gripper_closed_desired = False
        state_vector = create_lite6_state(
            lite6_model_type=lite6_model_type,
            positions_vector=positions_vector,
            velocities_vector=velocities_vector,
            gripper_closed_desired=gripper_closed_desired,
        )
        np.testing.assert_array_equal(
            state_vector,
            [
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                2.0,
                0.008,
                -0.008,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                3.0,
                0.0,
                0.0,
            ],
        )


if __name__ == "__main__":
    execute_pytest_file()
