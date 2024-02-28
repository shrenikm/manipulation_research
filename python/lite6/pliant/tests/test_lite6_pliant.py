import pytest
from pydrake.multibody.plant import MultibodyPlant, MultibodyPlantConfig
from pydrake.systems.framework import Diagram

from python.common.control.constructs import PIDGains
from python.common.testing_utils import execute_pytest_file
from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
    Lite6ControlType,
    Lite6PliantConfig,
)
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    Lite6PliantType,
    get_lite6_num_positions,
)


@pytest.mark.parametrize(
    "unsupported_model_type",
    [Lite6ModelType.ROBOT_WITH_V_GRIPPER, Lite6ModelType.ROBOT_WITHOUT_GRIPPER],
)
@pytest.mark.parametrize(
    "lite6_control_type",
    [control_type for control_type in Lite6ControlType],
)
def test_create_lite6_pliant_with_unsupported_model_type(
    unsupported_model_type: Lite6ModelType,
    lite6_control_type: Lite6ControlType,
) -> None:

    nq = get_lite6_num_positions(lite6_model_type=unsupported_model_type)
    id_pid_gains = PIDGains.from_scalar_gains(
        size=nq,
        kp_scalar=1.0,
        ki_scalar=1.0,
        kd_scalar=1.0,
    )

    config = Lite6PliantConfig(
        lite6_model_type=unsupported_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=Lite6PliantType.SIMULATION,
        inverse_dynamics_pid_gains=id_pid_gains,
        plant_config=MultibodyPlantConfig(time_step=0.001),
    )

    with pytest.raises(AssertionError):
        create_lite6_pliant(config=config)


@pytest.mark.parametrize(
    "lite6_model_type",
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
)
@pytest.mark.parametrize(
    "lite6_control_type",
    [control_type for control_type in Lite6ControlType],
)
def test_create_lite6_pliant_with_supported_type(
    lite6_model_type: Lite6ModelType,
    lite6_control_type: Lite6ControlType,
) -> None:

    nq = get_lite6_num_positions(lite6_model_type=lite6_model_type)
    id_pid_gains = PIDGains.from_scalar_gains(
        size=nq,
        kp_scalar=1.0,
        ki_scalar=1.0,
        kd_scalar=1.0,
    )

    config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=Lite6PliantType.SIMULATION,
        inverse_dynamics_pid_gains=id_pid_gains,
        plant_config=MultibodyPlantConfig(time_step=0.001),
    )

    pliant = create_lite6_pliant(config=config)

    # TODO: Better tests.
    assert isinstance(pliant.diagram, Diagram)
    assert isinstance(pliant.plant, MultibodyPlant)


if __name__ == "__main__":
    execute_pytest_file("test_create_lite6_pliant_with_supported_type")
