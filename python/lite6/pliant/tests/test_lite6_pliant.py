import pytest

from python.common.control.constructs import PIDGains
from python.lite6.pliant.lite6_pliant import (
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
    Lite6PliantConfig,
    create_lite6_pliant,
)
from python.common.testing_utils import execute_pytest_file
from python.lite6.utils.lite6_model_utils import Lite6ModelType


@pytest.mark.parametrize(
    "unsupported_model_type",
    [Lite6ModelType.ROBOT_WITH_V_GRIPPER, Lite6ModelType.ROBOT_WITHOUT_GRIPPER],
)
def test_create_lite6_pliant_with_unsupported_model_type(
    unsupported_model_type: Lite6ModelType,
) -> None:

    id_pid_gains = PIDGains(kp=1.0, ki=1.0, kd=1.0)

    config = Lite6PliantConfig(
        lite6_model_type=unsupported_model_type,
        run_on_hardware=False,
        time_step_s=1e-3,
        inverse_dynamics_pid_gains=id_pid_gains,
    )

    with pytest.raises(AssertionError):
        create_lite6_pliant(config=config)


@pytest.mark.parametrize(
    "supported_model_type",
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
)
def test_create_lite6_pliant_with_supported_type(
    supported_model_type: Lite6ModelType,
) -> None:
    id_pid_gains = PIDGains(kp=1.0, ki=1.0, kd=1.0)

    config = Lite6PliantConfig(
        lite6_model_type=supported_model_type,
        run_on_hardware=False,
        time_step_s=1e-3,
        inverse_dynamics_pid_gains=id_pid_gains,
    )

    create_lite6_pliant(config=config)


if __name__ == "__main__":
    execute_pytest_file("test_create_lite6_pliant_with_supported_type")
