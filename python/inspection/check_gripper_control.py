from typing import Generator

import numpy as np
from pydrake.all import DiagramBuilder
from pydrake.common.value import AbstractValue, Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context, Diagram, LeafSystem, System

from python.common.control.constructs import PIDGains
from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_PD_IP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    Lite6PliantConfig,
    auto_meshcat_recording,
    create_simulator_for_lite6_pliant,
)
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    Lite6PliantType,
)


class GripperCheckController(LeafSystem):
    def __init__(
        self,
        check_time_s: float,
    ):
        super().__init__()

        self.check_time_s = check_time_s

        self.gsd_output_port = self.DeclareAbstractOutputPort(
            name="gsd_output_port",
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_desired_output,
        )

    def _compute_gripper_status_desired_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:
        print(context.get_time())

        partition = context.get_time() // self.check_time_s
        if partition % 3 == 0:
            output_value.set_value(Lite6GripperStatus.CLOSED)
        elif partition % 3 == 1:
            output_value.set_value(Lite6GripperStatus.NEUTRAL)
        elif partition % 3 == 2:
            output_value.set_value(Lite6GripperStatus.OPEN)


def check_gripper_control(
    config: Lite6PliantConfig,
    check_time_s: float,
) -> None:

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.diagram,
    )

    gripper_controller = builder.AddSystem(
        GripperCheckController(check_time_s=check_time_s),
    )

    builder.Connect(
        gripper_controller.gsd_output_port,
        lite6_pliant.GetInputPort(LITE6_PLIANT_GSD_IP_NAME),
    )

    diagram = builder.Build()
    simulator = create_simulator_for_lite6_pliant(
        config=config,
        diagram=diagram,
    )
    simulator_context = simulator.get_mutable_context()
    lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)

    lite6_pliant.GetInputPort(
        port_name=LITE6_PLIANT_PD_IP_NAME,
    ).FixValue(lite6_pliant_context, np.zeros(LITE6_DOF, dtype=np.float64))

    lite6_pliant.GetInputPort(
        port_name=LITE6_PLIANT_VD_IP_NAME,
    ).FixValue(lite6_pliant_context, np.zeros(LITE6_DOF, dtype=np.float64))

    lite6_pliant.GetSystems

    with auto_meshcat_recording(
        config=config,
        meshcat=lite6_pliant_container.meshcat,
    ):
        simulator.AdvanceTo(
            boundary_time=21.0,
            interruptible=True,
        )


if __name__ == "__main__":

    lite6_model_type = Lite6ModelType.ROBOT_WITH_RP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.HARDWARE
    inverse_dynamics_pid_gains = PIDGains.from_scalar_gains(
        size=8,
        kp_scalar=100.0,
        ki_scalar=1.0,
        kd_scalar=20.0,
    )
    plant_config = MultibodyPlantConfig(time_step=0.001)

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
        inverse_dynamics_pid_gains=inverse_dynamics_pid_gains,
        plant_config=plant_config,
    )
    check_time_s = 3.0

    check_gripper_control(
        config=lite6_pliant_config,
        check_time_s=check_time_s,
    )
