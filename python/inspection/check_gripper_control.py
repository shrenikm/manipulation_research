from typing import List

import numpy as np
from pydrake.all import DiagramBuilder
from pydrake.common.value import AbstractValue, Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import Context, Diagram, LeafSystem, System

from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_PD_IP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    Lite6PliantConfig,
    Lite6PliantType,
    create_simulator_for_lite6_pliant,
    get_tuned_pid_gains_for_pliant_id_controller,
)
from python.lite6.systems.lite6_gripper_status_source import Lite6GripperStatusSource
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
)


class GripperCheckController(LeafSystem):
    def __init__(
        self,
        times: List[float],
        statuses: List[Lite6GripperStatus],
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

        partition = context.get_time() // self.check_time_s
        if partition % 3 == 0:
            output_value.set_value(Lite6GripperStatus.CLOSED)
        elif partition % 3 == 1:
            output_value.set_value(Lite6GripperStatus.NEUTRAL)
        elif partition % 3 == 2:
            output_value.set_value(Lite6GripperStatus.OPEN)


def check_gripper_control(
    config: Lite6PliantConfig,
    times: List[float],
    statuses: List[Lite6GripperStatus],
) -> None:

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.pliant_diagram,
    )

    gripper_status_source = builder.AddSystem(
        Lite6GripperStatusSource(times=times, statuses=statuses),
    )

    builder.Connect(
        gripper_status_source.gss_output_port,
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

    with lite6_pliant_container.auto_meshcat_recording():
        simulator.AdvanceTo(
            boundary_time=15.0,
            interruptible=True,
        )


if __name__ == "__main__":

    lite6_model_type = Lite6ModelType.ROBOT_WITH_RP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.SIMULATION
    id_controller_pid_gains = get_tuned_pid_gains_for_pliant_id_controller(
        lite6_control_type=lite6_control_type,
    )
    plant_config = MultibodyPlantConfig(time_step=0.001)
    hardware_control_loop_time_step = 0.001

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
        inverse_dynamics_pid_gains=id_controller_pid_gains,
        plant_config=plant_config,
        hardware_control_loop_time_step=hardware_control_loop_time_step,
    )
    times = [0.0, 2.0, 4.0, 6.0, 8.0]
    statuses = [
        Lite6GripperStatus.CLOSED,
        Lite6GripperStatus.NEUTRAL,
        Lite6GripperStatus.OPEN,
        Lite6GripperStatus.CLOSED,
        Lite6GripperStatus.CLOSED,
    ]

    check_gripper_control(
        config=lite6_pliant_config,
        times=times,
        statuses=statuses,
    )
