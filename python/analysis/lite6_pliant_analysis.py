from typing import Optional, Sequence

import numpy as np
from pydrake.all import DiagramBuilder, StartMeshcat
from pydrake.common.value import Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram, System
from pydrake.visualization import (
    AddDefaultVisualization,
    ApplyVisualizationConfig,
    VisualizationConfig,
)

from python.common.control.constructs import PIDGains
from python.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
)
from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from python.lite6.utils.lite6_model_utils import (
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    Lite6PliantType,
)


def analyze_lite6_pliant(
    config: Lite6PliantConfig,
) -> None:

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.diagram,
    )
    meshcat = lite6_pliant_container.meshcat

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)
    lite6_pliant.GetInputPort(
        port_name="positions_desired_input",
    ).FixValue(lite6_pliant_context, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    lite6_pliant.GetInputPort(
        port_name="velocities_desired_input",
    ).FixValue(lite6_pliant_context, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    lite6_pliant.GetInputPort(
        port_name="gripper_status_desired_input",
    ).FixValue(lite6_pliant_context, Value(Lite6GripperStatus.OPEN))

    diagram.ForcedPublish(simulator_context)

    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(10.0)
    meshcat.PublishRecording()


if __name__ == "__main__":

    lite6_model_type = Lite6ModelType.ROBOT_WITH_RP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.SIMULATION
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

    analyze_lite6_pliant(
        config=lite6_pliant_config,
    )
