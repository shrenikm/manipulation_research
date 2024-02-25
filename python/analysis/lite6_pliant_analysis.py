from typing import Optional, Sequence

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JointSliders,
    StartMeshcat,
)
from pydrake.common.value import Value
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import System
from pydrake.visualization import AddDefaultVisualization

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
    add_lite6_model_to_plant,
    get_default_height_for_object_model_type,
)


def analyze_lite6_pliant(
    config: Lite6PliantConfig,
) -> None:

    builder = DiagramBuilder()

    lite6_pliant: System = builder.AddNamedSystem(
        name="lite6_pliant",
        system=create_lite6_pliant(
            config=config,
        ),
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)

    lite6_pliant.GetInputPort(
        port_name="positions_desired",
    ).FixValue(lite6_pliant_context, np.zeros(6))

    lite6_pliant.GetInputPort(
        port_name="velocities_desired",
    ).FixValue(lite6_pliant_context, np.zeros(6))

    lite6_pliant.GetInputPort(
        port_name="gripper_status_desired",
    ).FixValue(lite6_pliant_context, Value(Lite6GripperStatus.CLOSED))

    meshcat = StartMeshcat()
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(10.0)
    meshcat.PublishRecording()


if __name__ == "__main__":

    lite6_model_type = Lite6ModelType.ROBOT_WITH_NP_GRIPPER
    lite6_control_type = Lite6ControlType.STATE
    lite6_pliant_type = Lite6PliantType.SIMULATION

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
    )

    analyze_lite6_pliant(
        config=lite6_pliant_config,
    )
