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

from python.analysis.pliant_analysis.lite6_pliant_analysis_choreographer import (
    Lite6PliantChoreographer,
    Lite6PliantChoreographerController,
    get_choreographer_config_yaml_filepath,
)
from python.common.control.constructs import PIDGains
from python.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
)
from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_PE_OP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    LITE6_PLIANT_VE_OP_NAME,
    Lite6PliantConfig,
    Lite6PliantType,
    create_simulator_for_lite6_pliant,
)
from python.lite6.utils.lite6_model_utils import (
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
)


def analyze_lite6_pliant(
    config: Lite6PliantConfig,
    choreographer: Lite6PliantChoreographer,
) -> None:

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.pliant_diagram,
    )

    choreographer_controller = builder.AddSystem(
        Lite6PliantChoreographerController(
            config=config,
            choreographer=choreographer,
        ),
    )

    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_PE_OP_NAME),
        choreographer_controller.cc_pe_input_port,
    )
    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_VE_OP_NAME),
        choreographer_controller.cc_ve_input_port,
    )
    builder.Connect(
        choreographer_controller.cc_output_port,
        lite6_pliant.GetInputPort(LITE6_PLIANT_VD_IP_NAME),
    )

    diagram = builder.Build()
    simulator = create_simulator_for_lite6_pliant(
        config=config,
        diagram=diagram,
    )
    simulator_context = simulator.get_mutable_context()
    lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)

    lite6_pliant.GetInputPort(
        port_name=LITE6_PLIANT_GSD_IP_NAME,
    ).FixValue(lite6_pliant_context, Value(Lite6GripperStatus.CLOSED))

    diagram.ForcedPublish(simulator_context)

    with lite6_pliant_container.auto_meshcat_recording():
        simulator.AdvanceTo(
            boundary_time=30.0,
            interruptible=True,
        )

    choreographer_controller.plot_recordings()


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
    choreographer = Lite6PliantChoreographer.from_yaml(
        yaml_filepath=get_choreographer_config_yaml_filepath(),
    )

    analyze_lite6_pliant(
        config=lite6_pliant_config,
        choreographer=choreographer,
    )
