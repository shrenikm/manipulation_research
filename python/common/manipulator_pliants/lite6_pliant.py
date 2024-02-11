from typing import Optional

import attr
import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Diagram, DiagramBuilder

from python.common.control.constructs import PIDGains
from python.common.robot_model_utils import add_robot_models_to_package_map
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    get_drake_lite6_urdf_path,
    get_lite6_urdf_base_frame_name,
)


@attr.frozen
class Lite6PliantConfig:
    lite6_model_type: Lite6ModelType
    run_on_hardware: bool
    time_step_s: float
    inverse_dynamics_pid_gains: PIDGains
    plant_config: Optional[MultibodyPlantConfig] = None


def create_lite6_pliant(config: Lite6PliantConfig) -> Diagram:

    builder: DiagramBuilder = DiagramBuilder()
    plant: MultibodyPlant
    scene_graph: SceneGraph
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=config.time_step_s,
    )

    if config.plant_config is not None:
        ApplyMultibodyPlantConfig(config.plant_config, plant)

    # Load the model
    parser = Parser(plant)
    package_map = parser.package_map()
    add_robot_models_to_package_map(package_map=package_map)

    manipulator_description_file_path = get_drake_lite6_urdf_path(
        lite6_model_type=lite6_model_type,
    )
    lite6_model = parser.AddModels(
        get_drake_lite6_urdf_path(lite6_model_type=config.lite6_model_type),
    )
    n = plant.num_positions(model_instance=lite6_model)

    lite6_controller_plant = MultibodyPlant(time_step=config.time_step_s)
    parser = Parser(lite6_controller_plant)
    package_map = parser.package_map()
    add_robot_models_to_package_map(package_map=package_map)
    lite6_controller_model = parser.AddModels(
        get_drake_lite6_urdf_path(lite6_model_type=config.lite6_model_type),
    )
    lite6_controller_plant.WeldFrames(
        lite6_controller_plant.world_frame(),
        lite6_controller_plant.GetFrameByName(
            get_lite6_urdf_base_frame_name(lite6_model_type=config.lite6_model_type)
        ),
    )
    lite6_controller_plant.Finalize()
    id_controller = InverseDynamicsController(
        lite6_controller_plant,
        kp=config.inverse_dynamics_pid_gains.kp * np.ones(n, dtype=np.float64),
        ki=config.inverse_dynamics_pid_gains.ki * np.ones(n, dtype=np.float64),
        kd=config.inverse_dynamics_pid_gains.kd * np.ones(n, dtype=np.float64),
        has_reference_acceleration=False,
    )

    plant.Finalize()
    diagram = builder.Build()

    return diagram
