import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    MultibodyPlant,
)
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Diagram, DiagramBuilder

from python.common.class_utils import StrEnum
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
    Lite6PliantConfig,
)
from python.lite6.utils.lite6_model_utils import add_lite6_model_to_plant


def create_lite6_pliant_for_hardware(config: Lite6PliantConfig) -> Diagram:
    raise NotImplementedError()


def create_lite6_pliant_for_simulation(config: Lite6PliantConfig) -> Diagram:
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
    lite6_model = add_lite6_model_to_plant(
        plant=plant,
        lite6_model_type=config.lite6_model_type,
    )

    plant.Finalize()

    n = plant.num_positions(model_instance=lite6_model)

    lite6_controller_plant = MultibodyPlant(time_step=config.time_step_s)
    lite6_controller_model = add_lite6_model_to_plant(
        plant=lite6_controller_plant,
        lite6_model_type=config.lite6_model_type,
    )
    lite6_controller_plant.Finalize()

    id_controller = InverseDynamicsController(
        lite6_controller_plant,
        kp=config.inverse_dynamics_pid_gains.kp * np.ones(n, dtype=np.float64),
        ki=config.inverse_dynamics_pid_gains.ki * np.ones(n, dtype=np.float64),
        kd=config.inverse_dynamics_pid_gains.kd * np.ones(n, dtype=np.float64),
        has_reference_acceleration=False,
    )

    diagram = builder.Build()

    return diagram


def create_lite6_pliant(config: Lite6PliantConfig) -> Diagram:
    assert (
        config.lite6_model_type in LITE6_PLIANT_SUPPORTED_MODEL_TYPES
    ), f"Unsupported model type. Must be one of {LITE6_PLIANT_SUPPORTED_MODEL_TYPES}"

    if config.run_on_hardware:
        return create_lite6_pliant_for_hardware(config=config)
    else:
        return create_lite6_pliant_for_simulation(config=config)
