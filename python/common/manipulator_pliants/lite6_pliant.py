from typing import Optional

import attr
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.systems.framework import Diagram, DiagramBuilder

from python.lite6.utils.lite6_model_utils import Lite6ModelType


@attr.frozen
class Lite6PliantConfig:
    model_type: Lite6ModelType
    run_on_hardware: bool
    time_step_s: float
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
