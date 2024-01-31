import attr
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    MultibodyPlant,
)
from pydrake.systems.framework import Diagram


@attr.frozen
class Lite6PliantConfig:
    run_on_hardware: bool
    time_step_s: float


def create_lite6_pliant(config: Lite6PliantConfig) -> Diagram:

    builder = DiagramBuilder()
    plant: MultibodyPlant, scene_graph: SceneGraph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=config.time_step_s,
    )
