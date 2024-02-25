from typing import Optional, Sequence

import numpy as np
from pydrake.all import (
    AbstractValue,
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JointSliders,
    LeafSystem,
    MeshcatVisualizer,
    Parser,
    RigidTransform,
    RollPitchYaw,
    StartMeshcat,
)
from pydrake.visualization import AddDefaultVisualization

from python.common.custom_types import FilePath
from python.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
    add_robot_models_to_package_map,
)
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    add_lite6_model_to_plant,
    get_drake_lite6_urdf_path,
)


def visualize_manipulator(
    lite6_model_type: Lite6ModelType,
    object_model_configs: Optional[Sequence[ObjectModelConfig]] = None,
    place_on_table: bool = True,
    show_frames: bool = False,
) -> None:
    builder = DiagramBuilder()

    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0)
    add_lite6_model_to_plant(
        plant=plant,
        lite6_model_type=lite6_model_type,
        place_on_table=place_on_table,
    )
    a = add_object_models_to_plant(
        plant=plant,
        object_model_configs=object_model_configs,
    )
    plant.Finalize()

    print(a)
    print(plant.num_positions(a[0]))

    meshcat.DeleteAddedControls()

    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    sliders.Run(diagram, None)


if __name__ == "__main__":

    meshcat = StartMeshcat()

    lite6_model_type = Lite6ModelType.ROBOT_WITH_NP_GRIPPER
    object_model_configs = [
        ObjectModelConfig(
            object_model_type=ObjectModelType.CUBE_1_INCH,
            position=[0.0, 0.0, 1.0],
        ),
    ]
    place_on_table = True
    show_frames = True
    visualize_manipulator(
        lite6_model_type=lite6_model_type,
        object_model_configs=object_model_configs,
        place_on_table=place_on_table,
        show_frames=show_frames,
    )
