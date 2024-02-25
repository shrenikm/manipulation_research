from typing import Optional, Sequence

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JointSliders,
    StartMeshcat,
)
from pydrake.visualization import AddDefaultVisualization

from python.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
)
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    add_lite6_model_to_plant,
    get_default_height_for_object_model_type,
)


def analyze_lite6_pliant(
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
    add_object_models_to_plant(
        plant=plant,
        object_model_configs=object_model_configs,
    )
    plant.Finalize()

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
            position=np.array(
                [
                    0.0,
                    0.0,
                    get_default_height_for_object_model_type(
                        ObjectModelType.CUBE_1_INCH
                    ),
                ]
            ),
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
