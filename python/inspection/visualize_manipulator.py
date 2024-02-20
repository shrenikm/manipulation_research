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
from python.common.robot_model_utils import add_robot_models_to_package_map
from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    get_drake_lite6_urdf_path,
)


def visualize_manipulator(
    manipulator_description_file_path: FilePath,
    show_frames: bool = False,
) -> None:
    builder = DiagramBuilder()

    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0)
    parser = Parser(plant)
    package_map = parser.package_map()
    add_robot_models_to_package_map(package_map=package_map)

    # Making the assumption that the first frame in the model is the base frame.
    # This is so that we can weld the base frame to the world frame.
    model_instance_index = parser.AddModels(manipulator_description_file_path)[0]
    frame_indices = plant.GetFrameIndices(model_instance=model_instance_index)
    base_frame = plant.get_frame(frame_indices[0])

    plant.WeldFrames(plant.world_frame(), base_frame)
    plant.Finalize()

    meshcat.DeleteAddedControls()

    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    sliders.Run(diagram, None)


if __name__ == "__main__":

    meshcat = StartMeshcat()

    lite6_model_type = Lite6ModelType.ROBOT_WITH_RP_GRIPPER
    manipulator_description_file_path = get_drake_lite6_urdf_path(
        lite6_model_type=lite6_model_type,
    )
    show_frames = True
    visualize_manipulator(
        manipulator_description_file_path=manipulator_description_file_path,
        show_frames=show_frames,
    )
