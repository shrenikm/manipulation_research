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


# Start the visualizer.
meshcat = StartMeshcat()

MANIPULATOR_DESCRIPTION_FILE_PATH = "lite6_description/drake_urdf/robot_with_gripper/lite6_robot_with_reverse_parallel_gripper.urdf"


def visualize_manipulator():
    builder = DiagramBuilder()

    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0)
    parser = Parser(plant)
    package_map = parser.package_map()
    package_map.Add(
        package_name="lite6_description",
        package_path="/home/shrenikm/Projects/drake_exp/experiment/lite6_description",
    )

    parser.AddModels(MANIPULATOR_DESCRIPTION_FILE_PATH)

    print(plant.GetFrameIndices())
    input()

    # plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link_base"))
    plant.Finalize()

    meshcat.DeleteAddedControls()

    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    sliders.Run(diagram, None)


if __name__ == "__main__":
    visualize_manipulator()
