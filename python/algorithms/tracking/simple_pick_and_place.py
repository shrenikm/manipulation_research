"""
Simple pick and place of a 1 inch block.
"""
import numpy as np
from pydrake.all import DiagramBuilder
from pydrake.common.value import Value
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import Diagram, EventStatus
from pydrake.trajectories import PiecewisePose

from python.analysis.pliant_analysis.lite6_pliant_analysis_choreographer import (
    Lite6PliantChoreographer,
    Lite6PliantChoreographerController,
    get_choreographer_config_yaml_filepath,
)
from python.common.model_utils import ObjectModelConfig, ObjectModelType
from python.lite6.pliant.lite6_pliant import create_lite6_pliant
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_PE_OP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    LITE6_PLIANT_VE_OP_NAME,
    Lite6PliantConfig,
    Lite6PliantType,
    create_simulator_for_lite6_pliant,
    get_tuned_pid_gains_for_pliant_id_controller,
)
from python.lite6.systems.lite6_gripper_status_source import Lite6GripperStatusSource
from python.lite6.utils.lite6_model_utils import (
    LITE6_GRIPPER_ACTIVATION_TIME,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    get_default_height_for_object_model_type,
    get_lite6_urdf_eef_tip_frame_name,
)

OBJECT_TO_GRIPPER_Z = 0.0
G_TO_F_Z = -0.05
G_F_TIME = 2.0
G_WAIT_TIME = 2.0


def execute_simple_pick_and_place(
    config: Lite6PliantConfig,
    X_WOPick: RigidTransform,
    X_WOPlace: RigidTransform,
) -> None:

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )
    # TODO: Handle plant stuff for hardware pliant
    main_plant = lite6_pliant_container.plant

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.pliant_diagram,
    )

    main_plant_context = main_plant.CreateDefaultContext()
    X_WG = main_plant.EvalBodyPoseInWorld(
        context=main_plant_context,
        body=main_plant.GetBodyByName(
            name=get_lite6_urdf_eef_tip_frame_name(
                lite6_model_type=config.lite6_model_type,
            ),
        ),
    )
    # X_OPickGPick = X_OPlaceGPlace = X_OG
    X_OG = RigidTransform(
        R=RotationMatrix.MakeXRotation(theta=np.pi),
        p=np.array([0.0, 0.0, OBJECT_TO_GRIPPER_Z], dtype=np.float64),
    )
    X_GF = RigidTransform(
        p=np.array([0.0, 0.0, G_TO_F_Z], dtype=np.float64),
    )
    X_WGPick = X_WOPick @ X_OG
    X_WGPlace = X_WOPlace @ X_OG

    X_WFPick = X_WGPick @ X_GF
    X_WFPlace = X_WGPlace @ X_GF

    # Constructing the trajectory. We have some duplicate poses
    # to wait for gripper open/close.
    poses = [
        X_WG,
        X_WFPick,
        X_WGPick,
        X_WGPick,
        X_WFPlace,
        X_WGPlace,
        X_WGPlace,
        X_WFPlace,
        X_WG,
    ]
    # To compute the times estimate, we use an estimated required gripper velocity and the
    # Euclidean distance between poses.
    gripper_velocity = 0.03
    GFPick_distance = np.linalg.norm(X_WG.translation() - X_WFPick.translation())
    GFPick_time = GFPick_distance / gripper_velocity

    GPickFPlace_distance = np.linalg.norm(
        X_WGPick.translation() - X_WFPlace.translation()
    )
    GPickFPlace_time = GPickFPlace_distance / gripper_velocity

    FPlaceG_distance = np.linalg.norm(X_WFPlace.translation() - X_WG.translation())
    FPlaceG_time = FPlaceG_distance / gripper_velocity

    print(GFPick_distance, GPickFPlace_distance)
    print(GFPick_time, GPickFPlace_time)

    pick_time = GFPick_time + G_F_TIME + G_WAIT_TIME

    times = [
        0.0,
        GFPick_time,
        GFPick_time + G_F_TIME,
        pick_time,
        pick_time + GPickFPlace_time,
        pick_time + GPickFPlace_time + G_F_TIME,
        pick_time + GPickFPlace_time + G_F_TIME + G_WAIT_TIME,
        pick_time + GPickFPlace_time + G_F_TIME + G_WAIT_TIME + G_F_TIME,
        pick_time + GPickFPlace_time + G_F_TIME + G_WAIT_TIME + G_F_TIME + FPlaceG_time,
    ]

    X_WG_trajectory = PiecewisePose.MakeLinear(
        times=times,
        poses=poses,
    )
    V_WG_trajectory = X_WG_trajectory.MakeDerivative(derivative_order=1)

    gripper_times = [
        0.0,
        LITE6_GRIPPER_ACTIVATION_TIME,
        GFPick_time + G_F_TIME,
        pick_time + GPickFPlace_time + G_F_TIME,
        pick_time + GPickFPlace_time + G_F_TIME + LITE6_GRIPPER_ACTIVATION_TIME,
    ]
    gripper_statuses = [
        Lite6GripperStatus.OPEN,
        Lite6GripperStatus.NEUTRAL,
        Lite6GripperStatus.CLOSED,
        Lite6GripperStatus.OPEN,
        Lite6GripperStatus.NEUTRAL,
    ]

    gripper_status_source = builder.AddSystem(
        Lite6GripperStatusSource(
            times=gripper_times,
            statuses=gripper_statuses,
        ),
    )

    builder.Connect(
        gripper_status_source.get_output_port(),
        lite6_pliant.GetInputPort(LITE6_PLIANT_GSD_IP_NAME),
    )

    # builder.Connect(
    #    lite6_pliant.GetOutputPort(LITE6_PLIANT_PE_OP_NAME),
    #    choreographer_controller.cc_pe_input_port,
    # )
    # builder.Connect(
    #    lite6_pliant.GetOutputPort(LITE6_PLIANT_VE_OP_NAME),
    #    choreographer_controller.cc_ve_input_port,
    # )
    # builder.Connect(
    #    choreographer_controller.cc_output_port,
    #    lite6_pliant.GetInputPort(LITE6_PLIANT_VD_IP_NAME),
    # )

    # diagram = builder.Build()
    # simulator = create_simulator_for_lite6_pliant(
    #    config=config,
    #    diagram=diagram,
    # )
    # simulator_context = simulator.get_mutable_context()
    # lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)

    # lite6_pliant.GetInputPort(
    #    port_name=LITE6_PLIANT_GSD_IP_NAME,
    # ).FixValue(lite6_pliant_context, Value(Lite6GripperStatus.NEUTRAL))

    # diagram.ForcedPublish(simulator_context)

    # def simulation_end_monitor(*_):
    #    if choreographer_controller.is_done():
    #        return EventStatus.ReachedTermination(
    #            diagram, "Choreography and recording done!"
    #        )

    ## Monitor to stop the simulation after choreography is done.
    # simulator.set_monitor(
    #    monitor=simulation_end_monitor,
    # )

    # with lite6_pliant_container.auto_meshcat_recording():
    #    simulator.AdvanceTo(
    #        boundary_time=np.inf,
    #        interruptible=True,
    #    )


if __name__ == "__main__":

    lite6_model_type = Lite6ModelType.ROBOT_WITH_RP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.SIMULATION
    id_controller_pid_gains = get_tuned_pid_gains_for_pliant_id_controller(
        lite6_control_type=lite6_control_type,
    )
    time_step = 0.001
    hardware_control_loop_time_step = 0.001
    plant_config = MultibodyPlantConfig(time_step=time_step)

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
    pick_object = object_model_configs[0]

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
        inverse_dynamics_pid_gains=id_controller_pid_gains,
        plant_config=plant_config,
        hardware_control_loop_time_step=hardware_control_loop_time_step,
        object_model_configs=object_model_configs,
    )

    X_WOPick = RigidTransform(p=pick_object.position)
    place_position = np.copy(pick_object.position)
    place_position[0] += 0.1
    X_WOPlace = RigidTransform(p=place_position)
    print(X_WOPick)
    print(X_WOPlace)

    execute_simple_pick_and_place(
        config=lite6_pliant_config,
        X_WOPick=X_WOPick,
        X_WOPlace=X_WOPlace,
    )