from typing import Tuple

import numpy as np
from pydrake.common.value import Value
from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    MultibodyPlant,
)
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Diagram, DiagramBuilder, System
from pydrake.systems.primitives import PassThrough

from python.common.class_utils import StrEnum
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_GSD_OP_NAME,
    LITE6_PLIANT_GSE_OP_NAME,
    LITE6_PLIANT_PD_IP_NAME,
    LITE6_PLIANT_PD_OP_NAME,
    LITE6_PLIANT_PE_OP_NAME,
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
    LITE6_PLIANT_VD_IP_NAME,
    LITE6_PLIANT_VD_OP_NAME,
    LITE6_PLIANT_VE_OP_NAME,
    Lite6PliantConfig,
    Lite6PliantDeMultiplexer,
    Lite6PliantMultiplexer,
)
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6GripperStatus,
    add_lite6_model_to_plant,
)


def add_and_export_pliant_input_ports(
    builder: DiagramBuilder,
) -> Tuple[System, System, System]:
    # Create the input systems to return.
    positions_desired = builder.AddNamedSystem(
        name="positions_desired",
        system=PassThrough(vector_size=LITE6_DOF),
    )
    velocities_desired = builder.AddNamedSystem(
        name="velocities_desired",
        system=PassThrough(vector_size=LITE6_DOF),
    )
    gripper_status_desired = builder.AddNamedSystem(
        name="gripper_status_desired",
        system=PassThrough(
            abstract_model_value=Value(Lite6GripperStatus.CLOSED),
        ),
    )

    # Export the ports.
    builder.ExportInput(
        input=positions_desired.get_input_port(),
        name=LITE6_PLIANT_PD_IP_NAME,
    )
    builder.ExportInput(
        input=velocities_desired.get_input_port(),
        name=LITE6_PLIANT_VD_IP_NAME,
    )
    builder.ExportInput(
        input=gripper_status_desired.get_input_port(),
        name=LITE6_PLIANT_GSD_IP_NAME,
    )

    return positions_desired, velocities_desired, gripper_status_desired


def create_lite6_pliant_for_hardware(config: Lite6PliantConfig) -> Diagram:
    raise NotImplementedError()


def create_lite6_pliant_for_simulation(config: Lite6PliantConfig) -> Diagram:
    builder: DiagramBuilder = DiagramBuilder()
    main_plant: MultibodyPlant
    scene_graph: SceneGraph
    main_plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=config.time_step_s,
    )

    if config.plant_config is not None:
        ApplyMultibodyPlantConfig(config.plant_config, main_plant)

    # Load the model
    lite6_model = add_lite6_model_to_plant(
        plant=main_plant,
        lite6_model_type=config.lite6_model_type,
    )

    main_plant.Finalize()

    nq = main_plant.num_positions(model_instance=lite6_model)
    nv = main_plant.num_velocities(model_instance=lite6_model)
    assert nq == nv

    (
        positions_desired,
        velocities_desired,
        gripper_status_desired,
    ) = add_and_export_pliant_input_ports(
        builder=builder,
    )

    lite6_controller_plant = MultibodyPlant(time_step=config.time_step_s)
    lite6_controller_model = add_lite6_model_to_plant(
        plant=lite6_controller_plant,
        lite6_model_type=config.lite6_model_type,
    )
    lite6_controller_plant.Finalize()

    id_controller = builder.AddNamedSystem(
        name="id_controller",
        system=InverseDynamicsController(
            lite6_controller_plant,
            kp=config.inverse_dynamics_pid_gains.kp,
            ki=config.inverse_dynamics_pid_gains.ki,
            kd=config.inverse_dynamics_pid_gains.kd,
            has_reference_acceleration=False,
        ),
    )

    lite6_multiplexer = builder.AddNamedSystem(
        name="lite6_multiplexer",
        system=Lite6PliantMultiplexer(config=config),
    )
    lite6_demultiplexer = builder.AddNamedSystem(
        name="lite6_demultiplexer",
        system=Lite6PliantDeMultiplexer(config=config),
    )

    # Connections to the multiplexer.
    builder.Connect(
        main_plant.get_state_output_port(
            model_instance=lite6_model,
        ),
        lite6_multiplexer.se_input_port,
    )
    builder.Connect(
        positions_desired.get_output_port(),
        lite6_multiplexer.pd_input_port,
    )
    builder.Connect(
        velocities_desired.get_output_port(),
        lite6_multiplexer.vd_input_port,
    )
    builder.Connect(
        gripper_status_desired.get_output_port(),
        lite6_multiplexer.gsd_input_port,
    )

    # Connections to the ID controller.
    builder.Connect(
        lite6_multiplexer.sd_output_port,
        id_controller.get_input_port_desired_state(),
    )
    builder.Connect(
        main_plant.get_state_output_port(
            model_instance=lite6_model,
        ),
        id_controller.get_input_port_estimated_state(),
    )
    builder.Connect(
        id_controller.get_output_port_control(),
        main_plant.get_actuation_input_port(
            model_instance=lite6_model,
        ),
    )

    #  Connections to the Demultiplexer.
    builder.Connect(
        main_plant.get_state_output_port(
            model_instance=lite6_model,
        ),
        lite6_demultiplexer.se_input_port,
    )

    # Export required output ports.
    builder.ExportOutput(
        output=positions_desired.get_output_port(),
        name=LITE6_PLIANT_PD_OP_NAME,
    )
    builder.ExportOutput(
        output=velocities_desired.get_output_port(),
        name=LITE6_PLIANT_VD_OP_NAME,
    )
    builder.ExportOutput(
        output=gripper_status_desired.get_output_port(),
        name=LITE6_PLIANT_GSD_OP_NAME,
    )
    builder.ExportOutput(
        output=lite6_demultiplexer.pe_output_port,
        name=LITE6_PLIANT_PE_OP_NAME,
    )
    builder.ExportOutput(
        output=lite6_demultiplexer.ve_output_port,
        name=LITE6_PLIANT_VE_OP_NAME,
    )
    builder.ExportOutput(
        output=lite6_demultiplexer.gse_output_port,
        name=LITE6_PLIANT_GSE_OP_NAME,
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
