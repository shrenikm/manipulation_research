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
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import PassThrough, PassThrough_

from python.common.class_utils import StrEnum
from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_SUPPORTED_MODEL_TYPES,
    Lite6PliantConfig,
)
from python.lite6.utils.lite6_model_utils import LITE6_DOF, add_lite6_model_to_plant

LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME = "positions_desired_input"
LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME = "velocities_desired_input"
LITE6_PLIANT_GRIPPER_CLOSED_STATUS_IP_NAME = "gripper_closed_status_input"


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

    positions_desired = builder.AddNamedSystem(
        name="positions_desired",
        system=PassThrough(vector_size=LITE6_DOF),
    )
    velocities_desired = builder.AddNamedSystem(
        name="velocities_desired",
        system=PassThrough(vector_size=LITE6_DOF),
    )
    gripper_closed_status = builder.AddNamedSystem(
        name="gripper_closed_status",
        system=PassThrough(abstract_model_value=Value(False)),
    )

    lite6_controller_plant = MultibodyPlant(time_step=config.time_step_s)
    lite6_controller_model = add_lite6_model_to_plant(
        plant=lite6_controller_plant,
        lite6_model_type=config.lite6_model_type,
    )
    lite6_controller_plant.Finalize()

    id_controller = InverseDynamicsController(
        lite6_controller_plant,
        kp=config.inverse_dynamics_pid_gains.kp * np.ones(nq, dtype=np.float64),
        ki=config.inverse_dynamics_pid_gains.ki * np.ones(nq, dtype=np.float64),
        kd=config.inverse_dynamics_pid_gains.kd * np.ones(nq, dtype=np.float64),
        has_reference_acceleration=False,
    )

    # Exporting ports.
    builder.ExportInput(
        input=positions_desired.get_input_port(),
        name=LITE6_PLIANT_POSITIONS_DESIRED_IP_NAME,
    )
    builder.ExportInput(
        input=velocities_desired.get_input_port(),
        name=LITE6_PLIANT_VELOCITIES_DESIRED_IP_NAME,
    )
    builder.ExportInput(
        input=gripper_closed_status.get_input_port(),
        name=LITE6_PLIANT_GRIPPER_CLOSED_STATUS_IP_NAME,
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
