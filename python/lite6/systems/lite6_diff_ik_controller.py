import numpy as np
from pydrake.common.value import Value
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsStatus,
    DoDifferentialInverseKinematics,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.exceptions import Lite6SystemError
from python.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    LITE6_GRIPPER_DOF,
    Lite6ControlType,
    Lite6ModelGroups,
    get_default_lite6_joint_positions,
    get_lite6_urdf_eef_tip_frame_name,
)

IK_PE_IP_NAME = "pe_input"
IK_VE_IP_NAME = "ve_input"
IK_GVD_IP_NAME = "gvd_input"
IK_VD_OP_NAME = "vd_output"


class Lite6DiffIKController(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
        plant: MultibodyPlant,
    ):
        # Currently only designed for velocity control
        if config.lite6_control_type != Lite6ControlType.VELOCITY:
            raise Lite6SystemError("Only velocity control is supported as of now.")
        if config.lite6_model_type not in Lite6ModelGroups.LITE6_ROBOT_MODELS:
            raise Lite6SystemError("Invalid lite6 model type.")

        super().__init__()

        self._config = config
        self._plant = plant

        self._context = plant.CreateDefaultContext()
        self._eef_tip_frame = plant.GetBodyByName(
            name=get_lite6_urdf_eef_tip_frame_name(
                lite6_model_type=config.lite6_model_type,
            ),
        ).body_frame()

        self._diff_ik_parameters = DifferentialInverseKinematicsParameters(
            num_positions=plant.num_positions(),
            num_velocities=plant.num_velocities(),
        )
        self._diff_ik_parameters.set_nominal_joint_position(
            get_default_lite6_joint_positions(
                lite6_model_type=config.lite6_model_type,
            ),
        )
        self._diff_ik_parameters.set_time_step(dt=config.plant_config.time_step)
        self._diff_ik_parameters.set_joint_position_limits(
            (plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()),
        )
        self._diff_ik_parameters.set_joint_velocity_limits(
            (plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()),
        )
        self._diff_ik_parameters.set_joint_acceleration_limits(
            (plant.GetAccelerationLowerLimits(), plant.GetAccelerationUpperLimits()),
        )

        # Input ports.
        self.ik_pe_input_port = self.DeclareVectorInputPort(
            name=IK_PE_IP_NAME,
            size=LITE6_DOF,
        )
        self.ik_ve_input_port = self.DeclareVectorInputPort(
            name=IK_VE_IP_NAME,
            size=LITE6_DOF,
        )
        self.ik_gvd_input_port = self.DeclareVectorInputPort(
            name=IK_GVD_IP_NAME,
            size=6,
        )

        self.ik_vd_output_port = self.DeclareVectorOutputPort(
            name=IK_VD_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_diff_ik_velocities_output,
        )

    def _compute_diff_ik_velocities_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:

        joint_q = self.ik_pe_input_port.Eval(context)
        joint_v = self.ik_ve_input_port.Eval(context)
        gripper_V = self.ik_gvd_input_port.Eval(context)

        # TODO: Hacky, setting gripper positions and velocities to zero now.
        # We will remove this part of the jacobian anyway, so it doesn't matter
        # what the value here is, but it's cleaner to give it the actual full
        # position plant estimates.
        if (
            self._config.lite6_model_type
            in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
        ):
            joint_q = np.hstack((joint_q, np.zeros(LITE6_GRIPPER_DOF)))
            joint_v = np.hstack((joint_v, np.zeros(LITE6_GRIPPER_DOF)))

        self._plant.SetPositions(
            self._context,
            joint_q,
        )
        jacobian = self._plant.CalcJacobianSpatialVelocity(
            context=self._context,
            with_respect_to=JacobianWrtVariable.kV,
            frame_B=self._eef_tip_frame,
            p_BoBp_B=np.zeros(3, dtype=np.float64),
            frame_A=self._plant.world_frame(),
            frame_E=self._plant.world_frame(),
        )
        print(jacobian)
        print(jacobian.shape)
        input()

        diff_ik_result = DoDifferentialInverseKinematics(
            joint_q,
            joint_v,
            gripper_V,
            jacobian,
            self._diff_ik_parameters,
        )

        if diff_ik_result.status != DifferentialInverseKinematicsStatus.kSolutionFound:
            raise Lite6SystemError("Diff IK failed!")
        output_vector.SetFromVector(
            value=diff_ik_result.joint_velocities(),
        )
