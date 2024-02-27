import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.systems.framework import BasicVector, Context, LeafSystem
from xarm.wrapper import XArmAPI

from python.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_GSE_OP_NAME,
    LITE6_PLIANT_PD_IP_NAME,
    LITE6_PLIANT_PE_OP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    LITE6_PLIANT_VE_OP_NAME,
    Lite6PliantConfig,
)
from python.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
)

LITE6_ROBOT_IP = "192.168.1.178"

LITE6_HARDWARE_PREFIX = "HARDWARE_"


class Lite6HardwareInterface(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self._gripper_status = Lite6GripperStatus.NEUTRAL

        # Declare a per step event to send the commands.
        self.DeclarePerStepPublishEvent(publish=self._send_commands)

        # Declare input and output ports.
        self.pd_input_port = self.DeclareVectorInputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_PD_IP_NAME,
            size=LITE6_DOF,
        )
        self.vd_input_port = self.DeclareVectorInputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_VD_IP_NAME,
            size=LITE6_DOF,
        )
        self.gsd_input_port = self.DeclareAbstractInputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_GSD_IP_NAME,
            model_value=Value(Lite6GripperStatus.CLOSED),
        )

        self.pe_output_port = self.DeclareVectorOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_PE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_positions_estimated_output,
        )
        self.ve_output_port = self.DeclareVectorOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_VE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_velocities_estimated_output,
        )
        self.gse_output_port = self.DeclareAbstractOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_GSE_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_estimated_output,
        )

        # Set up the connection to the robot and enable motion.
        self.arm = XArmAPI(
            port=LITE6_ROBOT_IP,
            is_radian=True,
        )
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)
        # Also stop the gripper
        self.arm.stop_lite6_gripper()

        # Set the mode depending on the type of control.
        if self.config.lite6_control_type == Lite6ControlType.STATE:
            self.arm.set_mode(mode=1)
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            self.arm.set_mode(mode=4)
        else:
            raise NotImplementedError("Invalid control type")

    def __del__(self):
        """
        Reset the arm once we're done using it through the system setup.
        """
        self.arm.stop_lite6_gripper()
        self.arm.reset(wait=True)
        self.arm.motion_enable(enable=False)

    def _send_commands(self, context: Context) -> None:
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        if self.config.lite6_control_type == Lite6ControlType.STATE:
            self.arm.set_servo_angle_j(
                angles=positions_desired_vector,
                speed=velocities_desired_vector,
                is_radian=True,
            )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            self.arm.vc_set_joint_velocity(
                speeds=velocities_desired_vector,
                is_radian=True,
                duration=0,
            )
        else:
            raise NotImplementedError("Invalid control type")

        if gripper_status_desired == Lite6GripperStatus.CLOSED:
            ret_code = self.arm.close_lite6_gripper()
        elif gripper_status_desired == Lite6GripperStatus.OPEN:
            ret_code = self.arm.open_lite6_gripper()
        elif gripper_status_desired == Lite6GripperStatus.NEUTRAL:
            ret_code = self.arm.close_lite6_gripper()
        else:
            raise NotImplementedError("Invalid desired gripper status")

        if ret_code == 0:
            self._gripper_status = gripper_status_desired
        else:
            # TODO: Log.
            print(f"Warning: Failed to set gripper to {gripper_status_desired}")

    def _compute_positions_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:

        ret_code, (positions_estimated_list, _, _) = self.arm.get_joint_states(
            is_radian=True
        )
        assert ret_code == 0

        output_vector.SetFromVector(
            value=np.array(positions_estimated_list, dtype=np.float64),
        )

    def _compute_velocities_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:

        ret_code, positions_estimated_list = self.arm.get_servo_angle(is_radian=True)
        assert ret_code == 0

        ret_code, (_, velocities_estimated_list, _) = self.arm.get_joint_states(
            is_radian=True
        )

        output_vector.SetFromVector(
            value=np.array(velocities_estimated_list, dtype=np.float64),
        )

    def _compute_gripper_status_estimated_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:

        output_value.set_value(self._gripper_status)
