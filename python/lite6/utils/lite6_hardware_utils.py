import time

import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.systems.framework import (
    BasicVector,
    Context,
    DependencyTicket,
    DiscreteValues,
    EventStatus,
    LeafSystem,
)
from xarm.wrapper import XArmAPI

from python.common.exceptions import Lite6PliantError
from python.common.logging_utils import MRLogger
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

LITE6_HARDWARE_PREFIX = "hardware_"


class Lite6HardwareInterface(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self._gripper_status = Lite6GripperStatus.NEUTRAL
        self._logger = MRLogger(self.__class__.__name__)

        _lite6_state_estimated_index = self.DeclareDiscreteState(2 * LITE6_DOF)
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=self.config.hardware_control_loop_time_step,
            offset_sec=0.0,
            update=self._estimate_state_and_send_commands,
        )

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
            prerequisites_of_calc={
                # self.all_input_ports_ticket(),
                self.discrete_state_ticket(_lite6_state_estimated_index),
            },
        )
        self.ve_output_port = self.DeclareVectorOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_VE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_velocities_estimated_output,
            prerequisites_of_calc={
                # self.all_input_ports_ticket(),
                self.discrete_state_ticket(_lite6_state_estimated_index),
            },
        )
        self.gse_output_port = self.DeclareAbstractOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_GSE_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_estimated_output,
            prerequisites_of_calc={
                # self.all_input_ports_ticket(),
                self.discrete_state_ticket(_lite6_state_estimated_index),
            },
        )

        # Set up the connection to the robot and enable motion.
        self._logger.info("Connecting to the robot ...")

        # import mock
        # self.arm = mock.MagicMock()

        self.arm = XArmAPI(
            port=LITE6_ROBOT_IP,
            is_radian=True,
        )

        self._logger.info("Connected to the robot!")

        self._logger.info("Resetting the robot before start.")

        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Set the mode depending on the type of control.
        if self.config.lite6_control_type == Lite6ControlType.STATE:
            self._logger.info("Setting robot to position control mode.")
            self.arm.set_mode(mode=1)
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            self._logger.info("Setting robot to velocity control mode.")
            self.arm.set_mode(mode=4)
        else:
            raise NotImplementedError("Invalid control type")

        # Also stop the gripper
        self.arm.stop_lite6_gripper()
        time.sleep(1.0)

    def _estimate_state_and_send_commands(
        self,
        context: Context,
        discrete_state: DiscreteValues,
    ) -> EventStatus:
        # First we estimate the current state using the xArm API.
        ret_code, (
            positions_estimated_list,
            velocities_estimated_list,
            _,
        ) = self.arm.get_joint_states(is_radian=True)
        if ret_code != 0:
            self.arm.emergency_stop()
            raise Lite6PliantError(
                f"Error while estimating state!. Error code: {ret_code}"
            )

        # The xArm API returns a vector of size 7 for the positions and velocities, so we need to drop the last value.
        positions_estimated_vector = np.array(
            positions_estimated_list, dtype=np.float64
        )[:LITE6_DOF]
        velocities_estimated_vector = np.array(
            velocities_estimated_list, dtype=np.float64
        )[:LITE6_DOF]
        state_estimated_vector = np.hstack(
            (positions_estimated_vector, velocities_estimated_vector)
        )

        discrete_state.set_value(state_estimated_vector)

        # Next we send the current commands from the input port.
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        ret_code = 0
        if self.config.lite6_control_type == Lite6ControlType.STATE:
            ret_code = self.arm.set_servo_angle_j(
                angles=positions_desired_vector,
                speed=velocities_desired_vector,
                is_radian=True,
            )
            if ret_code != 0:
                self.arm.emergency_stop()
                raise Lite6PliantError(
                    f"Error while trying to send position commands! Error code: {ret_code}"
                )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            ret_code = self.arm.vc_set_joint_velocity(
                speeds=velocities_desired_vector,
                is_radian=True,
                duration=0,
            )
            if ret_code != 0:
                self.arm.emergency_stop()
                raise Lite6PliantError(
                    f"Error while trying to send velocity commands! Error code: {ret_code}"
                )
        else:
            raise NotImplementedError("Invalid control type")

        # If the desired gripper state and current gripper state is the same don't do anything.
        # We don't want to keep commanding the same gripper state at a high frequency as it tends to break things.
        if self._gripper_status == gripper_status_desired:
            return EventStatus.Succeeded()

        ret_code = 0
        if gripper_status_desired == Lite6GripperStatus.CLOSED:
            ret_code = self.arm.close_lite6_gripper()
        elif gripper_status_desired == Lite6GripperStatus.OPEN:
            ret_code = self.arm.open_lite6_gripper() or ret_code
        elif gripper_status_desired == Lite6GripperStatus.NEUTRAL:
            ret_code = self.arm.stop_lite6_gripper() or ret_code
        else:
            raise NotImplementedError("Invalid desired gripper status")

        if ret_code != 0:
            self.arm.emergency_stop()
            raise Lite6PliantError(
                f"Error while trying to command the gripper! Error code: {ret_code}"
            )

        # Update the current gripper status.
        if ret_code == 0:
            self._gripper_status = gripper_status_desired

        return EventStatus.Succeeded()

    @staticmethod
    def get_system_name() -> str:
        return "lite6_hardware_interface"

    def reset(self):
        """
        Reset the arm once we're done using it through the system setup.
        """
        self._logger.info("Resetting the robot after end.")
        # self.arm.reset(wait=True)
        # Instead of disabling motion (which is stressful on the joints/motors), we just set the state to stop
        # so that we don't execute any stray commands by accident.
        # self.arm.set_state(state=4)

        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(mode=0)
        self.arm.set_state(state=0)
        time.sleep(1.0)
        self.arm.move_gohome(wait=True)
        self.arm.stop_lite6_gripper()
        time.sleep(1.0)

    def _send_commands(self, context: Context) -> EventStatus:
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        ret_code = 0

        if self.config.lite6_control_type == Lite6ControlType.STATE:
            ret_code = (
                self.arm.set_servo_angle_j(
                    angles=positions_desired_vector,
                    speed=velocities_desired_vector,
                    is_radian=True,
                )
                or ret_code
            )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            ret_code = (
                self.arm.vc_set_joint_velocity(
                    speeds=velocities_desired_vector,
                    is_radian=True,
                    duration=0,
                )
                or ret_code
            )
        else:
            raise NotImplementedError("Invalid control type")

        if self._gripper_status == gripper_status_desired:
            return EventStatus.Succeeded() if ret_code == 0 else EventStatus.Failed()

        if gripper_status_desired == Lite6GripperStatus.CLOSED:
            ret_code = self.arm.close_lite6_gripper() or ret_code
        elif gripper_status_desired == Lite6GripperStatus.OPEN:
            ret_code = self.arm.open_lite6_gripper() or ret_code
        elif gripper_status_desired == Lite6GripperStatus.NEUTRAL:
            ret_code = self.arm.stop_lite6_gripper() or ret_code
        else:
            raise NotImplementedError("Invalid desired gripper status")

        if ret_code == 0:
            self._gripper_status = gripper_status_desired
            return EventStatus.Succeeded()
        else:
            # self._logger.info(
            #    f"Warning: Failed to set gripper to {gripper_status_desired}"
            # )
            return EventStatus.Failed()

    def _compute_positions_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:

        state_estimated = context.get_discrete_state_vector().value()
        output_vector.set_value(state_estimated[:LITE6_DOF])
        return

    def _compute_velocities_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated = context.get_discrete_state_vector().value()
        output_vector.set_value(state_estimated[LITE6_DOF:])

    def _compute_gripper_status_estimated_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:

        output_value.set_value(self._gripper_status)
        output_value.set_value(self._gripper_status)
