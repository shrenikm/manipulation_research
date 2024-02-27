from pydrake.common.value import Value
from pydrake.systems.framework import Context, LeafSystem
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
        self.gse_output_port = self.DeclareVectorOutputPort(
            name=LITE6_HARDWARE_PREFIX + LITE6_PLIANT_GSE_OP_NAME,
            size=LITE6_DOF,
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

    def _send_commands(self, context: Context) -> None:
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

    def _compute_state_desired_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_output_vector = np.zeros(self.num_total_states, dtype=np.float64)

        state_estimated_vector = self.se_input_port.Eval(context)
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        if self.config.lite6_control_type == Lite6ControlType.STATE:
            # For full state control, we need to route the desired positions and
            # velocities into the non gripper state values of the output.
            # The gripper values are set according to the desired flag. Desired gripper
            # velocity will be 0 with the positions being open/closed according to the
            # GSD port input.
            state_output_vector = create_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                positions_vector=positions_desired_vector,
                velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            # For velocity control, we route the desired velocities into the output
            # state's velocities. The output's positions are obtained from the input
            # estimated positions state. This is the only case where we use the SE input
            # port value.
            positions_estimated_vector = get_joint_positions_from_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                state_vector=state_estimated_vector,
            )
            state_output_vector = create_lite6_state(
                lite6_model_type=self.config.lite6_model_type,
                positions_vector=positions_estimated_vector,
                velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        else:
            raise NotImplementedError

        output_vector.SetFromVector(
            value=state_output_vector,
        )
