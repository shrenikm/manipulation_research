from typing import Sequence

import attr
from pydrake.systems.framework import LeafSystem

from python.common.control.signals import ControlSignal
from python.common.custom_types import PositionsVector
from python.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from python.lite6.utils.lite6_model_utils import LITE6_DOF, Lite6ControlType


# TODO: Maybe move to common if generally useful
@attr.frozen
class ChoreographedSection:
    start_time_delay: float
    end_time_delay: float
    active_time: float
    control_signal: ControlSignal
    start_joint_positions: PositionsVector
    end_joint_positions: PositionsVector


@attr.frozen
class JointChoreographedSections:
    choreographed_sections = Sequence[ChoreographedSection]


CC_PE_INPUT_PORT = "cc_pe_input_port"
CC_VE_INPUT_PORT = "cc_ve_input_port"
CC_OUTPUT_PORT = "cc_output_port"


@attr.frozen
class Lite6PliantAnalysisChoreographer:
    joint_choreographed_sections = Sequence[JointChoreographedSections]


class Lite6PliantAnalysisChoreographerController(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
        choreographer: Lite6PliantAnalysisChoreographer,
    ):
        # Currently only designed for velocity control
        assert (
            config.lite6_control_type == Lite6ControlType.VELOCITY
        ), "Only velocity control is supproted as of now"

        super().__init__()

        self.choreographer = choreographer
        self.num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.cc_pe_input_port = self.DeclareVectorInputPort(
            name=CC_PE_INPUT_PORT,
            size=LITE6_DOF,
        )
        self.cc_ve_input_port = self.DeclareVectorInputPort(
            name=CC_VE_INPUT_PORT,
            size=LITE6_DOF,
        )

        self.cc_output_port = self.DeclareVectorOutputPort(
            name=CC_OUTPUT_PORT,
            size=LITE6_DOF,
            calc=self._compute_control_velocities_output,
        )

    def _compute_control_velocities_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_output_vector = np.zeros(self.num_total_states, dtype=np.float64)

        state_estimated_vector = self.se_input_port.Eval(context)
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        # TODO: Remove
        assert isinstance(positions_desired_vector, np.ndarray)
        assert isinstance(gripper_status_desired, Lite6GripperStatus)

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
