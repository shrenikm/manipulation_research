from enum import Enum, auto
from typing import Iterator, Sequence

import attr
import numpy as np
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.control.signals import ControlSignal
from python.common.custom_types import PositionsVector
from python.common.exceptions import Lite6PliantError
from python.common.logging import MRLogger
from python.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from python.lite6.utils.lite6_model_utils import LITE6_DOF, Lite6ControlType

CC_PE_INPUT_PORT = "cc_pe_input_port"
CC_VE_INPUT_PORT = "cc_ve_input_port"
CC_OUTPUT_PORT = "cc_output_port"


# TODO: Maybe move to common if generally useful
@attr.frozen
class ChoreographedSection:
    start_time_delay: float
    end_time_delay: float
    control_signal: ControlSignal
    start_joint_positions: PositionsVector
    end_joint_positions: PositionsVector


@attr.frozen
class JointChoreographedSections:
    joint_index: int
    choreographed_sections = Sequence[ChoreographedSection]

    def __len__(self) -> int:
        return len(self.choreographed_sections)

    def __getitem__(self, index: int) -> ChoreographedSection:
        return self.choreographed_sections[index]


@attr.frozen
class Lite6PliantAnalysisChoreographer:
    joint_choreographed_sections = Sequence[JointChoreographedSections]

    def __len__(self) -> int:
        return len(self.joint_choreographed_sections)

    def __getitem__(self, index: int) -> JointChoreographedSections:
        return self.joint_choreographed_sections[index]


class ChoreographedSectionStatus(Enum):
    START_DELAY = auto()
    ACTIVE = auto()
    END_DELAY = auto()


class Lite6PliantAnalysisChoreographerController(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
        choreographer: Lite6PliantAnalysisChoreographer,
    ):
        # Currently only designed for velocity control
        if config.lite6_control_type != Lite6ControlType.VELOCITY:
            raise Lite6PliantError("Only velocity control is supported as of now.")

        super().__init__()

        self.config = config
        self.choreographer = choreographer
        self._logger = MRLogger(self.__class__.__name__)

        # States.
        self._current_joint_index = 0
        self._current_section_index = 0
        self._status = ChoreographedSectionStatus.START_DELAY
        self._section_start_wait_time = None
        self._section_end_wait_time = None

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

    def _move_to_next_section(self, current_joint_index_done: bool) -> None:
        if current_joint_index_done:
            self._current_joint_index += 1
            self._current_section_index = 0
        else:
            self._current_section_index += 1

    def _compute_control_velocities_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:

        time_sec = context.get_time()
        pe_vector = self.cc_pe_input_port.Eval(context)
        ve_vector = self.cc_ve_input_port.Eval(context)

        jcs = self.choreographer[self._current_joint_index]
        cs = jcs[self._current_section_index]

        if self._status == ChoreographedSectionStatus.START_DELAY:
            # We wait by sending zero velocities.
            velocities_output_vector = np.zeros(LITE6_DOF, dtype=np.float64)
            if self._section_start_wait_time is None:
                self._section_start_wait_time = time_sec
            elif time_sec - self._section_start_wait_time > cs.start_time_delay:
                # Move to active.
                self._section_start_wait_time = None
                self._status = ChoreographedSectionStatus.ACTIVE
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Start delay done."
                )
        elif self._status == ChoreographedSectionStatus.ACTIVE:
            ...
        elif self._status == ChoreographedSectionStatus.END_DELAY:
            # We wait by sending zero velocities.
            velocities_output_vector = np.zeros(LITE6_DOF, dtype=np.float64)
            if self._section_end_wait_time is None:
                self._section_end_wait_time = time_sec
            elif time_sec - self._section_end_wait_time > cs.end_time_delay:
                # Move to active.
                self._section_end_wait_time = None
                self._status = ChoreographedSectionStatus.START_DELAY
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] End delay done."
                )

        output_vector.SetFromVector(
            value=velocities_output_vector,
        )
