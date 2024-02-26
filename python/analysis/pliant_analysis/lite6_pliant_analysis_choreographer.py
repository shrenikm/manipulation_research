from __future__ import annotations

import os
from enum import Enum, auto
from typing import Iterator, Sequence

import attr
import numpy as np
import yaml
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from python.common.control.signals import (
    ControlSignal,
    SineControlSignal,
    StepControlSignal,
)
from python.common.custom_types import FilePath, PositionsVector
from python.common.exceptions import Lite6PliantChoreographerError, Lite6PliantError
from python.common.logging import MRLogger
from python.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from python.lite6.utils.lite6_model_utils import LITE6_DOF, Lite6ControlType

CC_PE_INPUT_PORT = "cc_pe_input_port"
CC_VE_INPUT_PORT = "cc_ve_input_port"
CC_OUTPUT_PORT = "cc_output_port"
CHOREOGRAPHER_CONFIG_YAML_FILENAME = "choreographer_config.yaml"


def get_choreographer_config_yaml_filepath() -> FilePath:
    return os.path.join(
        os.path.dirname(__file__),
        CHOREOGRAPHER_CONFIG_YAML_FILENAME,
    )


# TODO: Maybe move to common if generally useful
@attr.frozen
class ChoreographedSection:
    start_time_delay: float
    end_time_delay: float
    active_time: float
    start_joint_positions: PositionsVector
    control_signal: ControlSignal


@attr.frozen
class JointChoreographedSections:
    joint_index: int
    choreographed_sections: Sequence[ChoreographedSection]

    def __len__(self) -> int:
        return len(self.choreographed_sections)

    def __getitem__(self, index: int) -> ChoreographedSection:
        return self.choreographed_sections[index]


@attr.frozen
class Lite6PliantChoreographer:
    joint_choreographed_sections: Sequence[JointChoreographedSections]

    def __len__(self) -> int:
        return len(self.joint_choreographed_sections)

    def __getitem__(self, index: int) -> JointChoreographedSections:
        return self.joint_choreographed_sections[index]

    @classmethod
    def from_yaml(
        cls,
        yaml_filepath: FilePath,
    ) -> Lite6PliantChoreographer:
        with open(file=yaml_filepath, mode="r") as stream:
            parsed_yaml = yaml.safe_load(stream=stream)
        jcs = []
        for jcs_dict in parsed_yaml.values():
            joint_index = jcs_dict.pop("joint_index")
            sections = []
            for sections_dict in jcs_dict.values():
                control_signal_dict = sections_dict.pop("control_signal")
                control_signal_type = control_signal_dict.pop("type")
                if control_signal_type == "step":
                    control_signal = StepControlSignal(**control_signal_dict)
                elif control_signal_type == "sine":
                    control_signal = SineControlSignal(**control_signal_dict)
                else:
                    raise Lite6PliantChoreographerError("Invalid control signal type")
                cs = ChoreographedSection(
                    start_time_delay=sections_dict.pop("start_time_delay"),
                    end_time_delay=sections_dict.pop("end_time_delay"),
                    active_time=sections_dict.pop("active_time"),
                    start_joint_positions=np.array(
                        sections_dict.pop("start_joint_positions")
                    ),
                    control_signal=control_signal,
                )
                sections.append(cs)
            jcs.append(
                JointChoreographedSections(
                    joint_index=joint_index,
                    choreographed_sections=sections,
                )
            )
        return cls(
            joint_choreographed_sections=jcs,
        )


class ChoreographedSectionStatus(Enum):
    START_DELAY = auto()
    PRE_ACTIVE = auto()
    ACTIVE = auto()
    END_DELAY = auto()


class Lite6PliantChoreographerController(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
        choreographer: Lite6PliantChoreographer,
    ):
        # Currently only designed for velocity control
        if config.lite6_control_type != Lite6ControlType.VELOCITY:
            raise Lite6PliantError("Only velocity control is supported as of now.")

        super().__init__()

        self.config = config
        self.choreographer = choreographer
        self.kp = 0.1
        self._logger = MRLogger(self.__class__.__name__)

        # States.
        self._current_joint_index = 0
        self._current_section_index = 0
        self._status = ChoreographedSectionStatus.START_DELAY
        self._section_start_wait_time = None
        self._section_active_time = None
        self._section_end_wait_time = None
        self._done = False

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

        if self._done:
            output_vector.SetFromVector(value=np.zeros(LITE6_DOF, dtype=np.float64))
            return

        current_time = context.get_time()
        pe_vector = self.cc_pe_input_port.Eval(context)
        # print("ccpe:", pe_vector.round(2))

        jcs = self.choreographer[self._current_joint_index]
        cs = jcs[self._current_section_index]

        velocities_output_vector = np.zeros(LITE6_DOF, dtype=np.float64)

        if self._status == ChoreographedSectionStatus.START_DELAY:
            # We wait by sending zero velocities.
            if self._section_start_wait_time is None:
                self._section_start_wait_time = current_time
            elif current_time - self._section_start_wait_time > cs.start_time_delay:
                # Move to active.
                self._section_start_wait_time = None
                self._status = ChoreographedSectionStatus.PRE_ACTIVE
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Start delay done."
                )
        elif self._status == ChoreographedSectionStatus.PRE_ACTIVE:
            if np.allclose(pe_vector, cs.start_joint_positions, atol=0.002):
                self._status = ChoreographedSectionStatus.ACTIVE
                self._section_active_time = current_time
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Pre active done."
                )
            else:
                # P controller to get the joints to the start positions.
                velocities_output_vector = self.kp * (
                    cs.start_joint_positions - pe_vector
                )
        elif self._status == ChoreographedSectionStatus.ACTIVE:
            if self._section_active_time is None:
                self._section_active_time = current_time
            elif current_time - self._section_active_time > cs.active_time:
                self._section_active_time = None
                self._status = ChoreographedSectionStatus.END_DELAY
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Active done."
                )
            else:
                signal = cs.control_signal.compute_signal(
                    time_step=current_time - self._section_active_time
                )
                velocities_output_vector[self._current_joint_index] = signal

        elif self._status == ChoreographedSectionStatus.END_DELAY:
            # We wait by sending zero velocities.
            if self._section_end_wait_time is None:
                self._section_end_wait_time = current_time
            elif current_time - self._section_end_wait_time > cs.end_time_delay:
                # Move to active.
                self._section_end_wait_time = None
                self._status = ChoreographedSectionStatus.END_DELAY
                self._logger.info(
                    f"[Joint {self._current_joint_index}][Section {self._current_section_index}] End delay done."
                )

                # If this is the last section, we go to the next joint and reset the section index to 0
                if self._current_section_index == len(jcs) - 1:
                    if self._current_joint_index == len(self.choreographer) - 1:
                        # If this is also the last joint, we are done.
                        self._done = True
                        self._logger.info("Choreography done!")
                    else:
                        self._current_joint_index += 1
                        self._current_section_index = 0
                        self._logger.info(
                            f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Current joint completed. Moving to the next one."
                        )
                else:
                    # Go to the next section.
                    self._current_section_index += 1
                    self._logger.info(
                        f"[Joint {self._current_joint_index}][Section {self._current_section_index}] Current section completed. Moving to the next one."
                    )

        output_vector.SetFromVector(
            value=velocities_output_vector,
        )
