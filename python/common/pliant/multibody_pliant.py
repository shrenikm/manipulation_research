from contextlib import contextmanager
from typing import Generator, Optional

import attr
from pydrake.geometry import Meshcat, SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram


@attr.define
class MultibodyPliantContainer:
    """
    For hardware, the plant, scene_graph and meshcat will all be None
    """

    diagram: Diagram
    plant: MultibodyPlant
    scene_graph: Optional[SceneGraph] = None
    meshcat: Optional[Meshcat] = None

    @contextmanager
    def auto_meshcat_recording(self) -> Generator[None, None, None]:
        """
        Sets up meshcat recording if a meshcat instance is given.

        Usage:
            with container.auto_meshcat_recording():
                simulator.AdvanceTo(...)
        """
        if self.meshcat is not None:
            self.meshcat.StartRecording(set_visualizations_while_recording=False)
        yield
        if self.meshcat is not None:
            self.meshcat.StopRecording()
            self.meshcat.PublishRecording()
