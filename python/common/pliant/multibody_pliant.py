from contextlib import contextmanager
from typing import Optional

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
