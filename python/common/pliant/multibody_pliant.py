from typing import Optional

import attr
from pydrake.geometry import MeshcatVisualizer, SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram


@attr.define
class MultibodyPliantContainer:

    diagram: Diagram
    plant: MultibodyPlant
    scene_graph: Optional[SceneGraph] = None
    meshcat: Optional[MeshcatVisualizer] = None
