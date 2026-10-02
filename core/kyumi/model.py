"""The in-memory picture of a .kyumi package: shapes, nodes, and the tree.

Everything here is plain data. Reading STEP, measuring and meshing live in
other modules; diff and categorize only ever need this file and `load()`.
"""

from __future__ import annotations

import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

FORMAT = "kyumi"
VERSION = 1

# Surface types we count per shape; "other" catches everything else (see format.md).
FACE_TYPES = ("plane", "cylinder", "cone", "sphere", "torus", "bspline", "other")

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


@dataclass
class Shape:
    """One piece of unique geometry. All lengths in mm."""

    id: str
    name: str
    volume: float
    area: float
    size: list[float]
    center: list[float]
    inertia: list[float]
    faces: dict[str, int]
    solids: int
    color: str | None
    fingerprint: str
    mesh: str | None = None  # path inside the zip, e.g. "meshes/s1.glb"
    brep: str | None = None  # path inside the zip, e.g. "breps/s1.brep"


@dataclass
class Node:
    """One place in the assembly tree. `shape` is None for a pure group."""

    id: str
    name: str
    parent: str | None
    shape: str | None
    transform: list[float] = field(default_factory=lambda: list(IDENTITY))
    color: str | None = None


@dataclass
class Model:
    source_file: str
    source_units: str
    shapes: dict[str, Shape]
    nodes: list[Node]
    path: Path | None = None  # the .kyumi this came from, so meshes/breps can be read

    def node(self, node_id: str) -> Node:
        return next(n for n in self.nodes if n.id == node_id)

    def children(self, node_id: str | None) -> list[Node]:
        return [n for n in self.nodes if n.parent == node_id]

    def roots(self) -> list[Node]:
        return self.children(None)

    def path_of(self, node_id: str) -> list[str]:
        """Names from the root down to this node, e.g. ['Drone', 'Propulsion', 'Motor_FL']."""
        names: list[str] = []
        current: str | None = node_id
        while current is not None:
            node = self.node(current)
            names.append(node.name)
            current = node.parent
        return names[::-1]

    def world_transform(self, node_id: str) -> np.ndarray:
        """4x4 matrix placing the node in world coordinates (parents applied first)."""
        node = self.node(node_id)
        local = np.array(node.transform).reshape(4, 4)
        if node.parent is None:
            return local
        return self.world_transform(node.parent) @ local

    def copies(self, shape_id: str) -> list[Node]:
        return [n for n in self.nodes if n.shape == shape_id]

    def color_of(self, node: Node) -> str | None:
        """Instance colour wins over shape colour."""
        if node.color:
            return node.color
        return self.shapes[node.shape].color if node.shape else None

    def read(self, inner_path: str) -> bytes:
        """Raw bytes of a file inside the package (a mesh, a brep, model.step)."""
        if self.path is None:
            raise ValueError("this model is not backed by a .kyumi file")
        with zipfile.ZipFile(self.path) as zf:
            return zf.read(inner_path)
