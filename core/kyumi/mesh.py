"""Turn one shape into a GLB triangle mesh for the viewer."""

from __future__ import annotations

import math

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.TopTools import TopTools_IndexedMapOfShape

# Fraction of the shape's diagonal used as the chord error. 1/500 is smooth
# enough to look round in the viewer without producing huge files.
DETAIL = 1 / 500
ANGLE = math.radians(20)


def mesh_glb(geometry: TopoDS_Shape, size: list[float]) -> bytes:
    diagonal = math.sqrt(sum(s * s for s in size)) or 1.0
    # Deflection is an absolute distance, so scale it to the shape: a 2 mm
    # screw and a 2 m frame should both look smooth, not equally rough.
    BRepMesh_IncrementalMesh(geometry, diagonal * DETAIL, False, ANGLE, True)
    vertices, triangles = triangulate(geometry)
    mesh = trimesh.Trimesh(vertices=vertices, faces=triangles, process=False)
    return mesh.export(file_type="glb")


def triangulate(geometry: TopoDS_Shape) -> tuple[np.ndarray, np.ndarray]:
    """Collect the per-face triangulations OpenCascade just computed into two arrays."""
    all_vertices: list[np.ndarray] = []
    all_triangles: list[np.ndarray] = []
    offset = 0
    faces = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(geometry, TopAbs_FACE, faces)
    for i in range(1, faces.Extent() + 1):
        face = TopoDS.Face_s(faces.FindKey(i))
        location = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, location)
        if tri is None:
            continue
        transform = location.Transformation()
        vertices = np.array(
            [tri.Node(n).Transformed(transform).Coord() for n in range(1, tri.NbNodes() + 1)],
            dtype=np.float32,
        )
        triangles = np.array([tri.Triangle(t).Get() for t in range(1, tri.NbTriangles() + 1)])
        triangles = triangles - 1 + offset  # OpenCascade counts from 1
        if face.Orientation() == TopAbs_REVERSED:
            triangles = triangles[:, ::-1]  # keep normals pointing outwards
        all_vertices.append(vertices)
        all_triangles.append(triangles)
        offset += len(vertices)
    if not all_vertices:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.int64)
    return np.vstack(all_vertices), np.vstack(all_triangles)
