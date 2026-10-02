"""Measure one shape: volume, area, size, inertia, face types, fingerprint.

Everything here is independent of where the shape sits in the assembly, which
is exactly what the fingerprint needs (see docs/format.md).
"""

from __future__ import annotations

import hashlib
import json

from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_SurfaceType
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.TopTools import TopTools_IndexedMapOfShape

from kyumi.model import FACE_TYPES, Shape
from kyumi.reader import RawShape

SURFACE_NAMES = {
    GeomAbs_SurfaceType.GeomAbs_Plane: "plane",
    GeomAbs_SurfaceType.GeomAbs_Cylinder: "cylinder",
    GeomAbs_SurfaceType.GeomAbs_Cone: "cone",
    GeomAbs_SurfaceType.GeomAbs_Sphere: "sphere",
    GeomAbs_SurfaceType.GeomAbs_Torus: "torus",
    GeomAbs_SurfaceType.GeomAbs_BSplineSurface: "bspline",
}


def measure(raw: RawShape) -> Shape:
    geometry = raw.geometry
    volume = GProp_GProps()
    BRepGProp.VolumeProperties_s(geometry, volume)
    surface = GProp_GProps()
    BRepGProp.SurfaceProperties_s(geometry, surface)

    faces = count_faces(geometry)
    inertia = sorted(volume.PrincipalProperties().Moments())
    return Shape(
        id=raw.id,
        name=raw.name,
        volume=volume.Mass(),
        area=surface.Mass(),
        size=bounding_size(geometry),
        center=list(volume.CentreOfMass().Coord()),
        inertia=inertia,
        faces=faces,
        solids=count(geometry, TopAbs_SOLID),
        color=raw.color,
        fingerprint=fingerprint(volume.Mass(), surface.Mass(), inertia, faces),
    )


def bounding_size(geometry: TopoDS_Shape) -> list[float]:
    box = Bnd_Box()
    # "Optimal" = tight box from the real surfaces, not the loose control-point box.
    BRepBndLib.AddOptimal_s(geometry, box, False, False)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return [xmax - xmin, ymax - ymin, zmax - zmin]


def count(geometry: TopoDS_Shape, kind: int) -> int:
    found = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(geometry, kind, found)
    return found.Extent()


def count_faces(geometry: TopoDS_Shape) -> dict[str, int]:
    counts = dict.fromkeys(FACE_TYPES, 0)
    faces = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(geometry, TopAbs_FACE, faces)
    for i in range(1, faces.Extent() + 1):
        surface_type = BRepAdaptor_Surface(TopoDS.Face_s(faces.FindKey(i))).GetType()
        counts[SURFACE_NAMES.get(surface_type, "other")] += 1
    return counts


def fingerprint(volume: float, area: float, inertia: list[float], faces: dict[str, int]) -> str:
    """16 hex chars that stay put when the shape moves and change when it changes."""
    values = [round_sig(volume), round_sig(area), [round_sig(m) for m in inertia], faces]
    text = json.dumps(values, sort_keys=True)
    return hashlib.sha256(text.encode()).hexdigest()[:16]


def round_sig(value: float, digits: int = 4) -> float:
    """Round to `digits` significant digits so float noise cannot flip the hash."""
    if value == 0:
        return 0.0
    return float(f"{value:.{digits - 1}e}")
