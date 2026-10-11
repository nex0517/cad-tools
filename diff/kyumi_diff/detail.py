"""Detail diff of one matched part: the material that was added and removed.

Both breps are compared in the part's own frame (placement is already reported
by the "moved" tag), so a part that only moved gives nothing on either side.
added = Cut(new, old), removed = Cut(old, new). The results are meshed to
added.glb / removed.glb in the new part's own frame, like meshes/*.glb: the
viewer places them with the new node's world transform.
"""

from __future__ import annotations

import itertools
import json
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from kyumi import Model, Shape
from kyumi.mesh import mesh_glb
from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.gp import gp_Trsf
from OCP.GProp import GProp_GProps
from OCP.TopoDS import TopoDS_Shape
from OCP.TopTools import TopTools_ListOfShape

from kyumi_diff.classify import Diff
from kyumi_diff.index import KEY_FORMAT, Index, Key

FUZZY = 0.001  # mm: booleans treat faces closer than this as coincident
EMPTY = 1e-6  # mm³: a result below this is "nothing"
SAME_MOMENT = 0.01  # relative: principal moments this close make the axes ambiguous
OK = "ok"
FAILED = "could not compute"
FRAME = "the new part's own coordinates; place with node new_id's world transform"


@dataclass
class Detail:
    key: Key
    old_id: str
    new_id: str
    status: str
    added: float  # mm³
    removed: float  # mm³
    volume_change: float  # new - old, from the manifests
    aligned: bool  # the old brep had to be moved onto the new one first
    added_glb: bytes | None = None
    removed_glb: bytes | None = None

    def to_json(self, files: dict[str, str | None]) -> str:
        data = {
            "key": self.key,
            "old_id": self.old_id,
            "new_id": self.new_id,
            "status": self.status,
            "added_mm3": round(self.added, 3),
            "removed_mm3": round(self.removed, 3),
            "volume_change_mm3": round(self.volume_change, 3),
            "aligned": self.aligned,
            "frame": FRAME,
            "files": files,
        }
        return json.dumps(data, indent=2) + "\n"


def parse_key(text: str) -> Key:
    """`Drone/Frame/CenterPlate` or `Drone › Frame › CenterPlate` (a name containing
    the separator itself cannot be typed; use the key from diff.json instead)."""
    parts = [p.strip() for p in text.replace("›", "/").split("/")]
    if not all(parts):
        raise ValueError(f"empty name in key {text!r}; keys are {KEY_FORMAT}")
    return parts


def find_pair(old: Model, new: Model, result: Diff, key: Key) -> tuple[str, str]:
    """(old id, new id) of the leaf called `key` in either package, or a clear error."""
    shown = " / ".join(key)
    if key in result.added:
        raise ValueError(f"{shown} was added in the new version, so there is nothing to compare")
    if key in result.removed:
        raise ValueError(f"{shown} was removed in the new version, so there is nothing to compare")
    old_ids = {tuple(k): node_id for node_id, k in Index(old).key.items()}
    new_ids = {tuple(k): node_id for node_id, k in Index(new).key.items()}
    if tuple(key) in new_ids:
        n_id = new_ids[tuple(key)]
        o_id = next(o for o, n in result.matches.items() if n == n_id)
    elif tuple(key) in old_ids:
        o_id = old_ids[tuple(key)]
        n_id = result.matches[o_id]
    else:
        raise ValueError(f"no node {shown} in either package; keys are {KEY_FORMAT}")
    if old.node(o_id).shape is None or new.node(n_id).shape is None:
        raise ValueError(f"{shown} is a group; detail compares one part")
    return o_id, n_id


def read_brep(model: Model, shape: Shape) -> TopoDS_Shape:
    if shape.brep is None:
        raise ValueError(
            f"{model.path} has no exact geometry (imported with --no-breps); "
            "re-import it without --no-breps to use --detail"
        )
    result = TopoDS_Shape()
    with tempfile.TemporaryDirectory() as tmp_dir:  # BRepTools reads files, not streams
        path = Path(tmp_dir) / "shape.brep"
        path.write_bytes(model.read(shape.brep))
        if not BRepTools.Read_s(result, str(path), BRep_Builder()):
            raise ValueError(f"{model.path}: {shape.brep} is not a readable BREP")
    return result


def volume_of(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def cut(a: TopoDS_Shape, b: TopoDS_Shape) -> TopoDS_Shape | None:
    """a minus b, or None when OpenCascade cannot do it."""
    op = BRepAlgoAPI_Cut()
    arguments, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    arguments.Append(a)
    tools.Append(b)
    op.SetArguments(arguments)
    op.SetTools(tools)
    op.SetFuzzyValue(FUZZY)
    try:
        op.Build()
    except Exception:  # noqa: BLE001 (OpenCascade raises its own Standard_Failure family)
        return None
    return op.Shape() if op.IsDone() else None


def compare(
    old_brep: TopoDS_Shape, new_brep: TopoDS_Shape
) -> tuple[TopoDS_Shape, TopoDS_Shape] | None:
    added, removed = cut(new_brep, old_brep), cut(old_brep, new_brep)
    if added is None or removed is None:
        return None
    return added, removed


def plausible(added: float, removed: float, volume_change: float, volume: float) -> bool:
    """A real edit changes the volume by about what it adds minus what it removes; a
    brep re-exported about a new origin "adds" and "removes" nearly the whole part."""
    return added + removed <= 2 * abs(volume_change) + 0.001 * volume


def principal_axes(shape: TopoDS_Shape) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(centre, moments, axes as columns), moments ascending."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    principal = props.PrincipalProperties()
    moments = np.array(principal.Moments())
    axes = np.array(
        [
            principal.FirstAxisOfInertia().Coord(),
            principal.SecondAxisOfInertia().Coord(),
            principal.ThirdAxisOfInertia().Coord(),
        ]
    ).T
    order = np.argsort(moments)
    return np.array(props.CentreOfMass().Coord()), moments[order], axes[:, order]


def alignments(old_brep: TopoDS_Shape, new_brep: TopoDS_Shape) -> list[np.ndarray]:
    """Rigid transforms (4x4) that may put the old brep onto the new one: centre of
    mass first, then centre + principal axes in every orientation that is ambiguous
    (axis signs always; axis order too when two moments are nearly equal)."""
    c_old, m_old, a_old = principal_axes(old_brep)
    c_new, m_new, a_new = principal_axes(new_brep)
    candidates = [rigid(np.eye(3), c_old, c_new)]
    ambiguous_order = np.any(np.abs(np.diff(m_old)) <= SAME_MOMENT * m_old[1:]) or np.any(
        np.abs(np.diff(m_new)) <= SAME_MOMENT * m_new[1:]
    )
    orders = list(itertools.permutations(range(3))) if ambiguous_order else [(0, 1, 2)]
    for order in orders:
        for signs in itertools.product((1.0, -1.0), repeat=3):
            rotation = (a_new[:, list(order)] * signs) @ a_old.T
            if np.linalg.det(rotation) > 0:  # a proper rotation, not a mirror
                candidates.append(rigid(rotation, c_old, c_new))
    return candidates


def rigid(rotation: np.ndarray, c_old: np.ndarray, c_new: np.ndarray) -> np.ndarray:
    """x' = R (x - c_old) + c_new as a 4x4 matrix."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = c_new - rotation @ c_old
    return matrix


def transformed(shape: TopoDS_Shape, matrix: np.ndarray) -> TopoDS_Shape:
    trsf = gp_Trsf()
    trsf.SetValues(*matrix[:3, :].flatten().tolist())
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def detail(old: Model, new: Model, result: Diff, key: Key) -> Detail:
    o_id, n_id = find_pair(old, new, result, key)
    o_shape, n_shape = old.shapes[old.node(o_id).shape], new.shapes[new.node(n_id).shape]
    old_brep, new_brep = read_brep(old, o_shape), read_brep(new, n_shape)
    volume_change = n_shape.volume - o_shape.volume
    found = Detail(key, o_id, n_id, FAILED, 0.0, 0.0, volume_change, False)

    best = compare(old_brep, new_brep)
    volumes = (volume_of(best[0]), volume_of(best[1])) if best else None
    if best is not None and not plausible(*volumes, volume_change, o_shape.volume):
        for matrix in alignments(old_brep, new_brep):
            attempt = compare(transformed(old_brep, matrix), new_brep)
            if attempt is None:
                continue
            attempt_volumes = (volume_of(attempt[0]), volume_of(attempt[1]))
            if sum(attempt_volumes) < sum(volumes):
                best, volumes, found.aligned = attempt, attempt_volumes, True
            if plausible(*attempt_volumes, volume_change, o_shape.volume):
                break
    if best is None:
        return found
    found.status = OK
    found.added, found.removed = volumes
    if found.added > EMPTY:
        found.added_glb = mesh_glb(best[0], n_shape.size)
    if found.removed > EMPTY:
        found.removed_glb = mesh_glb(best[1], n_shape.size)
    return found


def write_detail(found: Detail, out_dir: Path) -> list[Path]:
    """detail.json plus added.glb / removed.glb (only the non-empty ones)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    files: dict[str, str | None] = {"added": None, "removed": None}
    written: list[Path] = []
    for side, data in (("added", found.added_glb), ("removed", found.removed_glb)):
        if data is not None:
            path = out_dir / f"{side}.glb"
            path.write_bytes(data)
            files[side] = path.name
            written.append(path)
    json_path = out_dir / "detail.json"
    json_path.write_text(found.to_json(files), encoding="utf-8")
    return [json_path, *written]


def render_detail(found: Detail, written: list[Path]) -> str:
    name = " › ".join(found.key)
    if found.status != OK:
        return f"detail  {name}: {found.status} (the boolean failed; see detail.json)"
    text = f"detail  {name}: added {found.added:.1f} mm³, removed {found.removed:.1f} mm³"
    if found.aligned:
        text += " (old brep aligned by centre and axes first)"
    return text + "\n" + "\n".join(f"  wrote {p}" for p in written)
