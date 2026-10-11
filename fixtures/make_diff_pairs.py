"""Generate v1/v2 STEP pairs with one known change each, plus the answer diff must give.

Run from the repo root:
  python fixtures/make_diff_pairs.py                       ten drone pairs -> fixtures/pairs/drone/
  python fixtures/make_diff_pairs.py fixtures/real/x.step  four pairs -> fixtures/real/pairs/x/

A pair directory holds `v1.step`, one `<change>.step` per v2, and
`<change>.expected.json` describing the diff between them, written in terms of
node *paths* (ids are renumbered on every export, so they are useless across
files). Everything the generator knows about a model it learns through `load()`
and the `Model` API, never by opening the .kyumi zip.

The drone pairs are built by editing the Python tree in make_fixtures.py, so
geometry can change (resize, hole, add). Any other STEP file is edited in place
through OpenCascade's XCAF document: rename, move and delete only, with the
targets picked deterministically (sorted path) so reruns give the same pairs.
"""

from __future__ import annotations

import json
import math
import shutil
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
import numpy as np
from kyumi import Model, Node, import_step, load
from make_fixtures import (
    HERE,
    Group,
    Part,
    Proto,
    at,
    drone_tree,
    flatten,
    name_of,
    write_doc,
    write_step,
)
from OCP.gp import gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_Location, XCAFDoc_ShapeTool

TAGS = ("added", "removed", "moved", "renamed", "modified", "re-parented")
# From docs/diff-plan.md, "Classify": smaller differences than these are not a change.
POSITION_TOLERANCE = 0.01  # mm
ANGLE_TOLERANCE = 0.01  # degrees


@dataclass
class Expected:
    """What `kyumi diff v1 v2` must report. Paths are name lists from the root down."""

    old: str
    new: str
    note: str
    changes: list[dict] = field(default_factory=list)  # {"old": path, "new": path, "tags": [...]}
    added: list[list[str]] = field(default_factory=list)
    removed: list[list[str]] = field(default_factory=list)

    def change(self, old: list[str], new: list[str], *tags: str, **delta: float) -> None:
        entry: dict = {"old": old, "new": new, "tags": sorted(tags)}
        if delta:
            entry["delta"] = {k: round(v, 3) for k, v in delta.items()}
        self.changes.append(entry)

    def summary(self) -> dict[str, int]:
        counts = dict.fromkeys(TAGS, 0)
        for entry in self.changes:
            for tag in entry["tags"]:
                counts[tag] += 1
        counts["added"], counts["removed"] = len(self.added), len(self.removed)
        return counts

    def to_json(self) -> str:
        data = {
            "old": self.old,
            "new": self.new,
            "note": self.note,
            "changes": self.changes,
            "added": self.added,
            "removed": self.removed,
            "summary": self.summary(),
        }
        return json.dumps(data, indent=2) + "\n"


# ---------- the drone pairs: edit the Python tree ----------


def tree_paths(group: Group, prefix: tuple[str, ...] = ()) -> dict[str, list[str]]:
    """Name -> path for every node of the tree (drone names are unique)."""
    path = [*prefix, group.name]
    paths = {group.name: path}
    for child in group.children:
        if isinstance(child, Group):
            paths.update(tree_paths(child, tuple(path)))
        else:
            paths[child.name] = [*path, child.name]
    return paths


def find(group: Group, name: str) -> tuple[Group, Part | Group]:
    """The node called `name` and the group that holds it."""
    for child in group.children:
        if child.name == name:
            return group, child
        if isinstance(child, Group):
            try:
                return find(child, name)
            except KeyError:
                pass
    raise KeyError(name)


def part(group: Group, name: str) -> Part:
    found = find(group, name)[1]
    assert isinstance(found, Part), name
    return found


def world_center(tree: Group, name: str) -> np.ndarray:
    placed = next(p for p in flatten(tree, cq.Location()) if p.name == name)
    c = placed.proto.shape.val().moved(placed.loc).Center()
    return np.array([c.x, c.y, c.z])


def rename_motor(tree: Group, expected: Expected) -> None:
    before = tree_paths(tree)
    part(tree, "Motor_FL").name = "Motor_FrontLeft"
    expected.change(before["Motor_FL"], tree_paths(tree)["Motor_FrontLeft"], "renamed")


def move_motor(tree: Group, expected: Expected) -> None:
    motor = part(tree, "Motor_FR")
    motor.loc = cq.Location(cq.Vector(0, 0, 5)) * motor.loc  # 5 mm straight up
    path = tree_paths(tree)["Motor_FR"]
    expected.change(path, path, "moved", distance=5.0)


def delete_screw(tree: Group, expected: Expected) -> None:
    expected.removed.append(tree_paths(tree)["Screw_3"])
    parent, screw = find(tree, "Screw_3")
    parent.children.remove(screw)


def add_chip(tree: Group, expected: Expected) -> None:
    board = find(tree, "FlightController")[1]
    assert isinstance(board, Group)
    # Reuse the small chip's geometry: an added *copy* of an existing shape.
    board.children.append(
        Part("Chip_Extra", part(tree, "Chip_IMU").proto, at(8, -8, 0.8), "#111111")
    )
    expected.added.append(tree_paths(tree)["Chip_Extra"])


def resize_battery(tree: Group, expected: Expected) -> None:
    battery = part(tree, "Battery")
    longer = cq.Workplane().box(85, 35, 30).edges("|X").fillet(3)
    volume_change = longer.val().Volume() - battery.proto.shape.val().Volume()
    battery.proto = Proto("Battery", longer)
    path = tree_paths(tree)["Battery"]
    expected.change(path, path, "modified", volume=volume_change)


def drill_plate(tree: Group, expected: Expected) -> None:
    plate = part(tree, "CenterPlate")
    drilled = plate.proto.shape.faces(">Z").workplane().pushPoints([(0, 30)]).hole(4)
    volume_change = drilled.val().Volume() - plate.proto.shape.val().Volume()
    plate.proto = Proto("CenterPlate", drilled)
    path = tree_paths(tree)["CenterPlate"]
    expected.change(path, path, "modified", volume=volume_change)


def swap_props(tree: Group, expected: Expected) -> None:
    # Identical copies trade *names* only; each position keeps its geometry and
    # orientation. Position beats name, so diff must call this two renames.
    before = tree_paths(tree)
    fl, fr = part(tree, "Prop_FL"), part(tree, "Prop_FR")
    fl.name, fr.name = "Prop_FR", "Prop_FL"
    after = tree_paths(tree)
    expected.change(before["Prop_FL"], after["Prop_FR"], "renamed")
    expected.change(before["Prop_FR"], after["Prop_FL"], "renamed")


def rotate_frame(tree: Group, expected: Expected) -> None:
    frame = find(tree, "Frame")[1]
    frame.loc = at(angle=10) * frame.loc
    path = tree_paths(tree)["Frame"]
    # One move: the 13 parts inside keep their own transforms.
    expected.change(path, path, "moved", distance=0.0, angle=10.0)


def bake_copies(tree: Group, expected: Expected) -> None:
    """Every part becomes its own solid, already placed; the tree and names stay.

    This is what a sloppy export looks like: the four motors no longer share
    geometry, and every node transform is the identity. Nothing really changed.
    """
    for child in tree.children:
        if isinstance(child, Group):
            bake_copies(child, expected)
        else:
            baked = cq.Workplane(child.proto.shape.val().moved(child.loc))
            child.proto, child.loc = Proto(child.proto.name, baked), cq.Location()


def junk_all_names(tree: Group, expected: Expected) -> None:
    """Every node becomes Part1, Part2, ... (shapes Body1, ...): diff gets no help from names."""
    before = tree_paths(tree)
    renamed: dict[str, str] = {}
    protos: dict[int, Proto] = {}

    def visit(group: Group) -> None:
        renamed[group.name] = group.name = f"Part{len(renamed) + 1}"
        for child in group.children:
            if isinstance(child, Group):
                visit(child)
            else:
                renamed[child.name] = child.name = f"Part{len(renamed) + 1}"
                child.proto = protos.setdefault(
                    id(child.proto), Proto(f"Body{len(protos) + 1}", child.proto.shape)
                )

    visit(tree)
    after = tree_paths(tree)
    for old_name, new_name in renamed.items():
        expected.change(before[old_name], after[new_name], "renamed")


DRONE_CHANGES = {
    "rename": (rename_motor, "Motor_FL renamed to Motor_FrontLeft"),
    "move": (move_motor, "Motor_FR moved 5 mm up"),
    "delete": (delete_screw, "Screw_3 deleted"),
    "add": (add_chip, "Chip_Extra added to the FlightController (a copy of the Chip shape)"),
    "resize": (resize_battery, "Battery 10 mm longer"),
    "hole": (drill_plate, "4 mm hole drilled in CenterPlate"),
    "swap": (swap_props, "Prop_FL and Prop_FR trade names; positions unchanged (2 renamed)"),
    "rotate": (rotate_frame, "Frame group rotated 10 degrees (one move, not thirteen)"),
    "baked": (bake_copies, "same drone with every copy baked into its own solid (no change)"),
    "renamed": (junk_all_names, "every node is Part1..PartN, every shape Body1..BodyN"),
}


def make_drone_pairs(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "drone.step", out_dir / "v1.step")
    for change, (apply, note) in DRONE_CHANGES.items():
        expected = Expected("v1.step", f"{change}.step", note)
        tree = drone_tree()
        apply(tree, expected)
        write_step(tree, out_dir / f"{change}.step")
        write_expected(out_dir, change, expected)


# ---------- any STEP file: edit the XCAF document ----------


def read_doc(path: Path) -> TDocStd_Document:
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise SystemExit(f"{path}: not a STEP file OpenCascade can read")
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    if not reader.Transfer(doc):
        raise SystemExit(f"{path}: no usable geometry")
    return doc


def leaf_placements(doc: TDocStd_Document) -> list[tuple[list[str], TDF_Label]]:
    """(path, component label) for every placed part, in tree order."""
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    found: list[tuple[list[str], TDF_Label]] = []

    def visit(label: TDF_Label, path: list[str]) -> None:
        proto = TDF_Label()
        if not XCAFDoc_ShapeTool.GetReferredShape_s(label, proto):
            proto = label
        path = [*path, name_of(label)]
        if not XCAFDoc_ShapeTool.IsAssembly_s(proto):
            if XCAFDoc_ShapeTool.IsReference_s(label):
                found.append((path, label))
            return
        children = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(proto, children)
        for i in range(1, children.Length() + 1):
            visit(children.Value(i), path)

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        visit(roots.Value(i), [])
    return found


def pick_targets(doc: TDocStd_Document) -> list[list[str]]:
    """Paths of the parts to rename, move and delete: sorted, so reruns agree.

    Only parts whose path is unique and has no blank or numeric name qualify
    (a numeric placement name is OpenCascade standing in for a blank one, and
    the importer turns it back into "", which would not match the path).
    """
    paths = [path for path, _ in leaf_placements(doc)]
    unique = [p for p in paths if paths.count(p) == 1]
    usable = sorted(p for p in unique if all(name and not name.isdigit() for name in p))
    if not usable:
        return []
    return [usable[i % len(usable)] for i in range(3)]


def placement(doc: TDocStd_Document, path: list[str]) -> TDF_Label:
    return next(label for p, label in leaf_placements(doc) if p == path)


def xcaf_rename(doc: TDocStd_Document, path: list[str], expected: Expected) -> None:
    new_name = path[-1] + "_renamed"
    TDataStd_Name.Set_s(placement(doc, path), TCollection_ExtendedString(new_name))
    expected.change(path, [*path[:-1], new_name], "renamed")


def xcaf_move(doc: TDocStd_Document, path: list[str], expected: Expected) -> None:
    label = placement(doc, path)
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(5, 0, 0))
    moved = TopLoc_Location(shift) * XCAFDoc_ShapeTool.GetLocation_s(label)
    XCAFDoc_Location.Set_s(label, moved)
    XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).UpdateAssemblies()
    expected.change(path, path, "moved", distance=5.0)


def xcaf_delete(doc: TDocStd_Document, path: list[str], expected: Expected) -> None:
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    tool.RemoveComponent(placement(doc, path))
    tool.UpdateAssemblies()
    expected.removed.append(path)


def make_pairs(step: Path, out_dir: Path) -> list[str]:
    """no-change, move, delete and rename pairs for any STEP file. Returns the pair names."""
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(step, out_dir / "v1.step")
    targets = pick_targets(read_doc(step))
    plan: list[tuple[str, str, object, list[str] | None]] = [
        ("nochange", "v1 re-exported through the same writer, nothing else changed", None, None)
    ]
    if targets:
        rename, move, delete = targets
        plan += [
            ("rename", f"{' / '.join(rename)} renamed", xcaf_rename, rename),
            ("move", f"{' / '.join(move)} moved 5 mm along x", xcaf_move, move),
            ("delete", f"{' / '.join(delete)} deleted", xcaf_delete, delete),
        ]
    else:
        print(f"{step.name}: no placed parts to move, rename or delete; writing nochange only")
    for change, note, apply, path in plan:
        expected = Expected("v1.step", f"{change}.step", note)
        doc = read_doc(step)  # a fresh document per pair, so edits never pile up
        if apply is not None:
            apply(doc, path, expected)
        write_doc(doc, out_dir / f"{change}.step")
        write_expected(out_dir, change, expected)
    return [change for change, *_ in plan]


# ---------- checking an expected answer against the tag definitions ----------


def verify(old: Model, new: Model, expected: dict) -> None:
    """Raise if `expected` disagrees with docs/diff-plan.md's tag definitions.

    Matching is given, not computed: changed pairs come from `expected`, every
    other node must exist under the same path in both versions. Then each tag
    is re-derived from the two models and compared.
    """
    old_by_path = {tuple(old.path_of(n.id)): n for n in old.nodes}
    new_by_path = {tuple(new.path_of(n.id)): n for n in new.nodes}
    if len(old_by_path) < len(old.nodes) or len(new_by_path) < len(new.nodes):
        raise ValueError("node paths are not unique, cannot verify by path")

    pairs: dict[str, str] = {}  # old node id -> new node id
    tags: dict[str, list[str]] = {}  # old node id -> expected tags
    deltas: dict[str, dict[str, float]] = {}
    for change in expected["changes"]:
        o, n = old_by_path[tuple(change["old"])], new_by_path[tuple(change["new"])]
        pairs[o.id], tags[o.id], deltas[o.id] = n.id, change["tags"], change.get("delta", {})
    removed = {tuple(p) for p in expected["removed"]}
    added = {tuple(p) for p in expected["added"]}
    for path, o in old_by_path.items():
        if o.id in pairs or path in removed:
            continue
        if path not in new_by_path:
            raise AssertionError(f"{path} vanished but is not listed as removed")
        pairs[o.id], tags[o.id], deltas[o.id] = new_by_path[path].id, [], {}
    for path in added:
        if path in old_by_path or path not in new_by_path:
            raise AssertionError(f"{path} is listed as added but is not new")
    matched_new = set(pairs.values())
    unmatched = {tuple(new.path_of(n.id)) for n in new.nodes if n.id not in matched_new}
    if unmatched != added:
        raise AssertionError(f"new nodes without a match differ from 'added': {unmatched ^ added}")

    moved: set[str] = set()  # old ids of nodes tagged moved, parents are visited first
    for o in old.nodes:
        if o.id not in pairs:
            continue
        n = new.node(pairs[o.id])
        found = derive_tags(old, o, new, n, pairs, moved)
        if "moved" in found:
            moved.add(o.id)
        if sorted(found) != sorted(tags[o.id]):
            raise AssertionError(
                f"{old.path_of(o.id)}: expected {tags[o.id]}, definitions give {found}"
            )
        check_delta(old, o, new, n, deltas[o.id])
    summary = dict.fromkeys(TAGS, 0)
    for tag_list in tags.values():
        for tag in tag_list:
            summary[tag] += 1
    summary["added"], summary["removed"] = len(added), len(removed)
    if summary != expected["summary"]:
        raise AssertionError(f"summary {expected['summary']} should be {summary}")


def derive_tags(
    old: Model, o: Node, new: Model, n: Node, pairs: dict[str, str], moved: set[str]
) -> list[str]:
    found: list[str] = []
    if o.name != n.name:
        found.append("renamed")
    if (None if o.parent is None else pairs.get(o.parent)) != n.parent:
        found.append("re-parented")
    if o.shape and n.shape and old.shapes[o.shape].fingerprint != new.shapes[n.shape].fingerprint:
        found.append("modified")
    if o.shape is None:
        # A group has no geometry of its own: its placement is its world transform.
        if transforms_differ(old.world_transform(o.id), new.world_transform(n.id)):
            found.append("moved")
    elif o.parent in moved:
        # The group moved; the part only moved if its *own* transform changed.
        if transforms_differ(local(o), local(n)):
            found.append("moved")
    elif "modified" in found:
        # Editing a part shifts its centre of mass, so only the transform can tell.
        if transforms_differ(old.world_transform(o.id), new.world_transform(n.id)):
            found.append("moved")
    elif center_distance(old, o, new, n) > POSITION_TOLERANCE:
        # Same geometry: compare where it ended up, not the transform. A baked
        # export has identity transforms and the position inside the shape.
        found.append("moved")
    return found


def local(node: Node) -> np.ndarray:
    return np.array(node.transform).reshape(4, 4)


def transforms_differ(a: np.ndarray, b: np.ndarray) -> bool:
    return (
        float(np.linalg.norm(a[:3, 3] - b[:3, 3])) > POSITION_TOLERANCE
        or rotation_angle(a, b) > ANGLE_TOLERANCE
    )


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3].T @ b[:3, :3]
    cos = (np.trace(relative) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cos)))))


def center_distance(old: Model, o: Node, new: Model, n: Node) -> float:
    def world_center(model: Model, node: Node) -> np.ndarray:
        center = np.array([*model.shapes[node.shape].center, 1.0])
        return (model.world_transform(node.id) @ center)[:3]

    return float(np.linalg.norm(world_center(old, o) - world_center(new, n)))


def check_delta(old: Model, o: Node, new: Model, n: Node, delta: dict[str, float]) -> None:
    if "distance" in delta:
        if o.shape:
            distance = center_distance(old, o, new, n)
        else:
            distance = float(
                np.linalg.norm(old.world_transform(o.id)[:3, 3] - new.world_transform(n.id)[:3, 3])
            )
        if abs(distance - delta["distance"]) > POSITION_TOLERANCE:
            raise AssertionError(
                f"{old.path_of(o.id)}: moved {distance:.3f} mm, expected {delta['distance']}"
            )
    if "angle" in delta:
        angle = rotation_angle(old.world_transform(o.id), new.world_transform(n.id))
        if abs(angle - delta["angle"]) > ANGLE_TOLERANCE:
            raise AssertionError(
                f"{old.path_of(o.id)}: turned {angle:.3f} deg, expected {delta['angle']}"
            )
    if "volume" in delta:
        change = new.shapes[n.shape].volume - old.shapes[o.shape].volume
        if abs(change - delta["volume"]) > 0.01 * abs(delta["volume"]):
            raise AssertionError(
                f"{old.path_of(o.id)}: volume changed {change:.2f}, expected {delta['volume']}"
            )


def write_expected(out_dir: Path, change: str, expected: Expected) -> None:
    (out_dir / f"{change}.expected.json").write_text(expected.to_json(), encoding="utf-8")


def verify_pair(pair_dir: Path, change: str, work_dir: Path) -> None:
    """Import both versions with core and check the expected answer against the definitions."""
    expected = json.loads((pair_dir / f"{change}.expected.json").read_text(encoding="utf-8"))
    models = []
    for name in (expected["old"], expected["new"]):
        out = work_dir / f"{pair_dir.name}-{name}.kyumi"
        import_step(pair_dir / name, out)
        models.append(load(out))
    verify(models[0], models[1], expected)


def main(argv: list[str]) -> None:
    if not argv:
        out_dir = HERE / "pairs" / "drone"
        make_drone_pairs(out_dir)
        changes = list(DRONE_CHANGES)
    else:
        step = Path(argv[0])
        out_dir = HERE / "real" / "pairs" / step.stem
        changes = make_pairs(step, out_dir)
    with tempfile.TemporaryDirectory() as work:
        for change in changes:
            try:
                verify_pair(out_dir, change, Path(work))
                status = "ok"
            except ValueError as error:
                status = f"not verified ({error})"
            print(f"{out_dir.relative_to(HERE)}/{change}: {status}")


if __name__ == "__main__":
    main(sys.argv[1:])
