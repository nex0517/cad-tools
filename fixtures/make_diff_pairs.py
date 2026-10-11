"""Generate v1/v2 STEP pairs with one known change each, plus the answer diff must give.

Run from the repo root:
  python fixtures/make_diff_pairs.py                       drone pairs -> fixtures/pairs/drone/
  python fixtures/make_diff_pairs.py fixtures/real/x.step  four pairs -> fixtures/real/pairs/x/

A pair directory holds the v1 file, one `<change>.step` per v2, and
`<change>.expected.json` describing the diff between them. Nodes are named by
*key*: the names from the root down, where the k-th sibling with the same name
(k >= 1) is written "Name#k" (ids are renumbered on every export, so they are
useless across files). Everything the generator knows about a model it learns
through `load()` and the `Model` API, never by opening the .kyumi zip.

The drone pairs are built by editing the Python tree in make_fixtures.py, so
geometry can change (resize, hole, add). Any other STEP file is edited in place
through OpenCascade's XCAF document: rename, move and delete only, with the
targets picked deterministically (sorted key) so reruns give the same pairs.
"""

from __future__ import annotations

import json
import math
import shutil
import sys
import tempfile
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
import numpy as np
from kyumi import Model, Node, Shape, import_step, load
from kyumi_diff.index import KEY_FORMAT, Key, key_of, with_index
from make_fixtures import (
    HERE,
    Group,
    Part,
    Proto,
    arm_units_tree,
    at,
    drone_tree,
    name_of,
    restore_blank_names,
    write_doc,
    write_step,
)
from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Shape
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_Location, XCAFDoc_ShapeTool

TAGS = ("added", "removed", "moved", "renamed", "modified", "re-parented")
# From docs/diff-plan.md, "Classify": smaller differences than these are not a change.
POSITION_TOLERANCE = 0.01  # mm
ANGLE_TOLERANCE = 0.01  # degrees
MEASUREMENT_TOLERANCE = 0.001  # relative: volume, area or inertia must differ by 0.1 %


@dataclass
class Expected:
    """What `kyumi diff v1 v2` must report."""

    old: str
    new: str
    note: str
    changes: list[dict] = field(default_factory=list)  # {"old": key, "new": key, "tags": [...]}
    added: list[Key] = field(default_factory=list)
    removed: list[Key] = field(default_factory=list)

    def change(self, old: Key, new: Key, *tags: str, **delta: float) -> None:
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
            "keys": KEY_FORMAT,
            "changes": self.changes,
            "added": self.added,
            "removed": self.removed,
            "summary": self.summary(),
        }
        return json.dumps(data, indent=2) + "\n"


# ---------- the drone pairs: edit the Python tree ----------


def tree_keys(group: Group, prefix: Key = (), own: str | None = None) -> dict[int, Key]:
    """id(node) -> key for every node of the tree. Keyed by identity: names can repeat."""
    key = [*prefix, group.name if own is None else own]
    keys = {id(group): key}
    names = [child.name for child in group.children]
    for i, child in enumerate(group.children):
        child_key = [*key, with_index(child.name, names[:i].count(child.name))]
        if isinstance(child, Group):
            keys.update(tree_keys(child, key, child_key[-1]))
        else:
            keys[id(child)] = child_key
    return keys


def find(group: Group, name: str) -> tuple[Group, Part | Group]:
    """The node called `name` and the group that holds it (first match, tree order)."""
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


def rename_motor(tree: Group, expected: Expected) -> None:
    motor = part(tree, "Motor_FL")
    before = tree_keys(tree)[id(motor)]
    motor.name = "Motor_FrontLeft"
    expected.change(before, tree_keys(tree)[id(motor)], "renamed")


def move_motor(tree: Group, expected: Expected) -> None:
    motor = part(tree, "Motor_FR")
    motor.loc = cq.Location(cq.Vector(0, 0, 5)) * motor.loc  # 5 mm straight up
    key = tree_keys(tree)[id(motor)]
    expected.change(key, key, "moved", distance=5.0)


def delete_screw(tree: Group, expected: Expected) -> None:
    parent, screw = find(tree, "Screw_3")
    expected.removed.append(tree_keys(tree)[id(screw)])
    parent.children.remove(screw)


def add_chip(tree: Group, expected: Expected) -> None:
    board = find(tree, "FlightController")[1]
    assert isinstance(board, Group)
    # Reuse the small chip's geometry: an added *copy* of an existing shape.
    chip = Part("Chip_Extra", part(tree, "Chip_IMU").proto, at(8, -8, 0.8), "#111111")
    board.children.append(chip)
    expected.added.append(tree_keys(tree)[id(chip)])


def resize_battery(tree: Group, expected: Expected) -> None:
    battery = part(tree, "Battery")
    longer = cq.Workplane().box(85, 35, 30).edges("|X").fillet(3)
    volume_change = longer.val().Volume() - battery.proto.shape.val().Volume()
    battery.proto = Proto("Battery", longer)
    key = tree_keys(tree)[id(battery)]
    expected.change(key, key, "modified", volume=volume_change)


def drill_plate(tree: Group, expected: Expected) -> None:
    plate = part(tree, "CenterPlate")
    drilled = plate.proto.shape.faces(">Z").workplane().pushPoints([(0, 30)]).hole(4)
    volume_change = drilled.val().Volume() - plate.proto.shape.val().Volume()
    plate.proto = Proto("CenterPlate", drilled)
    key = tree_keys(tree)[id(plate)]
    expected.change(key, key, "modified", volume=volume_change)


def swap_props(tree: Group, expected: Expected) -> None:
    # Identical copies trade *names* only; each position keeps its geometry and
    # orientation. Position beats name, so diff must call this two renames.
    fl, fr = part(tree, "Prop_FL"), part(tree, "Prop_FR")
    before = tree_keys(tree)
    fl.name, fr.name = "Prop_FR", "Prop_FL"
    after = tree_keys(tree)
    expected.change(before[id(fl)], after[id(fl)], "renamed")
    expected.change(before[id(fr)], after[id(fr)], "renamed")


def world_loc(group: Group, target: Part | Group) -> cq.Location:
    """`target`'s placement in the root frame: the product of every loc on the way down."""
    for child in group.children:
        if child is target:
            return group.loc * child.loc
        if isinstance(child, Group):
            try:
                return group.loc * world_loc(child, target)
            except KeyError:
                pass
    raise KeyError(target.name)


def reparent_screw(tree: Group, expected: Expected) -> None:
    frame = find(tree, "Frame")[1]
    electronics = find(tree, "Electronics")[1]
    assert isinstance(frame, Group) and isinstance(electronics, Group)
    holder, screw = find(frame, "Screw_1")
    before = tree_keys(tree)[id(screw)]
    world = world_loc(tree, screw)
    holder.children.remove(screw)
    electronics.children.append(screw)
    screw.loc = world_loc(tree, electronics).inverse * world  # same place, new parent
    expected.change(before, tree_keys(tree)[id(screw)], "re-parented")


def rotate_frame(tree: Group, expected: Expected) -> None:
    frame = find(tree, "Frame")[1]
    frame.loc = at(angle=10) * frame.loc
    key = tree_keys(tree)[id(frame)]
    # One move: the 13 parts inside keep their own transforms.
    expected.change(key, key, "moved", distance=0.0, angle=10.0)


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
    before = tree_keys(tree)
    count = 0
    protos: dict[int, Proto] = {}

    def visit(group: Group) -> None:
        nonlocal count
        count += 1
        group.name = f"Part{count}"
        for child in group.children:
            if isinstance(child, Group):
                visit(child)
            else:
                count += 1
                child.name = f"Part{count}"
                child.proto = protos.setdefault(
                    id(child.proto), Proto(f"Body{len(protos) + 1}", child.proto.shape)
                )

    visit(tree)
    after = tree_keys(tree)
    for node_id, old_key in before.items():
        expected.change(old_key, after[node_id], "renamed")


def move_third_arm_unit(tree: Group, expected: Expected) -> None:
    unit = [c for c in tree.children if c.name == "ArmUnit"][2]
    unit.loc = cq.Location(cq.Vector(0, 0, 5)) * unit.loc
    key = tree_keys(tree)[id(unit)]  # ["Drone", "ArmUnit#2"]
    expected.change(key, key, "moved", distance=5.0)


@dataclass
class DronePair:
    note: str
    apply: Callable[[Group, Expected], None]
    base: str = "v1.step"  # the v1 file inside the pair directory
    tree: Callable[[], Group] = drone_tree


DRONE_PAIRS = {
    "rename": DronePair("Motor_FL renamed to Motor_FrontLeft", rename_motor),
    "move": DronePair("Motor_FR moved 5 mm up", move_motor),
    "delete": DronePair("Screw_3 deleted", delete_screw),
    "add": DronePair(
        "Chip_Extra added to the FlightController (a copy of the Chip shape)", add_chip
    ),
    "resize": DronePair("Battery 10 mm longer", resize_battery),
    "hole": DronePair("4 mm hole drilled in CenterPlate", drill_plate),
    "swap": DronePair(
        "Prop_FL and Prop_FR trade names; positions unchanged (2 renamed)", swap_props
    ),
    "reparent": DronePair(
        "Screw_1 moved from Frame into Electronics, world position kept (re-parented only)",
        reparent_screw,
    ),
    "rotate": DronePair("Frame group rotated 10 degrees (one move, not thirteen)", rotate_frame),
    "baked": DronePair(
        "same drone with every copy baked into its own solid (no change)", bake_copies
    ),
    "renamed": DronePair("every node is Part1..PartN, every shape Body1..BodyN", junk_all_names),
    "arms": DronePair(
        "third of four identical ArmUnit groups moved 5 mm up (one move)",
        move_third_arm_unit,
        base="arms_v1.step",
        tree=arm_units_tree,
    ),
}


def make_drone_pairs(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(HERE / "drone.step", out_dir / "v1.step")
    for change, pair in DRONE_PAIRS.items():
        if pair.base != "v1.step" and not (out_dir / pair.base).exists():
            write_step(pair.tree(), out_dir / pair.base)
        expected = Expected(pair.base, f"{change}.step", pair.note)
        tree = pair.tree()
        pair.apply(tree, expected)
        write_step(tree, out_dir / f"{change}.step")
        write_expected(out_dir, change, expected)


# ---------- any STEP file: edit the XCAF document ----------


@dataclass
class Placement:
    key: Key
    label: TDF_Label
    is_leaf: bool
    index: int  # among same-named siblings
    sibling_names: list[str]
    under_shared: bool  # some ancestor is a subassembly definition placed more than once
    shared_definition: bool  # this node's own definition is placed more than once


def read_doc(path: Path) -> TDocStd_Document:
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise SystemExit(f"{path}: not a STEP file OpenCascade can read")
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    if not reader.Transfer(doc):
        raise SystemExit(f"{path}: no usable geometry")
    # OpenCascade names a blank placement after its STEP entity id ("95"). The
    # importer undoes that; so must we, or the re-export would turn "" into "95".
    for placement in placements(doc):
        if name_of(placement.label).isdigit():
            TDataStd_Name.Set_s(placement.label, TCollection_ExtendedString(""))
    return doc


def placements(doc: TDocStd_Document) -> list[Placement]:
    """Every placed node (groups and parts) in tree order, with its key."""
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    found: list[Placement] = []

    def visit(label: TDF_Label, key: Key, index: int, siblings: list[str], shared: bool) -> None:
        definition = TDF_Label()
        if not XCAFDoc_ShapeTool.GetReferredShape_s(label, definition):
            definition = label
        users = XCAFDoc_ShapeTool.GetUsers_s(definition, TDF_LabelSequence(), False)
        if XCAFDoc_ShapeTool.IsReference_s(label):
            is_leaf = not XCAFDoc_ShapeTool.IsAssembly_s(definition)
            found.append(Placement(key, label, is_leaf, index, siblings, shared, users > 1))
        if not XCAFDoc_ShapeTool.IsAssembly_s(definition):
            return
        if users > 1:
            shared = True
        children = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(definition, children)
        names = [name_of(children.Value(i)) for i in range(1, children.Length() + 1)]
        for i, name in enumerate(names):
            index = names[:i].count(name)
            visit(children.Value(i + 1), [*key, with_index(name, index)], index, names, shared)

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        visit(roots.Value(i), [name_of(roots.Value(i))], 0, [], False)
    return found


def editable_parts(doc: TDocStd_Document) -> list[Placement]:
    """The parts generic mode may edit, in key order so reruns agree.

    Only parts that are the *last* of their same-named siblings qualify, so
    renaming or deleting one never shifts another node's "#k" index; blank
    names are skipped too. So are parts inside a subassembly whose definition
    is placed more than once: editing one of those edits every copy.
    """
    usable: list[Placement] = []
    skipped = 0
    for p in placements(doc):
        if not p.is_leaf:
            continue
        if p.under_shared:
            skipped += 1
            continue
        name = name_of(p.label)
        is_last = p.index == p.sibling_names.count(name) - 1
        if is_last and all(p.key):
            usable.append(p)
    if skipped:
        print(f"  skipped {skipped} parts inside subassemblies that are placed more than once")
    return sorted(usable, key=lambda p: p.key)


def pick_targets(doc: TDocStd_Document) -> list[Key]:
    """Keys of the parts to rename, move and delete."""
    usable = [p.key for p in editable_parts(doc)]
    if not usable:
        return []
    return [usable[i % len(usable)] for i in range(3)]


def pick_largest(doc: TDocStd_Document) -> Key | None:
    """Key of the editable part with the most volume: the one to drill. Parts whose
    definition is placed more than once are skipped: drilling one drills every copy."""
    usable = editable_parts(doc)
    drillable = [p for p in usable if not p.shared_definition]
    if len(drillable) < len(usable):
        skipped = len(usable) - len(drillable)
        print(f"  not drilling {skipped} parts whose geometry is placed more than once")
    if not drillable:
        return None
    return max(drillable, key=lambda p: volume_of(definition_shape(p.label))).key


def definition_shape(label: TDF_Label) -> TopoDS_Shape:
    definition = TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(label, definition)
    return XCAFDoc_ShapeTool.GetShape_s(definition)


def volume_of(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def placement(doc: TDocStd_Document, key: Key) -> Placement:
    return next(p for p in placements(doc) if p.key == key)


def xcaf_rename(doc: TDocStd_Document, key: Key, expected: Expected) -> None:
    target = placement(doc, key)
    new_name = name_of(target.label) + "_renamed"
    while new_name in target.sibling_names:
        new_name += "_"
    TDataStd_Name.Set_s(target.label, TCollection_ExtendedString(new_name))
    expected.change(key, [*key[:-1], new_name], "renamed")


def xcaf_move(doc: TDocStd_Document, key: Key, expected: Expected) -> None:
    label = placement(doc, key).label
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(5, 0, 0))
    moved = TopLoc_Location(shift) * XCAFDoc_ShapeTool.GetLocation_s(label)
    XCAFDoc_Location.Set_s(label, moved)
    XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).UpdateAssemblies()
    expected.change(key, key, "moved", distance=5.0)


def xcaf_delete(doc: TDocStd_Document, key: Key, expected: Expected) -> None:
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    label, definition = placement(doc, key).label, TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(label, definition)
    tool.RemoveComponent(label)
    # A definition nobody places any more would be written as a second root.
    if XCAFDoc_ShapeTool.GetUsers_s(definition, TDF_LabelSequence(), False) == 0:
        tool.RemoveShape(definition, False)
    tool.UpdateAssemblies()
    expected.removed.append(key)


def xcaf_drill(doc: TDocStd_Document, key: Key, expected: Expected) -> None:
    """A small hole through the part, along its thinnest direction, through the
    middle of its bounding box (or off-centre if the middle is already empty)."""
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    label, definition = placement(doc, key).label, TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(label, definition)
    if XCAFDoc_ShapeTool.GetUsers_s(definition, TDF_LabelSequence(), False) > 1:
        raise SystemExit(f"{' / '.join(key)}: placed more than once; drilling it drills every copy")
    shape = XCAFDoc_ShapeTool.GetShape_s(definition)
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, False)
    low, high = np.array(box.CornerMin().Coord()), np.array(box.CornerMax().Coord())
    sides, centre = high - low, (low + high) / 2
    axis = int(np.argmin(sides))
    radius = max(0.5, 0.05 * min(np.delete(sides, axis)))
    before = volume_of(shape)
    direction = np.eye(3)[axis]
    for offset in (0.0, 0.25, -0.25):  # fraction of the longest side, along it
        start = centre + offset * sides.max() * np.eye(3)[int(np.argmax(sides))]
        start -= direction * sides[axis]
        cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(*start), gp_Dir(*direction)), radius, 3 * sides[axis]
        ).Shape()
        drilled = BRepAlgoAPI_Cut(shape, cylinder).Shape()
        delta = volume_of(drilled) - before
        if delta < -0.001 * before:
            break
    else:
        raise SystemExit(f"{' / '.join(key)}: could not find material to drill")
    tool.SetShape(definition, drilled)
    tool.UpdateAssemblies()
    expected.change(key, key, "modified", volume=delta)


def make_pairs(step: Path, out_dir: Path) -> list[str]:
    """no-change, move, delete, rename and modify pairs for any STEP file; returns their names."""
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(step, out_dir / "v1.step")
    doc = read_doc(step)
    targets, largest = pick_targets(doc), pick_largest(doc)
    plan: list[tuple[str, str, Callable | None, Key | None]] = [
        ("nochange", "v1 re-exported through the same writer, nothing else changed", None, None)
    ]
    if targets:
        rename, move, delete = targets
        plan += [
            ("rename", f"{' / '.join(rename)} renamed", xcaf_rename, rename),
            ("move", f"{' / '.join(move)} moved 5 mm along x", xcaf_move, move),
            ("delete", f"{' / '.join(delete)} deleted", xcaf_delete, delete),
        ]
        if largest is not None:
            note = f"small hole drilled through {' / '.join(largest)}"
            plan.append(("modify", note, xcaf_drill, largest))
        else:
            print(f"{step.name}: every editable part is placed more than once; no modify pair")
    else:
        print(f"{step.name}: no placed parts to move, rename or delete; writing nochange only")
    for change, note, apply, key in plan:
        expected = Expected("v1.step", f"{change}.step", note)
        doc = read_doc(step)  # a fresh document per pair, so edits never pile up
        if apply is not None:
            apply(doc, key, expected)
        write_doc(doc, out_dir / f"{change}.step")
        restore_blank_names(out_dir / f"{change}.step")
        write_expected(out_dir, change, expected)
    return [change for change, *_ in plan]


# ---------- checking an expected answer against the tag definitions ----------


def verify(old: Model, new: Model, expected: dict) -> None:
    """Raise if `expected` disagrees with docs/diff-plan.md's tag definitions.

    Matching is given, not computed: changed pairs come from `expected`, every
    other node must exist under the same key in both versions. Then each tag
    is re-derived from the two models and compared.
    """
    old_by_key = {tuple(key_of(old, n.id)): n for n in old.nodes}
    new_by_key = {tuple(key_of(new, n.id)): n for n in new.nodes}
    pairs: dict[str, str] = {}  # old node id -> new node id
    tags: dict[str, list[str]] = {}  # old node id -> expected tags
    deltas: dict[str, dict[str, float]] = {}
    for change in expected["changes"]:
        o, n = old_by_key[tuple(change["old"])], new_by_key[tuple(change["new"])]
        pairs[o.id], tags[o.id], deltas[o.id] = n.id, change["tags"], change.get("delta", {})
    removed = {tuple(k) for k in expected["removed"]}
    added = {tuple(k) for k in expected["added"]}
    for key in removed:
        if key not in old_by_key or key in new_by_key:
            raise AssertionError(f"{key} is listed as removed but is not gone")
    for key in added:
        if key in old_by_key or key not in new_by_key:
            raise AssertionError(f"{key} is listed as added but is not new")
    for key, o in old_by_key.items():
        if o.id in pairs or key in removed:
            continue
        if key not in new_by_key:
            raise AssertionError(f"{key} vanished but is not listed as removed")
        pairs[o.id], tags[o.id], deltas[o.id] = new_by_key[key].id, [], {}
    matched_new = set(pairs.values())
    unmatched = {key for key, n in new_by_key.items() if n.id not in matched_new}
    if unmatched != added:
        raise AssertionError(f"new nodes without a match differ from 'added': {unmatched ^ added}")

    moved: set[str] = set()  # old ids of nodes tagged moved; parents are visited first
    for o in old.nodes:
        if o.id not in pairs:
            continue
        n = new.node(pairs[o.id])
        found = derive_tags(old, o, new, n, pairs, moved)
        if "moved" in found:
            moved.add(o.id)
        if sorted(found) != sorted(tags[o.id]):
            raise AssertionError(
                f"{key_of(old, o.id)}: expected {tags[o.id]}, definitions give {found}"
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
    modified = bool(o.shape and n.shape) and shapes_differ(old.shapes[o.shape], new.shapes[n.shape])
    if modified:
        found.append("modified")
    if ancestor_moved(old, o, moved):
        # Something above moved; this node only moved if its *own* transform changed.
        changed = transforms_differ(local(o), local(n))
    elif o.shape is None or modified or same_frame(old.shapes[o.shape], new.shapes[n.shape]):
        # Groups, modified parts (their centre of mass shifts) and parts whose
        # geometry is defined in the same frame: compare world transforms, which
        # also catches a rotation about the centre.
        changed = transforms_differ(old.world_transform(o.id), new.world_transform(n.id))
    else:
        # Re-baked geometry (`baked` pair: identity transform, placement inside
        # the shape): only where the centre ended up is comparable. A rotation
        # about the centre is invisible here.
        changed = center_distance(old, o, new, n) > POSITION_TOLERANCE
    if changed:
        found.append("moved")
    return found


def shapes_differ(a: Shape, b: Shape) -> bool:
    """Modified = fingerprint differs *and* a raw measurement confirms it (not just rounding)."""
    if a.fingerprint == b.fingerprint:
        return False
    if a.faces != b.faces or a.solids != b.solids:
        return True
    pairs = [(a.volume, b.volume), (a.area, b.area), *zip(a.inertia, b.inertia, strict=True)]
    return any(abs(x - y) > MEASUREMENT_TOLERANCE * max(abs(x), abs(y), 1e-9) for x, y in pairs)


def same_frame(a: Shape, b: Shape) -> bool:
    return float(np.linalg.norm(np.array(a.center) - np.array(b.center))) <= POSITION_TOLERANCE


def ancestor_moved(model: Model, node: Node, moved: set[str]) -> bool:
    current = node.parent
    while current is not None:
        if current in moved:
            return True
        current = model.node(current).parent
    return False


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
    where = key_of(old, o.id)
    if "distance" in delta:
        if o.shape:
            distance = center_distance(old, o, new, n)
        else:
            a, b = old.world_transform(o.id), new.world_transform(n.id)
            distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
        if abs(distance - delta["distance"]) > POSITION_TOLERANCE:
            raise AssertionError(f"{where}: moved {distance:.3f} mm, expected {delta['distance']}")
    if "angle" in delta:
        angle = rotation_angle(old.world_transform(o.id), new.world_transform(n.id))
        if abs(angle - delta["angle"]) > ANGLE_TOLERANCE:
            raise AssertionError(f"{where}: turned {angle:.3f} deg, expected {delta['angle']}")
    if "volume" in delta:
        change = new.shapes[n.shape].volume - old.shapes[o.shape].volume
        if abs(change - delta["volume"]) > 0.01 * abs(delta["volume"]):
            raise AssertionError(
                f"{where}: volume changed {change:.2f}, expected {delta['volume']}"
            )


def write_expected(out_dir: Path, change: str, expected: Expected) -> None:
    (out_dir / f"{change}.expected.json").write_text(expected.to_json(), encoding="utf-8")


def verify_pair(pair_dir: Path, change: str, work_dir: Path) -> None:
    """Import both versions with core and check the expected answer against the definitions."""
    expected = json.loads((pair_dir / f"{change}.expected.json").read_text(encoding="utf-8"))
    models = []
    for name in (expected["old"], expected["new"]):
        out = work_dir / f"{pair_dir.name}-{name}.kyumi"
        import_step(pair_dir / name, out, breps=False)
        models.append(load(out))
    verify(models[0], models[1], expected)


def main(argv: list[str]) -> None:
    if not argv:
        out_dir = HERE / "pairs" / "drone"
        make_drone_pairs(out_dir)
        changes = list(DRONE_PAIRS)
    else:
        step = Path(argv[0])
        out_dir = HERE / "real" / "pairs" / step.stem
        changes = make_pairs(step, out_dir)
    with tempfile.TemporaryDirectory() as work:
        for change in changes:
            verify_pair(out_dir, change, Path(work))
            print(f"{out_dir.relative_to(HERE)}/{change}: ok")


if __name__ == "__main__":
    main(sys.argv[1:])
