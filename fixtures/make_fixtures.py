"""Generate the STEP test fixtures with CadQuery.

Run from the repo root:  python fixtures/make_fixtures.py

Writes three files next to this script:
  drone.step       quadcopter with named subassemblies, colours and true instances
  drone_flat.step  the same parts, all directly under the root (no subassemblies)
  drone_messy.step the same parts the way a sloppy export looks: every copy is its
                   own solid baked into place, some parts have no colour, one has
                   no name
  bracket.step     a single part
  bracket_inch.step  the same part, with the file declaring inches instead of mm

Repeated parts (arms, motors, propellers, screws, ...) reuse the *same* CadQuery
object, which makes CadQuery write the geometry once and reference it from every
copy. `check_instancing` confirms that by counting solids in the written file.
"""

import re
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
from cadquery.occ_impl.assembly import toCAF
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_StepModelType
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

HERE = Path(__file__).parent

# Diagonal positions of the four arms (front-left, front-right, back-left, back-right).
CORNERS = {"FL": 135.0, "FR": 45.0, "BL": 225.0, "BR": 315.0}
ARM_LENGTH = 100.0
PLATE_SIZE = 80.0
PLATE_THICKNESS = 3.0
ARM_THICKNESS = 4.0


@dataclass
class Proto:
    """Unique geometry with its own name ("Motor"), separate from the instance names."""

    name: str
    shape: cq.Workplane


@dataclass
class Part:
    name: str  # instance name ("Motor_FL"); "" is allowed, real files have those
    proto: Proto
    loc: cq.Location
    color: str | None  # None = the file says nothing about this part's colour


@dataclass
class Group:
    name: str
    loc: cq.Location
    children: list["Part | Group"]


# ---------- shapes (each built once, reused for every copy) ----------


def make_plate() -> cq.Workplane:
    return (
        cq.Workplane().box(PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS).faces(">Z").workplane().hole(20)
    )


def make_arm() -> cq.Workplane:
    # Starts at x=0 so placing it is "rotate, then it points outward".
    return cq.Workplane().center(ARM_LENGTH / 2, 0).box(ARM_LENGTH, 16, ARM_THICKNESS)


def make_motor() -> cq.Workplane:
    body = cq.Workplane().circle(14).extrude(15)
    shaft = cq.Workplane().workplane(offset=15).circle(2.5).extrude(6)
    return body.union(shaft)


def make_propeller() -> cq.Workplane:
    hub = cq.Workplane().circle(4).extrude(6)
    blade = cq.Workplane().box(127, 12, 1.5).translate((0, 0, 3)).rotate((0, 0, 0), (1, 0, 0), 10)
    return hub.union(blade)


def make_screw() -> cq.Workplane:
    head = cq.Workplane().circle(2.75).extrude(3)
    shank = cq.Workplane().circle(1.5).extrude(-8)
    return head.union(shank)


def make_standoff() -> cq.Workplane:
    return cq.Workplane().polygon(6, 5.5).extrude(20).faces(">Z").workplane().hole(3)


def make_board() -> cq.Workplane:
    board = cq.Workplane().box(36, 36, 1.6)
    return board.faces(">Z").workplane().rect(30.5, 30.5, forConstruction=True).vertices().hole(3)


def make_chip(x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane().box(x, y, z).translate((0, 0, z / 2))


def make_battery() -> cq.Workplane:
    return cq.Workplane().box(75, 35, 30).edges("|X").fillet(3)


def make_bracket() -> cq.Workplane:
    # An L-bracket with a hole in each leg: a typical single part.
    profile = cq.Workplane("XZ").polyline([(0, 0), (40, 0), (40, 4), (4, 4), (4, 30), (0, 30)])
    solid = profile.close().extrude(-20)
    base_hole = cq.Workplane().center(25, 10).circle(2.5).extrude(4)
    upright_hole = cq.Workplane("YZ").center(10, 20).circle(2.5).extrude(4)
    return solid.cut(base_hole).cut(upright_hole)


# ---------- the drone tree ----------


def at(x: float = 0, y: float = 0, z: float = 0, angle: float = 0) -> cq.Location:
    """Translate to (x, y, z), then turn `angle` degrees about Z."""
    return cq.Location(cq.Vector(x, y, z), cq.Vector(0, 0, 1), angle)


def arm_tip(angle: float, along: float) -> tuple[float, float]:
    """Point `along` mm out on the arm that points at `angle` degrees."""
    v = cq.Location(cq.Vector(), cq.Vector(0, 0, 1), angle) * cq.Location(cq.Vector(along, 0, 0))
    t = v.toTuple()[0]
    return t[0], t[1]


def frame_group() -> Group:
    plate = Proto("CenterPlate", make_plate())
    arm, screw = Proto("Arm", make_arm()), Proto("Screw", make_screw())
    top = PLATE_THICKNESS / 2
    children: list[Part | Group] = [Part("CenterPlate", plate, at(), "#222222")]
    for corner, angle in CORNERS.items():
        x, y = arm_tip(angle, PLATE_SIZE / 2 - 20)
        children.append(
            Part(f"Arm_{corner}", arm, at(x, y, top + ARM_THICKNESS / 2, angle), "#333333")
        )
    screw_number = 1
    for angle in CORNERS.values():
        for along in (PLATE_SIZE / 2 - 14, PLATE_SIZE / 2 - 6):
            x, y = arm_tip(angle, along)
            loc = at(x, y, top + ARM_THICKNESS)
            children.append(Part(f"Screw_{screw_number}", screw, loc, "#888888"))
            screw_number += 1
    return Group("Frame", at(), children)


def propulsion_group() -> Group:
    motor, prop = Proto("Motor", make_motor()), Proto("Propeller", make_propeller())
    z = PLATE_THICKNESS / 2 + ARM_THICKNESS
    children: list[Part | Group] = []
    for corner, angle in CORNERS.items():
        x, y = arm_tip(angle, PLATE_SIZE / 2 - 20 + ARM_LENGTH - 10)
        children.append(Part(f"Motor_{corner}", motor, at(x, y, z), "#b0b0b0"))
        children.append(Part(f"Prop_{corner}", prop, at(x, y, z + 21, angle), "#ff7a00"))
    return Group("Propulsion", at(), children)


def electronics_group() -> Group:
    # "Part37": a junk shape name, like real exports often have.
    standoff, ic_small = Proto("Part37", make_standoff()), Proto("Chip", make_chip(3, 3, 1))
    children: list[Part | Group] = []
    # "Part37".."Part40": deliberately junk instance names too.
    for i, (x, y) in enumerate(
        [(15.25, 15.25), (-15.25, 15.25), (-15.25, -15.25), (15.25, -15.25)]
    ):
        children.append(Part(f"Part{37 + i}", standoff, at(x, y, 0), "#c0a000"))
    board = Group(
        "FlightController",
        at(0, 0, 20.8),
        [
            Part("Board", Proto("Board", make_board()), at(), "#1e7b3c"),
            Part("Chip_MCU", Proto("MCU", make_chip(10, 10, 1.2)), at(0, 0, 0.8), "#111111"),
            Part("Chip_IMU", ic_small, at(8, 8, 0.8), "#111111"),
            Part("Chip_Baro", ic_small, at(-8, 8, 0.8), "#111111"),
            Part("Connector", Proto("Connector", make_chip(6, 4, 3)), at(0, -13, 0.8), "#f0f0f0"),
        ],
    )
    children.append(board)
    return Group("Electronics", at(0, 0, PLATE_THICKNESS / 2), children)


def drone_tree() -> Group:
    battery = Proto("Battery", make_battery())
    battery_part = Part("Battery", battery, at(0, 0, -PLATE_THICKNESS / 2 - 15), "#2050c0")
    return Group(
        "Drone", at(), [frame_group(), propulsion_group(), electronics_group(), battery_part]
    )


def messy_tree() -> Group:
    """What a careless export looks like: no instances, missing colours, a blank name.

    Every copy gets its own solid, already rotated and moved into place, so the
    importer cannot rely on shared geometry: only the fingerprint can tell that
    the four motors are the same part.
    """
    parts: list[Part | Group] = []
    for part in flatten(drone_tree(), cq.Location()):
        baked = cq.Workplane(part.proto.shape.val().moved(part.loc))
        name = "" if part.name == "Chip_Baro" else part.name
        color = None if part.name in ("Arm_BL", "Screw_5") else part.color
        parts.append(Part(name, Proto(name, baked), at(), color))
    return Group("Drone", at(), parts)


# ---------- writing ----------


def to_assembly(group: Group) -> cq.Assembly:
    assy = cq.Assembly(name=group.name, loc=group.loc)
    for child in group.children:
        if isinstance(child, Group):
            assy.add(to_assembly(child), name=child.name, loc=child.loc)
        else:
            color = cq.Color(child.color) if child.color else None
            assy.add(child.proto.shape, name=child.name, loc=child.loc, color=color)
    return assy


def shape_names(group: Group) -> dict[str, str]:
    """Instance name -> shape name, for every part in the tree."""
    return {p.name: p.proto.name for p in flatten(group, cq.Location())}


def name_of(label: TDF_Label) -> str:
    # Check first: FindAttribute crashes (segfault) in OCP when there is no name.
    if not label.IsAttribute(TDataStd_Name.GetID_s()):
        return ""
    attr = TDataStd_Name()
    label.FindAttribute(TDataStd_Name.GetID_s(), attr)
    return attr.Get().ToExtString()


def fix_names(tool: XCAFDoc_ShapeTool, label: TDF_Label, shapes: dict[str, str]) -> None:
    """Make the names look like a real CAD export.

    CadQuery names a shared shape after its *first* instance ("Motor_FL" for all
    four motors) and leaves the placement of a subassembly unnamed (readers would
    show "14" instead of "Frame"). Real tools name the shape ("Motor") and every
    placement, so we do too.
    """
    components = TDF_LabelSequence()
    tool.GetComponents_s(label, components)
    for i in range(1, components.Length() + 1):
        component, target = components.Value(i), TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(component, target)
        if XCAFDoc_ShapeTool.IsAssembly_s(target):
            if not name_of(component):
                TDataStd_Name.Set_s(component, TCollection_ExtendedString(name_of(target)))
            fix_names(tool, target, shapes)
        else:
            # CadQuery replaces an empty instance name with a random UUID; put the "" back.
            instance = name_of(component) if name_of(component) in shapes else ""
            TDataStd_Name.Set_s(component, TCollection_ExtendedString(instance))
            TDataStd_Name.Set_s(target, TCollection_ExtendedString(shapes[instance]))


def write_step(group: Group, path: Path) -> None:
    """Like Assembly.export(path), but with one root and proper shape/placement names."""
    root, doc = toCAF(to_assembly(group), coloredSTEP=True)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    # toCAF wraps the top assembly in an extra placement label, which would make
    # the file read as "Drone -> Drone -> parts". Drop the wrapper, keep the assembly.
    top = TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(root, top)
    tool.RemoveShape(root, False)
    tool.UpdateAssemblies()
    fix_names(tool, top, shape_names(group))
    write_doc(doc, path)
    restore_blank_names(path)


def restore_blank_names(path: Path) -> None:
    # OpenCascade refuses to write an empty product name and puts its own
    # ("Open CASCADE STEP translator 7.9 ...") instead. We want the blank back.
    text = re.sub(r"'Open CASCADE STEP translator [^']*'", "''", path.read_text())
    path.write_text(text)


def write_single_part(
    shape: cq.Workplane, name: str, color: str, path: Path, unit: str = "MM"
) -> None:
    """A file with one named part and no assembly at all."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    label = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).AddShape(shape.val().wrapped, False)
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    colors.SetColor(label, cq.Color(color).wrapped.GetRGB(), XCAFDoc_ColorType.XCAFDoc_ColorGen)
    write_doc(doc, path, unit)


def write_doc(doc: TDocStd_Document, path: Path, unit: str = "MM") -> None:
    Interface_Static.SetCVal_s("write.step.unit", unit)
    writer = STEPCAFControl_Writer()
    writer.SetColorMode(True)
    writer.SetNameMode(True)
    writer.Transfer(doc, STEPControl_StepModelType.STEPControl_AsIs)
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise SystemExit(f"could not write {path}")


def flatten(group: Group, parent: cq.Location) -> list[Part]:
    """Every part of the tree, with its location made absolute."""
    parts: list[Part] = []
    for child in group.children:
        if isinstance(child, Group):
            parts += flatten(child, parent * child.loc)
        else:
            parts.append(Part(child.name, child.proto, parent * child.loc, child.color))
    return parts


def check_instancing(path: Path, expected_solids: int, expected_parts: int) -> None:
    """Fail loudly if CadQuery copied geometry instead of writing instances.

    In STEP, each unique solid is one MANIFOLD_SOLID_BREP entity and each placed
    copy is one NEXT_ASSEMBLY_USAGE_OCCURRENCE, so counting them is enough.
    """
    text = path.read_text()
    solids = text.count("MANIFOLD_SOLID_BREP(")
    uses = text.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE(")
    print(f"{path.name}: {solids} unique solids, {uses} placed occurrences")
    if solids != expected_solids or uses < expected_parts:
        raise SystemExit(f"{path.name}: expected {expected_solids} solids, geometry was copied")


def count_unique(parts: list[Part]) -> int:
    return len({id(p.proto) for p in parts})


def main() -> None:
    tree = drone_tree()
    parts = flatten(tree, cq.Location())

    write_step(tree, HERE / "drone.step")
    check_instancing(HERE / "drone.step", count_unique(parts), len(parts))

    flat = Group("Drone", at(), list(parts))
    write_step(flat, HERE / "drone_flat.step")
    check_instancing(HERE / "drone_flat.step", count_unique(parts), len(parts))

    # The messy file is *meant* to copy geometry: one solid per part.
    write_step(messy_tree(), HERE / "drone_messy.step")
    check_instancing(HERE / "drone_messy.step", len(parts), len(parts))

    bracket = make_bracket()
    write_single_part(bracket, "Bracket", "#7f8c8d", HERE / "bracket.step")
    # Same part, but the file declares inches: importers must convert it to mm.
    write_single_part(bracket, "Bracket", "#7f8c8d", HERE / "bracket_inch.step", unit="INCH")
    print(f"wrote {len(parts)} parts ({count_unique(parts)} unique), messy copy, brackets")


if __name__ == "__main__":
    main()
