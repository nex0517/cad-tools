"""Generate the STEP test fixtures with CadQuery.

Run from the repo root:  python fixtures/make_fixtures.py

Writes three files next to this script:
  drone.step       quadcopter with named subassemblies, colours and true instances
  drone_flat.step  the same parts, all directly under the root (no subassemblies)
  bracket.step     a single part
  bracket_inch.step  the same part, with the file declaring inches instead of mm

Repeated parts (arms, motors, propellers, screws, ...) reuse the *same* CadQuery
object, which makes CadQuery write the geometry once and reference it from every
copy. `check_instancing` confirms that by counting solids in the written file.
"""

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
class Part:
    name: str
    shape: cq.Workplane
    loc: cq.Location
    color: str


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
    plate, arm, screw = make_plate(), make_arm(), make_screw()
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
    motor, prop = make_motor(), make_propeller()
    z = PLATE_THICKNESS / 2 + ARM_THICKNESS
    children: list[Part | Group] = []
    for corner, angle in CORNERS.items():
        x, y = arm_tip(angle, PLATE_SIZE / 2 - 20 + ARM_LENGTH - 10)
        children.append(Part(f"Motor_{corner}", motor, at(x, y, z), "#b0b0b0"))
        children.append(Part(f"Prop_{corner}", prop, at(x, y, z + 21, angle), "#ff7a00"))
    return Group("Propulsion", at(), children)


def electronics_group() -> Group:
    standoff, ic_small = make_standoff(), make_chip(3, 3, 1)
    children: list[Part | Group] = []
    # "Part37".."Part40": deliberately junk names, like real exports often have.
    for i, (x, y) in enumerate(
        [(15.25, 15.25), (-15.25, 15.25), (-15.25, -15.25), (15.25, -15.25)]
    ):
        children.append(Part(f"Part{37 + i}", standoff, at(x, y, 0), "#c0a000"))
    board = Group(
        "FlightController",
        at(0, 0, 20.8),
        [
            Part("Board", make_board(), at(), "#1e7b3c"),
            Part("Chip_MCU", make_chip(10, 10, 1.2), at(0, 0, 0.8), "#111111"),
            Part("Chip_IMU", ic_small, at(8, 8, 0.8), "#111111"),
            Part("Chip_Baro", ic_small, at(-8, 8, 0.8), "#111111"),
            Part("Connector", make_chip(6, 4, 3), at(0, -13, 0.8), "#f0f0f0"),
        ],
    )
    children.append(board)
    return Group("Electronics", at(0, 0, PLATE_THICKNESS / 2), children)


def drone_tree() -> Group:
    battery = Part("Battery", make_battery(), at(0, 0, -PLATE_THICKNESS / 2 - 15), "#2050c0")
    return Group("Drone", at(), [frame_group(), propulsion_group(), electronics_group(), battery])


# ---------- writing ----------


def to_assembly(group: Group) -> cq.Assembly:
    assy = cq.Assembly(name=group.name, loc=group.loc)
    for child in group.children:
        if isinstance(child, Group):
            assy.add(to_assembly(child), name=child.name, loc=child.loc)
        else:
            assy.add(child.shape, name=child.name, loc=child.loc, color=cq.Color(child.color))
    return assy


def name_of(label: TDF_Label) -> str:
    # Check first: FindAttribute crashes (segfault) in OCP when there is no name.
    if not label.IsAttribute(TDataStd_Name.GetID_s()):
        return ""
    attr = TDataStd_Name()
    label.FindAttribute(TDataStd_Name.GetID_s(), attr)
    return attr.Get().ToExtString()


def name_subassembly_copies(tool: XCAFDoc_ShapeTool, label: TDF_Label) -> None:
    """Give every unnamed component the name of the thing it places.

    CadQuery leaves the *placement* of a subassembly unnamed (only the
    subassembly itself is named), so readers would show "14" instead of "Frame".
    Real CAD tools name both, so we do too.
    """
    components = TDF_LabelSequence()
    tool.GetComponents_s(label, components)
    for i in range(1, components.Length() + 1):
        component, target = components.Value(i), TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(component, target)
        if not name_of(component):
            TDataStd_Name.Set_s(component, TCollection_ExtendedString(name_of(target)))
        if XCAFDoc_ShapeTool.IsAssembly_s(target):
            name_subassembly_copies(tool, target)


def write_step(assy: cq.Assembly, path: Path) -> None:
    """Like assy.export(path), but with one root and names on subassembly placements."""
    root, doc = toCAF(assy, coloredSTEP=True)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    # toCAF wraps the top assembly in an extra placement label, which would make
    # the file read as "Drone -> Drone -> parts". Drop the wrapper, keep the assembly.
    top = TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(root, top)
    tool.RemoveShape(root, False)
    tool.UpdateAssemblies()
    name_subassembly_copies(tool, top)
    write_doc(doc, path)


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
            parts.append(Part(child.name, child.shape, parent * child.loc, child.color))
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
    return len({id(p.shape) for p in parts})


def main() -> None:
    tree = drone_tree()
    parts = flatten(tree, cq.Location())

    write_step(to_assembly(tree), HERE / "drone.step")
    check_instancing(HERE / "drone.step", count_unique(parts), len(parts))

    flat = Group("Drone", at(), list(parts))
    write_step(to_assembly(flat), HERE / "drone_flat.step")
    check_instancing(HERE / "drone_flat.step", count_unique(parts), len(parts))

    bracket = make_bracket()
    write_single_part(bracket, "Bracket", "#7f8c8d", HERE / "bracket.step")
    # Same part, but the file declares inches: importers must convert it to mm.
    write_single_part(bracket, "Bracket", "#7f8c8d", HERE / "bracket_inch.step", unit="INCH")
    print(f"wrote {len(parts)} parts ({count_unique(parts)} unique) and bracket.step")


if __name__ == "__main__":
    main()
