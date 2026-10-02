"""Read a STEP file with OpenCascade's XCAF reader, keeping names, colours,
the assembly tree and instances (one prototype shared by many placements).

Output is a `RawModel`: the tree as `Node`s plus, per unique shape, the raw
OpenCascade geometry. Measuring and meshing happen later, once per shape.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StepRepr import StepRepr_NextAssemblyUsageOccurrence
from OCP.TCollection import TCollection_ExtendedString
from OCP.TColStd import TColStd_SequenceOfAsciiString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopoDS import TopoDS_Shape
from OCP.XCAFDoc import (
    XCAFDoc_ColorTool,
    XCAFDoc_ColorType,
    XCAFDoc_DocumentTool,
    XCAFDoc_ShapeTool,
)

from kyumi.model import IDENTITY, Node


class UnreadableFile(Exception):
    """The file is missing, not STEP, or too broken for OpenCascade to parse."""


@dataclass
class RawShape:
    id: str
    name: str
    color: str | None
    geometry: TopoDS_Shape


@dataclass
class RawModel:
    units: str
    shapes: dict[str, RawShape] = field(default_factory=dict)
    nodes: list[Node] = field(default_factory=list)


# STEP spells units in many ways; the manifest uses short lower-case names.
UNIT_NAMES = {"millimetre": "mm", "millimeter": "mm", "metre": "m", "meter": "m", "inch": "inch"}


def read_step(path: Path) -> RawModel:
    if not path.is_file():
        raise UnreadableFile(f"{path}: file not found")
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise UnreadableFile(f"{path}: not a STEP file OpenCascade can read")

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    if not reader.Transfer(doc):
        raise UnreadableFile(f"{path}: STEP file contains no usable geometry")

    walker = _Walker(doc, _blank_placement_ids(reader))
    roots = TDF_LabelSequence()
    walker.shapes.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walker.visit(roots.Value(i), parent=None)
    if not walker.model.nodes:
        raise UnreadableFile(f"{path}: STEP file contains no shapes")
    walker.model.units = _file_units(reader)
    return walker.model


def _blank_placement_ids(reader: STEPCAFControl_Reader) -> set[str]:
    """Ids of the placements whose name is blank in the file.

    OpenCascade names a blank placement after its id instead (node "95"). That id
    is renumbered on every export, so diff would see a rename; we want the "" back.
    An id that some other placement really is named after is left out, so we
    never erase a real name (that rare file keeps OpenCascade's substitute).
    """
    model = reader.Reader().StepModel()
    blank_ids: set[str] = set()
    real_names: set[str] = set()
    for i in range(1, model.NbEntities() + 1):
        entity = model.Value(i)
        if isinstance(entity, StepRepr_NextAssemblyUsageOccurrence):
            name = entity.Name().ToCString()
            if name == "":
                blank_ids.add(entity.Id().ToCString())
            else:
                real_names.add(name)
    return blank_ids - real_names


def _file_units(reader: STEPCAFControl_Reader) -> str:
    lengths = TColStd_SequenceOfAsciiString()
    reader.ChangeReader().FileUnits(
        lengths, TColStd_SequenceOfAsciiString(), TColStd_SequenceOfAsciiString()
    )
    if lengths.Length() == 0:
        return "mm"
    raw = lengths.Value(1).ToCString().lower()
    return UNIT_NAMES.get(raw, raw)


class _Walker:
    """Walks the XCAF label tree once, numbering nodes and shapes as it goes."""

    def __init__(self, doc: TDocStd_Document, blank_placement_ids: set[str]) -> None:
        self.shapes: XCAFDoc_ShapeTool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        self.colors: XCAFDoc_ColorTool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
        self.model = RawModel(units="mm")
        self.shape_ids: dict[int, str] = {}  # XCAF label tag of the prototype -> "s1", ...
        self.blank_placement_ids = blank_placement_ids

    def visit(self, label: TDF_Label, parent: str | None) -> None:
        # A component label is a *reference* ("put prototype X here"); follow it
        # to the prototype, which holds the geometry or the list of children.
        proto = TDF_Label()
        if not self.shapes.GetReferredShape_s(label, proto):
            proto = label

        name = name_of(label)
        if self.shapes.IsReference_s(label) and name in self.blank_placement_ids:
            name = ""
        node = Node(
            id=f"n{len(self.model.nodes)}",
            name=name,
            parent=parent,
            shape=None if self.shapes.IsAssembly_s(proto) else self._shape_id(proto),
            transform=_matrix(label),
        )
        own_color = color_of(self.colors, label)
        if node.shape and own_color != self.model.shapes[node.shape].color:
            node.color = own_color
        self.model.nodes.append(node)

        if node.shape is None:
            children = TDF_LabelSequence()
            self.shapes.GetComponents_s(proto, children)
            for i in range(1, children.Length() + 1):
                self.visit(children.Value(i), parent=node.id)

    def _shape_id(self, proto: TDF_Label) -> str:
        """Same prototype label -> same shape id, so four motors become one shape."""
        if proto.Tag() not in self.shape_ids:
            shape_id = f"s{len(self.shape_ids) + 1}"
            self.shape_ids[proto.Tag()] = shape_id
            self.model.shapes[shape_id] = RawShape(
                id=shape_id,
                name=name_of(proto),
                color=color_of(self.colors, proto),
                geometry=self.shapes.GetShape_s(proto),
            )
        return self.shape_ids[proto.Tag()]


def _matrix(label: TDF_Label) -> list[float]:
    """The label's placement relative to its parent as a row-major 4x4."""
    if not XCAFDoc_ShapeTool.IsReference_s(label):
        return list(IDENTITY)
    t = XCAFDoc_ShapeTool.GetLocation_s(label).Transformation()
    rows = [[t.Value(r, c) for c in (1, 2, 3, 4)] for r in (1, 2, 3)]
    return [v for row in rows for v in row] + [0.0, 0.0, 0.0, 1.0]


def name_of(label: TDF_Label) -> str:
    # Check first: FindAttribute segfaults in OCP when the attribute is missing.
    if not label.IsAttribute(TDataStd_Name.GetID_s()):
        return ""
    attr = TDataStd_Name()
    label.FindAttribute(TDataStd_Name.GetID_s(), attr)
    return attr.Get().ToExtString()


def color_of(colors: XCAFDoc_ColorTool, label: TDF_Label) -> str | None:
    """'#rrggbb' from the label's surface or generic colour, or None."""
    color = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if colors.GetColor_s(label, kind, color):
            r, g, b = color.Values(Quantity_TypeOfColor.Quantity_TOC_sRGB)
            return f"#{round(r * 255):02x}{round(g * 255):02x}{round(b * 255):02x}"
    return None
