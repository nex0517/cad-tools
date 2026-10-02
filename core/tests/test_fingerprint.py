"""The fingerprint must ignore position and orientation but notice geometry."""

import math

import cadquery as cq
import pytest
from kyumi.measure import measure
from kyumi.reader import RawShape
from OCP.TopoDS import TopoDS_Shape


def fingerprint_of(shape: cq.Workplane) -> str:
    geometry: TopoDS_Shape = shape.val().wrapped
    return measure(RawShape("s1", "test", None, geometry)).fingerprint


def bracket() -> cq.Workplane:
    return cq.Workplane().box(40, 20, 30).faces(">Z").workplane().hole(6)


def test_same_after_move_and_rotate() -> None:
    original = bracket()
    moved = original.translate((123.4, -56.7, 89.0))
    rotated = original.rotate((0, 0, 0), (1, 1, 0), 37).rotate((5, 5, 5), (0, 0, 1), 120)
    assert fingerprint_of(moved) == fingerprint_of(original)
    assert fingerprint_of(rotated) == fingerprint_of(original)


def test_changes_when_resized() -> None:
    assert fingerprint_of(bracket()) != fingerprint_of(bracket().faces(">Z").workplane().hole(7))
    assert fingerprint_of(cq.Workplane().box(40, 20, 30)) != fingerprint_of(
        cq.Workplane().box(40, 20, 30.5)
    )


def test_measurements_are_sane() -> None:
    shape = measure(RawShape("s1", "box", None, cq.Workplane().box(10, 20, 30).val().wrapped))
    assert shape.volume == 6000
    assert shape.area == 2 * (200 + 300 + 600)
    assert shape.size == [10, 20, 30]
    assert shape.center == pytest.approx([0, 0, 0], abs=1e-9)
    assert shape.faces == {
        "plane": 6,
        "cylinder": 0,
        "cone": 0,
        "sphere": 0,
        "torus": 0,
        "bspline": 0,
        "other": 0,
    }
    assert shape.solids == 1
    assert len(shape.fingerprint) == 16 and all(c in "0123456789abcdef" for c in shape.fingerprint)
    assert math.isclose(shape.inertia[0], 6000 * (100 + 400) / 12)
