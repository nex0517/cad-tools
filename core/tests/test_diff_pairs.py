"""The v1/v2 pairs and their expected answers agree with the diff plan's tag definitions."""

import json
from pathlib import Path

import cadquery as cq
import pytest
from kyumi import Model, import_step, load
from make_diff_pairs import (
    DRONE_PAIRS,
    arm_units_tree,
    make_pairs,
    pick_targets,
    placements,
    read_doc,
    verify,
)
from make_fixtures import (
    at,
    drone_tree,
    flatten,
    name_of,
    overlaps,
    restore_blank_names,
    shared_tree,
    write_doc,
)
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

from tests.conftest import FIXTURES, PAIRS

PAIR_DIR = PAIRS / "drone"


def import_pair_file(pair_dir: Path, name: str, out_dir: Path) -> Model:
    out = out_dir / f"{pair_dir.name}-{name}.kyumi"
    import_step(pair_dir / name, out)
    return load(out)


@pytest.fixture(scope="module")
def drone_v1(tmp_path_factory: pytest.TempPathFactory) -> Model:
    return import_pair_file(PAIR_DIR, "v1.step", tmp_path_factory.mktemp("v1"))


def test_the_ten_planned_pairs_plus_arms_and_reparent_are_committed() -> None:
    names = {p.name.removesuffix(".expected.json") for p in PAIR_DIR.glob("*.expected.json")}
    assert names == set(DRONE_PAIRS) and len(names) == 12
    assert all((PAIR_DIR / f"{name}.step").is_file() for name in names)


@pytest.mark.parametrize("change", list(DRONE_PAIRS))
def test_expected_answer_matches_definitions(change: str, drone_v1: Model, tmp_path: Path) -> None:
    expected = json.loads((PAIR_DIR / f"{change}.expected.json").read_text(encoding="utf-8"))
    if expected["old"] == "v1.step":
        v1 = drone_v1
    else:
        v1 = import_pair_file(PAIR_DIR, expected["old"], tmp_path)
    v2 = import_pair_file(PAIR_DIR, expected["new"], tmp_path)
    verify(v1, v2, expected)


def test_swap_is_two_renames_and_renamed_is_only_renames() -> None:
    swap = json.loads((PAIR_DIR / "swap.expected.json").read_text(encoding="utf-8"))
    assert swap["summary"] == {**dict.fromkeys(swap["summary"], 0), "renamed": 2}
    renamed = json.loads((PAIR_DIR / "renamed.expected.json").read_text(encoding="utf-8"))
    assert renamed["summary"] == {**dict.fromkeys(renamed["summary"], 0), "renamed": 36}
    rotate = json.loads((PAIR_DIR / "rotate.expected.json").read_text(encoding="utf-8"))
    assert rotate["summary"]["moved"] == 1  # the Frame group, not its thirteen parts


@pytest.mark.parametrize(
    "source",
    [
        PAIR_DIR / "arms_v1.step",  # repeated names: four ArmUnit, two Screw per unit
        FIXTURES / "drone_messy.step",  # baked copies and one blank name
        FIXTURES / "drone_shared.step",  # one subassembly definition placed twice
    ],
)
def test_generic_mode_works_on_any_file(source: Path, tmp_path: Path) -> None:
    changes = make_pairs(source, tmp_path / "out")
    assert changes == ["nochange", "rename", "move", "delete", "modify"]
    v1 = import_pair_file(tmp_path / "out", "v1.step", tmp_path)
    for change in changes:
        expected = json.loads((tmp_path / "out" / f"{change}.expected.json").read_text("utf-8"))
        v2 = import_pair_file(tmp_path / "out", expected["new"], tmp_path)
        verify(v1, v2, expected)


def test_targets_are_picked_deterministically_and_never_shift_a_key() -> None:
    flat = [pick_targets(read_doc(FIXTURES / "drone_flat.step")) for _ in range(2)]
    assert flat[0] == flat[1] == [["Drone", "Arm_BL"], ["Drone", "Arm_BR"], ["Drone", "Arm_FL"]]
    # Only the last of same-named siblings qualifies: "Screw#1", never "Screw".
    arms = pick_targets(read_doc(PAIR_DIR / "arms_v1.step"))
    assert arms == [
        ["Drone", "ArmUnit", "Arm"],
        ["Drone", "ArmUnit", "Motor"],
        ["Drone", "ArmUnit", "Prop"],
    ]
    leaves = [p.key for p in placements(read_doc(PAIR_DIR / "arms_v1.step"))]
    assert ["Drone", "ArmUnit#3", "Screw#1"] in leaves and ["Drone", "ArmUnit#3", "Screw"] in leaves


def test_parts_inside_a_twice_placed_subassembly_are_never_picked(capsys) -> None:
    doc = read_doc(FIXTURES / "drone_shared.step")
    keys = [p.key for p in placements(doc)]
    assert ["Drone", "ArmUnit", "Motor"] in keys and ["Drone", "ArmUnit#1", "Motor"] in keys
    assert pick_targets(doc) == [["Drone", "CenterPlate"]] * 3
    assert "skipped 10 parts" in capsys.readouterr().out


def test_a_part_placed_twice_is_never_drilled(tmp_path: Path, capsys) -> None:
    """drone_flat with a second placement of the Battery's definition: drilling the
    Battery (the largest part) would drill both copies, so modify takes the next one."""
    doc = read_doc(FIXTURES / "drone_flat.step")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots, top = TDF_LabelSequence(), TDF_Label()
    tool.GetFreeShapes(roots)
    if not XCAFDoc_ShapeTool.GetReferredShape_s(roots.Value(1), top):
        top = roots.Value(1)
    components = TDF_LabelSequence()
    tool.GetComponents_s(top, components)
    battery = next(
        components.Value(i)
        for i in range(1, components.Length() + 1)
        if name_of(components.Value(i)) == "Battery"
    )
    definition = TDF_Label()
    XCAFDoc_ShapeTool.GetReferredShape_s(battery, definition)
    second = tool.AddComponent(top, definition, at(300, 0, 0).wrapped)
    TDataStd_Name.Set_s(second, TCollection_ExtendedString("Battery"))
    tool.UpdateAssemblies()
    write_doc(doc, tmp_path / "twice.step")
    restore_blank_names(tmp_path / "twice.step")

    changes = make_pairs(tmp_path / "twice.step", tmp_path / "out")
    assert "modify" in changes and "not drilling" in capsys.readouterr().out
    expected = json.loads((tmp_path / "out" / "modify.expected.json").read_text("utf-8"))
    assert expected["changes"][0]["new"][1] not in ("Battery", "Battery#1")
    v1 = import_pair_file(tmp_path / "out", "v1.step", tmp_path)
    v2 = import_pair_file(tmp_path / "out", "modify.step", tmp_path)
    verify(v1, v2, expected)


def test_single_part_file_gets_only_the_nochange_pair(tmp_path: Path) -> None:
    assert make_pairs(FIXTURES / "bracket.step", tmp_path / "bracket") == ["nochange"]


@pytest.mark.parametrize("tree", [drone_tree, arm_units_tree, shared_tree])
def test_fixture_parts_do_not_overlap(tree) -> None:
    assert overlaps(flatten(tree(), cq.Location())) == []
