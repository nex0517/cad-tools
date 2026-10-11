"""The v1/v2 pairs and their expected answers agree with the diff plan's tag definitions."""

import json
from pathlib import Path

import cadquery as cq
import pytest
from kyumi import Model, import_step, load
from make_diff_pairs import DRONE_CHANGES, make_pairs, pick_targets, read_doc, verify
from make_fixtures import drone_tree, flatten, overlaps

from tests.conftest import FIXTURES, PAIRS

DRONE_PAIRS = PAIRS / "drone"


def import_pair_file(pair_dir: Path, name: str, out_dir: Path) -> Model:
    out = out_dir / f"{pair_dir.name}-{name}.kyumi"
    import_step(pair_dir / name, out)
    return load(out)


@pytest.fixture(scope="module")
def drone_v1(tmp_path_factory: pytest.TempPathFactory) -> Model:
    return import_pair_file(DRONE_PAIRS, "v1.step", tmp_path_factory.mktemp("v1"))


def test_all_ten_drone_pairs_are_committed() -> None:
    names = {p.name.removesuffix(".expected.json") for p in DRONE_PAIRS.glob("*.expected.json")}
    assert names == set(DRONE_CHANGES) and len(names) == 10
    assert all((DRONE_PAIRS / f"{name}.step").is_file() for name in names)


@pytest.mark.parametrize("change", list(DRONE_CHANGES))
def test_expected_answer_matches_definitions(change: str, drone_v1: Model, tmp_path: Path) -> None:
    expected = json.loads((DRONE_PAIRS / f"{change}.expected.json").read_text(encoding="utf-8"))
    v2 = import_pair_file(DRONE_PAIRS, expected["new"], tmp_path)
    verify(drone_v1, v2, expected)


def test_swap_is_two_renames_and_renamed_is_only_renames() -> None:
    swap = json.loads((DRONE_PAIRS / "swap.expected.json").read_text(encoding="utf-8"))
    assert swap["summary"] == {**dict.fromkeys(swap["summary"], 0), "renamed": 2}
    renamed = json.loads((DRONE_PAIRS / "renamed.expected.json").read_text(encoding="utf-8"))
    assert renamed["summary"] == {**dict.fromkeys(renamed["summary"], 0), "renamed": 36}
    rotate = json.loads((DRONE_PAIRS / "rotate.expected.json").read_text(encoding="utf-8"))
    assert rotate["summary"]["moved"] == 1  # the Frame group, not its thirteen parts


def test_generic_mode_works_on_another_file(tmp_path: Path) -> None:
    changes = make_pairs(FIXTURES / "drone_flat.step", tmp_path / "flat")
    assert changes == ["nochange", "rename", "move", "delete"]
    for change in changes:
        expected = json.loads((tmp_path / "flat" / f"{change}.expected.json").read_text("utf-8"))
        v1 = import_pair_file(tmp_path / "flat", "v1.step", tmp_path)
        v2 = import_pair_file(tmp_path / "flat", expected["new"], tmp_path)
        verify(v1, v2, expected)


def test_targets_are_picked_deterministically() -> None:
    doc = read_doc(FIXTURES / "drone_flat.step")
    first, second = pick_targets(doc), pick_targets(read_doc(FIXTURES / "drone_flat.step"))
    assert first == second == [["Drone", "Arm_BL"], ["Drone", "Arm_BR"], ["Drone", "Arm_FL"]]


def test_single_part_file_gets_only_the_nochange_pair(tmp_path: Path) -> None:
    assert make_pairs(FIXTURES / "bracket.step", tmp_path / "bracket") == ["nochange"]


def test_drone_parts_do_not_overlap() -> None:
    assert overlaps(flatten(drone_tree(), cq.Location())) == []
