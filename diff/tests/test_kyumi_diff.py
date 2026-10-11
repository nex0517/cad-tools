"""`kyumi diff` reproduces every expected answer in fixtures/pairs/drone/."""

from __future__ import annotations

import json
import time
import zipfile
from dataclasses import replace
from pathlib import Path

import pytest
from kyumi import Model, Node, import_step, load
from kyumi.cli import main
from kyumi.package import manifest
from kyumi_diff import Diff, agrees, diff, render_text
from kyumi_diff.index import Index
from make_diff_pairs import make_pairs

FIXTURES = Path(__file__).resolve().parents[2] / "fixtures"
PAIRS = FIXTURES / "pairs" / "drone"

EXPECTED = sorted(PAIRS.glob("*.expected.json"))


def imported(step: Path, out_dir: Path) -> Model:
    out = out_dir / f"{step.parent.name}-{step.stem}.kyumi"
    if not out.exists():
        import_step(step, out, breps=False)
    return load(out)


def correct_matches(result: Diff, old: Model, new: Model, expected: dict) -> tuple[int, int]:
    """How many v1 parts the matcher paired with the partner expected.json names
    (a removed part is right when it has no partner), out of all v1 parts."""
    partner = {tuple(c["old"]): tuple(c["new"]) for c in expected["changes"]}
    removed = {tuple(k) for k in expected["removed"]}
    old_keys, new_keys = Index(old).key, Index(new).key
    correct = 0
    for leaf in old.leaves():
        key = tuple(old_keys[leaf.id])
        got = result.matches.get(leaf.id)
        want = None if key in removed else partner.get(key, key)
        correct += (got is None and want is None) or (
            got is not None and tuple(new_keys[got]) == want
        )
    return correct, len(old.leaves())


@pytest.fixture(scope="module")
def cache(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("kyumi")


@pytest.mark.parametrize("expected_file", EXPECTED, ids=[p.stem.split(".")[0] for p in EXPECTED])
def test_every_drone_pair_is_reproduced(expected_file: Path, cache: Path) -> None:
    expected = json.loads(expected_file.read_text(encoding="utf-8"))
    old, new = imported(PAIRS / expected["old"], cache), imported(PAIRS / expected["new"], cache)
    result = diff(old, new)
    correct, total = correct_matches(result, old, new, expected)
    change = expected_file.name.removesuffix(".expected.json")
    print(f"\n{change}: {correct}/{total} correct ({100 * correct / total:.1f}%)")
    assert correct == total
    assert agrees(result, expected)


def test_renamed_pair_matches_every_part_without_names(cache: Path) -> None:
    result = diff(imported(PAIRS / "v1.step", cache), imported(PAIRS / "renamed.step", cache))
    assert result.summary == {
        "added": 0,
        "removed": 0,
        "moved": 0,
        "renamed": 36,
        "modified": 0,
        "re-parented": 0,
    }


def test_swap_is_two_renames(cache: Path) -> None:
    result = diff(imported(PAIRS / "v1.step", cache), imported(PAIRS / "swap.step", cache))
    assert [c.tags for c in result.changes] == [["renamed"], ["renamed"]]


@pytest.mark.parametrize("source", ["drone_messy", "drone_shared"])
def test_generic_pairs_are_reproduced(source: str, tmp_path: Path) -> None:
    out = tmp_path / source
    for change in make_pairs(FIXTURES / f"{source}.step", out):
        expected = json.loads((out / f"{change}.expected.json").read_text(encoding="utf-8"))
        result = diff(
            imported(out / "v1.step", tmp_path), imported(out / expected["new"], tmp_path)
        )
        assert agrees(result, expected), change


def tiled(model: Model, copies: int) -> Model:
    """The drone repeated `copies` times under one root, 10 m apart: a 5,000-part
    model in which every tile has the same names (the realistic worst case, like
    CPT17's eight identical LegUnits)."""
    nodes: list[Node] = [Node("root", "Fleet", None, None)]
    for i in range(copies):
        for node in model.nodes:
            copy = replace(
                node, id=f"{node.id}_{i}", parent=f"{node.parent}_{i}" if node.parent else "root"
            )
            if node.parent is None:
                copy.transform = list(copy.transform)
                copy.transform[3] = (i % 16) * 10_000.0
                copy.transform[7] = (i // 16) * 10_000.0
            nodes.append(copy)
    return Model(model.source_file, model.source_units, model.shapes, nodes)


def packaged(model: Model, out: Path) -> Path:
    """A .kyumi holding just the manifest, which is all `load()` needs."""
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("manifest.json", json.dumps(manifest(model)))
    return out


def test_five_thousand_parts_load_and_diff_in_under_five_seconds(
    cache: Path, tmp_path: Path
) -> None:
    v1 = packaged(tiled(imported(PAIRS / "v1.step", cache), 162), tmp_path / "fleet_v1.kyumi")
    v2 = packaged(tiled(imported(PAIRS / "move.step", cache), 162), tmp_path / "fleet_v2.kyumi")
    start = time.perf_counter()
    old, new = load(v1), load(v2)
    result = diff(old, new)
    elapsed = time.perf_counter() - start
    parts = len(old.leaves())
    print(f"\n{parts} parts loaded and diffed in {elapsed:.2f} s")
    assert parts >= 5_000
    assert elapsed < 5.0
    assert result.summary == {
        "added": 0,
        "removed": 0,
        "moved": 162,
        "renamed": 0,
        "modified": 0,
        "re-parented": 0,
    }
    assert all(leaf.id in result.matches for leaf in old.leaves())


def test_different_or_missing_generator_versions_print_a_warning(
    cache: Path, capsys: pytest.CaptureFixture
) -> None:
    v1 = imported(PAIRS / "v1.step", cache)
    assert v1.generator == {"name": "kyumi-core", "version": "0.1.0"}
    diff(v1, replace(v1, generator={"name": "kyumi-core", "version": "9.9.9"}))
    assert "different generator versions" in capsys.readouterr().err
    diff(replace(v1, generator=None), v1)
    assert "'modified' results could be false" in capsys.readouterr().err
    diff(v1, v1)
    assert capsys.readouterr().err == ""


def test_cli_prints_keys_and_writes_json_with_ids(cache: Path, tmp_path: Path, capsys) -> None:
    v1, arms = imported(PAIRS / "arms_v1.step", cache), imported(PAIRS / "arms.step", cache)
    out = tmp_path / "diff.json"
    main(
        [
            "diff",
            str(cache / "drone-arms_v1.kyumi"),
            str(cache / "drone-arms.kyumi"),
            "-o",
            str(out),
        ]
    )
    text = capsys.readouterr().out
    assert "moved  Drone › ArmUnit#2  by 5.0 mm" in text
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["summary"]["moved"] == 1 and data["keys"]
    (change,) = data["changes"]
    assert change["old"] == change["new"] == ["Drone", "ArmUnit#2"]
    assert v1.node(change["old_id"]).name == arms.node(change["new_id"]).name == "ArmUnit"
    assert 0 < change["score"] <= 1
    assert render_text(diff(v1, v1)).count("\n") == 0


def test_cli_refuses_to_overwrite_an_input_with_the_json(cache: Path) -> None:
    imported(PAIRS / "v1.step", cache)
    package = cache / "drone-v1.kyumi"
    with pytest.raises(SystemExit):
        main(["diff", str(package), str(package), "-o", str(package)])
    assert len(load(package).nodes) == 36
