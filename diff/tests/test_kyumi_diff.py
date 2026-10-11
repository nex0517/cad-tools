"""`kyumi diff` reproduces every expected answer in fixtures/pairs/drone/."""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path

import pytest
from kyumi import Model, Node, import_step, load
from kyumi.cli import main
from kyumi_diff import diff, render_text, to_json
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


def normalised(data: dict) -> dict:
    """The parts of an answer that must agree; order of listing does not matter."""
    return {
        "changes": sorted(data["changes"], key=lambda c: (c["old"], c["new"])),
        "added": sorted(data["added"]),
        "removed": sorted(data["removed"]),
        "summary": data["summary"],
    }


@pytest.fixture(scope="module")
def cache(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("kyumi")


@pytest.mark.parametrize("expected_file", EXPECTED, ids=[p.stem.split(".")[0] for p in EXPECTED])
def test_every_drone_pair_is_reproduced(expected_file: Path, cache: Path) -> None:
    expected = json.loads(expected_file.read_text(encoding="utf-8"))
    result = diff(
        imported(PAIRS / expected["old"], cache), imported(PAIRS / expected["new"], cache)
    )
    assert normalised(json.loads(to_json(result))) == normalised(expected)


def test_renamed_pair_matches_every_part_without_names(cache: Path) -> None:
    result = diff(imported(PAIRS / "v1.step", cache), imported(PAIRS / "renamed.step", cache))
    print(
        f"\nrenamed pair: {result.matched_leaves}/{result.old_leaves} parts matched "
        f"({100 * result.match_rate:.0f} %) with every node and shape name changed"
    )
    assert result.match_rate == 1.0
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
        assert normalised(json.loads(to_json(result))) == normalised(expected), change


def tiled(model: Model, copies: int) -> Model:
    """The drone repeated `copies` times under one root, 10 m apart: a 5,000-part model."""
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


def test_five_thousand_parts_diff_in_under_five_seconds(cache: Path) -> None:
    v1 = tiled(imported(PAIRS / "v1.step", cache), 162)
    v2 = tiled(imported(PAIRS / "move.step", cache), 162)
    assert len(Index(v1).leaves) >= 5_000
    start = time.perf_counter()
    result = diff(v1, v2)
    elapsed = time.perf_counter() - start
    print(f"\n{len(Index(v1).leaves)} parts diffed in {elapsed:.2f} s")
    assert elapsed < 5.0
    assert result.summary["moved"] == 162 and result.match_rate == 1.0


def test_different_generators_print_a_warning(cache: Path, capsys: pytest.CaptureFixture) -> None:
    v1 = imported(PAIRS / "v1.step", cache)
    other = replace(v1, generator={"name": "kyumi-core", "version": "9.9.9"})
    diff(v1, other)
    assert "different generators" in capsys.readouterr().err
    diff(v1, v1)
    assert capsys.readouterr().err == ""


def test_cli_prints_summary_and_writes_json(cache: Path, tmp_path: Path, capsys) -> None:
    imported(PAIRS / "v1.step", cache), imported(PAIRS / "move.step", cache)
    out = tmp_path / "diff.json"
    main(["diff", str(cache / "drone-v1.kyumi"), str(cache / "drone-move.kyumi"), "-o", str(out)])
    text = capsys.readouterr().out
    assert "matched 31/31 parts (100 %)" in text and "Motor_FR  by 5.0 mm" in text
    assert json.loads(out.read_text(encoding="utf-8"))["summary"]["moved"] == 1
    assert (
        render_text(diff(load(cache / "drone-v1.kyumi"), load(cache / "drone-v1.kyumi"))).count(
            "\n"
        )
        == 1
    )
