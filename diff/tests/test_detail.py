"""`kyumi diff --detail`: the material added and removed in one part."""

from __future__ import annotations

import importlib
import json
import math
import tempfile
import zipfile
from pathlib import Path

import kyumi_diff
import pytest
from kyumi import Model, import_step, load
from kyumi.cli import main
from kyumi_diff.detail import FAILED, OK, detail, parse_key, transformed, volume_of
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape

detail_module = importlib.import_module("kyumi_diff.detail")  # kyumi_diff.detail is the function
PAIRS = Path(__file__).resolve().parents[2] / "fixtures" / "pairs" / "drone"
PLATE = ["Drone", "Frame", "CenterPlate"]
PLATE_HOLE_VOLUME = math.pi * 2.0**2 * 3.0  # 4 mm hole through the 3 mm plate


@pytest.fixture(scope="module")
def packages(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    """The pairs that need exact geometry, imported with breps (the default)."""
    out_dir = tmp_path_factory.mktemp("breps")
    paths = {}
    for name in ("v1", "hole", "resize", "move", "rotate", "delete"):
        paths[name] = out_dir / f"{name}.kyumi"
        import_step(PAIRS / f"{name}.step", paths[name])
    paths["no_breps"] = out_dir / "v1-no-breps.kyumi"
    import_step(PAIRS / "v1.step", paths["no_breps"], breps=False)
    return paths


def compared(packages: dict[str, Path], new: str, key: list[str]) -> kyumi_diff.Detail:
    old_model, new_model = load(packages["v1"]), load(packages[new])
    return detail(old_model, new_model, kyumi_diff.diff(old_model, new_model), key)


def expected_volume(pair: str) -> float:
    expected = json.loads((PAIRS / f"{pair}.expected.json").read_text(encoding="utf-8"))
    return expected["changes"][0]["delta"]["volume"]


def test_hole_is_removed_material_only(packages: dict[str, Path]) -> None:
    found = compared(packages, "hole", PLATE)
    assert found.status == OK and not found.aligned
    assert found.added == 0
    assert found.removed == pytest.approx(PLATE_HOLE_VOLUME, rel=0.01)
    assert found.removed == pytest.approx(-expected_volume("hole"), rel=0.01)
    assert found.added_glb is None and found.removed_glb is not None


def test_resize_is_added_material_only(packages: dict[str, Path]) -> None:
    found = compared(packages, "resize", ["Drone", "Battery"])
    assert found.status == OK
    assert found.added == pytest.approx(expected_volume("resize"), rel=0.01)
    assert found.removed == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize("pair", ["move", "rotate"])
def test_a_part_that_only_moved_has_nothing_on_either_side(
    packages: dict[str, Path], pair: str
) -> None:
    found = compared(packages, pair, PLATE)
    assert found.status == OK and not found.aligned
    assert found.added == 0 and found.removed == 0
    assert found.added_glb is None and found.removed_glb is None


def test_unmatched_or_group_keys_are_clear_errors(packages: dict[str, Path]) -> None:
    with pytest.raises(ValueError, match="was removed"):
        compared(packages, "delete", ["Drone", "Frame", "Screw_3"])
    with pytest.raises(ValueError, match="is a group"):
        compared(packages, "move", ["Drone", "Frame"])
    with pytest.raises(ValueError, match="no node"):
        compared(packages, "move", ["Drone", "Nothing"])


def test_missing_breps_say_how_to_fix_it(packages: dict[str, Path]) -> None:
    old_model, new_model = load(packages["no_breps"]), load(packages["hole"])
    with pytest.raises(ValueError, match="re-import it without --no-breps"):
        detail(old_model, new_model, kyumi_diff.diff(old_model, new_model), PLATE)


def test_parse_key_accepts_both_separators() -> None:
    assert parse_key("Drone/ArmUnit#2/Screw#1") == ["Drone", "ArmUnit#2", "Screw#1"]
    assert parse_key("Drone › ArmUnit#2 › Screw#1") == ["Drone", "ArmUnit#2", "Screw#1"]
    with pytest.raises(ValueError, match="empty name"):
        parse_key("Drone//Frame")


def test_cli_writes_detail_files(packages: dict[str, Path], tmp_path: Path, capsys) -> None:
    out_dir = tmp_path / "detail"
    main(
        [
            "diff",
            str(packages["v1"]),
            str(packages["hole"]),
            "--detail",
            "/".join(PLATE),
            "--out-dir",
            str(out_dir),
        ]
    )
    out = capsys.readouterr().out
    assert "detail  Drone › Frame › CenterPlate: added 0.0 mm³, removed 37.7 mm³" in out
    written = json.loads((out_dir / "detail.json").read_text(encoding="utf-8"))
    assert written["status"] == OK and written["files"] == {"added": None, "removed": "removed.glb"}
    assert written["key"] == PLATE and written["new_id"]
    assert (out_dir / "removed.glb").stat().st_size > 0 and not (out_dir / "added.glb").exists()


def test_a_failed_boolean_is_reported_not_raised(
    packages: dict[str, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    monkeypatch.setattr(detail_module, "cut", lambda a, b: None)
    main(
        [
            "diff",
            str(packages["v1"]),
            str(packages["hole"]),
            "--detail",
            "/".join(PLATE),
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert "could not compute" in capsys.readouterr().out
    written = json.loads((tmp_path / "detail.json").read_text(encoding="utf-8"))
    assert written["status"] == FAILED and written["files"] == {"added": None, "removed": None}
    assert not list(tmp_path.glob("*.glb"))


def re_exported(package: Path, out: Path, shape_id: str) -> None:
    """A copy of `package` whose brep `shape_id` is written about a different origin
    (moved 10 mm and turned 90° about z), with the manifest's centre moved to match —
    what a part re-exported from a CAD tool with a new local origin looks like."""
    matrix = [
        [0.0, -1.0, 0.0, 10.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    import numpy as np

    with zipfile.ZipFile(package) as src, zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as dst:
        manifest = json.loads(src.read("manifest.json"))
        shape = manifest["shapes"][shape_id]
        centre = np.array(matrix) @ np.array([*shape["center"], 1.0])
        shape["center"] = centre[:3].tolist()
        for item in src.infolist():
            if item.filename == "manifest.json":
                dst.writestr(item, json.dumps(manifest))
            elif item.filename == shape["brep"]:
                dst.writestr(item, moved_brep(src.read(item.filename), np.array(matrix)))
            else:
                dst.writestr(item, src.read(item.filename))


def moved_brep(text: bytes, matrix) -> bytes:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "in.brep"
        path.write_bytes(text)
        shape = TopoDS_Shape()
        assert BRepTools.Read_s(shape, str(path), BRep_Builder())
        out = Path(tmp) / "out.brep"
        assert BRepTools.Write_s(transformed(shape, matrix), str(out))
        return out.read_bytes()


def test_a_brep_re_exported_about_a_new_origin_is_aligned_first(
    packages: dict[str, Path], tmp_path: Path
) -> None:
    old_model = load(packages["v1"])
    plate_shape = old_model.shapes[
        next(n.shape for n in old_model.nodes if n.name == "CenterPlate")
    ]
    re_exported(packages["v1"], tmp_path / "moved-origin.kyumi", plate_shape.id)
    new_model: Model = load(tmp_path / "moved-origin.kyumi")
    found = detail(old_model, new_model, kyumi_diff.diff(old_model, new_model), PLATE)
    assert found.status == OK and found.aligned
    assert found.added == pytest.approx(0, abs=1e-3) and found.removed == pytest.approx(0, abs=1e-3)


def test_volume_of_reads_the_brep_back(packages: dict[str, Path]) -> None:
    model = load(packages["v1"])
    shape = model.shapes[next(n.shape for n in model.nodes if n.name == "CenterPlate")]
    assert volume_of(detail_module.read_brep(model, shape)) == pytest.approx(shape.volume, rel=1e-6)
