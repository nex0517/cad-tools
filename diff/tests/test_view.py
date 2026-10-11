"""Viewer diff mode: `kyumi diff --detail-all` and what `kyumi view OLD NEW` serves."""

from __future__ import annotations

import importlib
import json
import threading
import time
from http.server import HTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
from kyumi import Model, import_step, load
from kyumi.cli import diff_view, main
from kyumi.viewer import make_handler
from kyumi_diff import changes_json, detail_all, diff, load_details
from kyumi_diff.index import Index
from make_diff_pairs import make_pairs
from test_kyumi_diff import imported, packaged, tiled

detail_module = importlib.import_module("kyumi_diff.detail")
FIXTURES = Path(__file__).resolve().parents[2] / "fixtures"
PAIRS = FIXTURES / "pairs" / "drone"
EXPECTED = sorted(PAIRS.glob("*.expected.json"))
PLATE = ["Drone", "Frame", "CenterPlate"]


@pytest.fixture(scope="module")
def packages(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    out_dir = tmp_path_factory.mktemp("view")
    paths = {}
    for name in ("v1", "hole", "move", "delete", "reparent"):
        paths[name] = out_dir / f"{name}.kyumi"
        import_step(PAIRS / f"{name}.step", paths[name], breps=name in ("v1", "hole"))
    paths["no_breps"] = out_dir / "v1-no-breps.kyumi"
    import_step(PAIRS / "v1.step", paths["no_breps"], breps=False)
    return paths


@pytest.fixture(scope="module")
def hole_detail(packages: dict[str, Path], tmp_path_factory: pytest.TempPathFactory) -> Path:
    out = tmp_path_factory.mktemp("detail") / "hole"
    old, new = load(packages["v1"]), load(packages["hole"])
    detail_all(old, new, diff(old, new), out)
    return out


def details_in(folder: Path) -> dict:
    return json.loads((folder / "details.json").read_text(encoding="utf-8"))


# ---- Part A: --detail-all ---------------------------------------------------


def test_detail_all_writes_one_entry_for_the_hole(hole_detail: Path) -> None:
    data = details_in(hole_detail)
    assert data["old"] == "v1.kyumi" and data["new"] == "hole.kyumi"
    (entry,) = data["parts"]
    assert entry["key"] == PLATE and entry["status"] == "ok"
    assert entry["removed_mm3"] == pytest.approx(37.7, abs=0.05)
    assert entry["files"] == {"added": None, "removed": f"{entry['new_id']}.removed.glb"}
    assert entry["seconds"] >= 0
    assert sorted(p.name for p in hole_detail.iterdir()) == sorted(
        ["details.json", entry["files"]["removed"]]
    )


def test_detail_all_on_a_pair_without_modified_parts_is_empty(
    packages: dict[str, Path], tmp_path: Path
) -> None:
    old, new = load(packages["v1"]), load(packages["move"])
    detail_all(old, new, diff(old, new), tmp_path / "move")
    assert details_in(tmp_path / "move")["parts"] == []


def test_detail_all_carries_on_after_a_failed_boolean(
    packages: dict[str, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(detail_module, "cut", lambda a, b: None)
    old, new = load(packages["v1"]), load(packages["hole"])
    detail_all(old, new, diff(old, new), tmp_path / "failed")
    (entry,) = details_in(tmp_path / "failed")["parts"]
    assert entry["status"] == "could not compute"
    assert entry["files"] == {"added": None, "removed": None}
    assert not list((tmp_path / "failed").glob("*.glb"))


def test_detail_all_without_breps_writes_nothing(packages: dict[str, Path], tmp_path: Path) -> None:
    out = tmp_path / "nb"
    args = [str(packages["no_breps"]), str(packages["hole"]), "--detail-all", "--out-dir", str(out)]
    with pytest.raises(SystemExit, match="re-import it without --no-breps"):
        main(["diff", *args])
    assert not out.exists()


def test_a_rerun_removes_stale_meshes(packages: dict[str, Path], tmp_path: Path) -> None:
    out = tmp_path / "rerun"
    out.mkdir()
    (out / "stale.glb").write_bytes(b"old run")
    (out / "n2.added.glb").write_bytes(b"old run")
    old, new = load(packages["v1"]), load(packages["hole"])
    detail_all(old, new, diff(old, new), out)
    (entry,) = details_in(out)["parts"]
    assert sorted(p.name for p in out.glob("*.glb")) == [entry["files"]["removed"]]


# ---- Part B: /changes.json ----------------------------------------------------


def expected_classes(new: Model, expected: dict) -> dict[str, tuple[str | None, bool]]:
    """What expected.json says each new node is, from its own tags or, for a part
    without a move of its own, its nearest moved / re-parented ancestor."""
    order = ("added", "modified", "moved", "re-parented", "renamed")
    by_key = {tuple(c["new"]): c["tags"] for c in expected["changes"]}
    added = {tuple(k) for k in expected["added"]}
    index = Index(new)
    classes = {}
    for node in new.nodes:
        key = tuple(index.key[node.id])
        own = sorted(by_key.get(key, []), key=order.index)
        if key in added:
            classes[node.id] = ("added", False)
            continue
        if own and own[0] in ("modified",) or {"moved", "re-parented"} & set(own):
            classes[node.id] = (own[0], False)
            continue
        inherited = next(
            (
                sorted(
                    {"moved", "re-parented"} & set(by_key[tuple(index.key[a])]), key=order.index
                )[0]
                for a in index.ancestors(node.id)
                if {"moved", "re-parented"} & set(by_key.get(tuple(index.key[a]), []))
            ),
            None,
        )
        if inherited:
            classes[node.id] = (inherited, True)
        else:
            classes[node.id] = (own[0] if own else None, False)
    return classes


@pytest.mark.parametrize("expected_file", EXPECTED, ids=[p.stem.split(".")[0] for p in EXPECTED])
def test_every_new_node_gets_the_class_expected_json_implies(
    expected_file: Path, tmp_path_factory: pytest.TempPathFactory
) -> None:
    cache = tmp_path_factory.getbasetemp() / "view-cache"
    cache.mkdir(exist_ok=True)
    expected = json.loads(expected_file.read_text(encoding="utf-8"))
    old = imported(PAIRS / expected["old"], cache)
    new = imported(PAIRS / expected["new"], cache)
    result = diff(old, new)
    payload = changes_json(old, new, result, None, None)
    got = {i: (s["class"], s["inherited"]) for i, s in payload["status"].items()}
    assert got == expected_classes(new, expected)
    assert payload["summary"] == expected["summary"]
    assert len(payload["ghosts"]) == expected["summary"]["removed"]


def test_rotate_and_arms_colour_the_whole_group(tmp_path: Path) -> None:
    old, new = imported(PAIRS / "v1.step", tmp_path), imported(PAIRS / "rotate.step", tmp_path)
    status = changes_json(old, new, diff(old, new), None, None)["status"]
    frame = next(i for i, s in status.items() if s["new_key"] == ["Drone", "Frame"])
    assert status[frame]["class"] == "moved" and not status[frame]["inherited"]
    under_frame = [s for s in status.values() if s["new_key"][:2] == ["Drone", "Frame"]][1:]
    assert under_frame and all(s["class"] == "moved" and s["inherited"] for s in under_frame)
    assert all(
        s["class"] is None for s in status.values() if s["new_key"][:2] != ["Drone", "Frame"]
    )

    old, new = imported(PAIRS / "arms_v1.step", tmp_path), imported(PAIRS / "arms.step", tmp_path)
    status = changes_json(old, new, diff(old, new), None, None)["status"]
    moved = {tuple(s["new_key"]) for s in status.values() if s["class"] == "moved"}
    assert all(k[:2] == ("Drone", "ArmUnit#2") for k in moved) and len(moved) > 1


def test_delete_pair_has_one_ghost_at_the_old_transform(packages: dict[str, Path]) -> None:
    old, new = load(packages["v1"]), load(packages["delete"])
    view = diff_view(old, new, None)
    (ghost,) = view.changes["ghosts"]
    assert ghost["key"] == ["Drone", "Frame", "Screw_3"]
    assert ghost["world"] == old.world_transform(ghost["old_id"]).T.flatten().tolist()
    assert ghost["mesh"] == f"/old/{old.shapes[old.node(ghost['old_id']).shape].mesh}"
    with served(new, view) as base:
        assert urlopen(base + ghost["mesh"]).read() == old.read(ghost["mesh"][len("/old/") :])


def test_ghost_bytes_come_from_the_old_package_not_the_new(tmp_path: Path) -> None:
    """drone_messy makes every copy its own shape, so deleting one renumbers the
    rest: the removed part's `meshes/sN.glb` names a different shape in the new
    package. The ghost must still be the old bytes."""
    out = tmp_path / "messy"
    assert "delete" in make_pairs(FIXTURES / "drone_messy.step", out)
    old, new = imported(out / "v1.step", tmp_path), imported(out / "delete.step", tmp_path)
    view = diff_view(old, new, None)
    (ghost,) = view.changes["ghosts"]
    mesh = ghost["mesh"][len("/old/") :]
    assert mesh in {s.mesh for s in new.shapes.values()}
    assert old.read(mesh) != new.read(mesh)
    with served(new, view) as base:
        assert urlopen(base + ghost["mesh"]).read() == old.read(mesh)


def test_move_and_reparent_centres(packages: dict[str, Path]) -> None:
    old = load(packages["v1"])
    (move,) = changes_json(
        old, load(packages["move"]), diff(old, load(packages["move"])), None, None
    )["moves"]
    assert move["distance"] == pytest.approx(5.0, abs=0.001)
    new = load(packages["reparent"])
    (reparent,) = changes_json(old, new, diff(old, new), None, None)["moves"]
    assert reparent["distance"] == pytest.approx(0.0, abs=0.001)


class served:
    """A viewer server for one test: `with served(model, view) as base: urlopen(base + ...)`."""

    def __init__(self, model: Model, view) -> None:
        self.server = HTTPServer(("127.0.0.1", 0), make_handler(model, view))

    def __enter__(self) -> str:
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        return f"http://127.0.0.1:{self.server.server_port}"

    def __exit__(self, *exc: object) -> None:
        self.server.shutdown()


def status_of(url: str, host: str | None = None) -> int:
    try:
        request = Request(url, headers={"Host": host} if host else {})
        return urlopen(request).status
    except HTTPError as error:
        return error.code


def test_routes_are_a_whitelist(packages: dict[str, Path], hole_detail: Path) -> None:
    old, new = load(packages["v1"]), load(packages["delete"])
    with served(new, diff_view(old, new, None)) as base:
        (ghost,) = diff_view(old, new, None).changes["ghosts"]
        assert status_of(base + ghost["mesh"]) == 200
        kept = next(s.mesh for s in old.shapes.values() if f"/old/{s.mesh}" != ghost["mesh"])
        assert status_of(f"{base}/old/{kept}") == 404  # that shape was not removed
        assert status_of(f"{base}/old/../manifest.json") == 404
        assert status_of(f"{base}/old/manifest.json") == 404
        for route in ("/changes.json", ghost["mesh"]):
            assert status_of(base + route, host="evil.example:80") == 403

    old, new = load(packages["v1"]), load(packages["hole"])
    view = diff_view(old, new, hole_detail)
    plate = next(i for i, s in view.changes["status"].items() if s["new_key"] == PLATE)
    unchanged = next(
        i for i, s in view.changes["status"].items() if s["class"] is None and byshape(new, i)
    )
    with served(new, view) as base:
        assert status_of(f"{base}/detail/{plate}/removed.glb") == 200
        assert status_of(f"{base}/detail/{plate}/added.glb") == 404  # null side
        assert status_of(f"{base}/detail/{unchanged}/removed.glb") == 404
        assert status_of(f"{base}/detail/old:{plate}/removed.glb") == 404
        assert status_of(f"{base}/detail/{plate}/removed.glb", host="evil.example:80") == 403


def byshape(model: Model, node_id: str) -> bool:
    return model.node(node_id).shape is not None


def test_detail_folder_is_checked_against_this_diff(
    packages: dict[str, Path], hole_detail: Path, tmp_path: Path
) -> None:
    old, new = load(packages["v1"]), load(packages["hole"])
    view = diff_view(old, new, hole_detail)
    (entry,) = details_in(hole_detail)["parts"]
    with served(new, view) as base:
        glb = urlopen(f"{base}/detail/{entry['new_id']}/removed.glb").read()
    assert glb == (hole_detail / entry["files"]["removed"]).read_bytes()
    assert view.changes["status"][entry["new_id"]]["detail"]["state"] == "loaded"

    with pytest.raises(ValueError, match="made from other packages"):
        diff_view(old, load(packages["move"]), hole_detail)

    edited = tmp_path / "edited"
    edited.mkdir()
    data = details_in(hole_detail)
    data["parts"][0]["files"]["removed"] = "../x.glb"
    (edited / "details.json").write_text(json.dumps(data), encoding="utf-8")
    (edited / "x.glb").write_bytes(b"not ours")
    with pytest.raises(ValueError, match="made from other packages"):
        load_details(edited, old, new, diff(old, new))


def test_a_modified_part_without_detail_shows_the_command(packages: dict[str, Path]) -> None:
    old, new = load(packages["v1"]), load(packages["hole"])
    view = diff_view(old, new, None)
    plate = next(s for s in view.changes["status"].values() if s["new_key"] == PLATE)
    assert plate["detail"]["state"].startswith("not computed: run kyumi diff v1.kyumi hole.kyumi")


def test_view_cli_needs_two_packages_for_detail(
    packages: dict[str, Path], hole_detail: Path
) -> None:
    with pytest.raises(SystemExit, match="--detail needs two packages"):
        main(["view", str(packages["v1"]), "--detail", str(hole_detail), "--no-browser"])


def test_changes_json_for_five_thousand_parts_builds_in_under_two_seconds(tmp_path: Path) -> None:
    v1 = packaged(tiled(imported(PAIRS / "v1.step", tmp_path), 162), tmp_path / "fleet_v1.kyumi")
    v2 = packaged(tiled(imported(PAIRS / "move.step", tmp_path), 162), tmp_path / "fleet_v2.kyumi")
    old, new = load(v1), load(v2)
    result = diff(old, new)
    start = time.perf_counter()
    payload = changes_json(old, new, result, None, None)
    elapsed = time.perf_counter() - start
    print(f"\n/changes.json for {len(new.leaves())} parts in {elapsed:.2f} s")
    assert elapsed < 2.0
    assert len(payload["moves"]) == 162
