"""Viewer diff mode: what `kyumi view OLD NEW [--detail DIR]` sends the page
(docs/diff-plan.md, section 4). The server works the status of every node out
here, in Python; the page only paints it. Uses only `load()` and the Model API.

Old and new ids collide (both packages have an `n7` and a `meshes/s1.glb`), so
the two id spaces never share a lookup: `status` and `moves` are keyed by NEW
ids, `ghosts` carry OLD ids and are served under `/old/meshes/`.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from kyumi import Model

from kyumi_diff.classify import Change, Diff
from kyumi_diff.index import Index, Key

CLASSES = ("modified", "moved", "re-parented", "renamed")  # strongest first
LOADED = "loaded"
NOT_COMPUTED = "not computed: run kyumi diff {old} {new} --detail-all --out-dir {out_dir}"
VOLUME_TOLERANCE = 0.001  # mm³: details.json rounds volumes to 3 decimals

# A verified details.json entry plus its mesh bytes, keyed by side ("added"/"removed").
DetailFiles = dict[str, dict[str, bytes | None]]


def changes_json(
    old: Model,
    new: Model,
    result: Diff,
    details: dict[str, dict] | None,
    detail_dir: Path | None,
) -> dict:
    """`/changes.json`: summary, per-new-id status, ghosts of removed parts, moves."""
    old_index, new_index = Index(old), Index(new)
    by_new_id = {c.new_id: c for c in result.changes}
    matched_new = set(result.matches.values())
    hint = NOT_COMPUTED.format(
        old=package_name(old), new=package_name(new), out_dir=detail_dir or "DIR"
    )
    status: dict[str, dict] = {}
    for node in new_index.ordered():  # parents before children, so inheritance can look up
        own = by_new_id.get(node.id)
        own_class = strongest(own.tags) if own else None
        if node.shape is not None and node.id not in matched_new:
            own_class = "added"
        inherited_from = None
        if own_class not in ("modified", "added") and not has_move(own):
            inherited_from = nearest_moved_ancestor(new_index, node.id, by_new_id)
        if inherited_from is not None:
            cls, inherited = strongest(by_new_id[inherited_from].tags), True
        else:
            cls, inherited = own_class, False
        entry = {
            "class": cls,
            "tags": list(own.tags) if own else [],
            "delta": dict(own.delta) if own else {},
            "inherited": inherited,
            "old_key": own.old if own else None,
            "new_key": new_index.key[node.id],
        }
        if own and "modified" in own.tags and node.shape is not None:
            entry["detail"] = detail_status(details, node.id, hint)
        status[node.id] = entry
    for node in reversed(new_index.ordered()):  # children before parents
        if node.shape is None:
            kids = [status[k.id]["class"] for k in new_index.children.get(node.id, [])]
            status[node.id]["children_class"] = strongest_class([*kids])

    ghosts = []
    for leaf in old_index.leaves:
        if leaf.id in result.matches:
            continue
        shape = old.shapes[leaf.shape]
        ghosts.append(
            {
                "old_id": leaf.id,
                "key": old_index.key[leaf.id],
                "name": leaf.name,
                "mesh": f"/old/{shape.mesh}" if shape.mesh else None,
                "world": old_index.world[leaf.id].T.flatten().tolist(),
                "volume": shape.volume,
                "size": shape.size,
            }
        )

    moves = []
    for change in result.changes:
        if not has_move(change):
            continue
        start = centre_of(old_index, change.old_id)
        end = centre_of(new_index, change.new_id)
        moves.append(
            {
                "new_id": change.new_id,
                "old_id": change.old_id,
                "from": start,
                "to": end,
                "distance": float(np.linalg.norm(np.array(end) - np.array(start))),
            }
        )

    return {
        "old": package_name(old),
        "new": package_name(new),
        "summary": result.summary,
        "status": status,
        "ghosts": ghosts,
        "moves": moves,
    }


def has_move(change: Change | None) -> bool:
    return change is not None and bool({"moved", "re-parented"} & set(change.tags))


def strongest(tags: list[str]) -> str | None:
    return strongest_class([t for t in tags if t in CLASSES])


def strongest_class(classes: list[str | None]) -> str | None:
    order = ("added", *CLASSES)
    present = [c for c in classes if c is not None]
    return min(present, key=order.index) if present else None


def nearest_moved_ancestor(index: Index, node_id: str, by_new_id: dict[str, Change]) -> str | None:
    for ancestor in index.ancestors(node_id):
        if has_move(by_new_id.get(ancestor)):
            return ancestor
    return None


def centre_of(index: Index, node_id: str) -> list[float]:
    """World centre of a part, or the mean of its parts' centres for a group."""
    node = index.by_id[node_id]
    if node.shape is not None:
        return [float(v) for v in index.center[node_id]]
    leaves = [n.id for n in descendants(index, node_id) if n.shape is not None]
    if not leaves:
        return [float(v) for v in index.world[node_id][:3, 3]]
    return [float(v) for v in np.mean([index.center[i] for i in leaves], axis=0)]


def descendants(index: Index, node_id: str) -> list:
    out, stack = [], list(index.children.get(node_id, []))
    while stack:
        node = stack.pop()
        out.append(node)
        stack.extend(index.children.get(node.id, []))
    return out


def detail_status(details: dict[str, dict] | None, new_id: str, hint: str) -> dict:
    """What the panel says about a modified part's exact detail."""
    entry = (details or {}).get(new_id)
    if entry is None:
        return {"state": hint}
    state = LOADED if entry["status"] == "ok" else entry["status"]
    return {
        "state": state,
        "added_mm3": entry["added_mm3"],
        "removed_mm3": entry["removed_mm3"],
        "volume_change_mm3": entry["volume_change_mm3"],
        "aligned": entry["aligned"],
        "files": {side: entry["files"][side] is not None for side in ("added", "removed")},
    }


def ghost_meshes(old: Model, result: Diff) -> set[str]:
    """Mesh paths (`meshes/sN.glb`) of the removed parts: the only old files served."""
    return {
        old.shapes[leaf.shape].mesh
        for leaf in old.leaves()
        if leaf.id not in result.matches and old.shapes[leaf.shape].mesh
    }


def load_details(
    detail_dir: Path, old: Model, new: Model, result: Diff
) -> tuple[dict[str, dict], DetailFiles]:
    """Read and check a `--detail-all` folder against this very diff. Every entry
    must name a change tagged `modified` (same key, same ids), its volume change
    must be the two shape volumes' difference, and its files must be exactly
    `<new_id>.<side>.glb` (or null) and exist. Anything else: refuse to start."""
    json_path = detail_dir / "details.json"
    if not json_path.is_file():
        raise ValueError(f"{detail_dir} has no details.json; run kyumi diff ... --detail-all")
    data = json.loads(json_path.read_text(encoding="utf-8"))
    modified = {c.new_id: c for c in result.changes if "modified" in c.tags}
    entries: dict[str, dict] = {}
    files: DetailFiles = {}
    for entry in data.get("parts", []):
        key = " › ".join(str(k) for k in entry.get("key", [])) or "?"
        change = modified.get(str(entry.get("new_id")))
        if change is None or list(change.new) != entry["key"] or change.old_id != entry["old_id"]:
            raise mismatch(detail_dir, key, old, new)
        expected = new.shapes[new.node(change.new_id).shape].volume
        expected -= old.shapes[old.node(change.old_id).shape].volume
        if abs(entry["volume_change_mm3"] - expected) > VOLUME_TOLERANCE + 1e-9:
            raise mismatch(detail_dir, key, old, new)
        sides: dict[str, bytes | None] = {}
        for side in ("added", "removed"):
            name = entry["files"][side]
            if name is None:
                sides[side] = None
                continue
            wanted = f"{change.new_id}.{side}.glb"
            path = detail_dir / wanted  # built here, never from the json
            if name != wanted or not path.is_file():
                raise mismatch(detail_dir, key, old, new)
            sides[side] = path.read_bytes()
        entries[change.new_id] = entry
        files[change.new_id] = sides
    return entries, files


def mismatch(detail_dir: Path, key: str, old: Model, new: Model) -> ValueError:
    return ValueError(
        f"{detail_dir} was made from other packages ({key} doesn't match); "
        f"re-run kyumi diff {package_name(old)} {package_name(new)} --detail-all "
        f"--out-dir {detail_dir}"
    )


def package_name(model: Model) -> str:
    return model.path.name if model.path is not None else model.source_file


def key_text(key: Key) -> str:
    return " › ".join(key)
