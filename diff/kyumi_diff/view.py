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
    """`/changes.json`: summary, per-id status for both versions, ghosts of removed
    parts, moves. `status` is keyed by new ids, `old_status` by old ids; the two id
    spaces never mix, each entry names its partner in the other version."""
    old_index, new_index = Index(old), Index(new)
    hint = NOT_COMPUTED.format(
        old=package_name(old), new=package_name(new), out_dir=detail_dir or "DIR"
    )
    new_of = result.matches  # old id -> new id (ids, not keys: keys can collide)
    old_of = {n: o for o, n in new_of.items()}
    status = side_status(
        new_index, {c.new_id: c for c in result.changes}, old_of, "added", "old_id", details, hint
    )
    old_status = side_status(
        old_index, {c.old_id: c for c in result.changes}, new_of, "removed", "new_id"
    )

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
        "old_status": old_status,
        "ghosts": ghosts,
        "moves": moves,
    }


def side_status(
    index: Index,
    by_id: dict[str, Change],
    partner_of: dict[str, str],
    missing_class: str,
    partner_field: str,
    details: dict[str, dict] | None = None,
    hint: str = "",
) -> dict[str, dict]:
    """One version's nodes classified: its own change, `missing_class` when the node
    has no partner in the other version, or the move of its nearest moved ancestor."""
    status: dict[str, dict] = {}
    for node in index.ordered():  # parents before children, so inheritance can look up
        own = by_id.get(node.id)
        own_class = strongest(own.tags) if own else None
        if node.id not in partner_of:
            own_class = missing_class  # parts and groups alike
        inherited_from = None
        if own_class not in ("modified", missing_class) and not has_move(own):
            inherited_from = nearest_moved_ancestor(index, node.id, by_id)
        if inherited_from is not None:
            cls, inherited = strongest(by_id[inherited_from].tags), True
        else:
            cls, inherited = own_class, False
        this_side, other_side = ("new", "old") if missing_class == "added" else ("old", "new")
        entry = {
            "class": cls,
            "tags": list(own.tags) if own else [],
            "delta": dict(own.delta) if own else {},
            "inherited": inherited,
            f"{this_side}_key": index.key[node.id],
            f"{other_side}_key": getattr(own, other_side) if own else None,
            partner_field: partner_of.get(node.id),
        }
        if details is not None and own and "modified" in own.tags and node.shape is not None:
            entry["detail"] = detail_status(details, node.id, hint)
        status[node.id] = entry
    for node in reversed(index.ordered()):  # children before parents
        if node.shape is None:
            kids = [status[k.id]["class"] for k in index.children.get(node.id, [])]
            status[node.id]["children_class"] = strongest_class([*kids])
    return status


def has_move(change: Change | None) -> bool:
    return change is not None and bool({"moved", "re-parented"} & set(change.tags))


def strongest(tags: list[str]) -> str | None:
    return strongest_class([t for t in tags if t in CLASSES])


def strongest_class(classes: list[str | None]) -> str | None:
    order = ("added", "removed", *CLASSES)
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


def load_details(
    detail_dir: Path, old: Model, new: Model, result: Diff
) -> tuple[dict[str, dict], DetailFiles]:
    """Read and check a `--detail-all` folder against this very diff. Every entry
    must name a change tagged `modified` (same key, same ids, both fingerprints),
    its volume change must be the two shape volumes' difference, and its files
    must be exactly
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
        o_shape = old.shapes[old.node(change.old_id).shape]
        n_shape = new.shapes[new.node(change.new_id).shape]
        fingerprints = (entry.get("old_fingerprint"), entry.get("new_fingerprint"))
        if fingerprints != (o_shape.fingerprint, n_shape.fingerprint):
            raise mismatch(detail_dir, key, old, new)  # same ids, other geometry
        expected = n_shape.volume - o_shape.volume
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
