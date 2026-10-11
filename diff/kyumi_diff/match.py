"""Pair every new node with the old node it "is" (docs/diff-plan.md, step 1).

Leaves first: candidates are narrowed by name, fingerprint and a spatial grid,
scored, then assigned greedily. Groups follow from their matched children.
"""

from __future__ import annotations

import math
from collections import Counter

from kyumi import Node

from kyumi_diff.index import Index

WEIGHTS = {
    "fingerprint": 0.40,
    "distance": 0.30,
    "node_name": 0.10,
    "shape_name": 0.10,
    "parent": 0.10,
}
MIN_SCORE = 0.3
# Grid cell = the typical part's size, so a cell and its 26 neighbours hold the parts
# that could plausibly be the same part slightly moved, however big the assembly.
# A fingerprint shared by more copies than this (1,280 screws in a tiled drone) is
# no longer a useful lookup on its own; those copies are only found through the grid.
MAX_FINGERPRINT_BUCKET = 64

Cell = tuple[int, int, int]


def match(old: Index, new: Index) -> dict[str, str]:
    """old node id -> new node id, for leaves and groups alike."""
    pairs = match_leaves(old, new)
    pairs.update(match_groups(old, new, pairs))
    return pairs


# ---------- leaves ----------


def match_leaves(old: Index, new: Index) -> dict[str, str]:
    cell_size = grid_cell_size(old)
    by_name: dict[str, list[Node]] = {}
    by_fingerprint: dict[str, list[Node]] = {}
    by_cell: dict[Cell, list[Node]] = {}
    for leaf in old.leaves:
        by_name.setdefault(leaf.name, []).append(leaf)
        by_fingerprint.setdefault(old.model.shapes[leaf.shape].fingerprint, []).append(leaf)
        by_cell.setdefault(cell_of(old.center[leaf.id], cell_size), []).append(leaf)

    scored: list[tuple[float, str, str]] = []
    for n in new.leaves:
        candidates: dict[str, Node] = {}
        for o in by_name.get(n.name, []) if n.name else []:
            candidates[o.id] = o
        same_fingerprint = by_fingerprint.get(new.model.shapes[n.shape].fingerprint, [])
        if len(same_fingerprint) <= MAX_FINGERPRINT_BUCKET:
            for o in same_fingerprint:
                candidates[o.id] = o
        for cell in neighbourhood(cell_of(new.center[n.id], cell_size)):
            for o in by_cell.get(cell, []):
                candidates[o.id] = o
        for o in candidates.values():
            s = score(old, o, new, n, cell_size)
            if s >= MIN_SCORE:
                scored.append((s, o.id, n.id))

    # Greedy: best score first; ties broken by the order the nodes appear in the file.
    scored.sort(key=lambda t: (-t[0], old.key[t[1]], new.key[t[2]]))
    pairs: dict[str, str] = {}
    taken_new: set[str] = set()
    for _, o_id, n_id in scored:
        if o_id in pairs or n_id in taken_new:
            continue
        pairs[o_id] = n_id
        taken_new.add(n_id)
    return pairs


def score(old: Index, o: Node, new: Index, n: Node, tolerance: float) -> float:
    o_shape, n_shape = old.model.shapes[o.shape], new.model.shapes[n.shape]
    distance = math.dist(old.center[o.id], new.center[n.id])
    total = WEIGHTS["distance"] * math.exp(-distance / tolerance)
    if o_shape.fingerprint == n_shape.fingerprint:
        total += WEIGHTS["fingerprint"]
    if o.name and o.name == n.name:
        total += WEIGHTS["node_name"]
    if o_shape.name and o_shape.name == n_shape.name:
        total += WEIGHTS["shape_name"]
    if old.parent_names[o.id] == new.parent_names[n.id]:
        total += WEIGHTS["parent"]
    return total


def grid_cell_size(index: Index) -> float:
    """The median part's bounding-box diagonal (at least 1 mm)."""
    if not index.leaves:
        return 1.0
    diagonals = sorted(math.hypot(*index.model.shapes[n.shape].size) for n in index.leaves)
    return max(diagonals[len(diagonals) // 2], 1.0)


def cell_of(point: tuple[float, float, float], size: float) -> Cell:
    return (int(point[0] // size), int(point[1] // size), int(point[2] // size))


def neighbourhood(cell: Cell) -> list[Cell]:
    x, y, z = cell
    return [(x + i, y + j, z + k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]


# ---------- groups ----------


def match_groups(old: Index, new: Index, leaf_pairs: dict[str, str]) -> dict[str, str]:
    """A new group matches the old group holding most of its matched children; a
    tie (eight identical LegUnits) goes to the old group whose children sit closest."""
    old_of_new = {n: o for o, n in leaf_pairs.items()}
    taken_old: set[str] = set()
    pairs: dict[str, str] = {}
    for group in sorted(new.groups, key=lambda g: -new.depth[g.id]):  # deepest first
        votes: Counter[str] = Counter()
        distance: dict[str, float] = {}
        for child in new.children.get(group.id, []):
            o_child = old_of_new.get(child.id)
            if o_child is None:
                continue
            o_parent = old.by_id[o_child].parent
            if o_parent is None or o_parent in taken_old:
                continue
            votes[o_parent] += 1
            if child.shape is not None:
                d = math.dist(old.center[o_child], new.center[child.id])
                distance[o_parent] = distance.get(o_parent, 0.0) + d
        if not votes:
            continue
        best = min(votes, key=lambda o: (-votes[o], distance.get(o, 0.0), old.key[o]))
        pairs[best] = group.id
        taken_old.add(best)
        old_of_new[group.id] = best
    return pairs
