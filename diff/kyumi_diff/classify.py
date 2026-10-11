"""Tag each matched pair and collect the unmatched (docs/diff-plan.md, step 2)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field

import numpy as np
from kyumi import Model, Node, Shape

from kyumi_diff.index import Index, Key
from kyumi_diff.match import match

TAGS = ("added", "removed", "moved", "renamed", "modified", "re-parented")
KEY_FORMAT = "names from the root down; the k-th same-named sibling (k >= 1) is 'Name#k'"
POSITION_TOLERANCE = 0.01  # mm
ANGLE_TOLERANCE = 0.01  # degrees
MEASUREMENT_TOLERANCE = 0.001  # relative: volume, area or inertia must differ by 0.1 %


@dataclass
class Change:
    old: Key
    new: Key
    tags: list[str]
    delta: dict[str, float] = field(default_factory=dict)


@dataclass
class Diff:
    old: str
    new: str
    changes: list[Change]
    added: list[Key]
    removed: list[Key]
    matched_leaves: int
    old_leaves: int

    @property
    def summary(self) -> dict[str, int]:
        counts = {tag: 0 for tag in TAGS}
        counts["added"], counts["removed"] = len(self.added), len(self.removed)
        for change in self.changes:
            for tag in change.tags:
                counts[tag] += 1
        return counts

    @property
    def match_rate(self) -> float:
        return self.matched_leaves / self.old_leaves if self.old_leaves else 1.0


def diff(old: Model, new: Model) -> Diff:
    warn_if_generators_differ(old, new)
    o_index, n_index = Index(old), Index(new)
    pairs = match(o_index, n_index)
    changes: list[Change] = []
    moved: set[str] = set()
    for o in o_index.ordered():  # parents first, so "did an ancestor move" is known
        n_id = pairs.get(o.id)
        if n_id is None:
            continue
        n = n_index.by_id[n_id]
        tags, delta = classify(o_index, o, n_index, n, pairs, moved)
        if "moved" in tags:
            moved.add(o.id)
        if tags:
            changes.append(Change(o_index.key[o.id], n_index.key[n.id], tags, delta))
    matched_new = set(pairs.values())
    return Diff(
        old=old.source_file,
        new=new.source_file,
        changes=changes,
        added=sorted(n_index.key[n.id] for n in new.nodes if n.id not in matched_new),
        removed=sorted(o_index.key[o.id] for o in old.nodes if o.id not in pairs),
        matched_leaves=sum(1 for o in o_index.leaves if o.id in pairs),
        old_leaves=len(o_index.leaves),
    )


def warn_if_generators_differ(old: Model, new: Model) -> None:
    if old.generator != new.generator:
        print(
            f"warning: the packages were written by different generators "
            f"({describe(old.generator)} vs {describe(new.generator)}); fingerprints may "
            f"not be comparable, so 'modified' results could be false",
            file=sys.stderr,
        )


def describe(generator: dict[str, str] | None) -> str:
    if not generator:
        return "unknown"
    return f"{generator.get('name', '?')} {generator.get('version', '?')}"


def classify(
    old: Index, o: Node, new: Index, n: Node, pairs: dict[str, str], moved: set[str]
) -> tuple[list[str], dict[str, float]]:
    tags: list[str] = []
    delta: dict[str, float] = {}
    if o.name != n.name:
        tags.append("renamed")
    if (None if o.parent is None else pairs.get(o.parent)) != n.parent:
        tags.append("re-parented")
    modified = o.shape is not None and shapes_differ(
        old.model.shapes[o.shape], new.model.shapes[n.shape]
    )
    if modified:
        tags.append("modified")
        delta["volume"] = new.model.shapes[n.shape].volume - old.model.shapes[o.shape].volume
    if any(a in moved for a in old.ancestors(o.id)):
        # Something above moved; this node only moved if its *own* transform changed.
        a, b = local(o), local(n)
        distance, angle = float(np.linalg.norm(a[:3, 3] - b[:3, 3])), rotation_angle(a, b)
    elif (
        o.shape is None
        or modified
        or same_frame(old.model.shapes[o.shape], new.model.shapes[n.shape])
    ):
        # Groups, modified parts (their centre of mass shifts) and parts whose
        # geometry is defined in the same frame: compare world transforms, which
        # also catches a rotation about the centre.
        a, b = old.world[o.id], new.world[n.id]
        distance, angle = float(np.linalg.norm(a[:3, 3] - b[:3, 3])), rotation_angle(a, b)
    else:
        # Re-baked geometry (identity transform, placement inside the shape): only
        # where the centre ended up is comparable; a rotation about it is invisible.
        distance, angle = math.dist(old.center[o.id], new.center[n.id]), 0.0
    if distance > POSITION_TOLERANCE or angle > ANGLE_TOLERANCE:
        tags.append("moved")
        delta["distance"] = distance
        if angle > ANGLE_TOLERANCE:
            delta["angle"] = angle
    return sorted(tags), {k: round(v, 3) for k, v in delta.items()}


def shapes_differ(a: Shape, b: Shape) -> bool:
    """Modified = fingerprint differs *and* a raw measurement confirms it (not just rounding)."""
    if a.fingerprint == b.fingerprint:
        return False
    if a.faces != b.faces or a.solids != b.solids:
        return True
    pairs = [(a.volume, b.volume), (a.area, b.area), *zip(a.inertia, b.inertia, strict=True)]
    return any(abs(x - y) > MEASUREMENT_TOLERANCE * max(abs(x), abs(y), 1e-9) for x, y in pairs)


def same_frame(a: Shape, b: Shape) -> bool:
    return float(np.linalg.norm(np.array(a.center) - np.array(b.center))) <= POSITION_TOLERANCE


def local(node: Node) -> np.ndarray:
    return np.array(node.transform).reshape(4, 4)


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3].T @ b[:3, :3]
    cos = (np.trace(relative) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cos)))))
