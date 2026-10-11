"""diff.json and the printed form of a Diff (docs/diff-plan.md, step 4)."""

from __future__ import annotations

import json
from dataclasses import asdict

from kyumi_diff.classify import Change, Diff
from kyumi_diff.index import KEY_FORMAT


def to_json(result: Diff) -> str:
    data = {
        "old": result.old,
        "new": result.new,
        "keys": KEY_FORMAT,
        "changes": [as_dict(c) for c in result.changes],
        "added": result.added,
        "removed": result.removed,
        "summary": result.summary,
    }
    return json.dumps(data, indent=2)


def as_dict(change: Change) -> dict:
    data = asdict(change)
    data["score"] = round(change.score, 3)
    if not change.delta:
        del data["delta"]
    return data


def agrees(result: Diff, expected: dict) -> bool:
    """Does the diff say what an expected.json says? Ids and scores are not compared:
    ids differ between files and scores are the matcher's business."""
    return comparable(json.loads(to_json(result))) == comparable(expected)


def comparable(answer: dict) -> dict:
    changes = [
        {k: v for k, v in c.items() if k not in ("old_id", "new_id", "score")}
        for c in answer["changes"]
    ]
    return {
        "changes": sorted(changes, key=lambda c: (c["old"], c["new"])),
        "added": sorted(answer["added"]),
        "removed": sorted(answer["removed"]),
        "summary": answer["summary"],
    }


def render_text(result: Diff) -> str:
    lines = ["  ".join(f"{tag} {count}" for tag, count in result.summary.items())]
    for change in result.changes:
        lines.append(f"{'+'.join(change.tags)}  {describe_change(change)}")
    lines += [f"added  {' › '.join(key)}" for key in result.added]
    lines += [f"removed  {' › '.join(key)}" for key in result.removed]
    return "\n".join(lines)


def describe_change(change: Change) -> str:
    text = " › ".join(change.old)
    if change.new != change.old:
        text += f"  →  {' › '.join(change.new)}"
    if "distance" in change.delta:
        text += f"  by {change.delta['distance']:.1f} mm"
    if "angle" in change.delta:
        text += f", {change.delta['angle']:.1f}°"
    if "volume" in change.delta:
        text += f"  volume {change.delta['volume']:+.1f} mm³"
    return text
