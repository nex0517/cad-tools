"""diff.json and the printed form of a Diff (docs/diff-plan.md, step 4)."""

from __future__ import annotations

import json
from dataclasses import asdict

from kyumi_diff.classify import KEY_FORMAT, Change, Diff


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
    if not change.delta:
        del data["delta"]
    return data


def render_text(result: Diff) -> str:
    lines = [
        f"matched {result.matched_leaves}/{result.old_leaves} parts "
        f"({100 * result.match_rate:.0f} %)",
        "  ".join(f"{tag} {count}" for tag, count in result.summary.items()),
    ]
    for change in result.changes:
        lines.append(f"{'+'.join(change.tags):<12} {describe_change(change)}")
    lines += [f"{'added':<12} {' › '.join(key)}" for key in result.added]
    lines += [f"{'removed':<12} {' › '.join(key)}" for key in result.removed]
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
