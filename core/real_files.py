"""Run the importer over the real STEP files in fixtures/real/ and write
docs/real-files.md with what happened to each.

Those files are not committed (their licences do not allow it), so this report
is how we track whether core copes with real-world exports. The v1/v2 pairs in
fixtures/real/pairs/<file>/ (from `fixtures/make_diff_pairs.py <file>`) are not
imported as single files; each is diffed (imported without breps, except the
modify pair, whose `--detail` volume is checked) and the result compared with
its expected.json. Every shape of every real file is also detail-diffed against
itself: coincident faces are where booleans break, so the counts of empty /
non-empty / "could not compute" results and the slowest part are recorded.

"junk names" counts leaves whose name carries no meaning: "", digits only,
Part/Body/Solid/Component + number, Onshape's "Part Studio N", each optionally
with a copy suffix (-2, :2, <2>). These are the files where categorize cannot
lean on names.

    python core/real_files.py
"""

from __future__ import annotations

import json
import re
import sys
import tempfile
import time
import zipfile
from pathlib import Path

from kyumi import Model, load
from kyumi.package import import_step
from kyumi_diff import Diff, agrees, detail, diff
from kyumi_diff.detail import FAILED, OK, compare, read_brep, volume_of

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / "fixtures" / "real"
PAIRS = REAL / "pairs"
REPORT = ROOT / "docs" / "real-files.md"


COLUMNS = (
    "file | import | units | nodes | shapes | depth | junk names | coloured | time | "
    ".kyumi size (with / without breps)"
)
JUNK_NAME = re.compile(
    r"^(|\d+|(part|body|solid|component)[ _-]?\d+|part studio \d+)"  # the base name
    r"( ?[-:]\d+| ?<\d+>)?$",  # optional copy suffix: -2, :2, <2>
    re.IGNORECASE,
)


def is_junk_name(name: str) -> bool:
    return JUNK_NAME.match(name) is not None


def percent(flags: list[bool]) -> str:
    return f"{100 * sum(flags) / len(flags):.0f} %" if flags else "-"


def report_line(step: Path, out_dir: Path) -> tuple[str, Model | None]:
    """One table row: counts and quality numbers, or the error that stopped the import."""
    out = out_dir / (step.stem + ".kyumi")
    try:
        timings = import_step(step, out)
        model = load(out)
    except Exception as error:  # noqa: BLE001 (any failure is a result worth recording)
        blanks = " |" * (COLUMNS.count("|") - 2)
        return (
            f"| {step.relative_to(REAL)} | failed: {type(error).__name__}: {error} |{blanks}",
            None,
        )
    leaves = model.leaves()
    groups = len(model.groups_by_fingerprint())
    cells = [
        str(step.relative_to(REAL)),
        "ok",
        model.source_units,
        str(len(model.nodes)),
        f"{len(model.shapes)} ({groups} distinct geometries)",
        str(max((len(model.path_of(n.id)) for n in model.nodes), default=0)),
        percent([is_junk_name(n.name) for n in leaves]),
        percent([model.color_of(n) is not None for n in leaves]),
        f"{sum(timings.values()):.1f} s",
        f"{out.stat().st_size / 1e6:.1f} MB / {size_without_breps(out) / 1e6:.1f} MB",
    ]
    return "| " + " | ".join(cells) + " |", model


def size_without_breps(package: Path) -> int:
    """What the .kyumi would weigh with `--no-breps`, without importing twice."""
    with zipfile.ZipFile(package) as zf:
        return sum(i.compress_size for i in zf.infolist() if not i.filename.startswith("breps/"))


SELF_COLUMNS = "file | shapes | empty | non-empty | could not compute | time | slowest part"


def self_comparison_line(model: Model) -> str:
    """Every shape detail-diffed against itself: the result should be empty each time."""
    counts = {"empty": 0, "non-empty": 0, FAILED: 0}
    slowest: tuple[float, str] = (0.0, "")
    start = time.perf_counter()
    for shape in model.shapes.values():
        began = time.perf_counter()
        brep = read_brep(model, shape)
        result = compare(brep, brep)
        if result is None:
            counts[FAILED] += 1
        elif volume_of(result[0]) + volume_of(result[1]) > 1e-6:
            counts["non-empty"] += 1
        else:
            counts["empty"] += 1
        slowest = max(slowest, (time.perf_counter() - began, shape.name))
    cells = [
        model.source_file,
        str(len(model.shapes)),
        *(str(n) for n in counts.values()),
        f"{time.perf_counter() - start:.1f} s",
        f"{slowest[1]} ({slowest[0]:.1f} s)",
    ]
    return "| " + " | ".join(cells) + " |"


PAIR_COLUMNS = "pair | diff | time | summary | detail"


def pair_lines(pair_dir: Path, out_dir: Path) -> list[str]:
    """One row per expected.json in a pair directory: does `kyumi diff` agree with it?"""
    rows = []
    for expected_file in sorted(pair_dir.glob("*.expected.json")):
        expected = json.loads(expected_file.read_text(encoding="utf-8"))
        name = f"{pair_dir.name}/{expected_file.name.removesuffix('.expected.json')}"
        modified = [c for c in expected["changes"] if "modified" in c["tags"]]
        try:
            start = time.perf_counter()
            models = []
            for step_name in (expected["old"], expected["new"]):
                suffix = "-breps" if modified else ""
                out = out_dir / f"{pair_dir.name}-{step_name}{suffix}.kyumi"
                if not out.exists():
                    import_step(pair_dir / step_name, out, breps=bool(modified))
                models.append(load(out))
            result = diff(*models)
            elapsed = time.perf_counter() - start
            detail_cell = detail_check(models, result, modified) if modified else ""
        except Exception as error:  # noqa: BLE001 (any failure is a result worth recording)
            rows.append(f"| {name} | failed: {type(error).__name__}: {error} | | | |")
            continue
        verdict = "ok" if agrees(result, expected) else "MISMATCH"
        summary = ", ".join(f"{k} {v}" for k, v in result.summary.items() if v)
        rows.append(
            f"| {name} | {verdict} | {elapsed:.1f} s | {summary or 'no change'} | {detail_cell} |"
        )
    return rows


def detail_check(models: list[Model], result: Diff, modified: list[dict]) -> str:
    """`--detail` on each modified part: removed - added must equal the expected volume change."""
    cells = []
    for change in modified:
        start = time.perf_counter()
        found = detail(models[0], models[1], result, change["new"])
        elapsed = time.perf_counter() - start
        if found.status != OK:
            cells.append(f"{found.status} ({elapsed:.1f} s)")
            continue
        wanted = change["delta"]["volume"]
        got = found.added - found.removed
        verdict = "ok" if abs(got - wanted) <= 0.01 * abs(wanted) else "MISMATCH"
        cells.append(
            f"{verdict}: added {found.added:.1f}, removed {found.removed:.1f} mm³ "
            f"(expected {wanted:+.1f}) in {elapsed:.1f} s"
        )
    return "; ".join(cells)


def main() -> None:
    steps = sorted(
        p
        for p in REAL.rglob("*")
        if p.suffix.lower() in (".step", ".stp") and PAIRS not in p.parents
    )
    lines = [
        "# Real files",
        "",
        "Generated by `python core/real_files.py` from the files in `fixtures/real/`",
        "(not committed). Re-run after changing the importer.",
        "",
    ]
    if not steps:
        lines.append("No STEP files found in `fixtures/real/`.")
    else:
        lines += [f"| {COLUMNS} |", "|---" * (COLUMNS.count("|") + 1) + "|"]
        self_lines = []
        with tempfile.TemporaryDirectory() as tmp:
            for step in steps:
                print(step.relative_to(REAL), file=sys.stderr)
                line, model = report_line(step, Path(tmp))
                lines.append(line)
                if model is not None:
                    self_lines.append(self_comparison_line(model))
        lines += ["", "## Every shape against itself", "", f"| {SELF_COLUMNS} |", "|---" * 7 + "|"]
        lines += self_lines
    pair_dirs = sorted(p for p in PAIRS.glob("*") if p.is_dir()) if PAIRS.is_dir() else []
    if pair_dirs:
        lines += ["", "## Pairs", "", f"| {PAIR_COLUMNS} |", "|---" * 5 + "|"]
        with tempfile.TemporaryDirectory() as tmp:
            for pair_dir in pair_dirs:
                print(pair_dir.relative_to(REAL), file=sys.stderr)
                lines += pair_lines(pair_dir, Path(tmp))
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {REPORT.relative_to(ROOT)} ({len(steps)} files, {len(pair_dirs)} pair sets)")


if __name__ == "__main__":
    main()
