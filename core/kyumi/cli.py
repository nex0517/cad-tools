"""Command line: kyumi import | info | view | diff."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from kyumi.model import Model, Node
from kyumi.package import import_step, load
from kyumi.reader import UnreadableFile
from kyumi.viewer import DiffView, serve

try:
    import kyumi_diff
except ImportError:  # the diff/ package is installed separately: pip install -e diff
    kyumi_diff = None


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="kyumi", description="STEP in, .kyumi out.")
    commands = parser.add_subparsers(dest="command", required=True)

    imp = commands.add_parser("import", help="convert a STEP file to a .kyumi package")
    imp.add_argument("step", type=Path)
    imp.add_argument("-o", "--out", type=Path, help="default: <step name>.kyumi")
    imp.add_argument("--no-breps", action="store_true", help="skip exact geometry (smaller file)")

    info = commands.add_parser("info", help="print the parts tree of a .kyumi package")
    info.add_argument("kyumi", type=Path)

    view = commands.add_parser("view", help="open a .kyumi package in the browser")
    view.add_argument("kyumi", type=Path)
    view.add_argument("new", type=Path, nargs="?", help="a second package: show what changed")
    view.add_argument(
        "--detail",
        type=Path,
        metavar="DIR",
        help="folder written by kyumi diff OLD NEW --detail-all (two packages only)",
    )
    view.add_argument("--port", type=int, default=8765, help="0 picks a free port")
    view.add_argument("--no-browser", action="store_true", help="just print the URL")

    dif = commands.add_parser("diff", help="what changed between two .kyumi packages")
    dif.add_argument("old", type=Path)
    dif.add_argument("new", type=Path)
    dif.add_argument("-o", "--out", type=Path, help="also write diff.json here")
    detail_mode = dif.add_mutually_exclusive_group()
    detail_mode.add_argument(
        "--detail",
        metavar="KEY",
        help="also compute the material added/removed in one part, e.g. Drone/Frame/CenterPlate",
    )
    detail_mode.add_argument(
        "--detail-all",
        action="store_true",
        help="compute the detail of every modified part ahead, for kyumi view --detail",
    )
    dif.add_argument(
        "--out-dir",
        type=Path,
        default=Path("detail"),
        help="where --detail / --detail-all write (default: detail/)",
    )

    args = parser.parse_args(argv)
    try:
        if args.command == "import":
            run_import(args.step, args.out or args.step.with_suffix(".kyumi"), not args.no_breps)
        elif args.command == "info":
            print_info(load(args.kyumi))
        elif args.command == "view":
            run_view(args.kyumi, args.new, args.detail, args.port, not args.no_browser)
        elif args.command == "diff":
            run_diff(args.old, args.new, args.out, args.detail, args.out_dir, args.detail_all)
    except (UnreadableFile, ValueError) as error:
        sys.exit(f"error: {error}")


def run_view(
    old: Path, new: Path | None, detail_dir: Path | None, port: int, open_browser: bool
) -> None:
    if new is None:
        if detail_dir is not None:
            raise ValueError("--detail needs two packages: kyumi view OLD NEW --detail DIR")
        serve(load(old), port, open_browser)
        return
    serve(load(new), port, open_browser, diff_view(load(old), load(new), detail_dir))


def diff_view(old_model: Model, new_model: Model, detail_dir: Path | None) -> DiffView:
    """Diff the two packages and check the detail folder, before the server starts."""
    if kyumi_diff is None:
        raise ValueError("kyumi view with two packages needs the diff package: pip install -e diff")
    result = kyumi_diff.diff(old_model, new_model)
    entries, files = {}, {}
    if detail_dir is not None:
        entries, files = kyumi_diff.load_details(detail_dir, old_model, new_model, result)
    changes = kyumi_diff.changes_json(old_model, new_model, result, entries, detail_dir)
    return DiffView(changes, old_model, kyumi_diff.ghost_meshes(old_model, result), files)


def run_diff(
    old: Path,
    new: Path,
    out: Path | None,
    detail: str | None = None,
    out_dir: Path = Path("detail"),
    detail_all: bool = False,
) -> None:
    if kyumi_diff is None:
        raise ValueError("kyumi diff needs the diff package: pip install -e diff")
    if out is not None and out.resolve() in (old.resolve(), new.resolve()):
        raise ValueError(f"-o {out} would overwrite an input package")
    old_model, new_model = load(old), load(new)
    result = kyumi_diff.diff(old_model, new_model)
    print(kyumi_diff.render_text(result))
    if out:
        out.write_text(kyumi_diff.to_json(result), encoding="utf-8")
    if detail is not None:
        found = kyumi_diff.detail(old_model, new_model, result, kyumi_diff.parse_key(detail))
        print(kyumi_diff.render_detail(found, kyumi_diff.write_detail(found, out_dir)))
    if detail_all:
        kyumi_diff.detail_all(old_model, new_model, result, out_dir)


def run_import(step: Path, out: Path, breps: bool) -> None:
    timings = import_step(step, out, breps)
    model = load(out)
    for stage, seconds in timings.items():
        print(f"{stage:>8}: {seconds * 1000:7.0f} ms")
    print(f"   total: {sum(timings.values()) * 1000:7.0f} ms")
    size_kb = out.stat().st_size / 1024
    print(f"wrote {out} ({size_kb:.0f} KB): {len(model.nodes)} nodes, {len(model.shapes)} shapes")


def print_info(model: Model) -> None:
    print(f"{model.source_file} (declared {model.source_units}, stored in mm)")
    for root in model.roots():
        print_node(model, root, depth=0)
    print(f"{len(model.nodes)} nodes, {len(model.shapes)} unique shapes")


def print_node(model: Model, node: Node, depth: int) -> None:
    line = "  " * depth + (node.name or "(unnamed)")
    if node.shape:
        shape = model.shapes[node.shape]
        copies = len(model.copies(node.shape))
        size = " x ".join(f"{s:.1f}" for s in shape.size)
        line += f"  [{shape.name or node.shape}, x{copies}, {size} mm]"
    print(line)
    for child in model.children(node.id):
        print_node(model, child, depth + 1)
