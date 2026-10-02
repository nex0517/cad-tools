"""Command line: kyumi import | info | view."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from kyumi.model import Model, Node
from kyumi.package import import_step, load
from kyumi.reader import UnreadableFile


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="kyumi", description="STEP in, .kyumi out.")
    commands = parser.add_subparsers(dest="command", required=True)

    imp = commands.add_parser("import", help="convert a STEP file to a .kyumi package")
    imp.add_argument("step", type=Path)
    imp.add_argument("-o", "--out", type=Path, help="default: <step name>.kyumi")
    imp.add_argument("--no-breps", action="store_true", help="skip exact geometry (smaller file)")

    info = commands.add_parser("info", help="print the parts tree of a .kyumi package")
    info.add_argument("kyumi", type=Path)

    args = parser.parse_args(argv)
    try:
        if args.command == "import":
            run_import(args.step, args.out or args.step.with_suffix(".kyumi"), not args.no_breps)
        elif args.command == "info":
            print_info(load(args.kyumi))
    except (UnreadableFile, ValueError) as error:
        sys.exit(f"error: {error}")


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
