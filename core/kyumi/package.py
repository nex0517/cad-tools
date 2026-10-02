"""Write and read .kyumi zip packages (see docs/format.md)."""

from __future__ import annotations

import json
import tempfile
import time
import zipfile
from dataclasses import asdict
from pathlib import Path

from OCP.BRepTools import BRepTools
from OCP.TopTools import TopTools_FormatVersion

from kyumi.measure import measure
from kyumi.mesh import mesh_glb
from kyumi.model import FORMAT, VERSION, Model, Node, Shape
from kyumi.reader import RawModel, read_step

GENERATOR = {"name": "kyumi-core", "version": "0.1.0"}


def import_step(step_path: Path, out_path: Path, breps: bool = True) -> dict[str, float]:
    """STEP -> .kyumi. Returns seconds spent per stage so the CLI can print them."""
    if out_path.resolve() == step_path.resolve():
        raise ValueError(f"{out_path}: output would overwrite the input STEP file")
    timings: dict[str, float] = {}

    start = time.perf_counter()
    raw = read_step(step_path)
    timings["read"] = time.perf_counter() - start

    start = time.perf_counter()
    shapes = {sid: measure(r) for sid, r in raw.shapes.items()}
    timings["measure"] = time.perf_counter() - start

    start = time.perf_counter()
    meshes = {sid: mesh_glb(r.geometry, shapes[sid].size) for sid, r in raw.shapes.items()}
    timings["mesh"] = time.perf_counter() - start

    start = time.perf_counter()
    write_package(out_path, step_path, raw, shapes, meshes, breps)
    timings["write"] = time.perf_counter() - start
    return timings


def write_package(
    out_path: Path,
    step_path: Path,
    raw: RawModel,
    shapes: dict[str, Shape],
    meshes: dict[str, bytes],
    breps: bool,
) -> None:
    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for sid, shape in shapes.items():
            shape.mesh = f"meshes/{sid}.glb"
            zf.writestr(shape.mesh, meshes[sid])
            if breps:
                shape.brep = f"breps/{sid}.brep"
                zf.writestr(shape.brep, brep_text(raw.shapes[sid]))
        zf.write(step_path, "model.step")
        model = Model(step_path.name, raw.units, shapes, raw.nodes)
        zf.writestr("manifest.json", json.dumps(manifest(model), indent=1))


def brep_text(raw_shape) -> str:
    # OCP's stream overload of BRepTools.Write crashes, so go through a temp file.
    # Triangles are left out: the .glb already has them and they bloat the text.
    with tempfile.NamedTemporaryFile(suffix=".brep") as tmp:
        BRepTools.Write_s(
            raw_shape.geometry,
            tmp.name,
            False,
            False,
            TopTools_FormatVersion.TopTools_FormatVersion_VERSION_1,
        )
        return Path(tmp.name).read_text()


def manifest(model: Model) -> dict:
    return {
        "format": FORMAT,
        "version": VERSION,
        "generator": GENERATOR,
        "source": {"file": model.source_file, "units": model.source_units},
        "shapes": {sid: without_id(asdict(s)) for sid, s in model.shapes.items()},
        "nodes": [compact_node(n) for n in model.nodes],
    }


def without_id(fields: dict) -> dict:
    fields.pop("id")
    return fields


def compact_node(node: Node) -> dict:
    fields = asdict(node)
    if fields["color"] is None:
        del fields["color"]  # optional in the format: only written when set
    return fields


def load(path: Path | str) -> Model:
    """Read a .kyumi package. Unknown fields and files are ignored on purpose."""
    path = Path(path)
    if not zipfile.is_zipfile(path):
        raise ValueError(f"{path}: not a .kyumi package (run `kyumi import` on STEP files first)")
    with zipfile.ZipFile(path) as zf:
        if "manifest.json" not in zf.namelist():
            raise ValueError(f"{path}: not a .kyumi package (no manifest.json inside the zip)")
        data = json.loads(zf.read("manifest.json"))
    if data.get("format") != FORMAT:
        raise ValueError(f"{path}: manifest.json is not a kyumi manifest")
    if data.get("version", 0) > VERSION:
        raise ValueError(f"{path}: format version {data['version']} is newer than this reader")

    shapes = {sid: Shape(id=sid, **known(Shape, s)) for sid, s in data["shapes"].items()}
    nodes = [Node(**known(Node, n)) for n in data["nodes"]]
    source = data.get("source", {})
    return Model(source.get("file", ""), source.get("units", "mm"), shapes, nodes, path)


def known(cls: type, fields: dict) -> dict:
    """Keep only the fields this version of the dataclass understands."""
    names = cls.__dataclass_fields__
    return {k: v for k, v in fields.items() if k in names}
