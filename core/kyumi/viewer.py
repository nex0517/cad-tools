"""`kyumi view`: a tiny local web server plus one HTML page that draws the model.

The browser gets three things: the page itself, `/model.json` (the tree with
world transforms, so the page does no matrix maths), and `/meshes/<id>.glb`
straight out of the .kyumi zip. Nothing is written to disk.

Diff mode (`kyumi view OLD NEW`) adds `/changes.json`, `/old/meshes/<id>.glb`
for removed parts and `/detail/<new id>/<side>.glb`; the CLI works all of that
out with the diff package and hands it in as a `DiffView`, so this module never
imports kyumi_diff.
"""

from __future__ import annotations

import json
import webbrowser
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from kyumi.model import Model

PAGE = Path(__file__).with_name("viewer.html")


@dataclass
class DiffView:
    """Everything diff mode serves besides the new model (built by kyumi_diff.view)."""

    changes: dict  # what /changes.json answers
    old: Model  # removed parts' meshes come from here
    old_meshes: set[str]  # the only old files served, e.g. {"meshes/s3.glb"}
    details: dict[str, dict[str, bytes | None]] = field(default_factory=dict)  # new id -> side

    def detail_glb(self, path: str) -> bytes | None:
        """`/detail/<new id>/<side>.glb` for a verified, non-empty side; else None."""
        parts = path.split("/")
        if len(parts) != 4 or parts[:2] != ["", "detail"] or not parts[3].endswith(".glb"):
            return None
        return self.details.get(parts[2], {}).get(parts[3][: -len(".glb")])


def model_json(model: Model) -> dict:
    """What the page needs: nodes with world transforms, shapes with measurements."""
    nodes = []
    for node in model.nodes:
        nodes.append(
            {
                "id": node.id,
                "name": node.name,
                "parent": node.parent,
                "shape": node.shape,
                "path": model.path_of(node.id),
                "color": model.color_of(node),
                # three.js wants column-major; our matrices are row-major, so transpose.
                "world": model.world_transform(node.id).T.flatten().tolist(),
            }
        )
    shapes = {
        sid: {
            "name": s.name,
            "mesh": s.mesh,
            "volume": s.volume,
            "area": s.area,
            "size": s.size,
            "faces": s.faces,
            "fingerprint": s.fingerprint,
            "copies": len(model.copies(sid)),
        }
        for sid, s in model.shapes.items()
    }
    return {"file": model.source_file, "nodes": nodes, "shapes": shapes}


def make_handler(model: Model, view: DiffView | None = None) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (name fixed by http.server)
            # A page on another site can make the browser call 127.0.0.1, but
            # its Host header names that site; only answer requests meant for us.
            if self.headers.get("Host") not in local_hosts(self.server.server_port):
                self.reply(403, "text/plain", b"forbidden: unexpected Host header")
            elif self.path == "/":
                self.reply(200, "text/html", PAGE.read_bytes())
            elif self.path == "/model.json":
                self.reply(200, "application/json", json.dumps(model_json(model)).encode())
            elif self.path.startswith("/meshes/") and self.path[1:] in mesh_paths(model):
                self.reply(200, "model/gltf-binary", model.read(self.path[1:]))
            elif view is not None and self.path == "/changes.json":
                self.reply(200, "application/json", json.dumps(view.changes).encode())
            elif view is not None and self.path.startswith("/old/meshes/"):
                if self.path[len("/old/") :] in view.old_meshes:
                    self.reply(200, "model/gltf-binary", view.old.read(self.path[len("/old/") :]))
                else:
                    self.reply(404, "text/plain", b"not found")
            elif view is not None and view.detail_glb(self.path) is not None:
                self.reply(200, "model/gltf-binary", view.detail_glb(self.path))
            else:
                self.reply(404, "text/plain", b"not found")

        def reply(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args: object) -> None:
            pass  # keep the terminal quiet

    return Handler


def local_hosts(port: int) -> set[str]:
    return {f"127.0.0.1:{port}", f"localhost:{port}"}


def mesh_paths(model: Model) -> set[str]:
    return {s.mesh for s in model.shapes.values() if s.mesh}


def serve(
    model: Model, port: int = 8765, open_browser: bool = True, view: DiffView | None = None
) -> None:
    try:
        server = HTTPServer(("127.0.0.1", port), make_handler(model, view))
    except OSError as error:
        raise ValueError(f"cannot listen on port {port} ({error.strerror}); try --port 0") from None
    url = f"http://127.0.0.1:{server.server_port}/"
    what = model.source_file if view is None else f"{view.old.source_file} → {model.source_file}"
    print(f"viewing {what} at {url}  (Ctrl+C to stop)")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
