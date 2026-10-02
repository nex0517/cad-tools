"""`kyumi view`: a tiny local web server plus one HTML page that draws the model.

The browser gets three things: the page itself, `/model.json` (the tree with
world transforms, so the page does no matrix maths), and `/meshes/<id>.glb`
straight out of the .kyumi zip. Nothing is written to disk.
"""

from __future__ import annotations

import json
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from kyumi.model import Model

PAGE = Path(__file__).with_name("viewer.html")


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


def make_handler(model: Model) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (name fixed by http.server)
            if self.path == "/":
                self.reply(200, "text/html", PAGE.read_bytes())
            elif self.path == "/model.json":
                self.reply(200, "application/json", json.dumps(model_json(model)).encode())
            elif self.path.startswith("/meshes/") and self.path[1:] in mesh_paths(model):
                self.reply(200, "model/gltf-binary", model.read(self.path[1:]))
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


def mesh_paths(model: Model) -> set[str]:
    return {s.mesh for s in model.shapes.values() if s.mesh}


def serve(model: Model, port: int = 8765, open_browser: bool = True) -> None:
    try:
        server = HTTPServer(("127.0.0.1", port), make_handler(model))
    except OSError as error:
        raise ValueError(f"cannot listen on port {port} ({error.strerror}); try --port 0") from None
    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"viewing {model.source_file} at {url}  (Ctrl+C to stop)")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
