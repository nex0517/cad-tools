"""The viewer's server side: what the page is given, and what the server answers."""

from __future__ import annotations

import threading
from http.server import HTTPServer
from pathlib import Path
from urllib.request import Request, urlopen

import pytest
from kyumi.model import Model
from kyumi.viewer import make_handler, model_json


def test_model_json_has_world_transforms(drone: Model) -> None:
    data = model_json(drone)
    motor = next(n for n in data["nodes"] if n["name"] == "Motor_FL")
    assert motor["path"] == ["Drone", "Propulsion", "Motor_FL"]
    assert motor["color"] == "#b0b0b0"
    # Column-major: translation sits in the last four entries.
    assert motor["world"][12:14] == pytest.approx([-77.78, 77.78], abs=0.01)
    assert data["shapes"][motor["shape"]]["copies"] == 4


def test_server_answers(drone: Model, tmp_path: Path) -> None:
    server = HTTPServer(("127.0.0.1", 0), make_handler(drone))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        assert b"<title>kyumi viewer</title>" in urlopen(base + "/").read()
        assert urlopen(base + "/model.json").headers["Content-Type"] == "application/json"
        glb = urlopen(base + "/meshes/s1.glb").read()
        assert glb[:4] == b"glTF"
        with pytest.raises(Exception, match="404"):
            urlopen(base + "/manifest.json")  # only the page, model.json and meshes are served
        # A request the browser was tricked into sending to us names another site.
        with pytest.raises(Exception, match="403"):
            urlopen(Request(base + "/model.json", headers={"Host": "evil.example:80"}))
    finally:
        server.shutdown()
