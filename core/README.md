# kyumi core

Turns a STEP file into a `.kyumi` package (see [../docs/format.md](../docs/format.md))
and shows it in a browser.

## Install

Mac / Linux:

```sh
python3.11 -m venv .venv && source .venv/bin/activate
pip install -e core            # pulls in OCP (OpenCascade), numpy, trimesh
pip install -e "core[dev]"     # adds cadquery (fixture generator), pytest, ruff
```

Windows (Command Prompt):

```bat
py -3.11 -m venv .venv && .venv\Scripts\activate
pip install -e core
pip install -e "core[dev]"
```

## Use

```sh
kyumi import fixtures/drone.step -o drone.kyumi   # prints time per stage
kyumi info drone.kyumi                            # the parts tree, copy counts, sizes
kyumi view drone.kyumi                            # 3D viewer in the browser (Ctrl+C to stop)
```

`kyumi import --no-breps` skips the exact geometry files (`breps/`), which diff
needs but the viewer does not.

From Python:

```python
from kyumi import load

model = load("drone.kyumi")
for node in model.nodes:
    if node.shape:
        print(model.path_of(node.id), model.shapes[node.shape].volume)
```

## How it works

| file | job |
|------|-----|
| `kyumi/reader.py`  | STEP → tree of nodes + raw geometry per unique shape (XCAF reader keeps names, colours, instances) |
| `kyumi/measure.py` | volume, area, size, inertia, face types, fingerprint — once per shape |
| `kyumi/mesh.py`    | triangle mesh (GLB) per shape, detail scaled to the shape's size |
| `kyumi/package.py` | writes/reads the zip; `import_step()` and `load()` |
| `kyumi/model.py`   | the `Model`, `Shape`, `Node` dataclasses that diff and categorize will use |
| `kyumi/viewer.py`  | `kyumi view`: serves `viewer.html`, `/model.json` and the meshes from the zip |
| `kyumi/viewer.html`| the page: three.js from a CDN, orbit, click-to-select, tree panel. No build step |
| `kyumi/cli.py`     | the `kyumi` command |
| `real_files.py`    | `python core/real_files.py`: imports every STEP in `fixtures/real/` (not committed) and writes `docs/real-files.md` |

## Tests

```sh
ruff format core && ruff check core
pytest core
```
