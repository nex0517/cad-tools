# kyumi core

Turns a STEP file into a `.kyumi` package (see [../docs/format.md](../docs/format.md))
and shows it in a browser.

## Install

```sh
python3.11 -m venv .venv && source .venv/bin/activate
pip install -e core            # pulls in cadquery, numpy, trimesh
pip install pytest ruff        # for development
```

## Use

```sh
kyumi import fixtures/drone.step -o drone.kyumi   # prints time per stage
kyumi info drone.kyumi                            # the parts tree, copy counts, sizes
kyumi view drone.kyumi                            # 3D viewer in the browser (Milestone 4)
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
| `kyumi/cli.py`     | the `kyumi` command |

## Tests

```sh
ruff format core && ruff check core
pytest core
```
