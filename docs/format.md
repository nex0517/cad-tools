# The .kyumi format, version 1

A `.kyumi` file is a normal zip archive. It packages one CAD model so that other
tools (the viewer, diff, categorize) never have to parse STEP themselves.

```
model.kyumi
├── manifest.json        the parts tree + measurements for each unique shape
├── model.step           untouched copy of the original file (byte for byte)
├── meshes/<shape>.glb   one triangle mesh per unique shape (for the viewer)
└── breps/<shape>.brep   exact geometry per unique shape (for detailed diff)
```

## Core idea: shapes vs. nodes

- A **shape** is a piece of unique geometry ("the motor").
- A **node** is a place in the assembly tree ("the front-left motor").

Four identical motors are **1 shape and 4 nodes**. Everything expensive
(measuring, meshing) is done once per shape, never once per copy.

## manifest.json

```json
{
  "format": "kyumi",
  "version": 1,
  "generator": { "name": "kyumi-core", "version": "0.1.0" },
  "source": { "file": "drone.step", "units": "mm" },
  "shapes": {
    "s1": {
      "name": "Motor",
      "mesh": "meshes/s1.glb",
      "brep": "breps/s1.brep",
      "volume": 6280.5,
      "area": 2310.2,
      "size": [28.0, 28.0, 15.0],
      "center": [0.0, 0.0, 7.5],
      "inertia": [180000.0, 180000.0, 300000.0],
      "faces": { "plane": 4, "cylinder": 6, "cone": 0, "sphere": 0,
                 "torus": 0, "bspline": 0, "other": 0 },
      "solids": 1,
      "color": "#2b2b2b",
      "fingerprint": "9f2c41ab03d7e815"
    }
  },
  "nodes": [
    { "id": "n0", "name": "Drone",    "parent": null, "shape": null, "transform": [16 numbers] },
    { "id": "n7", "name": "Motor_FL", "parent": "n3", "shape": "s1", "transform": [16 numbers],
      "color": "#ff0000" }
  ]
}
```

### Top level

| field       | type   | meaning |
|-------------|--------|---------|
| `format`    | string | always `"kyumi"` |
| `version`   | int    | format version, currently `1` (see *Versioning*) |
| `generator` | object | optional: which program wrote the file; for debugging only |
| `source`    | object | `file`: original file name. `units`: length unit the STEP file *declared* (`"mm"`, `"inch"`, `"m"`, ...). Informational only: every length in the manifest is already in mm |
| `shapes`    | object | shape id → shape (below) |
| `nodes`     | list   | every node of the tree (below) |

### Shapes

Shape ids are `s1`, `s2`, ... in the order the importer first meets them while
walking the tree. They are stable for the same input file but mean nothing
across different files (matching across versions is the diff tool's job).

| field         | type      | meaning |
|---------------|-----------|---------|
| `name`        | string    | name of the shape (STEP "product") exactly as found; may be `""` or junk like `"Part37"` |
| `mesh`        | string    | path inside the zip of the GLB mesh, in the shape's own coordinates |
| `brep`        | string    | path inside the zip of the exact geometry in OpenCascade's BREP text format, in the shape's own coordinates |
| `volume`      | number    | mm³ |
| `area`        | number    | total surface area, mm² |
| `size`        | [x, y, z] | extents of the axis-aligned bounding box in the shape's own coordinates, mm |
| `center`      | [x, y, z] | centre of mass in the shape's own coordinates, mm |
| `inertia`     | [a, b, c] | principal moments of inertia about the centre of mass, density 1, sorted ascending, mm⁵ |
| `faces`       | object    | number of faces per surface type; the keys above are always present |
| `solids`      | int       | number of solids (a "part" can contain several) |
| `color`       | string    | `"#rrggbb"` in sRGB, or `null` if the file has no colour for this shape |
| `fingerprint` | string    | 16 hex chars, see *Fingerprint* |

### Nodes

Nodes are listed parents-before-children; siblings keep the order they had in the
STEP file.

| field       | type        | meaning |
|-------------|-------------|---------|
| `id`        | string      | `n0`, `n1`, ... unique inside this file |
| `name`      | string      | instance name exactly as found (may be `""`) |
| `parent`    | string/null | id of the parent node; `null` for a root (a file may have several roots) |
| `shape`     | string/null | shape id for a leaf part; `null` for an assembly (a node that only groups children) |
| `transform` | 16 numbers  | 4×4 matrix, row-major, placing this node **relative to its parent** (mm). Last row is `0 0 0 1` |
| `color`     | string      | optional: colour set on this instance, overriding the shape's colour |

The world position of a node is `parent_world @ transform`. Transforms are stored
relative to the parent (as in STEP) so that moving a subassembly changes one
number, not every part inside it — diff relies on this to report
"the arm moved" instead of "the arm and its 9 screws moved".

## Fingerprint

A short hash that answers "is this the same geometry?" regardless of where the
part sits.

1. Take only measurements that do not change when the part is moved or rotated:
   `volume`, `area`, the sorted `inertia` moments, and the `faces` counts.
2. Round each number to 4 significant digits, so floating-point noise (for example
   from geometry that was moved and re-exported) does not change the result.
3. Write them as one text line and take the first 16 hex chars of its SHA-256.

Known limits, on purpose:
- Mirror images (a left and a right bracket) get the same fingerprint.
- A value sitting right on a rounding boundary can flip; diff must therefore treat
  "fingerprint differs" as a strong hint and confirm with the raw numbers.
- Very small edits (a 0.01 mm chamfer) may be lost in rounding.

## Units

All lengths are mm, whatever units the STEP file used: the importer converts on
read. `source.units` only records what the file declared.

## Versioning and unknown fields

- Readers must check `format == "kyumi"` and refuse files whose `version` is
  higher than they support, with a clear message.
- Readers must **ignore** fields they don't recognise, at every level (top level,
  shapes, nodes). This lets a future writer (e.g. a CAD plugin) add optional data
  — permanent part ids, materials, connections, labels — without breaking
  anything.
- Adding an optional field does **not** change `version`. Removing a field or
  changing what one means does.
- Writers must not depend on files inside the zip other than those listed here;
  readers ignore unknown files.

## What is deliberately not in v1

- Per-face colours (only part and instance colours are kept).
- PMI, materials, layers, permanent part ids.
- Contacts between parts (categorize wants this later; see its plan).
