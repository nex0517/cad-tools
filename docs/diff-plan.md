# Diff plan

Status: complete plan, written after building `core` (Milestones 1–4). It is
meant to be handed to a separate session as its own project.

## Goal

Given two versions of a model (`old.kyumi`, `new.kyumi`), report what changed so
an engineer can review it like a code diff:

- **Assembly level:** nodes added, removed, moved, renamed, re-parented.
- **Shape level:** a part whose geometry changed (fingerprint differs).
- **Detail level:** for one modified part, show the material that was added
  (green) and removed (red).

## Decisions already made

- STEP text cannot be diffed: entity numbers like `#120` are reshuffled on every
  export. We diff the `.kyumi` manifests instead.
- Matching parts across versions is the core problem. Combine name,
  fingerprint and position.
- Test data: v1/v2 pairs generated from the drone fixture with known, deliberate
  changes so results are checked automatically.

## What core gives us (and what it taught us)

Everything below is in `core` today and the plan relies on it:

- `load(path) -> Model`; `Model.world_transform(id)`, `path_of(id)`,
  `copies(shape_id)`, `color_of(node)`, `read("breps/s1.brep")`.
- Shape and node ids (`s1`, `n7`) are stable **within** a file only. They are
  assigned in read order, so an export that reorders parts renumbers them.
  Never match on ids.
- The fingerprint survives moving and rotating: `drone_messy.step` has every
  copy baked into a different rotation and all four motors still share one
  fingerprint (test: `test_messy_file`). It changes on a resize
  (`test_fingerprint.py`).
- Names are *two* things: the **shape** name (`Motor`) and the **node** name
  (`Motor_FL`). Real files can have either one junk or blank: in
  `drone_messy.step` the blank part comes back as shape `""`, node `"95"`
  (OpenCascade names an unnamed placement after its STEP entity id). Matching
  must cope with both names being useless.
- Transforms are relative to the parent. Moving the `Frame` group moves 13
  parts in world space but changes exactly one node's `transform`.
- A `.kyumi` for the 31-part drone imports in ~90 ms, so diff can afford to
  re-measure or re-mesh a handful of parts, but not thousands.

## Approach

### 1. Match nodes (the core problem)

Work on **leaf nodes** (nodes with a shape) first; groups are matched afterwards
from their children.

**Narrow candidates** so this stays fast on 5,000-part assemblies (comparing
every pair would be 25 million comparisons). Put every old leaf in three
dictionaries: by node name, by fingerprint, and by a coarse spatial grid cell
(world position of the shape's `center`, cell size = 5 % of the model's
bounding-box diagonal, including the 26 neighbouring cells). A new leaf's
candidates are the union of the three lookups. Typical candidate count: 1–10.

**Score** each candidate pair, 0–1 each, summed with weights:

| signal | weight | note |
|---|---|---|
| same fingerprint | 0.40 | strong, but four motors share one |
| same node name | 0.25 | names can be junk (`Part37`) or blank |
| same shape name | 0.10 | separate from the node name |
| world `center` distance | 0.15 | `exp(-d / tolerance)`; breaks ties between copies |
| same parent path | 0.10 | context |

**Assign** greedily: sort all candidate pairs by score, take the best, remove
both nodes, repeat. Reject pairs under 0.3. Greedy is simple and visibly
correct in the viewer; switch to the Hungarian algorithm only if the swap
tests below fail.

Groups: a group matches the old group that contains most of its matched
children (majority vote), tie broken by name.

### 2. Classify each matched pair

- **renamed**: node name differs.
- **moved**: world transform differs. Compare positions with a 0.01 mm
  tolerance and rotations with 0.01°, *after* checking whether the parent moved
  — a part whose own `transform` is unchanged is not "moved", only its group is.
- **re-parented**: parent path differs.
- **modified**: fingerprint differs. Confirm with raw measurements so a value
  that merely crossed a rounding boundary (volume 1234.4 → 1234.6) is reported
  as "unchanged (rounding)" instead: modified = any of volume, area, inertia
  differs by more than 0.1 % or a face count differs.
- A pair can carry several tags (moved + renamed).

Unmatched old leaves → **removed**; unmatched new leaves → **added**. If an
added and a removed node share a fingerprint, report it as **moved far**
(moved beyond the candidate grid) rather than as two changes.

### 3. Detail diff (one pair, on request)

1. `model.read(shape.brep)` for both, `BRepTools.Read` into `TopoDS_Shape`.
2. Place both with their world transforms (`BRepBuilderAPI_Transform`).
3. `BRepAlgoAPI_Cut(new, old)` = added material; `Cut(old, new)` = removed.
4. Mesh both with `core`'s `mesh_glb`, write them next to the diff as
   `added.glb` / `removed.glb`.
5. Booleans can fail on near-coincident faces; wrap with a fuzzy value
   (`SetFuzzyValue(0.001)`) and report "could not compute" rather than crash.

### 4. Output

`diff.json`:

```json
{
  "old": "drone_v1.kyumi", "new": "drone_v2.kyumi",
  "pairs":   [{"old": "n7", "new": "n9", "score": 0.93, "tags": ["moved"], "delta": {"distance": 5.0}}],
  "added":   ["n31"],
  "removed": ["n12"],
  "summary": {"added": 1, "removed": 1, "moved": 1, "renamed": 0, "modified": 0}
}
```

A `kyumi diff old.kyumi new.kyumi [-o diff.json]` command prints the summary
and one line per change (`moved  Drone › Propulsion › Motor_FL  by 5.0 mm`).
The viewer gets a diff mode: load both packages, colour added parts green,
removed red (ghosted), moved blue, modified orange; the tree shows tags.

## Milestones

1. **Test data generator** (`fixtures/make_diff_pairs.py`): derive v2 drones
   from the drone fixture with one known change each: rename `Motor_FL`; move
   one motor 5 mm; delete `Screw_3`; add a chip; resize the battery; drill a
   hole in `CenterPlate`; swap `Prop_FL` and `Prop_FR` (identical copies);
   rotate the whole `Frame` group; the same v1 exported as `drone_messy`
   (copies instead of instances). Also a `drone_renamed` v2 where every node
   is `Part1..PartN`. Each pair comes with its expected `diff.json`.
   (~1 session, including the fixture-overlap cleanup noted in #2.)
2. **Matching + classification** (`diff/match.py`, `diff/classify.py`) and
   `kyumi diff`. All pairs from step 1 produce their expected output.
   (~1 session.)
3. **Detail diff** for one pair, including the fuzzy-boolean fallback.
   (~0.5 session.)
4. **Viewer diff mode.** (~0.5 session.)

Rough total: 3 sessions of work, plus review turnarounds.

## Test strategy

- Every generated pair asserts the exact `diff.json` (ids differ between
  files, so assert on paths, not ids).
- The `drone_renamed` pair must still match every part correctly using
  fingerprint + position alone; this is the "how much do we depend on names"
  test and should be reported as a number, not just pass/fail.
- Swapping two identical props must report *no change* (or two moves of 0 mm,
  which we collapse to none).
- Rotating the `Frame` group must report **one** move, not thirteen.
- Detail diff: the drilled-hole pair must produce an empty `added` and a
  `removed` solid whose volume equals the hole's (π r² h within 1 %).
- Performance: a synthetic 5,000-leaf assembly (the drone tiled 160 times)
  matches in under 5 s.

## Risks

- **Identical copies near each other** (8 screws): position ties can be broken
  by the parent path, but if an export rearranges the tree the match may be
  arbitrary. Acceptable: swapping two identical screws is not a real change.
- **Mirror parts** share a fingerprint (left/right arm). Moments of inertia
  cannot tell them apart; position will. A future fingerprint could add a
  chirality bit.
- **Rounding boundaries**: a 4-significant-digit fingerprint flips on tiny
  edits of a value near a boundary. The raw-measurement confirmation in step 2
  handles this; the plan does not change the fingerprint.
- **Boolean robustness** on real-world geometry is the least predictable part;
  budget time for failures and always have the "could not compute" path.
- **Large assemblies**: `Model.node()` is a linear search; build dict indexes
  once in diff rather than changing core.

## Open questions for the startup

- Tolerances: is 0.01 mm / 0.01° the right definition of "moved", or should it
  depend on part size?
- Should a part that is both moved and modified be shown as one change or two?
- When a subassembly is replaced wholesale (new file, same parts), do engineers
  want "group modified" or the full list of leaf changes?
- Is comparing two *different* models ("how much of v2 is reused from v1")
  in scope? The matcher could do it; the output would need a different UI.

## What I need from core

All available today in `core`:

- `load(path) -> Model`; `Model.world_transform`, `path_of`, `copies`, `read`.
- Per node: parent-relative `transform`, `parent`, `name`.
- Per shape: `fingerprint` plus the raw `volume`, `area`, `inertia`, `faces`
  behind it; `center`; `name`; `brep` path.
- `mesh_glb(shape, size)` reusable for the detail-diff results.

Small additions that would help (not blocking):

- A dict-based index (`Model.by_id`) if 5,000-node assemblies make `node()`
  slow; measure first.
- A `Model.leaves()` helper (nodes with a shape).
