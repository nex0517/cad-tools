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
  `drone_messy.step` the blank part comes back as shape `""`, node `""`
  (OpenCascade would name the placement after its STEP entity id, which
  changes on every export; the importer puts the blank back). Matching must
  cope with both names being useless.
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
dictionaries: by node name, by fingerprint, and by a spatial grid cell (world
position of the shape's `center`, cell size = the median part's bounding-box
diagonal, including the 26 neighbouring cells). A new leaf's candidates are the
union of the three lookups. Typical candidate count: 1–10. Two things keep
that true on big models: the cell is sized by the *part*, not the model (5 % of
a 190 m fleet's diagonal would put a whole drone in one cell), and a
fingerprint shared by more than 64 copies (1,280 identical screws) is no longer
used as a lookup on its own — those copies are only found through the grid.

**Rule: among parts with the same fingerprint, position beats name.** Two
identical props that trade places are the *same* props with swapped labels
(two renames), not two 155 mm moves. The weights must encode this: with the
earlier draft (node name 0.25 > distance 0.15) the swap pair scored as two
moves, and no assignment algorithm can fix that, because the scores are wrong,
not the assignment.

**Score** each candidate pair, 0–1 each, summed with weights (`tolerance` in
the distance term is the grid cell size, so "near" means "within a part"):

| signal | weight | note |
|---|---|---|
| same fingerprint | 0.40 | strong, but four motors share one |
| world `center` distance | 0.30 | `exp(-d / tolerance)`; decides between copies |
| same node name | 0.10 | names can be junk (`Part37`) or blank; must stay below distance |
| same shape name | 0.10 | separate from the node name |
| same parent path | 0.10 | context |

Check on the swap pair: old `Prop_FL` vs the new prop *at its old position*
(now named `Prop_FR`) scores 0.40 + 0.30 + 0.10 + 0.10 = 0.90; vs the new
`Prop_FL` 155 mm away it scores 0.40 + 0 + 0.10 + 0.10 + 0.10 = 0.70. Position
wins; the pair is then tagged **renamed**.

**Assign** greedily: sort all candidate pairs by score, take the best, remove
both nodes, repeat. Reject pairs under 0.3. Greedy is simple and visibly
correct in the viewer; the Hungarian algorithm is only worth adding if a pair
is mis-assigned *despite* correct scores.

Groups: a group matches the old group that contains most of its matched
children (majority vote). Names are no tie-break — CPT17 has eight identical
`LegUnit` groups with the same name — so a tie goes to the old group whose
matched children are closest in world position (smallest total `center`
distance). Because leaves were matched by position first, identical groups
normally separate themselves by which leaves they hold; the `arms` pair (four
same-named `ArmUnit` groups, one moved 5 mm) checks exactly this.

### 2. Classify each matched pair

- **renamed**: node name differs.
- **moved**: the part sits somewhere else. Compare positions with a 0.01 mm
  tolerance and rotations with 0.01°, *after* checking whether the parent moved
  — a part whose own `transform` is unchanged is not "moved", only its group is.
  For an unchanged shape compare the world position of its `center`, not the
  raw transform: a baked export (`baked` pair) has identity transforms with the
  placement inside the geometry, and nothing moved. For a *modified* shape the
  centre of mass itself shifts (`hole` pair), so compare the world transforms.
  `fixtures/make_diff_pairs.py::verify` is the executable form of these rules.
- **re-parented**: the parent *node* differs — i.e. the new parent is not the
  match of the old parent. Compare matched parent pairs, not path strings, or
  renaming one group would mark every part inside it as re-parented.
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
2. Compare in the **part's own frame**: breps are stored untransformed, so use
   them as-is (no world transform). Placement is already reported by the
   "moved" tag; applying both world transforms here would turn a 5 mm move into
   fake added/removed material and hide the real edit. If the shape's local
   origin changed between versions (part re-exported), align by centre of mass
   and sorted principal axes first (`center`, `inertia` from the manifest).
3. `BRepAlgoAPI_Cut(new, old)` = added material; `Cut(old, new)` = removed.
   For display, place both results with the **new** world transform.
4. Mesh both with `core`'s `mesh_glb`, write them next to the diff as
   `added.glb` / `removed.glb`.
5. Booleans can fail on near-coincident faces; wrap with a fuzzy value
   (`SetFuzzyValue(0.001)`) and report "could not compute" rather than crash.

### 4. Output

`diff.json`:

```json
{
  "old": "v1.step", "new": "move.step",
  "keys": "names from the root down; the k-th same-named sibling (k >= 1) is 'Name#k'",
  "changes": [{"old": ["Drone", "Propulsion", "Motor_FR"], "new": ["Drone", "Propulsion", "Motor_FR"],
               "tags": ["moved"], "delta": {"distance": 5.0},
               "old_id": "12", "new_id": "12", "score": 0.9}],
  "added":   [],
  "removed": [["Drone", "Frame", "Screw_3"]],
  "summary": {"added": 0, "removed": 1, "moved": 1, "renamed": 0, "modified": 0, "re-parented": 0}
}
```

Nodes are named by key, not id (ids are not stable across files); `old_id`,
`new_id` and the match `score` are there for tools that want to follow a change
back into the two packages. Otherwise the file has the same shape as the
generated `*.expected.json`, so tests compare
the two directly. `kyumi diff old.kyumi new.kyumi [-o diff.json]` prints the
match rate (parts of the old model that found a partner), the summary and one
line per change (`moved  Drone › Propulsion › Motor_FR  by 5.0 mm`). If the two
packages were written by different generator versions it warns that
fingerprints may not be comparable.
The viewer gets a diff mode: load both packages, colour added parts green,
removed red (ghosted), moved blue, modified orange; the tree shows tags.

## Milestones

1. **Test data generator** (`fixtures/make_diff_pairs.py`): derive v2 drones
   from the drone fixture with one known change each: rename `Motor_FL`; move
   one motor 5 mm; delete `Screw_3`; add a chip; resize the battery; drill a
   hole in `CenterPlate`; swap `Prop_FL` and `Prop_FR` (identical copies,
   expected: **2 renamed**); rotate the whole `Frame` group; the same v1
   exported as `drone_messy` (copies instead of instances). Also a
   `drone_renamed` v2 where every node is `Part1..PartN` (expected: **every
   part renamed, nothing else**). Each pair comes with its expected
   `diff.json`; check all ten against the tag definitions in "Classify" before
   step 2 starts. (~1 session, including the fixture-overlap cleanup noted
   in #2.) **Done:** `fixtures/pairs/drone/` holds `v1.step`, one
   `<change>.step` per pair and `<change>.expected.json` (keys, tags, deltas,
   summary), all checked by `verify()`; plus `arms_v1.step` / `arms.step`, four
   identical `ArmUnit` groups with one moved, and `reparent.step`, `Screw_1`
   moved from `Frame` into `Electronics` with its world position kept (its
   local transform changes; expected: `re-parented` only). The same script run on any STEP file
   (`python fixtures/make_diff_pairs.py fixtures/real/x.step`) writes
   no-change / move / delete / rename pairs with expected answers into
   `fixtures/real/pairs/x/`.
2. **Matching + classification** (`diff/match.py`, `diff/classify.py`) and
   `kyumi diff`, using only the `Model` API. All pairs from step 1 produce
   their expected output. **Not done until it has also run on the real v1/v2
   pairs in `fixtures/real/pairs/`** (via `core/real_files.py`, results in
   `docs/real-files.md`). (~1 session.) **Built:** `diff/` (`kyumi_diff`
   package: `index`, `match`, `classify`, `report`), `kyumi diff`; every
   drone pair and the generic pairs from `drone_messy` / `drone_shared`
   reproduce their expected answer; the renamed pair matches 31/31 parts;
   5,022 parts (the drone tiled 162 times) diff in ~2.5 s.
3. **Detail diff** for one pair, including the fuzzy-boolean fallback.
   (~0.5 session.)
4. **Viewer diff mode.** (~0.5 session.)

Rough total: 3 sessions of work, plus review turnarounds.

## Test strategy

- Every generated pair asserts the exact `diff.json` (everything but ids and
  scores), and the match rate: v1 parts paired with the partner the expected
  answer names ÷ v1 parts, printed as `renamed: 31/31 correct (100.0%)`. Ids
  differ between files and paths are not unique (CPT17: eight `LegUnit > Clamp (3.3535)`),
  so nodes are named by **key**: the names from the root down, where the k-th
  sibling with the same name (k ≥ 1) is written `Name#k` — `Drone / ArmUnit#2`.
  Assumptions: the export keeps sibling order (every tool seen so far does),
  and deleting or renaming one of several same-named siblings shifts the
  index of those after it. The generator therefore always edits the *last*
  same-named sibling, and the matcher never matches on the index — it is a
  label for humans and tests, not a matching rule. If an export reorders
  siblings, keys with `#k` may point at a different copy; identical copies
  make that harmless, different geometry under one name (CPT17's two
  `Blade5` sizes) would show as a spurious modified pair.
- The `drone_renamed` pair must match every part using fingerprint + position
  alone and report **every part renamed, nothing else** (no moves, no
  added/removed); the match rate is the "how much do we depend on names"
  number and is reported, not just pass/fail.
- Swapping two identical props must report **2 renamed** (position beats
  name), not two moves.
- The real pairs in `fixtures/real/pairs/` must run to completion. The
  generated ones (`make_diff_pairs.py <file>`: no-change / rename / move /
  delete) have an expected answer; hand-made pairs only get eyeballed.
- Rotating the `Frame` group must report **one** move, not thirteen.
- Detail diff: the drilled-hole pair must produce an empty `added` and a
  `removed` solid whose volume equals the hole's (π r² h within 1 %).
- Performance: a synthetic 5,000-leaf assembly (the drone tiled 160 times)
  matches in under 5 s.

## Seen in real files

CPT17 (335 nodes, 316 leaves, depth 4): every copy is its own shape (316
shapes, 20 distinct geometries — `drone_messy` in real life); eight identical
`LegUnit` groups share one name, and inside each, 8 × `Clamp (3.3535)` etc.
share name, parent and geometry; `Blade5 (Cfrp or equivalent)` appears twice
per leg with *different* sizes. Import 44.5 s on Windows, of which writing
breps is 15.4 s (`--no-breps`: 27 s). Decide in step 3 whether breps become
optional or are built only for the shapes a detail diff asks about; until
then real files are imported with `--no-breps`.

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

Everything through `load()` and the `Model` API — diff never opens the zip
itself, so it can later run on kyumi's own loader.

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
