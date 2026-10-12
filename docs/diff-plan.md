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
a 190 m fleet's diagonal would put a whole drone in one cell), and a name or
a fingerprint shared by more than 64 copies (162 drones with the same part
names, 1,280 identical screws) is no longer used as a lookup on its own — those
copies are only found through the grid. That cap took the 5,000-part test from
850 k scored pairs (4.6–4.9 s) to 0.4 s including `load()`.

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

Unmatched old leaves → **removed**; unmatched new leaves → **added**. There
is no separate "moved far" tag: a part moved beyond the grid is still a
candidate through its fingerprint, and a same-fingerprint pair scores at least
0.40 (> the 0.30 cut-off) however far apart, so it matches and is reported as
**moved** with its distance. The one case this drops: a part moved far while
more than 64 copies of its name *and* its fingerprint exist (the grid is then
the only lookup), which comes out as added + removed. Not worth a second pass
until a real file shows it.

### 3. Detail diff (one pair, on request)

`kyumi diff old.kyumi new.kyumi --detail Drone/Frame/CenterPlate [--out-dir DIR]`
(`›` works too; the key comes from `diff.json`). It writes `added.glb`,
`removed.glb` and `detail.json` (volumes, status, the key and both ids).

1. `model.read(shape.brep)` for both, `BRepTools.Read` into `TopoDS_Shape`.
   A package imported with `--no-breps` gives a clear error saying to
   re-import without it. A key that was added, removed, or names a group is an
   error too (exit 1): detail compares one part present in both versions.
2. Compare in the **part's own frame**: breps are stored untransformed, so use
   them as-is. Placement is already reported by the "moved" tag; applying the
   world transforms here would turn a 5 mm move into fake added/removed
   material and hide the real edit. A part that only moved gives nothing on
   either side.
3. `BRepAlgoAPI_Cut(new, old)` = added material; `Cut(old, new)` = removed,
   both with `SetFuzzyValue(0.001)`.
4. **Alignment, only when needed.** A real edit moves the centre of mass too
   (the hole pair's centre shifts), so aligning by centre first would create
   fake slivers. Compare as-is first; only if the result is implausible
   (added + removed > 2·|ΔV| + 0.1 % of the part's volume) **and** large (more
   than 10 % of the part — a brep re-exported about a new origin "adds" and
   "removes" a good part of the shape, while a hole that moved across a
   symmetric plate is implausible too but small, and aligning would hide it)
   try moving the old brep onto the new one: centre of mass alone, then centre + principal
   axes. Axes are ambiguous (sign flips always; order too when two moments are
   within 1 %), so every proper rotation among those is tried and the first
   plausible one, else the smallest total, is kept; `detail.json` says
   `"aligned": true`.
5. Both results stay in the **new part's own frame**, exactly like
   `meshes/*.glb`, and `detail.json` carries `new_id`: the viewer (step 4)
   places them with that node's world transform, the same way as every other
   mesh. (The earlier idea, baking in the new world transform, would make these
   two files behave differently from every other mesh.) Empty results write no
   `.glb`; the file entry is `null`.
6. Booleans fail on near-coincident faces; a failed one gives status
   `"could not compute"` (exit 0), never a crash of the whole diff. A version
   with no solid at all (a surface model) is `"not a solid"`. "Empty" means
   the result has no solid, not a volume threshold. Every real
   shape is also compared against *itself* in `core/real_files.py` — the case
   where every face coincides — and the empty / non-empty / failed counts go
   into `docs/real-files.md`.

Timings on CPT17 (`docs/real-files.md`): the modify pair's `--detail` on the
largest part takes 3.0 s, but a self-comparison of every shape takes 255 s for
316 shapes and the slowest, `Blade2 (Cfrp or equivalent)`, 26.7 s on its own.
Largest by volume is not slowest: detail time depends on surface complexity
(curved blade faces), not size. For step 4: the viewer must not compute a
detail on click; details are computed ahead (`kyumi diff --detail`, or a
batch for every `modified` part) and the viewer only loads the files.

Breps decision (CPT17, 316 shapes: import 47.6 s with breps, 32.5 s without,
+15 s / +46 %; 12.9 MB with, 9.8 MB without, +3.1 MB): **(a) keep breps on by
default, `--no-breps` to skip**, with the clear error above when a detail diff
needs them. Rejected: (b) off by default — then every first `--detail` costs a
re-import; (c) building breps on demand from `model.step` inside the package —
that re-reads the whole STEP, ~32 s per package on CPT17, so ~65 s per
`--detail` request, for one part. Our importer is test scaffolding (Kyumi has
its own loader), so 15 s more on import does not matter.

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
fingerprints may not be comparable. `--detail KEY` adds the detail diff of one
part (section 3): `detail  Drone › Frame › CenterPlate: added 0.0 mm³, removed 37.7 mm³`
plus the files it wrote.
#### Viewer diff mode

`kyumi view OLD.kyumi NEW.kyumi [--detail DIR]` loads both packages, runs the
matcher itself (under a second even at 5,000 parts) and shows the **new** model
coloured by change: added green, removed red at 40 % opacity (ghosts: the old
model's meshes at their old world transforms), moved / re-parented blue with a
thin line from the old centre to the new one, modified orange, renamed in its
own colour with a tag in the tree; unchanged parts are grey until "show
unchanged" is ticked. The selection glow is white, outside those colours. The
page does no deciding: the server works every node's status out in Python
(`diff/kyumi_diff/view.py`, Model API only; `core/kyumi/viewer.py` never
imports it) and the page only paints `/changes.json`:

```json
{
  "old": "v1.kyumi", "new": "move.kyumi",
  "summary": {"added": 0, "removed": 0, "moved": 1, "renamed": 0, "modified": 0, "re-parented": 0},
  "status": {"n18": {"class": "moved", "tags": ["moved"], "delta": {"distance": 5.0},
                     "inherited": false, "old_key": ["Drone", "Propulsion", "Motor_FR"],
                     "new_key": ["Drone", "Propulsion", "Motor_FR"]},
             "n2":  {"class": "moved", "tags": [], "delta": {}, "inherited": true, "...": "..."}},
  "ghosts": [{"old_id": "n9", "key": ["Drone", "Frame", "Screw_3"], "name": "Screw_3",
              "mesh": "/old/meshes/s3.glb", "world": [16 numbers, column-major], "volume": 1.2, "size": [3]}],
  "moves": [{"new_id": "n18", "old_id": "n18", "from": [x, y, z], "to": [x, y, z], "distance": 5.0}]
}
```

`status` is keyed by **new** ids; a part with no move tag of its own inherits
`moved` / `re-parented` from its nearest tagged parent group (`inherited:
true`), so rotating `Frame` colours all thirteen parts; priority is modified >
moved / re-parented > renamed, groups also carry `children_class`, the
strongest class inside them. `ghosts` carry **old** ids and are the only old
things served (`/old/meshes/<id>.glb`, removed parts' shapes only): old and new
ids collide, so the page keeps them in separate maps (`old:<id>`). A moved
group's centre is the mean of its parts' centres. Modified parts also get
`detail`: `{"state": "loaded", "added_mm3", "removed_mm3", "volume_change_mm3",
"aligned", "files": {"added": false, "removed": true}}`, or `state` = the
command to run, or the entry's own status ("could not compute" / "not a solid").

Exact detail is never computed on click (one CPT17 shape takes 27 s, section
3). `kyumi diff OLD NEW --detail-all [--out-dir DIR]` runs `detail()` for every
`modified` part ahead and writes `DIR/details.json` plus
`<new_id>.added.glb` / `<new_id>.removed.glb` (an empty side is `null` and has
no file; the previous run's `details.json` and `*.glb` in DIR are deleted first,
so an interrupted run leaves no index rather than a stale one):

```json
{"old": "v1.kyumi", "new": "hole.kyumi",
 "parts": [{"key": ["Drone", "Frame", "CenterPlate"], "old_id": "n2", "new_id": "n2",
            "status": "ok", "added_mm3": 0.0, "removed_mm3": 37.699, "volume_change_mm3": -37.699,
            "aligned": false, "old_fingerprint": "6f1e…", "new_fingerprint": "a93c…",
            "frame": "the new part's own coordinates; ...",
            "files": {"added": null, "removed": "n2.removed.glb"}, "seconds": 0.02}]}
```

At start-up `kyumi view --detail DIR` checks every entry against its own diff
(a `modified` change with the same key and ids, both shapes' fingerprints equal
to the entry's `old_fingerprint` / `new_fingerprint`, `volume_change_mm3` = new
minus old shape volume within 0.001, files exactly `<new_id>.<side>.glb` and
present) and refuses to start otherwise ("DIR was made from other packages").
Key, ids and volume alone cannot tell a hole that was moved from the hole that
was drilled; the fingerprints can. A `modified` part the folder does not list
is shown as "not computed" with the command to run, not refused: every entry
that is present has been verified, so nothing wrong is ever shown. The
server builds file names itself and serves `/detail/<new_id>/<side>.glb` from
memory, only for verified non-null sides. Everything else is 404 and the Host
check covers every route. Clicking a modified part with detail loaded makes it
25 % transparent and places the green / red meshes with the node's world
transform, exactly like `meshes/*.glb`. Kyumi has its own viewer; these two
json shapes are what it would reuse.

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
   no-change / move / delete / rename / modify pairs with expected answers into
   `fixtures/real/pairs/x/` (modify = a small hole through the largest editable
   part whose geometry is placed only once — drilling a shared definition would
   drill every copy — so `--detail` has a known volume to hit on real files).
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
   (~0.5 session.) **Built:** `diff/kyumi_diff/detail.py`, `kyumi diff --detail`;
   hole = 37.7 mm³ removed and nothing added, resize = +10,422.7 mm³ added and
   nothing removed, moved/rotated parts give nothing; a re-exported brep is
   aligned first. Not done until the CPT17 modify pair and the self-comparison
   of all 316 shapes pass on a real machine (`core/real_files.py`).
4. **Viewer diff mode.** (~0.5 session.) **Built:** `kyumi view OLD NEW
   [--detail DIR]` and `kyumi diff --detail-all` (section 4, "Viewer diff
   mode"). Loads precomputed detail files only; never runs a detail diff on
   click (a single shape can take 27 s, section 3). Not done until the CPT17
   pairs look right on a real machine (`docs/real-files.md`, Viewer table).

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
per leg with *different* sizes. Import 47.6 s on Windows with breps, 32.5 s
without (12.9 MB / 9.8 MB). Decided in step 3 (section 3): breps stay on by
default; `--no-breps` is for runs that will never need `--detail`. Detail diff
of the largest part: 3.0 s, volume within 1 %; every shape against itself: all
316 empty, 255 s in total, slowest `Blade2 (Cfrp or equivalent)` at 26.7 s.

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
