# Diff plan (draft)

Status: first draft, written before `core` exists. It will be completed in
Milestone 5 with what was learned while building `core`.

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

## Approach (draft)

1. `load()` both packages.
2. **Match nodes.** First narrow the candidates, then score. Comparing every
   old node with every new one is 25 million pairs for a 5,000-part assembly,
   so a pair is only considered when the two nodes share a name, share a
   fingerprint, or sit within a small distance of each other in world
   coordinates (bucketed on a coarse grid). Score each candidate pair on:
   - same name (strong, but names can change or be junk),
   - same fingerprint (strong, but four motors share one),
   - close world position (breaks ties between identical copies),
   - same parent / same path (context).
   Assign pairs greedily from the best score down, or with the Hungarian
   algorithm if greedy proves fragile. Unmatched old nodes = removed,
   unmatched new nodes = added.
3. **Classify each matched pair:** renamed (name differs), moved (local
   transform differs beyond a tolerance), re-parented, modified (fingerprint
   differs, confirmed with raw measurements).
4. **Detail diff** on request for one modified pair: load both exact solids,
   align them by their world transforms, `new − old` = added material,
   `old − new` = removed material, mesh both results.
5. Output a `diff.json` plus a viewer mode that colours the tree and the 3D view.

## Test strategy (draft)

A script derives v2 files from the drone fixture with one known change each:
rename a part, move a motor, delete a screw, add a chip, resize the battery,
drill a hole in the frame, swap two identical props. Each test asserts the exact
expected diff.

## Open questions (draft)

- What is the tolerance for "moved"? (Probably 0.01 mm / 0.01°.)
- How should we report a part that is both moved and modified?

## What I need from core

- `load(path) -> Model` with shapes and nodes as in `docs/format.md`.
- Per node: local `transform`, and a helper for the **world** transform
  (position matching must compare world positions).
- Per node: its parent and its path (list of names from the root).
- Per shape: a **position-independent fingerprint** and the raw measurements
  behind it (`volume`, `area`, `inertia`, `faces`) to confirm near-ties.
- Per shape: `center` (centre of mass, local) so a node's world position is a
  real point on the part, not just its origin, which CAD tools place arbitrarily.
- Per shape: the **exact solid** (`breps/<shape>.brep`) for the boolean
  detail diff — meshes are not good enough for reliable subtraction. And a
  helper to load it as an OpenCascade shape.
- Transforms stored relative to the parent, so a moved subassembly shows up as
  one change.
- Stable shape and node ids within one file (they are not expected to match
  across files).
