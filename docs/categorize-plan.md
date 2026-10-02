# Categorize plan (draft)

Status: first draft, written before `core` exists. It will be completed in
Milestone 5 with what was learned while building `core`.

## Goal

Sort a model's parts into categories (e.g. a drone → airframe, propulsion,
electronics) for a flowchart view. Today engineers do this by hand.

## Decisions already made

- An LLM classifies the **whole assembly in one call** (context helps), using a
  "profile card" per shape: name, parent names, copy count, size, face types,
  colour, and later which parts it touches.
- Output: a category from a **fixed list** plus a confidence, as structured
  JSON, temperature 0.
- Collapse duplicates first. Simple rules handle obvious fasteners.
- Low-confidence parts escalate to a rendered image plus a vision model.
- The engineer confirms; corrections become examples for future runs.
- On a new version, carry labels over from matched parts (via diff) and classify
  only new parts.
- Evaluation comes first: hand-labelled assemblies plus "renamed" copies
  (every part → Part1, Part2, ...) to measure how much accuracy depends on names.
- Open question for the startup: what the categories are (possibly "which team
  owns this part").

## Approach (draft)

1. `load()` the package; build one profile card per **shape** (not per node).
2. Rules first: e.g. small, cylinder-dominated, many copies, name contains
   "screw/bolt/M3" → fastener. Rules only fire when confident.
3. One LLM call with all remaining cards + the tree outline + the category list;
   JSON schema output `{shape_id: {category, confidence}}`.
4. Cards under a confidence threshold → render the shape mesh to a PNG →
   vision model.
5. Store labels per shape in a sidecar file (not inside the `.kyumi` for now).

## Test strategy (draft)

Hand-label the drone fixture (and later real models); generate a renamed copy;
report accuracy with and without names. Run in CI with a recorded LLM response
so tests are deterministic and free.

## What I need from core

- `load(path) -> Model`.
- Per shape: `name` **exactly as in the file** (junk names are real data), `size`,
  `faces` counts, `volume`, `area`, `color`, `solids`.
- Per shape: **copy count** (how many nodes use it) — a helper on `Model`.
- Per node: parent and the list of parent names (tree path), and per shape the
  set of nodes using it.
- Per shape: a mesh in a standard format (`meshes/<shape>.glb`) that can be
  rendered to a PNG without CAD libraries, for the vision step.
- Units fixed to mm so size thresholds in rules mean the same thing in every file.
- Readers that ignore unknown fields, so labels or materials written by a future
  CAD plugin don't break anything.
- Later (not v1): which parts touch which ("contacts"). Left out of v1; the plan
  will say how to compute it from the exact solids if it proves valuable.
