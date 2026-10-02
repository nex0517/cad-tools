# Categorize plan

Status: complete plan, written after building `core` (Milestones 1–4). It is
meant to be handed to a separate session as its own project.

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

## What core gives us (and what it taught us)

- `load(path) -> Model`; per shape `name`, `size`, `volume`, `area`, `faces`,
  `solids`, `color`, `mesh`; per node `name`, `parent`, `path_of`, `copies`.
- **Two kinds of names.** A shape has its own name (`Motor`, `Chip`, or junk
  like `Part37`); each node has an instance name (`Motor_FL`, `Chip_Baro`,
  `Part38`). Both go on the card; either may be junk or blank. In
  `drone_messy.step` the blank part arrives as shape `""`, node `"95"`.
- **Instancing is not guaranteed.** `drone.step` has 11 shapes for 31 parts;
  `drone_messy.step` has 31 shapes for the same 31 parts (copies baked into
  place). "Collapse duplicates" must therefore group by **fingerprint**, not
  by shape id, or the messy file gets 31 cards instead of 11.
- **Colour is often missing** (`None`), and when present it is the sRGB hex the
  CAD user picked, which is sometimes meaningful (orange props) and sometimes
  default grey. Treat it as a weak hint.
- Face-type counts are a good cheap signal: screws and standoffs are
  cylinder-heavy, boards and plates are plane-only, the battery is a plain box.
- Everything is in mm, so size rules ("under 10 mm, more cylinders than planes,
  ≥ 4 copies → fastener") mean the same thing in every file.
- The viewer already renders each shape from its GLB with plain three.js; the
  vision step can reuse the same page headless (Playwright) to make PNGs,
  no CAD libraries needed.

## Approach

### 1. Profile cards

One card per **fingerprint group** (so the messy file and the clean file give
the same cards). The fingerprint cannot tell a part from its mirror image, so a
group is split further by **shape name** when its members have different
non-empty names (a left and a right bracket are usually named differently and
may have different roles); if the names are blank or junk, mirror parts share
one card and one label, which the mirror risk below accepts for v1.

```json
{
  "id": "g3",
  "shape_names": ["Motor"],              // distinct shape names in the group
  "instance_names": ["Motor_FL", "Motor_FR", "Motor_BL", "Motor_BR"],
  "parents": ["Drone/Propulsion"],       // distinct parent paths
  "copies": 4,
  "size_mm": [28.0, 28.0, 15.0],
  "volume_mm3": 6280.5,
  "faces": {"plane": 4, "cylinder": 6},
  "color": "#b0b0b0",
  "siblings": ["Prop_FL", "Prop_FR", "..."]   // names of parts under the same parent
}
```

Plus one **tree outline** (names only, indented) so the model sees the
structure once, not repeated per card. For a 5,000-part assembly, cards are
capped at ~300 groups per call; larger models are split by top-level
subassembly and each chunk gets the full outline.

### 2. Rules before the LLM

Rules fire only when all conditions hold and then skip the LLM:

- **fastener**: longest side < 25 mm, cylinders ≥ planes, copies ≥ 4, or name
  matches `screw|bolt|nut|washer|m[2-6]\b`.
- **carried over**: fingerprint group already labelled in the previous version
  (via diff's matching) → reuse the label with the old confidence.

Everything else goes to the model.

### 3. One LLM call

System prompt: the category list with one-line definitions, the output schema,
"answer for every card id", temperature 0. User message: tree outline + cards +
up to 20 **example cards with confirmed labels** from earlier runs (the
corrections store). Output schema:

```json
{"labels": [{"id": "g3", "category": "propulsion", "confidence": 0.92, "why": "4 copies under Propulsion, motor-sized cylinder"}]}
```

Validate: every id answered, category in the list, confidence in [0, 1].
Retry once on a schema error, then mark the missing ids `unknown`.

### 4. Vision fallback

Cards with confidence < 0.6: render the shape's GLB to a 512 px PNG with
`viewer.html` driven by Playwright (one page load, one screenshot per shape),
send image + card to a vision model, same output schema. Rendered images cost
seconds, so this is capped at 20 shapes per run.

### 5. Store and confirm

`model.labels.json` next to the `.kyumi` (not inside it: the package stays an
exact record of the import, labels change without re-zipping):

```json
{"version": 1, "source": "drone.kyumi",
 "labels": {"<fingerprint>": {"category": "propulsion", "confidence": 0.92,
                              "by": "llm", "confirmed": false}}}
```

Keyed by **fingerprint**, so labels survive re-import and reordering. The
viewer gets a label column in the tree and a dropdown to correct a category;
corrections are written back with `"by": "user", "confirmed": true` and appended
to the examples store (`~/.kyumi/examples.jsonl`).

### 6. New version

Run diff (see `diff-plan.md`), copy labels for matched pairs, classify only
added or modified shapes.

## Milestones

1. **Evaluation set first.** Hand-label `drone.step` (31 parts, 11 groups) into
   the trial categories airframe / propulsion / electronics / fastener / power;
   generate `drone_renamed.step` (every name → `PartN`) and reuse
   `drone_messy.step`. `categorize/eval.py` prints accuracy per file.
   (~0.5 session.)
2. **Cards + rules + one LLM call** (`categorize/cards.py`, `rules.py`,
   `classify.py`), `kyumi categorize model.kyumi`. Tests run against a
   recorded response. (~1 session.)
3. **Labels file + viewer column + corrections.** (~0.5 session.)
4. **Vision fallback** with Playwright renders. (~0.5 session.)
5. **Carry-over** on new versions — depends on diff milestone 2. (~0.5 session.)

Rough total: 3 sessions, plus one more real-world model hand-labelled before
anyone trusts the numbers.

## Test strategy

- `eval.py` reports accuracy on three files: clean, messy, renamed. The gap
  between clean and renamed is the headline number ("how much do we lean on
  names"). Target for the drone: ≥ 90 % clean, ≥ 70 % renamed.
- Cards for `drone.step` and `drone_messy.step` must be **identical** apart
  from names (same 11 groups, same sizes) — this proves grouping by
  fingerprint works.
- Rules: every screw and standoff in the drone is caught by the fastener rule;
  nothing else is.
- LLM tests use a recorded JSON response (no network, free, deterministic);
  one opt-in live test checks the real model still returns valid JSON.
- A schema-violation response is handled (retry, then `unknown`), tested with a
  recorded bad response.

## Risks

- **Categories are undefined.** The whole thing is only as good as the list;
  milestone 1 should be done *with* the startup, not before talking to them.
- **Names carry most of the signal** in real files; the renamed test will show
  how far geometry alone gets us. Expect fasteners and boards to be fine and
  "which bracket is airframe vs electronics" to need the tree context.
- **Mirror parts** (left/right) share a fingerprint, so without distinct names
  they share a card and a sidecar key. If the evaluation set shows this
  matters, add a chirality bit to the fingerprint in `core` (sign of the
  triple product of the principal axes) — a format change, so decide early.
- **Per-shape labels vs per-copy categories** (see open questions): if the
  answer is per copy, the labels file needs a per-path override and the LLM
  needs node-level cards for parts with copies under different parents.
- **Cost/latency** on large assemblies: one call with 300 cards is ~30 k
  tokens; chunking by subassembly keeps calls bounded but loses cross-chunk
  context.
- **Vision renders** of a single shape without context (a grey cylinder) may
  not help much; try with the neighbourhood rendered faintly before deciding.

## Open questions for the startup

- What are the categories? Possibly "which team owns this part". Fixed list
  per company, per project, or per model?
- Can copies of the same part need different categories? Labels are stored
  per *shape*, so all four motors share one label. If categories mean team
  ownership, the same screw might belong to the frame team in one place and the
  electronics team in another, which would need per-node labels instead.
- Which LLM provider, and may model data leave the customer's machine at all?
  If not, the rules + vision path needs a local model and the plan changes.
- Is "unknown" an acceptable output for the flowchart, or must every part get
  a category?

## Open questions (draft)

- What are the categories? Possibly "which team owns this part".
- Can copies of the same part need different categories? Labels are stored
  per *shape*, so all four motors share one label. If categories mean team
  ownership, the same screw might belong to the frame team in one place and the
  electronics team in another, which would need per-node labels instead.

## What I need from core

All available today in `core`:

- `load(path) -> Model`; per shape `name` exactly as in the file, `size`,
  `faces`, `volume`, `area`, `color`, `solids`, `fingerprint`, `mesh`.
- `Model.copies(shape_id)`, `path_of(node_id)`, `children`, `color_of`.
- Meshes as GLB, renderable by the existing viewer page for the vision step.
- Units fixed to mm; readers ignore unknown fields.

Small additions that would help (not blocking):

- `Model.groups_by_fingerprint() -> dict[str, list[Shape]]`, since both diff and
  categorize want it. Trivial to write locally until then.
- A `kyumi render model.kyumi --shape s3 out.png` helper if the Playwright route
  proves awkward; could live in the viewer module.
- Contacts ("which parts touch") are deliberately out of v1. If wanted later:
  for each pair of nodes whose world bounding boxes overlap, run
  `BRepExtrema_DistShapeShape` on the breps and call distance < 0.01 mm a
  contact. The fixture overlaps noted in #2 (standoffs in arms, screws through
  arms) must be fixed first or they would show as false contacts.
