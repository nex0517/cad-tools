# cad-tools

Trial tools for kyumi ("GitHub for engineers"). See `core/README.md` to get started.

## Decisions

- Kyumi already has its own loader and viewer. `core/` is test scaffolding only:
  it exists so `diff/` and `categorize/` have something to read while they are
  built, and is not meant to ship.
- `diff/` and `categorize/` use only `Model`'s public API (`load()`, the
  `Model`/`Shape`/`Node` dataclasses and `Model`'s helpers), never the `.kyumi`
  zip contents, so they can later be pointed at kyumi's own loader.
- Among parts with the same fingerprint, **position beats name** when matching
  versions (see `docs/diff-plan.md`).
- Diff is not done until it has run on the real v1/v2 pairs in
  `fixtures/real/pairs/` (not committed; `python core/real_files.py` reports on them).
