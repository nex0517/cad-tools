# diff

`kyumi diff old.kyumi new.kyumi [-o diff.json]` — which parts were added, removed,
moved, renamed, modified or re-parented between two versions. Reads both packages
through `load()` and the `Model` API only; the plan and the rules it implements
are in [../docs/diff-plan.md](../docs/diff-plan.md).

```
pip install -e diff
kyumi diff drone_v1.kyumi drone_v2.kyumi
```

Nodes are reported by key: the names from the root down, `Name#k` for the k-th
same-named sibling. Test data: `fixtures/pairs/drone/` (every pair's
`expected.json` must come out exactly) and any `fixtures/real/pairs/<file>/`.
