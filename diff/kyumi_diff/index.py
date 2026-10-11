"""Dict-based lookups over a Model: ids, children, keys and world positions.

`Model.node()` is a linear search, fine for 36 nodes and hopeless for 5,000, so
everything the matcher needs is computed once here.
"""

from __future__ import annotations

import numpy as np
from kyumi import Model, Node

Key = list[str]


KEY_FORMAT = "names from the root down; the k-th same-named sibling (k >= 1) is 'Name#k'"


def with_index(name: str, earlier_same_name: int) -> str:
    return f"{name}#{earlier_same_name}" if earlier_same_name else name


def key_of(model: Model, node_id: str) -> Key:
    """Like Model.path_of, but same-named siblings are told apart: "Name", "Name#1", ...

    Ids are stable within one file only and paths are not unique (CPT17 has eight
    `LegUnit > Clamp` siblings), so this is how a node is named in diff.json and
    in the generated expected answers.
    """
    key: Key = []
    current: str | None = node_id
    while current is not None:
        node = model.node(current)
        siblings = model.children(node.parent)
        position = [s.id for s in siblings].index(node.id)
        earlier = sum(1 for s in siblings[:position] if s.name == node.name)
        key.append(with_index(node.name, earlier))
        current = node.parent
    return key[::-1]


class Index:
    def __init__(self, model: Model) -> None:
        self.model = model
        self.by_id: dict[str, Node] = {n.id: n for n in model.nodes}
        self.children: dict[str | None, list[Node]] = {}
        for node in model.nodes:
            self.children.setdefault(node.parent, []).append(node)
        self.depth: dict[str, int] = {}
        self.key: dict[str, Key] = {}
        self.world: dict[str, np.ndarray] = {}
        for node in self.ordered():  # parents before children
            parent = self.by_id.get(node.parent) if node.parent else None
            self.depth[node.id] = 0 if parent is None else self.depth[parent.id] + 1
            siblings = self.children[node.parent]
            earlier = sum(1 for s in siblings[: siblings.index(node)] if s.name == node.name)
            parent_key = [] if parent is None else self.key[parent.id]
            self.key[node.id] = [*parent_key, with_index(node.name, earlier)]
            local = np.array(node.transform).reshape(4, 4)
            self.world[node.id] = local if parent is None else self.world[parent.id] @ local
        self.leaves = [n for n in model.nodes if n.shape is not None]
        self.groups = [n for n in model.nodes if n.shape is None]
        self.center: dict[str, tuple[float, float, float]] = {
            n.id: tuple((self.world[n.id] @ np.array([*model.shapes[n.shape].center, 1.0]))[:3])
            for n in self.leaves
        }
        self.parent_names: dict[str, tuple[str, ...]] = {
            n.id: tuple(name.split("#")[0] for name in self.key[n.id][:-1]) for n in model.nodes
        }

    def ordered(self) -> list[Node]:
        """Every node, each parent before its children, whatever the file order."""
        out: list[Node] = []
        stack = list(reversed(self.children.get(None, [])))
        while stack:
            node = stack.pop()
            out.append(node)
            stack.extend(reversed(self.children.get(node.id, [])))
        return out

    def ancestors(self, node_id: str) -> list[str]:
        found: list[str] = []
        current = self.by_id[node_id].parent
        while current is not None:
            found.append(current)
            current = self.by_id[current].parent
        return found
