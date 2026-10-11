"""Diff two .kyumi packages. Uses only `load()` and the Model API — never the zip."""

from kyumi_diff.classify import TAGS, Change, Diff, diff
from kyumi_diff.detail import Detail, detail, parse_key, render_detail, write_detail
from kyumi_diff.index import KEY_FORMAT, key_of
from kyumi_diff.report import agrees, render_text, to_json

__all__ = [
    "KEY_FORMAT",
    "TAGS",
    "Change",
    "Detail",
    "Diff",
    "agrees",
    "detail",
    "diff",
    "key_of",
    "parse_key",
    "render_detail",
    "render_text",
    "to_json",
    "write_detail",
]
