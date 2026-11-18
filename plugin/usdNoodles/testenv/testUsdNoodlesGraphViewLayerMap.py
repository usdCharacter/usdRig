#!/usr/bin/env python3
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Guard for the GraphView decomposition (issue #54, Phase 1).

Every method defined on ``GraphView`` in ``plugin/usdNoodles/graphView.py``
must carry a layer assignment in
``plugin/usdNoodles/graphview/layer_map.json``, and
``graphview/LAYER_MAP.md`` must assign the same methods to the same layers.
Adding a new method to ``GraphView`` without assigning it to a layer fails
this test; deleting a method without removing its map entry also fails,
keeping both views of the map authoritative through the Phase 2 extractions.

The guard reads ``graphView.py`` as source text through the AST, so it
needs neither Qt, GL, nor pxr and runs in any plain interpreter.
"""

from __future__ import annotations

import ast
import json
import pathlib
import re
import unittest

VALID_LAYERS = frozenset(
    {"model", "intents", "editing", "interaction", "render",
     "selection", "stage_sync", "core"}
)

# testenv sits directly inside the UsdNoodles package in the source tree, in
# the staged build/python/UsdNoodles package, and in the installed package, so
# the guarded files are always one directory above this file.
_PACKAGE_DIR = pathlib.Path(__file__).resolve().parent.parent
_SOURCE = _PACKAGE_DIR / "graphView.py"
_MAP_PATH = _PACKAGE_DIR / "graphview" / "layer_map.json"
_MAP_MARKDOWN = _PACKAGE_DIR / "graphview" / "LAYER_MAP.md"
_SECTION_RE = re.compile(r"^## (\w+) \(\d+ methods?\)$")
_ENTRY_RE = re.compile(r"^- `(\w+)` \(\d+ LOC\)$")


def _graphview_method_names(source_text):
    """Direct method names defined on the GraphView class."""
    tree = ast.parse(source_text)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "GraphView":
            return {
                n.name
                for n in node.body
                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
    return set()


def _markdown_layers(markdown_text):
    """{method: layer} read from the generated LAYER_MAP.md body."""
    assignment = {}
    layer = None
    for line in markdown_text.splitlines():
        section = _SECTION_RE.match(line)
        if section:
            layer = section.group(1)
            continue
        entry = _ENTRY_RE.match(line)
        if layer is not None and entry:
            assignment[entry.group(1)] = layer
    return assignment


class TestGraphViewLayerMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # No skip for the pair the guard is named for: a guard that skips when
        # its own inputs are missing passes vacuously, and every UsdNoodles
        # package layout stages graphView.py and layer_map.json beside testenv.
        absent = [str(path) for path in (_SOURCE, _MAP_PATH)
                  if not path.is_file()]
        if absent:
            raise AssertionError(
                "layer-map guard found no " + ", ".join(absent))
        cls.source_text = _SOURCE.read_text()
        cls.methods = _graphview_method_names(cls.source_text)
        cls.layer_map = json.loads(_MAP_PATH.read_text())["methods"]
        cls.markdown_map = (_markdown_layers(_MAP_MARKDOWN.read_text())
                            if _MAP_MARKDOWN.is_file() else None)

    def test_every_method_has_a_layer_assignment(self):
        """A method added to GraphView without a layer entry fails here."""
        missing = sorted(self.methods - set(self.layer_map))
        if missing:
            self.fail(
                "GraphView methods without a layer assignment in "
                f"graphview/layer_map.json: {missing}. Assign each new "
                "method to model/intents/editing/interaction/render/"
                "selection/stage_sync/core before adding it."
            )

    def test_layer_map_has_no_stale_entries(self):
        """Methods removed from GraphView must be dropped from the map."""
        stale = sorted(set(self.layer_map) - self.methods)
        if stale:
            self.fail(f"layer_map.json assigns layers to methods that no "
                      f"longer exist on GraphView: {stale}")

    @unittest.skipUnless(_MAP_MARKDOWN.is_file(),
                         "LAYER_MAP.md is documentation: the staged and "
                         "installed UsdNoodles packages hold only .py/.json")
    def test_layer_map_markdown_matches_the_json(self):
        """The generated human-readable view lists the same assignment."""
        if self.markdown_map != self.layer_map:
            missing = sorted(set(self.layer_map) - set(self.markdown_map))
            extra = sorted(set(self.markdown_map) - set(self.layer_map))
            moved = sorted(
                name
                for name in set(self.layer_map) & set(self.markdown_map)
                if self.layer_map[name] != self.markdown_map[name]
            )
            self.fail(
                "graphview/LAYER_MAP.md disagrees with layer_map.json "
                f"(missing={missing}, extra={extra}, moved={moved}); "
                "regenerate the markdown from the map"
            )

    def test_layer_values_are_valid(self):
        invalid = {
            name: layer
            for name, layer in self.layer_map.items()
            if layer not in VALID_LAYERS
        }
        if invalid:
            self.fail(f"layer_map.json uses unknown layers: {invalid}")

    def test_guard_detects_an_unassigned_method(self):
        """Self-test: the completeness check trips on a new method.

        Runs the completeness check itself on an isolated instance holding a
        synthetic GraphView method set, so removing the check's failure
        branch, or an extractor that stops seeing methods, fails here.
        """
        synthetic_source = (
            "class GraphView:\n"
            "    def __init__(self): pass\n"
            "    def brand_new_method(self): pass\n"
        )
        probe = TestGraphViewLayerMap(
            "test_every_method_has_a_layer_assignment")
        # Instance state shadows the class state setUpClass installed, so the
        # probe runs the check against the injected method set only.
        probe.methods = _graphview_method_names(synthetic_source)
        probe.layer_map = dict(self.layer_map)
        with self.assertRaises(AssertionError) as caught:
            probe.test_every_method_has_a_layer_assignment()
        self.assertIn("brand_new_method", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
