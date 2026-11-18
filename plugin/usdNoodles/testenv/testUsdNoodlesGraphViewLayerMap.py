#!/usr/bin/env python3
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Guard for the GraphView decomposition (issue #54, Phase 1).

Every method defined on ``GraphView`` in ``plugin/usdNoodles/graphView.py``
must carry a layer assignment in
``plugin/usdNoodles/graphview/layer_map.json``. Adding a new method to
``GraphView`` without assigning it to a layer fails this test; deleting a
method without removing its map entry also fails, keeping the map
authoritative through the Phase 2 extractions.

The guard reads ``graphView.py`` as source text through the AST, so it
needs neither Qt, GL, nor pxr and runs in any plain interpreter.
"""

from __future__ import annotations

import ast
import json
import pathlib
import unittest

VALID_LAYERS = frozenset(
    {"model", "intents", "editing", "interaction", "render",
     "selection", "stage_sync", "core"}
)


def _find_pair():
    """Locate graphView.py and layer_map.json in the source or staged tree."""
    here = pathlib.Path(__file__).resolve().parent
    candidates = [
        # Source tree: plugin/usdNoodles/testenv -> plugin/usdNoodles/
        here.parent,
        # Staged tree: build/python/UsdNoodles/testenv -> build/python/UsdNoodles/
        here.parent.parent.parent / "plugin" / "usdNoodles",
    ]
    for root in candidates:
        source = root / "graphView.py"
        layer_map = root / "graphview" / "layer_map.json"
        if source.is_file() and layer_map.is_file():
            return source, layer_map
    return None, None


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


_SOURCE, _MAP_PATH = _find_pair()


@unittest.skipUnless(_SOURCE, "graphView.py or layer_map.json not found")
class TestGraphViewLayerMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source_text = _SOURCE.read_text()
        cls.methods = _graphview_method_names(cls.source_text)
        cls.layer_map = json.loads(_MAP_PATH.read_text())["methods"]

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

    def test_layer_values_are_valid(self):
        invalid = {
            name: layer
            for name, layer in self.layer_map.items()
            if layer not in VALID_LAYERS
        }
        if invalid:
            self.fail(f"layer_map.json uses unknown layers: {invalid}")

    def test_guard_detects_an_unassigned_method(self):
        """Self-test: the check actually trips on a new unassigned method."""
        fake_source = (
            "class GraphView:\n"
            "    def __init__(self): pass\n"
            "    def brand_new_method(self): pass\n"
        )
        fake_methods = _graphview_method_names(fake_source)
        missing = fake_methods - set(self.layer_map)
        self.assertIn("brand_new_method", missing)


if __name__ == "__main__":
    unittest.main()
