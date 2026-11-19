#!/usr/bin/env python3
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct tests for the graph view's prim-path identity layer (issue #54).

``graphview/model.py`` is pure stdlib, so it is loaded by path instead of
through ``UsdNoodles``: importing the package would pull in pxr and Qt and
force these cases to skip on a machine that has neither.
"""

import importlib.util
import pathlib
import unittest

# testenv sits directly inside the UsdNoodles package in the source tree, in
# the staged build/python/UsdNoodles package, and in the installed package, so
# the module under test is always one directory above this file.
_PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
_MODEL_PATH = _PACKAGE_DIR / "graphview" / "model.py"


def _load_model_under_test():
    # No skip when the module is missing: a test that skips when its own
    # subject is absent passes vacuously.
    if not _MODEL_PATH.is_file():
        raise AssertionError(
            f"graph-view model test found no {_MODEL_PATH}")
    spec = importlib.util.spec_from_file_location(
        "_graphview_model_under_test", _MODEL_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_model = _load_model_under_test()


class NodeIdPrimPathTest(unittest.TestCase):
    """Node ids are prim paths, except when they are synthesized."""

    def _primPath(self, node_id):
        return _model.node_id_prim_path(node_id)

    def test_a_plain_node_id_is_already_a_prim_path(self):
        self.assertEqual(self._primPath("/World/Cube"), "/World/Cube")

    def test_a_synthesized_io_node_keeps_only_the_blueprint_path(self):
        self.assertEqual(
            self._primPath("/World/Bp#input:drive:x"), "/World/Bp")

    def test_the_id_is_split_at_the_first_hash_only(self):
        """A nested pin synthesizes a second ``#``; an implementation that
        splits on every ``#`` or rsplit()s keeps part of the pin."""
        self.assertEqual(
            self._primPath("/World/Bp#input:foo:bar#second"), "/World/Bp")

    def test_a_fully_synthesized_id_has_no_prim_path(self):
        self.assertEqual(self._primPath("#synth"), "")
        self.assertEqual(self._primPath(""), "")


class PathRelationTest(unittest.TestCase):
    """What counts as "this change touches that node".

    The resync loop asks it of every node when a resynced path is not itself
    a node id, and USD names both child prims and properties in that set.
    Getting it wrong is silent in both directions: too narrow and a new pin
    never appears, too wide and editing one prim invalidates the caches of
    every sibling whose name happens to start the same way.
    """

    def _related(self, a, b):
        # Symmetric by construction; assert that rather than assuming it.
        forward = _model.paths_related(a, b)
        self.assertEqual(forward, _model.paths_related(b, a))
        return forward

    def test_a_path_is_related_to_itself(self):
        self.assertTrue(self._related("/World/Cube", "/World/Cube"))

    def test_a_child_prim_is_related_to_its_ancestor(self):
        self.assertTrue(self._related("/World/Cube/Leg", "/World/Cube"))
        self.assertTrue(self._related("/World/Cube/Leg/Toe", "/World"))

    def test_a_property_is_related_to_its_prim(self):
        """Creating or removing an attribute resyncs the PROPERTY path, and
        that is what makes a new pin show up on the node."""
        self.assertTrue(self._related("/World/Cube.inputs:x", "/World/Cube"))
        self.assertTrue(self._related("/World/Cube.myRel", "/World/Cube"))

    def test_a_name_prefix_is_not_a_relation(self):
        self.assertFalse(self._related("/World/Cube", "/World/CubeTwo"))
        self.assertFalse(self._related("/World/Cube.inputs:x", "/World/Cub"))
        self.assertFalse(self._related("/World/Cube/Leg", "/World/Cub"))

    def test_unrelated_paths_are_unrelated(self):
        self.assertFalse(self._related("/A/B", "/C/D"))

    def test_everything_is_under_the_pseudo_root(self):
        self.assertTrue(self._related("/", "/World"))
        self.assertTrue(self._related("/", "/World/Cube.inputs:x"))

    def test_an_explicit_trailing_slash_already_ends_at_the_boundary(self):
        self.assertTrue(self._related("/World/Cube/", "/World/Cube/Leg"))

    def test_the_relation_is_case_sensitive(self):
        """Paths are compared as strings, so a differently spelled path is
        simply another path."""
        self.assertFalse(self._related("/world/cube", "/World/Cube"))

    def test_a_synthesized_node_id_is_not_related_to_its_prim(self):
        """Existing behaviour this extraction must not change: the caller at
        the node-following site passes a raw node id, and ``#`` is not a USD
        separator, so the synthesized id reads as unrelated to its prim."""
        self.assertFalse(self._related("/World/Bp#input:x", "/World/Bp"))


if __name__ == "__main__":
    unittest.main()
