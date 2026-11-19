#!/usr/bin/env python3
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct tests for the graph view's prim-path identity layer (issue #54).

``graphview/model.py`` is pure stdlib, so it is loaded by path instead of
through ``UsdNoodles``: importing the package would pull in pxr and Qt and
force these cases to skip on a machine that has neither. The spelling probe
and the alias wiring check read ``graphView.py`` through the AST, so they
need no pxr, Qt or GL either and cannot skip past a dropped alias. Only the
alias-identity case imports the widget, and it reports an explicit skip when
a dependency is missing; the wiring check is what makes that skip safe.
"""

import ast
import importlib.machinery
import importlib.util
import pathlib
import unittest

try:
    from UsdNoodles.graphView import GraphView, _isPathRelated, _nodeIdPrimPath
    from UsdNoodles.graphview import model

    _has_graph = True
except (ImportError, OSError) as error:  # pragma: no cover - import probe
    # OSError too: PyOpenGL raises it when the host has no GL library, and
    # this file's stdlib-only cases must still run and report 0 skips there.
    print("graph view alias imports failed: %s" % error)
    _has_graph = False

# testenv sits directly inside the UsdNoodles package in the source tree, in
# the staged build/python/UsdNoodles package, and in the installed package, so
# the module under test is always one directory above this file.
_PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
_MODEL_PATH = _PACKAGE_DIR / "graphview" / "model.py"
_GRAPHVIEW_SOURCE = _PACKAGE_DIR / "graphView.py"

# Every private name graphView.py keeps for a model function, and the model
# function each one must be bound to.
_MODEL_ALIASES = {
    "_nodeIdPrimPath": "node_id_prim_path",
    "_isPathRelated": "paths_related",
}


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


def _module_level_bindings(source_text):
    """{name: origin} for every name graphView.py binds at module level."""
    bindings = {}
    for node in ast.parse(source_text).body:
        if isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if node.level:
                module = "." * node.level + module
            for alias in node.names:
                bindings[alias.asname or alias.name] = (
                    f"import {alias.name} from {module}")
        elif isinstance(node, ast.Import):
            for alias in node.names:
                bindings[alias.asname or alias.name] = f"import {alias.name}"
        elif isinstance(
                node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bindings[node.name] = f"local definition of {node.name}"
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    bindings[target.id] = "local assignment"
    return bindings


def _alias_wiring_failures(bindings):
    """Description of the alias wiring problems, or None when the wiring is
    exactly the shared model functions."""
    missing = sorted(name for name in _MODEL_ALIASES if name not in bindings)
    if missing:
        return (
            "graphView.py does not import " + ", ".join(missing) + " from "
            "graphview.model, so graph view path identity has no shared "
            "implementation for " + ", ".join(missing) + "."
        )
    divergent = {
        name: bindings[name]
        for name, function in _MODEL_ALIASES.items()
        if bindings[name] != f"import {function} from .graphview.model"
    }
    if divergent:
        return (
            "graphView.py binds an alias to something other than the model "
            f"function: {divergent}"
        )
    return None


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


class PackageAndModuleSpellingTest(unittest.TestCase):
    """``graphview/`` and ``graphView.py`` must stay two different things.

    A redirect of the legacy mixed-case spelling to the package would break
    ``from UsdNoodles.graphView import GraphView`` in ``_qt_editor.py``, the
    ``tests/testUsdviewNoodles*.py`` integrations, and the testenv probes,
    silently on case-insensitive filesystems.
    """

    def test_each_spelling_resolves_to_its_own_file(self):
        package_spec = importlib.machinery.PathFinder.find_spec(
            "graphview", [str(_PACKAGE_DIR)])
        module_spec = importlib.machinery.PathFinder.find_spec(
            "graphView", [str(_PACKAGE_DIR)])

        self.assertIsNotNone(package_spec, "graphview package not found")
        self.assertTrue(package_spec.submodule_search_locations,
                        "graphview resolved to a module, not a package")
        self.assertEqual(
            pathlib.Path(package_spec.origin),
            _PACKAGE_DIR / "graphview" / "__init__.py")
        self.assertIsNotNone(module_spec, "graphView module not found")
        self.assertFalse(module_spec.submodule_search_locations,
                         "graphView resolved to the graphview package")
        self.assertEqual(pathlib.Path(module_spec.origin),
                         _PACKAGE_DIR / "graphView.py")

        print("graphview -> %s; graphView -> %s"
              % (package_spec.origin, module_spec.origin))


class GraphViewAliasWiringTest(unittest.TestCase):
    """``graphView.py`` imports the model functions; it owns no copy.

    Checked from source rather than by importing the widget: a missing alias
    makes the widget import fail, and a test that skips on that import would
    pass vacuously on any host without pxr, Qt or GL.
    """

    @classmethod
    def setUpClass(cls):
        if not _GRAPHVIEW_SOURCE.is_file():
            raise AssertionError(
                f"alias wiring check found no {_GRAPHVIEW_SOURCE}")
        cls.bindings = _module_level_bindings(_GRAPHVIEW_SOURCE.read_text())

    def test_each_alias_is_bound_to_the_model_function(self):
        failures = _alias_wiring_failures(self.bindings)
        if failures:
            self.fail(failures)

    def test_the_model_provides_every_function_the_aliases_import(self):
        for alias, function in _MODEL_ALIASES.items():
            if not callable(getattr(_model, function, None)):
                self.fail(
                    f"graphview/model.py defines no {function} for the "
                    f"{alias} alias")

    def test_the_wiring_check_trips_on_a_dropped_alias(self):
        """Self-test: prove the check fails rather than skips when an alias
        is gone, on a synthetic source that needs no dependencies."""
        synthetic = (
            "from .graphview.model import "
            "paths_related as _isPathRelated\n"
        )
        probe = GraphViewAliasWiringTest(
            "test_each_alias_is_bound_to_the_model_function")
        probe.bindings = _module_level_bindings(synthetic)
        with self.assertRaises(AssertionError) as caught:
            probe.test_each_alias_is_bound_to_the_model_function()
        self.assertIn("_nodeIdPrimPath", str(caught.exception))

    def test_the_wiring_check_trips_on_a_locally_redefined_alias(self):
        """Self-test: a private copy of the algorithm diverges, so it fails
        the wiring check even though the name is present."""
        synthetic = (
            "def _isPathRelated(a, b):\n    return a == b\n"
            "from .graphview.model import "
            "node_id_prim_path as _nodeIdPrimPath\n"
        )
        probe = GraphViewAliasWiringTest(
            "test_each_alias_is_bound_to_the_model_function")
        probe.bindings = _module_level_bindings(synthetic)
        with self.assertRaises(AssertionError) as caught:
            probe.test_each_alias_is_bound_to_the_model_function()
        self.assertIn("_isPathRelated", str(caught.exception))


@unittest.skipUnless(
    _has_graph,
    "UsdNoodles.graphView not importable (needs pxr, PySide6 and OpenGL)")
class GraphViewPathAliasTest(unittest.TestCase):
    """``graphView`` reads the model's functions, it does not own a copy.

    Identity rather than equal behaviour: one shared function object is what
    keeps the widget and the model from diverging on the next edit.
    """

    def test_the_module_aliases_are_the_model_functions(self):
        self.assertIs(_isPathRelated, model.paths_related)
        self.assertIs(_nodeIdPrimPath, model.node_id_prim_path)

    def test_graph_view_is_still_the_mixed_case_module(self):
        """Fails if the package ever shadows the module in a staged or
        installed tree."""
        self.assertEqual(GraphView.__module__, "UsdNoodles.graphView")


if __name__ == "__main__":
    unittest.main()
