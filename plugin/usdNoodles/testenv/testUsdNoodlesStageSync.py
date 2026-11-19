#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview/stage_sync layer.

The module functions run against real in-memory Usd.Stages with a recording
stub standing in for the view side -- the same split the reconciliation code
already assumes: it reads the view's state attributes and calls its hooks,
so a stub that records the hooks sees exactly the decisions the shipped
editor makes (rebuild links vs repaint only, drop vs follow across a move).
Reconciliation must never author a USD opinion; each case compares the root
layer's export before and after to prove it.

The import-identity case guards the two spelling-sensitive module names:
UsdNoodles.graphview (the package directory) and UsdNoodles.graphView (the
legacy module) must keep binding their own files on every platform.
"""

from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

try:
    from pxr import Sdf, Usd, UsdGeom
    from UsdNoodles.graphview import stage_sync

    _has_stage_sync = True
except ImportError as error:  # pragma: no cover - import probe
    print("stage sync imports failed: %s" % error)
    _has_stage_sync = False

try:
    from UsdNoodles.nodeGraphBlueprint import NodeGraphBlueprint

    _has_blueprint = True
except ImportError:  # pragma: no cover - native module not built
    _has_blueprint = False


def _layer_text(stage):
    """The whole root layer as text -- any authored opinion changes it."""
    return stage.GetRootLayer().ExportToString()


def _node_for(stage, path):
    """The slice of NodeModel the reconciliation reads.

    ``getUsdPrim`` tracks the model's current ``_primPath``, like the real
    NodeModel does after ``remapMovedNodes`` repoints it.
    """
    node = SimpleNamespace(
        id=path,
        name=path.rsplit("/", 1)[-1],
        selected=False,
        _prim=stage.GetPrimAtPath(path),
        _primPath=Sdf.Path(path),
        _position=(0.0, 0.0),
        _title_collapsed=False,
        titleCollapsed=False,
        _cachedType="Sphere",
        invalidateCache=MagicMock(),
        invalidate_value_row=MagicMock(),
    )
    node.getUsdPrim = lambda: stage.GetPrimAtPath(node._primPath)
    return node


def _synthetic_node(nodeId):
    """A Blueprint / Container I/O node: synthesized, never prim-backed."""
    return SimpleNamespace(
        id=nodeId,
        name=nodeId.split("#", 1)[1] if "#" in nodeId else nodeId,
        selected=False,
        _prim=None,
        _primPath=Sdf.Path(nodeId.split("#", 1)[0]),
        _position=(0.0, 0.0),
        _title_collapsed=False,
        titleCollapsed=False,
        _cachedType="Blueprint",
        getUsdPrim=lambda: None,
        invalidateCache=MagicMock(),
        invalidate_value_row=MagicMock(),
    )


class _StubGraph:
    """Enough of NodeGraph for reconciliation: the stage and the nodes."""

    def __init__(self, stage):
        self._stage = stage
        self.nodes = {}

    def getStage(self):
        return self._stage


class _StubView:
    """The context surface stage_sync reads, with recording hooks.

    Mirrors the real GraphView: ``nodes`` reads through to
    ``nodeGraph.nodes``, so a wholesale re-key is visible from both sides.
    The real scope rule runs through the real function -- a stub that
    guessed at it would pass while the shipped rule was wrong.
    """

    def __init__(self, stage, paths=(), scope="/"):
        self.nodeGraph = _StubGraph(stage)
        for path in paths:
            self.nodeGraph.nodes[path] = _node_for(stage, path)

        self._selectedNodes = set()
        self._marqueeBaseSelectedNodes = set()
        self._marqueePrevSelectedNodes = set()
        self._reconciledPositionVersions = {}
        self.nodeIdUnderCursor = ""
        self._renamingNodeId = ""
        self._valueEditTarget = None
        self._mungState = None
        self._tokenPopup = None
        self._hoveredPort = None
        self._tooltipProperty = None
        self._dragLinkSourceNode = None
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None
        self._reconnectOldSourceNodeId = None
        self._reconnectOldTargetNodeId = None
        self._currentPrimPath = None if scope == "/" else scope
        self._syncingSelection = False
        self._cachedShowAttributeValues = False
        self.linksChanged = False
        self.textChanged = False

        self._spatialIndex = MagicMock()
        # One recorder parent so cross-hook ORDER is assertable.
        self._hooks = MagicMock()
        self._invalidateBackReferenceLinkDataCache = (
            self._hooks.invalidateBackRefs
        )
        self._resetRenderCachesAfterBulkMove = self._hooks.resetRenderCaches
        self._clearRenderCache = self._hooks.clearRenderCache
        self._nodeRenderManager = self._hooks.nodeRenderManager
        self.update = self._hooks.update
        self._addNewNodeFast = MagicMock(
            side_effect=lambda prim, _stage: self._adopt(prim)
        )

    @property
    def nodes(self):
        return self.nodeGraph.nodes

    def _graphScopeParentPath(self):
        return stage_sync.graphScopeParentPath(self)

    def _valueRowOnlyChange(self, node, propertyName):
        return False

    def _isPinPropertyChange(self, propertyName):
        return False

    def _adopt(self, prim):
        pathStr = str(prim.GetPath())
        self.nodeGraph.nodes[pathStr] = _node_for(
            self.nodeGraph.getStage(), pathStr
        )
        return pathStr


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class GraphScopeParentPathTest(unittest.TestCase):
    """The scope rule the reconciliation measures adoption against."""

    def test_a_plain_graph_scopes_to_the_pseudo_root(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        self.assertEqual(stage_sync.graphScopeParentPath(view), "/")

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_blueprint_graph_scopes_to_its_own_prim(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(stage)
        view._currentPrimPath = Sdf.Path("/Keeper")
        self.assertEqual(stage_sync.graphScopeParentPath(view), "/Keeper")

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_scope_without_a_prim_reconciles_nothing(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        view.nodeGraph = NodeGraphBlueprint()
        view._currentPrimPath = None
        self.assertIsNone(stage_sync.graphScopeParentPath(view))


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ValidateNodesAgainstStageTest(unittest.TestCase):
    """Nodes whose prim is gone must not survive a resync."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        for name in ("Alpha", "Keeper"):
            UsdGeom.Sphere.Define(self.stage, "/%s" % name)

    def test_renamed_prim_leaves_no_node_behind(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/Alpha", "/Renamed")
        self.assertTrue(self.stage.GetEditTarget().GetLayer().Apply(edit))

        removed = stage_sync.validateNodesAgainstStage(view)

        self.assertEqual(removed, ["/Alpha"])
        self.assertEqual(sorted(view.nodes), ["/Keeper"])

    def test_delete_and_deactivation_drop_the_node_and_clear_the_bookkeeping(
        self,
    ) -> None:
        """A dropped id left in selection, marquee or the spatial index is a
        click on a dead node."""
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        view._selectedNodes.add("/Alpha")
        view._marqueeBaseSelectedNodes.add("/Alpha")
        view._marqueePrevSelectedNodes.add("/Alpha")
        self.stage.RemovePrim("/Alpha")
        self.stage.GetPrimAtPath("/Keeper").SetActive(False)

        removed = stage_sync.validateNodesAgainstStage(view)

        self.assertEqual(removed, ["/Alpha", "/Keeper"])
        self.assertEqual(view.nodes, {})
        self.assertEqual(view._selectedNodes, set())
        self.assertEqual(view._marqueeBaseSelectedNodes, set())
        self.assertEqual(view._marqueePrevSelectedNodes, set())
        view._spatialIndex.removeNode.assert_any_call("/Alpha")
        view._spatialIndex.removeNode.assert_any_call("/Keeper")

    def test_a_healthy_graph_is_left_alone(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        self.assertEqual(stage_sync.validateNodesAgainstStage(view), [])
        self.assertEqual(sorted(view.nodes), ["/Alpha", "/Keeper"])

    def test_synthesized_io_nodes_are_never_dropped(self) -> None:
        """Blueprint I/O nodes carry '#' and have no prim to validate."""
        view = _StubView(self.stage, ["/Alpha"])
        view.nodes["/Bp#input"] = _synthetic_node("/Bp#input")

        self.assertEqual(stage_sync.validateNodesAgainstStage(view), [])
        self.assertIn("/Bp#input", view.nodes)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class AdoptPrimsNewToScopeTest(unittest.TestCase):
    """The other half of a rename: the new prim needs a node."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(self.stage, "/Keeper")

    def test_new_prim_in_scope_is_adopted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Renamed")
        view = _StubView(self.stage, ["/Keeper"])

        added = stage_sync.adoptPrimsNewToScope(view, {"/Renamed"})

        self.assertEqual(added, ["/Renamed"])

    def test_property_paths_out_of_scope_and_already_shown_are_skipped(
        self,
    ) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        view = _StubView(self.stage, ["/Keeper"])

        self.assertEqual(
            stage_sync.adoptPrimsNewToScope(
                view, {"/Gone", "/Keeper.radius", "/Keeper/Child", "/Keeper"}
            ),
            [],
        )
        view._addNewNodeFast.assert_not_called()

    def test_inactive_prim_is_not_adopted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Hidden")
        self.stage.GetPrimAtPath("/Hidden").SetActive(False)
        view = _StubView(self.stage, ["/Keeper"])

        self.assertEqual(stage_sync.adoptPrimsNewToScope(view, {"/Hidden"}), [])

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_container_scope_adopts_only_its_own_children(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        UsdGeom.Sphere.Define(self.stage, "/Elsewhere")
        view = _StubView(self.stage, [], scope="/Keeper")
        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(self.stage)

        added = stage_sync.adoptPrimsNewToScope(
            view, {"/Keeper/Child", "/Elsewhere"}
        )

        self.assertEqual(added, ["/Keeper/Child"])


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class RemapMovedNodesTest(unittest.TestCase):
    """A namespace move re-keys the graph instead of dropping it."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        for path in (
            "/World",
            "/World/A",
            "/World/AB",
            "/WorldTwo",
            "/Zebra",
        ):
            UsdGeom.Sphere.Define(self.stage, path)

    def test_a_move_rekeys_descendants_and_spares_name_prefixes(self) -> None:
        view = _StubView(
            self.stage, ["/World", "/World/A", "/World/AB", "/WorldTwo"]
        )
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/World", "/Moved")
        self.stage.GetEditTarget().GetLayer().Apply(edit)
        before = _layer_text(self.stage)

        remap = stage_sync.remapMovedNodes(view, {"/World": "/Moved"})

        self.assertEqual(
            remap,
            {
                "/World": "/Moved",
                "/World/A": "/Moved/A",
                "/World/AB": "/Moved/AB",
            },
        )
        # Iteration order IS draw order: a pop/re-insert would reorder.
        self.assertEqual(
            list(view.nodes), ["/Moved", "/Moved/A", "/Moved/AB", "/WorldTwo"]
        )
        node = view.nodes["/Moved/A"]
        self.assertEqual(node.id, "/Moved/A")
        self.assertEqual(node._primPath, Sdf.Path("/Moved/A"))
        self.assertTrue(node._prim.IsValid())
        self.assertEqual(node.name, "A")
        # The namespace edit was the test's; reconciliation adds no opinion
        # of its own.
        self.assertEqual(_layer_text(self.stage), before)

    def test_every_endpoint_adjacent_cache_and_gesture_id_moves(self) -> None:
        view = _StubView(self.stage, ["/World", "/World/A"])
        view._selectedNodes.add("/World/A")
        view._marqueeBaseSelectedNodes.add("/World/A")
        view._marqueePrevSelectedNodes.add("/World")
        view._reconciledPositionVersions["/World/A"] = 7
        view.nodeIdUnderCursor = "/World/A"
        view._renamingNodeId = "/World"
        view._valueEditTarget = ("/World/A", "radius", 0, 0)
        view._mungState = {"nodeId": "/World/A", "primPath": "/World/A"}
        view._tokenPopup = {"nodeId": "/World", "primPath": "/World"}
        view._hoveredPort = ("/World/A", "input:x")
        view._tooltipProperty = ("/World/A", "radius")
        view._dragLinkSourceNode = "/World/A"
        view._dragLinkSyntheticTarget = ("/World", "output:x")
        view._dragLinkResolvedTarget = ("/World/A", "input:x")
        view._reconnectOldSourceNodeId = "/World"
        view._reconnectOldTargetNodeId = "/World/A"

        stage_sync.remapMovedNodes(view, {"/World": "/Outer/Moved"})

        self.assertEqual(view._selectedNodes, {"/Outer/Moved/A"})
        self.assertEqual(view._marqueeBaseSelectedNodes, {"/Outer/Moved/A"})
        self.assertEqual(view._marqueePrevSelectedNodes, {"/Outer/Moved"})
        self.assertEqual(view._reconciledPositionVersions, {"/Outer/Moved/A": 7})
        self.assertEqual(view.nodeIdUnderCursor, "/Outer/Moved/A")
        self.assertEqual(view._renamingNodeId, "/Outer/Moved")
        self.assertEqual(
            view._valueEditTarget, ("/Outer/Moved/A", "radius", 0, 0)
        )
        self.assertEqual(
            view._mungState,
            {"nodeId": "/Outer/Moved/A", "primPath": "/Outer/Moved/A"},
        )
        self.assertEqual(
            view._tokenPopup,
            {"nodeId": "/Outer/Moved", "primPath": "/Outer/Moved"},
        )
        self.assertEqual(view._hoveredPort, ("/Outer/Moved/A", "input:x"))
        self.assertEqual(view._tooltipProperty, ("/Outer/Moved/A", "radius"))
        self.assertEqual(view._dragLinkSourceNode, "/Outer/Moved/A")
        self.assertEqual(
            view._dragLinkSyntheticTarget, ("/Outer/Moved", "output:x")
        )
        self.assertEqual(
            view._dragLinkResolvedTarget, ("/Outer/Moved/A", "input:x")
        )
        self.assertEqual(view._reconnectOldSourceNodeId, "/Outer/Moved")
        self.assertEqual(view._reconnectOldTargetNodeId, "/Outer/Moved/A")

    def test_a_moved_scope_parent_moves_with_it(self) -> None:
        view = _StubView(self.stage, ["/World/A"], scope="/World")

        stage_sync.remapMovedNodes(view, {"/World": "/Outer/Moved"})

        self.assertEqual(view._currentPrimPath, Sdf.Path("/Outer/Moved"))

    def test_synthesized_ids_move_their_key_but_stay_unprimmed(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Bp")
        view = _StubView(self.stage, ["/Bp"])
        view.nodes["/Bp#input:drive"] = _synthetic_node("/Bp#input:drive")

        stage_sync.remapMovedNodes(view, {"/Bp": "/Bp2"})

        node = view.nodes["/Bp2#input:drive"]
        self.assertEqual(node.id, "/Bp2#input:drive")
        # The id only borrows the blueprint's path; the model's prim data is
        # its pin, not any prim at the new path.
        self.assertEqual(node._primPath, Sdf.Path("/Bp"))
        self.assertIsNone(node._prim)
        self.assertEqual(node.name, "input:drive")

    def test_the_invalidation_order_and_the_spatial_index_sweep(self) -> None:
        view = _StubView(self.stage, ["/World", "/Zebra"])

        stage_sync.remapMovedNodes(view, {"/World": "/Moved"})

        view._spatialIndex.removeNode.assert_called_once_with("/World")
        self.assertEqual(
            [c[0] for c in view._hooks.mock_calls],
            [
                "invalidateBackRefs",
                "nodeRenderManager.invalidateAll",
                "resetRenderCaches",
            ],
        )

    def test_a_move_with_no_nodes_behind_it_is_a_no_op(self) -> None:
        view = _StubView(self.stage, ["/Zebra"])

        self.assertEqual(
            stage_sync.remapMovedNodes(view, {"/Nothing": "/Here"}), {}
        )
        self.assertEqual(list(view.nodes), ["/Zebra"])
        view._hooks.resetRenderCaches.assert_not_called()


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ApplyUsdChangesTest(unittest.TestCase):
    """Notice classification, reconciliation and the repaint decisions."""

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        for name in ("Alpha", "Keeper"):
            UsdGeom.Sphere.Define(self.stage, "/%s" % name)

    def test_selection_sync_suppresses_everything(self) -> None:
        view = _StubView(self.stage, ["/Alpha"])
        view._syncingSelection = True
        self.stage.RemovePrim("/Alpha")

        stage_sync.applyUsdChanges(view, {"/Alpha"}, set(), {})

        self.assertIn("/Alpha", view.nodes)
        self.assertFalse(view.linksChanged)
        view._hooks.update.assert_not_called()

    def test_a_reported_move_is_followed_not_rebuilt(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/Alpha", "/Renamed")
        self.stage.GetEditTarget().GetLayer().Apply(edit)
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(
            view, {"/Alpha", "/Renamed"}, set(), {"/Alpha": "/Renamed"}
        )

        # No stale node and no duplicate: the old node rode across, and
        # adoption then found the new path already shown.
        self.assertEqual(list(view.nodes), ["/Renamed", "/Keeper"])
        view._addNewNodeFast.assert_not_called()
        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)
        view._hooks.update.assert_called_once()
        self.assertEqual(_layer_text(self.stage), before)

    def test_a_rename_without_a_move_map_drops_and_adopts(self) -> None:
        """USD without GetPrimResyncType: delete-and-create reconciliation."""
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/Alpha", "/Renamed")
        self.stage.GetEditTarget().GetLayer().Apply(edit)
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(view, {"/Alpha", "/Renamed"}, set())

        self.assertEqual(sorted(view.nodes), ["/Keeper", "/Renamed"])
        self.assertTrue(view.linksChanged)
        self.assertEqual(_layer_text(self.stage), before)

    def test_a_deletion_drops_the_node_and_rebuilds_links(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        self.stage.RemovePrim("/Alpha")
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(view, {"/Alpha"}, set())

        self.assertEqual(sorted(view.nodes), ["/Keeper"])
        self.assertTrue(view.linksChanged)
        view._hooks.clearRenderCache.assert_called_once()
        self.assertEqual(_layer_text(self.stage), before)

    def test_a_value_shown_in_a_cell_repaints_without_a_link_rebuild(
        self,
    ) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        view._cachedShowAttributeValues = True
        view._valueRowOnlyChange = lambda node, propertyName: True
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(view, set(), {"/Alpha.radius"})

        view.nodes["/Alpha"].invalidate_value_row.assert_called_once_with(
            "radius"
        )
        view.nodes["/Alpha"].invalidateCache.assert_not_called()
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)
        view._hooks.clearRenderCache.assert_not_called()
        view._hooks.update.assert_called_once()
        self.assertEqual(_layer_text(self.stage), before)

    def test_the_feature_flag_gates_the_row_probe_before_it_runs(self) -> None:
        """With values hidden the row cache is cold -- asking it would build
        every row to throw it away, so the flag must be checked first."""
        view = _StubView(self.stage, ["/Alpha"])
        view._cachedShowAttributeValues = False
        probe = MagicMock(return_value=True)
        view._valueRowOnlyChange = probe
        view._isPinPropertyChange = lambda propertyName: True

        stage_sync.applyUsdChanges(view, set(), {"/Alpha.inputs:x"})

        probe.assert_not_called()
        self.assertTrue(view.linksChanged)
        view._hooks.clearRenderCache.assert_called_once()

    def test_a_position_change_repaints_only(self) -> None:
        view = _StubView(self.stage, ["/Alpha"])
        stage_sync.applyUsdChanges(
            view, set(), {"/Alpha.ui:nodegraph:node:pos"}
        )

        self.assertIsNone(view.nodes["/Alpha"]._position)
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)
        view._hooks.clearRenderCache.assert_not_called()
        view._hooks.update.assert_called_once()

    def test_a_bare_prim_info_change_never_asks_for_the_empty_attribute(
        self,
    ) -> None:
        """The notice named the prim, not a property: propName is empty and
        GetAttribute("") raises a USD coding error. The guard must hold or a
        ChangeBlock-close notice turns a landed write into a reported
        failure."""
        view = _StubView(self.stage, ["/Alpha"])

        stage_sync.applyUsdChanges(view, set(), {"/Alpha"})

        # Survived, and treated as a plain repaint.
        view.nodes["/Alpha"].invalidateCache.assert_called_once()
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ImportIdentityTest(unittest.TestCase):
    """The two spelling-sensitive names must keep binding their own files.

    ``graphview/`` (the package) and ``graphView.py`` (the legacy module)
    differ only in case and separator. CPython's FileFinder checks the
    directory first and only lowercases the file suffix, so both spellings
    resolve correctly on case-insensitive platforms -- but a platform or a
    PYTHONCASEOK-style deviation that bound the wrong one must fail HERE,
    loudly, not silently half-import the editor.
    """

    def test_the_package_binds_the_graphview_directory(self) -> None:
        package = sys.modules.get("UsdNoodles.graphview")
        self.assertIsNotNone(package, "importing stage_sync binds the package")
        self.assertTrue(
            package.__file__.replace("\\", "/").endswith(
                "graphview/__init__.py"
            ),
            "UsdNoodles.graphview bound %s" % package.__file__,
        )

    def test_the_legacy_module_binds_graphView_py(self) -> None:
        try:
            import UsdNoodles.graphView
        except ImportError as error:  # pragma: no cover - Qt not installed
            self.skipTest("graphView not importable: %s" % error)
        self.assertTrue(
            UsdNoodles.graphView.__file__.replace("\\", "/").endswith(
                "graphView.py"
            ),
            "UsdNoodles.graphView bound %s" % UsdNoodles.graphView.__file__,
        )
        self.assertIsNot(
            UsdNoodles.graphView, sys.modules.get("UsdNoodles.graphview")
        )


if __name__ == "__main__":
    unittest.main()
