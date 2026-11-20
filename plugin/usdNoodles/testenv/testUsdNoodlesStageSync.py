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

Only OpenUSD is treated as an optional dependency here. The modules under
test -- the graphview package, the notice handler, and the editor module that
wires them -- are product code: an ImportError from one of them is a broken
staged package, so it fails the run instead of skipping the coverage.
"""

from __future__ import annotations

import dataclasses
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

try:
    from pxr import Sdf, Usd, UsdGeom, UsdUI

    _has_usd = True
except ImportError as error:  # pragma: no cover - OpenUSD not installed
    print("OpenUSD imports failed: %s" % error)
    _has_usd = False

if _has_usd:
    from UsdNoodles.graphview import stage_sync
    from UsdNoodles.graphView import _makeStageSync, GraphView
    from UsdNoodles.usdNoticeHandler import UsdNoticeHandler

    _has_stage_sync = True
else:
    _has_stage_sync = False

try:
    from UsdNoodles.nodeGraphBlueprint import NodeGraphBlueprint

    _has_blueprint = True
except ImportError:  # pragma: no cover - native module not built
    _has_blueprint = False

# The Qt bridge is the only optional piece here; the editor module that uses
# it is product code, so its import is not guarded -- a broken staged package
# fails the run instead of quietly skipping the dock coverage.
try:
    from pxr.Usdviewq.qt import QtCore, QtWidgets

    _has_qt = True
except ImportError as error:  # pragma: no cover - Qt bridge not installed
    print("Qt bridge imports failed: %s" % error)
    _has_qt = False

if _has_qt:
    from UsdNoodles._qt_editor import NoodlesEditorWindow

# A widget needs a live QApplication for its whole life. The headless harness
# drops its app when the GL check fails, and Qt then aborts on the next widget
# while QApplication.instance() still hands back the dead wrapper. Holding one
# from import keeps every widget test in the file on a live app.
_dock_app = None
if _has_qt:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    _dock_app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


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


class _KeyedOnlyNodes(dict):
    """A node map a USD notice may index but must never walk.

    ``__contains__`` and ``__getitem__`` do not consult ``__iter__`` on a
    dict, so the keyed lookups reconciliation makes keep working while any
    whole-graph scan raises -- which is the difference between a notice that
    costs the notice and one that costs the graph.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.iterated = []

    def _refuse(self, how):
        self.iterated.append(how)
        raise AssertionError(
            "the notice path walked the node map (%s)" % how
        )

    def __iter__(self):
        return self._refuse("__iter__")

    def keys(self):
        return self._refuse("keys")

    def values(self):
        return self._refuse("values")

    def items(self):
        return self._refuse("items")


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


class _ClosedHostReads(_StubView):
    """A view whose host surface refuses reads once the editor is closed.

    usdview's dataModel keeps ``signalStageReplaced`` connected, and a closed
    Qt widget is not a destroyed one, so the slot can fire while the api, the
    graph and the current prim path are going away. Asking for any of them
    then fails, which is what the teardown race looks like from the inside --
    the effect assertions alone only catch the damage a successful read does.
    """

    _HOST_READS = frozenset(("_usdviewApi", "nodeGraph", "_currentPrimPath"))

    def __init__(self, *args, **kwargs):
        object.__setattr__(self, "_hostClosed", False)
        self.host_reads = []
        super().__init__(*args, **kwargs)

    def closeHost(self):
        object.__setattr__(self, "_hostClosed", True)

    def openHost(self):
        object.__setattr__(self, "_hostClosed", False)

    def __getattribute__(self, name):
        if name in _ClosedHostReads._HOST_READS and object.__getattribute__(
            self, "_hostClosed"
        ):
            self.host_reads.append(name)
            raise AssertionError("a closed editor read %s" % name)
        return object.__getattribute__(self, name)


class _LifecycleView:
    """A headless editor: the model a reload mutates, recorded.

    The service's ``read``/``apply`` contract against plain state -- no Qt,
    no GL -- so the lifecycle cases below see exactly the epoch gating,
    scheduling and suppression the shipped editor gets.
    """

    def __init__(self, node_ids=()):
        self.nodes = {node_id: object() for node_id in node_ids}
        self.stage = None
        self.mode = "stage"
        self.prim_path = None
        self.syncing = False
        self._deferRenderCacheClear = False
        self.initialized = True
        self.updates_enabled = True
        self.history = ["undo-record"]
        self.loaded = None
        self.fail_loads = False
        self.frames = []
        self.applied = []
        self.finishes = 0

    def read(self):
        return stage_sync.StageContext(
            view=self,
            stage=self.stage,
            mode=self.mode,
            prim_path=self.prim_path,
            node_ids=tuple(self.nodes),
            syncing_selection=self.syncing,
        )

    def readNotice(self):
        return stage_sync.NoticeContext(
            view=self, syncing_selection=self.syncing
        )

    def apply(self, delta):
        self.applied.append(delta)
        if isinstance(delta, stage_sync.BeginReload):
            self.updates_enabled = False
            self.frames.append(delta.frame)
        elif isinstance(delta, stage_sync.Load):
            if self.fail_loads:
                raise RuntimeError("load boom")
            self.loaded = (delta.stage, delta.mode, delta.prim_path)
        elif isinstance(delta, stage_sync.ClearHistory):
            self.history = None
        elif isinstance(delta, stage_sync.FinishReload):
            self.finishes += 1
            self.updates_enabled = True


class _LifecycleHarness:
    """A StageSync over a _LifecycleView with recording sink and scheduler."""

    def __init__(self, view):
        self.view = view
        self.deltas = []
        self.queued = []
        self.warnings = []
        self.sync = stage_sync.StageSync(
            read=view.read,
            notice=view.readNotice,
            apply=view.apply,
            post=lambda callback: self.queued.append(callback),
            listen=lambda receiver: UsdNoticeHandler(receiver),
            warn=self.warnings.append,
            sink=self.deltas.append,
        )

    def run_queued(self):
        queued, self.queued = self.queued, []
        for callback in queued:
            callback()


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

    def test_an_expansion_change_collapses_and_relayouts(self) -> None:
        """A collapse repaints, relayouts and re-derives the links: the pins
        under the header move with it."""
        view = _StubView(self.stage, ["/Alpha"])
        api = UsdUI.NodeGraphNodeAPI(self.stage.GetPrimAtPath("/Alpha"))
        api.CreateExpansionStateAttr().Set("closed")
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(
            view, set(), {"/Alpha.ui:nodegraph:node:expansionState"}
        )

        self.assertTrue(view.nodes["/Alpha"]._title_collapsed)
        self.assertTrue(view.nodes["/Alpha"].titleCollapsed)
        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)
        self.assertEqual(_layer_text(self.stage), before)

    def test_an_expansion_notice_without_an_authored_value_is_silent(
        self,
    ) -> None:
        """No authored state means no collapse change: nothing repaints."""
        view = _StubView(self.stage, ["/Alpha"])
        before = _layer_text(self.stage)

        stage_sync.applyUsdChanges(
            view, set(), {"/Alpha.ui:nodegraph:node:expansionState"}
        )

        self.assertFalse(view.nodes["/Alpha"]._title_collapsed)
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)
        view._hooks.update.assert_not_called()
        self.assertEqual(_layer_text(self.stage), before)

    def test_a_node_type_change_drops_the_cached_type(self) -> None:
        view = _StubView(self.stage, ["/Alpha"])
        view.nodes["/Alpha"]._cachedType = "Sphere"

        stage_sync.applyUsdChanges(
            view, set(), {"/Alpha.info:implementationSource"}
        )

        self.assertIsNone(view.nodes["/Alpha"]._cachedType)
        self.assertFalse(view.linksChanged)
        self.assertTrue(view.textChanged)
        view.nodes["/Alpha"].invalidateCache.assert_not_called()

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
class ReplaceStalenessTest(unittest.TestCase):
    """Only the latest generation's deferred work may touch the model."""

    def test_replace_a_b_none_before_any_callback_runs(self) -> None:
        stageA = Usd.Stage.CreateInMemory()
        stageB = Usd.Stage.CreateInMemory()
        view = _LifecycleView(["/Alpha", "/Bp#input:drive"])
        env = _LifecycleHarness(view)

        env.sync.replace(stageA, "stage", None, "frame-A")
        env.sync.replace(stageB, "stage", None, "frame-B")
        env.sync.replace(None, "stage", None, "frame-None")

        # Nothing deferred has run yet: three finishes are queued, and every
        # emitted delta carries the epoch that produced it.
        self.assertEqual(len(env.queued), 3)
        self.assertEqual(
            [type(delta).__name__ for delta in env.deltas],
            [
                "BeginReload",
                "Remove",
                "Load",
                "BeginReload",
                "Remove",
                "Load",
                "BeginReload",
                "ClearHistory",
            ],
        )
        self.assertEqual(
            [delta.epoch for delta in env.deltas], [1, 1, 1, 2, 2, 2, 3, 3]
        )
        self.assertEqual(view.frames, ["frame-A", "frame-B", "frame-None"])
        # The reload records the whole graph it tore down, synthetic ids
        # included.
        self.assertEqual(env.deltas[1].ids, ("/Alpha", "/Bp#input:drive"))

        # A notice stamped with an older generation never reaches the model.
        applied = list(view.applied)
        env.sync.changed(1, {"/Alpha"}, set())
        env.sync.changed(2, {"/Alpha"}, set())
        self.assertEqual(view.applied, applied)
        self.assertEqual(len(env.deltas), 8)

        env.run_queued()

        # The two stale finishes did nothing at all; the null-stage finish
        # ran exactly once.
        finishes = [
            delta
            for delta in env.deltas
            if isinstance(delta, stage_sync.FinishReload)
        ]
        self.assertEqual(finishes, [stage_sync.FinishReload(3)])
        self.assertEqual(view.finishes, 1)
        self.assertTrue(view.updates_enabled)
        # No third load ran; the null stage cleared the history instead.
        self.assertEqual(view.loaded[0], stageB)
        self.assertIsNone(view.history)

    def test_finish_from_a_rebound_stage_is_dropped(self) -> None:
        """The queued callback checks its source binding, not just its
        epoch."""
        stageA = Usd.Stage.CreateInMemory()
        stageB = Usd.Stage.CreateInMemory()
        env = _LifecycleHarness(_LifecycleView())

        env.sync.replace(stageA, "stage", None, "frame")
        env.sync.bind(stageB)
        env.run_queued()

        self.assertEqual(env.view.finishes, 0)
        self.assertEqual(
            [
                delta
                for delta in env.deltas
                if isinstance(delta, stage_sync.FinishReload)
            ],
            [],
        )

    def test_selection_sync_suppresses_a_live_notice(self) -> None:
        stageA = Usd.Stage.CreateInMemory()
        view = _LifecycleView()
        view.syncing = True
        env = _LifecycleHarness(view)
        env.sync.replace(stageA, "stage", None, "frame")

        applied = list(view.applied)
        env.sync.changed(1, {"/Alpha"}, set())

        self.assertEqual(view.applied, applied)
        self.assertEqual(len(env.deltas), 3)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class LoadFailureTest(unittest.TestCase):
    """A failing load must not strand the editor or the next replace."""

    def test_a_failing_load_warns_restores_and_still_finishes(self) -> None:
        stageA = Usd.Stage.CreateInMemory()
        stageB = Usd.Stage.CreateInMemory()
        view = _LifecycleView()
        view.fail_loads = True
        env = _LifecycleHarness(view)

        env.sync.replace(stageA, "stage", None, "frame")

        self.assertEqual(len(env.warnings), 1)
        self.assertIn("load boom", env.warnings[0])
        self.assertFalse(view._deferRenderCacheClear)
        self.assertEqual(len(env.queued), 1)

        env.run_queued()
        self.assertEqual(view.finishes, 1)

        view.fail_loads = False
        env.sync.replace(stageB, "stage", None, "frame")
        env.run_queued()

        self.assertEqual(view.loaded[0], stageB)
        self.assertEqual(view.finishes, 2)
        self.assertEqual(
            [delta.epoch for delta in env.deltas], [1, 1, 1, 1, 2, 2, 2, 2]
        )


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ReloadModeTest(unittest.TestCase):
    """Each mode reaches the loader it names."""

    def test_each_mode_reaches_the_loader_it_names(self) -> None:
        for mode, prim_path in (
            ("stage", None),
            ("blueprint", "/Bp"),
            ("container", "/Box"),
        ):
            with self.subTest(mode=mode):
                stage = Usd.Stage.CreateInMemory()
                view = _LifecycleView()
                env = _LifecycleHarness(view)

                env.sync.replace(stage, mode, prim_path, "frame")

                loads = [
                    delta
                    for delta in env.deltas
                    if isinstance(delta, stage_sync.Load)
                ]
                self.assertEqual(len(loads), 1)
                self.assertIs(loads[0].stage, stage)
                self.assertEqual(loads[0].mode, mode)
                self.assertEqual(view.loaded, (stage, mode, prim_path))


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class SuppressionTest(unittest.TestCase):
    """Suppression lives on the editor's handler and always restores."""

    def test_suppression_restores_on_the_normal_path(self) -> None:
        env = _LifecycleHarness(_LifecycleView())
        handler = env.sync.handler()

        self.assertTrue(handler.isEnabled())
        with env.sync.suppressed():
            self.assertFalse(handler.isEnabled())
        self.assertTrue(handler.isEnabled())

    def test_suppression_restores_on_exception(self) -> None:
        env = _LifecycleHarness(_LifecycleView())
        handler = env.sync.handler()

        with self.assertRaises(RuntimeError):
            with env.sync.suppressed():
                raise RuntimeError("edit boom")
        self.assertTrue(handler.isEnabled())

    def test_nested_suppression_restores_the_outer_state(self) -> None:
        env = _LifecycleHarness(_LifecycleView())
        handler = env.sync.handler()

        with env.sync.suppressed():
            with self.assertRaises(RuntimeError):
                with env.sync.suppressed():
                    raise RuntimeError("inner boom")
            self.assertFalse(handler.isEnabled())
        self.assertTrue(handler.isEnabled())


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class CloseTest(unittest.TestCase):
    """Close revokes the receiver and invalidates anything still queued."""

    def test_close_drops_queued_finishes_and_later_notices(self) -> None:
        stageA = Usd.Stage.CreateInMemory()
        stageB = Usd.Stage.CreateInMemory()
        view = _LifecycleView()
        env = _LifecycleHarness(view)
        env.sync.replace(stageA, "stage", None, "frame")
        env.run_queued()
        self.assertEqual(view.finishes, 1)

        env.sync.replace(stageB, "stage", None, "frame")
        live = env.sync.epoch()
        env.sync.handler().register(stageB)
        self.assertIs(env.sync.handler().getStage(), stageB)
        env.sync.close()

        self.assertTrue(env.sync.isClosed())
        self.assertIsNone(env.sync.handler().getStage())
        env.run_queued()
        self.assertEqual(view.finishes, 1)
        applied = list(view.applied)
        env.sync.changed(live, {"/Alpha"}, set())
        self.assertEqual(view.applied, applied)
        self.assertEqual(
            [
                delta
                for delta in env.deltas
                if isinstance(delta, stage_sync.FinishReload)
            ],
            [stage_sync.FinishReload(1)],
        )


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class NoticeRegistrationTest(unittest.TestCase):
    """The handler delivers a real ObjectsChanged through the service."""

    @staticmethod
    def _rename(stage):
        editor = Usd.NamespaceEditor(stage)
        if not editor.RenamePrim(stage.GetPrimAtPath("/Alpha"), "Renamed"):
            raise AssertionError("the rename did not stage")
        if not editor.ApplyEdits():
            raise AssertionError("the rename did not apply")

    def test_a_live_rename_reconciles_through_the_handler(self) -> None:
        stages = []
        for _ in range(2):
            stage = Usd.Stage.CreateInMemory()
            UsdGeom.Sphere.Define(stage, "/Alpha")
            UsdGeom.Sphere.Define(stage, "/Keeper")
            stages.append(stage)
        stage, twin = stages

        view = _StubView(stage, ["/Alpha", "/Keeper"])
        deltas = []
        service = stage_sync.StageSync(
            read=lambda: stage_sync.StageContext(
                view=view,
                stage=stage,
                mode="stage",
                prim_path=None,
                node_ids=tuple(view.nodes),
                syncing_selection=view._syncingSelection,
            ),
            notice=lambda: stage_sync.NoticeContext(
                view=view, syncing_selection=view._syncingSelection
            ),
            apply=lambda delta: None,
            post=lambda callback: None,
            listen=lambda receiver: UsdNoticeHandler(receiver),
            warn=lambda message: None,
            sink=deltas.append,
        )
        service.bind(stage)
        service.handler().register(stage)

        self._rename(stage)
        self._rename(twin)

        # The same rename, with and without the service listening: the layer
        # texts match, so reconciliation authored no opinion of its own.
        self.assertEqual(_layer_text(stage), _layer_text(twin))
        # And the graph followed the prim through the real notice.
        self.assertEqual(list(view.nodes), ["/Renamed", "/Keeper"])
        self.assertEqual(
            deltas,
            [
                stage_sync.Remap(0, (("/Alpha", "/Renamed"),)),
                stage_sync.Invalidate(
                    0, ("/Alpha", "/Renamed"), True, True, True
                ),
            ],
        )
        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)
        view._hooks.update.assert_called_once()


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ReplaceAfterCloseTest(unittest.TestCase):
    """A stage replace delivered to a closed editor stays in the delivery.

    A closed Qt widget is not a destroyed one: usdview's dataModel keeps the
    signal connected and can hand a replacement to an editor that is already
    shutting down. Nothing may be read, applied, registered or queued then --
    a reload that tears down a graph nobody is displaying, and re-arms a
    notice receiver close had revoked, ends in a widget left with paint off.
    """

    def test_close_then_replace_touches_nothing(self) -> None:
        stageA = Usd.Stage.CreateInMemory()
        stageB = Usd.Stage.CreateInMemory()
        view = _LifecycleView(["/Alpha"])
        env = _LifecycleHarness(view)
        env.sync.replace(stageA, "stage", None, "frame")
        env.run_queued()
        handler = env.sync.handler()
        handler.register(stageA)
        self.assertIs(handler.getStage(), stageA)

        live = env.sync.epoch()
        applied = list(view.applied)
        published = list(env.deltas)

        env.sync.close()
        env.sync.replace(stageB, "blueprint", "/Bp", "late-frame")

        # No delta published, no delta applied: the view was not read, so no
        # BeginReload tore down its state and no Load replaced its graph.
        self.assertEqual(env.deltas, published)
        self.assertEqual(view.applied, applied)
        self.assertEqual(view.frames, ["frame"])
        self.assertEqual(view.loaded, (stageA, "stage", None))
        self.assertEqual(list(view.nodes), ["/Alpha"])
        # No listener on the late stage, and the editor's own is gone.
        self.assertIsNone(handler.getStage())
        # Nothing queued for the owner thread, and the live finish is the
        # only one that ever ran.
        self.assertEqual(env.queued, [])
        self.assertEqual(view.finishes, 1)
        # close() moved the epoch; the late replace did not, and its epoch is
        # stale with it -- so a notice stamped with the live generation still
        # cannot reach the model.
        self.assertEqual(env.sync.epoch(), live + 1)
        env.sync.changed(live, {"/Alpha"}, set())
        self.assertEqual(view.applied, applied)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ProductionAdapterTest(unittest.TestCase):
    """The editor's own wiring -- ``_makeStageSync`` -- against a stub view.

    The stubs above hand the service its readers; these cases run the
    readers the shipped editor builds, so a reader that reaches too much of
    the widget, or a policy the widget stops applying, fails here.
    """

    def _view(self, stage, paths, view_class=_StubView):
        """A stub view whose node map refuses to be walked (see
        ``_KeyedOnlyNodes``), plus the usdview api ``readStageContext`` reads.
        """
        view = view_class(stage, paths)
        view.nodeGraph.nodes = _KeyedOnlyNodes(view.nodeGraph.nodes)
        view._usdviewApi = SimpleNamespace(
            stage=stage, frame=Usd.TimeCode.Default()
        )
        return view

    def test_a_single_property_notice_never_walks_the_graph(self) -> None:
        """The notice path reads the view for the notice, not for a reload.

        One authored property value is one info-only path: reconciliation may
        index ``view.nodes`` for that path and nothing else. Taking the
        reload snapshot here would copy every node id per notice, so a value
        drag -- a notice per mouse-move -- would scale with the graph.

        The radius starts out authored, so the write below is a value change
        and not a property create, which USD reports as a prim resync.
        """
        stage = Usd.Stage.CreateInMemory()
        for name in ("Alpha", "Keeper"):
            prim = UsdGeom.Sphere.Define(stage, "/%s" % name).GetPrim()
            prim.GetAttribute("radius").Set(1.0)
        view = self._view(stage, ["/Alpha", "/Keeper"])
        service = _makeStageSync(view)
        service.bind(stage)
        service.handler().register(stage)

        stage.GetPrimAtPath("/Alpha").GetAttribute("radius").Set(3.0)

        self.assertEqual(view.nodes.iterated, [])
        view.nodes["/Alpha"].invalidateCache.assert_called_once()
        view.nodes["/Keeper"].invalidateCache.assert_not_called()
        # A value change repaints; it does not rebuild links or re-layout text.
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)
        view._hooks.update.assert_called_once()

    def test_the_shipped_notice_carries_no_reconcile_trace(self) -> None:
        """No sink means no trace: the shipped editor publishes nothing.

        ``_makeStageSync`` builds the service without a sink, so per notice
        there is nowhere to record. Sorting and copying the path sets for a
        trace nobody reads would tax every mouse-move value notice; the audit
        sink in ``NoticeRegistrationTest`` is what exercises the traced path.
        """
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(stage, "/Alpha").GetPrim().GetAttribute(
            "radius"
        ).Set(1.0)
        view = self._view(stage, ["/Alpha"])
        service = _makeStageSync(view)
        service.bind(stage)
        service.handler().register(stage)

        traces = []

        def capture(*_args, **kwargs):
            traces.append(kwargs.get("trace"))

        with patch.object(stage_sync, "applyUsdChanges", capture):
            stage.GetPrimAtPath("/Alpha").GetAttribute("radius").Set(3.0)

        self.assertEqual(len(traces), 1)
        self.assertIsNone(traces[0])

    def test_a_replace_delivered_after_close_stops_at_the_service(self) -> None:
        """usdview's signal reaches a closed editor through the real slot.

        The host reads are forbidden, not just watched: the slot must decide
        the editor is gone before it asks the api for a frame or a stage, or
        asks the graph which loader a reload would re-run.
        """
        stageA = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(stageA, "/Alpha")
        UsdGeom.Sphere.Define(stageA, "/Keeper")
        stageB = Usd.Stage.CreateInMemory()
        view = self._view(stageA, ["/Alpha", "/Keeper"], _ClosedHostReads)
        service = _makeStageSync(view)
        view._stageSync = service
        view._noticeHandler = service.handler()
        service.handler().register(stageA)
        service.close()
        # usdview has moved on to a new stage and emits to the editor that
        # already closed; the api it hands the slot is the replacement's.
        view._usdviewApi = SimpleNamespace(
            stage=stageB, frame=Usd.TimeCode.Default()
        )
        view.closeHost()

        try:
            with patch(
                "UsdNoodles.graphView.QtCore.QTimer.singleShot"
            ) as posted:
                GraphView._onStageReplaced(view)
        finally:
            view.openHost()

        self.assertEqual(view.host_reads, [])
        posted.assert_not_called()
        self.assertIsNone(service.handler().getStage())
        self.assertEqual(view.nodes.iterated, [])
        self.assertEqual(len(view.nodes), 2)
        self.assertIn("/Alpha", view.nodes)
        self.assertIn("/Keeper", view.nodes)
        # Not one view hook ran: no paint guard, no cache clear, no load.
        self.assertEqual(view._hooks.mock_calls, [])


if _has_qt:

    class _DockDataModel(QtCore.QObject):
        """The usdview data model, with only the signal the wiring needs."""

        signalStageReplaced = QtCore.Signal()

    class _DockApi:
        """A usdviewApi whose stage and frame are off-limits once it is closed.

        Both are properties rather than plain attributes so a read after close
        raises instead of quietly returning the replacement, and the count is
        bumped BEFORE the raise: PySide reports a slot exception instead of
        propagating it through ``emit()``, so the counters asserted from the
        test body are what make a forbidden read visible.
        """

        def __init__(self, stage):
            self.dataModel = _DockDataModel()
            self._frame = Usd.TimeCode.Default()
            self._stage = stage
            self.host_closed = False
            self.api_reads = 0
            self.post_close_reads = []

        def _borrow(self, name, value):
            self.api_reads += 1
            if self.host_closed:
                self.post_close_reads.append(name)
                raise AssertionError("a closed editor read the %s" % name)
            return value

        @property
        def stage(self):
            return self._borrow("stage", self._stage)

        @property
        def frame(self):
            return self._borrow("frame", self._frame)


@unittest.skipUnless(_has_qt, "Qt bridge not available")
class DockWindowCloseTest(unittest.TestCase):
    """Closing the dock runs the graph's teardown before the close is reported.

    Qt hides a dock's children without delivering their closeEvent, so the
    window's handler has to reach through to the graph's teardown. Without it
    the epoch stays live, the reload finish stays queued, and the notice
    receiver stays registered for a graph nobody is displaying. Neither close
    path reads the host api, and a replacement arriving after the close is
    refused before the api or the closed graph is touched.
    """

    def setUp(self):
        if _dock_app is None:  # pragma: no cover - Qt without a usable platform
            raise unittest.SkipTest("no QApplication could be created")
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(stage, "/Alpha")
        self.api = _DockApi(stage)
        self.window = NoodlesEditorWindow(self.api, 7)
        self.view = self.window.graphView
        self.closes = []

        def note(window_id):
            # What the close report sees: the teardown must already have run.
            self.closes.append(
                (
                    window_id,
                    self.view._stageSync.isClosed(),
                    self.view._undoHistory is None,
                )
            )

        self.window.windowClosed.connect(note)

    def tearDown(self):
        self.window.close()

    def _state(self):
        view = self.view
        return (
            view.initialized,
            view.updatesEnabled(),
            len(view.nodes),
            len(view.links),
            view._undoHistory,
            view._noticeHandler.getStage(),
        )

    def test_close_teardowns_then_ignores_a_later_replace(self):
        view = self.view
        service = view._stageSync
        queued = []
        ran = []

        with patch(
            "UsdNoodles.graphView.QtCore.QTimer.singleShot",
            lambda delay, callback: queued.append(callback),
        ):
            # A reload started through the real signal and not yet finished:
            # the finish is queued, paint is off and the notice receiver is
            # revoked until it does. Emitting while the editor is open also
            # proves the wiring is live, so the zero-count assertion after the
            # close measures a refused read rather than an unconnected slot.
            self.api.dataModel.signalStageReplaced.emit()
            self.assertEqual(len(queued), 1)
            self.assertFalse(view.initialized)
            self.assertFalse(view.updatesEnabled())
            live = service.epoch()
            reads = self.api.api_reads
            self.assertGreater(reads, 0)

            self.window.close()
            self.assertEqual(self.closes, [(7, True, True)])
            self.assertTrue(service.isClosed())
            self.assertEqual(service.epoch(), live + 1)
            self.assertIsNone(view._undoHistory)
            self.assertIsNone(view._noticeHandler.getStage())
            torn_down = self._state()

            # usdview moves on to another stage and emits through the same
            # signal; the host is now off-limits, so any read the slot makes
            # is recorded by the api before it raises. PySide reports a slot
            # exception instead of raising it back through emit(), so the
            # assertion lives out here, not in the slot.
            self.api.host_closed = True
            self.api.dataModel.signalStageReplaced.emit()

            # Drain what the reload left behind through the body an open
            # editor would have run, so an unwired finish is visible rather
            # than silent.
            with patch.object(
                stage_sync,
                "_finishReloadOnView",
                lambda view: ran.append("finish"),
            ):
                for callback in queued:
                    callback()

        self.assertEqual(ran, [])
        self.assertEqual(self.api.post_close_reads, [])
        self.assertEqual(self.api.api_reads, reads)
        self.assertEqual(len(queued), 1)  # the late replace queued nothing
        self.assertEqual(self._state(), torn_down)
        self.assertEqual(service.epoch(), live + 1)

    def test_both_close_paths_teardown_once(self):
        view = self.view
        service = view._stageSync
        live = service.epoch()
        reads = self.api.api_reads

        # Neither close handler may consult the host: the teardown works from
        # the view and its stage service alone.
        self.api.host_closed = True

        # The dock path first: Qt never forwards the close to the child.
        self.window.close()
        self.assertTrue(service.isClosed())
        self.assertEqual(self.closes, [(7, True, True)])

        # The widget path on top of it: the closed service is the guard, so the
        # epoch, the history and the notice receiver are untouched a second time.
        view.close()
        self.window.close()
        self.assertEqual(service.epoch(), live + 1)
        self.assertEqual(self.closes, [(7, True, True)])
        self.assertIsNone(view._undoHistory)
        self.assertIsNone(view._noticeHandler.getStage())
        self.assertEqual(self.api.post_close_reads, [])
        self.assertEqual(self.api.api_reads, reads)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class DeltaRecordsTest(unittest.TestCase):
    """The published trace: frozen, slotted, always stamped with an epoch."""

    def test_every_record_is_frozen_slotted_and_stamped(self) -> None:
        records = [
            stage_sync.BeginReload(7, "frame"),
            stage_sync.Load(7, object(), "blueprint", "/Bp"),
            stage_sync.ClearHistory(7),
            stage_sync.Remap(7, (("/A", "/B"),)),
            stage_sync.Remove(7, ("/A",)),
            stage_sync.Adopt(7, ("/B",)),
            stage_sync.Invalidate(7, ("/A",), True, False, True),
            stage_sync.FinishReload(7),
        ]
        for record in records:
            with self.subTest(record=type(record).__name__):
                self.assertEqual(record.epoch, 7)
                self.assertFalse(hasattr(record, "__dict__"))
                with self.assertRaises(dataclasses.FrozenInstanceError):
                    record.epoch = 8


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
        module = sys.modules["UsdNoodles.graphView"]
        self.assertTrue(
            module.__file__.replace("\\", "/").endswith("graphView.py"),
            "UsdNoodles.graphView bound %s" % module.__file__,
        )
        self.assertIsNot(
            module, sys.modules.get("UsdNoodles.graphview")
        )


if __name__ == "__main__":
    unittest.main()
