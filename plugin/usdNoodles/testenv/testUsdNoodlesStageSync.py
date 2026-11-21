#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview stage policy and adapter.

The adapter functions run against real in-memory Usd.Stages with a recording
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
    from UsdNoodles.graphview import stage_adapter, stage_sync
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

    ``getUsdPrim`` resolves the model's ``_primPath`` against the live stage,
    so a namespace edit expires exactly the nodes whose paths are gone --
    the expired-prim signal validation drops.
    """
    node = SimpleNamespace(
        id=path,
        name=path.rsplit("/", 1)[-1],
        selected=False,
        _prim=stage.GetPrimAtPath(path),
        _primPath=Sdf.Path(path),
        _position=(0.0, 0.0),
        position=(0.0, 0.0),
        _authoredPosition=False,
        zOrder=0,
        _title_collapsed=False,
        titleCollapsed=False,
        _cachedType="Sphere",
        invalidateCache=MagicMock(),
        invalidate_value_row=MagicMock(),
    )
    node.getUsdPrim = lambda: stage.GetPrimAtPath(node._primPath)
    node.hasAuthoredPosition = lambda: node._authoredPosition

    def _setDisplayPosition(value):
        node._position = value
        node.position = value

    node.setDisplayPosition = _setDisplayPosition
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
    """The view surface stage_adapter reads, with recording hooks.

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
        return stage_adapter.graphScopeParentPath(self)

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


class _RecordingHandler:
    __slots__ = ("events", "enabled")

    def __init__(self, events):
        self.events = events
        self.enabled = True

    def register(self, stage):
        self.events.append(("register", stage))

    def unregister(self):
        self.events.append(("unregister",))

    def isEnabled(self):
        return self.enabled

    def setEnabled(self, enabled):
        self.events.append(("setEnabled", enabled))
        self.enabled = enabled


class _RecordingHooks:
    """Record the service boundary without emulating an editor."""

    __slots__ = (
        "stage_context", "notice_context", "events", "queued", "load_error",
        "receiver", "handler", "sync",
    )

    def __init__(self, node_ids=()):
        self.stage_context = stage_sync.StageContext(node_ids)
        self.notice_context = stage_sync.NoticeContext(False)
        self.events = []
        self.queued = []
        self.load_error = None
        self.handler = _RecordingHandler(self.events)
        self.sync = stage_sync.StageSync(
            self.read, self.notice, self.apply, self.post, self.listen,
            warn=self.warn, sink=self.publish, reconcile=self.reconcile,
        )

    def read(self):
        self.events.append(("read", self.stage_context))
        return self.stage_context

    def notice(self):
        self.events.append(("notice", self.notice_context))
        return self.notice_context

    def apply(self, delta):
        self.events.append(("apply", delta))
        if isinstance(delta, stage_sync.Load) and self.load_error is not None:
            raise self.load_error

    def post(self, callback):
        self.events.append(("post", callback))
        self.queued.append(callback)

    def listen(self, receiver):
        self.events.append(("listen", receiver))
        self.receiver = receiver
        return self.handler

    def warn(self, message):
        self.events.append(("warn", message))

    def publish(self, delta):
        self.events.append(("publish", delta))

    def reconcile(self, resynced, info, trace, moved):
        self.events.append(("reconcile", resynced, info, trace, moved))


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class GraphScopeParentPathTest(unittest.TestCase):
    """The scope rule the reconciliation measures adoption against."""

    def test_a_plain_graph_scopes_to_the_pseudo_root(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        self.assertEqual(stage_adapter.graphScopeParentPath(view), "/")

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_blueprint_graph_scopes_to_its_own_prim(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(stage)
        view._currentPrimPath = Sdf.Path("/Keeper")
        self.assertEqual(stage_adapter.graphScopeParentPath(view), "/Keeper")

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_scope_without_a_prim_reconciles_nothing(self):
        stage = Usd.Stage.CreateInMemory()
        view = _StubView(stage)
        view.nodeGraph = NodeGraphBlueprint()
        view._currentPrimPath = None
        self.assertIsNone(stage_adapter.graphScopeParentPath(view))


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

        removed = stage_adapter.validateNodesAgainstStage(view)

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

        removed = stage_adapter.validateNodesAgainstStage(view)

        self.assertEqual(removed, ["/Alpha", "/Keeper"])
        self.assertEqual(view.nodes, {})
        self.assertEqual(view._selectedNodes, set())
        self.assertEqual(view._marqueeBaseSelectedNodes, set())
        self.assertEqual(view._marqueePrevSelectedNodes, set())
        view._spatialIndex.removeNode.assert_any_call("/Alpha")
        view._spatialIndex.removeNode.assert_any_call("/Keeper")

    def test_a_healthy_graph_is_left_alone(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        self.assertEqual(stage_adapter.validateNodesAgainstStage(view), [])
        self.assertEqual(sorted(view.nodes), ["/Alpha", "/Keeper"])

    def test_synthesized_io_nodes_are_never_dropped(self) -> None:
        """Blueprint I/O nodes carry '#' and have no prim to validate."""
        view = _StubView(self.stage, ["/Alpha"])
        view.nodes["/Bp#input"] = _synthetic_node("/Bp#input")

        self.assertEqual(stage_adapter.validateNodesAgainstStage(view), [])
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

        added = stage_adapter.adoptPrimsNewToScope(view, {"/Renamed"})

        self.assertEqual(added, ["/Renamed"])

    def test_property_paths_out_of_scope_and_already_shown_are_skipped(
        self,
    ) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        view = _StubView(self.stage, ["/Keeper"])

        self.assertEqual(
            stage_adapter.adoptPrimsNewToScope(
                view, {"/Gone", "/Keeper.radius", "/Keeper/Child", "/Keeper"}
            ),
            [],
        )
        view._addNewNodeFast.assert_not_called()

    def test_inactive_prim_is_not_adopted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Hidden")
        self.stage.GetPrimAtPath("/Hidden").SetActive(False)
        view = _StubView(self.stage, ["/Keeper"])

        self.assertEqual(stage_adapter.adoptPrimsNewToScope(view, {"/Hidden"}), [])

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_container_scope_adopts_only_its_own_children(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/Keeper/Child")
        UsdGeom.Sphere.Define(self.stage, "/Elsewhere")
        view = _StubView(self.stage, [], scope="/Keeper")
        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(self.stage)

        added = stage_adapter.adoptPrimsNewToScope(
            view, {"/Keeper/Child", "/Elsewhere"}
        )

        self.assertEqual(added, ["/Keeper/Child"])


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class AdoptRenamedPrimTest(unittest.TestCase):
    """A rename drops the old node and rebuilds from the live prim.

    Node identity always comes from the current stage: the model at the new
    path is built fresh from the prim there, never re-keyed from the old
    one. Only display-only state (a grid placement, the draw slot) is
    carried across, from a snapshot the caller took before the edit.
    """

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

    def _rename(self, oldPath, newPath):
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(oldPath, newPath)
        self.assertTrue(self.stage.GetEditTarget().GetLayer().Apply(edit))

    def test_the_new_node_is_built_from_the_stage_not_rekeyed(self) -> None:
        view = _StubView(self.stage, ["/World/A", "/Zebra"])
        oldNode = view.nodes["/World/A"]
        oldNode.setDisplayPosition((9.0, 9.0))
        oldNode.zOrder = 3
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World/A")
        self._rename("/World/A", "/World/Renamed")
        before = _layer_text(self.stage)

        nodeId = stage_adapter.adoptRenamedPrim(
            view, "/World/A", "/World/Renamed", *placement
        )

        self.assertEqual(nodeId, "/World/Renamed")
        self.assertNotIn("/World/A", view.nodes)
        # The distinguishing assertion: a re-key would hand back the same
        # object under a new id; a stage-derived node is a fresh model.
        self.assertIsNot(view.nodes["/World/Renamed"], oldNode)
        view._addNewNodeFast.assert_called_once()
        adoptedPrim, _stage = view._addNewNodeFast.call_args[0]
        self.assertEqual(str(adoptedPrim.GetPath()), "/World/Renamed")
        node = view.nodes["/World/Renamed"]
        self.assertEqual(node.id, "/World/Renamed")
        self.assertEqual(node._primPath, Sdf.Path("/World/Renamed"))
        self.assertTrue(node.getUsdPrim().IsValid())
        self.assertEqual(tuple(node._position), (9.0, 9.0))
        self.assertEqual(node.zOrder, 3)
        # The namespace edit was the test's; reconciliation adds no opinion
        # of its own.
        self.assertEqual(_layer_text(self.stage), before)

    def test_the_fresh_model_lands_in_the_vacated_draw_slot(self) -> None:
        view = _StubView(self.stage, ["/World", "/World/A", "/Zebra"])
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World/A")
        self._rename("/World/A", "/World/Renamed")

        stage_adapter.adoptRenamedPrim(
            view, "/World/A", "/World/Renamed", *placement
        )

        # Iteration order IS draw order: the adoption appended the fresh
        # model, so it moves back to the slot the rename vacated.
        self.assertEqual(
            list(view.nodes), ["/World", "/World/Renamed", "/Zebra"]
        )

    def test_an_authored_position_is_left_to_usd(self) -> None:
        def _adoptAuthored(prim, _stage):
            node = view._adopt(prim)
            view.nodes[node]._authoredPosition = True
            view.nodes[node].setDisplayPosition((1.0, 2.0))
            return node

        view = _StubView(self.stage, ["/World/A"])
        view._addNewNodeFast = MagicMock(side_effect=_adoptAuthored)
        view.nodes["/World/A"].setDisplayPosition((9.0, 9.0))
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World/A")
        self._rename("/World/A", "/World/Renamed")

        stage_adapter.adoptRenamedPrim(
            view, "/World/A", "/World/Renamed", *placement
        )

        # The rename carries the attribute to the new path; the snapshot is
        # display-only state and must not overwrite it.
        self.assertEqual(
            tuple(view.nodes["/World/Renamed"]._position), (1.0, 2.0)
        )

    def test_an_already_adopted_prim_is_not_added_twice(self) -> None:
        """The notice adopts inside the scope before the authoring site runs;
        the follow-up restores the placement onto that model."""
        view = _StubView(self.stage, ["/World/A", "/Zebra"])
        view.nodes["/World/A"].setDisplayPosition((9.0, 9.0))
        view.nodes["/World/A"].zOrder = 3
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World/A")
        self._rename("/World/A", "/World/Renamed")
        # What the synchronous notice left behind: the old node dropped, the
        # new prim adopted at the back without the display-only placement.
        del view.nodes["/World/A"]
        view._adopt(self.stage.GetPrimAtPath("/World/Renamed"))
        view._addNewNodeFast.reset_mock()

        nodeId = stage_adapter.adoptRenamedPrim(
            view, "/World/A", "/World/Renamed", *placement
        )

        self.assertEqual(nodeId, "/World/Renamed")
        self.assertEqual(list(view.nodes), ["/World/Renamed", "/Zebra"])
        view._addNewNodeFast.assert_not_called()
        node = view.nodes["/World/Renamed"]
        self.assertEqual(tuple(node._position), (9.0, 9.0))
        self.assertEqual(node.zOrder, 3)

    def test_a_dead_destination_drops_the_old_node_and_returns_none(
        self,
    ) -> None:
        view = _StubView(self.stage, ["/World/A", "/Zebra"])
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World/A")
        self.stage.RemovePrim("/World/A")

        nodeId = stage_adapter.adoptRenamedPrim(
            view, "/World/A", "/World/A", *placement
        )

        self.assertIsNone(nodeId)
        self.assertEqual(list(view.nodes), ["/Zebra"])
        view._addNewNodeFast.assert_not_called()

    def test_descendant_nodes_whose_prims_expired_are_dropped(self) -> None:
        """Renaming an ancestor expires every prim under it; only the renamed
        prim itself is adopted, from its live path."""
        view = _StubView(self.stage, ["/World", "/World/A", "/Zebra"])
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World")
        self._rename("/World", "/Moved")

        nodeId = stage_adapter.adoptRenamedPrim(
            view, "/World", "/Moved", *placement
        )

        self.assertEqual(nodeId, "/Moved")
        self.assertEqual(sorted(view.nodes), ["/Moved", "/Zebra"])

    def test_the_invalidation_tail_and_the_spatial_index_sweep(self) -> None:
        view = _StubView(self.stage, ["/World", "/Zebra"])
        placement = stage_adapter.savedRenamePlacement(view.nodes, "/World")
        self._rename("/World", "/Moved")

        stage_adapter.adoptRenamedPrim(view, "/World", "/Moved", *placement)

        view._spatialIndex.removeNode.assert_called_once_with("/World")
        self.assertEqual(
            [c[0] for c in view._hooks.mock_calls],
            [
                "invalidateBackRefs",
                "nodeRenderManager.invalidateAll",
                "resetRenderCaches",
            ],
        )

    def test_saved_placement_reads_the_node_or_nothing(self) -> None:
        view = _StubView(self.stage, ["/World/A"])
        view.nodes["/World/A"].setDisplayPosition((9.0, 9.0))
        view.nodes["/World/A"].zOrder = 3

        self.assertEqual(
            stage_adapter.savedRenamePlacement(view.nodes, "/World/A"),
            ((9.0, 9.0), 3, 0),
        )
        self.assertEqual(
            stage_adapter.savedRenamePlacement(view.nodes, "/World/Missing"),
            (None, None, None),
        )


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

        stage_adapter.applyUsdChanges(view, {"/Alpha"}, set())

        self.assertIn("/Alpha", view.nodes)
        self.assertFalse(view.linksChanged)
        view._hooks.update.assert_not_called()

    def test_a_rename_drops_the_old_node_and_adopts_from_the_stage(
        self,
    ) -> None:
        """The notice path never re-keys: the old node is dropped and the new
        prim adopted, so the model at the new path is built from the live
        stage rather than carried across the edit."""
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        oldNode = view.nodes["/Alpha"]
        edit = Sdf.BatchNamespaceEdit()
        edit.Add("/Alpha", "/Renamed")
        self.stage.GetEditTarget().GetLayer().Apply(edit)
        before = _layer_text(self.stage)

        stage_adapter.applyUsdChanges(view, {"/Alpha", "/Renamed"}, set())

        self.assertEqual(sorted(view.nodes), ["/Keeper", "/Renamed"])
        self.assertIsNot(view.nodes["/Renamed"], oldNode)
        view._addNewNodeFast.assert_called_once()
        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)
        view._hooks.update.assert_called_once()
        self.assertEqual(_layer_text(self.stage), before)

    def test_a_deletion_drops_the_node_and_rebuilds_links(self) -> None:
        view = _StubView(self.stage, ["/Alpha", "/Keeper"])
        self.stage.RemovePrim("/Alpha")
        before = _layer_text(self.stage)

        stage_adapter.applyUsdChanges(view, {"/Alpha"}, set())

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

        stage_adapter.applyUsdChanges(view, set(), {"/Alpha.radius"})

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

        stage_adapter.applyUsdChanges(view, set(), {"/Alpha.inputs:x"})

        probe.assert_not_called()
        self.assertTrue(view.linksChanged)
        view._hooks.clearRenderCache.assert_called_once()

    def test_a_position_change_repaints_only(self) -> None:
        view = _StubView(self.stage, ["/Alpha"])
        stage_adapter.applyUsdChanges(
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

        stage_adapter.applyUsdChanges(
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

        stage_adapter.applyUsdChanges(
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

        stage_adapter.applyUsdChanges(
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

        stage_adapter.applyUsdChanges(view, set(), {"/Alpha"})

        # Survived, and treated as a plain repaint.
        view.nodes["/Alpha"].invalidateCache.assert_called_once()
        self.assertFalse(view.linksChanged)
        self.assertFalse(view.textChanged)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ScopeMoveTest(unittest.TestCase):
    """A namespace move retargets the scope, its pins and displayed nodes.

    The blueprint/container scope is the prim the graph was opened on:
    when it -- or one of its ancestors -- moves, reconciliation follows
    the live scope before adopting children, and the synthetic I/O nodes
    are generated from the destination loader inputs -- stale view-only
    pins are dropped. Displayed ids at or under a classified source are
    rebuilt fresh from their live destinations; a prim the canvas never
    showed is never admitted, and a delete plus an unrelated create
    never retargets.
    """

    def setUp(self) -> None:
        self.stage = Usd.Stage.CreateInMemory()
        for path in ("/World", "/World/Bp", "/World/Bp/A", "/Other"):
            UsdGeom.Sphere.Define(self.stage, path)

    def _scopedView(self, paths, scope="/World/Bp"):
        view = _StubView(self.stage, paths, scope=scope)
        nodes = view.nodeGraph.nodes
        view.nodeGraph = NodeGraphBlueprint()
        view.nodeGraph.setStage(self.stage)
        view.nodeGraph.nodes.update(nodes)
        return view

    def _move(self, old, new):
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(old, new)
        self.assertTrue(self.stage.GetEditTarget().GetLayer().Apply(edit))

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_moved_scope_is_retargeted_before_child_adoption(self) -> None:
        view = self._scopedView(["/World/Bp/A"])
        oldNode = view.nodes["/World/Bp/A"]
        self._move("/World/Bp", "/World/Bp2")
        before = _layer_text(self.stage)

        stage_adapter.applyUsdChanges(
            view,
            {"/World/Bp", "/World/Bp2"},
            set(),
            movedPaths={"/World/Bp": "/World/Bp2"},
        )

        self.assertEqual(str(view._currentPrimPath), "/World/Bp2")
        self.assertEqual(_layer_text(self.stage), before)
        self.assertNotIn("/World/Bp/A", view.nodes)
        self.assertIsNot(view.nodes["/World/Bp2/A"], oldNode)
        # A child created afterwards appears under the new scope.
        UsdGeom.Sphere.Define(self.stage, "/World/Bp2/B")
        stage_adapter.applyUsdChanges(view, {"/World/Bp2/B"}, set())
        self.assertIn("/World/Bp2/B", view.nodes)

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_scope_pins_exist_only_at_the_new_path(self) -> None:
        self.stage.GetPrimAtPath("/World/Bp").CreateAttribute(
            "input:x", Sdf.ValueTypeNames.Float
        ).Set(1.0)
        view = self._scopedView([])
        pin = _synthetic_node("/World/Bp#input:x")
        view.nodes[pin.id] = pin
        view.nodes["/World/Bp#input:stale"] = _synthetic_node(
            "/World/Bp#input:stale"
        )
        keep = _synthetic_node("/Other#input:y")
        view.nodes[keep.id] = keep
        self._move("/World/Bp", "/World/Bp2")

        stage_adapter.applyUsdChanges(
            view,
            {"/World/Bp", "/World/Bp2"},
            set(),
            movedPaths={"/World/Bp": "/World/Bp2"},
        )

        self.assertNotIn("/World/Bp#input:x", view.nodes)
        self.assertNotIn("/World/Bp#input:stale", view.nodes)
        self.assertNotIn("/World/Bp2#input:stale", view.nodes)
        rebuilt = view.nodes["/World/Bp2#input:x"]
        self.assertIsNot(rebuilt, pin)
        self.assertEqual(rebuilt.name, "x")
        self.assertEqual(rebuilt.outputPins, ["x"])
        self.assertIs(view.nodes["/Other#input:y"], keep)

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_an_ancestor_move_retargets_the_scope(self) -> None:
        self.stage.GetPrimAtPath("/World/Bp").CreateAttribute(
            "input:x", Sdf.ValueTypeNames.Float
        ).Set(1.0)
        UsdGeom.Sphere.Define(self.stage, "/World/Bp/Sibling")
        view = self._scopedView(["/World/Bp/A"])
        oldNode = view.nodes["/World/Bp/A"]
        pin = _synthetic_node("/World/Bp#input:x")
        view.nodes[pin.id] = pin
        view.nodes["/World/Bp#input:stale"] = _synthetic_node(
            "/World/Bp#input:stale"
        )
        self._move("/World", "/Elsewhere")

        stage_adapter.applyUsdChanges(
            view,
            {"/World", "/Elsewhere"},
            set(),
            movedPaths={"/World": "/Elsewhere"},
        )

        self.assertEqual(
            str(view._currentPrimPath), "/Elsewhere/Bp"
        )
        self.assertNotIn("/World/Bp#input:x", view.nodes)
        self.assertNotIn("/Elsewhere/Bp#input:stale", view.nodes)
        rebuilt = view.nodes["/Elsewhere/Bp#input:x"]
        self.assertIsNot(rebuilt, pin)
        self.assertEqual(rebuilt.name, "x")
        self.assertEqual(rebuilt.outputPins, ["x"])
        self.assertNotIn("/World/Bp/A", view.nodes)
        self.assertIsNot(view.nodes["/Elsewhere/Bp/A"], oldNode)
        self.assertNotIn("/Elsewhere/Bp/Sibling", view.nodes)

    @unittest.skipUnless(_has_blueprint, "nodeGraphBlueprint not importable")
    def test_a_delete_plus_create_never_retargets_the_scope(self) -> None:
        """Without a USD-classified pair, two resyncs are just a delete
        and an unrelated create: the scope and its pins stay put."""
        view = self._scopedView([])
        pin = _synthetic_node("/World/Bp#input:x")
        view.nodes[pin.id] = pin
        self.stage.RemovePrim("/World/Bp")
        UsdGeom.Sphere.Define(self.stage, "/World/Bp2")

        stage_adapter.applyUsdChanges(
            view, {"/World/Bp", "/World/Bp2"}, set(), movedPaths={}
        )

        self.assertEqual(str(view._currentPrimPath), "/World/Bp")
        self.assertIs(view.nodes["/World/Bp#input:x"], pin)

    def test_a_displayed_move_source_outside_scope_is_rebuilt_fresh(
        self,
    ) -> None:
        UsdGeom.Sphere.Define(self.stage, "/World/Deep")
        UsdGeom.Sphere.Define(self.stage, "/World/Deep/Node")
        view = _StubView(self.stage, ["/World/Deep/Node"])
        oldNode = view.nodes["/World/Deep/Node"]
        oldNode.setDisplayPosition((500.0, 300.0))
        oldNode.zOrder = 7
        self._move("/World/Deep/Node", "/World/Deep/Elsewhere")
        before = _layer_text(self.stage)

        stage_adapter.applyUsdChanges(
            view,
            {"/World/Deep/Node", "/World/Deep/Elsewhere"},
            set(),
            movedPaths={"/World/Deep/Node": "/World/Deep/Elsewhere"},
        )

        self.assertNotIn("/World/Deep/Node", view.nodes)
        rebuilt = view.nodes["/World/Deep/Elsewhere"]
        self.assertIsNot(rebuilt, oldNode)
        self.assertEqual(rebuilt.position, (500.0, 300.0))
        self.assertEqual(rebuilt.zOrder, 7)
        self.assertTrue(view.linksChanged)
        self.assertEqual(_layer_text(self.stage), before)

    def test_an_undisplayed_move_source_is_not_admitted(self) -> None:
        UsdGeom.Sphere.Define(self.stage, "/World/Deep")
        UsdGeom.Sphere.Define(self.stage, "/World/Deep/Node")
        view = _StubView(self.stage, ["/Other"])
        self._move("/World/Deep/Node", "/World/Deep/Elsewhere")

        stage_adapter.applyUsdChanges(
            view,
            {"/World/Deep/Node", "/World/Deep/Elsewhere"},
            set(),
            movedPaths={"/World/Deep/Node": "/World/Deep/Elsewhere"},
        )

        self.assertNotIn("/World/Deep/Node", view.nodes)
        self.assertNotIn("/World/Deep/Elsewhere", view.nodes)
        self.assertEqual(sorted(view.nodes), ["/Other"])
        view._addNewNodeFast.assert_not_called()


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ReplaceStalenessTest(unittest.TestCase):
    """Only the latest generation's deferred work may touch the model."""

    def test_replace_a_b_none_before_any_callback_runs(self) -> None:
        stageA, stageB = object(), object()
        idsA = ("/Alpha", "/Bp#input:drive")
        idsB = ("/Beta",)
        hooks = _RecordingHooks(idsA)
        snapshotA = hooks.stage_context
        sync = hooks.sync

        sync.replace(stageA, "stage", None, "frame-A")
        hooks.stage_context = snapshotB = stage_sync.StageContext(idsB)
        sync.replace(stageB, "blueprint", "/Beta", "frame-B")
        sync.replace(None, "stage", None, "frame-None")

        finishA, finishB, finishNone = hooks.queued
        beginA = stage_sync.BeginReload(1, "frame-A")
        beginB = stage_sync.BeginReload(2, "frame-B")
        beginNone = stage_sync.BeginReload(3, "frame-None")
        loadA = stage_sync.Load(1, stageA, "stage", None)
        loadB = stage_sync.Load(2, stageB, "blueprint", "/Beta")
        clear = stage_sync.ClearHistory(3)
        self.assertEqual(hooks.events, [
            ("listen", hooks.receiver),
            ("read", snapshotA),
            ("publish", beginA), ("apply", beginA),
            ("publish", stage_sync.Remove(1, idsA)),
            ("publish", loadA), ("apply", loadA), ("post", finishA),
            ("read", snapshotB),
            ("publish", beginB), ("apply", beginB),
            ("publish", stage_sync.Remove(2, idsB)),
            ("publish", loadB), ("apply", loadB), ("post", finishB),
            ("read", snapshotB),
            ("publish", beginNone), ("apply", beginNone),
            ("publish", clear), ("apply", clear), ("post", finishNone),
        ])
        self.assertEqual(sync.epoch(), 3)
        self.assertIs(sync.handler(), hooks.handler)

        events = hooks.events[:]
        sync.changed(1, {"/Alpha"}, set())
        sync.changed(2, {"/Beta"}, set())
        finishA()
        finishB()
        self.assertEqual(hooks.events, events)

        finishNone()
        finish = stage_sync.FinishReload(3)
        self.assertEqual(hooks.events, events + [
            ("publish", finish), ("apply", finish),
        ])

    def test_finish_from_a_rebound_stage_is_dropped(self) -> None:
        # Equal but distinct bindings distinguish identity from equality.
        stageA, stageB = [], []
        hooks = _RecordingHooks()
        hooks.sync.replace(stageA, "stage", None, "frame")
        hooks.sync.bind(stageB)
        events = hooks.events[:]
        hooks.queued[0]()
        self.assertEqual(hooks.events, events)

    def test_selection_sync_suppresses_a_live_notice(self) -> None:
        hooks = _RecordingHooks()
        hooks.sync.replace(object(), "stage", None, "frame")
        hooks.events.clear()
        paths = ({"/Alpha"}, {"/Keeper.radius"})
        hooks.notice_context = muted = stage_sync.NoticeContext(True)
        hooks.receiver._handleUsdChanges(*paths)
        self.assertEqual(hooks.events, [("notice", muted)])

        hooks.notice_context = admitted = stage_sync.NoticeContext(False)
        hooks.receiver._handleUsdChanges(*paths)
        self.assertEqual(hooks.events[:2], [("notice", muted), ("notice", admitted)])
        self.assertEqual(len(hooks.events), 3)
        kind, resynced, info, trace, moved = hooks.events[2]
        self.assertEqual(kind, "reconcile")
        for original, received in zip(paths, (resynced, info)):
            self.assertIs(received, original)
        self.assertIsNotNone(trace)
        self.assertIsNone(moved)

    def test_move_pairs_reach_reconciliation_untouched(self) -> None:
        """The receiver carries the notice's own move classification to
        reconciliation without interpreting it."""
        hooks = _RecordingHooks()
        hooks.sync.replace(object(), "stage", None, "frame")
        hooks.events.clear()
        moved = {"/Alpha": "/Renamed"}
        hooks.receiver._handleUsdChanges(
            {"/Alpha", "/Renamed"}, set(), moved
        )
        kind, _, _, _, received = hooks.events[-1]
        self.assertEqual(kind, "reconcile")
        self.assertIs(received, moved)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class LoadFailureTest(unittest.TestCase):
    """A failing load must not strand the editor or the next replace."""

    def test_a_failing_load_warns_and_still_finishes_then_recovers(self) -> None:
        stageA, stageB = object(), object()
        hooks = _RecordingHooks()
        hooks.load_error = RuntimeError("load boom")
        hooks.sync.replace(stageA, "stage", None, "frame-A")

        (finishA,) = hooks.queued
        self.assertEqual(hooks.events[-3:], [
            ("apply", stage_sync.Load(1, stageA, "stage", None)),
            ("warn", "Failed to reload graph after stage replace: load boom"),
            ("post", finishA),
        ])
        finishA()
        self.assertEqual(hooks.events[-2:], [
            ("publish", stage_sync.FinishReload(1)),
            ("apply", stage_sync.FinishReload(1)),
        ])

        hooks.events.clear()
        hooks.load_error = None
        hooks.sync.replace(stageB, "container", "/Box", "frame-B")
        _, finishB = hooks.queued
        finishB()
        begin = stage_sync.BeginReload(2, "frame-B")
        load = stage_sync.Load(2, stageB, "container", "/Box")
        finish = stage_sync.FinishReload(2)
        self.assertEqual(hooks.events, [
            ("read", hooks.stage_context),
            ("publish", begin), ("apply", begin),
            ("publish", stage_sync.Remove(2, ())),
            ("publish", load), ("apply", load), ("post", finishB),
            ("publish", finish), ("apply", finish),
        ])


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ReloadModeTest(unittest.TestCase):
    def test_each_mode_emits_a_successful_load_and_finish(self) -> None:
        for mode, prim_path in (
            ("stage", None),
            ("blueprint", "/Bp"),
            ("container", "/Box"),
        ):
            with self.subTest(mode=mode):
                stage, frame = object(), object()
                hooks = _RecordingHooks()
                hooks.sync.replace(stage, mode, prim_path, frame)
                (callback,) = hooks.queued
                callback()

                effects = [event[1] for event in hooks.events if event[0] == "apply"]
                self.assertEqual(effects, [
                    stage_sync.BeginReload(1, frame),
                    stage_sync.Load(1, stage, mode, prim_path),
                    stage_sync.FinishReload(1),
                ])
                self.assertIs(effects[1].stage, stage)
                self.assertFalse(any(event[0] == "warn" for event in hooks.events))


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class SuppressionTest(unittest.TestCase):
    """Suppression lives on the editor's handler and always restores."""

    def test_suppression_restores_on_the_normal_path(self) -> None:
        hooks = _RecordingHooks()
        handler = hooks.sync.handler()
        self.assertTrue(handler.isEnabled())
        with hooks.sync.suppressed():
            self.assertFalse(handler.isEnabled())
        self.assertTrue(handler.isEnabled())

    def test_suppression_restores_on_exception(self) -> None:
        hooks = _RecordingHooks()
        with self.assertRaisesRegex(RuntimeError, "edit boom"):
            with hooks.sync.suppressed():
                raise RuntimeError("edit boom")
        self.assertTrue(hooks.handler.isEnabled())

    def test_nested_suppression_restores_the_outer_state(self) -> None:
        hooks = _RecordingHooks()
        with hooks.sync.suppressed():
            with self.assertRaisesRegex(RuntimeError, "inner boom"):
                with hooks.sync.suppressed():
                    raise RuntimeError("inner boom")
            self.assertFalse(hooks.handler.isEnabled())
        self.assertTrue(hooks.handler.isEnabled())
        self.assertEqual(hooks.events[1:], [
            ("setEnabled", False), ("setEnabled", False),
            ("setEnabled", False), ("setEnabled", True),
        ])


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class CloseTest(unittest.TestCase):
    """Close revokes the receiver and invalidates anything still queued."""

    def test_close_drops_queued_finishes_and_later_notices(self) -> None:
        hooks = _RecordingHooks()
        stage = object()
        hooks.sync.replace(stage, "stage", None, "frame")
        handler = hooks.sync.handler()
        self.assertIs(handler, hooks.handler)
        self.assertFalse(any(event[0] == "register" for event in hooks.events))
        handler.register(stage)
        live = hooks.sync.epoch()
        hooks.sync.close()
        self.assertTrue(hooks.sync.isClosed())
        self.assertEqual(hooks.sync.epoch(), live + 1)
        self.assertEqual(hooks.events[-2:], [("register", stage), ("unregister",)])
        events = hooks.events[:]
        hooks.queued[0]()
        hooks.receiver._handleUsdChanges({"/Alpha"}, set())
        hooks.sync.changed(live, {"/Alpha"}, set())
        hooks.sync.changed(live + 1, {"/Alpha"}, set())
        self.assertEqual(hooks.events, events)


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
            read=lambda: stage_sync.StageContext(tuple(view.nodes)),
            notice=lambda: stage_sync.NoticeContext(view._syncingSelection),
            apply=lambda delta: None,
            post=lambda callback: None,
            listen=lambda receiver: UsdNoticeHandler(receiver),
            warn=lambda message: None,
            sink=deltas.append,
            reconcile=lambda resynced, info, trace, moved: (
                stage_adapter.applyUsdChanges(
                    view, resynced, info, trace=trace, movedPaths=moved
                )
            ),
        )
        service.bind(stage)
        service.handler().register(stage)

        self._rename(stage)
        self._rename(twin)

        # The same rename, with and without the service listening: the layer
        # texts match, so reconciliation authored no opinion of its own.
        self.assertEqual(_layer_text(stage), _layer_text(twin))
        # And the graph reflects the stage through the real notice: the old
        # node dropped, the new prim adopted.
        self.assertEqual(list(view.nodes), ["/Keeper", "/Renamed"])
        self.assertEqual(
            deltas,
            [
                stage_sync.Remove(0, ("/Alpha",)),
                stage_sync.Adopt(0, ("/Renamed",)),
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
        hooks = _RecordingHooks(("/Alpha",))
        hooks.sync.replace(object(), "stage", None, "frame")
        (callback,) = hooks.queued
        callback()
        hooks.sync.close()
        closed_epoch = hooks.sync.epoch()
        events = hooks.events[:]

        hooks.sync.replace(object(), "blueprint", "/Bp", "late-frame")

        self.assertEqual(hooks.events, events)
        self.assertEqual(hooks.sync.epoch(), closed_epoch)


@unittest.skipUnless(_has_stage_sync, "stage_sync module not available")
class ProductionAdapterTest(unittest.TestCase):
    """The editor's own wiring -- ``_makeStageSync`` -- against a stub view.

    The stubs above hand the service its readers; these cases run the
    readers the shipped editor builds, so a reader that reaches too much of
    the widget, or a policy the widget stops applying, fails here.
    """

    def _view(self, stage, paths, view_class=_StubView):
        """A keyed-only node map plus the host API used by the slot."""
        view = view_class(stage, paths)
        view.nodeGraph.nodes = _KeyedOnlyNodes(view.nodeGraph.nodes)
        view._usdviewApi = SimpleNamespace(
            stage=stage, frame=Usd.TimeCode.Default()
        )
        return view

    def test_snapshot_readers_borrow_only_their_input(self) -> None:
        stage = stage_adapter.readStageContext(SimpleNamespace(nodes={"/Alpha": None}))
        notice = stage_adapter.readNoticeContext(SimpleNamespace(_syncingSelection=True))
        for snapshot, fields, values in (
            (stage, ("node_ids",), (("/Alpha",),)),
            (notice, ("syncing_selection",), (True,)),
        ):
            with self.subTest(snapshot=type(snapshot).__name__):
                self.assertEqual(tuple(field.name for field in dataclasses.fields(snapshot)), fields)
                self.assertEqual(dataclasses.astuple(snapshot), values)
                self.assertFalse(hasattr(snapshot, "__dict__"))
                with self.assertRaises(dataclasses.FrozenInstanceError):
                    setattr(snapshot, fields[0], None)

    def test_load_effect_brackets_the_selected_loader_even_on_failure(self) -> None:
        for mode, loader, args in (
            ("stage", "loadStage", ()),
            ("blueprint", "loadBlueprint", ("/Bp",)),
            ("container", "loadContainer", ("/Box",)),
        ):
            for fail in (False, True):
                with self.subTest(mode=mode, fail=fail):
                    stage = Usd.Stage.CreateInMemory()
                    view = _StubView(stage)
                    view._deferRenderCacheClear = False
                    view.initialized = False
                    calls = []

                    def load(*received):
                        calls.append((received, view._deferRenderCacheClear))
                        view.initialized = True  # Loader may initialize GL.
                        if fail:
                            raise RuntimeError("loader boom")

                    setattr(view, loader, load)
                    delta = stage_sync.Load(1, stage, mode, args[0] if args else None)
                    if fail:
                        with self.assertRaisesRegex(RuntimeError, "loader boom"):
                            stage_adapter.applyStageDelta(view, delta)
                    else:
                        stage_adapter.applyStageDelta(view, delta)
                    self.assertEqual(calls, [((stage, *args), True)])
                    self.assertFalse(view._deferRenderCacheClear)
                    self.assertFalse(view.initialized)

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

        with patch.object(stage_adapter, "applyUsdChanges", capture):
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
                stage_adapter,
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
