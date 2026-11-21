#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct gesture checks against the staged interaction owner.

A press-move-release sequence through the real gesture functions produces
the intent sequence (drag begin, drag update, commit) with no stage write
until the commit handler runs; a reconnect that ends on empty space
dispatches the disconnect intent; a cancelled drag dispatches nothing and
restores the pre-drag link.
"""

import inspect
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pxr import Gf, Usd

from UsdNoodles.graphView import GraphView, _makeStageSync
from UsdNoodles.graphview import interaction, intents
from UsdNoodles.graphview.selection import Selection
from pxr.Usdviewq.qt import QtCore


MOVED = [
    "_clearValueInteractionState",
    "_updateAnimator",
    "_updatePanAnimation",
    "_updateKeyboardState",
    "_finalizeNodeDrag",
    "_startLinkDrag",
    "_snapLinkDragToPort",
    "_updateLinkDrag",
    "_completeLinkDrag",
    "_startLinkReconnect",
    "_clearReconnectState",
    "_cancelLinkReconnect",
    "_finishReconnectDisconnect",
]

THIN_ENTRIES = [
    "mousePressEvent",
    "mouseMoveEvent",
    "mouseReleaseEvent",
    "mouseDoubleClickEvent",
]

GESTURE_INTENTS = (
    intents.BeginNodeDrag,
    intents.UpdateNodeDrag,
    intents.FinishNodeDrag,
    intents.BeginMarquee,
    intents.UpdateMarquee,
    intents.FinishMarquee,
    intents.Pan,
    intents.Zoom,
)


class _Node:
    """Node double with the production drag protocol (models.py)."""

    def __init__(self, node_id, position=(0.0, 0.0)):
        self.id = node_id
        self.selected = False
        self.zOrder = 0
        self.renderer = None
        self._position = Gf.Vec2d(*position)
        self._dragPosition = None
        self._dragging = False
        self.begin_calls = 0
        self.end_calls = 0
        self.immediate_writes = 0

    @property
    def position(self):
        if self._dragging and self._dragPosition is not None:
            return self._dragPosition
        return self._position

    @position.setter
    def position(self, value):
        if self._dragging:
            self._dragPosition = value
        else:
            self.immediate_writes += 1
            self._position = value

    def beginDrag(self):
        self.begin_calls += 1
        self._dragging = True
        self._dragPosition = self.position

    def endDrag(self):
        self.end_calls += 1
        if self._dragging and self._dragPosition is not None:
            self._dragging = False
            self._position = self._dragPosition
            self._dragPosition = None
        else:
            self._dragging = False
            self._dragPosition = None


class _RecordingRegistry(intents.CommandRegistry):
    """A command registry that logs every dispatched intent."""

    def __init__(self):
        super().__init__()
        self.dispatched = []

    def dispatch(self, intent):
        self.dispatched.append(intent)
        return super().dispatch(intent)


def _make_press(x, y):
    event = MagicMock()
    event.position.return_value.toPoint.return_value = QtCore.QPoint(x, y)
    event.buttons.return_value = QtCore.Qt.LeftButton
    event.modifiers.return_value = QtCore.Qt.KeyboardModifier(0)
    return event


def _make_move(x, y, buttons=None):
    event = MagicMock()
    event.position.return_value.toPoint.return_value = QtCore.QPoint(x, y)
    event.buttons.return_value = (
        QtCore.Qt.LeftButton if buttons is None else buttons
    )
    return event


def _make_node_drag_view(node, registry):
    """View shaped for a full press-move-release node drag on one node."""
    view = SimpleNamespace(
        setFocus=MagicMock(),
        _renamingNodeId="",
        _tokenPopup=None,
        _valueEditTarget=None,
        lastMousePos=None,
        ctrlPressedOnMouseDown=False,
        mousePosOnMouseDown=None,
        _reconcileMovedNodes=MagicMock(),
        nodeCreationHotbox=SimpleNamespace(is_visible=False),
        _cachedShowMinimap=True,
        minimap=SimpleNamespace(
            handleMousePress=MagicMock(return_value=False),
            updateHoverState=MagicMock(),
            handleMouseMove=MagicMock(return_value=False),
            handleMouseRelease=MagicMock(return_value=False),
        ),
        spacePressed=False,
        nodeIdUnderCursor=None,
        nodes={node.id: node},
        _hoveredPort=None,
        _valueCellAtPoint=MagicMock(return_value=None),
        _findArrowheadUnderCursor=MagicMock(return_value=-1),
        _findRowEdgeDragStart=MagicMock(return_value=None),
        _selectedNodes=set(),
        _selectedLinks=set(),
        links=[],
        zoom=1.0,
        panX=0.0,
        panY=0.0,
        _nextZOrder=0,
        textRenderer=SimpleNamespace(patchNodeTextDepth=MagicMock()),
        _nodeRenderManager=SimpleNamespace(markNodeQuadDirty=MagicMock()),
        _cppIconRenderer=SimpleNamespace(markIconsDirty=MagicMock()),
        _updateLinkSelectionFromNodes=MagicMock(),
        _nodeTransformFrame=SimpleNamespace(translate=MagicMock()),
        _applyMovedNodeDerivedState=MagicMock(),
        _mungState=None,
        _updateValueHoverCursor=MagicMock(),
        _updatePropertyTooltip=MagicMock(),
        _updateHoveredPort=MagicMock(return_value=False),
        _draggingLink=False,
        draggingNodes=False,
        marqueeActive=False,
        _stageSync=_makeStageSync(None),
        _syncSelectionToPrimtree=MagicMock(),
        update=MagicMock(),
        _commandRegistry=registry,
        linkRenderer=SimpleNamespace(
            findLinkUnderCursor=MagicMock(return_value=-1),
        ),
        _cachedLinkLineWidth=12.0,
        _linkUnderCursor=-1,
        _spatialIndex=None,
    )
    view._updateNodeUnderCursor = MagicMock(
        side_effect=lambda pos: setattr(view, "nodeIdUnderCursor", node.id)
    )
    view.clearSelection = lambda: GraphView.clearSelection(view)
    view.clearLinkSelection = lambda: GraphView.clearLinkSelection(view)
    view._finalizeNodeDrag = lambda: GraphView._finalizeNodeDrag(view)
    return view


def _register_recording_cores(registry, view):
    """Record each gesture intent, then run the production core for it."""
    registry.register(
        intents.BeginNodeDrag,
        lambda req: interaction.begin_node_drag(view, req.node_id, req.toggle),
    )
    registry.register(
        intents.UpdateNodeDrag,
        lambda req: interaction.move_dragged_nodes(view, req.dx, req.dy),
    )
    registry.register(
        intents.FinishNodeDrag,
        lambda req: GraphView._finalizeNodeDrag(view),
    )


class TestGestureBindings(unittest.TestCase):
    def test_moved_functions_are_bound_by_explicit_assignment(self):
        for name in MOVED:
            with self.subTest(name=name):
                self.assertIs(getattr(GraphView, name), getattr(interaction, name))

    def test_mouse_entries_stay_thin_qt_overrides(self):
        for name in THIN_ENTRIES:
            with self.subTest(name=name):
                entry = getattr(GraphView, name)
                self.assertTrue(inspect.isfunction(entry))
                self.assertEqual(
                    list(inspect.signature(entry).parameters), ["self", "event"]
                )


class TestNodeDragIntentSequence(unittest.TestCase):
    def test_press_move_release_dispatches_begin_update_commit(self):
        node = _Node("/A", (100.0, 200.0))
        registry = _RecordingRegistry()
        view = _make_node_drag_view(node, registry)
        _register_recording_cores(registry, view)
        pushes = []
        with patch(
            "UsdNoodles.graphView._push_undo_command",
            side_effect=lambda *args, **kwargs: pushes.append((args, kwargs)),
        ):
            GraphView.mousePressEvent(view, _make_press(10, 10))
            self.assertEqual(
                registry.dispatched, [intents.BeginNodeDrag("/A", False)]
            )
            self.assertTrue(node.selected)
            self.assertEqual(node.begin_calls, 1)
            self.assertEqual(node.end_calls, 0)
            self.assertEqual(pushes, [])

            GraphView.mouseMoveEvent(view, _make_move(30, 40))
            self.assertEqual(
                registry.dispatched,
                [
                    intents.BeginNodeDrag("/A", False),
                    intents.UpdateNodeDrag(20.0, 30.0),
                ],
            )
            self.assertEqual(node.position, Gf.Vec2d(120.0, 230.0))
            self.assertEqual(node.end_calls, 0)
            self.assertEqual(pushes, [])

            GraphView.mouseReleaseEvent(view, MagicMock())
            self.assertEqual(
                registry.dispatched,
                [
                    intents.BeginNodeDrag("/A", False),
                    intents.UpdateNodeDrag(20.0, 30.0),
                    intents.FinishNodeDrag(),
                ],
            )
            self.assertEqual(node.end_calls, 1)
            self.assertEqual(len(pushes), 1)
            self.assertFalse(view.draggingNodes)
            self.assertEqual(node.position, Gf.Vec2d(120.0, 230.0))
            self.assertEqual(node.immediate_writes, 0)

    def test_fallback_without_registry_matches_production_behavior(self):
        node = _Node("/A", (100.0, 200.0))
        view = _make_node_drag_view(node, None)
        pushes = []
        with patch(
            "UsdNoodles.graphView._push_undo_command",
            side_effect=lambda *args, **kwargs: pushes.append((args, kwargs)),
        ):
            GraphView.mousePressEvent(view, _make_press(10, 10))
            GraphView.mouseMoveEvent(view, _make_move(30, 40))
            GraphView.mouseReleaseEvent(view, MagicMock())
        self.assertTrue(node.selected)
        self.assertEqual(node.begin_calls, 1)
        self.assertEqual(node.end_calls, 1)
        self.assertEqual(len(pushes), 1)
        self.assertFalse(view.draggingNodes)
        self.assertEqual(node.position, Gf.Vec2d(120.0, 230.0))


class TestProductionRegistryWiring(unittest.TestCase):
    def test_registry_routes_begin_node_drag(self):
        node = _Node("/A", (0.0, 0.0))
        view = _make_node_drag_view(node, None)
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.BeginNodeDrag("/A", False))
        self.assertTrue(node.selected)
        self.assertTrue(view.draggingNodes)
        self.assertEqual(node.begin_calls, 1)

    def test_registry_routes_update_node_drag(self):
        node = _Node("/A", (0.0, 0.0))
        node.selected = True
        node.beginDrag()
        view = _make_node_drag_view(node, None)
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.UpdateNodeDrag(5.0, -3.0))
        self.assertEqual(node.position, Gf.Vec2d(5.0, -3.0))
        view._nodeTransformFrame.translate.assert_called_once_with("/A", 5.0, -3.0)

    def test_registry_routes_begin_marquee(self):
        node = _Node("/A")
        view = _make_node_drag_view(node, None)
        view._selection = Selection()
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.BeginMarquee((1.0, 2.0), "replace"))
        self.assertTrue(view.marqueeActive)
        self.assertEqual(view._marqueeMode, "replace")
        self.assertEqual(
            (view.marqueeStart.x(), view.marqueeStart.y()), (1.0, 2.0)
        )
        self.assertEqual(
            (view.marqueeCurrent.x(), view.marqueeCurrent.y()), (1.0, 2.0)
        )

    def test_registry_routes_update_marquee(self):
        node = _Node("/A")
        view = _make_node_drag_view(node, None)
        view.marqueeStart = QtCore.QPointF(0.0, 0.0)
        view._updateMarqueeSelection = MagicMock()
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.UpdateMarquee((9.0, 9.0)))
        self.assertEqual(
            (view.marqueeCurrent.x(), view.marqueeCurrent.y()), (9.0, 9.0)
        )
        view._updateMarqueeSelection.assert_called_once_with()

    def test_registry_routes_finish_marquee(self):
        node = _Node("/A")
        view = _make_node_drag_view(node, None)
        view.marqueeActive = True
        view._updateMarqueeSelection = MagicMock()
        view._syncLinkSelectionToPrimtree = MagicMock()
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.FinishMarquee())
        view._updateMarqueeSelection.assert_called_once_with()
        self.assertFalse(view.marqueeActive)
        self.assertEqual(view._marqueeBase.node_ids, set())

    def test_registry_routes_pan(self):
        node = _Node("/A")
        view = _make_node_drag_view(node, None)
        view.zoom = 2.0
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.Pan(10.0, 20.0))
        self.assertAlmostEqual(view.panX, -5.0)
        self.assertAlmostEqual(view.panY, -10.0)

    def test_registry_routes_zoom(self):
        node = _Node("/A")
        view = _make_node_drag_view(node, None)
        view._zoomAroundPoint = MagicMock()
        registry = GraphView._makeCommandRegistry(view)
        registry.dispatch(intents.Zoom((50.0, 60.0), 2.0))
        view._zoomAroundPoint.assert_called_once_with(2.0, (50.0, 60.0))

    def test_registry_coexists_with_editor_registrations(self):
        from UsdNoodles.graphview import editing

        view = SimpleNamespace(nodeGraph=SimpleNamespace(getStage=lambda: None))
        registry = GraphView._makeCommandRegistry(view)
        editor = editing.editor_for_view(view, registry)
        self.assertIs(editor.registry, registry)


class TestReconnectDisconnectIntent(unittest.TestCase):
    def test_release_on_empty_space_dispatches_disconnect(self):
        stage = Usd.Stage.CreateInMemory()
        view = SimpleNamespace(
            nodeGraph=SimpleNamespace(getStage=lambda: stage),
            links=[],
            nodes={},
            _reconnectingLink=True,
            _reconnectCapture=None,
            _reconnectOldSourceNodeId="/A",
            _reconnectOldSourcePort="out",
            _reconnectOldTargetNodeId="/B",
            _reconnectOldTargetPort="in",
            _reconnectOldSourcePropertyName="",
            _reconnectOldTargetPropertyName="",
            _draggingLink=True,
            _dragLinkValidTarget=False,
            _dragLinkResolvedTarget=None,
            _dragLinkSyntheticTarget=None,
            _hoveredPort=None,
            _dragLinkTempLink=SimpleNamespace(is_relationship_link=False),
            _updateHoveredPort=MagicMock(),
            _updateLinkDrag=MagicMock(),
            nodeCreationHotbox=SimpleNamespace(is_visible=False),
            _mungState=None,
            minimap=SimpleNamespace(handleMouseRelease=MagicMock(return_value=False)),
            update=MagicMock(),
        )
        view._completeLinkDrag = lambda: GraphView._completeLinkDrag(view)
        view._finishReconnectDisconnect = (
            lambda: GraphView._finishReconnectDisconnect(view)
        )
        view._clearReconnectState = lambda: GraphView._clearReconnectState(view)
        calls = []
        with patch(
            "UsdNoodles.graphview.editing._dispatch_connection",
            side_effect=lambda v, i, r=None: calls.append(i) or True,
        ):
            GraphView.mouseReleaseEvent(view, MagicMock())
        self.assertEqual(len(calls), 1)
        intent = calls[0]
        self.assertIsInstance(intent, intents.Disconnect)
        self.assertEqual(intent.description, "Disconnect")
        self.assertEqual(
            intent.endpoints,
            intents.LinkEndpoints("/A", "out", "/B", "in", "", ""),
        )
        self.assertEqual(intent.prior_order, ())
        self.assertFalse(view._reconnectingLink)
        self.assertFalse(view._draggingLink)
        self.assertIsNone(view._reconnectOldSourceNodeId)


class TestCancelledDrag(unittest.TestCase):
    def test_cancelled_drag_dispatches_nothing_and_restores_link(self):
        src = _Node("/A")
        tgt = _Node("/B")
        seen = []
        registry = intents.CommandRegistry()
        for intent_type in GESTURE_INTENTS:
            registry.register(intent_type, seen.append)
        view = SimpleNamespace(
            nodes={"/A": src, "/B": tgt},
            _reconnectingLink=True,
            _reconnectCapture=None,
            _reconnectOldSourceNodeId="/A",
            _reconnectOldSourcePort="out",
            _reconnectOldTargetNodeId="/B",
            _reconnectOldTargetPort="in",
            _reconnectOldSourcePropertyName="",
            _reconnectOldTargetPropertyName="",
            _addLinkToGraph=MagicMock(),
            _commandRegistry=registry,
            update=MagicMock(),
        )
        view._clearReconnectState = lambda: GraphView._clearReconnectState(view)
        GraphView._cancelLinkReconnect(view)
        self.assertEqual(seen, [])
        view._addLinkToGraph.assert_called_once_with(
            "/A",
            "out",
            "/B",
            "in",
            src,
            tgt,
            sourcePropertyName="",
            targetPropertyName="",
        )
        self.assertFalse(view._reconnectingLink)
        self.assertIsNone(view._reconnectOldSourceNodeId)
        self.assertIsNone(view._reconnectOldTargetNodeId)


if __name__ == "__main__":
    unittest.main()
