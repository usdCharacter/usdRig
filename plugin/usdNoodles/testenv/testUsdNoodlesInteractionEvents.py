#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct event checks against the staged interaction owner.

Real Qt events go through the live widget: a Delete key press dispatches
the DeleteSelection intent, Tab toggles the creation hotbox, the wheel
zooms (or scrolls a visible hotbox), ShortcutOverride/HoverMove flow
through event(), and a handler exception is caught by the Qt boundary
guard. Shortcut-conflict migration resolves through the moved binding
exactly as before, and the pin-selection menu still builds through the
patched legacy QtWidgets.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pxr.Usdviewq.qt import QtCore, QtGui

import UsdNoodles.graphView as legacy_module
from headless.base import NoodlesHeadlessTestCase
from headless.harness import _configure_offscreen_env
from headless.helpers import GraphTestHelper
from pxr import Usd
from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview import interaction, interaction_events, intents
from UsdNoodles.graphview.interaction import GraphWidgetBase

UNGUARDED = [
    "event",
    "enterEvent",
    "leaveEvent",
    "contextMenuEvent",
    "closeEvent",
    "timerEvent",
    "focusNextPrevChild",
    "_shouldInterceptShortcut",
    "_setupShortcuts",
    "_resolveShortcutConflicts",
    "updateShortcuts",
    "_getContextMenuConf",
    "_initContextMenu",
    "_updateContextMenuState",
    "_showPinSelectionMenu",
    "_updateNodeCreationHotbox",
    "_openSelectedContainer",
    "_openPreferences",
    "_onPreferencesClosed",
    "_showHotkeyPopup",
    "_showPopupMessage",
    "_showWarningPopup",
    "_openTokenPopup",
    "_handleTokenPopupPress",
    "_closeTokenPopup",
    "_onValueCellPressed",
]

GUARDED = ["keyPressEvent", "keyReleaseEvent", "wheelEvent"]


class TestWidgetBaseBindings(unittest.TestCase):
    def test_graph_view_derives_from_widget_base(self):
        from pxr.Usdviewq.qt import QGLWidget

        self.assertTrue(issubclass(GraphView, GraphWidgetBase))
        self.assertIs(GraphWidgetBase.__bases__[0], QGLWidget)

    def test_signal_lives_on_base(self):
        self.assertIs(
            GraphView.nodeGraphReplaced, GraphWidgetBase.nodeGraphReplaced
        )

    def test_event_bindings_are_explicit_assignments(self):
        for name in UNGUARDED:
            with self.subTest(name=name):
                self.assertIs(
                    getattr(GraphView, name), getattr(interaction_events, name)
                )
        for name in GUARDED:
            with self.subTest(name=name):
                bound = getattr(GraphView, name)
                self.assertIs(bound.__wrapped__, getattr(interaction_events, name))
                self.assertEqual(bound.__name__, name)
        slot = GraphWidgetBase.__dict__["_apiSchemaHotboxItems"]
        self.assertIsInstance(slot, staticmethod)
        self.assertIs(slot.__func__, interaction_events._apiSchemaHotboxItems)

    def test_legacy_module_keeps_patch_surface(self):
        for name in (
            "QtWidgets",
            "NoodlesConfig",
            "_push_undo_command",
            "_guardsQtBoundary",
            "collect_links_for_prim",
            "positionWriteGeneration",
            "layout_graph",
            "NodeVertex",
            "warn_if_non_persistent_edit_target",
        ):
            with self.subTest(name=name):
                self.assertTrue(hasattr(legacy_module, name))
        self.assertIs(
            legacy_module._guardsQtBoundary, interaction.guards_qt_boundary
        )


class _FakeSettings:
    """Minimal fake of NoodlesConfig that records set() calls."""

    def __init__(self, values, defaults):
        self._values = dict(values)
        self._defaults = dict(defaults)
        self.set_calls = []

    def get(self, key, fallback=""):
        return self._values.get(key, fallback)

    def set(self, key, value):
        self._values[key] = value
        self.set_calls.append((key, value))

    def getSettingInfo(self, key):
        if key not in self._defaults:
            return None
        return {"name": key, "default": self._defaults[key]}


def _resolve_conflicts(shortcuts, settings):
    entries = [(name, key, False, None, None) for name, key in shortcuts]
    view = SimpleNamespace()
    with patch("UsdNoodles.graphView.NoodlesConfig") as config:
        config.settings.return_value = settings
        if settings is not None:
            config.get.side_effect = settings.get
            config.set.side_effect = settings.set
        GraphView._resolveShortcutConflicts(view, entries)


class TestShortcutConflictResolution(unittest.TestCase):
    def test_conflict_resets_earlier_to_default(self):
        settings = _FakeSettings(
            values={"shortcutToggle": "D", "shortcutRemove": "D"},
            defaults={"shortcutToggle": "G", "shortcutRemove": "D"},
        )
        _resolve_conflicts(
            [("Toggle Link Dimming", "shortcutToggle"),
             ("Remove from Graph", "shortcutRemove")],
            settings,
        )
        self.assertEqual(settings.set_calls, [("shortcutToggle", "G")])

    def test_latest_definition_wins_across_three_entries(self):
        settings = _FakeSettings(
            values={"shortcutA": "D", "shortcutB": "D", "shortcutC": "D"},
            defaults={"shortcutA": "X", "shortcutB": "Y", "shortcutC": "D"},
        )
        _resolve_conflicts(
            [("A", "shortcutA"), ("B", "shortcutB"), ("C", "shortcutC")],
            settings,
        )
        self.assertEqual(
            sorted(settings.set_calls),
            sorted([("shortcutA", "X"), ("shortcutB", "Y")]),
        )


class TestGuardCatchesHandlerException(unittest.TestCase):
    def test_broken_view_drops_event_and_warns(self):
        with patch("UsdNoodles.graphView.Tf.Warn") as warn:
            result = GraphView.keyPressEvent(SimpleNamespace(), MagicMock())
        self.assertIsNone(result)
        warn.assert_called_once()
        self.assertIn("keyPressEvent", warn.call_args[0][0])

    def test_paint_during_teardown_returns_true(self):
        view = SimpleNamespace(
            _isStageViewMidTeardown=MagicMock(return_value=True)
        )
        event = MagicMock()
        event.type.return_value = QtCore.QEvent.Paint
        self.assertTrue(GraphView.event(view, event))


def _make_pin_menu_view(**overrides):
    view = SimpleNamespace(
        _dragLinkSourceNode="src",
        _dragLinkSourcePort="out",
        _reconnectingLink=False,
        lastMousePos=QtCore.QPoint(7, 8),
        mapToGlobal=MagicMock(return_value=QtCore.QPoint(70, 80)),
        _createConnection=MagicMock(),
        _cancelLinkReconnect=MagicMock(),
    )
    for key, value in overrides.items():
        setattr(view, key, value)
    return view


class TestPinSelectionMenu(unittest.TestCase):
    def _trigger(self, view, pin):
        with patch("UsdNoodles.graphView.QtWidgets") as mock_qt:
            GraphView._showPinSelectionMenu(view, "nodeA", ["p1", "p2"], False)
        menu = mock_qt.QMenu.return_value
        mock_qt.QMenu.assert_called_once_with(view)
        menu.popup.assert_called_once_with(QtCore.QPoint(70, 80))
        on_trigger = menu.triggered.connect.call_args[0][0]
        action = MagicMock()
        action.data.return_value = pin
        on_trigger(action)
        return mock_qt

    def test_trigger_connects_drag_source_to_chosen_pin(self):
        view = _make_pin_menu_view()
        self._trigger(view, "p2")
        view._createConnection.assert_called_once_with(
            "src", "out", "nodeA", "p2"
        )
        view._cancelLinkReconnect.assert_not_called()

    def test_identical_reconnect_cancels_instead(self):
        view = _make_pin_menu_view(
            _reconnectingLink=True,
            _reconnectOldSourceNodeId="src",
            _reconnectOldSourcePort="out",
            _reconnectOldTargetNodeId="nodeA",
            _reconnectOldTargetPort="p1",
        )
        self._trigger(view, "p1")
        view._cancelLinkReconnect.assert_called_once_with()
        view._createConnection.assert_not_called()


def _gl_context_available():
    """Whether a throwaway GL context can be created (never crashes).

    The headless harness segfaults inside native GL init when no context
    is available (proven identical on the base commit), so the GL-backed
    class probes first and skips cleanly instead of crashing setup.
    """
    from pxr.Usdviewq.qt import QtGui, QtWidgets

    _configure_offscreen_env()
    if QtWidgets.QApplication.instance() is None:
        QtWidgets.QApplication([])
    surface = QtGui.QOffscreenSurface()
    surface.create()
    if not surface.isValid():
        return False
    return bool(QtGui.QOpenGLContext().create())


class _NoGLWidgetFixture(unittest.TestCase):
    """Live widget without GL init: construction, events and intents.

    Widget construction is GL-free (proven by probe); only rendering
    needs a context, and these tests never paint (Qt posts paint events
    but nothing here runs the event loop). Mirrors the headless base's
    per-test fresh graph so the same bodies run with and without GL.
    """

    graph_view = None
    harness = None
    helper = None

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        _configure_offscreen_env()
        from pxr.Usdviewq.qt import QtWidgets

        if QtWidgets.QApplication.instance() is None:
            QtWidgets.QApplication([])
        cls.graph_view = GraphView(parent=None)
        cls.graph_view.resize(800, 600)
        # GL-created render state does not exist without initializeGL.
        # The load path only resets its caches, so stand in a mock; the
        # event, shortcut, menu and popup paths never touch it.
        cls.graph_view._cppIconRenderer = MagicMock(name="cppIconRenderer")
        cls.harness = SimpleNamespace(graph_view=cls.graph_view)
        cls.helper = GraphTestHelper(cls.harness)

    @classmethod
    def tearDownClass(cls):
        if cls.graph_view is not None:
            cls.graph_view.close()
            cls.graph_view.deleteLater()
            cls.graph_view = None
            cls.harness = None
            cls.helper = None
        super().tearDownClass()

    def setUp(self):
        super().setUp()
        self._stage = Usd.Stage.CreateInMemory()
        self._bp_path = "/TestBP"
        self._stage.DefinePrim(self._bp_path, "Blueprint")
        self.harness.graph_view.loadBlueprint(self._stage, self._bp_path)
        # The load binds the stage's undo history; clear it last so the
        # first test does not clear the pre-load None history.
        self.helper.clear_undo_stack()

    def tearDown(self):
        self._stage = None
        self._bp_path = None
        super().tearDown()

    @property
    def stage(self):
        return self._stage

    @property
    def bp_path(self):
        return self._bp_path


class LiveEventDeliveryTests:
    """Real Qt events through the widget; the intents they dispatch.

    Mixin: the same bodies run against the GL-backed headless harness
    (where a context exists) and the GL-free fixture (everywhere).
    """

    def _reset_input_state(self):
        view = self.harness.graph_view
        view.nodeCreationHotbox.hide()
        view._renamingNodeId = ""
        view._mungState = None
        view._tokenPopup = None
        view._valueEditTarget = None
        view.spacePressed = False
        self.helper.clear_selection()
        return view

    def _recorded_intents(self, view):
        dispatched = []
        original = view._commandRegistry.dispatch

        def recording(intent):
            dispatched.append(intent)
            return original(intent)

        view._commandRegistry.dispatch = recording
        self.addCleanup(
            setattr, view._commandRegistry, "dispatch", original
        )
        return dispatched

    def _key_event(self, event_type, key, modifiers=None, text=""):
        return QtGui.QKeyEvent(
            event_type,
            key,
            QtCore.Qt.NoModifier if modifiers is None else modifiers,
            text,
        )

    def test_delete_key_dispatches_delete_selection_intent(self):
        view = self._reset_input_state()
        node_id = self.helper.add_node(
            self.stage, self.bp_path, "TestNode", (100, 200)
        )
        self.assertIsNotNone(node_id)
        self.helper.select_node(node_id)
        dispatched = self._recorded_intents(view)

        view.keyPressEvent(
            self._key_event(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Delete)
        )

        self.assertEqual(dispatched, [intents.DeleteSelection()])
        self.assertEqual(self.helper.get_node_count(), 0)

    def test_live_guard_catches_exception_and_warns(self):
        view = self._reset_input_state()
        dispatched = self._recorded_intents(view)
        with (
            patch.object(
                view, "_autoLayoutNodes", side_effect=RuntimeError("boom")
            ),
            patch("UsdNoodles.graphView.Tf.Warn") as warn,
        ):
            result = view.keyPressEvent(
                self._key_event(QtCore.QEvent.KeyPress, QtCore.Qt.Key_L)
            )
        self.assertIsNone(result)
        self.assertEqual(dispatched, [])
        warn.assert_called_once()
        self.assertIn("keyPressEvent", warn.call_args[0][0])

    def test_tab_toggles_hotbox_and_routes_keys(self):
        view = self._reset_input_state()
        dispatched = self._recorded_intents(view)
        hotbox = view.nodeCreationHotbox
        view.lastMousePos = QtCore.QPoint(10, 10)

        view.keyPressEvent(
            self._key_event(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Tab)
        )
        self.assertTrue(hotbox.is_showing)

        view.keyPressEvent(
            self._key_event(
                QtCore.QEvent.KeyPress, QtCore.Qt.Key_X, text="x"
            )
        )
        self.assertTrue(hotbox.is_showing)

        view.keyPressEvent(
            self._key_event(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Tab)
        )
        self.assertFalse(hotbox.is_showing)
        self.assertEqual(dispatched, [])

    def test_wheel_zooms_and_hotbox_scroll_takes_over(self):
        view = self._reset_input_state()

        def wheel():
            return QtGui.QWheelEvent(
                QtCore.QPointF(10, 10),
                QtCore.QPointF(100, 100),
                QtCore.QPoint(0, 0),
                QtCore.QPoint(0, 120),
                QtCore.Qt.NoButton,
                QtCore.Qt.NoModifier,
                QtCore.Qt.ScrollUpdate,
                False,
            )

        zoom_before = view.zoom
        view.wheelEvent(wheel())
        self.assertGreater(view.zoom, zoom_before)

        view.nodeCreationHotbox.show(QtCore.QPoint(10, 10))
        # Visibility comes from animator ticks; force it as the
        # scrollbar suite does.
        view.nodeCreationHotbox._alpha.current = 1.0
        zoom_shown = view.zoom
        with patch.object(
            view.nodeCreationHotbox, "handle_scroll_pixels"
        ) as scrolled:
            view.wheelEvent(wheel())
        scrolled.assert_called_once()
        self.assertEqual(view.zoom, zoom_shown)
        view.nodeCreationHotbox.hide()

    def test_context_menu_updates_state(self):
        view = self._reset_input_state()

        def show_menu():
            with patch.object(view._contextMenu, "exec_") as shown:
                view.contextMenuEvent(
                    QtGui.QContextMenuEvent(
                        QtGui.QContextMenuEvent.Mouse,
                        QtCore.QPoint(5, 5),
                        QtCore.QPoint(50, 50),
                    )
                )
            shown.assert_called_once_with(QtCore.QPoint(50, 50))

        show_menu()
        self.assertFalse(view._contextMenuActions["Clear Selection"].isEnabled())
        self.assertFalse(view._contextMenuActions["Open Container"].isEnabled())

        node_id = self.helper.add_node(
            self.stage, self.bp_path, "TestNode", (100, 200)
        )
        self.helper.select_node(node_id)
        show_menu()
        self.assertTrue(view._contextMenuActions["Clear Selection"].isEnabled())

    def test_shortcut_override_and_hover_flow_through_event(self):
        view = self._reset_input_state()
        override = self._key_event(
            QtCore.QEvent.ShortcutOverride, QtCore.Qt.Key_J, text="j"
        )
        self.assertTrue(view.event(override))
        self.assertTrue(override.isAccepted())

        hover = QtGui.QHoverEvent(
            QtCore.QEvent.HoverMove,
            QtCore.QPointF(30, 40),
            QtCore.QPointF(0, 0),
            QtCore.Qt.NoModifier,
            QtGui.QPointingDevice.primaryPointingDevice(),
        )
        result = view.event(hover)
        self.assertIsInstance(result, bool)
        self.assertEqual(view.lastMousePos, QtCore.QPoint(30, 40))

    def test_enter_leave_timer_and_focus_child(self):
        view = self._reset_input_state()
        view.enterEvent(
            QtGui.QEnterEvent(
                QtCore.QPointF(1, 1),
                QtCore.QPointF(1, 1),
                QtCore.QPointF(10, 10),
            )
        )
        view._valueHover = ("node", "prop")
        view.leaveEvent(QtCore.QEvent(QtCore.QEvent.Leave))
        self.assertIsNone(view._valueHover)
        self.assertIsNone(view._valueCursorSet)
        self.assertFalse(view.focusNextPrevChild(True))

        view._showPopupMessage("ping", duration_ms=60000)
        timer_id = view.hotkeyPopup._timer_id
        self.assertIsNotNone(timer_id)
        view.timerEvent(QtCore.QTimerEvent(987654321))
        self.assertEqual(view.hotkeyPopup._alpha.target, 1.0)
        view.timerEvent(QtCore.QTimerEvent(timer_id))
        self.assertEqual(view.hotkeyPopup._alpha.target, 0.0)

    def test_open_preferences_reuses_dialog(self):
        view = self._reset_input_state()
        view._preferencesDlg = None
        with patch(
            "UsdNoodles.graphview.interaction_events.NoodlesPreferences"
        ) as dialog_cls:
            view._openPreferences()
            view._openPreferences()
        dialog_cls.assert_called_once_with(view, view)
        dialog = dialog_cls.return_value
        dialog.finished.connect.assert_called_once_with(
            view._onPreferencesClosed
        )
        self.assertEqual(dialog.show.call_count, 2)
        view._preferencesDlg = None

    def test_show_warning_popup_records_message(self):
        view = self._reset_input_state()
        view._showWarningPopup("careful")
        self.assertEqual(view.hotkeyPopup._message, "careful")

    def test_close_event_runs_lifecycle_and_accepts(self):
        self._reset_input_state()
        extra = GraphView(parent=None)
        self.addCleanup(extra.deleteLater)
        event = QtGui.QCloseEvent()
        with patch(
            "UsdNoodles.graphview.stage_adapter.closeStageLifecycle"
        ) as closed:
            extra.closeEvent(event)
        closed.assert_called_once_with(extra)
        self.assertTrue(event.isAccepted())


class TestLiveEventDelivery(NoodlesHeadlessTestCase, LiveEventDeliveryTests):
    """The live bodies against the GL-backed headless harness."""

    @classmethod
    def setUpClass(cls):
        if not _gl_context_available():
            raise unittest.SkipTest(
                "Skipping GL-backed live tests — no GL context; "
                "TestLiveEventDeliveryNoGL runs the same bodies"
            )
        super().setUpClass()


class TestLiveEventDeliveryNoGL(_NoGLWidgetFixture, LiveEventDeliveryTests):
    """The live bodies against a constructed-but-uninitialized widget."""


if __name__ == "__main__":
    unittest.main()

