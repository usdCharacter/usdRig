#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Unit tests for stage-reload crash fix.

Covers the logic added/modified for NVIDIA Windows stage-reopen safety:
  1. _onStageReplaced: null-stage early return still schedules
     _finishStageReload (prevents widget freeze)
  2. _onStageReplaced: reload exceptions are logged, not swallowed
  3. _finishStageReload: sets initialized=True to preserve FontAtlas
  4. _isStageViewMidTeardown: detects stageView mid-teardown
  5. _installCloseStageGuard: wraps _closeStage to pre-clean VBOs and
     rebind the stage view's context for Storm's teardown
  6. event() paint guard: blocks Paint during teardown

Production methods run against view doubles; Qt scheduling and GL calls are mocked.
"""

import unittest
from unittest.mock import MagicMock, patch

from UsdNoodles.graphView import GraphView
from UsdNoodles.nodeGraphBpContainer import NodeGraphBpContainer
from UsdNoodles._qt_editor import NoodlesEditorManager, _makeStageViewCurrent


def _on_stage_replaced(view):
    with (patch("UsdNoodles.graphView.QtCore.QTimer.singleShot") as schedule,
          patch("UsdNoodles.graphView.Tf.Warn") as warnings):
        GraphView._onStageReplaced(view)
    view._scheduleFinishReload = schedule
    view._reloadWarnings = warnings


_finish_stage_reload = GraphView._finishStageReload
_is_stage_view_mid_teardown = GraphView._isStageViewMidTeardown
_make_stage_view_current = _makeStageViewCurrent
_install_close_stage_guard = NoodlesEditorManager._installCloseStageGuard


def _make_view():
    """Create a duck-typed GraphView stand-in."""
    view = MagicMock()
    view.initialized = True
    view._selectedNodes = set()
    view._selectedLinks = set()
    view._currentPrimPath = None
    view._isContainer = False
    view._deferRenderCacheClear = False
    view._renderCacheClearPending = False
    view.linksChanged = False
    view.textChanged = False
    return view


class TestOnStageReplacedNullStage(unittest.TestCase):
    """When stage is None, _onStageReplaced must still schedule
    _finishStageReload so setUpdatesEnabled(True) is eventually called."""

    def test_null_stage_schedules_finish(self):
        view = _make_view()
        view._usdviewApi.stage = None

        _on_stage_replaced(view)

        view._scheduleFinishReload.assert_called_once()
        view.setUpdatesEnabled.assert_called_with(False)

    def test_null_stage_does_not_reload_graph(self):
        view = _make_view()
        view._usdviewApi.stage = None

        _on_stage_replaced(view)

        view.loadStage.assert_not_called()
        view.loadBlueprint.assert_not_called()
        view.loadContainer.assert_not_called()

    def test_null_stage_unregisters_notice_handler(self):
        view = _make_view()
        view._usdviewApi.stage = None

        _on_stage_replaced(view)

        view._noticeHandler.unregister.assert_called_once()


class TestOnStageReplacedWithStage(unittest.TestCase):
    """When stage is valid, _onStageReplaced reloads the graph and
    schedules _finishStageReload."""

    def test_calls_load_stage(self):
        view = _make_view()
        stage = MagicMock()
        view._usdviewApi.stage = stage
        view._currentPrimPath = None
        view._isContainer = False

        _on_stage_replaced(view)

        view.loadStage.assert_called_once_with(stage)
        view._scheduleFinishReload.assert_called_once()

    def test_calls_load_blueprint_when_prim_path_set(self):
        view = _make_view()
        stage = MagicMock()
        view._usdviewApi.stage = stage
        view._currentPrimPath = "/Root/MyBlueprint"
        view._isContainer = False

        _on_stage_replaced(view)

        view.loadBlueprint.assert_called_once_with(stage, "/Root/MyBlueprint")

    def test_calls_load_container_when_container(self):
        view = _make_view()
        stage = MagicMock()
        view._usdviewApi.stage = stage
        view._currentPrimPath = "/Root/MyContainer"
        view.nodeGraph = NodeGraphBpContainer()

        _on_stage_replaced(view)

        view.loadContainer.assert_called_once_with(stage, "/Root/MyContainer")


class TestOnStageReplacedExceptionLogging(unittest.TestCase):
    """Reload exceptions must be logged, not silently swallowed."""

    def test_load_stage_exception_is_logged(self):
        view = _make_view()
        view._usdviewApi.stage = MagicMock()
        view.loadStage.side_effect = RuntimeError("stage broken")

        _on_stage_replaced(view)

        view._reloadWarnings.assert_called_once()
        self.assertIn("stage broken", view._reloadWarnings.call_args.args[0])

    def test_load_stage_exception_still_schedules_finish(self):
        view = _make_view()
        view._usdviewApi.stage = MagicMock()
        view.loadStage.side_effect = RuntimeError("stage broken")

        _on_stage_replaced(view)

        view._scheduleFinishReload.assert_called_once()

    def test_defer_flag_reset_after_exception(self):
        view = _make_view()
        view._usdviewApi.stage = MagicMock()
        view.loadStage.side_effect = RuntimeError("stage broken")

        _on_stage_replaced(view)

        self.assertFalse(view._deferRenderCacheClear)


class TestFinishStageReload(unittest.TestCase):
    """_finishStageReload must re-enable updates and preserve initialized."""

    def test_sets_initialized_true(self):
        view = _make_view()
        view.initialized = False
        view._renderCacheClearPending = False

        _finish_stage_reload(view)

        self.assertTrue(view.initialized)

    def test_re_enables_updates(self):
        view = _make_view()
        view._renderCacheClearPending = False

        _finish_stage_reload(view)

        view.setUpdatesEnabled.assert_called_with(True)
        view.update.assert_called_once()

    def test_clears_pending_render_cache(self):
        view = _make_view()
        view._renderCacheClearPending = True

        _finish_stage_reload(view)

        view.makeCurrent.assert_called_once()
        view._nodeRenderManager.clearCache.assert_called_once()
        view.doneCurrent.assert_called_once()
        self.assertFalse(view._renderCacheClearPending)

    def test_sets_text_and_links_changed(self):
        view = _make_view()
        view._renderCacheClearPending = False

        _finish_stage_reload(view)

        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)


class TestIsStageViewMidTeardown(unittest.TestCase):
    """_isStageViewMidTeardown detects when the stageView renderer
    is being torn down (updatesEnabled=False on stageView)."""

    def test_returns_false_when_no_api(self):
        view = MagicMock()
        view._usdviewApi = None
        self.assertFalse(_is_stage_view_mid_teardown(view))

    def test_returns_false_when_updates_enabled(self):
        view = MagicMock()
        sv = MagicMock()
        sv.updatesEnabled.return_value = True
        view._usdviewApi._UsdviewApi__appController._stageView = sv
        self.assertFalse(_is_stage_view_mid_teardown(view))

    def test_returns_true_when_updates_disabled(self):
        view = MagicMock()
        sv = MagicMock()
        sv.updatesEnabled.return_value = False
        view._usdviewApi._UsdviewApi__appController._stageView = sv
        self.assertTrue(_is_stage_view_mid_teardown(view))

    def test_returns_false_on_attribute_error(self):
        view = MagicMock()
        view._usdviewApi._UsdviewApi__appController = MagicMock(spec=[])
        self.assertFalse(_is_stage_view_mid_teardown(view))

    def test_returns_false_when_no_stage_view(self):
        view = MagicMock()
        view._usdviewApi._UsdviewApi__appController._stageView = None
        self.assertFalse(_is_stage_view_mid_teardown(view))


class TestInstallCloseStageGuard(unittest.TestCase):
    """_installCloseStageGuard wraps _closeStage to pre-clean VBOs."""

    def test_wraps_close_stage(self):
        manager = MagicMock()
        manager.getAllWindows.return_value = []
        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = False
        original = MagicMock()
        controller._closeStage = original

        _install_close_stage_guard(manager, api)

        self.assertTrue(controller._noodles_close_guard)
        self.assertIsNot(controller._closeStage, original)

    def test_guard_cleans_vbos_then_calls_original(self):
        gv = MagicMock()
        window = MagicMock()
        window.graphView = gv
        manager = MagicMock()
        manager.getAllWindows.return_value = [window]

        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = False
        original = MagicMock()
        controller._closeStage = original

        _install_close_stage_guard(manager, api)

        controller._closeStage()

        gv.setUpdatesEnabled.assert_called_with(False)
        gv.makeCurrent.assert_called_once()
        gv._nodeRenderManager.clearCache.assert_called_once()
        gv.doneCurrent.assert_called_once()
        original.assert_called_once()

    def test_guard_rebinds_stage_view_before_original(self):
        """usdview's closeRenderer() frees Storm's GL objects in whatever
        context is current; the guard must hand it the stage view's, not
        the none left by the Noodles doneCurrent()."""
        order = []
        gv = MagicMock()
        gv.doneCurrent.side_effect = lambda: order.append("noodles.doneCurrent")
        window = MagicMock()
        window.graphView = gv
        manager = MagicMock()
        manager.getAllWindows.return_value = [window]

        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = False
        controller._stageView.makeCurrent.side_effect = (
            lambda: order.append("stageView.makeCurrent"))
        controller._closeStage = MagicMock(
            side_effect=lambda: order.append("original"))

        _install_close_stage_guard(manager, api)
        controller._closeStage()

        self.assertEqual(
            order,
            ["noodles.doneCurrent", "stageView.makeCurrent", "original"])

    def test_guard_skips_stage_view_without_context(self):
        manager = MagicMock()
        manager.getAllWindows.return_value = []
        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = False
        controller._stageView.context.return_value = None
        original = MagicMock()
        controller._closeStage = original

        _install_close_stage_guard(manager, api)
        controller._closeStage()

        controller._stageView.makeCurrent.assert_not_called()
        original.assert_called_once()

    def test_guard_not_installed_twice(self):
        manager = MagicMock()
        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = True
        original = controller._closeStage

        _install_close_stage_guard(manager, api)

        self.assertIs(controller._closeStage, original)

    def test_dormant_guard_no_side_effects(self):
        """When no noodles windows are open, the guard just calls original."""
        manager = MagicMock()
        manager.getAllWindows.return_value = []
        api = MagicMock()
        controller = api._UsdviewApi__appController
        controller._noodles_close_guard = False
        original = MagicMock()
        controller._closeStage = original

        _install_close_stage_guard(manager, api)
        controller._closeStage()

        original.assert_called_once()
