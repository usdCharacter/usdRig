#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Test the stage lifecycle and the host's GL teardown guards."""

import os
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from pxr import Tf, Usd
from pxr.Usdviewq.qt import QtWidgets
from UsdNoodles.graphView import GraphView
from UsdNoodles.nodeGraphBpContainer import NodeGraphBpContainer
from UsdNoodles.nodeGraphBlueprint import NodeGraphBlueprint
from UsdNoodles.nodeGraphStage import NodeGraphStage
from UsdNoodles._qt_editor import NoodlesEditorManager, _makeStageViewCurrent

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
_is_stage_view_mid_teardown = GraphView._isStageViewMidTeardown
_make_stage_view_current = _makeStageViewCurrent
_install_close_stage_guard = NoodlesEditorManager._installCloseStageGuard


class TestReloadLifecycle(unittest.TestCase):
    def setUp(self):
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.view = GraphView()
        self.addCleanup(self.view.close)
        self.view._nodeRenderManager = MagicMock()
        self.view.makeCurrent = MagicMock()
        self.view.doneCurrent = MagicMock()
        self.view.update = MagicMock()
        self.posts = []
        patcher = patch('UsdNoodles.graphView.QtCore.QTimer.singleShot',
                        side_effect=lambda delay, callback: self.posts.append(callback))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.stage = Usd.Stage.CreateInMemory()
        self.view._usdviewApi = SimpleNamespace(stage=self.stage)

    def assertUnbound(self):
        view = self.view
        self.assertIsNone(view.nodeGraph.getStage())
        self.assertIsNone(view._stageSync.boundStage())
        self.assertIsNone(view._currentPrimPath)
        self.assertIsNone(view._undoHistory)
        self.assertFalse(view.nodes)
        self.assertFalse(view._startupSamplePending)

    def test_null_stage_drops_graph_and_recovers_paint(self):
        self.view.loadStage(self.stage)
        self.assertFalse(self.posts)
        self.view._usdviewApi.stage = None
        self.view._onStageReplaced()
        self.assertUnbound()
        self.assertFalse(self.view.updatesEnabled())
        self.assertEqual(len(self.posts), 1)
        self.posts.pop()()
        self.assertTrue(self.view.updatesEnabled())

    def test_replacement_preserves_graph_mode_and_scope(self):
        for loader, prim_type, graph_type in (
            (self.view.loadStage, None, NodeGraphStage),
            (self.view.loadBlueprint, 'Blueprint', NodeGraphBlueprint),
            (self.view.loadContainer, 'Container', NodeGraphBpContainer),
        ):
            with self.subTest(graph_type=graph_type):
                stage = Usd.Stage.CreateInMemory()
                replacement = Usd.Stage.CreateInMemory()
                path = None
                if prim_type:
                    path = '/Scope'
                    stage.DefinePrim(path, prim_type)
                    replacement.DefinePrim(path, prim_type)
                    loader(stage, path)
                else:
                    loader(stage)
                self.assertFalse(self.posts)
                self.view._usdviewApi.stage = replacement
                self.view._onStageReplaced()
                self.assertIsInstance(self.view.nodeGraph, graph_type)
                self.assertEqual(self.view._currentPrimPath, path)
                self.assertIs(self.view.nodeGraph.getStage(), replacement)
                self.assertIs(self.view._stageSync.boundStage(), replacement)
                self.posts.pop()()

    def test_failed_replacement_warns_and_finishes_pending_cache_clear(self):
        view = self.view
        view.loadStage(Usd.Stage.CreateInMemory())
        with (patch.object(view, '_seedNodeZOrders', side_effect=RuntimeError('stage broken')),
              patch('UsdNoodles.graphView.Tf.Warn') as warn):
            view._onStageReplaced()
        self.assertIn('stage broken', warn.call_args.args[0])
        self.assertUnbound()
        self.assertFalse(view.initialized)
        self.assertFalse(view.updatesEnabled())
        self.assertFalse(view._deferRenderCacheClear)
        view.makeCurrent.assert_not_called()
        view._nodeRenderManager.clearCache.assert_not_called()
        self.assertEqual(len(self.posts), 1)
        self.posts.pop()()
        self.assertTrue(view.initialized)
        self.assertTrue(view.updatesEnabled())
        self.assertTrue(view.linksChanged)
        self.assertTrue(view.textChanged)
        self.assertFalse(view._renderCacheClearPending)
        view.makeCurrent.assert_called_once()
        view._nodeRenderManager.clearCache.assert_called_once()
        view.doneCurrent.assert_called_once()

    def test_missing_scoped_prim_leaves_no_graph_or_binding(self):
        for loader in (self.view.loadBlueprint, self.view.loadContainer):
            with self.subTest(loader=loader.__name__):
                self.view.loadStage(self.stage)
                with self.assertRaisesRegex(RuntimeError, '/Missing'):
                    loader(self.stage, '/Missing')
                self.assertUnbound()
                self.assertFalse(self.posts)

    def test_failed_tf_acquisition_leaves_no_graph_and_recovery_receives_notices(self):
        view = self.view
        previous = Usd.Stage.CreateInMemory()
        view.loadStage(previous)
        with patch.object(Tf.Notice, 'Register', side_effect=RuntimeError('Tf failed')):
            with self.assertRaisesRegex(RuntimeError, 'Tf failed'):
                view.loadStage(self.stage)
        self.assertUnbound()
        self.assertFalse(self.posts)
        view.loadStage(self.stage)
        with patch.object(view, '_addNewNodeFast', return_value=None) as adopt:
            previous.DefinePrim('/Former', 'Xform')
            adopt.assert_not_called()
            self.stage.DefinePrim('/Current', 'Xform')
            adopt.assert_called_once()
            self.assertEqual(str(adopt.call_args.args[0].GetPath()), '/Current')


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
