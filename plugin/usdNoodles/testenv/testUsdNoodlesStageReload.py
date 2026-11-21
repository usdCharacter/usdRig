#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Test the stage lifecycle and the host's GL teardown guards."""

import os
from pathlib import Path
from threading import Event, Thread, get_ident
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, call, patch

from pxr import Gf, Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtWidgets
from UsdNoodles.undoHistory import history_for_stage
from UsdNoodles.usdNoticeHandler import UsdNoticeHandler
from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview.stage_sync import FinishReload
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
        self.assertTrue(view._renderCacheClearPending)
        view._clearPendingRenderCache()
        view.makeCurrent.assert_not_called()
        view._nodeRenderManager.clearCache.assert_not_called()
        self.assertTrue(view._renderCacheClearPending)
        with patch.object(view, '_hasCurrentRenderContext', return_value=True):
            view._nodeRenderManager.clearCache.side_effect = RuntimeError('clear failed')
            with self.assertRaisesRegex(RuntimeError, 'clear failed'):
                view._clearPendingRenderCache()
            view._clearPendingRenderCache()
            view._nodeRenderManager.clearCache.assert_called_once()
        self.assertFalse(view._renderCacheClearPending)

    def test_reload_clears_with_current_context_before_enabling_updates(self):
        view = self.view
        view.loadStage(self.stage)
        view._onStageReplaced()
        view._nodeRenderManager.clearCache.assert_not_called()
        view.makeCurrent.reset_mock()
        view.doneCurrent.reset_mock()

        def clear_cache():
            self.assertFalse(view.updatesEnabled())
            self.assertFalse(view._renderCacheClearPending)

        view._nodeRenderManager.clearCache.side_effect = clear_cache
        with (patch.object(view, 'context', return_value=Mock(isValid=lambda: True)),
              patch.object(view, '_hasCurrentRenderContext', return_value=True)):
            self.posts.pop()()
        view.makeCurrent.assert_called_once()
        view._nodeRenderManager.clearCache.assert_called_once()
        view.doneCurrent.assert_called_once()
        self.assertTrue(view.updatesEnabled())

    def test_posted_finish_resumes_after_cache_clear_failure_without_retry(self):
        view = self.view
        view.loadStage(self.stage)
        view._onStageReplaced()
        self.assertFalse(view.updatesEnabled())
        self.assertTrue(view._renderCacheClearPending)
        view.doneCurrent.reset_mock()
        clear = view._nodeRenderManager.clearCache
        clear.assert_not_called()
        clear.side_effect = RuntimeError('native cache deletion failed')

        with (patch.object(view, 'context', return_value=Mock(isValid=lambda: True)),
              patch.object(view, '_hasCurrentRenderContext', return_value=True),
              patch('UsdNoodles.graphview.stage_adapter.Tf.Warn') as warn):
            self.posts.pop()()
            warn.assert_called_once()
            self.assertIn('native cache deletion failed', warn.call_args.args[0])
            view.doneCurrent.assert_called_once()
            self.assertTrue(view.initialized)
            self.assertTrue(view.updatesEnabled())
            self.assertFalse(view._renderCacheClearPending)
            view._clearPendingRenderCache()
        clear.assert_called_once()

    def _queueReplacement(self):
        view = self.view
        self.effects = Mock()
        self.effects.attach_mock(view.update, 'update')
        self.effects.attach_mock(view.makeCurrent, 'makeCurrent')
        self.effects.attach_mock(view.doneCurrent, 'doneCurrent')
        self.effects.attach_mock(view._nodeRenderManager.clearCache, 'clearCache')
        setUpdatesEnabled = self.enterContext(patch.object(
            view, 'setUpdatesEnabled', wraps=view.setUpdatesEnabled))
        self.effects.attach_mock(setUpdatesEnabled, 'setUpdatesEnabled')

        def record(delta):
            if isinstance(delta, FinishReload):
                self.effects.finish(delta)

        self.enterContext(patch.object(view._stageSync, '_sink', record))
        view.loadStage(Usd.Stage.CreateInMemory())
        self.assertFalse(self.posts)
        view._onStageReplaced()
        self.assertEqual(len(self.posts), 1)
        self.assertIs(view._stageSync.boundStage(), self.stage)
        return view._stageSync.epoch()

    def _takePendingFinishes(self, replacedEpoch):
        view = self.view
        self.assertEqual(view._stageSync.epoch(), replacedEpoch + 1)
        self.assertEqual(len(self.posts), 2)
        self.assertFalse(view.updatesEnabled())
        self.assertFalse(view.initialized)
        self.assertTrue(view._renderCacheClearPending)
        self.assertFalse(view._deferRenderCacheClear)
        self.effects.setUpdatesEnabled.assert_called_once_with(False)
        self.effects.makeCurrent.assert_not_called()
        self.effects.clearCache.assert_not_called()
        self.effects.doneCurrent.assert_not_called()
        self.effects.finish.assert_not_called()
        return self.posts.pop(0), self.posts.pop(0)

    def _assertOnlySurvivingFinishPaints(self, replacedEpoch):
        view = self.view
        stale, surviving = self._takePendingFinishes(replacedEpoch)
        self.effects.reset_mock()
        stale()
        self.assertEqual(self.effects.mock_calls, [])
        self.assertFalse(view.updatesEnabled())
        self.assertFalse(view.initialized)
        self.assertTrue(view._renderCacheClearPending)

        surviving()
        self.assertEqual(self.effects.mock_calls, [
            call.finish(FinishReload(replacedEpoch + 1)),
            call.setUpdatesEnabled(True), call.update(),
        ])
        self.assertTrue(view.updatesEnabled())
        self.assertTrue(view.initialized)
        self.assertTrue(view._renderCacheClearPending)
        self.assertFalse(self.posts)
        self.effects.reset_mock()
        surviving()
        self.assertEqual(self.effects.mock_calls, [])

    def test_pending_finish_moves_to_different_stage(self):
        replacedEpoch = self._queueReplacement()
        directStage = Usd.Stage.CreateInMemory()
        self.view.loadStage(directStage)
        self.assertIs(self.view.nodeGraph.getStage(), directStage)
        self.assertIs(self.view._stageSync.boundStage(), directStage)
        self._assertOnlySurvivingFinishPaints(replacedEpoch)

    def test_pending_finish_moves_to_same_stage_blueprint(self):
        self.stage.DefinePrim('/Scope', 'Blueprint')
        replacedEpoch = self._queueReplacement()
        clearRenderCache = self.view._clearRenderCache

        def clearWhileDeferred():
            self.assertTrue(self.view._deferRenderCacheClear)
            clearRenderCache()

        with patch.object(self.view, '_clearRenderCache', side_effect=clearWhileDeferred):
            self.view.loadBlueprint(self.stage, '/Scope')
        self.assertIsInstance(self.view.nodeGraph, NodeGraphBlueprint)
        self.assertEqual(self.view._currentPrimPath, '/Scope')
        self.assertIs(self.view.nodeGraph.getStage(), self.stage)
        self.assertIs(self.view._stageSync.boundStage(), self.stage)
        self._assertOnlySurvivingFinishPaints(replacedEpoch)

    def test_failed_direct_load_takes_over_pending_finish(self):
        self.stage.DefinePrim('/Scope', 'Container')
        replacedEpoch = self._queueReplacement()
        with (patch.object(self.view, '_seedNodeZOrders',
                           side_effect=RuntimeError('load failed')),
              self.assertRaisesRegex(RuntimeError, 'load failed')):
            self.view.loadContainer(self.stage, '/Scope')
        self.assertUnbound()
        self._assertOnlySurvivingFinishPaints(replacedEpoch)

    def test_direct_null_load_takes_over_pending_finish(self):
        replacedEpoch = self._queueReplacement()
        self.view.loadStage(None)
        self.assertUnbound()
        self._assertOnlySurvivingFinishPaints(replacedEpoch)

    def test_sample_load_takes_over_pending_finish_without_binding(self):
        replacedEpoch = self._queueReplacement()
        with TemporaryDirectory() as directory:
            sample = Path(directory) / 'empty.json'
            sample.write_text('{"nodes": []}', encoding='utf-8')
            self.view._loadSampleGraph(str(sample))
        stage = self.view.nodeGraph.getStage()
        self.assertIsNotNone(stage)
        self.assertIsNot(stage, self.stage)
        self.assertIsNone(self.view._stageSync.boundStage())
        self._assertOnlySurvivingFinishPaints(replacedEpoch)

    def test_close_cancels_both_pending_finishes(self):
        replacedEpoch = self._queueReplacement()
        directStage = Usd.Stage.CreateInMemory()
        self.view.loadStage(directStage)
        self.assertIs(self.view._stageSync.boundStage(), directStage)
        callbacks = self._takePendingFinishes(replacedEpoch)
        self.view.close()
        self.assertTrue(self.view._stageSync.isClosed())
        self.assertIsNone(self.view._stageSync.boundStage())
        self.assertEqual(self.view._stageSync.epoch(), replacedEpoch + 2)
        self.effects.reset_mock()
        for callback in callbacks:
            callback()
            self.assertEqual(self.effects.mock_calls, [])
            self.assertFalse(self.view.updatesEnabled())
            self.assertFalse(self.view.initialized)
        self.assertFalse(self.posts)

    def test_reentrant_graph_signal_sees_committed_stage_and_sender(self):
        view = self.view
        other = Usd.Stage.CreateInMemory()
        seen = []

        def replaced():
            stage = view.nodeGraph.getStage()
            self.assertIs(view._stageSync.boundStage(), stage)
            self.assertIs(view._undoHistory, history_for_stage(stage))
            seen.append(stage)
            if stage is other:
                view.loadStage(self.stage)

        # Qt swallows exceptions from Python slots; check each observation outside.
        view.nodeGraphReplaced.connect(replaced)
        view.loadStage(other)
        self.assertEqual(seen, [other, self.stage])
        other.DefinePrim('/Retired', 'Xform')
        self.assertNotIn('/Retired', view.nodes)
        self.stage.DefinePrim('/Live', 'Xform')
        self.assertIn('/Live', view.nodes)

    def test_interrupted_load_failure_does_not_reset_newer_graph(self):
        view = self.view
        populate = view._populateStage
        other = Usd.Stage.CreateInMemory()

        def interrupt(stage):
            if stage is other:
                view.loadStage(self.stage)
                raise RuntimeError('interrupted')
            populate(stage)

        with patch.object(view, '_populateStage', side_effect=interrupt):
            with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                view.loadStage(other)
        self.assertIs(view.nodeGraph.getStage(), self.stage)
        self.assertIs(view._stageSync.boundStage(), self.stage)
        self.assertIs(view._undoHistory, history_for_stage(self.stage))

    def test_stale_startup_retry_cannot_reinstall_replaced_stage(self):
        view = self.view
        manager = NoodlesEditorManager()
        startup = Usd.Stage.CreateInMemory()
        manager._usdviewApi = SimpleNamespace(stage=startup)
        window = SimpleNamespace(graphView=view, _updateWindowTitle=Mock())
        view.initialized = False
        manager._loadStageInWindow(window)
        self.posts.pop(0)()
        retry = self.posts.pop(0)
        manager._usdviewApi.stage = self.stage
        view._onStageReplaced()
        self.posts.pop(0)()
        self.assertTrue(view.initialized)
        retry()
        self.assertIs(view.nodeGraph.getStage(), self.stage)
        self.assertIs(view._stageSync.boundStage(), self.stage)
        window._updateWindowTitle.assert_not_called()
        self.assertFalse(self.posts)

    def test_ready_request_is_superseded_or_closed(self):
        manager = NoodlesEditorManager()
        first, second = Mock(), Mock()
        window = SimpleNamespace(graphView=self.view)
        self.assertTrue(self.view._startupSamplePending)
        manager._whenGraphReady(window, first)
        manager._whenGraphReady(window, second)
        self.assertFalse(self.view._startupSamplePending)
        self.view.initialized = True
        self.posts.pop(0)()
        self.posts.pop(0)()
        first.assert_not_called()
        second.assert_called_once_with(self.view)
        manager._whenGraphReady(window, first)
        self.view.close()
        self.posts.pop(0)()
        first.assert_not_called()

    def test_final_gesture_release_delivers_info_and_resync(self):
        self.stage.DefinePrim('/Old', 'Xform').CreateAttribute(
            'testValue', Sdf.ValueTypeNames.Float).Set(1.0)
        self.view.loadStage(self.stage)
        sync = self.view._stageSync
        outer = sync.acquireSuppression()
        gesture = sync.acquireSuppression(reconcile=True)
        with patch('UsdNoodles.graphview.stage_adapter.applyUsdChanges') as reconcile:
            self.stage.GetAttributeAtPath('/Old.testValue').Set(2.0)
            self.stage.DefinePrim('/New', 'Xform')
            gesture.release()
            reconcile.assert_not_called()
            outer.release()
            reconcile.assert_called_once()
            _, resync, info = reconcile.call_args.args
            self.assertEqual(resync, {'/New'})
            self.assertEqual(info, {'/Old.testValue'})

    def test_worker_notice_selected_before_release(self):
        for action in ('deliver', 'rebind', 'replace', 'close', 'scoped'):
            with self.subTest(action=action):
                view = GraphView()
                self.addCleanup(view.close)
                stage = Usd.Stage.CreateInMemory()
                attr = stage.DefinePrim('/Value').CreateAttribute(
                    'number', Sdf.ValueTypeNames.Float)
                sync = view._stageSync
                sync.bind(stage)
                token = sync.acquireSuppression(reconcile=action != 'scoped')
                selected, resume = Event(), Event()
                writes, reconciliations = [], []

                def readMoves(notice):
                    # Path extraction is downstream of callback selection.
                    selected.set()
                    if action != 'scoped' and not resume.wait(5):
                        raise AssertionError('Owner did not resume notice delivery')
                    return {}

                def write():
                    writes.append(attr.Set(2.0))

                def reconcile(_view, resync, info, **kwargs):
                    reconciliations.append((get_ident(), resync, info))

                with patch.object(UsdNoticeHandler, '_movedPaths', staticmethod(readMoves)), \
                     patch('UsdNoodles.graphview.stage_adapter.applyUsdChanges',
                           side_effect=reconcile):
                    worker = Thread(target=write, daemon=True)
                    worker.start()
                    try:
                        if action == 'scoped':
                            worker.join(5)
                            self.assertFalse(worker.is_alive())
                        else:
                            self.assertTrue(selected.wait(5))
                        token.release()
                        self.assertEqual(reconciliations, [])
                        if action == 'rebind':
                            # Same stage and epoch, but a different registration.
                            sync.bind(stage)
                    finally:
                        resume.set()
                        worker.join(5)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(writes, [True])
                    self.assertEqual(reconciliations, [])
                    if action == 'replace':
                        replacement = Usd.Stage.CreateInMemory()
                        sync.loadGraph(replacement, lambda: None, lambda: None)
                        self.assertIs(sync.boundStage(), replacement)
                    elif action == 'close':
                        view.close()
                        self.assertTrue(sync.isClosed())
                    self.app.processEvents()
                    expected = ([(get_ident(), set(), {'/Value.number'})]
                                if action == 'deliver' else [])
                    self.assertEqual(reconciliations, expected)

    def test_final_release_follows_parent_move_then_child_reparent(self):
        stage, view = self.stage, self.view
        prim = stage.DefinePrim('/A/Child', 'Xform')
        view.loadStage(stage)
        self.assertTrue(view._addPrimAsNode(prim, stage, []))
        node = view.nodes['/A/Child']
        node.setDisplayPosition(Gf.Vec2d(500, 300))
        view._selection.update("replace", {'/A/Child'}, ())
        token = view._stageSync.acquireSuppression(reconcile=True)
        self.addCleanup(token.release)
        editor = Usd.NamespaceEditor(stage)
        for source, destination in (('/A', '/B'), ('/B/Child', '/D')):
            self.assertTrue(editor.MovePrimAtPath(source, destination))
            self.assertTrue(editor.ApplyEdits())
        self.assertIs(view.nodes['/A/Child'], node)
        token.release()
        rebuilt = view.nodes['/D']
        self.assertIsNot(rebuilt, node)
        self.assertEqual(rebuilt.getUsdPrim(), stage.GetPrimAtPath('/D'))
        self.assertEqual(view._selectedNodes, {'/D'})
        self.assertEqual(tuple(rebuilt.position), (500.0, 300.0))
        self.assertNotIn('/A/Child', view.nodes)
        self.assertNotIn('/B/Child', view.nodes)

    def test_first_preview_can_be_canceled_by_synchronous_notice(self):
        for action in ('switch', 'close'):
            with self.subTest(action=action):
                view = self.view
                stage = Usd.Stage.CreateInMemory()
                attr = stage.DefinePrim('/Value').CreateAttribute('number', Sdf.ValueTypeNames.Float)
                attr.Set(2.5)
                replacement = Usd.Stage.CreateInMemory()
                replacement_attr = replacement.DefinePrim('/Value').CreateAttribute(
                    'number', Sdf.ValueTypeNames.Float)
                replacement_attr.Set(9.0)
                view.loadStage(stage)
                history = view._undoHistory
                hit = SimpleNamespace(
                    row=SimpleNamespace(editable=True, kind='numeric', mungable=True,
                                        components=(2.5,), type_name='float'),
                    comp=0, nodeId='/Value', primPath='/Value', propertyName='number',
                )
                view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))
                gesture = view._mungState
                observed = []

                def during_write(notice, sender):
                    if observed:
                        return
                    observed.append(attr.Get())
                    key.Revoke()
                    if action == 'switch':
                        view.loadStage(replacement)
                    else:
                        view.close()

                key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, during_write, stage)
                self.addCleanup(key.Revoke)
                view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(100, 0),
                                                modifiers=lambda: QtCore.Qt.NoModifier))
                self.assertEqual(observed, [3.5])
                self.assertEqual(stage.GetRootLayer().GetAttributeAtPath(attr.GetPath()).default, 2.5)
                self.assertEqual(gesture['current'], (2.5,))
                self.assertFalse(history.canUndo())
                self.assertEqual(replacement_attr.Get(), 9.0)
                self.assertIsNone(view._mungState)
                self.assertIsNone(view._mungSuppression)
                self.assertIs(view._stageSync.boundStage(), replacement if action == 'switch' else None)

    def test_preview_end_preserves_a_later_external_opinion(self):
        for action, during_notice in (
            ('switch', False), ('close', False), ('release', False),
            ('switch', True), ('close', True), ('release', True),
        ):
            with self.subTest(action=action, during_notice=during_notice):
                view = GraphView()
                self.addCleanup(view.close)
                view._nodeRenderManager = MagicMock()
                stage = Usd.Stage.CreateInMemory()
                attr = stage.DefinePrim('/Value').CreateAttribute('number', Sdf.ValueTypeNames.Float)
                time = Usd.TimeCode(10)
                self.assertTrue(attr.Set(2, time))
                view.loadStage(stage)
                view._currentTimeCode = time
                view._cachedValueWriteMode = 'animation'
                history = view._undoHistory
                hit = SimpleNamespace(
                    row=SimpleNamespace(editable=True, kind='numeric', mungable=True,
                                        components=(2,), type_name='float'),
                    comp=0, nodeId='/Value', primPath='/Value', propertyName='number',
                )
                view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))

                layer = stage.GetRootLayer()
                observed = []

                def external_write():
                    observed.append(layer.QueryTimeSample(attr.GetPath(), 10))
                    self.assertTrue(attr.Set(4, time))

                if during_notice:
                    def on_preview(notice, sender):
                        key.Revoke()
                        external_write()

                    key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_preview, stage)
                    self.addCleanup(key.Revoke)
                view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(100, 0),
                                                modifiers=lambda: QtCore.Qt.NoModifier))
                if not during_notice:
                    external_write()
                self.assertEqual(observed, [3])

                with patch.object(view, '_startValueFlash') as flash:
                    if action == 'switch':
                        view.loadStage(Usd.Stage.CreateInMemory())
                    elif action == 'close':
                        view.close()
                    else:
                        view._finishMung()
                    flash.assert_not_called()
                self.assertEqual(layer.QueryTimeSample(attr.GetPath(), 10), 4)
                self.assertEqual(attr.Get(time), 4)
                self.assertFalse(history.canUndo())
                self.assertFalse(history.canRedo())
                self.assertIsNone(view._mungSuppression)

    def test_refused_first_preview_does_not_record_undo_or_flash(self):
        stage, view = self.stage, self.view
        attr = stage.DefinePrim('/Value').CreateAttribute('number', Sdf.ValueTypeNames.Float)
        attr.Set(2.5)
        view.loadStage(stage)
        hit = SimpleNamespace(
            row=SimpleNamespace(editable=True, kind='numeric', mungable=True,
                                components=(2.5,), type_name='float'),
            comp=0, nodeId='/Value', primPath='/Value', propertyName='number',
        )
        view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))
        layer = stage.GetRootLayer()
        layer.SetPermissionToEdit(False)
        self.addCleanup(layer.SetPermissionToEdit, True)
        with patch.object(view, '_showWarningPopup') as warning, \
                patch.object(view, '_startValueFlash') as flash:
            view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(100, 0),
                                            modifiers=lambda: QtCore.Qt.NoModifier))
            warning.assert_called_once_with('number: edit target is locked')
            self.assertIsNone(view._mungState.get('opinion'))
            self.assertEqual(view._mungState['current'], (2.5,))
            view._finishMung()
            flash.assert_not_called()
        self.assertEqual(attr.Get(), 2.5)
        self.assertFalse(view._undoHistory.canUndo())
        self.assertIsNone(view._mungSuppression)

    def test_canceled_preview_restores_original_mapped_sample(self):
        stage, view = self.stage, self.view
        layer = Sdf.Layer.CreateAnonymous()
        root = stage.GetRootLayer()
        root.subLayerPaths.append(layer.identifier)
        root.subLayerOffsets[0] = Sdf.LayerOffset(10, 2)
        stage.SetEditTarget(stage.GetEditTargetForLocalLayer(layer))
        attr = stage.DefinePrim('/Value').CreateAttribute('number', Sdf.ValueTypeNames.Float)
        time = Usd.TimeCode(20)
        self.assertTrue(attr.Set(2.5, time))
        view.loadStage(stage)
        history = view._undoHistory
        view._currentTimeCode = time
        view._cachedValueWriteMode = 'animation'
        hit = SimpleNamespace(
            row=SimpleNamespace(editable=True, kind='numeric', mungable=True,
                                components=(2.5,), type_name='float'),
            comp=0, nodeId='/Value', primPath='/Value', propertyName='number',
        )
        view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))
        view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(100, 0),
                                        modifiers=lambda: QtCore.Qt.NoModifier))
        self.assertAlmostEqual(layer.QueryTimeSample(attr.GetPath(), 5), 3.5)
        stage.SetEditTarget(root)
        replacement = Usd.Stage.CreateInMemory()
        view.loadStage(replacement)
        self.assertEqual(layer.QueryTimeSample(attr.GetPath(), 5), 2.5)
        self.assertEqual(layer.ListTimeSamplesForPath(attr.GetPath()), [5.0])
        self.assertFalse(history.canUndo())
        self.assertIsNone(view._mungSuppression)
        self.assertIs(view._stageSync.boundStage(), replacement)

    def test_release_records_the_last_owned_preview_for_redo(self):
        stage, view = self.stage, self.view
        attr = stage.DefinePrim('/Value').CreateAttribute('number', Sdf.ValueTypeNames.Float)
        self.assertTrue(attr.Set(2))
        view.loadStage(stage)
        history = view._undoHistory
        hit = SimpleNamespace(
            row=SimpleNamespace(editable=True, kind='numeric', mungable=True,
                                components=(2,), type_name='float'),
            comp=0, nodeId='/Value', primPath='/Value', propertyName='number',
        )
        view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))
        for dx, expected in ((100, 3), (200, 4)):
            view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(dx, 0),
                                            modifiers=lambda: QtCore.Qt.NoModifier))
            self.assertEqual(attr.Get(), expected)
            self.assertFalse(history.canUndo())
        view._finishMung()
        self.assertTrue(history.canUndo())
        history.undo(view)
        self.assertEqual(attr.Get(), 2)
        self.assertFalse(history.canUndo())
        history.redo(view)
        self.assertEqual(attr.Get(), 4)

    def test_cancel_preview_follows_renamed_property_spec(self):
        stage, view = self.stage, self.view
        root = stage.GetRootLayer()
        layer = Sdf.Layer.CreateAnonymous()
        root.subLayerPaths.append(layer.identifier)
        root.subLayerOffsets[0] = Sdf.LayerOffset(10, 2)
        stage.SetEditTarget(stage.GetEditTargetForLocalLayer(layer))
        prim = stage.DefinePrim('/A', 'Xform')
        attr = prim.CreateAttribute('x', Sdf.ValueTypeNames.Float)
        time = Usd.TimeCode(20)
        self.assertTrue(attr.Set(2.5, time))
        view.loadStage(stage)
        self.assertTrue(view._addPrimAsNode(prim, stage, []))
        node = view.nodes['/A']
        history = view._undoHistory
        view._currentTimeCode = time
        view._cachedValueWriteMode = 'animation'
        hit = SimpleNamespace(
            row=SimpleNamespace(editable=True, kind="numeric", mungable=True,
                                components=(2.5,), type_name='float'),
            comp=0, nodeId='/A', primPath='/A', propertyName='x',
        )
        view._onValueCellPressed(hit, SimpleNamespace(position=lambda: QtCore.QPointF()))
        view._updateMung(SimpleNamespace(position=lambda: QtCore.QPointF(100, 0),
                                        modifiers=lambda: QtCore.Qt.NoModifier))
        self.assertEqual(stage.GetAttributeAtPath('/A.x').Get(time), 3.5)
        self.assertEqual(layer.QueryTimeSample('/A.x', 5), 3.5)
        self.assertIsNotNone(view._mungSuppression)

        editor = Usd.NamespaceEditor(stage)
        self.assertTrue(editor.MovePrimAtPath('/A', '/B'))
        self.assertTrue(editor.ApplyEdits())
        self.assertIs(view.nodes['/A'], node)
        self.assertEqual(layer.QueryTimeSample('/B.x', 5), 3.5)
        stage.SetEditTarget(root)
        view._cancelMung()

        self.assertFalse(stage.GetPrimAtPath('/A'))
        rebuilt = view.nodes['/B']
        self.assertIsNot(rebuilt, node)
        self.assertEqual(rebuilt.getUsdPrim(), stage.GetPrimAtPath('/B'))
        self.assertEqual(stage.GetAttributeAtPath('/B.x').Get(time), 2.5)
        self.assertEqual(layer.QueryTimeSample('/B.x', 5), 2.5)
        self.assertEqual(layer.ListTimeSamplesForPath('/B.x'), [5.0])
        self.assertFalse(root.GetAttributeAtPath('/B.x'))
        self.assertIs(view._undoHistory, history)
        self.assertFalse(history.canUndo())
        self.assertFalse(history.canRedo())
        self.assertIsNone(view._mungSuppression)

        with patch.object(rebuilt, 'invalidate_value_row', wraps=rebuilt.invalidate_value_row) as invalidate:
            self.assertTrue(stage.GetAttributeAtPath('/B.x').Set(7.0, time))
            invalidate.assert_called_with('x')

    def test_history_reload_surfaces_missing_scope_without_publishing(self):
        self.stage.DefinePrim('/Scope', 'Blueprint')
        self.view.loadBlueprint(self.stage, '/Scope')
        old = self.view.nodeGraph
        self.stage.RemovePrim('/Scope')
        replaced = Mock()
        self.view.nodeGraphReplaced.connect(replaced)
        with self.assertRaisesRegex(RuntimeError, '/Scope'):
            self.view._reloadCurrentGraph()
        self.assertIs(self.view.nodeGraph, old)
        self.assertIs(self.view._stageSync.boundStage(), self.stage)
        replaced.assert_not_called()

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
        previous.DefinePrim('/Former', 'Xform')
        self.assertNotIn('/Former', view.nodes)
        self.stage.DefinePrim('/Current', 'Xform')
        self.assertIn('/Current', view.nodes)
        self.assertEqual(
            str(view.nodes['/Current'].getUsdPrim().GetPath()), '/Current'
        )


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
