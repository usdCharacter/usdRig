#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Run the shipped graph loaders against live Tf senders without painting."""

import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtWidgets
from UsdNoodles._qt_editor import NoodlesLayerEditorWidget
from UsdNoodles.core import FontMetrics
from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview import stage_sync
from UsdNoodles.nodeGraph import NodeGraph
from UsdNoodles.nodeGraphBlueprint import NodeGraphBlueprint
from UsdNoodles.nodeGraphBpContainer import NodeGraphBpContainer
from UsdNoodles.nodeGraphJson import NodeGraphJson
from UsdNoodles.nodeGraphStage import NodeGraphStage
from UsdNoodles.undoHistory import history_for_stage


_BLUEPRINT = Sdf.Path('/Recipe')
_CONTAINER = Sdf.Path('/Recipe/Nested')


def _scoped_stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim(_BLUEPRINT, 'Blueprint')
    stage.DefinePrim(_CONTAINER, 'Container')
    for scope in (_BLUEPRINT, _CONTAINER):
        stage.DefinePrim(scope.AppendChild('Visible'), 'Shader')
    return stage


class LoaderLifecycleTest(unittest.TestCase):
    def setUp(self):
        if sys.platform == 'linux' and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.view = GraphView()
        self.addCleanup(self.view.close)
        self.view.fontAtlas = FontMetrics()
        self.view.fontAtlas.ascender = 0.8
        self.view.fontAtlas.descender = -0.2
        self.view.fontAtlas.lineHeight = 1.2
        self.view.textRenderer = SimpleNamespace(
            calculateTextWidth=lambda text, size: len(text) * size * 0.5,
            resetPositionCaches=Mock(),
        )
        self.view.makeCurrent = Mock()
        self.view.doneCurrent = Mock()
        self.view._nodeRenderManager = Mock(
            wraps=self.view._nodeRenderManager, clearCache=Mock()
        )

        self.host = SimpleNamespace(stage=Usd.Stage.CreateInMemory())
        self.view._usdviewApi = self.host
        active_window = SimpleNamespace(graphView=self.view)
        self.enterContext(patch.object(NoodlesLayerEditorWidget, '_activeNoodlesWindow',
                                       return_value=active_window))
        self.layers = NoodlesLayerEditorWidget(self.host)
        self.addCleanup(self.layers.close)
        self.reconcile = self.enterContext(patch.object(
            stage_sync, 'applyUsdChanges', wraps=stage_sync.applyUsdChanges
        ))
        self.posts = []
        self.enterContext(patch.object(
            QtCore.QTimer, 'singleShot', side_effect=lambda delay, fn: self.posts.append(fn)
        ))

    def assert_graph(self, graph_type, model_stage, bound_stage, scope):
        view = self.view
        self.assertIs(type(view.nodeGraph), graph_type)
        self.assertIs(view.nodeGraph.getStage(), model_stage)
        self.assertIs(view._stageSync.boundStage(), bound_stage)
        self.assertEqual(view._currentPrimPath, scope)
        expected_history = history_for_stage(model_stage)
        self.assertIs(view._undoHistory, expected_history)
        self.assertFalse(view._selectedNodes)
        self.assertFalse(view._selectedLinks)
        self.assertIsNone(view._lastSelectedNode)
        self.assertIsNone(view._valueEditTarget)
        self.assertIsNone(view._dragLinkSourceNode)
        self.assertFalse(any(node.selected for node in view.nodes.values()))
        self.assertIs(self.layers._attachedNodeGraph, view.nodeGraph)
        self.assertIs(self.layers._activeStage(), model_stage)
        displayed = [self.layers._listWidget.item(i).data(QtCore.Qt.UserRole)
                     for i in range(self.layers._listWidget.count())]
        if model_stage is None:
            self.assertEqual(displayed, [])
        else:
            self.assertIn(model_stage.GetRootLayer(), displayed)
            if self.host.stage is not model_stage:
                self.assertNotIn(self.host.stage.GetRootLayer(), displayed)

    def assert_dispatch(self, stage, path, delivered):
        self.reconcile.reset_mock()
        self.assertTrue(stage.DefinePrim(path, 'Shader'))
        if delivered:
            self.reconcile.assert_called_once()
            view, resynced, *_ = self.reconcile.call_args.args
            self.assertIs(view, self.view)
            self.assertEqual(resynced, {str(path)})
            self.assertIs(view.nodeGraph.getStage(), stage)
        else:
            self.reconcile.assert_not_called()

    def select_and_pin(self, node_id):
        view = self.view
        view.nodes[node_id].selected = True
        view._selectedNodes.add(node_id)
        view._lastSelectedNode = node_id
        view._valueEditTarget = (node_id, 'inputs:amount', 0)
        view._dragLinkSourceNode = node_id

    def test_all_loaders_switch_senders_and_scopes(self):
        view = self.view
        first, blueprint, container = Usd.Stage.CreateInMemory(), _scoped_stage(), _scoped_stage()
        first.DefinePrim('/NotOnCanvas', 'Shader')
        view.loadStage(first)
        self.assert_graph(NodeGraphStage, first, first, None)
        self.assertTrue(view.nodeGraph.syncSelectionToPrimTree)
        self.assertFalse(view.nodes)
        self.assert_dispatch(first, '/StageEdit', True)
        self.select_and_pin('/StageEdit')

        view.loadBlueprint(blueprint, _BLUEPRINT, useFieldnames=False)
        self.assert_graph(NodeGraphBlueprint, blueprint, blueprint, _BLUEPRINT)
        self.assertIn('/Recipe/Visible', view.nodes)
        self.assert_dispatch(first, '/RetiredStageEdit', False)
        self.assert_dispatch(blueprint, '/Recipe/BlueprintEdit', True)
        self.assertIn('/Recipe/BlueprintEdit', view.nodes)
        self.select_and_pin('/Recipe/Visible')
        history = view._undoHistory
        epoch = view._stageSync.epoch()

        view.loadContainer(blueprint, _CONTAINER)
        self.assert_graph(NodeGraphBpContainer, blueprint, blueprint, _CONTAINER)
        self.assertIs(view._undoHistory, history)
        self.assertEqual(view._stageSync.epoch(), epoch + 1)
        self.assertIn('/Recipe/Nested/Visible', view.nodes)
        self.assertNotIn('/Recipe/Visible', view.nodes)
        self.assert_dispatch(blueprint, '/Recipe/OutsideContainer', True)
        self.assertNotIn('/Recipe/OutsideContainer', view.nodes)
        self.assert_dispatch(blueprint, '/Recipe/Nested/SameSenderEdit', True)
        self.assertIn('/Recipe/Nested/SameSenderEdit', view.nodes)

        view.loadContainer(container, _CONTAINER)
        self.assert_graph(NodeGraphBpContainer, container, container, _CONTAINER)
        self.assert_dispatch(blueprint, '/Recipe/Nested/RetiredBlueprintEdit', False)
        self.assert_dispatch(container, '/Recipe/Nested/ContainerEdit', True)
        self.assertIn('/Recipe/Nested/ContainerEdit', view.nodes)
        self.select_and_pin('/Recipe/Nested/ContainerEdit')

        with TemporaryDirectory() as directory:
            sample = Path(directory) / 'sample.json'
            sample.write_text('{"nodes": [{"Sample": {"pos": [80, 120]}}]}',
                              encoding='utf-8')
            view._loadSampleGraph(str(sample))
        converted = view.nodeGraph.getStage()
        self.assertIsNotNone(converted)
        self.assert_graph(NodeGraphJson, converted, None, None)
        self.assertIn('/jsonGraph/Sample', view.nodes)
        self.assert_dispatch(container, '/Recipe/Nested/RetiredContainerEdit', False)
        self.assert_dispatch(converted, '/jsonGraph/UnboundSampleEdit', False)
        self.select_and_pin('/jsonGraph/Sample')

        view.loadStage(first)
        self.assert_graph(NodeGraphStage, first, first, None)
        self.assertFalse(view.nodes)
        self.assert_dispatch(converted, '/jsonGraph/RetiredSampleEdit', False)
        self.assert_dispatch(container, '/Recipe/Nested/StillRetired', False)
        self.assert_dispatch(first, '/ReboundStageEdit', True)
        self.assertEqual(set(view.nodes), {'/ReboundStageEdit'})
        self.assertFalse(self.posts)

    def _exercise_failed_switch(self, failure):
        view = self.view
        for replace in (False, True):
            with self.subTest(replace=replace):
                old, new = _scoped_stage(), _scoped_stage()
                self.host.stage = old
                view.loadBlueprint(old, _BLUEPRINT)
                self.select_and_pin('/Recipe/Visible')
                old_layers = (old.GetRootLayer(), old.GetSessionLayer())
                before = [layer.ExportToString() for layer in old_layers]
                view.makeCurrent.reset_mock()
                view.doneCurrent.reset_mock()
                view.textChanged = False
                self.reconcile.reset_mock()
                if replace:
                    self.host.stage = new

                def fail(*args, **kwargs):
                    expected = old if failure == 'populate' else new
                    self.assertIs(view.nodeGraph.getStage(), expected)
                    self.assertIs(view._undoHistory, history_for_stage(expected))
                    self.assertIsNone(view._stageSync.boundStage())
                    if failure != 'populate':
                        self.assertIn('/Recipe/Visible', view.nodes)
                    raise RuntimeError(f'injected {failure}')

                target, method = {
                    'populate': (NodeGraphBlueprint, 'load'),
                    'frame': (view, 'frameAll'),
                    'Tf': (Tf.Notice, 'Register'),
                }[failure]
                with patch.object(target, method, side_effect=fail) as injected:
                    if replace:
                        with patch.object(Tf, 'Warn') as warn:
                            view._onStageReplaced()
                        warn.assert_called_once()
                        self.assertIn(f'injected {failure}', warn.call_args.args[0])
                    else:
                        with self.assertRaisesRegex(RuntimeError, f'injected {failure}'):
                            view.loadBlueprint(new, _BLUEPRINT)
                    injected.assert_called_once()
                self.reconcile.assert_not_called()
                self.assert_graph(NodeGraph, None, None, None)
                self.assertFalse(view.nodes)
                self.assertTrue(view.textChanged)
                self.assertTrue(view._renderCacheClearPending)
                self.assertFalse(view._deferRenderCacheClear)
                view.makeCurrent.assert_not_called()
                self.assertEqual([layer.ExportToString() for layer in old_layers], before)
                self.assert_dispatch(old, '/Recipe/RetiredAfterFailure', False)
                self.assert_dispatch(new, '/Recipe/UncommittedAfterFailure', False)
                if replace:
                    self.assertFalse(view.updatesEnabled())
                    self.assertEqual(len(self.posts), 1)
                    self.posts.pop()()
                    self.assertTrue(view.updatesEnabled())
                    self.assertTrue(view.initialized)
                    self.assertTrue(view._renderCacheClearPending)
                    view.makeCurrent.assert_not_called()
                    view.doneCurrent.assert_not_called()
                else:
                    self.assertTrue(view.updatesEnabled())
                    self.assertFalse(self.posts)

                view.loadBlueprint(new, _BLUEPRINT)
                self.assert_graph(NodeGraphBlueprint, new, new, _BLUEPRINT)
                self.assert_dispatch(old, '/Recipe/StillRetiredAfterRecovery', False)
                self.assert_dispatch(new, '/Recipe/RecoveryEdit', True)
                self.assertIn('/Recipe/RecoveryEdit', view.nodes)

    def test_candidate_load_failure_before_publication(self):
        self._exercise_failed_switch('populate')

    def test_scoped_frame_failure(self):
        self._exercise_failed_switch('frame')

    def test_tf_acquisition_failure(self):
        self._exercise_failed_switch('Tf')


if __name__ == '__main__':
    unittest.main()
