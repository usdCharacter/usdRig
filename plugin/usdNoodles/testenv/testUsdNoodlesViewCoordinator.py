# Copyright (c) Meta Platforms, Inc. and affiliates.
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.

"""The composed view's host and owner-thread lifecycle, without showing GL."""
import os
import threading
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pxr import Usd
from pxr.Usdviewq.qt import QtWidgets
from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview import stage_adapter
from UsdNoodles.nodeGraph import NodeGraph


class TestViewCoordinator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.view = GraphView()
        self.addCleanup(self.view.deleteLater)
        self.addCleanup(stage_adapter.closeStageLifecycle, self.view)

    def test_replacement_notifies_once_and_allows_resubscribe(self):
        old = self.view.nodeGraph
        replacements, stages = [], []

        def reattach():
            graph = self.view.nodeGraph
            replacements.append(graph)
            old.removeStageChangedCallback(stages.append)
            graph.addStageChangedCallback(stages.append)

        old.addStageChangedCallback(stages.append)
        self.view.nodeGraphReplaced.connect(reattach)
        new = NodeGraph()
        self.view.nodeGraph = new
        self.view.nodeGraph = new
        self.assertEqual(replacements, [new])
        old.setStage(Usd.Stage.CreateInMemory())
        stage = Usd.Stage.CreateInMemory()
        new.setStage(stage)
        self.assertEqual(stages, [stage])

    def test_close_cancels_timers_and_revokes_notices(self):
        seen = []
        stage = Usd.Stage.CreateInMemory()
        self.view._noticeHandler.register(stage)
        self.view._afterUsdviewRefresh(lambda: seen.append('timer'))
        self.view._tooltipTimer.start()
        stage_adapter.closeStageLifecycle(self.view)
        stage_adapter.closeStageLifecycle(self.view)
        self.app.processEvents()
        self.assertEqual(seen, [])
        self.assertFalse(self.view._tooltipTimer.isActive())
        self.assertIsNone(self.view._noticeHandler.getStage())

    def test_worker_notice_reaches_owner_and_pending_notice_dies_on_close(self):
        stage = Usd.Stage.CreateInMemory()
        self.view._noticeHandler.register(stage)
        seen = []

        def send(path):
            worker = threading.Thread(target=stage.DefinePrim, args=(path,))
            worker.start()
            worker.join()

        with patch.object(stage_adapter, 'applyUsdChanges',
                          side_effect=lambda *_a, **_kw: seen.append(threading.get_ident())):
            send('/First')
            self.assertEqual(seen, [])
            self.app.processEvents()
            self.assertEqual(seen, [threading.get_ident()])
            send('/Second')
            stage_adapter.closeStageLifecycle(self.view)
            self.app.processEvents()
            self.assertEqual(seen, [threading.get_ident()])

    def test_headless_driver_uses_canonical_class(self):
        from UsdNoodles.graphview.view import GraphView as CanonicalGraphView
        from headless.harness import GLNotAvailableError, NoodlesTestHarness
        from UsdNoodles.graphview.resources import GLContextNotCurrentError

        self.assertIs(GraphView, CanonicalGraphView)
        driver = NoodlesTestHarness()
        self.addCleanup(driver.teardown)
        try:
            driver.setup()
        except (GLNotAvailableError, GLContextNotCurrentError) as error:
            if driver.graph_view is not None:
                self.assertIs(type(driver.graph_view), CanonicalGraphView)
            self.skipTest(str(error))
        self.assertIs(type(driver.graph_view), CanonicalGraphView)
