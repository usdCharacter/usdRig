#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Undo ownership and replay across graph views, without a GL context."""

import gc
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import weakref

from pxr import Gf, Sdf, Usd
from UsdNoodles.graphView import GraphView, _push_undo_command
from UsdNoodles.undoHistory import history_for_stage, set_history_backend


def _stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Value").CreateAttribute("amount", Sdf.ValueTypeNames.Double).Set(0)
    return stage


def _value(stage):
    return stage.GetAttributeAtPath("/Value.amount").Get()


class _View:
    _cachedValueWriteMode = "default"

    def __init__(self, stage):
        self.nodes = {}
        self.reloads = 0
        self.bind(stage)

    def bind(self, stage):
        self.nodeGraph = SimpleNamespace(getStage=lambda: stage)
        self._initUndoTracking(stage)

    def _initUndoTracking(self, stage):
        GraphView._initUndoTracking(self, stage)

    def edit(self, value):
        stage = self.nodeGraph.getStage()
        old = _value(stage)
        stage.GetAttributeAtPath("/Value.amount").Set(value)
        GraphView._pushValueUndo(self, "/Value", "amount", (old,), (value,),
                                 Usd.TimeCode.Default())

    def _reloadCurrentGraph(self):
        self.reloads += 1


class UndoHistoryTest(unittest.TestCase):
    def test_json_sample_binds_history_to_its_converted_stage(self):
        view = _View(None)
        view.textRenderer = SimpleNamespace(calculateTextWidth=lambda _text: 0)
        view.fontAtlas = None
        with TemporaryDirectory() as directory:
            sample = Path(directory) / "empty.json"
            sample.write_text('{"nodes": []}', encoding="utf-8")
            GraphView._loadSampleGraph(view, str(sample))
        stage = view.nodeGraph.getStage()
        self.assertIsNotNone(stage)
        self.assertIs(view._undoHistory, history_for_stage(stage))
        stage.DefinePrim("/Value").CreateAttribute(
            "amount", Sdf.ValueTypeNames.Double).Set(0)
        view.edit(11)
        GraphView._performUndo(view)
        self.assertEqual(_value(stage), 0)

    def test_graph_without_a_stage_does_not_record_commands(self):
        view = _View(None)
        _push_undo_command("No stage", lambda _view: None, lambda _view: None,
                           view=view)
        self.assertIsNone(view._undoHistory)

    def test_opening_another_stage_preserves_and_routes_history(self):
        a, b = _stage(), _stage()
        av = _View(a)
        av.edit(11)
        bv = _View(b)
        self.assertTrue(av._undoHistory.canUndo())
        self.assertFalse(bv._undoHistory.canUndo())
        bv.edit(22)
        GraphView._performUndo(bv)
        self.assertEqual((_value(a), _value(b)), (11, 0))
        GraphView._performUndo(av)
        self.assertEqual((_value(a), _value(b)), (0, 0))
        GraphView._performRedo(av)
        GraphView._performRedo(bv)
        self.assertEqual((_value(a), _value(b)), (11, 22))

    def test_same_stage_views_share_order_and_survive_origin_rebind(self):
        a, b = _stage(), _stage()
        origin = _View(a)
        origin.edit(11)
        peer = _View(a.GetPrimAtPath("/Value").GetStage())
        self.assertIs(origin._undoHistory, peer._undoHistory)
        peer.edit(22)
        origin.bind(b)
        origin.edit(33)
        GraphView._performUndo(peer)
        self.assertEqual((_value(a), _value(b)), (11, 33))
        GraphView._performUndo(peer)
        self.assertEqual((_value(a), _value(b)), (0, 33))
        origin_ref = weakref.ref(origin)
        del origin
        gc.collect()
        self.assertIsNone(origin_ref())
        GraphView._performRedo(peer)
        self.assertEqual(_value(a), 11)

    def test_stage_identity_includes_distinct_sessions(self):
        a = _stage()
        b = Usd.Stage.Open(a.GetRootLayer(), Sdf.Layer.CreateAnonymous())
        av, bv = _View(a), _View(b)
        self.assertIsNot(av._undoHistory, bv._undoHistory)
        with self.assertRaisesRegex(RuntimeError, "different stage"):
            av._undoHistory.undo(bv)

    def test_last_owner_releases_history_and_commands(self):
        stage = _stage()
        first, second = _View(stage), _View(stage)
        first.edit(11)
        history_ref = weakref.ref(first._undoHistory)
        first.bind(None)
        self.assertTrue(second._undoHistory.canUndo())
        second.bind(None)
        gc.collect()
        self.assertIsNone(history_ref())
        self.assertFalse(history_for_stage(stage).canUndo())

    def test_replay_uses_recorded_edit_target_and_restores_current_target(self):
        stage = _stage()
        view = _View(stage)
        view.edit(11)
        stage.SetEditTarget(stage.GetSessionLayer())
        GraphView._performUndo(view)
        self.assertEqual(_value(stage), 0)
        self.assertEqual(stage.GetEditTarget().GetLayer(), stage.GetSessionLayer())
        self.assertIsNone(stage.GetSessionLayer().GetPropertyAtPath("/Value.amount"))

    def test_failed_callbacks_leave_undo_and_redo_entries_available(self):
        stage = _stage()
        view = _View(stage)
        history = view._undoHistory

        def fail(_view):
            raise RuntimeError("edit refused")

        history.push("failure", fail, fail)
        with self.assertRaisesRegex(RuntimeError, "edit refused"):
            history.undo(view)
        self.assertTrue(history.canUndo())
        self.assertFalse(history.canRedo())
        history.clear()
        history.push("failure on redo", fail, lambda _view: None)
        history.undo(view)
        with self.assertRaisesRegex(RuntimeError, "edit refused"):
            history.redo(view)
        self.assertFalse(history.canUndo())
        self.assertTrue(history.canRedo())

    def test_move_undo_authors_nodes_absent_from_invoking_canvas(self):
        stage = _stage()
        origin, peer = _View(stage), _View(stage)
        GraphView._restoreNodePositions(origin, {"/Value": (2000, 3000)})
        _push_undo_command(
            "Move",
            lambda view: GraphView._restoreNodePositions(view, {"/Value": (2000, 3000)}),
            lambda view: GraphView._restoreNodePositions(view, {"/Value": (0, 0)}),
            view=origin,
        )
        origin.bind(_stage())
        GraphView._performUndo(peer)
        attr = stage.GetAttributeAtPath("/Value.ui:nodegraph:node:pos")
        self.assertEqual(attr.Get(), Gf.Vec2f(0, 0))
        GraphView._performRedo(peer)
        self.assertEqual(attr.Get(), Gf.Vec2f(2, 3))


class _Backend:
    def __init__(self):
        self.calls = []
        self.listeners = []
        self.replaying = False
        self.entries = []

    def Push(self, description, do_func, undo_func, edit_target, view_ref):
        self.calls.append(("push", description, view_ref()))
        self.entries.append((description, do_func, undo_func))

    def Undo(self):
        self.calls.append(("undo",))

    def Redo(self):
        self.calls.append(("redo",))

    def CanUndo(self):
        return bool(self.entries)

    def CanRedo(self):
        return False

    def UndoText(self):
        return self.entries[-1][0] if self.entries else ""

    def RedoText(self):
        return "redo text"

    def AddReloadListener(self, fn):
        self.listeners.append(fn)
        return len(self.listeners)

    def IsReplaying(self):
        return self.replaying


class HistoryBackendTest(unittest.TestCase):
    def tearDown(self):
        set_history_backend(None)

    def test_backend_receives_history_calls(self):
        stage = _stage()
        view = _View(stage)
        backend = _Backend()
        set_history_backend(lambda s: backend if s is stage else None)
        self.assertTrue(view._undoHistory.bridged())
        view.edit(5)
        self.assertEqual(backend.calls[0][0], "push")
        self.assertIs(backend.calls[0][2], view)
        self.assertTrue(view._undoHistory.canUndo())
        self.assertFalse(view._undoHistory.canRedo())
        self.assertEqual(view._undoHistory.undoDescription(), backend.calls[0][1])
        self.assertEqual(view._undoHistory.redoDescription(), "redo text")
        view._undoHistory.undo(view)
        view._undoHistory.redo(view)
        self.assertEqual([c[0] for c in backend.calls], ["push", "undo", "redo"])
        # The backend owns the stack; Noodles' own lists stay empty.
        self.assertEqual(view._undoHistory._undo, [])

    def test_one_reload_listener_per_backend(self):
        stage = _stage()
        view = _View(stage)
        backend = _Backend()
        set_history_backend(lambda s: backend if s is stage else None)
        view.edit(1)
        view.edit(2)
        view._undoHistory.undo(view)
        self.assertEqual(len(backend.listeners), 1)
        backend.listeners[0]()
        # With a QApplication (an earlier test may have made one) the reload
        # runs one event-loop turn later; pump the loop until it has run.
        try:
            from pxr.Usdviewq.qt import QtCore, QtWidgets
            app = QtWidgets.QApplication.instance()
        except Exception:
            app = None
        for _ in range(10):
            if view.reloads or app is None:
                break
            QtCore.QCoreApplication.processEvents()
        self.assertEqual(view.reloads, 1)

    def test_replay_guard_suppresses_notice_handling(self):
        stage = _stage()
        view = _View(stage)
        seen = []
        view._handleUsdChanges = lambda *a, **k: seen.append(a) or "handled"
        backend = _Backend()
        set_history_backend(lambda s: backend if s is stage else None)
        view.edit(1)
        view._handleUsdChanges(1)
        self.assertEqual(seen, [(1,)])
        backend.replaying = True
        self.assertIsNone(view._handleUsdChanges(2))
        self.assertEqual(seen, [(1,)])

    def test_no_backend_uses_own_lists(self):
        stage = _stage()
        view = _View(stage)
        self.assertFalse(view._undoHistory.bridged())
        view.edit(3)
        self.assertTrue(view._undoHistory.canUndo())
        GraphView._performUndo(view)
        self.assertEqual(_value(stage), 0)


if __name__ == "__main__":
    unittest.main()
