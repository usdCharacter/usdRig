#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/graphEditorModel.py.

The model's whole host -- stage discovery, the undo stack, edit-target
bracketing, the edit hook's answers, visibility, the frame, canvas
refresh, preview and the commit queue -- arrives as callables, so every
group below runs against an in-memory stage with the canvas injected as
callables: no widget, no usdview, no Qt import.
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Ts, Usd  # noqa: E402

import graphEditorModel as gem  # noqa: E402
import graphModel as gm  # noqa: E402
import rigExecUndo  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def _Spline(points, typeName="double"):
    spline = Ts.Spline(typeName)
    for time, value in points:
        knot = Ts.Knot(typeName=typeName, time=time, value=value,
                       nextInterp=Ts.InterpCurve)
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        spline.SetKnot(knot)
    gm.SetCurveType(spline, gm.CURVE_BEZIER)
    return spline


def _Times(spline):
    return sorted(knot.GetTime() for knot in spline.GetKnots().values())


def _Stage(txPoints, tyPoints=None):
    """Root-layer file state, with the session layer as the edit target:
    the layout usdview gives the editor (edits land in the session)."""
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/Scope/Ctrl")
    tx = prim.CreateAttribute("tx", Sdf.ValueTypeNames.Double)
    tx.SetSpline(_Spline(txPoints))
    if tyPoints is not None:
        ty = prim.CreateAttribute("ty", Sdf.ValueTypeNames.Double)
        ty.SetSpline(_Spline(tyPoints))
    stage.SetEditTarget(
        stage.GetEditTargetForLocalLayer(stage.GetSessionLayer()))
    return stage


def _Refs(*names):
    colors = {"tx": gm.CURVE_RED, "ty": gm.CURVE_GREEN}
    return [gm.CurveRef("/Scope/Ctrl", name, colors[name])
            for name in names]


def _ResolvingSpline(stage, attrName):
    attr = stage.GetAttributeAtPath(
        Sdf.Path("/Scope/Ctrl").AppendProperty(attrName))
    return gm.SplineFor(attr)


def _SessionSpec(stage, attrName):
    return stage.GetSessionLayer().GetAttributeAtPath(
        Sdf.Path("/Scope/Ctrl").AppendProperty(attrName))


class _Host(object):
    """The model host as recording callables."""

    def __init__(self, stage, refs, frame=1024.0, visible=None,
                 undo_stack=None, key_spline=None, begin=None, finish=None):
        self._stage = stage
        self._refs = list(refs)
        self.frame = frame
        self._visible = visible
        self._keySpline = key_spline
        self._begin = begin
        self._finish = finish
        self.stack = (undo_stack if undo_stack is not None
                      else rigExecUndo.UndoStack())
        self.notices = []
        self.redraws = 0
        self.syncs = 0
        self.viewports = 0
        self.queued = []
        self.previewPushes = []
        self.previewEnds = []
        self.republishes = 0
        self.selections = []
        self.docEdits = []
        self.hookEdits = []
        self.begins = []
        self.finishes = []
        self.aborts = []
        self.afters = 0

    def _Recorder(self, log):
        host = self

        class _Scope(object):
            def __enter__(self):
                log.append("enter")

            def __exit__(self, *exc):
                log.append("exit")
                return False

        return _Scope()

    def documentEdit(self):
        return self._Recorder(self.docEdits)

    def editContext(self):
        return self._Recorder(self.hookEdits)

    def beginEdit(self, label, indices, drag):
        if self._begin is not None:
            return self._begin(label, indices, drag)
        token = object()
        self.begins.append((label, list(indices), bool(drag), token))
        return token

    def finishEdit(self, token, changed):
        self.finishes.append((token, bool(changed)))
        if self._finish is not None:
            return self._finish(token, changed)
        return None

    def abortEdit(self, token):
        self.aborts.append(token)

    def keySpline(self, index, authored):
        if self._keySpline is not None:
            return self._keySpline(index, authored)
        return authored

    def model(self):
        return gem.GraphEditorModel(
            stage=lambda: self._stage,
            discover=lambda: list(self._refs),
            undo_stack=self.stack,
            document_edit=self.documentEdit,
            begin_edit=self.beginEdit,
            edit_context=self.editContext,
            finish_edit=self.finishEdit,
            abort_edit=self.abortEdit,
            key_spline=self.keySpline,
            curve_spline=lambda index, authored, edited: (
                edited if edited is not None else authored),
            selection_changed=self.selections.append,
            after_hook=lambda: setattr(self, "afters", self.afters + 1),
            is_visible=(lambda index: True if self._visible is None
                        else index in self._visible),
            frame=lambda: self.frame,
            notify=self.notices.append,
            canvas_update=lambda: setattr(self, "redraws",
                                          self.redraws + 1),
            sync_widgets=lambda: setattr(self, "syncs", self.syncs + 1),
            update_viewport=lambda: setattr(self, "viewports",
                                            self.viewports + 1),
            preview_push=lambda pending, stage: self.previewPushes.append(
                (dict(pending), stage)),
            preview_end=lambda stage, publish: self.previewEnds.append(
                (stage, publish)),
            preview_republish=lambda: setattr(self, "republishes",
                                              self.republishes + 1),
            commit_soon=self.queued.append)


def _Pump(host):
    while host.queued:
        host.queued.pop(0)()


# Cancel versus commit

def TestCancelVsCommit():
    points = ((1001.0, 0.0), (1024.0, 5.0), (1048.0, 2.0))
    stage = _Stage(points)
    host = _Host(stage, _Refs("tx"))
    model = host.model()
    _Check(model.RefreshCurves() == [], "first discovery has no previous")
    model.SelectKeys([("Ctrl.tx", 1024.0)])
    _Check(model.SelectedKeys() == [("Ctrl.tx", 1024.0)],
           "one key selected: %s" % (model.SelectedKeys(),))
    _Check(host.selections == [False], "select reports non-additive")
    txPath = Sdf.Path("/Scope/Ctrl.tx")
    _Check(_SessionSpec(stage, "tx") is None,
           "nothing authored into the session layer yet")

    # A drag previews: the key spline moves, the stage does not.
    _Check(model.BeginKeyDrag() is True, "the drag starts")
    model.DragKeys(2.0, 1.5)
    _Check(model.KeySpline(0).GetKnot(1026.0) is not None,
           "the previewed key spline moved to 1026")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1024.0) is not None
           and _ResolvingSpline(stage, "tx").GetKnot(1026.0) is None,
           "the stage still resolves the key at 1024")
    _Check(_SessionSpec(stage, "tx") is None,
           "a preview authors nothing into the session layer")
    _Check(len(host.previewPushes) == 1
           and set(host.previewPushes[0][0]) == set([txPath])
           and host.previewPushes[0][1] is stage,
           "the drag pushed one pending value for the stage: %s"
           % (host.previewPushes,))

    _Check(model.AbortGesture() is True, "the drag aborts")
    _Check(_Times(_ResolvingSpline(stage, "tx")) == [1001.0, 1024.0, 1048.0],
           "an aborted drag leaves the stage's knots alone")
    _Check(not host.stack.CanUndo(), "an aborted drag pushes no undo step")
    _Check(host.previewEnds == [(stage, True)],
           "the abort ends the preview with a republish")
    _Check(model.SelectedKeys() == [],
           "the moved-away selection is pruned after the abort")

    # The same drag committed authors into the session layer, once.
    _Check(model.BeginKeyDrag() is False,
           "no selection, no drag: the abort dropped it")
    model.SelectKeys([("Ctrl.tx", 1024.0)])
    _Check(model.BeginKeyDrag() is True, "the drag restarts")
    model.DragKeys(4.0, 0.0)
    _Check(model.ReleaseGesture() is True, "the release queues the commit")
    _Check(model.HasPendingCommit(), "the commit is pending")
    _Check(len(host.queued) == 1, "one queued commit")
    _Check(_SessionSpec(stage, "tx") is None
           and not host.stack.CanUndo(),
           "a queued commit authors nothing until it runs")
    _Pump(host)
    _Check(not model.HasPendingCommit(), "the queued commit ran")
    moved = _ResolvingSpline(stage, "tx")
    _Check(moved.GetKnot(1028.0) is not None
           and moved.GetKnot(1024.0) is None
           and _Close(moved.GetKnot(1028.0).GetValue(), 5.0),
           "the key moved four snapped frames at the same value")
    _Check(_SessionSpec(stage, "tx") is not None,
           "the commit authored into the session layer")
    _Check(host.stack.CanUndo()
           and host.stack.UndoText() == "Move Keys",
           "one undo step, labelled: %r" % host.stack.UndoText())
    label, indices, drag, token = host.begins[-1]
    _Check((label, indices, drag) == ("Move Keys", [0], True),
           "the hook bracketed the drag: %r" % (host.begins[-1],))
    _Check(host.finishes[-1] == (token, True),
           "the hook finished the committed drag")
    _Check(host.docEdits == ["enter", "exit"],
           "the commit ran inside the host document edit")
    _Check(model.SelectedKeys() == [("Ctrl.tx", 1028.0)],
           "the moved key stays selected")

    host.stack.Undo()
    _Check(model.RefreshAfterUndo() is True, "the undo re-reads")
    _Check(_Times(_ResolvingSpline(stage, "tx")) == [1001.0, 1024.0, 1048.0],
           "undo puts the key back on 1024")
    _Check(_SessionSpec(stage, "tx") is None,
           "undo removed the session opinion")
    host.stack.Redo()
    _Check(model.RefreshAfterUndo() is True, "the redo re-reads")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1028.0) is not None,
           "redo moves the key to 1028 again")

    _Check(host.redraws > 0 and host.syncs > 0 and host.viewports > 0,
           "canvas, widgets and viewport were refreshed")
    _Check(host.notices == [], "no refusal was reported: %s" % host.notices)


# Ordered key edits

def TestOrderedKeyEdits():
    txPoints = ((1001.0, 0.0), (1048.0, 2.0))
    tyPoints = ((1001.0, 1.0), (1048.0, 3.0))
    stage = _Stage(txPoints, tyPoints)
    host = _Host(stage, _Refs("tx", "ty"), visible=set([0]))
    model = host.model()
    model.RefreshCurves()
    _Check(model.VisibleIndices() == [0], "ty stays hidden")

    _Check(model.InsertKeyAtCurrentFrame() is True,
           "a key lands at the current frame")
    knot = _ResolvingSpline(stage, "tx").GetKnot(1024.0)
    _Check(knot is not None, "tx gained the key at 1024")
    keyed = knot.GetValue()
    _Check(_ResolvingSpline(stage, "ty").GetKnot(1024.0) is None,
           "the hidden curve is not keyed")
    _Check(model.SelectedKeys() == [("Ctrl.tx", 1024.0)],
           "the new key selects itself")
    _Check(host.stack.UndoText() == "Insert Key", "first step labelled")

    _Check(model.BeginKeyDrag() is True, "the new key drags")
    model.DragKeys(4.0, 1.0)
    model.ReleaseGesture()
    _Pump(host)
    moved = _ResolvingSpline(stage, "tx")
    _Check(moved.GetKnot(1028.0) is not None
           and moved.GetKnot(1024.0) is None
           and _Close(moved.GetKnot(1028.0).GetValue(), keyed + 1.0),
           "the drag moved the key four frames up by one")
    _Check(_ResolvingSpline(stage, "ty").GetKnot(1028.0) is None,
           "the hidden curve is not dragged")
    _Check(host.stack.UndoText() == "Move Keys", "second step labelled")

    _Check(model.DeleteSelectedKeys() is True, "the key deletes")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1028.0) is None,
           "tx lost the key")
    _Check(model.SelectedKeys() == [], "a delete clears the selection")
    _Check(host.stack.UndoText() == "Delete Keys", "third step labelled")

    host.stack.Undo()
    model.RefreshAfterUndo()
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1028.0) is not None,
           "undo restores the deleted key at 1028")
    _Check(host.stack.UndoText() == "Move Keys", "steps unwind in order")
    host.stack.Undo()
    model.RefreshAfterUndo()
    back = _ResolvingSpline(stage, "tx")
    _Check(back.GetKnot(1024.0) is not None
           and back.GetKnot(1028.0) is None,
           "undo puts the key back on 1024")
    _Check(host.stack.UndoText() == "Insert Key", "steps unwind in order")
    host.stack.Undo()
    model.RefreshAfterUndo()
    _Check(_Times(_ResolvingSpline(stage, "tx")) == [1001.0, 1048.0],
           "undo removes the inserted key")
    _Check(host.stack.Undo() is False, "the stack is spent")
    _Check(_Times(_ResolvingSpline(stage, "ty")) == [1001.0, 1048.0],
           "ty never changed: %s"
           % (_Times(_ResolvingSpline(stage, "ty")),))

    host.stack.Redo()
    model.RefreshAfterUndo()
    host.stack.Redo()
    model.RefreshAfterUndo()
    host.stack.Redo()
    model.RefreshAfterUndo()
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1028.0) is None
           and _Times(_ResolvingSpline(stage, "tx")) == [1001.0, 1048.0],
           "redo replays insert, move and delete in order")


# Tangent break/unify lifecycle

def _Samples(spline, times):
    return [spline.Eval(time) for time in times]


def TestTangentBreakUnify():
    points = ((1001.0, 0.0), (1024.0, 5.0), (1048.0, 2.0))
    stage = _Stage(points)
    host = _Host(stage, _Refs("tx"))
    model = host.model()
    model.RefreshCurves()
    model.SelectKeys([("Ctrl.tx", 1024.0)])
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(gm.IsUnified(knot), "an automatic key starts unified")
    samples = _Samples(model.KeySpline(0),
                       [1001.0, 1010.0, 1024.0, 1040.0, 1048.0])

    _Check(model.Break() is True, "break applies")
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(not gm.IsUnified(knot), "the key is broken")
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmCustom
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "both sides are authored now")
    for before, after in zip(
            samples,
            _Samples(model.KeySpline(0),
                     [1001.0, 1010.0, 1024.0, 1040.0, 1048.0])):
        _Check(_Close(before, after),
               "breaking a tangent does not bend the curve")
    pre0 = knot.GetPreTanSlope()
    post0 = knot.GetPostTanSlope()
    _Check(host.stack.UndoText() == "Break Tangents", "break is one step")

    _Check(model.TangentDragWeighted(0) is True,
           "a Bezier drag authors widths while Weighted is on")
    _Check(model.BeginTangentDrag(0, 1024.0, gm.SIDE_OUT, 1.0) is True,
           "the out handle drag starts")
    model.DragTangent(0.5, 2.0)
    _Check(model.AbortGesture() is True, "the tangent drag aborts")
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(_Close(knot.GetPostTanSlope(), post0),
           "an aborted tangent drag bends nothing")

    _Check(model.BeginTangentDrag(0, 1024.0, gm.SIDE_OUT, 1.0) is True,
           "the out handle drag restarts")
    model.DragTangent(0.5, 2.0)
    model.ReleaseGesture()
    _Pump(host)
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(_Close(knot.GetPostTanSlope(), 0.5)
           and _Close(knot.GetPostTanWidth(), 2.0),
           "the out handle took the dragged slope and width")
    _Check(_Close(knot.GetPreTanSlope(), pre0),
           "the broken in side kept its own slope")
    _Check(not gm.IsUnified(knot), "the key stays broken")
    _Check(host.stack.UndoText() == "Edit Tangent",
           "the tangent drag is one step")

    _Check(model.Unify() is True, "unify applies")
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(gm.IsUnified(knot)
           and _Close(knot.GetPreTanSlope(), 0.5)
           and _Close(knot.GetPostTanSlope(), 0.5),
           "the out slope wins on both sides")
    _Check(_Close(knot.GetPostTanWidth(), 2.0),
           "unify keeps the dragged width")

    host.stack.Undo()
    model.RefreshAfterUndo()
    knot = model.KeySpline(0).GetKnot(1024.0)
    _Check(not gm.IsUnified(knot)
           and _Close(knot.GetPreTanSlope(), pre0)
           and _Close(knot.GetPostTanSlope(), 0.5),
           "undo re-breaks the key with its dragged slope")
    host.stack.Redo()
    model.RefreshAfterUndo()
    _Check(gm.IsUnified(model.KeySpline(0).GetKnot(1024.0)),
           "redo re-unifies the key")


# Host takeover routing

def TestHostTakeover():
    points = ((1001.0, 0.0), (1024.0, 5.0), (1048.0, 2.0))
    stage = _Stage(points)
    host = _Host(stage, _Refs("tx"), frame=1030.0)
    model = host.model()
    model.RefreshCurves()

    model.SelectKeys([("Ctrl.tx", 1024.0)])
    model.ToggleKey(0, 1024.0)
    model.ToggleKey(0, 1024.0)
    _Check(host.selections == [False, True, True],
           "selection changes reach the host: %s" % (host.selections,))

    # A command brackets through the host hook and lands on the host
    # stack; the document edit only brackets a drag's commit.
    _Check(model.InsertKeyAtCurrentFrame() is True, "insert applies")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1030.0) is not None,
           "the insert keyed the host frame")
    label, indices, drag, token = host.begins[-1]
    _Check((label, indices, drag) == ("Insert Key", [0], False),
           "commands bracket as non-drags: %r" % (host.begins[-1],))
    _Check(host.finishes[-1] == (token, True)
           and host.afters == 1
           and host.docEdits == [],
           "a command finishes through the hook, outside any document edit")
    _Check(host.hookEdits == ["enter", "enter", "exit", "exit"],
           "the command bracketed the write in the hook edit context")
    _Check(host.stack.CanUndo()
           and host.stack.UndoText() == "Insert Key",
           "the host stack took the push")

    _Check(model.BeginKeyDrag() is True, "the drag starts")
    model.DragKeys(4.0, 0.0)
    model.ReleaseGesture()
    _Check(len(host.queued) == 1, "the release queues through the host")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1034.0) is None,
           "nothing authors before the queued commit runs")
    _Pump(host)
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1034.0) is not None,
           "the queued commit moved the key")
    _Check(host.docEdits == ["enter", "exit"],
           "the drag commit ran inside the host document edit")
    _Check(host.finishes[-1][1] is True, "the drag finished committed")
    _Check(host.stack.UndoText() == "Move Keys",
           "the host stack took the drag's push")
    _Check(host.hookEdits.count("enter") == host.hookEdits.count("exit"),
           "every hook edit context closed")

    # The knot filter hides keys from the canvas without touching the
    # stage: the filtered knot is unselectable.
    def _Drop1048(index, authored):
        del index
        filtered = Ts.Spline(authored)
        if filtered.GetKnot(1048.0) is not None:
            filtered.RemoveKnot(1048.0)
        return filtered

    filtered = _Host(stage, _Refs("tx"), key_spline=_Drop1048).model()
    filtered.RefreshCurves()
    _Check(filtered.KeySpline(0).GetKnot(1048.0) is None
           and filtered.AuthoredSpline(0).GetKnot(1048.0) is not None,
           "the hook filters the key spline, not the authored spline")
    filtered.SelectKeys([("Ctrl.tx", 1048.0)])
    _Check(filtered.SelectedKeys() == [],
           "a filtered knot cannot be selected")

    # A hook that refuses stops the edit before anything authors.
    refused = _Host(stage, _Refs("tx"),
                    begin=lambda label, indices, drag: None).model()
    refused.RefreshCurves()
    refused.SelectKeys([("Ctrl.tx", 1024.0)])
    _Check(refused.BeginKeyDrag() is False, "a refused drag never starts")
    _Check(refused.InsertKeyAtCurrentFrame() is False,
           "a refused command never applies")
    _Check(refused.DeleteSelectedKeys() is False,
           "a refused delete never applies")
    _Check(not refused.undoStack.CanUndo(), "a refusal pushes nothing")

    # A hook that rolls back after the write reports why.
    rolled = _Host(stage, _Refs("tx"), frame=1036.0,
                   finish=lambda token, changed: "host rolled it back")
    rolledModel = rolled.model()
    rolledModel.RefreshCurves()
    _Check(rolledModel.InsertKeyAtCurrentFrame() is False,
           "a rolled-back command reports failure")
    _Check(rolled.notices == ["host rolled it back"],
           "the host refusal reaches the status line")
    _Check(rolled.finishes[-1][1] is True,
           "finish still saw the write it rolled back")

    # Without a stack, edits apply with no history.
    bareModel = gem.GraphEditorModel(
        stage=lambda: stage, discover=lambda: _Refs("tx"),
        undo_stack=None, frame=lambda: 1040.0)
    bareModel.RefreshCurves()
    _Check(bareModel.InsertKeyAtCurrentFrame() is True,
           "an edit without a stack still applies")
    _Check(_ResolvingSpline(stage, "tx").GetKnot(1040.0) is not None,
           "the key landed")
    _Check(bareModel.undoStack is None, "no stack was invented")


def main():
    groups = [
        ("cancel versus commit", TestCancelVsCommit),
        ("ordered key edits", TestOrderedKeyEdits),
        ("tangent break/unify", TestTangentBreakUnify),
        ("host takeover", TestHostTakeover),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    _Check(not [name for name in sys.modules if name.startswith("PySide")],
           "the model test imported no Qt binding")
    print("RIGEXEC_GRAPH_EDITOR_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
