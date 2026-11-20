#
# RigExec graph editor: selection, drag and edit lifecycle over the
# animation curves, with no Qt in sight.
#
# Topology is graphModel's and mapping is graphScreen's; this module
# owns what an edit MEANS -- which keys are selected, what a drag
# moves, how a command brackets its writes -- and knows nothing about
# the widgets that show it. The panel maps events and forwards ONE
# operation per event here; every host read and write arrives as a
# callable, so the whole lifecycle runs headlessly against an
# in-memory stage (tests/python/test_graph_editor_model.py).
#
import contextlib
import math
import os
import sys

from pxr import Sdf, Tf, Ts, Usd

try:
    import graphModel
    import graphScreen
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import graphModel
    import graphScreen
    import rigExecUndo


SIDE_IN = graphModel.SIDE_IN
SIDE_OUT = graphModel.SIDE_OUT
SIDE_BOTH = graphModel.SIDE_BOTH


def _NoCurves():
    return []


def _FrameZero():
    return 0.0


def _AlwaysVisible(index):
    del index
    return True


def _NoOp(*args):
    del args


def _BeginFreely(label, indices, drag):
    del label, indices, drag
    return True


def _FinishFreely(token, changed):
    del token, changed
    return None


def _KeySplineAuthored(index, authored):
    del index
    return authored


def _CurveSplineEdited(index, authored, edited):
    del index
    return edited if edited is not None else authored


def _RunNow(callback):
    callback()


class Refused(Exception):
    """
    A model operation declined to change the spline.

    Raised through _Apply so the EditRecorder rolls the whole command
    back: several curves are edited in one pass, and a command that one
    of them refuses must not be half-applied to the rest.
    """

    def __init__(self, time=None):
        super(Refused, self).__init__("refused")
        self.time = time


class Gesture(object):
    """
    One undo step's worth of editing.

    Holds the spline every affected curve started with, the
    EditRecorder bracketing the whole gesture, and the live writes that
    make the viewport follow a drag. Every move recomputes from
    `origin`, never from the last write, so a drag cannot accumulate its
    own rounding or re-clamp an already clamped key.
    """

    def __init__(self, owner, indices, label, preview=False):
        self.owner = owner
        self.indices = list(indices)
        self.label = label
        self.origin = {}
        self.times = {}
        self.preOnly = False
        self.tangent = None           # (curveIndex, time, side)
        self.width = 0.0              # the grabbed handle's start width
        # A drag previews: the moves stay here, one key spline per edited
        # curve, and nothing is authored until Flush at the commit. The
        # recorder then begins right before that one write, so a drag
        # that is abandoned has nothing to restore.
        self.preview = {} if preview else None
        self.pushed = False           # a sample reached the host preview
        self.stage = owner.Stage()
        # The queued commit authors where the press would have.
        self.target = self.stage.GetEditTarget()
        self.token = None             # the edit hook's BeginEdit result
        paths = []
        curves = owner.Curves()
        for index in self.indices:
            self.origin[index] = owner.SplineCopy(index)
            paths.append(curves[index].attrPath)
        self.recorder = rigExecUndo.EditRecorder(owner.Stage(), paths)
        self._begun = False
        if self.preview is None:
            self._Begin()

    def _Begin(self):
        self.recorder.Begin()
        self._begun = True

    def TargetContext(self):
        return Usd.EditContext(self.stage, self.target)

    def Origin(self, index):
        return Ts.Spline(self.origin[index])

    def Write(self, index, spline):
        """A live write (no preview), or the move kept as the preview."""
        if self.preview is not None:
            self.owner._SetPreview(index, spline)
            return
        self.owner.WriteSpline(index, spline, self.label)

    def Flush(self):
        """Author the previewed splines, once, in one change block."""
        if self.preview is None:
            return
        self._Begin()
        with Sdf.ChangeBlock():
            for index in sorted(self.preview):
                self.owner.WriteSpline(index, self.preview[index],
                                       self.label)

    def Commit(self):
        if not self._begun:
            return None
        edit = self.recorder.Commit(self.label)
        if edit is not None and self.owner.undoStack is not None:
            self.owner.undoStack.Push(edit)
        return edit

    def Abort(self):
        if self._begun:
            self.recorder.Abort()
            self._begun = False


class GraphEditorModel(object):
    """
    The graph editor's curves, selection, drags and edit commands.

    Every host read and write arrives as a callable given to the
    constructor; nothing here creates a widget or touches a usdviewApi
    object. Undo stays on the injected stack through rigExecUndo, and
    writes land in the session edit target the stage carries.
    """

    def __init__(self, stage, discover=None, undo_stack=None,
                 document_edit=None, begin_edit=None, edit_context=None,
                 finish_edit=None, abort_edit=None, key_spline=None,
                 curve_spline=None, selection_changed=None, after_hook=None,
                 is_visible=None, frame=None, notify=None,
                 canvas_update=None, sync_widgets=None, update_viewport=None,
                 preview_push=None, preview_end=None, preview_republish=None,
                 commit_soon=None):
        """
        `stage()` is the live stage. `discover()` lists the CurveRefs the
        editor shows. `undo_stack` takes pushed Edits (None: edits apply
        without history). `document_edit()` brackets a commit in the
        host's edit target. The `*_edit`, `key_spline`, `curve_spline`,
        `selection_changed` and `after_hook` callables are the edit
        hook's bracket, knot filter and notifications. `is_visible()`
        says whether a curve draws. `frame()` is the current frame.
        `notify()` explains a refused command. `canvas_update()`,
        `sync_widgets()` and `update_viewport()` refresh the host after
        a change. The `preview_*` callables carry a drag's live values
        to the viewport. `commit_soon()` queues a released drag's
        commit; the default runs it at once.
        """
        self._stage = stage
        self._discover = discover or _NoCurves
        self._undoStack = undo_stack
        self._documentEdit = document_edit or contextlib.nullcontext
        self._beginEdit = begin_edit or _BeginFreely
        self._editContext = edit_context or contextlib.nullcontext
        self._finishEdit = finish_edit or _FinishFreely
        self._abortEdit = abort_edit or _NoOp
        self._keySpline = key_spline or _KeySplineAuthored
        self._curveSpline = curve_spline or _CurveSplineEdited
        self._selectionChanged = selection_changed or _NoOp
        self._afterHook = after_hook or _NoOp
        self._isVisible = is_visible or _AlwaysVisible
        self._frame = frame or _FrameZero
        self._notify = notify or _NoOp
        self._canvasUpdate = canvas_update or _NoOp
        self._syncWidgets = sync_widgets or _NoOp
        self._updateViewport = update_viewport or _NoOp
        self._previewPush = preview_push or _NoOp
        self._previewEnd = preview_end or _NoOp
        self._previewRepublish = preview_republish or _NoOp
        self._commitSoon = commit_soon or _RunNow

        self._curves = []
        self._splines = []            # what each attribute resolves to
        self._keySplines = []         # its keys (the hook's KeySpline)
        self._curveSplines = []       # what is drawn (CurveSpline)
        self._selection = set()       # (curveIndex, time)
        self._tangentSide = SIDE_BOTH
        self._snapFrames = True
        self._weighted = True
        self._gesture = None
        self._queued = None           # the released gesture awaiting commit
        self._refused = False         # the last _Apply was the hook's no

    # -- stage and curves -----------------------------------------------

    def Stage(self):
        return self._stage()

    @property
    def undoStack(self):
        return self._undoStack

    def Curves(self):
        """The CurveRefs the editor is showing, in list order."""
        return list(self._curves)

    def _Attribute(self, index):
        stage = self.Stage()
        if not stage or index >= len(self._curves):
            return None
        return stage.GetAttributeAtPath(self._curves[index].attrPath)

    def SplineCopy(self, index):
        """
        A private copy of curve `index`'s KEY spline, always TYPED: what
        an edit starts from. That is the resolved spline itself unless an
        edit hook filters knots out (the hook's KeySpline), and the
        live preview while a previewing drag runs.

        graphModel.SplineFor is what makes an unanimated channel
        keyable: Usd hands back an untyped spline for an attribute that
        has none, and Ts.Knot(typeName="") raises.
        """
        spline = (self._keySplines[index]
                  if index < len(self._keySplines) else None)
        if spline is None:
            attr = self._Attribute(index)
            spline = (graphModel.SplineFor(attr) if attr is not None
                      else Ts.Spline("double"))
        copy = Ts.Spline(spline)
        if not copy.GetValueTypeName():
            attr = self._Attribute(index)
            if attr is not None:
                copy = graphModel.SplineFor(attr)
        return copy

    def WriteSpline(self, index, spline, label="Graph Edit"):
        """
        Author `spline` onto curve `index`, through the model.

        graphModel.ApplySpline is the editor's ONLY writer, so the rule
        that an EMPTY spline clears the opinion rather than authoring an
        empty curve (graphModel.ClearSpline: an empty spline still wins
        value resolution and resolves to no value, which reads back as
        None) holds on every path -- deleting a curve's last key, a live
        drag, a tangent button -- without this file restating it.

        The undo stack argument is None ON PURPOSE. The gesture around
        this write is what pushes, once, at its release; ApplySpline's
        own recorder then only measures this one write and its Edit is
        dropped, which is what lets a drag author on EVERY mouse move
        without leaving a hundred entries in the history.

        What is CACHED afterwards is what the attribute now RESOLVES to,
        not the spline handed in. Since an empty spline clears the edit
        target's opinion instead of authoring an empty curve, the two
        stopped being the same thing: delete a root-animated curve's
        last key with the session layer as the edit target and the root
        layer's spline reappears, so caching the empty spline would draw
        a curve with no keys while the viewport kept animating it. The
        ObjectsChanged notice that would otherwise correct this is
        discarded, correctly, because the gesture still holds _gesture.
        """
        if index >= len(self._curves) or not self.Stage():
            return False
        attr = self._Attribute(index)
        if attr is None or not attr:
            return False
        try:
            with self._editContext():
                graphModel.ApplySpline(self.Stage(),
                                       self._curves[index].attrPath,
                                       spline, None, label)
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not author %s: %s"
                    % (self._curves[index].attrPath, error))
            return False
        self._splines[index] = graphModel.SplineFor(attr)
        self._RefreshDerived([index])
        self._canvasUpdate()
        self._updateViewport()
        return True

    def KeySplines(self):
        """The KEY spline per curve (squares, hit tests, selection), None
        where the curve is not drawn."""
        return [self._keySplines[i] if self.IsVisibleCurve(i) else None
                for i in range(len(self._curves))]

    def KeySpline(self, index):
        """Curve `index`'s KEY spline, or None."""
        return (self._keySplines[index]
                if index < len(self._keySplines) else None)

    def DrawSplines(self):
        """The spline the canvas strokes per curve (the hook's
        CurveSpline; the resolved spline without a hook), None where the
        curve is not drawn."""
        return [self._curveSplines[i] if self.IsVisibleCurve(i) else None
                for i in range(len(self._curves))]

    def AuthoredSpline(self, index):
        """What curve `index`'s attribute resolves to (knots the hook
        filters out included), or None."""
        return self._splines[index] if index < len(self._splines) else None

    def _DerivedFor(self, index):
        """(key spline, curve spline) of curve `index`."""
        authored = self._splines[index] if index < len(self._splines) \
            else None
        gesture = self._gesture
        edited = None
        if gesture is not None and gesture.preview is not None:
            edited = gesture.preview.get(index)
        if authored is None:
            key = edited if edited is not None else authored
            return key, key
        if edited is not None:
            key = edited
        else:
            key = self._keySpline(index, authored)
        curve = self._curveSpline(index, authored, edited)
        return key, curve

    def _RefreshDerived(self, indices=None):
        count = len(self._splines)
        while len(self._keySplines) < count:
            self._keySplines.append(None)
            self._curveSplines.append(None)
        del self._keySplines[count:]
        del self._curveSplines[count:]
        for index in (range(count) if indices is None else indices):
            if index < count:
                self._keySplines[index], self._curveSplines[index] = \
                    self._DerivedFor(index)

    def _SetPreview(self, index, spline):
        """A previewing gesture's move: drawn, nothing authored."""
        gesture = self._gesture
        if gesture is None or gesture.preview is None:
            return
        gesture.preview[index] = spline
        self._RefreshDerived([index])
        self._canvasUpdate()

    def RefreshCurves(self):
        """
        Rebuild the curve set from the host's discovery, keeping the key
        selection of any curve that survives. Returns the previous
        CurveRefs; the host prunes its visibility and rebuilds its list.
        """
        self.FlushPendingCommit()
        previous = list(self._curves)
        self._curves = list(self._discover())
        self.RefreshSplines()
        self._RemapSelection(previous)
        return previous

    def ClearCurves(self):
        """Drop every curve and key, for a stage replacement."""
        self._curves = []
        self.RefreshSplines()
        self._selection = set()

    def RefreshSplines(self):
        stage = self.Stage()
        self._splines = []
        for ref in self._curves:
            attr = stage.GetAttributeAtPath(ref.attrPath) if stage else None
            if attr is None or not attr:
                self._splines.append(None)
                continue
            try:
                self._splines.append(graphModel.SplineFor(attr))
            except Exception:
                self._splines.append(None)
        self._RefreshDerived()

    def RefreshAfterUndo(self):
        """Re-read after an undo restored specs behind our back; False
        while a gesture holds the splines."""
        if self._gesture is not None:
            return False
        self.RefreshSplines()
        self.PruneSelection()
        return True

    def NoteExternalChange(self, paths):
        """Re-read when `paths` touches a listed curve; False when
        nothing listed changed, or while a gesture holds the splines."""
        if self._gesture is not None or not self._curves:
            return False
        if not paths:
            return False
        for ref in self._curves:
            attrPath = ref.attrPath
            for path in paths:
                if path == attrPath or attrPath.HasPrefix(path):
                    self.RefreshSplines()
                    self.PruneSelection()
                    return True
        return False

    def _RemapSelection(self, previous):
        """
        Carry the key selection across a curve-set rebuild by LABEL: the
        indices are positions in the new list and mean nothing in the
        old one.
        """
        if not self._selection:
            return
        labels = {}
        for index, ref in enumerate(self._curves):
            labels[ref.Label()] = index
        remapped = set()
        for index, time in self._selection:
            if index >= len(previous):
                continue
            target = labels.get(previous[index].Label())
            if target is None:
                continue
            spline = self._keySplines[target]
            if spline is not None and spline.GetKnot(time) is not None:
                remapped.add((target, time))
        self._selection = remapped

    def PruneSelection(self):
        """Drop selected keys that no longer exist on the stage."""
        kept = set()
        for index, time in self._selection:
            if index >= len(self._keySplines):
                continue
            spline = self._keySplines[index]
            if spline is not None and spline.GetKnot(time) is not None:
                kept.add((index, time))
        self._selection = kept

    def IsSampled(self, index):
        """Whether curve `index` is animated by time samples, not a spline."""
        return graphModel.IsSampledCurve(self._Attribute(index))

    def _SplineIndices(self, indices, what):
        """
        `indices` without the time-sampled curves, which have no tangents,
        infinities, curve type or arrival values to set: written back as
        samples, such an edit would change nothing. Says so when nothing
        is left.
        """
        kept = [index for index in indices if not self.IsSampled(index)]
        if not kept and indices:
            self._notify("%s does not apply to time samples." % what)
        return kept

    def IsVisibleCurve(self, index):
        if index >= len(self._curves):
            return False
        return bool(self._isVisible(index))

    def VisibleIndices(self):
        return [i for i in range(len(self._curves))
                if self.IsVisibleCurve(i)]

    # -- selection ------------------------------------------------------

    def SelectionPairs(self):
        """The raw (curveIndex, time) selection graphScreen wants."""
        return self._selection

    def SelectedKeys(self):
        """[(curve label, time)] for the selected keys, in list order."""
        return [(self._curves[index].Label(), time)
                for index, time in sorted(self._selection)]

    def SelectKeys(self, pairs):
        """Select exactly these (curve label, time) keys."""
        labels = dict((ref.Label(), index)
                      for index, ref in enumerate(self._curves))
        selection = set()
        for label, time in pairs:
            index = labels.get(label)
            if index is None:
                continue
            spline = self._keySplines[index]
            if spline is None:
                continue
            knot = spline.GetKnot(float(time))
            if knot is not None:
                selection.add((index, knot.GetTime()))
        self._selection = selection
        self._canvasUpdate()
        self._syncWidgets()
        self._selectionChanged(False)

    def SetSelection(self, pairs):
        self._selection = set(pairs)
        self._canvasUpdate()
        self._syncWidgets()
        self._selectionChanged(False)

    def ToggleKey(self, index, time):
        pair = (index, time)
        if pair in self._selection:
            self._selection.discard(pair)
        else:
            self._selection.add(pair)
        self._canvasUpdate()
        self._syncWidgets()
        self._selectionChanged(True)

    def SelectGlyphs(self, glyphs, add=False):
        pairs = set((g.curveIndex, g.time) for g in glyphs)
        self._selection = (self._selection | pairs) if add else pairs
        self._canvasUpdate()
        self._syncWidgets()
        self._selectionChanged(bool(add))

    def SelectedTimes(self):
        """{curve index: [time, ...]} for the selected keys."""
        times = {}
        for index, time in sorted(self._selection):
            times.setdefault(index, []).append(time)
        return times

    def KeyExtents(self, selectedOnly=False):
        """
        (tMin, tMax, vMin, vMax) over the keys, or None when there are
        none. Values come from the knots, not from the sampled curve: a
        Bezier can overshoot its keys and the editor frames the keys.
        When an edit hook draws a curve with knots of its own (derived
        knots it filtered out of the keys), framing everything takes
        their values too: those are values the attribute really takes.
        """
        times, values = [], []
        for index in self.VisibleIndices():
            spline = self._keySplines[index]
            if spline is None or spline.IsEmpty():
                continue
            for knot in spline.GetKnots().values():
                time = knot.GetTime()
                if selectedOnly and (index, time) not in self._selection:
                    continue
                times.append(time)
                values.append(float(knot.GetValue()))
                if knot.IsDualValued():
                    values.append(float(knot.GetPreValue()))
            curve = self._curveSplines[index]
            if not selectedOnly and curve is not None and curve is not spline:
                for knot in curve.GetKnots().values():
                    values.append(float(knot.GetValue()))
        if not times:
            return None
        return (min(times), max(times), min(values), max(values))

    def ActiveGestureLabel(self):
        """The live drag's undo label, or None while no drag runs."""
        gesture = self._gesture
        return gesture.label if gesture is not None else None

    # -- gestures -------------------------------------------------------

    def BeginKeyDrag(self, preOnly=False):
        """
        Start moving the selected keys; False when none are, or when the
        edit hook refuses (the hook's BeginEdit).

        `preOnly` shapes arrivals: the drag moves whole keys in time
        but adjusts only preValues in value, so grabbing a dual knot's
        pre square sculpts the step without moving the departure.
        """
        self.FlushPendingCommit()
        times = self.SelectedTimes()
        if not times or self._gesture is not None or not self.Stage():
            return False
        label = "Move Pre Values" if preOnly else "Move Keys"
        gesture = self._StartGesture(sorted(times), label)
        if gesture is None:
            return False
        gesture.times = times
        gesture.preOnly = bool(preOnly)
        return True

    def _StartGesture(self, indices, label):
        """A new drag gesture, bracketed by the edit hook; None when the
        hook refuses or the gesture cannot start."""
        token = self._beginEdit(label, list(indices), True)
        if not token:
            return None
        try:
            with self._editContext():
                gesture = Gesture(self, indices, label, preview=True)
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not start %s: %s"
                    % (label, error))
            self._abortEdit(token)
            return None
        gesture.token = token
        self._gesture = gesture
        return gesture

    def DragKeys(self, dt, dv):
        """Move the dragged keys by `dt` frames and `dv` in value: one
        mouse move's worth, already resolved from pixels by the host."""
        gesture = self._gesture
        if gesture is None or gesture is self._queued:
            return
        selection = set()
        for index in gesture.indices:
            spline = gesture.Origin(index)
            if gesture.preOnly:
                # Time moves the whole key (shared time), value shapes
                # arrivals only: MoveKeys with dv=0 keeps departures,
                # then each moved arrival += dv.
                moved = graphModel.MoveKeys(
                    spline, gesture.times[index], dt, 0.0,
                    self._snapFrames)
                for newTime in moved:
                    knot = spline.GetKnot(newTime)
                    if knot is not None and knot.IsDualValued():
                        knot.SetPreValue(knot.GetPreValue() + dv)
                        spline.SetKnot(knot)
            else:
                moved = graphModel.MoveKeys(
                    spline, gesture.times[index], dt, dv,
                    self._snapFrames)
            gesture.Write(index, spline)
            for time in moved:
                selection.add((index, time))
        self._selection = selection
        self.PushPreview()
        self._syncWidgets()

    def BeginTangentDrag(self, index, time, side, width):
        """Start dragging one tangent handle; False when the edit hook
        refuses (the hook's BeginEdit)."""
        self.FlushPendingCommit()
        if self._gesture is not None or not self.Stage():
            return False
        gesture = self._StartGesture([index], "Edit Tangent")
        if gesture is None:
            return False
        gesture.tangent = (index, time, side)
        gesture.width = width
        return True

    def TangentDragWeighted(self, index):
        """Whether a tangent drag on curve `index` authors widths:
        Weighted on, and Bezier (Hermite fixes the width)."""
        if not self._weighted:
            return False
        spline = self.SplineCopy(index)
        return graphModel.CurveTypeName(spline) != graphModel.CURVE_HERMITE

    def DragTangent(self, slope, width=None):
        """
        Point the dragged handle at `slope`, with `width` while the
        host's drag is weighted (None keeps the stored width). The
        slope is mirrored to the other side while the key's tangents
        are unified (spec 2.4).
        """
        gesture = self._gesture
        if (gesture is None or gesture.tangent is None
                or gesture is self._queued):
            return
        index, time, side = gesture.tangent
        spline = gesture.Origin(index)
        unified = graphModel.IsUnified(spline.GetKnot(time))
        graphModel.SetTangent(spline, time, side, slope, width)
        if unified:
            other = SIDE_IN if side == SIDE_OUT else SIDE_OUT
            # Slope only: unified tangents share an angle, not a
            # length, so the far handle keeps the weight it had.
            graphModel.SetTangent(spline, time, other, slope)
        gesture.Write(index, spline)
        self.PushPreview()
        self._syncWidgets()

    def PushPreview(self):
        """Send the dragged curves' current-frame values to Hydra."""
        gesture = self._gesture
        if gesture is None or not gesture.preview \
                or gesture is self._queued:
            return
        pending = {}
        for index in gesture.preview:
            if index >= len(self._curveSplines):
                continue
            spline = self._curveSplines[index]
            try:
                value = spline.Eval(self._frame()) if spline else None
            except Exception:
                value = None
            if value is not None:
                pending[self._curves[index].attrPath] = float(value)
        if not pending:
            return
        self._previewPush(pending, gesture.stage)
        gesture.pushed = True

    def _EndPreview(self, gesture, publish):
        if gesture.pushed:
            gesture.pushed = False
            self._previewEnd(gesture.stage, publish)

    def ReleaseGesture(self):
        """
        The drag was released: queue its commit for the next event-loop
        turn. The preview stays drawn, and in Hydra, until then.
        """
        gesture = self._gesture
        if gesture is None or gesture is self._queued:
            return False
        self._queued = gesture
        self._commitSoon(lambda: self._FlushQueued(gesture))
        return True

    def _FlushQueued(self, gesture):
        if self._queued is not gesture:
            return
        try:
            self.EndGesture()
        except Exception as error:
            Tf.Warn("rigExecUsdview: queued graph edit failed: %s" % error)

    def HasPendingCommit(self):
        return self._queued is not None

    def FlushPendingCommit(self):
        """Commit a released drag now; EndGesture's result."""
        if self._queued is None:
            return False
        return self.EndGesture()

    def EndGesture(self):
        """
        Commit the live gesture; True when it changed the edit target.

        The drag's previewed moves are authored here, once, in one change
        block and in the edit target captured at the press; the edit hook
        then finishes the edit (and may roll it back: False, the curves
        re-read).

        A drag that ends where it started, or on a curve a weaker layer
        owns, commits nothing -- the recorder compares the spec before
        and after -- and there is then no undo entry to offer.
        """
        gesture = self._gesture
        self._queued = None
        if gesture is None:
            return False
        changed = False
        failed = False
        previewed = bool(gesture.preview) and gesture.pushed
        # Ended first and without a republish when there is something to
        # author: the commit's own stage notice publishes the result.
        self._EndPreview(gesture, publish=False)
        try:
            with gesture.TargetContext(), \
                    self._documentEdit(), \
                    self._editContext():
                gesture.Flush()
                self._gesture = None
                changed = gesture.Commit() is not None
        except Exception as error:
            failed = True
            Tf.Warn("rigExecUsdview: graph editor could not record the "
                    "edit: %s" % error)
        finally:
            self._gesture = None
        if failed:
            try:
                with self._editContext():
                    gesture.Abort()
            except Exception:
                pass
            self._abortEdit(gesture.token)
            self.RefreshSplines()
            self.PruneSelection()
            changed = False
            refusal = None
        else:
            refusal = self._finishEdit(gesture.token, changed)
            self._RefreshDerived()
            self._afterHook()
        self._syncWidgets()
        if refusal:
            self._notify(refusal)
            changed = False
        if previewed and not changed:
            # No stage notice will publish: put the authored rig back.
            self._previewRepublish()
        self._canvasUpdate()
        return changed

    def AbortGesture(self):
        gesture = self._gesture
        self._gesture = None
        self._queued = None
        if gesture is None:
            return False
        self._EndPreview(gesture, publish=True)
        try:
            with self._editContext():
                gesture.Abort()
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor could not abort the "
                    "edit: %s" % error)
        self._abortEdit(gesture.token)
        self.RefreshSplines()
        self.PruneSelection()
        self._syncWidgets()
        self._canvasUpdate()
        self._updateViewport()
        return True

    def _Apply(self, indices, label, operation):
        """
        Run `operation(index, spline)` on a copy of each curve's spline
        and write them all as ONE undo step.

        This is the non-drag half of the same bracket Gesture gives a
        drag: a tangent button pressed with three curves selected is one
        step, not three. The edit hook brackets it the same way (it may
        refuse it up front, or roll it back afterwards: False).

        An operation that RAISES aborts the whole command: the recorder
        puts every curve back, so a command that cannot be applied to
        one curve is not half-applied to the others. `Refused` is how
        the Time field says "this would consume another key" (see
        SetKeyTime); anything else propagates.

        Returns True only when the edit target actually CHANGED -- when
        the recorder produced an Edit and it went on the undo stack.
        A command can run through cleanly and change nothing: deleting
        keys that a weaker layer owns clears an opinion the edit target
        never held. Reporting that as success would clear the selection
        and offer a Ctrl+Z for an edit that is not there.
        """
        self._refused = False
        self.FlushPendingCommit()
        indices = [i for i in indices if i < len(self._curves)]
        if not indices or not self.Stage() or self._gesture is not None:
            return False
        token = self._beginEdit(label, list(indices), False)
        if not token:
            self._refused = True
            return False
        try:
            with self._editContext():
                gesture = Gesture(self, indices, label)
                # Held while the writes land so the ObjectsChanged
                # handler does not re-read the very splines this call
                # is authoring.
                self._gesture = gesture
                try:
                    with Sdf.ChangeBlock():
                        for index in indices:
                            spline = gesture.Origin(index)
                            if operation(index, spline) is False:
                                continue
                            gesture.Write(index, spline)
                    changed = gesture.Commit() is not None
                except Exception:
                    gesture.Abort()
                    raise
                finally:
                    self._gesture = None
        except Exception:
            self._abortEdit(token)
            # The layer is back but the cached splines are the
            # half-applied ones written on the way to the failure.
            self.RefreshSplines()
            self.PruneSelection()
            raise
        refusal = self._finishEdit(token, changed)
        self._afterHook()
        self._syncWidgets()
        self._canvasUpdate()
        if refusal:
            self._refused = True
            self._notify(refusal)
            return False
        return changed

    # -- edit commands --------------------------------------------------

    def SetTangentType(self, mode):
        """tangent buttons, on the selected keys (spec 2.4)."""
        times = self.SelectedTimes()
        indices = self._SplineIndices(sorted(times), "Tangent type")
        if not indices:
            return False
        return self._Apply(
            indices, "Tangent %s" % mode.capitalize(),
            lambda index, spline: graphModel.SetTangentType(
                spline, times[index], mode, self._tangentSide))

    def SetTangentSide(self, side):
        """Narrow the tangent buttons to the In or Out side, or Both."""
        if side not in (SIDE_IN, SIDE_OUT, SIDE_BOTH):
            raise ValueError("unknown tangent side %r" % (side,))
        self._tangentSide = side

    def TangentSide(self):
        return self._tangentSide

    def Break(self):
        times = self.SelectedTimes()
        indices = self._SplineIndices(sorted(times), "Break tangents")
        if not indices:
            return False
        return self._Apply(
            indices, "Break Tangents",
            lambda index, spline: graphModel.BreakTangents(
                spline, times[index]))

    def Unify(self):
        times = self.SelectedTimes()
        indices = self._SplineIndices(sorted(times), "Unify tangents")
        if not indices:
            return False
        return self._Apply(
            indices, "Unify Tangents",
            lambda index, spline: graphModel.UnifyTangents(
                spline, times[index]))

    def InsertKeyAtCurrentFrame(self):
        """
        Key every visible curve at the current frame (spec 2.4).

        A curve with no knots at all cannot be evaluated, so it is keyed
        from the attribute's RESOLVED value instead -- which is how an
        unanimated avar gets its first key from the graph editor.
        """
        frame = graphModel.SnapTime(self._frame())
        indices = self.VisibleIndices()
        if not indices:
            return False
        applied = self._Apply(indices, "Insert Key",
                              lambda index, spline: self._InsertOne(
                                  index, spline, frame))
        if applied:
            self._selection = set((i, frame) for i in indices)
            self._syncWidgets()
            self._canvasUpdate()
        return applied

    def _InsertOne(self, index, spline, frame):
        authored = self.AuthoredSpline(index)
        key = self._keySplines[index] if index < len(self._keySplines) \
            else None
        if (key is not None and key is not authored
                and authored is not None and not authored.IsEmpty()
                and spline.GetKnot(float(frame)) is None):
            # `spline` is the KEY spline: with knots filtered out (a
            # baked segment held between its keys there) it no longer
            # evaluates to what the stage shows. The new key takes the
            # value the curve has on the stage, as without a hook.
            if not math.isfinite(float(frame)):
                raise ValueError("InsertKey requires a finite time: %r"
                                 % (frame,))
            value = authored.Eval(float(frame))
            if value is not None:
                graphModel.AuthorKnot(spline, float(frame), value)
                return True
        if graphModel.InsertKey(spline, frame) is not None:
            return True
        attr = self._Attribute(index)
        value = attr.Get(Usd.TimeCode(frame)) if attr is not None else None
        try:
            numeric = float(value if value is not None else 0.0)
        except (TypeError, ValueError):
            numeric = 0.0
        if not math.isfinite(numeric):
            numeric = 0.0
        graphModel.AuthorKnot(spline, frame, numeric)
        return True

    def InsertKeyOnCurve(self, index, time):
        """A double-click on a curve: one key, on that curve, there."""
        try:
            frame = (graphModel.SnapTime(time)
                     if self._snapFrames else float(time))
            if not math.isfinite(frame):
                return False
        except (ValueError, TypeError):
            return False
        try:
            applied = self._Apply([index], "Insert Key",
                                  lambda i, spline: self._InsertOne(i, spline,
                                                                    frame))
        except ValueError:
            return False
        if applied:
            self._selection = set([(index, frame)])
            self._syncWidgets()
            self._canvasUpdate()
        return applied

    def DeleteSelectedKeys(self):
        """
        Remove the selected keys from the EDIT TARGET; True when the
        layer changed.

        Deleting is a clear, not an erasure, so a key can survive it:
        the editor draws the curve the stage RESOLVES, and a key that
        belongs to a weaker layer than the edit target is still there
        afterwards. Two outcomes get a status line rather than a silent
        surprise.

        Nothing changed at all -- the edit target held no opinion on
        those keys -- keeps the selection, because the keys the artist
        picked are still on screen and still theirs to pick.
        """
        times = self.SelectedTimes()
        if not times:
            return False
        applied = self._Apply(
            sorted(times), "Delete Keys",
            lambda index, spline: graphModel.DeleteKeys(spline,
                                                        times[index]))
        survivors = self._SurvivingKeys(times)
        if not applied:
            if not self._refused:       # the edit hook said why already
                self._notify("Nothing deleted: those keys come from a "
                             "weaker layer, not the edit target.")
            return False
        self._selection = set()
        self._syncWidgets()
        self._canvasUpdate()
        if survivors:
            self._notify("Session keys cleared; file animation shows "
                         "through.")
        return True

    def _SurvivingKeys(self, times):
        """
        The curves among `times` still holding one of those keys after
        a delete, read off the RESOLVED spline the write cached.
        """
        survivors = []
        for index, group in times.items():
            spline = (self._keySplines[index]
                      if index < len(self._keySplines) else None)
            if spline is None:
                continue
            if any(spline.GetKnot(time) is not None for time in group):
                survivors.append(index)
        return survivors

    def AnimatedIndices(self):
        """
        The visible curves that actually HAVE keys.

        The editor lists a control's unanimated rig channels too, so an
        artist can key one from the graph (spec 1.3). They are not
        curves yet: extrapolation on a knotless spline means nothing,
        and authoring one would replace the channel's resolved value
        with an empty spline.
        """
        return [i for i in self.VisibleIndices()
                if self._splines[i] is not None
                and not self._splines[i].IsEmpty()]

    def SetInfinity(self, pre=None, post=None):
        """
        Infinity combos, on the visible curves that have keys.

        The visible set IS the selected set here: picking rows in the
        curve list isolates them, so what is drawn is what an artist
        would call selected. Knotless channels are skipped -- see
        AnimatedIndices.

        `pre` / `post` are independent: each combo passes only its own
        side and leaves the other None, so changing the post infinity
        cannot quietly rewrite every curve's pre infinity from whatever
        the other combo happened to be displaying.
        """
        indices = self.AnimatedIndices()
        if not indices:
            self._notify("Infinity needs a curve with keys.")
            return False
        indices = self._SplineIndices(indices, "Infinity")
        if not indices:
            return False
        return self._Apply(
            indices, "Set Infinity",
            lambda index, spline: graphModel.SetExtrapolation(spline, pre,
                                                              post))

    def SetCurveType(self, name):
        """
        Ts curve type for the visible curves that have keys.

        Like SetInfinity, the visible set IS the selected set, and
        knotless channels are skipped: authoring an empty spline just to
        remember a curve type would win value resolution and resolve to
        no value (graphModel.ApplySpline: empty clears). Key first (the
        new key inherits the curve's current type), then switch.
        """
        if name not in graphModel.CURVE_TYPES:
            raise ValueError("unknown curve type %r" % (name,))
        indices = self.AnimatedIndices()
        if not indices:
            self._notify("Curve type needs a curve with keys.")
            return False
        indices = self._SplineIndices(indices, "Curve type")
        if not indices:
            return False
        return self._Apply(
            indices, "Set Curve Type",
            lambda index, spline: graphModel.SetCurveType(spline, name))

    def CurveType(self):
        """Our name for the focus curve's Ts curve type, or None."""
        index = self.FocusCurveIndex()
        spline = (self._splines[index]
                  if index is not None and index < len(self._splines)
                  else None)
        return graphModel.CurveTypeName(spline)

    def FocusCurveIndex(self):
        """
        The curve the key stats and the Infinity combos DISPLAY.

        The lowest-indexed curve holding a selected key, else the first
        visible curve with keys. A knotless channel is never the focus:
        its extrapolation is Held by construction and showing that would
        say the artist's cycle had been lost.
        """
        animated = self.AnimatedIndices()
        if not animated:
            return None
        selected = sorted(index for index, _ in self._selection
                          if index in animated)
        return selected[0] if selected else animated[0]

    def SetSnapFrames(self, enabled):
        self._snapFrames = bool(enabled)

    def SnapFrames(self):
        return self._snapFrames

    def SetWeighted(self, enabled):
        self._weighted = bool(enabled)

    def Weighted(self):
        return self._weighted

    def SetKeyTime(self, time):
        """
        Put the selection at `time`: Time field (spec 2.4).

        graphModel.SetKeyTimes does the move per spline, refusing when
        the frame is already held by a key OUTSIDE the selection. Across
        several curves the selection has to stay RIGID, so each curve is
        asked for its own target -- the earliest selected key of ALL of
        them lands on `time` and the rest keep their offsets -- and one
        curve's refusal aborts the whole command rather than shearing
        the selection apart.
        """
        try:
            target = float(time)
            if not math.isfinite(target):
                self._notify("Frame must be a finite number: %r" % (time,))
                return False
        except (TypeError, ValueError):
            self._notify("Frame must be a finite number: %r" % (time,))
            return False
        times = self.SelectedTimes()
        if not times:
            return False
        earliest = min(t for group in times.values() for t in group)
        if abs(target - earliest) < 1e-12:
            return False

        def Move(index, spline):
            offset = min(times[index]) - earliest
            if not graphModel.SetKeyTimes(spline, times[index],
                                          target + offset):
                raise Refused(target + offset)
            return True

        try:
            applied = self._Apply(sorted(times), "Move Keys", Move)
        except Refused as refusal:
            self._notify("Frame %g already has a key; nothing moved."
                         % refusal.time)
            return False
        except ValueError as error:
            self._notify(str(error))
            return False
        if applied:
            self._SelectMovedKeys(times, target - earliest)
        return applied

    def _SelectMovedKeys(self, times, delta):
        """
        Follow the keys the Time field just moved.

        Their new times are read back off the stage rather than assumed:
        graphModel clamps a key that has no room, so the shift asked for
        is not always the shift applied.
        """
        selection = set()
        for index, group in times.items():
            spline = self._keySplines[index]
            if spline is None:
                continue
            for time in group:
                for candidate in (time + delta, time):
                    knot = spline.GetKnot(candidate)
                    if knot is not None:
                        selection.add((index, knot.GetTime()))
                        break
        self._selection = selection
        self._syncWidgets()
        self._canvasUpdate()

    def SetKeyValue(self, value):
        """Set every selected key's departure to `value` (Value)."""
        try:
            if not math.isfinite(float(value)):
                self._notify("Value must be a finite number: %r" % (value,))
                return False
        except (TypeError, ValueError):
            self._notify("Value must be a finite number: %r" % (value,))
            return False
        times = self.SelectedTimes()
        if not times:
            return False
        try:
            return self._Apply(
                sorted(times), "Set Key Value",
                lambda index, spline: graphModel.SetKeyValues(
                    spline, times[index], value))
        except ValueError as error:
            self._notify(str(error))
            return False

    def SetKeyPreValue(self, preValue):
        """Set every selected key's arrival; makes the selection dual."""
        try:
            if not math.isfinite(float(preValue)):
                self._notify("Pre must be a finite number: %r" % (preValue,))
                return False
        except (TypeError, ValueError):
            self._notify("Pre must be a finite number: %r" % (preValue,))
            return False
        times = self.SelectedTimes()
        indices = self._SplineIndices(sorted(times), "An arrival value")
        if not indices:
            return False
        try:
            return self._Apply(
                indices, "Set Key Pre Value",
                lambda index, spline: graphModel.SetKeyPreValues(
                    spline, times[index], preValue))
        except ValueError as error:
            self._notify(str(error))
            return False

    def MakeSingle(self):
        """Drop the arrival squares: dual keys back to single-valued."""
        times = self.SelectedTimes()
        if not times:
            return False
        return self._Apply(
            sorted(times), "Make Single",
            lambda index, spline: graphModel.ClearDualKnots(
                spline, times[index]))
