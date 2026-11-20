"""Qt-free gizmo gesture state. Call operations on the host's owner thread."""
import contextlib
import os
import sys

from pxr import Tf, Usd

try:
    import gizmoDrag
    import gizmoMarquee
    import gizmoScreen
    import gizmoSettings
    import rigExecUndo
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoDrag
    import gizmoMarquee
    import gizmoScreen
    import gizmoSettings
    import rigExecUndo


# Undo labels ("Move HandIK"), which is what the toolbar's Undo tooltip
# shows, so they read the same as the tool buttons.
_EDIT_VERBS = {
    gizmoScreen.TOOL_TRANSLATE: "Move",
    gizmoScreen.TOOL_ROTATE: "Rotate",
    gizmoScreen.TOOL_SCALE: "Scale",
}

def EditVerb(tool):
    return _EDIT_VERBS.get(tool, tool)


def _DragEditContext(drag):
    """The edit target a drag authors its structure in, chosen at press."""
    editTarget = getattr(drag, "editTarget", None)
    if editTarget is None:
        return contextlib.nullcontext()
    return Usd.EditContext(_WriterStage(drag), editTarget)


def _WriterStage(drag):
    """The stage a drag's writer collects for, or None."""
    try:
        return drag.target.writer.stage
    except AttributeError:
        return None


class GizmoModel:
    def __init__(self, *, stage, pushUndo, author, previewPush, previewEnd,
                 previewRepublish=lambda: None,
                 documentEdit=contextlib.nullcontext):
        # Preview callbacks are already scoped to this host session.
        self._stage = stage
        self._pushUndo = pushUndo
        self._author = author
        self._previewPush = previewPush
        self._previewEnd = previewEnd
        self._previewRepublish = previewRepublish
        self._documentEdit = documentEdit
        self.drag = self.releasedEdit = self.committing = None
        self.snapMode, self.snapReason = gizmoSettings.SNAP_OFF, ""
        self.ClearMarquee()

    def Begin(self, tool, target, handle, point, camera, viewport, settings, *,
              gimbal=False, ctrl=False, holdGrid=False, holdPoint=False,
              holdEdge=False, primePivot=lambda: None, rigWrittenSets=None,
              snapResolver=None):
        if target is None or not handle.grabbable or camera is None:
            return False
        options = settings.For(tool)
        # BEFORE AttributePaths() and BeginDrag(): the compensated
        # children's channels only join the undo set while the option
        # is on.
        target.SetPreserveChildren(bool(options.preserveChildren)
                                   and target.supportsPreserveChildren)
        # Frozen for the whole drag: BeginDrag() below captures the
        # pivot from it, and the handles were laid out on it.
        primePivot()
        try:
            stage = self._stage()
            recorder = rigExecUndo.EditRecorder(stage,
                                                target.AttributePaths())
            # The drag creates xformOps as it goes; they must land in the
            # layer the extension routes the commit's values to.
            with self._documentEdit():
                editTarget = stage.GetEditTarget()
                recorder.Begin()
            with Usd.EditContext(stage, editTarget):
                # A path curve moves the one control point pressed on.
                grab = getattr(target, "Grab", None)
                if grab is not None:
                    grab(handle)
                target.BeginDrag()
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not start the gizmo drag: %s"
                    % error)
            return False
        drag = gizmoDrag.DragState(
            tool, handle, target, point, camera, viewport,
            origin2d=handle.center,
            gimbal=gimbal, recorder=recorder)
        drag.ctrl = ctrl
        drag.editTarget = editTarget
        written, byTarget, allWritten, allByTarget = (
            rigWrittenSets(target) if rigWrittenSets else ((), (), False, False))
        drag.rigWritten = written
        drag.rigWrittenByTarget = byTarget
        drag.allRigWritten = allWritten
        drag.allRigWrittenByTarget = allByTarget
        # The bound method itself, never a lambda capturing the mode:
        # the holds change mid-drag, so a captured mode would make a
        # mid-drag V or C inert (spec item 13).
        drag.snapResolver = snapResolver
        mode, reason = gizmoDrag.ActiveSnapMode(
            options, tool, holdGrid, holdPoint, holdEdge)
        self.snapMode, self.snapReason = mode, reason
        self.drag = drag
        return True

    def Update(self, point, settings, *, holdSnap=False, holdGrid=False,
               holdPoint=False, holdEdge=False):
        drag = self.drag
        if drag is None:
            return
        try:
            options = settings.For(drag.tool)
            mode, reason = gizmoDrag.ActiveSnapMode(
                options, drag.tool, holdGrid, holdPoint, holdEdge)
            self.snapMode, self.snapReason = mode, reason
            with _DragEditContext(drag):
                gizmoDrag.ApplyDrag(
                    drag, point, options,
                    holdSnap=holdSnap, holdGrid=holdGrid,
                    ctrl=drag.ctrl, snapMode=mode,
                    gridSize=settings.gridSize)
            # The sample goes to Hydra, not to the stage. BEFORE Refresh():
            # Refresh re-reads the target's frames, and what it has to read is
            # the previewed values, or the handles would be drawn at the
            # pre-drag pose while the geometry moved (gizmoPreview.Push sets
            # both in the right order).
            self._previewPush(drag.target.writer.Pending(),
                              stage=drag.target.writer.stage)
            drag.target.RefreshDuringDrag()
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo drag failed: %s" % error)

    def Release(self):
        """Keep preview and press-time state until the host flushes the edit."""
        drag, self.drag = self.drag, None
        if drag is not None:
            self.releasedEdit = drag
        return drag

    def ReleasedEditLabel(self):
        drag = self.releasedEdit
        if drag is None or not drag.target.writer.HasPending():
            return None
        return "%s %s" % (EditVerb(drag.tool), drag.target.label)

    def Commit(self):
        drag = self.releasedEdit
        if drag is None:
            return
        # Clear before authoring: synchronous stage notices may call back.
        self.releasedEdit = None
        if _WriterStage(drag) != self._stage():
            self._previewEnd(stage=_WriterStage(drag), publish=False)
            drag.target.writer.Clear()
            return
        label = "%s %s" % (EditVerb(drag.tool),
                           drag.target.label)
        # With values to commit, the preview ends FIRST and WITHOUT a
        # republish: the host drops its overrides and leaves the drag pose
        # on screen, and the commit below fires the one stage notice that
        # evaluates and publishes the committed rig. Ending after the commit
        # evaluated twice -- the notice with the overrides still standing,
        # then the end -- and ending first WITH a republish would flash the
        # pre-drag pose. With nothing to commit the end publishes, as an
        # abort's does.
        pending = drag.target.writer.HasPending()
        self._previewEnd(stage=_WriterStage(drag), publish=not pending)
        authored = []
        try:
            # Author BEFORE the recorder commits: Begin() captured the layer as
            # it was, Commit() captures it as it now is, and the difference
            # between the two IS the undo entry -- so the values have to be on
            # the stage by now. The whole drag authors here, once.
            self.committing = (drag.target, drag.target.writer.Pending())
            try:
                # An editor extension may route this deferred commit (it
                # runs on a later turn) to its document layer; Track()
                # records that layer too when it is not the press-time one.
                with self._documentEdit():
                    drag.recorder.Track()
                    authored = self._author(drag.target.writer)
            finally:
                self.committing = None
            edit = drag.recorder.Commit(label)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not record the gizmo edit: %s"
                    % error)
            edit = None
        if pending and not authored:
            # The commit was expected to publish through its notice and
            # authored nothing: put the authored rig back on screen.
            self._previewRepublish()
        if edit is not None:
            self._pushUndo(edit)
        return drag.target.writer.Warnings()

    def Cancel(self):
        drag, self.drag = self.drag, None
        # First, and unconditionally: an abandoned preview would keep drawing a
        # pose nobody is holding any more, and this is the one path that can be
        # reached with the stage already gone.
        self._previewEnd(stage=_WriterStage(drag), publish=True)
        if drag is None:
            return
        if drag.target is not None and drag.target.writer is not None:
            # Nothing was authored, so there is nothing to take back; dropping
            # the collected values is the whole of it. recorder.Abort() below
            # still runs, because creating an xformOp IS authored structure
            # even when no value ever was.
            drag.target.writer.Clear()
        if not drag.target.prim.IsValid():
            # The stage went away under the drag (a replacement, or
            # usdview quitting). There is no layer left to restore into
            # and no prim to re-read; dropping the drag IS the abort.
            return False
        try:
            drag.recorder.Abort()
            drag.target.Refresh()
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not abort the gizmo drag: %s"
                    % error)
        return True

    def IsOwnCommit(self, target, resynced, changed, notice=None):
        """
        Whether this notice is a release authoring exactly the values its
        drag last previewed, and nothing else. The drag's last tick posed
        the target from those values and the rig keeps the pose it
        published for them, so the frames already hold; the host rebuilds
        the handles. A value the stage stores differently (a float
        attribute rounding a double) is not the previewed pose.
        """
        if self.committing is None or self.committing[0] is not target:
            return False
        values = self.committing[1]
        for path in resynced:
            if not path.IsPropertyPath() or path not in values:
                return False
            if notice and set(notice.GetChangedFields(path)) - {
                    "default", "timeSamples", "spline", "typeName"}:
                return False
        for path in changed:
            fields = set(notice.GetChangedFields(path)) if notice else set()
            if path in values:
                if fields - {"default", "timeSamples", "spline", "typeName"}:
                    return False
            elif (notice is None or not path.IsPrimPath() or fields or
                  not any(valuePath != path and valuePath.HasPrefix(path)
                          for valuePath in values)):
                return False
        for path, value in values.items():
            attr = target.stage.GetAttributeAtPath(path)
            if (not attr or attr.HasAuthoredConnections() or
                    attr.Get(target.time) != value):
                return False
        return True

    def ClearMarquee(self):
        self.marqueeAt = self.marqueeBand = None
        self._marqueeMode = gizmoMarquee.MODE_REPLACE
        self._marqueeWas = ()

    def BeginMarquee(self, point, selectedPaths, *, ctrl=False, shift=False):
        self.marqueeAt = point
        self.marqueeBand = None
        self._marqueeMode = gizmoMarquee.Mode(ctrl=ctrl, shift=shift)
        # usdview picks on press; resolve against the selection before that pick.
        self._marqueeWas = tuple(selectedPaths)

    def UpdateMarquee(self, point, ratio):
        if self.marqueeBand is None and not gizmoMarquee.PastSlop(
                self.marqueeAt, point, ratio):
            return False
        self.marqueeBand = (*self.marqueeAt, *point)
        return True

    def EndMarquee(self):
        result = self.marqueeBand, self._marqueeMode, self._marqueeWas
        self.ClearMarquee()
        return result
