#
# RigExec usdview timeline: the stage frame range behind the start/end
# fields beside usdview's time slider, and the usdview state that follows
# it. Qt-free, so the range rules and their undo are testable headlessly;
# timelineUI.py owns the widgets.
#
# usdview reads the stage range once, when the stage opens. These helpers
# re-apply it to a live usdview controller whenever the stage's range
# metadata changes, from a field, an undo, a script or a layer reload.
#
import math

from pxr import Sdf

import layerOpinionsModel
import rigExecUndo

START_KEY = "startTimeCode"
END_KEY = "endTimeCode"
FPS_KEY = "framesPerSecond"
TCPS_KEY = "timeCodesPerSecond"
RANGE_KEYS = (START_KEY, END_KEY)
TIME_KEYS = frozenset((START_KEY, END_KEY, FPS_KEY, TCPS_KEY))

EDIT_LABEL = "Frame Range"

_EPSILON = 1e-9


def ParseFrame(text):
    """A typed frame as a float, or None for anything that is not a finite
    number."""
    try:
        value = float(str(text).strip())
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def FormatFrame(value):
    """A frame spelled the way usdview spells its range fields."""
    return str(float(value))


def StageRange(stage):
    """(start, end) when the stage authors a range usdview plays, else None.

    usdview's own rule: both ends in one of the root or session layers.
    """
    if not stage or not stage.HasAuthoredTimeCodeRange():
        return None
    return (stage.GetStartTimeCode(), stage.GetEndTimeCode())


def DisplayedRange(stage):
    """The composed (start, end) the fields show, authored or not."""
    return (stage.GetStartTimeCode(), stage.GetEndTimeCode())


def TimeState(stage):
    """Everything about the stage that usdview's timeline is built from."""
    return (StageRange(stage), stage.GetFramesPerSecond(),
            stage.GetTimeCodesPerSecond())


def EditedRange(current, start=None, end=None):
    """The range after typing a new `start` or `end` into `current`.

    The other end is pushed along so start never passes end.
    """
    first, last = current
    if start is not None:
        first = start
        last = max(last, first)
    if end is not None:
        last = end
        first = min(first, last)
    return (first, last)


def _Tied(edge, stageEdge):
    return (edge is None or stageEdge is None or
            abs(edge - stageEdge) <= _EPSILON)


def FollowPlaybackRange(oldStage, newStage, playback):
    """The playback range after the stage range moves.

    `oldStage`, `newStage` and `playback` are (start, end) pairs or None;
    `playback` may also hold None ends (usdview's "no range"). An end that
    sat on the old stage end follows the new one; any other end is clamped
    into the new range. Without a new stage range, ends that followed it
    are dropped and the rest stay.
    """
    oldFirst, oldLast = oldStage if oldStage is not None else (None, None)
    first, last = playback if playback is not None else (None, None)
    if newStage is None:
        return (None if _Tied(first, oldFirst) else first,
                None if _Tied(last, oldLast) else last)
    newFirst, newLast = newStage
    first = newFirst if _Tied(first, oldFirst) else first
    last = newLast if _Tied(last, oldLast) else last
    first = min(max(first, newFirst), newLast)
    last = min(max(last, newFirst), newLast)
    return (first, last)


def FollowStep(oldRates, newRates, step):
    """usdview's frame step after frames or time codes per second change.

    A step still at the old rates' natural step (one frame) takes the new
    one; a step the artist chose stays.
    """
    oldFps, oldTcps = oldRates
    newFps, newTcps = newRates
    if oldFps <= 0 or newFps <= 0:
        return step
    if abs(step - oldTcps / oldFps) > _EPSILON:
        return step
    return newTcps / newFps


def RangeLayer(stage, editLayer):
    """The layer a range edit authors.

    Stage metadata composes only from the session and root layers, and the
    session wins. A session that already holds a range opinion takes the
    edit, so the edit is never shadowed; otherwise the edit target does
    when it is one of the two, else the root layer.
    """
    session = stage.GetSessionLayer()
    root = stage.GetRootLayer()
    if session and (session.HasStartTimeCode() or session.HasEndTimeCode()):
        return session
    if editLayer and editLayer in (session, root):
        return editLayer
    return root


def _Snapshots(layer):
    return [layerOpinionsModel.InfoSnapshot.Capture(
        layer, Sdf.Path.absoluteRootPath, key) for key in RANGE_KEYS]


def AuthorRange(stage, editLayer, start, end, undoStack=None,
                label=EDIT_LABEL):
    """Author start and end into one layer as a single undoable edit.

    Both ends go into the same layer because usdview plays a range only
    when one layer holds both. Returns the Edit pushed, or None when
    nothing changed. Raises RuntimeError on a layer that cannot be edited.
    """
    if start > end:
        raise ValueError("start %g is after end %g" % (start, end))
    layer = RangeLayer(stage, editLayer)
    if not layer.permissionToEdit:
        raise RuntimeError("%s is not editable" % layer.identifier)
    before = _Snapshots(layer)
    with Sdf.ChangeBlock():
        layer.startTimeCode = float(start)
        layer.endTimeCode = float(end)
    after = _Snapshots(layer)
    entries = [rigExecUndo.EditEntry(layer, Sdf.Path.absoluteRootPath, b, a)
               for b, a in zip(before, after)
               if (b.had, b.value) != (a.had, a.value)]
    if not entries:
        return None
    edit = rigExecUndo.Edit(label, entries)
    if undoStack is not None:
        undoStack.Push(edit)
    return edit


def SyncController(controller, oldState, newState):
    """Apply a changed stage TimeState to a live usdview AppController.

    Moves the playback range (realStartTimeCode/realEndTimeCode and the
    data model's frame range), the frame rate and step, then rebuilds the
    time slider through usdview's own _UpdateTimeSamples, which also clamps
    the current frame and enables or disables playback. Returns True when
    the current frame moved.
    """
    oldRange, oldFps, oldTcps = oldState
    newRange, newFps, newTcps = newState
    dataModel = controller._dataModel
    first, last = FollowPlaybackRange(
        oldRange, newRange,
        (controller.realStartTimeCode, controller.realEndTimeCode))
    controller.realStartTimeCode = first
    controller.realEndTimeCode = last
    dataModel.frameRangeBegin = first
    dataModel.frameRangeEnd = last
    if (oldFps, oldTcps) != (newFps, newTcps):
        # usdview's own fallback for a missing or broken rate.
        controller.framesPerSecond = newFps if newFps >= 1 else 24.0
        step = FollowStep((oldFps, oldTcps), (newFps, newTcps),
                          controller.step)
        if step != controller.step:
            controller.step = step
            controller._ui.stepSize.setText(str(step))
    frame = dataModel.currentFrame
    controller._UpdateTimeSamples(resetStageDataOnly=False)
    moved = dataModel.currentFrame != frame
    if moved:
        controller._updateOnFrameChange()
    return moved
