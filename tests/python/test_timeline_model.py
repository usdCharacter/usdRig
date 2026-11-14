#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/timelineModel.py: the stage frame
range the timeline's start/end fields author, its undo, and the usdview
playback range that follows it.

No Qt and no usdview: the controller is a stand-in with the attributes
SyncController drives.

Usage: test_timeline_model.py
"""
# Sibling module: this script's own directory is sys.path[0].
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402

import rigExecUndo  # noqa: E402
import timelineModel as model  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def TestParseAndEdit():
    _Check(model.ParseFrame(" 12.5 ") == 12.5, "plain number")
    for text in ("", "abc", "nan", "inf"):
        _Check(model.ParseFrame(text) is None, "rejects %r" % text)
    _Check(model.EditedRange((1, 24), start=10) == (10, 24), "start")
    _Check(model.EditedRange((1, 24), start=30) == (30, 30),
           "start past end pushes end")
    _Check(model.EditedRange((10, 24), end=5) == (5, 5),
           "end before start pushes start")


def TestFollowPlaybackRange():
    follow = model.FollowPlaybackRange
    _Check(follow((1, 24), (1, 48), (1, 24)) == (1, 48),
           "tied ends follow the stage range")
    _Check(follow((1, 24), (1, 48), (5, 20)) == (5, 20),
           "a chosen playback range stays")
    _Check(follow((1, 100), (1, 15), (10, 20)) == (10, 15),
           "a chosen range clamps into a shorter stage range")
    _Check(follow((1, 100), (50, 60), (10, 20)) == (50, 50),
           "a range wholly outside clamps onto the new start")
    _Check(follow(None, (1, 48), (None, None)) == (1, 48),
           "a static stage takes its first range")
    _Check(follow((1, 24), None, (1, 24)) == (None, None),
           "dropping the stage range drops a following playback range")
    _Check(follow((1, 24), None, (5, 24)) == (5, None),
           "only the ends that followed are dropped")


def TestFollowStep():
    _Check(model.FollowStep((24, 24), (30, 30), 1.0) == 1.0, "one frame")
    _Check(model.FollowStep((24, 24), (12, 24), 1.0) == 2.0,
           "natural step follows the rate")
    _Check(model.FollowStep((24, 24), (12, 24), 0.5) == 0.5,
           "a chosen step stays")


def TestRangeLayer():
    stage = Usd.Stage.CreateInMemory()
    root, session = stage.GetRootLayer(), stage.GetSessionLayer()
    _Check(model.RangeLayer(stage, session) == session,
           "the session edit target")
    _Check(model.RangeLayer(stage, root) == root, "the root edit target")
    other = Sdf.Layer.CreateAnonymous()
    _Check(model.RangeLayer(stage, other) == root,
           "a sublayer target falls back to the root layer")
    session.startTimeCode = 3
    _Check(model.RangeLayer(stage, root) == session,
           "a session opinion would shadow the root, so it takes the edit")


def TestAuthorRangeAndUndo():
    stage = Usd.Stage.CreateInMemory()
    root = stage.GetRootLayer()
    root.startTimeCode = 1
    root.endTimeCode = 24
    stack = rigExecUndo.UndoStack()
    session = stage.GetSessionLayer()
    edit = model.AuthorRange(stage, session, 5, 48, stack)
    _Check(edit is not None and stack.CanUndo(), "one undo entry")
    _Check(model.StageRange(stage) == (5, 48), "the composed range moved")
    _Check(session.HasStartTimeCode() and session.HasEndTimeCode(),
           "both ends in one layer, so usdview plays the range")
    _Check(model.AuthorRange(stage, session, 5, 48, stack) is None,
           "an unchanged range records nothing")
    stack.Undo()
    _Check(model.StageRange(stage) == (1, 24), "undo restores the range")
    _Check(not session.HasStartTimeCode() and
           not session.HasEndTimeCode(),
           "undo removes the session opinions it added")
    stack.Redo()
    _Check(model.StageRange(stage) == (5, 48), "redo")
    try:
        model.AuthorRange(stage, session, 10, 5, stack)
    except ValueError:
        pass
    else:
        raise AssertionError("an inverted range is refused")

    static = Usd.Stage.CreateInMemory()
    _Check(model.StageRange(static) is None, "no authored range")
    model.AuthorRange(static, static.GetRootLayer(), 0, 12)
    _Check(model.StageRange(static) == (0, 12),
           "authoring gives a static stage a playable range")


class _Ui(object):
    class _Field(object):
        def __init__(self):
            self.text = None

        def setText(self, text):
            self.text = text

    def __init__(self):
        self.stepSize = self._Field()


class _DataModel(object):
    def __init__(self, frame):
        self.currentFrame = Usd.TimeCode(frame)
        self.frameRangeBegin = None
        self.frameRangeEnd = None


class _Controller(object):
    """The AppController attributes SyncController reads and writes."""

    def __init__(self, playback, frame, step=1.0, fps=24.0):
        self.realStartTimeCode, self.realEndTimeCode = playback
        self.step = step
        self.framesPerSecond = fps
        self._ui = _Ui()
        self._dataModel = _DataModel(frame)
        self.updates = 0
        self.samples = None

    def _UpdateTimeSamples(self, resetStageDataOnly=False):
        first, last = self.realStartTimeCode, self.realEndTimeCode
        self.samples = (first, last)
        frame = self._dataModel.currentFrame.GetValue()
        if first is not None and last is not None:
            frame = min(max(frame, first), last)
            self._dataModel.currentFrame = Usd.TimeCode(frame)

    def _updateOnFrameChange(self):
        self.updates += 1


def TestSyncController():
    controller = _Controller((1, 24), 20)
    moved = model.SyncController(controller, ((1, 24), 24, 24),
                                 ((1, 10), 24, 24))
    _Check((controller.realStartTimeCode, controller.realEndTimeCode) ==
           (1, 10), "playback range follows the stage range")
    _Check(controller._dataModel.frameRangeEnd == 10,
           "the data model's range moves too")
    _Check(moved and controller.updates == 1 and
           controller._dataModel.currentFrame == Usd.TimeCode(10),
           "a clamped current frame redraws")
    moved = model.SyncController(controller, ((1, 10), 24, 24),
                                 ((1, 10), 12, 24))
    _Check(not moved and controller.framesPerSecond == 12 and
           controller.step == 2.0 and controller._ui.stepSize.text == "2.0",
           "a frame-rate change updates rate and step")


def main():
    TestParseAndEdit()
    TestFollowPlaybackRange()
    TestFollowStep()
    TestRangeLayer()
    TestAuthorRangeAndUndo()
    TestSyncController()
    print("test_timeline_model: OK")


if __name__ == "__main__":
    main()
