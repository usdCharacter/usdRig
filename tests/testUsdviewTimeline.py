#
# The timeline's frame range fields, end to end.
#
# Opens examples/ArmShotAnim.usda (frames 1001-1048, a rig-driven arm),
# types into the start and end fields and checks what usdview shows
# afterwards: its slider, playback range fields and current frame, through
# an undo and an edit made outside the fields.
#
from pxr import Usd


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Type(QtWidgets, field, text):
    field.setFocus()
    field.setText(text)
    field.setModified(True)
    field.editingFinished.emit()
    QtWidgets.QApplication.processEvents()


def _Settle(QtWidgets):
    for _ in range(3):
        QtWidgets.QApplication.processEvents()


def _CheckTimeline(app, extras, start, end, playback):
    ui = app._ui
    stage = app._dataModel.stage
    _Check((stage.GetStartTimeCode(), stage.GetEndTimeCode()) ==
           (start, end), "stage range %s" % ((start, end),))
    _Check((extras.startField.text(), extras.endField.text()) ==
           (str(float(start)), str(float(end))), "fields show the range")
    _Check((app.realStartTimeCode, app.realEndTimeCode) == playback,
           "playback range %s, got %s" % (
               playback, (app.realStartTimeCode, app.realEndTimeCode)))
    _Check((ui.rangeBegin.text(), ui.rangeEnd.text()) ==
           (str(playback[0]), str(playback[1])),
           "usdview's range fields show the playback range")
    _Check(app._timeSamples[0] == playback[0] and
           app._timeSamples[-1] == playback[1],
           "the time samples span the playback range")
    if len(app._timeSamples) > 1:
        _Check(ui.frameSlider.maximum() == len(app._timeSamples) - 1,
               "the slider spans the samples")
    frame = app._dataModel.currentFrame.GetValue()
    _Check(playback[0] <= frame <= playback[1],
           "the current frame %s is inside the playback range" % frame)
    _Check(ui.frameField.text() == str(round(frame, 2)),
           "the frame field shows the current frame")


def _TestFrameRange(app, extras, undo, QtWidgets):
    ui = app._ui
    stage = app._dataModel.stage
    _Check(ui.stageBegin.isHidden() and ui.stageEnd.isHidden(),
           "usdview's read-only range labels are replaced")
    _CheckTimeline(app, extras, 1001, 1048, (1001.0, 1048.0))

    app.setFrame(1040)
    _Type(QtWidgets, extras.endField, "1020")
    _CheckTimeline(app, extras, 1001, 1020, (1001.0, 1020.0))
    _Check(app._dataModel.currentFrame == Usd.TimeCode(1020),
           "a frame past the new end moves onto it")
    session = stage.GetSessionLayer()
    _Check(session.HasStartTimeCode() and session.HasEndTimeCode(),
           "both ends are authored into the edit target")

    undo.Undo()
    _Settle(QtWidgets)
    _CheckTimeline(app, extras, 1001, 1048, (1001.0, 1048.0))
    undo.Redo()
    _Settle(QtWidgets)
    _CheckTimeline(app, extras, 1001, 1020, (1001.0, 1020.0))

    # A chosen playback range is kept and clamped, not reset.
    ui.rangeBegin.setText("1005.0")
    app._rangeBeginChanged()
    _Type(QtWidgets, extras.startField, "1003")
    _CheckTimeline(app, extras, 1003, 1020, (1005.0, 1020.0))
    _Type(QtWidgets, extras.startField, "1030")
    _CheckTimeline(app, extras, 1030, 1030, (1030.0, 1030.0))
    _Check(not ui.playButton.isEnabled(), "one frame cannot play")
    _Type(QtWidgets, extras.endField, "1040")
    _CheckTimeline(app, extras, 1030, 1040, (1030.0, 1040.0))
    _Check(ui.playButton.isEnabled(), "playback comes back")

    # An edit made outside the fields reaches usdview too.
    session.startTimeCode = 1001
    session.endTimeCode = 1048
    _Settle(QtWidgets)
    _CheckTimeline(app, extras, 1001, 1048, (1001.0, 1048.0))


def testUsdviewInputFunction(appController):
    from pxr.Usdviewq.qt import QtWidgets
    import rigExecUsdview

    api = appController._usdviewApi
    container = rigExecUsdview.ContainerFor(api)
    _Check(container is not None, "the RigExec container is loaded")
    extras = getattr(api.qMainWindow, "_rigExecTimeline", None)
    _Check(extras is not None, "the timeline additions are installed")
    _TestFrameRange(appController, extras, container._UndoStack(),
                    QtWidgets)
    print("RIGEXEC_TIMELINE_OK frame range fields, undo, outside edits")
