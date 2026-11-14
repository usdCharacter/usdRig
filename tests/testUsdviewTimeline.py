#
# The timeline's frame range fields and the playblast, end to end.
#
# Opens examples/ArmShotAnim.usda (frames 1001-1048, a rig-driven arm),
# types into the start and end fields and checks what usdview shows
# afterwards: its slider, playback range fields and current frame, through
# an undo and an edit made outside the fields. Then playblasts offscreen,
# on screen and, when ffmpeg is available, to a movie, and checks the files
# and that the viewport and frame are put back.
#
# Set RIGEXEC_PLAYBLAST_KEEP=1 to keep the written frames.
#
import os
import shutil
import tempfile

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


def _Pixels(QtGui, path):
    image = QtGui.QImage(path)
    _Check(not image.isNull(), "readable image %s" % path)
    return image


def _TestPlayblast(app, api, QtGui, QtWidgets, directory):
    from pxr.Usdviewq.qt import QtCore
    import playblastModel as model
    import playblastUI

    def _Fail(parent, text):
        raise AssertionError("playblast warned: %s" % text)

    playblastUI._Warn = _Fail
    view = app._stageView
    frame = app._dataModel.currentFrame
    physical = view._physicalWindowSize
    options = model.PlayblastOptions(
        view=False, offScreen=True, timeRange=model.START_END,
        startTime=1001, endTime=1024, format=model.FORMAT_IMAGE,
        encoding="png", displaySize=model.SIZE_CUSTOM, width=320,
        height=180, scale=1.0, saveToFile=True,
        movieFile=os.path.join(directory, "offscreen"))
    result = playblastUI.Playblast(api, options)
    _Check(result is not None and len(result.paths) == 24,
           "24 frames written")
    _Check(os.path.basename(result.paths[0]) == "offscreen.1001.png",
           "Maya naming with padding: %s" % result.paths[0])
    first = _Pixels(QtGui, result.paths[0])
    middle = _Pixels(QtGui, result.paths[-1])
    _Check((first.width(), first.height()) == (320, 180),
           "custom size, larger or smaller than the window")
    _Check(first != middle, "the rig moves the arm between frames")
    _Check(app._dataModel.currentFrame == frame, "the frame is put back")
    _Check(not app._dataModel.playing, "not left playing")
    _Check(view._physicalWindowSize == physical and
           "width" not in view.__dict__ and "DrawAxis" not in view.__dict__
           and "devicePixelRatioF" not in view.__dict__,
           "the viewport's own size is untouched")
    _Check(view.updatesEnabled(), "viewport updates resume")

    # Without ornaments: the HUD's render timings differ run to run.
    settings = app._dataModel.viewSettings
    hud = settings.showHUD
    bare = playblastUI.Playblast(api, options.Copy(
        showOrnaments=False, endTime=1001,
        movieFile=os.path.join(directory, "bare")))
    _Check(bare is not None and settings.showHUD == hud and
           "DrawAxis" not in view.__dict__,
           "ornaments come back after a playblast without them")
    again = playblastUI.Playblast(api, options.Copy(
        offScreen=False, showOrnaments=False, endTime=1001,
        movieFile=os.path.join(directory, "onscreen")))
    _Check(again is not None and len(again.paths) == 1, "on screen")
    _Check(_Pixels(QtGui, again.paths[0]) ==
           _Pixels(QtGui, bare.paths[0]),
           "on screen and offscreen render the same frame alike")

    window = options.Copy(displaySize=model.SIZE_WINDOW, scale=0.5,
                          endTime=1002, encoding="jpg",
                          movieFile=os.path.join(directory, "window"))
    shot = playblastUI.Playblast(api, window)
    width, height = view.GetPhysicalWindowSize()
    image = _Pixels(QtGui, shot.paths[0])
    _Check((image.width(), image.height()) ==
           (int(round(width * 0.5)), int(round(height * 0.5))),
           "From Window at half scale")
    bareWindow = window.Copy(showOrnaments=False, endTime=1001,
                             encoding="png")
    full = playblastUI.Playblast(api, bareWindow.Copy(
        scale=1.0, movieFile=os.path.join(directory, "full")))
    half = playblastUI.Playblast(api, bareWindow.Copy(
        movieFile=os.path.join(directory, "half")))
    filtered = _Pixels(QtGui, full.paths[0]).convertToFormat(
        QtGui.QImage.Format_RGB32).scaled(
            image.width(), image.height(), QtCore.Qt.IgnoreAspectRatio,
            QtCore.Qt.SmoothTransformation)
    _Check(_Pixels(QtGui, half.paths[0]).convertToFormat(
        QtGui.QImage.Format_RGB32) == filtered,
           "below full scale, the display-size frame is filtered down")

    viewer = playblastUI.ShowResult(result, api.qMainWindow, "test")
    _Check(viewer._slider.maximum() == 23, "the flipbook holds every frame")
    viewer._Seek(5)
    _Check(viewer._frame.text() == "1006", "frame numbers in the viewer")
    viewer.close()

    if model.FindFfmpeg() is None:
        print("RIGEXEC_PLAYBLAST: no ffmpeg, movie formats skipped")
        return
    for format, encoding in ((model.FORMAT_QT, model.H264),
                             (model.FORMAT_AVI, model.MJPEG)):
        movie = playblastUI.Playblast(api, options.Copy(
            format=format, encoding=encoding, endTime=1010,
            movieFile=os.path.join(directory, "movie")))
        _Check(movie is not None and os.path.getsize(movie.paths[0]) > 0,
               "%s movie written" % format)
        _Check(movie.paths[0].endswith(model.MOVIE_EXTENSIONS[format]),
               "movie extension")


def _TestOptionsWindow(api):
    import playblastModel as model
    import playblastUI

    dialog = playblastUI.ShowOptions(api)
    _Check(playblastUI.ShowOptions(api) is dialog, "one window a session")
    dialog.SetOptions(model.PlayblastOptions())
    _Check((dialog.startTime.value(), dialog.endTime.value()) ==
           (1001, 1048) and not dialog.startTime.isEnabled(),
           "Start/End shows the time slider range until chosen")
    dialog.startEnd.setChecked(True)
    _Check(dialog.Options().timeRange == model.START_END and
           (dialog.Options().startTime, dialog.Options().endTime) ==
           (1001, 1048), "choosing Start/End starts from that range")
    options = model.PlayblastOptions(
        format=model.FORMAT_IMAGE, encoding="jpg", quality=55,
        displaySize=model.SIZE_CUSTOM, width=800, height=450, scale=0.75,
        timeRange=model.START_END, startTime=1010, endTime=1020,
        framePadding=6, offScreen=True, showOrnaments=False,
        saveToFile=True, movieFile="C:/out/blast")
    dialog.SetOptions(options)
    _Check(dialog.Options().ToDict() == options.Normalized(
        dialog._imageEncodings).ToDict(), "the window round-trips options")
    _Check(not dialog.removeTemporaryFiles.isEnabled(),
           "temporary files exist only for movies")
    dialog.displaySize.setCurrentIndex(
        dialog.displaySize.findData(model.SIZE_WINDOW))
    _Check(not dialog.width_.isEnabled() and
           dialog.Options().width == 800,
           "From Window keeps the custom size for later")
    dialog.close()


def testUsdviewInputFunction(appController):
    from pxr.Usdviewq.qt import QtGui, QtWidgets
    import rigExecUsdview

    api = appController._usdviewApi
    container = rigExecUsdview.ContainerFor(api)
    _Check(container is not None, "the RigExec container is loaded")
    extras = getattr(api.qMainWindow, "_rigExecTimeline", None)
    _Check(extras is not None, "the timeline additions are installed")
    row = appController._ui.playButtonContainer
    index = row.indexOf(appController._ui.playButton)
    _Check(row.indexOf(extras.playblastButton) == index + 1 and
           row.indexOf(extras.optionBox) == index + 2,
           "Playblast and its option box sit beside Play")

    _TestFrameRange(appController, extras, container._UndoStack(),
                    QtWidgets)
    directory = tempfile.mkdtemp(prefix="rigExecPlayblastTest_")
    try:
        _TestPlayblast(appController, api, QtGui, QtWidgets, directory)
        _TestOptionsWindow(api)
    finally:
        if not os.environ.get("RIGEXEC_PLAYBLAST_KEEP"):
            shutil.rmtree(directory, ignore_errors=True)
        else:
            print("RIGEXEC_PLAYBLAST frames kept in %s" % directory)
    print("RIGEXEC_TIMELINE_OK frame range fields, undo, outside edits, "
          "offscreen, on screen, window size, flipbook, options window")
