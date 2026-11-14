#
# RigExec usdview timeline row: editable start and end fields for the
# stage's frame range in place of usdview's read-only stage range labels.
# Any change to the stage's time metadata, from the fields or elsewhere,
# is re-applied to usdview through timelineModel.SyncController.
#
import os
import sys

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import editorExtension
    import timelineModel
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import editorExtension
    import timelineModel

_START_TIP = ("Start of the stage's frame range (startTimeCode). Usdview's "
              "playback range follows it.")
_END_TIP = ("End of the stage's frame range (endTimeCode). Usdview's "
            "playback range follows it.")


class TimelineExtras(QtCore.QObject):
    """The frame range fields of one session."""

    def __init__(self, usdviewApi, controller, undoStack):
        super(TimelineExtras, self).__init__(usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._controller = controller
        self._dataModel = controller._dataModel
        self._undoStack = undoStack
        self._noticeKey = None
        self._state = None
        ui = controller._ui
        self.startField = self._ReplaceLabel(
            ui.sliderTopContainer, ui.stageBegin, ui.rangeBegin,
            "rigExecFrameStart", _START_TIP)
        self.endField = self._ReplaceLabel(
            ui.sliderTopContainer, ui.stageEnd, ui.rangeEnd,
            "rigExecFrameEnd", _END_TIP)
        self.startField.editingFinished.connect(
            lambda: self._Commit(self.startField, True))
        self.endField.editingFinished.connect(
            lambda: self._Commit(self.endField, False))

        self._refreshTimer = QtCore.QTimer(self)
        self._refreshTimer.setSingleShot(True)
        self._refreshTimer.setInterval(0)
        self._refreshTimer.timeout.connect(self.Refresh)
        self._dataModel.signalStageReplaced.connect(self._OnStageReplaced)
        usdviewApi.qMainWindow.destroyed.connect(
            lambda *args: self._RevokeNotice())
        self._OnStageReplaced()

    def _ReplaceLabel(self, layout, label, model, name, tip):
        field = QtWidgets.QLineEdit(label.parentWidget())
        field.setObjectName(name)
        field.setSizePolicy(model.sizePolicy())
        field.setMaximumSize(model.maximumSize())
        field.setAlignment(model.alignment())
        field.setValidator(QtGui.QDoubleValidator(field))
        field.setToolTip(tip)
        index = layout.indexOf(label)
        layout.insertWidget(index if index >= 0 else layout.count(), field,
                            0, QtCore.Qt.AlignVCenter)
        label.hide()
        return field

    def _RevokeNotice(self):
        if self._noticeKey is not None:
            try:
                self._noticeKey.Revoke()
            except Exception:
                pass
        self._noticeKey = None

    def Shutdown(self):
        self._RevokeNotice()

    def _OnStageReplaced(self):
        self._RevokeNotice()
        stage = self._dataModel.stage
        self._state = timelineModel.TimeState(stage) if stage else None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._OnObjectsChanged, stage)
        self._ShowFields()

    def _OnObjectsChanged(self, notice, stage):
        root = Sdf.Path.absoluteRootPath
        try:
            if root in notice.GetResyncedPaths() or (
                    root in notice.GetChangedInfoOnlyPaths() and
                    timelineModel.TIME_KEYS.intersection(
                        notice.GetChangedFields(root))):
                self._refreshTimer.start()
        except RuntimeError:          # the window is gone
            self._RevokeNotice()

    def _ShowFields(self):
        stage = self._dataModel.stage
        enabled = bool(stage)
        for field, value in zip(
                (self.startField, self.endField),
                timelineModel.DisplayedRange(stage) if stage else ("", "")):
            field.setEnabled(enabled)
            if field.hasFocus() and field.isModified():
                continue
            field.setText(timelineModel.FormatFrame(value) if enabled
                          else "")
            field.setModified(False)

    def Refresh(self):
        """Show the stage's range, and re-apply it to usdview when it or
        the frame rate moved since the last look."""
        stage = self._dataModel.stage
        self._ShowFields()
        if not stage:
            return
        state = timelineModel.TimeState(stage)
        if state == self._state:
            return
        previous, self._state = self._state, state
        ui = self._controller._ui
        start, end = timelineModel.DisplayedRange(stage)
        ui.stageBegin.setText(str(start))
        ui.stageEnd.setText(str(end))
        if previous is None:
            return
        try:
            timelineModel.SyncController(self._controller, previous, state)
        except Exception as error:  # noqa: BLE001 -- never into Qt
            Tf.Warn("rigExecUsdview: could not update the timeline: %s"
                    % error)

    def _Commit(self, field, isStart):
        stage = self._dataModel.stage
        if not stage or not field.isModified():
            self._ShowFields()
            return
        field.setModified(False)
        value = timelineModel.ParseFrame(field.text())
        if value is None:
            self._ShowFields()
            return
        current = timelineModel.DisplayedRange(stage)
        edited = timelineModel.EditedRange(
            current, start=value if isStart else None,
            end=None if isStart else value)
        if (edited == current and
                timelineModel.StageRange(stage) is not None):
            self._ShowFields()
            return
        undoStack = self._undoStack() if callable(self._undoStack) else (
            self._undoStack)
        try:
            with editorExtension.DocumentEdit(self._api):
                timelineModel.AuthorRange(
                    stage, stage.GetEditTarget().GetLayer(),
                    edited[0], edited[1], undoStack)
        except (RuntimeError, ValueError, Tf.ErrorException) as error:
            self._api.PrintStatus("Frame range unchanged: %s" % error)
        self.Refresh()


def Install(usdviewApi, undoStack=None):
    """The session's TimelineExtras, created once per main window; None
    when usdview's timeline cannot be found."""
    window = usdviewApi.qMainWindow
    existing = getattr(window, "_rigExecTimeline", None)
    if existing is not None:
        return existing
    controller = getattr(usdviewApi, "_UsdviewApi__appController", None)
    ui = getattr(controller, "_ui", None)
    needed = ("sliderTopContainer", "stageBegin", "stageEnd", "rangeBegin",
              "rangeEnd")
    if ui is None or not all(hasattr(ui, name) for name in needed):
        return None
    extras = TimelineExtras(usdviewApi, controller, undoStack)
    window._rigExecTimeline = extras
    return extras
