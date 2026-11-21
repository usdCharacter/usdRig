#
# RigExec usdview plugin: the volumetric weight panel and its Qt widgets.
#
# The attribute widgets at the top of this file -- SetAtTime,
# AttributeValueMayHaveChanged, AttributeValueWidget and
# AttributeValueSelectWidget -- are adapted from Pixar's OpenUSD sample
# plugin at
#   extras/exec/examples/invertibleRigsExample/
#       invertibleRigsExampleUsdviewPlugin/plugin.py
# Copyright 2026 Pixar, licensed under the terms set forth in the
# LICENSE.txt file available at https://openusd.org/license.  The
# additions here are the 'sensitivity' knob on the munging line edit, the
# uniform-variability guard in SetAtTime, and Detach() so a rebuilt panel
# can revoke its notice keys before Qt deletes the C++ widgets.
#
# SetAtTime and AttributeValueMayHaveChanged moved to
# volumeWeightModel.py with the rest of the authoring; the names
# re-exported below are the model's, kept here so existing importers
# keep working.
#
import os
import sys

from pxr import Sdf, Tf, Ts, Usd, Vt
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import sessionRegistry
    import volumeWeightModel
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import sessionRegistry
    import volumeWeightModel

VOLUME_WEIGHT_TYPE_NAMES = volumeWeightModel.VOLUME_WEIGHT_TYPE_NAMES
PLACED_WEIGHT_TYPE_NAMES = volumeWeightModel.PLACED_WEIGHT_TYPE_NAMES
WEIGHT_OBJECT_TYPE_NAMES = volumeWeightModel.WEIGHT_OBJECT_TYPE_NAMES
SPLINE_PROFILE_NAMES = volumeWeightModel.SPLINE_PROFILE_NAMES
_MakeKnot = volumeWeightModel._MakeKnot
_MakeProfileSpline = volumeWeightModel._MakeProfileSpline
SetUndoStack = volumeWeightModel.SetUndoStack
Recording = volumeWeightModel.Recording
_Scope = volumeWeightModel._Scope
_AttributeScope = volumeWeightModel._AttributeScope
SetAtTime = volumeWeightModel.SetAtTime
_SetAtTime = volumeWeightModel._SetAtTime
SetVisibleAtTime = volumeWeightModel.SetVisibleAtTime
_SetVisibleAtTime = volumeWeightModel._SetVisibleAtTime
AttributeValueMayHaveChanged = volumeWeightModel.AttributeValueMayHaveChanged
EnsureDefaultSpline = volumeWeightModel.EnsureDefaultSpline
GetSplineKnotPoints = volumeWeightModel.GetSplineKnotPoints
InsertSplineKnot = volumeWeightModel.InsertSplineKnot
MoveSplineKnot = volumeWeightModel.MoveSplineKnot
DeleteSplineKnot = volumeWeightModel.DeleteSplineKnot
FindRigPrim = volumeWeightModel.FindRigPrim
FindWeightsScopeAncestor = volumeWeightModel.FindWeightsScopeAncestor
ChooseWeightParentPath = volumeWeightModel.ChooseWeightParentPath
ChooseWeightParentPrim = volumeWeightModel.ChooseWeightParentPrim
MakeUniquePrimName = volumeWeightModel.MakeUniquePrimName
DefaultPrimNameForType = volumeWeightModel.DefaultPrimNameForType
FindPointsPropertyPath = volumeWeightModel.FindPointsPropertyPath
ComputeCentreAndSize = volumeWeightModel.ComputeCentreAndSize
GetWeightTargetPrim = volumeWeightModel.GetWeightTargetPrim
ComputeScrubScale = volumeWeightModel.ComputeScrubScale
SetPlacementFromPoint = volumeWeightModel.SetPlacementFromPoint
SeedFalloffForSize = volumeWeightModel.SeedFalloffForSize
CreateVolumeWeightPrim = volumeWeightModel.CreateVolumeWeightPrim
SnapWeightToPrim = volumeWeightModel.SnapWeightToPrim
GetInputWeightPaths = volumeWeightModel.GetInputWeightPaths
SetInputWeightPaths = volumeWeightModel.SetInputWeightPaths
AddInputWeightPaths = volumeWeightModel.AddInputWeightPaths
RemoveInputWeightPaths = volumeWeightModel.RemoveInputWeightPaths
IsVolumeWeightPrim = volumeWeightModel.IsVolumeWeightPrim
IsWeightObjectPrim = volumeWeightModel.IsWeightObjectPrim


# Qt widgets

def _EventPos(event):
    """
    Returns a QPointF for a mouse event on either PySide binding.

    PySide6 deprecates QMouseEvent.pos() in favour of position(); the
    reference plugin predates that, so normalize here once instead of
    at every call site.
    """
    if hasattr(event, "position"):
        return event.position()
    return QtCore.QPointF(event.pos())


def _DetachWidget(widget):
    """
    Revokes a widget's notice key, if it has one, before Qt destroys it.

    Without this a rebuilt panel leaves live listeners pointing at
    widgets whose C++ half Qt has already deleted, and the next stage
    edit crashes usdview.
    """
    if widget is None:
        return
    detach = getattr(widget, "Detach", None)
    if detach is not None:
        detach()


def _ClearLayout(layout):
    """
    Empties a layout, detaching every widget on the way out.

    QFormLayout is special-cased because takeAt() empties a row but
    leaves the ROW itself behind: a panel that rebuilds on every
    selection change would grow a blank row per parameter per rebuild
    and march off the bottom of the window.  removeRow() is the call
    that actually shrinks the form.
    """
    if isinstance(layout, QtWidgets.QFormLayout):
        for row in reversed(range(layout.rowCount())):
            for role in (QtWidgets.QFormLayout.LabelRole,
                         QtWidgets.QFormLayout.FieldRole,
                         QtWidgets.QFormLayout.SpanningRole):
                item = layout.itemAt(row, role)
                if item is not None:
                    _DetachWidget(item.widget())
            layout.removeRow(row)
        return

    while layout.count():
        item = layout.takeAt(0)
        widget = item.widget()
        if widget is not None:
            _DetachWidget(widget)
            widget.setParent(None)
            widget.deleteLater()
            continue
        child = item.layout()
        if child is not None:
            _ClearLayout(child)


class SplineEdit:
    """Detached curve edit; authoring is deferred until the gesture commits."""
    def __init__(self, attr):
        self.spline = attr.GetSpline()

    def GetSpline(self):
        return Ts.Spline(self.spline)

    def SetSpline(self, spline):
        self.spline = Ts.Spline(spline)


_weightGestures = sessionRegistry.SessionRegistry("weight gestures")


class WeightGesture:
    """Owner-thread preview, followed by one queued authoring transaction."""

    def __init__(self, api, attr):
        import gizmoPreview
        previous = _weightGestures.Get(api)
        if previous is not None and not previous.closed:
            previous.Cancel()
        _weightGestures.Set(api, self)
        self.api = api
        self.stage = attr.GetStage()
        self.time = api.dataModel.currentFrame
        self.target = self.stage.GetEditTarget()
        self.pending = {}
        self.closed = False
        self.preview = gizmoPreview
        self._scheduled = False

    def Update(self, attr, value):
        if self.closed:
            return
        self.pending[attr.GetPath()] = value
        if not self._scheduled:
            self._scheduled = True
            QtCore.QTimer.singleShot(16, self._Publish)

    def _Publish(self):
        self._scheduled = False
        if self.closed or self.api.dataModel.stage != self.stage:
            return
        import gizmoMath
        values = {}
        for path, value in self.pending.items():
            if isinstance(value, Ts.Spline):
                # Match RigExecFalloffLutSize so release preserves the exact ramp.
                values[path.GetPrimPath().AppendProperty("inputs:falloffSamples")] = Vt.FloatArray(
                    [float(value.Eval(i / 256.0)) for i in range(257)])
            else:
                values[path] = value
        gizmoMath.SetPreviewValues(values, stage=self.stage)
        self.preview.Push(values, session=self.api, stage=self.stage)
        self.api.UpdateViewport()

    def Cancel(self):
        if self.closed:
            return
        self.closed = True
        if self.pending and _weightGestures.Get(self.api) is self:
            self.preview.End(session=self.api, stage=self.stage)
            self.api.UpdateViewport()

    def Commit(self):
        if self.closed:
            return
        self.closed = True
        pending = dict(self.pending)
        if not pending:
            return
        def finish():
            if self.api.dataModel.stage != self.stage:
                return
            # Drop the preview without republishing: the commit's stage
            # notice publishes once. Republish when nothing was authored.
            owned = _weightGestures.Get(self.api) is self
            if owned:
                self.preview.End(session=self.api, stage=self.stage,
                                 publish=False)
            authored = False
            try:
                paths = list({p.GetPrimPath() for p in pending})
                with Usd.EditContext(self.stage, self.target):
                    with Recording(self.stage, paths, "Edit weight"):
                        for path, value in pending.items():
                            attr = self.stage.GetAttributeAtPath(path)
                            if not attr:
                                continue
                            if isinstance(value, Ts.Spline):
                                if attr.GetSpline() != value:
                                    attr.SetSpline(value)
                                    authored = True
                            elif attr.Get(self.time) != value:
                                SetVisibleAtTime(attr, value, self.time)
                                authored = True
            finally:
                if owned and not authored:
                    self.preview.Republish(session=self.api)
                self.api.UpdateViewport()
        QtCore.QTimer.singleShot(0, finish)


class AttributeValueWidget(QtWidgets.QLineEdit):
    """
    A widget for displaying and authoring the value of a UsdAttribute.

    The widget text is initialized with the current value of the
    attribute.  If the attribute's value type supports munging (int or
    floating point), then the text is editable by entering a value, or
    by munging (clicking inside the text box and dragging the mouse
    horizontally).

    Adapted from Pixar's invertibleRigsExample usdview plugin.  The
    'sensitivity' keyword is the local addition: a falloff radius on a
    character measured in tens of units needs a much coarser scrub than
    inputs:invert, which lives in 0..1 and is unusable at the reference
    plugin's fixed 0.02 per pixel.
    """

    __mungButton = QtCore.Qt.MouseButton.LeftButton

    def __init__(self, usdAttribute, usdviewApi, parent=None,
                 sensitivity=0.02, minimum=None, maximum=None):
        super(AttributeValueWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._sensitivity = float(sensitivity)
        self._minimum = minimum
        self._maximum = maximum
        self._noticeKey = None

        # To enable "mung" behavior, this needs to track the position of
        # the mouse and the initial value when the button is clicked.
        self._mungBaseMousePos = None
        self._mungBaseValue = None
        self._gesture = None

        # Configure the widget for the attribute's value type.
        self._valueTypeName = self._usdAttribute.GetTypeName()
        if self._valueTypeName == Sdf.ValueTypeNames.Int:
            self.setValidator(QtGui.QIntValidator(parent=self))
        elif AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            self.setValidator(QtGui.QDoubleValidator(parent=self))
        elif (self._valueTypeName == Sdf.ValueTypeNames.Token or
              self._valueTypeName == Sdf.ValueTypeNames.String):
            pass
        else:
            self.setReadOnly(True)





        self._cachedText = self._TypedValueToString(
            self._usdAttribute.Get(self._usdviewApi.dataModel.currentFrame))
        self.setText(self._cachedText)

        self._usdviewApi.dataModel.currentFrameChanged.connect(
            self._onCurrentFrameChanged)
        self.editingFinished.connect(self._onEditingFinished)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())
        import gizmoPreview
        gizmoPreview.AddListener(self._onPreview)

    def _onPreview(self, pending):
        if self._usdAttribute.GetStage() != self._usdviewApi.dataModel.stage:
            return
        if pending is not None:
            gesture = _weightGestures.Get(self._usdviewApi)
            if gesture is None or gesture.closed:
                return
            value = pending.get(self._usdAttribute.GetPath())
            if not isinstance(value, (int, float)):
                return
        else:
            value = self._usdAttribute.Get(self._usdviewApi.dataModel.currentFrame)
        self._cachedText = self._TypedValueToString(value)
        self.setText(self._cachedText)

    def Detach(self):
        if getattr(self, "_detached", False):
            return
        self._detached = True
        # Revoke notices before Qt deletes the widget.
        import gizmoPreview
        gizmoPreview.RemoveListener(self._onPreview)
        if self._gesture is not None:
            self._gesture.Cancel()
            self._gesture = None
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        try:
            self._usdviewApi.dataModel.currentFrameChanged.disconnect(
                self._onCurrentFrameChanged)
        except (RuntimeError, TypeError):
            pass

    @staticmethod
    def _IsFloatingPointType(valueTypeName):
        """
        Returns true if valueTypeName is a floating point value type
        (regardless of precision).
        """
        return (valueTypeName == Sdf.ValueTypeNames.Double or
                valueTypeName == Sdf.ValueTypeNames.Float or
                valueTypeName == Sdf.ValueTypeNames.Half)

    @staticmethod
    def _IsNumericType(valueTypeName):
        """
        Returns true if valueTypeName is either an integer or floating
        point type.
        """
        return (AttributeValueWidget._IsFloatingPointType(valueTypeName) or
                valueTypeName == Sdf.ValueTypeNames.Int)

    def _TypedValueToString(self, typedValue):
        """
        Makes sure typed values are displayed nicely in the widget.
        Primarily, rounds floating point values to a few decimals.
        """
        if typedValue is None:
            return ""
        if AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            return str(round(typedValue, 3))
        return str(typedValue)

    def _StringToTypedValue(self, textValue):
        """
        Attempts to convert a string to a Python type corresponding to
        valueTypeName.  For numeric types, failure results in a zero
        value.  Raises TypeError for unsupported types.
        """
        if self._valueTypeName == Sdf.ValueTypeNames.Int:
            try:
                newValue = int(textValue)
            except ValueError:
                newValue = 0
        elif AttributeValueWidget._IsFloatingPointType(self._valueTypeName):
            try:
                newValue = float(textValue)
            except ValueError:
                newValue = 0.0
        elif (self._valueTypeName == Sdf.ValueTypeNames.Token or
              self._valueTypeName == Sdf.ValueTypeNames.String):
            newValue = textValue
        else:
            raise TypeError(
                "ValueTypeName %s not supported." % self._valueTypeName)

        return newValue

    def _ClampValue(self, value):
        """
        Applies the optional range given at construction.  invert and
        strength are meaningful in 0..1 and a scrub that runs off the
        end is an accident, not an intent.
        """
        if self._minimum is not None:
            value = max(value, self._minimum)
        if self._maximum is not None:
            value = min(value, self._maximum)
        return value

    def _authorValue(self, newValue):
        """
        Authors newValue to the underlying UsdAttribute at the current
        time, in the stage's current edit target.
        """
        currentTime = self._usdviewApi.dataModel.currentFrame
        if AttributeValueWidget._IsNumericType(self._valueTypeName):
            newValue = self._ClampValue(newValue)

        if self._gesture is not None:
            self._gesture.Update(self._usdAttribute, newValue)
            self._cachedText = self._TypedValueToString(newValue)
            self.setText(self._cachedText)
            return
        try:
            SetVisibleAtTime(self._usdAttribute, newValue, currentTime)
        except Exception as err:
            Tf.Warn("Failed to set <%s> to value: %s : %s"
                    % (self._usdAttribute.GetPath(), newValue, err))

    def _onCurrentFrameChanged(self, frame):
        """
        Handler for changes to currentFrame: re-read and redisplay.
        """
        if self._gesture is not None:
            self._gesture.Cancel()
            self._gesture = None
        self._cachedText = self._TypedValueToString(
            self._usdAttribute.Get(frame))
        self.setText(self._cachedText)

    def _onEditingFinished(self):
        """
        When the user is done editing, author the value.
        """
        if self._cachedText == self.text():
            return

        try:
            newValue = self._StringToTypedValue(self.text())
        except TypeError:
            return

        self._authorValue(newValue)

    def _onObjectsChanged(self, notice, stage):
        """
        Handler for Usd's ObjectsChanged notice: re-read and redisplay.
        """
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return

        currentTime = self._usdviewApi.dataModel.currentFrame
        value = self._usdAttribute.Get(currentTime)

        # UsdAttribute.Get() can return None -- no authored value and no
        # schema fallback, or an attribute holding only an empty spline.
        if value is None:
            return

        self._cachedText = self._TypedValueToString(value)
        self.setText(self._cachedText)

    def mousePressEvent(self, event):
        """
        Begins a mung operation.
        """
        if (event.buttons() == self.__mungButton and not self.isReadOnly()
                and self._IsNumericType(self._valueTypeName)):
            self._gesture = WeightGesture(self._usdviewApi, self._usdAttribute)
            self._mungBaseMousePos = _EventPos(event)
            self._mungBaseValue = self._StringToTypedValue(self.text())
            self.setCursor(QtGui.QCursor(QtCore.Qt.SizeHorCursor))
            self.selectAll()
        else:
            super(AttributeValueWidget, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        """
        Ends a mung operation.
        """
        if event.button() == self.__mungButton and not self.isReadOnly():
            if self._gesture is not None:
                self._gesture.Commit()
                self._gesture = None
            self._mungBaseMousePos = None
            self._mungBaseValue = None
            self.setCursor(QtGui.QCursor(QtCore.Qt.IBeamCursor))
        else:
            super(AttributeValueWidget, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        """
        If munging, updates the UsdAttribute value in response to the
        mouse position.
        """
        if (event.buttons() == self.__mungButton and self._mungBaseMousePos
                and self._IsNumericType(self._valueTypeName)):
            delta = _EventPos(event).x() - self._mungBaseMousePos.x()
            delta *= self._sensitivity
            value = self._mungBaseValue + delta
            if self._valueTypeName == Sdf.ValueTypeNames.Int:
                value = int(round(value))
            self._authorValue(value)
            self.selectAll()
        else:
            super(AttributeValueWidget, self).mouseMoveEvent(event)


class AttributeValueSelectWidget(QtWidgets.QComboBox):
    """
    A widget for displaying and authoring the value of a UsdAttribute
    whose value is selected from a list of possible values.

    Adapted from Pixar's invertibleRigsExample usdview plugin; Detach()
    and the valueChanged signal are the local additions.
    """

    # Emitted after a successful author so the panel can react to, say,
    # falloffProfile becoming "curve" without re-reading the stage.
    valueChanged = QtCore.Signal(str)

    def __init__(self, usdAttribute, usdviewApi, parent=None):
        super(AttributeValueSelectWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._noticeKey = None

        if self._usdAttribute is None or not self._usdAttribute:
            self.setEnabled(False)
            return

        if self._usdAttribute.HasMetadata("allowedTokens"):
            self._allowedTokens = [
                str(x) for x in
                self._usdAttribute.GetMetadata("allowedTokens")]
            self.addItems(self._allowedTokens)
        else:
            self._allowedTokens = []
            Tf.Warn("No combo box items for <%s>"
                    % self._usdAttribute.GetPath())

        current = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if current is not None:
            self.setCurrentText(str(current))

        self._usdviewApi.dataModel.currentFrameChanged.connect(
            self._onCurrentFrameChanged)
        self.currentIndexChanged.connect(self._onSelectionChanged)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())

    def Detach(self):
        """
        Revokes the notice key and disconnects the frame signal.
        """
        if getattr(self, "_detached", False):
            return
        self._detached = True
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        try:
            self._usdviewApi.dataModel.currentFrameChanged.disconnect(
                self._onCurrentFrameChanged)
        except (RuntimeError, TypeError):
            pass

    def _onCurrentFrameChanged(self, frame):
        """
        Re-read the attribute and update the selected item.  Signals are
        blocked so the resulting currentIndexChanged does not author
        the value straight back.
        """
        value = self._usdAttribute.Get(time=frame)
        if value is None:
            return
        self.blockSignals(True)
        self.setCurrentText(str(value))
        self.blockSignals(False)

    def _onSelectionChanged(self, index):
        """
        When the selection changes, author the value to the attribute.
        """
        if index < 0 or index >= len(self._allowedTokens):
            return
        newValue = self._allowedTokens[index]
        currentTime = self._usdviewApi.dataModel.currentFrame

        try:
            SetAtTime(self._usdAttribute, newValue, currentTime)
        except Exception as err:
            Tf.Warn("Failed to set <%s> to value: %s : %s"
                    % (self._usdAttribute.GetPath(), newValue, err))
            return

        self.valueChanged.emit(newValue)

    def _onObjectsChanged(self, notice, stage):
        """
        Re-read the attribute and update the selected item, blocking
        signals for the same reason as _onCurrentFrameChanged.
        """
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return
        value = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if value is None:
            return
        self.blockSignals(True)
        self.setCurrentText(str(value))
        self.blockSignals(False)


class ReadPhaseSelectWidget(QtWidgets.QComboBox):
    """
    A combo for the rigExecReadPhase metadata on one input relationship.

    A read phase is metadata on the relationship that names the input,
    never an attribute, so it is authored with SetMetadata. `base` is the
    default and is authored by clearing the field.
    """

    def __init__(self, relationship, phases, parent=None):
        super(ReadPhaseSelectWidget, self).__init__(parent)
        self._relationship = relationship
        self._phases = list(phases)
        self._noticeKey = None
        if not relationship:
            self.setEnabled(False)
            return
        self.addItems(self._phases)
        self._Refresh()
        self.currentIndexChanged.connect(self._onSelectionChanged)
        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._onObjectsChanged,
            relationship.GetStage())

    def Detach(self):
        """Revokes the notice key."""
        if getattr(self, "_detached", False):
            return
        self._detached = True
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None

    def _Current(self):
        value = self._relationship.GetMetadata("rigExecReadPhase")
        return str(value) if value else "base"

    def _Refresh(self):
        self.blockSignals(True)
        self.setCurrentText(self._Current())
        self.blockSignals(False)

    def _onSelectionChanged(self, index):
        if index < 0 or index >= len(self._phases):
            return
        phase = self._phases[index]
        if phase == self._Current():
            return
        path = self._relationship.GetPath()
        try:
            with Recording(self._relationship.GetStage(),
                           [path.GetPrimPath()],
                           "Set read phase %s" % path.name):
                if phase == "base":
                    self._relationship.ClearMetadata("rigExecReadPhase")
                else:
                    self._relationship.SetMetadata("rigExecReadPhase", phase)
        except Exception as err:
            Tf.Warn("Failed to set the read phase of <%s> to %s : %s"
                    % (path, phase, err))
            self._Refresh()

    def _onObjectsChanged(self, notice, stage):
        if AttributeValueMayHaveChanged(self._relationship.GetPath(),
                                        notice):
            self._Refresh()


class AttributeValueSliderWidget(QtWidgets.QWidget):
    """
    A munging line edit paired with a horizontal slider, for the
    parameters whose USEFUL range is 0..1 (inputs:invert,
    inputs:strength).

    The slider is a convenience, not the mechanism: the line edit is
    still the authority and still scrubs, so this widget behaves
    identically to a bare AttributeValueWidget if the slider is ignored.

    editMinimum/editMaximum are the hard limits the LINE EDIT enforces,
    and they are deliberately separate from the slider's travel.
    inputs:strength is a final multiplier, so 0..1 is the range worth
    giving a slider but not a range the artist may be held to: clamping
    the edit there silently turned a typed 2.5 into 1.0.  Pass None to
    leave that end open.
    """

    __resolution = 1000

    def __init__(self, usdAttribute, usdviewApi, parent=None,
                 minimum=0.0, maximum=1.0,
                 editMinimum=0.0, editMaximum=1.0):
        super(AttributeValueSliderWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = usdAttribute
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._noticeKey = None

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.setLayout(layout)

        # A 0..1 parameter wants a fine scrub; 400 pixels of drag should
        # cross the whole range rather than eight times over.
        self._edit = AttributeValueWidget(
            usdAttribute, usdviewApi, parent=self,
            sensitivity=(self._maximum - self._minimum) / 400.0,
            minimum=editMinimum, maximum=editMaximum)
        self._edit.setMaximumWidth(70)
        layout.addWidget(self._edit)

        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self._slider.setMinimum(0)
        self._slider.setMaximum(self.__resolution)
        self._slider.setValue(self._ValueToSlider(
            usdAttribute.Get(usdviewApi.dataModel.currentFrame)))
        self._slider.valueChanged.connect(self._onSliderChanged)
        # Drag samples preview; release queues one undoable commit.
        self._slider.sliderPressed.connect(self._onSliderPressed)
        self._slider.sliderReleased.connect(self._onSliderReleased)
        layout.addWidget(self._slider)

        self._noticeKey = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._onObjectsChanged,
            self._usdAttribute.GetStage())

    def Detach(self):
        if getattr(self, "_detached", False):
            return
        self._detached = True
        # A selection change cancels an unfinished slider gesture.
        if self._edit._gesture is not None:
            self._edit._gesture.Cancel()
            self._edit._gesture = None
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None
        self._edit.Detach()

    def _ValueToSlider(self, value):
        if value is None:
            value = self._minimum
        span = self._maximum - self._minimum
        if span <= 0.0:
            return 0
        fraction = (float(value) - self._minimum) / span
        return int(round(min(max(fraction, 0.0), 1.0) * self.__resolution))

    def _SliderToValue(self, position):
        span = self._maximum - self._minimum
        return self._minimum + span * (float(position) / self.__resolution)

    def _onSliderPressed(self):
        self._edit._gesture = WeightGesture(self._usdviewApi, self._usdAttribute)

    def _onSliderReleased(self):
        if self._edit._gesture is not None:
            self._edit._gesture.Commit()
            self._edit._gesture = None

    def _onSliderChanged(self, position):
        self._edit._authorValue(self._SliderToValue(position))

    def _onObjectsChanged(self, notice, stage):
        if not AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            return
        value = self._usdAttribute.Get(
            self._usdviewApi.dataModel.currentFrame)
        if value is None:
            return
        self._slider.blockSignals(True)
        self._slider.setValue(self._ValueToSlider(value))
        self._slider.blockSignals(False)


class FalloffCurveWidget(QtWidgets.QWidget):
    """
    A direct-manipulation editor for the Ts spline authored on
    rigExec:falloffCurve.

    The horizontal axis is the NORMALIZED band parameter r: r = 0 is the
    outer end of the band (inputs:falloffMax, where the field is off)
    and r = 1 is the inner end (inputs:falloffMin, where it is on).  The
    vertical axis is the resulting weight in 0..1.  Both ends are
    labelled because the direction is the one thing about a falloff
    curve that is impossible to guess from the picture.

    Interaction:
      * left-drag a knot handle to move it (clamped to the unit square;
        the first and last knots keep their x);
      * left-click empty plot area to insert a knot there;
      * right-click or double-click a knot to delete it, unless only two
        remain.
    """

    __hitRadius = 8.0
    __knotRadius = 4.0
    __sampleCount = 96

    def __init__(self, usdviewApi, usdAttribute=None, parent=None):
        super(FalloffCurveWidget, self).__init__(parent)

        self._usdviewApi = usdviewApi
        self._usdAttribute = None
        self._noticeKey = None
        self._dragKnotX = None
        self._edit = None
        self._gesture = None
        self._profile = None

        self.setMinimumHeight(170)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(False)
        self.setToolTip(
            "Left-drag a point to move it, left-click to insert, "
            "right-click a point to delete.")

        self.SetAttribute(usdAttribute)

    # -- binding ------------------------------------------------------

    def SetAttribute(self, usdAttribute):
        """
        Rebinds the editor to a different rigExec:falloffCurve
        attribute (or to None).

        The widget is rebound rather than rebuilt when the selection
        changes so that a drag in progress is the only thing that can be
        interrupted, and so the notice registration churns once per
        selection instead of once per repaint.
        """
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None

        self._CancelEdit()
        self._detached = False
        self._usdAttribute = usdAttribute if (
            usdAttribute is not None and usdAttribute) else None
        self._dragKnotX = None

        if self._usdAttribute is not None:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged,
                self._onObjectsChanged,
                self._usdAttribute.GetStage())

        self.update()

    def Detach(self):
        if getattr(self, "_detached", False):
            return
        self._detached = True
        self._CancelEdit()
        if self._noticeKey is not None:
            self._noticeKey.Revoke()
            self._noticeKey = None

    def _onObjectsChanged(self, notice, stage):
        """
        External edits -- another panel, the interpreter, a layer reload
        -- must show up here, so repaint from the stage on any notice
        that touches this attribute.
        """
        if self._usdAttribute is None:
            return
        if AttributeValueMayHaveChanged(
                self._usdAttribute.GetPath(), notice):
            self.update()

    # -- geometry -----------------------------------------------------

    def _PlotRect(self):
        """
        The bordered plot box, inset to leave room for axis labels.
        """
        return QtCore.QRectF(
            36.0, 10.0,
            max(float(self.width()) - 48.0, 1.0),
            max(float(self.height()) - 34.0, 1.0))

    def _ToPixels(self, rect, x, y):
        return QtCore.QPointF(
            rect.left() + float(x) * rect.width(),
            rect.bottom() - float(y) * rect.height())

    def _ToUnit(self, rect, point):
        x = (point.x() - rect.left()) / rect.width()
        y = (rect.bottom() - point.y()) / rect.height()
        return (min(max(x, 0.0), 1.0), min(max(y, 0.0), 1.0))

    def _KnotPoints(self):
        return GetSplineKnotPoints(self._edit or self._usdAttribute)

    def _HitKnot(self, rect, position):
        """
        Returns the x of the knot under the cursor, or None.
        """
        best = None
        bestDistance = self.__hitRadius
        for x, y in self._KnotPoints():
            pixel = self._ToPixels(rect, x, y)
            distance = ((pixel.x() - position.x()) ** 2 +
                        (pixel.y() - position.y()) ** 2) ** 0.5
            if distance <= bestDistance:
                best = x
                bestDistance = distance
        return best

    # -- painting -----------------------------------------------------

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        rect = self._PlotRect()
        palette = self.palette()
        textColor = palette.color(QtGui.QPalette.WindowText)

        # Plot box.
        painter.fillRect(rect, palette.color(QtGui.QPalette.Base))
        gridPen = QtGui.QPen(QtGui.QColor(textColor.red(), textColor.green(),
                                          textColor.blue(), 48))
        gridPen.setWidth(1)
        painter.setPen(gridPen)
        for step in range(1, 4):
            fraction = step / 4.0
            painter.drawLine(
                self._ToPixels(rect, fraction, 0.0),
                self._ToPixels(rect, fraction, 1.0))
            painter.drawLine(
                self._ToPixels(rect, 0.0, fraction),
                self._ToPixels(rect, 1.0, fraction))

        borderPen = QtGui.QPen(QtGui.QColor(textColor.red(), textColor.green(),
                                            textColor.blue(), 160))
        borderPen.setWidth(1)
        painter.setPen(borderPen)
        painter.drawRect(rect)

        # Axis labels.  r = 0 is the OUTER end of the band, r = 1 the
        # inner end, which is the opposite of what most people assume
        # from a left-to-right ramp, hence the words.
        font = painter.font()
        font.setPointSizeF(max(font.pointSizeF() - 1.0, 6.0))
        painter.setFont(font)
        painter.setPen(QtGui.QPen(textColor))
        painter.drawText(
            QtCore.QRectF(rect.left(), rect.bottom() + 2.0,
                          rect.width() * 0.5, 16.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            "outer (falloffMax)")
        painter.drawText(
            QtCore.QRectF(rect.left() + rect.width() * 0.5,
                          rect.bottom() + 2.0, rect.width() * 0.5, 16.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
            "inner (falloffMin)")
        painter.drawText(
            QtCore.QRectF(0.0, rect.top() - 6.0, 32.0, 14.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, "w=1")
        painter.drawText(
            QtCore.QRectF(0.0, rect.bottom() - 8.0, 32.0, 14.0),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, "w=0")

        if self._usdAttribute is None:
            painter.setPen(QtGui.QPen(
                QtGui.QColor(textColor.red(), textColor.green(),
                             textColor.blue(), 140)))
            painter.drawText(rect, QtCore.Qt.AlignCenter,
                             "no falloff curve attribute")
            painter.end()
            return

        spline = (self._edit or self._usdAttribute).GetSpline()
        knots = self._KnotPoints()
        if self._profile in SPLINE_PROFILE_NAMES:
            spline = _MakeProfileSpline(self._profile)
            knots = [(float(t), float(spline.GetKnot(t).GetValue()))
                     for t in spline.GetKnots()]
        if not spline or not knots:
            painter.setPen(QtGui.QPen(
                QtGui.QColor(textColor.red(), textColor.green(),
                             textColor.blue(), 140)))
            painter.drawText(rect, QtCore.Qt.AlignCenter,
                             "no spline authored -- pick a preset below")
            painter.end()
            return

        # The curve itself, sampled densely enough that a bezier segment
        # reads as a curve rather than as a chord.
        path = QtGui.QPainterPath()
        for index in range(self.__sampleCount + 1):
            x = float(index) / self.__sampleCount
            if self._profile == "linear":
                y = x
            elif self._profile == "smooth":
                y = x*x*(3-2*x)
            elif self._profile == "easeIn":
                y = x*x
            elif self._profile == "easeOut":
                y = 1-(1-x)*(1-x)
            elif self._profile == "constant":
                y = 1.0
            else:
                y = float(spline.Eval(x))
            point = self._ToPixels(rect, x, min(max(y, 0.0), 1.0))
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)

        curvePen = QtGui.QPen(QtGui.QColor(230, 90, 70))
        curvePen.setWidthF(2.0)
        painter.setPen(curvePen)
        painter.drawPath(path)

        # Knot handles.
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20)))
        for x, y in knots:
            active = (self._dragKnotX is not None and
                      abs(x - self._dragKnotX) < 1e-9)
            painter.setBrush(QtGui.QBrush(
                QtGui.QColor(255, 220, 90) if active
                else QtGui.QColor(250, 250, 250)))
            painter.drawEllipse(
                self._ToPixels(rect, x, min(max(y, 0.0), 1.0)),
                self.__knotRadius, self.__knotRadius)

        painter.end()

    # -- interaction --------------------------------------------------

    def mousePressEvent(self, event):
        if self._usdAttribute is None:
            return

        if event.button() not in (QtCore.Qt.LeftButton, QtCore.Qt.RightButton):
            return
        self._BeginEdit()
        rect = self._PlotRect()
        position = _EventPos(event)
        knotX = self._HitKnot(rect, position)

        if event.button() == QtCore.Qt.RightButton:
            if knotX is not None:
                if not DeleteSplineKnot(self._edit, knotX):
                    Tf.Warn("Refusing to delete: a falloff curve needs at "
                            "least two knots.")
                self.update()
            self._CommitEdit()
            return

        if event.button() != QtCore.Qt.LeftButton:
            super(FalloffCurveWidget, self).mousePressEvent(event)
            return

        if knotX is not None:
            self._dragKnotX = knotX
            self.update()
            return

        if not rect.contains(position):
            self._CancelEdit()
            return

        x, y = self._ToUnit(rect, position)
        self._dragKnotX = InsertSplineKnot(self._edit, x, y)
        self._gesture.Update(self._usdAttribute, self._edit.GetSpline())
        self.update()

    def mouseMoveEvent(self, event):
        if self._usdAttribute is None or self._dragKnotX is None:
            return

        rect = self._PlotRect()
        x, y = self._ToUnit(rect, _EventPos(event))
        newX = MoveSplineKnot(self._edit, self._dragKnotX, x, y)
        if newX is not None:
            self._dragKnotX = newX
            self._gesture.Update(self._usdAttribute, self._edit.GetSpline())
        self.update()

    def mouseReleaseEvent(self, event):
        if self._dragKnotX is None:
            return
        if self._usdAttribute is not None:
            rect = self._PlotRect()
            x, y = self._ToUnit(rect, _EventPos(event))
            MoveSplineKnot(self._edit, self._dragKnotX, x, y)
        self._dragKnotX = None
        self._CommitEdit()
        self.update()

    def mouseDoubleClickEvent(self, event):
        if self._usdAttribute is None:
            return
        self._BeginEdit()
        rect = self._PlotRect()
        knotX = self._HitKnot(rect, _EventPos(event))
        if knotX is None:
            self._CancelEdit()
            return
        if not DeleteSplineKnot(self._edit, knotX):
            Tf.Warn("Refusing to delete: a falloff curve needs at least "
                    "two knots.")
        self._dragKnotX = None
        self._CommitEdit()
        self.update()

    def _BeginEdit(self):
        if self._edit is None:
            self._edit = SplineEdit(self._usdAttribute)
            self._gesture = WeightGesture(self._usdviewApi, self._usdAttribute)

    def _CommitEdit(self):
        if self._gesture is not None:
            self._gesture.pending[self._usdAttribute.GetPath()] = self._edit.GetSpline()
            self._gesture.Commit()
        self._edit = None
        self._gesture = None

    def _CancelEdit(self):
        if getattr(self, "_gesture", None) is not None:
            self._gesture.Cancel()
        self._edit = None
        self._gesture = None
        self._dragKnotX = None
        self.update()

class VolumeWeightPanel(QtWidgets.QWidget):
    """
    The volumetric weight authoring window.

    One instance per usdview session; it follows the prim selection and
    rebuilds its parameter, placement and combine sections for whichever
    volume weight is selected.
    """

    # One panel per usdview session, filed under its main window: several
    # sessions can share this module in one process, and each panel paints
    # its overlay through its own session's container.
    _sessions = sessionRegistry.SessionRegistry("volume weight panels")

    @classmethod
    def GetInstance(cls, usdviewApi, setWeightOverlay=None,
                    hasWeightOverlay=None):
        """
        Returns the single panel for this session, creating it once.
        """
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(usdviewApi, VolumeWeightPanel(
                usdviewApi, setWeightOverlay, hasWeightOverlay))
        return panel

    def __init__(self, usdviewApi, setWeightOverlay=None,
                 hasWeightOverlay=None):
        super(VolumeWeightPanel, self).__init__(
            usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)

        self._usdviewApi = usdviewApi
        self._setWeightOverlay = setWeightOverlay
        self._hasWeightOverlay = hasWeightOverlay
        self._currentPrim = None
        self._lastOverlay = None
        self._builtSelection = None
        self._weightCache = None
        self._stageNotice = None

        self.setWindowTitle("RigExec: Volume Weights")
        self.setGeometry(0, 0, 500, 800)

        self._BuildUI()
        from weightViewport import WeightViewportController
        self._viewportController = WeightViewportController(self)

        # The prim selection lives on the selection data model, and the
        # signal it emits is signalPrimSelectionChanged(added, removed)
        # -- see Usdviewq/selectionDataModel.py.
        self._usdviewApi.dataModel.selection.signalPrimSelectionChanged \
            .connect(self._onPrimSelectionChanged)
        self._usdviewApi.dataModel.signalStageReplaced.connect(
            self._onStageReplaced)

        self._RebuildForSelection()

    # -- construction -------------------------------------------------

    def _BuildUI(self):
        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        self.setLayout(outer)

        scrollArea = QtWidgets.QScrollArea()
        scrollArea.setWidgetResizable(True)
        outer.addWidget(scrollArea)

        body = QtWidgets.QWidget()
        scrollArea.setWidget(body)

        self._rootLayout = QtWidgets.QVBoxLayout()
        self._rootLayout.setContentsMargins(10, 10, 10, 10)
        self._rootLayout.setSpacing(8)
        body.setLayout(self._rootLayout)

        self._BuildEditTargetLabel()
        self._BuildCreateSection()
        self._BuildSelectionLabel()
        self._BuildOverlaySection()
        self._tabs = QtWidgets.QTabWidget()
        self._rootLayout.addWidget(self._tabs)
        rootLayout = self._rootLayout
        for title in ("Weight", "Shape / placement", "Display / options"):
            page = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(page)
            self._tabs.addTab(page, title)
            self._rootLayout = layout
            if title == "Weight":
                self._BuildParametersSection()
                self._BuildFalloffSection()
                self._BuildCombineSection()
            elif title == "Shape / placement":
                self._shapeLayout = QtWidgets.QFormLayout()
                layout.addLayout(self._shapeLayout)
                self._BuildPlacementSection()
            else:
                self._advancedLayout = QtWidgets.QFormLayout()
                layout.addLayout(self._advancedLayout)
            layout.addStretch(1)
        self._rootLayout = rootLayout
        self._rootLayout.addStretch(1)

    def _BuildEditTargetLabel(self):
        # EDIT TARGET DISCIPLINE: every Set/SetSpline/SetTargets in this
        # panel goes through the plain Usd authoring API, which always
        # writes into stage.GetEditTarget().  usdview starts with the
        # SESSION layer as its edit target, so by default nothing this
        # panel authors survives closing the app -- it is a scratch pad.
        # Switching the edit target to the root layer (usdview's Edit
        # Target menu, or the Layer Stack view) is what makes weight
        # objects persist.  The label spells out which one is live right
        # now so nobody loses an afternoon of volume placement.
        self._editTargetLabel = QtWidgets.QLabel()
        self._editTargetLabel.setWordWrap(True)
        self._editTargetLabel.setToolTip(
            "All authoring goes into the stage's current edit target. "
            "usdview defaults to the session layer, which is discarded "
            "when the app closes; switch to the root layer to persist.")
        self._rootLayout.addWidget(self._editTargetLabel)

    def _BuildCreateSection(self):
        group = QtWidgets.QGroupBox()
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        group.setLayout(layout)

        for typeName in VOLUME_WEIGHT_TYPE_NAMES:
            label = DefaultPrimNameForType(typeName).replace("Weight", "")
            button = QtWidgets.QPushButton(label)
            button.setToolTip("Create a %s" % typeName)
            button.clicked.connect(
                lambda checked=False, name=typeName: self._onCreate(name))
            layout.addWidget(button)

        self._rootLayout.addWidget(group)

    def _BuildSelectionLabel(self):
        hint = QtWidgets.QLabel("Viewport: drag boundary | Shift-drag side | Esc cancels")
        hint.setToolTip("Drag an inner shell or outer cage to resize. "
                        "Shift-drag a sphere side to trim that half-axis, "
                        "or a bounded plane to resize its extent.")
        hint.setWordWrap(True)
        self._rootLayout.addWidget(hint)
        self._selectionLabel = QtWidgets.QLabel()
        self._selectionLabel.setWordWrap(True)
        self._rootLayout.addWidget(self._selectionLabel)

    def _BuildParametersSection(self):
        self._parametersGroup = QtWidgets.QGroupBox("Parameters")
        self._parametersLayout = QtWidgets.QFormLayout()
        self._parametersLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        self._parametersLayout.setFieldGrowthPolicy(
            QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        self._parametersLayout.setContentsMargins(10, 10, 10, 10)
        self._parametersGroup.setLayout(self._parametersLayout)
        self._rootLayout.addWidget(self._parametersGroup)

    def _BuildPlacementSection(self):
        self._placementGroup = QtWidgets.QGroupBox("Placement")
        placementOuter = QtWidgets.QVBoxLayout()
        placementOuter.setContentsMargins(10, 10, 10, 10)
        self._placementGroup.setLayout(placementOuter)

        self._placementLayout = QtWidgets.QFormLayout()
        self._placementLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        self._placementLayout.setFieldGrowthPolicy(
            QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        placementOuter.addLayout(self._placementLayout)

        self._snapButton = QtWidgets.QPushButton("Snap to selection")
        self._snapButton.setToolTip(
            "Move this volume to the bounding-box centre of the other "
            "selected prim.")
        self._snapButton.clicked.connect(self._onSnapToSelection)
        placementOuter.addWidget(self._snapButton)

        self._rootLayout.addWidget(self._placementGroup)

    def _BuildFalloffSection(self):
        self._falloffGroup = QtWidgets.QGroupBox("Falloff curve")
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        self._falloffGroup.setLayout(layout)

        presetRow = QtWidgets.QHBoxLayout()
        presetRow.addWidget(QtWidgets.QLabel("Preset:"))
        self._presetButtons = []
        for profile in SPLINE_PROFILE_NAMES:
            button = QtWidgets.QPushButton(profile)
            button.setToolTip(
                "Overwrite rigExec:falloffCurve with the %s shape and "
                "switch rigExec:falloffProfile to 'curve'." % profile)
            button.clicked.connect(
                lambda checked=False, name=profile: self._onPreset(name))
            presetRow.addWidget(button)
            self._presetButtons.append(button)
        layout.addLayout(presetRow)

        self._curveWidget = FalloffCurveWidget(self._usdviewApi, None, self)
        layout.addWidget(self._curveWidget)

        self._curveHint = QtWidgets.QLabel()
        self._curveHint.setWordWrap(True)
        layout.addWidget(self._curveHint)

        self._rootLayout.addWidget(self._falloffGroup)

    def _BuildCombineSection(self):
        self._combineGroup = QtWidgets.QGroupBox("Combine inputs")
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        self._combineGroup.setLayout(layout)

        self._combineModeLayout = QtWidgets.QFormLayout()
        self._combineModeLayout.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignRight)
        layout.addLayout(self._combineModeLayout)

        self._inputWeightsList = QtWidgets.QListWidget()
        self._inputWeightsList.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        self._inputWeightsList.setMinimumHeight(90)
        self._inputWeightsList.setToolTip(
            "Order matters for subtract and overlay.")
        layout.addWidget(self._inputWeightsList)

        buttonRow = QtWidgets.QHBoxLayout()
        addButton = QtWidgets.QPushButton("Add from selection")
        addButton.clicked.connect(self._onAddInputWeights)
        buttonRow.addWidget(addButton)
        removeButton = QtWidgets.QPushButton("Remove")
        removeButton.clicked.connect(self._onRemoveInputWeights)
        buttonRow.addWidget(removeButton)
        layout.addLayout(buttonRow)

        self._rootLayout.addWidget(self._combineGroup)

    def _BuildOverlaySection(self):
        group = QtWidgets.QGroupBox("Influence overlay")
        layout = QtWidgets.QVBoxLayout()
        group.setLayout(layout)

        self._overlayCheckBox = QtWidgets.QCheckBox("Show influence")
        self._overlayCheckBox.setToolTip(
            "Paint the chosen weight object's influence onto its target "
            "geometry using its Surface display setting.")
        self._overlayCheckBox.stateChanged.connect(self._onOverlayToggled)
        layout.addWidget(self._overlayCheckBox)

        # An EXPLICIT target, not the tree selection.
        #
        # The first cut drove the overlay from the selection, which meant
        # the checkbox sat disabled until you happened to select a weight
        # object -- and a disabled checkbox that explains nothing reads
        # exactly like a broken one. It also fought the actual workflow:
        # you want to watch an influence while dragging the JOINT.
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Painting:"))
        self._overlayTargetCombo = QtWidgets.QComboBox()
        self._overlayTargetCombo.setToolTip(
            "Which weight object's field to paint. Independent of the "
            "viewport selection, so the influence stays visible while "
            "you drag the joint that drives it.")
        self._overlayTargetCombo.currentIndexChanged.connect(
            self._onOverlayTargetChanged)
        row.addWidget(self._overlayTargetCombo, 1)
        layout.addLayout(row)

        legend = QtWidgets.QLabel(
            "Tint: gray 0 → red 1. Heatmap: blue 0 → cyan .25 → "
            "green .5 → yellow .75 → red 1. Bands: five .2-wide ranges.")
        legend.setWordWrap(True)
        legend.setToolTip(
            "Colors use resolved vertex weights. Faces interpolate the colors; "
            "bands are approximate on coarse geometry. Display clamps to 0–1.")
        self._overlayCheckBox.setToolTip(legend.text())
        legend.hide()

        self._overlayStatusLabel = QtWidgets.QLabel("")
        self._overlayStatusLabel.setWordWrap(True)
        layout.addWidget(self._overlayStatusLabel)

        self._rootLayout.addWidget(group)

    def _OverlayTargetPath(self):
        """The prim path the overlay should paint, or '' for none."""
        if self._overlayTargetCombo.count() == 0:
            return ""
        return str(self._overlayTargetCombo.currentData() or "")

    def _RefreshOverlayTargets(self, preferred=None):
        """
        Repopulates the target list, keeping the current choice when it
        survives and otherwise preferring \\p preferred (normally the
        selected weight object, so selecting one still does the
        convenient thing without the overlay DEPENDING on it).
        """
        combo = self._overlayTargetCombo
        previous = self._OverlayTargetPath()
        weights = self._WeightObjectsOnStage()

        combo.blockSignals(True)
        combo.clear()
        for prim in weights:
            path = str(prim.GetPath())
            combo.addItem("%s  (%s)" % (prim.GetName(), prim.GetTypeName().replace("RigExec", "")),
                          path)
        wanted = None
        if previous and previous in [str(p.GetPath()) for p in weights]:
            wanted = previous
        elif preferred is not None:
            wanted = str(preferred.GetPath())
        if wanted:
            index = combo.findData(wanted)
            if index >= 0:
                combo.setCurrentIndex(index)
        combo.blockSignals(False)

    # -- rebuild ------------------------------------------------------

    def _CurrentWeightPrim(self):
        """
        Returns the first selected prim that is a volume weight, or
        None.

        The stage check is not defensive noise: signalStageReplaced fires
        with dataModel.stage already None when usdview tears a stage down,
        and usdviewApi.selectedPrims walks that null stage
        (selectionDataModel.py:711 -> stage.GetPrimAtPath). Without this
        the panel raises on every stage close.
        """
        if self._usdviewApi.dataModel.stage is None:
            return None
        for prim in self._usdviewApi.selectedPrims:
            if IsVolumeWeightPrim(prim):
                return prim
        return None

    def _WeightObjectsOnStage(self):
        """
        Every weight object on the stage, in namespace order.

        The overlay target list is drawn from this rather than from the
        selection because a rigger wants to look at an influence while
        selecting and dragging something ELSE -- the joint, the control,
        the mesh. Tying the overlay to the tree selection makes the one
        thing you want to watch disappear the moment you touch anything.
        """
        stage = self._usdviewApi.dataModel.stage
        if stage is None:
            return []
        if self._weightCache is None:
            self._weightCache = [prim for prim in stage.TraverseAll()
                                 if IsWeightObjectPrim(prim)]
        return self._weightCache

    def _onStageReplaced(self):
        self._weightCache = None
        self._builtSelection = None
        self._lastOverlay = None
        self._WatchStage()
        if self.isVisible():
            self._RebuildForSelection()

    def _WatchStage(self):
        if self._stageNotice is not None:
            self._stageNotice.Revoke()
            self._stageNotice = None
        stage = self._usdviewApi.dataModel.stage
        if stage is not None:
            self._stageNotice = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._StageChanged, stage)

    def _StageChanged(self, notice, stage):
        if notice.GetResyncedPaths():
            self._weightCache = None
            self._builtSelection = None

    def hideEvent(self, event):
        self._viewportController.Cancel()
        self._viewportController.Detach()
        self._curveWidget.Detach()
        for widget in self.findChildren(QtWidgets.QWidget):
            detach = getattr(widget, "Detach", None)
            if detach is not None:
                detach()
        self._builtSelection = None
        super().hideEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self._viewportController.Attach()
        self._WatchStage()
        self._RebuildForSelection()

    def _onPrimSelectionChanged(self, added, removed):
        """
        Handler for
        selectionDataModel.signalPrimSelectionChanged(added, removed).
        """
        if self.isVisible():
            self._RebuildForSelection()

    def _RebuildForSelection(self, force=False):
        stage = self._usdviewApi.dataModel.stage
        if stage is not None:
            layer = stage.GetEditTarget().GetLayer()
            isSession = layer == stage.GetSessionLayer()
            self._editTargetLabel.setText(
                "Editing session layer (temporary)" if isSession else
                "Editing " + layer.GetDisplayName())
            self._editTargetLabel.setToolTip(layer.identifier)
        else:
            self._editTargetLabel.setText("Edit target: no stage")

        prim = self._CurrentWeightPrim()
        key = (stage, prim.GetPath() if prim else None)
        if self._builtSelection == key and not force:
            return
        self._builtSelection = key
        self._currentPrim = prim
        self._scrubScale = (ComputeScrubScale(prim, self._usdviewApi.frame)
                            if prim else 1.0)

        _ClearLayout(self._parametersLayout)
        _ClearLayout(self._placementLayout)
        _ClearLayout(self._shapeLayout)
        _ClearLayout(self._advancedLayout)
        _ClearLayout(self._combineModeLayout)
        self._inputWeightsList.clear()

        if prim is None:
            self._selectionLabel.setText(
                "Select a volume weight prim to edit it, or create one "
                "above.")
            self._parametersGroup.setEnabled(False)
            self._placementGroup.setEnabled(False)
            self._falloffGroup.setEnabled(False)
            self._combineGroup.setVisible(False)
            self._curveWidget.SetAttribute(None)
            self._SyncOverlayCheckBox(None)
            return

        self._selectionLabel.setText(
            "<b>%s</b> (%s)" % (prim.GetName(), prim.GetTypeName().replace("RigExec", "")))
        self._selectionLabel.setToolTip(str(prim.GetPath()))
        self._parametersGroup.setEnabled(True)

        self._PopulateParameters(prim)
        self._PopulatePlacement(prim)
        self._PopulateFalloff(prim)
        self._PopulateCombine(prim)
        self._SyncOverlayCheckBox(prim)

    def _AddAttributeRow(self, layout, prim, attrName, label=None,
                         sensitivity=0.02, normalized=False,
                         editMaximum=1.0):
        """
        Adds one attribute row, choosing the widget from the attribute's
        type: a combo for tokens with allowedTokens, a slider+edit for
        the 0..1 parameters, a munging edit for everything else.

        Returns the widget, or None when the prim does not have the
        attribute -- which is how scaleX/Y/Z and planeAxis disappear on
        the types that lack them without any per-type branching here.
        """
        attr = prim.GetAttribute(attrName)
        if not attr:
            return None
        if layout is self._parametersLayout:
            if attrName.startswith(("inputs:scale", "inputs:extent", "rigExec:plane")):
                layout = self._shapeLayout
            elif attrName.startswith("guide:") or attrName == "rigExec:rangePolicy":
                layout = self._advancedLayout

        if (attr.GetTypeName() == Sdf.ValueTypeNames.Token and
                attr.HasMetadata("allowedTokens")):
            widget = AttributeValueSelectWidget(attr, self._usdviewApi)
        elif normalized:
            widget = AttributeValueSliderWidget(
                attr, self._usdviewApi, editMaximum=editMaximum)
        else:
            widget = AttributeValueWidget(
                attr, self._usdviewApi, sensitivity=sensitivity)

        layout.addRow(label or attrName.split(":")[-1], widget)
        return widget

    def _PopulateParameters(self, prim):
        # A falloff radius is measured in stage units and may be tens or
        # hundreds; the scale factors and the normalized parameters are
        # not.  Sensitivity is per-row for exactly that reason.
        size = self._scrubScale
        radiusSensitivity = max(float(size), 1.0) / 200.0

        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:falloffMin",
            "Inner radius", sensitivity=radiusSensitivity)
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:falloffMax",
            "Outer radius", sensitivity=radiusSensitivity)
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:invert", "Invert",
            normalized=True)
        # strength is a final multiplier, not a normalized parameter: the
        # slider still travels 0..1 because that is the useful range, but
        # the edit is left open above it so an artist can over-drive.
        self._AddAttributeRow(
            self._parametersLayout, prim, "inputs:strength", "Strength",
            normalized=True, editMaximum=None)
        for axis in ("X", "Y", "Z"):
            self._AddAttributeRow(
                self._parametersLayout, prim, "inputs:scale%s" % axis,
                "scale%s" % axis, sensitivity=0.005)
        for axis in ("X", "Y", "Z"):
            for side, sign in (("Pos", "+"), ("Neg", "-")):
                self._AddAttributeRow(
                    self._parametersLayout, prim, "inputs:scale%s%s" % (axis, side),
                    "scale %s%s" % (sign, axis), sensitivity=0.005)
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:planeAxis", "planeAxis")
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:planeBounds",
            "planeBounds")
        # The extents are stage-unit distances like the falloff band, not
        # the 0..1 factors scaleX/Y/Z are, so they scrub at the band's
        # sensitivity rather than the scale factors'.  All three rows drop
        # themselves on the types that lack the attribute.
        for name in ("U", "V"):
            self._AddAttributeRow(
                self._parametersLayout, prim, "inputs:extent%s" % name,
                "extent%s" % name, sensitivity=radiusSensitivity)

        profileWidget = self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:falloffProfile",
            "Profile")
        if isinstance(profileWidget, AttributeValueSelectWidget):
            # The curve editor is only meaningful when the profile
            # actually reads the spline, so track the token live.
            profileWidget.valueChanged.connect(self._onProfileChanged)

        # Which points the field measures: the read phase declared on
        # rigExec:weightTarget, base (static) or preceding (in flight).
        self._advancedLayout.addRow(
            "Sample points", ReadPhaseSelectWidget(
                prim.GetRelationship("rigExec:weightTarget"),
                ("base", "preceding")))
        self._AddAttributeRow(
            self._parametersLayout, prim, "rigExec:rangePolicy",
            "rangePolicy")
        self._AddAttributeRow(
            self._parametersLayout, prim, "guide:drawMode", "guide:drawMode")
        self._AddAttributeRow(
            self._parametersLayout, prim, "guide:weightDisplay", "Surface display")
        self._AddAttributeRow(
            self._parametersLayout, prim, "guide:displayOpacity", "Guide opacity",
            normalized=True)
        self._AddAttributeRow(
            self._parametersLayout, prim, "guide:wireWidth", "Guide width",
            sensitivity=0.001)

        targets = []
        relationship = prim.GetRelationship("rigExec:weightTarget")
        if relationship:
            targets = [str(p) for p in relationship.GetTargets()]
        targetLabel = QtWidgets.QLabel(
            ", ".join(targets) if targets else "<not bound>")
        targetLabel.setWordWrap(True)
        self._parametersLayout.addRow("Surface", targetLabel)

    def _PopulatePlacement(self, prim):
        placed = prim.GetTypeName() in PLACED_WEIGHT_TYPE_NAMES
        self._placementGroup.setEnabled(placed)
        if not placed:
            return

        size = self._scrubScale
        translateSensitivity = max(float(size), 1.0) / 200.0

        for axis in ("tx", "ty", "tz"):
            self._AddAttributeRow(
                self._placementLayout, prim, "avars:%s" % axis, axis,
                sensitivity=translateSensitivity)
        # Rotations are in degrees, so a quarter degree per pixel gives
        # a full turn in a comfortable drag.
        for axis in ("rx", "ry", "rz"):
            self._AddAttributeRow(
                self._placementLayout, prim, "avars:%s" % axis, axis,
                sensitivity=0.25)

    def _PopulateFalloff(self, prim):
        attr = prim.GetAttribute("rigExec:falloffCurve")
        self._falloffGroup.setEnabled(bool(attr))
        self._curveWidget.SetAttribute(attr if attr else None)

        if not attr:
            self._curveHint.setText(
                "%s has no falloff curve." % prim.GetTypeName())
            self._curveWidget.setEnabled(False)
            return

        profileAttr = prim.GetAttribute("rigExec:falloffProfile")
        profile = profileAttr.Get() if profileAttr else None
        self._SetCurveEnabledForProfile(profile)

    def _SetCurveEnabledForProfile(self, profile):
        isCurve = (profile == "curve")
        self._curveWidget._profile = profile
        self._curveWidget.setEnabled(isCurve)
        self._curveWidget.update()
        if isCurve:
            self._curveHint.setText(
                "Drag points to shape the transition. Click to add a point; "
                "right-click to remove one. Surface influence previews while dragging.")
        else:
            self._curveHint.setText(
                "Using %s falloff. Choose a preset above to start editing "
                "a custom curve."
                % (profile if profile else "unset",))

    def _PopulateCombine(self, prim):
        isCombine = prim.GetTypeName() == "RigExecCombineWeight"
        self._combineGroup.setVisible(isCombine)
        if not isCombine:
            return

        self._AddAttributeRow(
            self._combineModeLayout, prim, "rigExec:combineMode",
            "combineMode")

        for path in GetInputWeightPaths(prim):
            self._inputWeightsList.addItem(str(path))

    # -- actions ------------------------------------------------------

    def _onCreate(self, typeName):
        stage = self._usdviewApi.dataModel.stage
        if stage is None:
            Tf.Warn("No stage: cannot create %s" % typeName)
            return

        # What is about to be authored, named before it exists: the new
        # weight, and the Weights scope when creating it is what brings
        # that scope into being. An absent snapshot is what makes undo
        # remove them again.
        selected = list(self._usdviewApi.selectedPrims)
        parentPath, parentExisted = ChooseWeightParentPath(stage, selected)
        parentPrim = stage.GetPrimAtPath(parentPath)
        name = (MakeUniquePrimName(parentPrim,
                                   DefaultPrimNameForType(typeName))
                if parentPrim else DefaultPrimNameForType(typeName))
        paths = [parentPath.AppendChild(name)]
        if not parentExisted:
            paths.append(parentPath)

        try:
            with Recording(stage, paths, "Create %s" % name):
                prim = CreateVolumeWeightPrim(
                    stage, typeName, selected, self._usdviewApi.frame)
        except Exception as err:
            Tf.Warn("Failed to create %s: %s" % (typeName, err))
            return

        # Selecting the new prim is what makes the panel show it, and it
        # also puts it under the artist's cursor in the prim browser.
        self._usdviewApi.ClearPrimSelection()
        self._usdviewApi.AddPrimToSelection(prim)
        self._usdviewApi.UpdateGUI()
        self._RebuildForSelection(force=True)

    def _onSnapToSelection(self):
        prim = self._currentPrim
        if prim is None:
            return

        source = None
        for candidate in self._usdviewApi.selectedPrims:
            if candidate and candidate.GetPath() != prim.GetPath():
                source = candidate
                break
        if source is None:
            Tf.Warn("Snap needs a second prim selected alongside the "
                    "volume weight.")
            return

        with Recording(prim.GetStage(), [prim.GetPath()],
                       "Snap %s" % prim.GetName()):
            SnapWeightToPrim(prim, source, self._usdviewApi.frame)
        self._RebuildForSelection(force=True)

    def _onPreset(self, profile):
        prim = self._currentPrim
        if prim is None:
            return

        attr = prim.GetAttribute("rigExec:falloffCurve")
        if not attr:
            return

        # The spline and the profile token are one decision, so they are
        # one undo entry: leaving the curve behind with the profile
        # reverted, or the other way round, is the trap the comment
        # below is about.
        with Recording(prim.GetStage(), [prim.GetPath()],
                       "%s falloff" % profile):
            try:
                EnsureDefaultSpline(attr, profile, force=True)
            except Exception as err:
                Tf.Warn("Failed to author %s spline on <%s>: %s"
                        % (profile, attr.GetPath(), err))
                return

            # Baking a shape the field would ignore is a trap, so the
            # preset buttons also flip the profile token to the one that
            # reads it.
            profileAttr = prim.GetAttribute("rigExec:falloffProfile")
            if profileAttr:
                profileAttr.Set("curve")

        self._RebuildForSelection(force=True)

    def _onProfileChanged(self, value):
        self._SetCurveEnabledForProfile(value)

    def _onAddInputWeights(self):
        prim = self._currentPrim
        if prim is None or prim.GetTypeName() != "RigExecCombineWeight":
            return

        paths = [p.GetPath() for p in self._usdviewApi.selectedPrims
                 if IsWeightObjectPrim(p) and p.GetPath() != prim.GetPath()]
        if not paths:
            Tf.Warn("Select one or more weight objects to add.")
            return

        with Recording(prim.GetStage(), [prim.GetPath()],
                       "Add input weights"):
            AddInputWeightPaths(prim, paths)
        self._RebuildForSelection(force=True)

    def _onRemoveInputWeights(self):
        prim = self._currentPrim
        if prim is None or prim.GetTypeName() != "RigExecCombineWeight":
            return

        paths = [item.text() for item in
                 self._inputWeightsList.selectedItems()]
        if not paths:
            return

        with Recording(prim.GetStage(), [prim.GetPath()],
                       "Remove input weights"):
            RemoveInputWeightPaths(prim, paths)
        self._RebuildForSelection(force=True)

    # -- viewport overlay ---------------------------------------------

    def _SyncOverlayCheckBox(self, prim):
        """
        Enables the overlay checkbox only when a weight is selected AND
        the imaging library actually exports the entry point.
        """
        available = True
        if self._hasWeightOverlay is not None:
            try:
                available = bool(self._hasWeightOverlay())
            except Exception:
                available = False
        elif self._setWeightOverlay is None:
            available = False

        self._RefreshOverlayTargets(preferred=prim)
        hasTarget = self._overlayTargetCombo.count() > 0

        self._overlayCheckBox.blockSignals(True)
        self._overlayCheckBox.setEnabled(available and hasTarget)
        if not available:
            self._overlayCheckBox.setChecked(False)
        self._overlayCheckBox.blockSignals(False)
        self._overlayTargetCombo.setEnabled(available and hasTarget)

        # Whatever state it is in, SAY SO. A disabled checkbox with no
        # explanation is indistinguishable from a broken one, which is
        # precisely how this landed the first time.
        if not available:
            self._overlayStatusLabel.setText(
                "rigExecImaging does not export "
                "RigExecImaging_SetWeightOverlayForStage -- rebuild the "
                "imaging library to enable the overlay.")
        elif not hasTarget:
            self._overlayStatusLabel.setText(
                "No weight objects on this stage yet. Create one above.")
        else:
            self._overlayStatusLabel.setText(
                "Influence appears on geometry bound to this weight.")

        # A checked box whose target changed must repoint rather than
        # keep painting the old one.
        if available and self._overlayCheckBox.isChecked():
            self._ApplyOverlay()

    def _ApplyOverlay(self):
        """Pushes the current on/off + target state to the imaging library."""
        if self._setWeightOverlay is None:
            return
        path = (self._OverlayTargetPath()
                if self._overlayCheckBox.isChecked() else "")
        key = (self._usdviewApi.dataModel.stage, path)
        if self._lastOverlay == key:
            return
        try:
            self._setWeightOverlay(path)
            self._lastOverlay = key
        except Exception as err:
            Tf.Warn("RigExecImaging_SetWeightOverlayForStage failed: %s"
                    % err)
            return
        # The scene index change does not itself schedule a repaint.
        self._usdviewApi.UpdateViewport()

    def _onOverlayToggled(self, state):
        self._ApplyOverlay()

    def _onOverlayTargetChanged(self, index):
        if self._overlayCheckBox.isChecked():
            self._ApplyOverlay()


def OpenVolumeWeightPanel(usdviewApi, setWeightOverlay=None,
                          hasWeightOverlay=None, undoStack=None):
    """
    Command-plugin entry point: shows the single panel instance.
    """
    if undoStack is not None:
        SetUndoStack(undoStack)
    panel = VolumeWeightPanel.GetInstance(
        usdviewApi, setWeightOverlay, hasWeightOverlay)
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
