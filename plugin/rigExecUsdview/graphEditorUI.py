#
# RigExec graph editor: the usdview window. Host wiring -- session
# registration, hotkeys, the graph edit hook, undo actions and the
# editor-extension takeover -- over graphEditorModel, which owns
# selection, drag and edit lifecycle, and graphCanvas, which owns the
# plot widget. Each host event maps to ONE model operation; the model
# answers through the callables this window gives it.
#
import math
import os
import sys

from pxr import Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    # PySide6 keeps QAction / QActionGroup in QtGui; usdview's qt shim
    # re-exports whichever module has them as QtActionWidgets.
    from pxr.Usdviewq.qt import QtActionWidgets
except ImportError:                                       # PySide2
    QtActionWidgets = QtWidgets

try:
    import editorExtension
    import gizmoMath
    import gizmoPreview
    import graphCanvas
    import graphEditorModel
    import graphModel
    import graphScreen
    import rigExecUndo
    import sessionRegistry
    # The canvas widget and the pixel helpers readers address here.
    from graphCanvas import GraphCanvas, _Color, _Position
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import editorExtension
    import gizmoMath
    import gizmoPreview
    import graphCanvas
    import graphEditorModel
    import graphModel
    import graphScreen
    import rigExecUndo
    import sessionRegistry
    from graphCanvas import GraphCanvas, _Color, _Position


SIDE_IN = graphModel.SIDE_IN
SIDE_OUT = graphModel.SIDE_OUT
SIDE_BOTH = graphModel.SIDE_BOTH

# Infinity menu, in menu order, as (label, graphModel name).
INFINITY_CHOICES = (
    ("Constant", "constant"),
    ("Linear", "linear"),
    ("Cycle", "cycle"),
    ("Cycle w/ Offset", "cycle_offset"),
    ("Oscillate", "oscillate"),
)

# Ts curve types (ts/types.h:101-105), as (label, graphModel name).
CURVE_TYPE_CHOICES = (
    ("Bezier", "bezier"),
    ("Hermite", "hermite"),
)

WEIGHTED_TIP = (
    "A handle drag moves the handle's end: its length and its angle. "
    "Off, the length is locked and the drag only rotates.")

TANGENT_BUTTONS = (
    ("Auto", graphModel.TANGENT_AUTO,
     "Auto tangents (Ts AutoEase) on the selected keys."),
    ("Spline", graphModel.TANGENT_SPLINE,
     "Catmull-Rom slope through the neighbouring keys."),
    ("Linear", graphModel.TANGENT_LINEAR,
     "Straight segment after the key (before it, for In)."),
    ("Flat", graphModel.TANGENT_FLAT,
     "Zero slope, so the curve levels off at the key."),
    ("Step", graphModel.TANGENT_STEP,
     "Held segment after the key (before it, for In)."),
)

# Focus widgets that own every key they are given, so a bare A or F
# typed into a spin box never reaches the canvas.
_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                 QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)


# Small widgets

class BlankableSpinBox(QtWidgets.QDoubleSpinBox):
    """
    A double spin box that can show NOTHING.

    Time / Value key stats are blank when the selected keys
    disagree, and a plain QDoubleSpinBox always shows a number -- which
    would read as "every selected key is at frame 1001" when in fact
    they are spread over the shot. Blankness is a display state only:
    the box keeps its last value so the arrows still work, and
    IsBlank() is what the panel checks before applying a typed number.
    """

    def __init__(self, parent=None):
        super(BlankableSpinBox, self).__init__(parent)
        self._blank = False
        self.setKeyboardTracking(False)
        self.lineEdit().textEdited.connect(self._onTextEdited)

    def _onTextEdited(self, text):
        del text
        self._blank = False

    def textFromValue(self, value):
        # Not just an empty line edit: QAbstractSpinBox re-renders its
        # text from the value on a show, a style change and an
        # interpretText, so a box blanked once would silently come back
        # reading 0.000 with nothing selected.
        if self._blank:
            return ""
        return super(BlankableSpinBox, self).textFromValue(value)

    def SetBlank(self):
        self._blank = True
        self.lineEdit().setText("")

    def SetNumber(self, value):
        self._blank = False
        value = float(value)
        self.setValue(value)
        # setValue does not refresh the line edit when the value is
        # unchanged, and the box may currently be showing blank.
        self.lineEdit().setText(self.textFromValue(value))

    def IsBlank(self):
        return self._blank or not self.lineEdit().text().strip()


class CurveListWidget(QtWidgets.QTreeWidget):
    """
    outliner column: one row per curve, with the colour it draws
    in, the prim it belongs to, the attribute, and a visibility check.

    Selecting rows ISOLATES those curves (behaviour): the canvas
    then draws only the selection, and `Show All` puts everything back.
    """

    def __init__(self, panel):
        super(CurveListWidget, self).__init__(panel)
        self._panel = panel
        self.setColumnCount(2)
        self.setHeaderLabels(["Curve", "Attribute"])
        self.setRootIsDecorated(False)
        self.setUniformRowHeights(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.setAlternatingRowColors(True)
        # usdview's dark palette draws the native check indicator almost
        # invisibly against the row, and the visibility tick is the one
        # control in this column: give it an outline of its own, the way
        # ViewportToolbar styles its checked tool buttons.
        self.setStyleSheet(
            "QTreeWidget::indicator { width: 12px; height: 12px;"
            " border: 1px solid #6e6e6e; border-radius: 2px; }"
            " QTreeWidget::indicator:unchecked { background: #202020; }"
            " QTreeWidget::indicator:checked { background: #5da0f0;"
            " border: 1px solid #9ccbff; }")
        self.setToolTip(
            "The animation curves in the editor. Tick to show or hide a "
            "curve; select rows to isolate them.")



# Edit hook

class _NullContext(object):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False



class GraphEditHook(object):
    """
    How an embedder brackets, previews and filters the editor's edits
    (a host that embeds the panel installs one with
    GraphEditorPanel.SetEditHook). Without a hook every path is
    the floating editor's own, unchanged. Hooks may subclass this or
    provide the same methods; every call is guarded, so a failing hook
    refuses the edit (or draws the authored curve) rather than raising
    into a Qt handler.

    EDITS. Every drag (key or tangent) and every command (_Apply: the
    tangent buttons, Insert / Delete, the Time / Value fields, infinity,
    curve type, dual values) runs as
        token = BeginEdit(panel, label, indices, drag)
            falsy refuses: nothing is authored, the panel does nothing
            (the hook reports why);
        every authoring step (the recorder's Begin, the writes, its
            Commit, which pushes onto panel.undoStack) runs inside
            EditContext(panel);
        FinishEdit(panel, token, changed)
            None keeps the edit; a refusal text means the hook rolled it
            back (the panel re-reads its curves and shows the text);
    and AbortEdit(panel, token) when the edit is abandoned (Escape, a
    raising operation, a new stage, SetEditHook). `label` is the edit's
    undo label ("Move Keys", "Edit Tangent", "Insert Key", "Tangent
    Flat", ...); `indices` the curves it touches.

    PREVIEW. A drag authors NOTHING until it is committed: each move
    recomputes the edited curves from their origin and keeps them in the
    panel, which draws them and sends their current-frame values to
    Hydra through gizmoPreview. The canvas release queues the commit
    (ReleaseGesture) for the next event-loop turn; EndGesture writes the
    curves once, in one change block, and AbortGesture discards them. So
    FinishEdit follows a mouse release one turn later.

    KNOT FILTER. KeySpline(panel, index, authored) is the spline whose
    knots are the curve's KEYS: drawn as squares, hit, selected, framed,
    dragged, deleted and keyed around. A hook may return a copy without
    the knots it derives from the keys; it then rebuilds them in
    FinishEdit, because what the panel writes is the edited KEY spline.
    CurveSpline(panel, index, authored, edited) is the spline the canvas
    strokes as the curve: `edited` is the live gesture's key spline while
    a preview runs, else None. A value the panel reads OFF the curve (a
    new key's value) still comes from `authored`, what the stage shows.

    SelectionChanged(panel, additive) follows a key selection made in the
    panel (a click, a toggle, a marquee, SelectKeys).
    """

    METHODS = ("BeginEdit", "EditContext", "FinishEdit", "AbortEdit",
               "KeySpline", "CurveSpline", "SelectionChanged")

    def BeginEdit(self, panel, label, indices, drag):
        del panel, label, indices, drag
        return True

    def EditContext(self, panel):
        del panel
        return _NullContext()

    def FinishEdit(self, panel, token, changed):
        del panel, token, changed
        return None

    def AbortEdit(self, panel, token):
        del panel, token

    def KeySpline(self, panel, index, authored):
        del panel, index
        return authored

    def CurveSpline(self, panel, index, authored, edited):
        del panel, index
        return edited if edited is not None else authored

    def SelectionChanged(self, panel, additive):
        del panel, additive



# Hotkeys

class GraphHotkeyFilter(QtCore.QObject):
    """
    The editor's hotkeys, taken at the APPLICATION level.

    See this module's banner: usdview's AppEventFilter swallows Escape
    outright and turns a bare F into a 3D viewport frame, both from an
    application-wide filter, so a widget-level handler in this window
    would never see either key. Qt runs the most recently installed
    application filter first, and this one is installed when the editor
    opens -- after usdview's -- so it sees them first.

    Nothing is claimed unless it is acted on, and nothing at all is
    claimed for another window, for a hidden editor, or while focus is
    in a text field.
    """

    def __init__(self, panel):
        super(GraphHotkeyFilter, self).__init__(panel)
        self._panel = panel

    def eventFilter(self, obj, event):
        try:
            kind = event.type()
            panel = self._panel
            if kind == QtCore.QEvent.KeyRelease:
                # Never consumed: whoever saw the press sees the release.
                if not event.isAutoRepeat():
                    panel.HandleKeyRelease(event.key())
                return False
            if kind not in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.ShortcutOverride):
                return False
            if not panel.isVisible() or not panel.OwnsEvent(obj):
                return False
            # Before any key acts, Ctrl+Z included.
            panel.FlushPendingCommit()
            if _TypingFocus():
                return False
            key = event.key()
            if key not in panel.HOTKEYS:
                return False
            if event.modifiers() & (QtCore.Qt.ControlModifier
                                    | QtCore.Qt.AltModifier
                                    | QtCore.Qt.MetaModifier):
                return False
            if kind == QtCore.QEvent.ShortcutOverride:
                # Accepting the override is what stops usdview's own
                # QAction shortcuts from eating the KeyPress that
                # follows; the key is acted on there, once.
                event.accept()
                return True
            if not panel.HandleKey(key, event.modifiers()):
                return False
            event.accept()
            return True
        except Exception as error:
            # An exception escaping an application-wide filter would
            # break every key in usdview, not just ours.
            Tf.Warn("rigExecUsdview: graph editor hotkey filter failed: %s"
                    % error)
            return False


def _TypingFocus():
    focus = QtWidgets.QApplication.focusWidget()
    if focus is None:
        return False
    if isinstance(focus, _TEXT_WIDGETS):
        return True
    return (isinstance(focus, QtWidgets.QComboBox) and focus.isEditable())



# Panel

class GraphEditorPanel(QtWidgets.QWidget):
    """
    The Graph Editor window: key stats and tools across the top, the
    curve list on the left, the canvas in the middle, a status line
    underneath.

    One instance per usdview session, parented to the main window
    exactly like VolumeWeightPanel, so it floats over usdview and shares
    its lifetime. Filed under the session's main window: several sessions
    can share this module in one process, and each gets its own editor.
    """

    _sessions = sessionRegistry.SessionRegistry("graph editors")

    HOTKEYS = (QtCore.Qt.Key_A, QtCore.Qt.Key_F, QtCore.Qt.Key_I,
               QtCore.Qt.Key_K,
               QtCore.Qt.Key_Home, QtCore.Qt.Key_Delete,
               QtCore.Qt.Key_Backspace, QtCore.Qt.Key_Escape)

    @classmethod
    def GetInstance(cls, usdviewApi, undoStack):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(
                usdviewApi, GraphEditorPanel(usdviewApi, undoStack))
        return panel

    @classmethod
    def Instance(cls, usdviewApi=None):
        """`usdviewApi`'s session's editor; without one, the current
        session's (see sessionRegistry.SessionRegistry.Current)."""
        if usdviewApi is None:
            return cls._sessions.Current()
        return cls._sessions.Get(usdviewApi)

    def __init__(self, usdviewApi, undoStack):
        super(GraphEditorPanel, self).__init__(
            usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)
        self.usdviewApi = usdviewApi
        self.undoStack = undoStack

        self._hook = None             # GraphEditHook, or None
        self._hookWarned = set()
        self._hidden = set()          # curve labels unticked
        self._isolated = set()        # curve labels selected in the list
        self._dragPress = None        # the key drag's press, in pixels
        # A tangent drag's (anchor, grab offset, weighted, width, side).
        self._tangentDrag = None
        self._updating = False
        self._notice = ""
        self._framed = False
        self._noticeKey = None
        self._frame = _FrameValue(usdviewApi.frame)
        # Curves, selection, drags and edits; every host read and write
        # below is a callable, so the model never touches a widget or
        # this window's usdviewApi object directly.
        self._model = graphEditorModel.GraphEditorModel(
            stage=self.Stage,
            discover=self._DiscoverCurves,
            undo_stack=undoStack,
            document_edit=lambda: editorExtension.DocumentEdit(
                self.usdviewApi),
            begin_edit=lambda label, indices, drag: self._BeginEdit(
                label, indices, drag),
            edit_context=self._EditContext,
            finish_edit=lambda token, changed: self._FinishEdit(
                token, changed),
            abort_edit=self._AbortEdit,
            key_spline=self._KeySplineThroughHook,
            curve_spline=self._CurveSplineThroughHook,
            selection_changed=self._SelectionChanged,
            after_hook=self._AfterHook,
            is_visible=self._CurveVisible,
            frame=self.CurrentFrame,
            notify=self._Notify,
            canvas_update=lambda: self.canvas.update(),
            sync_widgets=self._Sync,
            update_viewport=lambda: self.usdviewApi.UpdateViewport(),
            preview_push=self._PreviewPush,
            preview_end=self._PreviewEnd,
            preview_republish=self._PreviewRepublish,
            commit_soon=lambda callback: QtCore.QTimer.singleShot(
                0, callback))

        self.setWindowTitle("RigExec: Graph Editor")
        # A floor, not a size: below it a QHBoxLayout stops clipping the
        # tool rows and starts shrinking the checkboxes under their own
        # labels, which reads as a rendering fault rather than a squeeze.
        # 1080 fits Time + Value + Pre + the five key buttons.
        self.setMinimumWidth(1080)
        self.resize(1180, 640)

        # Decided before the UI: _InstallUndoActions reads it.
        self._yieldsInput = editorExtension.OwnsInput(usdviewApi)
        self._BuildUI()
        self._InstallUndoActions()
        if self._yieldsInput:
            # The host routes this window's keys to HandleKey.
            self._hotkeys = None
        else:
            self._hotkeys = GraphHotkeyFilter(self)
            application = QtWidgets.QApplication.instance()
            if application is not None:
                application.installEventFilter(self._hotkeys)

        model = usdviewApi.dataModel
        model.selection.signalPrimSelectionChanged.connect(
            self._onPrimSelectionChanged)
        model.selection.signalPropSelectionChanged.connect(
            self._onPropSelectionChanged)
        model.signalStageReplaced.connect(self._onStageReplaced)
        model.currentFrameChanged.connect(self._onFrameChanged)
        if undoStack is not None:
            undoStack.AddListener(self._onUndoStackChanged)
            # A host's stack may not have it; Undo() and Redo() flush too.
            addFlusher = getattr(undoStack, "AddFlusher", None)
            if addFlusher is not None:
                addFlusher(self.FlushPendingCommit)
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.aboutToQuit.connect(self.FlushPendingCommit)
        self._ObserveStage(model.stage)

        self.RefreshCurves()
        if self._yieldsInput:
            editorExtension.EditorAdded(
                usdviewApi, editorExtension.GRAPH_EDITOR, self)

    # -- construction ---------------------------------------------------

    def _BuildUI(self):
        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(6, 6, 6, 4)
        outer.setSpacing(4)
        self.setLayout(outer)

        outer.addLayout(self._BuildKeyStatRow())
        outer.addLayout(self._BuildTangentRow())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.curveList = CurveListWidget(self)
        self.curveList.itemChanged.connect(self._onCurveItemChanged)
        self.curveList.itemSelectionChanged.connect(self._onCurveIsolation)
        listSide = QtWidgets.QWidget()
        listLayout = QtWidgets.QVBoxLayout()
        listLayout.setContentsMargins(0, 0, 0, 0)
        listLayout.setSpacing(3)
        listLayout.addWidget(self.curveList)
        showAll = QtWidgets.QPushButton("Show All")
        showAll.setToolTip("Show every curve again and drop the isolation.")
        showAll.clicked.connect(self._onShowAll)
        listLayout.addWidget(showAll)
        listSide.setLayout(listLayout)
        splitter.addWidget(listSide)

        self.canvas = GraphCanvas(self)
        splitter.addWidget(self.canvas)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([230, 780])
        outer.addWidget(splitter, 1)

        self.statusLabel = QtWidgets.QLabel("")
        self.statusLabel.setToolTip(
            "What the editor is showing and what is selected.")
        outer.addWidget(self.statusLabel)

    def _BuildKeyStatRow(self):
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QtWidgets.QLabel("Time"))
        self.timeBox = BlankableSpinBox()
        self.timeBox.setDecimals(3)
        self.timeBox.setRange(-1.0e6, 1.0e6)
        self.timeBox.setFixedWidth(86)
        self.timeBox.setToolTip(
            "The selected keys' frame; blank when they differ. Typing "
            "moves every selected key by the same amount.")
        self.timeBox.editingFinished.connect(self._onTimeEdited)
        row.addWidget(self.timeBox)

        row.addWidget(QtWidgets.QLabel("Value"))
        self.valueBox = BlankableSpinBox()
        self.valueBox.setDecimals(4)
        self.valueBox.setRange(-1.0e9, 1.0e9)
        self.valueBox.setFixedWidth(96)
        self.valueBox.setToolTip(
            "The selected keys' departure value; blank when they differ. "
            "Typing sets every selected key's value, leaving any arrival "
            "(pre) value alone.")
        self.valueBox.editingFinished.connect(self._onValueEdited)
        row.addWidget(self.valueBox)

        row.addWidget(QtWidgets.QLabel("Pre"))
        self.preValueBox = BlankableSpinBox()
        self.preValueBox.setDecimals(4)
        self.preValueBox.setRange(-1.0e9, 1.0e9)
        self.preValueBox.setFixedWidth(96)
        self.preValueBox.setToolTip(
            "The selected keys' arrival (pre) value; blank when they "
            "differ or when no selected key is dual. Typing makes the "
            "selection dual and sets every arrival value.")
        self.preValueBox.editingFinished.connect(self._onPreValueEdited)
        row.addWidget(self.preValueBox)

        for text, tip, slot in (
                ("Frame All (A)", "Fit every visible curve.",
                 self._onFrameAll),
                ("Frame Selected (F)",
                 "Fit the selected keys, or everything when none are.",
                 self._onFrameSelected),
                ("Insert Key (I)",
                 "Key every visible curve at the current frame.",
                 self._onInsertKey),
                ("Delete (Del)", "Remove the selected keys.",
                 self._onDeleteKeys),
                ("Single", "Drop the arrival squares: dual keys back to "
                 "single-valued.", self._onMakeSingle)):
            button = QtWidgets.QPushButton(text)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            row.addWidget(button)

        # Snap Frames and Weighted ride with the key stats rather than
        # with the tangent buttons: the second row is already at the
        # window's minimum width, and a QHBoxLayout that overflows
        # shrinks its checkboxes UNDER their own text rather than
        # clipping the row.
        row.addSpacing(8)
        self.snapBox = QtWidgets.QCheckBox("Snap Frames")
        self.snapBox.setChecked(True)
        self.snapBox.setToolTip("Keep dragged keys on whole frames.")
        self.snapBox.toggled.connect(self._onSnapFrames)
        row.addWidget(self.snapBox)

        self.weightedBox = QtWidgets.QCheckBox("Weighted")
        self.weightedBox.setChecked(self._model.Weighted())
        self.weightedBox.setToolTip(WEIGHTED_TIP)
        self.weightedBox.toggled.connect(self._onWeighted)
        row.addWidget(self.weightedBox)
        for box in (self.snapBox, self.weightedBox):
            # A QCheckBox's minimumSizeHint is narrower than its label,
            # so a full row shrinks it until the next widget overwrites
            # the text. Pinning the minimum to the text's own width plus
            # the indicator makes the WINDOW refuse to go that narrow
            # instead; the style's size hint is not reliable before the
            # widget has been polished, so it is only a floor here.
            width = box.fontMetrics().horizontalAdvance(box.text()) + 30
            box.setMinimumWidth(max(width, box.sizeHint().width()))
        row.addStretch(1)
        return row

    def _BuildTangentRow(self):
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QtWidgets.QLabel("Tangent"))
        for label, mode, tip in TANGENT_BUTTONS:
            button = QtWidgets.QPushButton(label)
            button.setToolTip(tip)
            button.clicked.connect(
                lambda checked=False, m=mode: self.SetTangentType(m))
            row.addWidget(button)

        row.addSpacing(8)
        self._sideButtons = {}
        group = QtWidgets.QButtonGroup(self)
        for label, side in (("In", SIDE_IN), ("Out", SIDE_OUT),
                            ("Both", SIDE_BOTH)):
            button = QtWidgets.QRadioButton(label)
            button.setToolTip(
                "Which side of the key the tangent buttons apply to.")
            button.setChecked(side == self._model.TangentSide())
            button.toggled.connect(
                lambda checked, s=side: self._onSide(checked, s))
            group.addButton(button)
            row.addWidget(button)
            self._sideButtons[side] = button

        row.addSpacing(8)
        for label, tip, slot in (
                ("Break", "Let the two tangents of a key move "
                          "independently.", self._onBreak),
                ("Unify", "Re-link the two tangents; the out slope wins.",
                 self._onUnify)):
            button = QtWidgets.QPushButton(label)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            row.addWidget(button)

        row.addSpacing(8)
        row.addWidget(QtWidgets.QLabel("Infinity"))
        self.preInfinity = self._InfinityCombo(
            "What the curve does BEFORE its first key.",
            self._onPreInfinityChanged)
        row.addWidget(self.preInfinity)
        self.postInfinity = self._InfinityCombo(
            "What the curve does AFTER its last key.",
            self._onPostInfinityChanged)
        row.addWidget(self.postInfinity)

        row.addSpacing(8)
        row.addWidget(QtWidgets.QLabel("Curve"))
        self.curveTypeCombo = QtWidgets.QComboBox()
        self.curveTypeCombo.setToolTip(
            "Ts curve type for the visible curves with keys: "
            "Bezier has free tangent widths, Hermite fixes them.")
        self.curveTypeCombo.setFixedWidth(110)
        for label, name in CURVE_TYPE_CHOICES:
            self.curveTypeCombo.addItem(label, name)
        self.curveTypeCombo.currentIndexChanged.connect(
            self._onCurveTypeChanged)
        row.addWidget(self.curveTypeCombo)
        row.addStretch(1)
        return row

    def _InfinityCombo(self, tip, slot):
        combo = QtWidgets.QComboBox()
        combo.setToolTip(tip)
        combo.setFixedWidth(132)
        for label, name in INFINITY_CHOICES:
            combo.addItem(label, name)
        combo.currentIndexChanged.connect(slot)
        return combo

    def _InstallUndoActions(self):
        """
        Ctrl+Z / Ctrl+Shift+Z while the editor has focus, on the SAME
        stack the viewport gizmos push onto.

        When the gizmo toolbar is up, its own actions are reused rather
        than duplicated: they carry the same shortcuts with
        Qt.ApplicationShortcut context, and a second action with the
        same sequence would make every Ctrl+Z ambiguous -- Qt refuses to
        fire either and prints "Ambiguous shortcut overload". Adding the
        toolbar's actions to this window is enough to reach them from
        here, and the undo stack listener repaints the canvas whichever
        one runs.
        """
        actions = self._GizmoUndoActions()
        self._ownsUndoActions = actions is None
        # An input owner's application-wide Undo / Redo reach this window; a
        # window-local Cmd-Z would be an ambiguous overload.
        managed = getattr(self, "_yieldsInput", False)
        if actions is None:
            self.undoAction = QtActionWidgets.QAction("Undo", self)
            if not managed:
                self.undoAction.setShortcut(QtGui.QKeySequence("Ctrl+Z"))
                self.undoAction.setShortcutContext(QtCore.Qt.WindowShortcut)
            self.undoAction.triggered.connect(
                lambda checked=False: self.Undo())
            self.redoAction = QtActionWidgets.QAction("Redo", self)
            if not managed:
                self.redoAction.setShortcuts([
                    QtGui.QKeySequence("Ctrl+Shift+Z"),
                    QtGui.QKeySequence("Shift+Z"),
                    QtGui.QKeySequence("Ctrl+Y")])
                self.redoAction.setShortcutContext(QtCore.Qt.WindowShortcut)
            self.redoAction.triggered.connect(
                lambda checked=False: self.Redo())
            actions = [self.undoAction, self.redoAction]
        else:
            self.undoAction, self.redoAction = actions
        for action in actions:
            self.addAction(action)

    def _GizmoUndoActions(self):
        """The toolbar's undo/redo actions when they drive OUR stack."""
        try:
            import gizmoUI
        except Exception:
            return None
        controller = gizmoUI.GetController(self.usdviewApi)
        if controller is None or controller.undoStack is not self.undoStack:
            return None
        undo = getattr(controller.toolbar, "undoAction", None)
        redo = getattr(controller.toolbar, "redoAction", None)
        if undo is None or redo is None:
            return None
        return [undo, redo]

    # -- stage and curves -----------------------------------------------

    def Stage(self):
        return self.usdviewApi.stage

    def Curves(self):
        """The CurveRefs the editor is showing, in list order."""
        return self._model.Curves()

    def CurrentFrame(self):
        return self._frame

    def StageRange(self):
        """usdview's (start, end) frames, or (None, None)."""
        stage = self.Stage()
        if not stage:
            return (None, None)
        try:
            return (stage.GetStartTimeCode(), stage.GetEndTimeCode())
        except Exception:
            return (None, None)

    def SplineCopy(self, index):
        return self._model.SplineCopy(index)

    def WriteSpline(self, index, spline, label="Graph Edit"):
        return self._model.WriteSpline(index, spline, label)

    # -- edit hook (GraphEditHook) -----------------

    def SetEditHook(self, hook):
        """
        Install `hook` (a GraphEditHook, or any object with its METHODS),
        or None for the editor's own behaviour. An open gesture is
        aborted first; the curves are re-read through the new hook.
        """
        if hook is not None:
            missing = [name for name in GraphEditHook.METHODS
                       if not callable(getattr(hook, name, None))]
            if missing:
                raise TypeError("an edit hook needs %s"
                                % ", ".join(missing))
        self.FlushPendingCommit()
        self.AbortGesture()
        self._hook = hook
        self._hookWarned = set()
        self._model.RefreshSplines()
        self._model.PruneSelection()
        self.canvas.update()
        self._Sync()

    def EditHook(self):
        return self._hook

    def IsPreviewing(self):
        """Drags preview and author once, after the release."""
        return True

    def KeySplines(self):
        """The KEY spline per curve (squares, hit tests, selection), None
        where the curve is not drawn."""
        return self._model.KeySplines()

    def AuthoredSpline(self, index):
        """What curve `index`'s attribute resolves to (knots the hook
        filters out included), or None."""
        return self._model.AuthoredSpline(index)

    def _HookCall(self, name, fallback, *args):
        """hook.name(self, *args); `fallback()` when there is no hook or
        the hook raises (warned once per method)."""
        hook = self._hook
        if hook is None:
            return fallback()
        try:
            return getattr(hook, name)(self, *args)
        except Exception as error:      # never into a Qt handler
            if name not in self._hookWarned:
                self._hookWarned.add(name)
                Tf.Warn("rigExecUsdview: graph editor hook %s failed: %s"
                        % (name, error))
            return fallback()

    def _EditContext(self):
        if self._hook is None:
            return _NullContext()
        return self._HookCall("EditContext", _NullContext)

    def _BeginEdit(self, label, indices, drag):
        """The hook's token for a new edit; True without a hook; None
        when the hook refuses (it says why)."""
        if self._hook is None:
            return True
        token = self._HookCall("BeginEdit", lambda: None, label,
                               list(indices), bool(drag))
        return token if token else None

    def _FinishEdit(self, token, changed):
        """The hook's refusal text (it rolled the edit back) or None."""
        if self._hook is None:
            return None
        refusal = self._HookCall(
            "FinishEdit", lambda: "the edit could not be completed",
            token, bool(changed))
        return refusal or None

    def _AbortEdit(self, token):
        if self._hook is not None:
            self._HookCall("AbortEdit", lambda: None, token)

    def _KeySplineThroughHook(self, index, authored):
        return self._HookCall("KeySpline", lambda: authored,
                              index, authored)

    def _CurveSplineThroughHook(self, index, authored, edited):
        return self._HookCall(
            "CurveSpline",
            lambda: edited if edited is not None else authored,
            index, authored, edited)

    def _AfterHook(self):
        """The hook may have rewritten (or rolled back) the curves."""
        if self._hook is None:
            return
        self._model.RefreshSplines()
        self._model.PruneSelection()

    def RefreshCurves(self):
        """
        Rebuild the curve set from usdview's property / prim selection
        (spec 1.3), keeping the visibility and the key selection of any
        curve that survives.
        """
        previous = self._model.RefreshCurves()
        # Prune visibility sets for curves that no longer exist:
        # without this, re-selecting the same prim re-hides curves that
        # were hidden before the rebuild.
        labels = set(ref.Label() for ref in self._model.Curves())
        self._hidden &= labels
        self._isolated &= labels
        self._RebuildCurveList()
        if self._model.Curves() and (not previous or not self._framed):
            self.FrameAll()
        self._Sync()

    def _DiscoverCurves(self):
        """The CurveRefs usdview's selection calls for (spec 1.3)."""
        stage = self.Stage()
        if not stage:
            return []
        selection = self.usdviewApi.dataModel.selection
        try:
            propPaths = selection.getPropPaths()
        except Exception:
            propPaths = []
        try:
            primPaths = selection.getPrimPaths()
        except Exception:
            primPaths = []
        return graphModel.DiscoverCurves(stage, propPaths, primPaths)

    # -- curve list -----------------------------------------------------

    def _RebuildCurveList(self):
        self._updating = True
        # Block itemChanged while the list is rebuilt: clearing and adding
        # items fires itemChanged for removed items, which would try to
        # update _hidden mid-rebuild (bug #12).
        wasBlocked = self.curveList.signalsBlocked()
        self.curveList.blockSignals(True)
        try:
            self.curveList.clear()
            for index, ref in enumerate(self._model.Curves()):
                sampled = self.IsSampled(index)
                item = QtWidgets.QTreeWidgetItem(
                    [ref.primPath.name,
                     ref.attrName + (" (samples)" if sampled else "")])
                if sampled:
                    item.setToolTip(
                        1, "Animated by time samples: each key is a sample, "
                        "joined by the stage's interpolation. Edits stay "
                        "time samples; tangents, infinity and curve type "
                        "do not apply.")
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                hidden = ref.Label() in self._hidden
                item.setCheckState(
                    0, QtCore.Qt.Unchecked if hidden else QtCore.Qt.Checked)
                # Dimmed text as well as an empty tick: one glance has to
                # say which curves are on the canvas, and a bare check
                # indicator is small against sixteen coloured rows.
                brush = QtGui.QBrush(QtGui.QColor(120, 120, 120) if hidden
                                     else _Color(ref.color))
                item.setForeground(0, brush)
                item.setForeground(1, brush)
                item.setIcon(0, _SwatchIcon(ref.color))
                item.setData(0, QtCore.Qt.UserRole, ref.Label())
                self.curveList.addTopLevelItem(item)
                item.setSelected(ref.Label() in self._isolated)
            self.curveList.resizeColumnToContents(0)
        finally:
            self.curveList.blockSignals(wasBlocked)
            self._updating = False

    def _onCurveItemChanged(self, item, column):
        del column
        if self._updating:
            return
        label = item.data(0, QtCore.Qt.UserRole)
        if item.checkState(0) == QtCore.Qt.Checked:
            self._hidden.discard(label)
        else:
            self._hidden.add(label)
        self.canvas.update()
        self._Sync()

    def _onCurveIsolation(self):
        if self._updating:
            return
        self._isolated = set(
            item.data(0, QtCore.Qt.UserRole)
            for item in self.curveList.selectedItems())
        self.canvas.update()
        self._Sync()

    def _onShowAll(self):
        self._hidden = set()
        self._isolated = set()
        self._RebuildCurveList()
        self.canvas.update()
        self._Sync()

    def IsSampled(self, index):
        """Whether curve `index` is animated by time samples, not a spline."""
        return self._model.IsSampled(index)

    def IsVisibleCurve(self, index):
        return self._model.IsVisibleCurve(index)

    def _CurveVisible(self, index):
        label = self._model.Curves()[index].Label()
        if label in self._hidden:
            return False
        return not self._isolated or label in self._isolated

    def VisibleIndices(self):
        return self._model.VisibleIndices()

    def DrawSplines(self):
        """The spline the canvas strokes per curve, None where the
        curve is not drawn."""
        return self._model.DrawSplines()

    def SetVisibleCurves(self, labels):
        """Show exactly these curves, by CurveRef.Label(), and no others."""
        wanted = set(labels)
        self._hidden = set(ref.Label() for ref in self._model.Curves()
                           if ref.Label() not in wanted)
        self._isolated = set()
        self._RebuildCurveList()
        self.canvas.update()
        self._Sync()

    # -- selection ------------------------------------------------------

    def SelectionPairs(self):
        """The raw (curveIndex, time) selection graphScreen wants."""
        return self._model.SelectionPairs()

    def SelectedKeys(self):
        """[(curve label, time)] for the selected keys, in list order."""
        return self._model.SelectedKeys()

    def SelectKeys(self, pairs):
        """Select exactly these (curve label, time) keys."""
        self._model.SelectKeys(pairs)

    def SetSelection(self, pairs):
        self._model.SetSelection(pairs)

    def ToggleKey(self, index, time):
        self._model.ToggleKey(index, time)

    def SelectGlyphs(self, glyphs, add=False):
        self._model.SelectGlyphs(glyphs, add=add)

    def _SelectionChanged(self, additive):
        if self._hook is not None:
            self._HookCall("SelectionChanged", lambda: None, additive)

    def SelectedTimes(self):
        """{curve index: [time, ...]} for the selected keys."""
        return self._model.SelectedTimes()

    def KeyExtents(self, selectedOnly=False):
        return self._model.KeyExtents(selectedOnly=selectedOnly)

    # -- gestures -------------------------------------------------------

    def BeginKeyDrag(self, press, preOnly=False):
        """
        Start moving the selected keys; False when none are, or when the
        edit hook refuses (GraphEditHook.BeginEdit).

        `preOnly` shapes arrivals: the drag moves whole keys in time
        but adjusts only preValues in value, so grabbing a dual knot's
        pre square sculpts the step without moving the departure.
        """
        self._dragPress = press
        return self._model.BeginKeyDrag(preOnly=preOnly)

    def UpdateKeyDrag(self, current, shift=False):
        press = self._dragPress
        if press is None:
            return
        axis = (graphScreen.DominantAxis(press, current)
                if shift else None)
        dt, dv = graphScreen.ResolveKeyDrag(self.canvas.transform,
                                            press, current, axis)
        self._model.DragKeys(dt, dv)

    def BeginTangentDrag(self, glyph, anchor, press=None):
        started = self._model.BeginTangentDrag(
            glyph.curveIndex, glyph.time, glyph.side, glyph.width)
        if not started:
            return False
        # The handle end keeps its offset from the cursor, so a press
        # that lands off its centre does not move it.
        grab = ((glyph.x - press[0], glyph.y - press[1])
                if press is not None else (0.0, 0.0))
        self._tangentDrag = (
            anchor, grab,
            self._model.TangentDragWeighted(glyph.curveIndex),
            glyph.width, glyph.side)
        return True

    def UpdateTangentDrag(self, current):
        """
        The handle end follows the cursor: slope from its angle and,
        while Weighted is on, width from its length. Weighted off locks
        the length. The slope is mirrored to the other side while the
        key's tangents are unified (spec 2.4).

        Hermite fixes the width (ts/types.h:104), so widths are never
        authored there even when Weighted is on: Ts would clamp them
        back and the drag would promise what the spline will not keep.
        """
        drag = self._tangentDrag
        if drag is None:
            return
        anchor, grab, weighted, width, side = drag
        current = (current[0] + grab[0], current[1] + grab[1])
        slope, resolved = graphScreen.ResolveTangentDrag(
            self.canvas.transform, anchor, current, weighted, width,
            side=side)
        self._model.DragTangent(slope, resolved if weighted else None)

    def ReleaseGesture(self):
        return self._model.ReleaseGesture()

    def HasPendingCommit(self):
        return self._model.HasPendingCommit()

    def FlushPendingCommit(self):
        """Commit a released drag now; EndGesture's result."""
        return self._model.FlushPendingCommit()

    def EndGesture(self):
        return self._model.EndGesture()

    def AbortGesture(self):
        return self._model.AbortGesture()

    def _PreviewPush(self, pending, stage):
        gizmoMath.SetPreviewValues(pending, stage=stage)
        gizmoPreview.Push(pending, session=self.usdviewApi, stage=stage)
        self.usdviewApi.UpdateViewport()

    def _PreviewEnd(self, stage, publish):
        gizmoPreview.End(session=self.usdviewApi, stage=stage,
                         publish=publish)

    def _PreviewRepublish(self):
        gizmoPreview.Republish(session=self.usdviewApi)

    # -- edit commands --------------------------------------------------

    def SetTangentType(self, mode):
        """tangent buttons, on the selected keys (spec 2.4)."""
        return self._model.SetTangentType(mode)

    def SetTangentSide(self, side):
        """Narrow the tangent buttons to the In or Out side, or Both."""
        self._model.SetTangentSide(side)
        button = self._sideButtons.get(side)
        if button is not None and not button.isChecked():
            button.setChecked(True)
        self._Sync()

    def TangentSide(self):
        return self._model.TangentSide()

    def Break(self):
        return self._model.Break()

    def Unify(self):
        return self._model.Unify()

    def InsertKeyAtCurrentFrame(self):
        return self._model.InsertKeyAtCurrentFrame()

    def InsertKeyOnCurve(self, index, time):
        """A double-click on a curve: one key, on that curve, there."""
        return self._model.InsertKeyOnCurve(index, time)

    def DeleteSelectedKeys(self):
        return self._model.DeleteSelectedKeys()

    def AnimatedIndices(self):
        return self._model.AnimatedIndices()

    def SetInfinity(self, pre=None, post=None):
        return self._model.SetInfinity(pre=pre, post=post)

    def SetCurveType(self, name):
        return self._model.SetCurveType(name)

    def CurveType(self):
        """Our name for the focus curve's Ts curve type, or None."""
        return self._model.CurveType()

    def SetSnapFrames(self, enabled):
        self._model.SetSnapFrames(enabled)
        if self.snapBox.isChecked() != self._model.SnapFrames():
            self.snapBox.setChecked(self._model.SnapFrames())

    def SnapFrames(self):
        return self._model.SnapFrames()

    def SetWeighted(self, enabled):
        self._model.SetWeighted(enabled)
        if self.weightedBox.isChecked() != self._model.Weighted():
            self.weightedBox.setChecked(self._model.Weighted())

    def Weighted(self):
        return self._model.Weighted()

    def SetKeyTime(self, time):
        return self._model.SetKeyTime(time)

    def SetKeyValue(self, value):
        """Set every selected key's departure to `value` (Value)."""
        return self._model.SetKeyValue(value)

    def SetKeyPreValue(self, preValue):
        """Set every selected key's arrival; makes the selection dual."""
        return self._model.SetKeyPreValue(preValue)

    def MakeSingle(self):
        """Drop the arrival squares: dual keys back to single-valued."""
        return self._model.MakeSingle()

    def FrameAll(self):
        self.canvas.FrameAll()
        self._framed = True

    def FrameSelected(self):
        self.canvas.FrameSelected()
        self._framed = True

    def ScrubTo(self, time, apply=True):
        """
        Move usdview's playhead. appController.setFrame snaps to the
        nearest frame in the timeline and updates the slider, the frame
        field and the viewport; the data model alone would move only the
        value and leave usdview's own widgets behind.

        `apply` False moves the editor's playhead alone; the caller
        applies the frame later with a plain ScrubTo.
        """
        try:
            frame = graphModel.SnapTime(time)
        except (ValueError, TypeError):
            return
        if not math.isfinite(frame):
            return
        start, end = self.StageRange()
        if start is not None and end is not None:
            try:
                end = float(end)
                start = float(start)
            except (TypeError, ValueError):
                pass
            else:
                if math.isfinite(start) and math.isfinite(end):
                    frame = max(min(frame, end), start)
        if not apply:
            self._frame = frame
            self.canvas.update()
            return
        controller = getattr(self.usdviewApi,
                             "_UsdviewApi__appController", None)
        scrubbed = False
        if controller is not None and hasattr(controller, "setFrame"):
            try:
                controller.setFrame(frame)
                scrubbed = True
            except Exception:
                scrubbed = False
        if not scrubbed:
            self.usdviewApi.dataModel.currentFrame = Usd.TimeCode(frame)
        self._frame = frame
        self.canvas.update()

    def Undo(self):
        self.FlushPendingCommit()
        if self.undoStack is None or not self.undoStack.Undo():
            return False
        return True

    def Redo(self):
        self.FlushPendingCommit()
        if self.undoStack is None or not self.undoStack.Redo():
            return False
        return True

    # -- keys -----------------------------------------------------------

    def OwnsEvent(self, receiver):
        """Whether a key event belongs to this window."""
        if isinstance(receiver, QtWidgets.QWidget):
            return receiver.window() is self
        return QtWidgets.QApplication.activeWindow() is self

    def HandleKey(self, key, modifiers=None):
        """One of the editor's hotkeys; True when it was acted on."""
        if modifiers is not None and modifiers & (
                QtCore.Qt.ControlModifier | QtCore.Qt.AltModifier
                | QtCore.Qt.MetaModifier):
            return False
        self.FlushPendingCommit()
        if key == QtCore.Qt.Key_Escape:
            if self.AbortGesture():
                return True
            if self._model.SelectionPairs():
                self.SetSelection([])
            return True
        if key == QtCore.Qt.Key_K:
            self.canvas.SetScrubHeld(True)
            return True
        if key == QtCore.Qt.Key_A:
            self.FrameAll()
            return True
        if key == QtCore.Qt.Key_F:
            self.FrameSelected()
            return True
        if key == QtCore.Qt.Key_Home:
            self.canvas.FrameStageRange()
            self._framed = True
            return True
        if key == QtCore.Qt.Key_I:
            self.InsertKeyAtCurrentFrame()
            return True
        if key in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            self.DeleteSelectedKeys()
            return True
        return False

    def HandleKeyRelease(self, key):
        """The release of a held hotkey (K); True when it was one."""
        if key == QtCore.Qt.Key_K:
            self.canvas.SetScrubHeld(False)
            return True
        return False

    # -- slots ----------------------------------------------------------

    def _onFrameAll(self):
        self.FrameAll()

    def _onFrameSelected(self):
        self.FrameSelected()

    def _onInsertKey(self):
        self.InsertKeyAtCurrentFrame()

    def _onDeleteKeys(self):
        self.DeleteSelectedKeys()

    def _onBreak(self):
        self.Break()

    def _onUnify(self):
        self.Unify()

    def _onSide(self, checked, side):
        if checked and not self._updating:
            self._model.SetTangentSide(side)

    def _onWeighted(self, checked):
        self._model.SetWeighted(checked)

    def _onSnapFrames(self, checked):
        self._model.SetSnapFrames(checked)

    def _onPreInfinityChanged(self, index):
        # ONLY the pre side. Passing both combos' values would push the
        # displayed pre mode -- which comes from ONE curve, the focus
        # one -- onto every other curve every time the post combo moved.
        del index
        if not self._updating:
            self.SetInfinity(pre=self.preInfinity.currentData())

    def _onPostInfinityChanged(self, index):
        del index
        if not self._updating:
            self.SetInfinity(post=self.postInfinity.currentData())

    def _onCurveTypeChanged(self, index):
        del index
        if not self._updating:
            self.SetCurveType(self.curveTypeCombo.currentData())

    def _onTimeEdited(self):
        if self._updating or self.timeBox.IsBlank():
            return
        self.SetKeyTime(self.timeBox.value())

    def _onValueEdited(self):
        if self._updating or self.valueBox.IsBlank():
            return
        self.SetKeyValue(self.valueBox.value())

    def _onPreValueEdited(self):
        if self._updating or self.preValueBox.IsBlank():
            return
        self.SetKeyPreValue(self.preValueBox.value())

    def _onMakeSingle(self):
        self.MakeSingle()

    def _onPrimSelectionChanged(self, added=None, removed=None):
        del added, removed
        self.RefreshCurves()

    def _onPropSelectionChanged(self):
        self.RefreshCurves()

    def _onStageReplaced(self):
        self.AbortGesture()
        self._model.ClearCurves()
        self._hidden = set()
        self._isolated = set()
        self._framed = False
        self._frame = _FrameValue(self.usdviewApi.frame)
        self._ObserveStage(self.usdviewApi.dataModel.stage)
        self.RefreshCurves()

    def _onFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns, so re-reading the property here would
        # leave the playhead a scrub behind (rigExecUsdview._FrameValue).
        self.FlushPendingCommit()
        self._frame = _FrameValue(frame)
        self._model.PushPreview()
        self.canvas.update()
        self._Sync()

    def _onUndoStackChanged(self):
        # An undo restores layer specs behind our back; the cached
        # splines are stale until they are read again.
        if not self._model.RefreshAfterUndo():
            return
        self.canvas.update()
        self._Sync()
        self.usdviewApi.UpdateViewport()

    def _ObserveStage(self, stage):
        if self._noticeKey is not None:
            try:
                self._noticeKey.Revoke()
            except Exception:
                pass
            self._noticeKey = None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._onObjectsChanged, stage)

    def _onObjectsChanged(self, notice, stage):
        """
        Re-read a listed curve whose attribute was authored elsewhere,
        so a gizmo drag shows up in the graph live (spec 2.5).

        Not during our own gesture: the drag already holds the authored
        spline, and re-reading mid-drag would fight it.
        """
        del stage
        paths = set(notice.GetChangedInfoOnlyPaths())
        paths.update(notice.GetResyncedPaths())
        if self._model.NoteExternalChange(paths):
            self.canvas.update()
            self._Sync()

    # -- status ---------------------------------------------------------

    def Status(self):
        """One line about what the editor is showing (spec 2.1)."""
        if not self.Stage():
            return "No stage."
        curves = self._model.Curves()
        if not curves:
            return ("No spline-capable attributes in the selection. "
                    "Select a rig control, or a scalar attribute in the "
                    "property browser.")
        visible = len(self._model.VisibleIndices())
        text = "%d curve%s (%d shown)" % (len(curves),
                                          "" if len(curves) == 1
                                          else "s", visible)
        selected = len(self._model.SelectionPairs())
        if selected:
            text += ", %d key%s selected" % (
                selected, "" if selected == 1 else "s")
        else:
            text += ", no keys selected"
        text += "  frame %g" % self._frame
        label = self._model.ActiveGestureLabel()
        if label is not None:
            text += "  [%s]" % label
        if self._notice:
            text += "  --  %s" % self._notice
        return text

    def _Notify(self, message):
        """
        Say why a command did nothing, on the status line.

        The message survives exactly until the next thing the panel
        does: a refusal an artist never sees is a refusal that reads as
        a broken button, and one that outlives its cause reads as a
        broken editor.
        """
        self._notice = message or ""
        self.statusLabel.setText(self.Status())

    def _Sync(self):
        """Mirror the model onto the key stats, the status and the undo."""
        self._updating = True
        try:
            # A refusal explains the command that has just been refused,
            # so the next thing the panel does retires it.
            self._notice = ""
            self._SyncKeyStats()
            self._SyncInfinityCombos()
            self._SyncCurveTypeCombo()
            self._SyncWeightedEnabled()
            self.statusLabel.setText(self.Status())
            if self._ownsUndoActions and self.undoStack is not None:
                self.undoAction.setEnabled(self.undoStack.CanUndo())
                self.redoAction.setEnabled(self.undoStack.CanRedo())
        finally:
            self._updating = False

    def _SyncKeyStats(self):
        times, values, pres = [], [], []
        for index, time in self._model.SelectionPairs():
            times.append(time)
            spline = self._model.KeySpline(index)
            knot = spline.GetKnot(time) if spline is not None else None
            if knot is not None:
                values.append(float(knot.GetValue()))
                if knot.IsDualValued():
                    pres.append(float(knot.GetPreValue()))
        if len(set(times)) == 1:
            self.timeBox.SetNumber(times[0])
        else:
            self.timeBox.SetBlank()
        if values and max(values) - min(values) < 1e-12:
            self.valueBox.SetNumber(values[0])
        else:
            self.valueBox.SetBlank()
        # Pre is blank unless EVERY selected key is dual and the arrivals
        # agree: a single-valued selection has no arrival to show, and
        # typing into a blank Pre makes the selection dual.
        selectedCount = 0
        for index, time in self._model.SelectionPairs():
            spline = self._model.KeySpline(index)
            knot = spline.GetKnot(time) if spline is not None else None
            if knot is not None:
                selectedCount += 1
        if (selectedCount and len(pres) == selectedCount
                and max(pres) - min(pres) < 1e-12):
            self.preValueBox.SetNumber(pres[0])
        else:
            self.preValueBox.SetBlank()

    def FocusCurveIndex(self):
        return self._model.FocusCurveIndex()

    def _SyncInfinityCombos(self):
        """
        Show the focus curve's infinities, WITHOUT writing.

        Setting a combo's index fires its slot, so this only ever runs
        under the `_updating` guard `_Sync` holds; otherwise merely
        looking at a second curve would author the first one's modes
        onto it.
        """
        index = self.FocusCurveIndex()
        spline = (self._model.AuthoredSpline(index)
                  if index is not None else None)
        if spline is None:
            return
        for combo, getter in ((self.preInfinity, spline.GetPreExtrapolation),
                              (self.postInfinity,
                               spline.GetPostExtrapolation)):
            try:
                name = graphModel.EXTRAP_NAMES.get(getter().mode)
            except Exception:
                name = None
            if name is None:
                continue
            item = combo.findData(name)
            if item >= 0 and item != combo.currentIndex():
                combo.setCurrentIndex(item)

    def _SyncCurveTypeCombo(self):
        """
        Show the focus curve's Ts curve type, WITHOUT writing.

        Same guard contract as _SyncInfinityCombos: runs only under
        `_updating`, or selecting another curve would author the old
        focus curve's type onto every visible curve.
        """
        name = self.CurveType()
        if name is None:
            return
        item = self.curveTypeCombo.findData(name)
        if item >= 0 and item != self.curveTypeCombo.currentIndex():
            self.curveTypeCombo.setCurrentIndex(item)

    def _SyncWeightedEnabled(self):
        """
        Weighted matters only for Bezier (ts/types.h:103-104).

        Hermite fixes the tangent width to 1/3 of the segment, so a
        weighted drag there would move the handle and change nothing on
        eval (Ts clamps the stored width back). Disabling the box while
        the focus curve is Hermite says that up front; the drag path
        also ignores widths on Hermite so a scripted call cannot
        promise what the spline will not keep.
        """
        hermite = self.CurveType() == graphModel.CURVE_HERMITE
        self.weightedBox.setEnabled(not hermite)
        if hermite:
            self.weightedBox.setToolTip(
                "Weighted needs Bezier: Hermite fixes the tangent width.")
        else:
            self.weightedBox.setToolTip(WEIGHTED_TIP)

    # -- window ---------------------------------------------------------

    def changeEvent(self, event):
        if event.type() == QtCore.QEvent.ActivationChange \
                and not self.isActiveWindow():
            self.canvas.SetScrubHeld(False)
        super(GraphEditorPanel, self).changeEvent(event)

    def closeEvent(self, event):
        self.FlushPendingCommit()
        super(GraphEditorPanel, self).closeEvent(event)

    def showEvent(self, event):
        super(GraphEditorPanel, self).showEvent(event)
        self.canvas.setFocus()
        if not self._framed:
            self.FrameAll()


def _SwatchIcon(rgb, size=10):
    """A solid colour chip for the curve list's first column."""
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(_Color(rgb))
    return QtGui.QIcon(pixmap)


def _FrameValue(frame):
    """A Usd.TimeCode or number as a plain float frame."""
    if isinstance(frame, Usd.TimeCode):
        return 0.0 if frame.IsDefault() else frame.GetValue()
    try:
        return float(frame)
    except (TypeError, ValueError):
        return 0.0


# Entry points

def OpenGraphEditor(usdviewApi, undoStack=None):
    """
    Show the single Graph Editor for this session, creating it once.

    `undoStack` is the shared rigExecUndo.UndoStack the viewport gizmos
    push onto, so Ctrl+Z undoes graph and viewport edits from one
    history (spec 2.5).
    """
    if undoStack is None:
        undoStack = rigExecUndo.UndoStack()
    panel = GraphEditorPanel.GetInstance(usdviewApi, undoStack)
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel


def GetGraphEditor(usdviewApi=None):
    """The open Graph Editor of `usdviewApi`'s session (without one: the
    current session's), or None."""
    return GraphEditorPanel.Instance(usdviewApi)
