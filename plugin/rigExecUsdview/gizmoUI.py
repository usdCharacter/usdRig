"""usdview host wiring and flat-import compatibility for the viewport gizmo."""
from functools import partial
import math
import os
import sys

from pxr import Gf, Sdf, Tf, Usd, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import gizmoDrag
    import gizmoIcons
    import gizmoModel
    import gizmoWidgets
    import gizmoMarquee
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import gizmoSnap
    import gizmoPreview
    import pathCurveModel
    import editorExtension
    import sessionRegistry
    import stageViewReady
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoDrag
    import gizmoIcons
    import gizmoModel
    import gizmoWidgets
    import gizmoMarquee
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import gizmoSnap
    import gizmoPreview
    import pathCurveModel
    import editorExtension
    import sessionRegistry
    import stageViewReady


from gizmoWidgets import (GizmoOverlay, ViewportStatusBar, ViewportToolbar,
                          ToolSettingsPanel, ViewportHotkeyFilter,
                          QtActionWidgets, TOOL_LABELS, HIT_PIXELS,
                          _TEXT_WIDGETS)

TOOL_SELECT = gizmoSettings.TOOL_SELECT
TOOL_TRANSLATE = gizmoScreen.TOOL_TRANSLATE
TOOL_ROTATE = gizmoScreen.TOOL_ROTATE
TOOL_SCALE = gizmoScreen.TOOL_SCALE

_CAMERA_MODIFIERS = (QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier)


def StageView(usdviewApi):
    return stageViewReady.StageView(usdviewApi)


class GizmoController(QtCore.QObject):
    """
    Connects the gesture model to the usdview stage, view and input filters.

    Everything the viewport does goes through here: the toolbar and the
    settings panel only set state, and the overlay only reads it.
    """

    def _MakeGestureModel(self):
        api = self.usdviewApi
        return gizmoModel.GizmoModel(
            stage=lambda: api.stage, pushUndo=self.undoStack.Push,
            author=lambda writer: writer.CommitToStage(),
            previewPush=partial(gizmoPreview.Push, session=api),
            previewEnd=partial(gizmoPreview.End, session=api),
            previewRepublish=partial(gizmoPreview.Republish, session=api),
            documentEdit=lambda: editorExtension.DocumentEdit(api))

    def _GestureField(name):
        # Compatibility for hosts that seed drag state on a bare controller.
        def get(self):
            return getattr(self._gesture, name)

        def set(self, value):
            if not hasattr(self, "_gesture"):
                self._gesture = self._MakeGestureModel()
            setattr(self._gesture, name, value)

        return property(get, set)

    _drag = _GestureField("drag")
    _releasedEdit = _GestureField("releasedEdit")
    _committing = _GestureField("committing")
    del _GestureField

    def __init__(self, usdviewApi, undoStack, parent=None,
                 openGraphEditor=None, installFilters=True):
        super(GizmoController, self).__init__(parent)
        self.usdviewApi = usdviewApi
        self.undoStack = undoStack
        # False: no stage-view or application filter; an input owner
        # forwards events to the Handle* methods below.
        self._installFilters = installFilters
        # fn(tool) after SetTool changes the tool token.
        self.toolChangedHooks = []
        # The container's opener, so this module never imports
        # graphEditorUI at module scope and stays loadable in the
        # headless contexts the C++ tests use. None falls back to a
        # lazy import inside OpenGraphEditor.
        self._openGraphEditor = openGraphEditor
        self.settings = gizmoSettings.GizmoSettings()

        self._tool = TOOL_SELECT
        self._channels = gizmoMath.CHANNELS_POSE
        self._writeMode = gizmoMath.WRITE_ANIMATION
        self._target = None
        self._reason = "no stage"
        # The selection the current target was built from, lead last, so
        # a stage notice can tell a target that is merely stale from one
        # that is built on the wrong set of prims. The focus prim alone
        # cannot answer that any more: a group selection can gain or
        # lose a member with the lead unchanged.
        self._targetPaths = ()
        self._warnings = []
        self._handles = []
        self._gesture = self._MakeGestureModel()
        self._selected = None
        self._hover = None
        self._holdSnap = False
        self._holdGrid = False
        self._holdEdge = False
        self._holdPoint = False
        self._holdPreserve = False
        self._hoverPoint = None
        self._hoverSnap = None
        self._hoverSnapKind = None
        self._hoverPickPoint = None
        self._hoverNote = ""
        self._lastSnapNote = ""
        self._hoverRigStage = None
        self._hoverRigRoot = None
        self._hoverRigWritten = frozenset()
        self._hoverRigWrittenByTarget = frozenset()
        self._hoverRigAllWritten = False
        self._hoverRigAllWrittenByTarget = False
        self._snapGeom = {}
        self._ctrl = False
        self._claimedKey = None
        self._hotkeys = None
        self._visible = True
        self._gimbalAxes = None
        self._rebuilding = False
        self._panel = None
        self._noticeKey = None
        self._solverPosed = gizmoMath.SolverPosedCache()
        self._rubber = None
        self._frame = usdviewApi.frame

        view = StageView(usdviewApi)
        self._view = view
        self._viewDestroyed = False
        from controlPicking import ControlPicking
        self._controlPicking = ControlPicking(view) if view is not None else None
        self.overlay = GizmoOverlay(self, view) if view is not None else None
        self.statusBar = ViewportStatusBar()
        self.toolbar = ViewportToolbar(self, self.statusBar)

        self._InstallToolbar()
        self._InstallUndoShortcuts()
        if installFilters:
            self._hotkeys = ViewportHotkeyFilter(self)
        application = QtWidgets.QApplication.instance()
        if application is not None:
            if self._hotkeys is not None:
                application.installEventFilter(self._hotkeys)
            # Undo / redo are application-wide shortcuts, and two sessions
            # in one process would make every Ctrl+Z an ambiguous overload
            # that fires in neither window. See _ScopeShortcuts.
            application.focusWindowChanged.connect(
                self._onFocusWindowChanged)
            application.aboutToQuit.connect(self.FlushReleasedEdit)
        if view is not None:
            # The stage view keeps the MOUSE filter; keys go through the
            # application filter above (see ViewportHotkeyFilter).
            if installFilters:
                view.installEventFilter(self)
            view.destroyed.connect(self._onViewDestroyed)
            # WA_Hover, not setMouseTracking: the pre-selection
            # highlight needs mouse moves with no button down, and Qt
            # delivers those to a widget only if it tracks the mouse --
            # which for usdview's stage view would also turn on a GPU
            # pickObject per mouse move (stageView.mouseMoveEvent's
            # "none" camera mode). WA_Hover produces HoverMove events
            # instead, which nothing else in usdview listens for.
            self._previousHover = view.testAttribute(QtCore.Qt.WA_Hover)
            view.setAttribute(QtCore.Qt.WA_Hover, True)
            view.signalFrustumChanged.connect(self._onFrustumChanged)
        self.settings.AddListener(self._onSettingsChanged)
        self.undoStack.AddListener(self._onUndoStackChanged)

        model = usdviewApi.dataModel
        model.selection.signalPrimSelectionChanged.connect(
            self._onSelectionChanged)
        model.signalStageReplaced.connect(self._onStageReplaced)
        model.currentFrameChanged.connect(self._onFrameChanged)
        self._ObserveStage(model.stage)

        self.RefreshTarget()
        self._SyncOverlay()

    # -- installation ---------------------------------------------------

    def _InstallToolbar(self):
        """
        Above the viewport, inside the frame that holds the stage view,
        so it scrolls and hides with the viewport rather than joining
        usdview's own menu bar.
        """
        try:
            layout = self.usdviewApi._UsdviewApi__appController._ui \
                .glFrame.layout()
        except AttributeError:
            layout = None
        if layout is None:
            return
        layout.insertWidget(0, self.toolbar)
        layout.insertWidget(1, self.statusBar)
        self.toolbar.Sync()

    def IsToolbarRowReleased(self):
        return getattr(self, "_rowReleased", False)

    def ReleaseToolbarRow(self):
        """
        Take the toolbar and status rows out of glFrame for good, for a
        host that shows the toolbar elsewhere. The controller keeps both
        widgets. Idempotent; a plain session never calls it.
        """
        if getattr(self, "_rowReleased", False):
            return
        self._rowReleased = True
        for widget in (self.toolbar, self.statusBar):
            try:
                if widget.parent() is not None:
                    widget.hide()
                    widget.setParent(None)
            except RuntimeError:
                pass
        self.toolbar.SetToolButtonsVisible(False)

    def _UndoActions(self):
        return [a for a in (getattr(self.toolbar, "undoAction", None),
                            getattr(self.toolbar, "redoAction", None))
                if a is not None]

    def _InstallUndoShortcuts(self):
        """
        Give usdview's main window its own handle on the undo and redo
        actions.

        Their shortcuts are Qt.ApplicationShortcut, but Qt will not fire
        a shortcut whose owning widget is hidden -- and RigExec ->
        Viewport Tools hides the toolbar while leaving every edit on the
        undo stack, so Ctrl+Z died on a stack full of drags. Adding the
        SAME actions to the main window gives them a second home that is
        visible for as long as usdview is. They stay parented to the
        toolbar, so they are still destroyed with it, and Detach takes
        them back off for the case where the toolbar outlives its view.
        """
        if not getattr(self, "_installFilters", True):
            # An input owner owns the Undo / Redo chords.
            return
        window = self._MainWindow()
        if window is None:
            return
        for action in self._UndoActions():
            window.addAction(action)

    def Detach(self):
        if getattr(self, "_detached", False):
            return
        self._detached = True
        self._viewDestroyed = True
        stageViewReady._Cleanup("gizmo", self.FlushReleasedEdit)
        stageViewReady._Cleanup("gizmo", self.CancelGesture)
        if self._controlPicking is not None:
            self._controlPicking.Detach()
            self._controlPicking = None
        view, self._view = self._view, None
        if view is not None:
            if self._installFilters:
                view.removeEventFilter(self)
            view.setAttribute(QtCore.Qt.WA_Hover, self._previousHover)
            stageViewReady.Disconnect(view, "destroyed", self._onViewDestroyed)
            stageViewReady.Disconnect(view, "signalFrustumChanged",
                                      self._onFrustumChanged)
        application = QtWidgets.QApplication.instance()
        if application is not None:
            if self._hotkeys is not None:
                application.removeEventFilter(self._hotkeys)
            stageViewReady.Disconnect(application, "focusWindowChanged",
                                      self._onFocusWindowChanged)
            stageViewReady.Disconnect(application, "aboutToQuit",
                                      self.FlushReleasedEdit)
        self._hotkeys = None
        model = self.usdviewApi.dataModel
        stageViewReady.Disconnect(getattr(model, "selection", None),
                                  "signalPrimSelectionChanged",
                                  self._onSelectionChanged)
        stageViewReady.Disconnect(model, "signalStageReplaced", self._onStageReplaced)
        stageViewReady.Disconnect(model, "currentFrameChanged", self._onFrameChanged)
        settings = getattr(self, "settings", None)
        if settings is not None:
            settings.RemoveListener(self._onSettingsChanged)
        remove = getattr(self.undoStack, "RemoveListener", None)
        if remove is not None:
            remove(self._onUndoStackChanged)
        self._ObserveStage(None)
        if getattr(self, "_extensionNotified", False):
            editorExtension.EditorRemoved(self.usdviewApi, editorExtension.GIZMO, self)
        window = self._MainWindow()
        if window is not None:
            for action in self._UndoActions():
                stageViewReady.IgnoreDeleted(lambda: window.removeAction(action))
        for widget in (self._rubber, self.overlay, self.toolbar,
                       getattr(self, "statusBar", None)):
            stageViewReady.RemoveWidget(widget)
        self.overlay = self._rubber = None
        for controller in stageViewReady.Controllers("gizmo"):
            scope = getattr(controller, "_ScopeShortcuts", None)
            if scope is not None:
                scope()

    def _onFocusWindowChanged(self, *args):
        self._ScopeShortcuts()

    def _ScopeShortcuts(self):
        """
        Keep the toolbar's undo / redo application-wide only while this
        session owns the active window.

        Qt.ApplicationShortcut is what lets Ctrl+Z work with focus in a
        panel, but when several usdview sessions share one process every
        session's Ctrl+Z is the same application shortcut and Qt fires
        none of them ("Ambiguous shortcut overload"). So, with more than
        one session, the session in front keeps the application-wide
        context and every other session's actions become window-scoped
        (the rule usdOrchestrate's host applies to usdview's own
        actions). A session alone in its process is exactly as before.
        A window no session owns changes nothing.
        """
        context = QtCore.Qt.ApplicationShortcut
        installed = stageViewReady.Controllers("gizmo")
        if len(installed) > 1:
            windows = [controller._MainWindow() for controller in installed]
            owner = next((window for window in sessionRegistry._WindowChain(
                sessionRegistry.ActiveWindow())
                if any(window is main for main in windows)), None)
            if owner is None:
                return
            if owner is not self._MainWindow():
                context = QtCore.Qt.WindowShortcut
        for action in self._UndoActions():
            try:
                if action.shortcutContext() != context:
                    action.setShortcutContext(context)
            except RuntimeError:
                continue

    def _onViewDestroyed(self, *args):
        self._viewDestroyed = True
        self._view = None
        self.overlay = None
        # The rubber band is a CHILD of the view Qt has just destroyed;
        # the Python wrapper outlives the C++ object, so touching it
        # again would raise RuntimeError on the next marquee.
        self._rubber = None
        self._gesture.ClearMarquee()
        self.Detach()

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

    # -- state ----------------------------------------------------------

    def Tool(self):
        return self._tool

    def Orientation(self):
        """
        The ACTIVE TOOL's Axis Orientation token.

        One field, read by three things: the toolbar's Global/Local
        toggle, the Tool Settings combo box, and _Orientation() where it
        actually decides which frame the handles are laid out on. There
        is no second copy for the toolbar to drift out of step with.
        """
        return self.settings.For(self._tool).orientation

    def SetOrientation(self, orientation):
        """
        Put the active tool on `orientation`.

        The write goes through the settings model, whose Notify() runs
        _onSettingsChanged -- which rebuilds the handles and re-syncs
        BOTH the toolbar and the panel. So the toggle and the combo box
        cannot disagree, and a drag that is in progress redraws on the
        new axes without being interrupted.
        """
        if orientation not in gizmoSettings.OrientationChoices(self._tool):
            return False
        self.settings.For(self._tool).orientation = orientation
        return True

    def ToggleOrientation(self):
        """Global <-> Local, the toolbar toggle's keyboard twin (L)."""
        return self.SetOrientation(
            gizmoSettings.NextToggleOrientation(self.Orientation()))

    def GroupPivot(self):
        """The ACTIVE TOOL's group pivot token (gizmoMath.GROUP_PIVOT_*)."""
        return self.settings.For(self._tool).groupPivot

    def SetGroupPivot(self, mode):
        """
        Choose what a MULTI-selection turns about.

        Like SetOrientation, the write goes through the settings model,
        so _onSettingsChanged re-primes the live target
        (_PrimeGroupPivot) and re-syncs the toolbar and the panel: the
        button, the combo box and the target cannot drift apart.
        """
        if mode not in gizmoSettings.GroupPivotChoices(self._tool):
            return False
        self.settings.For(self._tool).groupPivot = mode
        return True

    def CycleGroupPivot(self):
        """Centre -> Last Selected -> Individual Origins, wrapping
        (Shift+P)."""
        return self.SetGroupPivot(gizmoSettings.NextGroupPivot(
            self.GroupPivot(), self._tool))

    def SetTool(self, tool):
        if tool == self._tool:
            return
        self._AbortDrag()
        self._tool = tool
        self._selected = None
        self._hover = None
        self.RefreshTarget()
        self.toolbar.Sync()
        if self._panel is not None:
            self._panel.Rebuild()
        for hook in list(getattr(self, "toolChangedHooks", ())):
            try:
                hook(tool)
            except Exception as error:
                Tf.Warn("rigExecUsdview: tool change hook failed: %s"
                        % error)

    def Channels(self):
        return self._channels

    def SetChannels(self, channels):
        if channels == self._channels:
            return
        self._AbortDrag()
        self._channels = channels
        self.RefreshTarget()
        self.toolbar.Sync()

    def WriteMode(self):
        return self._writeMode

    def SetWriteMode(self, mode):
        if mode == self._writeMode:
            return
        self._AbortDrag()
        self._writeMode = mode
        self.RefreshTarget()
        self.toolbar.Sync()

    def Target(self):
        return self._target

    def Reason(self):
        return self._reason

    def Handles(self):
        return self._handles

    def IsDragging(self):
        return self._drag is not None

    def SelectedHandleName(self):
        return self._selected

    def HoverHandleName(self):
        return self._hover

    def DragAngle(self):
        """Degrees swept so far by a rotate drag; 0 when not rotating."""
        return self._drag.angle if self._drag is not None else 0.0

    def IsVisible(self):
        return self._visible

    # -- input from an owner (installFilters=False; editorExtension) ------
    # Each method runs the body the matching filter runs, so forwarded
    # events take the same code.

    def InstallsFilters(self):
        return getattr(self, "_installFilters", True)

    def HasView(self):
        return self._view is not None

    def HasTarget(self):
        return self._target is not None

    def ForwardViewEvent(self, event):
        """The stage-view filter for one event; True when consumed."""
        view = self._view
        return bool(view is not None and self.eventFilter(view, event))

    def HandlePress(self, event):
        """A viewport press: ends a text edit first, as a click does."""
        self._DropTypingFocus()
        return bool(self._OnPress(event))

    def IsMarqueePending(self):
        return self._gesture.marqueeAt is not None

    def HandleMove(self, event):
        return bool(self._OnMove(event))

    def HandleRelease(self, event):
        return bool(self._OnRelease(event))

    def HandleHover(self, point):
        """Pre-selection at physical pixel `point`; None: the pointer left."""
        self._UpdateHover(point)

    def HandleKeyRelease(self, event, key):
        """`event`: a QKeyEvent, or any object with isAutoRepeat(),
        modifiers() and key()."""
        if key == self._claimedKey:
            self._claimedKey = None
        return bool(self._ReleaseHold(event, key))

    def HandleDragKey(self, key):
        return bool(self._DragKey(key))

    def HandleToolKey(self, key, modifiers):
        """`modifiers`: the Qt modifiers the key acts with."""
        self._keyModifiers = modifiers
        return bool(self._ToolKey(key))

    def CancelGesture(self):
        """Abort a drag and drop a pending marquee."""
        if self.IsDragging():
            self._AbortDrag()
        gesture = getattr(self, "_gesture", None)
        if gesture is not None:
            if gesture.marqueeAt is not None:
                gesture.ClearMarquee()
                self._HideBand()
        elif self._marqueeAt is not None:
            # Keep the unbound delegation hook usable by legacy hosts.
            self._marqueeAt = self._marqueeBand = None
            self._marqueeWas = ()
            self._HideBand()

    def ClearHolds(self):
        self._ClearHolds()

    def SetVisible(self, visible):
        visible = bool(visible)
        if visible == self._visible:
            return
        self._visible = visible
        self._AbortDrag()
        if getattr(self, "_rowReleased", False):
            # Shown elsewhere (or nowhere): never show a parentless
            # toolbar or status row as a window.
            if self.toolbar.parent() is not None:
                self.toolbar.setVisible(visible)
        else:
            self.toolbar.setVisible(visible)
            self.statusBar.setVisible(visible)
        if self.overlay is not None:
            self.overlay.setVisible(visible)
        self._RebuildHandles()

    # -- target and handles ---------------------------------------------

    def _Writer(self):
        return gizmoMath.Writer(self.usdviewApi.stage, self._frame,
                                self._writeMode)

    def _FocusPrim(self):
        """
        The one prim the gizmo edits (design assumption 3).

        usdview's focus prim, except that its selection model keeps the
        PSEUDO-ROOT focused when a prim is added to an already-cleared
        selection -- which is exactly what UsdviewApi.ClearPrimSelection
        followed by AddPrimToSelection does, and therefore what every
        script driving usdview does. Falling back to the last real prim
        in the selection makes the documented API work; a click in the
        viewport or the prim tree sets the focus properly and never
        reaches the fallback.
        """
        api = self.usdviewApi
        prim = api.prim
        if prim and not prim.IsPseudoRoot():
            return prim
        for candidate in reversed(list(api.selectedPrims or [])):
            if candidate and not candidate.IsPseudoRoot():
                return candidate
        return None

    def _SelectedPrims(self):
        """
        Everything the gizmo will manipulate, LEAD LAST.

        Lead last because that is what gizmoMath.MakeGroupTarget takes,
        and because the LAST-picked control is what orients the group
        frame and answers to the Last Selected pivot -- the editor
        convention.

        NOT the focus prim, and this was measured the hard way.
        usdview's focus prim is getPrimPaths()[0] (selectionDataModel
        .py:623-627) -- the FIRST prim in the selection, not the last.
        Shift-clicking the shoulder, then the elbow, then the wrist in a
        live usdview gives api.prim == arm_l_fk_shoulder_l_bind, so
        treating the focus prim as the lead put the FIRST control picked
        in charge of the gizmo's axes, which is backwards from every
        application. The selection is insertion-ordered, so the last real entry
        IS the last-picked prim.

        The pseudo-root is dropped rather than led: usdview leaves `/`
        in the selection after ClearPrimSelection() followed by
        AddPrimToSelection() -- measured, a four-control script
        selection reads back as ['/', shoulder, elbow, wrist, spine]
        with `/` as the focus -- and every documented way of driving
        usdview goes through that pair. _FocusPrim is the fallback for
        the case where the selection resolves to nothing at all.
        """
        api = self.usdviewApi
        prims = []
        seen = set()
        for prim in (api.selectedPrims or []):
            if not prim or prim.IsPseudoRoot():
                continue
            path = prim.GetPath()
            if path not in seen:
                seen.add(path)
                prims.append(prim)
        if prims:
            return prims
        lead = self._FocusPrim()
        return [lead] if lead is not None else []

    def _SelectionSignature(self):
        """The paths _SelectedPrims would resolve to, for comparison."""
        return tuple(p.GetPath() for p in self._SelectedPrims())

    @staticmethod
    def _TargetPrimPaths(target):
        """
        Every prim `target` edits: one for a plain target, all of them
        for a group.
        """
        members = getattr(target, "members", None)
        if members:
            return [m.prim.GetPath() for m in members]
        return [target.prim.GetPath()]

    def RefreshTarget(self):
        """
        Re-resolve the edit target from usdview's SELECTION, then
        rebuild the handles and the status line.

        One selected prim gives the single target it always did; several
        give a gizmoMath.GroupTarget over all of them, pivoted on their
        centroid and oriented like the lead. The two paths are kept
        apart deliberately -- a single selection must not start paying
        for a group -- and they meet again at the Target interface, so
        everything below here (handles, drag, preview, undo) is unaware
        of which one it is holding.
        """
        self.FlushReleasedEdit()
        if self._viewDestroyed:
            return
        stage = self.usdviewApi.stage
        prims = self._SelectedPrims() if stage else []
        prim = prims[-1] if prims else None
        # The hover preview resolved against the old target: the next
        # hover move re-picks. Cleared before toolbar.Sync() below, so
        # the status never names a stale candidate.
        self._hoverSnap = None
        self._hoverSnapKind = None
        self._hoverPickPoint = None
        self._hoverNote = ""
        self._warnings = []
        if not stage:
            self._target, self._reason = None, "no stage"
        elif len(prims) > 1:
            # ONE Writer for the whole group, handed to every member:
            # that is what lets the drag push all of their previewed
            # values to Hydra in a single UpdatePreview per mouse sample
            # (gizmoPreview.Push takes writer.Pending()) and author them
            # in a single change block on release.
            self._target, self._reason = gizmoMath.MakeGroupTarget(
                stage, prims, self._channels, self._Writer(),
                self._solverPosed)
        else:
            # A curve a rig's path mover samples edits its control points.
            self._target = pathCurveModel.MakeTarget(
                stage, prim, self._frame, self._writeMode)
            self._reason = ""
            if self._target is None:
                self._target, self._reason = gizmoMath.MakeTarget(
                    stage, prim, self._channels, self._Writer(),
                    self._solverPosed)
        self._targetPaths = tuple(p.GetPath() for p in prims)
        self._PrimePreserveChildren()
        self._PrimeGroupPivot()
        self._RebuildHandles()
        self.toolbar.Sync()
        if self._panel is not None:
            self._panel.Sync()

    def _PrimePreserveChildren(self):
        """
        Tell the target whether Preserve Children is on, and let it work
        out which children it would have to skip.

        AttributePaths() is what recomputes target.skippedChildren, so
        calling it here is what puts the count in the status label
        BEFORE a drag rather than only after one -- an artist who ticks
        the box wants to know then that two of the children will not
        move with the parent, not once it is too late. _BeginDrag sets
        the same state again in the order the Target requires.
        """
        target = self._target
        if target is None:
            return
        try:
            target.SetPreserveChildren(
                (bool(self.settings.For(self._tool).preserveChildren)
                 or self._holdPreserve)
                and target.supportsPreserveChildren)
            target.AttributePaths()
        except Exception as error:
            Tf.Warn("rigExecUsdview: preserve-children probe failed: %s"
                    % error)

    def _PrimeGroupPivot(self):
        """
        Tell a group target what to turn about (Tool Settings > Group
        Pivot; gizmoMath.GROUP_PIVOT_*).

        Set here rather than read inside the maths so the answer cannot
        change under a live drag: BeginDrag() captures the pivot once,
        and a mid-drag settings write would otherwise move it while the
        cursor was down. A single target has no such setting and is
        left alone.
        """
        target = self._target
        setter = getattr(target, "SetPivotMode", None)
        if setter is None:
            return
        settings = self.settings.For(self._tool)
        setter(settings.groupPivot)
        orient = getattr(target, "SetAxisOrientation", None)
        if orient is not None:
            orient(settings.orientation)

    def _Orientation(self, target):
        """
        (orientation matrix, gimbal axes) for the active Axis
        Orientation. World needs no frame at all -- the identity is the
        world basis -- and Gimbal falls back to Object for a target with
        no Euler channels to decompose.
        """
        settings = self.settings.For(self._tool)
        choice = settings.orientation
        if choice == gizmoSettings.ORIENT_GIMBAL:
            state = target.RotationState()
            if state is not None:
                order, angles = state
                return (target.ObjectFrame(),
                        gizmoMath.GimbalAxes(order, angles[0], angles[1],
                                             angles[2],
                                             target.GimbalFrame()))
            choice = gizmoSettings.ORIENT_OBJECT
        if choice == gizmoSettings.ORIENT_OBJECT:
            return target.ObjectFrame(), None
        if choice == gizmoSettings.ORIENT_PARENT:
            return target.ChannelFrame(), None
        return Gf.Matrix4d(1.0), None

    def _ToolSupported(self, target):
        if self._tool == TOOL_TRANSLATE:
            return target.supportsTranslate
        if self._tool == TOOL_ROTATE:
            return target.supportsRotate
        if self._tool == TOOL_SCALE:
            return target.supportsScale
        return False

    def _RebuildHandles(self):














        if self._rebuilding:
            return
        self._rebuilding = True
        try:
            self._handles = self._ComputeHandles()
        finally:
            self._rebuilding = False
        if self.overlay is not None:
            self.overlay.update()

    def _ComputeHandles(self):
        target = self._target
        if (not self._visible or target is None
                or self._tool == TOOL_SELECT
                or not self._ToolSupported(target)):
            return []
        camera, viewport, ratio = self._Camera()
        if camera is None:
            return []
        # A path curve puts a handle on each control point instead.
        build = getattr(target, "BuildHandles", None)
        if build is not None:
            self._gimbalAxes = None
            return build(camera, viewport, ratio)
        try:
            matrix = target.GizmoMatrix()
            orientation, gimbal = self._Orientation(target)
        except Exception as error:
            Tf.Warn("rigExecUsdview: gizmo frame failed: %s" % error)
            return []
        settings = self.settings.For(self._tool)
        # What the rings were actually laid out on, not what the option
        # asks for: Gimbal falls back to Object for a target with no
        # Euler channels, and a drag has to write through the entry
        # point that matches the axes it can see.
        self._gimbalAxes = gimbal
        return gizmoScreen.BuildHandles(
            self._tool, matrix, camera, viewport, ratio,
            sizePixels=self.settings.manipulatorSize,
            orientation=orientation, gimbalAxes=gimbal,
            freeRotate=bool(settings.freeRotate))

    def _Camera(self):
        """
        (Gf.Camera, viewport, devicePixelRatio) or (None, None, 1.0).

        resolveCamera() is NOT free of side effects -- it conforms the
        frustum to the viewport and emits signalFrustumChanged when that
        changes anything -- so this is called only where a camera is
        really needed. Anything that just wants pixels uses _Ratio().
        """
        view = self._view
        if view is None:
            return None, None, 1.0
        try:
            camera, _ = view.resolveCamera()
            viewport = view.computeWindowViewport()
            ratio = self._Ratio()
        except Exception:
            return None, None, 1.0
        if camera is None:
            return None, None, 1.0
        return camera, viewport, ratio

    def _Ratio(self):
        """The stage view's device pixel ratio; 1.0 without a view."""
        try:
            return float(self._view.devicePixelRatioF())
        except (AttributeError, TypeError):
            return 1.0

    def HandleScreenPositions(self):
        """
        {handle name: [(x, y), ...]} in LOGICAL pixels, which is what Qt
        mouse events use -- a test can hand one of these straight to
        QTest.mousePress.

        Rings report their FRONT points only (the back half is neither
        drawn nor pickable), planes their four corners, and the centre
        and the free-rotate ball a single point.
        """
        ratio = self._Ratio()
        positions = {}
        for handle in self._handles:
            if handle.kind in ("ring", "view"):
                points = [p for run in handle.frontPoints for p in run]
            else:
                points = list(handle.points)
            positions[handle.name] = [(p[0] / ratio, p[1] / ratio)
                                      for p in points]
        return positions

    def PieSlice(self):
        """
        (polygon in physical pixels, rgb) for the rotation-amount wedge,
        or None. Drawn from the PRESS-TIME handle, which is the ring the
        cursor is actually on -- see _Drag.
        """
        drag = self._drag
        if drag is None or drag.tool != TOOL_ROTATE:
            return None
        if drag.handle.kind not in ("ring", "view"):
            return None
        if abs(drag.angle) < 1e-6:
            return None
        polygon = gizmoScreen.PiePolygon(drag.handle, drag.startParameter,
                                         drag.angle)
        return polygon, drag.handle.color

    def SnapCandidate(self):
        """
        The live snap candidate, or None (spec 4.5).

        The drag's while dragging, the hover preview otherwise. Never
        published through Handles(): the Select tool asserts
        HandleScreenPositions() == {}.
        """
        if self._drag is not None:
            return self._drag.snap
        return self._hoverSnap

    def SnapMarker(self):
        """
        (center, extras, leader) for the overlay's snap marker, in
        LOGICAL pixels, or None when there is no candidate to draw.

        Re-projected from the stored WORLD point on every paint, not
        from the pixel captured at pick time: a tumble or dolly with
        no mouse move must carry the marker with the geometry, and
        the pick-time pixel is stale the moment the camera moves.
        """
        candidate = self.SnapCandidate()
        if candidate is None:
            return None
        ratio = self._Ratio()
        center = self._ProjectLogical(candidate.point, ratio)
        if center is None:
            return None
        screen = (center.x() * ratio, center.y() * ratio)
        return (center,
                self._SnapMarkerExtras(candidate, center, ratio),
                self._SnapMarkerLeader(candidate, ratio, screen))

    def _SnapMarkerExtras(self, candidate, center, ratio):
        """The kind-specific lines of the snap marker (spec 4.5)."""
        kind = getattr(candidate, "kind", None)
        if kind == gizmoSettings.SNAP_SURFACE:
            normal = Gf.Vec3d(candidate.normal)
            if normal.GetLength() < 1e-12:
                return []
            end = self._ProjectLength(candidate.point, normal,
                                      18.0, ratio)
            return [(center, end)] if end is not None else []
        if kind == gizmoSettings.SNAP_EDGE:
            lines = []
            for world in getattr(candidate, "edgeEnds", None) or ():
                end = self._ProjectLogical(world, ratio)
                if end is not None:
                    lines.append((center, end))
            return lines
        if kind == gizmoSettings.SNAP_GRID:
            lines = []
            for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0),
                         Gf.Vec3d(0, 0, 1)):
                end = self._ProjectLength(candidate.point, axis,
                                          16.0, ratio)
                if end is not None:
                    lines.append((center, end))
            return lines
        return []

    def _SnapMarkerLeader(self, candidate, ratio, screen):
        """
        The grabbed (or hovered) handle's centre, or None when there
        is none or it already coincides with the candidate: the faint
        line between the two while they differ (spec 4.5).

        `screen` is the candidate's freshly re-projected PHYSICAL
        pixel from SnapMarker, not the pick-time one it carries.
        """
        start = None
        if self._drag is not None and self._drag.handle is not None:
            # The press-time handle's centre is frozen (gizmoDrag.py:
            # 63-68) while _RebuildHandles reprojects the live pivot
            # every Translate move, so read the LIVE handle -- the
            # same lookup the hover branch below already does.
            live = self._Handle(self._drag.handle.name) \
                or self._drag.handle
            start = getattr(live, "center", None)
        elif self._hover is not None:
            handle = self._Handle(self._hover)
            if handle is not None:
                start = handle.center
        if start is None:
            return None
        if math.hypot(screen[0] - start[0],
                       screen[1] - start[1]) < 2.0 * ratio:
            return None
        return QtCore.QPointF(start[0] / ratio, start[1] / ratio)

    def _ProjectLogical(self, worldPoint, ratio, camera=None,
                          viewport=None):
        """A world point as a logical QPointF, or None."""
        if camera is None or viewport is None:
            camera, viewport, _ = self._Camera()
            if camera is None:
                return None
        try:
            screen = gizmoScreen.ProjectPoint(
                gizmoScreen.ViewProjection(camera), viewport,
                worldPoint)
        except Exception:
            return None
        if screen is None:
            return None
        return QtCore.QPointF(screen[0] / ratio, screen[1] / ratio)

    def _ProjectLength(self, worldPoint, direction, pixels, ratio):
        """
        worldPoint pushed along direction by `pixels` LOGICAL screen
        pixels, as a logical QPointF, or None. What sizes the surface
        normal tick and the grid axis ticks in screen space.
        """
        camera, viewport, _ = self._Camera()
        if camera is None:
            return None
        try:
            perPixel = gizmoScreen.WorldPerPixel(
                camera, viewport, worldPoint)
        except Exception:
            return None
        if perPixel is None:
            return None
        unit = Gf.Vec3d(direction)
        if unit.GetLength() < 1e-12:
            return None
        # WorldPerPixel is per PHYSICAL pixel
        # (gizmoScreen.py:189-200: computeWindowViewport is
        # physical, stageView.py:1586-1587), so the logical-pixel
        # constant crosses the device ratio on the way in -- the
        # file's own convention (gizmoSnap.SNAP_PIXELS * ratio,
        # 2.0 * ratio).
        end = Gf.Vec3d(worldPoint) + \
            unit.GetNormalized() * (perPixel * pixels * ratio)
        return self._ProjectLogical(end, ratio, camera, viewport)

    def _ActiveSnap(self):
        """
        (mode, reason) in force with the live holds: ActiveSnapMode
        over the current tool, so a mid-drag V outranks the sticky
        mode and a Rotate V reports itself Move-only instead of
        snapping (gizmoDrag.py:170-171, always a tuple).
        """
        settings = self.settings.For(self._tool)
        return gizmoDrag.ActiveSnapMode(
            settings, self._tool, self._holdGrid, self._holdPoint,
            self._holdEdge)

    def SnapMode(self):
        """The snap mode actually in force -- a bare SNAP_* string."""
        return self._ActiveSnap()[0]

    def SnapReason(self):
        """Why a requested mode was downgraded, or the empty string."""
        return self._ActiveSnap()[1]

    def _SnapClause(self):
        """
        "  snap: ..." / "  grid: ...", or "" when no snap is active
        or armed (spec 4.5). Appended, never substituted: the rotate
        branch must keep "deg", the tool branch "outranked" and
        "unavailable". The rotate grid clause is degrees on the
        dragged gimbal channel ("grid: 15 deg"), never the Move
        world-grid wording.
        """
        drag = self._drag
        if drag is not None:
            mode, reason = self._gesture.snapMode, self._gesture.snapReason
            detail = getattr(drag, "snapReason", "") or ""
            candidate = drag.snap
            note = self._lastSnapNote
        else:
            mode, reason = self._ActiveSnap()
            detail = ""
            candidate = self._hoverSnap
            note = self._hoverNote
        if mode == gizmoSettings.SNAP_OFF:
            reason = detail or reason
            return "  snap: %s" % reason if reason else ""
        if mode == gizmoSettings.SNAP_GRID:
            if self._tool == TOOL_ROTATE:
                # Rotate's grid is an ABSOLUTE degree grid on the
                # dragged gimbal channel and ignores gridSize (spec
                # 1.8, 4.3), so neither Move wording is true here.
                # `detail` on this tool is only ever _Rotate's
                # downgrade reason, which spec 4.3 words with the
                # "snap: " prefix.
                if detail:
                    return "  snap: %s" % detail
                if drag is None and self._gimbalAxes is None:
                    return "  snap: Grid needs a Gimbal ring"
                return "  grid: %g deg" % (
                    self.settings.For(TOOL_ROTATE).stepSize,)
            return "  %s" % (detail or "grid: world")
        if candidate is not None and \
                getattr(candidate, "kind", None) == mode:
            return "  snap: %s" % gizmoSettings.SnapLabel(mode)
        if note:
            return "  snap: %s" % note
        return "  snap: no target"

    # -- status ---------------------------------------------------------

    def Status(self):
        if not self._visible:
            return "Viewport tools hidden"
        if self._tool == TOOL_SELECT:
            return ("Select: usdview picking. Choose Move, Rotate or "
                    "Scale for a manipulator.")
        target = self._target
        if target is None:
            return self._reason
        verb = gizmoModel.EditVerb(self._tool)
        if not self._ToolSupported(target):
            return "%s is unavailable for %s (%s)" % (
                verb, target.label, target.kind)
        if self._drag is not None and self._drag.tool == TOOL_ROTATE:
            return "%s %s  %.1f deg%s" % (verb, target.label,
                                          self._drag.angle,
                                          self._SnapClause())
        # The target's writer decides: a path curve's unkeyed points take
        # their default whatever the toolbar says.
        writeMode = getattr(getattr(target, "writer", None), "mode",
                            self._writeMode)
        text = "%s %s%s  [%s / %s]%s" % (
            verb, target.label, self._GroupPivotClause(target),
            "Pivot" if self._channels == gizmoMath.CHANNELS_PIVOT
            else "Pose",
            "Default" if writeMode == gizmoMath.WRITE_DEFAULT
            else "Animation",
            self._SnapClause())
        skipped = self.SkippedChildren()
        if skipped:
            text += "  (%d child%s not preserved)" % (
                len(skipped), "" if len(skipped) == 1 else "ren")
        # Post-drag warnings from the Writer, plus the target's own
        # standing advisory about a drag that authors correctly and
        # moves nothing (a pivot on a joint whose solver has authored
        # bone lengths). One "warning:" prefix carries both.
        notes = list(self._warnings)
        advisory = target.Advisory()
        if advisory:
            notes.append(advisory)
        if notes:
            text += "  warning: " + "; ".join(notes)
        return text

    def _GroupPivotClause(self, target):
        """
        " about Last Selected" for a group turning about anything but
        its centre, and "" otherwise.

        Silent on the default deliberately: the centre is what a
        selection does unless it was told not to, and a status line that
        narrates the default trains people to stop reading it. A
        NON-default is exactly the thing an artist forgets they set.
        """
        mode = getattr(target, "pivotMode", None)
        if mode is None or mode == gizmoMath.GROUP_PIVOT_CENTER:
            return ""
        if not gizmoSettings.GroupPivotChoices(self._tool):
            return ""
        return " about %s" % gizmoSettings.GroupPivotLabel(mode)

    def SkippedChildren(self):
        """
        The target's explanations for children a Preserve Children drag
        will NOT hold still (a rig-placed child, an incompatible op
        stack, a non-zero pivot, a mirrored transform). Empty unless the
        option is on.
        """
        target = self._target
        if target is None:
            return []
        return list(getattr(target, "skippedChildren", []))

    # -- undo -----------------------------------------------------------

    def Undo(self):
        self.FlushReleasedEdit()
        # Undo is ignored while a manipulator is held. Running it here
        # would restore an earlier edit that the drag's next event then
        # overwrites from its own base, and the release would push over
        # the redo branch -- the earlier edit lost from history with its
        # effect half applied.
        if self._drag is not None or not self.undoStack.Undo():
            return False
        self._AfterUndoRedo()
        return True

    def Redo(self):
        self.FlushReleasedEdit()
        if self._drag is not None or not self.undoStack.Redo():
            return False
        self._AfterUndoRedo()
        return True

    def FollowExternalPreview(self, primPath=None):
        """
        Redraw the handles against values another panel is previewing.

        The Avar Editor previews a slider drag through the same Hydra
        channel a drag here uses, and puts the same values in
        gizmoMath's preview map, but nothing on the stage changes, so the
        stage-notice refresh never fires. The panel calls this per sample
        instead. The cheap per-sample refresh is used for the target the
        slider is moving (its parent frames cannot change); anything else
        re-reads in full. Ignored while a manipulator drag of our own is
        in flight, which already refreshes on every mouse move.
        """
        self.FlushReleasedEdit()
        if self._drag is not None:
            return
        if self._target is None:
            # Nothing to redraw here, but the viewport still has to repaint:
            # a dial on a control with no manipulator (a limb's params node)
            # moves the rig all the same.
            self.usdviewApi.UpdateViewport()
            return
        own = getattr(self._target, "prim", None)
        cheap = (primPath is not None and own is not None
                 and own.GetPath() == primPath)
        try:
            if cheap:
                self._target.RefreshDuringDrag()
            else:
                self._target.Refresh()
        except Exception:
            try:
                self._target.Refresh()
            except Exception:
                self.RefreshTarget()
        self._RebuildHandles()
        self.usdviewApi.UpdateViewport()

    def _AfterUndoRedo(self):
        # Restoring layer specs does not schedule a repaint by itself:
        # the evaluator republishes on the ObjectsChanged notice, but
        # nothing tells the viewport to draw the new frame.
        if self._target is not None:
            try:
                self._target.Refresh()
            except Exception:
                self.RefreshTarget()
        self._RebuildHandles()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    # -- settings window ------------------------------------------------

    def StatusBar(self):
        """The status row widget under the tool buttons."""
        return self.statusBar

    def ShowToolSettings(self):
        if self._panel is None:
            self._panel = ToolSettingsPanel.GetInstance(self)
        # Sync() rebuilds by itself when the tool has changed; rebuilding
        # unconditionally would throw away and recreate every row widget
        # on each click of the button.
        self._panel.Sync()
        self._panel.show()
        self._panel.raise_()
        self._panel.activateWindow()
        return self._panel

    def OpenGraphEditor(self):
        """
        The Graph… button: the animation curves of whatever is selected,
        editable on THIS controller's undo stack.

        The container hands its own opener in at install time so it owns
        the panel; the lazy import is the fallback for an install that
        did not, and keeps this module importable without Qt's graph
        editor present.
        """
        self.FlushReleasedEdit()
        if self._openGraphEditor is not None:
            return self._openGraphEditor()
        try:
            try:
                import graphEditorUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import graphEditorUI
            return graphEditorUI.OpenGraphEditor(self.usdviewApi,
                                                 self.undoStack)
        except Exception as error:
            Tf.Warn("rigExecUsdview: graph editor unavailable: %s" % error)
            return None

    # -- signals --------------------------------------------------------

    def _onSettingsChanged(self):
        self.FlushReleasedEdit()
        self._PrimePreserveChildren()
        self._PrimeGroupPivot()
        self._RebuildHandles()
        # Re-pick, never redraw stale: switching the Snap: dropdown
        # without moving the cursor must move the marker (spec 4.5).
        self._RefreshHoverSnap()
        # The button text and the status clause both read the sticky
        # mode, so a dropdown or panel write that leaves them showing
        # the old one lies until the next hover move. _RefreshHoverSnap
        # above only syncs when the candidate changed.
        self.toolbar.Sync()
        if self._panel is not None:
            self._panel.Sync()

    def _onUndoStackChanged(self):
        if not self._viewDestroyed:
            self.toolbar.Sync()

    def _onFrustumChanged(self):
        self._RebuildHandles()

    def _onSelectionChanged(self, added=None, removed=None):
        # _selected survives: the active handle is kept across a
        # selection change, so middle-drag still repeats it on the prim
        # you just picked. The hover is stale by definition.
        self._AbortDrag()
        self._hover = None
        # Selecting a control of the half a limb is not driving with
        # switches the limb to it, matched; _SwitchPresentedLimbs
        # refreshes nothing itself, so the one refresh below covers both.
        self._SwitchPresentedLimbs()
        self.RefreshTarget()

    def _onFrameChanged(self, frame):
        self.FlushReleasedEdit()
        # The SIGNAL's frame, never dataModel.currentFrame: the setter
        # emits before it assigns, so re-reading the property here would
        # author one frame behind (rigExecUsdview._FrameValue).
        self._frame = frame if isinstance(frame, Usd.TimeCode) \
            else Usd.TimeCode(float(frame))
        # Points are cached per frame: a scrub moves every one.
        self._snapGeom.clear()
        self._AbortDrag()
        self.RefreshTarget()

    def _onStageReplaced(self):
        self._AbortDrag()
        self.undoStack.Clear()
        self._solverPosed.Clear()
        # New stage, new paths: both snap caches die (the hover one
        # would trip on stage identity anyway).
        self._snapGeom.clear()
        self._hoverRigStage = None
        self._frame = self.usdviewApi.frame
        self._ObserveStage(self.usdviewApi.dataModel.stage)
        self.RefreshTarget()

    def _onObjectsChanged(self, notice, stage):
        """
        Refresh the target, but only for a notice that can have moved it.

        Notices arrive from the volume weight panel, a timeline scrub,
        and anything else authoring in the session, not
        just from the gizmo. Refreshing unconditionally bought each of
        them a full rig walk plus a resolveCamera() -- which conforms
        the frustum and can emit signalFrustumChanged, see _Camera --
        plus a reprojection of the whole manipulator, so the filter has
        to come BEFORE any of that, not inside it.
        """
        if self._viewDestroyed:
            return
        resynced = notice.GetResyncedPaths()
        changed = notice.GetChangedInfoOnlyPaths()
        target = self._target
        ownCommit = target is not None and self._IsOwnCommit(
            target, resynced, changed, notice)
        # The hover preview's rig-written sets are only as fresh as
        # the last authoring notice: editor panels author rigExec:moves
        # and rigExec:weightTarget in this session, and a stale set makes
        # the marker offer a vertex
        # the drag (which rebuilds its sets in _BeginDrag) then
        # refuses as rig-deformed. Likewise the snap geometry cache
        # may only keep prims no notice path touches. Both go ABOVE
        # the in-drag guard: a notice arriving mid-drag must not be
        # dropped, and the drag path is unaffected (it reads
        # drag.rigWritten, and its own cache entries survive unless
        # the notice names their prim).
        self._hoverRigStage = None
        if self._snapGeom:
            self._DropSnapGeom(notice.GetResyncedPaths(),
                               notice.GetChangedInfoOnlyPaths())
        if ownCommit:
            return
        # Inside a drag the target is already being refreshed by the
        # drag itself, and re-resolving it here would throw away the
        # base values every Apply* is computed from.
        if self._drag is not None:
            return
        # Both kinds: a new or cleared joints relationship resyncs, but
        # RE-targeting an existing one is info-only (SolverPosedCache).
        if resynced:
            self._solverPosed.InvalidateResynced(resynced)
        if changed:
            self._solverPosed.InvalidateChanged(changed)
        # A missing target may be exactly what this notice creates, and a
        # SELECTION that no longer matches needs the full re-resolve --
        # compared as the whole set, not just the focus prim, because a
        # group can gain or lose a member with the lead unchanged.
        if target is None or self._SelectionSignature() != self._targetPaths:
            self.RefreshTarget()
            return
        # Any member answers for the group: they share a rig root, and a
        # notice that moved one of them moved the pivot.
        if not any(gizmoMath.NoticeAffectsTarget(
                resynced, changed, path, target.RigRootPath())
                for path in self._TargetPrimPaths(target)):
            return
        try:
            target.Refresh()
        except Exception:
            self.RefreshTarget()
            return
        self._RebuildHandles()

    def _IsOwnCommit(self, target, resynced, changed, notice=None):
        return self._gesture.IsOwnCommit(target, resynced, changed, notice)

    # -- event filter ---------------------------------------------------

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind in (QtCore.QEvent.Resize, QtCore.QEvent.Show):
            self._SyncOverlay()
            self._RebuildHandles()
            return False
        if kind == QtCore.QEvent.Paint:
            # Geometry only. Storm repaints on a 5 ms timer until the
            # render converges, so rebuilding the handles here would
            # reproject the whole manipulator two hundred times a
            # second for nothing; every reason they could be stale --
            # the camera (signalFrustumChanged, which the view's own
            # paint emits), the viewport (Resize), the values
            # (ObjectsChanged), the frame, the selection, the settings
            # -- has its own hook.
            self._SyncOverlay(restack=False)
            return False
        # Key events are NOT handled here: usdview never lets the stage
        # view hold focus, so they arrive through the application-level
        # ViewportHotkeyFilter instead.
        if kind == QtCore.QEvent.HoverMove:
            self._UpdateHover(self._Position(event))
            return False
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._UpdateHover(None)
            return False
        if kind == QtCore.QEvent.MouseButtonPress:
            # A click in the viewport ends a text edit -- see
            # _DropTypingFocus. Before _OnPress and regardless of what
            # it returns: a plain selection click is the commonest way
            # into this and it claims nothing.
            self._DropTypingFocus()
            return self._OnPress(event)
        if kind == QtCore.QEvent.MouseMove:
            return self._OnMove(event)
        if kind == QtCore.QEvent.MouseButtonRelease:
            return self._OnRelease(event)
        return False

    def _SyncOverlay(self, restack=True):
        """
        Keep the overlay exactly over the stage view.

        Restacking is skipped on a plain repaint: raise_() schedules
        work of its own, and doing it inside the view's paint would put
        the widget in a loop with itself.
        """
        overlay = self.overlay
        view = self._view
        if overlay is None or view is None:
            return
        if overlay.geometry() != view.rect():
            overlay.setGeometry(view.rect())
        overlay.setVisible(self._visible)
        if restack:
            overlay.raise_()

    def _Repaint(self):
        if self.overlay is not None:
            self.overlay.update()

    def _Position(self, event):
        """
        The cursor in the PHYSICAL pixels gizmoScreen works in.

        Qt reports widget-local LOGICAL pixels and computeWindowViewport
        is physical; usdview's own mousePressEvent bridges the two with
        devicePixelRatioF and anything picking by hand has to as well.
        Skipping it is invisible on a 1.0-ratio display and puts every
        pick at half the cursor's position on a HiDPI one.
        """
        try:
            position = event.position()
            x, y = position.x(), position.y()
        except AttributeError:
            x, y = event.x(), event.y()
        ratio = self._Ratio()
        return (x * ratio, y * ratio)

    def _HitTest(self, point):
        ratio = self._Ratio()
        return gizmoScreen.HitTest(self._handles, point[0], point[1],
                                   HIT_PIXELS * ratio)

    def _Handle(self, name):
        for handle in self._handles:
            if handle.name == name:
                return handle
        return None

    def _OnPress(self, event):
        if event.modifiers() & _CAMERA_MODIFIERS:
            return False              # usdview's camera drags win
        if not self._visible or self._tool == TOOL_SELECT or \
                not self._handles:
            # No handles to aim at -- Select tool, hidden gizmo, or
            # nothing selected. A press here is a marquee in waiting.
            return self._BeginMarquee(event)
        point = self._Position(event)
        self._ctrl = bool(event.modifiers() & QtCore.Qt.ControlModifier)
        if event.button() == QtCore.Qt.LeftButton:
            handle = self._HitTest(point)
            if handle is None:
                # THE GIZMO WINS, and it did not want this pixel: off the
                # handles, the press starts a marquee (and still reaches
                # usdview, which picks the prim under it).
                return self._BeginMarquee(event)
            if not self._BeginDrag(handle, point):
                return False
            self._selected = handle.name
            self._Repaint()
            return True
        if event.button() == QtCore.Qt.MiddleButton:
            # "Middle-drag anywhere repeats the selected handle":
            # the artist does not have to hit the handle again.
            handle = self._Handle(self._selected)
            if handle is None or not handle.grabbable:
                return False
            return self._BeginDrag(handle, point)
        return False

    def _OnMove(self, event):
        if self._drag is not None:
            self._drag.ctrl = bool(
                event.modifiers() & QtCore.Qt.ControlModifier)
            self._UpdateDrag(self._Position(event))
            return True
        if self._gesture.marqueeAt is not None:
            return self._UpdateMarquee(event)
        if not self._visible or self._tool == TOOL_SELECT:
            return False
        if event.buttons():
            return False              # a camera or picking drag
        self._UpdateHover(self._Position(event))
        return False

    def _UpdateHover(self, point):
        """The pre-selection highlight; `point` None clears it."""
        self._hoverPoint = point
        if point is None or not self._visible or self._tool == TOOL_SELECT:
            name = None
        else:
            handle = self._HitTest(point)
            name = handle.name if handle is not None else None
        if name != self._hover:
            self._hover = name
            self._Repaint()
        self._RefreshHoverSnap()

    def _HoverSnapKey(self):
        """What the hover preview shows, for change detection."""
        candidate = self._hoverSnap
        if candidate is None:
            return (None, self._hoverNote)
        return (getattr(candidate, "kind", None),
                str(candidate.point), str(candidate.primPath),
                candidate.index, self._hoverNote)

    def _RefreshHoverSnap(self):
        """
        The armed-mode hover preview (spec 4.5): "hold V and see what
        lights up". The same mode-keyed throttle as the drag path, so
        switching the Snap: dropdown without moving the cursor
        re-picks instead of redrawing a stale marker. toolbar.Sync()
        only when the candidate actually changed.
        """
        if self._drag is not None:
            return
        mode = self.SnapMode()
        if mode in (gizmoSettings.SNAP_OFF, gizmoSettings.SNAP_GRID) \
                or self._hoverPoint is None or self._target is None:
            if self._hoverSnap is not None or self._hoverNote:
                self._hoverSnap = None
                self._hoverSnapKind = None
                self._hoverPickPoint = None
                self._hoverNote = ""
                self._Repaint()
                self.toolbar.Sync()
            return
        point = self._hoverPoint
        if self._hoverSnapKind == mode \
                and self._hoverPickPoint is not None:
            travel = math.hypot(
                point[0] - self._hoverPickPoint[0],
                point[1] - self._hoverPickPoint[1])
            if travel <= gizmoSnap.PICK_MOVE_PIXELS:
                return
        before = self._HoverSnapKey()
        self._hoverNote = ""
        try:
            candidate = self._ResolveSnap(point[0], point[1], mode)
        except Exception:
            candidate = None
        self._hoverSnap = candidate
        self._hoverSnapKind = mode
        self._hoverPickPoint = (point[0], point[1])
        if self._HoverSnapKey() != before:
            self._Repaint()
            self.toolbar.Sync()

    def _OnRelease(self, event):
        if self._drag is None:
            if self._gesture.marqueeAt is not None \
                    and event.button() == QtCore.Qt.LeftButton:
                self._EndMarquee()
            return False
        if event.button() not in (QtCore.Qt.LeftButton,
                                  QtCore.Qt.MiddleButton):
            return False
        self._EndDrag()
        return True

    # -- marquee (box) selection ----------------------------------------

    def _TouchPoseActive(self):
        """True while THIS session's TouchPose is running its own marquee.

        TouchPose selects touch REGIONS with the identical gesture and
        the identical three modes, and both filters sit on the same stage
        view, so without this an animator with TouchPose on would get two
        selections applied to one drag -- and which one landed last would
        depend on the order the panels happened to be opened in. The
        native marquee stands down for as long as TouchPose is on; it is
        the one that knows about the skin.

        STANDING DOWN IS NOT DROPPING OUT. It used to be: while TouchPose
        was on, a box round the IK controls selected nothing, because
        regions only cover the mesh and nothing else was looking. So
        TouchPose now folds `gizmoMarquee.ControlsInBand` into its own
        pick (touchPoseUI.TouchPoseController._ControlsInBand) and the
        band catches controls either way. Box selection is native to
        RigExec; which subsystem happens to own the drag is not something
        the animator should be able to feel.

        Never constructs a controller (that would load the mesh), and any
        failure -- no touchPose on sys.path at all -- answers False,
        which is the session this feature exists for.
        """
        try:
            import touchPoseUI
        except Exception:
            return False
        try:
            controller = touchPoseUI.TouchPoseController.ForApi(
                self.usdviewApi)
            return controller is not None and bool(controller.active)
        except Exception:
            return False

    def _SelectedPaths(self):
        """The selected prim paths, pseudo-root dropped, lead last.

        THE PSEUDO-ROOT IS NOT A SELECTION: `clearPrims()` leaves `/`
        behind, so the current set always looks non-empty and a toggle
        would carry `/` forward for ever.
        """
        try:
            prims = self.usdviewApi.dataModel.selection.getPrims()
        except Exception:
            return []
        return [str(p.GetPath()) for p in prims
                if p and p.IsValid() and not p.IsPseudoRoot()]

    def _BeginMarquee(self, event):
        """Arm a marquee on a left press. Never consumes the press.

        Requirement 4 of the feature: a click must keep behaving exactly
        as it does today, and usdview picks on the PRESS, so swallowing
        it would kill single-prim picking outright. The press is recorded
        and let through; only a drag past the slop turns into a band.
        """
        if event.button() != QtCore.Qt.LeftButton:
            return False
        if self._TouchPoseActive():
            return False
        self._gesture.BeginMarquee(
            self._Position(event), self._SelectedPaths(),
            ctrl=bool(event.modifiers() & QtCore.Qt.ControlModifier),
            shift=bool(event.modifiers() & QtCore.Qt.ShiftModifier))
        return False

    def _UpdateMarquee(self, event):
        """Grow the band; consume the move only once there IS a band.

        Below the slop nothing is consumed, so a plain click's event
        stream is byte-for-byte what it was before this existed. Above
        it the move is swallowed, which keeps stageView.mouseMoveEvent
        from calling updateGL for every pixel of a drag that changes
        nothing on screen but the rubber band.
        """
        if not (event.buttons() & QtCore.Qt.LeftButton):
            # The button came up somewhere this filter never saw -- the
            # cursor left the window mid-drag, or another widget grabbed
            # it. Drop the armed press rather than leave it to fire on
            # some unrelated later move.
            self._gesture.ClearMarquee()
            self._HideBand()
            return False
        if not self._gesture.UpdateMarquee(self._Position(event), self._Ratio()):
            return False
        self._ShowBand()
        return True

    def _EndMarquee(self):
        """Apply the band, if the press ever became one."""
        band, mode, was = self._gesture.EndMarquee()
        self._HideBand()
        if band is None:
            return False          # a click: usdview already answered it
        self.Marquee(band[0], band[1], band[2], band[3], mode, was)
        return True

    def Marquee(self, x0, y0, x1, y1, mode=None, current=None):
        """Select every control the band catches. Physical pixels.

        Public because the in-app test drives it directly: a synthetic
        QTest drag cannot be trusted to reproduce the exact pixel path
        that decides what a band contains.
        """
        mode = gizmoMarquee.MODE_REPLACE if mode is None else mode
        model = getattr(self.usdviewApi, "dataModel", None)
        stage = getattr(model, "stage", None) if model is not None else None
        if stage is None:
            return []
        camera, viewport, _ratio = self._Camera()
        wanted = gizmoMarquee.ControlsInBand(
            stage, camera, viewport, self.usdviewApi.frame,
            (x0, y0, x1, y1), self._solverPosed)
        if current is None:
            current = self._SelectedPaths()
        keep = gizmoMarquee.Resolve(list(current), wanted, mode)
        self._ApplySelection(stage, keep)
        return keep

    def _ApplySelection(self, stage, paths):
        """Write an ordered path list into usdview's selection.

        batchPrimChanges when the build has it, so a band over forty
        controls emits one selection change rather than forty -- each of
        which would otherwise rebuild the gizmo's target.
        """
        selection = self.usdviewApi.dataModel.selection
        prims = []
        for path in paths:
            prim = stage.GetPrimAtPath(Sdf.Path(path))
            if prim and prim.IsValid():
                prims.append(prim)
        batch = getattr(selection, "batchPrimChanges", None)
        if batch is None:
            selection.clearPrims()
            for prim in prims:
                selection.addPrim(prim)
            return
        with batch:
            selection.clearPrims()
            for prim in prims:
                selection.addPrim(prim)

    def _ShowBand(self):
        """Draw the band with Qt's own rubber band over the stage view.

        A QRubberBand and not the GizmoOverlay: the overlay is hidden
        whenever the gizmo is hidden (SetVisible), and the marquee has to
        draw in exactly that state.
        """
        view = self._view
        if view is None or self._gesture.marqueeBand is None:
            return
        if self._rubber is None:
            self._rubber = QtWidgets.QRubberBand(
                QtWidgets.QRubberBand.Rectangle, view)
        ratio = self._Ratio()
        x0, y0, x1, y1 = [v / ratio for v in self._gesture.marqueeBand]
        self._rubber.setGeometry(QtCore.QRect(
            QtCore.QPoint(int(min(x0, x1)), int(min(y0, y1))),
            QtCore.QPoint(int(max(x0, x1)), int(max(y0, y1)))))
        self._rubber.show()
        self._rubber.raise_()

    def _HideBand(self):
        if self._rubber is not None:
            self._rubber.hide()

    # -- keys -----------------------------------------------------------

    # The keys a live drag owns outright, wherever the cursor is.
    _DRAG_KEYS = (QtCore.Qt.Key_Escape, QtCore.Qt.Key_J, QtCore.Qt.Key_X,
                  QtCore.Qt.Key_C, QtCore.Qt.Key_V, QtCore.Qt.Key_B)

    # Everything else, which needs the cursor over the viewport.
    _TOOL_KEYS = {QtCore.Qt.Key_Q: TOOL_SELECT,
                  QtCore.Qt.Key_W: TOOL_TRANSLATE,
                  QtCore.Qt.Key_E: TOOL_ROTATE,
                  QtCore.Qt.Key_R: TOOL_SCALE}
    _SIZE_KEYS = (QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal,
                  QtCore.Qt.Key_Minus)
    _PIVOT_KEYS = (QtCore.Qt.Key_D, QtCore.Qt.Key_Insert)

    def _MainWindow(self):
        """usdview's main window, or None (a headless probe has none)."""
        try:
            return self.usdviewApi.qMainWindow
        except Exception:
            return None

    def _InMainWindow(self, receiver):
        """Whether the event belongs to usdview's own main window."""
        main = self._MainWindow()
        if main is None:
            return False
        if isinstance(receiver, QtWidgets.QWidget):
            return receiver.window() is main
        # A QWindow or another non-widget receiver: a real key press
        # implies the window is active anyway.
        return QtWidgets.QApplication.activeWindow() is main

    def OwnsWindowEvent(self, receiver):
        """
        Whether an event delivered to `receiver` belongs to THIS session:
        its main window, or a window (a panel) that main window owns.

        The application-wide hotkey filter asks this before it does
        anything, so one usdview window's keys and focus changes never
        drive another session's gizmo.
        """
        return sessionRegistry.SessionOfEvent(receiver, self._MainWindow())

    @staticmethod
    def _TypingFocus():
        focus = QtWidgets.QApplication.focusWidget()
        if focus is None:
            return False
        if isinstance(focus, _TEXT_WIDGETS):
            return True
        return (isinstance(focus, QtWidgets.QComboBox)
                and focus.isEditable())

    def _DropTypingFocus(self):
        """End a text edit so the viewport can have the keyboard back.

        USDVIEW WILL NOT DO THIS. appEventFilter.JealousFocus lists
        QLineEdit, QComboBox, QTextEdit, QPlainTextEdit, QAbstractSlider
        and QAbstractSpinBox, and SetFocusFromMousePos runs for Escape
        alone -- "Don't touch if there's a greedy focus widget". So once
        the caret is in the path field under the menu bar it stays
        there through every mouse move and every viewport click, and
        Q / W / E / R are typed into the field instead of changing the
        tool. Reported from a real session, and the report was exact:
        "this usually is happening after I select something in the
        viewport".

        A viewport click ends a field edit. This
        does the same, and clears rather than re-homing the focus:
        usdview's stage view reports NoFocus, so there is nothing to
        give it to, and a null focus widget is precisely what makes
        _TypingFocus false for the keys that follow.
        """
        focus = QtWidgets.QApplication.focusWidget()
        if focus is None or not self._TypingFocus():
            return False
        focus.clearFocus()
        return True

    def _ToolKeyOverTheView(self, receiver, kind, event):
        """Q / W / E / R with the cursor over the viewport, and it wins.

        The viewport owns the tool keys whenever the cursor is over it,
        even with a text field focused: usdview hands the path field
        under the menu bar a focus it will not give up (see
        _DropTypingFocus), so gating on the focus widget alone left the
        tools dead after a viewport selection. The field loses the
        caret on the way through, which is the same thing the click
        would have done.

        Only these four, and only unmodified: the drag, size and pivot
        keys stay behind the typing gate, so a stray keystroke while a
        field is focused can at worst change the active tool -- visible,
        and undone by pressing the one the artist wanted.
        """
        if kind == QtCore.QEvent.KeyRelease:
            return False
        if event.key() not in self._TOOL_KEYS:
            return False
        if event.modifiers() & (QtCore.Qt.ControlModifier
                                | QtCore.Qt.AltModifier
                                | QtCore.Qt.MetaModifier):
            return False
        if not (self._CursorOverView() or receiver is self._view):
            return False
        self._DropTypingFocus()
        return True

    def _CursorOverView(self):
        view = self._view
        if view is None:
            return False
        if view.underMouse():
            return True
        try:
            return view.rect().contains(
                view.mapFromGlobal(QtGui.QCursor.pos()))
        except Exception:
            return False

    def HandleHotkey(self, receiver, event, kind):
        """
        One key event from the application filter. True consumes it.

        The gates, in order: the viewport tools must be visible, the
        event must belong to usdview's main window, and the focus widget
        must not be a text field. A live drag then owns Escape / J / X /
        C / V wherever the cursor is; the tool, size, pivot and the
        outside-drag C / V keys need the cursor over the viewport, or
        the event delivered straight at the view (which is what QTest
        does and a real keyboard never does).
        """
        if not self._visible or self._view is None:
            return False
        if not self._InMainWindow(receiver):
            return False
        key = event.key()
        # Release the latch AND the holds BEFORE the typing gate: a
        # key pressed over the viewport and released after the focus
        # moved into a text field would otherwise stay latched, and
        # -- since _ToolKey arms C / V outside a drag, where no
        # _ClearHolds ever runs -- stay HELD, point-snapping the next
        # drag into a silent no-op. The claim still happens below the
        # gate: a KeyRelease delivered to a line edit is never ours
        # to consume.
        if kind == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
            if key == self._claimedKey:
                self._claimedKey = None
            handled = self._ReleaseHold(event, key)
            if self._TypingFocus():
                return False
            return self._Claim(event, handled)
        if (self._TypingFocus()
                and not self._ToolKeyOverTheView(receiver, kind, event)):
            return False
        if kind == QtCore.QEvent.KeyRelease:
            # Auto-repeat only: the block above took the real one,
            # and _ReleaseHold ignores repeats anyway.
            return False
        if self._drag is not None and key in self._DRAG_KEYS:
            return self._Claim(event, self._Act(kind, key, self._DragKey))
        if event.modifiers() & (QtCore.Qt.ControlModifier
                                | QtCore.Qt.AltModifier
                                | QtCore.Qt.MetaModifier):
            return False              # leave usdview's Ctrl+... alone
        if not (self._CursorOverView() or receiver is self._view):
            return False
        self._keyModifiers = event.modifiers()
        return self._Claim(event, self._Act(kind, key, self._ToolKey))

    @staticmethod
    def _Claim(event, handled):
        """
        Mark a key we acted on as accepted, not merely filtered.

        Returning True from an event filter stops the delivery, but Qt's
        shortcut map looks at the ACCEPTED flag on a ShortcutOverride to
        decide whether to run the matching shortcut. Without this, J
        both snapped the drag and fired usdview's Toggle Framed View,
        which moved the camera out from under the manipulator.
        """
        if handled:
            event.accept()
        return handled

    def _Act(self, kind, key, handler):
        """
        Run `handler` exactly ONCE per physical key press, whichever of
        the several deliveries Qt makes arrives first.

        One press reaches an application filter many times. Qt sends a
        ShortcutOverride and then a KeyPress, and it delivers each of
        them to the focus widget and then, while they stay unaccepted,
        to every ancestor up to the window -- five hops in usdview. It
        is also unknowable which of the two Qt will let through: a key
        matching any shortcut is swallowed by the shortcut map before
        the KeyPress, while QTest stops at an override it sees accepted.
        So the rule is: act on the first delivery, remember the key, and
        swallow every later one until its KeyRelease clears the latch.
        Acting only in the override loses a real keyboard, acting only
        in the KeyPress loses J and Escape, and acting on each delivery
        grew the manipulator by 10% five times per keypress.
        """
        del kind                      # every delivery is treated alike
        if key == self._claimedKey:
            return True               # already acted on this press
        if not handler(key):
            return False
        self._claimedKey = key
        return True

    def _DragKey(self, key):
        if key == QtCore.Qt.Key_Escape:
            self._AbortDrag()
            return True
        if key == QtCore.Qt.Key_J:
            if not self._holdSnap:
                self._holdSnap = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_X:
            if not self._holdGrid:
                self._holdGrid = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_C:
            if not self._holdEdge:
                self._holdEdge = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_V:
            if not self._holdPoint:
                self._holdPoint = True
                self._ReapplyDrag()
            return True
        if key == QtCore.Qt.Key_B:
            if not self._holdPreserve:
                self._holdPreserve = True
                # No _ReapplyDrag: arming mid-drag would leave the
                # children outside the undo record the recorder has
                # already begun (Target.AttributePaths says why). Hold B
                # BEFORE the drag; pressed during one it arms the next.
                self._PrimePreserveChildren()
            return True
        return False

    def _ToolKey(self, key):
        if key in self._TOOL_KEYS:
            self.SetTool(self._TOOL_KEYS[key])
            return True
        if key in self._SIZE_KEYS:
            self.settings.ScaleManipulator(
                1.0 / 1.1 if key == QtCore.Qt.Key_Minus else 1.1)
            return True
        if key == QtCore.Qt.Key_L:
            # Global <-> Local, the toolbar toggle's keyboard twin. A
            # bare letter like the tool keys, and free: usdview binds no
            # L, and it is not one of the drag holds (J / X / C / V / B).
            # The RESULT is returned, not True: under the Select tool
            # there is no orientation to set, and a key we did nothing
            # with must go back to usdview rather than be swallowed.
            return self.ToggleOrientation()
        if key == QtCore.Qt.Key_P:
            # The group pivot cycle, the toolbar button's keyboard twin,
            # on Shift+P: bare P toggles the hover picker. Returns its
            # result for the same reason L does -- Select offers no group
            # pivot, so the key stays usdview's there.
            if not (getattr(self, "_keyModifiers", QtCore.Qt.NoModifier)
                    & QtCore.Qt.ShiftModifier):
                return False
            return self.CycleGroupPivot()
        if key in self._PIVOT_KEYS:
            self.SetChannels(
                gizmoMath.CHANNELS_POSE
                if self._channels == gizmoMath.CHANNELS_PIVOT
                else gizmoMath.CHANNELS_PIVOT)
            return True
        if key in (QtCore.Qt.Key_C, QtCore.Qt.Key_V):
            # usdview binds both -- C is Auto Compute Clipping
            # Planes (mainWindowUI.py:1598, appController.py:833),
            # V is Show USD Validation (mainWindowUI.py:1481,
            # appController.py:824) -- so they follow the J sharing
            # rule, NOT the tool-key rule above, which carries no
            # tool condition and would swallow both keys under every
            # tool. Only the Move tool with a target arms them;
            # everywhere else the key stays usdview's (spec 8.5).
            want = gizmoSettings.SNAP_EDGE \
                if key == QtCore.Qt.Key_C \
                else gizmoSettings.SNAP_POINT
            if self._target is None or want not in \
                    gizmoSettings.SnapChoices(self._tool):
                return False
            if key == QtCore.Qt.Key_C:
                self._holdEdge = True
            else:
                self._holdPoint = True
            self._RefreshHoverSnap()
            self.toolbar.Sync()
            return True
        return False

    def _ReleaseHold(self, event, key):
        if event.isAutoRepeat():
            return False
        if key == QtCore.Qt.Key_J and self._holdSnap:
            self._holdSnap = False
            self._ReapplyDrag()
            return self._drag is not None
        if key == QtCore.Qt.Key_X and self._holdGrid:
            self._holdGrid = False
            self._ReapplyDrag()
            return self._drag is not None
        if key == QtCore.Qt.Key_C and self._holdEdge:
            self._holdEdge = False
            self._ReapplyDrag()
            self._RefreshHoverSnap()
            self.toolbar.Sync()
            return self._drag is not None
        if key == QtCore.Qt.Key_B and self._holdPreserve:
            self._holdPreserve = False
            self._PrimePreserveChildren()
            return True
        if key == QtCore.Qt.Key_V and self._holdPoint:
            self._holdPoint = False
            self._ReapplyDrag()
            self._RefreshHoverSnap()
            self.toolbar.Sync()
            return self._drag is not None
        return False

    def _ReapplyDrag(self):
        """Re-run the live drag so a held modifier takes effect at once."""
        if self._drag is not None:
            self._UpdateDrag(self._drag.current)

    # -- snap pick and resolver -----------------------------------------

    def _SnapPick(self, x, y):
        """
        (primPath, point, normal) under the cursor, guides excluded,
        or None.

        Bypasses view.pick(): the plugin forces displayGuide on, so
        pick() returns a RigExec guide for every hit. The fresh
        RenderParams carries the view settings with showGuides =
        False; view._renderParams is never mutated, since pick()
        reuses it for the artist's own picks. showGuides=False drops
        joints and solvers but NOT controls, whose schema purpose is
        already "default": a control still wins this pick, and
        _ResolveSnap rejects it by prim type instead. Any failure
        degrades to None rather than killing the drag.
        """
        view = self._view
        if view is None:
            return None
        try:
            # Function-level like stageView.py's own pick(), which
            # keeps the GL import off the create-first-image path.
            from pxr import UsdImagingGL
            from OpenGL import GL
        except ImportError as error:
            Tf.Warn("rigExecUsdview: snap pick unavailable: %s"
                    % error)
            return None
        try:
            stage = self.usdviewApi.stage
            if stage is None:
                return None
            inBounds, frustum = view.computePickFrustum(x, y)
            if not inBounds:
                return None
            dataModel = self.usdviewApi.dataModel
            viewSettings = dataModel.viewSettings
            params = UsdImagingGL.RenderParams()
            try:
                params.frame = dataModel.currentFrame
            except AttributeError:
                params.frame = self._frame
            params.complexity = viewSettings.complexity.value
            try:
                params.drawMode = view._renderModeDict[
                    viewSettings.renderMode]
            except (AttributeError, KeyError):
                # Read-only fallback: pick() keeps the artist's own
                # draw mode here, and reading never disturbs it.
                params.drawMode = view._renderParams.drawMode
            params.showGuides = False
            params.showProxy = viewSettings.displayProxy
            params.showRender = viewSettings.displayRender
            params.enableSceneMaterials = \
                viewSettings.enableSceneMaterials
            params.enableSceneLights = viewSettings.enableSceneLights
            renderer = view._getRenderer()
            if renderer is None:
                return None
            view.makeCurrent()
            GL.glDepthMask(GL.GL_TRUE)
            pickParams = UsdImagingGL.Engine.PickParams()
            pickParams.resolveMode = "resolveNearestToCenter"
            hits = renderer.TestIntersection(
                pickParams, frustum.ComputeViewMatrix(),
                frustum.ComputeProjectionMatrix(),
                stage.GetPseudoRoot(), params)
        except Exception as error:
            Tf.Warn("rigExecUsdview: snap pick failed: %s" % error)
            return None
        if not hits:
            return None
        hit = hits[0]
        path = getattr(hit, "hitPrimPath", None)
        point = getattr(hit, "hitPoint", None)
        if path is None or point is None:
            return None
        normal = getattr(hit, "hitNormal", None)
        if normal is None:
            normal = Gf.Vec3d(0, 0, 1)
        else:
            normal = Gf.Vec3d(normal[0], normal[1], normal[2])
        return (path, Gf.Vec3d(point[0], point[1], point[2]),
                normal)

    @staticmethod
    def _RigRoot(target):
        """The target's rig root, or None when it has none."""
        try:
            return target.RigRootPath() if target is not None \
                else None
        except Exception:
            return None

    def _RigWrittenSets(self, stage, rigRoot):
        """
        (rigWritten, rigWrittenByTarget, allWritten,
        allWrittenByTarget): prim paths whose points a mover writes
        (spec 4.4 step 3a), plus whether EVERY visible PointBased prim
        is in each set.

        Collected over the whole stage the way
        rigExecUsdview._FindRigPaths does
        (rigExecUsdview.py:438-445), not through the hit prim's
        ancestry: the deformed Geom prims are siblings of the rig, so
        an ancestry walk would miss them. rigWrittenByTarget is the
        subset whose writing prim sits at or under rigRoot, empty when
        that is None. The flags are what _NoTargetNote reads: the
        "(rig-driven geometry excluded)" hint is only true when there
        is no other geometry to land on.
        """
        written, byTarget = set(), set()
        if stage is None:
            return frozenset(), frozenset(), False, False
        # PrimRange is path order, so a writing prim (/Mover) sorts
        # AFTER the geometry it writes (/Mesh): membership is tested
        # after the traversal, not inside it.
        visiblePaths = []
        try:
            invisible = UsdGeom.Tokens.invisible
            for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                primPath = prim.GetPath()
                for name in ("rigExec:moves",
                             "rigExec:weightTarget"):
                    relationship = prim.GetRelationship(name)
                    if relationship is None:
                        continue
                    for target in relationship.GetTargets():
                        try:
                            writtenPath = target.GetPrimPath()
                        except Exception:
                            continue
                        written.add(writtenPath)
                        if rigRoot is not None and (
                                primPath == rigRoot
                                or primPath.HasPrefix(rigRoot)):
                            byTarget.add(writtenPath)
                if prim.IsA(UsdGeom.PointBased) and \
                        UsdGeom.Imageable(prim).ComputeVisibility() \
                        != invisible:
                    visiblePaths.append(primPath)
        except Exception as error:
            Tf.Warn("rigExecUsdview: rig-written scan failed: %s"
                    % error)
        written, byTarget = frozenset(written), frozenset(byTarget)
        return (written, byTarget,
                bool(visiblePaths) and all(
                    p in written for p in visiblePaths),
                bool(visiblePaths) and all(
                    p in byTarget for p in visiblePaths))

    def _HoverRigSets(self):
        """
        The rig-written sets for the hover preview, cached per
        (stage, rig root): the traversal is stage-wide, so redoing it
        on every hover move would crawl on a production asset. The
        drag path rebuilds its own sets in _BeginDrag instead, where
        one traversal per press is affordable and always fresh.
        """
        stage = self.usdviewApi.stage
        rigRoot = self._RigRoot(self._target)
        if stage is not self._hoverRigStage \
                or rigRoot != self._hoverRigRoot:
            written, byTarget, allWritten, allByTarget = \
                self._RigWrittenSets(stage, rigRoot)
            self._hoverRigWritten = written
            self._hoverRigWrittenByTarget = byTarget
            self._hoverRigAllWritten = allWritten
            self._hoverRigAllWrittenByTarget = allByTarget
            self._hoverRigStage = stage
            self._hoverRigRoot = rigRoot
        return self._hoverRigWritten, \
            self._hoverRigWrittenByTarget, \
            self._hoverRigAllWritten, \
            self._hoverRigAllWrittenByTarget

    def _ResolveSnap(self, x, y, mode):
        """
        The DragState.snapResolver injection (spec 4.1, 4.4): a pick
        into a gizmoSnap.SnapCandidate, or None.

        The mode travels on every call, never bound at the press: the
        holds change mid-drag, so a captured mode would make a mid-drag
        V or C inert. Rejections: the dragged prim and its ancestors
        and descendants (the stage renders as already written, so the
        object would glue itself to the cursor ray); anything at or
        under the target's rig root, but only when that root is not
        None -- xform targets have none, and Sdf.Path.HasPrefix(None)
        raises, so this guards the way
        gizmoMath.NoticeAffectsTarget already does
        (gizmoMath.py:342); RigExec-typed prims (the guides
        showGuides=False cannot drop -- controls keep purpose
        "default"); rig-deformed prims for Point/Edge, and
        own-rig-deformed ones for Surface. WHY a miss happened lands
        on _lastSnapNote (drag) or _hoverNote (hover) for the status
        clause.
        """
        stage = self.usdviewApi.stage
        if self._drag is not None:
            target = self._drag.target
            rigWritten = self._drag.rigWritten
            rigWrittenByTarget = self._drag.rigWrittenByTarget
            allWritten = self._drag.allRigWritten
            allByTarget = self._drag.allRigWrittenByTarget
            camera = self._drag.camera
            viewport = self._drag.viewport
            noteSink = "_lastSnapNote"
        else:
            target = self._target
            rigWritten, rigWrittenByTarget, allWritten, \
                allByTarget = self._HoverRigSets()
            camera, viewport, _ = self._Camera()
            noteSink = "_hoverNote"
            if camera is None:
                return None
        # Surface is rejected only by its own set, so its misses
        # read the Surface flag; every other miss reads the Point /
        # Edge one (spec 4.4 step 3a).
        noTarget = self._NoTargetNote(
            allByTarget if mode == gizmoSettings.SNAP_SURFACE
            else allWritten)
        setattr(self, noteSink, "")
        if target is None:
            return None
        hit = self._SnapPick(x, y)
        if hit is None:
            setattr(self, noteSink, noTarget)
            return None
        hitPath, hitPoint, hitNormal = hit
        try:
            primPath = target.prim.GetPath()
        except Exception:
            return None
        if hitPath == primPath or hitPath.HasPrefix(primPath) \
                or primPath.HasPrefix(hitPath):
            setattr(self, noteSink, noTarget)
            return None
        rigRoot = self._RigRoot(target)
        if rigRoot is not None and (hitPath == rigRoot
                                    or hitPath.HasPrefix(rigRoot)):
            setattr(self, noteSink,
                    "%s is on the dragged rig" % hitPath)
            return None
        # RigExec's guides are synthesized Hydra widgets, never stage
        # geometry. showGuides=False drops joints and solvers, whose
        # schema fallback is purpose "guide", but NOT controls, whose
        # fallback is the stock UsdGeomImageable "default"
        # (libs/rigExecSchema/schema.usda:123-199,
        # libs/rigExecImaging/sceneIndices.cpp:939-944). Point and
        # Edge already refuse them for not being PointBased; Surface
        # would otherwise land the pivot on a drawn control circle.
        hitPrim = stage.GetPrimAtPath(hitPath) if stage else None
        if hitPrim is not None and hitPrim.IsValid() and \
                str(hitPrim.GetTypeName()).startswith("RigExec"):
            setattr(self, noteSink, "%s is a rig guide" % hitPath)
            return None
        if mode in (gizmoSettings.SNAP_POINT, gizmoSettings.SNAP_EDGE) \
                and hitPath in rigWritten:
            setattr(self, noteSink,
                    "%s is rig-deformed" % hitPath)
            return None
        if mode == gizmoSettings.SNAP_SURFACE \
                and hitPath in rigWrittenByTarget:
            setattr(self, noteSink,
                    "%s is rig-deformed" % hitPath)
            return None
        if mode == gizmoSettings.SNAP_SURFACE:
            return self._SurfaceCandidate(
                hitPath, hitPoint, hitNormal, camera, viewport,
                noteSink, allByTarget)
        if mode == gizmoSettings.SNAP_POINT:
            return self._PointCandidate(
                hitPath, x, y, camera, viewport, noteSink,
                allWritten)
        if mode == gizmoSettings.SNAP_EDGE:
            return self._EdgeCandidate(
                hitPath, x, y, camera, viewport, noteSink,
                allWritten)
        return None

    @staticmethod
    def _NoTargetNote(allRigWritten):
        # State the consequence rather than hiding it, but only when
        # it is true: the parenthetical claims EVERY visible PointBased
        # prim is rig-written, so it reads the flag, not the set (spec
        # 4.4 step 3a).
        if allRigWritten:
            return "no target (rig-driven geometry excluded)"
        return "no target"

    def _PrimWorldPoints(self, hitPath):
        """
        The hit prim's authored points in world, or None.

        XformCache, not ComputeRigFrames: the latter composes frames
        from rest/avars only and drops the prim's own xformOps, and it
        takes a path SET, not the cache (gizmoMath.py:445, 465).
        Rig-deformed prims never reach here -- the resolver rejects
        them first, since their authored array is not what Hydra
        draws.
        """
        try:
            stage = self.usdviewApi.stage
            prim = stage.GetPrimAtPath(hitPath) if stage else None
            if prim is None or not prim.IsValid() \
                    or not prim.IsA(UsdGeom.PointBased):
                return None
            points = UsdGeom.PointBased(prim).GetPointsAttr().Get(
                self._frame)
            if not points:
                return None
            matrix = UsdGeom.XformCache(
                self._frame).GetLocalToWorldTransform(prim)
            return [matrix.Transform(Gf.Vec3d(p)) for p in points]
        except Exception:
            return None

    def _SnapPairs(self, hitPath, pointCount):
        """
        (i, j) index pairs for the hit prim's snap segments, or None.

        Mesh edges from the face counts/indices, curve segments from
        consecutive CVs -- the topology half _EdgeCandidate used to
        rebuild on every move. Cached beside the world points in
        _SnapGeom (gizmoSnap.MeshEdges / CurveSegments).
        """
        try:
            stage = self.usdviewApi.stage
            prim = stage.GetPrimAtPath(hitPath) if stage else None
            if prim is None or not prim.IsValid():
                return None
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                pairs = gizmoSnap.MeshEdges(
                    mesh.GetFaceVertexCountsAttr().Get(),
                    mesh.GetFaceVertexIndicesAttr().Get())
            elif prim.IsA(UsdGeom.BasisCurves):
                pairs = gizmoSnap.CurveSegments(
                    UsdGeom.BasisCurves(prim).
                    GetCurveVertexCountsAttr().Get(), pointCount)
            else:
                return None
        except Exception:
            return None
        return pairs or None

    def _SnapGeom(self, hitPath, camera, viewport, ratio,
                  wantPairs):
        """
        The cached geometry the candidates resolve against, or None.

        WHY a cache at all: the per-prim world points, the mesh edge
        list and the segment list are constant for a whole drag but
        were rebuilt from scratch on every move -- 21 ms (Point) and
        58 ms (Edge) per resolve on a 4225-point mesh against the
        spec's 4 ms budget, and the same code ran on plain hover with
        a mode merely armed. The world half is keyed (prim, frame);
        the screen projection alongside it rekeys on the camera view
        matrix plus the viewport, so a hover camera move rebuilds
        only the projection while a drag (fixed camera) builds once
        per press. Queries gather the 3x3 cells around the cursor, so
        only that subset reaches NearestPoint / NearestSegment; the
        returned index maps back to the prim's own numbering.
        Invalidated in _onObjectsChanged (the notice's own paths),
        _onFrameChanged and _onStageReplaced: skipping that is a
        stale-list IndexError.
        """
        key = str(hitPath)
        entry = self._snapGeom.get(key)
        if entry is None or entry.get("frame") != self._frame:
            world = self._PrimWorldPoints(hitPath)
            if not world:
                self._snapGeom.pop(key, None)
                return None
            if len(self._snapGeom) >= 64:
                self._snapGeom.clear()
            entry = {"frame": self._frame, "world": world,
                     "pairs": None, "segments": None,
                     "viewKey": None, "proj": None,
                     "cells": None, "segCells": None}
            self._snapGeom[key] = entry
        world = entry["world"]
        if wantPairs and entry["pairs"] is None:
            pairs = self._SnapPairs(hitPath, len(world))
            if not pairs:
                return None
            entry["pairs"] = pairs
            entry["segments"] = [(world[i], world[j])
                                 for (i, j) in pairs]
            # The projection below may already be built -- a Point
            # resolve on this prim ran first -- and it buckets only
            # the segments it knew. Force it to run again so the new
            # segments join segCells.
            entry["viewKey"] = None
        viewProj = gizmoScreen.ViewProjection(camera)
        viewKey = (str(viewProj), viewport[0], viewport[1],
                   viewport[2], viewport[3], ratio)
        if entry.get("viewKey") != viewKey:
            cell = gizmoSnap.SNAP_PIXELS * ratio
            proj = []
            cells = {}
            for index, point in enumerate(world):
                try:
                    screen, _ = gizmoScreen.ProjectPointWithW(
                        viewProj, viewport, point)
                except Exception:
                    screen = None
                proj.append(screen)
                if screen is not None:
                    cells.setdefault(
                        (math.floor(screen[0] / cell),
                         math.floor(screen[1] / cell)),
                        []).append(index)
            segCells = {}
            if entry["segments"] is not None:
                for segIndex, (i, j) in enumerate(
                        entry["pairs"]):
                    screenA, screenB = proj[i], proj[j]
                    if screenA is None or screenB is None:
                        continue
                    x0, x1 = sorted((screenA[0], screenB[0]))
                    y0, y1 = sorted((screenA[1], screenB[1]))
                    for cx in range(math.floor(x0 / cell),
                                    math.floor(x1 / cell) + 1):
                        for cy in range(math.floor(y0 / cell),
                                        math.floor(y1 / cell) + 1):
                            segCells.setdefault(
                                (cx, cy), []).append(segIndex)
            entry["viewKey"] = viewKey
            entry["proj"] = proj
            entry["cells"] = cells
            entry["segCells"] = segCells
        return entry

    @staticmethod
    def _SnapCells(cells, x, y, cell):
        """
        The pooled 3x3 cells around the cursor, deduped, in prim
        order: NearestPoint / NearestSegment keep the first winner
        on a tie (gizmoSnap.py:232, 292), so ascending order is what
        keeps the subset answer identical to the full scan.
        """
        found = []
        cx, cy = math.floor(x / cell), math.floor(y / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                found.extend(cells.get((cx + dx, cy + dy), ()))
        return sorted(dict.fromkeys(found))

    def _DropSnapGeom(self, resynced, changed):
        """
        Forget the cached geometry a notice may have moved.

        A resync above a prim rebuilds its subtree and an info-only
        change to one of its properties (including the points array
        itself, which arrives as a property path) moves its points,
        so either drops the entry; every untouched prim keeps its
        one build per (prim, frame, camera). String prefixes, not
        Sdf paths: the notice paths may be property paths, where a
        "/Target.points" change must drop "/Target".
        """
        if not self._snapGeom:
            return
        touched = set(str(p) for p in list(resynced)
                      + list(changed))
        # A notice path may be a property path ("/Target.points"),
        # whose prim prefix is what the cache is keyed on.
        for path in list(touched):
            touched.add(path.split(".")[0])
        for key in list(self._snapGeom):
            for path in touched:
                if path == key or key.startswith(path + "/") \
                        or path.startswith(key + "/"):
                    del self._snapGeom[key]
                    break

    def _PointCandidate(self, hitPath, x, y, camera, viewport,
                        noteSink, allRigWritten):
        ratio = self._Ratio()
        entry = self._SnapGeom(hitPath, camera, viewport, ratio,
                               False)
        if entry is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        cell = gizmoSnap.SNAP_PIXELS * ratio
        nearby = self._SnapCells(entry["cells"], x, y, cell)
        if not nearby:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        try:
            found = gizmoSnap.NearestPoint(
                [entry["world"][i] for i in nearby], (x, y),
                gizmoScreen.ViewProjection(camera), viewport,
                gizmoSnap.SNAP_PIXELS * ratio)
        except Exception:
            found = None
        if found is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        index, point, screen = found
        return gizmoSnap.SnapCandidate(
            point, Gf.Vec3d(0, 0, 0), gizmoSettings.SNAP_POINT,
            hitPath, nearby[index], screen)

    def _EdgeCandidate(self, hitPath, x, y, camera, viewport,
                       noteSink, allRigWritten):
        ratio = self._Ratio()
        entry = self._SnapGeom(hitPath, camera, viewport, ratio,
                               True)
        if entry is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        cell = gizmoSnap.SNAP_PIXELS * ratio
        nearby = self._SnapCells(entry["segCells"], x, y, cell)
        if not nearby:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        segments = entry["segments"]
        try:
            found = gizmoSnap.NearestSegment(
                [segments[k] for k in nearby], (x, y),
                gizmoScreen.ViewProjection(camera), viewport,
                gizmoSnap.SNAP_PIXELS * ratio)
        except Exception:
            found = None
        if found is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        segIndex, point, screen = found
        original = nearby[segIndex]
        candidate = gizmoSnap.SnapCandidate(
            point, Gf.Vec3d(0, 0, 0), gizmoSettings.SNAP_EDGE,
            hitPath, original, screen)
        # The picked segment's world ends, for the marker's two
        # adjacent halves (spec 4.5).
        candidate.edgeEnds = segments[original]
        return candidate

    def _SurfaceCandidate(self, hitPath, hitPoint, hitNormal,
                          camera, viewport, noteSink,
                          allRigWritten):
        try:
            screen = gizmoScreen.ProjectPoint(
                gizmoScreen.ViewProjection(camera), viewport,
                hitPoint)
        except Exception:
            screen = None
        if screen is None:
            setattr(self, noteSink,
                    self._NoTargetNote(allRigWritten))
            return None
        return gizmoSnap.SnapCandidate(
            Gf.Vec3d(hitPoint), Gf.Vec3d(hitNormal),
            gizmoSettings.SNAP_SURFACE, hitPath, -1, screen)

    # -- drag -----------------------------------------------------------

    def _BeginDrag(self, handle, point):
        """
        Bracket a drag: snapshot the attributes, record the base values,
        and build the Qt-free DragState the maths runs on.
        """
        self.FlushReleasedEdit()
        target = self._target
        if target is None or not handle.grabbable:
            return False
        if self._SwitchPresentedLimbs():
            # The grabbed controls now drive from where they were drawn;
            # the drag runs on a target rebuilt from them.
            self.RefreshTarget()
            target = self._target
            if target is None:
                return False
        camera, viewport, _ = self._Camera()
        if not self._gesture.Begin(
                self._tool, target, handle, point, camera, viewport, self.settings,
                gimbal=self._gimbalAxes is not None, ctrl=self._ctrl,
                holdGrid=self._holdGrid, holdPoint=self._holdPoint,
                holdEdge=self._holdEdge, primePivot=self._PrimeGroupPivot,
                rigWrittenSets=lambda t: self._RigWrittenSets(
                    self.usdviewApi.stage, self._RigRoot(t)),
                snapResolver=self._ResolveSnap):
            return False
        self._lastSnapNote = ""
        self.toolbar.Sync()
        self._Repaint()
        return True

    def _SwitchPresentedLimbs(self):
        """
        Selecting (or grabbing) a control of the half a limb is not
        driving with: switch that limb to it, matched (ikfkMatch), as its
        own undo entry, so the control drives from where it is drawn.
        False when no selected control is presented, or no values could
        be read.
        """
        stage = self.usdviewApi.stage
        prims = self._SelectedPrims() if stage else []
        if not prims:
            return False
        time = self._frame
        halves = []
        for prim in prims:
            posed = self._solverPosed.For(gizmoMath.FindRigRoot(prim))
            half = gizmoMath.PresentedBy(stage, prim.GetPath(), time, posed)
            if half is not None and half not in halves:
                halves.append(half)
        if not halves:
            return False
        try:
            import ikfkMatch
            switches = set(h.switch for h in halves)
            limbs = [l for l in self._MatchLimbs(stage)
                     if l.switchPath in switches]
            if not limbs:
                return False
            ikfkMatch.SwitchLimbs(stage, limbs, time, self._writeMode,
                                  self.undoStack)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not switch the limb before the "
                    "drag: %s" % error)
            return False
        return True

    def _MatchLimbs(self, stage):
        """The stage's IK/FK limbs, found once per stage."""
        import ikfkMatch
        if getattr(self, "_matchLimbStage", None) is not stage:
            self._matchLimbStage = stage
            self._matchLimbs = ikfkMatch.FindLimbs(stage)
        return self._matchLimbs

    def _UpdateDrag(self, point):
        if not self.IsDragging():
            return
        self._gesture.Update(point, self.settings, holdSnap=self._holdSnap,
                             holdGrid=self._holdGrid, holdPoint=self._holdPoint,
                             holdEdge=self._holdEdge)
        self._RebuildHandles()
        self._Repaint()
        self.toolbar.Sync()
        self.usdviewApi.UpdateViewport()

    def _ClearHolds(self):
        """
        Drop the snap holds (J / X / C / V) and the key latch at the
        end of a drag.

        A KeyRelease can land on a widget this filter never sees (the
        focus moves under the cursor while a key is down), and a hold
        that survived its drag would silently snap the next one.
        """
        self._holdSnap = False
        self._holdGrid = False
        self._holdEdge = False
        self._holdPoint = False
        self._holdPreserve = False
        self._claimedKey = None

    def _EndDrag(self):
        drag = self._gesture.Release()
        self._ClearHolds()
        if drag is None:
            return
        QtCore.QTimer.singleShot(0, lambda: self._FlushReleasedToken(drag))
        self.toolbar.Sync()
        self._Repaint()

    def ReleasedEditLabel(self):
        return self._gesture.ReleasedEditLabel()

    def _FlushReleasedToken(self, drag):
        if self._gesture.releasedEdit is drag:
            try:
                self.FlushReleasedEdit()
            except Exception as error:
                Tf.Warn("rigExecUsdview: queued gizmo edit failed: %s" % error)

    def FlushReleasedEdit(self):
        warnings = self._gesture.Commit()
        if warnings is None:
            return
        self._warnings = warnings
        if not self._viewDestroyed:
            self._RebuildHandles()
            self.toolbar.Sync()
            self._Repaint()

    def _AbortDrag(self):
        self.FlushReleasedEdit()
        self._ClearHolds()
        cancelled = self._gesture.Cancel()
        if cancelled is None:
            return
        if not cancelled:
            self._RebuildHandles()
            return
        self._RebuildHandles()
        self.toolbar.Sync()
        self._Repaint()
        self.usdviewApi.UpdateViewport()


# Installation

def InstallViewportTools(usdviewApi, undoStack,
                         retries=stageViewReady.DEFAULT_RETRIES,
                         openGraphEditor=None):
    yielded = False

    def _Create():
        nonlocal yielded
        yielded = editorExtension.OwnsInput(usdviewApi)
        return GizmoController(usdviewApi, undoStack,
                               openGraphEditor=openGraphEditor,
                               installFilters=not yielded)

    def _Prepare(controller):
        module = sys.modules.get("rigExecUsdview")
        container = module.ContainerFor(usdviewApi) if module is not None else None
        if container is not None:
            container._BindViewportTools()
        if yielded:
            controller._extensionNotified = True
            editorExtension.EditorAdded(usdviewApi, editorExtension.GIZMO,
                                        controller, required=True)

    def _OnReady(controller):
        for installed in stageViewReady.Controllers("gizmo"):
            scope = getattr(installed, "_ScopeShortcuts", None)
            if scope is not None:
                scope()

    return stageViewReady.Register(usdviewApi, "gizmo", _Create,
                                   retries=retries, prepare=_Prepare,
                                   onReady=_OnReady).controller


def GetController(usdviewApi=None):
    return stageViewReady.Controller(usdviewApi, "gizmo")


def __getattr__(name):
    if name == "_controller":
        return GetController()
    raise AttributeError("module %r has no attribute %r" % (__name__, name))
