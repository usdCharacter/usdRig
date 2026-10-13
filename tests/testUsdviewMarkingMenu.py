#
# THE RIGHT-CLICK MARKING MENU, end to end, in a real usdview.
#
# Real mouse events to the stage view, on the biped:
#
#   * with a control selected, a right-click anywhere in the viewport opens
#     the marking menu instead of usdview's prim menu;
#   * a flick runs its command with nothing drawn: West keys, South zeroes,
#     South-West selects the mirrored control;
#   * a quick tap leaves the ring up to be clicked, a click on a label runs
#     it, and Escape dismisses it;
#   * a submenu leaves a circle where its parent was centred, and moving
#     back onto it closes the submenu;
#   * the column below the ring is taken held or clicked: usdview's own
#     prim menu opens from it;
#   * right-clicking a control that is not selected selects it, and the
#     menu acts on it -- and a TouchPose region does the same for its
#     control;
#   * with nothing selected and no control under the cursor, and with Alt
#     held, the right-click stays usdview's.
#
from pxr import Sdf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

CONTROLS = "/Biped/Rig/Main/Shot/Aux/Controls"
L_ARM = CONTROLS + "/L_Arm"
R_ARM = CONTROLS + "/R_Arm"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _Clock(object):
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _Event(view, kind, point, button, buttons=None, modifiers=None):
    pos = QtCore.QPointF(float(point[0]), float(point[1]))
    glob = view.mapToGlobal(QtCore.QPoint(int(point[0]), int(point[1])))
    return QtGui.QMouseEvent(
        kind, pos, QtCore.QPointF(float(glob.x()), float(glob.y())), button,
        button if buttons is None else buttons,
        modifiers or QtCore.Qt.NoModifier)


class _Mouse(object):
    def __init__(self, app, view, clock):
        self._app = app
        self._view = view
        self._clock = clock

    def Send(self, kind, point, button, buttons=None, modifiers=None):
        event = _Event(self._view, kind, point, button, buttons, modifiers)
        QtWidgets.QApplication.sendEvent(self._view, event)
        self._app._processEvents()

    def Flick(self, start, offset):
        """A right-button flick, over well inside the show delay."""
        right = QtCore.Qt.RightButton
        end = (start[0] + offset[0], start[1] + offset[1])
        self._clock.now += 1000.0
        self.Send(QtCore.QEvent.MouseButtonPress, start, right)
        for i in (1, 2, 3):
            self._clock.now += 10.0
            point = (start[0] + offset[0] * i / 3.0,
                     start[1] + offset[1] * i / 3.0)
            self.Send(QtCore.QEvent.MouseMove, point, QtCore.Qt.NoButton,
                      right)
        self._clock.now += 10.0
        self.Send(QtCore.QEvent.MouseButtonRelease, end, right,
                  QtCore.Qt.NoButton)

    def Tap(self, point):
        right = QtCore.Qt.RightButton
        self._clock.now += 1000.0
        self.Send(QtCore.QEvent.MouseButtonPress, point, right)
        self._clock.now += 50.0
        self.Send(QtCore.QEvent.MouseButtonRelease, point, right,
                  QtCore.Qt.NoButton)

    def Press(self, point):
        self._clock.now += 1000.0
        self.Send(QtCore.QEvent.MouseButtonPress, point,
                  QtCore.Qt.RightButton)

    def HeldMove(self, point, ms=20.0):
        self._clock.now += ms
        self.Send(QtCore.QEvent.MouseMove, point, QtCore.Qt.NoButton,
                  QtCore.Qt.RightButton)
        # The menu's own timer ticks too; let it see the new time.
        self._app._processEvents()

    def Release(self, point):
        self._clock.now += 20.0
        self.Send(QtCore.QEvent.MouseButtonRelease, point,
                  QtCore.Qt.RightButton, QtCore.Qt.NoButton)

    def LeftClick(self, point):
        left = QtCore.Qt.LeftButton
        self._clock.now += 300.0
        self.Send(QtCore.QEvent.MouseButtonPress, point, left)
        self.Send(QtCore.QEvent.MouseButtonRelease, point, left,
                  QtCore.Qt.NoButton)


def _Select(api, paths):
    selection = api.dataModel.selection
    selection.clearPrims()
    for path in paths:
        selection.addPrim(api.stage.GetPrimAtPath(path))


def _Selected(api):
    return [str(p.GetPath()) for p in api.dataModel.selection.getPrims()
            if not p.IsPseudoRoot()]


def _ControlPoint(menu, view):
    """A viewport point a control is drawn under, found by asking the
    viewport's own pick on a grid."""
    w, h = view.width(), view.height()
    for fy in range(2, 19):
        for fx in range(2, 19):
            point = (w * fx / 20.0, h * fy / 20.0)
            hovered = menu.HoveredControls(point)
            if hovered:
                return point, hovered[0]
    return None, None


def _EmptyPoint(menu, view):
    for point in ((8.0, view.height() - 8.0), (8.0, 8.0),
                  (view.width() - 8.0, view.height() * 0.5)):
        if not menu.HoveredControls(point):
            return point
    return None


def testUsdviewInputFunction(appController):
    import markingMenuUI

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    view = markingMenuUI.StageView(api)
    menu = markingMenuUI.GetMenu(api)
    _Check(menu is not None, "the marking menu is installed on the viewport")
    clock = _Clock()
    menu.clock = clock
    mouse = _Mouse(appController, view, clock)
    empty = _EmptyPoint(menu, view)
    _Check(empty is not None, "there is empty viewport to click")
    frame = Usd.TimeCode(api.frame) if not isinstance(
        api.frame, Usd.TimeCode) else api.frame

    # --- nothing selected, nothing under the cursor: usdview's ----------
    _Select(api, [])
    event = _Event(view, QtCore.QEvent.MouseButtonPress, empty,
                   QtCore.Qt.RightButton)
    _Check(not menu.eventFilter(view, event),
           "with no control in play the right-click is usdview's")
    _Select(api, [L_ARM])
    event = _Event(view, QtCore.QEvent.MouseButtonPress, empty,
                   QtCore.Qt.RightButton,
                   modifiers=QtCore.Qt.AltModifier)
    _Check(not menu.eventFilter(view, event),
           "Alt + right-drag stays the camera's zoom")

    # --- a flick West keys the selection --------------------------------
    arm = stage.GetPrimAtPath(L_ARM)
    _Check(arm and arm.IsValid(), "the biped has %s" % L_ARM)
    mouse.Flick(empty, (-60.0, 0.0))
    _Check(not menu.IsOpen(), "a flick finishes the gesture")
    _Check(menu.lastCommand == ("KeySelection", None),
           "West is Key: %r" % (menu.lastCommand,))
    keyed = [a.GetName() for a in arm.GetAttributes()
             if a.HasSpline() or a.GetNumTimeSamples() > 0]
    _Check(keyed, "the selected control got keys")
    _Check(_Selected(api) == [L_ARM], "and the selection is unchanged")

    # --- South zeroes ----------------------------------------------------
    mouse.Flick(empty, (0.0, 60.0))
    _Check(menu.lastCommand == ("ZeroPose", None),
           "South is Zero pose: %r" % (menu.lastCommand,))
    left = [a.GetName() for a in arm.GetAttributes()
            if a.GetName().startswith("avars:") and a.HasAuthoredValue()
            and (a.HasSpline() or a.GetNumTimeSamples() > 0)]
    _Check(not left, "zero cleared the keys it just made: %s" % left)

    # --- South-West selects the mirrored control -------------------------
    mouse.Flick(empty, (-45.0, 45.0))
    _Check(menu.lastCommand == ("SelectCounterpart", None),
           "South-West is Select counterpart: %r" % (menu.lastCommand,))
    _Check(_Selected(api) == [R_ARM],
           "L_Arm's counterpart is R_Arm: %s" % _Selected(api))

    # --- a tap leaves the ring up; a click on a label runs it -----------
    menu.lastCommand = None
    mouse.Tap(empty)
    _Check(menu.IsOpen() and menu._window.isVisible(),
           "a tap leaves the ring up to be clicked")
    window = menu._window
    rects = window.painter.ItemRects(window.menu, window.centre)
    _Check("selection.frame" in rects, "the ring shows Frame: %s"
           % sorted(rects))
    target = rects["selection.frame"].center()
    point = view.mapFromGlobal(window.mapToGlobal(target.toPoint()))
    mouse.LeftClick((point.x(), point.y()))
    _Check(menu.lastCommand == ("FrameSelection", None),
           "clicking Frame ran it: %r" % (menu.lastCommand,))
    _Check(not menu.IsOpen() and not window.isVisible(),
           "and the ring went away")

    shot = __import__("os").environ.get("RIGEXEC_MARKING_SHOT")
    mouse.Tap(empty)
    if shot:
        appController._mainWindow.grab().save(shot)
        window.grab().save(shot.replace(".png", "_ring.png"))
    try:
        from PySide6 import QtTest
    except ImportError:
        from PySide2 import QtTest
    QtTest.QTest.keyClick(view, QtCore.Qt.Key_Escape)
    appController._processEvents()
    _Check(not menu.IsOpen(), "Escape dismisses the ring")

    # --- a submenu, and the circle back to its parent -------------------
    import markingMenuModel as mm
    spaced = None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "RigExecSpaceSwitch" and                 len(prim.GetAttribute("rigExec:spaceLabels").Get() or []) > 1:
            targets = prim.GetRelationship("rigExec:target").GetTargets()
            if targets:
                spaced = str(targets[0])
                break
    _Check(spaced is not None, "the biped has a control with named spaces")
    _Select(api, [spaced])
    centre = empty
    mouse.Press(centre)
    mouse.HeldMove((centre[0] + 1.0, centre[1]), ms=200.0)   # shown
    _Check(window.isVisible(), "the ring is up while held")
    step = mm.RADIUS_PX / 4.0
    for i in (1, 2, 3, 4, 5):
        mouse.HeldMove((centre[0] + step * i * 0.7071,
                        centre[1] - step * i * 0.7071))
    gesture = menu._gesture
    _Check(gesture is not None and gesture.depth == 1,
           "pulling North-East past the ring opened Space")
    _Check(len(window.parents) == 1, "and left a circle where the root was")
    if shot:
        window.grab().save(shot.replace(".png", "_submenu.png"))
    mouse.HeldMove((centre[0] + 3.0, centre[1] - 3.0))
    _Check(menu._gesture.depth == 0, "moving back onto it closed Space")
    _Check(not window.parents, "and the circle went with it")
    menu.Cancel()

    # --- the column: usdview's own prim menu, held or clicked ------------
    opened = []
    real = appController._showPrimContextMenu
    appController._showPrimContextMenu = lambda item: opened.append(item)
    try:
        def rowPoint():
            rows, _box = window.painter.ColumnRects(
                window.menu, window.centre, window.parents)
            row = rows["selection.usdviewMenu"].center().toPoint()
            local = view.mapFromGlobal(window.mapToGlobal(row))
            return (local.x(), local.y())

        mouse.Press(centre)
        mouse.HeldMove((centre[0] + 1.0, centre[1]), ms=200.0)
        row = rowPoint()
        mouse.HeldMove(((centre[0] + row[0]) * 0.5,
                        (centre[1] + row[1]) * 0.5))
        mouse.HeldMove(row)
        _Check(window.lit == "selection.usdviewMenu",
               "the row lights while held: %r" % window.lit)
        mouse.Release(row)
        _Check(menu.lastCommand == ("ShowUsdviewPrimMenu", None) and
               len(opened) == 1, "releasing on the row opened usdview's "
               "prim menu: %r" % (menu.lastCommand,))

        mouse.Tap(centre)
        row = rowPoint()
        mouse.LeftClick(row)
        _Check(len(opened) == 2, "and so does clicking it")
    finally:
        appController._showPrimContextMenu = real

    # --- right-click on an unselected control selects it ----------------
    _Select(api, [])
    point, control = _ControlPoint(menu, view)
    _Check(point is not None, "a control is drawn somewhere in the view")
    mouse.Tap(point)
    _Check(menu.IsOpen(), "right-clicking a control opens the menu")
    _Check(_Selected(api) == [control],
           "and selects that control: %s" % _Selected(api))
    menu.Cancel()
    _Check(not menu.IsOpen(), "cancelled")

    # --- a TouchPose region stands for its control -----------------------
    import os
    import sys
    # Beside this plugin's own directory (testusdview runs the script
    # with no __file__ of its own).
    plugin = os.path.join(os.path.dirname(os.path.abspath(
        markingMenuUI.__file__)), os.pardir, "touchPose")
    if plugin not in sys.path:
        sys.path.insert(0, plugin)
    import touchPoseUI
    panel = touchPoseUI.TouchPosePanel.GetInstance(api)
    touch = panel.controller
    # Frame above zoomed onto one control; put the whole character back.
    _Select(api, [])
    appController._resetView()
    appController._processEvents()
    touch.SetActive(True)
    appController._processEvents()
    _Check(touch.active, "TouchPose is on")
    ratio = view.devicePixelRatioF()
    spot = region = None
    for fy in range(4, 18):
        for fx in range(4, 17):
            candidate = (view.width() * fx / 20.0, view.height() * fy / 20.0)
            got, face = touch.RegionAt(candidate[0] * ratio,
                                       candidate[1] * ratio)
            if got is not None and got.control:
                spot, region = candidate, got
                break
        if spot is not None:
            break
    _Check(spot is not None, "a TouchPose region is on screen")
    _Select(api, [])
    mouse.Tap(spot)
    _Check(menu.IsOpen(), "right-clicking a region opens the menu")
    _Check(_Selected(api) == [str(region.control)],
           "on the region's control: %s vs %s"
           % (_Selected(api), region.control))
    menu.Cancel()
    touch.SetActive(False)
    appController._processEvents()
    print("MARKING_MENU_OK")
