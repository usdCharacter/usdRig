#
# RigExec usdview marking menu: the right-click radial menu for rig
# controls in the viewport.
#
# Right-click in the viewport while controls are selected, or over a
# control (or a TouchPose region, which stands for its control), and this
# menu replaces usdview's own right-click menu:
#
#   * flick toward a direction and release: the command runs, nothing is
#     ever drawn;
#   * hold: the ring appears around the press point, and releasing over
#     an item runs it;
#   * a quick tap leaves the ring up to be clicked; Escape or a click away
#     from it dismisses it;
#   * a submenu leaves a circle where each menu above it was centred, and
#     moving onto one goes back to that menu;
#   * a label or a row of the column below the ring is taken whenever the
#     cursor is on it, held or clicked, ahead of the direction.
#
# The menu acts on the selection. Right-clicking a control that is not
# selected selects it first, the way a click would, so what the menu acts
# on is always what is highlighted. TouchPose regions are just bigger
# targets for the same controls.
#
# What the gesture means -- timings, sectors, submenus, the state machine
# -- is markingMenuModel's, and what the Selection menu holds is
# markingMenuDefs'. This module paints, routes the mouse and runs the
# commands the item ids name.
#
# Painted in a frameless, transparent top-level window over the viewport,
# so a ring near the viewport's edge is not clipped, and taking no input:
# the stage view's own event filter drives the gesture.
#
import math
import os
import re
import sys

from pxr import Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import markingMenuDefs as defs
import markingMenuModel as mm
import sessionRegistry

# Prims the menu treats as controls: what the picker selects.
CONTROL_TYPES = ("RigExecControl", "RigExecJoint")

# usdview's camera claim: Alt or Meta with a press arms its own tumble,
# truck or zoom (stageView.py mousePressEvent), so never ours.
_CAMERA_MODIFIERS = QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier

# The look: dark rounded pills with a light border, lit blue under the
# cursor.
_FILL = QtGui.QColor(73, 73, 73, 255)
_BORDER = QtGui.QColor(133, 133, 133, 255)
_TEXT = QtGui.QColor(255, 255, 255)
_LIT_FILL = QtGui.QColor(109, 160, 177, 255)
_LIT_BORDER = QtGui.QColor(172, 218, 255, 255)
_LIT_TEXT = QtGui.QColor(0, 0, 0)
_DISABLED_TEXT = QtGui.QColor(134, 134, 134, 255)
_CHECK = QtGui.QColor(129, 212, 250)
_GUIDE = QtGui.QColor(128, 128, 128, 255)
_RADIUS = 7.0
_BORDER_WIDTH = 2.0

# Transform channels "Reset to rest" puts back, and the value each rests
# at. Everything else a control carries is left to Zero pose.
_REST_VALUES = {"avars:tx": 0.0, "avars:ty": 0.0, "avars:tz": 0.0,
                "avars:rx": 0.0, "avars:ry": 0.0, "avars:rz": 0.0,
                "avars:rspin": 0.0,
                "avars:sx": 1.0, "avars:sy": 1.0, "avars:sz": 1.0}


def StageView(usdviewApi):
    # usdview keeps its app controller name-mangled on the api object.
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _AppController(usdviewApi):
    return getattr(usdviewApi, "_UsdviewApi__appController", None)


def _EventPoint(event):
    try:
        point = event.position()
    except AttributeError:                            # PySide2
        point = event.pos()
    return (point.x(), point.y())


def _IsControl(prim):
    return bool(prim and prim.IsValid()
                and str(prim.GetTypeName()) in CONTROL_TYPES)


def ControlOf(prim):
    """The control a picked prim belongs to: itself, or the nearest
    ancestor that is one (a control's drawn shape is a child prim)."""
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if _IsControl(prim):
            return prim
        prim = prim.GetParent()
    return None


# --- Counterparts --------------------------------------------------------

_SIDE_SWAPS = (
    (re.compile(r"^L_"), "R_"), (re.compile(r"^R_"), "L_"),
    (re.compile(r"^l_"), "r_"), (re.compile(r"^r_"), "l_"),
    (re.compile(r"_L_"), "_R_"), (re.compile(r"_R_"), "_L_"),
    (re.compile(r"_l_"), "_r_"), (re.compile(r"_r_"), "_l_"),
    (re.compile(r"_L$"), "_R"), (re.compile(r"_R$"), "_L"),
    (re.compile(r"_l$"), "_r"), (re.compile(r"_r$"), "_l"),
    (re.compile(r"^Left"), "Right"), (re.compile(r"^Right"), "Left"),
    (re.compile(r"^left"), "right"), (re.compile(r"^right"), "left"),
)


def CounterpartName(name):
    """The name of the control on the other side, or None for a name that
    carries no side (anything on the centre line). The first side marker
    found is swapped: L_Arm -> R_Arm, eye_l_def -> eye_r_def."""
    for pattern, replacement in _SIDE_SWAPS:
        if pattern.search(name):
            return pattern.sub(replacement, name, count=1)
    return None


def CounterpartPath(stage, path):
    """The mirrored control's path, or None. Looked for under the same
    parent first (a mirrored pair usually shares one), then by name
    anywhere under the rig, each side of the path mirrored as it goes."""
    path = Sdf.Path(str(path))
    name = CounterpartName(path.name)
    if name is None:
        return None
    sibling = path.GetParentPath().AppendChild(name)
    if _IsControl(stage.GetPrimAtPath(sibling)):
        return sibling
    # A control under a sided parent (R_UpArm under R_Shoulder): mirror
    # every sided element of the path.
    mirrored = []
    for element in path.pathString.split("/"):
        swapped = CounterpartName(element) if element else None
        mirrored.append(swapped or element)
    candidate = Sdf.Path("/".join(mirrored))
    if _IsControl(stage.GetPrimAtPath(candidate)):
        return candidate
    for prim in stage.Traverse():
        if prim.GetName() == name and _IsControl(prim):
            return prim.GetPath()
    return None


# --- What the menu knows about the selection -----------------------------


class SelectionContext(object):
    """markingMenuModel's context for the controls the menu acts on: the
    predicates, state, labels and child lists of markingMenuDefs."""

    def __init__(self, menu, controls):
        self._menu = menu
        self.stage = menu.Stage()
        self.time = menu.Time()
        self.controls = [p for p in (self.stage.GetPrimAtPath(Sdf.Path(c))
                                     for c in controls) if _IsControl(p)]
        self._limbs = None
        self._space = None

    # -- the rig around the controls --------------------------------------

    def Limbs(self):
        """The switchable limbs the controls belong to, in order."""
        if self._limbs is None:
            self._limbs = []
            found = self._menu.KnownLimbs()
            if found is not None:
                ikfkMatch, limbs = found
                for prim in self.controls:
                    limb = ikfkMatch.LimbFor(limbs, prim.GetPath())
                    if limb is not None and limb not in self._limbs:
                        self._limbs.append(limb)
        return self._limbs

    def Space(self):
        """(space switch prim, active attribute, labels) for the lead
        control (the last selected), or None."""
        if self._space is None:
            self._space = False
            lead = self.controls[-1] if self.controls else None
            if lead is not None:
                for prim in self.stage.Traverse():
                    if prim.GetTypeName() != "RigExecSpaceSwitch":
                        continue
                    targets = prim.GetRelationship(
                        "rigExec:target").GetTargets()
                    if lead.GetPath() not in targets:
                        continue
                    active = prim.GetRelationship(
                        "rigExec:activeSpaceAttribute").GetTargets()
                    labels = prim.GetAttribute(
                        "rigExec:spaceLabels").Get() or []
                    if active and len(labels) > 1:
                        self._space = (prim, active[0], list(labels))
                        break
        return self._space or None

    # -- markingMenuModel's four questions --------------------------------

    def Test(self, name):
        if name == "control":
            return bool(self.controls)
        if name == "limb":
            return bool(self.Limbs())
        if name == "hasSpaces":
            return self.Space() is not None
        return False

    def Read(self, name):
        if name == "limbIsIk":
            limbs = self.Limbs()
            return bool(limbs) and limbs[0].IsIk(self.stage, self.time)
        if name == "space":
            space = self.Space()
            if space is None:
                return None
            attr = self.stage.GetAttributeAtPath(space[1])
            value = attr.Get(self.time) if attr else None
            return int(round(value)) if value is not None else 0
        return None

    def Label(self, itemId, default):
        if itemId == "selection.fkik":
            limbs = self.Limbs()
            if limbs:
                return "Switch to %s" % (
                    "FK" if limbs[0].IsIk(self.stage, self.time) else "IK")
        if itemId == "selection.space":
            space = self.Space()
            if space is not None:
                return "Space"
        return default

    def Children(self, name):
        if name == "spaces":
            space = self.Space()
            if space is None:
                return ()
            return [dict(id="selection.space.%d" % i, label=str(label),
                         kind=mm.RADIO, read="space", value=i,
                         write="SetSpace")
                    for i, label in enumerate(space[2])]
        return ()


# --- The painted menu ----------------------------------------------------


class MenuPainter(object):
    """Where each item of a resolved menu is drawn and what is under a
    point, in the window's logical pixels around a centre."""

    def __init__(self, font):
        self.font = QtGui.QFont(font)
        if self.font.pointSizeF() > 0:
            self.font.setPointSizeF(self.font.pointSizeF() * 1.1)
        metrics = QtGui.QFontMetricsF(self.font)
        self.metrics = metrics
        self.base = metrics.height()
        # An ellipse rather than a circle: wide labels need the room on
        # the left and right more than above and below.
        r = mm.RADIUS_PX
        ry = r * 0.7
        dx, dy = r * 0.8, r * 0.45 * 0.7 + self.base * 1.2
        self._anchors = {"N": (0.0, -ry), "S": (0.0, ry),
                         "E": (r, 0.0), "W": (-r, 0.0),
                         "NE": (dx, -dy), "NW": (-dx, -dy),
                         "SE": (dx, dy), "SW": (-dx, dy)}

    def _Size(self, text):
        width = self.metrics.horizontalAdvance(text)
        return (width + self.base * 2.2, self.base * 1.6)

    def _Text(self, resolved):
        text = resolved.label
        if resolved.isSubmenu:
            text += "  ▸"
        return text

    def ItemRects(self, menu, centre, avoid=()):
        """{item id: QRectF} for the ring, anchored so each label grows
        away from the centre.

        `avoid` is the back circles of the menus above. A submenu opened
        toward one side has its label for the way back lying on its
        parent's circle, and reaching for that label would go back; such
        a label slides sideways, off the path to the circle, keeping its
        direction from the centre."""
        out = {}
        cx, cy = centre
        for direction, resolved in menu.slots.items():
            ax, ay = self._anchors[direction]
            w, h = self._Size(self._Text(resolved))
            if resolved.checked:
                w += self.base
            x, y = cx + ax, cy + ay
            if "W" in direction:
                x -= w
                y -= h * 0.5
            elif "E" in direction:
                y -= h * 0.5
            elif direction == "N":
                x -= w * 0.5
                y -= h
            else:                                   # S
                x -= w * 0.5
            rect = QtCore.QRectF(x, y, w, h)
            out[resolved.id] = self._Clear(rect, direction, avoid)
        return out

    @staticmethod
    def _Clear(rect, direction, avoid):
        """`rect` moved across its direction until no back circle in
        `avoid` lies under it (with a margin)."""
        if not avoid:
            return rect
        reach = mm.BACK_RADIUS_PX + 6.0
        dx, dy = mm.DIRECTION_VECTORS[direction]
        # Across the direction: up for East and West, aside for North and
        # South, and the outward perpendicular for a diagonal.
        if dy == 0:
            px, py = 0.0, -1.0
        elif dx == 0:
            px, py = 1.0, 0.0
        else:
            # The perpendicular that points away from the horizontal
            # axis, so a diagonal label moves further up or down.
            px, py = (dy, -dx) if dy * dx > 0 else (-dy, dx)
            if py * dy < 0:
                px, py = -px, -py
            length = math.hypot(px, py)
            px, py = px / length, py / length
        moved = QtCore.QRectF(rect)
        for _ in range(40):
            hit = False
            for ax, ay in avoid:
                box = QtCore.QRectF(ax - reach, ay - reach, 2 * reach,
                                    2 * reach)
                if moved.intersects(box):
                    hit = True
                    break
            if not hit:
                break
            moved.translate(px * 4.0, py * 4.0)
        return moved

    def CoveredDirections(self, menu, centre, circles):
        """The directions whose labels, as drawn, would cover one of the
        back circles: the Gesture's `blocker`, so a submenu leaves them
        empty."""
        rects = self.ItemRects(menu, centre)
        return [resolved.direction for resolved in menu.slots.values()
                if self._OnACircle(rects[resolved.id], circles)]

    @staticmethod
    def _OnACircle(rect, avoid):
        reach = mm.BACK_RADIUS_PX + 6.0
        for ax, ay in avoid:
            if rect.intersects(QtCore.QRectF(ax - reach, ay - reach,
                                             2 * reach, 2 * reach)):
                return True
        return False

    def ColumnRects(self, menu, centre, avoid=()):
        """{item id: QRectF} for the overflow column, and the column's own
        box (None when nothing overflows). Below the ring, unless a back
        circle in `avoid` is there: then above it, or beside it."""
        if not menu.overflow:
            return {}, None
        width = max(self._Size(self._Text(r))[0] for r in menu.overflow)
        h = self.base * 1.5
        height = h * len(menu.overflow)
        gap = mm.RADIUS_PX * 0.7 + self.base * 3.0
        cx, cy = centre
        places = [(cx - width * 0.5, cy + gap),                 # below
                  (cx - width * 0.5, cy - gap - height),        # above
                  (cx + mm.RADIUS_PX * 1.6, cy - height * 0.5),  # right
                  (cx - mm.RADIUS_PX * 1.6 - width,              # left
                   cy - height * 0.5)]
        x, y = places[0]
        for px, py in places:
            if not self._OnACircle(QtCore.QRectF(px, py, width, height),
                                   avoid):
                x, y = px, py
                break
        rows = {}
        for i, resolved in enumerate(menu.overflow):
            rows[resolved.id] = QtCore.QRectF(x, y + i * h, width, h)
        box = QtCore.QRectF(x - _BORDER_WIDTH, y - _BORDER_WIDTH,
                            width + 2 * _BORDER_WIDTH,
                            h * len(menu.overflow) + 2 * _BORDER_WIDTH)
        return rows, box

    def ItemAt(self, menu, centre, point, avoid=()):
        """The item whose label is under `point`, or None. The labels win
        over the sectors: a long label reaches into its neighbour's."""
        p = QtCore.QPointF(point[0], point[1])
        rows, _ = self.ColumnRects(menu, centre, avoid)
        for itemId, rect in list(self.ItemRects(menu, centre,
                                                avoid).items()) + \
                list(rows.items()):
            if rect.contains(p):
                return menu.Find(itemId)
        return None

    def Paint(self, painter, menu, centre, lit, cursor, mode, parents=(),
              litBack=None):
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        painter.setFont(self.font)
        cx, cy = centre
        # The way back: a faint trail through every menu above this one,
        # and a circle where each was centred.
        if parents:
            trail = list(parents) + [centre]
            painter.setPen(QtGui.QPen(QtGui.QColor(128, 128, 128, 140), 2.0,
                                      QtCore.Qt.DashLine))
            for a, b in zip(trail, trail[1:]):
                painter.drawLine(QtCore.QPointF(a[0], a[1]),
                                 QtCore.QPointF(b[0], b[1]))
            for index, (px, py) in enumerate(parents):
                hot = index == litBack
                painter.setBrush(QtGui.QBrush(_LIT_FILL if hot else _FILL))
                painter.setPen(QtGui.QPen(_LIT_BORDER if hot else _BORDER,
                                          _BORDER_WIDTH))
                painter.drawEllipse(QtCore.QPointF(px, py),
                                    mm.BACK_RADIUS_PX, mm.BACK_RADIUS_PX)
                # An arrow pointing back, so the circle reads as "return".
                painter.setPen(QtGui.QPen(_LIT_TEXT if hot else _TEXT, 2.0))
                s = mm.BACK_RADIUS_PX * 0.4
                painter.drawPolyline([QtCore.QPointF(px + s * 0.4, py - s),
                                      QtCore.QPointF(px - s * 0.6, py),
                                      QtCore.QPointF(px + s * 0.4, py + s)])
        # The guide from the centre to the cursor, under everything.
        if cursor is not None:
            pen = QtGui.QPen(_GUIDE, 3.0)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QtCore.QPointF(cx, cy),
                             QtCore.QPointF(cursor[0], cursor[1]))
        # The centre mark: a grey ring inside a black one.
        radius = max(6.0, self.base * 0.5)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(_GUIDE, 5.0))
        painter.drawEllipse(QtCore.QPointF(cx, cy), radius * 0.5,
                            radius * 0.5)
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 2.0))
        painter.drawEllipse(QtCore.QPointF(cx, cy), radius * 0.5 + 2.0,
                            radius * 0.5 + 2.0)

        rects = self.ItemRects(menu, centre, parents)
        for resolved in menu.slots.values():
            self._PaintPill(painter, resolved, rects[resolved.id],
                            resolved.id == lit)
        rows, box = self.ColumnRects(menu, centre, parents)
        if box is not None:
            painter.setBrush(QtGui.QBrush(_FILL))
            painter.setPen(QtGui.QPen(_BORDER, _BORDER_WIDTH))
            painter.drawRoundedRect(box, _RADIUS, _RADIUS)
            for resolved in menu.overflow:
                rect = rows[resolved.id]
                if resolved.id == lit and resolved.enabled:
                    painter.setBrush(QtGui.QBrush(_LIT_FILL))
                    painter.setPen(QtCore.Qt.NoPen)
                    painter.drawRect(rect)
                self._PaintLabel(painter, resolved, rect,
                                 resolved.id == lit,
                                 QtCore.Qt.AlignLeft)

    def _PaintPill(self, painter, resolved, rect, lit):
        lit = lit and resolved.enabled
        if not resolved.enabled:
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.setPen(QtCore.Qt.NoPen)
        else:
            painter.setBrush(QtGui.QBrush(_LIT_FILL if lit else _FILL))
            painter.setPen(QtGui.QPen(_LIT_BORDER if lit else _BORDER,
                                      _BORDER_WIDTH))
        painter.drawRoundedRect(rect, _RADIUS, _RADIUS)
        self._PaintLabel(painter, resolved, rect, lit,
                         QtCore.Qt.AlignHCenter)

    def _PaintLabel(self, painter, resolved, rect, lit, align):
        box = QtCore.QRectF(rect).adjusted(self.base * 0.6, 0,
                                           -self.base * 0.6, 0)
        if resolved.checked:
            # A check mark for a toggle or radio that is on.
            mark = box.left() + self.base * 0.45
            pen = QtGui.QPen(_LIT_TEXT if lit else _CHECK, 2.0)
            painter.setPen(pen)
            mid = box.center().y()
            s = self.base * 0.3
            painter.drawPolyline([
                QtCore.QPointF(mark - s, mid),
                QtCore.QPointF(mark - s * 0.3, mid + s * 0.7),
                QtCore.QPointF(mark + s, mid - s * 0.8)])
            box.setLeft(box.left() + self.base)
        colour = (_DISABLED_TEXT if not resolved.enabled
                  else _LIT_TEXT if lit else _TEXT)
        painter.setPen(colour)
        painter.drawText(box, align | QtCore.Qt.AlignVCenter,
                         self._Text(resolved))


class MarkingMenuWindow(QtWidgets.QWidget):
    """The transparent window the menu is painted in. Takes no input."""

    def __init__(self, parent=None):
        flags = (QtCore.Qt.Tool | QtCore.Qt.FramelessWindowHint
                 | QtCore.Qt.WindowStaysOnTopHint
                 | QtCore.Qt.NoDropShadowWindowHint
                 | QtCore.Qt.WindowDoesNotAcceptFocus)
        super(MarkingMenuWindow, self).__init__(parent, flags)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(QtCore.Qt.WA_ShowWithoutActivating, True)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.painter = MenuPainter(QtWidgets.QApplication.font())
        self.menu = None
        self.centre = None      # local logical pixels
        self.lit = None         # item id under the cursor
        self.cursor = None
        self.mode = mm.MODE_FLICK
        self.parents = []       # local centres of the menus above
        self.litBack = None     # index of the back circle under the cursor

    def Cover(self, view):
        """Lay the window over the screen the viewport is on."""
        screen = view.screen() if hasattr(view, "screen") else None
        if screen is None:
            screen = QtGui.QGuiApplication.primaryScreen()
        self.setGeometry(screen.geometry())

    def ToLocal(self, view, point):
        global_ = view.mapToGlobal(QtCore.QPoint(int(round(point[0])),
                                                 int(round(point[1]))))
        local = self.mapFromGlobal(global_)
        # Keep the sub-pixel part of the view point.
        return (local.x() + (point[0] - round(point[0])),
                local.y() + (point[1] - round(point[1])))

    def paintEvent(self, event):
        if self.menu is None or self.centre is None:
            return
        painter = QtGui.QPainter(self)
        try:
            self.painter.Paint(painter, self.menu, self.centre, self.lit,
                               self.cursor, self.mode, self.parents,
                               self.litBack)
        finally:
            painter.end()


# --- The controller ------------------------------------------------------


class ControlMarkingMenu(QtCore.QObject):
    """Owns the right-click gesture over the viewport for one usdview."""

    def __init__(self, usdviewApi, undoStack=None, switchIkFk=None,
                 knownLimbs=None, parent=None):
        super(ControlMarkingMenu, self).__init__(parent)
        self._api = usdviewApi
        self._undo = undoStack
        self._switchIkFk = switchIkFk
        self._knownLimbs = knownLimbs
        self._view = StageView(usdviewApi)
        self._gesture = None
        self._controls = []
        self._pressed = False
        self._window = None
        # The item the gesture's direction last lit, which a label under
        # the cursor overrides.
        self._sectorLit = None
        # Where the last mouse event put the cursor, in viewport pixels,
        # and whether one arrived since the timer last ticked.
        self._lastPoint = None
        self._eventSinceTick = False
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._OnTick)
        self._escape = _EscapeFilter(self)
        # Every command the menu runs, by the `write` name its item
        # carries. Exposed so a test can watch one run.
        self.lastCommand = None
        # The gesture's clock in milliseconds; None is the wall clock. A
        # test sets its own so a tap and a hold do not depend on how fast
        # the machine draws.
        self.clock = None
        if self._view is not None:
            self._view.setAttribute(QtCore.Qt.WA_Hover, True)
            self._view.installEventFilter(self)

    # -- the world ---------------------------------------------------------

    def Stage(self):
        return getattr(self._api, "stage", None)

    def Time(self):
        frame = getattr(self._api, "frame", None)
        if isinstance(frame, Usd.TimeCode):
            return frame
        try:
            return Usd.TimeCode(float(frame))
        except (TypeError, ValueError):
            return Usd.TimeCode.Default()

    def KnownLimbs(self):
        if self._knownLimbs is None:
            return None
        try:
            return self._knownLimbs(self.Stage())
        except Exception:
            return None

    def _Selection(self):
        model = getattr(self._api, "dataModel", None)
        return getattr(model, "selection", None)

    def SelectedControls(self):
        selection = self._Selection()
        if selection is None:
            return []
        return [str(p.GetPath()) for p in selection.getPrims()
                if _IsControl(p)]

    def _Select(self, paths):
        selection = self._Selection()
        stage = self.Stage()
        if selection is None or stage is None:
            return
        prims = [stage.GetPrimAtPath(Sdf.Path(p)) for p in paths]
        batch = getattr(selection, "batchPrimChanges", None)
        if batch is not None:
            with batch:
                selection.clearPrims()
                for prim in prims:
                    if prim and prim.IsValid():
                        selection.addPrim(prim)
        else:
            selection.clearPrims()
            for prim in prims:
                if prim and prim.IsValid():
                    selection.addPrim(prim)

    def IsOpen(self):
        return self._gesture is not None

    # -- what the right-click is about --------------------------------------

    def HoveredControls(self, point):
        """The controls under a viewport point: a TouchPose region's (its
        regions are larger targets for the same controls), else a control
        the viewport picks there. Empty when neither."""
        view = self._view
        ratio = view.devicePixelRatioF() if view is not None else 1.0
        x, y = point[0] * ratio, point[1] * ratio
        try:
            import touchPoseUI
            touch = touchPoseUI.TouchPoseController.ForApi(self._api)
        except Exception:
            touch = None
        if touch is not None and getattr(touch, "active", False):
            try:
                region, face = touch.RegionAt(x, y)
            except Exception:
                region, face = None, -1
            control = getattr(region, "control", None) if region else None
            if control:
                return [str(control)]
        stage = self.Stage()
        if view is None or stage is None:
            return []
        try:
            inBounds, frustum = view.computePickFrustum(x, y)
            hits = view.pick(frustum) if inBounds else []
        except Exception:
            hits = []
        for hit in hits[:1]:
            control = ControlOf(stage.GetPrimAtPath(hit.hitPrimPath))
            if control is not None:
                return [str(control.GetPath())]
        return []

    def Targets(self, point):
        """The controls a right-click at `point` acts on, selecting the
        hovered one first if it is not already selected. Empty when the
        click is about no control, which leaves it to usdview."""
        selected = self.SelectedControls()
        hovered = self.HoveredControls(point)
        if hovered:
            if set(hovered) <= set(selected):
                return selected
            self._Select(hovered)
            return hovered
        return selected

    # -- the gesture --------------------------------------------------------

    def Open(self, point, controls):
        """Start a gesture at a viewport point for `controls`."""
        self._controls = list(controls)
        context = SelectionContext(self, self._controls)
        if self._window is None:
            self._window = MarkingMenuWindow(self._view.window()
                                             if self._view else None)
        self._window.Cover(self._view)
        # The painter tells the gesture which directions would draw a
        # label on a back circle, so a submenu leaves them empty.
        self._gesture = mm.Gesture(
            defs.SELECTION, context, clock=self.clock,
            blocker=self._window.painter.CoveredDirections)
        self._lastPoint = point
        self._Handle(self._gesture.Press(point[0], point[1]))
        self._timer.start()
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.installEventFilter(self._escape)

    def _Close(self):
        self._gesture = None
        self._pressed = False
        self._timer.stop()
        if self._window is not None:
            self._window.hide()
            self._window.menu = None
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.removeEventFilter(self._escape)
        if self._view is not None and \
                QtWidgets.QWidget.mouseGrabber() is self._view:
            self._view.releaseMouse()

    def Cancel(self):
        if self._gesture is not None:
            self._Handle(self._gesture.Cancel())

    def _Handle(self, emissions):
        window = self._window
        for emission in emissions:
            kind = emission.kind
            if kind in (mm.SHOW, mm.OPEN):
                window.menu = emission.menu
                window.centre = window.ToLocal(self._view, emission.centre)
                window.mode = emission.mode
                window.lit = None
                window.litBack = None
                self._sectorLit = None
                window.parents = [window.ToLocal(self._view, c) for c in
                                  self._gesture.parentCentres] \
                    if self._gesture is not None else []
                window.show()
                window.raise_()
                window.update()
                if emission.mode == mm.MODE_CLICK and self._view and \
                        QtWidgets.QWidget.mouseGrabber() is not self._view:
                    # Up for clicking: every click, even one outside the
                    # viewport, is the menu's to take or dismiss.
                    self._view.grabMouse()
            elif kind == mm.HIGHLIGHT:
                self._sectorLit = emission.item.id if emission.item else None
                window.lit = self._sectorLit
                window.update()
            elif kind == mm.CONFIRM:
                item = emission.item
                self._Close()
                self.Run(item)
            elif kind == mm.CANCEL:
                self._Close()

    def _ItemUnder(self, point):
        """The drawn item under a viewport point, or None."""
        window = self._window
        if window is None or not window.isVisible() or window.menu is None:
            return None
        return window.painter.ItemAt(window.menu, window.centre,
                                     window.ToLocal(self._view, point),
                                     window.parents)

    def _MovedTo(self, point):
        window = self._window
        if window is not None and window.isVisible():
            local = window.ToLocal(self._view, point)
            window.cursor = local if window.mode == mm.MODE_FLICK \
                and self._pressed else None
            if window.menu is not None:
                # A label or column row under the cursor is what lights,
                # held or clicked; elsewhere a held stroke lights its
                # direction and a menu up for clicking lights nothing.
                over = self._ItemUnder(point)
                if over is not None:
                    window.lit = over.id
                elif window.mode == mm.MODE_FLICK and self._pressed:
                    window.lit = self._sectorLit
                else:
                    window.lit = None
            window.litBack = self._gesture.BackTarget(point[0], point[1]) \
                if self._gesture is not None else None
            window.update()

    def _OnTick(self):
        if self._gesture is None:
            return
        self._Handle(self._gesture.Tick())
        if self._gesture is None:
            return
        if not self._eventSinceTick and                 self._gesture.state == mm.STATE_CLICK and self._view:
            # Up for clicking, the viewport may report no button-less
            # moves at all: follow the cursor itself.
            local = self._view.mapFromGlobal(QtGui.QCursor.pos())
            point = (local.x(), local.y())
            if point != self._lastPoint:
                self._lastPoint = point
                self._Handle(self._gesture.Move(point[0], point[1]))
        self._eventSinceTick = False
        if self._gesture is not None and self._lastPoint is not None:
            self._MovedTo(self._lastPoint)

    def _ClickAt(self, point):
        """A press while the menu is up for clicking: a label or column
        row under it is chosen outright; anything else is the model's."""
        window = self._window
        if self._gesture.BackTarget(point[0], point[1]) is not None:
            # A back circle outranks any label drawn near it.
            self._Handle(self._gesture.Press(point[0], point[1]))
            return
        over = self._ItemUnder(point)
        if over is not None:
            if over.enabled:
                self._Handle(self._gesture.Choose(over.id))
            return
        self._Handle(self._gesture.Press(point[0], point[1]))

    def eventFilter(self, watched, event):
        if watched is not self._view:
            return False
        kind = event.type()
        if self._gesture is not None:
            if kind in (QtCore.QEvent.MouseButtonPress,
                        QtCore.QEvent.MouseButtonDblClick):
                point = _EventPoint(event)
                if self._gesture.state == mm.STATE_CLICK:
                    self._ClickAt(point)
                else:
                    self._Handle(self._gesture.Press(point[0], point[1]))
                return True
            if kind in (QtCore.QEvent.MouseMove, QtCore.QEvent.HoverMove):
                point = _EventPoint(event)
                self._lastPoint = point
                self._eventSinceTick = True
                self._Handle(self._gesture.Move(point[0], point[1]))
                self._MovedTo(point)
                return True
            if kind == QtCore.QEvent.MouseButtonRelease:
                point = _EventPoint(event)
                self._pressed = False
                over = self._ItemUnder(point)
                self._Handle(self._gesture.Release(
                    point[0], point[1],
                    itemId=over.id if over is not None else None))
                return True
            return False
        if kind == QtCore.QEvent.MouseButtonPress and \
                event.button() == QtCore.Qt.RightButton and \
                not event.modifiers() & _CAMERA_MODIFIERS:
            point = _EventPoint(event)
            controls = self.Targets(point)
            if not controls:
                return False
            self._pressed = True
            self.Open(point, controls)
            return True
        return False

    # -- the commands -------------------------------------------------------

    def Run(self, resolved):
        """Run a confirmed item, by the `write` name it carries."""
        item = resolved.item
        name = item.write
        self.lastCommand = (name, item.value)
        handler = getattr(self, "_Do" + (name or ""), None)
        if handler is None:
            self._Status("%s: not available yet" % resolved.label)
            return
        try:
            handler(item)
        except Exception as error:
            Tf.Warn("rigExecUsdview: %s failed: %s" % (resolved.label, error))
            self._Status("%s failed: %s" % (resolved.label, error))

    def _Status(self, text):
        window = getattr(self._api, "qMainWindow", None)
        bar = window.statusBar() if window is not None else None
        if bar is not None:
            bar.showMessage(text, 6000)

    def _Prims(self):
        stage = self.Stage()
        return [p for p in (stage.GetPrimAtPath(Sdf.Path(c))
                            for c in self._controls) if _IsControl(p)]

    def _WriteMode(self):
        import gizmoMath
        try:
            import gizmoUI
            controller = gizmoUI.GetController(self._api)
            if controller is not None:
                return controller.WriteMode()
        except Exception:
            pass
        return gizmoMath.WRITE_ANIMATION

    def _Write(self, values, mode, label):
        """Author {attribute: value} as one edit and one undo entry."""
        import gizmoMath
        import rigExecUndo
        stage = self.Stage()
        writer = gizmoMath.Writer(stage, self.Time(), mode)
        for attr, value in values:
            writer.Set(attr, value)
        if not writer.HasPending():
            return 0
        scope = None
        if self._undo is not None:
            scope = rigExecUndo.SpecScope(stage, list(writer.Pending()),
                                          self._undo, label)
            scope.__enter__()
        try:
            authored = writer.CommitToStage()
        except Exception as error:
            if scope is not None:
                scope.__exit__(type(error), error, None)
            raise
        if scope is not None:
            scope.__exit__(None, None, None)
        return len(authored)

    def _DoKeySelection(self, item):
        """A key on every posed channel of the controls, at this frame."""
        import avarEditorModel
        import gizmoMath
        time = self.Time()
        values = []
        for prim in self._Prims():
            for attr in avarEditorModel.PoseChannels(prim, self.Stage()):
                value = attr.Get(time)
                if isinstance(value, (int, float)) and \
                        not isinstance(value, bool):
                    values.append((attr, value))
        count = self._Write(values, gizmoMath.WRITE_ANIMATION, "Key")
        self._Status("Keyed %d channels on %d controls" % (
            count, len(self._controls)))

    def _DoResetToRest(self, item):
        """The controls' transform channels back to rest, written the way
        a drag writes (keyed in animation mode)."""
        time = self.Time()
        values = []
        for prim in self._Prims():
            for name, rest in _REST_VALUES.items():
                attr = prim.GetAttribute(name)
                if attr and attr.Get(time) is not None and \
                        attr.Get(time) != rest:
                    values.append((attr, rest))
        count = self._Write(values, self._WriteMode(), "Reset to rest")
        self._Status("Reset %d channels to rest" % count)

    def _DoZeroPose(self, item):
        """Every pose channel cleared back to what the rig says, as the
        picker's Zero Ctrls does: an IK/FK dial, a space and a foot roll
        come back as well as the transforms."""
        import avarEditorModel
        import rigExecUndo
        stage = self.Stage()
        doomed = []
        for prim in self._Prims():
            for attr in avarEditorModel.PoseChannels(prim, stage):
                if attr.HasAuthoredValue():
                    doomed.append(attr.GetPath())
        if not doomed:
            self._Status("Nothing to zero")
            return
        scope = None
        if self._undo is not None:
            scope = rigExecUndo.SpecScope(stage, doomed, self._undo,
                                          "Zero pose")
            scope.__enter__()
        try:
            with Sdf.ChangeBlock():
                for path in doomed:
                    stage.GetAttributeAtPath(path).Clear()
        except Exception as error:
            if scope is not None:
                scope.__exit__(type(error), error, None)
            raise
        if scope is not None:
            scope.__exit__(None, None, None)
        self._Status("Zeroed %d channels" % len(doomed))

    def _DoFrameSelection(self, item):
        main = getattr(self._api, "qMainWindow", None)
        if main is None:
            return
        actionType = getattr(QtGui, "QAction", None) or QtWidgets.QAction
        action = main.findChild(actionType, "actionFrame_Selected")
        if action is not None and action.isEnabled():
            action.trigger()

    def _DoToggleFkIk(self, item):
        if self._switchIkFk is not None:
            self._switchIkFk(self._api)

    def _DoSetSpace(self, item):
        """Switch the lead control's space without moving it."""
        import spaceMatch
        stage = self.Stage()
        context = SelectionContext(self, self._controls)
        space = context.Space()
        if space is None:
            return
        switch, activePath, labels = space
        control = context.controls[-1].GetPath()
        time = self.Time()
        plan = spaceMatch.Plan(stage, control, activePath, item.value, time)
        spaceMatch.Apply(stage, plan, time, self._WriteMode(), self._undo,
                         "Switch %s to %s" % (control.name,
                                              labels[int(item.value)]))
        self._Status("%s -> %s" % (control.name, labels[int(item.value)]))

    def _DoSelectCounterpart(self, item):
        stage = self.Stage()
        found = []
        for path in self._controls:
            other = CounterpartPath(stage, path)
            if other is not None and str(other) not in found:
                found.append(str(other))
        if not found:
            self._Status("No mirrored control for the selection")
            return
        self._Select(found)
        self._Status("Selected %s" % ", ".join(
            Sdf.Path(p).name for p in found))

    def _DoShowUsdviewPrimMenu(self, item):
        controller = _AppController(self._api)
        if controller is None or not self._controls:
            return
        path = Sdf.Path(self._controls[-1])
        try:
            controller._showPrimContextMenu(controller._getItemAtPath(path))
        except Exception as error:
            self._Status("usdview's prim menu is unavailable: %s" % error)


class _EscapeFilter(QtCore.QObject):
    """Escape dismisses the menu wherever focus is. Installed on the
    application only while the menu is live."""

    def __init__(self, menu):
        super(_EscapeFilter, self).__init__(menu)
        self._menu = menu

    def eventFilter(self, obj, event):
        if event.type() in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.ShortcutOverride) and \
                event.key() == QtCore.Qt.Key_Escape and self._menu.IsOpen():
            self._menu.Cancel()
            event.accept()
            return True
        return False


_menus = sessionRegistry.SessionRegistry("control marking menus")


def GetMenu(usdviewApi):
    return _menus.Get(usdviewApi)


def InstallMarkingMenu(usdviewApi, undoStack=None, switchIkFk=None,
                       knownLimbs=None):
    """The session's right-click marking menu, installed once."""
    menu = _menus.Get(usdviewApi)
    if menu is None and StageView(usdviewApi) is not None:
        menu = _menus.Set(usdviewApi, ControlMarkingMenu(
            usdviewApi, undoStack, switchIkFk, knownLimbs))
    return menu
