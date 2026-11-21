"""Qt display widgets and application hotkey filter for the viewport gizmo."""
import math
import os
import sys

from pxr import Tf
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    # PySide6 keeps QAction / QActionGroup in QtGui; usdview's qt shim
    # re-exports whichever module has them as QtActionWidgets.
    from pxr.Usdviewq.qt import QtActionWidgets
except ImportError:                                       # PySide2
    QtActionWidgets = QtWidgets

try:
    import gizmoIcons
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import sessionRegistry
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoIcons
    import gizmoMath
    import gizmoScreen
    import gizmoSettings
    import sessionRegistry


TOOL_SELECT = gizmoSettings.TOOL_SELECT
TOOL_TRANSLATE = gizmoScreen.TOOL_TRANSLATE
TOOL_ROTATE = gizmoScreen.TOOL_ROTATE
TOOL_SCALE = gizmoScreen.TOOL_SCALE

# animator names, not USD's: an animator reaches for "Move", not
# "Translate". The token stays gizmoScreen's so one spelling reaches the
# geometry code.
TOOL_LABELS = {
    TOOL_SELECT: "Select",
    TOOL_TRANSLATE: "Move",
    TOOL_ROTATE: "Rotate",
    TOOL_SCALE: "Scale",
}

# Line width of every handle outline, in LOGICAL pixels (design 8.1).
LINE_WIDTH = 2.0

# Opacity of the pieces the spec dims rather than drops: a locked axis
# (pointing at the camera), a planar square's fill, the rotation pie and
# the free-rotate ball.
LOCKED_OPACITY = 0.4
PLANE_FILL_OPACITY = 0.5
PIE_OPACITY = 0.3
SPHERE_OPACITY = 0.5
# The silhouette is drawn at SPHERE_OPACITY per the spec; the interior
# gets a lighter wash, enough for the ball to read as a disc without
# hiding the three rings drawn inside it. It needs to be there at all
# because the silhouette lands exactly on the axis ring of the same
# radius and is covered by it whenever that ring faces the camera.
SPHERE_FILL_OPACITY = 0.30

# The snap candidate marker: orange reads against the axis primaries,
# the selection yellow and the pale hover highlight alike, so the
# landing preview never hides inside the manipulator's own colours.
COLOR_SNAP = (1.0, 0.55, 0.1)
# Opacity of the leader from the grabbed handle to the candidate.
LEADER_OPACITY = 0.35

# An arrowhead this many times its base radius long, which is the
# proportion that reads clearly at manipulator size.
CONE_LENGTH_RATIO = 3.0

# How near a click has to land, in LOGICAL pixels, before it counts as
# hitting a handle outline.
HIT_PIXELS = gizmoScreen.HIT_PIXELS

def _SnapTooltip(prefix=""):
    """Tooltip of the Move pivot snap button; `prefix` precedes each hold key."""
    return ("Where the Move pivot lands: hold %sX (grid), %sC (edge) or "
            "%sV (point) for one drag, or keep a mode on here. Rotate "
            "offers Grid on a Gimbal ring only." % (prefix, prefix, prefix))


def _Color(rgb, opacity=1.0):
    return QtGui.QColor(int(round(rgb[0] * 255)), int(round(rgb[1] * 255)),
                        int(round(rgb[2] * 255)),
                        int(round(max(0.0, min(1.0, opacity)) * 255)))


# Overlay

class GizmoOverlay(QtWidgets.QWidget):
    """
    The manipulator, painted over the stage view.

    A child widget rather than session-layer prims (design assumption
    1.7): the gizmo has to be screen-constant and unoccluded, and
    authoring geometry for it would re-enter the evaluator on every
    camera move. Mouse-transparent, because the controller's event
    filter on the stage view is what does the picking -- the overlay
    must never take a click that was meant for usdview.
    """

    def __init__(self, controller, parent):
        super(GizmoOverlay, self).__init__(parent)
        self._controller = controller
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        # WA_NoSystemBackground plus no auto-fill is what leaves the
        # stage view showing through; WA_TranslucentBackground is a
        # top-level-window attribute and would do nothing on a child.
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)
        self.setAutoFillBackground(False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

    # -- geometry -------------------------------------------------------

    def _Ratio(self):
        parent = self.parentWidget()
        try:
            return float(parent.devicePixelRatioF()) if parent else 1.0
        except AttributeError:
            return 1.0

    def _Point(self, point, ratio):
        """A physical-pixel handle point as a logical QPointF."""
        return QtCore.QPointF(point[0] / ratio, point[1] / ratio)

    def _Polygon(self, points, ratio):
        return QtGui.QPolygonF([self._Point(p, ratio) for p in points])

    # -- painting -------------------------------------------------------

    def paintEvent(self, event):
        handles = self._controller.Handles()
        if not handles:
            return
        ratio = self._Ratio()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        try:
            tool = self._controller.Tool()
            # The free-rotate ball first so the rings sit on top of it,
            # then rings, then everything else: the spec's draw order is
            # also the hit order, so what looks on top is what is picked.
            for handle in handles:
                if handle.kind == "sphere":
                    self._DrawSphere(painter, handle, ratio)
            for handle in handles:
                if handle.kind in ("ring", "view"):
                    self._DrawRing(painter, handle, ratio)
            self._DrawPie(painter, ratio)
            for handle in handles:
                if handle.kind == "axis":
                    if tool == TOOL_TRANSLATE:
                        self._DrawArrow(painter, handle, ratio)
                    else:
                        self._DrawScaleAxis(painter, handle, ratio)
            for handle in handles:
                if handle.kind == "plane":
                    self._DrawPlane(painter, handle, ratio)
            for handle in handles:
                if handle.kind == "center":
                    self._DrawCenter(painter, handle, ratio)
            for handle in handles:
                if handle.kind == "point":
                    self._DrawPoint(painter, handle, ratio)
            self._DrawSnap(painter, ratio)
        finally:
            painter.end()

    def _Pen(self, handle, width=LINE_WIDTH):
        pen = QtGui.QPen(self._HandleColor(handle))
        pen.setWidthF(width)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        return pen

    def _HandleColor(self, handle, opacity=1.0):
        """
        Three states: the handle under the cursor is the pale
        pre-selection highlight, the last-dragged handle stays yellow
        (middle-drag repeats it), and an ungrabbable one is its own
        colour dimmed so the artist can see it is there and inert.
        """
        controller = self._controller
        if not handle.grabbable:
            return _Color(handle.color, LOCKED_OPACITY * opacity)
        if controller.IsDragging() and \
                controller.SelectedHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_SELECTED, opacity)
        if controller.HoverHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_HOVER, opacity)
        if controller.SelectedHandleName() == handle.name:
            return _Color(gizmoScreen.COLOR_SELECTED, opacity)
        return _Color(handle.color, opacity)

    def _DrawArrow(self, painter, handle, ratio):
        """A move axis: a line stopping short of a filled cone tip."""
        start = self._Point(handle.points[0], ratio)
        end = self._Point(handle.points[-1], ratio)
        dx, dy = end.x() - start.x(), end.y() - start.y()
        length = math.hypot(dx, dy)
        radius = gizmoScreen.CONE_RADIUS * handle.sizePixels / ratio
        coneLength = min(radius * CONE_LENGTH_RATIO, length * 0.9)
        painter.setPen(self._Pen(handle))
        painter.setBrush(QtCore.Qt.NoBrush)
        if length < 1e-6:
            return
        ux, uy = dx / length, dy / length
        base = QtCore.QPointF(end.x() - ux * coneLength,
                              end.y() - uy * coneLength)
        painter.drawLine(start, base)
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawPolygon(QtGui.QPolygonF([
            end,
            QtCore.QPointF(base.x() - uy * radius, base.y() + ux * radius),
            QtCore.QPointF(base.x() + uy * radius, base.y() - ux * radius)]))

    def _DrawScaleAxis(self, painter, handle, ratio):
        """A scale axis: a line ending in a filled cube (a square)."""
        start = self._Point(handle.points[0], ratio)
        end = self._Point(handle.points[-1], ratio)
        side = gizmoScreen.CUBE_SIDE * handle.sizePixels / ratio
        painter.setPen(self._Pen(handle))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLine(start, end)
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(QtCore.QRectF(end.x() - side * 0.5,
                                       end.y() - side * 0.5, side, side))

    def _DrawPlane(self, painter, handle, ratio):
        polygon = self._Polygon(handle.points, ratio)
        painter.setBrush(QtGui.QBrush(
            self._HandleColor(handle, PLANE_FILL_OPACITY)))
        painter.setPen(self._Pen(handle))
        painter.drawPolygon(polygon)

    def _DrawCenter(self, painter, handle, ratio):
        point = self._Point(handle.points[0], ratio)
        side = gizmoScreen.CENTER_SIDE * handle.sizePixels / ratio
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(QtCore.QRectF(point.x() - side * 0.5,
                                       point.y() - side * 0.5, side, side))

    def _DrawPoint(self, painter, handle, ratio):
        """A curve's control point: a filled square with a dark rim."""
        point = self._Point(handle.points[0], ratio)
        side = handle.sizePixels / ratio
        pen = QtGui.QPen(_Color((0.0, 0.0, 0.0), LOCKED_OPACITY))
        pen.setWidthF(1.0)
        painter.setPen(pen)
        painter.setBrush(QtGui.QBrush(self._HandleColor(handle)))
        painter.drawRect(QtCore.QRectF(point.x() - side * 0.5,
                                       point.y() - side * 0.5, side, side))

    def _DrawRing(self, painter, handle, ratio):
        """
        Only frontPoints: the half of each ring behind the
        ring centre so three overlapping circles stay tellable apart,
        and a fully visible run already repeats its first point, so one
        drawPolyline closes it without a chord across the manipulator.
        """
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(self._Pen(handle))
        for run in handle.frontPoints:
            if len(run) > 1:
                painter.drawPolyline(self._Polygon(run, ratio))

    def _DrawSphere(self, painter, handle, ratio):
        point = self._Point(handle.points[0], ratio)
        radius = handle.radiusPixels / ratio
        painter.setBrush(QtGui.QBrush(
            _Color(gizmoScreen.COLOR_SPHERE, SPHERE_FILL_OPACITY)))
        pen = QtGui.QPen(_Color(gizmoScreen.COLOR_SPHERE, SPHERE_OPACITY))
        pen.setWidthF(LINE_WIDTH)
        painter.setPen(pen)
        painter.drawEllipse(point, radius, radius)

    def _DrawSnap(self, painter, ratio):
        """
        The snap landing preview: a diamond plus a crosshair, with the
        kind-specific extra SnapMarker() computed (spec 4.5).
        """
        marker = self._controller.SnapMarker()
        if marker is None:
            return
        center, extras, leader = marker
        pen = QtGui.QPen(_Color(COLOR_SNAP))
        pen.setWidthF(LINE_WIDTH)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        size, arm = 6.0, 11.0
        painter.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(center.x(), center.y() - size),
            QtCore.QPointF(center.x() + size, center.y()),
            QtCore.QPointF(center.x(), center.y() + size),
            QtCore.QPointF(center.x() - size, center.y())]))
        painter.drawLine(QtCore.QPointF(center.x() - arm, center.y()),
                         QtCore.QPointF(center.x() + arm, center.y()))
        painter.drawLine(QtCore.QPointF(center.x(), center.y() - arm),
                         QtCore.QPointF(center.x(), center.y() + arm))
        for start, end in extras:
            painter.drawLine(start, end)
        if leader is not None:
            faint = QtGui.QPen(_Color(COLOR_SNAP, LEADER_OPACITY))
            faint.setWidthF(1.0)
            painter.setPen(faint)
            painter.drawLine(leader, center)

    def _DrawPie(self, painter, ratio):
        """
        The rotation-amount wedge, from where the ring was grabbed to
        where the sweep has reached.
        """
        wedge = self._controller.PieSlice()
        if not wedge:
            return
        polygon, color = wedge
        if len(polygon) < 3:
            return
        painter.setBrush(QtGui.QBrush(_Color(color, PIE_OPACITY)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawPolygon(self._Polygon(polygon, ratio))


# Toolbar

class ViewportStatusBar(QtWidgets.QLabel):
    """
    The gizmo's status line, on its own full-width row under the tools.

    A row of its own because QToolBar moves whatever does not fit into
    an overflow chevron, and at usdview's DEFAULT viewport width the
    status label and the Tool Settings button were already behind it --
    exactly the width at which "why is there no gizmo" and the live
    rotation angle matter most. Elided rather than wrapped so the row
    keeps one constant height and the viewport never jumps as the text
    changes; the whole string stays on the tooltip.
    """

    def __init__(self, parent=None):
        super(ViewportStatusBar, self).__init__(parent)
        self._full = ""
        # fn(text) on every change of the status text; an input owner may
        # show it elsewhere.
        self.statusListeners = []
        self.setContentsMargins(6, 1, 6, 1)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                           QtWidgets.QSizePolicy.Fixed)
        self.setToolTip("What the gizmo is editing, or why there is none.")

    def SetStatus(self, text):
        text = text or ""
        if text != self._full:
            self._full = text
            self.setToolTip(text or "No viewport gizmo.")
            for listener in list(self.statusListeners):
                try:
                    listener(text)
                except Exception as error:
                    Tf.Warn("rigExecUsdview: status listener failed: %s"
                            % error)
        self._Elide()

    def FullText(self):
        """The untruncated status; what a test should assert on."""
        return self._full

    def resizeEvent(self, event):
        super(ViewportStatusBar, self).resizeEvent(event)
        self._Elide()

    def _Elide(self):
        metrics = QtGui.QFontMetrics(self.font())
        width = max(0, self.width() - 12)
        QtWidgets.QLabel.setText(
            self, metrics.elidedText(self._full, QtCore.Qt.ElideRight,
                                     width))


class ViewportToolbar(QtWidgets.QToolBar):
    """
    The strip above the viewport: tools, channel set, write mode, undo,
    the Tool Settings button and the status label that always says why
    there is no gizmo.
    """

    def __init__(self, controller, statusBar, parent=None):
        super(ViewportToolbar, self).__init__("RigExec Viewport Tools",
                                              parent)
        self._controller = controller
        self._status = statusBar
        self.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.setMovable(False)
        self.setFloatable(False)
        # Two things this stylesheet buys: the checked state, which the
        # default style barely distinguishes ("which tool am I in" is the
        # one thing this bar must say), and tight padding, because the row
        # has to FIT usdview's default viewport -- items past it go into
        # QToolBar's overflow chevron where nobody finds them.
        # ICONS, NOT WORDS (gizmoIcons): glyphs are the width we choose, so
        # the row is the same on every platform.
        self.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        self.setIconSize(QtCore.QSize(18, 18))
        # 3px of side padding. Snap's width is pinned to the width of
        # "Snap: Surface" in the PLATFORM's UI font; a font wider than
        # this one is what pushes the Settings gear into the overflow
        # chevron.
        self.setStyleSheet(
            "QToolBar { padding: 0px; spacing: 1px; }"
            " QToolButton { padding: 2px 3px; margin: 0px; }"
            " QToolButton:checked { background: #4879b4; color: white;"
            " border: 1px solid #79a6dc; border-radius: 3px; }")
        self.setContentsMargins(0, 0, 0, 0)
        # Set when Qt destroys the bar. An owner may reparent the bar, so
        # it can die before the stage view, and a late undo-stack or
        # selection notice must then not touch its deleted actions.
        self._dead = [False]
        self.destroyed.connect(
            lambda *args, dead=self._dead: dead.__setitem__(0, True))

        self._toolActions = {}
        self._channelActions = {}
        self._writeActions = {}
        self._orientAction = None
        self._groupAction = None
        self._BuildTools()
        # Beside the manipulator buttons, with no separator between: the
        # question "which axes are these handles on" belongs to the tool
        # buttons, not to the channel set two groups along, and the row
        # has no room for a separator that says nothing.
        self._BuildOrientation()
        self.addSeparator()
        self._BuildChannels()
        self.addSeparator()
        self._BuildWrite()
        self.addSeparator()
        self._BuildSnap()
        self._BuildGroupPivot()
        # No separator of its own: Snap shares this one with Undo,
        # saving 7 px the default-width row cannot spare.
        self._BuildUndo()
        self.addSeparator()
        self._BuildSettingsButton()
        self._BuildGraphButton()

        # No expanding spacer before the label: QToolBar moves whatever
        # does not fit into an overflow menu, and a right-aligned status
        # label is the first thing to disappear on a narrow viewport --
        # which is exactly when an artist needs to read it.
    def _AddChecked(self, group, text, tooltip, checked, handler, icon=None):
        # The text is kept even though the button shows only the icon: it is
        # what the overflow menu, the accessibility tree and the tests read.
        action = QtActionWidgets.QAction(text, self)
        if icon is not None:
            action.setIcon(gizmoIcons.Icon(icon))
        action.setCheckable(True)
        action.setChecked(checked)
        action.setToolTip(tooltip)
        group.addAction(action)
        self.addAction(action)
        action.triggered.connect(handler)
        return action

    def _BuildTools(self):
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            TOOL_SELECT: "Select (Q): no manipulator; usdview picking "
                         "only.",
            TOOL_TRANSLATE: "Move (W): drag an axis, a planar square or "
                            "the light-blue centre. Ctrl+axis moves in "
                            "the perpendicular plane.",
            TOOL_ROTATE: "Rotate (E): drag a ring, the outer view ring, "
                         "or inside the ball to free-rotate.",
            TOOL_SCALE: "Scale (R): drag a cube for one axis, a planar "
                        "square for two, the centre cube for uniform.",
        }
        glyphs = {TOOL_SELECT: "select", TOOL_TRANSLATE: "move",
                  TOOL_ROTATE: "rotate", TOOL_SCALE: "scale"}
        for tool in (TOOL_SELECT, TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE):
            self._toolActions[tool] = self._AddChecked(
                group, TOOL_LABELS[tool], tips[tool],
                tool == self._controller.Tool(),
                lambda checked=False, t=tool: self._onTool(t),
                icon=glyphs[tool])

    def _BuildOrientation(self):
        """
        ONE button, Global <-> Local, beside the manipulator buttons.

        It writes the ACTIVE TOOL's `orientation` field -- the same field
        the Tool Settings panel's Axis Orientation combo box writes -- so
        there is one answer to "which axes am I on" and flipping either
        control moves the other. The panel keeps Parent and Gimbal, which
        are not everyday switches; the bar carries the two that are.

        ONE BUTTON RATHER THAN A PAIR, for the reason the whole row is
        glyphs (see the class comment): QToolBar folds what does not fit
        into an overflow chevron from the END, and the row was measured
        against usdview's default ~598 logical px. A two-state pair costs
        twice the width to say one thing. This shows the mode it is IN
        -- like an axis-orientation button -- and clicking
        moves to the other.

        For ONE selected prim these mean what they always have: Global
        ("World") draws on the world axes, Local ("Object")
        on the prim's own posed frame. For a MULTI-selection Local means
        the LAST-SELECTED control's frame, which is the usual answer and
        the only stable one; it changes the AXES only. WHERE
        a group turns is a separate setting (Tool Settings > Group
        Pivot) and defaults to the selection centre either way.
        """
        self._orientAction = QtActionWidgets.QAction("Global", self)
        self._orientAction.setCheckable(True)
        self._orientAction.triggered.connect(self._onOrientationToggle)
        self.addAction(self._orientAction)
        self._SyncOrientation()

    def _BuildChannels(self):
        # No section label: the row already overflows at usdview's
        # default viewport width (glFrame 598 logical px), and the
        # Pose/Pivot tooltips below name the group already.
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            gizmoMath.CHANNELS_POSE:
                "Pose: edit the animation channels (avars for a rig "
                "prim, the xformOps for a plain xform).",
            gizmoMath.CHANNELS_PIVOT:
                "Pivot (D / Insert): edit the rest offset a rig prim's "
                "avars ride on, or a plain xform's pivot.",
        }
        for channels, label, glyph in (
                (gizmoMath.CHANNELS_POSE, "Pose", "pose"),
                (gizmoMath.CHANNELS_PIVOT, "Pivot", "pivot")):
            self._channelActions[channels] = self._AddChecked(
                group, label, tips[channels],
                channels == self._controller.Channels(),
                lambda checked=False, c=channels: self._onChannels(c),
                icon=glyph)

    def _BuildWrite(self):
        # No section label either (see _BuildChannels): the
        # Animation/Default tooltips carry the meaning, and the
        # separators still delimit the groups.
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        tips = {
            gizmoMath.WRITE_ANIMATION:
                "Animation: author at the current frame (a spline knot, "
                "or a time sample where one already exists).",
            gizmoMath.WRITE_DEFAULT:
                "Default: author the attribute's default value. A "
                "spline or time samples outrank it; the status label "
                "warns when that happens.",
        }
        for mode, label, glyph in (
                (gizmoMath.WRITE_ANIMATION, "Animation", "animation"),
                (gizmoMath.WRITE_DEFAULT, "Default", "default")):
            self._writeActions[mode] = self._AddChecked(
                group, label, tips[mode], mode == self._controller.WriteMode(),
                lambda checked=False, m=mode: self._onWrite(m),
                icon=glyph)

    def _BuildSnap(self):
        # One button, not four toggles: the row already overflows at
        # usdview's default viewport width, and no test would catch
        # that (the end-to-end test resizes wide first). The menu holds
        # the mutually exclusive modes; the button text names the one
        # actually in force, holds included.
        self._snapButton = QtWidgets.QToolButton(self)
        # The one button that keeps its words. Every other label on this row
        # names what the button IS, which a glyph can say; this one names the
        # mode in force -- "Snap: Surface" -- and a glyph would have to change
        # per mode to say the same thing, which is five glyphs to answer a
        # question the text already answers.
        self._snapButton.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self._snapButton.setPopupMode(
            QtWidgets.QToolButton.InstantPopup)
        self._snapButton.setToolTip(_SnapTooltip())
        self._snapMenu = QtWidgets.QMenu(self._snapButton)
        group = QtActionWidgets.QActionGroup(self._snapMenu)
        group.setExclusive(True)
        self._snapActions = {}
        for mode in (gizmoSettings.SNAP_OFF, gizmoSettings.SNAP_GRID,
                     gizmoSettings.SNAP_POINT, gizmoSettings.SNAP_EDGE,
                     gizmoSettings.SNAP_SURFACE):
            action = QtActionWidgets.QAction(
                gizmoSettings.SnapLabel(mode), self._snapMenu)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked=False, m=mode: self._onSnap(m))
            group.addAction(action)
            self._snapMenu.addAction(action)
            self._snapActions[mode] = action
        self._snapButton.setMenu(self._snapMenu)
        self.addWidget(self._snapButton)
        # Pin the width to the widest label ("Snap: Surface"): the
        # button text shortens to bare "Snap" when off (see
        # _SyncSnap), and without this the row reflows and Redo drops
        # into the chevron the moment the artist picks Surface. Sized
        # from the button's own sizeHint so style padding and the
        # popup arrow are counted exactly.
        self._snapButton.setText(
            "Snap: %s" % gizmoSettings.SnapLabel(
                gizmoSettings.SNAP_SURFACE))
        self._snapButton.setMinimumWidth(
            self._snapButton.sizeHint().width())

    def _BuildGroupPivot(self):
        """
        WHERE a MULTI-selection turns, on the bar rather than in a panel.

        This started life as a combo box in Tool Settings, next to the
        Axis Orientation combo -- and that is exactly the control the
        artist could not find, which is why Global/Local is on the bar
        now. Burying the pivot mode in the same drawer cost them the
        same afternoon twice, so it is here.

        A CYCLING ICON, not a text button with a menu, and the row's
        width is the whole reason. Measured inside usdview at its
        default layout: the viewport frame is 601 logical px and the row
        was 640 with this drawn as "Group: Centre" -- QToolBar folds
        the overflow from the END, so the Settings gear and the Graph
        button went behind a chevron nobody thinks to open, which is the
        exact failure the class comment above already records once. One
        glyph is 31 px against that button's 96, and three glyphs that
        differ in one mark say "same question, different answer" better
        than three words do.

        It writes the ACTIVE TOOL's `groupPivot` field, the same field
        the Tool Settings combo writes, so the two cannot disagree.
        Disabled rather than hidden where it means nothing (Move and
        Select, per gizmoSettings.GroupPivotChoices): a control that
        comes and goes reflows the row, and reflow is what puts things
        in the chevron.
        """
        self._groupAction = QtActionWidgets.QAction("Group Pivot", self)
        self._groupAction.setCheckable(True)
        self._groupAction.triggered.connect(self._onGroupPivotCycle)
        self.addAction(self._groupAction)
        self._SyncGroupPivot()

    _GROUP_GLYPHS = {
        gizmoMath.GROUP_PIVOT_CENTER: "groupCentre",
        gizmoMath.GROUP_PIVOT_LEAD: "groupLead",
        gizmoMath.GROUP_PIVOT_INDIVIDUAL: "groupEach",
    }

    def _SyncGroupPivot(self):
        """
        Mirror the active tool's group pivot onto the button.

        The GLYPH and the action text always name the REAL setting, even
        when one prim is selected and the setting is therefore inert --
        the same honesty the orientation button owes for Parent and
        Gimbal. What changes with the selection is the TOOLTIP, which
        says whether it is doing anything right now; the status line
        names a non-default pivot too (_GroupPivotClause).

        Checked means "not the default": the highlight is the bar's way
        of saying an option is in force, and Selection Centre is what a
        fresh selection does.
        """
        if self._groupAction is None:
            return
        controller = self._controller
        tool = controller.Tool()
        choices = gizmoSettings.GroupPivotChoices(tool)
        mode = controller.settings.For(tool).groupPivot
        self._groupAction.setEnabled(bool(choices))
        self._groupAction.setChecked(
            bool(choices) and mode != gizmoMath.GROUP_PIVOT_CENTER)
        self._groupAction.setText(
            "Group Pivot: %s" % gizmoSettings.GroupPivotLabel(mode))
        self._groupAction.setIcon(gizmoIcons.Icon(
            self._GROUP_GLYPHS.get(mode, "groupCentre")))
        if not choices:
            self._groupAction.setToolTip(
                "Group Pivot (Shift+P): where several selected controls move, "
                "turn and scale about. Not used by Select.")
            return
        if tool == TOOL_TRANSLATE:
            meaning = {
                gizmoMath.GROUP_PIVOT_CENTER:
                    "move together by the same distance in the same "
                    "direction",
                gizmoMath.GROUP_PIVOT_INDIVIDUAL:
                    "each move the same distance along its OWN axes",
            }.get(mode, "move together")
            target = controller.Target()
            count = (len(self._controller._TargetPrimPaths(target))
                     if target is not None else 0)
            active = ("Acting on %d controls." % count if count > 1
                      else "Inert until more than one control is "
                      "selected.")
            self._groupAction.setToolTip(
                "Group Pivot: %s, several selected controls %s. %s "
                "Click or press Shift+P for the next one."
                % (gizmoSettings.GroupPivotLabel(mode), meaning, active))
            return
        meaning = {
            gizmoMath.GROUP_PIVOT_CENTER:
                "about the CENTRE of the selection",
            gizmoMath.GROUP_PIVOT_LEAD:
                "about the LAST-SELECTED control, which stays put",
            gizmoMath.GROUP_PIVOT_INDIVIDUAL:
                "each about its OWN origin -- a chain curls instead of "
                "swinging",
        }[mode]
        target = controller.Target()
        count = (len(self._controller._TargetPrimPaths(target))
                 if target is not None else 0)
        active = ("Acting on %d controls." % count if count > 1
                  else "Inert until more than one control is selected.")
        self._groupAction.setToolTip(
            "Group Pivot: %s, several selected controls turn %s. %s "
            "Click or press Shift+P for the next one."
            % (gizmoSettings.GroupPivotLabel(mode), meaning, active))

    def _onGroupPivotCycle(self, checked=False):
        # The CONTROLLER picks what comes next, not the checkbox state:
        # this cycles three modes, not two, and Sync puts the widget
        # back in step whatever the click left it in.
        self._controller.CycleGroupPivot()
        self._SyncGroupPivot()

    def _BuildUndo(self):
        # Qt.ApplicationShortcut so undo works with focus in the prim
        # tree or the attribute view, not only over the viewport.
        # An input owner (installFilters=False) owns the Undo / Redo chords:
        # the buttons stay but carry no shortcuts.
        managed = not getattr(self._controller, "InstallsFilters",
                              lambda: True)()
        self.undoAction = QtActionWidgets.QAction("Undo", self)
        self.undoAction.setIcon(gizmoIcons.Icon("undo"))
        if not managed:
            self.undoAction.setShortcut(QtGui.QKeySequence("Ctrl+Z"))
            self.undoAction.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.undoAction.triggered.connect(
            lambda checked=False: self._controller._TriggerUndoRedo("Undo"))
        self.addAction(self.undoAction)

        self.redoAction = QtActionWidgets.QAction("Redo", self)
        self.redoAction.setIcon(gizmoIcons.Icon("redo"))
        # Ctrl+Shift+Z is the user's ask, Shift+Z is a common redo, Ctrl+Y is
        # what a Windows-trained hand reaches for.
        if not managed:
            self.redoAction.setShortcuts([QtGui.QKeySequence("Ctrl+Shift+Z"),
                                          QtGui.QKeySequence("Shift+Z"),
                                          QtGui.QKeySequence("Ctrl+Y")])
            self.redoAction.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.redoAction.triggered.connect(
            lambda checked=False: self._controller._TriggerUndoRedo("Redo"))
        self.addAction(self.redoAction)

    def _BuildSettingsButton(self):
        self.settingsAction = QtActionWidgets.QAction("Settings…", self)
        self.settingsAction.setIcon(gizmoIcons.Icon("settings"))
        self.settingsAction.setToolTip(
            "Axis orientation, snapping and the manipulator size for "
            "the active tool.")
        self.settingsAction.triggered.connect(
            lambda checked=False: self._controller.ShowToolSettings())
        self.addAction(self.settingsAction)

    def _BuildGraphButton(self):
        self.graphAction = QtActionWidgets.QAction("Graph…", self)
        self.graphAction.setIcon(gizmoIcons.Icon("graph"))
        self.graphAction.setToolTip(
            "Open the Graph Editor: the selected attributes' animation "
            "curves, their keys and their tangents.")
        self.graphAction.triggered.connect(
            lambda checked=False: self._controller.OpenGraphEditor())
        self.addAction(self.graphAction)

    # -- slots ----------------------------------------------------------

    def _onTool(self, tool):
        self._controller.SetTool(tool)

    def _onChannels(self, channels):
        self._controller.SetChannels(channels)

    def _onWrite(self, mode):
        self._controller.SetWriteMode(mode)

    def _onOrientationToggle(self, checked=False):
        # The CONTROLLER decides what comes next, not the button's new
        # checked state: the button can be showing Parent or Gimbal,
        # where "the other one" is not the opposite of a checkbox. Sync
        # puts the widget back in step whatever happens.
        self._controller.ToggleOrientation()
        self._SyncOrientation()

    def _onSnap(self, mode):
        self._controller.settings.For(
            self._controller.Tool()).snapMode = mode

    # -- refresh --------------------------------------------------------

    def Sync(self):
        """Mirror the controller's state onto the widgets."""
        # Test stand-ins borrow this method without running __init__
        # (tests/python/test_gizmo_deferred_release.py), so the shutdown
        # guard and _SyncNow are reached without relying on either.
        dead = getattr(self, "_dead", None)
        if dead is not None and dead[0]:
            return
        try:
            ViewportToolbar._SyncNow(self)
        except RuntimeError as error:
            if "already deleted" not in str(error):
                raise
            # Qt deleted the bar or the status row (before the stage view,
            # at shutdown): nothing left to mirror on.
            if dead is not None:
                dead[0] = True

    def _SyncNow(self):
        controller = self._controller
        for tool, action in self._toolActions.items():
            action.setChecked(tool == controller.Tool())
        for channels, action in self._channelActions.items():
            action.setChecked(channels == controller.Channels())
        for mode, action in self._writeActions.items():
            action.setChecked(mode == controller.WriteMode())
        self._SyncOrientation()
        self._SyncSnap()
        self._SyncGroupPivot()
        stack = controller.undoStack
        # Greyed out during a drag as well as when empty: the actions
        # are application shortcuts and would otherwise fire with a
        # mouse button down (see GizmoController.Undo).
        dragging = controller.IsDragging()
        releasedLabel = controller.ReleasedEditLabel()
        self.undoAction.setEnabled((stack.CanUndo() or releasedLabel is not None)
                                   and not dragging)
        self.redoAction.setEnabled(stack.CanRedo() and not dragging)
        self.undoAction.setToolTip(
            "Undo %s (Ctrl+Z)" % (releasedLabel or stack.UndoText())
            if releasedLabel is not None or stack.CanUndo()
            else "Nothing to undo (Ctrl+Z)")
        self.redoAction.setToolTip(
            "Redo %s (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)" % stack.RedoText()
            if stack.CanRedo()
            else "Nothing to redo (Ctrl+Shift+Z, Shift+Z, Ctrl+Y)")
        self._status.SetStatus(controller.Status())

    def _SyncOrientation(self):
        """
        Put the button on the mode actually in force.

        The LABEL and the GLYPH name the current mode, and the checked
        highlight means Local -- so the bar answers "which axes am I on"
        at a glance, and the tooltip answers "and what does clicking do".
        Parent and Gimbal, which only the panel can set, are named in the
        tooltip and leave the button unchecked on the world glyph rather
        than being silently rounded to Global: the bar must not claim a
        mode the gizmo is not in.
        """
        if self._orientAction is None:
            return
        controller = self._controller
        tool = controller.Tool()
        choices = gizmoSettings.OrientationChoices(tool)
        current = controller.Orientation()
        # Select has no manipulator and nothing to orient.
        self._orientAction.setEnabled(bool(choices))
        local = bool(choices) and current == gizmoSettings.ORIENT_OBJECT
        known = current in gizmoSettings.ORIENT_TOGGLE
        self._orientAction.setChecked(local)
        self._orientAction.setText(
            gizmoSettings.ToggleLabel(current) if known
            else gizmoSettings.OrientationLabel(current))
        self._orientAction.setIcon(gizmoIcons.Icon(
            "local" if local else "global"))
        if not choices:
            self._orientAction.setToolTip(
                "Axis Orientation (L): pick Move, Rotate or Scale first.")
            return
        if known:
            here = ("Local: the handles are on the prim's own posed "
                    "frame -- for several selected prims, on the "
                    "last-selected one's." if local
                    else "Global: the handles are on the WORLD axes.")
            there = "Global" if local else "Local"
        else:
            here = ("Axis Orientation is %s, set in Tool Settings."
                    % gizmoSettings.OrientationLabel(current))
            there = "Global"
        self._orientAction.setToolTip(
            "%s Click or press L for %s. The values written are always "
            "the prim's own channels." % (here, there))

    def _SyncSnap(self):
        """Mirror the sticky snap mode onto the Snap: button."""
        controller = self._controller
        mode = controller.SnapMode()
        # Bare "Snap" when off saves 19 px the default-width row
        # needs; the width stays pinned to "Snap: Surface" (see
        # _BuildSnap) so arming a mode never reflows the row.
        self._snapButton.setText(
            "Snap" if mode == gizmoSettings.SNAP_OFF
            else "Snap: %s" % gizmoSettings.SnapLabel(mode))
        choices = gizmoSettings.SnapChoices(controller.Tool())
        self._snapButton.setEnabled(bool(choices))
        sticky = controller.settings.For(controller.Tool()).snapMode
        for snap, action in self._snapActions.items():
            action.setVisible(snap in choices)
            action.setChecked(snap == sticky)

    def StatusLabel(self):
        """The status row widget (a ViewportStatusBar, not in the bar)."""
        return self._status

    def SetToolButtonsVisible(self, visible):
        """Show or hide the Select / Move / Rotate / Scale buttons.

        Hidden while a host picks the tool elsewhere: a second set of
        tool buttons could disagree with it."""
        for action in self._toolActions.values():
            action.setVisible(bool(visible))

    def ToolButtonsVisible(self):
        return any(a.isVisible() for a in self._toolActions.values())

    def SetSnapHoldPrefix(self, prefix):
        """Prefix the hold keys named in the snap button's tooltip
        (for example "Control+")."""
        self._snapButton.setToolTip(_SnapTooltip(prefix))


# Tool settings window

class ToolSettingsPanel(QtWidgets.QWidget):
    """
    The Tool Settings for the active tool (design spec 8.6).

    One window per session, parented to usdview's main window exactly
    like VolumeWeightPanel, and rebuilt whenever the tool changes so it
    only ever shows rows that mean something for the tool in hand.
    """

    # One panel per usdview session, filed under its main window: several
    # sessions can share this module in one process.
    _sessions = sessionRegistry.SessionRegistry("gizmo tool settings")

    @classmethod
    def GetInstance(cls, controller):
        panel = cls._sessions.Get(controller.usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(controller.usdviewApi,
                                      ToolSettingsPanel(controller))
        return panel

    @classmethod
    def ForgetInstance(cls, usdviewApi, panel):
        if cls._sessions.Get(usdviewApi) is panel:
            cls._sessions.Pop(usdviewApi)

    def __init__(self, controller):
        super(ToolSettingsPanel, self).__init__(
            controller.usdviewApi.qMainWindow, QtCore.Qt.WindowType.Window)
        self._controller = controller
        self._tool = None
        self._updating = False
        self.setWindowTitle("RigExec: Tool Settings")
        # A floor, not a fixed size: adjustSize() after each Rebuild
        # fits the height to the tool's rows, and without the floor it
        # also shrinks the width until "Prevent Negative Scale" clips.
        self.setMinimumWidth(360)
        self.resize(360, 300)

        outer = QtWidgets.QVBoxLayout()
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)
        self.setLayout(outer)

        self._title = QtWidgets.QLabel("")
        font = self._title.font()
        font.setBold(True)
        self._title.setFont(font)
        outer.addWidget(self._title)

        self._body = QtWidgets.QWidget()
        self._form = QtWidgets.QFormLayout()
        self._form.setContentsMargins(0, 0, 0, 0)
        self._body.setLayout(self._form)
        outer.addWidget(self._body)
        outer.addStretch(1)

        self._resetButton = QtWidgets.QPushButton("Reset Tool")
        self._resetButton.setToolTip(
            "Restore this tool's defaults.")
        self._resetButton.clicked.connect(self._onReset)
        outer.addWidget(self._resetButton)

        self.Rebuild()

    # -- construction ---------------------------------------------------

    def _Clear(self):
        while self._form.count():
            item = self._form.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

    def Rebuild(self):
        """
        Build the rows for the active tool. Called on every tool change;
        cheap enough to do wholesale, and wholesale is what keeps a
        stale Rotate-only row from surviving into the Scale tool.
        """
        controller = self._controller
        tool = controller.Tool()
        self._tool = tool
        self._title.setText("%s Tool" % TOOL_LABELS.get(tool, tool))
        self._Clear()
        if tool == TOOL_SELECT:
            note = QtWidgets.QLabel(
                "The Select tool has no manipulator. Pick a prim in the "
                "viewport, then choose Move, Rotate or Scale.")
            note.setWordWrap(True)
            self._form.addRow(note)
        else:
            self._orientation = QtWidgets.QComboBox()
            self._orientation.setToolTip(
                "Which axes the handles are drawn along. The values "
                "written are always this prim's own channels.")
            for choice in gizmoSettings.OrientationChoices(tool):
                self._orientation.addItem(
                    gizmoSettings.OrientationLabel(choice), choice)
            self._orientation.currentIndexChanged.connect(
                self._onOrientation)
            self._form.addRow("Axis Orientation", self._orientation)

            self._stepSnap = QtWidgets.QCheckBox("Step Snap")
            self._stepSnap.setToolTip(
                "Quantise the drag to whole steps, relative to where it "
                "started. Hold J for the same thing during one drag.")
            self._stepSnap.toggled.connect(self._onStepSnap)
            self._form.addRow("", self._stepSnap)

            self._stepSize = QtWidgets.QDoubleSpinBox()
            self._stepSize.setRange(0.0001, 100000.0)
            self._stepSize.setDecimals(4)
            self._stepSize.setSingleStep(
                5.0 if tool == TOOL_ROTATE else 0.5)
            self._stepSize.setToolTip(
                "Degrees per step." if tool == TOOL_ROTATE
                else "Units per step.")
            self._stepSize.valueChanged.connect(self._onStepSize)
            self._form.addRow("Step Size", self._stepSize)

            snapChoices = gizmoSettings.SnapChoices(tool)
            if snapChoices:
                self._snapMode = QtWidgets.QComboBox()
                self._snapMode.setToolTip(
                    "Where the pivot lands. Hold X (grid), C (edge) "
                    "or V (point) for the same thing during one "
                    "drag.")
                for choice in snapChoices:
                    self._snapMode.addItem(
                        gizmoSettings.SnapLabel(choice), choice)
                self._snapMode.currentIndexChanged.connect(
                    self._onSnapMode)
                self._form.addRow("Snap To", self._snapMode)
                if gizmoSettings.SNAP_GRID in snapChoices:
                    self._gridSize = QtWidgets.QDoubleSpinBox()
                    self._gridSize.setRange(
                        gizmoSettings.GRID_SIZE_MIN,
                        gizmoSettings.GRID_SIZE_MAX)
                    self._gridSize.setDecimals(4)
                    self._gridSize.setSingleStep(0.5)
                    self._gridSize.setToolTip(
                        "World units per grid line for Move grid "
                        "snapping. Rotate's grid is degrees and uses "
                        "Step Size instead.")
                    self._gridSize.valueChanged.connect(
                        self._onGridSize)
                    self._form.addRow("Grid Size", self._gridSize)
            if tool == TOOL_ROTATE and snapChoices:
                note = QtWidgets.QLabel(
                    "Point, edge and surface snapping are Move-only. "
                    "Grid lands the dragged Gimbal ring's channel on "
                    "a multiple of Step Size.")
                note.setWordWrap(True)
                self._form.addRow(note)

            if tool == TOOL_ROTATE:
                self._freeRotate = QtWidgets.QCheckBox("Free Rotate")
                self._freeRotate.setToolTip(
                    "Draw the trackball, and let a drag inside it turn "
                    "the object about any axis.")
                self._freeRotate.toggled.connect(self._onFreeRotate)
                self._form.addRow("", self._freeRotate)

            if tool == TOOL_SCALE:
                self._preventNegative = QtWidgets.QCheckBox(
                    "Prevent Negative Scale")
                self._preventNegative.setToolTip(
                    "Stop a drag through the origin from mirroring the "
                    "object; the scale is clamped just above zero.")
                self._preventNegative.toggled.connect(
                    self._onPreventNegative)
                self._form.addRow("", self._preventNegative)

            groupChoices = gizmoSettings.GroupPivotChoices(tool)
            if groupChoices:
                self._groupPivot = QtWidgets.QComboBox()
                self._groupPivot.setToolTip(
                    "What SEVERAL selected controls turn about. Inert "
                    "for a single selection.")
                for choice in groupChoices:
                    self._groupPivot.addItem(
                        gizmoSettings.GroupPivotLabel(choice), choice)
                self._groupPivot.currentIndexChanged.connect(
                    self._onGroupPivot)
                self._form.addRow("Group Pivot", self._groupPivot)

            self._preserveChildren = QtWidgets.QCheckBox(
                "Preserve Children")
            self._preserveChildren.toggled.connect(self._onPreserveChildren)
            self._form.addRow("", self._preserveChildren)

        self._size = QtWidgets.QDoubleSpinBox()
        self._size.setRange(gizmoSettings.MANIPULATOR_SIZE_MIN,
                            gizmoSettings.MANIPULATOR_SIZE_MAX)
        self._size.setDecimals(0)
        self._size.setSingleStep(10.0)
        self._size.setToolTip(
            "How big the manipulator is drawn, in pixels. + and - over "
            "the viewport change it by 10%.")
        self._size.valueChanged.connect(self._onSize)
        self._form.addRow("Manipulator Size", self._size)

        self.Sync()
        # The row count changes with the tool; without this the window
        # keeps its first tool's height and shows an empty band.
        self.adjustSize()
        self.resize(max(self.width(), self.minimumWidth()), self.height())

    # -- refresh --------------------------------------------------------

    def Sync(self):
        """
        Push the model's values onto the widgets.

        `_updating` guards the round trip: every widget writes to the
        settings, the settings notify the controller, and the controller
        syncs the panel back. Without the guard a spin box would fight
        its own clamped value.
        """
        controller = self._controller
        if controller.Tool() != self._tool:
            self.Rebuild()
            return
        settings = controller.settings.For(self._tool)
        self._updating = True
        try:
            if self._tool != TOOL_SELECT:
                index = self._orientation.findData(settings.orientation)
                if index >= 0:
                    self._orientation.setCurrentIndex(index)
                self._stepSnap.setChecked(bool(settings.stepSnap))
                self._stepSize.setValue(float(settings.stepSize))
                snapChoices = gizmoSettings.SnapChoices(self._tool)
                if snapChoices:
                    index = self._snapMode.findData(
                        settings.snapMode)
                    if index >= 0:
                        self._snapMode.setCurrentIndex(index)
                    if gizmoSettings.SNAP_GRID in snapChoices:
                        self._gridSize.setValue(
                            float(controller.settings.gridSize))
                if self._tool == TOOL_ROTATE:
                    self._freeRotate.setChecked(bool(settings.freeRotate))
                if self._tool == TOOL_SCALE:
                    self._preventNegative.setChecked(
                        bool(settings.preventNegativeScale))
                if gizmoSettings.GroupPivotChoices(self._tool):
                    index = self._groupPivot.findData(settings.groupPivot)
                    if index >= 0:
                        self._groupPivot.setCurrentIndex(index)
                self._SyncPreserveChildren(settings)
            self._size.setValue(float(controller.settings.manipulatorSize))
        finally:
            self._updating = False

    def _SyncPreserveChildren(self, settings):
        """
        The checkbox is disabled, and says why, for a target that cannot
        honour it -- a rig control's children are placed by the
        evaluator, so compensating them here would fight the rig.
        """
        target = self._controller.Target()
        supported = bool(target is not None
                         and target.supportsPreserveChildren)
        self._preserveChildren.setEnabled(supported)
        self._preserveChildren.setChecked(
            bool(settings.preserveChildren) and supported)
        if supported:
            skipped = self._controller.SkippedChildren()
            tip = ("Re-author the children after the drag so they keep "
                   "their world transforms.")
            if skipped:
                tip += "\n\nNot preserved:\n- " + "\n- ".join(skipped)
            self._preserveChildren.setToolTip(tip)
        elif target is None:
            self._preserveChildren.setToolTip("No editable prim selected.")
        else:
            self._preserveChildren.setToolTip(
                target.preserveChildrenReason
                or "Not available for this prim.")

    # -- slots ----------------------------------------------------------

    def _Settings(self):
        return self._controller.settings.For(self._tool)

    def _onOrientation(self, index):
        if self._updating or index < 0:
            return
        self._Settings().orientation = self._orientation.itemData(index)

    def _onGroupPivot(self, index):
        if self._updating or index < 0:
            return
        self._Settings().groupPivot = self._groupPivot.itemData(index)

    def _onStepSnap(self, checked):
        if not self._updating:
            self._Settings().stepSnap = bool(checked)

    def _onStepSize(self, value):
        if not self._updating:
            self._Settings().stepSize = float(value)

    def _onSnapMode(self, index):
        if self._updating or index < 0:
            return
        self._Settings().snapMode = self._snapMode.itemData(index)

    def _onGridSize(self, value):
        if not self._updating:
            self._controller.settings.gridSize = float(value)

    def _onFreeRotate(self, checked):
        if not self._updating:
            self._Settings().freeRotate = bool(checked)

    def _onPreventNegative(self, checked):
        if not self._updating:
            self._Settings().preventNegativeScale = bool(checked)

    def _onPreserveChildren(self, checked):
        if not self._updating:
            self._Settings().preserveChildren = bool(checked)

    def _onSize(self, value):
        if not self._updating:
            self._controller.settings.manipulatorSize = float(value)

    def _onReset(self):
        self._controller.settings.Reset(self._tool)
        self.Sync()


# Focus widgets that own every key they are given. The viewport hotkeys
# are bare letters, so typing a prim name into usdview's search box must
# never switch the tool.
_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                 QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)


class ViewportHotkeyFilter(QtCore.QObject):
    """
    The viewport hotkeys, taken at the APPLICATION level.

    A widget-level filter cannot work here: usdview's stage view sets no
    focus policy (it reports NoFocus), and usdview's own application
    filter calls SetFocusFromMousePos on every mouse move
    (appEventFilter.py), which walks up from the stage view and hands
    focus to the main window. So a key pressed over the viewport is
    delivered to the main window, never to the view, and Q / W / E / R,
    the size keys, D / Insert and the Escape abort were all dead in a
    real session even though QTest keys sent straight at the view worked.

    Qt runs application filters most-recently-installed first, and this
    plugin loads after usdview installed its own, so this one sees the
    key first -- which is also what lets a live drag take Escape ahead
    of appEventFilter.py's unconditional focus reset.

    Everything this filter claims, GizmoController.HandleHotkey decides;
    nothing is claimed unless it is acted on.
    """

    def __init__(self, controller):
        super(ViewportHotkeyFilter, self).__init__(controller)
        self._controller = controller

    def eventFilter(self, obj, event):
        try:
            kind = event.type()
            if kind in (QtCore.QEvent.MouseButtonPress, QtCore.QEvent.KeyPress,
                        QtCore.QEvent.ShortcutOverride, QtCore.QEvent.Close):
                if self._controller.OwnsWindowEvent(obj):
                    self._controller.FlushReleasedEdit()
            if kind == QtCore.QEvent.WindowDeactivate:
                # Every session in the process has one of these filters
                # on the one QApplication. Another session's window
                # deactivating is none of this session's business: act
                # only for our own windows, and do nothing at all
                # otherwise.
                if not self._controller.OwnsWindowEvent(obj):
                    return False
                # Cmd-Tab with a hold down: usdview never sees the
                # release at all, so drop the holds (and the latch)
                # here. _ClearHolds is idempotent; the Sync keeps the
                # Snap: button from naming a hold that is gone.
                self._controller._ClearHolds()
                self._controller.toolbar.Sync()
                return False
            if kind not in (QtCore.QEvent.KeyPress,
                            QtCore.QEvent.KeyRelease,
                            QtCore.QEvent.ShortcutOverride):
                return False
            return self._controller.HandleHotkey(obj, event, kind)
        except Exception as error:
            # An exception escaping an application-wide filter would
            # break every key in usdview, not just ours.
            Tf.Warn("rigExecUsdview: gizmo hotkey filter failed: %s"
                    % error)
            return False
