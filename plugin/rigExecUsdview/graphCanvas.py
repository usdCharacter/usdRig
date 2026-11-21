#
# RigExec graph editor: the canvas widget. The plot -- grid, rulers,
# curves, keys, tangent handles, playhead -- and every mouse gesture
# that edits them. Pixel mapping, gesture geometry, cursor and hit
# feedback live here; selection, drag and edit lifecycle live in
# graphEditorModel, topology and mapping in graphModel and graphScreen.
# This module paints what graphScreen computes and turns Qt events into
# the panel's edit calls, so hit-testing and drag mappings stay
# headlessly testable.
#
import math
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import graphModel
    import graphScreen
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import graphModel
    import graphScreen


SIDE_IN = graphModel.SIDE_IN
SIDE_OUT = graphModel.SIDE_OUT
SIDE_BOTH = graphModel.SIDE_BOTH

# graph editor palette, dark enough that a saturated curve colour
# carries on its own (spec section 2.2).
COLOR_BACKGROUND = QtGui.QColor(43, 43, 43)
COLOR_RULER = QtGui.QColor(56, 56, 56)
COLOR_RULER_EDGE = QtGui.QColor(28, 28, 28)
COLOR_GRID = QtGui.QColor(62, 62, 62)
COLOR_AXIS = QtGui.QColor(96, 96, 96)
COLOR_TEXT = QtGui.QColor(178, 178, 178)
COLOR_SELECTED = QtGui.QColor(255, 214, 51)
COLOR_HOVER = QtGui.QColor(255, 246, 196)
COLOR_TANGENT = QtGui.QColor(196, 196, 196)
COLOR_PLAYHEAD = QtGui.QColor(90, 190, 255)
COLOR_PLAYHEAD_TEXT = QtGui.QColor(20, 20, 20)
COLOR_MARQUEE = QtGui.QColor(210, 210, 210)
# Painted OVER the plot outside usdview's start/end frames, so the shot
# range reads without hiding the curve that leaves it.
COLOR_OUTSIDE = QtGui.QColor(0, 0, 0, 70)

CURVE_WIDTH = 1.6
GRID_WIDTH = 1.0
PLAYHEAD_WIDTH = 1.4

# One wheel notch is 120 eighths of a degree; this is the magnification
# per notch, matching the pace of usdview's own viewport dolly.
WHEEL_ZOOM = 1.15
# Pixels of Alt+right drag per doubling of the visible range.
ZOOM_DRAG_PIXELS = 220.0

# How far a press may travel and still count as a click rather than a
# marquee, in logical pixels.
CLICK_SLOP = 3.0


def _Color(rgb, alpha=255):
    """A graphModel 0..1 curve colour as a QColor."""
    return QtGui.QColor(int(round(rgb[0] * 255)), int(round(rgb[1] * 255)),
                        int(round(rgb[2] * 255)), alpha)


def _Position(event):
    """
    The cursor in the canvas's LOGICAL pixels.

    No devicePixelRatioF here, unlike gizmoUI._Position: gizmoScreen
    works in the physical pixels computeWindowViewport reports, while
    graphScreen is handed the widget's logical size and QPainter draws
    in logical pixels too. Multiplying would put every pick at twice the
    cursor's position on a HiDPI display.
    """
    try:
        point = event.position()
        return (float(point.x()), float(point.y()))
    except AttributeError:                                # Qt 5
        return (float(event.x()), float(event.y()))


def _IsFinite(value):
    return not (math.isnan(value) or math.isinf(value))


def _FormatTick(value, step):
    """A grid label: whole numbers while the step is whole."""
    if step >= 1.0:
        return "%d" % int(round(value))
    decimals = max(0, int(math.ceil(-math.log10(step))) + 1)
    return "%.*f" % (decimals, value)


class GraphCanvas(QtWidgets.QWidget):
    """
    The plot: grid, rulers, curves, keys, tangent handles, playhead --
    and every mouse gesture that edits them.

    Geometry is graphScreen's; this class only paints what graphScreen
    computes and turns Qt events into the panel's edit calls, so the
    hit-testing and the drag mappings stay headlessly testable.
    """

    def __init__(self, panel):
        super(GraphCanvas, self).__init__(panel)
        self._panel = panel
        self.transform = graphScreen.ViewTransform(
            600, 400, graphScreen.RULER_MARGINS)
        self.setMinimumSize(320, 200)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMouseTracking(True)
        self.setAutoFillBackground(False)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)

        self._hover = None          # (curveIndex, time) under the cursor
        self._marquee = None        # (x, y, w, h) while a marquee drags
        # pan / zoom / scrub / scrubHold / marquee / keys / tangent
        self._mode = None
        self._scrubHeld = False     # K is down
        self._axis = None           # a Shift+Alt drag's locked axis
        self._press = None
        self._last = None
        self._pressRanges = None
        self._pressModifiers = None  # modifiers at press, for marquee add

    # -- geometry -------------------------------------------------------

    def resizeEvent(self, event):
        super(GraphCanvas, self).resizeEvent(event)
        self.transform.Resize(self.width(), self.height())

    def PlotRectF(self):
        x, y, w, h = self.transform.PlotRect()
        return QtCore.QRectF(x, y, w, h)

    def _Splines(self):
        """The spline drawn per curve, None where the curve is hidden."""
        return self._panel.DrawSplines()

    def _Glyphs(self):
        """(key glyphs, tangent glyphs) for the visible curves: from the
        KEY splines, so knots an edit hook filters out are never squares
        or handles (GraphEditHook.KeySpline)."""
        curves = self._panel.Curves()
        splines = self._panel.KeySplines()
        selection = self._panel.SelectionPairs()
        return (graphScreen.KeyGlyphs(curves, splines, self.transform,
                                      selection),
                graphScreen.TangentGlyphs(curves, splines, self.transform,
                                          selection))

    def _Polylines(self):
        """Pixel polylines per curve, indexed by curve; [] when hidden."""
        result = []
        for spline in self._Splines():
            if spline is None:
                result.append([])
            else:
                result.append(graphScreen.SamplePolylines(spline,
                                                          self.transform))
        return result

    def KeyPixels(self):
        """
        {curve label: [(x, y, time)]} in LOGICAL pixels.

        One entry per KNOT, at the value square; a dual-valued knot's
        pre-value square is drawn as well but is the same key, so it
        would only make the map ambiguous.
        """
        curves = self._panel.Curves()
        result = {}
        for glyph in self._Glyphs()[0]:
            label = curves[glyph.curveIndex].Label()
            result.setdefault(label, []).append(
                (glyph.x, glyph.y, glyph.time))
        return result

    def TangentPixels(self):
        """{curve label: [(x, y, time, side)]} in LOGICAL pixels."""
        curves = self._panel.Curves()
        result = {}
        for glyph in self._Glyphs()[1]:
            label = curves[glyph.curveIndex].Label()
            result.setdefault(label, []).append(
                (glyph.x, glyph.y, glyph.time, glyph.side))
        return result

    # -- framing --------------------------------------------------------

    def FrameAll(self):
        self._Frame(self._panel.KeyExtents(selectedOnly=False))

    def FrameSelected(self):
        extents = self._panel.KeyExtents(selectedOnly=True)
        if extents is None:
            extents = self._panel.KeyExtents(selectedOnly=False)
        self._Frame(extents)

    def FrameStageRange(self):
        """Home: back to usdview's own frame range, values framed."""
        start, end = self._panel.StageRange()
        extents = self._panel.KeyExtents(selectedOnly=False)
        if start is None or end is None:
            self._Frame(extents)
            return
        vMin, vMax = (extents[2], extents[3]) if extents else (-1.0, 1.0)
        self.transform.Frame(start, end, vMin, vMax)
        self.update()

    def _Frame(self, extents):
        if extents is None:
            start, end = self._panel.StageRange()
            extents = (start if start is not None else 0.0,
                       end if end is not None else 24.0, -1.0, 1.0)
        self.transform.Frame(*extents)
        self.update()

    # -- painting -------------------------------------------------------

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            keys, tangents = self._Glyphs()
            self._DrawBackground(painter)
            painter.save()
            painter.setClipRect(self.PlotRectF())
            self._DrawGrid(painter)
            self._DrawOutsideRange(painter)
            self._DrawCurves(painter)
            self._DrawTangents(painter, keys, tangents)
            self._DrawKeys(painter, keys)
            self._DrawPlayhead(painter)
            self._DrawMarquee(painter)
            painter.restore()
            self._DrawRulers(painter)
            self._DrawPlayheadTab(painter)
        finally:
            painter.end()

    def _DrawBackground(self, painter):
        painter.fillRect(self.rect(), COLOR_BACKGROUND)
        plot = self.PlotRectF()
        painter.fillRect(QtCore.QRectF(0.0, 0.0, self.width(), plot.top()),
                         COLOR_RULER)
        painter.fillRect(QtCore.QRectF(0.0, 0.0, plot.left(), self.height()),
                         COLOR_RULER)

    def _DrawOutsideRange(self, painter):
        """usdview's start / end frames, shaded outside (spec 2.2)."""
        start, end = self._panel.StageRange()
        if start is None or end is None:
            return
        plot = self.PlotRectF()
        left = self.transform.TimeToX(start)
        right = self.transform.TimeToX(end)
        if left > plot.left():
            painter.fillRect(
                QtCore.QRectF(plot.left(), plot.top(),
                              min(left, plot.right()) - plot.left(),
                              plot.height()), COLOR_OUTSIDE)
        if right < plot.right():
            painter.fillRect(
                QtCore.QRectF(max(right, plot.left()), plot.top(),
                              plot.right() - max(right, plot.left()),
                              plot.height()), COLOR_OUTSIDE)

    def _DrawGrid(self, painter):
        plot = self.PlotRectF()
        timeTicks, valueTicks = graphScreen.GridLines(self.transform)
        pen = QtGui.QPen(COLOR_GRID)
        pen.setWidthF(GRID_WIDTH)
        axisPen = QtGui.QPen(COLOR_AXIS)
        axisPen.setWidthF(GRID_WIDTH)
        for time in timeTicks:
            x = self.transform.TimeToX(time)
            painter.setPen(axisPen if abs(time) < 1e-9 else pen)
            painter.drawLine(QtCore.QPointF(x, plot.top()),
                             QtCore.QPointF(x, plot.bottom()))
        for value in valueTicks:
            y = self.transform.ValueToY(value)
            painter.setPen(axisPen if abs(value) < 1e-9 else pen)
            painter.drawLine(QtCore.QPointF(plot.left(), y),
                             QtCore.QPointF(plot.right(), y))

    def _DrawCurves(self, painter):
        """
        Each visible curve as polylines, with the EXTRAPOLATED tails
        dashed (spec 2.2).

        The split at the first and last knot's pixel column is
        graphScreen.SplitExtrapolation, so it is covered headlessly by
        tests/python/test_graph_screen.py -- it used to live here and
        cut the PRE tail at the wrong bound, painting a solid run at the
        first key's value clean across the knot range.

        Non-finite samples are dropped first: linear extrapolation off a
        knot whose tangent algorithm is None samples as NaN in this Ts
        build, and a NaN point poisons the whole polyline.
        """
        curves = self._panel.Curves()
        splines = self._Splines()
        for index, spline in enumerate(splines):
            if spline is None or spline.IsEmpty():
                continue
            times = graphModel.KeyTimes(spline)
            if not times:
                continue
            xLow = self.transform.TimeToX(times[0])
            xHigh = self.transform.TimeToX(times[-1])
            color = _Color(curves[index].color)
            solidPen = QtGui.QPen(color)
            solidPen.setWidthF(CURVE_WIDTH)
            dashedPen = QtGui.QPen(color)
            dashedPen.setWidthF(CURVE_WIDTH)
            dashedPen.setStyle(QtCore.Qt.DashLine)
            for polyline in graphScreen.SamplePolylines(spline,
                                                        self.transform):
                for run in _FinitePolylines(polyline):
                    inside, outside = graphScreen.SplitExtrapolation(
                        run, xLow, xHigh)
                    painter.setPen(solidPen)
                    self._Strokes(painter, inside)
                    painter.setPen(dashedPen)
                    self._Strokes(painter, outside)

    @staticmethod
    def _Strokes(painter, runs):
        for points in runs:
            if len(points) > 1:
                painter.drawPolyline(QtGui.QPolygonF(
                    [QtCore.QPointF(p[0], p[1]) for p in points]))

    def _DrawKeys(self, painter, keys):
        curves = self._panel.Curves()
        half = graphScreen.KEY_PIXELS * 0.5
        painter.setPen(QtCore.Qt.NoPen)
        for glyph in keys:
            if glyph.selected:
                color = COLOR_SELECTED
            elif self._hover == (glyph.curveIndex, glyph.time):
                color = COLOR_HOVER
            else:
                color = _Color(curves[glyph.curveIndex].color)
            painter.setBrush(QtGui.QBrush(color))
            painter.drawRect(QtCore.QRectF(glyph.x - half, glyph.y - half,
                                           half * 2.0, half * 2.0))
            if glyph.IsDualValued():
                painter.drawRect(QtCore.QRectF(
                    glyph.x - half, glyph.preY - half, half * 2.0,
                    half * 2.0))

    def _DrawTangents(self, painter, keys, tangents):
        """
        The selected keys' handles: a line from the key to the end box,
        hollow when the tangent is produced by an algorithm rather than
        authored (spec 2.2).
        """
        anchors = {}
        for glyph in keys:
            anchors[(glyph.curveIndex, glyph.time)] = glyph
        half = graphScreen.TANGENT_PIXELS * 0.5
        pen = QtGui.QPen(COLOR_TANGENT)
        pen.setWidthF(1.0)
        for glyph in tangents:
            key = anchors.get((glyph.curveIndex, glyph.time))
            if key is None:
                continue
            y = key.y
            if glyph.side == SIDE_IN and key.IsDualValued():
                y = key.preY
            painter.setPen(pen)
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.drawLine(QtCore.QPointF(key.x, y),
                             QtCore.QPointF(glyph.x, glyph.y))
            box = QtCore.QRectF(glyph.x - half, glyph.y - half,
                                half * 2.0, half * 2.0)
            if glyph.locked:
                painter.drawRect(box)
            else:
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(QtGui.QBrush(COLOR_TANGENT))
                painter.drawRect(box)

    def _DrawPlayhead(self, painter):
        plot = self.PlotRectF()
        x = self.transform.TimeToX(self._panel.CurrentFrame())
        pen = QtGui.QPen(COLOR_PLAYHEAD)
        pen.setWidthF(PLAYHEAD_WIDTH)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLine(QtCore.QPointF(x, plot.top()),
                         QtCore.QPointF(x, plot.bottom()))

    def PlayheadTab(self, metrics=None):
        """The current-time tab in the frame ruler as (QRectF, text), or
        None while the playhead is outside the plot."""
        plot = self.PlotRectF()
        x = self.transform.TimeToX(self._panel.CurrentFrame())
        if x < plot.left() or x > plot.right():
            return None
        if metrics is None:
            metrics = QtGui.QFontMetrics(self.font())
        text = "%g" % self._panel.CurrentFrame()
        width = metrics.horizontalAdvance(text) + 10.0
        return (QtCore.QRectF(x - width * 0.5, 3.0, width,
                              plot.top() - 4.0), text)

    def _DrawPlayheadTab(self, painter):
        tab = self.PlayheadTab(QtGui.QFontMetrics(painter.font()))
        if tab is None:
            return
        rect, text = tab
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(COLOR_PLAYHEAD)
        painter.drawRoundedRect(rect, 3.0, 3.0)
        painter.setPen(QtGui.QPen(COLOR_PLAYHEAD_TEXT))
        painter.drawText(rect, QtCore.Qt.AlignCenter, text)

    def _DrawMarquee(self, painter):
        if self._marquee is None:
            return
        x, y, w, h = self._marquee
        pen = QtGui.QPen(COLOR_MARQUEE)
        pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRect(QtCore.QRectF(x, y, w, h).normalized())

    def _DrawRulers(self, painter):
        """The frame ruler along the top and the value ruler down the
        left, labelled from the same ticks the grid was drawn from."""
        plot = self.PlotRectF()
        timeTicks, valueTicks = graphScreen.GridLines(self.transform)
        _, _, w, h = self.transform.PlotRect()
        timeStep = graphScreen.NiceStep(self.transform.TimeSpan(), w)
        valueStep = graphScreen.NiceStep(self.transform.ValueSpan(), h)
        font = painter.font()
        font.setPointSizeF(max(7.0, font.pointSizeF() * 0.85))
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        painter.setPen(QtGui.QPen(COLOR_RULER_EDGE))
        painter.drawLine(QtCore.QPointF(plot.left(), plot.top()),
                         QtCore.QPointF(plot.right(), plot.top()))
        painter.drawLine(QtCore.QPointF(plot.left(), plot.top()),
                         QtCore.QPointF(plot.left(), plot.bottom()))
        painter.setPen(QtGui.QPen(COLOR_TEXT))
        for time in timeTicks:
            x = self.transform.TimeToX(time)
            text = _FormatTick(time, timeStep)
            painter.drawText(
                QtCore.QPointF(x - metrics.horizontalAdvance(text) * 0.5,
                               plot.top() - metrics.descent() - 5.0), text)
        for value in valueTicks:
            y = self.transform.ValueToY(value)
            text = _FormatTick(value, valueStep)
            painter.drawText(
                QtCore.QPointF(
                    max(2.0, plot.left() - 6.0
                        - metrics.horizontalAdvance(text)),
                    y + metrics.ascent() * 0.5 - 1.0), text)

    # -- events ---------------------------------------------------------

    def _OnRuler(self, point):
        return graphScreen.OnTimeRuler(self.transform, point[1])

    def SetScrubHeld(self, held):
        self._scrubHeld = bool(held)

    def ScrubHeld(self):
        return self._scrubHeld

    def mousePressEvent(self, event):
        self._panel.FlushPendingCommit()
        point = _Position(event)
        modifiers = event.modifiers()
        button = event.button()
        self._press = point
        self._last = point
        self._pressModifiers = modifiers
        if modifiers & QtCore.Qt.AltModifier:
            if button in (QtCore.Qt.MiddleButton, QtCore.Qt.RightButton):
                self._mode = ("pan" if button == QtCore.Qt.MiddleButton
                              else "zoom")
                self._axis = None
                self._pressRanges = (self.transform.timeRange,
                                     self.transform.valueRange)
                event.accept()
                return
            # Alt + other button: no graph gesture; clear stale state and
            # stop propagation so usdview's viewport doesn't also pan.
            self._pressRanges = None
            event.accept()
            return
        if self._scrubHeld and button in (QtCore.Qt.LeftButton,
                                          QtCore.Qt.MiddleButton):
            # K + middle moves the playhead alone; the frame follows at
            # the release.
            self._mode = ("scrub" if button == QtCore.Qt.LeftButton
                          else "scrubHold")
            self._Scrub(point)
            event.accept()
            return
        if button == QtCore.Qt.LeftButton and self._OnRuler(point):
            self._mode = "scrub"
            self._Scrub(point)
            event.accept()
            return
        if button == QtCore.Qt.LeftButton:
            self._PressLeft(point, modifiers)
            if self._mode is not None:
                event.accept()
            return
        if button == QtCore.Qt.MiddleButton:
            # "move the picked keys from anywhere": the selection
            # is not touched, so a middle drag never loses it.
            if self._panel.BeginKeyDrag(point):
                self._mode = "keys"
                event.accept()
                return
        event.ignore()

    def _PressLeft(self, point, modifiers):
        keys, tangents = self._Glyphs()
        tangent = graphScreen.HitTangent(tangents, point[0], point[1])
        key = graphScreen.HitKey(keys, point[0], point[1])
        # A tangent of zero width has its end box ON its own key, and
        # both pick radii are 6 px, so "handle first" would make such a
        # key ungrabbable. Whichever glyph the cursor is actually nearer
        # wins, and a tie goes to the key: an artist aiming at a handle
        # aims OUT along it, away from the key.
        if tangent is not None and (
                key is None
                or _Distance(point, tangent) < _KeyDistance(point, key)):
            anchor = self._Anchor(keys, tangent)
            if self._panel.BeginTangentDrag(tangent, anchor, point):
                self._mode = "tangent"
            return
        toggle = bool(modifiers & (QtCore.Qt.ShiftModifier
                                   | QtCore.Qt.ControlModifier))
        if key is not None:
            if toggle:
                self._panel.ToggleKey(key.curveIndex, key.time)
                return
            if not key.selected:
                self._panel.SetSelection([(key.curveIndex, key.time)])
            preOnly = _GrabbedPreSquare(point, key)
            if self._panel.BeginKeyDrag(point, preOnly=preOnly):
                self._mode = "keys"
            return
        self._mode = "marquee"
        self._marquee = (point[0], point[1], 0.0, 0.0)
        if not toggle:
            self._panel.SetSelection([])

    @staticmethod
    def _Anchor(keys, tangent):
        """
        The pixel the tangent handle hangs off: the key's value square,
        or its PRE-value square for the in side of a dual-valued knot,
        which is where TangentGlyphs measured the handle from.
        """
        for glyph in keys:
            if (glyph.curveIndex == tangent.curveIndex
                    and glyph.time == tangent.time):
                if tangent.side == SIDE_IN and glyph.IsDualValued():
                    return (glyph.x, glyph.preY)
                return (glyph.x, glyph.y)
        return (tangent.x, tangent.y)

    def mouseMoveEvent(self, event):
        point = _Position(event)
        mode = self._mode
        if mode is None:
            self._UpdateHover(point)
            return
        shift = bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        if mode == "pan":
            self.transform.timeRange, self.transform.valueRange = \
                self._pressRanges
            self.transform.Pan(*self._Travel(point, shift))
            self.update()
        elif mode == "zoom":
            self._Zoom(point, shift)
        elif mode in ("scrub", "scrubHold"):
            self._Scrub(point)
        elif mode == "marquee":
            self._marquee = (self._press[0], self._press[1],
                             point[0] - self._press[0],
                             point[1] - self._press[1])
            self.update()
        elif mode == "keys":
            self._panel.UpdateKeyDrag(
                point, bool(event.modifiers() & QtCore.Qt.ShiftModifier))
        elif mode == "tangent":
            self._panel.UpdateTangentDrag(point)
        self._last = point

    def _Scrub(self, point):
        self._panel.ScrubTo(self.transform.XToTime(point[0]),
                            apply=self._mode != "scrubHold")

    def _Travel(self, point, shift):
        """
        The pixel travel since the press. With Shift it is held to the
        axis the drag first moved along, until Shift is let go.
        """
        if not shift:
            self._axis = None
        elif self._axis is None and _Separation(
                point, self._press) > CLICK_SLOP:
            self._axis = graphScreen.DominantAxis(self._press, point)
        return graphScreen.ConstrainDelta(
            point[0] - self._press[0], point[1] - self._press[1],
            self._axis)

    def _Zoom(self, point, shift=False):
        """
        Alt+right drag: rightward magnifies time, upward
        magnifies value, both about the press point so the frame under
        the cursor when the drag started stays there.
        """
        self.transform.timeRange, self.transform.valueRange = \
            self._pressRanges
        dx, dy = self._Travel(point, shift)
        factorX = 2.0 ** (dx / ZOOM_DRAG_PIXELS)
        factorY = 2.0 ** (-dy / ZOOM_DRAG_PIXELS)
        self.transform.ZoomAbout(self._press[0], self._press[1],
                                 factorX, factorY)
        self.update()

    def mouseReleaseEvent(self, event):
        point = _Position(event)
        mode = self._mode
        self._mode = None
        # Press ranges are only for zoom; clear them whenever the gesture
        # ends so a stale range cannot leak to the next Alt+right drag.
        pressMods = self._pressModifiers
        self._pressRanges = None
        self._pressModifiers = None
        self._axis = None
        if mode == "scrubHold":
            self._panel.ScrubTo(self.transform.XToTime(point[0]))
        if mode == "marquee":
            rect = self._marquee
            self._marquee = None
            if rect is not None and (abs(rect[2]) > CLICK_SLOP
                                     or abs(rect[3]) > CLICK_SLOP):
                keys = graphScreen.KeysInRect(self._Glyphs()[0], rect)
                # Use the modifiers that were held at press, not at release.
                # Releasing Shift before mouse-up would otherwise flip
                # add/remove (bug #4).
                mods = pressMods if pressMods is not None else event.modifiers()
                add = bool(mods & (QtCore.Qt.ShiftModifier
                                   | QtCore.Qt.ControlModifier))
                self._panel.SelectGlyphs(keys, add=add)
            self.update()
            event.accept()
        elif mode in ("keys", "tangent"):
            self._panel.ReleaseGesture()
            event.accept()
        elif mode in ("pan", "zoom", "scrub", "scrubHold"):
            event.accept()
        self._UpdateHover(point)

    def mouseDoubleClickEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            return
        point = _Position(event)
        self._mode = None
        self._marquee = None
        self._pressRanges = None
        self._pressModifiers = None
        index = graphScreen.HitCurve(self._Polylines(), point[0], point[1])
        if index is None:
            return
        self._panel.InsertKeyOnCurve(
            index, graphScreen.CurveTimeAtX(self.transform, point[0]))
        event.accept()

    def wheelEvent(self, event):
        try:
            delta = event.angleDelta().y()
            point = event.position()
            x, y = point.x(), point.y()
        except AttributeError:                            # Qt 5
            delta = event.delta()
            x, y = float(event.x()), float(event.y())
        if not delta:
            return
        factor = WHEEL_ZOOM ** (delta / 120.0)
        self.transform.ZoomAbout(x, y, factor, factor)
        self.update()
        event.accept()

    def leaveEvent(self, event):
        super(GraphCanvas, self).leaveEvent(event)
        # An active drag keeps the mouse grab, so leaveEvent normally
        # fires only when no button is held. Clearing hover is enough;
        # cancelling the drag here would leave the panel's gesture open
        # but the canvas idle, and the next press would start a new
        # gesture while the old one still holds the EditRecorder.
        self._UpdateHover(None)

    def _UpdateHover(self, point):
        name = None
        if point is not None and (self._OnRuler(point) or self._scrubHeld):
            self.setCursor(QtCore.Qt.SizeHorCursor)
        else:
            self.unsetCursor()
        if point is not None:
            glyph = graphScreen.HitKey(self._Glyphs()[0], point[0], point[1])
            if glyph is not None:
                name = (glyph.curveIndex, glyph.time)
        if name != self._hover:
            self._hover = name
            self.update()

    def keyPressEvent(self, event):
        # The application filter normally gets here first; this is the
        # path for a key event posted straight at the canvas, which is
        # what a test does and a real keyboard never does.
        if not self._panel.HandleKey(event.key(), event.modifiers()):
            super(GraphCanvas, self).keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() \
                or not self._panel.HandleKeyRelease(event.key()):
            super(GraphCanvas, self).keyReleaseEvent(event)

    def focusOutEvent(self, event):
        # The K release may land on another widget.
        self._scrubHeld = False
        super(GraphCanvas, self).focusOutEvent(event)


def _Separation(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _Distance(point, glyph):
    return math.hypot(point[0] - glyph.x, point[1] - glyph.y)


def _KeyDistance(point, glyph):
    """A key is aimed at by EITHER square of a dual-valued knot."""
    distance = _Distance(point, glyph)
    if glyph.preY is not None:
        distance = min(distance,
                       math.hypot(point[0] - glyph.x, point[1] - glyph.preY))
    return distance


def _GrabbedPreSquare(point, glyph):
    """
    Whether a press on `glyph` grabbed its arrival (pre) square.

    Single knots have only the departure square, so this is False.
    On a dual knot the nearer square wins; a tie goes to the departure,
    matching the key-over-handle tie-break above (an artist aiming at a
    handle aims OUT along it, away from the key).
    """
    if glyph.preY is None:
        return False
    distValue = math.hypot(point[0] - glyph.x, point[1] - glyph.y)
    distPre = math.hypot(point[0] - glyph.x, point[1] - glyph.preY)
    return distPre < distValue


def _FinitePolylines(points):
    """
    `points` split at any non-finite sample, dropping it.

    Ts samples an extrapolated tail as NaN when the end knot has no
    tangent algorithm (verified in this build for ExtrapLinear off a
    TangentAlgorithmNone knot), and QPainter draws a polyline containing
    one NaN as a stray line across the whole widget.
    """
    runs = []
    current = []
    for point in points:
        if _IsFinite(point[0]) and _IsFinite(point[1]):
            current.append(point)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return runs
