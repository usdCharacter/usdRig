"""The hover picker's layout and gestures, with no Qt in sight.

The hover picker draws each picker tab straight into the viewport as a
group of buttons hanging off a small circle (the tab's handle), in 2D
screen space. This module owns where each group is, how big, how opaque,
and whether it is collapsed, plus the arithmetic the gestures apply to
those. The drawing and the mouse live in `pickerHoverUI`.

Coordinates are the viewport's logical pixels, origin top-left. A tab's
(x, y) is the centre of its handle; its buttons hang below it with their
top-left corner under the handle's left edge, at the tab's scale.

A tab is saved against the viewport corner nearest to where it was put
down -- so many pixels from the right edge and the top, say -- and placed
from that for whatever size the viewport is now (`Place`). It follows its
corner as the window is resized, and is pushed back inside a viewport too
small for where it was saved: the whole group when it fits, the handle at
least. The saved anchor itself only changes when the tab is moved, so it
goes back to its own place when the viewport grows again.

The layout is a viewer preference, never scene data: it is saved per user
(`ToJson` / `FromJson`), keyed by the picker and the tab it shows.
"""
import json
import math

# The handle's radius and the gap between it and the buttons, in screen
# pixels. Neither scales with the tab: the handle has to stay grabbable
# however small the buttons are made.
HANDLE_RADIUS = 7.0
HANDLE_GAP = 6.0
# Grabbing slop around the handle, so a 14 px circle is not a precision
# target.
HANDLE_SLOP = 3.0

MIN_SCALE = 0.2
MAX_SCALE = 4.0
MIN_OPACITY = 0.05
MAX_OPACITY = 1.0
# Shift-drag on a handle: the scale multiplies by exp(rate * px), so equal
# drags give equal ratios whatever the current size.
SCALE_RATE = 0.006
# Middle-drag: a full sweep from faint to solid is 200 px.
OPACITY_RATE = 1.0 / 200.0

# Where the first handle goes the first time hover mode is shown, and how
# far apart the handles sit. Right of usdview's top-left HUD block (250 px
# wide) so a fresh layout never sits on its text.
FIRST_HANDLE = (270.0, 24.0)
HANDLE_SPACING = 26.0
# A fresh tab is sized so its buttons take at most this fraction of the
# viewport's height, and never more than its authored size.
FIT_FRACTION = 0.45

# The overall-opacity knob, bottom-right: inset from the corner and lifted
# clear of usdview's bottom-right HUD block (two 15 px lines).
KNOB_RADIUS = 8.0
KNOB_INSET = 18.0
KNOB_HUD_CLEARANCE = 36.0


def _Clamp(value, low, high):
    return max(low, min(high, value))


def TabKey(pickerName, panelId):
    """The key a tab's layout is saved under."""
    return "%s/%s" % (pickerName or "", panelId or "")


class TabLayout(object):
    """One tab's place on screen."""

    def __init__(self, key, x=0.0, y=0.0, scale=1.0, opacity=1.0,
                 collapsed=False):
        self.key = key
        # Where the handle is drawn now, for the viewport's current size.
        self.x = float(x)
        self.y = float(y)
        # Where it is saved: the corner it is anchored to ("left"/"right",
        # "top"/"bottom") and the handle's distance from those edges.
        self.ax, self.ay = "left", "top"
        self.dx, self.dy = float(x), float(y)
        self.scale = _Clamp(float(scale), MIN_SCALE, MAX_SCALE)
        self.opacity = _Clamp(float(opacity), MIN_OPACITY, MAX_OPACITY)
        self.collapsed = bool(collapsed)

    def ToDict(self):
        # The anchor, not where the handle happens to be drawn: that is
        # derived from the anchor for whatever size the viewport is.
        return {"scale": self.scale, "opacity": self.opacity,
                "collapsed": self.collapsed,
                "anchor": [self.ax, self.ay],
                "offset": [self.dx, self.dy]}

    @classmethod
    def FromDict(cls, key, data):
        tab = cls(key, data.get("x", 0.0), data.get("y", 0.0),
                  data.get("scale", 1.0), data.get("opacity", 1.0),
                  data.get("collapsed", False))
        # A layout saved before anchors were kept is a top-left one.
        anchor = data.get("anchor")
        offset = data.get("offset")
        if isinstance(anchor, (list, tuple)) and len(anchor) == 2 and \
                anchor[0] in ("left", "right") and \
                anchor[1] in ("top", "bottom") and \
                isinstance(offset, (list, tuple)) and len(offset) == 2:
            tab.ax, tab.ay = anchor[0], anchor[1]
            tab.dx, tab.dy = float(offset[0]), float(offset[1])
        return tab

    # -- the viewport -----------------------------------------------------

    def Anchor(self, viewWidth, viewHeight):
        """Save where the handle is now against the nearest corner."""
        self.ax = "left" if self.x <= viewWidth * 0.5 else "right"
        self.ay = "top" if self.y <= viewHeight * 0.5 else "bottom"
        self.dx = self.x if self.ax == "left" else viewWidth - self.x
        self.dy = self.y if self.ay == "top" else viewHeight - self.y

    def Place(self, viewWidth, viewHeight, size=None):
        """Put the handle where its anchor says for a viewport this size,
        then keep it on screen. `size` is the buttons' (width, height) on
        screen, or None when only the handle is drawn."""
        x = self.dx if self.ax == "left" else viewWidth - self.dx
        y = self.dy if self.ay == "top" else viewHeight - self.dy
        r = HANDLE_RADIUS
        left, top = x - r, y - r
        right, bottom = x + r, y + r
        if size is not None and not self.collapsed:
            cx, cy = x - r, y + r + HANDLE_GAP
            right = max(right, cx + size[0])
            bottom = max(bottom, cy + size[1])
        # The whole group, where it fits.
        if right - left <= viewWidth:
            if left < 0.0:
                x -= left
            elif right > viewWidth:
                x -= right - viewWidth
        if bottom - top <= viewHeight:
            if top < 0.0:
                y -= top
            elif bottom > viewHeight:
                y -= bottom - viewHeight
        # And the handle always: it is how the tab is got back.
        margin = r + 2.0
        self.x = _Clamp(x, margin, max(margin, viewWidth - margin))
        self.y = _Clamp(y, margin, max(margin, viewHeight - margin))

    # -- geometry ---------------------------------------------------------

    def ButtonsCorner(self):
        """Screen point the tab's content origin is drawn at."""
        return (self.x - HANDLE_RADIUS,
                self.y + HANDLE_RADIUS + HANDLE_GAP)

    def ToScreen(self, origin, point):
        """A panel-space point as a screen point. `origin` is the panel's
        content origin (pickerModel.content_box)."""
        cx, cy = self.ButtonsCorner()
        return (cx + (point[0] - origin[0]) * self.scale,
                cy + (point[1] - origin[1]) * self.scale)

    def ToPanel(self, origin, point):
        """The inverse of ToScreen."""
        cx, cy = self.ButtonsCorner()
        return (origin[0] + (point[0] - cx) / self.scale,
                origin[1] + (point[1] - cy) / self.scale)

    def HitsHandle(self, point):
        return math.hypot(point[0] - self.x, point[1] - self.y) <= (
            HANDLE_RADIUS + HANDLE_SLOP)

    # -- gestures ---------------------------------------------------------

    def Moved(self, dx, dy):
        self.x += dx
        self.y += dy

    def Scaled(self, startScale, dx, dy):
        """Shift-drag from where it started: right or up grows, left or
        down shrinks. The handle stays put; the buttons follow it."""
        # The exponent is bounded first: the result clamps anyway, and a
        # drag far off screen must not overflow on the way there.
        power = _Clamp(SCALE_RATE * (dx - dy), -50.0, 50.0)
        self.scale = _Clamp(startScale * math.exp(power), MIN_SCALE,
                            MAX_SCALE)

    def Faded(self, startOpacity, dx):
        """Middle-drag from where it started: right is more opaque."""
        self.opacity = _Clamp(startOpacity + dx * OPACITY_RATE,
                              MIN_OPACITY, MAX_OPACITY)


def FitScale(contentHeight, viewHeight):
    """The scale a fresh tab starts at."""
    if contentHeight <= 0.0 or viewHeight <= 0.0:
        return 1.0
    return _Clamp(min(1.0, FIT_FRACTION * viewHeight / contentHeight),
                  MIN_SCALE, MAX_SCALE)


class Layout(object):
    """Every tab's place, the overall opacity, and whether hover is on."""

    def __init__(self):
        self.tabs = {}
        self.opacity = 1.0
        self.enabled = False

    def Tab(self, key):
        return self.tabs.get(key)

    def Ensure(self, keys, fitScales=None):
        """Give every key that has no layout yet one: handles side by side
        along the top, right of anything already placed in that row, all
        collapsed but the first tab of a fresh layout.

        `fitScales` maps a key to the scale it should start at."""
        fitScales = fitScales or {}
        placed = [t for t in self.tabs.values()
                  if t.ax == "left" and t.ay == "top" and
                  abs(t.dy - FIRST_HANDLE[1]) < 1e-6]
        nextX = FIRST_HANDLE[0]
        if placed:
            nextX = max(t.dx for t in placed) + HANDLE_SPACING
        # Adjacent handles would open their groups on top of each other,
        # so a fresh layout opens only its first tab; the rest wait as
        # handles in the row.
        opened = bool(self.tabs)
        for key in keys:
            if key in self.tabs:
                continue
            self.tabs[key] = TabLayout(key, nextX, FIRST_HANDLE[1],
                                       fitScales.get(key, 1.0),
                                       collapsed=opened)
            opened = True
            nextX += HANDLE_SPACING

    def Place(self, viewWidth, viewHeight, sizes=None):
        """Place every tab for a viewport this size. `sizes` maps a key
        to its buttons' on-screen (width, height)."""
        sizes = sizes or {}
        for key, tab in self.tabs.items():
            tab.Place(viewWidth, viewHeight, sizes.get(key))

    def Faded(self, startOpacity, dx):
        """The overall opacity, from a drag on the knob."""
        self.opacity = _Clamp(startOpacity + dx * OPACITY_RATE,
                              MIN_OPACITY, MAX_OPACITY)

    def TabOpacity(self, key):
        """What a tab is drawn at: its own opacity times the overall."""
        tab = self.tabs.get(key)
        return (tab.opacity if tab else 1.0) * self.opacity

    # -- persistence ------------------------------------------------------

    def ToJson(self):
        return json.dumps({
            "enabled": self.enabled,
            "opacity": self.opacity,
            "tabs": {k: t.ToDict() for k, t in sorted(self.tabs.items())},
        }, sort_keys=True)

    @classmethod
    def FromJson(cls, text):
        layout = cls()
        try:
            data = json.loads(text) if text else {}
        except (TypeError, ValueError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        layout.enabled = bool(data.get("enabled", False))
        try:
            layout.opacity = _Clamp(float(data.get("opacity", 1.0)),
                                    MIN_OPACITY, MAX_OPACITY)
        except (TypeError, ValueError):
            layout.opacity = 1.0
        tabs = data.get("tabs")
        for key, tab in (tabs.items() if isinstance(tabs, dict) else ()):
            if isinstance(tab, dict):
                try:
                    layout.tabs[key] = TabLayout.FromDict(key, tab)
                except (TypeError, ValueError):
                    pass
        return layout


def KnobCentre(viewWidth, viewHeight):
    """Where the overall-opacity knob sits in a viewport this size."""
    return (viewWidth - KNOB_INSET - KNOB_RADIUS,
            viewHeight - KNOB_HUD_CLEARANCE - KNOB_RADIUS)


def HitsKnob(viewWidth, viewHeight, point):
    cx, cy = KnobCentre(viewWidth, viewHeight)
    return math.hypot(point[0] - cx, point[1] - cy) <= (
        KNOB_RADIUS + HANDLE_SLOP)
