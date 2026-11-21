#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Immutable per-frame capture: the render layer's only view of the graph.

``FrameSource.capture(viewport, interaction_state, time) -> Frame`` pins
one immutable snapshot: the model revision and identity generation,
detached node geometry/ports/colors, value and glyph inputs, link
endpoints, overlay and hotbox geometry, the ``NodeTransformFrame``
inputs, and the hit-candidate lists. ``nv-render-*`` consume ``Frame``
only; nothing downstream touches the live graph, the stage, or the view.

Capture runs on the owner thread and advances due animations first: the
bound view's animator hooks run before anything is pinned (mirroring the
"Animator Update" first step of the paint pass), then time-derived
outputs (pan ease, value-flash expiry) resolve purely from
``(interaction_state, time)``. Capture is available on demand for input
before the first paint: ``hit(point)`` captures from the bound view when
no frame exists yet.

Records are frozen dataclasses and tuples: positions, sizes, colors and
endpoints are plain ``(x, y)`` / ``(r, g, b[, a])`` tuples, never live
USD handles, mutable pybind objects, closures over a view, or dicts.
Where existing code measures on the fly (value text metrics, glyph
vertices), the frame pins the inputs (row kind/count/components, font
size, show flags), never a new Python cache of manager output.

Sharing: the source subscribes to ``GraphModelState`` changes and each
capture replaces only the affected node records and incident edges,
reusing the recorded objects for everything else (``is``-identical
across captures). A pan changes the viewport half of the frame, never
graph records. A stage switch (graph replacement) bumps
``identity_generation``; retained frames keep their generation and stay
usable -- ``Frame.hit`` is pure -- after the model moves on.

This module imports no ``graphview/model.py``, ``selection.py`` or
``editing.py``: it reads the model through the passed view/state objects
(duck-typed ``subscribe``/``revision``/``graph`` and ``ChangeSet``
fields), so a render-layer import of a model writer stays impossible.
Link-curve sampling uses the native ``LinkGeometry`` curve when the
native module is importable and a straight-line fallback otherwise, so
this module (and its tests) load with the standard library alone.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field


# Link pick tolerance mirrors LinkRenderer.findLinkUnderCursor: the drawn
# ribbon half-width floored at 2 on-screen px.
_LINK_HALF_WIDTH_FRACTION = 0.5
_MIN_LINK_PICK_TOLERANCE_PX = 2.0
# Pan completion snap mirrors GraphView._updatePanAnimation.
_PAN_SNAP_WORLD = 0.5
# Pending-change cap: beyond it the next capture rebuilds everything
# rather than replaying an unbounded queue.
_MAX_PENDING_CHANGESETS = 64


def _pair(point, default=(0.0, 0.0)):
    """A ``(x, y)`` float pair from any point-like value.

    Accepts indexable pairs (tuples, lists, Gf.Vec2d) and Qt point-likes
    (``x()``/``y()``); anything else yields ``default``.
    """
    try:
        if hasattr(point, "x") and callable(point.x):
            return (float(point.x()), float(point.y()))
        return (float(point[0]), float(point[1]))
    except Exception:
        return (float(default[0]), float(default[1]))


def _rect(value):
    """A ``(x0, y0, x1, y1)`` float rect from a Rect-like, or None."""
    if value is None:
        return None
    try:
        if hasattr(value, "left"):
            return (
                float(value.left),
                float(value.top),
                float(value.right),
                float(value.bottom),
            )
        return (
            float(value[0]),
            float(value[1]),
            float(value[2]),
            float(value[3]),
        )
    except Exception:
        return None


def _float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _str(value, default=""):
    try:
        if value is None:
            return default
        return str(value)
    except Exception:
        return default


def _key_tuple(key):
    """Field-tuple identity of a model- or selection-layer link key.

    Duck-typed so this module never imports the key's owner.
    """
    return (
        _str(getattr(key, "kind", "")),
        _str(getattr(key, "owner_property", "")),
        _str(getattr(key, "source_id", "")),
        _str(getattr(key, "source_pin", "")),
        _str(getattr(key, "target_id", "")),
        _str(getattr(key, "target_pin", "")),
        _str(getattr(key, "dangling_id", "")),
    )


@dataclass(frozen=True, slots=True)
class Viewport:
    """The pinned viewport half of a frame.

    ``(pan_x, pan_y)`` is the top-left world corner and ``zoom`` the
    screen-per-world scale, matching the projection helpers: world =
    screen / zoom + pan.
    """

    pan_x: float = 0.0
    pan_y: float = 0.0
    zoom: float = 1.0
    width: float = 0.0
    height: float = 0.0

    @property
    def world_bounds(self):
        """Visible ``(min_x, min_y, max_x, max_y)`` in world space."""
        zoom = self.zoom if self.zoom else 1.0
        return (
            self.pan_x,
            self.pan_y,
            self.pan_x + self.width / zoom,
            self.pan_y + self.height / zoom,
        )

    def screen_to_world(self, point):
        """World ``(x, y)`` for a screen-space point."""
        x, y = _pair(point)
        zoom = self.zoom if self.zoom else 1.0
        return (x / zoom + self.pan_x, y / zoom + self.pan_y)

    def world_to_screen(self, point):
        """Screen-space ``(x, y)`` for a world point."""
        x, y = _pair(point)
        zoom = self.zoom if self.zoom else 1.0
        return ((x - self.pan_x) * zoom, (y - self.pan_y) * zoom)


@dataclass(frozen=True, slots=True)
class LinkId:
    """Canonical identity of one link, spelled field-for-field like the
    model layer's key so ``_key_tuple`` equates them without an import.
    """

    kind: str = "attribute"
    owner_property: str = ""
    source_id: str = ""
    source_pin: str = ""
    target_id: str = ""
    target_pin: str = ""
    dangling_id: str = ""


def link_id_for(link):
    """Derive the canonical :class:`LinkId` for a link object."""
    relationship = bool(getattr(link, "is_relationship_link", False))
    dangling_id = ""
    if bool(getattr(link, "isDangling", False)):
        if _str(getattr(link, "danglingDirection", "")) == "output":
            dangling_id = _str(getattr(link, "targetNodeId", ""))
        else:
            dangling_id = _str(getattr(link, "sourceNodeId", ""))
    return LinkId(
        kind="relationship" if relationship else "attribute",
        owner_property=_str(getattr(link, "propertyName", "")),
        source_id=_str(getattr(link, "sourceNodeId", "")),
        source_pin=_str(getattr(link, "sourcePort", "")),
        target_id=_str(getattr(link, "targetNodeId", "")),
        target_pin=_str(getattr(link, "targetPort", "")),
        dangling_id=dangling_id,
    )


@dataclass(frozen=True, slots=True)
class PortRecord:
    """One pin's draw/hit facts: world center plus its visible row slot."""

    name: str = ""
    is_output: bool = False
    center: tuple = (0.0, 0.0)
    row_slot: int = 0
    row_kind: int = 0

    def __post_init__(self):
        object.__setattr__(self, "center", _pair(self.center))


@dataclass(frozen=True, slots=True)
class ValueInput:
    """The inputs one value cell renders from: row identity plus the raw
    components. Measured widths and fitted texts stay with the text
    managers; the frame pins only what they measure from.

    ``type_name`` selects the fitted display format, ``state`` (spelling
    ``noodlesValues.STATE_*``) and ``editable`` select the pill colors, so
    the text primitive resolves both from the record, never a live row.
    """

    pin_name: str = ""
    property_name: str = ""
    kind: str = ""
    count: int = 0
    components: tuple = ()
    has_swatch: bool = False
    type_name: str = ""
    state: str = "authored"
    editable: bool = True

    def __post_init__(self):
        object.__setattr__(self, "components", tuple(self.components))


@dataclass(frozen=True, slots=True)
class GlyphInputs:
    """The inputs glyph baking measures from: font size and show flags."""

    font_size: float = 12.0
    show_values: bool = True
    min_pixel_height: float = 0.0


@dataclass(frozen=True, slots=True)
class NodeRecord:
    """One node's detached draw/hit facts.

    ``position`` is the base position; transient drag offsets ride in
    :class:`TransformFrameInputs` and are applied by the renderer (and
    by the hit candidates) without rewriting this record. ``node_type``,
    the pin-type pairs and ``title_icon_path`` exist for the render-owned
    mirror, whose C++ readers fall back to ``node.type`` for colors and
    labels when display color and pin types are absent.
    """

    node_id: str = ""
    position: tuple = (0.0, 0.0)
    size: tuple = (0.0, 0.0)
    z_order: int = 0
    title: str = ""
    type_name: str = ""
    node_type: str = ""
    title_icon_path: str = ""
    title_collapsed: bool = False
    input_pins: tuple = ()
    output_pins: tuple = ()
    input_pin_types: tuple = ()
    output_pin_types: tuple = ()
    port_start_y: float = 0.0
    port_line_height: float = 0.0
    input_centers: tuple = ()
    output_centers: tuple = ()
    input_row_kinds: tuple = ()
    input_row_slots: tuple = ()
    output_row_kinds: tuple = ()
    output_row_slots: tuple = ()
    ports: tuple = ()
    value_inputs: tuple = ()
    glyph_inputs: GlyphInputs = field(default_factory=GlyphInputs)
    value_cell_width: float = 0.0
    port_width: float = 0.0
    display_color: tuple | None = None
    selected: bool = False
    hovered: bool = False
    has_renderer: bool = False
    generation: int = 0

    def __post_init__(self):
        object.__setattr__(self, "position", _pair(self.position))
        object.__setattr__(self, "size", _pair(self.size))
        object.__setattr__(self, "input_pins", tuple(self.input_pins))
        object.__setattr__(self, "output_pins", tuple(self.output_pins))
        object.__setattr__(
            self,
            "input_pin_types",
            tuple(
                (str(pin), str(pin_type))
                for pin, pin_type in self.input_pin_types
            ),
        )
        object.__setattr__(
            self,
            "output_pin_types",
            tuple(
                (str(pin), str(pin_type))
                for pin, pin_type in self.output_pin_types
            ),
        )
        object.__setattr__(
            self, "input_centers", tuple(float(v) for v in self.input_centers)
        )
        object.__setattr__(
            self, "output_centers", tuple(float(v) for v in self.output_centers)
        )
        object.__setattr__(
            self, "input_row_kinds", tuple(int(v) for v in self.input_row_kinds)
        )
        object.__setattr__(
            self, "input_row_slots", tuple(int(v) for v in self.input_row_slots)
        )
        object.__setattr__(
            self, "output_row_kinds", tuple(int(v) for v in self.output_row_kinds)
        )
        object.__setattr__(
            self, "output_row_slots", tuple(int(v) for v in self.output_row_slots)
        )
        object.__setattr__(self, "ports", tuple(self.ports))
        object.__setattr__(self, "value_inputs", tuple(self.value_inputs))
        if self.display_color is not None:
            object.__setattr__(
                self,
                "display_color",
                tuple(float(v) for v in self.display_color),
            )

    @property
    def bounds(self):
        """Base ``(min_x, min_y, max_x, max_y)`` in world space."""
        x, y = self.position
        w, h = self.size
        return (x, y, x + w, y + h)


@dataclass(frozen=True, slots=True)
class LinkRecord:
    """One link's detached draw/hit facts.

    ``start``/``end`` are the curve inputs (the same pair the draw path
    samples); ``bounds`` is the sampled-curve box. Dangling links are
    explicit records: they draw and hit independently of adjacency.
    ``index`` is the link's position in the graph list at capture.
    """

    link_id: LinkId = field(default_factory=LinkId)
    index: int = -1
    start: tuple = (0.0, 0.0)
    end: tuple = (0.0, 0.0)
    vertical_end_tangent: bool = False
    bounds: tuple = (0.0, 0.0, 0.0, 0.0)
    is_dangling: bool = False
    dangling_direction: str = ""
    is_relationship: bool = False
    selected: bool = False
    hovered: bool = False
    generation: int = 0

    def __post_init__(self):
        object.__setattr__(self, "start", _pair(self.start))
        object.__setattr__(self, "end", _pair(self.end))
        bounds = _rect(self.bounds)
        object.__setattr__(
            self, "bounds", bounds if bounds is not None else (0.0,) * 4
        )


@dataclass(frozen=True, slots=True)
class PopupRecord:
    """A screen-space popup: visibility, rect, and detached entry labels."""

    visible: bool = False
    rect: tuple | None = None
    entries: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "rect", _rect(self.rect))
        object.__setattr__(
            self, "entries", tuple(_str(e) for e in self.entries)
        )


@dataclass(frozen=True, slots=True)
class ValueEditorRecord:
    """The inline value editor: target, detached text, and screen rect."""

    node_id: str = ""
    property_name: str = ""
    component: int = 0
    text: str = ""
    rect: tuple | None = None
    has_error: bool = False

    def __post_init__(self):
        object.__setattr__(self, "rect", _rect(self.rect))


@dataclass(frozen=True, slots=True)
class FlashRecord:
    """A post-commit value flash: target plus its absolute end time."""

    node_id: str = ""
    property_name: str = ""
    component: int = 0
    end_time: float = 0.0
    kind: str = "commit"


@dataclass(frozen=True, slots=True)
class OverlayState:
    """Overlay geometry, shared shape for capture input and output.

    Capture resolves the time-derived members (an expired flash becomes
    None) and pins the rest verbatim.
    """

    status_node_id: str = ""
    marquee_rect: tuple | None = None
    hotkey_popup: PopupRecord = field(default_factory=PopupRecord)
    navigation_popup: PopupRecord = field(default_factory=PopupRecord)
    rename_node_id: str = ""
    value_editor: ValueEditorRecord | None = None
    token_popup: PopupRecord = field(default_factory=PopupRecord)
    mung_visible: bool = False
    mung_rect: tuple | None = None
    mung_target: tuple | None = None
    flash: FlashRecord | None = None

    def __post_init__(self):
        object.__setattr__(self, "marquee_rect", _rect(self.marquee_rect))
        object.__setattr__(self, "mung_rect", _rect(self.mung_rect))
        if self.mung_target is not None:
            try:
                node_id, prop = tuple(self.mung_target)[:2]
                object.__setattr__(
                    self, "mung_target", (_str(node_id), _str(prop))
                )
            except Exception:
                object.__setattr__(self, "mung_target", None)


@dataclass(frozen=True, slots=True)
class HotboxRecord:
    """Node-creation hotbox: visibility, screen rect, entries, selection."""

    visible: bool = False
    rect: tuple | None = None
    entries: tuple = ()
    selected_index: int = -1

    def __post_init__(self):
        object.__setattr__(self, "rect", _rect(self.rect))
        object.__setattr__(
            self, "entries", tuple(_str(e) for e in self.entries)
        )


@dataclass(frozen=True, slots=True)
class TransformOffset:
    """One transient node offset, keyed by generation as well as id so a
    stale drag delta can never apply to a rebuilt node."""

    node_id: str = ""
    generation: int = 0
    dx: float = 0.0
    dy: float = 0.0


@dataclass(frozen=True, slots=True)
class TransformFrameInputs:
    """The ``NodeTransformFrame`` inputs: one offset map per capture."""

    generation: int = 0
    offsets: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "offsets", tuple(self.offsets))

    def offset_for(self, node_id):
        """``(dx, dy)`` for a live-generation node, else ``(0.0, 0.0)``."""
        for offset in self.offsets:
            if (
                offset.node_id == node_id
                and offset.generation == self.generation
            ):
                return (offset.dx, offset.dy)
        return (0.0, 0.0)


@dataclass(frozen=True, slots=True)
class PanAnim:
    """Pan animation inputs: current/target corners, speed, and whether the
    animation is still running."""

    current_x: float = 0.0
    current_y: float = 0.0
    target_x: float = 0.0
    target_y: float = 0.0
    speed: float = 18.0
    animating: bool = False


@dataclass(frozen=True, slots=True)
class AnimationOutputs:
    """Time-resolved animation outputs pinned by one capture."""

    advanced_time: float = 0.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    pan_animating: bool = False
    flash: FlashRecord | None = None
    link_dimming: float = 1.0


@dataclass(frozen=True, slots=True)
class NodeHitCandidate:
    """One node's hit facts: id, offset-applied bounds, draw order."""

    node_id: str = ""
    bounds: tuple = (0.0, 0.0, 0.0, 0.0)
    z_order: int = 0

    def __post_init__(self):
        bounds = _rect(self.bounds)
        object.__setattr__(
            self, "bounds", bounds if bounds is not None else (0.0,) * 4
        )


@dataclass(frozen=True, slots=True)
class LinkHitCandidate:
    """One link's hit facts: graph index, id, and curve bounds."""

    index: int = -1
    link_id: LinkId = field(default_factory=LinkId)
    bounds: tuple = (0.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        bounds = _rect(self.bounds)
        object.__setattr__(
            self, "bounds", bounds if bounds is not None else (0.0,) * 4
        )


@dataclass(frozen=True, slots=True)
class HitCandidates:
    """The frame's hit data: node candidates topmost-first, links in draw
    order. Shared across captures while the graph records are unchanged."""

    nodes: tuple = ()
    links: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "nodes", tuple(self.nodes))
        object.__setattr__(self, "links", tuple(self.links))


@dataclass(frozen=True, slots=True)
class HitParams:
    """Pick constants pinned with the frame so hit and draw agree."""

    link_line_width: float = 12.0
    port_hit_radius: float = 8.0
    show_values: bool = True
    value_min_pixel_height: float = 0.0


@dataclass(frozen=True, slots=True)
class ValueCellCandidate:
    """One caller-measured value cell: the frame pins it, never measures."""

    node_id: str = ""
    pin_name: str = ""
    component: int = 0
    rect: tuple = (0.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        bounds = _rect(self.rect)
        object.__setattr__(
            self, "rect", bounds if bounds is not None else (0.0,) * 4
        )


@dataclass(frozen=True, slots=True)
class InteractionState:
    """Frozen capture inputs from the interaction layer.

    Selection, hover, drag offsets, overlay/hotbox geometry, pick
    constants, value/glyph inputs and animation state -- everything the
    frame resolves or pins that is not the graph itself.
    """

    selected_nodes: tuple = ()
    selected_links: tuple = ()
    node_under_cursor: str = ""
    link_under_cursor: int = -1
    hovered_port: tuple | None = None
    value_hover: tuple | None = None
    port_hit_radius: float = 8.0
    link_line_width: float = 12.0
    show_values: bool = True
    value_min_pixel_height: float = 0.0
    content_changed: bool = False
    font_size: float = 12.0
    time_code: float | None = None
    transform_inputs: TransformFrameInputs = field(
        default_factory=TransformFrameInputs
    )
    pan: PanAnim = field(default_factory=PanAnim)
    link_dimming: float = 1.0
    overlay: OverlayState = field(default_factory=OverlayState)
    hotbox: HotboxRecord = field(default_factory=HotboxRecord)
    value_cells: tuple = ()

    def __post_init__(self):
        object.__setattr__(
            self, "selected_nodes", tuple(_str(n) for n in self.selected_nodes)
        )
        object.__setattr__(
            self, "selected_links", tuple(self.selected_links)
        )
        object.__setattr__(self, "value_cells", tuple(self.value_cells))


def _sample_curve(start, end, vertical_end_tangent):
    """World-space polyline for a link curve, as ``(x, y)`` tuples.

    Uses the native ``LinkGeometry`` adaptive sampling -- the same curve
    the draw path samples -- when the native module is importable, else
    a straight 16-segment fallback so capture still works headless.
    """
    try:
        from ..linkRenderer import sampleLinkCurveAdaptive

        points = sampleLinkCurveAdaptive(
            start, end, bool(vertical_end_tangent)
        )
        return [(float(p[0]), float(p[1])) for p in points] or [
            (float(start[0]), float(start[1])),
            (float(end[0]), float(end[1])),
        ]
    except Exception:
        x0, y0 = float(start[0]), float(start[1])
        x1, y1 = float(end[0]), float(end[1])
        return [
            (x0 + (x1 - x0) * i / 16.0, y0 + (y1 - y0) * i / 16.0)
            for i in range(17)
        ]


def _curve_bounds(start, end, vertical_end_tangent):
    """Tight ``(min_x, min_y, max_x, max_y)`` box of the sampled curve."""
    points = _sample_curve(start, end, vertical_end_tangent)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def _point_segment_distance_sq(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        return (px - ax) * (px - ax) + (py - ay) * (py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) * (px - cx) + (py - cy) * (py - cy)


def _uses_vertical_curve(link):
    """Whether a link takes the prim-target relationship curve.

    Same predicate as the draw path's curve choice: a relationship link
    whose target is a whole prim (no port, no property).
    """
    return (
        bool(getattr(link, "is_relationship_link", False))
        and bool(_str(getattr(link, "targetNodeId", "")))
        and not _str(getattr(link, "targetPort", ""))
        and not _str(getattr(link, "targetPropertyName", ""))
    )


def _detached_components(row):
    """Plain-tuple components of a value row (floats stay floats)."""
    detached = []
    for value in getattr(row, "components", ()) or ():
        try:
            detached.append(float(value))
        except Exception:
            detached.append(_str(value))
    return tuple(detached)


def _value_inputs_for(node, live_time_code, show_values):
    """Detached :class:`ValueInput` tuple for one node, input rows only."""
    if not show_values:
        return ()
    rows_fn = getattr(node, "value_rows", None)
    if not callable(rows_fn):
        return ()
    try:
        rows = rows_fn(live_time_code)
    except Exception:
        return ()
    if not rows:
        return ()
    try:
        items = sorted(
            rows.items(), key=lambda kv: (_str(kv[0][0]), bool(kv[0][1]))
        )
    except Exception:
        return ()
    inputs = []
    for (pin_name, is_output), row in items:
        if is_output:
            continue
        inputs.append(
            ValueInput(
                pin_name=_str(pin_name),
                property_name=_str(getattr(row, "property_name", "")),
                kind=_str(getattr(row, "kind", "")),
                count=int(getattr(row, "count", 0) or 0),
                components=_detached_components(row),
                has_swatch=bool(getattr(row, "has_swatch", False)),
                type_name=_str(getattr(row, "type_name", "")),
                state=_str(getattr(row, "state", "authored"), "authored"),
                editable=bool(getattr(row, "editable", True)),
            )
        )
    return tuple(inputs)


def _port_records(node_id, node, position, collapsed):
    """Detached :class:`PortRecord` tuple from the node's cached layout."""
    del node_id
    if collapsed:
        return ()
    try:
        input_pins = [_str(p) for p in (node.inputPins or ())]
    except Exception:
        input_pins = []
    try:
        output_pins = [_str(p) for p in (node.outputPins or ())]
    except Exception:
        output_pins = []
    try:
        in_centers = [float(v) for v in (node.layoutInputCenterY or ())]
    except Exception:
        in_centers = []
    try:
        out_centers = [float(v) for v in (node.layoutOutputCenterY or ())]
    except Exception:
        out_centers = []
    try:
        in_kinds = [int(v) for v in (node.inputRowKinds or ())]
    except Exception:
        in_kinds = []
    try:
        in_slots = [int(v) for v in (node.inputRowSlots or ())]
    except Exception:
        in_slots = []
    try:
        out_kinds = [int(v) for v in (node.outputRowKinds or ())]
    except Exception:
        out_kinds = []
    try:
        out_slots = [int(v) for v in (node.outputRowSlots or ())]
    except Exception:
        out_slots = []
    x, y = position
    try:
        width = float(node.size[0])
    except Exception:
        width = 0.0
    ports = []
    for i, name in enumerate(input_pins):
        if i >= len(in_centers):
            break
        ports.append(
            PortRecord(
                name=name,
                is_output=False,
                center=(x, y + in_centers[i]),
                row_slot=in_slots[i] if i < len(in_slots) else i,
                row_kind=in_kinds[i] if i < len(in_kinds) else 0,
            )
        )
    for i, name in enumerate(output_pins):
        if i >= len(out_centers):
            break
        ports.append(
            PortRecord(
                name=name,
                is_output=True,
                center=(x + width, y + out_centers[i]),
                row_slot=out_slots[i] if i < len(out_slots) else i,
                row_kind=out_kinds[i] if i < len(out_kinds) else 0,
            )
        )
    return tuple(ports)


def _display_color_for(node):
    """Detached ``(r, g, b[, a])`` display color, or None."""
    color = getattr(node, "displayColor", None)
    if color is None:
        color = getattr(node, "_display_color", None)
    if color is None:
        return None
    try:
        return tuple(float(v) for v in color)
    except Exception:
        return None


def _port_width_for(node, view):
    """World-space port glyph width from the node's (or default) renderer."""
    renderer = getattr(node, "renderer", None)
    if renderer is None and view is not None:
        renderer = getattr(view, "defaultRenderer", None)
    width_fn = getattr(renderer, "getPortWidth", None)
    if not callable(width_fn):
        return 0.0
    try:
        return float(width_fn())
    except Exception:
        return 0.0


def _pin_type_pairs(mapping):
    """Deterministic ``((pin, type), ...)`` pairs for a pin-type map."""
    if mapping is None:
        return ()
    try:
        if hasattr(mapping, "items"):
            items = list(mapping.items())
        elif hasattr(mapping, "keys"):
            items = [(key, mapping[key]) for key in mapping.keys()]
        else:
            items = [(key, mapping[key]) for key in list(mapping)]
    except Exception:
        return ()
    try:
        return tuple(sorted((str(k), str(v)) for k, v in items))
    except Exception:
        return ()


def build_node_record(
    node_id,
    node,
    *,
    generation,
    selected=False,
    hovered=False,
    glyph_inputs=None,
    live_time_code=None,
    view=None,
):
    """Detach one live node into a :class:`NodeRecord`.

    Reads geometry, pins, the cached C++ layout, value-row inputs (via
    the node's own ``value_rows`` at the view's live time code) and
    display color; keeps no reference to the node.
    """
    position = _pair(getattr(node, "position", (0.0, 0.0)))
    size = _pair(getattr(node, "size", (0.0, 0.0)))
    collapsed = bool(
        getattr(node, "is_title_collapsed", False)
        or getattr(node, "_title_collapsed", False)
    )
    try:
        input_pins = tuple(_str(p) for p in (node.inputPins or ()))
    except Exception:
        input_pins = ()
    try:
        output_pins = tuple(_str(p) for p in (node.outputPins or ()))
    except Exception:
        output_pins = ()
    try:
        in_centers = tuple(float(v) for v in (node.layoutInputCenterY or ()))
    except Exception:
        in_centers = ()
    try:
        out_centers = tuple(
            float(v) for v in (node.layoutOutputCenterY or ())
        )
    except Exception:
        out_centers = ()
    try:
        in_kinds = tuple(int(v) for v in (node.inputRowKinds or ()))
    except Exception:
        in_kinds = ()
    try:
        in_slots = tuple(int(v) for v in (node.inputRowSlots or ()))
    except Exception:
        in_slots = ()
    try:
        out_kinds = tuple(int(v) for v in (node.outputRowKinds or ()))
    except Exception:
        out_kinds = ()
    try:
        out_slots = tuple(int(v) for v in (node.outputRowSlots or ()))
    except Exception:
        out_slots = ()
    glyphs = glyph_inputs if glyph_inputs is not None else GlyphInputs()
    return NodeRecord(
        node_id=_str(node_id),
        position=position,
        size=size,
        z_order=int(getattr(node, "zOrder", 0) or 0),
        title=_str(getattr(node, "name", "") or getattr(node, "title", "")),
        type_name=_str(
            getattr(node, "schemaTypeName", "")
            or getattr(node, "typeName", "")
        ),
        node_type=_str(getattr(node, "type", "")),
        title_icon_path=_str(getattr(node, "titleIconPath", "") or ""),
        title_collapsed=collapsed,
        input_pins=input_pins,
        output_pins=output_pins,
        input_pin_types=_pin_type_pairs(getattr(node, "inputPinTypes", None)),
        output_pin_types=_pin_type_pairs(
            getattr(node, "outputPinTypes", None)
        ),
        port_start_y=_float(getattr(node, "layoutPortStartY", 0.0)),
        port_line_height=_float(
            getattr(node, "layoutPortLineHeight", 0.0)
        ),
        input_centers=in_centers,
        output_centers=out_centers,
        input_row_kinds=in_kinds,
        input_row_slots=in_slots,
        output_row_kinds=out_kinds,
        output_row_slots=out_slots,
        ports=_port_records(node_id, node, position, collapsed),
        value_inputs=_value_inputs_for(
            node, live_time_code, glyphs.show_values
        ),
        glyph_inputs=glyphs,
        value_cell_width=_float(getattr(node, "_value_cell_width", 0.0)),
        port_width=_port_width_for(node, view),
        display_color=_display_color_for(node),
        selected=bool(selected),
        hovered=bool(hovered),
        has_renderer=bool(getattr(node, "renderer", None)),
        generation=int(generation),
    )


def build_link_record(
    link, index, *, generation, selected=False, hovered=False
):
    """Detach one live link into a :class:`LinkRecord`."""
    start = _pair(getattr(link, "start", (0.0, 0.0)))
    end = _pair(getattr(link, "end", (0.0, 0.0)))
    vertical = _uses_vertical_curve(link)
    return LinkRecord(
        link_id=link_id_for(link),
        index=int(index),
        start=start,
        end=end,
        vertical_end_tangent=vertical,
        bounds=_curve_bounds(start, end, vertical),
        is_dangling=bool(getattr(link, "isDangling", False)),
        dangling_direction=_str(getattr(link, "danglingDirection", "")),
        is_relationship=bool(getattr(link, "is_relationship_link", False)),
        selected=bool(selected),
        hovered=bool(hovered),
        generation=int(generation),
    )


def _contains(bounds, x, y):
    min_x, min_y, max_x, max_y = bounds
    return min_x <= x <= max_x and min_y <= y <= max_y


def _expanded(bounds, margin):
    min_x, min_y, max_x, max_y = bounds
    return (min_x - margin, min_y - margin, max_x + margin, max_y + margin)


@dataclass(frozen=True, slots=True)
class Hit:
    """One pick result, in screen space and in the frame's world space."""

    kind: str = ""
    node_id: str = ""
    pin: str = ""
    is_output: bool = False
    link_index: int = -1
    link_id: LinkId | None = None
    component: int = 0
    detail: str = ""
    screen: tuple = (0.0, 0.0)
    world: tuple = (0.0, 0.0)
    revision: int = -1
    identity_generation: int = -1

    def __post_init__(self):
        object.__setattr__(self, "screen", _pair(self.screen))
        object.__setattr__(self, "world", _pair(self.world))


@dataclass(frozen=True, slots=True)
class Frame:
    """One immutable capture: revision, generation, viewport, node and
    link records (dangling links as explicit records sharing storage
    with ``links``), overlay/hotbox records, transform inputs, hit
    candidates and resolved animations. ``nv-render-*`` consume this."""

    revision: int = 0
    identity_generation: int = 0
    time: float = 0.0
    viewport: Viewport = field(default_factory=Viewport)
    nodes: tuple = ()
    links: tuple = ()
    dangling_links: tuple = ()
    overlay: OverlayState = field(default_factory=OverlayState)
    hotbox: HotboxRecord = field(default_factory=HotboxRecord)
    transform_inputs: TransformFrameInputs = field(
        default_factory=TransformFrameInputs
    )
    hit_candidates: HitCandidates = field(default_factory=HitCandidates)
    hit_params: HitParams = field(default_factory=HitParams)
    animations: AnimationOutputs = field(default_factory=AnimationOutputs)
    content_changed: bool = False
    node_under_cursor: str = ""
    link_under_cursor: int = -1
    hovered_port: tuple | None = None
    value_hover: tuple | None = None
    value_cells: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "nodes", tuple(self.nodes))
        object.__setattr__(self, "links", tuple(self.links))
        object.__setattr__(
            self, "dangling_links", tuple(self.dangling_links)
        )
        object.__setattr__(self, "value_cells", tuple(self.value_cells))

    def hit(self, point):
        """Pick ``point`` (screen space) against this frame's candidates.

        Pure: uses only this frame's revision, candidates, transforms
        and draw ordering, so a retained frame hits after the model
        moves on. Returns a :class:`Hit` or None.
        """
        return hit_frame(self, point)


def find_node(frame, node_id):
    """The :class:`NodeRecord` for ``node_id``, or None (linear scan)."""
    for record in frame.nodes:
        if record.node_id == node_id:
            return record
    return None


def find_link(frame, link_id):
    """The :class:`LinkRecord` matching ``link_id``, or None."""
    wanted = _key_tuple(link_id)
    for record in frame.links:
        if _key_tuple(record.link_id) == wanted:
            return record
    return None


def _hit_overlay(frame, x, y):
    """Screen-space overlay/hotbox pick, or None.

    Order mirrors press dispatch: the hotbox and modal editors intercept
    before graph picks.
    """
    overlay = frame.overlay
    hotbox = frame.hotbox
    if hotbox.visible and hotbox.rect is not None and _contains(
        hotbox.rect, x, y
    ):
        return ("hotbox", "", "", False, -1, None, 0, "")
    editor = overlay.value_editor
    if (
        editor is not None
        and editor.rect is not None
        and _contains(editor.rect, x, y)
    ):
        return (
            "overlay",
            editor.node_id,
            editor.property_name,
            False,
            -1,
            None,
            editor.component,
            "value_editor",
        )
    for detail, popup in (
        ("hotkey_popup", overlay.hotkey_popup),
        ("navigation_popup", overlay.navigation_popup),
        ("token_popup", overlay.token_popup),
    ):
        if (
            popup.visible
            and popup.rect is not None
            and _contains(popup.rect, x, y)
        ):
            return ("overlay", "", "", False, -1, None, 0, detail)
    if (
        overlay.mung_visible
        and overlay.mung_rect is not None
        and _contains(overlay.mung_rect, x, y)
    ):
        return ("overlay", "", "", False, -1, None, 0, "mung_ladder")
    if overlay.rename_node_id and _hit_rename_rect(frame, x, y):
        return ("overlay", overlay.rename_node_id, "", False, -1, None, 0, "rename")
    return None


def _hit_rename_rect(frame, x, y):
    """Whether a screen point lands in the rename editor band.

    The rename editor rides the renamed node's title bar; the band is
    the top row-height of that node in screen space.
    """
    record = find_node(frame, frame.overlay.rename_node_id)
    if record is None:
        return False
    dx, dy = frame.transform_inputs.offset_for(record.node_id)
    x0, y0 = frame.viewport.world_to_screen(
        (record.position[0] + dx, record.position[1] + dy)
    )
    x1, y1 = frame.viewport.world_to_screen(
        (
            record.position[0] + dx + record.size[0],
            record.position[1]
            + dy
            + (record.port_line_height or record.size[1]),
        )
    )
    return _contains(
        (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)), x, y
    )


def _hit_value_cell(frame, world_x, world_y):
    """World-space value-cell pick over caller-measured candidates."""
    if not frame.hit_params.show_values:
        return None
    zoom = frame.viewport.zoom if frame.viewport.zoom else 1.0
    for cell in frame.value_cells:
        record = find_node(frame, cell.node_id)
        if record is None:
            continue
        if (
            float(record.port_line_height) * zoom
            < float(frame.hit_params.value_min_pixel_height)
        ):
            continue
        if _contains(cell.rect, world_x, world_y):
            return (
                "value_cell",
                cell.node_id,
                cell.pin_name,
                False,
                -1,
                None,
                cell.component,
                "",
            )
    return None


def _hit_port(frame, world_x, world_y):
    """World-space pin-row pick, topmost-first like the draw order.

    Each pin's band is its cached center +/- half the row height -- the
    same per-pin layout the renderer uses, so picks match what's drawn.
    """
    by_id = {}
    for record in frame.nodes:
        by_id[record.node_id] = record
    for candidate in frame.hit_candidates.nodes:
        record = by_id.get(candidate.node_id)
        if record is None or record.title_collapsed:
            continue
        dx, dy = frame.transform_inputs.offset_for(record.node_id)
        left = record.position[0] + dx
        right = left + record.size[0]
        if not (left <= world_x <= right):
            continue
        half = record.port_line_height * 0.5
        for pins, centers, is_output in (
            (record.input_pins, record.input_centers, False),
            (record.output_pins, record.output_centers, True),
        ):
            for i, pin in enumerate(pins):
                if i >= len(centers):
                    break
                center = record.position[1] + dy + centers[i]
                if center - half <= world_y <= center + half:
                    return (
                        "port",
                        record.node_id,
                        pin,
                        is_output,
                        -1,
                        None,
                        0,
                        "",
                    )
    return None


def _hit_node(frame, world_x, world_y):
    """World-space node-body pick over offset-applied bounds."""
    for candidate in frame.hit_candidates.nodes:
        if _contains(candidate.bounds, world_x, world_y):
            return ("node", candidate.node_id, "", False, -1, None, 0, "")
    return None


def _hit_link(frame, world_x, world_y):
    """World-space link pick: bounds broad-phase, curve narrow-phase."""
    zoom = frame.viewport.zoom if frame.viewport.zoom else 1.0
    tolerance = max(
        _LINK_HALF_WIDTH_FRACTION * frame.hit_params.link_line_width,
        _MIN_LINK_PICK_TOLERANCE_PX / zoom,
    )
    by_index = {}
    for record in frame.links:
        by_index.setdefault(record.index, record)
    for candidate in frame.hit_candidates.links:
        if not _contains(
            _expanded(candidate.bounds, tolerance), world_x, world_y
        ):
            continue
        record = by_index.get(candidate.index)
        if record is None or _key_tuple(
            record.link_id
        ) != _key_tuple(candidate.link_id):
            record = find_link(frame, candidate.link_id)
        if record is None:
            continue
        points = _sample_curve(
            record.start, record.end, record.vertical_end_tangent
        )
        tolerance_sq = tolerance * tolerance
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            if (
                _point_segment_distance_sq(
                    world_x, world_y, ax, ay, bx, by
                )
                <= tolerance_sq
            ):
                return (
                    "link",
                    "",
                    "",
                    False,
                    record.index,
                    record.link_id,
                    0,
                    "dangling" if record.is_dangling else "",
                )
    return None


def hit_frame(frame, point):
    """Pick a screen-space ``point`` against ``frame``; a :class:`Hit` or
    None. Order: overlay/hotbox, value cells, ports, nodes, links."""
    screen = _pair(point)
    world = frame.viewport.screen_to_world(screen)
    found = _hit_overlay(frame, screen[0], screen[1])
    if found is None:
        found = _hit_value_cell(frame, world[0], world[1])
    if found is None:
        found = _hit_port(frame, world[0], world[1])
    if found is None:
        found = _hit_node(frame, world[0], world[1])
    if found is None:
        found = _hit_link(frame, world[0], world[1])
    if found is None:
        return None
    kind, node_id, pin, is_output, index, link_id, comp, detail = found
    return Hit(
        kind=kind,
        node_id=node_id,
        pin=pin,
        is_output=is_output,
        link_index=index,
        link_id=link_id,
        component=comp,
        detail=detail,
        screen=screen,
        world=world,
        revision=frame.revision,
        identity_generation=frame.identity_generation,
    )


def advance_pan(pan, dt):
    """One exponential ease-out step toward the pan target.

    Mirrors the native animator's ease plus the view's snap rule: within
    half a world unit the pan lands exactly and stops animating.
    """
    if not pan.animating:
        return (pan.current_x, pan.current_y, False)
    try:
        rate = 1.0 - math.exp(-float(pan.speed) * max(float(dt), 0.0))
    except Exception:
        rate = 1.0
    x = pan.current_x + (pan.target_x - pan.current_x) * rate
    y = pan.current_y + (pan.target_y - pan.current_y) * rate
    if (
        abs(x - pan.target_x) < _PAN_SNAP_WORLD
        and abs(y - pan.target_y) < _PAN_SNAP_WORLD
    ):
        return (pan.target_x, pan.target_y, False)
    return (x, y, True)


def _live_time_code(view):
    """The view's live time code for value-row reads (never synthesized)."""
    if view is None:
        return None
    return getattr(view, "_currentTimeCode", None)


def _now():
    """Wall-clock seconds (module-level: ``capture`` shadows ``time``)."""
    return time.time()


class FrameSource:
    """Owns capture state: subscription, record caches, and generations.

    ``FrameSource(view, model_state)`` subscribes to the state when one
    is given and reads the graph through it; with ``model_state=None``
    it reads ``view.nodeGraph`` (or ``view.nodes``/``view.links``)
    duck-typed and detects edits by comparing rebuilt records. Either
    way capture shares unchanged record objects across frames.
    """

    def __init__(self, view=None, model_state=None):
        self._view = view
        self._state = None
        self._unsubscribe = None
        self._owner_thread = threading.get_ident()
        self._identity_generation = 0
        self._graph_token = None
        self._revision = None
        self._duck_revision = 0
        self._node_cache = {}
        self._link_cache = ()
        self._node_tuple = ()
        self._dangling_marker = None
        self._dangling_tuple = ()
        self._candidates_cache = None
        self._candidates_key = None
        self._pending = []
        self._pending_overflow = False
        self._latest = None
        self._last_time = None
        self._last_selection = (frozenset(), frozenset())
        self._last_hover = ("", -1, None, None)
        self._last_value_key = None
        if model_state is not None:
            self.attach(model_state)

    @property
    def latest(self):
        """The most recently captured frame, or None."""
        return self._latest

    @property
    def revision(self):
        """The revision pinned by the latest capture (None before any)."""
        return self._revision

    @property
    def identity_generation(self):
        return self._identity_generation

    def _check_thread(self):
        if threading.get_ident() != self._owner_thread:
            raise RuntimeError(
                "FrameSource capture is confined to its owner thread"
            )

    def attach(self, model_state):
        """Subscribe to ``model_state`` (a stage switch or replacement).

        Unsubscribes the previous state; the next capture pins the new
        graph, bumps the identity generation when the graph object
        changed, and rebuilds. Attaching a state over the same graph
        reuses every equal record.
        """
        self._check_thread()
        if model_state is self._state:
            return
        self.detach()
        subscribe = getattr(model_state, "subscribe", None)
        if not callable(subscribe):
            raise TypeError("attach needs a state with subscribe()")
        self._state = model_state
        self._unsubscribe = subscribe(self._on_change)

    def detach(self):
        """Drop the model subscription; later captures read the view."""
        self._check_thread()
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            finally:
                self._unsubscribe = None
        self._state = None
        self._pending = []
        self._pending_overflow = False

    def close(self):
        """Detach; the retained frames stay usable."""
        self.detach()

    def _on_change(self, changeset):
        """Queue one model publication for the next capture."""
        self._pending.append(changeset)
        if len(self._pending) > _MAX_PENDING_CHANGESETS:
            self._pending = []
            self._pending_overflow = True

    def _resolve_graph(self):
        """``(nodes, links, token, revision)`` from state or view."""
        if self._state is not None:
            graph = self._state.graph
            return (
                graph.nodes,
                graph.links,
                ("state", id(graph)),
                int(self._state.revision),
            )
        view = self._view
        graph = getattr(view, "nodeGraph", None) if view is not None else None
        if graph is not None:
            nodes = getattr(graph, "nodes", None)
            links = getattr(graph, "links", None)
            if isinstance(nodes, dict) and isinstance(links, list):
                return (nodes, links, ("graph", id(graph)), None)
        nodes = getattr(view, "nodes", None) if view is not None else None
        links = getattr(view, "links", None) if view is not None else None
        if not isinstance(nodes, dict):
            nodes = {}
        if not isinstance(links, list):
            links = []
        return (nodes, links, ("pair", id(nodes), id(links)), None)

    def _interaction_dirty(self, interaction_state):
        """Dirties from selection, hover, and value inputs.

        Returns ``(dirty_nodes, dirty_links, dirty_link_indices,
        value_dirty)``: node ids, link-key tuples, link indices whose
        hover flag may have changed, and whether every node's value
        inputs must rebuild.
        """
        selected_nodes = frozenset(interaction_state.selected_nodes)
        selected_links = frozenset(
            _key_tuple(key) for key in interaction_state.selected_links
        )
        last_nodes, last_links = self._last_selection
        dirty_nodes = set(selected_nodes ^ last_nodes)
        dirty_links = set(selected_links ^ last_links)
        dirty_link_indices = set()
        self._last_selection = (selected_nodes, selected_links)
        hover = (
            interaction_state.node_under_cursor,
            interaction_state.link_under_cursor,
            interaction_state.hovered_port,
            interaction_state.value_hover,
        )
        last_hover = self._last_hover
        self._last_hover = hover
        if hover != last_hover:
            for tag in (hover, last_hover):
                if tag[0]:
                    dirty_nodes.add(tag[0])
                if tag[2]:
                    try:
                        dirty_nodes.add(_str(tag[2][0]))
                    except Exception:
                        pass
                if tag[3]:
                    try:
                        dirty_nodes.add(_str(tag[3][0]))
                    except Exception:
                        pass
                try:
                    dirty_link_indices.add(int(tag[1]))
                except Exception:
                    pass
        value_key = (
            bool(interaction_state.show_values),
            float(interaction_state.font_size),
            interaction_state.time_code,
            bool(interaction_state.content_changed),
        )
        previous = self._last_value_key
        self._last_value_key = value_key
        value_dirty = previous is not None and (
            value_key != previous or value_key[3]
        )
        return dirty_nodes, dirty_links, dirty_link_indices, value_dirty

    def _changeset_dirty(self):
        """Node/link dirties from the queued model publications."""
        dirty_nodes = set()
        dirty_links = set()
        for changeset in self._pending:
            for node_id in getattr(changeset, "changed_nodes", ()):
                dirty_nodes.add(_str(node_id))
            for node_id in getattr(changeset, "removed_nodes", ()):
                dirty_nodes.add(_str(node_id))
            for remap in getattr(changeset, "remaps", ()):
                try:
                    dirty_nodes.add(_str(remap[0]))
                    dirty_nodes.add(_str(remap[1]))
                except Exception:
                    pass
            for key in getattr(changeset, "changed_links", ()):
                dirty_links.add(_key_tuple(key))
            for key in getattr(changeset, "removed_links", ()):
                dirty_links.add(_key_tuple(key))
        self._pending = []
        return dirty_nodes, dirty_links

    def _build_nodes(
        self, nodes, dirty, selected, hover_ids, glyphs, live_time_code
    ):
        """Node records in graph order, reusing clean cached objects.

        Clean ids skip the live read entirely; a rebuilt record that
        still compares equal reuses the cached object, so a no-op
        publication shares storage.
        """
        records = []
        cache = self._node_cache
        for node_id, node in nodes.items():
            key = _str(node_id)
            cached = cache.get(key)
            if cached is not None and key not in dirty:
                records.append(cached)
                continue
            fresh = build_node_record(
                key,
                node,
                generation=self._identity_generation,
                selected=key in selected,
                hovered=key in hover_ids,
                glyph_inputs=glyphs,
                live_time_code=live_time_code,
                view=self._view,
            )
            if cached is not None and cached == fresh:
                records.append(cached)
            else:
                records.append(fresh)
        return records

    def _build_links(
        self, links, dirty_nodes, dirty_links, dirty_indices, selected, hover
    ):
        """Link records in graph order, reusing clean cached objects.

        A slot is reused only when its index, key and endpoint
        membership all still match; anything incident to a dirty node
        is rebuilt, and index shifts (insert/remove ahead) rebuild the
        shifted tail since the pinned index changed. A rebuilt record
        that still compares equal reuses the cached object.
        """
        records = []
        cached = self._link_cache
        for index, link in enumerate(links):
            key = _key_tuple(link_id_for(link))
            endpoints = (
                _str(getattr(link, "sourceNodeId", "")),
                _str(getattr(link, "targetNodeId", "")),
            )
            old = cached[index] if index < len(cached) else None
            if (
                old is not None
                and old.index == index
                and _key_tuple(old.link_id) == key
                and key not in dirty_links
                and endpoints[0] not in dirty_nodes
                and endpoints[1] not in dirty_nodes
                and index not in dirty_indices
                and (key in selected) == old.selected
                and (index == hover) == old.hovered
            ):
                records.append(old)
                continue
            fresh = build_link_record(
                link,
                index,
                generation=self._identity_generation,
                selected=key in selected,
                hovered=index == hover,
            )
            if old is not None and old == fresh:
                records.append(old)
            else:
                records.append(fresh)
        return records

    def _caches_match(self, nodes, links):
        """Whether the caches already describe these collections."""
        if set(self._node_cache) != {_str(k) for k in nodes.keys()}:
            return False
        cached = self._link_cache
        if len(cached) != len(links):
            return False
        for index, link in enumerate(links):
            old = cached[index]
            if old.index != index:
                return False
            if _key_tuple(old.link_id) != _key_tuple(link_id_for(link)):
                return False
        return True

    def _build_candidates(self, node_records, link_records, transforms):
        """Hit candidates from records plus transient offsets."""
        node_candidates = []
        for record in node_records:
            dx, dy = transforms.offset_for(record.node_id)
            x, y = record.position
            w, h = record.size
            node_candidates.append(
                NodeHitCandidate(
                    node_id=record.node_id,
                    bounds=(x + dx, y + dy, x + dx + w, y + dy + h),
                    z_order=record.z_order,
                )
            )
        node_candidates.sort(key=lambda c: c.z_order, reverse=True)
        link_candidates = [
            LinkHitCandidate(
                index=record.index,
                link_id=record.link_id,
                bounds=record.bounds,
            )
            for record in link_records
        ]
        return HitCandidates(
            nodes=tuple(node_candidates), links=tuple(link_candidates)
        )

    def _advance_view_animations(self):
        """Run the bound view's animator hooks, if it has them."""
        view = self._view
        if view is None:
            return
        for name in ("_updateAnimator", "_updatePanAnimation"):
            hook = getattr(view, name, None)
            if callable(hook):
                hook()

    def capture(self, viewport, interaction_state, time=None):
        """Pin one immutable :class:`Frame` and return it.

        Runs on the owner thread. Advances the bound view's animations
        first, then resolves time-derived outputs, then pins records:
        only affected node records and incident edges are rebuilt, and
        a stage switch (a new graph object) bumps the generation and
        rebuilds everything.
        """
        self._check_thread()
        self._advance_view_animations()
        now = time if time is not None else _now()
        try:
            now = float(now)
        except Exception:
            now = _now()
        if self._last_time is None:
            dt = 0.0
        else:
            dt = max(now - self._last_time, 0.0)
        self._last_time = now

        incoming_flash = interaction_state.overlay.flash
        active_flash = None
        if incoming_flash is not None and now < float(
            incoming_flash.end_time
        ):
            active_flash = incoming_flash
        overlay = interaction_state.overlay
        resolved_overlay = OverlayState(
            status_node_id=overlay.status_node_id,
            marquee_rect=overlay.marquee_rect,
            hotkey_popup=overlay.hotkey_popup,
            navigation_popup=overlay.navigation_popup,
            rename_node_id=overlay.rename_node_id,
            value_editor=overlay.value_editor,
            token_popup=overlay.token_popup,
            mung_visible=overlay.mung_visible,
            mung_rect=overlay.mung_rect,
            mung_target=overlay.mung_target,
            flash=active_flash,
        )
        pan_x, pan_y, pan_animating = advance_pan(
            interaction_state.pan, dt
        )

        nodes, links, token, state_revision = self._resolve_graph()
        live_ids = {_str(k) for k in nodes.keys()}
        if self._graph_token is None:
            self._graph_token = token
            switched = False
        elif token != self._graph_token:
            self._graph_token = token
            self._identity_generation += 1
            switched = True
        else:
            switched = False

        dirty_nodes, dirty_links, dirty_indices, value_dirty = (
            self._interaction_dirty(interaction_state)
        )
        if self._pending or self._pending_overflow:
            # Stale publications are harmless: records are rebuilt from
            # the live graph and dropped only when the live graph no
            # longer holds their id, so a replace record read against
            # the old graph rebuilds (and reuses) instead of wiping.
            model_nodes, model_links = self._changeset_dirty()
            dirty_nodes |= model_nodes
            dirty_links |= model_links
        verify_all = (
            switched
            or self._pending_overflow
            or self._latest is None
            or self._state is None
            or (
                self._state is not None
                and self._revision is not None
                and state_revision != self._revision
                and not dirty_nodes
                and not dirty_links
            )
        )
        self._pending_overflow = False
        if verify_all or value_dirty:
            dirty_nodes |= live_ids

        selected_nodes = frozenset(interaction_state.selected_nodes)
        selected_links = frozenset(
            _key_tuple(key) for key in interaction_state.selected_links
        )
        hover_ids = set()
        if interaction_state.node_under_cursor:
            hover_ids.add(interaction_state.node_under_cursor)
        if interaction_state.hovered_port:
            try:
                hover_ids.add(_str(interaction_state.hovered_port[0]))
            except Exception:
                pass
        if interaction_state.value_hover:
            try:
                hover_ids.add(_str(interaction_state.value_hover[0]))
            except Exception:
                pass
        glyphs = GlyphInputs(
            font_size=float(interaction_state.font_size),
            show_values=bool(interaction_state.show_values),
            min_pixel_height=float(
                interaction_state.value_min_pixel_height
            ),
        )
        live_time_code = _live_time_code(self._view)

        if (
            self._state is not None
            and not dirty_nodes
            and not dirty_links
            and not dirty_indices
            and self._revision == state_revision
            and self._caches_match(nodes, links)
        ):
            node_tuple = self._node_tuple
            link_tuple = self._link_cache
            revision = state_revision
        else:
            for gone in [
                k for k in self._node_cache if k not in live_ids
            ]:
                del self._node_cache[gone]
            node_records = self._build_nodes(
                nodes,
                dirty_nodes,
                selected_nodes,
                hover_ids,
                glyphs,
                live_time_code,
            )
            self._node_cache = {
                record.node_id: record for record in node_records
            }
            link_records = self._build_links(
                links,
                dirty_nodes,
                dirty_links,
                dirty_indices,
                selected_links,
                interaction_state.link_under_cursor,
            )
            if len(node_records) == len(
                self._node_tuple
            ) and all(
                new is old
                for new, old in zip(node_records, self._node_tuple)
            ):
                node_tuple = self._node_tuple
            else:
                node_tuple = tuple(node_records)
                self._node_tuple = node_tuple
            if len(link_records) == len(
                self._link_cache
            ) and all(
                new is old
                for new, old in zip(link_records, self._link_cache)
            ):
                link_tuple = self._link_cache
            else:
                link_tuple = tuple(link_records)
                self._link_cache = link_tuple
            if self._state is not None:
                revision = state_revision
            elif (
                self._latest is not None
                and node_tuple == self._latest.nodes
                and link_tuple == self._latest.links
            ):
                revision = self._revision
            elif self._latest is None:
                revision = self._duck_revision
            else:
                self._duck_revision += 1
                revision = self._duck_revision
        self._revision = revision

        transforms = interaction_state.transform_inputs
        candidates_key = (node_tuple, link_tuple, transforms)
        if (
            self._candidates_cache is not None
            and self._candidates_key is not None
            and self._candidates_key[0] is node_tuple
            and self._candidates_key[1] is link_tuple
            and self._candidates_key[2] == transforms
        ):
            candidates = self._candidates_cache
        else:
            candidates = self._build_candidates(
                node_tuple, link_tuple, transforms
            )
            self._candidates_cache = candidates
            self._candidates_key = (node_tuple, link_tuple, transforms)

        if self._dangling_marker is link_tuple:
            dangling_tuple = self._dangling_tuple
        else:
            dangling_tuple = tuple(
                record for record in link_tuple if record.is_dangling
            )
            self._dangling_marker = link_tuple
            self._dangling_tuple = dangling_tuple

        frame = Frame(
            revision=int(revision),
            identity_generation=self._identity_generation,
            time=now,
            viewport=viewport,
            nodes=node_tuple,
            links=link_tuple,
            dangling_links=dangling_tuple,
            overlay=resolved_overlay,
            hotbox=interaction_state.hotbox,
            transform_inputs=transforms,
            hit_candidates=candidates,
            hit_params=HitParams(
                link_line_width=float(
                    interaction_state.link_line_width
                ),
                port_hit_radius=float(interaction_state.port_hit_radius),
                show_values=bool(interaction_state.show_values),
                value_min_pixel_height=float(
                    interaction_state.value_min_pixel_height
                ),
            ),
            animations=AnimationOutputs(
                advanced_time=now,
                pan_x=pan_x,
                pan_y=pan_y,
                pan_animating=pan_animating,
                flash=active_flash,
                link_dimming=float(interaction_state.link_dimming),
            ),
            content_changed=bool(interaction_state.content_changed),
            node_under_cursor=interaction_state.node_under_cursor,
            link_under_cursor=interaction_state.link_under_cursor,
            hovered_port=interaction_state.hovered_port,
            value_hover=interaction_state.value_hover,
            value_cells=interaction_state.value_cells,
        )
        self._latest = frame
        return frame

    def capture_from_view(self, time=None):
        """Capture from the bound view's live state (input before paint)."""
        if self._view is None:
            raise RuntimeError("capture_from_view needs a bound view")
        return self.capture(
            viewport_from_view(self._view),
            interaction_state_from_view(self._view),
            time,
        )

    def hit(self, point):
        """Pick a screen-space ``point`` against the latest frame.

        Captures on demand from the bound view when no frame exists yet,
        so input works before the first paint. Pure frame hits are
        thread-safe; the on-demand capture needs the owner thread.
        """
        frame = self._latest
        if frame is None:
            if self._view is None:
                raise RuntimeError("no frame captured yet")
            self._check_thread()
            frame = self.capture_from_view()
        return frame.hit(point)


def _widget_size(view, name):
    """Widget dimension through a Qt method or a plain attribute."""
    value = getattr(view, name, 0.0)
    if callable(value):
        try:
            value = value()
        except Exception:
            return 0.0
    return _float(value)


def viewport_from_view(view):
    """Detach a :class:`Viewport` from a live (or stub) view."""
    return Viewport(
        pan_x=_float(getattr(view, "panX", 0.0)),
        pan_y=_float(getattr(view, "panY", 0.0)),
        zoom=_float(getattr(view, "zoom", 1.0), 1.0),
        width=_widget_size(view, "width"),
        height=_widget_size(view, "height"),
    )


def _popup_from(obj):
    """Detach a :class:`PopupRecord` from a live popup, guarded."""
    if obj is None:
        return PopupRecord()
    visible = getattr(obj, "is_visible", None)
    if visible is None:
        visible = getattr(obj, "visible", False)
    try:
        entries = tuple(
            _str(e) for e in (getattr(obj, "entries", ()) or ())
        )
    except Exception:
        entries = ()
    if not entries:
        try:
            entries = tuple(
                _str(getattr(item, "label", item))
                for item in (getattr(obj, "items", ()) or ())
            )
        except Exception:
            entries = ()
    return PopupRecord(
        visible=bool(visible),
        rect=_rect(getattr(obj, "rect", None)),
        entries=entries,
    )


def _flash_from(value):
    """Detach a :class:`FlashRecord` from a live flash tuple, or None.

    Accepts the 5-tuple ``(node, prop, comp, end, kind)`` and the legacy
    4-tuple without a component.
    """
    if value is None:
        return None
    try:
        parts = tuple(value)
    except Exception:
        return None
    if len(parts) == 5:
        node_id, prop, comp, end, kind = parts
    elif len(parts) == 4:
        node_id, prop, end, kind = parts
        comp = 0
    else:
        return None
    try:
        return FlashRecord(
            node_id=_str(node_id),
            property_name=_str(prop),
            component=int(comp),
            end_time=float(end),
            kind=_str(kind, "commit"),
        )
    except Exception:
        return None


def _mung_target_from(view):
    """The ``(node_id, property_name)`` mung drag target, or None."""
    if not bool(getattr(view, "_mungActive", False)):
        return None
    state = getattr(view, "_mungState", None)
    if not isinstance(state, dict):
        return None
    node_id = _str(state.get("nodeId", ""))
    prop = _str(state.get("prop", ""))
    if not node_id or not prop:
        return None
    return (node_id, prop)


def _value_editor_from(view):
    """Detach the inline value editor state, or None when idle."""
    target = getattr(view, "_valueEditTarget", None)
    if target is None:
        return None
    try:
        node_id, prop, comp = tuple(target)
    except Exception:
        return None
    editor = getattr(view, "_valueEditInput", None)
    text = ""
    if editor is not None:
        for attr in ("text", "value", "get_text"):
            candidate = getattr(editor, attr, None)
            if isinstance(candidate, str):
                text = candidate
                break
            if callable(candidate):
                try:
                    text = _str(candidate())
                    break
                except Exception:
                    continue
    return ValueEditorRecord(
        node_id=_str(node_id),
        property_name=_str(prop),
        component=int(comp or 0),
        text=text,
        rect=_rect(getattr(view, "_valueEditRect", None)),
        has_error=bool(getattr(view, "_valueEditError", False)),
    )


def _marquee_from(view):
    """Normalized screen-space marquee rect, or None when idle."""
    if not bool(getattr(view, "marqueeActive", False)):
        return None
    try:
        x0, y0 = _pair(getattr(view, "marqueeStart", None))
        x1, y1 = _pair(getattr(view, "marqueeCurrent", None))
    except Exception:
        return None
    if abs(x1 - x0) < 1.0 or abs(y1 - y0) < 1.0:
        return None
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def _hotbox_from(view):
    """Detach the node-creation hotbox state."""
    box = getattr(view, "nodeCreationHotbox", None)
    if box is None:
        return HotboxRecord()
    try:
        entries = tuple(
            _str(e) for e in (getattr(box, "entries", ()) or ())
        )
    except Exception:
        entries = ()
    try:
        selected = int(getattr(box, "selected_index", -1))
    except Exception:
        selected = -1
    return HotboxRecord(
        visible=bool(getattr(box, "is_visible", False)),
        rect=_rect(getattr(box, "rect", None)),
        entries=entries,
        selected_index=selected,
    )


def _pan_from(view):
    """Detach the pan animation state."""
    anim_x = getattr(view, "_panXAnim", None)
    anim_y = getattr(view, "_panYAnim", None)
    speed = _float(getattr(anim_x, "speed", 18.0), 18.0)
    return PanAnim(
        current_x=_float(
            getattr(anim_x, "current", getattr(view, "panX", 0.0))
        ),
        current_y=_float(
            getattr(anim_y, "current", getattr(view, "panY", 0.0))
        ),
        target_x=_float(
            getattr(anim_x, "target", getattr(view, "panX", 0.0))
        ),
        target_y=_float(
            getattr(anim_y, "target", getattr(view, "panY", 0.0))
        ),
        speed=speed,
        animating=bool(getattr(view, "_panAnimating", False)),
    )


def _port_hit_radius_from(view):
    """Port pick radius: drawn glyph width times the cached multiplier."""
    multiplier = getattr(view, "_cachedPortHitRadiusMultiplier", None)
    if multiplier is None:
        multiplier = getattr(view, "_portHitRadiusMultiplierDefault", 1.5)
    renderer = getattr(view, "defaultRenderer", None)
    width_fn = getattr(renderer, "getPortWidth", None)
    try:
        width = float(width_fn()) if callable(width_fn) else 8.0
    except Exception:
        width = 8.0
    try:
        return float(multiplier) * width
    except Exception:
        return 8.0


def _detached_time_code(view):
    """Best-effort float annotation of the view's live time code."""
    code = getattr(view, "_currentTimeCode", None)
    if code is None:
        return None
    get_value = getattr(code, "GetValue", None)
    if callable(get_value):
        try:
            value = float(get_value())
        except Exception:
            return None
    else:
        try:
            value = float(code)
        except Exception:
            return None
    try:
        if value != value:  # NaN would poison every value-key compare
            return None
    except Exception:
        return None
    return value


def interaction_state_from_view(view):
    """Detach an :class:`InteractionState` from a live (or stub) view.

    Reads selection, hover, pick constants, value/glyph inputs,
    transform generation, pan animation, and overlay/hotbox geometry
    through duck-typed attributes. The native transform frame has no
    readback, so drag offsets and caller-measured value cells arrive
    via explicit :class:`InteractionState` fields, never re-measured
    here.
    """
    selection = getattr(view, "_selection", None)
    try:
        node_ids = tuple(
            sorted(_str(n) for n in (selection.node_ids or ()))
        )
    except Exception:
        node_ids = ()
    try:
        link_keys = tuple(
            sorted(
                (_LinkId_from_key(k) for k in (selection.link_keys or ())),
                key=_key_tuple,
            )
        )
    except Exception:
        link_keys = ()
    hovered = getattr(view, "_hoveredPort", None)
    if hovered is not None:
        try:
            node_id, pin, is_output = tuple(hovered)
            hovered = (_str(node_id), _str(pin), bool(is_output))
        except Exception:
            hovered = None
    value_hover = getattr(view, "_valueHover", None)
    if value_hover is not None:
        try:
            node_id, prop, comp = tuple(value_hover)
            value_hover = (_str(node_id), _str(prop), int(comp))
        except Exception:
            value_hover = None
    try:
        link_under = int(getattr(view, "_linkUnderCursor", -1))
    except Exception:
        link_under = -1
    font_size = 12.0
    font_fn = getattr(view, "_valueFontSize", None)
    if callable(font_fn):
        try:
            font_size = float(font_fn())
        except Exception:
            font_size = 12.0
    token_popup = getattr(view, "_tokenPopup", None)
    token_record = PopupRecord(
        visible=token_popup is not None,
        rect=_rect(
            getattr(view, "_tokenPopupRect", None)
            if not isinstance(token_popup, dict)
            else (token_popup or {}).get("rect")
        ),
    )
    overlay = OverlayState(
        status_node_id=_str(getattr(view, "nodeIdUnderCursor", "")),
        marquee_rect=_marquee_from(view),
        hotkey_popup=_popup_from(getattr(view, "hotkeyPopup", None)),
        navigation_popup=_popup_from(
            getattr(view, "navigationHotkeyPopup", None)
        ),
        rename_node_id=_str(getattr(view, "_renamingNodeId", "")),
        value_editor=_value_editor_from(view),
        token_popup=token_record,
        mung_visible=bool(getattr(view, "_mungActive", False)),
        mung_rect=_rect(getattr(view, "_mungRect", None)),
        mung_target=_mung_target_from(view),
        flash=_flash_from(getattr(view, "_valueFlash", None)),
    )
    try:
        generation = int(getattr(view, "_transformGeneration", 0))
    except Exception:
        generation = 0
    return InteractionState(
        selected_nodes=node_ids,
        selected_links=link_keys,
        node_under_cursor=_str(getattr(view, "nodeIdUnderCursor", "")),
        link_under_cursor=link_under,
        hovered_port=hovered,
        value_hover=value_hover,
        port_hit_radius=_port_hit_radius_from(view),
        link_line_width=_float(
            getattr(view, "_cachedLinkLineWidth", 12.0), 12.0
        ),
        show_values=bool(getattr(view, "_cachedShowAttributeValues", True)),
        value_min_pixel_height=_float(
            getattr(view, "_cachedValueMinPixelHeight", 0.0)
        ),
        content_changed=bool(getattr(view, "textChanged", False)),
        font_size=font_size,
        time_code=_detached_time_code(view),
        transform_inputs=TransformFrameInputs(generation=generation),
        pan=_pan_from(view),
        link_dimming=_float(
            getattr(getattr(view, "linkDimming", None), "current", 1.0), 1.0
        ),
        overlay=overlay,
        hotbox=_hotbox_from(view),
        value_cells=(),
    )


def _LinkId_from_key(key):
    """A :class:`LinkId` from a selection/model-layer key, duck-typed."""
    fields = _key_tuple(key)
    return LinkId(
        kind=fields[0],
        owner_property=fields[1],
        source_id=fields[2],
        source_pin=fields[3],
        target_id=fields[4],
        target_pin=fields[5],
        dangling_id=fields[6],
    )
