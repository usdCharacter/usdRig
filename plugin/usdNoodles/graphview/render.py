#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""The drawing entry and the render-owned graph mirror.

``Renderer.draw(frame)`` is the only drawing entry. Each draw first syncs
the render-owned native mirror from the frame's records, then dispatches to
the primitive draws (``render_nodes``, ``render_links``, ``render_text``,
``render_overlays``, ``render_hotbox``) when their modules exist. The
primitives arrive in later links; until then a draw syncs the mirror,
applies the frame's transform inputs, and records which primitives ran.

The mirror is a native ``GraphModel`` owned by the ``Renderer``, built from
fresh bound ``NodeData``/``LinkData`` populated from frame records -- never
Python stand-ins, never the live model. Native graph construction for
rendering happens only in this module. A draw never writes the live model:
hover, selection and endpoints are applied to the mirror copies, so the
live position versions and text fields are unchanged across a draw. (One
ripple: populating a copy's position advances the process-global
``positionWriteGeneration``; the next reconcile pass scans once, finds no
live node changed, and stands down.)

Sync builds a candidate, then swaps: nodes sync first, then links, then
adjacency (``buildConnectionCache``), then validation, and only a fully
valid candidate replaces the mirror. Validation rejects duplicate node IDs
and link keys, complete edges whose ends are missing (membership) or whose
pins the end nodes do not carry (ports), and edges whose ends face the wrong
way (direction: attribute ends must be output/input-capable, where every
non-relationship row is mirror-dual and relationship pins are single-sided;
relationship sources must be the owner output side, with whole-prim targets
and the authored sources-to-sources flip exempted). A bad complete edge
raises ``MirrorBuildError`` -- it is never silently dropped -- and the
previous coherent mirror stays, still paired with its own hit data.
Explicit dangling records sync into the mirror independently of complete
adjacency and skip membership/port/direction checks; they are still
uniqueness- and count-checked like everything else.

Idle and pan draws reuse the mirror untouched: same revision, same identity
generation and identical record storage means zero copies and zero rebuild.
On a change only affected records are repopulated into the owned copies;
the native handoff re-syncs the snapshot on change only, never per paint.
All renderer caches are private and revision-keyed; the mirror, its
revision/generation, and the hit data swap atomically on success only.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``: ``paintGL``, ``_paintGLInner``, ``_renderConfig``,
``_seedNodeZOrders``, ``_worldSpaceProjectionMatrix`` and
``_screenSpaceProjectionMatrix``. Names the suite patches at module scope
(``NodeVertex``) and legacy-only helpers (``_ProfileSection``) resolve at
call time through ``legacy_symbol``, never through an import-time copy.
"""

from __future__ import annotations

import importlib
import time
from dataclasses import dataclass, field

from OpenGL import GL
from OpenGL.error import GLError
from pxr import Gf, Tf
from pxr.Usdviewq.qt import QtGui
from UsdNoodles.core import GraphModel, LinkData, NodeData

from ..constants import MAX_RENDER_DEPTH
from ..pinUtils import is_relationship_input_pin, is_relationship_pin
from .legacy import legacy_symbol


class MirrorBuildError(ValueError):
    """A candidate mirror failed validation; the previous mirror is kept."""


@dataclass(frozen=True, slots=True)
class RenderTransforms:
    """The projection inputs one draw paints with, as plain 16-tuples.

    Column-major, matching ``QMatrix4x4.data()`` order, derived from the
    frame's viewport with the exact geometry of the moved projection
    helpers (world: y-down ortho over the panned/scaled view; screen:
    y-down ortho over the widget).
    """

    world: tuple = (0.0,) * 16
    screen: tuple = (0.0,) * 16

    def __post_init__(self):
        object.__setattr__(self, "world", tuple(float(v) for v in self.world))
        object.__setattr__(
            self, "screen", tuple(float(v) for v in self.screen)
        )


def _ortho(left, right, bottom, top, near, far):
    """Column-major ortho matrix matching ``QMatrix4x4.ortho``."""
    return (
        2.0 / (right - left),
        0.0,
        0.0,
        0.0,
        0.0,
        2.0 / (top - bottom),
        0.0,
        0.0,
        0.0,
        0.0,
        -2.0 / (far - near),
        0.0,
        -(right + left) / (right - left),
        -(top + bottom) / (top - bottom),
        -(far + near) / (far - near),
        1.0,
    )


def transforms_for_viewport(viewport):
    """The :class:`RenderTransforms` for a frame viewport (no Qt needed)."""
    zoom = viewport.zoom if viewport.zoom else 1.0
    width, height = max(viewport.width, 1.0), max(viewport.height, 1.0)
    scaled_width = width / zoom
    scaled_height = height / zoom
    world = _ortho(
        viewport.pan_x,
        viewport.pan_x + scaled_width,
        viewport.pan_y + scaled_height,
        viewport.pan_y,
        -MAX_RENDER_DEPTH,
        MAX_RENDER_DEPTH,
    )
    screen = _ortho(
        0.0,
        width,
        height,
        0.0,
        -MAX_RENDER_DEPTH,
        MAX_RENDER_DEPTH,
    )
    return RenderTransforms(world=world, screen=screen)


def _link_key(link_id):
    """Canonical identity tuple for one link record's ``LinkId``."""
    return (
        str(getattr(link_id, "kind", "")),
        str(getattr(link_id, "owner_property", "")),
        str(getattr(link_id, "source_id", "")),
        str(getattr(link_id, "source_pin", "")),
        str(getattr(link_id, "target_id", "")),
        str(getattr(link_id, "target_pin", "")),
        str(getattr(link_id, "dangling_id", "")),
    )


def _tuples_identical(current, previous):
    """Whether two record tuples share every element object."""
    if current is previous:
        return True
    if len(current) != len(previous):
        return False
    return all(new is old for new, old in zip(current, previous))


def _output_capable(pin, record):
    """Whether *pin* can source a link on this node record.

    Output pins qualify, and so does any non-relationship input pin: every
    such row is mirror-dual, exposing an output port on the right edge
    (``NodeModel.inputPins``). Relationship pins are single-sided.
    """
    if pin in record.output_pins:
        return True
    return pin in record.input_pins and not is_relationship_pin(pin)


def _input_capable(pin, record):
    """Whether *pin* can sink a link on this node record (dual of above)."""
    if pin in record.input_pins:
        return True
    return pin in record.output_pins and not is_relationship_pin(pin)


def _is_sources_flip(link_id):
    """Whether the edge preserves an authored sources-to-sources direction.

    ``pinUtils.make_link_for_relationship`` keeps the authored direction when
    a relationship input pin targets another relationship input pin instead
    of treating the current prim as the output side; such an edge sources
    from an input pin by design.
    """
    return is_relationship_input_pin(
        str(getattr(link_id, "owner_property", ""))
    ) and is_relationship_input_pin(str(getattr(link_id, "source_pin", "")))


def _populate_node(copy, record):
    """Populate every mirrored native field of *copy* from *record*."""
    copy.id = record.node_id
    copy.name = record.title
    copy.type = record.node_type
    copy.schemaTypeName = record.type_name
    copy.position = Gf.Vec2d(record.position[0], record.position[1])
    copy.size = Gf.Vec2d(record.size[0], record.size[1])
    copy.inputPins = list(record.input_pins)
    copy.outputPins = list(record.output_pins)
    copy.inputRowKinds = list(record.input_row_kinds)
    copy.outputRowKinds = list(record.output_row_kinds)
    copy.inputRowSlots = list(record.input_row_slots)
    copy.outputRowSlots = list(record.output_row_slots)
    copy.selected = bool(record.selected)
    copy.titleCollapsed = bool(record.title_collapsed)
    copy.zOrder = int(record.z_order)
    if record.display_color is not None:
        color = tuple(record.display_color)
        copy.displayColorR = float(color[0])
        copy.displayColorG = float(color[1])
        copy.displayColorB = float(color[2])
    copy.relationshipInputPins = [
        pin for pin in record.input_pins if is_relationship_pin(pin)
    ]
    copy.relationshipOutputPins = [
        pin for pin in record.output_pins if is_relationship_pin(pin)
    ]
    for pin, pin_type in record.input_pin_types:
        copy.inputPinTypes[pin] = pin_type
    for pin, pin_type in record.output_pin_types:
        copy.outputPinTypes[pin] = pin_type
    copy.titleIconPath = record.title_icon_path


def _populate_link(copy, record):
    """Populate every mirrored native field of *copy* from *record*.

    """
    link_id = record.link_id
    copy.sourceNodeId = str(link_id.source_id)
    copy.sourcePort = str(link_id.source_pin)
    copy.targetNodeId = str(link_id.target_id)
    copy.targetPort = str(link_id.target_pin)
    copy.start = Gf.Vec2d(record.start[0], record.start[1])
    copy.end = Gf.Vec2d(record.end[0], record.end[1])
    copy.selected = bool(record.selected)
    copy.hovered = bool(record.hovered)
    copy.highlighted = bool(record.highlighted)
    copy.targetPropertyName = record.target_property
    copy.isDangling = bool(record.is_dangling)
    copy.danglingDirection = str(record.dangling_direction)
    copy.is_relationship_link = bool(record.is_relationship)


def _validate_records(frame):
    """Validate the frame's records; raise ``MirrorBuildError`` naming why.

    Checks unique node IDs and link keys, then every complete edge's
    membership, ports and direction. Explicit dangling records draw
    independently of complete adjacency and skip those three checks.
    """
    by_id = {}
    for record in frame.nodes:
        if record.node_id in by_id:
            raise MirrorBuildError(
                "duplicate node id %r in frame revision %r"
                % (record.node_id, frame.revision)
            )
        by_id[record.node_id] = record
    seen_keys = set()
    for record in frame.links:
        key = _link_key(record.link_id)
        if key in seen_keys:
            raise MirrorBuildError(
                "duplicate link %r in frame revision %r"
                % (key, frame.revision)
            )
        seen_keys.add(key)
        if record.is_dangling:
            continue
        link_id = record.link_id
        if link_id.source_id not in by_id:
            raise MirrorBuildError(
                "complete link %r sources from missing node %r"
                % (key, link_id.source_id)
            )
        if link_id.target_id not in by_id:
            raise MirrorBuildError(
                "complete link %r targets missing node %r"
                % (key, link_id.target_id)
            )
        source = by_id[link_id.source_id]
        target = by_id[link_id.target_id]
        if link_id.source_pin not in (
            source.input_pins + source.output_pins
        ):
            raise MirrorBuildError(
                "complete link %r sources from unknown pin %r on node %r"
                % (key, link_id.source_pin, link_id.source_id)
            )
        whole_prim_target = (
            link_id.kind == "relationship" and not link_id.target_pin
        )
        if not whole_prim_target and link_id.target_pin not in (
            target.input_pins + target.output_pins
        ):
            raise MirrorBuildError(
                "complete link %r targets unknown pin %r on node %r"
                % (key, link_id.target_pin, link_id.target_id)
            )
        if link_id.kind == "relationship":
            if not _output_capable(link_id.source_pin, source) and not (
                _is_sources_flip(link_id)
            ):
                raise MirrorBuildError(
                    "relationship link %r sources from input-only pin %r"
                    % (key, link_id.source_pin)
                )
            if not whole_prim_target and not _input_capable(
                link_id.target_pin, target
            ):
                raise MirrorBuildError(
                    "relationship link %r targets output-only pin %r"
                    % (key, link_id.target_pin)
                )
        else:
            if not _output_capable(link_id.source_pin, source):
                raise MirrorBuildError(
                    "attribute link %r sources from input-only pin %r"
                    % (key, link_id.source_pin)
                )
            if not _input_capable(link_id.target_pin, target):
                raise MirrorBuildError(
                    "attribute link %r targets output-only pin %r"
                    % (key, link_id.target_pin)
                )


def _try_import_primitive(name):
    """The ``draw`` of ``graphview.<name>``, or None when it has not landed.

    Only the module's own absence is tolerated: anything else -- a broken
    module, a missing ``draw`` -- propagates loudly.
    """
    try:
        module = importlib.import_module("." + name, __package__)
    except ModuleNotFoundError as error:
        if error.name == __package__ + "." + name:
            return None
        raise
    return module.draw


class Renderer:
    """Owns the render mirror and draws frames through the primitives."""

    _PRIMITIVE_MODULES = (
        "render_links",
        "render_nodes",
        "render_text",
        "render_overlays",
        "render_hotbox",
    )

    def __init__(self, resources=None):
        self._resources = resources
        self._current_frame = None
        self._mirror = None
        self._mirror_revision = None
        self._mirror_generation = None
        self._hit_data = None
        self._last_nodes = ()
        self._last_links = ()
        self._node_copies = {}
        self._node_sources = {}
        self._link_copies = []
        self._link_sources = []
        self._node_copies_last_draw = 0
        self._link_copies_last_draw = 0
        self._builds_total = 0
        self._draws_total = 0
        self._primitives_drawn = ()

    @property
    def resources(self):
        """The attached ``RenderResources`` bundle, or None without GL."""
        return self._resources

    @property
    def mirror(self):
        """The current render-owned native mirror, or None before the first
        successful sync."""
        return self._mirror

    @property
    def mirror_revision(self):
        """Model revision the current mirror was built from."""
        return self._mirror_revision

    @property
    def mirror_generation(self):
        """Identity generation the current mirror was built from."""
        return self._mirror_generation

    @property
    def hit_data(self):
        """Hit candidates paired with the current mirror (same swap)."""
        return self._hit_data

    @property
    def mirror_node_ids(self):
        """Node IDs in the current mirror, in frame order."""
        return tuple(record.node_id for record in self._last_nodes)

    @property
    def node_copies_last_draw(self):
        """Node records repopulated by the latest draw."""
        return self._node_copies_last_draw

    @property
    def link_copies_last_draw(self):
        """Link records repopulated by the latest draw."""
        return self._link_copies_last_draw

    @property
    def copies_last_draw(self):
        """Node plus link records repopulated by the latest draw."""
        return self._node_copies_last_draw + self._link_copies_last_draw

    @property
    def builds_total(self):
        """Candidate mirrors built (idle and pan draws build none)."""
        return self._builds_total

    @property
    def draws_total(self):
        """Draws that synced successfully."""
        return self._draws_total

    @property
    def primitives_drawn(self):
        """Primitive modules that drew on the latest draw."""
        return self._primitives_drawn

    def sync(self, frame):
        """Sync the mirror to *frame* and return it.

        Reuses the mirror untouched on identical revision, generation and
        record storage (idle and pan). Otherwise repopulates only affected
        records, builds a candidate (nodes, links, adjacency), validates,
        and swaps. Raises ``MirrorBuildError`` on a bad candidate and keeps
        the previous mirror paired with its own hit data.
        """
        if (
            self._mirror is not None
            and frame.revision == self._mirror_revision
            and frame.identity_generation == self._mirror_generation
            and _tuples_identical(frame.nodes, self._last_nodes)
            and _tuples_identical(frame.links, self._last_links)
        ):
            self._node_copies_last_draw = 0
            self._link_copies_last_draw = 0
            return self._mirror
        node_count, link_count = self._repopulate(frame)
        self._node_copies_last_draw = node_count
        self._link_copies_last_draw = link_count
        candidate = GraphModel()
        candidate.syncNodesFromModels(dict(self._node_copies))
        candidate.syncLinksFromModels(list(self._link_copies))
        candidate.buildConnectionCache()
        self._check_candidate(frame, candidate)
        self._mirror = candidate
        self._mirror_revision = frame.revision
        self._mirror_generation = frame.identity_generation
        self._hit_data = frame.hit_candidates
        self._last_nodes = frame.nodes
        self._last_links = frame.links
        self._builds_total += 1
        return candidate

    def _mirror_for_draw(self, frame):
        current = self._current_frame
        if current is not None and (
            (frame.identity_generation, frame.revision) <
            (current.identity_generation, current.revision)
            or (frame is not current and frame.time < current.time)
        ):
            return Renderer().sync(frame)
        mirror = self.sync(frame)
        self._current_frame = frame
        return mirror

    def draw(self, frame):
        """Draw *frame*: sync the mirror, apply transforms, run primitives.

        Returns the mirror the frame drew with. A sync failure raises
        ``MirrorBuildError`` before anything draws; a primitive failure
        propagates after the (already coherent) swap.
        """
        mirror = self._mirror_for_draw(frame)
        self._draws_total += 1
        self._apply_transform_inputs(frame)
        transforms = transforms_for_viewport(frame.viewport)
        drawn = []
        for name in self._PRIMITIVE_MODULES:
            painter = _try_import_primitive(name)
            if painter is None:
                continue
            painter(frame, self._resources, mirror, transforms)
            drawn.append(name)
        self._primitives_drawn = tuple(drawn)
        return mirror

    # -- internals ----------------------------------------------------------

    def _repopulate(self, frame):
        """Repopulate owned copies for affected records; return copy counts.

        A copy is reused only when the record object it was populated from
        is the incoming record object; added and replaced records get
        populated copies; removed IDs are dropped. Counts name repopulated
        records only. Keying reuse on the populated-from source (rather than
        the last good frame) keeps a failed build's staging from leaking
        into the next attempt.
        """
        node_count = 0
        seen_ids = set()
        for record in frame.nodes:
            seen_ids.add(record.node_id)
            if self._node_sources.get(record.node_id) is record:
                continue
            copy = self._node_copies.get(record.node_id)
            if copy is None:
                copy = NodeData()
                self._node_copies[record.node_id] = copy
            _populate_node(copy, record)
            self._node_sources[record.node_id] = record
            node_count += 1
        for node_id in list(self._node_copies):
            if node_id not in seen_ids:
                del self._node_copies[node_id]
                del self._node_sources[node_id]
        old_links = {}
        for record, copy in zip(self._link_sources, self._link_copies):
            old_links[_link_key(record.link_id)] = (record, copy)
        link_count = 0
        copies = []
        sources = []
        for record in frame.links:
            key = _link_key(record.link_id)
            previous = old_links.get(key)
            if previous is not None and previous[0] is record:
                copies.append(previous[1])
                sources.append(record)
                continue
            copy = LinkData()
            _populate_link(copy, record)
            copies.append(copy)
            sources.append(record)
            link_count += 1
        self._link_copies = copies
        self._link_sources = sources
        return (node_count, link_count)

    def _check_candidate(self, frame, candidate):
        """Validate records plus the built candidate; raise when invalid."""
        for node_id, copy in self._node_copies.items():
            if not isinstance(copy, NodeData):
                raise MirrorBuildError(
                    "node copy %r is not bound NodeData" % (node_id,)
                )
        for copy in self._link_copies:
            if not isinstance(copy, LinkData):
                raise MirrorBuildError("link copy is not bound LinkData")
        _validate_records(frame)
        if candidate.getLinkCount() != len(self._link_copies):
            raise MirrorBuildError(
                "candidate holds %d links for %d records; refusing half a graph"
                % (candidate.getLinkCount(), len(self._link_copies))
            )

    def _apply_transform_inputs(self, frame):
        """Apply the frame's transient offsets to the transform frame.

        The frame carries the complete offset map per capture, so the inputs
        apply idempotently: reset, then translate each live-generation
        offset. Stale generations never apply. Skipped without resources.
        """
        resources = self._resources
        transform_frame = getattr(resources, "transform_frame", None)
        if transform_frame is None:
            return
        inputs = frame.transform_inputs
        transform_frame.reset()
        for offset in inputs.offsets:
            if offset.generation != inputs.generation:
                continue
            transform_frame.translate(offset.node_id, offset.dx, offset.dy)


# -- moved GraphView methods (first parameter ``view``) -------------------------


def paintGL(view):
    if not view.isVisible():
        return

    if not view.initialized:
        view.initializeGL()
        if not view.initialized:
            return

    view.makeCurrent()

    if not view._hasCurrentRenderContext():
        return

    # Detect stale GL context (e.g. after usdview "Reopen Stage").
    # The font atlas texture serves as a canary — if it is no longer
    # a valid texture name, the context was recreated and all cached
    # GL resources (VAOs, VBOs, shaders) are stale.
    if view.fontAtlas and view.fontAtlas.texture:
        if not GL.glIsTexture(view.fontAtlas.texture):
            view._reinitializeGL()
            if not view.initialized:
                return

    try:
        view._clearPendingRenderCache()
        _paintGLInner(view)
    except GLError as e:
        Tf.Warn(f"GL error during paintGL: {e}")


def _paintGLInner(view):
    # Resolved once per call, so a patch active during the call is honored.
    _profile_section = legacy_symbol("_ProfileSection")
    # Enable profiling based on cached config
    view._profiler.enabled = view._cachedEnableRenderProfiling
    view._profiler.startFrame()

    with _profile_section(view._profiler, "Animator Update"):
        view._updateAnimator()

    with _profile_section(view._profiler, "Pan Animation"):
        view._updatePanAnimation()

    with _profile_section(view._profiler, "Keyboard State"):
        view._updateKeyboardState()

    # Apply any node-position changes to the spatial index / links / port
    # highlights before rendering or hit-testing this frame. O(1) when
    # nothing moved.
    with _profile_section(view._profiler, "Reconcile Moves"):
        view._reconcileMovedNodes()

    with _profile_section(view._profiler, "Rebuild Links"):
        if view.linksChanged:
            view._rebuildLinks()

    with _profile_section(view._profiler, "GL Clear"):
        GL.glClearColor(0.25, 0.25, 0.25, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glDepthFunc(GL.GL_LEQUAL)

    from . import render_hotbox, render_links, render_overlays
    from .frame import FrameSource

    resources = view._render_resources
    if resources.frame_source is None:
        resources.frame_source = FrameSource(view)
    frame = resources.frame_source.capture_from_view(advance_view=False)
    resources.link_config = view._renderConfig()
    mirror = resources.frame_renderer._mirror_for_draw(frame)
    transforms = transforms_for_viewport(frame.viewport)

    # Rendering:

    with _profile_section(view._profiler, "Group Stickers"):
        # group stickers first (background layer, behind everything)
        if view.drawNodes and view.groupStickers:
            view.paintGroupStickers()

    with _profile_section(view._profiler, "Links"):
        render_links.draw(frame, resources, mirror, transforms)

    with _profile_section(view._profiler, "Nodes"):
        # nodes (on top of links)
        if view.drawNodes:
            view.paintNodes()

    with _profile_section(view._profiler, "Port Highlights"):
        if view.nodes:
            view._renderPortHighlightsCpp()

    with _profile_section(view._profiler, "Text"):
        # text (on top of nodes)
        if view.drawText:
            view.paintText()

    with _profile_section(view._profiler, "Value Cells"):
        # Inline attribute values. A separate Python pass, because node
        # text is generated and drawn entirely in C++ by the pinned
        # external noodles renderer -- there is no hook to add a string to
        # a row's layout. Same shape as the marquee and status overlay.
        if view.drawText and view._cachedShowAttributeValues:
            view._paintValueCells()

    with _profile_section(view._profiler, "Status Overlay"):
        # status overlay (screen space, on top of everything)
        render_overlays.draw_status(frame, resources, transforms)

    with _profile_section(view._profiler, "Minimap"):
        # minimap (screen space, upper right corner)
        if view._cachedShowMinimap:
            view.minimap.paint(legacy_symbol("NodeVertex"), MAX_RENDER_DEPTH)

    with _profile_section(view._profiler, "Hotkey Popups"):
        view._paintHotkeyPopups()

    with _profile_section(view._profiler, "Marquee"):
        # marquee selection (on top of everything else)
        render_overlays.draw_marquee(frame, resources, transforms)

    with _profile_section(view._profiler, "Node Creation Hotbox"):
        render_hotbox.draw(frame, resources, mirror, transforms)

    with _profile_section(view._profiler, "Node Rename"):
        if view._renamingNodeId:
            view._paintNodeRename()

    with _profile_section(view._profiler, "Value Editor"):
        if view._valueEditTarget:
            view._paintValueEditor()
        if view._tokenPopup:
            view._paintTokenPopup()
        if view._mungActive:
            view._paintMungLadder()

    view._profiler.endFrame()
    view._printProfilingStatsIfNeeded()


def _renderConfig(view):
    """Cached C++ RenderConfig shared by the node-quad and link paths.

    The config depends only on settings, so it is rebuilt lazily after a
    settings change (invalidated to None in _initCachedSettings), never per
    frame or per geometry rebuild.
    """
    if view._cachedRenderConfig is None:
        from ..nodeGraph import build_render_config

        view._cachedRenderConfig = build_render_config()
    return view._cachedRenderConfig


def _seedNodeZOrders(view):
    """Give each freshly loaded node a distinct, increasing zOrder.

    Loaded nodes otherwise keep the C++ default zOrder = 0, so every node
    lands in the same depth band; overlapping node backgrounds, title bars,
    shadows, and title text then z-fight under GL_LEQUAL on the first frame.
    Assigning a distinct zOrder per node (mirroring the runtime add-node and
    reload paths) separates the depth bands so overlaps render cleanly
    front-to-back. _nextZOrder is left at the max so later add / bring-to-front
    actions stack on top.
    """
    view._nextZOrder = 0
    for node in view.nodes.values():
        view._nextZOrder += 1
        node.zOrder = view._nextZOrder


def _worldSpaceProjectionMatrix(view):
    # Reuse the cached matrix unless the view (pan/zoom/size) changed since
    # it was built; the key comparison makes this self-invalidating.
    key = (view.panX, view.panY, view.zoom, view.width(), view.height())
    if key != view._worldProjCacheKey:
        projection = QtGui.QMatrix4x4()
        scaledWidth = view.width() / view.zoom
        scaledHeight = view.height() / view.zoom
        projection.ortho(
            view.panX,
            view.panX + scaledWidth,
            view.panY + scaledHeight,
            view.panY,
            -MAX_RENDER_DEPTH,
            MAX_RENDER_DEPTH,
        )
        view._worldProjCacheKey = key
        view._worldProjCache = projection
    return view._worldProjCache


def _screenSpaceProjectionMatrix(view):
    key = (view.width(), view.height())
    if key != view._screenProjCacheKey:
        projection = QtGui.QMatrix4x4()
        projection.ortho(
            0, view.width(), view.height(), 0, -MAX_RENDER_DEPTH, MAX_RENDER_DEPTH
        )
        view._screenProjCacheKey = key
        view._screenProjCache = projection
    return view._screenProjCache


def _hasCurrentRenderContext(view):
    ctx = view.context()
    return ctx is not None and ctx.isValid() and ctx == type(ctx).currentContext()


def _clearPendingRenderCache(view):
    if (view._renderCacheClearPending and not view._deferRenderCacheClear
            and view._hasCurrentRenderContext()):
        view._renderCacheClearPending = False
        view._nodeRenderManager.clearCache()


def _clearRenderCache(view):
    if not view._deferRenderCacheClear and view.initialized and not view._renderCacheClearPending:
        view.makeCurrent()
        if view._hasCurrentRenderContext():
            try:
                view._nodeRenderManager.clearCache()
            finally:
                view.doneCurrent()
            return
    view._renderCacheClearPending = True


def _resetRenderCachesAfterBulkMove(view, rebuild_links=True):
    """Invalidate every baked vertex cache + the drag-transform frame and flag
    text for rebuild, after nodes are repositioned or (re)loaded wholesale.

    ``rebuild_links`` controls link refresh. The load paths pass True
    (connections are new, so a full rebuild is required). A pure move
    (auto-layout) passes False and refreshes only the moved links' endpoints
    afterward via ``_updateLinksForMovedNodes`` -- connections are unchanged,
    so re-reading every prim's connections from USD would be wasted work.
    """
    view._clearRenderCache()
    view._invalidateGraphGeometry()
    if rebuild_links:
        view.linksChanged = True


def _invalidateGraphGeometry(view):
    view.textRenderer.resetPositionCaches()
    # Pending/initial loaders can run before initializeGL creates icons.
    if hasattr(view, "_cppIconRenderer"):
        view._cppIconRenderer.resetPositionCaches()
    view._nodeRenderManager.resetNodeQuadCaches()
    view._nodeTransformFrame.reset()
    view.textChanged = True


class _ProfileSection:
    """Context manager that records elapsed time into a RenderProfiler."""

    __slots__ = ("_profiler", "_name", "_t0")

    def __init__(self, profiler, name):
        self._profiler = profiler
        self._name = name

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_exc):
        elapsed_ms = (time.perf_counter() - self._t0) * 1000.0
        self._profiler.recordTime(self._name, elapsed_ms)
        return False
