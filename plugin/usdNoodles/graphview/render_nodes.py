#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Node, port and sticker primitives: the node-side draw owner.

``draw(frame, resources, mirror, transforms)`` paints node quads, port
highlights and title-bar icons from a detached frame, the render-owned
native mirror, and the one shared ``NodeTransformFrame`` -- never from
the live model, and it authors nothing. The frame, mirror and
transform inputs arrive from ``render.Renderer.draw``; the managers,
atlas and shaders arrive on the ``resources`` bundle. Any missing
piece (no resources, no manager, no mirror, no nodes) skips its
submit, so resource-less draws still sync the mirror.

Node, text and icon placement share ONE transform frame: the bundle's
``transform_frame``, which ``resources.initialize`` wires into the
node, text and icon managers. This module never constructs a
``NodeTransformFrame`` of its own. Transient offsets are keyed by
object generation as well as ID: ``draw`` reconciles the shared frame
against the frame's records, discarding an offset when its generation
is stale, when its node ID is gone, or when the record for that ID
was replaced since the previous draw, and retaining offsets only for
an unchanged identity. The renderer's own reset-plus-translate
already applied the live-generation offsets; the reconcile touches
the frame only when identity filtering drops something it applied.

Selection colors come from the frame's ``selected`` flags (carried
into the mirror by the renderer) plus the themed stroke color, which
is resolved from ``NoodlesConfig`` at call time. A selection-set
change marks the node quads dirty, exactly as the legacy paint path
does.

Group stickers move here as ``paintGroupStickers(view)`` for the
legacy paint path; ``draw`` does not paint them yet because the frame
carries no sticker records. Likewise ``draw`` submits the mirror's
synced snapshot fields (pins, row slots, sizes) without the legacy
Python layout pass, which reads live ``NodeModel`` objects: drawing
nothing from the live model is the point.

Moved verbatim from ``GraphView`` as module-level functions whose
first parameter is ``view``: ``paintNodes``, ``_drawNodeVertices``,
``_renderPortHighlightsCpp``, ``paintGroupStickers``,
``_relationshipLinkColor`` and ``_isPrimTargetRelationshipLink``.
The suite-patched ``NodeVertex`` resolves at call time through
``legacy_symbol``, never through an import-time copy.
"""

from __future__ import annotations

import ctypes

from OpenGL import GL
from OpenGL.error import GLError
from pxr import Tf
from UsdNoodles.render import NODE_VERTEX_ATTRIB_LAYOUT

from .legacy import legacy_symbol


_CACHE_ATTR = "_render_nodes_seen"
_SELECTION_ATTR = "_render_nodes_selection"


# -- shared placement -------------------------------------------------------
#
# Node, text and icon placement read the same transform inputs through
# these helpers, so the three agree on where a node and its ports are.
# All inputs are duck-typed frame records; nothing here touches Qt, GL,
# the live model, or the stage.


def effective_offset(transform_inputs, node_id):
    """The live ``(dx, dy)`` transient offset for *node_id*.

    Only offsets recorded under the inputs' own generation apply; a
    stale generation (the object was replaced under a new identity)
    yields ``(0.0, 0.0)``.
    """
    dx, dy = 0.0, 0.0
    generation = transform_inputs.generation
    for offset in transform_inputs.offsets:
        if (
            offset.node_id == node_id
            and offset.generation == generation
        ):
            dx, dy = offset.dx, offset.dy
    return (dx, dy)


def select_node_offsets(frame):
    """The live ``{node_id: (dx, dy)}`` offset map for *frame*.

    Keeps an offset only when its generation matches the frame's live
    generation and its node ID is present in the frame's node records;
    stale generations and removed IDs are dropped.
    """
    inputs = frame.transform_inputs
    generation = inputs.generation
    present = set()
    for record in frame.nodes:
        present.add(record.node_id)
    live = {}
    for offset in inputs.offsets:
        if offset.generation != generation:
            continue
        if offset.node_id not in present:
            continue
        live[offset.node_id] = (offset.dx, offset.dy)
    return live


def placed_node_bounds(record, transform_inputs):
    """The world ``(min_x, min_y, max_x, max_y)`` bounds of *record*.

    The record's base position and size plus its live transient
    offset: the box the node quad, its text and its icons share.
    """
    dx, dy = effective_offset(transform_inputs, record.node_id)
    x, y = record.position
    w, h = record.size
    return (x + dx, y + dy, x + dx + w, y + dy + h)


def placed_port_centers(record, transform_inputs):
    """The ``((name, is_output, x, y), ...)`` world port centers.

    Each of the record's detached port centers plus its live transient
    offset: the points the highlight circles and the hit tests share.
    """
    dx, dy = effective_offset(transform_inputs, record.node_id)
    return tuple(
        (
            port.name,
            port.is_output,
            port.center[0] + dx,
            port.center[1] + dy,
        )
        for port in record.ports
    )


def selected_node_ids(frame):
    """The frozenset of selected node IDs in *frame*."""
    return frozenset(
        record.node_id for record in frame.nodes if record.selected
    )


# -- draw entry ---------------------------------------------------------------


def draw(frame, resources, mirror, transforms):
    """Paint the node primitives for *frame*.

    Reconciles transient offsets on the shared transform frame, then
    submits node quads, port highlights and title-bar icons from the
    render-owned *mirror*. Each submit is skipped when its manager,
    the mirror, the atlas, or the frame's nodes are missing. A
    primitive failure propagates to ``Renderer.draw`` after the
    (already coherent) mirror swap, exactly as the renderer documents.
    """
    _reconcile_transient_offsets(frame, resources)
    _submit_node_quads(frame, resources, mirror, transforms)
    _submit_port_highlights(frame, resources, mirror, transforms)
    _submit_node_icons(frame, resources, mirror, transforms)


def _reconcile_transient_offsets(frame, resources):
    """Drop replaced-identity offsets from the shared transform frame.

    The renderer already reset the frame and translated every
    live-generation offset. When identity filtering -- stale IDs or
    records replaced since the previous draw -- drops something it
    applied, reset again and translate only the surviving offsets.
    Otherwise the frame is untouched. The previous draw's record
    objects are remembered on the render-owned *resources* bundle.
    """
    transform_frame = None
    if resources is not None:
        transform_frame = getattr(resources, "transform_frame", None)
    if transform_frame is None:
        return
    live = select_node_offsets(frame)
    current = {}
    for record in frame.nodes:
        current[record.node_id] = record
    previous = getattr(resources, _CACHE_ATTR, None)
    if isinstance(previous, dict):
        for node_id in list(live):
            old = previous.get(node_id)
            if old is not None and old is not current.get(node_id):
                del live[node_id]
    setattr(resources, _CACHE_ATTR, current)
    inputs = frame.transform_inputs
    applied = {}
    for offset in inputs.offsets:
        if offset.generation == inputs.generation:
            applied[offset.node_id] = (offset.dx, offset.dy)
    if live == applied:
        return
    transform_frame.reset()
    for node_id, (dx, dy) in live.items():
        transform_frame.translate(node_id, dx, dy)


def _theme_value(key, default):
    """The ``NoodlesConfig`` value for *key*, resolved at call time.

    Reads through the legacy module so a patched ``NoodlesConfig``
    binding is honored; when the legacy module was never imported
    (resource-less renderer draws), reads the config module directly.
    Either way the lookup happens per call, never as an import-time
    copy.
    """
    try:
        config = legacy_symbol("NoodlesConfig")
    except (KeyError, AttributeError):
        from ..noodlesConfig import NoodlesConfig as config
    get = getattr(config, "get", None)
    if get is None:
        return default
    try:
        return get(key, default)
    except Exception:
        return default


def _selected_stroke_color():
    """The themed selected-node stroke color, as a detached list."""
    return list(_theme_value("nodeSelectedStrokeColor", [1.0, 1.0, 0.3, 1.0]))


def _submit_node_quads(frame, resources, mirror, transforms):
    """Submit node quads for the mirror; mark quads dirty on change."""
    manager = None
    if resources is not None:
        manager = getattr(resources, "node_render_manager", None)
    if manager is None or mirror is None:
        return
    if not getattr(frame, "nodes", ()):
        return
    atlas = getattr(resources, "font_atlas", None)
    if atlas is None:
        return
    from ..nodeGraph import build_render_config

    config = build_render_config()
    current = selected_node_ids(frame)
    if current != getattr(resources, _SELECTION_ATTR, frozenset()):
        manager.markNodeQuadDirty(True)
    setattr(resources, _SELECTION_ATTR, current)
    content_changed = bool(getattr(frame, "content_changed", False))
    manager.renderNodesFromGraph(
        mirror,
        config,
        atlas,
        list(transforms.world),
        config.nodeCornerRadius,
        _selected_stroke_color(),
        content_changed,
    )


def _submit_port_highlights(frame, resources, mirror, transforms):
    """Submit port highlight circles for the mirror from frame hover."""
    manager = None
    if resources is not None:
        manager = getattr(resources, "node_render_manager", None)
    if manager is None or mirror is None:
        return
    if not getattr(frame, "nodes", ()):
        return
    from ..nodeGraph import build_render_config

    config = build_render_config()
    config.nodePortRingThickness = _theme_value("portRingThickness", 2.0)
    config.portConnectedFillColor = list(
        _theme_value("portConnectedFillColor", [0.7, 0.8, 0.0, 1.0])
    )
    config.portConnectedRingColor = list(
        _theme_value("portConnectedRingColor", [1.0, 1.0, 1.0, 0.0])
    )
    config.portDisconnectedFillColor = list(
        _theme_value("portDisconnectedFillColor", [0.235, 0.235, 0.235, 1.0])
    )
    config.portDisconnectedRingColor = list(
        _theme_value("portDisconnectedRingColor", [0.7, 0.8, 0.0, 1.0])
    )
    config.portHoverColor = list(
        _theme_value("portHoverColor", [0.4, 0.7, 1.0, 1.0])
    )
    hovered = getattr(frame, "hovered_port", None)
    content_changed = bool(getattr(frame, "content_changed", False))
    manager.renderPortHighlightsFromGraph(
        mirror,
        config,
        list(transforms.world),
        config.nodePortWidth * 0.5,
        hovered is not None,
        hovered[0] if hovered else "",
        hovered[1] if hovered else "",
        bool(hovered[2]) if hovered else False,
        False,
        "",
        "",
        False,
        content_changed,
    )


class _TupleProjection:
    """A projection exposing ``QMatrix4x4.data()`` over plain tuples.

    The icon facade still takes a Qt matrix; this shim feeds it the
    draw's world transform without importing Qt into the call path.
    """

    def __init__(self, values):
        self._values = list(values)

    def data(self):
        return list(self._values)


def _submit_node_icons(frame, resources, mirror, transforms):
    """Submit title-bar icons for the mirror from frame records."""
    icons = None
    if resources is not None:
        icons = getattr(resources, "icon_renderer", None)
    if icons is None or mirror is None:
        return
    if not getattr(frame, "nodes", ()):
        return
    if not getattr(icons, "isInitialized", False):
        return
    viewport = frame.viewport
    content_changed = bool(getattr(frame, "content_changed", False))
    icons.renderNodeIcons(
        mirror,
        _TupleProjection(transforms.world),
        viewport.zoom,
        content_changed,
        viewport.pan_x,
        viewport.pan_y,
        float(viewport.width),
        float(viewport.height),
        True,
    )


# -- moved GraphView methods (first parameter ``view``) -------------------------


def paintNodes(view):
    if not view.nodes or not view.shaderLibrary or not view.fontAtlas:
        return

    try:
        projection = view._worldSpaceProjectionMatrix()
        projList = list(projection.data())
        graph = view.nodeGraph

        # Node-quad appearance depends on selection (colors, stroke, and the
        # graffi highlight quad), so a selection-set change must regenerate.
        # Detecting it here covers every selection path (click, marquee,
        # selectAll, clearSelection, prim-tree sync) without flagging each.
        current_selection = frozenset(view._selectedNodes)
        if current_selection != getattr(
            view, "_lastNodeQuadSelection", frozenset()
        ):
            view._nodeRenderManager.markNodeQuadDirty(True)
        view._lastNodeQuadSelection = current_selection

        content_changed = view.textChanged
        needs_rebuild = (
            content_changed or view._nodeRenderManager.needsNodeRebuild()
        )

        # A content change can move a node's rows (pins added/removed, fold
        # toggled, font sizing). Run the single C++ layout producer for every
        # node here — the one chokepoint right before the authoritative
        # snapshot — so each node's display pins, row slots, size, and per-pin
        # centers are recomputed together in one pass from the current
        # content. This closes the stale-cache window for every mutation path
        # (creation, fold, USD-notice/invalidateCache, font change) and is
        # what makes the text/pin/stripe vertical drift structurally
        # impossible. Gated on content_changed so selection-only rebuilds stay
        # cheap (no relayout).
        if content_changed and view.fontAtlas:
            calcWidth = (
                view.textRenderer.calculateTextWidth
                if view.textRenderer
                else lambda t, s: s * 5
            )
            for node in view.nodes.values():
                # Force the lazy pin getters so an invalidated node reloads
                # its raw content before we lay it out from that content.
                _ = node.inputPins
                _ = node.outputPins
                view.nodeGraph._calculateNodeSize(node, calcWidth, view.fontAtlas)

        # GraphModel.nodes is the authoritative render snapshot; re-sync from
        # the Python models only when geometry will be regenerated, then
        # render from it. Keep sync and render adjacent — the freshness
        # invariant depends on no frame boundary between them.
        if needs_rebuild:
            graph.syncNodesFromModels(graph.nodes)

        config = view._renderConfig()
        view._nodeRenderManager.renderNodesFromGraph(
            graph,
            config,
            view.fontAtlas,
            projList,
            config.nodeCornerRadius,
            view._cachedNodeSelectedStrokeColor,
            content_changed,
        )

    except Exception as e:
        Tf.Warn(f"Error rendering nodes: {e}")
        import traceback

        traceback.print_exc()


def _drawNodeVertices(
    view,
    vertexData,
    projection,
    cornerRadius,
    strokeColor,
    generation=0,
    selectionChanged=False,
):
    """
    Draw vertex data using the node shader (used by UI overlays).

    This is a simplified version for drawing dynamic UI overlays like status bars,
    hotkey popups, and marquee selections. Does not use caching since overlay
    data changes every frame.

    Args:
        vertexData: List of NodeVertex objects to render
        projection: Projection matrix for rendering
        cornerRadius: Corner radius for rounded corners
        strokeColor: Stroke color tuple (r, g, b, a)
        generation: Ignored (kept for API compatibility)
        selectionChanged: Ignored (kept for API compatibility)
    """
    if not vertexData or not view.shaderLibrary:
        return

    shader = view.shaderLibrary.get("node")
    if not shader:
        return

    packedData = b"".join(v.pack() for v in vertexData)
    vertexCount = len(vertexData)

    vbo = None
    vao = None
    try:
        vbo = GL.glGenBuffers(1)
        vao = GL.glGenVertexArrays(1)

        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(
            GL.GL_ARRAY_BUFFER, len(packedData), packedData, GL.GL_STREAM_DRAW
        )

        # Configure vertex attributes
        stride = legacy_symbol("NodeVertex").size()
        for (
            index,
            numComponents,
            glTypeName,
            normalized,
            _stride,
            offset,
        ) in NODE_VERTEX_ATTRIB_LAYOUT:
            glType = getattr(GL, glTypeName)
            GL.glVertexAttribPointer(
                index,
                numComponents,
                glType,
                normalized,
                stride,
                ctypes.c_void_p(offset),
            )
            GL.glEnableVertexAttribArray(index)

        # Set up shader and draw
        shader.use()

        matrixData = projection.data()
        matrix = (ctypes.c_float * 16)(*matrixData)
        projLoc = shader.getUniformLocation("uProjection")
        if projLoc >= 0:
            GL.glUniformMatrix4fv(projLoc, 1, GL.GL_FALSE, matrix)

        radiusLoc = shader.getUniformLocation("uCornerRadius")
        if radiusLoc >= 0:
            GL.glUniform1f(radiusLoc, cornerRadius)

        strokeLoc = shader.getUniformLocation("uInnerStrokeColor")
        if strokeLoc >= 0:
            GL.glUniform4f(
                strokeLoc,
                strokeColor[0],
                strokeColor[1],
                strokeColor[2],
                strokeColor[3],
            )

        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, vertexCount)
    except GLError as e:
        Tf.Warn(f"GL error during node rendering: {e}")
    finally:
        # Clean up temporary resources
        GL.glBindVertexArray(0)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        shader.release()
        if vbo is not None:
            GL.glDeleteBuffers(1, [vbo])
        if vao is not None:
            GL.glDeleteVertexArrays(1, [vao])


def _renderPortHighlightsCpp(view):
    """C++-owned port-highlight pass.

    Delegates the entire port-circle / relationship-triangle assembly to
    NodeRenderManager.renderPortHighlightsFromGraph, which reads the
    authoritative GraphModel snapshot (synced in paintNodes/paintLinks) and
    the C++ connection index. This is the sole port-highlight render path.
    """
    graph = view.nodeGraph
    config = view._renderConfig()
    # Push the cached port appearance onto the shared render config (cheap and
    # idempotent); the C++ producer reads these instead of the Python caches.
    # These writes intentionally persist after useCppPortHighlights is toggled
    # off: the fields have no other consumers, so leaving them set is harmless.
    config.nodePortRingThickness = view._cachedPortRingThickness
    config.portConnectedFillColor = list(view._cachedPortConnectedFillColor)
    config.portConnectedRingColor = list(view._cachedPortConnectedRingColor)
    config.portDisconnectedFillColor = list(view._cachedPortDisconnectedFillColor)
    config.portDisconnectedRingColor = list(view._cachedPortDisconnectedRingColor)
    config.portHoverColor = list(view._cachedPortHoverColor)

    projList = list(view._worldSpaceProjectionMatrix().data())
    cornerRadius = config.nodePortWidth * 0.5

    hovered = view._hoveredPort
    # Rebuild the highlight geometry only when something it depends on
    # changed: an in-place connection change (_portHighlightsDirty), a
    # content / color / relayout / move change (textChanged is still set this
    # frame -- paintText clears it later), or an in-progress node / link drag
    # (the circles bake the live drag offset, so they must rebuild each drag
    # frame). Hover and link-drag-state changes are detected C++-side by
    # comparing PortHighlightState, so they need not be flagged here. A plain
    # pan / zoom changes none of these, so the C++ side skips the O(ports)
    # build + hash and just redraws the cached geometry.
    port_geometry_changed = bool(
        view._portHighlightsDirty
        or view.textChanged
        or view.draggingNodes
        or view._draggingLink
    )
    view._nodeRenderManager.renderPortHighlightsFromGraph(
        graph,
        config,
        projList,
        cornerRadius,
        hovered is not None,
        hovered[0] if hovered else "",
        hovered[1] if hovered else "",
        bool(hovered[2]) if hovered else False,
        bool(view._draggingLink),
        view._dragLinkSourceNode or "",
        view._dragLinkSourcePort or "",
        bool(view._dragLinkSourceIsOutput),
        port_geometry_changed,
    )
    view._portHighlightsDirty = False


def paintGroupStickers(view):
    if not view.groupStickers or not view.shaderLibrary:
        return

    try:
        projection = view._worldSpaceProjectionMatrix()
        view._groupStickerRenderer.renderStickers(view.groupStickers, projection)

    except Exception as e:
        Tf.Warn(f"Error rendering group stickers: {e}")
        import traceback

        traceback.print_exc()


def _relationshipLinkColor(view, link, hoveredColor, selectedColor):
    if getattr(link, "hovered", False):
        return tuple(hoveredColor)
    if getattr(link, "selected", False):
        return tuple(selectedColor)
    c = view._cachedLinkRelationshipBaseColor
    return (c[0], c[1], c[2], 1.0)


def _isPrimTargetRelationshipLink(link):
    return (
        bool(getattr(link, "is_relationship_link", False))
        and not str(getattr(link, "targetPort", ""))
        and not str(getattr(link, "targetPropertyName", ""))
    )
