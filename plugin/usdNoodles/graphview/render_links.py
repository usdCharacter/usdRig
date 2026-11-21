#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""GraphView links primitives and legacy adapters."""

import ctypes
import math

from OpenGL import GL
from OpenGL.error import GLError
from pxr import Tf

from ..linkRenderer import sampleLinkCurve
from ..pinUtils import arrowhead_points, is_relationship_pin
from . import render_nodes
from .render_text import _Projection


def paintLinks(view):
    """Legacy unbound-call adapter; production uses the detached draw path."""
    if not view.linkRenderer or not view.links:
        return

    graph = view.nodeGraph
    # Refresh the held C++ link snapshot only when it is stale (hover change
    # or a flagged link-field mutation), then render straight from it — no
    # per-call LinkData marshalling inside the draw. The C++ side filters out
    # dangling links and partitions regular vs relationship (relationshipOnly);
    # the shader insets whole-prim relationship ends for the arrowhead.
    view._syncLinksIfNeeded(graph)

    projection = view._worldSpaceProjectionMatrix()

    # Use cached colors (updated via settings signal, not every frame)
    hoveredColor = view._cachedLinkHoveredColor
    selectedColor = view._cachedLinkSelectedColor
    config = view._renderConfig()

    for drawSelected, dimming in ((True, 0.3), (False, 1.0)):
        view.linkRenderer.renderLinksFromGraph(
            graph,
            projection,
            view.zoom,
            view.panX,
            view.panY,
            view.width(),
            view.height(),
            config,
            relationshipOnly=False,
            dimming=dimming,
            drawSelected=drawSelected,
            hoveredColor=hoveredColor,
            selectedColor=selectedColor,
            baseColor=list(view._cachedLinkAttributeBaseColor[:3]),
            highlightedColor=list(view._cachedLinkHighlightedColor[:3]),
            cacheKey=0,
        )
        view.linkRenderer.renderLinksFromGraph(
            graph,
            projection,
            view.zoom,
            view.panX,
            view.panY,
            view.width(),
            view.height(),
            config,
            relationshipOnly=True,
            dimming=dimming,
            drawSelected=drawSelected,
            hoveredColor=hoveredColor,
            selectedColor=selectedColor,
            baseColor=list(view._cachedLinkRelationshipBaseColor[:3]),
            highlightedColor=list(view._cachedLinkHighlightedColor[:3]),
            cacheKey=1,
        )

    view._renderRelationshipArrowheads(
        projection,
        hoveredColor,
        selectedColor,
    )


def _relationshipArrowheadPoints(
    view,
    link,
    arrow_length,
    arrow_width,
):
    return _arrow_points(link.start, link.end,
                         render_nodes._isPrimTargetRelationshipLink(link),
                         arrow_length, arrow_width)


def _arrow_points(start, end, vertical, length, width):
    tip = tuple(end)
    if vertical:
        previous = (tip[0], tip[1] - length)
    else:
        curve = sampleLinkCurve(tuple(start), tip, 20)
        if len(curve) < 2:
            return None
        previous = tuple(curve[-2])
    return arrowhead_points(*tip, *previous, length=length, width=width)


def _renderRelationshipArrowheads(
    view,
    projection,
    hoveredColor,
    selectedColor,
):
    arrowhead_vertices = []
    arrow_length = max(10.0 / max(view.zoom, 1e-6), 2.0)
    arrow_width = max(8.0 / max(view.zoom, 1e-6), 2.0)
    for link in view.links:
        if not bool(getattr(link, "is_relationship_link", False)):
            continue
        if getattr(link, "isDangling", False):
            continue
        arrow_points = _relationshipArrowheadPoints(
            view,
            link,
            arrow_length,
            arrow_width,
        )
        if arrow_points is None:
            continue
        view._appendTriangleVertices(
            arrowhead_vertices,
            arrow_points,
            0.0,
            view._relationshipLinkColor(link, hoveredColor, selectedColor),
        )
    if arrowhead_vertices:
        view._drawNodeVertices(
            arrowhead_vertices,
            projection,
            0.0,
            (0.0, 0.0, 0.0, 0.0),
        )


def _renderDanglingIndicators(view, danglingLinks, projection):
    """Render circle indicators at the endpoints of dangling links.

    Each dangling link gets a filled circle at the unpopulated end,
    with a hover highlight when the cursor is over it.
    """
    if not view.shaderLibrary:
        return

    shader = view.shaderLibrary.get("basic")
    if not shader:
        return

    INDICATOR_RADIUS = 6.0
    NUM_SEGMENTS = 16

    # Build vertex data for all indicator circles
    vertices = []
    for link in danglingLinks:
        # The indicator goes at the dangling end (link.end)
        cx, cy = link.end[0], link.end[1]

        # Generate triangle fan vertices for the circle
        for i in range(NUM_SEGMENTS):
            angle0 = 2.0 * math.pi * i / NUM_SEGMENTS
            angle1 = 2.0 * math.pi * (i + 1) / NUM_SEGMENTS
            # Triangle: center, edge0, edge1
            vertices.extend([cx, cy, 0.0])
            vertices.extend(
                [
                    cx + INDICATOR_RADIUS * math.cos(angle0),
                    cy + INDICATOR_RADIUS * math.sin(angle0),
                    0.0,
                ]
            )
            vertices.extend(
                [
                    cx + INDICATOR_RADIUS * math.cos(angle1),
                    cy + INDICATOR_RADIUS * math.sin(angle1),
                    0.0,
                ]
            )

    if not vertices:
        return

    # Determine color based on hover state
    hoveredIdx = view._danglingIndicatorUnderCursor
    hasHover = hoveredIdx >= 0

    try:
        vbo = vao = None
        vbo = GL.glGenBuffers(1)
        vao = GL.glGenVertexArrays(1)

        packedData = (ctypes.c_float * len(vertices))(*vertices)

        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(
            GL.GL_ARRAY_BUFFER,
            ctypes.sizeof(packedData),
            packedData,
            GL.GL_STREAM_DRAW,
        )

        # Position attribute (location 0, vec3)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(
            0, 3, GL.GL_FLOAT, GL.GL_FALSE, 12, ctypes.c_void_p(0)
        )

        shader.use()

        matrixData = projection.data()
        matrix = (ctypes.c_float * 16)(*matrixData)
        projLoc = shader.getUniformLocation("uProjection")
        if projLoc >= 0:
            GL.glUniformMatrix4fv(projLoc, 1, GL.GL_FALSE, matrix)

        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        vertsPerCircle = NUM_SEGMENTS * 3

        # Draw non-hovered indicators in base color
        colorLoc = shader.getUniformLocation("uColor")
        baseColor = (0.55, 0.57, 0.60, 0.85)
        hoverColor = (0.85, 0.90, 0.95, 1.0)

        for i, link in enumerate(danglingLinks):
            linkIdx = view.links.index(link) if hasHover else -1
            isHovered = hasHover and linkIdx == hoveredIdx
            color = hoverColor if isHovered else baseColor
            if colorLoc >= 0:
                GL.glUniform4f(colorLoc, *color)
            GL.glDrawArrays(GL.GL_TRIANGLES, i * vertsPerCircle, vertsPerCircle)

    except GLError as e:
        Tf.Warn(f"GL error during dangling indicator rendering: {e}")
    finally:
        GL.glBindVertexArray(0)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        shader.release()
        if vbo is not None:
            GL.glDeleteBuffers(1, [vbo])
        if vao is not None:
            GL.glDeleteVertexArrays(1, [vao])


def _renderTemporaryLink(view):
    """
    Render the temporary link being dragged with color feedback.

    Color indicates connection validity:
    - Green: Valid target port (correct direction)
    - Red: Invalid target (same direction as source)
    - Active color (configurable): Dragging (no port hovered)
    """
    # Determine base color based on validity
    if view._dragLinkValidTarget:
        # Green for valid target
        baseColor = [0.0, 1.0, 0.0]
    elif view._hoveredPort and view._hoveredPort[2] == view._dragLinkSourceIsOutput:
        # Red for invalid target (same direction)
        baseColor = [1.0, 0.0, 0.0]
    else:
        # Active/dragging color (cached, updated via settings signal)
        if is_relationship_pin(str(view._dragLinkSourcePort)):
            baseColor = list(view._cachedLinkRelationshipBaseColor[:3])
        else:
            activeColor = view._cachedLinkActiveColor
            baseColor = [activeColor[0], activeColor[1], activeColor[2]]

    # The free end is the cursor side; the anchored end is the source pin.
    # Snapshot both as plain float tuples so the inset/restore below cannot
    # be aliased by the C++ LinkData accessors.
    temp_link = view._dragLinkTempLink
    is_relationship = bool(getattr(temp_link, "is_relationship_link", False))
    if view._dragLinkSourceIsOutput:
        # Dragging from output, free end is at 'end' (input side)
        free_end_attr = "end"
        endPos = (float(temp_link.end[0]), float(temp_link.end[1]))
        startPos = (float(temp_link.start[0]), float(temp_link.start[1]))
    else:
        # Dragging from input, free end is at 'start' (output side)
        free_end_attr = "start"
        endPos = (float(temp_link.start[0]), float(temp_link.start[1]))
        startPos = (float(temp_link.end[0]), float(temp_link.end[1]))

    arrow_length = max(12.0 / max(view.zoom, 1e-6), 4.0)
    arrow_width = max(10.0 / max(view.zoom, 1e-6), 3.0)
    # Face the arrowhead down only when snapped to a valid relationship
    # target (dropped on a node's top edge); otherwise it faces left/right.
    use_vertical = is_relationship and view._dragLinkValidTarget

    # Render using existing link renderer.
    # Pass as a single-element list with baseColor to override uniform.
    # When snapped to a whole-prim relationship target the noodle must stop
    # at the arrowhead BASE, not overshoot to its tip (the node top). Committed
    # links get that inset from the shader via the per-instance isPrimTarget
    # flag, but the per-call renderLinks path used for the drag preview always
    # passes isPrimTarget=0 -- so inset the free end here by the same
    # arrow_length the arrowhead uses, then restore it before the arrowhead
    # and hit geometry, which key off the true node top.
    projection = view._worldSpaceProjectionMatrix()
    hoveredColor = view._cachedLinkHoveredColor
    selectedColor = view._cachedLinkSelectedColor
    saved_free_end = None
    if use_vertical:
        saved_free_end = endPos
        setattr(temp_link, free_end_attr, (endPos[0], endPos[1] - arrow_length))
    view.linkRenderer.renderLinks(
        [temp_link],
        projection,
        view.zoom,
        view.panX,
        view.panY,
        view.width(),
        view.height(),
        view._renderConfig(),
        dimming=1.0,
        drawSelected=False,
        hoveredColor=hoveredColor,
        selectedColor=selectedColor,
        baseColor=baseColor,
        cacheKey=2,
    )
    if saved_free_end is not None:
        setattr(temp_link, free_end_attr, saved_free_end)

    projList = list(projection.data())

    if is_relationship:
        # Draw triangle for relationship links
        dx = endPos[0] - startPos[0]

        if use_vertical:
            # Match the persistent arrowhead layout
            # (_relationshipArrowheadPoints): tip sits exactly on the top
            # edge of the target prim and the triangle body extends UPWARD.
            # The previous +arrow_length/2
            # offset pushed the tip into the prim and made the arrowhead
            # read as "under" the node during a reroute drag.
            tip_x = endPos[0]
            tip_y = endPos[1]
            prev_x = tip_x
            prev_y = tip_y - arrow_length
        elif dx > 0:
            tip_x = endPos[0] + arrow_length * 0.5
            tip_y = endPos[1]
            prev_x = tip_x - arrow_length
            prev_y = tip_y
        else:
            tip_x = endPos[0] - arrow_length * 0.5
            tip_y = endPos[1]
            prev_x = tip_x + arrow_length
            prev_y = tip_y

        arrow_points = arrowhead_points(
            tip_x,
            tip_y,
            prev_x,
            prev_y,
            length=arrow_length,
            width=arrow_width,
        )
        if len(arrow_points) < 3:
            return

        view._nodeRenderManager.renderEndpointTriangle(
            arrow_points[0][0],
            arrow_points[0][1],
            arrow_points[1][0],
            arrow_points[1][1],
            arrow_points[2][0],
            arrow_points[2][1],
            0.0,
            projList,
            baseColor + [1.0],
        )
    else:
        # Draw circle for regular links via SDF quad (matches port pin visual)
        renderer = view.defaultRenderer
        radius = renderer.getPortWidth() if renderer else 8.0
        strokeWidth = view._cachedPortRingThickness
        strokeColor = [c * 0.7 for c in baseColor] + [1.0]
        view._nodeRenderManager.renderEndpointCircle(
            endPos[0],
            endPos[1],
            radius,
            0.0,
            projList,
            baseColor + [1.0],
            strokeColor,
            strokeWidth,
        )


def draw(frame, resources, mirror, transforms):
    """Draw ribbons first, then endpoint affordances and the drag preview."""
    renderer = getattr(resources, 'link_renderer', None)
    manager = getattr(resources, 'node_render_manager', None)
    config = getattr(resources, 'link_config', None)
    if renderer is not None and config is None:
        from ..nodeGraph import build_render_config

        config = build_render_config()
    viewport = frame.viewport
    projection = _Projection(transforms.world)
    args = (projection, viewport.zoom, viewport.pan_x, viewport.pan_y,
            viewport.width, viewport.height, config)
    hovered, selected, attribute, relationship, highlighted = frame.link_colors
    if frame.draw_links:
        if renderer is not None and mirror is not None and frame.links:
            for draw_selected, dimming in ((True, 0.3), (False, 1.0)):
                for rel, color in ((False, attribute), (True, relationship)):
                    renderer.renderLinksFromGraph(
                        mirror, *args, relationshipOnly=rel, dimming=dimming,
                        drawSelected=draw_selected, hoveredColor=list(hovered),
                        selectedColor=list(selected), baseColor=list(color[:3]),
                        highlightedColor=list(highlighted[:3]), cacheKey=int(rel),
                    )
        if manager is not None:
            for link in frame.links:
                if not link.is_relationship or link.is_dangling:
                    continue
                length = max(10.0 / max(viewport.zoom, 1e-6), 2.0)
                width = max(8.0 / max(viewport.zoom, 1e-6), 2.0)
                points = _arrow_points(link.start, link.end, not link.link_id.target_pin and not link.target_property,
                                       length, width)
                color = hovered if link.hovered else selected if link.selected else relationship
                _triangle(manager, projection, points, color)
            for link in frame.dangling_links:
                color = ((0.85, 0.90, 0.95, 1.0)
                         if frame.dangling_hover >= 0 and link.index == frame.dangling_hover
                         else (0.55, 0.57, 0.60, 0.85))
                manager.renderEndpointCircle(*link.end, 6.0, 0.0, projection.data(),
                                             list(color), list(color), 0.0)

    temp = frame.temporary_link
    if temp is None:
        return
    link = temp.link
    anchor, end = (link.start, link.end) if temp.free_at_end else (link.end, link.start)
    length = max(12.0 / max(viewport.zoom, 1e-6), 4.0)
    width = max(10.0 / max(viewport.zoom, 1e-6), 3.0)
    if renderer is not None:
        from dataclasses import replace
        from UsdNoodles.core import LinkData
        from .render import _populate_link

        wire = link
        if temp.snapped:
            inset = (end[0], end[1] - length)
            wire = replace(link, end=inset) if temp.free_at_end else replace(link, start=inset)
        native = LinkData()
        _populate_link(native, wire)
        renderer.renderLinks([native], *args, baseColor=list(temp.color),
                             hoveredColor=list(hovered), selectedColor=list(selected), cacheKey=2)
    if manager is None:
        return
    color = (*temp.color, 1.0)
    if link.is_relationship:
        if temp.snapped:
            tip, previous = end, (end[0], end[1] - length)
        else:
            sign = 1 if end[0] > anchor[0] else -1
            tip = (end[0] + sign * length * 0.5, end[1])
            previous = (tip[0] - sign * length, tip[1])
        _triangle(manager, projection, arrowhead_points(*tip, *previous, length=length, width=width), color)
    else:
        manager.renderEndpointCircle(*end, temp.radius, 0.0, projection.data(),
                                     list(color), [*(c * 0.7 for c in temp.color), 1.0],
                                     temp.stroke_width)


def _triangle(manager, projection, points, color):
    if points and len(points) == 3:
        manager.renderEndpointTriangle(*(v for point in points for v in point),
                                       0.0, projection.data(), list(color))
