#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Hit testing, rows, hover and viewport jumps: the interaction owner reads.

Every function here reads geometry and hit candidates; nothing authors to
the stage, pushes history, or mutates live native link data. Hover queries
record intent on the view (``_hoveredPort``, ``_valueHover``,
``_tooltipProperty``); the ``_syncLinksIfNeeded`` family that mirrors hover
onto live links stays with its owner for ``nv-render-links``.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``. Names the suite patches on the legacy module
(``QtWidgets``) and legacy-only helpers (``_ProfileSection``,
``_ValueHit``) resolve at call time through ``legacy_symbol``; everything
else is imported directly.
"""

from pxr import Gf
from pxr.Usdviewq.qt import QtCore

from ..widgets import valueCell
from . import selection as _selection
from .legacy import legacy_symbol
from .stage_sync import _nodeIdPrimPath


def _getNodeIdAtWorldPos(view, worldPos, margin=0.0):
    if margin > 0.0:
        queryBounds = Gf.Range2d(
            Gf.Vec2d(worldPos[0] - margin, worldPos[1] - margin),
            Gf.Vec2d(worldPos[0] + margin, worldPos[1] + margin),
        )
        nodeIds, _ = view._spatialIndex.queryRegion(queryBounds)
    else:
        nodeIds, _ = view._spatialIndex.queryPoint(worldPos)

    sortedNodeIds = sorted(
        (nid for nid in nodeIds if nid in view.nodes),
        key=lambda nid: view.nodes[nid].zOrder,
        reverse=True,
    )

    margin_vec = Gf.Vec2d(float(margin), float(margin))
    for nodeId in sortedNodeIds:
        node = view.nodes[nodeId]
        nodeBounds = Gf.Range2d(
            node.position - margin_vec,
            node.position + node.size + margin_vec,
        )
        if nodeBounds.Contains(worldPos):
            return nodeId
    return ""


def _getPropertyAtPoint(view, screenPos):
    """Return (nodeId, pinName) for the property row under screenPos, or None."""
    worldMousePos = (QtCore.QPointF(screenPos) / view.zoom) + QtCore.QPointF(
        view.panX, view.panY
    )
    worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

    renderer = view.defaultRenderer
    if not renderer:
        return None

    # Iterate topmost-first so overlapping nodes return the visible one.
    sortedNodes = sorted(
        view.nodes.items(),
        key=lambda kv: getattr(kv[1], "zOrder", 0),
        reverse=True,
    )
    for nodeId, node in sortedNodes:
        if node.is_title_collapsed:
            continue
        if (
            worldPos[0] < node.position[0]
            or worldPos[0] > node.position[0] + node.size[0]
        ):
            continue

        # Each pin's band is its cached center +/- half the row height — the
        # same per-pin layout the renderer uses, so clicks match what's drawn.
        half = node.layoutPortLineHeight * 0.5
        inputCenters = node.layoutInputCenterY
        for i, pinName in enumerate(node.inputPins):
            if i >= len(inputCenters):
                break
            center = node.position[1] + inputCenters[i]
            if center - half <= worldPos[1] <= center + half:
                return (nodeId, pinName)

        outputCenters = node.layoutOutputCenterY
        for i, pinName in enumerate(node.outputPins):
            if i >= len(outputCenters):
                break
            center = node.position[1] + outputCenters[i]
            if center - half <= worldPos[1] <= center + half:
                return (nodeId, pinName)

    return None


def _findPropertyRowHover(view, node, nodeId, worldPos):
    _temp_link = getattr(view, "_dragLinkTempLink", None)
    if not view._draggingLink or not getattr(
        _temp_link, "is_relationship_link", False
    ):
        return None
    if getattr(node, "_title_collapsed", False):
        return None
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    if (
        worldPos[0] < node.position[0]
        or worldPos[0] > node.position[0] + node.size[0]
    ):
        return None

    is_relationship_pin_for_node = getattr(
        node,
        "is_relationship_pin",
        lambda _pin_name, _is_output: False,
    )
    port_start_y, port_line_height = view._getRowHitMetrics(node, renderer)
    center_x = node.position[0] + node.size[0] * 0.5
    prefer_output = worldPos[0] >= center_x
    output_spec = (
        True,
        getattr(node, "outputPins", []),
        list(node.outputRowKinds),
        list(node.outputRowSlots),
    )
    input_spec = (
        False,
        getattr(node, "inputPins", []),
        list(node.inputRowKinds),
        list(node.inputRowSlots),
    )
    side_specs = (
        [output_spec, input_spec] if prefer_output else [input_spec, output_spec]
    )

    closest_hover = None
    closest_distance = None
    for is_output, pins, row_kinds, row_slots in side_specs:
        for i, pin_name in enumerate(pins):
            if is_relationship_pin_for_node(pin_name, is_output):
                continue

            row_kind = row_kinds[i] if i < len(row_kinds) else 0
            if row_kind == 1 or row_kind == 2:
                continue

            row_slot = row_slots[i] if i < len(row_slots) else i
            row_y = port_start_y + row_slot * port_line_height
            row_y1 = row_y + port_line_height
            if worldPos[1] < row_y or worldPos[1] > row_y1:
                continue

            port_pos = view._getPortPosition(node, pin_name, is_output)
            row_center_y = (
                float(port_pos[1])
                if port_pos is not None
                else row_y + port_line_height * 0.5
            )
            # Prefer the side closest to the cursor half when both sides
            # share the same visible row slot.
            side_bias = 0.0 if (is_output == prefer_output) else port_line_height
            distance = abs(float(worldPos[1]) - row_center_y) + side_bias
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_hover = (nodeId, pin_name, is_output)
    return closest_hover


def _findRelationshipRowHover(view, node, nodeId, worldPos):
    _temp_link = getattr(view, "_dragLinkTempLink", None)
    if not view._draggingLink or not getattr(
        _temp_link, "is_relationship_link", False
    ):
        return None
    if getattr(node, "_title_collapsed", False):
        return None
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    if (
        worldPos[0] < node.position[0]
        or worldPos[0] > node.position[0] + node.size[0]
    ):
        return None

    is_relationship_pin_for_node = getattr(
        node,
        "is_relationship_pin",
        lambda _pin_name, _is_output: False,
    )
    output_row_kinds = list(node.outputRowKinds)
    output_row_slots = list(node.outputRowSlots)
    port_start_y, port_line_height = view._getRowHitMetrics(node, renderer)

    closest_hover = None
    closest_distance = None
    for i, pinName in enumerate(getattr(node, "outputPins", [])):
        if not is_relationship_pin_for_node(pinName, True):
            continue
        if i < len(output_row_kinds) and output_row_kinds[i] == 2:
            continue
        row_slot = output_row_slots[i] if i < len(output_row_slots) else i
        row_y = port_start_y + row_slot * port_line_height
        row_y1 = row_y + port_line_height
        if worldPos[1] < row_y or worldPos[1] > row_y1:
            continue
        port_pos = view._getPortPosition(node, pinName, True)
        row_center_y = (
            float(port_pos[1])
            if port_pos is not None
            else row_y + port_line_height * 0.5
        )
        distance = abs(float(worldPos[1]) - row_center_y)
        if closest_distance is None or distance < closest_distance:
            closest_distance = distance
            closest_hover = (nodeId, pinName, True)
    return closest_hover


def _relationshipDragRowHover(view, worldPos, portExtent):
    """Whole-row relationship/property drop target during a relationship drag.

    Returns ``(nodeId, pinName, isOutput)`` for the relationship or property
    row under ``worldPos``, or ``None``. Only active while dragging a
    relationship link; the port glyphs themselves are hit by the C++
    ``portAtPoint`` pass in :meth:`_updateHoveredPort`.
    """
    _temp_link = getattr(view, "_dragLinkTempLink", None)
    if not view._draggingLink or not getattr(
        _temp_link, "is_relationship_link", False
    ):
        return None

    candidateNodes = []
    if view.nodeIdUnderCursor and view.nodeIdUnderCursor in view.nodes:
        candidateNodes.append(view.nodeIdUnderCursor)
    queryBounds = Gf.Range2d(
        Gf.Vec2d(worldPos[0] - portExtent, worldPos[1] - portExtent),
        Gf.Vec2d(worldPos[0] + portExtent, worldPos[1] + portExtent),
    )
    nearbyNodeIds, _ = view._spatialIndex.queryRegion(queryBounds)
    for nodeId in nearbyNodeIds:
        if nodeId not in candidateNodes:
            candidateNodes.append(nodeId)

    for nodeId in candidateNodes:
        node = view.nodes.get(nodeId)
        if node is None or not node.renderer:
            continue
        rowHover = view._findRelationshipRowHover(node, nodeId, worldPos)
        if rowHover is None:
            rowHover = view._findPropertyRowHover(node, nodeId, worldPos)
        if rowHover is not None:
            return rowHover
    return None


def _getRowHitMetrics(view, node, renderer):
    # Read from the cached layout (single source of truth) so hover row bands
    # match the rendered rows exactly. port_start_y is the band-area top (world);
    # row_slot * port_line_height gives each band top, [top, top+line_height].
    port_line_height = node.layoutPortLineHeight
    if port_line_height <= 0.0:
        # Layout not computed yet — degrade without crashing.
        port_line_height = max(renderer.getPortWidth() * 2.0, 16.0)
    return node.position[1] + node.layoutPortStartY, port_line_height


def _valueCellAtPoint(view, worldPoint):
    """The value cell under a WORLD position, or None."""
    if not view._cachedShowAttributeValues:
        return None
    x, y = view._asWorldXY(worldPoint)
    nodeId = view.nodeIdUnderCursor
    if not nodeId or nodeId not in view.nodes:
        nodeId = view._getNodeIdAtWorldPos(Gf.Vec2d(x, y))
    if not nodeId or nodeId not in view.nodes:
        return None
    node = view.nodes[nodeId]
    # The paint pass skips cells below _cachedValueMinPixelHeight; testing
    # what is not drawn would arm invisible cells, and costs a full
    # row-geometry walk per mouse-move for nothing.
    if (
        float(node.layoutPortLineHeight) * view.zoom
        < float(view._cachedValueMinPixelHeight)
    ):
        return None
    primPath = _nodeIdPrimPath(nodeId)
    for row, rect, subs, _h in view._valueRowGeometry(node):
        if not (rect.top <= y <= rect.bottom):
            continue
        if not valueCell.rect_contains(rect, x, y):
            continue
        # The gaps between sub-cells belong to the cell too: a press that
        # lands a pixel off a component must not fall through to a node
        # drag, so it resolves to the nearest component instead.
        index = min(
            range(len(subs)),
            key=lambda i: abs(x - (subs[i].left + subs[i].right) * 0.5),
        )
        for i, sub in enumerate(subs):
            if valueCell.rect_contains(sub, x, y):
                index = i
                break
        return legacy_symbol("_ValueHit")(nodeId, primPath, row.property_name, index, subs[index], row)
    return None


def _findRowEdgeDragStart(view, node, worldPos):
    """Resolve a press near a node's left/right edge to ``(pin, is_output)``
    for starting a new connection.

    Only the outer tenth of the node width on each side starts a connection:
    a press in the left band starts an input-side drag, the right band an
    output-side drag. A press in the middle of the row, on the title (above
    the first row), or below the last row falls through to a node-body drag
    (moving the node). A row is grabbable from its side even when no pin
    glyph is drawn (REQ 3 hides it until the pin is connected or dragged) -
    aim near the node edge rather than the inward-offset row label.
    """
    if getattr(node, "_title_collapsed", False):
        return None
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    nx = float(node.position[0])
    nw = float(node.size[0])
    if worldPos[0] < nx or worldPos[0] > nx + nw:
        return None
    # Only the outer tenth of each side starts a connection; the middle 80%
    # stays a node-body drag handle for moving the node.
    edge_band = nw * 0.1
    if worldPos[0] >= nx + nw - edge_band:
        is_output = True
    elif worldPos[0] <= nx + edge_band:
        is_output = False
    else:
        return None
    pin_name = view._pinAtRowEdge(node, worldPos, is_output)
    if pin_name is None:
        return None
    return (pin_name, is_output)


def _rowEdgePinCandidates(view, node, is_output):
    """(pin_name, row_slot) pairs that own a port on the ``is_output`` side.

    Output candidates include real output pins plus dual pins (whose right
    edge mirrors an input row); input candidates likewise include real input
    pins plus output-dual pins (whose left edge mirrors an output row). So a
    press on either edge of any dual row resolves to the right pin regardless
    of which list backs it.
    """
    candidates = []
    if is_output:
        out_pins = getattr(node, "outputPins", [])
        out_slots = list(node.outputRowSlots)
        out_kinds = list(node.outputRowKinds)
        for i, pin in enumerate(out_pins):
            if i < len(out_kinds) and out_kinds[i] == 2:
                continue
            candidates.append((pin, out_slots[i] if i < len(out_slots) else i))
        out_set = set(out_pins)
        dual = getattr(node, "_dual_pin_names", set())
        in_pins = getattr(node, "inputPins", [])
        in_slots = list(node.inputRowSlots)
        in_kinds = list(node.inputRowKinds)
        for i, pin in enumerate(in_pins):
            if pin not in dual or pin in out_set:
                continue
            if i < len(in_kinds) and in_kinds[i] == 2:
                continue
            candidates.append((pin, in_slots[i] if i < len(in_slots) else i))
    else:
        in_pins = getattr(node, "inputPins", [])
        in_slots = list(node.inputRowSlots)
        in_kinds = list(node.inputRowKinds)
        for i, pin in enumerate(in_pins):
            if i < len(in_kinds) and in_kinds[i] == 2:
                continue
            candidates.append((pin, in_slots[i] if i < len(in_slots) else i))
        in_set = set(in_pins)
        out_dual = getattr(node, "_output_dual_pin_names", set())
        out_pins = getattr(node, "outputPins", [])
        out_slots = list(node.outputRowSlots)
        out_kinds = list(node.outputRowKinds)
        for i, pin in enumerate(out_pins):
            if pin not in out_dual or pin in in_set:
                continue
            if i < len(out_kinds) and out_kinds[i] == 2:
                continue
            candidates.append((pin, out_slots[i] if i < len(out_slots) else i))
    return candidates


def _pinAtRowEdge(view, node, worldPos, is_output):
    """Pin whose row band contains ``worldPos[1]`` on the ``is_output`` side."""
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    port_start_y, port_line_height = view._getRowHitMetrics(node, renderer)
    best = None
    best_distance = None
    for pin_name, row_slot in view._rowEdgePinCandidates(node, is_output):
        row_y = port_start_y + row_slot * port_line_height
        if worldPos[1] < row_y or worldPos[1] > row_y + port_line_height:
            continue
        port_pos = view._getPortPosition(node, pin_name, is_output)
        row_center_y = (
            float(port_pos[1])
            if port_pos is not None
            else row_y + port_line_height * 0.5
        )
        distance = abs(float(worldPos[1]) - row_center_y)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best = pin_name
    return best


def _findArrowheadUnderCursor(view, worldPos):
    """Index into self.links of the relationship link whose arrowhead is
    under worldPos, or -1. Grabbing the arrowhead (the link's target end)
    starts a reconnect; dropping it on empty space disconnects the link.
    """
    arrow_length = max(10.0 / max(view.zoom, 1e-6), 2.0)
    arrow_width = max(8.0 / max(view.zoom, 1e-6), 2.0)
    hit_radius = max(arrow_length, arrow_width)
    hitRadiusSq = hit_radius * hit_radius
    best = -1
    best_distance = None
    for i, link in enumerate(view.links):
        if not bool(getattr(link, "is_relationship_link", False)):
            continue
        if getattr(link, "isDangling", False):
            continue
        dx = worldPos[0] - float(link.end[0])
        dy = worldPos[1] - float(link.end[1])
        distance = dx * dx + dy * dy
        if distance <= hitRadiusSq and (
            best_distance is None or distance < best_distance
        ):
            best_distance = distance
            best = i
    return best


def _findDanglingIndicatorUnderCursor(view, worldPos):
    """Find which dangling link's circle indicator is under the cursor.

    Returns the index into self.links of the dangling link whose indicator
    contains worldPos, or -1 if none.
    """
    INDICATOR_RADIUS = 6.0
    HIT_RADIUS = INDICATOR_RADIUS + 4.0
    hitRadiusSq = HIT_RADIUS * HIT_RADIUS

    for i, link in enumerate(view.links):
        if not link.isDangling:
            continue
        cx, cy = link.end[0], link.end[1]
        dx = worldPos[0] - cx
        dy = worldPos[1] - cy
        if dx * dx + dy * dy <= hitRadiusSq:
            return i
    return -1


def _findDanglingLinkAtPort(view, worldPos):
    """Find a dangling link whose present-node port is under worldPos.

    Returns the index into self.links, or -1 if none.
    """
    for i, link in enumerate(view.links):
        if not link.isDangling:
            continue
        if link.sourceNodeId in view.nodes:
            nodeId = link.sourceNodeId
            portName = link.sourcePort
            isOutput = True
        elif link.targetNodeId in view.nodes:
            nodeId = link.targetNodeId
            portName = link.targetPort
            isOutput = False
        else:
            continue
        node = view.nodes.get(nodeId)
        if not node:
            continue
        renderer = node.renderer if node.renderer else view.defaultRenderer
        if not renderer:
            continue
        portPos = view._getPortPosition(node, portName, isOutput)
        if not portPos:
            continue
        hitRadius = renderer.getPortWidth() * view._cachedPortHitRadiusMultiplier
        dx = worldPos[0] - portPos[0]
        dy = worldPos[1] - portPos[1]
        if dx * dx + dy * dy <= hitRadius * hitRadius:
            return i
    return -1


def _updateHoveredPort(view, pos):
    """
    Update which port (if any) the cursor is hovering over.

    Args:
        pos: Screen position of cursor (QPoint)

    Returns:
        bool: True if hover state changed, False otherwise
    """
    worldMousePos = (QtCore.QPointF(pos) / view.zoom) + QtCore.QPointF(
        view.panX, view.panY
    )
    worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

    # Hit radius = the drawn port circle (renderer.getPortWidth() ==
    # nodePortWidth / 2) scaled by the configured multiplier. One radius for
    # every port kind, matching what is drawn.
    portWidth = view.defaultRenderer.getPortWidth() if view.defaultRenderer else 8.0
    hitRadius = portWidth * view._cachedPortHitRadiusMultiplier

    # Single C++ hit-test over every node's ports (normal / folded-header /
    # dual / output-dual / synthetic relationship / collapsed aggregate),
    # spatially rejected to the cursor's neighbourhood inside portAtPoint.
    hit = view._nodeRenderManager.portAtPoint(
        view.nodeGraph,
        worldPos[0],
        worldPos[1],
        hitRadius,
    )
    if hit and hit["found"]:
        return view._setHoveredPort(
            (hit["nodeId"], hit["portName"], hit["isOutput"])
        )

    # During a relationship-link drag an attribute can also land on a whole
    # relationship / property row (a wider target than the port glyph).
    rowHover = view._relationshipDragRowHover(worldPos, portWidth * 2.0)
    if rowHover is not None:
        return view._setHoveredPort(rowHover)

    # No port found - clear hover if needed
    if view._hoveredPort is not None:
        view._hoveredPort = None
        return True

    return False  # No change


def _setHoveredPort(view, newHover):
    """Update _hoveredPort, returning True iff it actually changed."""
    if view._hoveredPort != newHover:
        view._hoveredPort = newHover
        return True
    return False


def _updateNodeUnderCursor(view, pos):
    with legacy_symbol("_ProfileSection")(view._profiler, "Update Node Under Cursor"):
        worldMousePos = (QtCore.QPointF(pos) / view.zoom) + QtCore.QPointF(
            view.panX, view.panY
        )
        mousePoint = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
        view.nodeIdUnderCursor = view._getNodeIdAtWorldPos(mousePoint)


def _updateValueHoverCursor(view, screenPos):
    """Track the hovered cell: its fill lifts and the cursor changes."""
    hit = None
    if not (view.draggingNodes or view._draggingLink or view.spacePressed):
        world = (QtCore.QPointF(screenPos) / view.zoom) + QtCore.QPointF(
            view.panX, view.panY
        )
        hit = view._valueCellAtPoint(world)

    hover = None
    cursor = None
    if hit is not None:
        hover = (hit.nodeId, hit.propertyName, hit.comp)
        if hit.row.editable:
            cursor = view._VALUE_CURSORS.get(hit.row.kind)
    if hover != view._valueHover:
        view._valueHover = hover
        view.update()
    if cursor != view._valueCursorSet:
        view.setCursor(cursor if cursor is not None else QtCore.Qt.ArrowCursor)
        view._valueCursorSet = cursor


def _showPropertyTooltip(view):
    """Show the tooltip for the currently hovered property."""
    if view._tooltipProperty is None or view._tooltipScreenPos is None:
        return
    nodeId, pinName = view._tooltipProperty
    node = view.nodes.get(nodeId)
    if node is None:
        return
    doc = ""
    if hasattr(node, "get_pin_documentation"):
        doc = node.get_pin_documentation(pinName)
    if not doc:
        return
    legacy_symbol("QtWidgets").QToolTip.showText(view._tooltipScreenPos, doc, view)


def _updatePropertyTooltip(view, screenPos):
    """Update tooltip state based on which property row the cursor is over."""
    prop = view._getPropertyAtPoint(screenPos)
    if prop != view._tooltipProperty:
        view._tooltipTimer.stop()
        legacy_symbol("QtWidgets").QToolTip.hideText()
        view._tooltipProperty = prop
        if prop is not None:
            view._tooltipTimer.start()
    if prop is not None:
        # Track the latest cursor position so the eventual tooltip appears
        # where the user is actually pointing, not where they entered the row.
        view._tooltipScreenPos = view.mapToGlobal(screenPos)


def _clearPropertyTooltip(view):
    view._tooltipTimer.stop()
    view._tooltipProperty = None
    legacy_symbol("QtWidgets").QToolTip.hideText()


def _centerOnNode(view, node, animate=True):
    nodeCenter = node.position + node.size * 0.5

    # Calculate the viewport size in world coordinates
    vpWidth = view.width() / view.zoom
    vpHeight = view.height() / view.zoom

    # Calculate target pan position
    targetPanX = nodeCenter[0] - vpWidth * 0.5
    targetPanY = nodeCenter[1] - vpHeight * 0.5

    if animate:
        # Set up animated pan with cached speed
        panSpeed = view._cachedPanAnimationSpeed

        view._panXAnim.current = view.panX
        view._panXAnim.target = targetPanX
        view._panXAnim.speed = panSpeed

        view._panYAnim.current = view.panY
        view._panYAnim.target = targetPanY
        view._panYAnim.speed = panSpeed

        view._panAnimating = True
    else:
        # Immediate jump
        view.panX = targetPanX
        view.panY = targetPanY
        view._panAnimating = False

    view.update()


def _setZoom(view, newZoom):
    center = Gf.Vec2d(view.width() * 0.5, view.height() * 0.5)
    view._zoomAroundPoint(newZoom, center)
    view.update()


def _zoomAroundPoint(view, newZoom, focusPoint):
    """Zoom the view around a specific point.

    Zoom limits:
    - Min zoom: graph bounds occupy about 25% of screen (can zoom out 4x beyond fit)
    - Max zoom: a typical node (200x100) takes up about 1/5 of the screen
    """
    # Calculate min zoom so graph bounds occupy ~25% of screen
    # This means we can zoom out 4x beyond the "fit to screen" level
    minZoom = 0.001
    graphBounds = view.minimap.getGraphBounds()
    if not graphBounds.IsEmpty():
        graphSize = graphBounds.GetSize()
        if graphSize[0] > 0 and graphSize[1] > 0:
            # Calculate zoom where graph fills screen, then allow 4x more zoom out
            # (graph at 25% of screen = 4x zoom out from fill)
            zoomForWidth = view.width() / graphSize[0] if graphSize[0] > 0 else 1.0
            zoomForHeight = (
                view.height() / graphSize[1] if graphSize[1] > 0 else 1.0
            )
            fitZoom = min(zoomForWidth, zoomForHeight)
            minZoom = fitZoom * 0.25  # Allow zooming out to 25% of fit
            minZoom = max(0.001, minZoom)

    # Calculate max zoom so one node takes up about 1/5 of the screen
    # Typical node size is around 200x100 units
    typicalNodeWidth = 200.0
    typicalNodeHeight = 100.0
    # Node should be ~1/5 of screen, so screen = 5 * node
    maxZoomForWidth = (
        view.width() / (typicalNodeWidth * 5.0) if typicalNodeWidth > 0 else 100.0
    )
    maxZoomForHeight = (
        view.height() / (typicalNodeHeight * 5.0)
        if typicalNodeHeight > 0
        else 100.0
    )
    # Use the larger zoom (more zoomed in) but cap at reasonable maximum
    maxZoom = min(max(maxZoomForWidth, maxZoomForHeight), 100.0)
    # Ensure max is at least greater than min
    maxZoom = max(maxZoom, minZoom * 2.0)

    newZoom = max(minZoom, min(newZoom, maxZoom))
    focus = Gf.Vec2d(focusPoint[0], focusPoint[1])
    delta = focus * (1.0 / view.zoom - 1.0 / newZoom)
    view.panX += delta[0]
    view.panY += delta[1]
    view.zoom = newZoom


def navigateBackward(view):
    """Navigate backward (toward inputs/sources) using the [ key.

    Behavior:
    - If a link is selected: jump to the source node (data provider)
    - If a node is selected: select the first input link
    - If the node has no inputs: swap direction and select first output link
    """
    link = view._getFirstSelectedLink()
    node = view._getFirstSelectedNode()

    if link and not node:
        view._jumpToSourceNode(link)
    elif node:
        view._selectFirstLinkInDirection(node, "input")


def navigateForward(view):
    """Navigate forward (toward outputs/targets) using the ] key.

    Behavior:
    - If a link is selected: jump to the target node (data receiver)
    - If a node is selected: select the first output link
    - If the node has no outputs: swap direction and select first input link
    """
    link = view._getFirstSelectedLink()
    node = view._getFirstSelectedNode()

    if link and not node:
        view._jumpToTargetNode(link)
    elif node:
        view._selectFirstLinkInDirection(node, "output")


def _jumpToSourceNode(view, link):
    view._showHotkeyPopup("[", "left")

    # Track direction for subsequent pin selection
    view._lastJumpDirection = "source"

    # Get the data source node (the node providing data via its output)
    sourceNodeId = getattr(link, "data_sourceNodeId", None)

    # Check if this is a dangling link
    if link.isDangling and sourceNodeId and sourceNodeId not in view.nodes:
        # Get the link index
        linkIndex = -1
        for i, l in enumerate(view.links):
            if l is link:
                linkIndex = i
                break

        # Check if this is the same dangling link we tried to follow before
        if linkIndex == view._lastDanglingLinkIndex:
            view._danglingLinkFollowCount += 1
            if view._danglingLinkFollowCount >= 1:
                # Second attempt - add the missing node
                if view._addNodeFromDanglingLink(link):
                    view._lastDanglingLinkIndex = -1
                    view._danglingLinkFollowCount = 0
                    # Now jump to the newly added node
                    if sourceNodeId in view.nodes:
                        view._jumpToSourceNodeInternal(link, sourceNodeId)
                return
        else:
            # First time selecting this dangling link - just show it's selected
            view._lastDanglingLinkIndex = linkIndex
            view._danglingLinkFollowCount = 0
            view._showPopupMessage("Follow again to add node")
            return

    if not sourceNodeId or sourceNodeId not in view.nodes:
        return

    view._jumpToSourceNodeInternal(link, sourceNodeId)


def _jumpToTargetNode(view, link):
    view._showHotkeyPopup("]", "right")

    # Track direction for subsequent pin selection
    view._lastJumpDirection = "target"

    # Get the data target node (the node receiving data via its input)
    targetNodeId = getattr(link, "data_targetNodeId", None)

    # Check if this is a dangling link
    if link.isDangling and targetNodeId and targetNodeId not in view.nodes:
        # Get the link index
        linkIndex = -1
        for i, l in enumerate(view.links):
            if l is link:
                linkIndex = i
                break

        # Check if this is the same dangling link we tried to follow before
        if linkIndex == view._lastDanglingLinkIndex:
            view._danglingLinkFollowCount += 1
            if view._danglingLinkFollowCount >= 1:
                # Second attempt - add the missing node
                if view._addNodeFromDanglingLink(link):
                    view._lastDanglingLinkIndex = -1
                    view._danglingLinkFollowCount = 0
                    # Now jump to the newly added node
                    if targetNodeId in view.nodes:
                        view._jumpToTargetNodeInternal(link, targetNodeId)
                return
        else:
            # First time selecting this dangling link - just show it's selected
            view._lastDanglingLinkIndex = linkIndex
            view._danglingLinkFollowCount = 0
            view._showPopupMessage("Follow again to add node")
            return

    if not targetNodeId or targetNodeId not in view.nodes:
        return

    view._jumpToTargetNodeInternal(link, targetNodeId)


def _jumpToSourceNodeInternal(view, link, sourceNodeId):
    """Internal helper to complete the jump to source node."""
    sourceNode = view.nodes[sourceNodeId]




    for i, l in enumerate(view.links):
        if l is link:
            view._lastOutputLinkIndex = i
            break

    # Clear current selection and select the source node
    view.clearSelection()
    view.clearLinkSelection()
    _selection.select_node(view, sourceNode.id)
    view._lastSelectedNode = sourceNode

    # Reset dangling link tracking
    view._lastDanglingLinkIndex = -1
    view._danglingLinkFollowCount = 0

    # Center on the node (maintains current zoom)
    view._centerOnNode(sourceNode)

    # Sync selection to primtree
    view._syncSelectionToPrimtree()


def _jumpToTargetNodeInternal(view, link, targetNodeId):
    """Internal helper to complete the jump to target node."""
    targetNode = view.nodes[targetNodeId]




    for i, l in enumerate(view.links):
        if l is link:
            view._lastInputLinkIndex = i
            break

    # Clear current selection and select the target node
    view.clearSelection()
    view.clearLinkSelection()
    _selection.select_node(view, targetNode.id)
    view._lastSelectedNode = targetNode

    # Reset dangling link tracking
    view._lastDanglingLinkIndex = -1
    view._danglingLinkFollowCount = 0

    # Center on the node (maintains current zoom)
    view._centerOnNode(targetNode)

    # Sync selection to primtree
    view._syncSelectionToPrimtree()


def _navigateAlongRelationshipLink(view, link, worldPos):
    """Jump to the endpoint of ``link`` farthest from ``worldPos``.

    Returns True when navigation occurred. Used by the double-click
    handler so users can follow a relationship into a connected node
    that may be scrolled outside the current viewport.
    """
    sourceNodeId = getattr(link, "data_sourceNodeId", "") or getattr(
        link, "sourceNodeId", ""
    )
    targetNodeId = getattr(link, "data_targetNodeId", "") or getattr(
        link, "targetNodeId", ""
    )

    startPos = getattr(link, "start", None)
    endPos = getattr(link, "end", None)
    prefer_target = True
    if startPos is not None and endPos is not None:
        dx_start = float(worldPos[0]) - float(startPos[0])
        dy_start = float(worldPos[1]) - float(startPos[1])
        dx_end = float(worldPos[0]) - float(endPos[0])
        dy_end = float(worldPos[1]) - float(endPos[1])
        prefer_target = (dx_start * dx_start + dy_start * dy_start) <= (
            dx_end * dx_end + dy_end * dy_end
        )

    if prefer_target and targetNodeId and targetNodeId in view.nodes:
        view._jumpToTargetNodeInternal(link, targetNodeId)
        return True
    if not prefer_target and sourceNodeId and sourceNodeId in view.nodes:
        view._jumpToSourceNodeInternal(link, sourceNodeId)
        return True
    # Fall back to whichever endpoint is reachable.
    if targetNodeId and targetNodeId in view.nodes:
        view._jumpToTargetNodeInternal(link, targetNodeId)
        return True
    if sourceNodeId and sourceNodeId in view.nodes:
        view._jumpToSourceNodeInternal(link, sourceNodeId)
        return True
    return False
