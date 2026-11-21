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

Gestures extend the same module: node-drag, link-drag, reconnect, marquee
and viewport phases split out of the Qt mouse handlers, each ending by
dispatching its intent through the view's command registry when the view
has one, and running the same core directly for doubles that never ran
``__init__``. Link completion and reconnect teardown keep their existing
Connect/Reconnect/Disconnect dispatch; cancel paths dispatch nothing and
restore the pre-gesture state.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``. Names the suite patches on the legacy module
(``QtWidgets``, ``_push_undo_command``,
``warn_if_non_persistent_edit_target``) and legacy-only helpers
(``_ProfileSection``, ``_ValueHit``) resolve at call time through
``legacy_symbol``; everything else is imported directly.
"""

from pxr import Gf, Tf, Usd
from pxr.Usdviewq.qt import QtCore

from ..widgets import valueCell
from . import editing as _editing
from . import intents as _intents
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


def _clearValueInteractionState(view):
    """Abandon any value edit / drag / popup without authoring.

    Called wherever the graph underneath is about to be replaced. A mung
    left live across a stage reload would hold a prim path that no longer
    means anything and would author onto whatever now sits there.
    """
    if view._mungState is not None and view._mungActive:
        view._noticeHandler.setEnabled(True)
    view._mungState = None
    view._mungActive = False
    view._valueEditTarget = None
    view._valueEditInput.clear()
    view._tokenPopup = None
    # The fitted-text cache is keyed by node id, so a rebuilt graph must
    # not inherit it (stale entries would only miss on their component
    # keys, but clearing bounds the map's growth across reloads).
    view._valueFitCache.clear()


def _updateAnimator(view):
    """Advance the animator by the elapsed frame time."""
    if not view.frameTimer.isValid():
        view.frameTimer.start()
        view.animator.update(0)
    else:
        nsecsElapsed = view.frameTimer.nsecsElapsed()
        view.frameTimer.restart()
        dt = nsecsElapsed / 1e9
        view.animator.update(dt)


def _updatePanAnimation(view):
    """Update pan position from animation and check for completion."""
    if not view._panAnimating:
        return
    view.panX = view._panXAnim.current
    view.panY = view._panYAnim.current

    panXDone = abs(view._panXAnim.current - view._panXAnim.target) < 0.5
    panYDone = abs(view._panYAnim.current - view._panYAnim.target) < 0.5
    if panXDone and panYDone:
        view.panX = view._panXAnim.target
        view.panY = view._panYAnim.target
        view._panAnimating = False


def _updateKeyboardState(view):
    """Poll keyboard modifiers and update animation state."""
    if view.animator.isAnimating():
        view.update()
    else:
        view.frameTimer.invalidate()


def _startLinkDrag(view, nodeId, portName, isOutput):
    """
    Begin dragging a new link from a port.

    Args:
        nodeId: ID of the source node
        portName: Name of the source port
        isOutput: True if dragging from output port, False for input port
    """
    view._draggingLink = True
    view._dragLinkSourceNode = nodeId
    view._dragLinkSourcePort = portName
    view._dragLinkSourceIsOutput = isOutput
    view._dragLinkSyntheticTarget = None
    view._dragLinkResolvedTarget = None
    _start_node = view.nodes.get(view._dragLinkSourceNode)
    view._dragLinkTempLink.is_relationship_link = (
        _start_node is not None
        and _start_node.is_relationship_pin(
            str(portName), view._dragLinkSourceIsOutput
        )
    )
    view._dragLinkTempLink.sourcePropertyName = ""
    view._dragLinkTempLink.targetPropertyName = ""

    # Initialize temporary link
    node = view.nodes[nodeId]
    portPos = view._getPortPosition(node, portName, isOutput)

    # Link renderer expects start=output, end=input for correct bezier direction.
    # When dragging from an input port, the fixed port is the "end" (input side)
    # and the cursor will be the "start" (output side).
    if isOutput:
        view._dragLinkTempLink.sourceNodeId = nodeId
        view._dragLinkTempLink.sourcePort = portName
        view._dragLinkTempLink.targetNodeId = ""
        view._dragLinkTempLink.targetPort = ""
        if view._dragLinkTempLink.is_relationship_link:
            view._dragLinkTempLink.sourcePropertyName = str(portName)
        view._dragLinkTempLink.start = portPos
        view._dragLinkTempLink.end = portPos
    else:
        view._dragLinkTempLink.sourceNodeId = ""
        view._dragLinkTempLink.sourcePort = ""
        view._dragLinkTempLink.targetNodeId = nodeId
        view._dragLinkTempLink.targetPort = portName
        if view._dragLinkTempLink.is_relationship_link:
            view._dragLinkTempLink.targetPropertyName = str(portName)
        view._dragLinkTempLink.end = portPos
        view._dragLinkTempLink.start = portPos

    view._dragLinkValidTarget = False

    if not isOutput and not view._reconnectingLink:
        existingLink = view._findLinkToInput(nodeId, portName)
        if existingLink and not getattr(existingLink, "isDangling", False):
            view._reconnectingLink = True
            view._reconnectCapture = _editing._capture_reconnect(view, existingLink)
            view._reconnectOldSourceNodeId = existingLink.sourceNodeId
            view._reconnectOldSourcePort = existingLink.sourcePort
            view._reconnectOldTargetNodeId = existingLink.targetNodeId
            view._reconnectOldTargetPort = existingLink.targetPort
            view._reconnectOldSourcePropertyName = str(
                getattr(existingLink, "sourcePropertyName", "")
            )
            view._reconnectOldTargetPropertyName = str(
                getattr(existingLink, "targetPropertyName", "")
            )
            view._removeLinkFromGraph(
                existingLink.sourceNodeId,
                existingLink.sourcePort,
                existingLink.targetNodeId,
                existingLink.targetPort,
                sourcePropertyName=str(
                    getattr(existingLink, "sourcePropertyName", "")
                ),
                targetPropertyName=str(
                    getattr(existingLink, "targetPropertyName", "")
                ),
            )


def _snapLinkDragToPort(view, nodeId, portName, isOutput, relationship_hover):
    """Snap the temporary drag link to a hovered port and record the resolved target."""
    targetNode = view.nodes[nodeId]
    if relationship_hover:
        # Attribute → relationship pin: store a resolved target so
        # _completeLinkDrag uses it directly instead of falling back
        # to _hoveredPort, which would apply the output-swap logic
        # and incorrectly invert source/target routing.
        targetPropertyName = view._resolvePortPropertyName(
            targetNode, portName, isOutput
        )
        view._dragLinkResolvedTarget = (
            nodeId,
            portName,
            isOutput,
            targetPropertyName,
        )
        portPos = view._getPortPosition(targetNode, portName, isOutput)
        view._dragLinkTempLink.targetNodeId = nodeId
        view._dragLinkTempLink.targetPort = portName
        view._dragLinkTempLink.end = portPos
    else:
        portPos = view._getPortPosition(targetNode, portName, isOutput)
        if isOutput:
            view._dragLinkTempLink.sourceNodeId = nodeId
            view._dragLinkTempLink.sourcePort = portName
            view._dragLinkTempLink.start = portPos
        else:
            view._dragLinkTempLink.targetNodeId = nodeId
            view._dragLinkTempLink.targetPort = portName
            view._dragLinkTempLink.end = portPos


def _updateLinkDrag(view, screenPos):
    """
    Update the temporary link being dragged.

    Args:
        screenPos: Screen position of cursor (QPoint)
    """
    worldMousePos = (QtCore.QPointF(screenPos) / view.zoom) + QtCore.QPointF(
        view.panX, view.panY
    )
    worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
    view._dragLinkCursorPos = worldPos

    # Link renderer expects start=output, end=input for correct bezier direction.
    # When dragging from an output port, cursor moves the "end" (input side).
    # When dragging from an input port, cursor moves the "start" (output side).
    if view._dragLinkSourceIsOutput:
        view._dragLinkTempLink.targetNodeId = ""
        view._dragLinkTempLink.targetPort = ""
        view._dragLinkTempLink.targetPropertyName = ""
        view._dragLinkTempLink.end = worldPos
    else:
        view._dragLinkTempLink.sourceNodeId = ""
        view._dragLinkTempLink.sourcePort = ""
        view._dragLinkTempLink.sourcePropertyName = ""
        view._dragLinkTempLink.start = worldPos

    # Check if hovering over a valid target port
    view._dragLinkValidTarget = False
    view._dragLinkSyntheticTarget = None
    view._dragLinkResolvedTarget = None

    if view._hoveredPort:
        nodeId, portName, isOutput = view._hoveredPort

        # Validate: output can only connect to input (and vice versa)
        relationship_drag = getattr(
            view._dragLinkTempLink, "is_relationship_link", False
        )
        _hover_node = view.nodes.get(nodeId)
        relationship_hover = _hover_node is not None and getattr(
            _hover_node, "is_relationship_pin", lambda *_: False
        )(str(portName), isOutput)
        if relationship_drag:
            if nodeId in view.nodes and not (
                nodeId == view._dragLinkSourceNode
                and portName == view._dragLinkSourcePort
                and isOutput == view._dragLinkSourceIsOutput
            ):
                targetNode = view.nodes[nodeId]
                targetPropertyName = view._resolvePortPropertyName(
                    targetNode,
                    portName,
                    isOutput,
                )
                linkTargetIsOutput = not bool(view._dragLinkSourceIsOutput)
                view._dragLinkResolvedTarget = (
                    nodeId,
                    portName,
                    linkTargetIsOutput,
                    targetPropertyName,
                )
                view._dragLinkValidTarget = True
                view._dragLinkTempLink.is_relationship_link = True
                portPos = view._getLinkEndpointPosition(
                    targetNode,
                    portName,
                    linkTargetIsOutput,
                    targetPropertyName,
                    preferPropertySide=relationship_drag,
                )
                if linkTargetIsOutput:
                    view._dragLinkTempLink.sourceNodeId = nodeId
                    view._dragLinkTempLink.sourcePort = portName
                    view._dragLinkTempLink.sourcePropertyName = targetPropertyName
                    view._dragLinkTempLink.start = portPos
                else:
                    view._dragLinkTempLink.targetNodeId = nodeId
                    view._dragLinkTempLink.targetPort = portName
                    view._dragLinkTempLink.targetPropertyName = targetPropertyName
                    view._dragLinkTempLink.end = portPos
                return
        elif (
            isOutput != view._dragLinkSourceIsOutput or relationship_hover
        ) and not (
            nodeId == view._dragLinkSourceNode
            and portName == view._dragLinkSourcePort
        ):
            # Valid direction: opposite port types, or attribute dragged onto a
            # relationship pin (USD relationships are directionless so the
            # output/input distinction is relaxed for relationship drop targets).
            view._dragLinkValidTarget = True
            view._snapLinkDragToPort(nodeId, portName, isOutput, relationship_hover)
            return

    if getattr(view._dragLinkTempLink, "is_relationship_link", False):
        targetNodeId = view.nodeIdUnderCursor
        if not targetNodeId:
            hover_margin = max(2.0, 4.0 / max(float(view.zoom), 1e-6))
            targetNodeId = view._getNodeIdAtWorldPos(worldPos, margin=hover_margin)
        if targetNodeId and targetNodeId in view.nodes:
            targetNode = view.nodes[targetNodeId]
            # Restrict the prim-target drop zone to the title bar area at the top of
            # the node. Without this, dropping on a property row (especially when the
            # relationship pin is itself the first property) creates a prim-target
            # connection — which surprises users who meant to drop on the property's
            # port or to cancel the drag. Self-prim-targets remain allowed as long
            # as the drop lands inside the title bar.
            renderer = getattr(targetNode, "renderer", None) or getattr(
                view, "defaultRenderer", None
            )
            if renderer is not None:
                get_metrics = getattr(view, "_getRowHitMetrics", None)
                if get_metrics is not None:
                    port_start_y, _ = get_metrics(targetNode, renderer)
                    if worldPos[1] >= port_start_y:
                        return
            targetIsOutput = not bool(view._dragLinkSourceIsOutput)
            portPos = view._getNodeTopCenterPosition(
                targetNode,
            )
            view._dragLinkSyntheticTarget = (
                targetNodeId,
                "",
                targetIsOutput,
                "",
            )
            view._dragLinkValidTarget = True
            view._dragLinkTempLink.is_relationship_link = True
            if targetIsOutput:
                view._dragLinkTempLink.sourceNodeId = targetNodeId
                view._dragLinkTempLink.sourcePort = ""
                view._dragLinkTempLink.sourcePropertyName = ""
                view._dragLinkTempLink.start = portPos
            else:
                view._dragLinkTempLink.targetNodeId = targetNodeId
                view._dragLinkTempLink.targetPort = ""
                view._dragLinkTempLink.targetPropertyName = ""
                view._dragLinkTempLink.end = portPos


def _completeLinkDrag(view):
    """
    Complete link drag - create connection if valid target.

    This is called when the mouse is released during a link drag.
    If a valid target port is hovered, creates the connection.
    If the target is a collapsed node with multiple pins, shows a
    context menu to let the user choose which pin to connect to.
    Otherwise (released on empty space), cancels the drag.
    """
    targetData = view._dragLinkResolvedTarget or view._dragLinkSyntheticTarget
    _temp_link = getattr(view, "_dragLinkTempLink", None)
    relationship_drag = getattr(_temp_link, "is_relationship_link", False)
    if targetData is None and view._hoveredPort and not relationship_drag:
        hoveredNodeId, hoveredPortName, hoveredIsOutput = view._hoveredPort
        targetData = (hoveredNodeId, hoveredPortName, hoveredIsOutput, "")
    if not view._dragLinkValidTarget or not targetData:
        # Cancelled - released on empty space or invalid target
        view._dragLinkSyntheticTarget = None
        view._dragLinkResolvedTarget = None
        if view._reconnectingLink:
            view._finishReconnectDisconnect()
        return

    targetNodeId, targetPortName, targetIsOutput, targetPropertyName = targetData
    usingSyntheticTarget = targetData == view._dragLinkSyntheticTarget

    # Check if target node is title-collapsed with multiple pins on this side
    if (
        not relationship_drag
        and not usingSyntheticTarget
        and targetNodeId in view.nodes
    ):
        targetNode = view.nodes[targetNodeId]
        if getattr(targetNode, "_title_collapsed", False):
            # Get the original pins for this side
            if targetIsOutput:
                originalPins = list(
                    getattr(targetNode, "_original_output_pins", None)
                    or targetNode.outputPins
                )
            else:
                originalPins = list(
                    getattr(targetNode, "_original_input_pins", None)
                    or targetNode.inputPins
                )
            if len(originalPins) > 1:
                # Show context menu to choose pin
                view._showPinSelectionMenu(
                    targetNodeId,
                    originalPins,
                    targetIsOutput,
                )
                return
            elif len(originalPins) == 1:
                # Only one pin, use it directly
                targetPortName = originalPins[0]

    # Normalize so we know which is input and which is output
    target_node_model = view.nodes.get(targetNodeId)
    target_is_relationship = (
        target_node_model is not None
        and target_node_model.is_relationship_pin(
            str(targetPortName), targetIsOutput
        )
    )
    if relationship_drag:
        if view._dragLinkSourceIsOutput:
            outputNodeId, outputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
            inputNodeId, inputPortName = targetNodeId, targetPortName
            outputPropertyName = str(view._dragLinkSourcePort)
            inputPropertyName = targetPropertyName
        else:
            outputNodeId, outputPortName = targetNodeId, targetPortName
            inputNodeId, inputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
            outputPropertyName = targetPropertyName
            inputPropertyName = str(view._dragLinkSourcePort)
    elif target_is_relationship:
        # Attribute → relationship pin: preserve the drag-source's original
        # direction so the relationship-owner ends up on the opposite endpoint
        # and create_connection can find it via GetRelationship(target_port).
        if view._dragLinkSourceIsOutput:
            outputNodeId, outputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
            inputNodeId, inputPortName = targetNodeId, targetPortName
        else:
            outputNodeId, outputPortName = targetNodeId, targetPortName
            inputNodeId, inputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
        outputPropertyName = ""
        inputPropertyName = ""
    elif targetIsOutput:
        # Target is output, source must be input - swap them
        outputNodeId, outputPortName = targetNodeId, targetPortName
        inputNodeId, inputPortName = (
            view._dragLinkSourceNode,
            view._dragLinkSourcePort,
        )
        outputPropertyName = ""
        inputPropertyName = ""
    else:
        # Target is input, source must be output
        inputNodeId, inputPortName = targetNodeId, targetPortName
        outputNodeId, outputPortName = (
            view._dragLinkSourceNode,
            view._dragLinkSourcePort,
        )
        outputPropertyName = ""
        inputPropertyName = ""

    if view._reconnectingLink and (
        outputNodeId == view._reconnectOldSourceNodeId
        and outputPortName == view._reconnectOldSourcePort
        and inputNodeId == view._reconnectOldTargetNodeId
        and inputPortName == view._reconnectOldTargetPort
    ):
        view._cancelLinkReconnect()
        return

    # Reject exact duplicate connections
    if view._hasExactLink(
        outputNodeId,
        outputPortName,
        inputNodeId,
        inputPortName,
        outputPropertyName,
        inputPropertyName,
    ):
        if view._reconnectingLink:
            view._cancelLinkReconnect()
        return

    # Check if target input already has a connection
    oldLink = (
        None
        if relationship_drag or target_is_relationship
        else view._findLinkToInput(inputNodeId, inputPortName)
    )
    if oldLink:
        # Highlight old link in red for brief moment
        oldLink.highlighted = True
        oldLink.highlightColor = (1.0, 0.0, 0.0, 1.0)
        view._linksNeedSync = True
        view.update()

        # Snapshot and consume reconnect state now so a new reconnect
        # started during the highlight delay cannot overwrite it.
        reconnect_state = None
        if view._reconnectingLink:
            reconnect_state = (_editing._reconnect_endpoints(view)
                               if isinstance(view.nodeGraph.getStage(), Usd.Stage) else (
                                   view._reconnectOldSourceNodeId, view._reconnectOldSourcePort,
                                   view._reconnectOldTargetNodeId, view._reconnectOldTargetPort,
                               ))
            view._clearReconnectState()

        # Use QTimer to remove highlight after 200ms, then create new connection
        from PySide6 import QtCore

        QtCore.QTimer.singleShot(
            200,
            lambda: view._finishConnectionReplacement(
                oldLink,
                outputNodeId,
                outputPortName,
                inputNodeId,
                inputPortName,
                reconnect_state,
                outputPropertyName,
                inputPropertyName,
            ),
        )
    else:
        # No existing connection, create directly
        view._createConnection(
            outputNodeId,
            outputPortName,
            inputNodeId,
            inputPortName,
            sourcePropertyName=outputPropertyName,
            targetPropertyName=inputPropertyName,
        )
    view._dragLinkSyntheticTarget = None
    view._dragLinkResolvedTarget = None


def _startLinkReconnect(view, linkIndex, worldClickPos):
    """Disconnect a link from its nearest end and start dragging to reconnect.

    The link is removed visually (local graph only) while the user drags.
    USD is not modified until the drag completes, so cancelling restores
    the original connection and a successful reconnection is a single undo.
    """
    link = view.links[linkIndex]

    is_relationship_link = bool(getattr(link, "is_relationship_link", False))

    if is_relationship_link:
        # Relationship links always anchor on the source (the relationship
        # pin) because the target is a whole-node prim target, not a pin.
        anchorNodeId = link.sourceNodeId
        anchorPort = link.sourcePort
        anchorIsOutput = True
    else:
        startPos = link.start
        endPos = link.end
        startDistSq = (worldClickPos[0] - startPos[0]) ** 2 + (
            worldClickPos[1] - startPos[1]
        ) ** 2
        endDistSq = (worldClickPos[0] - endPos[0]) ** 2 + (
            worldClickPos[1] - endPos[1]
        ) ** 2

        if startDistSq < endDistSq:
            anchorNodeId = link.targetNodeId
            anchorPort = link.targetPort
            anchorIsOutput = False
        else:
            anchorNodeId = link.sourceNodeId
            anchorPort = link.sourcePort
            anchorIsOutput = True

    view._reconnectingLink = True
    view._reconnectCapture = _editing._capture_reconnect(view, link)
    view._reconnectOldSourceNodeId = link.sourceNodeId
    view._reconnectOldSourcePort = link.sourcePort
    view._reconnectOldTargetNodeId = link.targetNodeId
    view._reconnectOldTargetPort = link.targetPort
    view._reconnectOldSourcePropertyName = str(
        getattr(link, "sourcePropertyName", "")
    )
    view._reconnectOldTargetPropertyName = str(
        getattr(link, "targetPropertyName", "")
    )

    view._removeLinkFromGraph(
        link.sourceNodeId,
        link.sourcePort,
        link.targetNodeId,
        link.targetPort,
        sourcePropertyName=str(getattr(link, "sourcePropertyName", "")),
        targetPropertyName=str(getattr(link, "targetPropertyName", "")),
    )

    if anchorNodeId not in view.nodes:
        view._cancelLinkReconnect()
        return

    view._startLinkDrag(anchorNodeId, anchorPort, anchorIsOutput)

    if anchorIsOutput:
        view._dragLinkTempLink.end = worldClickPos
    else:
        view._dragLinkTempLink.start = worldClickPos


def _clearReconnectState(view):
    """Zero reconnect bookkeeping fields."""
    view._reconnectingLink = False
    view._reconnectOldSourceNodeId = None
    view._reconnectOldSourcePort = None
    view._reconnectOldTargetNodeId = None
    view._reconnectOldTargetPort = None
    view._reconnectOldSourcePropertyName = ""
    view._reconnectOldTargetPropertyName = ""
    view._reconnectCapture = None


def _cancelLinkReconnect(view):
    """Restore the original link when reconnection fails (error recovery)."""
    srcId = view._reconnectOldSourceNodeId
    srcPort = view._reconnectOldSourcePort
    tgtId = view._reconnectOldTargetNodeId
    tgtPort = view._reconnectOldTargetPort
    srcPropName = view._reconnectOldSourcePropertyName
    tgtPropName = view._reconnectOldTargetPropertyName
    capture = getattr(view, "_reconnectCapture", None)
    view._clearReconnectState()
    if isinstance(capture, _editing.ReconnectCapture):
        _editing._refresh_connections(view, capture.order)
        return

    srcNode = view.nodes.get(srcId)
    tgtNode = view.nodes.get(tgtId)
    if srcNode and tgtNode:
        view._addLinkToGraph(
            srcId,
            srcPort,
            tgtId,
            tgtPort,
            srcNode,
            tgtNode,
            sourcePropertyName=srcPropName,
            targetPropertyName=tgtPropName,
        )


def _finishReconnectDisconnect(view):
    """Delete the original connection when dropped on empty space."""
    if isinstance(view.nodeGraph.getStage(), Usd.Stage):
        capture = _editing._reconnect_endpoints(view)
        view._clearReconnectState()
        return _editing._dispatch_connection(view, _intents.Disconnect(
            capture.endpoints, description="Disconnect", prior_order=capture.order))
    srcId = view._reconnectOldSourceNodeId
    srcPort = view._reconnectOldSourcePort
    tgtId = view._reconnectOldTargetNodeId
    tgtPort = view._reconnectOldTargetPort

    inputNode = view.nodes.get(tgtId)
    outputNode = view.nodes.get(srcId)
    if not inputNode or not outputNode:
        view._cancelLinkReconnect()
        return

    inputPrim = inputNode.getUsdPrim()
    outputPrim = outputNode.getUsdPrim()
    if not inputPrim or not outputPrim:
        view._cancelLinkReconnect()
        return

    library = view._findLibraryForPrim(inputPrim)
    if not library:
        view._cancelLinkReconnect()
        return

    view._clearReconnectState()

    stage = view.nodeGraph.getStage()
    legacy_symbol("warn_if_non_persistent_edit_target")(stage)

    view._noticeHandler.setEnabled(False)
    try:
        success = library.delete_connection(
            stage, outputPrim, srcPort, inputPrim, tgtPort
        )
        if success:
            delete_fn = _editing._make_connection_edit(
                srcId, srcPort, tgtId, tgtPort, create=False
            )
            create_fn = _editing._make_connection_edit(
                srcId, srcPort, tgtId, tgtPort, create=True
            )

            def redo(gv):
                delete_fn(gv)
                gv.linksChanged = True
                gv.update()

            def undo(gv):
                create_fn(gv)
                gv.linksChanged = True
                gv.update()

            legacy_symbol("_push_undo_command")("Disconnect", redo, undo, view=view)

            # Re-derive links from USD so the disconnected pin's port
            # highlight (the relationship arrow / port circle) reflects the
            # now-empty target. GetTargets is the source of truth; the
            # in-place removal done at reconnect-start is not guaranteed to
            # match relationship links (their propertyName fields can differ
            # from the record), which would otherwise leave a stale link in
            # self.links keeping the port "connected" (arrow stuck yellow).
            # Mirrors the not-found branch below and _deleteConnection.
            view.linksChanged = True
            view._rebuildLinks()
            view.update()
        else:
            # Connection not found in USD — it may have been replaced by
            # another connection to the same pin. Rebuild links from USD
            # so the UI reflects the actual stage state.
            view.linksChanged = True
            view._rebuildLinks()
    finally:
        view._noticeHandler.setEnabled(True)


def _finalizeNodeDrag(view):
    """Finalize a node drag: write positions to USD and push undo command."""
    # Capture old positions (pre-drag) and new positions (post-drag)
    # before calling endDrag() which writes to USD.
    old_positions = {}
    new_positions = {}
    for nid, node in view.nodes.items():
        if node.selected and node._dragging:
            old_positions[nid] = (node._position[0], node._position[1])
            if node._dragPosition is not None:
                new_positions[nid] = (
                    node._dragPosition[0],
                    node._dragPosition[1],
                )

    def finish():
        for node in view.nodes.values():
            if node.selected:
                node.endDrag()

    if new_positions and getattr(view, "_editor", None) is not None:
        _editing.edit_node_positions(view, new_positions, "Move Nodes", finish,
                                     old_display=old_positions)
        return
    view._noticeHandler.setEnabled(False)
    try:
        finish()
    finally:
        view._noticeHandler.setEnabled(True)

    # Push undo command if positions actually changed
    Tf.Status(
        f"_finalizeNodeDrag: old_positions={len(old_positions)}, "
        f"new_positions={len(new_positions)}"
    )
    if old_positions and new_positions:
        legacy_symbol("_push_undo_command")(
            "Move Nodes",
            lambda gv, p=new_positions: gv._restoreNodePositions(p),
            lambda gv, p=old_positions: gv._restoreNodePositions(p),
            view=view,
        )


def _dispatch_gesture(view, intent, fallback):
    """Dispatch a gesture intent, or run the fallback without a registry.

    Doubles that never ran ``__init__`` carry no command registry; they
    take the fallback, which is the same core the registered handler runs
    for a real view, so both paths stay behavior-identical.
    """
    registry = getattr(view, "_commandRegistry", None)
    if registry is None:
        return fallback()
    return registry.dispatch(intent)


def begin_node_drag(view, node_id, toggle):
    """Select the pressed node and begin dragging the selection."""
    clickedNode = view.nodes[node_id]
    if not toggle:
        if not clickedNode.selected:
            view.clearSelection()
            view.clearLinkSelection()
        _selection.select_node(view, clickedNode.id)
    else:
        _selection.select_node(view, clickedNode.id, toggle=True)
    view.draggingNodes = True

    # Bring the clicked node to front. Node-background quads are
    # regenerated from node.zOrder every frame, so the box raises
    # for free; for text we patch only this node's depth in the held
    # vertex buffer (O(node glyphs)). We must NOT set
    # self.textChanged here — that re-lays out every node's text and
    # cost ~2s for a dozen nodes due to the O(N^2) string-by-string
    # Python<->C++ vertex marshalling.
    view._nextZOrder += 1
    clickedNode.zOrder = view._nextZOrder
    view.textRenderer.patchNodeTextDepth(clickedNode)
    # Node-quad depth is baked at generation; re-bumping z-order needs
    # a regen (the selection-set check above misses a re-click on an
    # already-selected node).
    view._nodeRenderManager.markNodeQuadDirty(True)
    # The title icon's depth is also baked at generation, but the icon
    # path only rebuilds on a content change (which selection skips for
    # perf). Without this, the icon stays at its old depth and the
    # re-raised node quad occludes it (the icon visibly drops on
    # select). markIconsDirty re-bakes it at the new zOrder next frame,
    # reading the snapshot the node-quad path re-syncs this same frame,
    # so the icon rides to front with the node.
    view._cppIconRenderer.markIconsDirty()

    # Begin drag on all selected nodes for batched USD writes
    for node in view.nodes.values():
        if node.selected:
            node.beginDrag()

    # Update link selection based on current mode
    view._updateLinkSelectionFromNodes()


def move_dragged_nodes(view, dx, dy):
    """Move every selected node by a world-space delta."""
    movedNodeIds = set()
    for node in view.nodes.values():
        if node.selected:
            node.position = node.position + Gf.Vec2d(dx, dy)
            movedNodeIds.add(node.id)
            # Cheap render-offset move (text + quad ride one frame).
            view._nodeTransformFrame.translate(node.id, dx, dy)

    # Refresh derived state for the dragged nodes (the reconcile is
    # skipped while dragging). This does not advance the reconcile
    # watermark, so the release frame reconciles these nodes once more
    # from their final positions.
    view._applyMovedNodeDerivedState(movedNodeIds)


def begin_marquee(view, origin, mode):
    """Arm a marquee selection from a world-space origin point."""
    view._marqueeBase = view._selection.copy()
    view._marqueeMode = mode
    view.marqueeActive = True
    view.marqueeStart = QtCore.QPointF(origin[0], origin[1])
    view.marqueeCurrent = QtCore.QPointF(origin[0], origin[1])


def update_marquee(view, current):
    """Extend the marquee rectangle and reselect from its base."""
    view.marqueeCurrent = QtCore.QPointF(current[0], current[1])
    view._updateMarqueeSelection()


def finish_marquee(view):
    """Commit the marquee selection and disarm the rectangle."""
    view._updateMarqueeSelection()
    view.marqueeActive = False
    view._marqueeBase = _selection.Selection()
    if view._selectedLinks and not view._selectedNodes:
        view._syncLinkSelectionToPrimtree()


def pan_viewport(view, dx, dy):
    """Pan the viewport by a screen-space pixel delta."""
    view.panX -= dx / view.zoom
    view.panY -= dy / view.zoom


def zoom_viewport(view, focus, zoom):
    """Zoom the viewport around a screen-space focus point."""
    view._zoomAroundPoint(zoom, focus)


def double_click_gesture(view, event):
    """Follow a link or begin a rename; False when nothing was hit."""
    worldMousePos = (
        QtCore.QPointF(event.position().toPoint()) / view.zoom
    ) + QtCore.QPointF(view.panX, view.panY)
    worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

    # Check if we clicked on a connected port that has a dangling link.
    # _findDanglingLinkAtPort by construction only returns indices of
    # links with isDangling=True, so we don't re-check the flag here.
    portDanglingIdx = view._findDanglingLinkAtPort(worldPos)
    if portDanglingIdx >= 0 and portDanglingIdx < len(view.links):
        view._addNodesFromDanglingLink(view.links[portDanglingIdx])
        return True

    # Also check via curve hit-test for backward compatibility
    clickedLinkIndex = view.linkRenderer.findLinkUnderCursor(
        worldPos,
        view.links,
        view.zoom,
        view._cachedLinkLineWidth,
        spatialIndex=view._spatialIndex,
        debug=False,
    )

    if clickedLinkIndex >= 0 and clickedLinkIndex < len(view.links):
        link = view.links[clickedLinkIndex]
        if link.isDangling:
            view._addNodesFromDanglingLink(link)
            return True
        if getattr(link, "is_relationship_link", False):
            if view._navigateAlongRelationshipLink(link, worldPos):
                return True

    # A node under the cursor: rename it. Last, so the link and port cases
    # above keep the meaning they already had -- they hit-test geometry
    # that sits on top of a node, and stealing their double-click would
    # cost the only way to follow a dangling link.
    nodeId = view._getNodeIdAtWorldPos(worldPos)
    if nodeId:
        view._beginRename(nodeId)
        return True

    return False


def press_intercept_overlays(view, event):
    """Settle open editors and route overlay presses; True stops the press."""
    view.setFocus()
    # Clicking away commits, which is what a text field everywhere else
    # does. Committing before the click is dispatched matters: the click
    # may land on another node, and that node's selection should follow
    # the rename rather than race it.
    if view._renamingNodeId:
        view._commitRename()
    # An open token popup owns the click: inside it picks, outside closes.
    # Either way the press stops here, so a pick cannot also select a node.
    if view._tokenPopup:
        view._handleTokenPopupPress(event.position().toPoint())
        view.update()
        return True
    # A value editor commits on click-away, the same as the rename above
    # and as every text field everywhere else.
    if view._valueEditTarget:
        view._commitValueEdit()
    view.lastMousePos = event.position().toPoint()
    view.ctrlPressedOnMouseDown = event.modifiers() & QtCore.Qt.ControlModifier
    view.mousePosOnMouseDown = event.position().toPoint()
    # Resolve any unpainted position change before hit-testing what is under
    # the cursor (e.g. a programmatic move then an immediate click). O(1) when
    # nothing moved.
    view._reconcileMovedNodes()

    # If hotbox is visible, check if click is within hotbox
    if view.nodeCreationHotbox.is_visible:
        if view.nodeCreationHotbox.handle_mouse_press(event.position().toPoint()):
            view.update()
            return True

    if event.buttons() & QtCore.Qt.LeftButton and view._cachedShowMinimap:
        if view.minimap.handleMousePress(event.position().toPoint()):
            return True

    view._updateNodeUnderCursor(event.position().toPoint())
    return False


def press_title_caret_toggle(view, event):
    """Toggle title collapse on a caret press; True stops the press."""
    if view.nodeIdUnderCursor and view.nodeIdUnderCursor in view.nodes:
        clickedNodeForTitle = view.nodes[view.nodeIdUnderCursor]
        if clickedNodeForTitle.renderer:
            worldMousePos = (
                QtCore.QPointF(event.position().toPoint()) / view.zoom
            ) + QtCore.QPointF(view.panX, view.panY)
            worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
            renderer = clickedNodeForTitle.renderer
            nodeMarginH = renderer.getPortMarginH()
            titleFontSize = renderer.getTitleFontSize()
            # Cached title height (includes the schema-type subtitle) so the
            # caret hit box matches the rendered title bar exactly.
            titleHeight = clickedNodeForTitle.layoutTitleHeight
            nx = clickedNodeForTitle.position[0]
            ny = clickedNodeForTitle.position[1]
            caret_size = titleFontSize * 0.5
            caret_x = nx + nodeMarginH
            caret_y_top = ny
            caret_y_bot = ny + titleHeight
            caret_x_right = caret_x + caret_size
            if (
                caret_x <= worldPos[0] <= caret_x_right
                and caret_y_top <= worldPos[1] <= caret_y_bot
            ):
                clickedNodeForTitle.toggle_title_fold()
                if view.fontAtlas:
                    view.nodeGraph._calculateNodeSize(
                        clickedNodeForTitle,
                        view.textRenderer.calculateTextWidth,
                        view.fontAtlas,
                    )
                view.textChanged = True
                view.linksChanged = True
                view._nodeRenderManager.invalidateAll()
                view.update()
                return True
    return False


def press_group_fold_toggle(view, event):
    """Toggle a group fold on a header press; True stops the press."""
    if view.nodeIdUnderCursor and view.nodeIdUnderCursor in view.nodes:
        clickedNodeForFold = view.nodes[view.nodeIdUnderCursor]
        if clickedNodeForFold.renderer:
            worldMousePos = (
                QtCore.QPointF(event.position().toPoint()) / view.zoom
            ) + QtCore.QPointF(view.panX, view.panY)
            worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
            headerResult = clickedNodeForFold.renderer.getGroupHeaderAtPoint(
                clickedNodeForFold, worldPos, view.fontAtlas
            )
            if headerResult and headerResult["found"]:
                headerName = headerResult["portName"]
                isOutput = headerResult["isOutput"]
                clickedNodeForFold.toggle_fold(headerName, isOutput)
                if view.fontAtlas:
                    view.nodeGraph._calculateNodeSize(
                        clickedNodeForFold,
                        view.textRenderer.calculateTextWidth,
                        view.fontAtlas,
                    )
                view.textChanged = True
                view.linksChanged = True
                view._nodeRenderManager.invalidateAll()
                view.update()
                return True
    return False


def press_value_cell(view, event):
    """Claim the press for a value cell; True stops the press."""
    # A value cell claims the press before anything else on the node.
    #
    # It is checked ahead of the hovered-port branch and well ahead of
    # _findRowEdgeDragStart, but the ordering is belt and braces: the
    # cell rect stops a full PAD short of the connection gutter
    # (widgets/valueCell.cell_rect), so the two regions cannot both
    # contain one point. Returning here is what guarantees a press on
    # a value never degenerates into a connection drag even if a future
    # layout change broke that inequality.
    valueHit = view._valueCellAtPoint(
        (
            QtCore.QPointF(event.position().toPoint()) / view.zoom
        )
        + QtCore.QPointF(view.panX, view.panY)
    )
    if valueHit is not None:
        view._onValueCellPressed(valueHit, event)
        view.update()
        return True
    return False


def press_hovered_port_link_start(view, event):
    """Start a link drag from the hovered port; True stops the press."""
    # Check if clicking on a port to start link creation
    if view._hoveredPort:
        nodeId, portName, isOutput = view._hoveredPort
        # Start link drag
        view._startLinkDrag(nodeId, portName, isOutput)
        view.update()
        return True
    return False


def press_arrowhead_reconnect(view, event):
    """Grab a relationship arrowhead to reconnect; True stops the press."""
    worldMousePosForLink = (
        QtCore.QPointF(event.position().toPoint()) / view.zoom
    ) + QtCore.QPointF(view.panX, view.panY)
    worldPosForLink = Gf.Vec2d(
        worldMousePosForLink.x(), worldMousePosForLink.y()
    )

    # Grab a relationship arrowhead to reconnect it (or drop on empty to
    # disconnect). Checked before node selection so an arrowhead that
    # overlaps its target node is still grabbable.
    arrowheadLinkIndex = view._findArrowheadUnderCursor(worldPosForLink)
    if arrowheadLinkIndex >= 0:
        view.clearSelection()
        view.clearLinkSelection()
        view._startLinkReconnect(arrowheadLinkIndex, worldPosForLink)
        view.update()
        return True
    return False


def press_row_edge_link_start(view, event):
    """Start a link drag from a row edge band; True stops the press."""
    # Start a new connection by dragging from a row's left/right edge
    # band (the outer tenth of each side), even with no visible pin.
    # Presses in the middle of the row, on the title, or below the last
    # row fall through to node selection/drag.
    if view.nodeIdUnderCursor and view.nodeIdUnderCursor in view.nodes:
        worldMousePosForLink = (
            QtCore.QPointF(event.position().toPoint()) / view.zoom
        ) + QtCore.QPointF(view.panX, view.panY)
        worldPosForLink = Gf.Vec2d(
            worldMousePosForLink.x(), worldMousePosForLink.y()
        )
        edgeNode = view.nodes[view.nodeIdUnderCursor]
        edgeHit = view._findRowEdgeDragStart(edgeNode, worldPosForLink)
        if edgeHit:
            view._startLinkDrag(view.nodeIdUnderCursor, edgeHit[0], edgeHit[1])
            view.update()
            return True
    return False


def press_node_or_empty(view, event):
    """Select and begin a node drag, or dispatch a link click or marquee."""
    clickedNode = None
    if view.nodeIdUnderCursor and view.nodeIdUnderCursor in view.nodes:
        clickedNode = view.nodes[view.nodeIdUnderCursor]

    if clickedNode:
        toggle = bool(view.ctrlPressedOnMouseDown)
        _dispatch_gesture(
            view,
            _intents.BeginNodeDrag(clickedNode.id, toggle),
            lambda: begin_node_drag(view, clickedNode.id, toggle),
        )
    else:
        worldMousePos = (
            QtCore.QPointF(event.position().toPoint()) / view.zoom
        ) + QtCore.QPointF(view.panX, view.panY)
        worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

        clickedLinkIndex = view.linkRenderer.findLinkUnderCursor(
            worldPos,
            view.links,
            view.zoom,
            view._cachedLinkLineWidth,
            spatialIndex=view._spatialIndex,
            debug=False,
        )

        if clickedLinkIndex >= 0:
            if view.ctrlPressedOnMouseDown:
                view._toggleLinkSelection(clickedLinkIndex)
            else:
                clickedLink = view.links[clickedLinkIndex]
                # Dangling links are selectable on click
                # Regular and relationship links start reconnect on click
                if clickedLink.isDangling:
                    view.clearSelection()
                    view._selectLink(clickedLinkIndex, addToSelection=False)
                else:
                    view.clearSelection()
                    view.clearLinkSelection()
                    view._startLinkReconnect(clickedLinkIndex, worldPos)
        else:
            if not view.ctrlPressedOnMouseDown:
                view.clearSelection()
                view.clearLinkSelection()
                # Do not sync empty selection back to prim tree — the user
                # may press A next to add the still-selected prim(s).
            origin = (worldMousePos.x(), worldMousePos.y())
            mode = "union" if view.ctrlPressedOnMouseDown else "replace"
            _dispatch_gesture(
                view,
                _intents.BeginMarquee(origin, mode),
                lambda: begin_marquee(view, origin, mode),
            )


def move_intercept_overlays(view, event):
    """Route hotbox and minimap moves; True stops the move."""
    # If hotbox is visible, update hover state
    if view.nodeCreationHotbox.is_visible:
        if view.nodeCreationHotbox.handle_mouse_move(event.position().toPoint()):
            view.update()
            return True

    if view._cachedShowMinimap:
        view.minimap.updateHoverState(event.position().toPoint())

        if view.minimap.handleMouseMove(event.position().toPoint()):
            view.lastMousePos = event.position().toPoint()
            return True

    return False


def move_value_drag(view, event):
    """Feed a move to the in-progress value drag; True stops the move."""
    # A value drag owns the pointer: no hover work, no node drag, no link
    # drag. Checked first so none of the scans below run per sample.
    if view._mungState is not None and (event.buttons() & QtCore.Qt.LeftButton):
        view._updateMung(event)
        view.lastMousePos = event.position().toPoint()
        view.update()
        return True
    return False


def move_update_hover(view, event, is_viewport_drag):
    """Refresh hover state unless a viewport drag bypasses it."""
    if not is_viewport_drag:
        view._updateNodeUnderCursor(event.position().toPoint())
        view._updateValueHoverCursor(event.position().toPoint())

        if not view.draggingNodes:
            if view._updateHoveredPort(event.position().toPoint()):
                view.update()

        if not view.draggingNodes and not view._draggingLink:
            # Update property tooltip on hover
            view._updatePropertyTooltip(event.position().toPoint())

            # Update link under cursor for hover highlighting
            worldMousePos = (
                QtCore.QPointF(event.position().toPoint()) / view.zoom
            ) + QtCore.QPointF(view.panX, view.panY)
            worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

            newLinkUnderCursor = view.linkRenderer.findLinkUnderCursor(
                worldPos,
                view.links,
                view.zoom,
                view._cachedLinkLineWidth,
                spatialIndex=view._spatialIndex,
            )
            if newLinkUnderCursor != view._linkUnderCursor:
                view._linkUnderCursor = newLinkUnderCursor
                view.update()


def move_link_drag(view, event):
    """Advance the in-progress link drag; True stops the move."""
    # Handle link dragging
    if view._draggingLink:
        view._updateLinkDrag(event.position().toPoint())
        view.update()
        return True
    return False


def move_viewport_drag(view, event):
    """Pan or pan-zoom the viewport; True stops the move."""
    if event.buttons() & QtCore.Qt.MiddleButton or (
        view.spacePressed and event.buttons() & QtCore.Qt.LeftButton
    ):
        if view.ctrlPressedOnMouseDown:
            newZoom = view.zoom * (
                1.0
                + (event.position().toPoint().x() - view.lastMousePos.x()) / 120.0
            )
            focus = (view.mousePosOnMouseDown.x(), view.mousePosOnMouseDown.y())
            _dispatch_gesture(
                view,
                _intents.Zoom(focus, newZoom),
                lambda: zoom_viewport(view, focus, newZoom),
            )
        else:
            dx = event.position().toPoint().x() - view.lastMousePos.x()
            dy = event.position().toPoint().y() - view.lastMousePos.y()
            _dispatch_gesture(
                view,
                _intents.Pan(dx, dy),
                lambda: pan_viewport(view, dx, dy),
            )
        view.update()
        view.lastMousePos = event.position().toPoint()
        return True
    return False


def move_left_drag(view, event):
    """Extend the marquee or move dragged nodes; True skips the tail."""
    if event.buttons() & QtCore.Qt.LeftButton:
        if view.marqueeActive:
            worldMousePos = (
                QtCore.QPointF(event.position().toPoint()) / view.zoom
            ) + QtCore.QPointF(view.panX, view.panY)
            current = (worldMousePos.x(), worldMousePos.y())
            _dispatch_gesture(
                view,
                _intents.UpdateMarquee(current),
                lambda: update_marquee(view, current),
            )
            view.update()
            return True

        if view.draggingNodes:
            dx = (
                event.position().toPoint().x() - view.lastMousePos.x()
            ) / view.zoom
            dy = (
                event.position().toPoint().y() - view.lastMousePos.y()
            ) / view.zoom

            _dispatch_gesture(
                view,
                _intents.UpdateNodeDrag(dx, dy),
                lambda: move_dragged_nodes(view, dx, dy),
            )

    return False


def release_intercept_overlays(view, event):
    """Route hotbox and value-drag releases; True stops the release."""
    # Release the hotbox scrollbar first. It is the only overlay that
    # takes a press-drag-release, and the handlers below would otherwise
    # treat the release as ending a marquee or a link drag that never
    # started.
    if view.nodeCreationHotbox.is_visible:
        if view.nodeCreationHotbox.handle_mouse_release(
                event.position().toPoint()):
            view.update()
            return True
    # Finish a value press: a drag commits as ONE undo entry, a click with
    # no motion opens the keyboard editor instead. Before the link-drag
    # branch because a value press never started one.
    if view._mungState is not None:
        view._finishMung()
        view.update()
        return True
    return False


def release_link_drag(view, event):
    """Complete the in-progress link drag; True stops the release."""
    # Handle link drag completion
    if view._draggingLink:
        # Refresh hover state at the release position so that
        # _completeLinkDrag sees the actual cursor location.  On Windows,
        # Qt may not emit a final mouseMoveEvent before the release,
        # leaving _hoveredPort / _dragLinkValidTarget stale from a
        # previous frame — which prevents _finishReconnectDisconnect
        # from running and skips the "Disconnect" undo entry.
        releasePos = event.position().toPoint()
        view._updateHoveredPort(releasePos)
        view._updateLinkDrag(releasePos)
        view._completeLinkDrag()
        view._draggingLink = False
        view._dragLinkSyntheticTarget = None
        view._dragLinkResolvedTarget = None
        view.update()
        return True
    return False


def release_minimap(view, event):
    """Release a minimap viewport drag; True stops the release."""
    # Always release a minimap viewport drag, even if the minimap was hidden
    # mid-drag: handleMouseRelease() is a no-op when no drag is in progress,
    # but if "Show Minimap" is toggled off after a drag starts, gating this on
    # _cachedShowMinimap would strand _viewportDragging=True and let a later
    # move hijack a pan once the minimap is shown again.
    if view.minimap.handleMouseRelease():
        return True
    return False


def release_node_drag_and_marquee(view, event):
    """Commit a node drag and a marquee, then sync the selection."""
    # End drag on all selected nodes - writes final positions to USD session layer
    if view.draggingNodes:
        _dispatch_gesture(
            view, _intents.FinishNodeDrag(), lambda: view._finalizeNodeDrag()
        )
        # Final node positions were just baked into the snapshot; rebuild the
        # port highlights once so the circles match (the drag itself kept them
        # live via the geometryChanged signal while draggingNodes was True).
        view._portHighlightsDirty = True

    view.draggingNodes = False

    if view.marqueeActive:
        _dispatch_gesture(
            view, _intents.FinishMarquee(), lambda: finish_marquee(view)
        )
        view.update()

    hasSelectedNodes = any(node.selected for node in view.nodes.values())

    if hasSelectedNodes:
        view._syncSelectionToPrimtree()
