#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#
"""Explicit bindings for the legacy GraphView method surface."""

import functools

from ..nodeGraphJson import NodeGraphJson
from ..pinUtils import arrowhead_points
from . import editing as _editing, intents as _intents, selection as _selection
from . import interaction as _interaction, interaction_events, stage_adapter
from . import model as _graphview_model, resources as _resources
from . import render, render as _render, render_hotbox as _render_hotbox
from . import render_links as _render_links, render_nodes as _render_nodes
from . import render_overlays as _render_overlays, render_text as _render_text


def _loadSampleGraph(view, jsonPath):
    view.nodeGraph = NodeGraphJson()
    view._initUndoTracking(None)
    view.nodeGraph.load(
        jsonPath, view.textRenderer.calculateTextWidth, view.fontAtlas
    )
    view._initUndoTracking(view.nodeGraph.getStage())
    view.linksChanged = True


def bind_compat(cls):
    cls._onStageReplaced = stage_adapter._onStageReplaced
    cls._finishStageReload = stage_adapter._finishStageReload
    cls._graphScopeParentPath = stage_adapter.graphScopeParentPath
    cls._validateNodesAgainstStage = stage_adapter.validateNodesAgainstStage
    cls._adoptPrimsNewToScope = stage_adapter.adoptPrimsNewToScope
    cls._adoptRenamedPrim = stage_adapter.adoptRenamedPrim
    cls._remapMovedNodes = stage_adapter.remapMovedNodes
    cls._handleUsdChanges = stage_adapter.handleUsdChanges
    cls._clearRenderCache = render._clearRenderCache
    cls._loadSampleGraph = _loadSampleGraph
    cls.loadBlueprint = stage_adapter.loadBlueprint
    cls.loadContainer = stage_adapter.loadContainer
    cls.loadStage = stage_adapter.loadStage
    cls._resetRenderCachesAfterBulkMove = render._resetRenderCachesAfterBulkMove
    cls._valueRowOnlyChange = staticmethod(stage_adapter._valueRowOnlyChange)
    cls._isPinPropertyChange = staticmethod(stage_adapter._isPinPropertyChange)
    cls.mouseDoubleClickEvent = _interaction.guards_qt_boundary(interaction_events.mouseDoubleClickEvent)
    cls.mousePressEvent = _interaction.guards_qt_boundary(interaction_events.mousePressEvent)
    cls.mouseMoveEvent = _interaction.guards_qt_boundary(interaction_events.mouseMoveEvent)
    cls.mouseReleaseEvent = _interaction.guards_qt_boundary(interaction_events.mouseReleaseEvent)
    cls._reloadLinksFromUSD = stage_adapter._reloadLinksFromUSD
    cls._reloadCurrentGraph = stage_adapter._reloadCurrentGraph
    cls._isStageViewMidTeardown = stage_adapter._isStageViewMidTeardown
    cls._worldProjCacheKey = None
    cls._worldProjCache = None
    cls._screenProjCacheKey = None
    cls._screenProjCache = None
    cls._WARNING_POPUP_DURATION_MS = 1400
    cls._initUndoTracking = _editing._initUndoTracking
    cls._applyNodePositions = _editing._applyNodePositions
    cls._restoreNodePositions = _editing._restoreNodePositions
    cls._authorNodePosition = _editing._authorNodePosition
    cls._performUndo = _editing._performUndo
    cls._performRedo = _editing._performRedo
    cls._persistNodePositions = _editing._persistNodePositions
    cls._saveLayerAsSpelled = staticmethod(_editing._saveLayerAsSpelled)
    cls._saveStage = _editing._saveStage
    cls._deleteAllSelected = _editing._deleteAllSelected
    cls._deactivateNodesInUsd = _editing._deactivateNodesInUsd
    cls._pushDeleteSelectedUndo = _editing._pushDeleteSelectedUndo
    cls._applyRestoreDeleted = _editing._applyRestoreDeleted
    cls._applyDeleteSelected = _editing._applyDeleteSelected
    cls.deleteSelectedNodes = _editing.deleteSelectedNodes
    cls._createNodeFromHotbox = _editing._createNodeFromHotbox
    cls._resolveParentPath = _editing._resolveParentPath
    cls._beginRename = _editing._beginRename
    cls._cancelRename = _editing._cancelRename
    cls._commitRename = _editing._commitRename
    cls._onNodeCreated = _editing._onNodeCreated
    cls.linkSelectionMode = property(
        _selection.get_link_selection_mode, _selection.set_link_selection_mode
    )
    cls._updateLinkSelectionFromNodes = _selection._updateLinkSelectionFromNodes
    cls._selectLink = _selection._selectLink
    cls._toggleLinkSelection = _selection._toggleLinkSelection
    cls._syncLinkSelectionToPrimtree = _selection._syncLinkSelectionToPrimtree
    cls.clearLinkSelection = _selection.clearLinkSelection
    cls._getFirstSelectedLink = _selection._getFirstSelectedLink
    cls._selectFirstLinkInDirection = _selection._selectFirstLinkInDirection
    cls._getFirstSelectedNode = _selection._getFirstSelectedNode
    cls.selectLinkByPinIndex = _selection.selectLinkByPinIndex
    cls._onPropSelectionChanged = _selection._onPropSelectionChanged
    cls._onPrimSelectionChanged = _selection._onPrimSelectionChanged
    cls._syncSelectionToPrimtree = _selection._syncSelectionToPrimtree
    cls._frameInPrimTree = _selection._frameInPrimTree
    cls._setLinkSelectionMode = _selection._setLinkSelectionMode
    cls._toggleFrameInPrimTree = _selection._toggleFrameInPrimTree
    cls._clearSelectionAndSync = _selection._clearSelectionAndSync
    cls._canOpenSelectedContainer = _selection._canOpenSelectedContainer
    cls._selectNodesInMarquee = _selection._selectNodesInMarquee
    cls._selectLinksInMarquee = _selection._selectLinksInMarquee
    cls._updateMarqueeSelection = _selection._updateMarqueeSelection
    cls.selectAll = _selection.selectAll
    cls.clearSelection = _selection.clearSelection
    cls._getContainerChildNodes = _selection._getContainerChildNodes
    cls.addNodesFromPrimTreeSelection = _selection.addNodesFromPrimTreeSelection
    cls.removeSelectedNodes = _selection.removeSelectedNodes
    cls._resolveSelectedPrims = _selection._resolveSelectedPrims
    cls._resolveSelectedParentPrim = _selection._resolveSelectedParentPrim
    cls._addBackReferenceDanglingLinks = _graphview_model.add_back_reference_dangling_links
    cls._addLinkToGraph = _graphview_model.add_link_to_graph
    cls._addNewNodeFast = _graphview_model.add_new_node_fast
    cls._addNodeFromDanglingLink = _graphview_model.add_node_from_dangling_link
    cls._addNodesFromDanglingLink = _graphview_model.add_nodes_from_dangling_link
    cls._addPrimAsNode = _graphview_model.add_prim_as_node
    cls._classifyRelationshipLinkFromUsd = staticmethod(_graphview_model.classify_relationship_link_from_usd)
    cls._collectSiblingDanglingLinks = _graphview_model.collect_sibling_dangling_links
    cls._computeLinkBounds = _graphview_model.compute_link_bounds
    cls._findLinkToInput = _graphview_model.find_link_to_input
    cls._findPinIndex = _graphview_model.find_pin_index
    cls._getLinkEndpointPosition = _graphview_model.get_link_endpoint_position
    cls._getLinksForNodeDirection = _graphview_model.get_links_for_node_direction
    cls._getLinksForNodeFlat = _graphview_model.get_links_for_node_flat
    cls._hasExactLink = _graphview_model.has_exact_link
    cls._invalidateBackReferenceLinkDataCache = _graphview_model.invalidate_back_reference_link_data_cache
    cls._makeLinkFromUsdData = staticmethod(_graphview_model.make_link_from_usd_data)
    cls._populateNodeLinksFromPrim = _graphview_model.populate_node_links_from_prim
    cls._processNodeLinks = _graphview_model.process_node_links
    cls._rebuildConnectionCache = _graphview_model.rebuild_connection_cache
    cls._rebuildLinks = _graphview_model.rebuild_links
    cls._reconcileMovedNodes = _graphview_model.reconcile_moved_nodes
    cls._removeLinkFromGraph = _graphview_model.remove_link_from_graph
    cls._resolveLinkDisplayPinName = _graphview_model.resolve_link_display_pin_name
    cls._resolvePortPropertyName = _graphview_model.resolve_port_property_name
    cls._setupDanglingLinkEndpoint = _graphview_model.setup_dangling_link_endpoint
    cls._setupLinkEndpoints = _graphview_model.setup_link_endpoints
    cls._surfaceConnectionEndpointPins = _graphview_model.surface_connection_endpoint_pins
    cls._syncLinksIfNeeded = _graphview_model.sync_links_if_needed
    cls._updateLinksForMovedNodes = _graphview_model.update_links_for_moved_nodes
    cls._updateNodeLinkEndpoints = _graphview_model.update_node_link_endpoints
    cls._appendTriangleVertices = _graphview_model.append_triangle_vertices
    cls._applyMovedNodeDerivedState = _graphview_model.apply_moved_node_derived_state
    cls._autoLayoutNodes = _graphview_model.auto_layout_nodes
    cls._batchExtents = _graphview_model.batch_extents
    cls._getNodeTopCenterPosition = staticmethod(
        _graphview_model.get_node_top_center_position
    )
    cls._getPortPosition = _graphview_model.get_port_position
    cls._getPropertyOutputSide = staticmethod(_graphview_model.get_property_output_side)
    cls._getRelationshipRowEndpointPosition = (
        _graphview_model.get_relationship_row_endpoint_position
    )
    cls._getVisiblePortPosition = _graphview_model.get_visible_port_position
    cls._gridPlaceNodes = _graphview_model.grid_place_nodes
    cls._mirrorPortPosition = staticmethod(_graphview_model.mirror_port_position)
    cls._nodeHasAuthoredPosition = _graphview_model.node_has_authored_position
    cls._nodeHasVisiblePort = _graphview_model.node_has_visible_port
    cls._placeAddedNodes = _graphview_model.place_added_nodes
    cls._positionNodeInViewport = _graphview_model.position_node_in_viewport
    cls._pushBelowOccupied = _graphview_model.push_below_occupied
    cls._updateNodeSpatialBounds = _graphview_model.update_node_spatial_bounds
    cls.getConnectedNodes = _graphview_model.get_connected_nodes
    cls._asWorldXY = staticmethod(_graphview_model.as_world_xy)
    cls._editableValueCells = _graphview_model.editable_value_cells
    cls._findValueRow = _graphview_model.find_value_row
    cls._valueAxisWidth = _graphview_model.value_axis_width
    cls._valueCellRectFor = _graphview_model.value_cell_rect_for
    cls._valueDigitAdvance = _graphview_model.value_digit_advance
    cls._valueFittedTexts = _graphview_model.value_fitted_texts
    cls._valueFontSize = _graphview_model.value_font_size
    cls._valueRowGeometry = _graphview_model.value_row_geometry
    cls._valueRowLayout = _graphview_model.value_row_layout
    cls._valueStateColors = _graphview_model.value_state_colors
    cls._valueTextWidth = _graphview_model.value_text_width
    cls._finishConnectionReplacement = _editing._finishConnectionReplacement
    cls._deleteLinksInUsd = _editing._deleteLinksInUsd
    cls._abortConnection = _editing._abortConnection
    cls._resolveConnectionEndpoints = _editing._resolveConnectionEndpoints
    cls._deleteOldConnection = _editing._deleteOldConnection
    cls._createConnection = _editing._createConnection
    cls._deleteConnection = _editing._deleteConnection
    cls._applyApiSchemaFromHotbox = _editing._applyApiSchemaFromHotbox
    cls._showApiSchemaResult = _editing._showApiSchemaResult
    cls._writeValueComponents = _editing._writeValueComponents
    cls._pushValueUndo = _editing._pushValueUndo
    cls._updateMung = _editing._updateMung
    cls._finishMung = _editing._finishMung
    cls._cancelMung = _editing._cancelMung
    cls._toggleBoolValue = _editing._toggleBoolValue
    cls._beginValueEdit = _editing._beginValueEdit
    cls._cancelValueEdit = _editing._cancelValueEdit
    cls._commitValueEdit = _editing._commitValueEdit
    cls._advanceValueEdit = _editing._advanceValueEdit
    cls._commitTokenPopup = _editing._commitTokenPopup
    cls._collectSelectedLinkData = _intents._collectSelectedLinkData
    cls._collectIncidentLinkKeys = _intents._collectIncidentLinkKeys
    cls._makeSerializedLinkRecord = staticmethod(_intents._makeSerializedLinkRecord)
    cls._deserializeLinkRecord = staticmethod(_intents._deserializeLinkRecord)
    cls._linkMatchesRecord = classmethod(_intents._linkMatchesRecord)
    cls._expandSdfListOpItems = staticmethod(_intents._expandSdfListOpItems)
    cls._buildAttrSearchPaths = staticmethod(_intents._buildAttrSearchPaths)
    cls._buildRelSearchEntries = staticmethod(_intents._buildRelSearchEntries)
    cls._layerAuthorsAttrConnection = classmethod(_intents._layerAuthorsAttrConnection)
    cls._layerAuthorsRelTarget = classmethod(_intents._layerAuthorsRelTarget)
    cls._findConnectionAuthoringLayer = _intents._findConnectionAuthoringLayer
    cls._editOnLayer = _intents._editOnLayer
    cls._buildConnectionKwargs = staticmethod(_intents._buildConnectionKwargs)
    cls._groupLinksByLayer = staticmethod(_intents._groupLinksByLayer)
    cls._runLinkOperationOnLayer = _intents._runLinkOperationOnLayer
    cls._findLibraryForPrim = _intents._findLibraryForPrim
    cls._getNodeIdAtWorldPos = _interaction._getNodeIdAtWorldPos
    cls._getPropertyAtPoint = _interaction._getPropertyAtPoint
    cls._findPropertyRowHover = _interaction._findPropertyRowHover
    cls._findRelationshipRowHover = _interaction._findRelationshipRowHover
    cls._relationshipDragRowHover = _interaction._relationshipDragRowHover
    cls._getRowHitMetrics = _interaction._getRowHitMetrics
    cls._valueCellAtPoint = _interaction._valueCellAtPoint
    cls._findRowEdgeDragStart = _interaction._findRowEdgeDragStart
    cls._rowEdgePinCandidates = _interaction._rowEdgePinCandidates
    cls._pinAtRowEdge = _interaction._pinAtRowEdge
    cls._findArrowheadUnderCursor = _interaction._findArrowheadUnderCursor
    cls._findDanglingIndicatorUnderCursor = _interaction._findDanglingIndicatorUnderCursor
    cls._findDanglingLinkAtPort = _interaction._findDanglingLinkAtPort
    cls._updateHoveredPort = _interaction._updateHoveredPort
    cls._setHoveredPort = _interaction._setHoveredPort
    cls._updateNodeUnderCursor = _interaction._updateNodeUnderCursor
    cls._updateValueHoverCursor = _interaction._updateValueHoverCursor
    cls._showPropertyTooltip = _interaction._showPropertyTooltip
    cls._updatePropertyTooltip = _interaction._updatePropertyTooltip
    cls._clearPropertyTooltip = _interaction._clearPropertyTooltip
    cls._centerOnNode = _interaction._centerOnNode
    cls._setZoom = _interaction._setZoom
    cls._zoomAroundPoint = _interaction._zoomAroundPoint
    cls.navigateBackward = _interaction.navigateBackward
    cls.navigateForward = _interaction.navigateForward
    cls._jumpToSourceNode = _interaction._jumpToSourceNode
    cls._jumpToTargetNode = _interaction._jumpToTargetNode
    cls._jumpToSourceNodeInternal = _interaction._jumpToSourceNodeInternal
    cls._jumpToTargetNodeInternal = _interaction._jumpToTargetNodeInternal
    cls._navigateAlongRelationshipLink = _interaction._navigateAlongRelationshipLink
    cls._clearValueInteractionState = _interaction._clearValueInteractionState
    cls._updateAnimator = _interaction._updateAnimator
    cls._updatePanAnimation = _interaction._updatePanAnimation
    cls._updateKeyboardState = _interaction._updateKeyboardState
    cls._finalizeNodeDrag = _interaction._finalizeNodeDrag
    cls._startLinkDrag = _interaction._startLinkDrag
    cls._snapLinkDragToPort = _interaction._snapLinkDragToPort
    cls._updateLinkDrag = _interaction._updateLinkDrag
    cls._completeLinkDrag = _interaction._completeLinkDrag
    cls._startLinkReconnect = _interaction._startLinkReconnect
    cls._clearReconnectState = _interaction._clearReconnectState
    cls._cancelLinkReconnect = _interaction._cancelLinkReconnect
    cls._finishReconnectDisconnect = _interaction._finishReconnectDisconnect
    cls.initializeGL = _resources.initializeGL
    cls._reinitializeGL = _resources._reinitializeGL
    cls.resizeGL = _resources.resizeGL
    cls.paintLinks = _render_links.paintLinks
    cls._relationshipArrowheadPoints = staticmethod(functools.partial(
        _render_links._relationshipArrowheadPoints, None
    ))
    cls._renderRelationshipArrowheads = _render_links._renderRelationshipArrowheads
    cls._renderDanglingIndicators = _render_links._renderDanglingIndicators
    cls._renderTemporaryLink = _render_links._renderTemporaryLink
    cls.paintStatusOverlay = _render_overlays.paintStatusOverlay
    cls.paintMarquee = _render_overlays.paintMarquee
    cls._paintNodeCreationHotbox = _render_hotbox._paintNodeCreationHotbox
    cls.paintGL = _render.paintGL
    cls._paintGLInner = _render._paintGLInner
    cls._renderConfig = _render._renderConfig
    cls._seedNodeZOrders = _render._seedNodeZOrders
    cls._worldSpaceProjectionMatrix = _render._worldSpaceProjectionMatrix
    cls._screenSpaceProjectionMatrix = _render._screenSpaceProjectionMatrix
    cls.paintNodes = _render_nodes.paintNodes
    cls._drawNodeVertices = _render_nodes._drawNodeVertices
    cls._renderPortHighlightsCpp = _render_nodes._renderPortHighlightsCpp
    cls.paintGroupStickers = _render_nodes.paintGroupStickers
    cls._relationshipLinkColor = _render_nodes._relationshipLinkColor
    cls._isPrimTargetRelationshipLink = staticmethod(
        _render_nodes._isPrimTargetRelationshipLink
    )
    cls.paintText = _render_text.paintText
    cls._paintValueCells = _render_text._paintValueCells
    cls._paintValueEditor = _render_text._paintValueEditor
    cls._emitValueText = _render_text._emitValueText
    cls._valueTextBand = _render_text._valueTextBand
    cls._valueGlyph = _render_text._valueGlyph
    cls._valueBaselineY = _render_text._valueBaselineY
    cls._appendBoolSwitch = _render_text._appendBoolSwitch
    cls._appendChevron = _render_text._appendChevron
    cls._paintNodeRename = _render_text._paintNodeRename
    cls._paintMungLadder = _render_text._paintMungLadder
    cls._paintTokenPopup = _render_text._paintTokenPopup
    cls._activeValueFlash = _render_text._activeValueFlash
    cls._startValueFlash = _render_text._startValueFlash
    cls.paintHotkeyPopup = _render_text.paintHotkeyPopup
    cls.paintNavigationHotkeyPopup = _render_text.paintNavigationHotkeyPopup
    cls._paintHotkeyPopups = _render_text._paintHotkeyPopups
