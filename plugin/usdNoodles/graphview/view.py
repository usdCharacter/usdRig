#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#
"""Compose the noodle editor's model, widget, stage lifecycle and commands."""

import traceback

from pxr import Gf, Tf, Usd
from UsdNoodles.core import Animator, LinkData, LinkSelectionMode
from UsdNoodles.render import NodeRenderManager, NodeTransformFrame, RenderProfiler
from UsdNoodles.spatial import SpatialIndex

from ..linkRenderer import LinkRenderer
from ..nodeFactory import NodeFactory
from ..nodeGraph import NodeGraph, createNodeRenderer, subscribe_edit_target_warning
from ..nodeLibs.registry import NodeLibraryRegistry
from ..textRenderer import TextRenderer
from ..widgets import (
    GroupStickerRenderer, HotkeyPopup, Minimap, NavigationHotkeyPopup,
    NodeCreationHotbox, StatusOverlay,
)
from ..widgets import valueCell
from ..widgets.selectableList import SelectableList
from ..widgets.textInputWidget import TextInputWidget
from . import editing as _editing, intents as _intents, selection as _selection
from . import interaction as _interaction
from .compat import bind_compat
from .interaction import GraphWidgetBase
from .legacy import legacy_symbol
from .stage_adapter import _frameOf, _makeStageSync


def _connect(connections, signal, callback):
    signal.connect(callback)
    connections.append((signal, callback))


def _disconnect(connections):
    while connections:
        signal, callback = connections.pop()
        signal.disconnect(callback)


def _disposeView(view):
    _disconnect(view._hostConnections)
    _disconnect(view._connections)
    view._usdviewApi = None
    view._disposeOwnerCallbacks()
    resources = getattr(view, "_render_resources", None)
    if resources is not None and resources.frame_source is not None:
        resources.frame_source.close()
        resources.frame_source = None


def _getNodeGraph(view):
    return view._nodeGraph


def _setNodeGraph(view, value):
    if view._nodeGraph is value:
        return
    view._nodeGraph = value
    if hasattr(view, "_selection"):
        view._selection.update("replace", (), ())
        view._marqueeBase = _selection.Selection()
        view.marqueeActive = False
    view.nodeGraphReplaced.emit()


def __init__(view, parent=None, settingsObject=None):
    GraphWidgetBase.__init__(view, parent)
    view._initOwnerCallbacks()
    view._connections = []
    view._hostConnections = []
    view._nodeGraph = None
    view._undoHistory = None
    view._commandRegistry = view._makeCommandRegistry()
    view._editor = _editing.editor_for_view(view, view._commandRegistry)
    view._assetsPath = None
    view._configureWidgetFormat()
    NoodlesConfig = legacy_symbol("NoodlesConfig")

    if settingsObject:
        NoodlesConfig.initialize(settingsObject, graphView=view)
    view.zoom = 1.0
    view.panX = 0.0
    view.panY = 0.0
    view.lastMousePos = view._widgetPoint()
    view.nodeGraph = NodeGraph()
    view._startupSamplePending = True

    subscribe_edit_target_warning(view._showWarningPopup)
    view._currentPrimPath = None
    view.drawLinks = True
    view.drawNodes = True
    view.drawText = True
    view.animator = Animator()
    view.linkDimming = view.animator.addProperty()
    view.linkDimming.speed = 1.0
    view.linkDimming.target = 1.0
    view.linkDimming.current = 1.0
    view.initialized = False
    view.shaderLibrary = None
    view.linkRenderer = LinkRenderer()
    view.textRenderer = TextRenderer()
    view.statusOverlay = StatusOverlay()
    view.defaultRenderer = createNodeRenderer("default")
    view.draggingNodes = False
    view.spacePressed = False
    view.ctrlPressedOnMouseDown = False
    view.mousePosOnMouseDown = view._widgetPoint()
    view.nodeIdUnderCursor = ""
    view.marqueeActive = False
    view.marqueeStart = view._widgetPointF()
    view.marqueeCurrent = view._widgetPointF()
    view.textChanged = True

    panSpeed = NoodlesConfig.get("panAnimationSpeed", 18.0)
    view._panXAnim = view.animator.addProperty()
    view._panXAnim.speed = panSpeed
    view._panXAnim.target = 0.0
    view._panXAnim.current = 0.0
    view._panYAnim = view.animator.addProperty()
    view._panYAnim.speed = panSpeed
    view._panYAnim.target = 0.0
    view._panYAnim.current = 0.0
    view._panAnimating = False
    view.hotkeyPopup = HotkeyPopup(view.animator)
    view.hotkeyPopup.set_parent_widget(view)
    view.navigationHotkeyPopup = NavigationHotkeyPopup(view.animator)
    view.frameTimer = view._widgetElapsedTimer()
    view._usdviewApi = None
    view.minimap = Minimap(view)
    view._nodeLibraries = NodeLibraryRegistry().discover()

    enabled_names = NoodlesConfig.get("enabledLibraries", None)
    if enabled_names is not None:
        for lib in view._nodeLibraries:
            lib.enabled = lib.get_name() in enabled_names
    view._nodeFactory = NodeFactory(view._nodeLibraries)
    view.nodeCreationHotbox = NodeCreationHotbox(view.animator)
    view.nodeCreationHotbox.set_parent_widget(view)
    view._updateNodeCreationHotbox()  # Configure from libraries
    view.nodeCreationHotbox.on_create = view._createNodeFromHotbox
    view._configureWidgetInput()
    view._lastPrimTreeSelection = []

    view._previousActiveWindow = None

    view._linkSelectionMode = LinkSelectionMode.NODES_ONLY
    view._selection = _selection.Selection()
    view._marqueeBase = _selection.Selection()
    view._marqueeMode = "replace"
    view._primTreeAdapter = _selection.make_prim_tree_adapter(view)

    view._renamingNodeId = ""
    view._renameInput = TextInputWidget()
    view._renameInput.placeholder = "prim name"

    view._valueEditTarget = None
    view._valueEditInput = TextInputWidget()
    view._valueEditInput.placeholder = "value"
    view._valueEditFresh = True

    view._valueEditError = False
    view._mungState = None
    view._mungActive = False
    view._mungSuppression = None
    view._tokenPopup = None
    view._tokenPopupList = SelectableList(max_visible_items=8)
    view._currentTimeCode = Usd.TimeCode.Default()
    view._valueCursorSet = None
    view._valueHover = None
    view._valueFlash = None
    view._cachedShowAttributeValues = NoodlesConfig.get(
        "showAttributeValues", True
    )
    view._cachedValueWriteMode = NoodlesConfig.get("valueWriteMode", "auto")

    view._linkUnderCursor = -1
    view._linksNeedSync = True
    view._lastSyncedLinkHover = -1
    view._portHighlightsDirty = True
    view._reconcilePositionGen = 0
    view._reconciledPositionVersions = {}

    view._portHitRadiusMultiplierDefault = 1.5

    view._hoveredPort = None  # tuple: (nodeId, portName, isOutput) or None
    view._hoveredPortColor = (1.0, 1.0, 0.0, 1.0)  # Yellow

    view._tooltipProperty = None  # (nodeId, pinName) or None
    view._tooltipScreenPos = None  # QPoint for tooltip position
    view._tooltipTimer = view._createTooltipTimer()

    view._draggingLink = False
    view._dragLinkSourceNode = None  # nodeId
    view._dragLinkSourcePort = None  # portName
    view._dragLinkSourceIsOutput = None  # bool
    view._dragLinkTempLink = None  # Reserved temporary LinkData (initialized below)
    view._dragLinkCursorPos = None  # Current cursor world position
    view._dragLinkValidTarget = False  # Is current hover a valid connection?
    view._dragLinkSyntheticTarget = None  # nodeId, portName, isOutput
    view._dragLinkResolvedTarget = None  # nodeId, portName, isOutput, propertyName

    view._dragLinkTempLink = LinkData()
    view._dragLinkTempLink.sourceNodeId = ""
    view._dragLinkTempLink.targetNodeId = ""
    view._dragLinkTempLink.sourcePort = ""
    view._dragLinkTempLink.targetPort = ""
    view._dragLinkTempLink.sourcePropertyName = ""
    view._dragLinkTempLink.targetPropertyName = ""
    view._dragLinkTempLink.is_relationship_link = False

    view._reconnectingLink = False
    view._reconnectOldSourceNodeId = None
    view._reconnectOldSourcePort = None
    view._reconnectOldTargetNodeId = None
    view._reconnectOldTargetPort = None
    view._reconnectOldSourcePropertyName = ""
    view._reconnectOldTargetPropertyName = ""
    view._reconnectCapture = None

    view._frameInPrimTreeOnSelect = NoodlesConfig.get(
        "frameInPrimTreeOnSelect", True
    )

    view._lastJumpDirection = "target"

    view._lastSelectedNode = None
    view._lastPinIndex = -1
    view._lastLinkCycleIndex = 0

    view._lastInputLinkIndex = -1
    view._lastOutputLinkIndex = -1

    view._lastDanglingLinkIndex = -1
    view._danglingLinkFollowCount = 0

    view._contextMenu = None
    view._contextMenuActions = {}
    view._initContextMenu()

    view._setupShortcuts()

    view._preferencesDlg = None

    view._initCachedSettings()

    settings = NoodlesConfig.settings()
    if settings:
        _connect(view._connections, settings.signalSettingChanged, view._onSettingsChanged)

    view.fontAtlas = None

    view._profiler = RenderProfiler(60)
    view._profiler.enabled = NoodlesConfig.get("enableRenderProfiling", False)
    view._profilingFrameCount = 0

    view._nodeRenderManager = NodeRenderManager()
    view._nodeTransformFrame = NodeTransformFrame()
    view._cachedRenderConfig = None
    view._groupStickerRenderer = GroupStickerRenderer()

    view._deferRenderCacheClear = False
    view._renderCacheClearPending = False

    view._spatialIndex = SpatialIndex(maxDepth=4, maxItems=20)

    view._nextZOrder = 0

    view._stageSync = _makeStageSync(view)

def nodes(view):
    return view.nodeGraph.nodes


def links(view):
    return view.nodeGraph.links


def groupStickers(view):
    return view.nodeGraph.groupStickers


def _getLinksChanged(view):
    return view.nodeGraph.linksChanged


def _setLinksChanged(view, value):
    view.nodeGraph.linksChanged = value


def setUsdviewApi(view, usdviewApi):
    if view._stageSync.isClosed():
        return
    _disconnect(view._hostConnections)
    view._usdviewApi = usdviewApi
    dataModel = getattr(usdviewApi, "dataModel", None)
    selection = getattr(dataModel, "selection", None)
    for source, signal_name, callback in (
        (selection, "signalPrimSelectionChanged", view._onPrimSelectionChanged),
        (selection, "signalPropSelectionChanged", view._onPropSelectionChanged),
        (dataModel, "signalStageReplaced", view._onStageReplaced),
        (dataModel, "currentFrameChanged", view._onFrameChanged),
    ):
        signal = getattr(source, signal_name, None)
        if signal is not None:
            _connect(view._hostConnections, signal, callback)

    view._currentTimeCode = _frameOf(
        getattr(usdviewApi, "frame", Usd.TimeCode.Default())
    )


def _onFrameChanged(view, frame):
    """Follow usdview's playhead so time-sampled values read at the frame.

    The SIGNAL's frame, never ``dataModel.currentFrame``: the setter emits
    before it assigns, so reading the model back here gives the PREVIOUS
    frame. (The avar editor records the same trap at
    rigExecUsdview/avarEditorUI.py:573.)

    Guarded explicitly: this is a Qt slot, not a call from inside an
    already-guarded handler, so an exception escaping it would take the
    shiboken path @_guardsQtBoundary exists to keep us off.
    """
    try:
        newTime = _frameOf(frame)
        if newTime == view._currentTimeCode:
            return
        view._currentTimeCode = newTime
        # Only the rows that can change with time need re-reading, but the
        # per-node dict is cheap and dropping it whole cannot go stale.
        for node in view.nodes.values():
            if hasattr(node, "invalidate_value_row"):
                node.invalidate_value_row()
        view.update()
    except Exception:
        Tf.Warn(
            "GraphView._onFrameChanged raised; the frame was ignored:\n%s"
            % traceback.format_exc()
        )


def _onSettingsChanged(view):
    """Called when settings change via preferences dialog.
    Updates cached values so we don't read from config every frame."""
    view._initCachedSettings()
    view.updateShortcuts()
    view._profiler.enabled = view._cachedEnableRenderProfiling
    if view._profiler.enabled:
        view._profiler.reset()
    view._panXAnim.speed = view._cachedPanAnimationSpeed
    view._panYAnim.speed = view._cachedPanAnimationSpeed
    # Recalculate all node sizes + layouts (font sizes / margins may have
    # changed, which affects the cached layout metrics every renderer reads).
    if view.nodeGraph and view.fontAtlas:
        for node in view.nodes.values():
            view.nodeGraph._calculateNodeSize(
                node,
                view.textRenderer.calculateTextWidth
                if view.textRenderer
                else lambda t, s: s * 5,
                view.fontAtlas,
            )
    view.textChanged = True
    view._clearRenderCache()
    view.update()


def _initCachedSettings(view):
    """Initialize/update all cached settings from NoodlesConfig.
    Called on init and when settings change."""
    NoodlesConfig = legacy_symbol("NoodlesConfig")
    # Drop the cached RenderConfig so the next paint rebuilds it from the new
    # settings.
    view._cachedRenderConfig = None
    view._cachedLinkHoveredColor = NoodlesConfig.get("linkHoveredColor", [1.0, 1.0, 0.0, 1.0])
    view._cachedLinkSelectedColor = NoodlesConfig.get("linkSelectedColor", [1.0, 1.0, 0.3, 1.0])
    view._cachedLinkActiveColor = NoodlesConfig.get("linkActiveColor", [0.0, 0.8, 1.0, 1.0])
    view._cachedLinkHighlightedColor = NoodlesConfig.get("linkHighlightedColor", [0.31, 0.78, 0.47, 1.0])
    view._cachedLinkAttributeBaseColor = NoodlesConfig.get("linkAttributeBaseColor", [0.31, 0.55, 0.86, 1.0])
    view._cachedLinkRelationshipBaseColor = NoodlesConfig.get("linkRelationshipBaseColor", [0.78, 0.24, 0.24, 1.0])
    view._cachedNodeSelectedStrokeColor = NoodlesConfig.get("nodeSelectedStrokeColor", [1.0, 1.0, 0.3, 1.0])
    view._cachedPanAnimationSpeed = NoodlesConfig.get("panAnimationSpeed", 18.0)
    # Hit detection. Link picking matches the drawn ribbon width (its visible
    # 0.5 SDF edge); see linkRenderer.findLinkUnderCursor.
    view._cachedLinkLineWidth = NoodlesConfig.get("linkLineWidth", 12.0)
    view._cachedPortHitRadiusMultiplier = NoodlesConfig.get("portHitRadiusMultiplier", 1.5)
    view._cachedFrameInPrimTreeOnSelect = NoodlesConfig.get("frameInPrimTreeOnSelect", True)
    view._cachedEnableRenderProfiling = NoodlesConfig.get("enableRenderProfiling", False)
    view._cachedProfilingPrintInterval = NoodlesConfig.get("profilingPrintInterval", 60)
    view._cachedNodeMarginV = NoodlesConfig.get("nodeMarginV", 18.0)
    view._cachedNodePinFontSize = NoodlesConfig.get("nodePinFontSize", 18.0)
    view._cachedPortRingThickness = NoodlesConfig.get("portRingThickness", 2.0)
    view._cachedPortConnectedFillColor = NoodlesConfig.get("portConnectedFillColor", [0.7, 0.8, 0.0, 1.0])
    view._cachedPortConnectedRingColor = NoodlesConfig.get("portConnectedRingColor", [1.0, 1.0, 1.0, 0.0])
    view._cachedPortDisconnectedFillColor = NoodlesConfig.get("portDisconnectedFillColor", [0.235, 0.235, 0.235, 1.0])
    view._cachedPortDisconnectedRingColor = NoodlesConfig.get("portDisconnectedRingColor", [0.7, 0.8, 0.0, 1.0])
    view._cachedPortHoverColor = NoodlesConfig.get("portHoverColor", [0.4, 0.7, 1.0, 1.0])
    # Minimap overlay visibility. Off skips both the per-frame minimap draw
    # (which uploads a rect per node) and its mouse hit-testing.
    view._cachedShowMinimap = NoodlesConfig.get("showMinimap", True)
    view._cachedShowAttributeValues = NoodlesConfig.get("showAttributeValues", True)
    view._cachedValueMinPixelHeight = NoodlesConfig.get("valueMinPixelHeight", 9.0)
    view._cachedMungDragThresholdPx = NoodlesConfig.get("mungDragThresholdPx", 3.0)
    view._cachedValueWriteMode = NoodlesConfig.get("valueWriteMode", "auto")
    view._cachedValueFontScale = NoodlesConfig.get("valueFontScale", 0.9)
    # Every value-cell colour is derived from the node theme, so a change
    # to the node background or the selection accent carries through to
    # the cells without anyone maintaining a second palette.
    view._cachedValueTheme = valueCell.build_theme(
        NoodlesConfig.get("nodeBgHigh", 90),
        NoodlesConfig.get("nodeBgLow", 86),
        NoodlesConfig.get("selectedNodeStrokeColor", [1.0, 1.0, 0.117, 1.0]),
        NoodlesConfig.get("portConnectedFillColor", [0.7, 0.8, 0.0, 1.0]),
        NoodlesConfig.get(
            "portDisconnectedFillColor", [0.235, 0.235, 0.235, 1.0]
        ),
        NoodlesConfig.get("valueConnectedColor", [0.45, 0.62, 0.85, 1.0]),
        NoodlesConfig.get("valueAnimatedColor", [0.95, 0.72, 0.25, 1.0]),
        NoodlesConfig.get("valueAuthoredColor", [0.93, 0.93, 0.95, 1.0]),
        NoodlesConfig.get("valueFallbackColor", [0.93, 0.93, 0.95, 0.42]),
    )
    # Digit advances are measured once per font size so numbers are set
    # TABULAR: a value dragged from 1.9 to 2.0 must not make the whole
    # number jump sideways because '1' is half the width of '0'.
    view._valueDigitAdvanceCache = {}
    # Paint-pass caches, recreated here (and only here) so a font change
    # cannot leave stale glyphs behind: fitted value texts keyed by
    # (node, property, components, width, font), single-glyph vertices
    # keyed by (character, size), axis-label widths, and text-band
    # geometry. Every key carries what the entry depends on, so a value,
    # layout or font change misses and recomputes instead of going stale.
    view._valueFitCache = {}
    view._valueGlyphCache = {}
    view._valueAxisWidthCache = {}
    view._valueTextBandCache = {}


def _toggleDrawLinks(view, checked):
    view.drawLinks = checked
    view.update()


def _toggleDrawNodes(view, checked):
    view.drawNodes = checked
    view.update()


def _toggleDrawText(view, checked):
    view.drawText = checked
    view.update()


def _toggleShowAttributeValues(view, checked):
    """Turn the inline value cells on or off.

    Shipped alongside the feature on purpose: the cells WIDEN nodes, so a
    graph laid out before they existed gets denser, and the user needs one
    switch rather than an argument.
    """
    view._cachedShowAttributeValues = bool(checked)
    legacy_symbol("NoodlesConfig").set("showAttributeValues", bool(checked))
    view._clearValueInteractionState()
    # The width reserve is decided by the layout pass, so this is a
    # content change, not a repaint.
    view.textChanged = True
    view.linksChanged = True
    view._nodeRenderManager.invalidateAll()
    view.update()


def _setValueWriteMode(view, mode):
    """Where inline value edits land: auto / default / animation.

    Same three tokens the avar editor and the gizmo toolbar use
    (gizmoMath.WRITE_DEFAULT / WRITE_ANIMATION), so a user who knows one
    knows this one.
    """
    view._cachedValueWriteMode = mode
    legacy_symbol("NoodlesConfig").set("valueWriteMode", mode)


def _toggleLinkDimming(view, checked):
    view.linkDimming.target = 1.0 if checked else 0.0
    view.update()


def _visibleWorldBounds(view):
    """What the camera currently shows, as a world-space rect.

    ``(panX, panY)`` is its top-left corner; the extent is the widget size
    divided by the zoom.
    """
    zoom = view.zoom if view.zoom else 1.0
    return Gf.Range2d(
        Gf.Vec2d(view.panX, view.panY),
        Gf.Vec2d(view.panX + view.width() / zoom, view.panY + view.height() / zoom),
    )


def _isVisible(view, bounds):
    """Whether any part of *bounds* falls inside the current view."""
    if bounds.IsEmpty():
        return False
    return not Gf.Range2d.GetIntersection(
        view._visibleWorldBounds(), bounds
    ).IsEmpty()


def _viewportAnchor(view):
    """Top-left of what the camera currently shows, in world space, inset.

    The inset is a fraction of the visible extent rather than a fixed
    distance so it reads as the same margin whatever the zoom.
    """
    visible = view._visibleWorldBounds()
    size = Gf.Vec2d(visible.GetSize())
    return (
        visible.GetMin()[0] + size[0] * 0.05,
        visible.GetMin()[1] + size[1] * 0.05,
    )


def currentPrimPath(view):
    return view._currentPrimPath


def _printProfilingStatsIfNeeded(view):
    """Print profiling stats every N frames when profiling is enabled."""
    if view._profiler.enabled:
        view._profilingFrameCount += 1
        print_interval = view._cachedProfilingPrintInterval
        if view._profilingFrameCount >= print_interval:
            view._profiler.printStats()
            view._profilingFrameCount = 0
            view._profiler.reset()


def frameAll(view):
    """Frame all nodes in the graph, ignoring current selection."""
    if not view.nodes:
        return

    sceneBounds = Gf.Range2d()
    for node in view.nodes.values():
        nodeBounds = Gf.Range2d(node.position, node.position + node.size)
        sceneBounds = Gf.Range2d.GetUnion(sceneBounds, nodeBounds)

    if sceneBounds.IsEmpty():
        return

    boundsSize = Gf.Vec2d(sceneBounds.GetSize())
    if boundsSize[0] == 0 or boundsSize[1] == 0:
        return

    # Frame with a comfortable margin (1.1 = 10% padding)
    view._frameNodeBounds(sceneBounds, margin=1.1)
    view.update()


def frameScene(view):
    if not view.nodes:
        return

    if view._usdviewApi:
        try:
            dataModel = view._usdviewApi.dataModel
            if dataModel and dataModel.selection:
                selectedPrims = dataModel.selection.getPrims()
                if selectedPrims:
                    matchedNodes = []
                    for prim in selectedPrims:
                        primPathStr = str(prim.GetPath())
                        if primPathStr in view.nodes:
                            matchedNodes.append(view.nodes[primPathStr])

                    if matchedNodes:
                        view.clearSelection()
                        for node in matchedNodes:
                            _selection.select_node(view, node.id)
        except Exception:
            pass

    hasSelection = any(node.selected for node in view.nodes.values())
    sceneBounds = Gf.Range2d()
    selectionBounds = Gf.Range2d()

    for node in view.nodes.values():
        nodeBounds = Gf.Range2d(node.position, node.position + node.size)
        sceneBounds = Gf.Range2d.GetUnion(sceneBounds, nodeBounds)
        if node.selected:
            selectionBounds = Gf.Range2d.GetUnion(selectionBounds, nodeBounds)

    bounds = selectionBounds if hasSelection else sceneBounds
    if bounds.IsEmpty():
        return

    boundsSize = Gf.Vec2d(bounds.GetSize())
    if boundsSize[0] == 0 or boundsSize[1] == 0:
        return

    if hasSelection:
        # Expand bounds to 3x so selection occupies ~1/3 of view
        boundsCenter = Gf.Vec2d(
            (bounds.GetMin()[0] + bounds.GetMax()[0]) * 0.5,
            (bounds.GetMin()[1] + bounds.GetMax()[1]) * 0.5,
        )
        expandedSize = boundsSize * 3.0
        bounds = Gf.Range2d(
            boundsCenter - expandedSize * 0.5, boundsCenter + expandedSize * 0.5
        )
        boundsSize = Gf.Vec2d(bounds.GetSize())

    viewSize = Gf.Vec2d(view.width(), view.height())
    scale = Gf.Vec2d(viewSize[0] / boundsSize[0], viewSize[1] / boundsSize[1])
    view.zoom = min(scale[0], scale[1])

    scaledView = viewSize / view.zoom
    offset = (scaledView - boundsSize) * 0.5
    view.panX = bounds.GetMin()[0] - offset[0]
    view.panY = bounds.GetMin()[1] - offset[1]
    view.update()


def frameSelectionWithConnections(view):
    if not view.nodes:
        return

    selectedNodeIds = set()
    for nodeId, node in view.nodes.items():
        if node.selected:
            selectedNodeIds.add(nodeId)

    if not selectedNodeIds:
        view.frameScene()
        return

    connectedNodeIds = set()
    for nodeId in selectedNodeIds:
        connectedNodeIds.update(view.getConnectedNodes(nodeId))

    allNodeIds = selectedNodeIds | connectedNodeIds

    bounds = Gf.Range2d()
    for nodeId in allNodeIds:
        if nodeId not in view.nodes:
            continue
        node = view.nodes[nodeId]
        nodeBounds = Gf.Range2d(node.position, node.position + node.size)
        bounds = Gf.Range2d.GetUnion(bounds, nodeBounds)

    if not bounds.IsEmpty():
        view._frameNodeBounds(bounds)


def _frameNodeBounds(view, bounds, margin=1.0):
    """Frame the view to show the given bounds with an optional margin.

    Args:
        bounds: Gf.Range2d containing the bounds to frame
        margin: Scale factor for the view (1.0 = fit exactly, 1.4 = 40% extra space)
    """
    if bounds.IsEmpty():
        return

    boundsSize = Gf.Vec2d(bounds.GetSize())
    if boundsSize[0] == 0 or boundsSize[1] == 0:
        return

    boundsCenter = Gf.Vec2d(
        (bounds.GetMin()[0] + bounds.GetMax()[0]) * 0.5,
        (bounds.GetMin()[1] + bounds.GetMax()[1]) * 0.5,
    )
    expandedSize = boundsSize * margin
    bounds = Gf.Range2d(
        boundsCenter - expandedSize * 0.5, boundsCenter + expandedSize * 0.5
    )
    boundsSize = Gf.Vec2d(bounds.GetSize())

    viewSize = Gf.Vec2d(view.width(), view.height())
    scale = Gf.Vec2d(viewSize[0] / boundsSize[0], viewSize[1] / boundsSize[1])
    view.zoom = min(scale[0], scale[1])

    scaledView = viewSize / view.zoom
    offset = (scaledView - boundsSize) * 0.5
    view.panX = bounds.GetMin()[0] - offset[0]
    view.panY = bounds.GetMin()[1] - offset[1]


def _afterUsdviewRefresh(view, callback):
    """Run after usdview has rebuilt the prim browser for a namespace edit."""
    _interaction.start_refresh_timer(view, callback)


def _makeCommandRegistry(view):
    registry = _intents.CommandRegistry()
    for intent_type, method in (
        (_intents.SelectAll, "selectAll"),
        (_intents.ClearSelection, "_clearSelectionAndSync"),
        (_intents.FinishNodeDrag, "_finalizeNodeDrag"),
        (_intents.AutoLayout, "_autoLayoutNodes"),
        (_intents.OpenContainer, "_openSelectedContainer"),
        (_intents.SaveStage, "_saveStage"),
        (_intents.Undo, "_performUndo"),
        (_intents.Redo, "_performRedo"),
        (_intents.DeleteSelection, "_deleteAllSelected"),
    ):
        registry.register(intent_type, lambda _, method=method: getattr(view, method)())
    registry.register(
        _intents.Navigate,
        lambda request: getattr(view, {
            "backward": "navigateBackward",
            "forward": "navigateForward",
        }[request.direction])(),
    )
    registry.register(
        _intents.BeginNodeDrag,
        lambda request: _interaction.begin_node_drag(
            view, request.node_id, request.toggle
        ),
    )
    registry.register(
        _intents.UpdateNodeDrag,
        lambda request: _interaction.move_dragged_nodes(
            view, request.dx, request.dy
        ),
    )
    registry.register(
        _intents.BeginMarquee,
        lambda request: _interaction.begin_marquee(
            view, request.origin, request.mode
        ),
    )
    registry.register(
        _intents.UpdateMarquee,
        lambda request: _interaction.update_marquee(view, request.current),
    )
    registry.register(
        _intents.FinishMarquee,
        lambda request: _interaction.finish_marquee(view),
    )
    registry.register(
        _intents.Pan,
        lambda request: _interaction.pan_viewport(view, request.dx, request.dy),
    )
    registry.register(
        _intents.Zoom,
        lambda request: _interaction.zoom_viewport(
            view, request.focus, request.zoom
        ),
    )

    return registry


class GraphView(GraphWidgetBase):
    """Coordinate the graph editor’s widget, model and host lifecycle."""
    _AUTO_GRID_COLUMNS = 30
    _AUTO_GRID_GAP = 150.0
    _selectedNodes = property(_selection.selected_node_ids)
    _selectedLinks = property(_selection.selected_link_indices)
    _syncingSelection = property(_selection.syncing_selection)
    nodeGraph = property(_getNodeGraph, _setNodeGraph)
    nodes = property(nodes)
    links = property(links)
    groupStickers = property(groupStickers)
    linksChanged = property(_getLinksChanged, _setLinksChanged)
    currentPrimPath = property(currentPrimPath)
    _disposeView = _disposeView
    __init__ = __init__
    setUsdviewApi = setUsdviewApi
    _onFrameChanged = _onFrameChanged
    _onSettingsChanged = _onSettingsChanged
    _initCachedSettings = _initCachedSettings
    _toggleDrawLinks = _toggleDrawLinks
    _toggleDrawNodes = _toggleDrawNodes
    _toggleDrawText = _toggleDrawText
    _toggleShowAttributeValues = _toggleShowAttributeValues
    _setValueWriteMode = _setValueWriteMode
    _toggleLinkDimming = _toggleLinkDimming
    _visibleWorldBounds = _visibleWorldBounds
    _isVisible = _isVisible
    _viewportAnchor = _viewportAnchor
    _printProfilingStatsIfNeeded = _printProfilingStatsIfNeeded
    frameAll = frameAll
    frameScene = frameScene
    frameSelectionWithConnections = frameSelectionWithConnections
    _frameNodeBounds = _frameNodeBounds
    _afterUsdviewRefresh = _afterUsdviewRefresh
    _makeCommandRegistry = _makeCommandRegistry


bind_compat(GraphView)
