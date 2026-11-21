#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

import ctypes
import functools
import math
import os
import time
import traceback
from collections import defaultdict, namedtuple
from pathlib import Path

from UsdNoodles.core import (
    Animator,
    LinkData,
    LinkSelectionMode,
    positionWriteGeneration,
)
from UsdNoodles.render import (
    FontAtlas,
    NODE_VERTEX_ATTRIB_LAYOUT,
    NodeRenderManager,
    NodeTransformFrame,
    NodeVertex,
    RenderProfiler,
    ShaderLibrary,
)
from UsdNoodles.spatial import SpatialIndex
from OpenGL import GL
from OpenGL.error import GLError
from pxr import Gf, Glf, Sdf, Tf, Usd
from pxr.Usdviewq.qt import QGLFormat, QGLWidget, QtCore, QtGui, QtWidgets

from ._schema_pin_names import (
    get_display_pin_name_for_property_name,
    resolve_property_name_for_display_pin,
)

# The stage reconciliation algorithms live in graphview/stage_sync; the two
# path helpers are re-exported here for the existing callers and tests that
# import them from this module.
from .graphview import editing as _editing
from .graphview.editing import (
    _make_value_edit, _make_api_schema_undo,
    _make_connection_edit, _make_reconnect_edits,
    _make_prim_activation_undo, _make_rename_undo, _push_undo_command,
)
from .graphview import intents as _intents
from .graphview import interaction as _interaction
from .graphview import render as _render
from .graphview import render_nodes as _render_nodes
from .graphview import render_text as _render_text
from .graphview import resources as _resources
from .graphview import selection as _selection
from .graphview import stage_sync
from .graphview.stage_sync import _isPathRelated, _nodeIdPrimPath
from .graphview import model as _graphview_model
from .constants import MAX_RENDER_DEPTH, node_content_depth
from .linkRenderer import computeLinkCurveBounds, LinkRenderer, sampleLinkCurve
from .nodeFactory import NodeFactory
from .nodeGraph import (
    createNodeRenderer,
    NodeGraph,
    subscribe_edit_target_warning,
    warn_if_non_persistent_edit_target,
)
from .nodeGraphBlueprint import NodeGraphBlueprint
from .nodeGraphJson import NodeGraphJson
from .nodeLayout import layout_graph
from .nodeLibs.registry import NodeLibraryRegistry
from .noodlesConfig import NoodlesConfig
from .noodlesPreferences import NoodlesPreferences
from . import noodlesValues
from .pinUtils import (
    arrowhead_points,
    collect_links_for_prim,
    get_relationship_link_metadata,
    is_hidden_namespace_pin,
    is_multi_target_relationship_input_pin,
    is_relationship_input_pin,
    is_relationship_pin,
    split_direction_hint,
)
from .textRenderer import TextRenderer
from .usdNoticeHandler import UsdNoticeHandler
from .utils import M, MenuBuilder
from .widgets import valueCell
from .widgets.selectableList import SelectableList, SelectableListItem
from .widgets.textInputWidget import TextInputWidget
from .widgets import (
    GroupStickerRenderer,
    HotkeyPopup,
    Minimap,
    NavigationHotkeyPopup,
    NodeCreationHotbox,
    StatusOverlay,
)


_guardsQtBoundary = _interaction.guards_qt_boundary


# One value cell under the cursor. ``rect`` is the SUB-cell (one component of
# a vector), which is the unit everything downstream edits.
_ValueHit = namedtuple("_ValueHit", "nodeId primPath propertyName comp rect row")


def _frameOf(value):
    """A ``Usd.TimeCode`` from whatever usdview handed us.

    usdview's ``frame`` is a TimeCode on some paths and a bare float on
    others; both have to end up as one thing here. Copied from
    ``rigExecUsdview/avarEditorUI.py:_FrameOf`` so the two agree.
    """
    if isinstance(value, Usd.TimeCode):
        return value
    try:
        return Usd.TimeCode(float(value))
    except (TypeError, ValueError):
        return Usd.TimeCode.Default()


def _makeStageSync(view):
    """This editor's stage-lifecycle service, bound to *view*.

    Policy lives in graphview/stage_sync, which imports no Qt or GL; the
    widget-owned post and warn resolve this module's ``QtCore`` and ``Tf``
    at call time, so the StageReload patch points keep intercepting them.
    """
    return stage_sync.StageSync(
        read=lambda: stage_sync.readStageContext(view),
        notice=lambda: stage_sync.readNoticeContext(view),
        apply=lambda delta: stage_sync.applyStageDelta(view, delta),
        post=lambda callback: QtCore.QTimer.singleShot(0, callback),
        listen=lambda receiver: UsdNoticeHandler(receiver),
        warn=lambda message: Tf.Warn(message),
    )






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


class GraphView(_interaction.GraphWidgetBase):
    # The stage-lifecycle service, built in __init__. Views that never ran
    # it (test doubles) see None; the reload entry points below run the
    # same policy for those through a transient service.
    _stageSync = None

    _worldProjCacheKey = None
    _worldProjCache = None
    _screenProjCacheKey = None
    _screenProjCache = None

    @property
    def nodeGraph(self):
        return self._nodeGraph

    @nodeGraph.setter
    def nodeGraph(self, value):
        self._nodeGraph = value
        if hasattr(self, "_selection"):
            self._selection.update("replace", (), ())
            self._marqueeBase = _selection.Selection()
            self.marqueeActive = False
        self.nodeGraphReplaced.emit()

    def __init__(self, parent=None, settingsObject=None):
        super().__init__(parent)

        # Initialize backing field before any property setter touches it.
        self._nodeGraph = None
        self._undoHistory = None
        self._commandRegistry = self._makeCommandRegistry()
        self._editor = _editing.editor_for_view(self, self._commandRegistry)
        self._assetsPath = None

        glFormat = QGLFormat()
        glFormat.setVersion(3, 2)
        glFormat.setProfile(QGLFormat.OpenGLContextProfile.CoreProfile)
        glFormat.setSamples(4)
        # needed on osx
        QGLFormat.setDefaultFormat(glFormat)

        self.setFormat(glFormat)

        from .noodlesConfig import NoodlesConfig

        # Initialize configuration system if settings provided
        if settingsObject:
            NoodlesConfig.initialize(settingsObject, graphView=self)

        self.zoom = 1.0
        self.panX = 0.0
        self.panY = 0.0
        self.lastMousePos = QtCore.QPoint()

        self.nodeGraph = NodeGraph()

        # Show a toast popup when an authoring site lands an opinion on a
        # non-persistent layer (session / anonymous). Subscriber is module-
        # level so widgets/models without a back-reference to this view can
        # still surface the warning. Removed in closeEvent below.
        subscribe_edit_target_warning(self._showWarningPopup)

        self._currentPrimPath = None

        self.drawLinks = True
        self.drawNodes = True
        self.drawText = True

        self.animator = Animator()

        self.linkDimming = self.animator.addProperty()
        self.linkDimming.speed = 1.0
        self.linkDimming.target = 1.0
        self.linkDimming.current = 1.0

        self.initialized = False

        self.shaderLibrary = None
        self.linkRenderer = LinkRenderer()
        self.textRenderer = TextRenderer()
        self.statusOverlay = StatusOverlay()

        # Fallback renderer for nodes without a custom renderer, and the source
        # of the global port-hit extent / link-circle radius. Built through the
        # same factory as node renderers so it honors NoodlesConfig too (default
        # style; equals the struct defaults on an unmodified config).
        self.defaultRenderer = createNodeRenderer("default")

        self.draggingNodes = False
        self.spacePressed = False
        self.ctrlPressedOnMouseDown = False
        self.mousePosOnMouseDown = QtCore.QPoint()
        self.nodeIdUnderCursor = ""

        self.marqueeActive = False
        self.marqueeStart = QtCore.QPointF()
        self.marqueeCurrent = QtCore.QPointF()

        self.textChanged = True

        # Get pan animation speed from config
        panSpeed = NoodlesConfig.get("panAnimationSpeed", 18.0)
        self._panXAnim = self.animator.addProperty()
        self._panXAnim.speed = panSpeed
        self._panXAnim.target = 0.0
        self._panXAnim.current = 0.0

        self._panYAnim = self.animator.addProperty()
        self._panYAnim.speed = panSpeed
        self._panYAnim.target = 0.0
        self._panYAnim.current = 0.0

        self._panAnimating = False

        self.hotkeyPopup = HotkeyPopup(self.animator)
        self.hotkeyPopup.set_parent_widget(self)

        self.navigationHotkeyPopup = NavigationHotkeyPopup(self.animator)

        self.frameTimer = QtCore.QElapsedTimer()

        self._usdviewApi = None

        self.minimap = Minimap(self)

        # Node creation hotbox with library system
        # Discover node libraries via plugInfo.json registry
        self._nodeLibraries = NodeLibraryRegistry().discover()

        # Restore persisted library enable/disable state
        enabled_names = NoodlesConfig.get("enabledLibraries", None)
        if enabled_names is not None:
            for lib in self._nodeLibraries:
                lib.enabled = lib.get_name() in enabled_names

        # Initialize node factory for creating nodes from USD prims
        self._nodeFactory = NodeFactory(self._nodeLibraries)

        self.nodeCreationHotbox = NodeCreationHotbox(self.animator)
        self.nodeCreationHotbox.set_parent_widget(self)
        self._updateNodeCreationHotbox()  # Configure from libraries
        self.nodeCreationHotbox.on_create = self._createNodeFromHotbox

        self.setAttribute(QtCore.Qt.WA_Hover, True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMouseTracking(True)
        self.setMinimumSize(400, 300)

        # Canvas click resets usdview prim selection to '/'; cache the last real selection.
        self._lastPrimTreeSelection = []

        self._previousActiveWindow = None

        self._linkSelectionMode = LinkSelectionMode.NODES_ONLY
        self._selection = _selection.Selection()
        self._marqueeBase = _selection.Selection()
        self._marqueeMode = "replace"
        self._primTreeAdapter = _selection.make_prim_tree_adapter(self)

        # Inline prim rename. Non-empty _renamingNodeId is the whole "a rename
        # is in progress" flag: it gates key routing, painting, and what a
        # mouse press means.
        self._renamingNodeId = ""
        self._renameInput = TextInputWidget()
        self._renameInput.placeholder = "prim name"

        # Inline attribute value editing. Three mutually exclusive modes, each
        # gated by its own "is this in progress" state, exactly as the rename
        # above is gated by a non-empty _renamingNodeId:
        #   _valueEditTarget  (nodeId, propertyName, componentIndex) -- typing
        #   _mungState        dict                                   -- dragging
        #   _tokenPopup       dict                                   -- picking
        # The editor is a GL-drawn TextInputWidget, not a QLineEdit: this is a
        # QGLWidget inside usdview's dock, and a child QWidget stacked over a
        # GL surface has real z-order and repaint problems on Windows (the
        # same class of problem _onStageReplaced documents below).
        self._valueEditTarget = None
        self._valueEditInput = TextInputWidget()
        self._valueEditInput.placeholder = "value"
        # The first printable key REPLACES, emulating the select-all a real
        # line edit does on focus (volumeWeightUI.py:1052 does the same).
        self._valueEditFresh = True

        self._valueEditError = False
        self._mungState = None
        self._mungActive = False
        self._tokenPopup = None
        self._tokenPopupList = SelectableList(max_visible_items=8)
        # usdNoodles had no notion of time before this feature. The frame is
        # tracked only so a value edit on an already-animated attribute lands
        # on the frame the user is looking at.
        self._currentTimeCode = Usd.TimeCode.Default()
        self._valueCursorSet = None
        # (nodeId, propertyName, componentIndex) under the pointer, so the
        # hovered component lights up and the cursor matches what a press
        # would do there.
        self._valueHover = None
        # A short post-commit fill flash, and the error tint a rejected value
        # leaves on the outline: (nodeId, prop, comp, endTimeSeconds, kind).
        self._valueFlash = None
        # Seeded from the schema defaults here and re-read in
        # _initCachedSettings: _initContextMenu runs FIRST and builds its
        # checkboxes from these, so they have to exist before then.
        self._cachedShowAttributeValues = NoodlesConfig.get(
            "showAttributeValues", True
        )
        self._cachedValueWriteMode = NoodlesConfig.get("valueWriteMode", "auto")

        self._linkUnderCursor = -1
        # Gate for the per-frame link snapshot sync (see _syncLinksIfNeeded):
        # _linksNeedSync is set whenever a rendered link field (selection,
        # highlight, endpoints, membership) changes in place; _lastSyncedLinkHover
        # tracks the hovered index already mirrored to C++. Both start "dirty" so
        # the first paint mirrors the links.
        self._linksNeedSync = True
        self._lastSyncedLinkHover = -1
        # Gate for the C++ port-highlight rebuild (see _renderPortHighlightsCpp):
        # set when connections change in place; hover / drag / content are derived
        # live. Starts dirty so the first paint builds the highlight geometry.
        self._portHighlightsDirty = True
        # Watermarks for _reconcileMovedNodes: the global move generation last
        # reconciled (O(1) skip when nothing moved) and the per-node position
        # version already applied to the spatial index / links / highlights.
        self._reconcilePositionGen = 0
        self._reconciledPositionVersions = {}

        # Port hit radius multiplier default (read live from config in _updateHoveredPort)
        self._portHitRadiusMultiplierDefault = 1.5

        # Port hover state for highlighting
        self._hoveredPort = None  # tuple: (nodeId, portName, isOutput) or None
        self._hoveredPortColor = (1.0, 1.0, 0.0, 1.0)  # Yellow

        # Property tooltip state
        self._tooltipProperty = None  # (nodeId, pinName) or None
        self._tooltipScreenPos = None  # QPoint for tooltip position
        self._tooltipTimer = QtCore.QTimer()
        self._tooltipTimer.setSingleShot(True)
        self._tooltipTimer.setInterval(500)
        self._tooltipTimer.timeout.connect(self._showPropertyTooltip)

        # Link creation/dragging state
        self._draggingLink = False
        self._dragLinkSourceNode = None  # nodeId
        self._dragLinkSourcePort = None  # portName
        self._dragLinkSourceIsOutput = None  # bool
        self._dragLinkTempLink = None  # Reserved temporary LinkData (initialized below)
        self._dragLinkCursorPos = None  # Current cursor world position
        self._dragLinkValidTarget = False  # Is current hover a valid connection?
        self._dragLinkSyntheticTarget = None  # nodeId, portName, isOutput
        self._dragLinkResolvedTarget = None  # nodeId, portName, isOutput, propertyName

        # Initialize temporary link object (not added to self.links array)
        self._dragLinkTempLink = LinkData()
        self._dragLinkTempLink.sourceNodeId = ""
        self._dragLinkTempLink.targetNodeId = ""
        self._dragLinkTempLink.sourcePort = ""
        self._dragLinkTempLink.targetPort = ""
        self._dragLinkTempLink.sourcePropertyName = ""
        self._dragLinkTempLink.targetPropertyName = ""
        self._dragLinkTempLink.is_relationship_link = False

        # Link reconnection state (click-on-link to re-drag)
        self._reconnectingLink = False
        self._reconnectOldSourceNodeId = None
        self._reconnectOldSourcePort = None
        self._reconnectOldTargetNodeId = None
        self._reconnectOldTargetPort = None
        self._reconnectOldSourcePropertyName = ""
        self._reconnectOldTargetPropertyName = ""
        self._reconnectCapture = None

        # Frame in prim tree option - when enabled, selecting nodes will scroll
        # the prim tree view to show the selected prim
        self._frameInPrimTreeOnSelect = NoodlesConfig.get(
            "frameInPrimTreeOnSelect", True
        )

        # Track last jump direction for navigation (default to "target" for forward traversal)
        # "target" means: after jumping, select outputs to continue forward
        # "source" means: after jumping, select inputs to continue backward
        self._lastJumpDirection = "target"

        # Track the last selected node for link navigation
        # This allows pressing 1,2,3 etc. to switch links from the same node
        self._lastSelectedNode = None
        # Track the last pin index and cycle index for cycling through multiple links on same pin
        self._lastPinIndex = -1
        self._lastLinkCycleIndex = 0





        self._lastInputLinkIndex = -1
        self._lastOutputLinkIndex = -1

        # Track the last dangling link that was selected via navigation
        # This allows detecting when user tries to "follow" a dangling link a second time
        self._lastDanglingLinkIndex = -1
        self._danglingLinkFollowCount = 0

        self._contextMenu = None
        self._contextMenuActions = {}
        self._initContextMenu()

        self._setupShortcuts()

        # Preferences dialog instance (created on first open)
        self._preferencesDlg = None

        # Cached settings (updated on settings change, not every frame)
        self._initCachedSettings()

        # Connect to settings changed signal for live updates
        settings = NoodlesConfig.settings()
        if settings:
            settings.signalSettingChanged.connect(self._onSettingsChanged)

        # Rendering
        self.fontAtlas = None

        # Profiler
        self._profiler = RenderProfiler(60)
        self._profiler.enabled = NoodlesConfig.get("enableRenderProfiling", False)
        self._profilingFrameCount = 0

        # Rendering managers - encapsulate rendering logic and data
        self._nodeRenderManager = NodeRenderManager()
        # Shared node-local coordinate frame: the text and node-quad paths sample
        # one transform texture, so a drag moves both with a single update.
        self._nodeTransformFrame = NodeTransformFrame()
        # Cached C++ RenderConfig shared by the node-quad and link paths; rebuilt
        # lazily after settings change (invalidated to None in _initCachedSettings).
        self._cachedRenderConfig = None
        self._groupStickerRenderer = GroupStickerRenderer()

        # When True, _clearRenderCache() defers its GL-side work to
        # _finishStageReload() instead of running it inline.  Used during
        # _onStageReplaced() to avoid issuing glDelete*/glGen* against the
        # NVIDIA Windows driver while the stageView's renderer is mid-teardown.
        self._deferRenderCacheClear = False
        self._renderCacheClearPending = False

        # Spatial acceleration structure for selection operations
        self._spatialIndex = SpatialIndex(maxDepth=4, maxItems=20)

        # Z-order counter for node layering (bring-to-front on selection)
        self._nextZOrder = 0

        # The stage-lifecycle service owns the notice receiver; the handler
        # object stays on the view because the loaders register through it
        # and the authoring sites suppress through it.
        self._stageSync = _makeStageSync(self)
        self._noticeHandler = self._stageSync.handler()

        # Register Ctrl+S shortcut for saving using QAction
        self._saveAction = QtGui.QAction("Save Stage", self)
        self._saveAction.setShortcut(QtGui.QKeySequence.Save)
        self._saveAction.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
        self._saveAction.triggered.connect(self._saveStage)
        self.addAction(self._saveAction)

    _selectedNodes = property(_selection.selected_node_ids)
    _selectedLinks = property(_selection.selected_link_indices)
    _syncingSelection = property(_selection.syncing_selection)

    @property
    def nodes(self):
        return self.nodeGraph.nodes

    @property
    def links(self):
        return self.nodeGraph.links

    @property
    def groupStickers(self):
        return self.nodeGraph.groupStickers

    @property
    def linksChanged(self):
        return self.nodeGraph.linksChanged

    @linksChanged.setter
    def linksChanged(self, value):
        self.nodeGraph.linksChanged = value

    # Warning toasts (non-persistent edit target, save errors) linger 1s longer
    # than navigation popups so users have time to read them.
    _WARNING_POPUP_DURATION_MS = 1400

    def setUsdviewApi(self, usdviewApi):
        self._usdviewApi = usdviewApi

        # Connect to usdview selection changes
        if usdviewApi and hasattr(usdviewApi, "dataModel"):
            dataModel = usdviewApi.dataModel
            if dataModel and hasattr(dataModel, "selection"):
                # Listen for selection changes in the primtree
                dataModel.selection.signalPrimSelectionChanged.connect(
                    self._onPrimSelectionChanged
                )
                # Listen for property selection changes
                dataModel.selection.signalPropSelectionChanged.connect(
                    self._onPropSelectionChanged
                )
            if hasattr(dataModel, "signalStageReplaced"):
                dataModel.signalStageReplaced.connect(self._onStageReplaced)
            if hasattr(dataModel, "currentFrameChanged"):
                dataModel.currentFrameChanged.connect(self._onFrameChanged)

        self._currentTimeCode = _frameOf(
            getattr(usdviewApi, "frame", Usd.TimeCode.Default())
        )

    def _onFrameChanged(self, frame):
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
            if newTime == self._currentTimeCode:
                return
            self._currentTimeCode = newTime
            # Only the rows that can change with time need re-reading, but the
            # per-node dict is cheap and dropping it whole cannot go stale.
            for node in self.nodes.values():
                if hasattr(node, "invalidate_value_row"):
                    node.invalidate_value_row()
            self.update()
        except Exception:
            Tf.Warn(
                "GraphView._onFrameChanged raised; the frame was ignored:\n%s"
                % traceback.format_exc()
            )

    def _onStageReplaced(self):
        sync = getattr(self, "_stageSync", None)
        if not isinstance(sync, stage_sync.StageSync):
            # Test doubles never ran __init__: same policy, transient service.
            sync = _makeStageSync(self)
            self._stageSync = sync
        if sync.isClosed():
            # usdview keeps this signal connected and a closed widget is not
            # a destroyed one: decide before the api or graph is read.
            return
        frame = _frameOf(
            getattr(self._usdviewApi, "frame", Usd.TimeCode.Default())
            if self._usdviewApi
            else Usd.TimeCode.Default()
        )
        stage = self._usdviewApi.stage if self._usdviewApi else None
        sync.replace(
            stage, stage_sync.stageLoadMode(self), self._currentPrimPath, frame
        )

    def _finishStageReload(self, epoch=None):
        sync = getattr(self, "_stageSync", None)
        if not isinstance(sync, stage_sync.StageSync):
            # Test doubles never ran __init__: run the same policy through a
            # transient service rather than a second copy of it.
            sync = _makeStageSync(self)
            self._stageSync = sync
        sync.finish(sync.epoch() if epoch is None else epoch)

    def _graphScopeParentPath(self):
        return stage_sync.graphScopeParentPath(self)

    def _validateNodesAgainstStage(self):
        return stage_sync.validateNodesAgainstStage(self)

    def _adoptPrimsNewToScope(self, resyncedPaths):
        return stage_sync.adoptPrimsNewToScope(self, resyncedPaths)

    def _remapMovedNodes(self, movedPaths):
        return stage_sync.remapMovedNodes(self, movedPaths)

    def _handleUsdChanges(self, resyncedPaths, infoChangedPaths, movedPaths=None):
        stage_sync.applyUsdChanges(
            self, resyncedPaths, infoChangedPaths, movedPaths
        )

    def _onSettingsChanged(self):
        """Called when settings change via preferences dialog.
        Updates cached values so we don't read from config every frame."""
        self._initCachedSettings()
        # Update shortcuts as well
        self.updateShortcuts()
        # Update profiler state
        self._profiler.enabled = self._cachedEnableRenderProfiling
        if self._profiler.enabled:
            self._profiler.reset()
        # Update pan animation speed
        self._panXAnim.speed = self._cachedPanAnimationSpeed
        self._panYAnim.speed = self._cachedPanAnimationSpeed
        # Recalculate all node sizes + layouts (font sizes / margins may have
        # changed, which affects the cached layout metrics every renderer reads).
        if self.nodeGraph and self.fontAtlas:
            for node in self.nodes.values():
                self.nodeGraph._calculateNodeSize(
                    node,
                    self.textRenderer.calculateTextWidth
                    if self.textRenderer
                    else lambda t, s: s * 5,
                    self.fontAtlas,
                )
        self.textChanged = True
        self._clearRenderCache()
        # Trigger redraw to apply new settings immediately
        self.update()

    def _initCachedSettings(self):
        """Initialize/update all cached settings from NoodlesConfig.
        Called on init and when settings change."""
        # Drop the cached RenderConfig so the next paint rebuilds it from the new
        # settings.
        self._cachedRenderConfig = None
        # Link colors
        self._cachedLinkHoveredColor = NoodlesConfig.get(
            "linkHoveredColor", [1.0, 1.0, 0.0, 1.0]
        )
        self._cachedLinkSelectedColor = NoodlesConfig.get(
            "linkSelectedColor", [1.0, 1.0, 0.3, 1.0]
        )
        self._cachedLinkActiveColor = NoodlesConfig.get(
            "linkActiveColor", [0.0, 0.8, 1.0, 1.0]
        )
        self._cachedLinkHighlightedColor = NoodlesConfig.get(
            "linkHighlightedColor", [0.31, 0.78, 0.47, 1.0]
        )
        self._cachedLinkAttributeBaseColor = NoodlesConfig.get(
            "linkAttributeBaseColor", [0.31, 0.55, 0.86, 1.0]
        )
        self._cachedLinkRelationshipBaseColor = NoodlesConfig.get(
            "linkRelationshipBaseColor", [0.78, 0.24, 0.24, 1.0]
        )
        self._cachedNodeSelectedStrokeColor = NoodlesConfig.get(
            "nodeSelectedStrokeColor", [1.0, 1.0, 0.3, 1.0]
        )
        # Animation
        self._cachedPanAnimationSpeed = NoodlesConfig.get("panAnimationSpeed", 18.0)
        # Hit detection. Link picking matches the drawn ribbon width (its visible
        # 0.5 SDF edge); see linkRenderer.findLinkUnderCursor.
        self._cachedLinkLineWidth = NoodlesConfig.get("linkLineWidth", 12.0)
        self._cachedPortHitRadiusMultiplier = NoodlesConfig.get(
            "portHitRadiusMultiplier", 1.5
        )
        # Behavior
        self._cachedFrameInPrimTreeOnSelect = NoodlesConfig.get(
            "frameInPrimTreeOnSelect", True
        )
        # Profiling
        self._cachedEnableRenderProfiling = NoodlesConfig.get(
            "enableRenderProfiling", False
        )
        self._cachedProfilingPrintInterval = NoodlesConfig.get(
            "profilingPrintInterval", 60
        )
        # Node layout
        self._cachedNodeMarginV = NoodlesConfig.get("nodeMarginV", 18.0)
        self._cachedNodePinFontSize = NoodlesConfig.get("nodePinFontSize", 18.0)
        # Port colors
        self._cachedPortRingThickness = NoodlesConfig.get("portRingThickness", 2.0)
        self._cachedPortConnectedFillColor = NoodlesConfig.get(
            "portConnectedFillColor", [0.7, 0.8, 0.0, 1.0]
        )
        self._cachedPortConnectedRingColor = NoodlesConfig.get(
            "portConnectedRingColor", [1.0, 1.0, 1.0, 0.0]
        )
        self._cachedPortDisconnectedFillColor = NoodlesConfig.get(
            "portDisconnectedFillColor", [0.235, 0.235, 0.235, 1.0]
        )
        self._cachedPortDisconnectedRingColor = NoodlesConfig.get(
            "portDisconnectedRingColor", [0.7, 0.8, 0.0, 1.0]
        )
        self._cachedPortHoverColor = NoodlesConfig.get(
            "portHoverColor", [0.4, 0.7, 1.0, 1.0]
        )
        # Minimap overlay visibility. Off skips both the per-frame minimap draw
        # (which uploads a rect per node) and its mouse hit-testing.
        self._cachedShowMinimap = NoodlesConfig.get("showMinimap", True)
        # Inline attribute values
        self._cachedShowAttributeValues = NoodlesConfig.get(
            "showAttributeValues", True
        )
        self._cachedValueMinPixelHeight = NoodlesConfig.get(
            "valueMinPixelHeight", 9.0
        )
        self._cachedMungDragThresholdPx = NoodlesConfig.get(
            "mungDragThresholdPx", 3.0
        )
        self._cachedValueWriteMode = NoodlesConfig.get("valueWriteMode", "auto")
        self._cachedValueFontScale = NoodlesConfig.get("valueFontScale", 0.9)
        # Every value-cell colour is derived from the node theme, so a change
        # to the node background or the selection accent carries through to
        # the cells without anyone maintaining a second palette.
        self._cachedValueTheme = valueCell.build_theme(
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
        self._valueDigitAdvanceCache = {}
        # Paint-pass caches, recreated here (and only here) so a font change
        # cannot leave stale glyphs behind: fitted value texts keyed by
        # (node, property, components, width, font), single-glyph vertices
        # keyed by (character, size), axis-label widths, and text-band
        # geometry. Every key carries what the entry depends on, so a value,
        # layout or font change misses and recomputes instead of going stale.
        self._valueFitCache = {}
        self._valueGlyphCache = {}
        self._valueAxisWidthCache = {}
        self._valueTextBandCache = {}

    def _toggleDrawLinks(self, checked):
        self.drawLinks = checked
        self.update()

    def _toggleDrawNodes(self, checked):
        self.drawNodes = checked
        self.update()

    def _toggleDrawText(self, checked):
        self.drawText = checked
        self.update()

    def _toggleShowAttributeValues(self, checked):
        """Turn the inline value cells on or off.

        Shipped alongside the feature on purpose: the cells WIDEN nodes, so a
        graph laid out before they existed gets denser, and the user needs one
        switch rather than an argument.
        """
        self._cachedShowAttributeValues = bool(checked)
        NoodlesConfig.set("showAttributeValues", bool(checked))
        self._clearValueInteractionState()
        # The width reserve is decided by the layout pass, so this is a
        # content change, not a repaint.
        self.textChanged = True
        self.linksChanged = True
        self._nodeRenderManager.invalidateAll()
        self.update()

    def _setValueWriteMode(self, mode):
        """Where inline value edits land: auto / default / animation.

        Same three tokens the avar editor and the gizmo toolbar use
        (gizmoMath.WRITE_DEFAULT / WRITE_ANIMATION), so a user who knows one
        knows this one.
        """
        self._cachedValueWriteMode = mode
        NoodlesConfig.set("valueWriteMode", mode)

    def _toggleLinkDimming(self, checked):
        self.linkDimming.target = 1.0 if checked else 0.0
        self.update()

    def _clearRenderCache(self):
        """Clear the node render manager cache within this widget's GL context.

        makeCurrent()/doneCurrent() ensures that glDeleteBuffers/glDeleteVertexArrays
        operate on this widget's OpenGL context, not the stageView's.  Without the
        context switch the 3D viewport's render state may be corrupted.

        When self._deferRenderCacheClear is True (set by _onStageReplaced),
        only flag the pending clear; _finishStageReload will run the actual
        glDelete* calls after the next event-loop tick.  This avoids issuing
        GL deletes during the NVIDIA Windows non-functional window while the
        stageView's renderer is mid-teardown.
        """
        if self._deferRenderCacheClear:
            self._renderCacheClearPending = True
            return
        self.makeCurrent()
        self._nodeRenderManager.clearCache()
        self.doneCurrent()

    def _loadSampleGraph(self, jsonPath):
        self.nodeGraph = NodeGraphJson()
        self._initUndoTracking(None)
        self.nodeGraph.load(
            jsonPath, self.textRenderer.calculateTextWidth, self.fontAtlas
        )
        self._initUndoTracking(self.nodeGraph.getStage())
        self.linksChanged = True

    def loadBlueprint(self, stage, primPath, useFieldnames=True):
        self._currentPrimPath = primPath

        self.nodeGraph = NodeGraphBlueprint()
        self.nodeGraph.setStage(stage)
        self._initUndoTracking(stage)
        self.nodeGraph.load(
            stage,
            primPath,
            self.textRenderer.calculateTextWidth,
            self.fontAtlas,
            useFieldnames,
            nodeFactory=self._nodeFactory,  # Pass the factory
        )

        self._seedNodeZOrders()
        self._gridPlaceNodes(list(self.nodes.values()))

        self._resetRenderCachesAfterBulkMove()

        self._noticeHandler.register(stage)

        if self.nodes:
            self.frameAll()

    def loadContainer(self, stage, primPath, useFieldnames=True):
        from .nodeGraphBpContainer import NodeGraphBpContainer

        self._currentPrimPath = primPath

        self.nodeGraph = NodeGraphBpContainer()
        self.nodeGraph.setStage(stage)
        self._initUndoTracking(stage)
        self.nodeGraph.load(
            stage,
            primPath,
            self.textRenderer.calculateTextWidth,
            self.fontAtlas,
            useFieldnames,
            nodeFactory=self._nodeFactory,  # Pass the factory
        )

        self._seedNodeZOrders()
        self._gridPlaceNodes(list(self.nodes.values()))

        self._resetRenderCachesAfterBulkMove()

        # Register for USD change notices on this stage
        self._noticeHandler.register(stage)

        if self.nodes:
            self.frameAll()

    def loadStage(self, stage):
        """Open an EMPTY graph on *stage*: no prims graphed at all.

        The canvas is the user's, not the loader's: prims arrive via 'A'
        (``addNodesFromPrimTreeSelection``), node creation, and the
        notice-driven adoption of newly appearing scope children -- never
        from an opening scan. ``NodeGraphStage.load`` keeps its
        root-children scan as the explicit loader primitive; the view
        simply does not invoke it here.
        """
        from .nodeGraphStage import NodeGraphStage

        self._currentPrimPath = None

        self.nodeGraph = NodeGraphStage()
        self.nodeGraph.setStage(stage)
        self._initUndoTracking(stage)
        self.nodeGraph.syncSelectionToPrimTree = True
        self.nodeGraph.linksChanged = True

        self._seedNodeZOrders()

        self._resetRenderCachesAfterBulkMove()
        self._noticeHandler.register(stage)

    # Auto-layout grid for nodes that have no position of their own, in
    # logical/world coordinates (independent of window size): ~30 columns per
    # row, cell sized to the batch's largest node plus a gap so content-sized
    # nodes keep clear spacing.
    _AUTO_GRID_COLUMNS = 30
    _AUTO_GRID_GAP = 150.0

    def _visibleWorldBounds(self):
        """What the camera currently shows, as a world-space rect.

        ``(panX, panY)`` is its top-left corner; the extent is the widget size
        divided by the zoom.
        """
        zoom = self.zoom if self.zoom else 1.0
        return Gf.Range2d(
            Gf.Vec2d(self.panX, self.panY),
            Gf.Vec2d(self.panX + self.width() / zoom, self.panY + self.height() / zoom),
        )

    def _isVisible(self, bounds):
        """Whether any part of *bounds* falls inside the current view."""
        if bounds.IsEmpty():
            return False
        return not Gf.Range2d.GetIntersection(
            self._visibleWorldBounds(), bounds
        ).IsEmpty()

    def _viewportAnchor(self):
        """Top-left of what the camera currently shows, in world space, inset.

        The inset is a fraction of the visible extent rather than a fixed
        distance so it reads as the same margin whatever the zoom.
        """
        visible = self._visibleWorldBounds()
        size = Gf.Vec2d(visible.GetSize())
        return (
            visible.GetMin()[0] + size[0] * 0.05,
            visible.GetMin()[1] + size[1] * 0.05,
        )

    def _resetRenderCachesAfterBulkMove(self, rebuild_links=True):
        """Invalidate every baked vertex cache + the drag-transform frame and flag
        text for rebuild, after nodes are repositioned or (re)loaded wholesale.

        ``rebuild_links`` controls link refresh. The load paths pass True
        (connections are new, so a full rebuild is required). A pure move
        (auto-layout) passes False and refreshes only the moved links' endpoints
        afterward via ``_updateLinksForMovedNodes`` -- connections are unchanged,
        so re-reading every prim's connections from USD would be wasted work.
        """
        self._clearRenderCache()
        self.textRenderer.resetPositionCaches()
        self._cppIconRenderer.resetPositionCaches()
        self._nodeRenderManager.resetNodeQuadCaches()
        self._nodeTransformFrame.reset()
        self.textChanged = True
        if rebuild_links:
            self.linksChanged = True

    @property
    def currentPrimPath(self):
        return self._currentPrimPath

    @staticmethod
    def _valueRowOnlyChange(node, propertyName):
        """Whether a property change is *only* a value an inline cell shows.

        True means the view can answer it by re-reading one node's value rows
        -- no cache invalidation, no relayout, no link rebuild. See the call
        site in ``_handleUsdChanges`` for why that distinction is the single
        most important performance decision in the value feature.

        A connection-driven attribute is excluded: its links are the thing
        that changed shape, and the older branches below know how to rebuild
        them.
        """
        if not propertyName:
            return False
        has_value_row = getattr(node, "has_value_row", None)
        if has_value_row is None:
            return False
        try:
            if not has_value_row(propertyName):
                return False
            prim = getattr(node, "_prim", None)
            if prim is not None and prim.IsValid():
                attr = prim.GetAttribute(propertyName)
                if attr and attr.IsValid() and attr.HasAuthoredConnections():
                    return False
        except Exception:
            return False
        return True

    @staticmethod
    def _isPinPropertyChange(propertyName):
        """True when a property change should rebuild links.

        Covers any direction-hint pin (``in:``/``input:``/``inputs:`` and
        ``out:``/``output:``/``outputs:``), relationship pins, and any other
        namespaced attribute shown as a pin (e.g. ``rig1:space``) — but not
        hidden ``ui:`` editor metadata. Bare attrs are left to the existing
        value-change handling.
        """
        if split_direction_hint(propertyName)[0] is not None:
            return True
        if is_relationship_pin(propertyName):
            return True
        if ":" not in propertyName:
            return False
        hide_ui = bool(NoodlesConfig.get("hideUiNamespacePins", True))
        return not is_hidden_namespace_pin(propertyName, hide_ui=hide_ui)

    def _paintNodeCreationHotbox(self):
        """Render the node creation hotbox overlay."""
        projection = self._screenSpaceProjectionMatrix()
        self.nodeCreationHotbox.render(
            self.textRenderer,
            NodeVertex,
            self._drawNodeVertices,
            projection,
            self.width(),
            self.height(),
            MAX_RENDER_DEPTH,
        )

    def _printProfilingStatsIfNeeded(self):
        """Print profiling stats every N frames when profiling is enabled."""
        if self._profiler.enabled:
            self._profilingFrameCount += 1
            print_interval = self._cachedProfilingPrintInterval
            if self._profilingFrameCount >= print_interval:
                self._profiler.printStats()
                self._profilingFrameCount = 0
                self._profiler.reset()

    @_guardsQtBoundary
    def mouseDoubleClickEvent(self, event):
        """Handle double-click events.

        Double-clicking on a dangling link or on a connected port whose
        connected node is missing adds the missing node to the graph.
        Double-clicking a non-dangling relationship link jumps to the
        endpoint farthest from the cursor so users can follow links to
        connected nodes that lie outside the current viewport.
        """
        if event.button() != QtCore.Qt.LeftButton:
            return
        if _interaction.double_click_gesture(self, event):
            return
        # Default behavior for double-click on non-dangling elements
        super().mouseDoubleClickEvent(event)

    @_guardsQtBoundary
    def mousePressEvent(self, event):
        if _interaction.press_intercept_overlays(self, event):
            return

        if event.buttons() & QtCore.Qt.LeftButton and not self.spacePressed:
            if _interaction.press_title_caret_toggle(self, event):
                return
            if _interaction.press_group_fold_toggle(self, event):
                return
            if _interaction.press_value_cell(self, event):
                return
            if _interaction.press_hovered_port_link_start(self, event):
                return
            if _interaction.press_arrowhead_reconnect(self, event):
                return
            if _interaction.press_row_edge_link_start(self, event):
                return
            _interaction.press_node_or_empty(self, event)

        self.update()

    @_guardsQtBoundary
    def mouseMoveEvent(self, event):
        if _interaction.move_intercept_overlays(self, event):
            return

        # During an interactive pan or pan-zoom drag (middle-button, or
        # space + left-button), skip all hover/hit-testing entirely. The cursor
        # is not hovering content during a viewport drag, and these per-move
        # scans (node-under-cursor, hovered port, property tooltip, and link
        # hit-testing) are the dominant cost on large scenes -- they are why
        # panning feels heavier than zooming. The pan/zoom itself is handled
        # below; this mirrors the node-drag and link-drag branches, which also
        # bypass hover updates.
        isViewportDrag = bool(event.buttons() & QtCore.Qt.MiddleButton) or (
            self.spacePressed and bool(event.buttons() & QtCore.Qt.LeftButton)
        )

        if _interaction.move_value_drag(self, event):
            return

        _interaction.move_update_hover(self, event, isViewportDrag)
        if _interaction.move_link_drag(self, event):
            return
        if _interaction.move_viewport_drag(self, event):
            return
        if _interaction.move_left_drag(self, event):
            return

        self.lastMousePos = event.position().toPoint()
        self.update()

    @_guardsQtBoundary
    def mouseReleaseEvent(self, event):
        if _interaction.release_intercept_overlays(self, event):
            return
        if _interaction.release_link_drag(self, event):
            return
        if _interaction.release_minimap(self, event):
            return
        _interaction.release_node_drag_and_marquee(self, event)

    def _reloadCurrentGraph(self):
        """Reload the graph from USD after undo/redo.

        Creates a fresh nodeGraph and re-runs the full load path so that
        nodes, links, spatial index, and text are rebuilt cleanly from
        the (now-modified) USD layer state.

        What the loader produces is not the whole graph, though. A loader walks
        the direct children of one scope; the canvas holds whatever the user put
        there, at any depth, via 'A' (``addNodesFromPrimTreeSelection``) or node
        creation under a prim-tree selection -- membership that lives only in
        memory and that no loader can re-derive. Those nodes are re-added
        afterwards, along with the positions they were placed at when nothing
        authored ``ui:nodegraph:node:pos`` to read back. Without that, an
        unrelated undo answers by wiping the canvas: stage graphs open
        empty, so the loader contributes nothing and the re-add loop below
        is the whole membership.
        """
        from .nodeGraphBpContainer import NodeGraphBpContainer
        from .nodeGraphStage import NodeGraphStage

        stage = self.nodeGraph.getStage()
        if not stage:
            self.update()
            return

        # Every node object is about to be rebuilt, so a value edit or drag
        # holding one is already talking about the previous graph.
        self._clearValueInteractionState()

        # Save z-order map before reload so restored nodes keep their
        # front-to-back visual ordering (important for undo of deletes).
        savedZOrders = {nid: node.zOrder for nid, node in self.nodes.items()}
        # Membership and display-only placement: view state with no USD backing,
        # so the reload below is the only thing that could lose it.
        savedNodeIds = list(self.nodes.keys())
        savedDisplayPositions = {
            nid: Gf.Vec2d(node.position)
            for nid, node in self.nodes.items()
            if not self._nodeHasAuthoredPosition(node)
        }

        # Disable notice handler during reload to avoid recursive processing
        self._noticeHandler.setEnabled(False)
        try:
            # Create a fresh nodeGraph of the same type
            if isinstance(self.nodeGraph, NodeGraphBpContainer):
                self.nodeGraph = NodeGraphBpContainer()
                self.nodeGraph.setStage(stage)
                self.nodeGraph.load(
                    stage,
                    self._currentPrimPath,
                    self.textRenderer.calculateTextWidth,
                    self.fontAtlas,
                    nodeFactory=self._nodeFactory,
                )
            elif isinstance(self.nodeGraph, NodeGraphStage):
                # No loader scan: stage graphs open empty, so a reload
                # preserves exactly the curated membership the loop below
                # re-adds rather than re-deriving the loader's idea of the
                # graph (which would graph root prims the user never asked
                # for after the first undo).
                self.nodeGraph = NodeGraphStage()
                self.nodeGraph.setStage(stage)
                self.nodeGraph.syncSelectionToPrimTree = True
            else:
                self.nodeGraph = NodeGraphBlueprint()
                self.nodeGraph.setStage(stage)
                self.nodeGraph.load(
                    stage,
                    self._currentPrimPath,
                    self.textRenderer.calculateTextWidth,
                    self.fontAtlas,
                    nodeFactory=self._nodeFactory,
                )

            # Put back the nodes the user curated onto the canvas that the
            # loader's scope does not cover. A prim that genuinely went away
            # (deleted, or deactivated by an undo of its creation) is skipped --
            # the loaders apply the same rule.
            for nodeId in savedNodeIds:
                if nodeId in self.nodes or "#" in nodeId:
                    continue
                prim = stage.GetPrimAtPath(nodeId)
                if not prim or not prim.IsValid() or not prim.IsActive():
                    continue
                try:
                    self._addPrimAsNode(prim, stage, [])
                except Exception as error:
                    Tf.Warn("Graph reload could not re-add %s: %s" % (nodeId, error))

            # Re-apply placements that were never authored. A node whose
            # position IS authored is left alone -- USD owns that one, and the
            # reload may be undoing a move that authored it.
            for nodeId, position in savedDisplayPositions.items():
                node = self.nodes.get(nodeId)
                if node is not None and not self._nodeHasAuthoredPosition(node):
                    node.setDisplayPosition(position)

            # Restore saved zOrder values for nodes that existed before reload.
            # New nodes (from undo restoring deleted nodes) get the next zOrder.
            # This preserves front-to-back visual ordering across undo/redo.
            maxSaved = max(savedZOrders.values()) if savedZOrders else 0
            self._nextZOrder = maxSaved
            for nid, node in self.nodes.items():
                if nid in savedZOrders:
                    node.zOrder = savedZOrders[nid]
                else:
                    self._nextZOrder += 1
                    node.zOrder = self._nextZOrder

            self.textChanged = True
            self.linksChanged = True
            self._clearRenderCache()
            self._nodeRenderManager.invalidateAll()
            # Reset position caches so text, icons, and node quads pick up new positions
            self.textRenderer.resetPositionCaches()
            self._cppIconRenderer.resetPositionCaches()
            self._nodeRenderManager.resetNodeQuadCaches()
            self._nodeTransformFrame.reset()
            self.update()
        finally:
            self._noticeHandler.setEnabled(True)

    def _isStageViewMidTeardown(self):
        if not self._usdviewApi:
            return False
        try:
            controller = self._usdviewApi._UsdviewApi__appController
            sv = getattr(controller, "_stageView", None)
            return sv is not None and not sv.updatesEnabled()
        except AttributeError:
            return False

    def _reloadLinksFromUSD(self):
        """Reload links from USD after connection changes.

        This rebuilds the links array by reading USD connections directly,
        without reloading the entire graph. Much more efficient than a full reload.
        """
        # Rebuild links directly from USD
        self._rebuildLinks()

        # Update the view
        self.update()

    def frameAll(self):
        """Frame all nodes in the graph, ignoring current selection."""
        if not self.nodes:
            return

        # Calculate bounds of all nodes
        sceneBounds = Gf.Range2d()
        for node in self.nodes.values():
            nodeBounds = Gf.Range2d(node.position, node.position + node.size)
            sceneBounds = Gf.Range2d.GetUnion(sceneBounds, nodeBounds)

        if sceneBounds.IsEmpty():
            return

        boundsSize = Gf.Vec2d(sceneBounds.GetSize())
        if boundsSize[0] == 0 or boundsSize[1] == 0:
            return

        # Frame with a comfortable margin (1.1 = 10% padding)
        self._frameNodeBounds(sceneBounds, margin=1.1)
        self.update()

    def frameScene(self):
        if not self.nodes:
            return

        if self._usdviewApi:
            try:
                dataModel = self._usdviewApi.dataModel
                if dataModel and dataModel.selection:
                    selectedPrims = dataModel.selection.getPrims()
                    if selectedPrims:
                        matchedNodes = []
                        for prim in selectedPrims:
                            primPathStr = str(prim.GetPath())
                            if primPathStr in self.nodes:
                                matchedNodes.append(self.nodes[primPathStr])

                        if matchedNodes:
                            self.clearSelection()
                            for node in matchedNodes:
                                _selection.select_node(self, node.id)
            except Exception:
                pass

        hasSelection = any(node.selected for node in self.nodes.values())
        sceneBounds = Gf.Range2d()
        selectionBounds = Gf.Range2d()

        for node in self.nodes.values():
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

        viewSize = Gf.Vec2d(self.width(), self.height())
        scale = Gf.Vec2d(viewSize[0] / boundsSize[0], viewSize[1] / boundsSize[1])
        self.zoom = min(scale[0], scale[1])

        # Center bounds in view
        scaledView = viewSize / self.zoom
        offset = (scaledView - boundsSize) * 0.5
        self.panX = bounds.GetMin()[0] - offset[0]
        self.panY = bounds.GetMin()[1] - offset[1]
        self.update()

    def frameSelectionWithConnections(self):
        if not self.nodes:
            return

        selectedNodeIds = set()
        for nodeId, node in self.nodes.items():
            if node.selected:
                selectedNodeIds.add(nodeId)

        if not selectedNodeIds:
            self.frameScene()
            return

        connectedNodeIds = set()
        for nodeId in selectedNodeIds:
            connectedNodeIds.update(self.getConnectedNodes(nodeId))

        allNodeIds = selectedNodeIds | connectedNodeIds

        bounds = Gf.Range2d()
        for nodeId in allNodeIds:
            if nodeId not in self.nodes:
                continue
            node = self.nodes[nodeId]
            nodeBounds = Gf.Range2d(node.position, node.position + node.size)
            bounds = Gf.Range2d.GetUnion(bounds, nodeBounds)

        if not bounds.IsEmpty():
            self._frameNodeBounds(bounds)


    def paintLinks(self):
        """Paint all links in the scene."""
        if not self.linkRenderer or not self.links:
            return

        graph = self.nodeGraph
        # Refresh the held C++ link snapshot only when it is stale (hover change
        # or a flagged link-field mutation), then render straight from it — no
        # per-call LinkData marshalling inside the draw. The C++ side filters out
        # dangling links and partitions regular vs relationship (relationshipOnly);
        # the shader insets whole-prim relationship ends for the arrowhead.
        self._syncLinksIfNeeded(graph)

        projection = self._worldSpaceProjectionMatrix()

        # Use cached colors (updated via settings signal, not every frame)
        hoveredColor = self._cachedLinkHoveredColor
        selectedColor = self._cachedLinkSelectedColor
        config = self._renderConfig()

        for drawSelected, dimming in ((True, 0.3), (False, 1.0)):
            self.linkRenderer.renderLinksFromGraph(
                graph,
                projection,
                self.zoom,
                self.panX,
                self.panY,
                self.width(),
                self.height(),
                config,
                relationshipOnly=False,
                dimming=dimming,
                drawSelected=drawSelected,
                hoveredColor=hoveredColor,
                selectedColor=selectedColor,
                baseColor=list(self._cachedLinkAttributeBaseColor[:3]),
                highlightedColor=list(self._cachedLinkHighlightedColor[:3]),
                cacheKey=0,
            )
            self.linkRenderer.renderLinksFromGraph(
                graph,
                projection,
                self.zoom,
                self.panX,
                self.panY,
                self.width(),
                self.height(),
                config,
                relationshipOnly=True,
                dimming=dimming,
                drawSelected=drawSelected,
                hoveredColor=hoveredColor,
                selectedColor=selectedColor,
                baseColor=list(self._cachedLinkRelationshipBaseColor[:3]),
                highlightedColor=list(self._cachedLinkHighlightedColor[:3]),
                cacheKey=1,
            )

        self._renderRelationshipArrowheads(
            projection,
            hoveredColor,
            selectedColor,
        )

    @staticmethod
    def _relationshipArrowheadPoints(
        link,
        arrow_length,
        arrow_width,
    ):
        tip_x = float(link.end[0])
        tip_y = float(link.end[1])
        if GraphView._isPrimTargetRelationshipLink(link):
            # Tip sits on the node top (link.end); the body extends up to the
            # arrowhead base. The wire ribbon is inset to that base by the shader.
            prev_x = tip_x
            prev_y = tip_y - arrow_length
            return arrowhead_points(
                tip_x,
                tip_y,
                prev_x,
                prev_y,
                length=arrow_length,
                width=arrow_width,
            )

        curve_points = sampleLinkCurve(
            (link.start[0], link.start[1]),
            (link.end[0], link.end[1]),
            20,
        )
        if len(curve_points) < 2:
            return None
        return arrowhead_points(
            float(curve_points[-1][0]),
            float(curve_points[-1][1]),
            float(curve_points[-2][0]),
            float(curve_points[-2][1]),
            length=arrow_length,
            width=arrow_width,
        )

    def _renderRelationshipArrowheads(
        self,
        projection,
        hoveredColor,
        selectedColor,
    ):
        arrowhead_vertices = []
        arrow_length = max(10.0 / max(self.zoom, 1e-6), 2.0)
        arrow_width = max(8.0 / max(self.zoom, 1e-6), 2.0)
        for link in self.links:
            if not bool(getattr(link, "is_relationship_link", False)):
                continue
            if getattr(link, "isDangling", False):
                continue
            arrow_points = GraphView._relationshipArrowheadPoints(
                link,
                arrow_length,
                arrow_width,
            )
            if arrow_points is None:
                continue
            self._appendTriangleVertices(
                arrowhead_vertices,
                arrow_points,
                0.0,
                self._relationshipLinkColor(link, hoveredColor, selectedColor),
            )
        if arrowhead_vertices:
            self._drawNodeVertices(
                arrowhead_vertices,
                projection,
                0.0,
                (0.0, 0.0, 0.0, 0.0),
            )

    def _renderDanglingIndicators(self, danglingLinks, projection):
        """Render circle indicators at the endpoints of dangling links.

        Each dangling link gets a filled circle at the unpopulated end,
        with a hover highlight when the cursor is over it.
        """
        if not self.shaderLibrary:
            return

        shader = self.shaderLibrary.get("basic")
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
        hoveredIdx = self._danglingIndicatorUnderCursor
        hasHover = hoveredIdx >= 0

        try:
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
                linkIdx = self.links.index(link) if hasHover else -1
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

    def _renderTemporaryLink(self):
        """
        Render the temporary link being dragged with color feedback.

        Color indicates connection validity:
        - Green: Valid target port (correct direction)
        - Red: Invalid target (same direction as source)
        - Active color (configurable): Dragging (no port hovered)
        """
        # Determine base color based on validity
        if self._dragLinkValidTarget:
            # Green for valid target
            baseColor = [0.0, 1.0, 0.0]
        elif self._hoveredPort and self._hoveredPort[2] == self._dragLinkSourceIsOutput:
            # Red for invalid target (same direction)
            baseColor = [1.0, 0.0, 0.0]
        else:
            # Active/dragging color (cached, updated via settings signal)
            if is_relationship_pin(str(self._dragLinkSourcePort)):
                baseColor = list(self._cachedLinkRelationshipBaseColor[:3])
            else:
                activeColor = self._cachedLinkActiveColor
                baseColor = [activeColor[0], activeColor[1], activeColor[2]]

        # The free end is the cursor side; the anchored end is the source pin.
        # Snapshot both as plain float tuples so the inset/restore below cannot
        # be aliased by the C++ LinkData accessors.
        temp_link = self._dragLinkTempLink
        is_relationship = bool(getattr(temp_link, "is_relationship_link", False))
        if self._dragLinkSourceIsOutput:
            # Dragging from output, free end is at 'end' (input side)
            free_end_attr = "end"
            endPos = (float(temp_link.end[0]), float(temp_link.end[1]))
            startPos = (float(temp_link.start[0]), float(temp_link.start[1]))
        else:
            # Dragging from input, free end is at 'start' (output side)
            free_end_attr = "start"
            endPos = (float(temp_link.start[0]), float(temp_link.start[1]))
            startPos = (float(temp_link.end[0]), float(temp_link.end[1]))

        arrow_length = max(12.0 / max(self.zoom, 1e-6), 4.0)
        arrow_width = max(10.0 / max(self.zoom, 1e-6), 3.0)
        # Face the arrowhead down only when snapped to a valid relationship
        # target (dropped on a node's top edge); otherwise it faces left/right.
        use_vertical = is_relationship and self._dragLinkValidTarget

        # Render using existing link renderer.
        # Pass as a single-element list with baseColor to override uniform.
        # When snapped to a whole-prim relationship target the noodle must stop
        # at the arrowhead BASE, not overshoot to its tip (the node top). Committed
        # links get that inset from the shader via the per-instance isPrimTarget
        # flag, but the per-call renderLinks path used for the drag preview always
        # passes isPrimTarget=0 -- so inset the free end here by the same
        # arrow_length the arrowhead uses, then restore it before the arrowhead
        # and hit geometry, which key off the true node top.
        projection = self._worldSpaceProjectionMatrix()
        hoveredColor = self._cachedLinkHoveredColor
        selectedColor = self._cachedLinkSelectedColor
        saved_free_end = None
        if use_vertical:
            saved_free_end = endPos
            setattr(temp_link, free_end_attr, (endPos[0], endPos[1] - arrow_length))
        self.linkRenderer.renderLinks(
            [temp_link],
            projection,
            self.zoom,
            self.panX,
            self.panY,
            self.width(),
            self.height(),
            self._renderConfig(),
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

            self._nodeRenderManager.renderEndpointTriangle(
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
            renderer = self.defaultRenderer
            radius = renderer.getPortWidth() if renderer else 8.0
            strokeWidth = self._cachedPortRingThickness
            strokeColor = [c * 0.7 for c in baseColor] + [1.0]
            self._nodeRenderManager.renderEndpointCircle(
                endPos[0],
                endPos[1],
                radius,
                0.0,
                projList,
                baseColor + [1.0],
                strokeColor,
                strokeWidth,
            )

    def paintStatusOverlay(self):
        if not self.nodeIdUnderCursor or not self.shaderLibrary or not self.fontAtlas:
            return

        if self.nodeIdUnderCursor not in self.nodes:
            return

        node = self.nodes[self.nodeIdUnderCursor]
        projection = self._screenSpaceProjectionMatrix()

        self.statusOverlay.render(
            node,
            self.textRenderer,
            NodeVertex,
            self._drawNodeVertices,
            projection,
            self.height(),
            MAX_RENDER_DEPTH,
        )

    def paintMarquee(self):
        if not self.marqueeActive or not self.shaderLibrary:
            return

        startPt = Gf.Vec2d(self.marqueeStart.x(), self.marqueeStart.y())
        currentPt = Gf.Vec2d(self.marqueeCurrent.x(), self.marqueeCurrent.y())
        marqueeBounds = Gf.Range2d.GetUnion(
            Gf.Range2d(startPt, startPt), Gf.Range2d(currentPt, currentPt)
        )

        size = marqueeBounds.GetSize()
        if size[0] < 1.0 or size[1] < 1.0:
            return  # Too small to render

        minPt = marqueeBounds.GetMin()
        maxPt = marqueeBounds.GetMax()
        w, h = size[0], size[1]
        depth = MAX_RENDER_DEPTH - 1.0  # Near the front

        r, g, b, a = 100, 150, 255, 80  # Semi-transparent blue

        marqueeVertexData = [
            NodeVertex(
                minPt[0],
                minPt[1],
                depth,
                0,
                0,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
            NodeVertex(
                maxPt[0],
                minPt[1],
                depth,
                w,
                0,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
            NodeVertex(
                minPt[0],
                maxPt[1],
                depth,
                0,
                h,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
            NodeVertex(
                maxPt[0],
                minPt[1],
                depth,
                w,
                0,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
            NodeVertex(
                maxPt[0],
                maxPt[1],
                depth,
                w,
                h,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
            NodeVertex(
                minPt[0],
                maxPt[1],
                depth,
                0,
                h,
                w,
                h,
                r,
                g,
                b,
                a,
                2.0,
                0,
                0,
                0,
                255,
                0.0,
            ),
        ]

        try:
            projection = self._worldSpaceProjectionMatrix()
            strokeColor = (150 / 255.0, 200 / 255.0, 1.0, 180 / 255.0)
            # Always force upload for marquee since it changes every frame during drag
            self._drawNodeVertices(
                marqueeVertexData,
                projection,
                0.0,
                strokeColor,
                generation=0,
                selectionChanged=True,
            )

        except Exception as e:
            Tf.Warn(f"Error rendering marquee: {e}")
            import traceback

            traceback.print_exc()

    def _frameNodeBounds(self, bounds, margin=1.0):
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

        # Apply margin by expanding the bounds
        boundsCenter = Gf.Vec2d(
            (bounds.GetMin()[0] + bounds.GetMax()[0]) * 0.5,
            (bounds.GetMin()[1] + bounds.GetMax()[1]) * 0.5,
        )
        expandedSize = boundsSize * margin
        bounds = Gf.Range2d(
            boundsCenter - expandedSize * 0.5, boundsCenter + expandedSize * 0.5
        )
        boundsSize = Gf.Vec2d(bounds.GetSize())

        viewSize = Gf.Vec2d(self.width(), self.height())
        scale = Gf.Vec2d(viewSize[0] / boundsSize[0], viewSize[1] / boundsSize[1])
        self.zoom = min(scale[0], scale[1])

        # Center bounds in view
        scaledView = viewSize / self.zoom
        offset = (scaledView - boundsSize) * 0.5
        self.panX = bounds.GetMin()[0] - offset[0]
        self.panY = bounds.GetMin()[1] - offset[1]


    # Inline attribute values
    #
    # Geometry, hit-testing, the drag, the keyboard editor and the token
    # popup. The invariant the whole group rests on is stated once, in
    # widgets/valueCell.cell_rect: the value cell and the two connection
    # gutters are disjoint, so a press can mean exactly one of the two.

    # Cursor per cell kind: the pointer says what a press will do before it
    # happens -- drag a number, type into text, open a list, flip a switch.
    _VALUE_CURSORS = {
        noodlesValues.KIND_FLOAT: QtCore.Qt.SizeHorCursor,
        noodlesValues.KIND_INT: QtCore.Qt.SizeHorCursor,
        noodlesValues.KIND_TEXT: QtCore.Qt.IBeamCursor,
        noodlesValues.KIND_TOKEN_ENUM: QtCore.Qt.PointingHandCursor,
        noodlesValues.KIND_BOOL: QtCore.Qt.PointingHandCursor,
    }

    def _afterUsdviewRefresh(self, callback):
        """Run *callback* once usdview has rebuilt its prim browser.

        A namespace edit makes usdview schedule a full prim-browser rebuild on
        a zero-interval QTimer (AppController.updateGUI). Selecting the new
        path before that runs makes usdview look it up in a prim-to-item map
        that still describes the old namespace; its updateSelection then fails
        on the missing item with an AttributeError inside usdview's own slot,
        and the prim browser does not follow the rename.

        Another zero-interval QTimer started now fires after usdview's, because
        timers with equal timeouts fire in the order they were started, so the
        callback sees the rebuilt tree. QTimer.singleShot(0) is not the same:
        it posts an event, and some event dispatchers deliver posted events
        before pending timers.
        """
        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(0)

        def fire():
            timer.deleteLater()
            callback()

        timer.timeout.connect(fire)
        timer.start()

    _initUndoTracking = _editing._initUndoTracking
    _applyNodePositions = _editing._applyNodePositions
    _restoreNodePositions = _editing._restoreNodePositions
    _authorNodePosition = _editing._authorNodePosition
    _performUndo = _editing._performUndo
    _performRedo = _editing._performRedo
    _persistNodePositions = _editing._persistNodePositions
    _saveLayerAsSpelled = staticmethod(_editing._saveLayerAsSpelled)
    _saveStage = _editing._saveStage
    _deleteAllSelected = _editing._deleteAllSelected
    _deactivateNodesInUsd = _editing._deactivateNodesInUsd
    _pushDeleteSelectedUndo = _editing._pushDeleteSelectedUndo
    _applyRestoreDeleted = _editing._applyRestoreDeleted
    _applyDeleteSelected = _editing._applyDeleteSelected
    deleteSelectedNodes = _editing.deleteSelectedNodes
    _createNodeFromHotbox = _editing._createNodeFromHotbox
    _resolveParentPath = _editing._resolveParentPath
    _beginRename = _editing._beginRename
    _cancelRename = _editing._cancelRename
    _commitRename = _editing._commitRename
    _onNodeCreated = _editing._onNodeCreated

    linkSelectionMode = property(
        _selection.get_link_selection_mode, _selection.set_link_selection_mode
    )
    _updateLinkSelectionFromNodes = _selection._updateLinkSelectionFromNodes
    _selectLink = _selection._selectLink
    _toggleLinkSelection = _selection._toggleLinkSelection
    _syncLinkSelectionToPrimtree = _selection._syncLinkSelectionToPrimtree
    clearLinkSelection = _selection.clearLinkSelection
    _getFirstSelectedLink = _selection._getFirstSelectedLink
    _selectFirstLinkInDirection = _selection._selectFirstLinkInDirection
    _getFirstSelectedNode = _selection._getFirstSelectedNode
    selectLinkByPinIndex = _selection.selectLinkByPinIndex
    _onPropSelectionChanged = _selection._onPropSelectionChanged
    _onPrimSelectionChanged = _selection._onPrimSelectionChanged
    _syncSelectionToPrimtree = _selection._syncSelectionToPrimtree
    _frameInPrimTree = _selection._frameInPrimTree
    _setLinkSelectionMode = _selection._setLinkSelectionMode
    _toggleFrameInPrimTree = _selection._toggleFrameInPrimTree
    _clearSelectionAndSync = _selection._clearSelectionAndSync
    _canOpenSelectedContainer = _selection._canOpenSelectedContainer
    _selectNodesInMarquee = _selection._selectNodesInMarquee
    _selectLinksInMarquee = _selection._selectLinksInMarquee
    _updateMarqueeSelection = _selection._updateMarqueeSelection
    selectAll = _selection.selectAll
    clearSelection = _selection.clearSelection
    _getContainerChildNodes = _selection._getContainerChildNodes
    addNodesFromPrimTreeSelection = _selection.addNodesFromPrimTreeSelection
    removeSelectedNodes = _selection.removeSelectedNodes
    _resolveSelectedPrims = _selection._resolveSelectedPrims
    _resolveSelectedParentPrim = _selection._resolveSelectedParentPrim

    # Graph/topology behavior moved to graphview/model.py (nv-model-graph).
    # Explicit bindings, never forwarding bodies, so unbound
    # GraphView._method(duck_self, ...) calls keep working.
    _addBackReferenceDanglingLinks = _graphview_model.add_back_reference_dangling_links
    _addLinkToGraph = _graphview_model.add_link_to_graph
    _addNewNodeFast = _graphview_model.add_new_node_fast
    _addNodeFromDanglingLink = _graphview_model.add_node_from_dangling_link
    _addNodesFromDanglingLink = _graphview_model.add_nodes_from_dangling_link
    _addPrimAsNode = _graphview_model.add_prim_as_node
    _classifyRelationshipLinkFromUsd = staticmethod(_graphview_model.classify_relationship_link_from_usd)
    _collectSiblingDanglingLinks = _graphview_model.collect_sibling_dangling_links
    _computeLinkBounds = _graphview_model.compute_link_bounds
    _findLinkToInput = _graphview_model.find_link_to_input
    _findPinIndex = _graphview_model.find_pin_index
    _getLinkEndpointPosition = _graphview_model.get_link_endpoint_position
    _getLinksForNodeDirection = _graphview_model.get_links_for_node_direction
    _getLinksForNodeFlat = _graphview_model.get_links_for_node_flat
    _hasExactLink = _graphview_model.has_exact_link
    _invalidateBackReferenceLinkDataCache = _graphview_model.invalidate_back_reference_link_data_cache
    _makeLinkFromUsdData = staticmethod(_graphview_model.make_link_from_usd_data)
    _populateNodeLinksFromPrim = _graphview_model.populate_node_links_from_prim
    _processNodeLinks = _graphview_model.process_node_links
    _rebuildConnectionCache = _graphview_model.rebuild_connection_cache
    _rebuildLinks = _graphview_model.rebuild_links
    _reconcileMovedNodes = _graphview_model.reconcile_moved_nodes
    _removeLinkFromGraph = _graphview_model.remove_link_from_graph
    _resolveLinkDisplayPinName = _graphview_model.resolve_link_display_pin_name
    _resolvePortPropertyName = _graphview_model.resolve_port_property_name
    _setupDanglingLinkEndpoint = _graphview_model.setup_dangling_link_endpoint
    _setupLinkEndpoints = _graphview_model.setup_link_endpoints
    _surfaceConnectionEndpointPins = _graphview_model.surface_connection_endpoint_pins
    _syncLinksIfNeeded = _graphview_model.sync_links_if_needed
    _updateLinksForMovedNodes = _graphview_model.update_links_for_moved_nodes
    _updateNodeLinkEndpoints = _graphview_model.update_node_link_endpoints

    # Placement and geometry bindings preserve unbound calls on view doubles.
    _appendTriangleVertices = _graphview_model.append_triangle_vertices
    _applyMovedNodeDerivedState = _graphview_model.apply_moved_node_derived_state
    _autoLayoutNodes = _graphview_model.auto_layout_nodes
    _batchExtents = _graphview_model.batch_extents
    _getNodeTopCenterPosition = staticmethod(
        _graphview_model.get_node_top_center_position
    )
    _getPortPosition = _graphview_model.get_port_position
    _getPropertyOutputSide = staticmethod(_graphview_model.get_property_output_side)
    _getRelationshipRowEndpointPosition = (
        _graphview_model.get_relationship_row_endpoint_position
    )
    _getVisiblePortPosition = _graphview_model.get_visible_port_position
    _gridPlaceNodes = _graphview_model.grid_place_nodes
    _mirrorPortPosition = staticmethod(_graphview_model.mirror_port_position)
    _nodeHasAuthoredPosition = _graphview_model.node_has_authored_position
    _nodeHasVisiblePort = _graphview_model.node_has_visible_port
    _placeAddedNodes = _graphview_model.place_added_nodes
    _positionNodeInViewport = _graphview_model.position_node_in_viewport
    _pushBelowOccupied = _graphview_model.push_below_occupied
    _updateNodeSpatialBounds = _graphview_model.update_node_spatial_bounds
    getConnectedNodes = _graphview_model.get_connected_nodes
    _asWorldXY = staticmethod(_graphview_model.as_world_xy)
    _editableValueCells = _graphview_model.editable_value_cells
    _findValueRow = _graphview_model.find_value_row
    _valueAxisWidth = _graphview_model.value_axis_width
    _valueCellRectFor = _graphview_model.value_cell_rect_for
    _valueDigitAdvance = _graphview_model.value_digit_advance
    _valueFittedTexts = _graphview_model.value_fitted_texts
    _valueFontSize = _graphview_model.value_font_size
    _valueRowGeometry = _graphview_model.value_row_geometry
    _valueRowLayout = _graphview_model.value_row_layout
    _valueStateColors = _graphview_model.value_state_colors
    _valueTextWidth = _graphview_model.value_text_width

    def _makeCommandRegistry(self):
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
            registry.register(intent_type, lambda _, method=method: getattr(self, method)())
        registry.register(
            _intents.Navigate,
            lambda request: getattr(self, {
                "backward": "navigateBackward",
                "forward": "navigateForward",
            }[request.direction])(),
        )
        registry.register(
            _intents.BeginNodeDrag,
            lambda request: _interaction.begin_node_drag(
                self, request.node_id, request.toggle
            ),
        )
        registry.register(
            _intents.UpdateNodeDrag,
            lambda request: _interaction.move_dragged_nodes(
                self, request.dx, request.dy
            ),
        )
        registry.register(
            _intents.BeginMarquee,
            lambda request: _interaction.begin_marquee(
                self, request.origin, request.mode
            ),
        )
        registry.register(
            _intents.UpdateMarquee,
            lambda request: _interaction.update_marquee(self, request.current),
        )
        registry.register(
            _intents.FinishMarquee,
            lambda request: _interaction.finish_marquee(self),
        )
        registry.register(
            _intents.Pan,
            lambda request: _interaction.pan_viewport(self, request.dx, request.dy),
        )
        registry.register(
            _intents.Zoom,
            lambda request: _interaction.zoom_viewport(
                self, request.focus, request.zoom
            ),
        )

        return registry

    _finishConnectionReplacement = _editing._finishConnectionReplacement
    _deleteLinksInUsd = _editing._deleteLinksInUsd
    _abortConnection = _editing._abortConnection
    _resolveConnectionEndpoints = _editing._resolveConnectionEndpoints
    _deleteOldConnection = _editing._deleteOldConnection
    _createConnection = _editing._createConnection
    _deleteConnection = _editing._deleteConnection
    _applyApiSchemaFromHotbox = _editing._applyApiSchemaFromHotbox
    _showApiSchemaResult = _editing._showApiSchemaResult
    _writeValueComponents = _editing._writeValueComponents
    _pushValueUndo = _editing._pushValueUndo
    _updateMung = _editing._updateMung
    _finishMung = _editing._finishMung
    _cancelMung = _editing._cancelMung
    _toggleBoolValue = _editing._toggleBoolValue
    _beginValueEdit = _editing._beginValueEdit
    _cancelValueEdit = _editing._cancelValueEdit
    _commitValueEdit = _editing._commitValueEdit
    _advanceValueEdit = _editing._advanceValueEdit
    _commitTokenPopup = _editing._commitTokenPopup

    # Legacy link-record and layer-operation descriptors.
    _collectSelectedLinkData = _intents._collectSelectedLinkData
    _collectIncidentLinkKeys = _intents._collectIncidentLinkKeys
    _makeSerializedLinkRecord = staticmethod(_intents._makeSerializedLinkRecord)
    _deserializeLinkRecord = staticmethod(_intents._deserializeLinkRecord)
    _linkMatchesRecord = classmethod(_intents._linkMatchesRecord)
    _expandSdfListOpItems = staticmethod(_intents._expandSdfListOpItems)
    _buildAttrSearchPaths = staticmethod(_intents._buildAttrSearchPaths)
    _buildRelSearchEntries = staticmethod(_intents._buildRelSearchEntries)
    _layerAuthorsAttrConnection = classmethod(_intents._layerAuthorsAttrConnection)
    _layerAuthorsRelTarget = classmethod(_intents._layerAuthorsRelTarget)
    _findConnectionAuthoringLayer = _intents._findConnectionAuthoringLayer
    _editOnLayer = _intents._editOnLayer
    _buildConnectionKwargs = staticmethod(_intents._buildConnectionKwargs)
    _groupLinksByLayer = staticmethod(_intents._groupLinksByLayer)
    _runLinkOperationOnLayer = _intents._runLinkOperationOnLayer
    _findLibraryForPrim = _intents._findLibraryForPrim

    # Hit testing, rows and hover moved to graphview/interaction.py
    # (nv-interaction-hits). Explicit bindings, never forwarding bodies,
    # so unbound GraphView._method(duck_self, ...) calls keep working.
    _getNodeIdAtWorldPos = _interaction._getNodeIdAtWorldPos
    _getPropertyAtPoint = _interaction._getPropertyAtPoint
    _findPropertyRowHover = _interaction._findPropertyRowHover
    _findRelationshipRowHover = _interaction._findRelationshipRowHover
    _relationshipDragRowHover = _interaction._relationshipDragRowHover
    _getRowHitMetrics = _interaction._getRowHitMetrics
    _valueCellAtPoint = _interaction._valueCellAtPoint
    _findRowEdgeDragStart = _interaction._findRowEdgeDragStart
    _rowEdgePinCandidates = _interaction._rowEdgePinCandidates
    _pinAtRowEdge = _interaction._pinAtRowEdge
    _findArrowheadUnderCursor = _interaction._findArrowheadUnderCursor
    _findDanglingIndicatorUnderCursor = _interaction._findDanglingIndicatorUnderCursor
    _findDanglingLinkAtPort = _interaction._findDanglingLinkAtPort
    _updateHoveredPort = _interaction._updateHoveredPort
    _setHoveredPort = _interaction._setHoveredPort
    _updateNodeUnderCursor = _interaction._updateNodeUnderCursor
    _updateValueHoverCursor = _interaction._updateValueHoverCursor
    _showPropertyTooltip = _interaction._showPropertyTooltip
    _updatePropertyTooltip = _interaction._updatePropertyTooltip
    _clearPropertyTooltip = _interaction._clearPropertyTooltip
    _centerOnNode = _interaction._centerOnNode
    _setZoom = _interaction._setZoom
    _zoomAroundPoint = _interaction._zoomAroundPoint
    navigateBackward = _interaction.navigateBackward
    navigateForward = _interaction.navigateForward
    _jumpToSourceNode = _interaction._jumpToSourceNode
    _jumpToTargetNode = _interaction._jumpToTargetNode
    _jumpToSourceNodeInternal = _interaction._jumpToSourceNodeInternal
    _jumpToTargetNodeInternal = _interaction._jumpToTargetNodeInternal
    _navigateAlongRelationshipLink = _interaction._navigateAlongRelationshipLink

    # Drag, marquee and reconnect gestures moved to graphview/interaction.py
    # (nv-interaction-gestures). Explicit bindings, never forwarding bodies,
    # so unbound GraphView._method(duck_self, ...) calls keep working.
    _clearValueInteractionState = _interaction._clearValueInteractionState
    _updateAnimator = _interaction._updateAnimator
    _updatePanAnimation = _interaction._updatePanAnimation
    _updateKeyboardState = _interaction._updateKeyboardState
    _finalizeNodeDrag = _interaction._finalizeNodeDrag
    _startLinkDrag = _interaction._startLinkDrag
    _snapLinkDragToPort = _interaction._snapLinkDragToPort
    _updateLinkDrag = _interaction._updateLinkDrag
    _completeLinkDrag = _interaction._completeLinkDrag
    _startLinkReconnect = _interaction._startLinkReconnect
    _clearReconnectState = _interaction._clearReconnectState
    _cancelLinkReconnect = _interaction._cancelLinkReconnect
    _finishReconnectDisconnect = _interaction._finishReconnectDisconnect

    # GL lifecycle moved to graphview/resources.py (nv-render-resources).
    # Explicit bindings, never forwarding bodies, so unbound
    # GraphView._method(duck_self, ...) calls keep working.
    initializeGL = _resources.initializeGL
    _reinitializeGL = _resources._reinitializeGL
    resizeGL = _resources.resizeGL

    # Drawing entry moved to graphview/render.py (nv-render-resources).
    # Explicit bindings, never forwarding bodies, so unbound
    # GraphView._method(duck_self, ...) calls keep working.
    paintGL = _render.paintGL
    _paintGLInner = _render._paintGLInner
    _renderConfig = _render._renderConfig
    _seedNodeZOrders = _render._seedNodeZOrders
    _worldSpaceProjectionMatrix = _render._worldSpaceProjectionMatrix
    _screenSpaceProjectionMatrix = _render._screenSpaceProjectionMatrix

    # Node, port and sticker primitives moved to graphview/render_nodes.py
    # (nv-render-nodes). Explicit bindings, never forwarding bodies, so
    # unbound GraphView._method(duck_self, ...) calls keep working.
    paintNodes = _render_nodes.paintNodes
    _drawNodeVertices = _render_nodes._drawNodeVertices
    _renderPortHighlightsCpp = _render_nodes._renderPortHighlightsCpp
    paintGroupStickers = _render_nodes.paintGroupStickers
    _relationshipLinkColor = _render_nodes._relationshipLinkColor
    _isPrimTargetRelationshipLink = staticmethod(
        _render_nodes._isPrimTargetRelationshipLink
    )

    # Text, value cells and in-place editors moved to graphview/render_text.py
    # (nv-render-text). Explicit bindings, never forwarding bodies, so
    # unbound GraphView._method(duck_self, ...) calls keep working.
    paintText = _render_text.paintText
    _paintValueCells = _render_text._paintValueCells
    _paintValueEditor = _render_text._paintValueEditor
    _emitValueText = _render_text._emitValueText
    _valueTextBand = _render_text._valueTextBand
    _valueGlyph = _render_text._valueGlyph
    _valueBaselineY = _render_text._valueBaselineY
    _appendBoolSwitch = _render_text._appendBoolSwitch
    _appendChevron = _render_text._appendChevron
    _paintNodeRename = _render_text._paintNodeRename
    _paintMungLadder = _render_text._paintMungLadder
    _paintTokenPopup = _render_text._paintTokenPopup
    _activeValueFlash = _render_text._activeValueFlash
    _startValueFlash = _render_text._startValueFlash
    paintHotkeyPopup = _render_text.paintHotkeyPopup
    paintNavigationHotkeyPopup = _render_text.paintNavigationHotkeyPopup
    _paintHotkeyPopups = _render_text._paintHotkeyPopups
