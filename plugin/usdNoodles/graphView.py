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


def _guardsQtBoundary(handler):
    """Stop a Python exception escaping into Qt's virtual dispatch.

    These handlers override C++ virtuals, so an exception raised inside one has
    to be carried back out through shiboken. That path is not robust in this
    PySide build: a mouse release whose handler raised took the process down
    with SIGSEGV inside PepException_GetArgs, reading 0xffffffffffffffff, with
    no traceback -- so the Python bug that started it was invisible, which is
    the worst part.

    event()'s own `except RuntimeError` cannot cover this. It wraps
    super().event(), and super().event() is where the boundary crash happens,
    so it only ever sees the failures shiboken survived long enough to report.
    Catching here, before the exception is handed back to C++, is what keeps
    the process alive AND produces a traceback to act on.

    The event is dropped, which for a mouse or key handler means one lost
    interaction rather than a lost session.
    """
    @functools.wraps(handler)
    def guarded(self, event):
        try:
            return handler(self, event)
        except Exception:
            Tf.Warn("%s.%s raised; the event was dropped. This is a bug -- "
                    "please report the traceback:\n%s"
                    % (type(self).__name__, handler.__name__,
                       traceback.format_exc()))
            return None

    return guarded


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


class GraphView(QGLWidget):
    # Emitted whenever ``self.nodeGraph`` is reassigned (e.g. when a new stage
    # or blueprint is loaded and the underlying NodeGraph instance is swapped).
    # Listeners that subscribed to NodeGraph callbacks need this so they can
    # re-register on the new instance.
    nodeGraphReplaced = QtCore.Signal()

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

    def _updateNodeCreationHotbox(self):
        """
        Update the node creation hotbox configuration from enabled libraries.

        This method iterates through all enabled node libraries, collects their
        families and node types, and configures the hotbox to display them.
        Library references are embedded in the node data for later use.

        The applied API schemas registered in this build are appended after the
        libraries. They are not node types - picking one applies it to the
        selected prims rather than creating a prim.
        """
        # Collect all node types with library references from enabled libraries
        node_items = []

        for library in self._nodeLibraries:
            if not library.enabled:
                continue

            # Get families from this library
            families = library.get_families()

            for family in families:
                # Get node types for this family
                node_types = library.get_node_types(family)

                if not node_types:
                    continue

                # Create node items with embedded library references
                for node_type in node_types:
                    node_item = {
                        "name": node_type["name"],
                        "family": family,
                        "identifier": node_type["identifier"],
                        "library": library,  # Reference to the library for creation
                        "kind": "prim",
                    }
                    node_items.append(node_item)

        node_items.extend(self._apiSchemaHotboxItems())

        # Configure the hotbox with the collected node items
        self.nodeCreationHotbox.configure_from_libraries(node_items)

    @staticmethod
    def _apiSchemaHotboxItems():
        """Build hotbox entries for every applied API schema in this build.

        Each entry is suffixed "*api" and grouped under the plugin that
        defines it, so the list reads as schemas to apply rather than prims to
        create. The marker doubles as a filter term: the list filters on a
        plain substring of the display name, and no prim type contains "*", so
        typing it narrows the hotbox to schemas alone. It stays out of the
        "identifier" field, which is what ApplyAPI receives.

        get_api_schemas() returns them sorted by plugin, which the list widget
        requires: it emits a group header whenever the group changes, so a
        non-contiguous group would repeat its header.
        """
        from .apiSchemas import API_SCHEMA_KIND, get_api_schemas

        return [
            {
                "name": f"{schema.identifier} *api",
                "family": f"API Schemas: {schema.plugin}",
                "identifier": schema.identifier,
                "library": None,
                "kind": API_SCHEMA_KIND,
            }
            for schema in get_api_schemas()
        ]

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

    def _showHotkeyPopup(self, keyText, direction):
        if direction == "left":
            message = f"<- {keyText}"
        else:
            message = f"{keyText} ->"

        cursorPos = self.mapFromGlobal(QtGui.QCursor.pos())
        self.hotkeyPopup.show(message, cursorPos)
        self.update()

    # Warning toasts (non-persistent edit target, save errors) linger 1s longer
    # than navigation popups so users have time to read them.
    _WARNING_POPUP_DURATION_MS = 1400

    def _showPopupMessage(self, message, duration_ms=None):
        cursorPos = self.mapFromGlobal(QtGui.QCursor.pos())
        self.hotkeyPopup.show(message, cursorPos, duration_ms=duration_ms)
        self.update()

    def _showWarningPopup(self, message):
        self._showPopupMessage(message, duration_ms=self._WARNING_POPUP_DURATION_MS)

    def timerEvent(self, event):
        if self.hotkeyPopup.handle_timer_event(event.timerId()):
            self.update()



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

    def _clearValueInteractionState(self):
        """Abandon any value edit / drag / popup without authoring.

        Called wherever the graph underneath is about to be replaced. A mung
        left live across a stage reload would hold a prim path that no longer
        means anything and would author onto whatever now sits there.
        """
        if self._mungState is not None and self._mungActive:
            self._noticeHandler.setEnabled(True)
        self._mungState = None
        self._mungActive = False
        self._valueEditTarget = None
        self._valueEditInput.clear()
        self._tokenPopup = None
        # The fitted-text cache is keyed by node id, so a rebuilt graph must
        # not inherit it (stale entries would only miss on their component
        # keys, but clearing bounds the map's growth across reloads).
        self._valueFitCache.clear()

    def closeEvent(self, event):
        stage_sync.closeStageLifecycle(self)
        super().closeEvent(event)

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

    def enterEvent(self, event):
        app = QtWidgets.QApplication.instance()
        if app:
            self._previousActiveWindow = app.activeWindow()

        topLevel = self.window()
        if topLevel:
            topLevel.activateWindow()
        self.setFocus(QtCore.Qt.MouseFocusReason)

        super().enterEvent(event)

    def leaveEvent(self, event):
        if self._previousActiveWindow and self._previousActiveWindow.isVisible():
            self._previousActiveWindow.activateWindow()
        self._previousActiveWindow = None

        self._clearPropertyTooltip()

        # The resize cursor belongs to a value cell the pointer has now left.
        if self._valueCursorSet is not None:
            self.setCursor(QtCore.Qt.ArrowCursor)
            self._valueCursorSet = None
        self._valueHover = None
        # A drag whose button release happened outside the widget would
        # otherwise leave _mungState live and the notice handler disabled.
        if self._mungState is not None and not (
            QtWidgets.QApplication.mouseButtons() & QtCore.Qt.LeftButton
        ):
            self._cancelMung()

        super().leaveEvent(event)

    def _resolveShortcutConflicts(self, shortcuts):
        """Reset stored shortcuts that collide with a later-defined action.

        When a default shortcut is reassigned between releases (e.g. "D" moved
        from shortcutToggleLinkDimming to shortcutRemoveFromGraph), a user's
        persisted state can still bind the key to the original action. The
        manual dispatch in keyPressEvent iterates _shortcutActions in insertion
        order and triggers the first match, which means the older action wins
        and the new one is silently masked.

        Walk the shortcuts list and, for each duplicate normalized key
        sequence, reset every earlier configKey to its schema default so the
        latest definition takes effect.
        """
        settings = NoodlesConfig.settings()
        if settings is None:
            return

        # Collect the configured key string for each shortcut in declaration
        # order. We compare on QKeySequence.toString() to normalize formatting
        # (e.g. "ctrl+s" vs "Ctrl+S").
        loaded = []
        for _name, configKey, _checkable, _checkedGetter, _handler in shortcuts:
            shortcutStr = NoodlesConfig.get(configKey, "")
            normalized = (
                QtGui.QKeySequence(shortcutStr).toString() if shortcutStr else ""
            )
            loaded.append((configKey, normalized))

        # For each later entry, blank out any earlier entry that holds the
        # same key. Walking from the end keeps the latest-defined action
        # authoritative without re-iterating.
        seen = {}
        for configKey, normalized in reversed(loaded):
            if not normalized:
                continue
            if normalized in seen:
                # An entry defined LATER already claims this key, so reset
                # this earlier entry to its schema default to break the tie.
                info = settings.getSettingInfo(configKey)
                if not info:
                    continue
                defaultStr = info.get("default", "")
                defaultNormalized = (
                    QtGui.QKeySequence(defaultStr).toString() if defaultStr else ""
                )
                # Only rewrite when the default would actually free the key,
                # otherwise we would loop the same conflict on the next launch.
                if defaultNormalized == normalized:
                    continue
                Tf.Status(
                    f"Resetting shortcut {configKey!r} from {normalized!r} to "
                    f"default {defaultStr!r} (conflicts with {seen[normalized]!r})"
                )
                NoodlesConfig.set(configKey, defaultStr)
            else:
                seen[normalized] = configKey

    def _setupShortcuts(self):
        """Setup shortcuts from configuration."""
        shortcuts = [
            # (name, configKey, checkable, initialCheckedGetter, handler)
            (
                "Toggle Links",
                "shortcutToggleLinks",
                True,
                lambda: self.drawLinks,
                self._toggleDrawLinks,
            ),
            (
                "Toggle Nodes",
                "shortcutToggleNodes",
                True,
                lambda: self.drawNodes,
                self._toggleDrawNodes,
            ),
            (
                "Toggle Text",
                "shortcutToggleText",
                True,
                lambda: self.drawText,
                self._toggleDrawText,
            ),
            (
                "Toggle Link Dimming",
                "shortcutToggleLinkDimming",
                True,
                lambda: self.linkDimming.target > 0.0,
                self._toggleLinkDimming,
            ),
            ("Select All", "shortcutSelectAll", False, None, self.selectAll),
            (
                "Navigate Backward",
                "shortcutNavigateBackward",
                False,
                None,
                self.navigateBackward,
            ),
            (
                "Navigate Forward",
                "shortcutNavigateForward",
                False,
                None,
                self.navigateForward,
            ),
            ("Frame Selection", "shortcutFrameSelection", False, None, self.frameScene),
            ("Frame All", "shortcutFrameAll", False, None, self.frameAll),
            (
                "Frame with Connections",
                "shortcutFrameWithConnections",
                False,
                None,
                self.frameSelectionWithConnections,
            ),
            (
                "Remove from Graph",
                "shortcutRemoveFromGraph",
                False,
                None,
                self.removeSelectedNodes,
            ),
            (
                "Add from Prim Tree",
                "shortcutAddFromPrimTree",
                False,
                None,
                self.addNodesFromPrimTreeSelection,
            ),
        ]

        # Migration: resolve shortcut conflicts. When a default shortcut moves
        # from one action to another (e.g. "D" moved from shortcutToggleLinkDimming
        # to shortcutRemoveFromGraph), a user's persisted value can still claim
        # the key, masking the newer action. Reset any earlier action whose
        # current value collides with a later action's value so the later
        # definition wins.
        self._resolveShortcutConflicts(shortcuts)

        # Store actions for dynamic updates
        self._shortcutActions = {}

        for name, configKey, checkable, checkedGetter, handler in shortcuts:
            # Get shortcut from config (with default fallback already in schema)
            shortcutStr = NoodlesConfig.get(configKey, "")

            action = QtGui.QAction(name, self)

            # Only set shortcut if not empty
            if shortcutStr:
                action.setShortcut(QtGui.QKeySequence(shortcutStr))

            action.setShortcutContext(QtCore.Qt.WidgetShortcut)

            if checkable:
                action.setCheckable(True)
                action.setChecked(checkedGetter())

            action.triggered.connect(handler)
            self.addAction(action)

            # Store for dynamic updates
            self._shortcutActions[configKey] = action

    def updateShortcuts(self):
        """Update all shortcuts from current configuration.
        Called when preferences are applied."""
        if not hasattr(self, "_shortcutActions"):
            return

        for configKey, action in self._shortcutActions.items():
            shortcutStr = NoodlesConfig.get(configKey, "")
            if shortcutStr:
                action.setShortcut(QtGui.QKeySequence(shortcutStr))
            else:
                # Clear shortcut if empty string
                action.setShortcut(QtGui.QKeySequence())

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

    def _cleanupNodeVBOs(self):
        """
        Clean up all rendering resources.

        Called when the OpenGL context is lost or being reinitialized.
        """
        self._nodeRenderManager.cleanup()
        self._groupStickerRenderer.cleanup()
        self.linkRenderer.cleanup()
        self.textRenderer._cpp.cleanup()
        self._nodeTransformFrame.cleanup()
        if hasattr(self, "_cppIconRenderer") and self._cppIconRenderer.isInitialized:
            self._cppIconRenderer.cleanup()

    def _reinitializeGL(self):
        """Handle GL context change — clean up stale resources and reinitialize."""
        self.makeCurrent()
        self._cleanupNodeVBOs()
        if self.shaderLibrary:
            self.shaderLibrary.cleanup()
        if self.fontAtlas:
            self.fontAtlas.cleanup()
        self.initialized = False
        self.fontAtlas = None
        self.shaderLibrary = None
        self.initializeGL()
        # initializeGL() calls doneCurrent() — restore context for callers
        # (paintGL needs it current for _paintGLInner)
        self.makeCurrent()
        self.linksChanged = True
        # The atlas was regenerated, so every cached glyph vertex and every
        # width-derived fit is suspect. Dropping them only costs one warmup
        # frame; keeping them risks wrong UVs.
        self._valueGlyphCache.clear()
        self._valueFitCache.clear()
        self._valueAxisWidthCache.clear()
        self._valueTextBandCache.clear()
        self._valueDigitAdvanceCache.clear()

    def initializeGL(self):
        needsInit = not self.initialized

        if self.initialized:
            self.makeCurrent()
            contextChanged = False

            if self.fontAtlas and self.fontAtlas.texture:
                if not GL.glIsTexture(self.fontAtlas.texture):
                    contextChanged = True
            elif self.fontAtlas is None:
                contextChanged = True

            if contextChanged:
                self._cleanupNodeVBOs()
                if self.shaderLibrary:
                    self.shaderLibrary.cleanup()
                if self.fontAtlas:
                    self.fontAtlas.cleanup()
                needsInit = True
                self.initialized = False
                self.fontAtlas = None
                self.shaderLibrary = None

        if not needsInit:
            return

        self.makeCurrent()

        Glf.RegisterDefaultDebugOutputMessageCallback()

        from pxr import Plug

        plugin = Plug.Registry().GetPluginWithName("UsdNoodles")

        if plugin:
            # Plugin resource path points to: <install>/plugin/usd/usdNoodles/resources/
            resourcePath = Path(plugin.resourcePath)

            assetsInResources = resourcePath / "assets"
            if assetsInResources.exists():
                assetsPath = assetsInResources
            else:
                assetsPath = resourcePath
        else:
            resourcePath = Path(__file__).parent
            assetsPath = resourcePath / "assets"
            if not assetsPath.exists():
                assetsPath = resourcePath
        self._assetsPath = assetsPath

        atlasTexturePath = assetsPath / "fonts" / "Poppins-Regular.png"
        fontMetricsPath = assetsPath / "fonts" / "Poppins-Regular.json"

        if not atlasTexturePath.exists():
            Tf.Warn(f"Font atlas not found at {atlasTexturePath}")
            self.doneCurrent()
            return

        self.fontAtlas = FontAtlas()
        self.fontAtlas.initialize(str(atlasTexturePath), str(fontMetricsPath))

        if not self.nodes:
            sampleGraphPath = assetsPath / "bodyGraph.json"
            if sampleGraphPath.exists():
                self._loadSampleGraph(str(sampleGraphPath))
                if self.nodes:
                    self.frameScene()

        self.shaderLibrary = ShaderLibrary()
        self.shaderLibrary.initialize(str(assetsPath))

        # Initialize icon renderer
        from .iconRenderer import CppIconRenderer





        self._cppIconRenderer = CppIconRenderer()
        self._cppIconRenderer.initialize(self.shaderLibrary, str(assetsPath))

        # Initialize rendering managers
        self._nodeRenderManager.initialize(self.shaderLibrary)
        self._groupStickerRenderer.initialize(self.shaderLibrary)
        self.linkRenderer.initialize(self.shaderLibrary)
        self.textRenderer.initialize(self.shaderLibrary, self.fontAtlas)

        # All C++ render paths share one transform frame (text + node quads +
        # icons) so they sample one transform texture and move together on drag.
        self._nodeRenderManager.setTransformFrame(self._nodeTransformFrame)
        self.textRenderer.setTransformFrame(self._nodeTransformFrame)
        self._cppIconRenderer.setTransformFrame(self._nodeTransformFrame)

        self.initialized = True
        self.textChanged = True
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

    def _seedNodeZOrders(self):
        """Give each freshly loaded node a distinct, increasing zOrder.

        Loaded nodes otherwise keep the C++ default zOrder = 0, so every node
        lands in the same depth band; overlapping node backgrounds, title bars,
        shadows, and title text then z-fight under GL_LEQUAL on the first frame.
        Assigning a distinct zOrder per node (mirroring the runtime add-node and
        reload paths) separates the depth bands so overlaps render cleanly
        front-to-back. _nextZOrder is left at the max so later add / bring-to-front
        actions stack on top.
        """
        self._nextZOrder = 0
        for node in self.nodes.values():
            self._nextZOrder += 1
            node.zOrder = self._nextZOrder

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

    def _getContextMenuConf(self):
        return [
            (
                M.SUBMENU,
                "View",
                [
                    (
                        M.CHECK,
                        "Show Links",
                        None,
                        self._toggleDrawLinks,
                        self.drawLinks,
                    ),
                    (
                        M.CHECK,
                        "Show Nodes",
                        None,
                        self._toggleDrawNodes,
                        self.drawNodes,
                    ),
                    (M.CHECK, "Show Text", None, self._toggleDrawText, self.drawText),
                    (
                        M.CHECK,
                        "Show Attribute Values",
                        None,
                        self._toggleShowAttributeValues,
                        self._cachedShowAttributeValues,
                    ),
                    (M.SEPARATOR,),
                    (
                        M.CHECK,
                        "Link Dimming",
                        None,
                        self._toggleLinkDimming,
                        self.linkDimming.target > 0.0,
                    ),
                ],
            ),
            (
                M.SUBMENU,
                "Write Values",
                [
                    (
                        M.CHECK,
                        "Auto (key only if already animated)",
                        None,
                        lambda: self._setValueWriteMode("auto"),
                        self._cachedValueWriteMode == "auto",
                    ),
                    (
                        M.CHECK,
                        "Default (the rest value)",
                        None,
                        lambda: self._setValueWriteMode("default"),
                        self._cachedValueWriteMode == "default",
                    ),
                    (
                        M.CHECK,
                        "Animation (key at current frame)",
                        None,
                        lambda: self._setValueWriteMode("animation"),
                        self._cachedValueWriteMode == "animation",
                    ),
                ],
            ),
            (
                M.SUBMENU,
                "Zoom",
                [
                    (M.ACTION, "100%", None, lambda: self._setZoom(1.0)),
                    (M.ACTION, "50%", None, lambda: self._setZoom(0.5)),
                    (M.ACTION, "25%", None, lambda: self._setZoom(0.25)),
                    (M.SEPARATOR,),
                    (M.ACTION, "Zoom In", None, lambda: self._setZoom(self.zoom * 1.5)),
                    (
                        M.ACTION,
                        "Zoom Out",
                        None,
                        lambda: self._setZoom(self.zoom / 1.5),
                    ),
                ],
            ),
            (M.SEPARATOR,),
            (M.ACTION, "Frame All (A)", None, self.frameAll),
            (M.ACTION, "Frame Selection (F)", None, self.frameScene),
            (
                M.ACTION,
                "Frame Selection with Connections (Shift+F)",
                None,
                self.frameSelectionWithConnections,
            ),
            (M.ACTION, "Auto Layout (L)", None, self._autoLayoutNodes),
            (M.SEPARATOR,),
            (M.ACTION, "Select All (Ctrl+A)", None, self.selectAll),
            (M.ACTION, "Clear Selection", None, self._clearSelectionAndSync),
            (
                M.ACTION,
                "Delete Selected Nodes (Delete)",
                None,
                self.deleteSelectedNodes,
            ),
            (M.SEPARATOR,),
            (
                M.SUBMENU,
                "Link Selection Mode",
                [
                    (
                        M.CHECK,
                        "Nodes Only",
                        None,
                        lambda: self._setLinkSelectionMode(
                            LinkSelectionMode.NODES_ONLY
                        ),
                        self._linkSelectionMode == LinkSelectionMode.NODES_ONLY,
                    ),
                    (
                        M.CHECK,
                        "With All Links",
                        None,
                        lambda: self._setLinkSelectionMode(
                            LinkSelectionMode.WITH_ALL_LINKS
                        ),
                        self._linkSelectionMode == LinkSelectionMode.WITH_ALL_LINKS,
                    ),
                    (
                        M.CHECK,
                        "With Input Links",
                        None,
                        lambda: self._setLinkSelectionMode(
                            LinkSelectionMode.WITH_INPUTS
                        ),
                        self._linkSelectionMode == LinkSelectionMode.WITH_INPUTS,
                    ),
                    (
                        M.CHECK,
                        "With Output Links",
                        None,
                        lambda: self._setLinkSelectionMode(
                            LinkSelectionMode.WITH_OUTPUTS
                        ),
                        self._linkSelectionMode == LinkSelectionMode.WITH_OUTPUTS,
                    ),
                ],
            ),
            (M.SEPARATOR,),
            (M.ACTION, "Navigate Backward ([)", None, self.navigateBackward),
            (M.ACTION, "Navigate Forward (])", None, self.navigateForward),
            (M.SEPARATOR,),
            (
                M.CHECK,
                "Frame in Prim Tree on Select",
                None,
                self._toggleFrameInPrimTree,
                self._frameInPrimTreeOnSelect,
            ),
            (M.SEPARATOR,),
            (M.ACTION, "Open Container", None, self._openSelectedContainer),
        ]

    def _initContextMenu(self):
        menuConf = self._getContextMenuConf()
        builder = MenuBuilder(self)
        self._contextMenu, self._contextMenuActions = (
            builder.buildContextMenuWithActions(menuConf)
        )

        # Pre-warm the menu to force Qt to cache layout/rendering data
        # This significantly improves the first-time display performance
        # by triggering Qt's internal menu preparation once during initialization
        self._contextMenu.popup(QtCore.QPoint(-10000, -10000))
        self._contextMenu.hide()

    def _updateContextMenuState(self):
        actions = self._contextMenuActions

        hasSelection = len(self._selectedNodes) > 0
        hasLinkSelection = len(self._selectedLinks) > 0

        actions["Show Links"].setChecked(self.drawLinks)
        actions["Show Nodes"].setChecked(self.drawNodes)
        actions["Show Text"].setChecked(self.drawText)
        actions["Show Attribute Values"].setChecked(self._cachedShowAttributeValues)
        actions["Auto (key only if already animated)"].setChecked(
            self._cachedValueWriteMode == "auto"
        )
        actions["Default (the rest value)"].setChecked(
            self._cachedValueWriteMode == "default"
        )
        actions["Animation (key at current frame)"].setChecked(
            self._cachedValueWriteMode == "animation"
        )
        actions["Link Dimming"].setChecked(self.linkDimming.target > 0.0)

        actions["Frame Selection (F)"].setEnabled(hasSelection)
        actions["Frame Selection with Connections (Shift+F)"].setEnabled(hasSelection)
        actions["Clear Selection"].setEnabled(hasSelection or hasLinkSelection)

        actions["Nodes Only"].setChecked(
            self._linkSelectionMode == LinkSelectionMode.NODES_ONLY
        )
        actions["With All Links"].setChecked(
            self._linkSelectionMode == LinkSelectionMode.WITH_ALL_LINKS
        )
        actions["With Input Links"].setChecked(
            self._linkSelectionMode == LinkSelectionMode.WITH_INPUTS
        )
        actions["With Output Links"].setChecked(
            self._linkSelectionMode == LinkSelectionMode.WITH_OUTPUTS
        )

        hasLinkOrNodeSelection = hasLinkSelection or hasSelection
        actions["Navigate Backward ([)"].setEnabled(hasLinkOrNodeSelection)
        actions["Navigate Forward (])"].setEnabled(hasLinkOrNodeSelection)

        actions["Frame in Prim Tree on Select"].setChecked(
            self._frameInPrimTreeOnSelect
        )

        # Enable "Open Container" only if exactly one node is selected and it's a container
        canOpenContainer = self._canOpenSelectedContainer()
        actions["Open Container"].setEnabled(canOpenContainer)

    def contextMenuEvent(self, event):
        self._updateContextMenuState()
        self._contextMenu.exec_(event.globalPos())

    def _openSelectedContainer(self):
        """Open the selected container node in a new Noodles editor window."""
        if not self._canOpenSelectedContainer():
            Tf.Warn("Cannot open container: no single container node selected")
            return

        selectedNodeId = next(iter(self._selectedNodes))
        node = self.nodes[selectedNodeId]

        # Use the node's ID as the prim path (it should be a valid USD path)
        containerPath = node.id

        # Skip virtual nodes (they have # in their path)
        if "#" in containerPath:
            Tf.Warn(f"Cannot open virtual node as container: {containerPath}")
            return

        if not self._usdviewApi:
            Tf.Warn("Cannot open container: usdviewApi not available")
            return

        # Import here to avoid circular dependency
        from UsdNoodles import OpenNewNoodlesEditorForContainer

        try:
            OpenNewNoodlesEditorForContainer(self._usdviewApi, containerPath)
        except Exception as e:
            Tf.Warn(f"Error opening container: {e}")
            import traceback

            traceback.print_exc()

    def _openPreferences(self):
        """Open the preferences dialog."""
        if self._preferencesDlg is None:
            self._preferencesDlg = NoodlesPreferences(self, self)
            # Clean up when dialog closes
            self._preferencesDlg.finished.connect(self._onPreferencesClosed)

        self._preferencesDlg.show()
        self._preferencesDlg.raise_()
        self._preferencesDlg.activateWindow()

    def _onPreferencesClosed(self):
        """Called when preferences dialog closes."""
        # Keep instance for reuse, but could set to None to recreate each time
        pass

    def _drawNodeVertices(
        self,
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
        if not vertexData or not self.shaderLibrary:
            return

        shader = self.shaderLibrary.get("node")
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
            stride = NodeVertex.size()
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

    def resizeGL(self, w, h):
        self.makeCurrent()
        ctx = self.context()
        if ctx is None or not ctx.isValid():
            return
        GL.glViewport(0, 0, w, h)
        # Invalidate caches on resize
        self._nodeRenderManager.invalidateAll()
        self.doneCurrent()

    def paintGL(self):
        if not self.isVisible():
            return

        if not self.initialized:
            self.initializeGL()
            if not self.initialized:
                return

        self.makeCurrent()

        # Verify the GL context is actually current before issuing GL calls.
        # When docked alongside another QOpenGLWidget (e.g. usdview's Hydra
        # viewport), makeCurrent() can silently fail if the context was lost
        # or the surface is not yet valid.
        ctx = self.context()
        if ctx is None or not ctx.isValid():
            return

        # Detect stale GL context (e.g. after usdview "Reopen Stage").
        # The font atlas texture serves as a canary — if it is no longer
        # a valid texture name, the context was recreated and all cached
        # GL resources (VAOs, VBOs, shaders) are stale.
        if self.fontAtlas and self.fontAtlas.texture:
            if not GL.glIsTexture(self.fontAtlas.texture):
                self._reinitializeGL()
                if not self.initialized:
                    return

        try:
            self._paintGLInner()
        except GLError as e:
            Tf.Warn(f"GL error during paintGL: {e}")

    def _paintGLInner(self):
        # Enable profiling based on cached config
        self._profiler.enabled = self._cachedEnableRenderProfiling
        self._profiler.startFrame()

        with _ProfileSection(self._profiler, "Animator Update"):
            self._updateAnimator()

        with _ProfileSection(self._profiler, "Pan Animation"):
            self._updatePanAnimation()

        with _ProfileSection(self._profiler, "Keyboard State"):
            self._updateKeyboardState()

        # Apply any node-position changes to the spatial index / links / port
        # highlights before rendering or hit-testing this frame. O(1) when
        # nothing moved.
        with _ProfileSection(self._profiler, "Reconcile Moves"):
            self._reconcileMovedNodes()

        with _ProfileSection(self._profiler, "Rebuild Links"):
            if self.linksChanged:
                self._rebuildLinks()

        with _ProfileSection(self._profiler, "GL Clear"):
            GL.glClearColor(0.25, 0.25, 0.25, 1.0)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            GL.glDepthFunc(GL.GL_LEQUAL)

        # Rendering:

        with _ProfileSection(self._profiler, "Group Stickers"):
            # group stickers first (background layer, behind everything)
            if self.drawNodes and self.groupStickers:
                self.paintGroupStickers()

        with _ProfileSection(self._profiler, "Links"):
            # links (below nodes)
            if self.drawLinks:
                self.paintLinks()

        # Render temporary link during drag (after links, before nodes)
        if self._draggingLink and self._dragLinkTempLink:
            self._renderTemporaryLink()

        with _ProfileSection(self._profiler, "Nodes"):
            # nodes (on top of links)
            if self.drawNodes:
                self.paintNodes()

        with _ProfileSection(self._profiler, "Port Highlights"):
            if self.nodes:
                self._renderPortHighlightsCpp()

        with _ProfileSection(self._profiler, "Text"):
            # text (on top of nodes)
            if self.drawText:
                self.paintText()

        with _ProfileSection(self._profiler, "Value Cells"):
            # Inline attribute values. A separate Python pass, because node
            # text is generated and drawn entirely in C++ by the pinned
            # external noodles renderer -- there is no hook to add a string to
            # a row's layout. Same shape as the marquee and status overlay.
            if self.drawText and self._cachedShowAttributeValues:
                self._paintValueCells()

        with _ProfileSection(self._profiler, "Status Overlay"):
            # status overlay (screen space, on top of everything)
            self.paintStatusOverlay()

        with _ProfileSection(self._profiler, "Minimap"):
            # minimap (screen space, upper right corner)
            if self._cachedShowMinimap:
                self.minimap.paint(NodeVertex, MAX_RENDER_DEPTH)

        with _ProfileSection(self._profiler, "Hotkey Popups"):
            self._paintHotkeyPopups()

        with _ProfileSection(self._profiler, "Marquee"):
            # marquee selection (on top of everything else)
            if self.marqueeActive:
                self.paintMarquee()

        with _ProfileSection(self._profiler, "Node Creation Hotbox"):
            if self.nodeCreationHotbox.is_visible:
                self._paintNodeCreationHotbox()

        with _ProfileSection(self._profiler, "Node Rename"):
            if self._renamingNodeId:
                self._paintNodeRename()

        with _ProfileSection(self._profiler, "Value Editor"):
            if self._valueEditTarget:
                self._paintValueEditor()
            if self._tokenPopup:
                self._paintTokenPopup()
            if self._mungActive:
                self._paintMungLadder()

        self._profiler.endFrame()
        self._printProfilingStatsIfNeeded()

    def _updateAnimator(self):
        """Advance the animator by the elapsed frame time."""
        if not self.frameTimer.isValid():
            self.frameTimer.start()
            self.animator.update(0)
        else:
            nsecsElapsed = self.frameTimer.nsecsElapsed()
            self.frameTimer.restart()
            dt = nsecsElapsed / 1e9
            self.animator.update(dt)

    def _updatePanAnimation(self):
        """Update pan position from animation and check for completion."""
        if not self._panAnimating:
            return
        self.panX = self._panXAnim.current
        self.panY = self._panYAnim.current

        panXDone = abs(self._panXAnim.current - self._panXAnim.target) < 0.5
        panYDone = abs(self._panYAnim.current - self._panYAnim.target) < 0.5
        if panXDone and panYDone:
            self.panX = self._panXAnim.target
            self.panY = self._panYAnim.target
            self._panAnimating = False

    def _updateKeyboardState(self):
        """Poll keyboard modifiers and update animation state."""
        if self.animator.isAnimating():
            self.update()
        else:
            self.frameTimer.invalidate()

    def _renderPortHighlightsCpp(self):
        """C++-owned port-highlight pass.

        Delegates the entire port-circle / relationship-triangle assembly to
        NodeRenderManager.renderPortHighlightsFromGraph, which reads the
        authoritative GraphModel snapshot (synced in paintNodes/paintLinks) and
        the C++ connection index. This is the sole port-highlight render path.
        """
        graph = self.nodeGraph
        config = self._renderConfig()
        # Push the cached port appearance onto the shared render config (cheap and
        # idempotent); the C++ producer reads these instead of the Python caches.
        # These writes intentionally persist after useCppPortHighlights is toggled
        # off: the fields have no other consumers, so leaving them set is harmless.
        config.nodePortRingThickness = self._cachedPortRingThickness
        config.portConnectedFillColor = list(self._cachedPortConnectedFillColor)
        config.portConnectedRingColor = list(self._cachedPortConnectedRingColor)
        config.portDisconnectedFillColor = list(self._cachedPortDisconnectedFillColor)
        config.portDisconnectedRingColor = list(self._cachedPortDisconnectedRingColor)
        config.portHoverColor = list(self._cachedPortHoverColor)

        projList = list(self._worldSpaceProjectionMatrix().data())
        cornerRadius = config.nodePortWidth * 0.5

        hovered = self._hoveredPort
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
            self._portHighlightsDirty
            or self.textChanged
            or self.draggingNodes
            or self._draggingLink
        )
        self._nodeRenderManager.renderPortHighlightsFromGraph(
            graph,
            config,
            projList,
            cornerRadius,
            hovered is not None,
            hovered[0] if hovered else "",
            hovered[1] if hovered else "",
            bool(hovered[2]) if hovered else False,
            bool(self._draggingLink),
            self._dragLinkSourceNode or "",
            self._dragLinkSourcePort or "",
            bool(self._dragLinkSourceIsOutput),
            port_geometry_changed,
        )
        self._portHighlightsDirty = False

    def _paintHotkeyPopups(self):
        """Render hotkey popup overlays."""
        if self.hotkeyPopup.is_visible:
            self.paintHotkeyPopup()
        if self.navigationHotkeyPopup.is_visible:
            self.paintNavigationHotkeyPopup()

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

    def _worldSpaceProjectionMatrix(self):
        # Reuse the cached matrix unless the view (pan/zoom/size) changed since
        # it was built; the key comparison makes this self-invalidating.
        key = (self.panX, self.panY, self.zoom, self.width(), self.height())
        if key != self._worldProjCacheKey:
            projection = QtGui.QMatrix4x4()
            scaledWidth = self.width() / self.zoom
            scaledHeight = self.height() / self.zoom
            projection.ortho(
                self.panX,
                self.panX + scaledWidth,
                self.panY + scaledHeight,
                self.panY,
                -MAX_RENDER_DEPTH,
                MAX_RENDER_DEPTH,
            )
            self._worldProjCacheKey = key
            self._worldProjCache = projection
        return self._worldProjCache

    def _screenSpaceProjectionMatrix(self):
        key = (self.width(), self.height())
        if key != self._screenProjCacheKey:
            projection = QtGui.QMatrix4x4()
            projection.ortho(
                0, self.width(), self.height(), 0, -MAX_RENDER_DEPTH, MAX_RENDER_DEPTH
            )
            self._screenProjCacheKey = key
            self._screenProjCache = projection
        return self._screenProjCache

    @_guardsQtBoundary
    def wheelEvent(self, event):
        # If hotbox is visible, scroll the list instead of zooming
        if self.nodeCreationHotbox.is_visible:
            # Pixels first, degrees second.
            #
            # A macOS trackpad sends high-resolution pixelDelta, and the OS
            # keeps sending events after the fingers lift -- that IS the
            # momentum, so honouring the pixels gives native inertia for
            # free. The old `-angleDelta // 120` threw all of it away: a
            # trackpad's angleDelta arrives in single digits, integer-divided
            # by a 120-unit wheel notch to exactly 0, so nothing moved until
            # enough events had been discarded to cross a notch and the list
            # jumped a whole row. That is the choppiness.
            pixels = event.pixelDelta().y()
            if pixels:
                self.nodeCreationHotbox.handle_scroll_pixels(-float(pixels))
            else:
                # A real wheel (or any device without pixel resolution):
                # 120 units is one notch, three rows, as before.
                degrees = event.angleDelta().y()
                self.nodeCreationHotbox.handle_scroll_pixels(
                    -float(degrees) / 120.0 * 3.0
                    * self.nodeCreationHotbox.row_height())
            self.update()
            return

        mousePos = event.position()
        delta = event.angleDelta().y()
        # Use multiplicative zoom factor for smoother, bounded zooming
        # Each wheel notch (typically 120 units) zooms by this factor
        zoomFactor = 1.15  # 15% zoom per wheel notch
        steps = delta / 120.0  # Normalize to wheel notches
        newZoom = self.zoom * pow(zoomFactor, steps)
        self._zoomAroundPoint(newZoom, (mousePos.x(), mousePos.y()))
        self.update()




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

        worldMousePos = (
            QtCore.QPointF(event.position().toPoint()) / self.zoom
        ) + QtCore.QPointF(self.panX, self.panY)
        worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

        # Check if we clicked on a connected port that has a dangling link.
        # _findDanglingLinkAtPort by construction only returns indices of
        # links with isDangling=True, so we don't re-check the flag here.
        portDanglingIdx = self._findDanglingLinkAtPort(worldPos)
        if portDanglingIdx >= 0 and portDanglingIdx < len(self.links):
            self._addNodesFromDanglingLink(self.links[portDanglingIdx])
            return

        # Also check via curve hit-test for backward compatibility
        clickedLinkIndex = self.linkRenderer.findLinkUnderCursor(
            worldPos,
            self.links,
            self.zoom,
            self._cachedLinkLineWidth,
            spatialIndex=self._spatialIndex,
            debug=False,
        )

        if clickedLinkIndex >= 0 and clickedLinkIndex < len(self.links):
            link = self.links[clickedLinkIndex]
            if link.isDangling:
                self._addNodesFromDanglingLink(link)
                return
            if getattr(link, "is_relationship_link", False):
                if self._navigateAlongRelationshipLink(link, worldPos):
                    return

        # A node under the cursor: rename it. Last, so the link and port cases
        # above keep the meaning they already had -- they hit-test geometry
        # that sits on top of a node, and stealing their double-click would
        # cost the only way to follow a dangling link.
        nodeId = self._getNodeIdAtWorldPos(worldPos)
        if nodeId:
            self._beginRename(nodeId)
            return

        # Default behavior for double-click on non-dangling elements
        super().mouseDoubleClickEvent(event)

    @_guardsQtBoundary
    def mousePressEvent(self, event):
        self.setFocus()
        # Clicking away commits, which is what a text field everywhere else
        # does. Committing before the click is dispatched matters: the click
        # may land on another node, and that node's selection should follow
        # the rename rather than race it.
        if self._renamingNodeId:
            self._commitRename()
        # An open token popup owns the click: inside it picks, outside closes.
        # Either way the press stops here, so a pick cannot also select a node.
        if self._tokenPopup:
            self._handleTokenPopupPress(event.position().toPoint())
            self.update()
            return
        # A value editor commits on click-away, the same as the rename above
        # and as every text field everywhere else.
        if self._valueEditTarget:
            self._commitValueEdit()
        self.lastMousePos = event.position().toPoint()
        self.ctrlPressedOnMouseDown = event.modifiers() & QtCore.Qt.ControlModifier
        self.mousePosOnMouseDown = event.position().toPoint()
        # Resolve any unpainted position change before hit-testing what is under
        # the cursor (e.g. a programmatic move then an immediate click). O(1) when
        # nothing moved.
        self._reconcileMovedNodes()

        # If hotbox is visible, check if click is within hotbox
        if self.nodeCreationHotbox.is_visible:
            if self.nodeCreationHotbox.handle_mouse_press(event.position().toPoint()):
                self.update()
                return

        if event.buttons() & QtCore.Qt.LeftButton and self._cachedShowMinimap:
            if self.minimap.handleMousePress(event.position().toPoint()):
                return

        self._updateNodeUnderCursor(event.position().toPoint())

        if event.buttons() & QtCore.Qt.LeftButton and not self.spacePressed:
            # Check if clicking on the title bar caret to toggle title collapse
            if self.nodeIdUnderCursor and self.nodeIdUnderCursor in self.nodes:
                clickedNodeForTitle = self.nodes[self.nodeIdUnderCursor]
                if clickedNodeForTitle.renderer:
                    worldMousePos = (
                        QtCore.QPointF(event.position().toPoint()) / self.zoom
                    ) + QtCore.QPointF(self.panX, self.panY)
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
                        if self.fontAtlas:
                            self.nodeGraph._calculateNodeSize(
                                clickedNodeForTitle,
                                self.textRenderer.calculateTextWidth,
                                self.fontAtlas,
                            )
                        self.textChanged = True
                        self.linksChanged = True
                        self._nodeRenderManager.invalidateAll()
                        self.update()
                        return

            # Check if clicking on a group header to toggle fold
            if self.nodeIdUnderCursor and self.nodeIdUnderCursor in self.nodes:
                clickedNodeForFold = self.nodes[self.nodeIdUnderCursor]
                if clickedNodeForFold.renderer:
                    worldMousePos = (
                        QtCore.QPointF(event.position().toPoint()) / self.zoom
                    ) + QtCore.QPointF(self.panX, self.panY)
                    worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
                    headerResult = clickedNodeForFold.renderer.getGroupHeaderAtPoint(
                        clickedNodeForFold, worldPos, self.fontAtlas
                    )
                    if headerResult and headerResult["found"]:
                        headerName = headerResult["portName"]
                        isOutput = headerResult["isOutput"]
                        clickedNodeForFold.toggle_fold(headerName, isOutput)
                        if self.fontAtlas:
                            self.nodeGraph._calculateNodeSize(
                                clickedNodeForFold,
                                self.textRenderer.calculateTextWidth,
                                self.fontAtlas,
                            )
                        self.textChanged = True
                        self.linksChanged = True
                        self._nodeRenderManager.invalidateAll()
                        self.update()
                        return

            # A value cell claims the press before anything else on the node.
            #
            # It is checked ahead of the hovered-port branch and well ahead of
            # _findRowEdgeDragStart, but the ordering is belt and braces: the
            # cell rect stops a full PAD short of the connection gutter
            # (widgets/valueCell.cell_rect), so the two regions cannot both
            # contain one point. Returning here is what guarantees a press on
            # a value never degenerates into a connection drag even if a future
            # layout change broke that inequality.
            valueHit = self._valueCellAtPoint(
                (
                    QtCore.QPointF(event.position().toPoint()) / self.zoom
                )
                + QtCore.QPointF(self.panX, self.panY)
            )
            if valueHit is not None:
                self._onValueCellPressed(valueHit, event)
                self.update()
                return

            # Check if clicking on a port to start link creation
            if self._hoveredPort:
                nodeId, portName, isOutput = self._hoveredPort
                # Start link drag
                self._startLinkDrag(nodeId, portName, isOutput)
                self.update()
                return

            worldMousePosForLink = (
                QtCore.QPointF(event.position().toPoint()) / self.zoom
            ) + QtCore.QPointF(self.panX, self.panY)
            worldPosForLink = Gf.Vec2d(
                worldMousePosForLink.x(), worldMousePosForLink.y()
            )

            # Grab a relationship arrowhead to reconnect it (or drop on empty to
            # disconnect). Checked before node selection so an arrowhead that
            # overlaps its target node is still grabbable.
            arrowheadLinkIndex = self._findArrowheadUnderCursor(worldPosForLink)
            if arrowheadLinkIndex >= 0:
                self.clearSelection()
                self.clearLinkSelection()
                self._startLinkReconnect(arrowheadLinkIndex, worldPosForLink)
                self.update()
                return

            # Start a new connection by dragging from a row's left/right edge
            # band (the outer tenth of each side), even with no visible pin.
            # Presses in the middle of the row, on the title, or below the last
            # row fall through to node selection/drag.
            if self.nodeIdUnderCursor and self.nodeIdUnderCursor in self.nodes:
                edgeNode = self.nodes[self.nodeIdUnderCursor]
                edgeHit = self._findRowEdgeDragStart(edgeNode, worldPosForLink)
                if edgeHit:
                    self._startLinkDrag(self.nodeIdUnderCursor, edgeHit[0], edgeHit[1])
                    self.update()
                    return

            clickedNode = None
            if self.nodeIdUnderCursor and self.nodeIdUnderCursor in self.nodes:
                clickedNode = self.nodes[self.nodeIdUnderCursor]

            if clickedNode:
                if not self.ctrlPressedOnMouseDown:
                    if not clickedNode.selected:
                        self.clearSelection()
                        self.clearLinkSelection()
                    _selection.select_node(self, clickedNode.id)
                else:
                    _selection.select_node(self, clickedNode.id, toggle=True)
                self.draggingNodes = True

                # Bring the clicked node to front. Node-background quads are
                # regenerated from node.zOrder every frame, so the box raises
                # for free; for text we patch only this node's depth in the held
                # vertex buffer (O(node glyphs)). We must NOT set
                # self.textChanged here — that re-lays out every node's text and
                # cost ~2s for a dozen nodes due to the O(N^2) string-by-string
                # Python<->C++ vertex marshalling.
                self._nextZOrder += 1
                clickedNode.zOrder = self._nextZOrder
                self.textRenderer.patchNodeTextDepth(clickedNode)
                # Node-quad depth is baked at generation; re-bumping z-order needs
                # a regen (the selection-set check above misses a re-click on an
                # already-selected node).
                self._nodeRenderManager.markNodeQuadDirty(True)
                # The title icon's depth is also baked at generation, but the icon
                # path only rebuilds on a content change (which selection skips for
                # perf). Without this, the icon stays at its old depth and the
                # re-raised node quad occludes it (the icon visibly drops on
                # select). markIconsDirty re-bakes it at the new zOrder next frame,
                # reading the snapshot the node-quad path re-syncs this same frame,
                # so the icon rides to front with the node.
                self._cppIconRenderer.markIconsDirty()

                # Begin drag on all selected nodes for batched USD writes
                for node in self.nodes.values():
                    if node.selected:
                        node.beginDrag()

                # Update link selection based on current mode
                self._updateLinkSelectionFromNodes()
            else:
                worldMousePos = (
                    QtCore.QPointF(event.position().toPoint()) / self.zoom
                ) + QtCore.QPointF(self.panX, self.panY)
                worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

                clickedLinkIndex = self.linkRenderer.findLinkUnderCursor(
                    worldPos,
                    self.links,
                    self.zoom,
                    self._cachedLinkLineWidth,
                    spatialIndex=self._spatialIndex,
                    debug=False,
                )

                if clickedLinkIndex >= 0:
                    if self.ctrlPressedOnMouseDown:
                        self._toggleLinkSelection(clickedLinkIndex)
                    else:
                        clickedLink = self.links[clickedLinkIndex]
                        # Dangling links are selectable on click
                        # Regular and relationship links start reconnect on click
                        if clickedLink.isDangling:
                            self.clearSelection()
                            self._selectLink(clickedLinkIndex, addToSelection=False)
                        else:
                            self.clearSelection()
                            self.clearLinkSelection()
                            self._startLinkReconnect(clickedLinkIndex, worldPos)
                else:
                    if not self.ctrlPressedOnMouseDown:
                        self.clearSelection()
                        self.clearLinkSelection()
                        # Do not sync empty selection back to prim tree — the user
                        # may press A next to add the still-selected prim(s).
                    self._marqueeBase = self._selection.copy()
                    self._marqueeMode = "union" if self.ctrlPressedOnMouseDown else "replace"
                    self.marqueeActive = True
                    self.marqueeStart = worldMousePos
                    self.marqueeCurrent = worldMousePos

        self.update()

    @_guardsQtBoundary
    def mouseMoveEvent(self, event):
        # If hotbox is visible, update hover state
        if self.nodeCreationHotbox.is_visible:
            if self.nodeCreationHotbox.handle_mouse_move(event.position().toPoint()):
                self.update()
                return

        if self._cachedShowMinimap:
            self.minimap.updateHoverState(event.position().toPoint())

            if self.minimap.handleMouseMove(event.position().toPoint()):
                self.lastMousePos = event.position().toPoint()
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

        # A value drag owns the pointer: no hover work, no node drag, no link
        # drag. Checked first so none of the scans below run per sample.
        if self._mungState is not None and (event.buttons() & QtCore.Qt.LeftButton):
            self._updateMung(event)
            self.lastMousePos = event.position().toPoint()
            self.update()
            return

        if not isViewportDrag:
            self._updateNodeUnderCursor(event.position().toPoint())
            self._updateValueHoverCursor(event.position().toPoint())

            if not self.draggingNodes:
                if self._updateHoveredPort(event.position().toPoint()):
                    self.update()

            if not self.draggingNodes and not self._draggingLink:
                # Update property tooltip on hover
                self._updatePropertyTooltip(event.position().toPoint())

                # Update link under cursor for hover highlighting
                worldMousePos = (
                    QtCore.QPointF(event.position().toPoint()) / self.zoom
                ) + QtCore.QPointF(self.panX, self.panY)
                worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())

                newLinkUnderCursor = self.linkRenderer.findLinkUnderCursor(
                    worldPos,
                    self.links,
                    self.zoom,
                    self._cachedLinkLineWidth,
                    spatialIndex=self._spatialIndex,
                )
                if newLinkUnderCursor != self._linkUnderCursor:
                    self._linkUnderCursor = newLinkUnderCursor
                    self.update()

        # Handle link dragging
        if self._draggingLink:
            self._updateLinkDrag(event.position().toPoint())
            self.update()
            return

        if event.buttons() & QtCore.Qt.MiddleButton or (
            self.spacePressed and event.buttons() & QtCore.Qt.LeftButton
        ):
            if self.ctrlPressedOnMouseDown:
                newZoom = self.zoom * (
                    1.0
                    + (event.position().toPoint().x() - self.lastMousePos.x()) / 120.0
                )
                focus = (self.mousePosOnMouseDown.x(), self.mousePosOnMouseDown.y())
                self._zoomAroundPoint(newZoom, focus)
            else:
                self.panX -= (
                    event.position().toPoint().x() - self.lastMousePos.x()
                ) / self.zoom
                self.panY -= (
                    event.position().toPoint().y() - self.lastMousePos.y()
                ) / self.zoom
            self.update()
            self.lastMousePos = event.position().toPoint()
            return

        if event.buttons() & QtCore.Qt.LeftButton:
            if self.marqueeActive:
                worldMousePos = (
                    QtCore.QPointF(event.position().toPoint()) / self.zoom
                ) + QtCore.QPointF(self.panX, self.panY)
                self.marqueeCurrent = worldMousePos
                self._updateMarqueeSelection()
                self.update()
                return

            if self.draggingNodes:
                dx = (
                    event.position().toPoint().x() - self.lastMousePos.x()
                ) / self.zoom
                dy = (
                    event.position().toPoint().y() - self.lastMousePos.y()
                ) / self.zoom

                movedNodeIds = set()
                for node in self.nodes.values():
                    if node.selected:
                        node.position = node.position + Gf.Vec2d(dx, dy)
                        movedNodeIds.add(node.id)
                        # Cheap render-offset move (text + quad ride one frame).
                        self._nodeTransformFrame.translate(node.id, dx, dy)

                # Refresh derived state for the dragged nodes (the reconcile is
                # skipped while dragging). This does not advance the reconcile
                # watermark, so the release frame reconciles these nodes once more
                # from their final positions.
                self._applyMovedNodeDerivedState(movedNodeIds)

        self.lastMousePos = event.position().toPoint()
        self.update()

    @_guardsQtBoundary
    def mouseReleaseEvent(self, event):
        # Release the hotbox scrollbar first. It is the only overlay that
        # takes a press-drag-release, and the handlers below would otherwise
        # treat the release as ending a marquee or a link drag that never
        # started.
        if self.nodeCreationHotbox.is_visible:
            if self.nodeCreationHotbox.handle_mouse_release(
                    event.position().toPoint()):
                self.update()
                return
        # Finish a value press: a drag commits as ONE undo entry, a click with
        # no motion opens the keyboard editor instead. Before the link-drag
        # branch because a value press never started one.
        if self._mungState is not None:
            self._finishMung()
            self.update()
            return
        # Handle link drag completion
        if self._draggingLink:
            # Refresh hover state at the release position so that
            # _completeLinkDrag sees the actual cursor location.  On Windows,
            # Qt may not emit a final mouseMoveEvent before the release,
            # leaving _hoveredPort / _dragLinkValidTarget stale from a
            # previous frame — which prevents _finishReconnectDisconnect
            # from running and skips the "Disconnect" undo entry.
            releasePos = event.position().toPoint()
            self._updateHoveredPort(releasePos)
            self._updateLinkDrag(releasePos)
            self._completeLinkDrag()
            self._draggingLink = False
            self._dragLinkSyntheticTarget = None
            self._dragLinkResolvedTarget = None
            self.update()
            return

        # Always release a minimap viewport drag, even if the minimap was hidden
        # mid-drag: handleMouseRelease() is a no-op when no drag is in progress,
        # but if "Show Minimap" is toggled off after a drag starts, gating this on
        # _cachedShowMinimap would strand _viewportDragging=True and let a later
        # move hijack a pan once the minimap is shown again.
        if self.minimap.handleMouseRelease():
            return

        # End drag on all selected nodes - writes final positions to USD session layer
        if self.draggingNodes:
            self._finalizeNodeDrag()
            # Final node positions were just baked into the snapshot; rebuild the
            # port highlights once so the circles match (the drag itself kept them
            # live via the geometryChanged signal while draggingNodes was True).
            self._portHighlightsDirty = True

        self.draggingNodes = False

        if self.marqueeActive:
            self._updateMarqueeSelection()
            self.marqueeActive = False
            self._marqueeBase = _selection.Selection()
            if self._selectedLinks and not self._selectedNodes:
                self._syncLinkSelectionToPrimtree()
            self.update()

        hasSelectedNodes = any(node.selected for node in self.nodes.values())

        if hasSelectedNodes:
            self._syncSelectionToPrimtree()




    def _finalizeNodeDrag(self):
        """Finalize a node drag: write positions to USD and push undo command."""
        # Capture old positions (pre-drag) and new positions (post-drag)
        # before calling endDrag() which writes to USD.
        old_positions = {}
        new_positions = {}
        for nid, node in self.nodes.items():
            if node.selected and node._dragging:
                old_positions[nid] = (node._position[0], node._position[1])
                if node._dragPosition is not None:
                    new_positions[nid] = (
                        node._dragPosition[0],
                        node._dragPosition[1],
                    )

        def finish():
            for node in self.nodes.values():
                if node.selected:
                    node.endDrag()

        if new_positions and getattr(self, "_editor", None) is not None:
            _editing.edit_node_positions(self, new_positions, "Move Nodes", finish,
                                         old_display=old_positions)
            return
        self._noticeHandler.setEnabled(False)
        try:
            finish()
        finally:
            self._noticeHandler.setEnabled(True)

        # Push undo command if positions actually changed
        Tf.Status(
            f"_finalizeNodeDrag: old_positions={len(old_positions)}, "
            f"new_positions={len(new_positions)}"
        )
        if old_positions and new_positions:
            _push_undo_command(
                "Move Nodes",
                lambda gv, p=new_positions: gv._restoreNodePositions(p),
                lambda gv, p=old_positions: gv._restoreNodePositions(p),
                view=self,
            )

    @_guardsQtBoundary
    def keyPressEvent(self, event):
        # Value interaction is checked FIRST, ahead of even the Tab/hotbox
        # branch below: Tab moves to the next editable cell while an editor is
        # open, and Esc abandons a drag. Both keys already mean something else
        # in this widget, so whoever is in the middle of an interaction has to
        # be asked before the global meanings get a turn.
        #
        # Esc during a value drag restores the value the drag started from and
        # leaves the undo stack exactly as if the drag had never happened.
        if self._mungState is not None:
            if event.key() == QtCore.Qt.Key_Escape:
                self._cancelMung()
                self.update()
                return

        # The token popup owns the keyboard while it is open.
        if self._tokenPopup:
            if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
                self._commitTokenPopup()
            elif event.key() == QtCore.Qt.Key_Escape:
                self._closeTokenPopup()
            elif event.key() == QtCore.Qt.Key_Up:
                self._tokenPopupList.move_selection(-1)
            elif event.key() == QtCore.Qt.Key_Down:
                self._tokenPopupList.move_selection(1)
            self.update()
            return

        # A value editor owns ALL of the keyboard, for the same reason the
        # rename below does: the shortcuts further down are single letters,
        # and typing 0.5 into a cell must not toggle link dimming.
        if self._valueEditTarget:
            key = event.key()
            if key in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
                self._commitValueEdit()
            elif key == QtCore.Qt.Key_Escape:
                self._cancelValueEdit()
            elif key in (QtCore.Qt.Key_Tab, QtCore.Qt.Key_Backtab):
                backwards = key == QtCore.Qt.Key_Backtab or bool(
                    event.modifiers() & QtCore.Qt.ShiftModifier
                )
                self._advanceValueEdit(backwards)
            else:
                if (
                    self._valueEditFresh
                    and event.text()
                    and event.text().isprintable()
                ):
                    # Emulates the select-all a real line edit does on focus.
                    self._valueEditInput.clear()
                self._valueEditFresh = False
                self._valueEditInput.handle_key(event)
            self.update()
            return

        # Tab key - toggle hotbox
        if event.key() == QtCore.Qt.Key_Tab:
            if self.nodeCreationHotbox.is_showing:
                self.nodeCreationHotbox.hide()
            else:
                self.nodeCreationHotbox.show(self.lastMousePos)
            self.update()
            return

        # When hotbox is showing, delegate key handling to it
        if self.nodeCreationHotbox.is_showing:
            if self.nodeCreationHotbox.handle_key(event):
                self.update()
                return

        # A rename in progress owns the keyboard, and owns ALL of it: the
        # shortcuts below are single letters, so letting an unhandled key fall
        # through would make typing a name called Z undo the graph.
        if self._renamingNodeId:
            if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
                self._commitRename()
            elif event.key() == QtCore.Qt.Key_Escape:
                self._cancelRename()
            else:
                self._renameInput.handle_key(event)
            self.update()
            return

        if event.key() == QtCore.Qt.Key_Space:
            self.spacePressed = True
            self.setCursor(QtCore.Qt.OpenHandCursor)
        elif event.key() == QtCore.Qt.Key_Z:
            modifiers = event.modifiers()
            if modifiers & QtCore.Qt.ControlModifier:
                if modifiers & QtCore.Qt.ShiftModifier:
                    self._performRedo()
                else:
                    self._performUndo()
                return
        elif event.key() == QtCore.Qt.Key_Y:
            if event.modifiers() & QtCore.Qt.ControlModifier:
                self._performRedo()
                return
        # 1-9 for selecting links by pin index
        # Shift+number reverses the direction (inputs instead of outputs, or vice versa)
        elif event.key() >= QtCore.Qt.Key_1 and event.key() <= QtCore.Qt.Key_9:
            pinIndex = event.key() - QtCore.Qt.Key_1  # 0-based index
            reverseDirection = bool(event.modifiers() & QtCore.Qt.ShiftModifier)
            self.selectLinkByPinIndex(pinIndex, reverseDirection)

        # Delete or Backspace: Delete all selected items at once
        elif event.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
            hasLinks = bool(self._selectedLinks)
            hasNodes = bool(self._selectedNodes)
            if hasLinks or hasNodes:
                registry = getattr(self, "_commandRegistry", None)
                if registry is None:
                    self._deleteAllSelected()
                else:
                    registry.dispatch(_intents.DeleteSelection())
            return

        # L: auto-layout the whole graph (plain key, no Ctrl/Alt/Meta).
        elif event.key() == QtCore.Qt.Key_L and not (
            event.modifiers()
            & (
                QtCore.Qt.ControlModifier
                | QtCore.Qt.AltModifier
                | QtCore.Qt.MetaModifier
            )
        ):
            self._autoLayoutNodes()
            event.accept()
            return

        # Fallback: dispatch to configurable shortcut QActions.
        # _shouldInterceptShortcut intercepts ShortcutOverride for all keys
        # that match a configured shortcut, which bypasses Qt's shortcut
        # system. QAction shortcuts don't fire in that case, so we manually
        # match and trigger them here.
        if hasattr(self, "_shortcutActions"):
            event_key = int(event.key()) | (
                event.modifiers().value & ~QtCore.Qt.KeypadModifier.value
            )
            for _configKey, action in self._shortcutActions.items():
                shortcut = action.shortcut()
                if shortcut.isEmpty():
                    continue
                combo = shortcut[0]
                combo_int = (
                    combo.toCombined() if hasattr(combo, "toCombined") else int(combo)
                )
                if combo_int == event_key:
                    action.trigger()
                    event.accept()
                    return

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

    @_guardsQtBoundary
    def keyReleaseEvent(self, event):
        if self._renamingNodeId:
            return
        if event.key() == QtCore.Qt.Key_Space:
            self.spacePressed = False
            self.setCursor(QtCore.Qt.ArrowCursor)

    def _shouldInterceptShortcut(self, event):
        """Check if a ShortcutOverride event should be intercepted by the graph view.

        Prevents usdview's ApplicationShortcut-scoped actions (e.g. 'J' for
        Toggle Framed View) from firing while Noodles has focus.
        """
        # When the hotbox is showing, intercept ALL keys so they reach
        # the text input instead of triggering usdview shortcuts
        if self.nodeCreationHotbox.is_showing:
            return True

        # Tab is always intercepted (toggles the hotbox). Tab is not a
        # printable character so the check below won't catch it.
        if event.key() == QtCore.Qt.Key_Tab:
            return True

        # An interaction that owns the keyboard owns its NON-PRINTABLE keys
        # too. Without this, Qt takes the ShortcutOverride we declined as
        # permission to route the key elsewhere -- Escape goes to the
        # enclosing QDockWidget and the KeyPress never reaches
        # keyPressEvent at all, so "Esc cancels" silently does nothing. The
        # printable-key branch below cannot cover these: none of them
        # produce text.
        if (
            self._renamingNodeId
            or self._valueEditTarget
            or self._mungState is not None
            or self._tokenPopup
        ):
            if event.key() in (
                QtCore.Qt.Key_Escape,
                QtCore.Qt.Key_Return,
                QtCore.Qt.Key_Enter,
                QtCore.Qt.Key_Backtab,
                QtCore.Qt.Key_Backspace,
                QtCore.Qt.Key_Delete,
                QtCore.Qt.Key_Up,
                QtCore.Qt.Key_Down,
                QtCore.Qt.Key_Left,
                QtCore.Qt.Key_Right,
                QtCore.Qt.Key_Home,
                QtCore.Qt.Key_End,
            ):
                return True

        # Intercept unmodified (or Shift-only) printable keys to prevent
        # usdview's ApplicationShortcut-scoped actions from firing while
        # Noodles has focus. Modifier combos (Ctrl+X, Alt+X) are left
        # alone so they can still reach usdview or QAction shortcuts,
        # UNLESS they match a configured Noodles shortcut.
        modifiers = event.modifiers() & ~QtCore.Qt.KeypadModifier
        if modifiers in (QtCore.Qt.NoModifier, QtCore.Qt.ShiftModifier):
            text = event.text()
            if text and text.isprintable():
                return True

        # Intercept keys that match configurable Noodles shortcuts (including
        # Ctrl/Alt combos) so they reach keyPressEvent instead of being
        # captured by usdview's ApplicationShortcut-scoped QActions.
        if hasattr(self, "_shortcutActions"):
            event_key = int(event.key()) | (
                event.modifiers().value & ~QtCore.Qt.KeypadModifier.value
            )
            for action in self._shortcutActions.values():
                shortcut = action.shortcut()
                if shortcut.isEmpty():
                    continue
                combo = shortcut[0]
                combo_int = (
                    combo.toCombined() if hasattr(combo, "toCombined") else int(combo)
                )
                if combo_int == event_key:
                    return True

        return False

    def _isStageViewMidTeardown(self):
        if not self._usdviewApi:
            return False
        try:
            controller = self._usdviewApi._UsdviewApi__appController
            sv = getattr(controller, "_stageView", None)
            return sv is not None and not sv.updatesEnabled()
        except AttributeError:
            return False

    def event(self, event):
        if event.type() == QtCore.QEvent.Paint:
            if self._isStageViewMidTeardown():
                return True
        if event.type() == QtCore.QEvent.ShortcutOverride:
            if self._shouldInterceptShortcut(event):
                event.accept()
                return True
        if event.type() == QtCore.QEvent.HoverMove:
            self.lastMousePos = event.position().toPoint()
            self._updateNodeUnderCursor(self.lastMousePos)
            if not self.hasFocus():
                self.setFocus()
            self.update()
        try:
            return super().event(event)
        except RuntimeError as e:
            Tf.Warn(f"RuntimeError in event handling: {e}")
            return False

    def _startLinkReconnect(self, linkIndex, worldClickPos):
        """Disconnect a link from its nearest end and start dragging to reconnect.

        The link is removed visually (local graph only) while the user drags.
        USD is not modified until the drag completes, so cancelling restores
        the original connection and a successful reconnection is a single undo.
        """
        link = self.links[linkIndex]

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

        self._reconnectingLink = True
        self._reconnectCapture = _editing._capture_reconnect(self, link)
        self._reconnectOldSourceNodeId = link.sourceNodeId
        self._reconnectOldSourcePort = link.sourcePort
        self._reconnectOldTargetNodeId = link.targetNodeId
        self._reconnectOldTargetPort = link.targetPort
        self._reconnectOldSourcePropertyName = str(
            getattr(link, "sourcePropertyName", "")
        )
        self._reconnectOldTargetPropertyName = str(
            getattr(link, "targetPropertyName", "")
        )

        self._removeLinkFromGraph(
            link.sourceNodeId,
            link.sourcePort,
            link.targetNodeId,
            link.targetPort,
            sourcePropertyName=str(getattr(link, "sourcePropertyName", "")),
            targetPropertyName=str(getattr(link, "targetPropertyName", "")),
        )

        if anchorNodeId not in self.nodes:
            self._cancelLinkReconnect()
            return

        self._startLinkDrag(anchorNodeId, anchorPort, anchorIsOutput)

        if anchorIsOutput:
            self._dragLinkTempLink.end = worldClickPos
        else:
            self._dragLinkTempLink.start = worldClickPos

    def _clearReconnectState(self):
        """Zero reconnect bookkeeping fields."""
        self._reconnectingLink = False
        self._reconnectOldSourceNodeId = None
        self._reconnectOldSourcePort = None
        self._reconnectOldTargetNodeId = None
        self._reconnectOldTargetPort = None
        self._reconnectOldSourcePropertyName = ""
        self._reconnectOldTargetPropertyName = ""
        self._reconnectCapture = None

    def _cancelLinkReconnect(self):
        """Restore the original link when reconnection fails (error recovery)."""
        srcId = self._reconnectOldSourceNodeId
        srcPort = self._reconnectOldSourcePort
        tgtId = self._reconnectOldTargetNodeId
        tgtPort = self._reconnectOldTargetPort
        srcPropName = self._reconnectOldSourcePropertyName
        tgtPropName = self._reconnectOldTargetPropertyName
        capture = getattr(self, "_reconnectCapture", None)
        self._clearReconnectState()
        if isinstance(capture, _editing.ReconnectCapture):
            _editing._refresh_connections(self, capture.order)
            return

        srcNode = self.nodes.get(srcId)
        tgtNode = self.nodes.get(tgtId)
        if srcNode and tgtNode:
            self._addLinkToGraph(
                srcId,
                srcPort,
                tgtId,
                tgtPort,
                srcNode,
                tgtNode,
                sourcePropertyName=srcPropName,
                targetPropertyName=tgtPropName,
            )

    def _finishReconnectDisconnect(self):
        """Delete the original connection when dropped on empty space."""
        if isinstance(self.nodeGraph.getStage(), Usd.Stage):
            capture = _editing._reconnect_endpoints(self)
            self._clearReconnectState()
            return _editing._dispatch_connection(self, _intents.Disconnect(
                capture.endpoints, description="Disconnect", prior_order=capture.order))
        srcId = self._reconnectOldSourceNodeId
        srcPort = self._reconnectOldSourcePort
        tgtId = self._reconnectOldTargetNodeId
        tgtPort = self._reconnectOldTargetPort

        inputNode = self.nodes.get(tgtId)
        outputNode = self.nodes.get(srcId)
        if not inputNode or not outputNode:
            self._cancelLinkReconnect()
            return

        inputPrim = inputNode.getUsdPrim()
        outputPrim = outputNode.getUsdPrim()
        if not inputPrim or not outputPrim:
            self._cancelLinkReconnect()
            return

        library = self._findLibraryForPrim(inputPrim)
        if not library:
            self._cancelLinkReconnect()
            return

        self._clearReconnectState()

        stage = self.nodeGraph.getStage()
        warn_if_non_persistent_edit_target(stage)

        self._noticeHandler.setEnabled(False)
        try:
            success = library.delete_connection(
                stage, outputPrim, srcPort, inputPrim, tgtPort
            )
            if success:
                delete_fn = _make_connection_edit(
                    srcId, srcPort, tgtId, tgtPort, create=False
                )
                create_fn = _make_connection_edit(
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

                _push_undo_command("Disconnect", redo, undo, view=self)

                # Re-derive links from USD so the disconnected pin's port
                # highlight (the relationship arrow / port circle) reflects the
                # now-empty target. GetTargets is the source of truth; the
                # in-place removal done at reconnect-start is not guaranteed to
                # match relationship links (their propertyName fields can differ
                # from the record), which would otherwise leave a stale link in
                # self.links keeping the port "connected" (arrow stuck yellow).
                # Mirrors the not-found branch below and _deleteConnection.
                self.linksChanged = True
                self._rebuildLinks()
                self.update()
            else:
                # Connection not found in USD — it may have been replaced by
                # another connection to the same pin. Rebuild links from USD
                # so the UI reflects the actual stage state.
                self.linksChanged = True
                self._rebuildLinks()
        finally:
            self._noticeHandler.setEnabled(True)

    def _startLinkDrag(self, nodeId, portName, isOutput):
        """
        Begin dragging a new link from a port.

        Args:
            nodeId: ID of the source node
            portName: Name of the source port
            isOutput: True if dragging from output port, False for input port
        """
        self._draggingLink = True
        self._dragLinkSourceNode = nodeId
        self._dragLinkSourcePort = portName
        self._dragLinkSourceIsOutput = isOutput
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None
        _start_node = self.nodes.get(self._dragLinkSourceNode)
        self._dragLinkTempLink.is_relationship_link = (
            _start_node is not None
            and _start_node.is_relationship_pin(
                str(portName), self._dragLinkSourceIsOutput
            )
        )
        self._dragLinkTempLink.sourcePropertyName = ""
        self._dragLinkTempLink.targetPropertyName = ""

        # Initialize temporary link
        node = self.nodes[nodeId]
        portPos = self._getPortPosition(node, portName, isOutput)

        # Link renderer expects start=output, end=input for correct bezier direction.
        # When dragging from an input port, the fixed port is the "end" (input side)
        # and the cursor will be the "start" (output side).
        if isOutput:
            self._dragLinkTempLink.sourceNodeId = nodeId
            self._dragLinkTempLink.sourcePort = portName
            self._dragLinkTempLink.targetNodeId = ""
            self._dragLinkTempLink.targetPort = ""
            if self._dragLinkTempLink.is_relationship_link:
                self._dragLinkTempLink.sourcePropertyName = str(portName)
            self._dragLinkTempLink.start = portPos
            self._dragLinkTempLink.end = portPos
        else:
            self._dragLinkTempLink.sourceNodeId = ""
            self._dragLinkTempLink.sourcePort = ""
            self._dragLinkTempLink.targetNodeId = nodeId
            self._dragLinkTempLink.targetPort = portName
            if self._dragLinkTempLink.is_relationship_link:
                self._dragLinkTempLink.targetPropertyName = str(portName)
            self._dragLinkTempLink.end = portPos
            self._dragLinkTempLink.start = portPos

        self._dragLinkValidTarget = False




        if not isOutput and not self._reconnectingLink:
            existingLink = self._findLinkToInput(nodeId, portName)
            if existingLink and not getattr(existingLink, "isDangling", False):
                self._reconnectingLink = True
                self._reconnectCapture = _editing._capture_reconnect(self, existingLink)
                self._reconnectOldSourceNodeId = existingLink.sourceNodeId
                self._reconnectOldSourcePort = existingLink.sourcePort
                self._reconnectOldTargetNodeId = existingLink.targetNodeId
                self._reconnectOldTargetPort = existingLink.targetPort
                self._reconnectOldSourcePropertyName = str(
                    getattr(existingLink, "sourcePropertyName", "")
                )
                self._reconnectOldTargetPropertyName = str(
                    getattr(existingLink, "targetPropertyName", "")
                )
                self._removeLinkFromGraph(
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

    def _snapLinkDragToPort(self, nodeId, portName, isOutput, relationship_hover):
        """Snap the temporary drag link to a hovered port and record the resolved target."""
        targetNode = self.nodes[nodeId]
        if relationship_hover:
            # Attribute → relationship pin: store a resolved target so
            # _completeLinkDrag uses it directly instead of falling back
            # to _hoveredPort, which would apply the output-swap logic
            # and incorrectly invert source/target routing.
            targetPropertyName = self._resolvePortPropertyName(
                targetNode, portName, isOutput
            )
            self._dragLinkResolvedTarget = (
                nodeId,
                portName,
                isOutput,
                targetPropertyName,
            )
            portPos = self._getPortPosition(targetNode, portName, isOutput)
            self._dragLinkTempLink.targetNodeId = nodeId
            self._dragLinkTempLink.targetPort = portName
            self._dragLinkTempLink.end = portPos
        else:
            portPos = self._getPortPosition(targetNode, portName, isOutput)
            if isOutput:
                self._dragLinkTempLink.sourceNodeId = nodeId
                self._dragLinkTempLink.sourcePort = portName
                self._dragLinkTempLink.start = portPos
            else:
                self._dragLinkTempLink.targetNodeId = nodeId
                self._dragLinkTempLink.targetPort = portName
                self._dragLinkTempLink.end = portPos

    def _updateLinkDrag(self, screenPos):
        """
        Update the temporary link being dragged.

        Args:
            screenPos: Screen position of cursor (QPoint)
        """
        worldMousePos = (QtCore.QPointF(screenPos) / self.zoom) + QtCore.QPointF(
            self.panX, self.panY
        )
        worldPos = Gf.Vec2d(worldMousePos.x(), worldMousePos.y())
        self._dragLinkCursorPos = worldPos

        # Link renderer expects start=output, end=input for correct bezier direction.
        # When dragging from an output port, cursor moves the "end" (input side).
        # When dragging from an input port, cursor moves the "start" (output side).
        if self._dragLinkSourceIsOutput:
            self._dragLinkTempLink.targetNodeId = ""
            self._dragLinkTempLink.targetPort = ""
            self._dragLinkTempLink.targetPropertyName = ""
            self._dragLinkTempLink.end = worldPos
        else:
            self._dragLinkTempLink.sourceNodeId = ""
            self._dragLinkTempLink.sourcePort = ""
            self._dragLinkTempLink.sourcePropertyName = ""
            self._dragLinkTempLink.start = worldPos

        # Check if hovering over a valid target port
        self._dragLinkValidTarget = False
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None

        if self._hoveredPort:
            nodeId, portName, isOutput = self._hoveredPort

            # Validate: output can only connect to input (and vice versa)
            relationship_drag = getattr(
                self._dragLinkTempLink, "is_relationship_link", False
            )
            _hover_node = self.nodes.get(nodeId)
            relationship_hover = _hover_node is not None and getattr(
                _hover_node, "is_relationship_pin", lambda *_: False
            )(str(portName), isOutput)
            if relationship_drag:
                if nodeId in self.nodes and not (
                    nodeId == self._dragLinkSourceNode
                    and portName == self._dragLinkSourcePort
                    and isOutput == self._dragLinkSourceIsOutput
                ):
                    targetNode = self.nodes[nodeId]
                    targetPropertyName = self._resolvePortPropertyName(
                        targetNode,
                        portName,
                        isOutput,
                    )
                    linkTargetIsOutput = not bool(self._dragLinkSourceIsOutput)
                    self._dragLinkResolvedTarget = (
                        nodeId,
                        portName,
                        linkTargetIsOutput,
                        targetPropertyName,
                    )
                    self._dragLinkValidTarget = True
                    self._dragLinkTempLink.is_relationship_link = True
                    portPos = self._getLinkEndpointPosition(
                        targetNode,
                        portName,
                        linkTargetIsOutput,
                        targetPropertyName,
                        preferPropertySide=relationship_drag,
                    )
                    if linkTargetIsOutput:
                        self._dragLinkTempLink.sourceNodeId = nodeId
                        self._dragLinkTempLink.sourcePort = portName
                        self._dragLinkTempLink.sourcePropertyName = targetPropertyName
                        self._dragLinkTempLink.start = portPos
                    else:
                        self._dragLinkTempLink.targetNodeId = nodeId
                        self._dragLinkTempLink.targetPort = portName
                        self._dragLinkTempLink.targetPropertyName = targetPropertyName
                        self._dragLinkTempLink.end = portPos
                    return
            elif (
                isOutput != self._dragLinkSourceIsOutput or relationship_hover
            ) and not (
                nodeId == self._dragLinkSourceNode
                and portName == self._dragLinkSourcePort
            ):
                # Valid direction: opposite port types, or attribute dragged onto a
                # relationship pin (USD relationships are directionless so the
                # output/input distinction is relaxed for relationship drop targets).
                self._dragLinkValidTarget = True
                self._snapLinkDragToPort(nodeId, portName, isOutput, relationship_hover)
                return

        if getattr(self._dragLinkTempLink, "is_relationship_link", False):
            targetNodeId = self.nodeIdUnderCursor
            if not targetNodeId:
                hover_margin = max(2.0, 4.0 / max(float(self.zoom), 1e-6))
                targetNodeId = self._getNodeIdAtWorldPos(worldPos, margin=hover_margin)
            if targetNodeId and targetNodeId in self.nodes:
                targetNode = self.nodes[targetNodeId]
                # Restrict the prim-target drop zone to the title bar area at the top of
                # the node. Without this, dropping on a property row (especially when the
                # relationship pin is itself the first property) creates a prim-target
                # connection — which surprises users who meant to drop on the property's
                # port or to cancel the drag. Self-prim-targets remain allowed as long
                # as the drop lands inside the title bar.
                renderer = getattr(targetNode, "renderer", None) or getattr(
                    self, "defaultRenderer", None
                )
                if renderer is not None:
                    get_metrics = getattr(self, "_getRowHitMetrics", None)
                    if get_metrics is not None:
                        port_start_y, _ = get_metrics(targetNode, renderer)
                        if worldPos[1] >= port_start_y:
                            return
                targetIsOutput = not bool(self._dragLinkSourceIsOutput)
                portPos = self._getNodeTopCenterPosition(
                    targetNode,
                )
                self._dragLinkSyntheticTarget = (
                    targetNodeId,
                    "",
                    targetIsOutput,
                    "",
                )
                self._dragLinkValidTarget = True
                self._dragLinkTempLink.is_relationship_link = True
                if targetIsOutput:
                    self._dragLinkTempLink.sourceNodeId = targetNodeId
                    self._dragLinkTempLink.sourcePort = ""
                    self._dragLinkTempLink.sourcePropertyName = ""
                    self._dragLinkTempLink.start = portPos
                else:
                    self._dragLinkTempLink.targetNodeId = targetNodeId
                    self._dragLinkTempLink.targetPort = ""
                    self._dragLinkTempLink.targetPropertyName = ""
                    self._dragLinkTempLink.end = portPos

    def _completeLinkDrag(self):
        """
        Complete link drag - create connection if valid target.

        This is called when the mouse is released during a link drag.
        If a valid target port is hovered, creates the connection.
        If the target is a collapsed node with multiple pins, shows a
        context menu to let the user choose which pin to connect to.
        Otherwise (released on empty space), cancels the drag.
        """
        targetData = self._dragLinkResolvedTarget or self._dragLinkSyntheticTarget
        _temp_link = getattr(self, "_dragLinkTempLink", None)
        relationship_drag = getattr(_temp_link, "is_relationship_link", False)
        if targetData is None and self._hoveredPort and not relationship_drag:
            hoveredNodeId, hoveredPortName, hoveredIsOutput = self._hoveredPort
            targetData = (hoveredNodeId, hoveredPortName, hoveredIsOutput, "")
        if not self._dragLinkValidTarget or not targetData:
            # Cancelled - released on empty space or invalid target
            self._dragLinkSyntheticTarget = None
            self._dragLinkResolvedTarget = None
            if self._reconnectingLink:
                self._finishReconnectDisconnect()
            return

        targetNodeId, targetPortName, targetIsOutput, targetPropertyName = targetData
        usingSyntheticTarget = targetData == self._dragLinkSyntheticTarget

        # Check if target node is title-collapsed with multiple pins on this side
        if (
            not relationship_drag
            and not usingSyntheticTarget
            and targetNodeId in self.nodes
        ):
            targetNode = self.nodes[targetNodeId]
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
                    self._showPinSelectionMenu(
                        targetNodeId,
                        originalPins,
                        targetIsOutput,
                    )
                    return
                elif len(originalPins) == 1:
                    # Only one pin, use it directly
                    targetPortName = originalPins[0]

        # Normalize so we know which is input and which is output
        target_node_model = self.nodes.get(targetNodeId)
        target_is_relationship = (
            target_node_model is not None
            and target_node_model.is_relationship_pin(
                str(targetPortName), targetIsOutput
            )
        )
        if relationship_drag:
            if self._dragLinkSourceIsOutput:
                outputNodeId, outputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
                inputNodeId, inputPortName = targetNodeId, targetPortName
                outputPropertyName = str(self._dragLinkSourcePort)
                inputPropertyName = targetPropertyName
            else:
                outputNodeId, outputPortName = targetNodeId, targetPortName
                inputNodeId, inputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
                outputPropertyName = targetPropertyName
                inputPropertyName = str(self._dragLinkSourcePort)
        elif target_is_relationship:
            # Attribute → relationship pin: preserve the drag-source's original
            # direction so the relationship-owner ends up on the opposite endpoint
            # and create_connection can find it via GetRelationship(target_port).
            if self._dragLinkSourceIsOutput:
                outputNodeId, outputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
                inputNodeId, inputPortName = targetNodeId, targetPortName
            else:
                outputNodeId, outputPortName = targetNodeId, targetPortName
                inputNodeId, inputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
            outputPropertyName = ""
            inputPropertyName = ""
        elif targetIsOutput:
            # Target is output, source must be input - swap them
            outputNodeId, outputPortName = targetNodeId, targetPortName
            inputNodeId, inputPortName = (
                self._dragLinkSourceNode,
                self._dragLinkSourcePort,
            )
            outputPropertyName = ""
            inputPropertyName = ""
        else:
            # Target is input, source must be output
            inputNodeId, inputPortName = targetNodeId, targetPortName
            outputNodeId, outputPortName = (
                self._dragLinkSourceNode,
                self._dragLinkSourcePort,
            )
            outputPropertyName = ""
            inputPropertyName = ""

        if self._reconnectingLink and (
            outputNodeId == self._reconnectOldSourceNodeId
            and outputPortName == self._reconnectOldSourcePort
            and inputNodeId == self._reconnectOldTargetNodeId
            and inputPortName == self._reconnectOldTargetPort
        ):
            self._cancelLinkReconnect()
            return

        # Reject exact duplicate connections
        if self._hasExactLink(
            outputNodeId,
            outputPortName,
            inputNodeId,
            inputPortName,
            outputPropertyName,
            inputPropertyName,
        ):
            if self._reconnectingLink:
                self._cancelLinkReconnect()
            return

        # Check if target input already has a connection
        oldLink = (
            None
            if relationship_drag or target_is_relationship
            else self._findLinkToInput(inputNodeId, inputPortName)
        )
        if oldLink:
            # Highlight old link in red for brief moment
            oldLink.highlighted = True
            oldLink.highlightColor = (1.0, 0.0, 0.0, 1.0)
            self._linksNeedSync = True
            self.update()

            # Snapshot and consume reconnect state now so a new reconnect
            # started during the highlight delay cannot overwrite it.
            reconnect_state = None
            if self._reconnectingLink:
                reconnect_state = (_editing._reconnect_endpoints(self)
                                   if isinstance(self.nodeGraph.getStage(), Usd.Stage) else (
                                       self._reconnectOldSourceNodeId, self._reconnectOldSourcePort,
                                       self._reconnectOldTargetNodeId, self._reconnectOldTargetPort,
                                   ))
                self._clearReconnectState()

            # Use QTimer to remove highlight after 200ms, then create new connection
            from PySide6 import QtCore

            QtCore.QTimer.singleShot(
                200,
                lambda: self._finishConnectionReplacement(
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
            self._createConnection(
                outputNodeId,
                outputPortName,
                inputNodeId,
                inputPortName,
                sourcePropertyName=outputPropertyName,
                targetPropertyName=inputPropertyName,
            )
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None

    def _showPinSelectionMenu(self, targetNodeId, pins, targetIsOutput):
        """Show a context menu to select which pin to connect to on a collapsed node."""
        menu = QtWidgets.QMenu(self)
        menu.setTitle("Select Pin")
        for pin in pins:
            action = menu.addAction(pin)
            action.setData(pin)

        def onPinSelected(action):
            selectedPin = action.data()
            if targetIsOutput:
                outputNodeId, outputPortName = targetNodeId, selectedPin
                inputNodeId, inputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
            else:
                inputNodeId, inputPortName = targetNodeId, selectedPin
                outputNodeId, outputPortName = (
                    self._dragLinkSourceNode,
                    self._dragLinkSourcePort,
                )
            if self._reconnectingLink and (
                outputNodeId == self._reconnectOldSourceNodeId
                and outputPortName == self._reconnectOldSourcePort
                and inputNodeId == self._reconnectOldTargetNodeId
                and inputPortName == self._reconnectOldTargetPort
            ):
                self._cancelLinkReconnect()
                return
            self._createConnection(
                outputNodeId, outputPortName, inputNodeId, inputPortName
            )

        menu.triggered.connect(onPinSelected)
        if self._reconnectingLink:
            from PySide6 import QtCore

            menu.aboutToHide.connect(
                lambda: QtCore.QTimer.singleShot(
                    0,
                    lambda: (
                        self._finishReconnectDisconnect()
                        if self._reconnectingLink
                        else None
                    ),
                )
            )
        menu.popup(self.mapToGlobal(self.lastMousePos))






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

    def _relationshipLinkColor(self, link, hoveredColor, selectedColor):
        if getattr(link, "hovered", False):
            return tuple(hoveredColor)
        if getattr(link, "selected", False):
            return tuple(selectedColor)
        c = self._cachedLinkRelationshipBaseColor
        return (c[0], c[1], c[2], 1.0)

    @staticmethod
    def _isPrimTargetRelationshipLink(link):
        return (
            bool(getattr(link, "is_relationship_link", False))
            and not str(getattr(link, "targetPort", ""))
            and not str(getattr(link, "targetPropertyName", ""))
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

    def _renderConfig(self):
        """Cached C++ RenderConfig shared by the node-quad and link paths.

        The config depends only on settings, so it is rebuilt lazily after a
        settings change (invalidated to None in _initCachedSettings), never per
        frame or per geometry rebuild.
        """
        if self._cachedRenderConfig is None:
            from .nodeGraph import build_render_config

            self._cachedRenderConfig = build_render_config()
        return self._cachedRenderConfig

    def paintNodes(self):
        if not self.nodes or not self.shaderLibrary or not self.fontAtlas:
            return

        try:
            projection = self._worldSpaceProjectionMatrix()
            projList = list(projection.data())
            graph = self.nodeGraph

            # Node-quad appearance depends on selection (colors, stroke, and the
            # graffi highlight quad), so a selection-set change must regenerate.
            # Detecting it here covers every selection path (click, marquee,
            # selectAll, clearSelection, prim-tree sync) without flagging each.
            current_selection = frozenset(self._selectedNodes)
            if current_selection != getattr(
                self, "_lastNodeQuadSelection", frozenset()
            ):
                self._nodeRenderManager.markNodeQuadDirty(True)
            self._lastNodeQuadSelection = current_selection

            content_changed = self.textChanged
            needs_rebuild = (
                content_changed or self._nodeRenderManager.needsNodeRebuild()
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
            if content_changed and self.fontAtlas:
                calcWidth = (
                    self.textRenderer.calculateTextWidth
                    if self.textRenderer
                    else lambda t, s: s * 5
                )
                for node in self.nodes.values():
                    # Force the lazy pin getters so an invalidated node reloads
                    # its raw content before we lay it out from that content.
                    _ = node.inputPins
                    _ = node.outputPins
                    self.nodeGraph._calculateNodeSize(node, calcWidth, self.fontAtlas)

            # GraphModel.nodes is the authoritative render snapshot; re-sync from
            # the Python models only when geometry will be regenerated, then
            # render from it. Keep sync and render adjacent — the freshness
            # invariant depends on no frame boundary between them.
            if needs_rebuild:
                graph.syncNodesFromModels(graph.nodes)

            config = self._renderConfig()
            self._nodeRenderManager.renderNodesFromGraph(
                graph,
                config,
                self.fontAtlas,
                projList,
                config.nodeCornerRadius,
                self._cachedNodeSelectedStrokeColor,
                content_changed,
            )

        except Exception as e:
            Tf.Warn(f"Error rendering nodes: {e}")
            import traceback

            traceback.print_exc()

    def paintGroupStickers(self):
        if not self.groupStickers or not self.shaderLibrary:
            return

        try:
            projection = self._worldSpaceProjectionMatrix()
            self._groupStickerRenderer.renderStickers(self.groupStickers, projection)

        except Exception as e:
            Tf.Warn(f"Error rendering group stickers: {e}")
            import traceback

            traceback.print_exc()

    def paintText(self):
        if not self.nodes or not self.shaderLibrary or not self.fontAtlas:
            return

        try:
            projection = self._worldSpaceProjectionMatrix()

            # Render node icons in title bars. The icons are generated + drawn
            # entirely in C++ from the authoritative GraphModel snapshot (no
            # per-frame quad walk across the language boundary, and they ride
            # node drags via the shared transform frame). The C++ cull of
            # off-screen icons is always-on, matching the glyph cull below.
            if (
                hasattr(self, "_cppIconRenderer")
                and self._cppIconRenderer.isInitialized
            ):
                self._cppIconRenderer.renderNodeIcons(
                    self.nodeGraph,
                    projection,
                    self.zoom,
                    self.textChanged,
                    self.panX,
                    self.panY,
                    float(self.width()),
                    float(self.height()),
                    True,
                )

            success = self.textRenderer.renderNodeText(
                self.nodeGraph,
                projection,
                self.zoom,
                self.textChanged,
                self.panX,
                self.panY,
                float(self.width()),
                float(self.height()),
                True,
            )

            # Only clear textChanged flag if rendering succeeded
            if success and self.textChanged:
                self.textChanged = False

        except Exception as e:
            Tf.Warn(f"Error rendering text: {e}")
            import traceback

            traceback.print_exc()

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

    def paintHotkeyPopup(self):
        if not self.shaderLibrary or not self.fontAtlas:
            return

        projection = self._screenSpaceProjectionMatrix()
        self.hotkeyPopup.render(
            self.textRenderer,
            NodeVertex,
            self._drawNodeVertices,
            projection,
            self.width(),
            self.height(),
            MAX_RENDER_DEPTH,
        )

    def paintNavigationHotkeyPopup(self):
        if not self.shaderLibrary or not self.fontAtlas:
            return

        projection = self._screenSpaceProjectionMatrix()
        self.navigationHotkeyPopup.render(
            self.textRenderer,
            NodeVertex,
            self._drawNodeVertices,
            projection,
            self.width(),
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


    def _valueTextBand(self, rowHeight):
        """(top offset, height) of the value pill within a row band.

        The pill is hung off the ROW LABEL's baseline, not centred in the
        row: the C++ text pass draws the property name and, under it, the
        ``(type)`` line, and a value floating between the two reads as a
        separate widget. Sharing the name's baseline is what makes the value
        look like the end of the same line.
        """
        atlas = self.textRenderer.font_atlas if self.textRenderer else None
        if atlas is None:
            return (valueCell.CELL_TOP_RATIO * rowHeight,
                    valueCell.CELL_HEIGHT_RATIO * rowHeight)
        pinFont = float(self._cachedNodePinFontSize)
        typeFont = (
            float(NoodlesConfig.get("nodePinTypeFontSize", 26.0))
            if NoodlesConfig.get("showPinTypeLabels", True)
            else 0.0
        )
        size = self._valueFontSize()
        # Pure function of (row height, fonts, atlas metrics): one entry per
        # distinct row height, recomputed only when a dependency changes.
        key = (
            round(float(rowHeight), 3),
            pinFont,
            typeFont,
            size,
            float(atlas.lineHeight),
            float(atlas.ascender),
            float(atlas.descender),
        )
        band = self._valueTextBandCache.get(key)
        if band is not None:
            return band
        lineHeight = float(atlas.lineHeight)
        block = (pinFont + typeFont) * lineHeight
        baseline = (rowHeight - block) * 0.5 + float(atlas.ascender) * pinFont
        pad = 0.07 * rowHeight
        top = baseline - float(atlas.ascender) * size - pad
        height = (float(atlas.ascender) - float(atlas.descender)) * size + 2.0 * pad
        # Never taller than the row it belongs to.
        if height > rowHeight:
            height = rowHeight
        if top < 0.0:
            top = 0.0
        if top + height > rowHeight:
            top = max(0.0, rowHeight - height)
        band = (top, height)
        self._valueTextBandCache[key] = band
        return band

    def _valueBaselineY(self, rect):
        """Text baseline inside a value pill."""
        atlas = self.textRenderer.font_atlas
        size = self._valueFontSize()
        blockHeight = (float(atlas.ascender) - float(atlas.descender)) * size
        top = rect.top + ((rect.bottom - rect.top) - blockHeight) * 0.5
        return top + float(atlas.ascender) * size

    def _valueGlyph(self, ch, size):
        """``(vertices, width)`` for one character, generated at the origin.

        A value is the same dozen glyphs every frame, so each character is
        generated once per size at (0, 0, 0) and translated by the caller --
        the two C++ calls per glyph per frame this replaces were the
        single largest cost in the value paint pass. Cleared with the other
        paint caches when the atlas is regenerated.
        """
        key = (ch, size)
        cached = self._valueGlyphCache.get(key)
        if cached is None:
            renderer = self.textRenderer
            glyph = []
            renderer.generateTextVertices(
                ch, 0.0, 0.0, 0.0, size, glyph, node_index=-1.0
            )
            cached = (tuple(glyph), renderer.calculateTextWidth(ch, size))
            self._valueGlyphCache[key] = cached
        return cached

    def _emitValueText(self, text, x, baselineY, depth, size, out,
                       tabular=False, italic=0.0):
        """Append one value string's glyphs to *out*, starting at *x*.

        ``tabular`` sets digits on the widest digit's advance, so a number
        does not shuffle sideways while it is being dragged. ``italic``
        shears the glyphs around the baseline -- the atlas has no italic
        face, and a read-only value has to look different from an editable
        one at a glance.
        """
        if not text:
            return 0.0
        renderer = self.textRenderer
        if not tabular and italic <= 0.0:
            renderer.generateTextVertices(
                text, x, baselineY, depth, size, out, node_index=-1.0
            )
            return renderer.calculateTextWidth(text, size)

        advance = self._valueDigitAdvance(size) if tabular else 0.0
        cursor = x
        for ch in text:
            verts, natural = self._valueGlyph(ch, size)
            step = advance if (tabular and ch.isdigit()) else natural
            ox = cursor + (step - natural) * 0.5
            oy = baselineY
            # Glyph vertices are flat floats, 6 per vertex (x, y, z, u, v,
            # nodeIndex); the cached glyph is translated into place, and an
            # italic lean is two lines about the baseline.
            if italic > 0.0:
                for i in range(0, len(verts), 6):
                    gx = verts[i] + ox
                    gy = verts[i + 1] + oy
                    out.append(gx + (baselineY - gy) * italic)
                    out.append(gy)
                    out.append(verts[i + 2] + depth)
                    out.append(verts[i + 3])
                    out.append(verts[i + 4])
                    out.append(verts[i + 5])
            else:
                for i in range(0, len(verts), 6):
                    out.append(verts[i] + ox)
                    out.append(verts[i + 1] + oy)
                    out.append(verts[i + 2] + depth)
                    out.append(verts[i + 3])
                    out.append(verts[i + 4])
                    out.append(verts[i + 5])
            cursor += step
        return cursor - x

    def _onValueCellPressed(self, hit, event):
        """A left press landed on a value cell. Decide what it means."""
        row = hit.row
        if not row.editable:
            # Read-only is answered, not ignored: silence here reads as a
            # broken cell. It also must NOT fall through to a connection drag.
            self._showPopupMessage("%s: %s" % (row.property_name, row.reason))
            return
        if row.kind == noodlesValues.KIND_BOOL:
            self._toggleBoolValue(hit)
            return
        if row.kind == noodlesValues.KIND_TOKEN_ENUM:
            self._openTokenPopup(hit)
            return

        # Numeric or text: nothing is authored yet. The release decides
        # whether this was a drag (mung) or a click (keyboard editor).
        mungable = bool(row.mungable)
        base = row.components[hit.comp] if mungable else 0.0
        self._mungState = {
            "nodeId": hit.nodeId,
            "primPath": hit.primPath,
            "prop": hit.propertyName,
            "comp": hit.comp,
            "baseScreenX": float(event.position().x()),
            "original": tuple(row.components),
            "current": tuple(row.components),
            "base": float(base) if mungable else 0.0,
            "step": (
                noodlesValues.ladder_step(base, row.type_name) if mungable else 0.0
            ),
            "typeName": row.type_name,
            "time": self._currentTimeCode,
            "mungable": mungable,
        }
        self._mungActive = False

    def _openTokenPopup(self, hit):
        """A token with ``allowedTokens``: pick from exactly those, nothing else."""
        tokens = list(hit.row.tokens)
        if not tokens:
            self._beginValueEdit(hit.nodeId, hit.propertyName, 0)
            return
        # The popup wears the node's clothes: same fill, same accent for the
        # current value, same text colours. A native menu here would be the
        # one part of the interaction that did not belong to the graph.
        theme = self._cachedValueTheme
        self._tokenPopupList.font_size = 15.0
        self._tokenPopupList.background_color = theme.popupBg
        self._tokenPopupList.item_color = theme.textNormal
        self._tokenPopupList.selected_color = theme.popupRow
        self._tokenPopupList.set_items(
            [SelectableListItem(token) for token in tokens]
        )
        current = hit.row.components[0] if hit.row.components else ""
        self._tokenPopupList.selected_index = (
            tokens.index(current) if current in tokens else 0
        )
        self._tokenPopupList.scroll_selection_into_view()
        self._tokenPopup = {
            "nodeId": hit.nodeId,
            "primPath": hit.primPath,
            "prop": hit.propertyName,
            "tokens": tuple(tokens),
            "rect": hit.rect,
            "time": self._currentTimeCode,
            "original": tuple(hit.row.components),
        }
        self.update()

    def _closeTokenPopup(self):
        self._tokenPopup = None
        self.update()

    def _handleTokenPopupPress(self, screenPos):
        """Inside the popup picks a token; anywhere else just closes it."""
        bounds = self._tokenPopupList.bounds
        if bounds:
            x, y, width, height = bounds
            if x <= screenPos.x() <= x + width and y <= screenPos.y() <= y + height:
                index = self._tokenPopupList.item_index_at_pixel(screenPos.y() - y)
                if index is not None:
                    self._tokenPopupList.selected_index = index
                    self._commitTokenPopup()
                    return
        self._closeTokenPopup()

    def focusNextPrevChild(self, nextChild):
        """Let keyPressEvent use Tab for creation and inline value editing."""
        return False

    def _paintValueCells(self):
        """Draw every visible value cell in the node's own visual language.

        A Python pass by necessity: node text is generated and drawn entirely
        in C++ by the pinned external noodles renderer, so there is no hook to
        put a value string into a row's own layout. Everything it draws --
        fills, hairlines, text, the bool switch, the token chevron -- is
        derived from the node theme (``_cachedValueTheme``), so the cells read
        as part of the node rather than as a widget stacked on top of it.

        Batched to a handful of draw calls: one per (corner radius, stroke)
        bucket for the quads, one per colour bucket for the glyphs, so the
        cost is O(visible value rows) with no per-row GL state changes.
        """
        if not self.nodes or not self.shaderLibrary or not self.fontAtlas:
            return
        if not self.textRenderer or not self.textRenderer.font_atlas:
            return

        projection = self._worldSpaceProjectionMatrix()
        fontSize = self._valueFontSize()
        if fontSize <= 0.0:
            return
        theme = self._cachedValueTheme
        minPixelHeight = float(self._cachedValueMinPixelHeight)
        editing = self._valueEditTarget
        munging = self._mungState if self._mungActive else None
        hover = self._valueHover
        flash = self._activeValueFlash()

        quadBatches = defaultdict(list)  # (radius, strokeColor) -> vertices
        textBatches = defaultdict(list)  # rgba -> glyph floats
        caretVertices = []

        def measure(text):
            return self.textRenderer.calculateTextWidth(text, fontSize)

        for nodeId, node in self.nodes.items():
            rowHeight = float(node.layoutPortLineHeight)
            # The LOD gate, and it FADES rather than pops: value text is
            # 0.9x the pin font, so it turns to noise a little before the
            # labels do, and a cell that vanished between one wheel notch
            # and the next would read as a bug.
            if rowHeight <= 0.0:
                continue
            pixels = rowHeight * self.zoom
            if pixels < minPixelHeight:
                continue
            lodAlpha = min(1.0, (pixels - minPixelHeight) / max(1e-6, minPixelHeight))
            lodAlpha = max(0.0, lodAlpha)
            if lodAlpha <= 0.01:
                continue
            try:
                position = Gf.Vec2d(node.position)
                bounds = Gf.Range2d(position, position + Gf.Vec2d(node.size))
            except Exception:
                continue
            if not self._isVisible(bounds):
                continue

            # Cells ride the node's own z-order band, exactly as its text and
            # icons do, so they raise together when a node is brought forward.
            depth = node_content_depth(node.zOrder)
            radius = None
            for row, rect, subs, h in self._valueRowGeometry(node):
                if radius is None:
                    radius = round(valueCell.corner_radius(h), 3)
                innerPad = valueCell.INNER_PAD_RATIO * h
                rowHovered = hover is not None and hover[0] == nodeId and (
                    hover[1] == row.property_name
                )
                rowActive = (
                    editing is not None
                    and editing[0] == nodeId
                    and editing[1] == row.property_name
                ) or (
                    munging is not None
                    and munging["nodeId"] == nodeId
                    and munging["prop"] == row.property_name
                )
                fill, stroke, textColor, italic = self._valueStateColors(
                    row, theme, rowHovered, rowActive
                )
                if flash is not None and flash[0] == nodeId and (
                    flash[1] == row.property_name
                ):
                    fill = theme.fillError if flash[3] == "error" else theme.fillAccent
                    stroke = theme.error if flash[3] == "error" else theme.accent

                if fill is not None:
                    valueCell.append_quad(
                        quadBatches[(radius, stroke)],
                        NodeVertex,
                        rect,
                        depth,
                        fill,
                        lodAlpha,
                    )
                elif stroke is not None:
                    valueCell.append_quad(
                        quadBatches[(radius, stroke)],
                        NodeVertex,
                        rect,
                        depth,
                        (0.0, 0.0, 0.0, 0.0),
                        lodAlpha,
                    )

                # A time-sampled value carries the animation tint down its
                # leading edge: present at a glance, not another word to read.
                if row.state == noodlesValues.STATE_ANIMATED:
                    valueCell.append_quad(
                        quadBatches[(0.0, None)],
                        NodeVertex,
                        valueCell.Rect(
                            rect.left, rect.top, rect.left + 0.05 * h, rect.bottom
                        ),
                        depth,
                        theme.textAnimated,
                        lodAlpha,
                    )
                if row.has_swatch and len(row.components) >= 3:
                    valueCell.append_quad(
                        quadBatches[(round(radius * 0.7, 3), theme.hairline)],
                        NodeVertex,
                        valueCell.swatch_rect(rect, h),
                        depth,
                        (
                            float(row.components[0]),
                            float(row.components[1]),
                            float(row.components[2]),
                            1.0,
                        ),
                        lodAlpha,
                    )

                baselineY = self._valueBaselineY(rect)
                tabular = row.kind in (
                    noodlesValues.KIND_FLOAT,
                    noodlesValues.KIND_INT,
                )
                # One measure for the whole row, and a TABULAR one for
                # numbers, so the fitted format matches the widths the
                # glyphs will actually be set at.
                if tabular:
                    def rowMeasure(text, _size=fontSize):
                        return self._valueTextWidth(text, _size, True)
                else:
                    rowMeasure = measure
                axisLabels = (
                    valueCell.COLOR_AXIS_LABELS
                    if row.has_swatch
                    else valueCell.AXIS_LABELS
                )
                hairWidth = max(0.6 / max(self.zoom, 1e-3), 0.012 * h)

                # Fit ONCE for the whole value, so every component of a
                # vector is shown to the same number of decimals -- and once
                # per value, not once per frame (see _valueFittedTexts).
                slotWidth = valueCell.text_max_width(subs[0], h)
                if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                    slotWidth -= 0.20 * h * 2.2
                if row.count > 1:
                    slotWidth -= max(
                        self._valueAxisWidth(label, fontSize * 0.8)
                        for label in axisLabels[: row.count]
                    ) + innerPad * 0.5
                texts = self._valueFittedTexts(
                    nodeId, row, slotWidth, fontSize, rowMeasure
                )

                for index, sub in enumerate(subs):
                    componentHovered = rowHovered and hover[2] == index
                    # Components of one value live in ONE pill, divided by
                    # hairlines rather than by gaps, so a vector reads as a
                    # single value with parts.
                    if index > 0:
                        valueCell.append_quad(
                            quadBatches[(0.0, None)],
                            NodeVertex,
                            valueCell.Rect(
                                sub.left - hairWidth * 0.5,
                                rect.top + 0.12 * (rect.bottom - rect.top),
                                sub.left + hairWidth * 0.5,
                                rect.bottom - 0.12 * (rect.bottom - rect.top),
                            ),
                            depth,
                            theme.hairline,
                            lodAlpha,
                        )
                    if componentHovered and fill is not None and not rowActive:
                        valueCell.append_quad(
                            quadBatches[(radius, None)],
                            NodeVertex,
                            sub,
                            depth,
                            theme.fillHover,
                            lodAlpha * 0.8,
                        )

                    if row.kind == noodlesValues.KIND_BOOL:
                        self._appendBoolSwitch(
                            quadBatches, sub, h, depth, row, theme, lodAlpha
                        )
                        continue

                    available = valueCell.text_max_width(sub, h)
                    caretSize = 0.20 * h
                    if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                        available -= caretSize * 2.2
                    axisWidth = 0.0
                    if row.count > 1:
                        axisWidth = (
                            self._valueAxisWidth(
                                axisLabels[min(index, 3)], fontSize * 0.8
                            )
                            + innerPad * 0.5
                        )
                        available -= axisWidth
                    if available <= 0.0:
                        continue
                    if (
                        editing is not None
                        and editing[0] == nodeId
                        and editing[1] == row.property_name
                        and editing[2] == index
                    ):
                        continue  # the editor is drawing this one

                    if row.count > 1:
                        axisColor = valueCell.AXIS_COLORS[min(index, 3)]
                        key = (axisColor[0], axisColor[1], axisColor[2],
                               0.85 * lodAlpha)
                        self._emitValueText(
                            axisLabels[min(index, 3)],
                            sub.left + innerPad,
                            baselineY,
                            depth,
                            fontSize * 0.8,
                            textBatches[key],
                        )

                    text = texts[index] if index < len(texts) else ""
                    if not text:
                        continue
                    if row.kind in (
                        noodlesValues.KIND_TEXT,
                        noodlesValues.KIND_TOKEN_ENUM,
                    ):
                        textX = sub.left + innerPad
                    else:
                        textX = (
                            sub.right
                            - innerPad
                            - self._valueTextWidth(text, fontSize, tabular)
                        )
                    key = (textColor[0], textColor[1], textColor[2],
                           textColor[3] * lodAlpha)
                    self._emitValueText(
                        text,
                        textX,
                        baselineY,
                        depth,
                        fontSize,
                        textBatches[key],
                        tabular=tabular,
                        italic=italic,
                    )
                    if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                        cx = sub.right - innerPad - caretSize * 0.6
                        cy = (rect.top + rect.bottom) * 0.5
                        self._appendChevron(
                            caretVertices, cx, cy, caretSize, depth,
                            textColor, lodAlpha
                        )

        for (radius, stroke), vertices in quadBatches.items():
            if not vertices:
                continue
            self._drawNodeVertices(
                vertices,
                projection,
                radius,
                stroke if stroke is not None else (0.0, 0.0, 0.0, 0.0),
                generation=-1,
            )
        if caretVertices:
            self._drawNodeVertices(
                caretVertices, projection, 0.0, (0.0, 0.0, 0.0, 0.0), generation=-1
            )
        for color, vertices in textBatches.items():
            if vertices:
                self.textRenderer.drawTextVertices(vertices, projection, color)

        if flash is not None:
            # Keep repainting while the flash is running; it is the only
            # animation this pass owns.
            self.update()

    def _appendBoolSwitch(self, quadBatches, sub, h, depth, row, theme, alpha):
        """A rounded checkbox drawn in the PIN's colours, not its own.

        On is the connected-port fill, off is the disconnected one, so a
        boolean reads the same way a wired port does two columns to the left.
        """
        innerPad = valueCell.INNER_PAD_RATIO * h
        side = 0.46 * h
        top = sub.top + ((sub.bottom - sub.top) - side) * 0.5
        box = valueCell.Rect(
            sub.right - innerPad - side, top, sub.right - innerPad, top + side
        )
        on = bool(row.components[0])
        radius = round(side * 0.28, 3)
        valueCell.append_quad(
            quadBatches[(radius, theme.hairline)],
            NodeVertex,
            box,
            depth,
            theme.boolOn if on else theme.boolOff,
            alpha,
        )
        if on:
            inset = side * 0.28
            valueCell.append_quad(
                quadBatches[(round(radius * 0.6, 3), None)],
                NodeVertex,
                valueCell.Rect(
                    box.left + inset,
                    box.top + inset,
                    box.right - inset,
                    box.bottom - inset,
                ),
                depth,
                (0.10, 0.11, 0.09, 0.9),
                alpha,
            )

    def _appendChevron(self, out, cx, cy, size, depth, color, alpha):
        """The small downward chevron that says "this opens a list"."""
        self._appendTriangleVertices(
            out,
            [
                (cx - size * 0.55, cy - size * 0.28),
                (cx + size * 0.55, cy - size * 0.28),
                (cx, cy + size * 0.34),
            ],
            depth,
            (color[0], color[1], color[2], color[3] * alpha),
        )

    def _activeValueFlash(self):
        """The running commit/error flash, or None once it has expired."""
        flash = self._valueFlash
        if flash is None:
            return None
        if time.time() >= flash[2]:
            self._valueFlash = None
            return None
        return flash

    def _startValueFlash(self, nodeId, propertyName, kind="commit"):
        """A short tint on the cell that just committed (or was rejected)."""
        duration = 0.12 if kind == "commit" else 0.6
        self._valueFlash = (nodeId, propertyName, time.time() + duration, kind)
        self.update()

    def _paintValueEditor(self):
        """Draw the keyboard editor INSIDE the cell it is editing.

        Not the screen-space TextInputWidget the rename uses: an editor that
        appears somewhere other than the value it edits, in a different font
        at a different size, is the thing that makes an inline field feel
        bolted on. This is the same pill, the same radius, the same padding
        and the same baseline as the cell it replaces, with the node's own
        selection accent as its outline -- so entering and leaving edit mode
        moves nothing.
        """
        target = self._valueEditTarget
        if target is None:
            return
        node = self.nodes.get(target[0])
        if node is None:
            return
        rect = self._valueCellRectFor(node, target[1], target[2])
        if rect is None:
            return
        if not self.textRenderer or not self.textRenderer.font_atlas:
            return

        row = self._findValueRow(target[0], target[1])
        theme = self._cachedValueTheme
        projection = self._worldSpaceProjectionMatrix()
        depth = node_content_depth(node.zOrder) + 0.0002
        rowHeight = float(node.layoutPortLineHeight) or 1.0
        innerPad = valueCell.INNER_PAD_RATIO * rowHeight
        fontSize = self._valueFontSize()
        radius = round(valueCell.corner_radius(rowHeight), 3)
        error = self._valueEditError

        quads = []
        valueCell.append_quad(
            quads,
            NodeVertex,
            rect,
            depth,
            theme.fillError if error else theme.fillEdit,
        )
        self._drawNodeVertices(
            quads,
            projection,
            radius,
            theme.error if error else theme.accent,
            generation=-1,
        )

        text = self._valueEditInput.text
        tabular = row is not None and row.kind in (
            noodlesValues.KIND_FLOAT,
            noodlesValues.KIND_INT,
        )
        leftAligned = row is not None and row.kind in (
            noodlesValues.KIND_TEXT,
            noodlesValues.KIND_TOKEN_ENUM,
        )
        width = self._valueTextWidth(text, fontSize, tabular)
        if leftAligned:
            textX = rect.left + innerPad
        else:
            textX = rect.right - innerPad - width
        baselineY = self._valueBaselineY(rect)

        # Select-all on entry, shown the way a text field shows it.
        overlays = []
        if self._valueEditFresh and text:
            valueCell.append_quad(
                overlays,
                NodeVertex,
                valueCell.Rect(
                    textX - innerPad * 0.25,
                    rect.top + (rect.bottom - rect.top) * 0.14,
                    textX + width + innerPad * 0.25,
                    rect.bottom - (rect.bottom - rect.top) * 0.14,
                ),
                depth,
                theme.selection,
            )

        cursorText = text[: self._valueEditInput.cursor_position]
        cursorX = textX + self._valueTextWidth(cursorText, fontSize, tabular)
        self._valueEditInput._update_cursor_blink()
        if self._valueEditInput._cursor_visible:
            caretWidth = max(1.2 / max(self.zoom, 1e-3), 0.015 * rowHeight)
            valueCell.append_quad(
                overlays,
                NodeVertex,
                valueCell.Rect(
                    cursorX,
                    rect.top + (rect.bottom - rect.top) * 0.16,
                    cursorX + caretWidth,
                    rect.bottom - (rect.bottom - rect.top) * 0.16,
                ),
                depth,
                theme.caret,
            )
        if overlays:
            self._drawNodeVertices(
                overlays, projection, 0.0, (0.0, 0.0, 0.0, 0.0), generation=-1
            )

        glyphs = []
        self._emitValueText(
            text, textX, baselineY, depth, fontSize, glyphs, tabular=tabular
        )
        if glyphs:
            self.textRenderer.drawTextVertices(
                glyphs, projection, theme.textNormal
            )
        # The caret blinks, so this pass owns an animation while it is open.
        self.update()

    def _paintMungLadder(self):
        """A small readout of the step the drag is currently using.

        ladder without the ladder: the multiplier is the only part
        a user needs while dragging, and it belongs next to the cursor rather
        than in a status bar on the other side of the window.
        """
        state = self._mungState
        if state is None or not self._mungActive or not state["mungable"]:
            return
        if not self.textRenderer or not self.textRenderer.font_atlas:
            return
        modifiers = QtWidgets.QApplication.keyboardModifiers()
        multiplier = noodlesValues.mung_multiplier(
            bool(modifiers & QtCore.Qt.ShiftModifier),
            bool(modifiers & QtCore.Qt.ControlModifier),
        )
        step = state["step"] * multiplier
        label = "x%g /px" % step

        theme = self._cachedValueTheme
        projection = self._screenSpaceProjectionMatrix()
        atlas = self.textRenderer.font_atlas
        fontSize = 15.0
        padX, padY = 9.0, 5.0
        width = self.textRenderer.calculateTextWidth(label, fontSize) + padX * 2.0
        height = fontSize * (atlas.ascender - atlas.descender) + padY * 2.0
        # The view's own last mouse position, not QCursor.pos(): the drag is
        # driven by the events this widget received, and a synthetic one (a
        # test, a tablet remap) never moved the OS pointer at all.
        cursor = self.lastMousePos or self.mapFromGlobal(QtGui.QCursor.pos())
        x = min(max(0.0, float(cursor.x()) + 18.0), max(0.0, self.width() - width))
        y = min(max(0.0, float(cursor.y()) - height - 12.0),
                max(0.0, self.height() - height))

        quads = []
        valueCell.append_quad(
            quads,
            NodeVertex,
            valueCell.Rect(x, y, x + width, y + height),
            MAX_RENDER_DEPTH,
            theme.popupBg,
        )
        self._drawNodeVertices(quads, projection, 4.0, theme.hairline, generation=-1)

        glyphs = []
        self.textRenderer.generateTextVertices(
            label,
            x + padX,
            y + padY + atlas.ascender * fontSize,
            MAX_RENDER_DEPTH,
            fontSize,
            glyphs,
            node_index=-1.0,
        )
        if glyphs:
            self.textRenderer.drawTextVertices(
                glyphs, projection, theme.accent, disable_depth=True
            )

    def _paintTokenPopup(self):
        """Draw the allowedTokens list under the cell, in the node's style."""
        popup = self._tokenPopup
        if popup is None:
            return
        if not self.textRenderer or not self.textRenderer.font_atlas:
            return

        theme = self._cachedValueTheme
        projection = self._screenSpaceProjectionMatrix()
        rect = popup["rect"]
        # The list is set in the SAME on-screen size as the cell it dropped
        # out of, so the token you are choosing looks like the token you are
        # replacing rather than like an operating-system menu.
        onScreenFont = self._valueFontSize() * self.zoom
        self._tokenPopupList.font_size = max(11.0, min(onScreenFont, 30.0))
        self._tokenPopupList.item_height = (
            self._tokenPopupList.font_size * 1.55
        )
        self._tokenPopupList.padding = self._tokenPopupList.font_size * 0.45
        width, height = self._tokenPopupList.calculate_size(self.textRenderer)
        width = max(width, valueCell.rect_width(rect) * self.zoom)
        x = (rect.left - self.panX) * self.zoom
        y = (rect.bottom - self.panY) * self.zoom + 3.0
        # Clamped into the widget, and flipped above the cell rather than
        # drawn off the bottom edge.
        x = max(0.0, min(x, max(0.0, float(self.width()) - width)))
        if y + height > float(self.height()):
            y = max(0.0, (rect.top - self.panY) * self.zoom - height - 3.0)

        background = []
        valueCell.append_quad(
            background,
            NodeVertex,
            valueCell.Rect(x, y, x + width, y + height),
            MAX_RENDER_DEPTH,
            theme.popupBg,
        )
        self._drawNodeVertices(
            background, projection, 6.0, theme.hairline, generation=-1
        )
        self._tokenPopupList.render(
            self.textRenderer,
            x,
            y,
            MAX_RENDER_DEPTH,
            width,
            NodeVertex,
            self._drawNodeVertices,
            projection,
        )

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

    def _paintNodeRename(self):
        """Draw the inline editor over the node being renamed."""
        node = self.nodes.get(self._renamingNodeId)
        if node is None:
            return

        projection = self._screenSpaceProjectionMatrix()
        x = (float(node.position[0]) - self.panX) * self.zoom
        y = (float(node.position[1]) - self.panY) * self.zoom
        # Never narrower than the text: a zoomed-out node is a few pixels wide
        # and the field has to stay legible to be typed into.
        minWidth, _ = self._renameInput.calculate_size(self.textRenderer)
        width = max(float(node.size[0]) * self.zoom, minWidth)

        self._renameInput.render(
            self.textRenderer,
            x,
            y,
            MAX_RENDER_DEPTH,
            width,
            NodeVertex,
            self._drawNodeVertices,
            projection,
        )

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
