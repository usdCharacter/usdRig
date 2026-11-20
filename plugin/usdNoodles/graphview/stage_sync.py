#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Stage reconciliation and stage reload policy: the graph follows the stage.

The stage_sync layer of the noodle editor: USD notice classification, scope
validation, namespace-move re-keying, the invalidation decisions they drive,
and the replace / reload / close lifecycle in ``StageSync``. Every function
takes the view as a duck-typed context, reading only its state attributes and
calling only the listed view hooks, so the layer runs headless and against
stub views while the widget side effects (repaint, render caches) stay behind
those hooks. Nothing here imports Qt or GL: scheduling and warning arrive as
callables the view supplies. Reconciliation is read-only: it never authors a
USD opinion.
"""

from contextlib import contextmanager
from dataclasses import dataclass

from pxr import Sdf, Tf

from UsdNoodles.nodeGraph import unsubscribe_edit_target_warning


# ---------------------------------------------------------------------------
# The published trace of one lifecycle step.
#
# Each record names the epoch that produced it, so a consumer can tell the
# current generation's work from a stale one. They are records, not a queue:
# the reload and notice paths emit them as the work happens, and the model
# mutations they describe are the functions below. Paths are strings, id
# collections are tuples, and endpoint / port metadata keeps the order the
# graph iterates in.
# ---------------------------------------------------------------------------

MODE_STAGE = "stage"
MODE_BLUEPRINT = "blueprint"
MODE_CONTAINER = "container"


@dataclass(frozen=True, slots=True)
class BeginReload:
    """Paint is off and the previous stage's transient state is dropped.

    *frame* is the time the reloaded graph reads at.
    """

    epoch: int
    frame: object


@dataclass(frozen=True, slots=True)
class Load:
    """Re-run the loader for *mode* (one of MODE_STAGE / MODE_BLUEPRINT /
    MODE_CONTAINER) on *stage*. The stage is live only inside the
    owner-thread dispatch that applies this record."""

    epoch: int
    stage: object
    mode: str
    prim_path: object


@dataclass(frozen=True, slots=True)
class ClearHistory:
    """Drop the undo history: its records name prims on a stage that is gone."""

    epoch: int


@dataclass(frozen=True, slots=True)
class Remap:
    """Nodes re-keyed across a namespace move, ``(old, new)`` pairs."""

    epoch: int
    pairs: tuple


@dataclass(frozen=True, slots=True)
class Remove:
    """Node ids the graph lost -- a dropped prim, or a rebuilt graph."""

    epoch: int
    ids: tuple


@dataclass(frozen=True, slots=True)
class Adopt:
    """Scope paths the graph took on because a prim appeared in USD."""

    epoch: int
    paths: tuple


@dataclass(frozen=True, slots=True)
class Invalidate:
    """The rebuild / repaint decision a notice pass ended with."""

    epoch: int
    paths: tuple
    links: bool
    text: bool
    paint: bool


@dataclass(frozen=True, slots=True)
class FinishReload:
    """The deferred render-cache clear, links and text dirty, paint back on."""

    epoch: int


@dataclass(frozen=True, slots=True)
class StageContext:
    """The view's state as one owner-thread read, for one reload decision.

    ``view`` stays the duck-typed context the functions here expect; the rest
    is snapshotted so a caller can see what the service decided from without
    reaching back into the widget. This is the reload path's read: it names
    every node the graph is about to lose, so it is the all-node snapshot and
    costs what the graph costs.
    """

    view: object
    stage: object
    mode: str
    prim_path: object
    node_ids: tuple
    syncing_selection: bool


@dataclass(frozen=True, slots=True)
class NoticeContext:
    """What one USD notice reads from the view: the view and its mute flag.

    Deliberately not ``StageContext``: a notice reaches at most the nodes its
    own paths name, so copying every node id -- or asking which loader a
    rebuild would re-run -- per notice would make a single-property edit scale
    with the size of the graph.
    """

    view: object
    syncing_selection: bool


def _nodeIdPrimPath(nodeId):
    """The prim-path half of a node id.

    Most node ids ARE prim paths. Blueprint / Container I/O nodes are
    synthesized rather than prim-backed and carry their pin in a ``#`` suffix
    (``/Bp#input:foo:bar``); the part before the ``#`` is still the prim path
    they hang off, which is what makes them follow that prim when it moves.
    """
    return nodeId.split("#", 1)[0]


def _isPathRelated(pathA, pathB):
    """Whether one path is the other, or contains the other.

    "Contains" spans both separators USD uses: ``/`` for a child prim and ``.``
    for a property, so ``/World/Cube`` is related to ``/World/Cube/Leg`` and to
    ``/World/Cube.inputs:x`` alike -- a resync can name either, and both mean
    "the node at /World/Cube is stale".

    The boundary is the point. Plain ``startswith`` reads ``/World/Cube`` as an
    ancestor of ``/World/CubeTwo``, so editing one prim invalidates unrelated
    siblings whose names merely share a prefix.
    """
    if pathA == pathB:
        return True
    longer, shorter = (pathA, pathB) if len(pathA) > len(pathB) else (pathB, pathA)
    if not longer.startswith(shorter):
        return False
    # The pseudo-root is the one path that already ends in its own separator.
    if shorter.endswith("/"):
        return True
    return longer[len(shorter)] in "/."


def graphScopeParentPath(view):
    """The prim whose children this graph shows.

    NodeGraphStage walks the pseudo-root's children; the blueprint and
    container graphs walk the children of the prim they were opened on.
    Reconciling against the stage has to use the same rule the loaders
    use, or a resync deep in the hierarchy would add a node this view was
    never meant to show.
    """
    from ..nodeGraphBlueprint import NodeGraphBlueprint
    from ..nodeGraphBpContainer import NodeGraphBpContainer

    if isinstance(view.nodeGraph, (NodeGraphBlueprint, NodeGraphBpContainer)):
        return str(view._currentPrimPath) if view._currentPrimPath else None
    return "/"


def validateNodesAgainstStage(view):
    """Drop every node whose prim is no longer on the stage.

    As far as USD notices are concerned a rename is a delete plus an add:
    the old path resyncs with nothing behind it any more. The resync loop
    above only invalidates caches for paths still present in self.nodes,
    so without this the old node keeps drawing from its cached name while
    everything that goes through the stage — selection, framing,
    connecting — silently does nothing, because the prim it names is gone.
    That is the "node stuck under the old name that will not select".

    Deactivated prims count as gone: the loaders skip inactive children,
    so keeping their nodes would leave the view disagreeing with what a
    reload would produce.

    Returns:
        list[str]: node ids removed.
    """
    stale = []
    for nodeId, node in list(view.nodes.items()):
        # Blueprint I/O nodes are synthesized, not backed by a prim.
        if "#" in nodeId:
            continue
        try:
            prim = node.getUsdPrim()
        except Exception:
            prim = None
        if prim and prim.IsValid() and prim.IsActive():
            continue
        stale.append(nodeId)

    for nodeId in stale:
        view.nodes.pop(nodeId, None)
        view._selectedNodes.discard(nodeId)
        view._marqueeBaseSelectedNodes.discard(nodeId)
        view._marqueePrevSelectedNodes.discard(nodeId)
        # _rebuildLinks rebuilds the whole index, but that is deferred to
        # the next paint; drop it now so a click landing first cannot hit
        # a node that no longer exists.
        try:
            view._spatialIndex.removeNode(nodeId)
        except Exception:
            pass

    if stale:
        Tf.Status("Graph validation dropped %d node(s) whose prims are "
                  "gone: %s" % (len(stale), ", ".join(sorted(stale))))
    return stale


def adoptPrimsNewToScope(view, resyncedPaths):
    """Add nodes for prims that appeared in this graph's scope.

    The other half of a rename: the new path resyncs with a real prim
    behind it and nothing in the graph pointing at it, so without this a
    renamed node disappears from the editor until a manual reload.

    Only paths in this notice are considered, and only direct children of
    the scope this graph loads. Note this can re-add a node the user hid
    with 'D' (which removes from the view without touching USD) if that
    same prim is later edited — the view follows the stage, which is the
    contract being asked for here.

    Returns:
        list[str]: node ids added.
    """
    stage = view.nodeGraph.getStage()
    scope = view._graphScopeParentPath()
    if not stage or scope is None:
        return []

    added = []
    for pathStr in sorted(resyncedPaths):
        if "." in pathStr or pathStr in view.nodes:
            continue
        prim = stage.GetPrimAtPath(pathStr)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        if str(prim.GetPath().GetParentPath()) != scope:
            continue
        try:
            nodeId = view._addNewNodeFast(prim, stage)
        except Exception as error:
            Tf.Warn("Graph validation could not add %s: %s" % (pathStr, error))
            continue
        if nodeId:
            added.append(nodeId)
    return added


def remapMovedNodes(view, movedPaths):
    """Carry nodes across a namespace move instead of dropping them.

    A node id IS a prim path, so a rename or reparent changes the identity
    of every node at or under the moved prim. The alternative -- rebuilding
    the graph from a loader -- cannot work here, because graph membership
    is not derivable from the stage: 'A' (``addNodesFromPrimTreeSelection``)
    puts prims at ANY depth on the canvas and 'D' takes them off again
    without touching USD, so the set of nodes on screen is the user's, not
    the loader's. Re-keying is the only operation that preserves it.

    Everything keyed by the old id moves with it: the node dict, the
    NodeModel's own identity and its expired ``Usd.Prim``, the selection and
    marquee sets, the spatial index, and the scattered cursor / drag / hover
    ids. Links are NOT re-keyed -- ``_rebuildLinks`` re-derives every one of
    them from USD, which is also what picks up the relationship targets and
    connections USD retargeted as part of the move.

    Idempotent: paths with no node behind them are skipped, so it is safe to
    call from the authoring site as well as from the notice, whichever gets
    there first.

    Args:
        movedPaths: ``{oldPrimPath: newPrimPath}`` as strings.

    Returns:
        dict[str, str]: the ``{oldNodeId: newNodeId}`` actually applied.
    """
    if not movedPaths:
        return {}

    stage = view.nodeGraph.getStage()

    # A move of /A implies a move of everything under it, but USD reports
    # only the prim that actually moved -- descendants are not enumerated in
    # the notice. Walk the graph instead and rewrite every id that sits at
    # or under a moved path.
    remap = {}
    for nodeId in view.nodes:
        primPath = _nodeIdPrimPath(nodeId)
        for oldPath, newPath in movedPaths.items():
            if not _isPathRelated(primPath, oldPath) or len(primPath) < len(
                oldPath
            ):
                continue
            newId = newPath + nodeId[len(oldPath):]
            if newId != nodeId:
                remap[nodeId] = newId
            break

    if not remap:
        return {}

    for oldId, newId in remap.items():
        node = view.nodes[oldId]
        node.id = newId
        # Synthesized Blueprint I/O nodes are not prim-backed: their id only
        # borrows the blueprint's path, and their title is a pin name. Move
        # the id and leave the rest of the model alone.
        if "#" not in newId:
            # Repoint the model at the prim's new home. The Usd.Prim
            # captured at construction expired the moment the namespace edit
            # landed -- left in place it reads as "this node's prim is
            # gone", and the next resync would have
            # validateNodesAgainstStage drop the node we just rescued.
            node._primPath = Sdf.Path(newId)
            if stage is not None:
                node._prim = stage.GetPrimAtPath(newId)
            # The title is the prim's name, and the name changed.
            node.name = Sdf.Path(newId).name
        # Hit-testing resolves through the spatial index, and the link
        # rebuild that refills it is deferred to the next paint -- drop the
        # dead key now so a click landing first cannot hit it.
        try:
            view._spatialIndex.removeNode(oldId)
        except Exception:
            pass

    # Re-key the node dict in one pass, preserving insertion order: it is
    # the draw and iteration order, and a pop/re-insert would send every
    # renamed node to the back.
    view.nodeGraph.nodes = {
        remap.get(nodeId, nodeId): node for nodeId, node in view.nodes.items()
    }

    def _remapId(value):
        return remap.get(value, value)

    def _remapIdInTuple(value, index=0):
        if not value:
            return value
        asList = list(value)
        asList[index] = _remapId(asList[index])
        return tuple(asList)

    view._selectedNodes = {_remapId(n) for n in view._selectedNodes}
    view._marqueeBaseSelectedNodes = {
        _remapId(n) for n in view._marqueeBaseSelectedNodes
    }
    view._marqueePrevSelectedNodes = {
        _remapId(n) for n in view._marqueePrevSelectedNodes
    }
    view._reconciledPositionVersions = {
        _remapId(n): v for n, v in view._reconciledPositionVersions.items()
    }
    view.nodeIdUnderCursor = _remapId(view.nodeIdUnderCursor)
    view._renamingNodeId = _remapId(view._renamingNodeId)
    # An open value editor / drag / popup follows the prim it is editing,
    # for the same reason the rename does: the node is still on screen and
    # still the one being typed into, only its id changed.
    view._valueEditTarget = _remapIdInTuple(view._valueEditTarget)
    for _state in (view._mungState, view._tokenPopup):
        if _state and _state.get("nodeId") in remap:
            _state["nodeId"] = remap[_state["nodeId"]]
            _state["primPath"] = _nodeIdPrimPath(_state["nodeId"])
    view._hoveredPort = _remapIdInTuple(view._hoveredPort)
    view._tooltipProperty = _remapIdInTuple(view._tooltipProperty)
    view._dragLinkSourceNode = _remapId(view._dragLinkSourceNode)
    view._dragLinkSyntheticTarget = _remapIdInTuple(view._dragLinkSyntheticTarget)
    view._dragLinkResolvedTarget = _remapIdInTuple(view._dragLinkResolvedTarget)
    view._reconnectOldSourceNodeId = _remapId(view._reconnectOldSourceNodeId)
    view._reconnectOldTargetNodeId = _remapId(view._reconnectOldTargetNodeId)

    # The blueprint / container graphs scope themselves to the prim they were
    # opened on (see graphScopeParentPath). If that prim moved, the scope has
    # to move with it or reconciliation starts measuring against a path that
    # no longer resolves and empties the graph.
    if view._currentPrimPath is not None:
        currentStr = str(view._currentPrimPath)
        for oldPath, newPath in movedPaths.items():
            if _isPathRelated(currentStr, oldPath) and len(currentStr) >= len(
                oldPath
            ):
                view._currentPrimPath = Sdf.Path(
                    newPath + currentStr[len(oldPath):]
                )
                break

    # Node ids are baked into the vertex, text and icon caches, so the old
    # id's geometry would linger while the new id drew nothing. Links are
    # re-derived from USD by the rebuild this schedules, which is also what
    # refills the spatial index.
    view._invalidateBackReferenceLinkDataCache()
    view._nodeRenderManager.invalidateAll()
    view._resetRenderCachesAfterBulkMove()

    Tf.Status(
        "Graph followed %d node(s) across a namespace move: %s"
        % (len(remap), ", ".join(f"{o} -> {n}" for o, n in sorted(remap.items())))
    )
    return remap


def applyUsdChanges(
    view, resyncedPaths, infoChangedPaths, movedPaths=None, trace=None
):
    """Handle USD changes detected via the Notice system.

    This is called by UsdNoticeHandler (through the service's receiver, and
    through the view's delegator for direct callers) when USD prims/properties
    change. It invalidates caches and triggers redraws as needed.

    Args:
        resyncedPaths: Set of path strings that had structural changes
                      (prims added/removed, connections changed)
        infoChangedPaths: Set of path strings that had property value changes
                         (positions, metadata)
        movedPaths: Optional ``{oldPath: newPath}`` for prims USD reports as
                    renamed or reparented rather than deleted and recreated.
        trace: Optional ``_ReconcileTrace`` publishing what this pass decided
               (see Remap / Remove / Adopt / Invalidate).
    """
    # Ignore changes if we're in the middle of syncing selection
    # (those are likely our own edits)
    if view._syncingSelection:
        return

    # Follow moved prims FIRST. Both loops below, and the reconciliation
    # that follows them, ask "is this path one of my nodes?" -- so the
    # nodes have to be sitting under their new paths before any of that
    # runs, or the move reads as a delete of the old path plus an
    # unrelated create of the new one.
    movedNodes = remapMovedNodes(view, movedPaths) if movedPaths else {}
    if trace is not None and movedNodes:
        trace.remap(tuple(movedNodes.items()))

    needsLinkRebuild = False
    needsRepaint = False
    needsTextUpdate = False
    if movedNodes:
        needsLinkRebuild = True
        needsRepaint = True
        needsTextUpdate = True

    # Resynced paths represent structural USD changes (prims added/removed,
    # connection authored/cleared). Drop the cached per-prim back-reference
    # scan so _addBackReferenceDanglingLinks re-traverses the stage on the
    # next _rebuildLinks. Info-only changes (positions, metadata) do not
    # affect what USD authored, so they leave the cache intact.
    if resyncedPaths:
        view._invalidateBackReferenceLinkDataCache()

    # Process resynced paths (structural changes - more expensive)
    for pathStr in resyncedPaths:
        # Check if this is a node we have in our graph
        if pathStr in view.nodes:
            node = view.nodes[pathStr]
            node.invalidateCache()
            needsLinkRebuild = True
            needsRepaint = True
            needsTextUpdate = True
        else:
            for nodeId in view.nodes:
                if _isPathRelated(pathStr, nodeId):
                    view.nodes[nodeId].invalidateCache()
                    needsLinkRebuild = True
                    needsRepaint = True
                    needsTextUpdate = True
                    break

    # Process info-only changes (property value changes - cheaper)
    for pathStr in infoChangedPaths:
        # Property paths look like: /Prim.property or /Prim.inputs:foo
        if "." in pathStr:
            primPath, propName = pathStr.rsplit(".", 1)
        else:
            primPath = pathStr
            propName = ""

        if primPath in view.nodes:
            node = view.nodes[primPath]

            # Check what kind of property changed
            if "ui:nodegraph:node:pos" in propName:
                node._position = None
                needsRepaint = True
            elif "ui:nodegraph:node:expansionState" in propName:
                # Expansion state changed - update title collapse
                try:
                    from pxr import UsdUI

                    exp_attr = UsdUI.NodeGraphNodeAPI(
                        node._prim
                    ).GetExpansionStateAttr()
                    if exp_attr.IsValid() and exp_attr.HasAuthoredValue():
                        state = str(exp_attr.Get())
                        collapsed = state in ("closed", "minimized")
                    else:
                        collapsed = False
                    if node._title_collapsed != collapsed:
                        node._title_collapsed = collapsed
                        node.titleCollapsed = collapsed
                        needsRepaint = True
                        needsTextUpdate = True
                        needsLinkRebuild = True
                except Exception:
                    pass
            elif view._cachedShowAttributeValues and view._valueRowOnlyChange(
                node, propName
            ):
                # A value the node already shows in an inline cell changed.
                # Refresh that ONE node's cells and stop. Guarded by the
                # feature flag FIRST: with values hidden the row cache is
                # cold, and asking it would build every row (a full schema
                # walk plus a USD read per pin) only to throw them away.
                #
                # Deliberately not textChanged: that re-lays out EVERY node
                # in the graph (paintNodes, whose own comment records ~2s
                # for a dozen nodes). A namespaced attribute like
                # rigExec:controlSpace matches _isPinPropertyChange below,
                # so without this branch a value drag would pay a full
                # graph relayout per mouse-move. Nothing structural can
                # have changed here: a type or row-set change is a resync,
                # and the resync loop above already invalidated the node.
                node.invalidate_value_row(propName)
                needsRepaint = True
            elif view._isPinPropertyChange(propName):
                # Pin or connection changed (direction-hint, relationship,
                # or any other namespaced pin such as rig1:space)
                node.invalidateCache()
                needsLinkRebuild = True
                needsRepaint = True
                needsTextUpdate = True
            elif "info:implementationSource" in propName:
                # Node type changed
                node._cachedType = None
                needsRepaint = True
                needsTextUpdate = True
            else:
                # Some other property - do a full cache invalidation to be safe
                node.invalidateCache()
                needsRepaint = True
                # Bare attributes may have connections - trigger link rebuild.
                #
                # ``propName`` is empty when the notice named the PRIM and
                # not a property (pathStr had no '.'), and
                # ``GetAttribute("")`` is not a miss -- it raises a USD
                # coding error from _GetDefiningSpecType. Harmless while
                # nothing was watching, but an edit authored inside an
                # Sdf.ChangeBlock dispatches its notices at the block's
                # close, so the error came back out of attr.Set and made a
                # write that had ALREADY landed report failure.
                if propName and ":" not in propName and node._prim:
                    attr = node._prim.GetAttribute(propName)
                    if attr and attr.IsValid() and attr.HasAuthoredConnections():
                        needsLinkRebuild = True

    # Reconcile the graph with the stage. The loops above only touch paths
    # that are already nodes, so neither of them can notice that a prim was
    # deleted, deactivated or renamed out from under one.
    if resyncedPaths:
        removed = validateNodesAgainstStage(view)
        adopted = adoptPrimsNewToScope(view, resyncedPaths)
        if removed or adopted:
            if trace is not None:
                trace.remove(tuple(removed))
                trace.adopt(tuple(adopted))
            needsLinkRebuild = True
            needsRepaint = True
            needsTextUpdate = True

    if trace is not None and (
        needsLinkRebuild or needsTextUpdate or needsRepaint
    ):
        trace.invalidate(
            tuple(sorted(set(resyncedPaths) | set(infoChangedPaths))),
            needsLinkRebuild,
            needsTextUpdate,
            needsRepaint,
        )

    # Apply the updates
    if needsLinkRebuild:
        view.linksChanged = True
        view._clearRenderCache()

    if needsTextUpdate:
        view.textChanged = True

    if needsRepaint:
        view.update()


# ---------------------------------------------------------------------------
# Stage replace / reload / close policy
# ---------------------------------------------------------------------------


def stageLoadMode(view):
    """Which loader a reload of this view has to re-run.

    The mode is the view's own history, not the stage's: a container graph and
    a blueprint graph both scope themselves to a prim, and a plain graph does
    not, so the choice follows the graph that is currently loaded -- read it
    BEFORE any loader replaces it.
    """
    from ..nodeGraphBpContainer import NodeGraphBpContainer

    if (
        isinstance(view.nodeGraph, NodeGraphBpContainer)
        and view._currentPrimPath
    ):
        return MODE_CONTAINER
    if view._currentPrimPath is not None:
        return MODE_BLUEPRINT
    return MODE_STAGE


def readStageContext(view):
    """One owner-thread read of everything a reload decides from."""
    api = view._usdviewApi
    return StageContext(
        view=view,
        stage=api.stage if api else None,
        mode=stageLoadMode(view),
        prim_path=view._currentPrimPath,
        node_ids=tuple(view.nodes),
        syncing_selection=view._syncingSelection,
    )


def readNoticeContext(view):
    """One owner-thread read for a USD notice -- no whole-graph snapshot."""
    return NoticeContext(
        view=view, syncing_selection=view._syncingSelection
    )


def _beginReloadOnView(view, frame):
    # Block paint events at the Qt level.  Python-level guards
    # (view.initialized) are insufficient because QGLWidget's internal event
    # handling still touches the GL surface before paintGL() runs, and on
    # NVIDIA Windows drivers this can crash if the stageView's context was
    # just destroyed/recreated on the same thread.
    view.setUpdatesEnabled(False)
    view.initialized = False

    view._noticeHandler.unregister()
    view._invalidateBackReferenceLinkDataCache()

    # A value drag / editor / popup pinned to the old stage must not survive
    # into the new one.
    view._clearValueInteractionState()
    view._currentTimeCode = frame

    # Clear state that holds dead Usd.Prim / Usd.Stage references
    view._lastPrimTreeSelection = []
    view._lastSelectedNode = None
    view._selectedNodes.clear()
    view._selectedLinks.clear()
    view._linkUnderCursor = -1
    # Drop per-node position watermarks for the torn-down graph; the next
    # graph's nodes reconcile fresh (and this bounds the map's growth).
    view._reconciledPositionVersions.clear()
    view._lastPinIndex = -1
    view._lastLinkCycleIndex = 0
    view._lastInputLinkIndex = -1
    view._lastOutputLinkIndex = -1
    view._lastDanglingLinkIndex = -1


def _loadOnView(view, delta):
    if delta.mode == MODE_CONTAINER:
        view.loadContainer(delta.stage, delta.prim_path)
    elif delta.mode == MODE_BLUEPRINT:
        view.loadBlueprint(delta.stage, delta.prim_path)
    else:
        view.loadStage(delta.stage)


def _finishReloadOnView(view):
    # Do NOT set view.fontAtlas/view.shaderLibrary to None here.
    # CPython's refcounting would immediately call their C++
    # destructors (which call glDelete*) with no GL context
    # current, corrupting the NVIDIA driver state.  Instead let
    # initializeGL() replace them while the context IS current --
    # the old destructors then run safely.

    # Run the render-cache clear that the loaders deferred, now that the
    # driver state has stabilized.  Doing it during BeginReload would race
    # with the stageView's renderer teardown and crash the NVIDIA driver on
    # its worker thread.
    if view._renderCacheClearPending:
        view._renderCacheClearPending = False
        view.makeCurrent()
        try:
            view._nodeRenderManager.clearCache()
        finally:
            view.doneCurrent()

    view.linksChanged = True
    view.textChanged = True
    view.initialized = True
    view.setUpdatesEnabled(True)
    view.update()


def applyStageDelta(view, delta):
    """Perform one reload delta on *view*, synchronously, owner thread.

    Remap / Remove / Adopt / Invalidate are records of work reconciliation
    already did, so they carry nothing to apply here.
    """
    if isinstance(delta, BeginReload):
        _beginReloadOnView(view, delta.frame)
    elif isinstance(delta, Load):
        _loadOnView(view, delta)
    elif isinstance(delta, ClearHistory):
        view._undoHistory = None
    elif isinstance(delta, FinishReload):
        _finishReloadOnView(view)


def closeStageLifecycle(view):
    """Close an editor's stage lifecycle, exactly once.

    Both Qt close paths call it: the widget's own closeEvent and the dock
    window's, which Qt does not forward to the child. The closed service is
    the guard -- a second call must not move the epoch again or revoke a
    receiver that is already gone, because every queued delta and notice is
    dropped by comparing against that epoch.
    """
    service = getattr(view, "_stageSync", None)
    if service is None or service.isClosed():
        return
    unsubscribe_edit_target_warning(view._showWarningPopup)
    view._undoHistory = None
    service.close()


class _ReconcileTrace:
    """Builds the reconcile records for one pass and hands them to the sink."""

    def __init__(self, sink, epoch):
        self._sink = sink
        self._epoch = epoch

    def _publish(self, record):
        if self._sink is not None:
            self._sink(record)

    def remap(self, pairs):
        self._publish(Remap(self._epoch, pairs))

    def remove(self, ids):
        self._publish(Remove(self._epoch, ids))

    def adopt(self, paths):
        self._publish(Adopt(self._epoch, paths))

    def invalidate(self, paths, links, text, paint):
        self._publish(
            Invalidate(self._epoch, paths, links, text, paint)
        )


class _NoticeReceiver:
    """What the UsdNoticeHandler calls, stamped with a stage generation.

    ``UsdNoticeHandler`` keeps its protocol -- an object with
    ``_handleUsdChanges(resyncedPaths, infoChangedPaths, movedPaths=None)`` --
    but in production the object behind it is this receiver, not the widget.
    It carries the epoch of its binding, so a delivery that no longer matches
    the live generation never reaches the model.
    """

    def __init__(self, service):
        self._service = service
        self._epoch = service.epoch()
        self._revoked = False

    def rebind(self, epoch):
        self._epoch = epoch
        self._revoked = False

    def revoke(self):
        self._revoked = True

    def _handleUsdChanges(
        self, resyncedPaths, infoChangedPaths, movedPaths=None
    ):
        if self._revoked:
            return
        self._service.changed(
            self._epoch, resyncedPaths, infoChangedPaths, movedPaths
        )


class StageSync:
    """One editor's stage lifecycle: replace, bind, notice, finish, close.

    Constructed per editor with the callables the view supplies -- ``read()``
    for one owner-thread read of the view for a reload, ``notice()`` for the
    narrower read a USD notice needs, ``apply(delta)`` to perform a mutation
    synchronously on the owner thread, ``post(callback)`` to schedule on the
    widget owner thread, ``listen(receiver)`` to build the editor's
    UsdNoticeHandler, and ``warn(message)`` for the diagnostics channel. None
    of them may import Qt or GL here, which is what lets the whole lifecycle
    run headless against a recording stub.

    Every replace moves one *epoch*. Deferred callbacks and notice deliveries
    carry the epoch they were produced under, and anything that no longer
    matches the live epoch is dropped: a stage replace in flight cannot be
    undone by a timer tick from the previous stage.
    """

    def __init__(self, read, notice, apply, post, listen, warn=None, sink=None):
        self._read = read
        self._notice = notice
        self._apply = apply
        self._post = post
        self._warn = warn
        self._sink = sink
        self._epoch = 0
        self._boundStage = None
        self._closed = False
        self._receiver = _NoticeReceiver(self)
        self._handler = listen(self._receiver)

    # -- the editor's notice handler -------------------------------------

    def handler(self):
        """The UsdNoticeHandler built by ``listen``.

        The view keeps it as ``view._noticeHandler``: the authoring sites
        suppress through that object, and the loaders register the stage on it
        (``register`` unregisters first), so this service never registers a
        second listener.
        """
        return self._handler

    def epoch(self):
        return self._epoch

    def isClosed(self):
        return self._closed

    @contextmanager
    def suppressed(self):
        """Ignore USD notices for the duration of the block.

        The state lives on the editor's UsdNoticeHandler, and the value in
        front when the block opened is restored on the normal path and on an
        exception, so nesting cannot leave the editor deaf.
        """
        handler = self._handler
        previous = handler.isEnabled()
        handler.setEnabled(False)
        try:
            yield
        finally:
            handler.setEnabled(previous)

    # -- replace / reload -------------------------------------------------

    def replace(self, stage, mode, prim_path, frame):
        """usdview replaced the stage: tear the graph down, rebuild it.

        The epoch moves first, so every callback an earlier replace queued --
        and the notice receiver stamped with the old stage -- is stale before
        any teardown work starts.

        A closed editor ignores the delivery entirely: the widget can outlive
        its close event, and a replacement that got through would re-arm the
        notice receiver and reload a graph nobody is displaying.
        """
        if self._closed:
            return
        ctx = self._read()
        view = ctx.view
        self._epoch += 1
        epoch = self._epoch

        begin = BeginReload(epoch, frame)
        self._publish(begin)
        self._apply(begin)
        self.bind(stage)

        if stage is None:
            clear = ClearHistory(epoch)
            self._publish(clear)
            self._apply(clear)
            self._scheduleFinish(epoch)
            return

        # The loaders build a whole new nodeGraph, so every id in the graph
        # read above is gone; recorded rather than applied.
        self._publish(Remove(epoch, ctx.node_ids))
        load = Load(epoch, stage, mode, prim_path)
        self._publish(load)

        # While this is set, the _clearRenderCache the loaders run only marks
        # the clear pending: the glDelete* calls wait for the finish, when the
        # NVIDIA driver is out of its non-functional window.
        view._deferRenderCacheClear = True
        try:
            self._apply(load)
        except Exception as error:
            if self._warn is not None:
                self._warn(
                    "Failed to reload graph after stage replace: %s" % error
                )
        finally:
            view._deferRenderCacheClear = False

        # A loader can bring the widget back up through initializeGL; the
        # finish below is what decides the generation is live again.
        view.initialized = False
        self._scheduleFinish(epoch)

    def bind(self, stage):
        """Record the stage this generation listens to, and stamp the receiver.

        The loaders call ``register`` on the UsdNoticeHandler themselves, so
        nothing here registers -- that would double-register. Re-binding
        replaces whatever epoch the receiver carried, which is what revokes
        the previous binding: a notice from a replaced stage cannot reach the
        model.
        """
        self._boundStage = stage
        self._receiver.rebind(self._epoch)
        return stage

    def finish(self, epoch):
        """Bring one generation's rendering back up -- the latest live one only.

        A stale epoch does nothing at all: its render caches belong to a graph
        that no longer exists, and re-enabling paint would let the widget touch
        a GL surface the newer replace has already torn down.
        """
        if self._closed or epoch != self._epoch:
            return
        delta = FinishReload(epoch)
        self._publish(delta)
        self._apply(delta)

    def close(self):
        """Revoke the notice receiver and invalidate anything still queued."""
        self._closed = True
        # Past every queued finish and every receiver stamp: the epoch moving
        # is what makes the deferred callbacks return without touching the
        # model.
        self._epoch += 1
        self._boundStage = None
        self._receiver.revoke()
        if self._handler is not None:
            self._handler.unregister()

    # -- USD notices ------------------------------------------------------

    def changed(self, epoch, resyncedPaths, infoChangedPaths, movedPaths=None):
        """One USD notice, delivered under the epoch that bound its stage."""
        if self._closed or epoch != self._epoch:
            return
        ctx = self._notice()
        if ctx.syncing_selection:
            return
        applyUsdChanges(
            ctx.view,
            resyncedPaths,
            infoChangedPaths,
            movedPaths,
            trace=(
                _ReconcileTrace(self._sink, epoch)
                if self._sink is not None
                else None
            ),
        )

    # -- internals --------------------------------------------------------

    def _publish(self, delta):
        if self._sink is not None:
            self._sink(delta)

    def _scheduleFinish(self, epoch):
        """One finish per replace, on the widget owner thread.

        The callback carries its epoch and the binding it was queued under:
        a replace or a close since then makes it stale, and a stale finish
        must not re-enable paint on a graph that has moved on.
        """
        binding = self._boundStage

        def _run():
            if (
                self._closed
                or epoch != self._epoch
                or binding is not self._boundStage
            ):
                return
            self.finish(epoch)

        self._post(_run)
