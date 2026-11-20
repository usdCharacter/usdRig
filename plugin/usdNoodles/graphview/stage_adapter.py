#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Owner-thread view effects for stage reconciliation, reload and close.

Reconciliation reads USD without authoring opinions and updates the graph
through the view's state and hooks.
"""

from pxr import Sdf, Tf

from UsdNoodles.nodeGraph import unsubscribe_edit_target_warning

from . import model, selection
from .stage_sync import (
    MODE_BLUEPRINT,
    MODE_CONTAINER,
    MODE_STAGE,
    BeginReload,
    ClearHistory,
    FinishReload,
    Load,
    NoticeContext,
    StageContext,
    _isPathRelated,
    _nodeIdPrimPath,
)


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


def _dropNodeId(view, nodeId):
    """Forget one node id everywhere the view keys by it."""
    state = model.state_for_view(view)
    if state is not None:
        state.apply(model.GraphDelta(ops=(model.RemoveNode(nodeId),)))
    else:
        view.nodes.pop(nodeId, None)
    if hasattr(view, "_selection"):
        selection.reconcile(view)
    else:
        view._selectedNodes.discard(nodeId)
        view._marqueeBaseSelectedNodes.discard(nodeId)
        view._marqueePrevSelectedNodes.discard(nodeId)
    if state is not None:
        return
    # _rebuildLinks rebuilds the whole index, but that is deferred to
    # the next paint; drop it now so a click landing first cannot hit
    # a node that no longer exists.
    try:
        view._spatialIndex.removeNode(nodeId)
    except Exception:
        pass


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
        _dropNodeId(view, nodeId)

    if stale:
        Tf.Status("Graph validation dropped %d node(s) whose prims are "
                  "gone: %s" % (len(stale), ", ".join(sorted(stale))))
    return stale


def _addLivePrimNode(view, pathStr):
    """Build a fresh node from the live prim at *pathStr*, or None.

    A path already shown resolves to itself, so a notice that adopted the
    prim and the authoring site that follows it cannot double-add.
    """
    if pathStr in view.nodes:
        return pathStr
    stage = view.nodeGraph.getStage()
    if not stage:
        return None
    prim = stage.GetPrimAtPath(pathStr)
    if not prim or not prim.IsValid() or not prim.IsActive():
        return None
    try:
        return view._addNewNodeFast(prim, stage) or None
    except Exception as error:
        Tf.Warn("Graph validation could not add %s: %s" % (pathStr, error))
        return None


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
        if str(Sdf.Path(pathStr).GetParentPath()) != scope:
            continue
        nodeId = _addLivePrimNode(view, pathStr)
        if nodeId:
            added.append(nodeId)
    return added


def _movedDestinationForPath(pathStr, movedPaths):
    """The destination of *pathStr* through USD-classified move pairs.

    USD reports only the prim that actually moved; a displayed descendant
    resolves by path-component prefix replacement against the most
    specific source. No stage walk: callers pass displayed ids only, so
    an undisplayed sibling is never admitted by the move alone.
    """
    if not movedPaths:
        return None
    try:
        candidate = Sdf.Path(pathStr)
    except Exception:
        return None
    bestOld = None
    bestNew = None
    bestLen = -1
    for oldPath, newPath in movedPaths.items():
        try:
            old = Sdf.Path(oldPath)
            new = Sdf.Path(newPath)
        except Exception:
            continue
        try:
            if not candidate.HasPrefix(old):
                continue
        except Exception:
            continue
        if len(oldPath) > bestLen:
            bestLen = len(oldPath)
            bestOld = old
            bestNew = new
    if bestOld is None:
        return None
    try:
        return str(candidate.ReplacePrefix(bestOld, bestNew))
    except Exception:
        return None


def retargetMovedScope(view, movedPaths):
    """Follow the blueprint/container scope across a namespace move.

    The blueprint and container graphs scope themselves to the prim they
    were opened on (see ``graphScopeParentPath``). When that prim -- or
    one of its ancestors -- moves, the scope has to move with it or
    reconciliation starts measuring against a path that no longer
    resolves: new children never appear and the graph disagrees with
    what a reload would produce. Runs before child adoption, which
    measures against the scope.

    Only USD-classified pairs are followed; a delete plus an unrelated
    create never retargets the scope.

    Returns:
        tuple[str, str] | None: the ``(oldScope, newScope)`` the view
        moved across, or None when the scope did not move.
    """
    if not movedPaths:
        return None
    current = getattr(view, "_currentPrimPath", None)
    if current is None:
        return None
    currentStr = str(current)
    newScope = _movedDestinationForPath(currentStr, movedPaths)
    if newScope is None or newScope == currentStr:
        return None
    view._currentPrimPath = Sdf.Path(newScope)
    return (currentStr, newScope)


def rebuiltScopePins(view, oldScope, newScope):
    """Fresh I/O models for a moved scope, generated from the destination.

    Blueprint and container I/O nodes are synthesized, not prim-backed:
    their ids borrow the scope path (``/Bp#input:x``) and their pins come
    from the scope prim's attributes and its children's connection
    metadata. A namespace move carries those to the new scope, so the
    fresh models are exactly what the loader run against the moved stage
    would emit -- the shared ``scope_endpoints`` derivation, under the
    graph's own fieldname mode -- never a copy of the cached pin lists,
    which may hold view-only pins the stage never authored.

    Only display placement carries over, for pins the destination still
    has under the same complete suffix: the setters below never author
    USD.

    Returns:
        dict[str, tuple]: ``{oldId: (newId, freshModel)}``.
    """
    from UsdNoodles.core import FontMetrics

    from ..models import NodeModel
    from ..nodeGraphBlueprint import NodeGraphBlueprint
    from ..nodeGraphBpContainer import NodeGraphBpContainer
    from ..scopeEndpoints import (
        accept_blueprint_child,
        accept_container_child,
        endpoint_origins,
        endpoint_position,
        scope_endpoints,
    )

    if isinstance(view.nodeGraph, NodeGraphBpContainer):
        inputType, outputType = "ContainerInput", "ContainerOutput"
        acceptChild = accept_container_child
    elif isinstance(view.nodeGraph, NodeGraphBlueprint):
        inputType, outputType = "BlueprintInput", "BlueprintOutput"
        acceptChild = accept_blueprint_child
    else:
        return {}

    stage = view.nodeGraph.getStage()
    if stage is None:
        return {}
    try:
        scopePrim = stage.GetPrimAtPath(newScope)
    except Exception:
        return {}
    if not scopePrim or not scopePrim.IsValid():
        return {}

    newScopeStr = str(newScope)
    oldScopeStr = str(oldScope)
    try:
        inputSpecs, outputSpecs = scope_endpoints(
            scopePrim,
            newScopeStr,
            getattr(view.nodeGraph, "_useFieldnames", True),
            acceptChild,
        )
    except Exception:
        return {}

    if not (inputSpecs or outputSpecs):
        return {}

    fontAtlas = getattr(view, "fontAtlas", None)
    fontMetrics = fontAtlas if fontAtlas is not None else FontMetrics()
    measure = (
        view.textRenderer.calculateTextWidth
        if fontAtlas is not None
        else _measureScopeTextWithoutAtlas
    )
    sizeNode = view.nodeGraph._calculateNodeSize
    factory = getattr(view, "_nodeFactory", None)

    # Use destination child bounds, including children not on the canvas.
    # These temporary models are never installed or used for adoption.
    children = []
    for child in scopePrim.GetChildren():
        if not acceptChild(child):
            continue
        if factory is not None:
            node = factory.create_node_from_prim(child, stage)
            if node is None:
                continue
        else:
            node = NodeModel(stage=stage, primPath=child.GetPath())
        # Populate the lazy USD fields consumed by the native layout pass.
        _ = (
            node.position,
            node.name,
            node.type,
            node.schemaTypeName,
            node.inputPins,
            node.outputPins,
            node.inputPinTypes,
            node.outputPinTypes,
        )
        sizeNode(node, measure, fontMetrics)
        children.append(node)
    inputX, outputX, startY = endpoint_origins(children)

    rebuilt = {}
    for isInput, specs, x in (
        (True, inputSpecs, inputX),
        (False, outputSpecs, outputX),
    ):
        for index, (suffix, displayName) in enumerate(specs):
            newId = newScopeStr + suffix
            oldId = oldScopeStr + suffix
            node = NodeModel()
            node.id = newId
            node.name = displayName
            if isInput:
                node.type = inputType
                node.outputPins = [displayName]
            else:
                node.type = outputType
                node.inputPins = [displayName]
            node.setDisplayPosition(endpoint_position(x, startY, index))
            sizeNode(node, measure, fontMetrics)
            old = view.nodes.get(oldId)
            if old is not None:
                position = getattr(old, "position", None)
                if position is not None:
                    try:
                        node.setDisplayPosition(position)
                    except Exception:
                        pass
                zOrder = getattr(old, "zOrder", None)
                if zOrder is not None:
                    try:
                        node.zOrder = zOrder
                    except Exception:
                        pass
            rebuilt[oldId] = (newId, node)
    return rebuilt


def _measureScopeTextWithoutAtlas(text, fontSize):
    return len(text) * fontSize * 0.5


def installScopePins(view, rebuilt, oldScope=None):
    """Drop the stale synthetic ids and insert the prebuilt models.

    Args:
        rebuilt: ``{oldId: (newId, freshModel)}`` from ``rebuiltScopePins``.
        oldScope: the scope the view moved away from. Any synthetic id
            still hanging off it afterwards is a stale view-only pin the
            destination loader did not emit, and is dropped.

    Returns:
        list[str]: the fresh synthetic node ids.
    """
    fresh = []
    for oldId, (newId, node) in rebuilt.items():
        if oldId in view.nodes:
            _dropNodeId(view, oldId)
        if newId in view.nodes:
            continue
        state = model.state_for_view(view)
        if state is not None:
            state.apply(model.GraphDelta(ops=(model.InsertNode(newId, node),)))
        else:
            view.nodes[newId] = node
        fresh.append(newId)
    if oldScope is not None:
        try:
            oldPrefix = Sdf.Path(oldScope)
        except Exception:
            oldPrefix = None
        for nodeId in list(view.nodes):
            if "#" not in nodeId:
                continue
            primHalf = _nodeIdPrimPath(nodeId)
            stale = False
            if oldPrefix is not None:
                try:
                    stale = Sdf.Path(primHalf).HasPrefix(oldPrefix)
                except Exception:
                    stale = False
            else:
                stale = _isPathRelated(primHalf, oldScope) and len(
                    primHalf
                ) >= len(oldScope)
            if stale:
                _dropNodeId(view, nodeId)
    return fresh


def capturedDisplayedMoves(view, movedPaths):
    """The displayed nodes a namespace move must rebuild, and their state.

    A node id IS a prim path, so a displayed id at or under a classified
    move source names a node whose prim now lives at the prefix-replaced
    destination -- inside or outside the loader's scope. The destination
    model is built from the live prim after validation drops the source
    id, but the display-only placement has to be read BEFORE that, while
    the source node is still there.

    Only displayed ids resolve: a pair whose source was never displayed
    is an unrelated prim the graph must not admit, and no stage walk
    enumerates undisplayed siblings. Synthetic ids hang off the scope
    and are rebuilt by ``rebuiltScopePins`` instead.

    Returns:
        dict[str, tuple]: ``{sourceId: (destination, position, zOrder)}``.
    """
    if not movedPaths:
        return {}
    nodes = view.nodes
    captured = {}
    for nodeId in list(nodes):
        if "#" in nodeId:
            continue
        destination = _movedDestinationForPath(nodeId, movedPaths)
        if destination is None or destination == nodeId:
            continue
        position, zOrder, _ = savedRenamePlacement(nodes, nodeId)
        captured[nodeId] = (destination, position, zOrder)
    return captured


def adoptMovedDisplayedPrims(view, captured):
    """Rebuild captured moves from their live destinations.

    The fresh model is built from the prim at the destination path, so
    node identity is always whatever the stage holds now -- never a
    re-keyed object carried across the edit. Display-only placement
    survives the same way the inline rename does it; an authored
    position is left to the fresh model to read back from USD.

    Returns:
        list[str]: node ids adopted.
    """
    adopted = []
    for oldPath in sorted(captured):
        newPath, position, zOrder = captured[oldPath]
        if newPath in view.nodes:
            continue
        if _addLivePrimNode(view, newPath) is None:
            continue
        node = view.nodes[newPath]
        if zOrder is not None:
            node.zOrder = zOrder
        if position is not None and not node.hasAuthoredPosition():
            node.setDisplayPosition(position)
        adopted.append(newPath)
    return adopted


def savedRenamePlacement(nodes, oldNodeId):
    """The display-only state a rename must carry onto the rebuilt node.

    Read BEFORE the namespace edit: the resync notice it emits can drop the
    old node -- and adopt the new prim at the back -- before the authoring
    site continues. Missing node, missing state: a rename of a path the
    canvas does not show adopts fresh.
    """
    node = nodes.get(oldNodeId)
    try:
        index = list(nodes).index(oldNodeId)
    except ValueError:
        index = None
    return getattr(node, "position", None), getattr(node, "zOrder", None), index


def adoptRenamedPrim(
    view,
    oldNodeId,
    newPathStr,
    displayPosition=None,
    zOrder=None,
    oldIndex=None,
):
    """Follow one rename by rebuilding the node from the current stage.

    A rename is a delete plus an add: the node at the old id is dropped and
    a fresh model is built from the live prim at the new path, so node
    identity is always whatever the stage holds now -- never a re-keyed
    object carried across the edit. The adoption is scope-independent: the
    authoring site names the prim the user just renamed, which a notice
    alone could not re-add outside the loader's scope.

    Validation runs first, so a rename whose notice was suppressed (an
    enclosing edit) still drops the old node and any descendant nodes whose
    prims expired with it; when the notice already ran this is a no-op.
    Display-only state -- a grid placement, the draw order, the draw slot --
    has no USD backing, so the caller snapshots it before the namespace edit
    (see ``savedRenamePlacement``) and it is re-applied onto the fresh model
    here, the same split ``_reloadCurrentGraph`` uses. An authored position
    is left alone: the rename carries the attribute to the new path and the
    fresh model reads it back.

    Args:
        oldNodeId: node id naming the prim's path before the rename.
        newPathStr: the prim's path after the rename.
        displayPosition: the old node's cached position, re-applied only
            when the fresh model has no authored position.
        zOrder: the old node's draw order, restored onto the fresh model.
        oldIndex: the old node's slot in the node order, which the fresh
            model moves back to.

    Returns:
        str | None: the adopted node id, or None when the new path holds
        no live prim -- the old node is dropped either way.
    """
    validateNodesAgainstStage(view)

    nodeId = _addLivePrimNode(view, newPathStr)
    if nodeId is None:
        view._invalidateBackReferenceLinkDataCache()
        view._nodeRenderManager.invalidateAll()
        view._resetRenderCachesAfterBulkMove()
        return None

    # Dict order is draw order: the adoption appended the fresh model, so
    # move it back to the slot the rename vacated.
    if oldIndex is not None:
        nodeIds = list(view.nodes)
        nodeIds.remove(nodeId)
        nodeIds.insert(min(oldIndex, len(nodeIds)), nodeId)
        view.nodeGraph.nodes = {key: view.nodes[key] for key in nodeIds}

    node = view.nodes[nodeId]
    if zOrder is not None:
        node.zOrder = zOrder
    if displayPosition is not None and not node.hasAuthoredPosition():
        node.setDisplayPosition(displayPosition)

    # The rename retargeted the connections and relationships that named the
    # old path; the rebuild this schedules re-derives every link from USD,
    # which also refills the spatial index.
    view._invalidateBackReferenceLinkDataCache()
    view._nodeRenderManager.invalidateAll()
    view._resetRenderCachesAfterBulkMove()

    Tf.Status(
        "Graph adopted the renamed prim %s from the stage (was %s)"
        % (nodeId, oldNodeId)
    )
    return nodeId


def _nodePositionChange(node):
    """An authored position moved: only the cached position is stale."""
    node._position = None
    return (True, False, False)


def _nodeExpansionChange(node):
    """Expansion state decides the title collapse, which repaints, relayouts
    and re-derives the links that hang off the collapsed header."""
    try:
        from pxr import UsdUI

        exp_attr = UsdUI.NodeGraphNodeAPI(node._prim).GetExpansionStateAttr()
        if exp_attr.IsValid() and exp_attr.HasAuthoredValue():
            state = str(exp_attr.Get())
            collapsed = state in ("closed", "minimized")
        else:
            collapsed = False
        if node._title_collapsed != collapsed:
            node._title_collapsed = collapsed
            node.titleCollapsed = collapsed
            return (True, True, True)
    except Exception:
        pass
    return (False, False, False)


def _nodeValueRowChange(node, propName):
    """A value the node already shows inline changed: refresh only that node's
    cells. Not textChanged -- that re-lays out every node, and nothing
    structural changed here; a row-set change arrives as a resync."""
    node.invalidate_value_row(propName)
    return (True, False, False)


def _nodePinChange(node):
    """Pin or connection changed: the graph's links are derived from USD."""
    node.invalidateCache()
    return (True, True, True)


def _nodeTypeChange(node):
    """Node type changed."""
    node._cachedType = None
    return (True, False, True)


def _otherPropertyChange(node, propName):
    """Any other property: invalidate the whole cache to be safe.

    Bare attributes may have connections, so they trigger a link rebuild.
    ``propName`` is empty when the notice named the prim rather than a
    property, and ``GetAttribute("")`` is not a miss -- it raises a USD coding
    error -- so an empty name must never be queried.
    """
    node.invalidateCache()
    if propName and ":" not in propName and node._prim:
        attr = node._prim.GetAttribute(propName)
        if attr and attr.IsValid() and attr.HasAuthoredConnections():
            return (True, True, False)
    return (True, False, False)


def _applyPropertyChange(view, node, propName):
    """Classify one property notice on an existing node.

    Returns ``(needsRepaint, needsLinkRebuild, needsTextUpdate)``.
    """
    if "ui:nodegraph:node:pos" in propName:
        return _nodePositionChange(node)
    if "ui:nodegraph:node:expansionState" in propName:
        return _nodeExpansionChange(node)
    # The feature flag gates the probe FIRST: with values hidden the row cache
    # is cold, and asking it would build every row (a full schema walk plus a
    # USD read per pin) only to throw them away.
    if view._cachedShowAttributeValues and view._valueRowOnlyChange(
        node, propName
    ):
        return _nodeValueRowChange(node, propName)
    if view._isPinPropertyChange(propName):
        return _nodePinChange(node)
    if "info:implementationSource" in propName:
        return _nodeTypeChange(node)
    return _otherPropertyChange(node, propName)


def applyUsdChanges(view, resyncedPaths, infoChangedPaths, trace=None, movedPaths=None):
    """Handle USD changes detected via the Notice system.

    This is called by UsdNoticeHandler (through the service's receiver, and
    through the view's delegator for direct callers) when USD prims/properties
    change. It invalidates caches and triggers redraws as needed. A rename
    is a delete plus an add: the old node is dropped and the new prim is
    adopted from the stage, never re-keyed.

    Args:
        resyncedPaths: Set of path strings that had structural changes
                      (prims added/removed, connections changed)
        infoChangedPaths: Set of path strings that had property value changes
                         (positions, metadata)
        trace: Optional ``_ReconcileTrace`` publishing what this pass decided
               (see Remove / Adopt / Invalidate).
        movedPaths: Optional ``{oldPrimPath: newPrimPath}`` as classified
                    by the USD notice itself. Displayed move sources are
                    rebuilt from their live destinations -- even outside
                    the loader's scope -- and a moved blueprint/container
                    scope is retargeted before child adoption.
    """
    # Ignore changes if we're in the middle of syncing selection
    # (those are likely our own edits)
    if view._syncingSelection:
        return

    with model.transaction_for(view):
        _applyUsdChangesPass(view, resyncedPaths, infoChangedPaths, trace, movedPaths)


def _applyUsdChangesPass(view, resyncedPaths, infoChangedPaths, trace, movedPaths):

    needsLinkRebuild = False
    needsRepaint = False
    needsTextUpdate = False

    # Resynced paths represent structural USD changes (prims added/removed,
    # connection authored/cleared). Drop the cached per-prim back-reference
    # scan so _addBackReferenceDanglingLinks re-traverses the stage on the
    # next _rebuildLinks. Info-only changes (positions, metadata) do not
    # affect what USD authored, so they leave the cache intact.
    if resyncedPaths:
        view._invalidateBackReferenceLinkDataCache()

    moved = movedPaths or {}
    # A moving scope is retargeted -- and its synthetic pins generated
    # from the destination loader inputs -- BEFORE the invalidation pass
    # below: invalidating a prim-less node wipes the pin caches it cannot
    # re-read from USD. The fresh models stay out of the graph until the
    # pass is over (see installScopePins).
    scopeMove = retargetMovedScope(view, moved if resyncedPaths else {})
    freshPins = rebuiltScopePins(view, *scopeMove) if scopeMove else {}

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
            repaint, rebuildLinks, updateText = _applyPropertyChange(
                view, view.nodes[primPath], propName
            )
            needsRepaint |= repaint
            needsLinkRebuild |= rebuildLinks
            needsTextUpdate |= updateText

    # Reconcile the graph with the stage. The loops above only touch paths
    # that are already nodes, so neither of them can notice that a prim was
    # deleted, deactivated or renamed out from under one.
    if resyncedPaths:
        installScopePins(view, freshPins, scopeMove[0] if scopeMove else None)
        # Capture BEFORE validation drops the source ids: the adoption
        # below rebuilds them from their live destinations.
        displayMoves = capturedDisplayedMoves(view, moved)
        removed = validateNodesAgainstStage(view)
        adopted = adoptMovedDisplayedPrims(view, displayMoves)
        adopted.extend(adoptPrimsNewToScope(view, resyncedPaths))
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
# Stage replace / reload / close effects
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
    """Snapshot the old graph's IDs on the owner thread."""
    return StageContext(node_ids=tuple(view.nodes))


def readNoticeContext(view):
    """One owner-thread read for a USD notice -- no whole-graph snapshot."""
    return NoticeContext(syncing_selection=view._syncingSelection)


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
    if hasattr(view, "_selection"):
        view._selection.update("replace", (), ())
    else:
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
    old_state = model.attached_state(view)
    old_graph = getattr(view, "nodeGraph", None)
    if delta.mode == MODE_CONTAINER:
        loader, args = view.loadContainer, (delta.stage, delta.prim_path)
    elif delta.mode == MODE_BLUEPRINT:
        loader, args = view.loadBlueprint, (delta.stage, delta.prim_path)
    else:
        loader, args = view.loadStage, (delta.stage,)

    # Defer GL deletion until the host renderer has finished teardown.
    view._deferRenderCacheClear = True
    try:
        loader(*args)
        model.note_graph_replaced(view, old_state, old_graph)
    finally:
        view._deferRenderCacheClear = False
        # A loader can initialize GL; only FinishReload should resume paint.
        view.initialized = False


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

    Remove / Adopt / Invalidate are records of work reconciliation already
    did, so they carry nothing to apply here.
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
