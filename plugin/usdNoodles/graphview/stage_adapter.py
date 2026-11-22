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

from pxr import Gf, Sdf, Tf, Usd

from UsdNoodles.nodeGraph import NodeGraph, unsubscribe_edit_target_warning

from ..nodeGraphBlueprint import NodeGraphBlueprint
from ..pinUtils import is_hidden_namespace_pin, is_relationship_pin, split_direction_hint
from ..usdNoticeHandler import UsdNoticeHandler
from .legacy import legacy_symbol
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
    StageSync,
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
        state = model.state_for_view(view)
        if state is None:
            return view._addNewNodeFast(prim, stage) or None
        node = model.construct_node_for_insert(view, prim, stage)
        if node is not None:
            state.apply(model.GraphDelta(ops=(model.InsertNode(node.id, node),)))
            return node.id
        return None
    except Exception as error:
        Tf.Warn("Graph validation could not add %s: %s" % (pathStr, error))
        return None


def adoptPrimsNewToScope(view, resyncedPaths):
    """Add nodes for prims that appeared in this graph's scope.

    The other half of a rename: the new path resyncs with a real prim
    behind it and nothing in the graph pointing at it, so without this a
    renamed node disappears from the editor until a manual reload.

    Only paths in this notice are considered, and only direct children of
    the scope this graph loads. A resync of the pseudo-root ("/" — USD's
    spelling for a stage-wide resync) names no child, so it instead
    reconciles membership against the scope's current direct children.
    Note this can re-add a node the user hid with 'D' (which removes from
    the view without touching USD) if that same prim is later edited — the
    view follows the stage, which is the contract being asked for here.

    Returns:
        list[str]: node ids added.
    """
    stage = view.nodeGraph.getStage()
    scope = view._graphScopeParentPath()
    if not stage or scope is None:
        return []

    paths = set(resyncedPaths)
    if "/" in paths:
        paths.discard("/")
        scopePrim = (
            stage.GetPseudoRoot() if scope == "/"
            else stage.GetPrimAtPath(scope)
        )
        if scopePrim and scopePrim.IsValid():
            paths.update(
                str(prim.GetPath()) for prim in scopePrim.GetChildren()
            )

    added = []
    for pathStr in sorted(paths):
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
    if not isinstance(movedPaths, dict):
        destination = pathStr
        for moves in movedPaths:
            destination = _movedDestinationForPath(destination, moves) or destination
        return destination if destination != pathStr else None
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

    state = model.state_for_view(view)
    if state is not None:
        # The model performs the node-object updates, the
        # order-preserving re-key, and the incident-link endpoint
        # re-keying as one delta; the view-owned selection, cursor,
        # scope, and render lines below are unchanged.
        state.apply(
            model.GraphDelta(
                ops=(model.MoveNodes(pairs=tuple(remap.items())),)
            )
        )
    else:
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

    if hasattr(view, '_selection'):
        # Link topology is re-derived from USD on the deferred rebuild.
        view._selection._rewrite(movedPaths)
        view._marqueeBase._rewrite(movedPaths)
    else:
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


def handleUsdChanges(view, resynced, info, movedPaths=None):
    return applyUsdChanges(view, resynced, info, movedPaths=movedPaths)


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
        selectedDestinations = {
            displayMoves[nodeId][0] for nodeId in view._selectedNodes
            if nodeId in displayMoves
        }
        removed = validateNodesAgainstStage(view)
        adopted = adoptMovedDisplayedPrims(view, displayMoves)
        view._selectedNodes.update(selectedDestinations.intersection(adopted))
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


def prepareGraphSwitch(view):
    view._startupSamplePending = False
    view._clearValueInteractionState()
    view._lastPrimTreeSelection = []
    view._lastSelectedNode = None
    if hasattr(view, "_selection"):
        view._selection.update("replace", (), ())
        view._marqueeBase.update("replace", (), ())
    else:
        view._selectedNodes.clear()
        view._selectedLinks.clear()
        view._marqueeBaseSelectedNodes.clear()
        view._marqueeBaseSelectedLinks.clear()
        view._marqueePrevSelectedNodes.clear()
        view._marqueePrevSelectedLinks.clear()
    view.marqueeActive = False
    view.draggingNodes = False
    view.nodeIdUnderCursor = ""
    view._renamingNodeId = ""
    view._renameInput.clear()
    view._linkUnderCursor = -1
    view._lastSyncedLinkHover = -1
    view._hoveredPort = None
    view._tooltipTimer.stop()
    view._tooltipProperty = None
    view._tooltipScreenPos = None
    view._draggingLink = False
    view._dragLinkSourceNode = None
    view._dragLinkSourcePort = None
    view._dragLinkSourceIsOutput = None
    view._dragLinkCursorPos = None
    view._dragLinkValidTarget = False
    view._dragLinkSyntheticTarget = None
    view._dragLinkResolvedTarget = None
    view._clearReconnectState()
    view._lastPinIndex = -1
    view._lastLinkCycleIndex = 0
    view._lastInputLinkIndex = -1
    view._lastOutputLinkIndex = -1
    view._lastDanglingLinkIndex = -1
    view._reconcilePositionGen = 0
    view._reconciledPositionVersions.clear()
    view._spatialIndex.clear()
    view._invalidateBackReferenceLinkDataCache()
    view._linksNeedSync = True
    view._portHighlightsDirty = True


def resetUnboundGraph(view):
    """Drop graph, scope and history without touching USD or GL."""
    view._currentPrimPath = None
    view._undoHistory = None
    view._nodeGraph = NodeGraph()
    view._prepareGraphSwitch()
    view._nextZOrder = 0
    view._renderCacheClearPending = True
    view._invalidateGraphGeometry()
    view.update()


def deferReloadRendering(view, defer):
    view._deferRenderCacheClear = defer
    if not defer:
        # Only FinishReload can resume painting, even if a loader initialized GL.
        view.initialized = False


def _beginReloadOnView(view, frame):
    # Block paint events at the Qt level.  Python-level guards
    # (view.initialized) are insufficient because QGLWidget's internal event
    # handling still touches the GL surface before paintGL() runs, and on
    # NVIDIA Windows drivers this can crash if the stageView's context was
    # just destroyed/recreated on the same thread.
    view.initialized = False
    view._currentTimeCode = frame
    view.setUpdatesEnabled(False)


def _loadOnView(view, delta):
    old_state = model.attached_state(view)
    old_graph = getattr(view, "nodeGraph", None)
    if delta.mode == MODE_CONTAINER:
        view._populateContainer(delta.stage, delta.prim_path)
    elif delta.mode == MODE_BLUEPRINT:
        view._populateBlueprint(delta.stage, delta.prim_path)
    else:
        view._populateStage(delta.stage)
    model.note_graph_replaced(view, old_state, old_graph)


def _finishReloadOnView(view, epoch):
    # Do NOT set view.fontAtlas/view.shaderLibrary to None here.
    # CPython's refcounting would immediately call their C++
    # destructors (which call glDelete*) with no GL context
    # current, corrupting the NVIDIA driver state.  Instead let
    # initializeGL() replace them while the context IS current --
    # the old destructors then run safely.

    if view._renderCacheClearPending:
        context = view.context()
        if context is not None and context.isValid():
            view.makeCurrent()
            try:
                # Leave the clear pending for paintGL if makeCurrent failed.
                view._clearPendingRenderCache()
            except Exception as error:
                Tf.Warn(f"usdNoodles reload could not clear the render cache: {error}")
            finally:
                if view._hasCurrentRenderContext():
                    view.doneCurrent()

    if view._stageSync.isClosed() or view._stageSync.epoch() != epoch:
        return
    view.linksChanged = True
    view.textChanged = True
    view.initialized = True
    view.setUpdatesEnabled(True)
    if not view._stageSync.isClosed() and view._stageSync.epoch() == epoch:
        view.update()


def applyStageDelta(view, delta):
    """Perform one reload delta on *view*, synchronously, owner thread.

    Remove / Adopt / Invalidate are records of work reconciliation
    already did, so they carry nothing to apply here.
    """
    if isinstance(delta, BeginReload):
        _beginReloadOnView(view, delta.frame)
    elif isinstance(delta, Load):
        _loadOnView(view, delta)
    elif isinstance(delta, ClearHistory):
        view._undoHistory = None
    elif isinstance(delta, FinishReload):
        _finishReloadOnView(view, delta.epoch)


def closeStageLifecycle(view):
    """Close an editor's stage lifecycle, exactly once.

    Both Qt close paths call it: the widget's own closeEvent and the dock
    window's, which Qt does not forward to the child. The closed service is
    the guard -- a second call must not move the epoch again or revoke a
    receiver that is already gone, because every queued delta and notice is
    dropped by comparing against that epoch.
    """
    service = view._stageSync
    if service.isClosed():
        return
    try:
        view._clearValueInteractionState()
    finally:
        service.close()
        view._undoHistory = None
        unsubscribe_edit_target_warning(view._showWarningPopup)
    dispose = getattr(view, "_disposeView", None)
    if dispose is not None:
        dispose()


def _onStageReplaced(view):
    sync = getattr(view, "_stageSync", None)
    if not isinstance(sync, StageSync):
        # Test doubles never ran __init__: same policy, transient service.
        sync = _makeStageSync(view)
        view._stageSync = sync
    if sync.isClosed():
        # usdview keeps this signal connected and a closed widget is not
        # a destroyed one: decide before the api or graph is read.
        return
    frame = _frameOf(
        getattr(view._usdviewApi, "frame", Usd.TimeCode.Default())
        if view._usdviewApi
        else Usd.TimeCode.Default()
    )
    stage = view._usdviewApi.stage if view._usdviewApi else None
    sync.replace(
        stage, stageLoadMode(view), view._currentPrimPath, frame
    )


def loadBlueprint(view, stage, primPath, useFieldnames=True):
    view._stageSync.loadGraph(
        stage,
        (lambda: view._populateBlueprint(stage, primPath, useFieldnames))
        if stage is not None
        else view._resetUnboundGraph,
        view._resetUnboundGraph,
    )


def populateBlueprint(view, stage, primPath, useFieldnames=True):
    populateScopedGraph(view, NodeGraphBlueprint, stage, primPath, useFieldnames)


def loadContainer(view, stage, primPath, useFieldnames=True):
    view._stageSync.loadGraph(
        stage,
        (lambda: view._populateContainer(stage, primPath, useFieldnames))
        if stage is not None
        else view._resetUnboundGraph,
        view._resetUnboundGraph,
    )


def populateContainer(view, stage, primPath, useFieldnames=True):
    from ..nodeGraphBpContainer import NodeGraphBpContainer

    populateScopedGraph(view, NodeGraphBpContainer, stage, primPath, useFieldnames)


def populateScopedGraph(view, graphType, stage, primPath, useFieldnames):
    # The model loaders own the scope check and raise on a missing scope;
    # the switch must fail rather than bind a graph scoped to nothing.
    epoch = view._stageSync.epoch()
    graph = graphType()
    graph.load(
        stage,
        primPath,
        view.textRenderer.calculateTextWidth,
        view.fontAtlas,
        useFieldnames,
        nodeFactory=view._nodeFactory,
    )
    if view._stageSync.isClosed() or view._stageSync.epoch() != epoch:
        return
    view._currentPrimPath = primPath
    view._nodeGraph = graph
    view._initUndoTracking(stage)
    view._seedNodeZOrders()
    view._gridPlaceNodes(list(view.nodes.values()))
    view._resetRenderCachesAfterBulkMove()
    if view.nodes:
        view.frameAll()


def loadStage(view, stage):
    """Open an EMPTY graph on *stage*: no prims graphed at all.

    The canvas is the user's, not the loader's: prims arrive via 'A'
    (``addNodesFromPrimTreeSelection``), node creation, and the
    notice-driven adoption of newly appearing scope children -- never
    from an opening scan. ``NodeGraphStage.load`` keeps its
    root-children scan as the explicit loader primitive; the view
    simply does not invoke it here.
    """
    view._stageSync.loadGraph(
        stage,
        (lambda: view._populateStage(stage))
        if stage is not None
        else view._resetUnboundGraph,
        view._resetUnboundGraph,
    )


def populateStage(view, stage):
    from ..nodeGraphStage import NodeGraphStage

    epoch = view._stageSync.epoch()
    graph = NodeGraphStage()
    graph.setStage(stage)
    graph.syncSelectionToPrimTree = True
    graph.linksChanged = True
    if view._stageSync.isClosed() or view._stageSync.epoch() != epoch:
        return
    view._currentPrimPath = None
    view._nodeGraph = graph
    view._initUndoTracking(stage)
    view._seedNodeZOrders()
    view._resetRenderCachesAfterBulkMove()


def loadSampleGraph(view, jsonPath):
    view._stageSync.loadGraph(
        None, lambda: view._populateSampleGraph(jsonPath), view._resetUnboundGraph
    )


def populateSampleGraph(view, jsonPath):
    from ..nodeGraphJson import NodeGraphJson

    epoch = view._stageSync.epoch()
    graph = NodeGraphJson()
    graph.load(jsonPath, view.textRenderer.calculateTextWidth, view.fontAtlas)
    stage = graph.getStage()
    if stage is None:
        raise RuntimeError(f"Could not load sample graph: {jsonPath}")
    if view._stageSync.isClosed() or view._stageSync.epoch() != epoch:
        return
    # The layer editor reads the converted stage from nodeGraphReplaced.
    view._currentPrimPath = None
    view._nodeGraph = graph
    view._initUndoTracking(stage)
    view._resetRenderCachesAfterBulkMove()
    if view.nodes:
        view.frameScene()


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
    hide_ui = bool(legacy_symbol("NoodlesConfig").get("hideUiNamespacePins", True))
    return not is_hidden_namespace_pin(propertyName, hide_ui=hide_ui)


def _reloadLinksFromUSD(view):
    """Rebuild links from USD connections without reloading the graph."""
    view._rebuildLinks()
    view.update()


def _reloadCurrentGraph(view):
    """Rebuild after undo/redo, preserving curated membership and placement."""
    from ..nodeGraphBpContainer import NodeGraphBpContainer
    from ..nodeGraphStage import NodeGraphStage

    stage = view.nodeGraph.getStage()
    if not stage:
        view.update()
        return

    # Every node object is about to be rebuilt, so a value edit or drag
    # holding one is already talking about the previous graph.
    view._clearValueInteractionState()

    savedZOrders = {nid: node.zOrder for nid, node in view.nodes.items()}
    savedNodeIds = list(view.nodes.keys())
    savedDisplayPositions = {
        nid: Gf.Vec2d(node.position)
        for nid, node in view.nodes.items()
        if not view._nodeHasAuthoredPosition(node)
    }

    # Disable notice handler during reload to avoid recursive processing
    with view._stageSync.suppressed():
        epoch = view._stageSync.epoch()
        graph = type(view.nodeGraph)() if isinstance(
            view.nodeGraph, (NodeGraphStage, NodeGraphBlueprint, NodeGraphBpContainer)
        ) else NodeGraphBlueprint()
        if isinstance(graph, NodeGraphStage):
            graph.setStage(stage)
            graph.syncSelectionToPrimTree = True
        else:
            graph.load(
                stage,
                view._currentPrimPath,
                view.textRenderer.calculateTextWidth,
                view.fontAtlas,
                nodeFactory=view._nodeFactory,
            )
        if view._stageSync.isClosed() or view._stageSync.epoch() != epoch:
            return
        view._nodeGraph = graph

        for nodeId in savedNodeIds:
            if nodeId in view.nodes or "#" in nodeId:
                continue
            prim = stage.GetPrimAtPath(nodeId)
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            try:
                view._addPrimAsNode(prim, stage, [])
            except Exception as error:
                Tf.Warn("Graph reload could not re-add %s: %s" % (nodeId, error))

        for nodeId, position in savedDisplayPositions.items():
            node = view.nodes.get(nodeId)
            if node is not None and not view._nodeHasAuthoredPosition(node):
                node.setDisplayPosition(position)

        maxSaved = max(savedZOrders.values()) if savedZOrders else 0
        view._nextZOrder = maxSaved
        for nid, node in view.nodes.items():
            if nid in savedZOrders:
                node.zOrder = savedZOrders[nid]
            else:
                view._nextZOrder += 1
                node.zOrder = view._nextZOrder

        view.textChanged = True
        view.linksChanged = True
        view._clearRenderCache()
        view._nodeRenderManager.invalidateAll()
        view.textRenderer.resetPositionCaches()
        view._cppIconRenderer.resetPositionCaches()
        view._nodeRenderManager.resetNodeQuadCaches()
        view._nodeTransformFrame.reset()
        view.update()

    if view._stageSync.epoch() == epoch:
        view.nodeGraphReplaced.emit()


def _isStageViewMidTeardown(view):
    if not view._usdviewApi:
        return False
    try:
        controller = view._usdviewApi._UsdviewApi__appController
        sv = getattr(controller, "_stageView", None)
        return sv is not None and not sv.updatesEnabled()
    except AttributeError:
        return False


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
    return StageSync(
        read=lambda: readStageContext(view),
        notice=lambda: readNoticeContext(view),
        apply=lambda delta: applyStageDelta(view, delta),
        post=lambda callback: legacy_symbol("QtCore").QTimer.singleShot(0, callback),
        on_owner=getattr(type(view), "_isOwnerThread", None) and view._isOwnerThread,
        post_notice=getattr(type(view), "_postOwnerCallback", None) and view._postOwnerCallback,
        listen=lambda receiver: UsdNoticeHandler(receiver),
        warn=lambda message: Tf.Warn(message),
        prepare=lambda: view._prepareGraphSwitch(),
        reset=lambda: view._resetUnboundGraph(),
        defer=lambda value: deferReloadRendering(view, value),
        reconcile=lambda resynced, info, trace, moved: applyUsdChanges(
            view, resynced, info, trace=trace, movedPaths=moved
        ),
        graphReplaced=getattr(getattr(view, "nodeGraphReplaced", None), "emit", None),
    )
