#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Stage reconciliation: the graph follows the stage, never the reverse.

The stage_sync layer of the noodle editor: USD notice classification, scope
validation, namespace-move re-keying, and the invalidation decisions they
drive. Every function takes the view as a duck-typed context, reading only
its state attributes and calling only the listed view hooks, so the layer
runs headless and against stub views while the widget side effects (repaint,
render caches) stay behind those hooks. Reconciliation is read-only: it
never authors a USD opinion.
"""

from pxr import Sdf, Tf


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


def applyUsdChanges(view, resyncedPaths, infoChangedPaths, movedPaths=None):
    """Handle USD changes detected via the Notice system.

    This is called by UsdNoticeHandler (through the view's delegator) when
    USD prims/properties change. It invalidates caches and triggers redraws
    as needed.

    Args:
        resyncedPaths: Set of path strings that had structural changes
                      (prims added/removed, connections changed)
        infoChangedPaths: Set of path strings that had property value changes
                         (positions, metadata)
        movedPaths: Optional ``{oldPath: newPath}`` for prims USD reports as
                    renamed or reparented rather than deleted and recreated.
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
            needsLinkRebuild = True
            needsRepaint = True
            needsTextUpdate = True

    # Apply the updates
    if needsLinkRebuild:
        view.linksChanged = True
        view._clearRenderCache()

    if needsTextUpdate:
        view.textChanged = True

    if needsRepaint:
        view.update()
