#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Canonical graph selection and the usdview prim-tree bridge."""

from dataclasses import dataclass, replace

from pxr import Gf, Tf
from UsdNoodles.core import LinkSelectionMode

from ..pinUtils import direction_hint_candidate_names, split_direction_hint
from ..touchPose import collect_graph_prims, is_touch_group
from .legacy import legacy_symbol


@dataclass(frozen=True)
class LinkKey:
    kind: str
    owner_property: str
    source_id: str
    source_pin: str
    target_id: str
    target_pin: str
    dangling_id: str

    @classmethod
    def from_link(cls, link):
        source = str(getattr(link, 'data_sourceNodeId', '') or link.sourceNodeId)
        target = str(getattr(link, 'data_targetNodeId', '') or link.targetNodeId)
        is_input = getattr(link, 'is_input_link', False)
        relationship = getattr(link, 'is_relationship_link', False)
        owner = str(getattr(link, 'propertyOwnerNodeId', '') or
                    (target if is_input else source))
        name = str(getattr(link, 'propertyName', ''))
        if not name:
            endpoint = 'target' if is_input else 'source'
            name = str(getattr(link, endpoint + 'PropertyName', ''))
        if not name:
            port = str(link.targetPort if is_input else link.sourcePort)
            name = direction_hint_candidate_names(port, is_input=is_input)[0] if port else ''
        dangling = ''
        if getattr(link, 'isDangling', False):
            missing_source = getattr(link, 'danglingDirection', '') == 'input'
            dangling = (source if missing_source else target) or (
                'source' if missing_source else 'target'
            )
        return cls('relationship' if relationship else 'attribute',
                   f'{owner}.{name}' if owner and name else '',
                   source, str(getattr(link, 'sourcePropertyName', '') or link.sourcePort),
                   target, str(getattr(link, 'targetPropertyName', '') or link.targetPort),
                   dangling)


def _path_rewriter(path_map):
    prefixes = sorted(path_map, key=len, reverse=True)

    def rewrite(path):
        for old in prefixes:
            if path == old or (path.startswith(old) and path[len(old)] in '/.#'):
                return path_map[old] + path[len(old):]
        return path

    return rewrite


class Selection:
    def __init__(self, node_ids=(), link_keys=()):
        self.node_ids = set(node_ids)
        self.link_keys = set(link_keys)

    def copy(self):
        return Selection(self.node_ids, self.link_keys)

    def update(self, mode, node_ids, link_keys):
        nodes, links = set(node_ids), set(link_keys)
        if mode == 'replace':
            self.node_ids, self.link_keys = nodes, links
        elif mode in ('union', 'add'):
            self.node_ids |= nodes
            self.link_keys |= links
        elif mode in ('toggle', 'symmetric_difference'):
            self.node_ids ^= nodes
            self.link_keys ^= links
        else:
            raise ValueError(f'Unknown selection mode: {mode}')

    def marquee(self, base, mode, node_ids, link_keys):
        result = base.copy()
        result.update(mode, node_ids, link_keys)
        self.update('replace', result.node_ids, result.link_keys)

    def _rewrite(self, path_map):
        rewrite = _path_rewriter(path_map)
        self.node_ids = {rewrite(path) for path in self.node_ids}
        self.link_keys = {
            replace(key, owner_property=rewrite(key.owner_property),
                    source_id=rewrite(key.source_id), target_id=rewrite(key.target_id),
                    dangling_id=rewrite(key.dangling_id))
            for key in self.link_keys
        }

    def remap(self, path_map, model):
        self._rewrite(path_map)
        model = getattr(model, 'graph', model)
        self.node_ids.intersection_update(model.nodes)
        if self.link_keys:
            self.link_keys.intersection_update(
                link if isinstance(link, LinkKey) else LinkKey.from_link(link)
                for link in model.links
            )


class PrimTreeAdapter:
    """Synchronize path sets without accepting synchronous echo signals."""

    def __init__(self, read_paths, write_paths, selection, *, enabled=lambda: True):
        self.read_paths = read_paths
        self.write_paths = write_paths
        self.selection = selection
        self.enabled = enabled
        self._depth = 0

    @property
    def syncing(self):
        return self._depth > 0

    def from_tree(self, *, read=None, current=None, apply=None):
        if self.syncing or not self.enabled():
            return False
        self._depth += 1
        try:
            paths = set((read or self.read_paths)())
            if paths == (current() if current else self.selection.node_ids):
                return False
            if apply:
                apply(paths)
            else:
                self.selection.update('replace', paths, self.selection.link_keys)
            return True
        finally:
            self._depth -= 1

    def to_tree(self, *, read=None, paths=None, write=None):
        if self.syncing or not self.enabled():
            return False
        self._depth += 1
        try:
            desired = set(self.selection.node_ids if paths is None else paths())
            if desired == set((read or self.read_paths)()):
                return False
            (write or self.write_paths)(desired)
            return True
        finally:
            self._depth -= 1


def selected_node_ids(view):
    return view._selection.node_ids


def syncing_selection(view):
    return view._primTreeAdapter.syncing


def selected_link_indices(view):
    keys = view._selection.link_keys
    return {i for i, link in enumerate(view.links) if LinkKey.from_link(link) in keys} if keys else set()


def project_nodes(view):
    for node_id, node in view.nodes.items():
        node.selected = node_id in view._selectedNodes


def project_links(view):
    indices = view._selectedLinks
    for i, link in enumerate(view.links):
        link.selected = i in indices
    view._linksNeedSync = True


def select_node(view, node_id, *, toggle=False):
    selected = view._selectedNodes  # Also supports legacy duck-typed views.
    if toggle and node_id in selected:
        selected.discard(node_id)
    else:
        selected.add(node_id)
    view.nodes[node_id].selected = node_id in selected


def reconcile(view):
    if hasattr(view, '_selection'):
        view._selection.remap({}, view)
        view._marqueeBase.remap({}, view)
        project_nodes(view)
        project_links(view)
        view._updateLinkSelectionFromNodes()


def _tree_selection(view):
    api = view._usdviewApi
    return api.dataModel.selection if api and api.dataModel else None


def _prim_paths(view):
    return {str(prim.GetPath()) for prim in _tree_selection(view).getPrims()} - {'/'}


def _prop_paths(view):
    return {str(prop.GetPath()) for prop in _tree_selection(view).getProps()}


def _write_tree_paths(view, paths, *, properties=False):
    stage = view._usdviewApi.stage
    tree = _tree_selection(view)
    prim_paths = {path for path in paths if '.' not in path}
    prop_paths = paths - prim_paths
    if prim_paths != _prim_paths(view):
        prims = [stage.GetPrimAtPath(path) for path in sorted(prim_paths)]
        with tree.batchPrimChanges:
            tree.clearPrims()
            for prim in prims:
                if prim:
                    tree.addPrim(prim)
        if prims and prims[0] and view._frameInPrimTreeOnSelect:
            view._frameInPrimTree(prims[0])
    if properties and prop_paths != _prop_paths(view):
        with tree.batchPropChanges:
            tree.clearProps()
            for path in sorted(prop_paths):
                prop = stage.GetPropertyAtPath(path)
                if prop:
                    tree.addProp(prop)


def make_prim_tree_adapter(view, selection=None):
    return PrimTreeAdapter(
        lambda: _prim_paths(view), lambda paths: _write_tree_paths(view, paths),
        view._selection if selection is None else selection,
        enabled=lambda: bool(view.nodeGraph and view.nodeGraph.syncSelectionToPrimTree
                             and _tree_selection(view) and view._usdviewApi.stage),
    )


def _link_tree_paths(view):
    stage = view._usdviewApi.stage
    paths = set()
    for i in view._selectedLinks:
        link = view.links[i]
        key = LinkKey.from_link(link)
        owner, _, name = key.owner_property.partition('.')
        owner = owner or str(getattr(link, 'propertyOwnerNodeId', '') or
                             (key.target_id if getattr(link, 'is_input_link', False) else key.source_id))
        if not owner or '#' in owner:
            continue
        prim = stage.GetPrimAtPath(owner)
        if not prim:
            continue
        paths.add(owner)
        if name and prim.GetProperty(name):
            paths.add(key.owner_property)
        elif key.kind != 'relationship':
            is_input = getattr(link, 'is_input_link', False)
            pin = link.targetPort if is_input else link.sourcePort
            for candidate in direction_hint_candidate_names(pin, is_input=is_input):
                attr = prim.GetAttribute(candidate)
                if attr:
                    paths.add(str(attr.GetPath()))
                    break
    return paths


def _change_link_selection(view, mode, index):
    if hasattr(view, '_selection'):
        keys = {LinkKey.from_link(view.links[index])}
        nodes = view._selection.node_ids if mode == 'replace' else ()
        view._selection.update(mode, nodes, keys)
    elif mode == 'replace':
        view._selectedLinks.clear()
        view._selectedLinks.add(index)
    elif mode == 'toggle':
        view._selectedLinks.symmetric_difference_update({index})
    else:
        view._selectedLinks.add(index)
    project_links(view)
    view._syncLinkSelectionToPrimtree()
    view.update()


def get_link_selection_mode(view):
    return view._linkSelectionMode


def set_link_selection_mode(view, mode):
    view._linkSelectionMode = mode
    view._updateLinkSelectionFromNodes()
    view.update()


def _updateLinkSelectionFromNodes(view):
    with legacy_symbol("_ProfileSection")(view._profiler, "Update Link Selection"):
        for link in view.links:
            link.highlighted = False
        # Highlight flags changed on the link snapshot; re-mirror to C++.
        view._linksNeedSync = True

        if view._linkSelectionMode == LinkSelectionMode.NODES_ONLY:
            return

        selectedNodeIds = {
            nodeId for nodeId, node in view.nodes.items() if node.selected
        }

        if not selectedNodeIds:
            return

        for _i, link in enumerate(view.links):
            dataSource = getattr(link, "data_sourceNodeId", None)
            dataTarget = getattr(link, "data_targetNodeId", None)

            sourceSelected = dataSource in selectedNodeIds if dataSource else False
            targetSelected = dataTarget in selectedNodeIds if dataTarget else False

            shouldHighlight = False
            if view._linkSelectionMode == LinkSelectionMode.WITH_ALL_LINKS:
                shouldHighlight = sourceSelected or targetSelected
            elif view._linkSelectionMode == LinkSelectionMode.WITH_INPUTS:
                shouldHighlight = targetSelected
            elif view._linkSelectionMode == LinkSelectionMode.WITH_OUTPUTS:
                shouldHighlight = sourceSelected

            if shouldHighlight:
                link.highlighted = True


def _selectLink(view, linkIndex, addToSelection=False):
    if 0 <= linkIndex < len(view.links):
        _change_link_selection(view, 'union' if addToSelection else 'replace', linkIndex)


def _toggleLinkSelection(view, linkIndex):
    if 0 <= linkIndex < len(view.links):
        _change_link_selection(view, 'toggle', linkIndex)


def _syncLinkSelectionToPrimtree(view):
    if view._selectedLinks:
        view._primTreeAdapter.to_tree(
            read=lambda: _prim_paths(view) | _prop_paths(view),
            paths=lambda: _link_tree_paths(view) or (_prim_paths(view) | _prop_paths(view)),
            write=lambda paths: _write_tree_paths(view, paths, properties=True),
        )


def clearLinkSelection(view):
    if hasattr(view, '_selection'):
        view._selection.link_keys.clear()
    else:
        view._selectedLinks.clear()
    project_links(view)


def _getFirstSelectedLink(view):
    if not view._selectedLinks:
        return None

    # Get the first selected link index
    firstIndex = min(view._selectedLinks)
    if firstIndex < len(view.links):
        return view.links[firstIndex]
    return None


def _selectFirstLinkInDirection(view, node, direction):
    """Select the first link in the given direction from a node.

    Args:
        node: The NodeModel to find links for.
        direction: "input" or "output"

    If we previously arrived at this node via a link in the requested direction,
    we'll return to that same link instead of selecting the first one.

    If no links exist in the requested direction, swaps to the opposite
    direction and selects from there instead.
    """
    # Check if we have a history link to return to
    historyLinkIndex = -1
    if direction == "input" and view._lastInputLinkIndex >= 0:
        # Check if this link still connects to the current node
        if view._lastInputLinkIndex < len(view.links):
            historyLink = view.links[view._lastInputLinkIndex]
            historyTarget = getattr(historyLink, "data_targetNodeId", None)
            if historyTarget == node.id:
                historyLinkIndex = view._lastInputLinkIndex
    elif direction == "output" and view._lastOutputLinkIndex >= 0:
        # Check if this link still connects to the current node
        if view._lastOutputLinkIndex < len(view.links):
            historyLink = view.links[view._lastOutputLinkIndex]
            historySource = getattr(historyLink, "data_sourceNodeId", None)
            if historySource == node.id:
                historyLinkIndex = view._lastOutputLinkIndex

    # If we have a valid history link, use it
    if historyLinkIndex >= 0:
        # Show popup with direction indicator
        popupKey = "[" if direction == "input" else "]"
        popupDirection = "left" if direction == "input" else "right"
        view._showHotkeyPopup(popupKey, popupDirection)

        # Update tracking
        view._lastJumpDirection = "source" if direction == "input" else "target"
        view._lastSelectedNode = node

        # Clear the history since we're using it now
        if direction == "input":
            view._lastInputLinkIndex = -1
        else:
            view._lastOutputLinkIndex = -1

        # Clear node selection and select the link
        view.clearSelection()
        view.clearLinkSelection()
        view._selectLink(historyLinkIndex, addToSelection=False)

        view.update()
        return

    # No history link - try the requested direction first
    links = view._getLinksForNodeFlat(node, direction)

    # If no links in requested direction, swap to opposite
    if not links:
        oppositeDirection = "output" if direction == "input" else "input"
        links = view._getLinksForNodeFlat(node, oppositeDirection)
        if links:
            direction = oppositeDirection  # Update for popup display

    if not links:
        return  # No links at all

    # Select the first link
    linkIndex, _link = links[0]

    # Show popup with direction indicator
    popupKey = "[" if direction == "input" else "]"
    popupDirection = "left" if direction == "input" else "right"
    view._showHotkeyPopup(popupKey, popupDirection)

    # Update tracking for 1,2,3 key navigation
    view._lastJumpDirection = "source" if direction == "input" else "target"
    view._lastSelectedNode = node
    view._lastPinIndex = 0
    view._lastLinkCycleIndex = 0

    # Clear node selection and select the link
    view.clearSelection()
    view.clearLinkSelection()
    view._selectLink(linkIndex, addToSelection=False)

    view.update()


def _getFirstSelectedNode(view):
    for node in view.nodes.values():
        if node.selected:
            return node
    return None


def selectLinkByPinIndex(view, pinIndex, reverseDirection=False):
    """Select a link connected to the nth input or output of a node.

    Enhanced navigation features:
    - Uses _lastSelectedNode if no node is currently selected (allows 1,2,3 switching)
    - Supports reverseDirection (Shift+number) to go opposite direction
    - Falls back to opposite direction if current direction has no links
    - Cycles through multiple links on the same pin when pressed repeatedly

    Args:
        pinIndex: 0-based index of the pin (0 = first, 1 = second, etc.)
        reverseDirection: If True, use opposite direction from _lastJumpDirection
    """
    # Get the node to work with - either currently selected or last selected
    node = view._getFirstSelectedNode()

    if node is None and view._lastSelectedNode is not None:
        # No node selected, but we have a last selected node
        # This allows pressing 1,2,3 to switch links from the same node
        node = view._lastSelectedNode
    elif node is not None:
        # A node is selected - remember it for future use
        view._lastSelectedNode = node

    if not node:
        return

    # Determine primary direction based on last jump and reverse flag
    if view._lastJumpDirection == "target":
        primaryDirection = "output"
        fallbackDirection = "input"
    else:
        primaryDirection = "input"
        fallbackDirection = "output"

    # Reverse if Shift is held
    if reverseDirection:
        primaryDirection, fallbackDirection = fallbackDirection, primaryDirection

    # Get links for primary direction
    pinsToLinks = view._getLinksForNodeDirection(node, primaryDirection)

    # If no links in primary direction, try fallback
    if not pinsToLinks:
        pinsToLinks = view._getLinksForNodeDirection(node, fallbackDirection)

    if not pinsToLinks:
        return  # No links at all

    # Get sorted list of pin indices
    sortedPins = sorted(pinsToLinks.keys())

    if pinIndex >= len(sortedPins):
        return  # No pin at this index

    # Get the actual pin index from our sorted list
    actualPinIdx = sortedPins[pinIndex]
    linksForPin = pinsToLinks[actualPinIdx]

    if not linksForPin:
        return

    # Determine which link to select (cycling through multiple links on same pin)
    if pinIndex == view._lastPinIndex and len(linksForPin) > 1:
        # Same pin pressed again - cycle to next link
        view._lastLinkCycleIndex = (view._lastLinkCycleIndex + 1) % len(linksForPin)
    else:
        # Different pin - reset cycle
        view._lastPinIndex = pinIndex
        view._lastLinkCycleIndex = 0

    linkIndex, _link = linksForPin[view._lastLinkCycleIndex]

    # Determine popup direction and key text
    # If primaryDirection is "output", we're selecting outputs (arrow right)
    # If primaryDirection is "input", we're selecting inputs (arrow left)
    popupDirection = "right" if primaryDirection == "output" else "left"
    keyText = str(pinIndex + 1)  # Just show the number, arrow indicates direction
    view._showHotkeyPopup(keyText, popupDirection)

    # Clear node selection and select the link
    view.clearSelection()
    view.clearLinkSelection()
    view._selectLink(linkIndex, addToSelection=False)

    view.update()


def _onPropSelectionChanged(view):
    def read_matches():
        paths = _prop_paths(view)
        matches = set()
        for link in view.links:
            key = LinkKey.from_link(link)
            if key.owner_property in paths:
                matches.add(key)
                continue
            for path in paths:
                owner, _, name = path.partition('.')
                side, pin = split_direction_hint(name)
                source_match = (owner, pin) == (key.source_id, link.sourcePort)
                target_match = (owner, pin) == (key.target_id, link.targetPort)
                if side is None and (source_match or target_match):
                    matches.add(key)
                    break
                if source_match and getattr(link, 'is_input_link', True) == (side == 'input'):
                    matches.add(key)
                    break
        # Unrelated properties do not replace the graph's link selection.
        return matches or view._selection.link_keys

    def apply(keys):
        view._selection.update('replace', view._selection.node_ids, keys)
        project_links(view)
        view.update()

    view._primTreeAdapter.from_tree(read=read_matches,
                                   current=lambda: view._selection.link_keys, apply=apply)


def _onPrimSelectionChanged(view):
    def read_nodes():
        prims = [p for p in _tree_selection(view).getPrims() if str(p.GetPath()) != '/']
        if prims:
            view._lastPrimTreeSelection = prims
        return {str(p.GetPath()) for p in prims}.intersection(view.nodes)

    def apply(nodes):
        view._selection.update('replace', nodes, view._selection.link_keys)
        project_nodes(view)
        view._updateLinkSelectionFromNodes()
        view.update()

    view._primTreeAdapter.from_tree(read=read_nodes, apply=apply)


def _syncSelectionToPrimtree(view):
    with legacy_symbol("_ProfileSection")(view._profiler, "Sync Selection to Primtree"):
        def paths():
            stage = view._usdviewApi.stage
            valid = {path for path in view._selectedNodes
                     if '#' not in path and stage.GetPrimAtPath(path)}
            # A selection consisting only of synthetic pins has no tree equivalent.
            return valid if valid or not view._selectedNodes else _prim_paths(view)

        if hasattr(view, '_primTreeAdapter'):
            view._primTreeAdapter.to_tree(paths=paths)
        elif not view._syncingSelection:
            make_prim_tree_adapter(view, Selection()).to_tree(paths=paths)


def _frameInPrimTree(view, prim):
    if not view._usdviewApi or not prim:
        return

    try:
        appController = getattr(
            view._usdviewApi, "_UsdviewApi__appController", None
        )
        if not appController:
            return

        primPath = prim.GetPath()
        item = appController._getItemAtPath(primPath, ensureExpanded=True)

        if (
            item
            and hasattr(appController, "_ui")
            and hasattr(appController._ui, "primView")
        ):
            primView = appController._ui.primView
            # Scroll to make the item visible, centered if possible
            primView.scrollToItem(item)

    except Exception:
        pass


def _setLinkSelectionMode(view, mode):
    view.linkSelectionMode = mode


def _toggleFrameInPrimTree(view, checked):
    view._frameInPrimTreeOnSelect = checked


def _clearSelectionAndSync(view):
    view.clearSelection()
    view._syncSelectionToPrimtree()


def _canOpenSelectedContainer(view):
    """Check if exactly one node is selected and it's a Container type."""
    if len(view._selectedNodes) != 1:
        return False

    selectedNodeId = next(iter(view._selectedNodes))
    if selectedNodeId not in view.nodes:
        return False

    node = view.nodes[selectedNodeId]

    # Check if the node's type is "Container"
    # Containers can be:
    # 1. A node with type "Container" (directly set from the prim)
    # 2. A node whose ID is a valid USD prim path that has type "Container"
    if node.type == "Container":
        return True

    # Also check the actual prim type in case the node type wasn't set correctly
    if view._usdviewApi:
        stage = view._usdviewApi.stage
        if stage and "#" not in node.id:  # Skip virtual nodes
            prim = stage.GetPrimAtPath(node.id)
            if prim and prim.IsValid() and prim.GetTypeName() == "Container":
                return True

    return False


def _selectNodesInMarquee(view, nodeIds, marqueeBounds):
    newlySelected = set()
    for nodeId in nodeIds:
        if nodeId in view.nodes:
            node = view.nodes[nodeId]
            nodeBounds = Gf.Range2d(node.position, node.position + node.size)
            if not Gf.Range2d.GetIntersection(marqueeBounds, nodeBounds).IsEmpty():
                newlySelected.add(node.id)
    return newlySelected


def _selectLinksInMarquee(view, linkIndices, marqueeBounds):
    candidateLinks = []
    candidateIndices = []
    for linkIndex in linkIndices:
        if linkIndex < len(view.links):
            candidateLinks.append(view.links[linkIndex])
            candidateIndices.append(linkIndex)

    newlySelected = set()
    if candidateLinks:
        hitPositions = view.linkRenderer.findLinksInBounds(
            candidateLinks, marqueeBounds
        )
        for hitPos in hitPositions:
            linkIndex = candidateIndices[hitPos]
            newlySelected.add(linkIndex)
    return newlySelected


def _updateMarqueeSelection(view):
    """Project hits against the immutable gesture-start snapshot."""
    if not view.marqueeActive:
        return

    dx = abs(view.marqueeCurrent.x() - view.marqueeStart.x())
    dy = abs(view.marqueeCurrent.y() - view.marqueeStart.y())
    nodes, indices = set(), set()
    if max(dx, dy) >= 2.0 / view.zoom:
        bounds = Gf.Range2d(
            Gf.Vec2d(min(view.marqueeStart.x(), view.marqueeCurrent.x()),
                     min(view.marqueeStart.y(), view.marqueeCurrent.y())),
            Gf.Vec2d(max(view.marqueeStart.x(), view.marqueeCurrent.x()),
                     max(view.marqueeStart.y(), view.marqueeCurrent.y())),
        )
        node_ids, link_indices = view._spatialIndex.queryRegion(bounds)
        nodes = view._selectNodesInMarquee(node_ids, bounds)
        indices = view._selectLinksInMarquee(link_indices, bounds)

    if hasattr(view, '_selection'):
        view._selection.marquee(view._marqueeBase, view._marqueeMode, nodes,
                                {LinkKey.from_link(view.links[i]) for i in indices})
    else:
        # Legacy unbound callers supply a gesture snapshot in row coordinates.
        view._selectedNodes = view._marqueeBaseSelectedNodes | nodes
        view._selectedLinks = view._marqueeBaseSelectedLinks | indices
        view._marqueePrevSelectedNodes = set(view._selectedNodes)
        view._marqueePrevSelectedLinks = set(view._selectedLinks)
    project_nodes(view)
    project_links(view)


def selectAll(view):
    view._selectedNodes.update(view.nodes)
    project_nodes(view)
    view.update()
    view._syncSelectionToPrimtree()


def clearSelection(view):
    view._selectedNodes.clear()
    project_nodes(view)
    view.update()


def _getContainerChildNodes(view, containerPrim):
    """Get all child prims of a container/blueprint.

    Args:
        containerPrim: A Container or Blueprint prim

    Returns:
        List of child prims
    """
    return list(containerPrim.GetChildren())


def addNodesFromPrimTreeSelection(view):
    """Add nodes from the USD prim tree selection to the editor.

    When the user presses 'A', this method:
    1. Gets the currently selected prims from the prim tree
    2. If a Container or Blueprint is selected, adds all its child nodes
    3. Otherwise creates NodeModel instances for valid node prims
    4. Adds them to the graph with their incoming/outgoing links
    5. Links to nodes not in the graph become dangling links
    6. If the graph was empty, frames the view to show the added nodes
    """
    if not view._usdviewApi:
        Tf.Warn("Cannot add nodes: usdviewApi not available")
        return

    try:
        dataModel = view._usdviewApi.dataModel
        if not dataModel or not dataModel.selection:
            return

        stage = view._usdviewApi.stage
        if not stage:
            return

        livePrims = list(dataModel.selection.getPrims())
        # If only the root prim is selected (happens when canvas click resets selection),
        # fall back to the last meaningful prim tree selection. Also fall back when
        # the cached selection contains MORE non-root prims than the live API — this
        # guards against cases where the live selection has been narrowed (e.g. by
        # focus changes) but the user's intent was to add the full multi-selection.
        liveNonRoot = [
            p for p in livePrims if p.IsValid() and p.GetPath().pathString != "/"
        ]
        cachedNonRoot = [
            p
            for p in view._lastPrimTreeSelection
            if p.IsValid() and p.GetPath().pathString != "/"
        ]
        if len(cachedNonRoot) > len(liveNonRoot):
            selectedPrims = cachedNonRoot
        elif liveNonRoot:
            selectedPrims = liveNonRoot
        elif cachedNonRoot:
            selectedPrims = cachedNonRoot
        else:
            view._showPopupMessage("No prims selected")
            return

        # Deduplicate by path — getPrims() can list a prim multiple times
        # (e.g. via instance proxies) and we should only add each once.
        seenPaths = set()
        uniquePrims = []
        for p in selectedPrims:
            pathStr = str(p.GetPath())
            if pathStr == "/" or pathStr in seenPaths:
                continue
            seenPaths.add(pathStr)
            uniquePrims.append(p)
        selectedPrims = uniquePrims
        if not selectedPrims:
            view._showPopupMessage("No valid node prims selected")
            return

        # Whether this add is populating an empty canvas or landing in a
        # graph the user has already arranged decides both where the batch
        # goes and whether the camera is allowed to move (see below).
        graphWasEmpty = not view.nodes

        addedCount = 0
        selectedExistingCount = 0
        addedNodeIds = []

        for prim in selectedPrims:
            primType = prim.GetTypeName()

            # Check if this is a container-type prim (Container or Blueprint)
            # If so, add all its child nodes instead of the container itself
            # A TouchPose scope is the third kind of group: selecting it
            # means "show me the regions", exactly as selecting a Container
            # means "show me its nodes" (see touchPose.collect_graph_prims
            # for why the controls come too).
            isTouchGroup = is_touch_group(prim)
            if primType in ["Container", "Blueprint"] or isTouchGroup:
                if isTouchGroup:
                    childNodes = collect_graph_prims(prim, stage)
                else:
                    childNodes = view._getContainerChildNodes(prim)
                if childNodes:
                    for childPrim in childNodes:
                        childPathStr = str(childPrim.GetPath())
                        if childPathStr in view.nodes:
                            select_node(view, childPathStr)
                            selectedExistingCount += 1
                        elif view._addPrimAsNode(childPrim, stage, addedNodeIds):
                            addedCount += 1
                else:
                    # Empty container - show message
                    view._showPopupMessage(
                        f"Container '{prim.GetName()}' has no nodes"
                    )
            else:
                primPathStr = str(prim.GetPath())
                if primPathStr in view.nodes:
                    # Node already in graph — select it so the user can see it
                    select_node(view, primPathStr)
                    selectedExistingCount += 1
                elif view._addPrimAsNode(prim, stage, addedNodeIds):
                    addedCount += 1

        if addedCount > 0 or selectedExistingCount > 0:
            if addedCount > 0:
                # Place the newly added nodes by connection: each lands
                # beside the neighbors it wires to, and only the new
                # nodes move. Unconnected additions fall back to the
                # world origin into an empty canvas (framed onto below)
                # and to the viewport the user is looking at otherwise,
                # because the camera is about to stay exactly where
                # they left it and a node placed below far-off content
                # would simply never be seen.
                view._placeAddedNodes(
                    [view.nodes[nid] for nid in addedNodeIds if nid in view.nodes],
                    fallback=None if graphWasEmpty else view._viewportAnchor(),
                )
                view.linksChanged = True
                view.textChanged = True
                view._clearRenderCache()
                view.textRenderer.resetPositionCaches()
                view._cppIconRenderer.resetPositionCaches()
                view._nodeRenderManager.resetNodeQuadCaches()
                view._nodeTransformFrame.reset()

            # Enable prim tree syncing since we now have nodes with valid USD paths
            view.nodeGraph.syncSelectionToPrimTree = True

            # Move the camera only when the batch cannot be seen from where
            # it already is.
            #
            # Reframing on every add throws away the pan and zoom the user
            # chose, mid-task, every time they add a prim -- and it is not
            # needed, because the batch was just anchored into the viewport
            # above. But grid placement only moves nodes still at the USD
            # default (0, 0): a prim carrying an authored
            # ui:nodegraph:node:pos goes back to wherever it was last
            # dropped, which may be nowhere near the current view. Adding a
            # node the user cannot see and cannot find is worse than moving
            # the camera, so that case still frames.
            if addedNodeIds:
                # Calculate bounds of added nodes
                addedBounds = Gf.Range2d()
                for nodeId in addedNodeIds:
                    if nodeId in view.nodes:
                        node = view.nodes[nodeId]
                        nodeBounds = Gf.Range2d(
                            node.position, node.position + node.size
                        )
                        addedBounds = Gf.Range2d.GetUnion(addedBounds, nodeBounds)

                if not addedBounds.IsEmpty() and (
                    graphWasEmpty or not view._isVisible(addedBounds)
                ):
                    # Frame with 1.4 margin (40% extra space around nodes)
                    view._frameNodeBounds(addedBounds, margin=1.4)

            view.update()
            if addedCount > 0 and selectedExistingCount > 0:
                view._showPopupMessage(
                    f"Added {addedCount} node(s), selected {selectedExistingCount} existing"
                )
            elif addedCount > 0:
                view._showPopupMessage(f"Added {addedCount} node(s)")
            else:
                view._showPopupMessage(
                    f"Selected {selectedExistingCount} existing node(s)"
                )
        else:
            view._showPopupMessage("No valid nodes to add")

    except Exception as e:
        Tf.Warn(f"Error adding nodes from selection: {e}")
        import traceback

        traceback.print_exc()


def removeSelectedNodes(view):
    """Remove selected nodes from the editor view.

    When the user presses 'D', this method:
    1. Gets the currently selected nodes in the editor
    2. Removes them from the graph (visual only, not from USD)
    3. Updates links - any links to removed nodes become dangling
    """
    if not view._selectedNodes:
        view._showPopupMessage("No nodes selected")
        return

    removedCount = 0
    nodesToRemove = list(view._selectedNodes)

    for nodeId in nodesToRemove:
        if nodeId in view.nodes:
            del view.nodes[nodeId]
            view._selectedNodes.discard(nodeId)
            view._spatialIndex.removeNode(nodeId)
            removedCount += 1

    if removedCount > 0:
        view.linksChanged = True
        view.textChanged = True
        view._clearRenderCache()
        # Rebuild links immediately so surviving prims' connections to the
        # removed prim become dangling links right away. Without this, the
        # rebuild is deferred to the next paint, and any user input that
        # fires before that paint (e.g. a double-click on a now-stale
        # port) sees inconsistent state.
        view._rebuildLinks()
        view.update()
        view._showPopupMessage(f"Removed {removedCount} node(s)")
    else:
        view._showPopupMessage("No nodes removed")


def _resolveSelectedPrims(view):
    """Return the prims the user means by "the selection" in usdview.

    Clicking the Noodles canvas resets usdview's selection to the pseudo-
    root, so the live selection alone is not trustworthy while the editor
    has focus. Fall back to the last meaningful prim tree selection, the
    same way addNodesFromPrimTreeSelection does. The pseudo-root and
    duplicates (getPrims can repeat a prim via instance proxies) are
    dropped.

    Returns:
        List[Usd.Prim]: Selected prims, empty if nothing is selected.
    """
    if not view._usdviewApi:
        return []

    dataModel = getattr(view._usdviewApi, "dataModel", None)
    if not dataModel or not dataModel.selection:
        return []

    liveNonRoot = [
        prim
        for prim in dataModel.selection.getPrims()
        if prim and prim.IsValid() and prim.GetPath().pathString != "/"
    ]
    cachedNonRoot = [
        prim
        for prim in view._lastPrimTreeSelection
        if prim and prim.IsValid() and prim.GetPath().pathString != "/"
    ]
    if len(cachedNonRoot) > len(liveNonRoot):
        selectedPrims = cachedNonRoot
    else:
        selectedPrims = liveNonRoot or cachedNonRoot

    seenPaths = set()
    uniquePrims = []
    for prim in selectedPrims:
        pathStr = str(prim.GetPath())
        if pathStr in seenPaths:
            continue
        seenPaths.add(pathStr)
        uniquePrims.append(prim)
    return uniquePrims


def _resolveSelectedParentPrim(view):
    """Return the prim new prims should be created under, or None.

    Prefers usdview's focus prim - the last one clicked, which its own UI
    treats as primary - and falls back to the last prim of the resolved
    selection when the focus prim is unusable, which is what usdview
    reports after a click on the Noodles canvas.
    """
    selectedPrims = view._resolveSelectedPrims()
    if not selectedPrims:
        return None

    dataModel = getattr(view._usdviewApi, "dataModel", None)
    selection = getattr(dataModel, "selection", None) if dataModel else None
    getFocusPrim = getattr(selection, "getFocusPrim", None)
    if getFocusPrim is not None:
        try:
            focusPrim = getFocusPrim()
        except Exception:
            focusPrim = None
        if focusPrim and focusPrim.IsValid():
            focusPath = str(focusPrim.GetPath())
            if any(str(prim.GetPath()) == focusPath for prim in selectedPrims):
                return focusPrim

    return selectedPrims[-1]
