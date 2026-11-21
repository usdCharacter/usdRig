#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Shared fixture construction for the relationship scenario suite.

testUsdNoodlesGraphViewRelationships drives GraphView methods unbound against
duck-typed SimpleNamespace views. Everything here is the construction those
scenarios repeat around themselves -- link and node stubs, view scaffolding,
and the patch scaffolding around the USD link collectors -- never a scenario
body or an assertion. Each patch target stays a module-attribute string, so it
resolves through UsdNoodles.graphView when the patch is entered rather than
through an import-time copy of the symbol.

The leading underscore keeps this module out of ``-p "test*.py"`` discovery;
testenv/sitecustomize.py keeps the directory importable.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pxr import Gf
from UsdNoodles.graphView import GraphView, _makeStageSync
from UsdNoodles.pinUtils import RELATIONSHIP_LINK_BASE_COLOR
from pxr.Usdviewq.qt import QtCore


class Projection:
    def data(self):
        return [1.0] * 16


def fake_node_vertex(*args):
    return SimpleNamespace(x=args[0], y=args[1], z=args[2], raw=args)


def make_relationship_link():
    return SimpleNamespace(
        sourceNodeId="/Constraint",
        sourcePort="affects",
        targetNodeId="/Driven",
        targetPort="affects",
        sourcePropertyName="affects",
        targetPropertyName="",
        start=Gf.Vec2d(0.0, 0.0),
        end=Gf.Vec2d(20.0, 0.0),
        hovered=False,
        selected=False,
        is_relationship_link=True,
        isDangling=False,
    )


def patched_graph_symbol(name, **kwargs):
    """Patch a UsdNoodles.graphView module global for the block's duration."""
    return patch("UsdNoodles.graphView.%s" % name, **kwargs)


def patched_collect_links(**kwargs):
    """Patch UsdNoodles.graphView.collect_links_for_prim for the block."""
    return patched_graph_symbol("collect_links_for_prim", **kwargs)


def patched_module_collect_links(module, **kwargs):
    """Patch collect_links_for_prim in one of the node-graph loaders."""
    return patch("%s.collect_links_for_prim" % module, **kwargs)


def collect_record(
    source_node_id,
    source_pin,
    target_node_id,
    target_pin,
    *,
    property_owner=None,
    property_name=None,
    source_property=None,
    target_property=None,
    is_input_link=False,
    is_relationship_link=True,
):
    """One raw link record as the USD collectors hand it to the graph."""
    return {
        "sourceNodeId": source_node_id,
        "sourcePinName": source_pin,
        "targetNodeId": target_node_id,
        "targetPinName": target_pin,
        "is_input_link": is_input_link,
        "propertyOwnerNodeId": property_owner or source_node_id,
        "propertyName": property_name or source_pin,
        "is_relationship_link": is_relationship_link,
        "sourcePropertyName": source_property or source_pin,
        "targetPropertyName": target_property or "",
    }


def back_ref_stage(*traverse_paths):
    """A stage whose Traverse yields one MagicMock prim per path."""
    prims = []
    for path in traverse_paths:
        prim = MagicMock()
        prim.GetPath.return_value = path
        prims.append(prim)
    stage = MagicMock()
    stage.Traverse.return_value = prims
    return stage


def back_ref_link_data(
    source_node_id,
    source_pin,
    target_node_id,
    target_pin,
    *,
    source_property,
    target_property,
):
    """One cached back-reference record for _addBackReferenceDanglingLinks."""
    return {
        "sourceNodeId": source_node_id,
        "sourcePinName": source_pin,
        "targetNodeId": target_node_id,
        "targetPinName": target_pin,
        "sourcePropertyName": source_property,
        "targetPropertyName": target_property,
    }


def graph_link(
    source_node_id,
    source_pin,
    target_node_id,
    target_pin,
    *,
    source_property=None,
    target_property=None,
):
    """A link already in view.links, as the dedup keyset reads it."""
    return SimpleNamespace(
        sourceNodeId=source_node_id,
        sourcePort=source_pin,
        targetNodeId=target_node_id,
        targetPort=target_pin,
        sourcePropertyName=(
            source_property if source_property is not None else source_pin
        ),
        targetPropertyName=target_property or "",
    )


def make_back_ref_view(stage, nodes, links=None):
    """Build a SimpleNamespace view shaped for _addBackReferenceDanglingLinks."""
    view = SimpleNamespace(
        links=list(links or []),
        nodes=dict(nodes),
        nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=stage)),
        _backReferenceLinkDataCache=None,
        _setupDanglingLinkEndpoint=MagicMock(),
        _makeLinkFromUsdData=lambda d: SimpleNamespace(
            sourceNodeId=d["sourceNodeId"],
            sourcePort=d.get("sourcePinName", ""),
            targetNodeId=d["targetNodeId"],
            targetPort=d.get("targetPinName", ""),
            sourcePropertyName=d.get("sourcePropertyName", ""),
            targetPropertyName=d.get("targetPropertyName", ""),
            isDangling=False,
        ),
    )
    view._invalidateBackReferenceLinkDataCache = (
        lambda: GraphView._invalidateBackReferenceLinkDataCache(view)
    )
    return view


def make_temp_link_render_view(temp_link, valid_target):
    """Fake self for _renderTemporaryLink; captures the noodle's free end
    as it is passed to renderLinks so the prim-target inset is observable."""
    captured = {}

    def _capture_render(links, *args, **kwargs):
        captured["free_end"] = (float(links[0].end[0]), float(links[0].end[1]))

    return (
        SimpleNamespace(
            _dragLinkValidTarget=valid_target,
            _hoveredPort=None,
            _dragLinkSourceIsOutput=True,
            _dragLinkSourcePort="myRel",
            _dragLinkTempLink=temp_link,
            _cachedLinkRelationshipBaseColor=[1.0, 0.5, 0.5, 1.0],
            _cachedLinkActiveColor=[0.0, 0.0, 1.0, 1.0],
            _cachedLinkHoveredColor=[1.0, 1.0, 0.0],
            _cachedLinkSelectedColor=[1.0, 1.0, 0.3],
            _cachedPortRingThickness=1.0,
            zoom=1.0,
            panX=0.0,
            panY=0.0,
            width=MagicMock(return_value=800),
            height=MagicMock(return_value=600),
            _worldSpaceProjectionMatrix=MagicMock(
                return_value=SimpleNamespace(
                    data=MagicMock(return_value=[0.0] * 16)
                )
            ),
            _renderConfig=MagicMock(return_value=MagicMock()),
            linkRenderer=SimpleNamespace(
                renderLinks=MagicMock(side_effect=_capture_render)
            ),
            _nodeRenderManager=SimpleNamespace(
                renderEndpointTriangle=MagicMock(),
                renderEndpointCircle=MagicMock(),
            ),
            defaultRenderer=SimpleNamespace(
                getPortWidth=MagicMock(return_value=8.0)
            ),
        ),
        captured,
    )


def make_temp_link_triangle_view(*, end_pos, start_pos, valid_target):
    """Build a minimal view mock for _renderTemporaryLink triangle path."""
    temp_link = SimpleNamespace(
        start=start_pos,
        end=end_pos,
        is_relationship_link=True,
    )
    render_triangle = MagicMock()
    return (
        SimpleNamespace(
            _dragLinkValidTarget=valid_target,
            _hoveredPort=None,
            _dragLinkSourceIsOutput=True,
            _dragLinkSourcePort="affects",
            _dragLinkTempLink=temp_link,
            _cachedLinkHoveredColor=(1.0, 1.0, 0.0, 1.0),
            _cachedLinkSelectedColor=(1.0, 1.0, 0.3, 1.0),
            _cachedLinkActiveColor=(0.5, 0.5, 0.5, 1.0),
            _cachedLinkRelationshipBaseColor=(1.0, 0.5, 0.5, 1.0),
            linkRenderer=SimpleNamespace(renderLinks=MagicMock()),
            nodes={},
            zoom=1.0,
            panX=0.0,
            panY=0.0,
            width=lambda: 100,
            height=lambda: 80,
            _worldSpaceProjectionMatrix=lambda: Projection(),
            _nodeRenderManager=SimpleNamespace(
                renderEndpointTriangle=render_triangle
            ),
            defaultRenderer=SimpleNamespace(
                getPortWidth=MagicMock(return_value=8.0)
            ),
            _cachedPortRingThickness=2.0,
            _renderConfig=lambda: None,
        ),
        render_triangle,
    )


def make_connection_view(*, delete_old_returns=True, show_popup=True):
    """View shaped for _createConnection with an authoring library standing in
    for the USD write path. Returns (view, library, output_node, input_node).
    """
    stage = MagicMock()
    root_layer = MagicMock()
    edit_target = MagicMock()
    edit_target.GetLayer.return_value = root_layer
    stage.GetEditTarget.return_value = edit_target
    stage.GetRootLayer.return_value = root_layer
    library = MagicMock()
    library.create_connection.return_value = True
    output_node = SimpleNamespace(outputLinks=[])
    input_node = SimpleNamespace(inputLinks=[])
    fields = dict(
        _resolveConnectionEndpoints=MagicMock(
            return_value=(
                output_node,
                input_node,
                MagicMock(),
                MagicMock(),
                library,
            )
        ),
        nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=stage)),
        _stageSync=_makeStageSync(None),
        _abortConnection=MagicMock(),
        _deleteOldConnection=MagicMock(return_value=delete_old_returns),
        _addLinkToGraph=MagicMock(),
        _clearReconnectState=MagicMock(),
        _invalidateBackReferenceLinkDataCache=MagicMock(),
        _reconnectingLink=False,
        _reconnectOldSourceNodeId=None,
        _reconnectOldSourcePort=None,
        _reconnectOldTargetNodeId=None,
        _reconnectOldTargetPort=None,
        update=MagicMock(),
    )
    if show_popup:
        fields["_showPopupMessage"] = MagicMock()
    return (
        SimpleNamespace(**fields),
        library,
        output_node,
        input_node,
    )


def make_arrowhead_view(link):
    """View shaped for _renderRelationshipArrowheads over one relationship link."""
    view = SimpleNamespace(
        zoom=1.0,
        links=[link],
        _cachedLinkRelationshipBaseColor=RELATIONSHIP_LINK_BASE_COLOR + (1.0,),
        _appendTriangleVertices=lambda v, pts, d, c: (
            GraphView._appendTriangleVertices(None, v, pts, d, c)
        ),
        _drawNodeVertices=MagicMock(),
    )
    view._relationshipLinkColor = lambda rel_link, hovered, selected: (
        GraphView._relationshipLinkColor(view, rel_link, hovered, selected)
    )
    return view


def make_paint_links_view(links, *, width=100, height=80):
    """View shaped for paintLinks through the held-vector render path."""
    render_from_graph = MagicMock()
    view = SimpleNamespace(
        linkRenderer=SimpleNamespace(renderLinksFromGraph=render_from_graph),
        nodeGraph=SimpleNamespace(syncLinksFromModels=MagicMock()),
        links=links,
        _linkUnderCursor=-1,
        _lastSyncedLinkHover=-1,
        _linksNeedSync=True,
        _worldSpaceProjectionMatrix=lambda: Projection(),
        _cachedLinkHoveredColor=(1.0, 1.0, 0.0, 1.0),
        _cachedLinkSelectedColor=(1.0, 1.0, 0.3, 1.0),
        _cachedLinkAttributeBaseColor=(0.5, 0.5, 0.5, 1.0),
        _cachedLinkRelationshipBaseColor=RELATIONSHIP_LINK_BASE_COLOR + (1.0,),
        _cachedLinkHighlightedColor=(1.0, 1.0, 1.0, 1.0),
        zoom=1.0,
        panX=0.0,
        panY=0.0,
        width=lambda: width,
        height=lambda: height,
        _renderRelationshipArrowheads=MagicMock(),
        _renderConfig=lambda: None,
    )
    view._syncLinksIfNeeded = lambda graph: GraphView._syncLinksIfNeeded(view, graph)
    return view, render_from_graph


def make_regular_link():
    """A plain attribute link, the foil a relationship test contrasts against."""
    return SimpleNamespace(
        sourceNodeId="/A",
        sourcePort="out",
        targetNodeId="/B",
        targetPort="in",
        start=Gf.Vec2d(0.0, 0.0),
        end=Gf.Vec2d(10.0, 10.0),
        hovered=False,
        selected=False,
        is_relationship_link=False,
        isDangling=False,
    )


def make_dangling_link(
    source_node_id,
    source_port,
    target_node_id,
    target_port="",
    *,
    is_dangling=True,
    with_properties=True,
    source_property=None,
    target_property="",
    with_endpoints=False,
):
    """A dangling link record, as the expansion path reads it."""
    fields = dict(
        isDangling=is_dangling,
        sourceNodeId=source_node_id,
        sourcePort=source_port,
        targetNodeId=target_node_id,
        targetPort=target_port,
    )
    if with_properties:
        fields["sourcePropertyName"] = (
            source_port if source_property is None else source_property
        )
        fields["targetPropertyName"] = target_property
    if with_endpoints:
        fields["data_sourceNodeId"] = source_node_id
        fields["data_targetNodeId"] = target_node_id
    return SimpleNamespace(**fields)


def make_node_expansion_view(
    *,
    stage,
    create_node_from_prim,
    nodes,
    viewport_anchor,
    next_z_order=5,
    links=None,
    real_placement=True,
):
    """View shaped for the add-node-from-dangling-link expansion path.

    real_placement binds the real _placeAddedNodes onto the fake view, so
    placement is the production behaviour rather than a mock.
    """
    view = SimpleNamespace(
        _usdviewApi=SimpleNamespace(stage=stage),
        _showPopupMessage=MagicMock(),
        _nodeFactory=SimpleNamespace(
            create_node_from_prim=create_node_from_prim,
        ),
        _populateNodeLinksFromPrim=MagicMock(),
        _viewportAnchor=MagicMock(return_value=viewport_anchor),
        _nextZOrder=next_z_order,
        fontAtlas=None,
        nodeGraph=SimpleNamespace(
            _calculateNodeSize=MagicMock(),
            syncSelectionToPrimTree=False,
        ),
        textRenderer=SimpleNamespace(calculateTextWidth=MagicMock()),
        nodes=nodes,
        _clearRenderCache=MagicMock(),
        update=MagicMock(),
        linksChanged=False,
        textChanged=False,
    )
    if links is not None:
        view.links = links
    if real_placement:
        view._placeAddedNodes = lambda added, fallback=None: (
            GraphView._placeAddedNodes(view, added, fallback=fallback)
        )
    else:
        view._placeAddedNodes = MagicMock(return_value=[])
    return view


def make_sdf_prim(path, *, type_name="Shader"):
    """A prim double carrying a stringifiable SdfPath, as the prim-tree
    readers use it."""
    sdf_path = MagicMock()
    sdf_path.pathString = path
    sdf_path.__str__ = MagicMock(return_value=path)
    prim = MagicMock()
    prim.IsValid.return_value = True
    prim.GetPath.return_value = sdf_path
    prim.GetTypeName.return_value = type_name
    prim.GetName.return_value = path.rsplit("/", 1)[-1]
    return prim


def make_prim_tree_view(
    prims,
    *,
    last_selection=None,
    nodes=None,
    viewport_anchor=(0.0, 0.0),
    is_visible=True,
    added_position=(0.0, 0.0),
):
    """View shaped for addNodesFromPrimTreeSelection.

    The fake _addPrimAsNode records every path it is asked to add. Returns
    (view, added_paths).
    """
    added_paths = []
    view = SimpleNamespace(
        _usdviewApi=SimpleNamespace(
            dataModel=SimpleNamespace(
                selection=SimpleNamespace(getPrims=MagicMock(return_value=list(prims))),
            ),
            stage=MagicMock(),
        ),
        _lastPrimTreeSelection=list(last_selection or []),
        _showPopupMessage=MagicMock(),
        nodes=dict(nodes or {}),
        _selectedNodes=set(),
        _getContainerChildNodes=MagicMock(return_value=[]),
        nodeGraph=SimpleNamespace(syncSelectionToPrimTree=False),
        _clearRenderCache=MagicMock(),
        textRenderer=SimpleNamespace(resetPositionCaches=MagicMock()),
        _nodeRenderManager=SimpleNamespace(resetNodeQuadCaches=MagicMock()),
        _nodeTransformFrame=SimpleNamespace(reset=MagicMock()),
        # addNodesFromPrimTreeSelection places the freshly added nodes and
        # resets the text + icon position caches. Missing either of these
        # makes production raise inside the method's own try/except, which
        # turns everything below the raise into a vacuous pass.
        _placeAddedNodes=MagicMock(return_value=[]),
        _cppIconRenderer=SimpleNamespace(resetPositionCaches=MagicMock()),
        _viewportAnchor=MagicMock(return_value=viewport_anchor),
        # True means the placed batch lands where the camera is looking.
        _isVisible=MagicMock(return_value=is_visible),
        _frameNodeBounds=MagicMock(),
        update=MagicMock(),
        linksChanged=False,
        textChanged=False,
    )

    def fake_add(prim, _stage, addedNodeIds):
        path = str(prim.GetPath())
        added_paths.append(path)
        view.nodes[path] = SimpleNamespace(
            id=path,
            selected=False,
            position=Gf.Vec2d(*added_position),
            size=Gf.Vec2d(10.0, 10.0),
        )
        addedNodeIds.append(path)
        return True

    view._addPrimAsNode = MagicMock(side_effect=fake_add)
    return view, added_paths


def make_graph_node(node_id, *, position=(0.0, 0.0), size=(10.0, 10.0)):
    """A node already in the graph, as the placement and framing readers use it."""
    return SimpleNamespace(
        id=node_id,
        selected=False,
        position=Gf.Vec2d(*position),
        size=Gf.Vec2d(*size),
    )


_UNSET = object()


def make_link_drag_view(
    temp_link,
    *,
    source_port,
    source_node,
    nodes,
    valid_target=False,
    synthetic_target=None,
    resolved_target=None,
    hovered_port=None,
    node_id_under_cursor="",
    **extras,
):
    """View shaped for _updateLinkDrag over one temp link and node set.

    Each scenario keeps its own temp link and any extra accessor the branch
    under test needs (a node-hit probe, an endpoint resolver) in ``extras``,
    so a mock only exists where the original scenario wrote it.
    """
    view = SimpleNamespace(
        zoom=1.0,
        panX=0.0,
        panY=0.0,
        _dragLinkCursorPos=None,
        _dragLinkSourceIsOutput=True,
        _dragLinkSourcePort=source_port,
        _dragLinkTempLink=temp_link,
        _dragLinkValidTarget=valid_target,
        _dragLinkSyntheticTarget=synthetic_target,
        _dragLinkResolvedTarget=resolved_target,
        _hoveredPort=hovered_port,
        nodes=nodes,
        nodeIdUnderCursor=node_id_under_cursor,
        _dragLinkSourceNode=source_node,
    )
    for name, value in extras.items():
        setattr(view, name, value)
    return view


def make_complete_drag_view(
    *,
    source_port,
    source_node,
    has_exact=False,
    temp_link=_UNSET,
    **extras,
):
    """View shaped for _completeLinkDrag's duplicate/connection decisions."""
    fields = dict(
        _dragLinkSourcePort=source_port,
        _dragLinkSourceIsOutput=True,
        _dragLinkSourceNode=source_node,
        _dragLinkValidTarget=True,
        _dragLinkResolvedTarget=None,
        _dragLinkSyntheticTarget=None,
        _hoveredPort=None,
        _findLinkToInput=MagicMock(),
        _createConnection=MagicMock(),
        _hasExactLink=MagicMock(return_value=has_exact),
        update=MagicMock(),
        nodes={},
        _reconnectingLink=False,
    )
    if temp_link is not _UNSET:
        fields["_dragLinkTempLink"] = temp_link
    fields.update(extras)
    return SimpleNamespace(**fields)


def make_endpoint_node(output_pin, *, position=(10.0, 20.0), size=(40.0, 30.0)):
    """A node with one visible output row, as the endpoint resolver reads it."""
    return SimpleNamespace(
        position=Gf.Vec2d(*position),
        size=Gf.Vec2d(*size),
        renderer=SimpleNamespace(
            getPortPosition=MagicMock(return_value=Gf.Vec2d(50.0, 35.0))
        ),
        outputPins=[output_pin],
        inputPins=[],
        _dual_pin_names=set(),
    )


def make_endpoint_view(node, *, display_pin, port_position=None):
    """View wiring the real row/visible-port endpoint helpers onto one node."""
    view = SimpleNamespace(
        defaultRenderer=None,
        fontAtlas=None,
        _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
        _resolveLinkDisplayPinName=MagicMock(return_value=display_pin),
        _getPropertyOutputSide=GraphView._getPropertyOutputSide,
        _mirrorPortPosition=GraphView._mirrorPortPosition,
        _getPortPosition=(
            MagicMock()
            if port_position is None
            else MagicMock(return_value=port_position)
        ),
    )
    view._getVisiblePortPosition = lambda node_arg, pin_name, is_output: (
        GraphView._getVisiblePortPosition(view, node_arg, pin_name, is_output)
    )
    view._getRelationshipRowEndpointPosition = lambda node_arg, pin_name, is_output: (
        GraphView._getRelationshipRowEndpointPosition(
            view, node_arg, pin_name, is_output
        )
    )
    view._nodeHasVisiblePort = lambda node_arg, pin_name, is_output: (
        GraphView._nodeHasVisiblePort(view, node_arg, pin_name, is_output)
    )
    return view


def make_row_renderer():
    """The renderer metrics both _updateHoveredPort scenarios read."""
    return SimpleNamespace(
        getPortAtPoint=MagicMock(return_value={"found": False}),
        getPortWidth=MagicMock(return_value=8.0),
        getPortMarginV=MagicMock(return_value=4.0),
        getTitleFontSize=MagicMock(return_value=12.0),
        getPortFontSize=MagicMock(return_value=10.0),
        getPortTypeFontSize=MagicMock(return_value=8.0),
        getPortSpacing=MagicMock(return_value=6.0),
    )


def make_row_hover_node(
    renderer,
    *,
    size=(120.0, 80.0),
    input_pins=(),
    input_row_kinds=(),
    input_row_slots=(),
    output_pins=("affects",),
    output_row_kinds=(0,),
    output_row_slots=(0,),
):
    """A node with the cached layout and row kind/slot mirrors the hit tests read.

    Cached layout _getRowHitMetrics reads: title=12*1.0+4*2=20, row=10+8+6=24.
    """
    node = SimpleNamespace(
        renderer=renderer,
        position=Gf.Vec2d(10.0, 20.0),
        size=Gf.Vec2d(*size),
        layoutPortStartY=20.0,
        layoutPortLineHeight=24.0,
        outputPins=list(output_pins),
        _output_row_kinds=list(output_row_kinds),
        _output_row_slots=list(output_row_slots),
        # C++ view-model accessor names the migrated readers consume (the
        # _*_row_kinds/_*_row_slots mirrors above are kept so the pre-rename
        # readers lower in the stack still pass).
        outputRowKinds=list(output_row_kinds),
        outputRowSlots=list(output_row_slots),
        _title_collapsed=False,
        is_relationship_pin=lambda pin_name, is_output: (
            pin_name == "affects" and is_output
        ),
    )
    if input_pins:
        node.inputPins = list(input_pins)
        node._input_row_kinds = list(input_row_kinds)
        node._input_row_slots = list(input_row_slots)
        node.inputRowKinds = list(input_row_kinds)
        node.inputRowSlots = list(input_row_slots)
    return node


def make_row_hover_view(node, *, relationship_row_hover=True,
                        property_row_hover=False, port_position=None):
    """View shaped for _updateHoveredPort's row-hit path over one node.

    True binds the real GraphView helper, False leaves the accessor off the
    view entirely, and any other value is installed as-is.
    """
    view = SimpleNamespace(
        zoom=1.0,
        panX=0.0,
        panY=0.0,
        defaultRenderer=None,
        fontAtlas=SimpleNamespace(ascender=0.8, descender=-0.2, lineHeight=1.0),
        _cachedPortHitRadiusMultiplier=1.5,
        _draggingLink=True,
        _dragLinkSourcePort="sources",
        _dragLinkTempLink=SimpleNamespace(is_relationship_link=True),
        _hoveredPort=None,
        nodeIdUnderCursor="constraintB",
        nodes={"constraintB": node},
        _spatialIndex=SimpleNamespace(
            queryRegion=MagicMock(return_value=(["constraintB"], []))
        ),
        _nodeRenderManager=SimpleNamespace(
            portAtPoint=MagicMock(return_value={"found": False})
        ),
        nodeGraph=SimpleNamespace(),
        _getRowHitMetrics=lambda n, r: GraphView._getRowHitMetrics(view, n, r),
        _getPortPosition=MagicMock() if port_position is None else port_position,
    )
    view._setHoveredPort = lambda new_hover: GraphView._setHoveredPort(
        view, new_hover
    )
    view._relationshipDragRowHover = lambda world_pos_arg, port_extent_arg: (
        GraphView._relationshipDragRowHover(view, world_pos_arg, port_extent_arg)
    )
    if relationship_row_hover is True:
        view._findRelationshipRowHover = (
            lambda node_arg, node_id_arg, world_pos_arg: (
                GraphView._findRelationshipRowHover(
                    view, node_arg, node_id_arg, world_pos_arg
                )
            )
        )
    elif relationship_row_hover is not False:
        view._findRelationshipRowHover = relationship_row_hover
    if property_row_hover is True:
        view._findPropertyRowHover = (
            lambda node_arg, node_id_arg, world_pos_arg: (
                GraphView._findPropertyRowHover(
                    view, node_arg, node_id_arg, world_pos_arg
                )
            )
        )
    elif property_row_hover is not False:
        view._findPropertyRowHover = property_row_hover
    return view


def make_box_node(*, position=(10.0, 12.0), size=(30.0, 40.0)):
    """A node whose body geometry the drag and endpoint paths read."""
    return SimpleNamespace(position=Gf.Vec2d(*position), size=Gf.Vec2d(*size))


def make_node_prim(name, *, type_name="RigJoint"):
    """A valid prim double the node factory is asked to build a node from."""
    prim = MagicMock()
    prim.IsValid.return_value = True
    prim.GetTypeName.return_value = type_name
    prim.GetName.return_value = name
    return prim


def make_stage_at_path(prim=None, *, side_effect=None):
    """A stage that hands out prims by path, as the expansion path reads it."""
    stage = MagicMock()
    if side_effect is not None:
        stage.GetPrimAtPath.side_effect = side_effect
    else:
        stage.GetPrimAtPath.return_value = prim
    return stage


def make_link_click_view(link):
    """View shaped for mousePressEvent's link-dispatch branch over one link.

    Production gates the minimap branch on _cachedShowMinimap, which it sets
    in _initCachedSettings (default True); this unbound-call fixture never
    runs that, so the flag is set here. There is no rename, value editor, or
    token popup in progress, and no relationship arrowhead under the cursor
    (these are data links), so the click falls through to the regular
    link-dispatch path.
    """
    return SimpleNamespace(
        setFocus=MagicMock(),
        lastMousePos=None,
        ctrlPressedOnMouseDown=False,
        mousePosOnMouseDown=None,
        nodeCreationHotbox=SimpleNamespace(is_visible=False),
        _renamingNodeId="",
        # Same for the value editor and the token popup, and the value
        # cells are off so a press cannot be claimed by one.
        _valueEditTarget=None,
        _tokenPopup=None,
        _mungState=None,
        _cachedShowAttributeValues=False,
        _valueCellAtPoint=MagicMock(return_value=None),
        minimap=SimpleNamespace(handleMousePress=MagicMock(return_value=False)),
        _cachedShowMinimap=True,
        _updateNodeUnderCursor=MagicMock(),
        _reconcileMovedNodes=MagicMock(),
        spacePressed=False,
        nodeIdUnderCursor=None,
        nodes={},
        _hoveredPort=None,
        zoom=1.0,
        panX=0.0,
        panY=0.0,
        links=[link],
        linkRenderer=SimpleNamespace(
            findLinkUnderCursor=MagicMock(return_value=0),
        ),
        _cachedLinkLineWidth=12.0,
        _spatialIndex=None,
        _selectedNodes=set(),
        _selectedLinks=set(),
        clearSelection=MagicMock(),
        clearLinkSelection=MagicMock(),
        _selectLink=MagicMock(),
        _startLinkReconnect=MagicMock(),
        _findArrowheadUnderCursor=MagicMock(return_value=-1),
        _toggleLinkSelection=MagicMock(),
        _updateLinkSelectionFromNodes=MagicMock(),
        _syncLinkSelectionToPrimtree=MagicMock(),
        marqueeActive=False,
        marqueeStart=None,
        marqueeCurrent=None,
        update=MagicMock(),
        draggingNodes=False,
        _nextZOrder=0,
        textChanged=False,
        linksChanged=False,
    )


def make_link_click_link(*, isDangling):
    """The data link both click-dispatch scenarios click."""
    return SimpleNamespace(
        isDangling=isDangling,
        selected=False,
        hovered=False,
        highlighted=False,
        sourceNodeId="/A",
        sourcePort="out",
        targetNodeId="/B",
        targetPort="in",
        start=Gf.Vec2d(0.0, 0.0),
        end=Gf.Vec2d(20.0, 0.0),
        is_relationship_link=False,
        sourcePropertyName="out",
        targetPropertyName="in",
        data_sourceNodeId="/A",
        data_targetNodeId="/B",
        is_input_link=False,
    )


def make_mouse_press_event():
    """A left-button press at (50, 50) with no modifiers."""
    event = MagicMock()
    event.buttons.return_value = QtCore.Qt.MouseButton.LeftButton
    event.position.return_value.toPoint.return_value = QtCore.QPoint(50, 50)
    event.modifiers.return_value = QtCore.Qt.KeyboardModifier(0)
    return event


def make_row_metrics_renderer():
    """Renderer metrics behind the row-band numbers both row suites model."""
    return SimpleNamespace(
        getPortMarginV=MagicMock(return_value=4.0),
        getTitleFontSize=MagicMock(return_value=14.0),
        getPortFontSize=MagicMock(return_value=10.0),
        getPortTypeFontSize=MagicMock(return_value=8.0),
        getPortSpacing=MagicMock(return_value=2.0),
        getPortWidth=MagicMock(return_value=8.0),
    )


def make_row_metrics_view():
    return SimpleNamespace(
        fontAtlas=SimpleNamespace(ascender=0.8, descender=-0.2, lineHeight=1.2),
    )


def make_typed_row_node(schemaTypeName):
    """The cached layout _getRowHitMetrics reads (layoutPortStartY /
    layoutPortLineHeight) instead of computing from the renderer. layoutNode
    bakes the title + (optional schema subtitle) into layoutPortStartY, so a
    typed node's band starts below a bare node's; model that here. (The
    schema-subtitle offset itself is unit-tested in NoodlesCoreTest.) Values
    mirror the renderer mocks so this also holds on the pre-migration
    renderer-computed path in the lower diffs.
    """
    return SimpleNamespace(
        position=Gf.Vec2d(0.0, 0.0),
        size=Gf.Vec2d(100.0, 200.0),
        schemaTypeName=schemaTypeName,
        layoutPortLineHeight=10.0 * 1.2 + 8.0 * 1.2 + 2.0,
        layoutPortStartY=(
            14.0 * 1.0
            + 4.0 * 2.0
            + 4.0
            + (14.0 * 0.65 * 1.2 if schemaTypeName else 0.0)
        ),
    )


def make_edge_drag_node():
    """Typed node (schemaTypeName set, like every REQ 1 USD-prim node), one
    input row and one output row, both at slot 0. No glyphs are drawn (REQ 3)
    since neither pin is connected.

    Cached layout the migrated _getRowHitMetrics reads at PR7+ (typed node:
    title 14*1.0 + marginV*2 + marginV + schema subtitle 14*0.65*1.2 = 36.92;
    row = portFont*1.2 + portTypeFont*1.2 + spacing = 23.6). y=45 lands in the
    first row band [36.92, 60.52]; y=5 is above it.
    """
    return SimpleNamespace(
        position=Gf.Vec2d(0.0, 0.0),
        size=Gf.Vec2d(100.0, 200.0),
        schemaTypeName="IrFkController",
        renderer=None,
        inputPins=["a"],
        outputPins=["x"],
        _input_row_slots=[0],
        _output_row_slots=[0],
        _input_row_kinds=[0],
        _output_row_kinds=[0],
        # C++ view-model accessor names the migrated _rowEdgePinCandidates /
        # _getRowHitMetrics read (slot == index == 0 here, so the toy node
        # still passes; grouped nodes where slot != index exercise the real
        # path).
        inputRowSlots=[0],
        outputRowSlots=[0],
        inputRowKinds=[0],
        outputRowKinds=[0],
        _dual_pin_names=set(),
        _title_collapsed=False,
        layoutPortStartY=14.0 * 1.0 + 4.0 * 2.0 + 4.0 + 14.0 * 0.65 * 1.2,
        layoutPortLineHeight=10.0 * 1.2 + 8.0 * 1.2 + 2.0,
    )


def make_edge_drag_view():
    """View with the real production row-resolution methods bound, so the whole
    resolution stack runs. No glyph is drawn (REQ 3), so there is no port
    position to snap to; that exercises the band-center fallback inside
    _pinAtRowEdge.
    """
    view = SimpleNamespace(
        fontAtlas=SimpleNamespace(ascender=0.8, descender=-0.2, lineHeight=1.2),
        defaultRenderer=make_row_metrics_renderer(),
    )
    view._getRowHitMetrics = GraphView._getRowHitMetrics.__get__(view)
    view._rowEdgePinCandidates = GraphView._rowEdgePinCandidates.__get__(view)
    view._pinAtRowEdge = GraphView._pinAtRowEdge.__get__(view)
    view._findRowEdgeDragStart = GraphView._findRowEdgeDragStart.__get__(view)
    # No glyph is drawn (REQ 3), so there is no port position to snap to; this
    # exercises the band-center fallback inside _pinAtRowEdge.
    view._getPortPosition = lambda node, pin, is_output: None
    return view


def make_change_block_view():
    """View shaped for the layer-grouped delete/restore batching methods."""
    return SimpleNamespace(
        nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=MagicMock())),
        _groupNodesByLayer=MagicMock(return_value={}),
        _groupLinksByLayer=MagicMock(return_value={}),
        _setNodesActiveOnLayer=MagicMock(),
        _runLinkOperationOnLayer=MagicMock(),
    )


def make_endpoint_pin_link(source_id, source_port, target_id, target_port,
                           *, is_rel=False):
    """A stored link, as the endpoint-pin pass reads its endpoints."""
    return SimpleNamespace(
        sourceNodeId=source_id,
        sourcePort=source_port,
        targetNodeId=target_id,
        targetPort=target_port,
        is_relationship_link=is_rel,
    )


def make_pinnable_node(node_id, *, input_links=None, output_links=None,
                       add_returns=True):
    """A node recording each add_connection_endpoint_pins call it receives."""
    calls = []

    def add(input_names=(), output_names=()):
        calls.append((list(input_names), list(output_names)))
        return add_returns

    return SimpleNamespace(
        id=node_id,
        inputLinks=list(input_links or []),
        outputLinks=list(output_links or []),
        add_connection_endpoint_pins=add,
        added_calls=calls,
    )


def make_surface_view(nodes, *, font_atlas=True):
    """View shaped for _surfaceConnectionEndpointPins over a node list."""
    return SimpleNamespace(
        nodes={n.id: n for n in nodes},
        fontAtlas=object() if font_atlas else None,
        textRenderer=SimpleNamespace(calculateTextWidth=lambda *a, **k: 0.0),
        nodeGraph=SimpleNamespace(_calculateNodeSize=MagicMock()),
    )
