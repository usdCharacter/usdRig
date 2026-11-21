#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct hit-testing checks against the staged interaction owner.

Point-in-node and point-in-port agreement with the published bounds, row
edge and pin candidate ordering, arrowhead and dangling-indicator hits,
viewport jumps, and the read-only contract: a hover query records intent
on the view and leaves model position versions and link fields unchanged.
"""

import copy
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pxr import Gf

from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview import interaction
from UsdNoodles.widgets import valueCell
from pxr.Usdviewq.qt import QtCore


MOVED = [
    "_getNodeIdAtWorldPos",
    "_getPropertyAtPoint",
    "_findPropertyRowHover",
    "_findRelationshipRowHover",
    "_relationshipDragRowHover",
    "_getRowHitMetrics",
    "_valueCellAtPoint",
    "_findRowEdgeDragStart",
    "_rowEdgePinCandidates",
    "_pinAtRowEdge",
    "_findArrowheadUnderCursor",
    "_findDanglingIndicatorUnderCursor",
    "_findDanglingLinkAtPort",
    "_updateHoveredPort",
    "_setHoveredPort",
    "_updateNodeUnderCursor",
    "_updateValueHoverCursor",
    "_showPropertyTooltip",
    "_updatePropertyTooltip",
    "_clearPropertyTooltip",
    "_centerOnNode",
    "_setZoom",
    "_zoomAroundPoint",
    "navigateBackward",
    "navigateForward",
    "_jumpToSourceNode",
    "_jumpToTargetNode",
    "_jumpToSourceNodeInternal",
    "_jumpToTargetNodeInternal",
    "_navigateAlongRelationshipLink",
]


class _Renderer:
    def __init__(self, port_width=8.0):
        self._port_width = port_width

    def getPortWidth(self):
        return self._port_width


class _Node:
    def __init__(
        self,
        node_id="/A",
        position=(0.0, 0.0),
        size=(100.0, 200.0),
        z_order=0,
        inputPins=None,
        outputPins=None,
        input_slots=None,
        output_slots=None,
        input_kinds=None,
        output_kinds=None,
        dual=None,
        output_dual=None,
        title_collapsed=False,
        renderer=None,
        port_start_y=30.0,
        port_line_height=20.0,
        input_centers=None,
        output_centers=None,
        position_version=7,
    ):
        self.id = node_id
        self.position = Gf.Vec2d(*position)
        self.size = Gf.Vec2d(*size)
        self.zOrder = z_order
        self.inputPins = list(inputPins or [])
        self.outputPins = list(outputPins or [])
        self._input_row_slots = (
            list(input_slots)
            if input_slots is not None
            else list(range(len(self.inputPins)))
        )
        self._output_row_slots = (
            list(output_slots)
            if output_slots is not None
            else list(range(len(self.outputPins)))
        )
        self._input_row_kinds = (
            list(input_kinds) if input_kinds is not None else [0] * len(self.inputPins)
        )
        self._output_row_kinds = (
            list(output_kinds)
            if output_kinds is not None
            else [0] * len(self.outputPins)
        )
        self._dual_pin_names = set(dual or set())
        self._output_dual_pin_names = set(output_dual or set())
        self._title_collapsed = title_collapsed
        self.renderer = renderer
        self.layoutPortStartY = port_start_y
        self.layoutPortLineHeight = port_line_height
        self.layoutInputCenterY = list(
            input_centers
            if input_centers is not None
            else [port_start_y + i * port_line_height for i in range(len(self.inputPins))]
        )
        self.layoutOutputCenterY = list(
            output_centers
            if output_centers is not None
            else [port_start_y + i * port_line_height for i in range(len(self.outputPins))]
        )
        self.positionVersion = position_version
        self.selected = False

    inputRowSlots = property(lambda self: self._input_row_slots)
    outputRowSlots = property(lambda self: self._output_row_slots)
    inputRowKinds = property(lambda self: self._input_row_kinds)
    outputRowKinds = property(lambda self: self._output_row_kinds)
    is_title_collapsed = property(lambda self: self._title_collapsed)


class _SpatialIndex:
    """A real region test over the nodes dict, so margin and z-order paths
    are genuinely exercised rather than echoed."""

    def __init__(self, nodes):
        self._nodes = nodes

    def queryPoint(self, worldPos):
        x, y = float(worldPos[0]), float(worldPos[1])
        hits = [
            nid
            for nid, node in self._nodes.items()
            if node.position[0] <= x <= node.position[0] + node.size[0]
            and node.position[1] <= y <= node.position[1] + node.size[1]
        ]
        return hits, []

    def queryRegion(self, bounds):
        hits = [
            nid
            for nid, node in self._nodes.items()
            if not (
                bounds.GetMax()[0] < node.position[0]
                or bounds.GetMin()[0] > node.position[0] + node.size[0]
                or bounds.GetMax()[1] < node.position[1]
                or bounds.GetMin()[1] > node.position[1] + node.size[1]
            )
        ]
        return hits, []


def _make_link(**overrides):
    link = SimpleNamespace(
        sourceNodeId="/A",
        sourcePort="out",
        targetNodeId="/B",
        targetPort="in",
        sourcePropertyName="",
        targetPropertyName="",
        propertyName="",
        isDangling=False,
        danglingDirection="",
        is_relationship_link=False,
        hovered=False,
        selected=False,
        start=(0.0, 0.0),
        end=(100.0, 0.0),
        data_sourceNodeId="/A",
        data_targetNodeId="/B",
    )
    for key, value in overrides.items():
        setattr(link, key, value)
    return link


class _HitView:
    """Duck view whose hit methods are the real moved functions."""

    _getNodeIdAtWorldPos = GraphView._getNodeIdAtWorldPos
    _getPropertyAtPoint = GraphView._getPropertyAtPoint
    _findPropertyRowHover = GraphView._findPropertyRowHover
    _findRelationshipRowHover = GraphView._findRelationshipRowHover
    _relationshipDragRowHover = GraphView._relationshipDragRowHover
    _getRowHitMetrics = GraphView._getRowHitMetrics
    _valueCellAtPoint = GraphView._valueCellAtPoint
    _findRowEdgeDragStart = GraphView._findRowEdgeDragStart
    _rowEdgePinCandidates = GraphView._rowEdgePinCandidates
    _pinAtRowEdge = GraphView._pinAtRowEdge
    _findArrowheadUnderCursor = GraphView._findArrowheadUnderCursor
    _findDanglingIndicatorUnderCursor = GraphView._findDanglingIndicatorUnderCursor
    _findDanglingLinkAtPort = GraphView._findDanglingLinkAtPort
    _updateHoveredPort = GraphView._updateHoveredPort
    _setHoveredPort = GraphView._setHoveredPort
    _updateNodeUnderCursor = GraphView._updateNodeUnderCursor
    _updateValueHoverCursor = GraphView._updateValueHoverCursor
    _showPropertyTooltip = GraphView._showPropertyTooltip
    _updatePropertyTooltip = GraphView._updatePropertyTooltip
    _clearPropertyTooltip = GraphView._clearPropertyTooltip
    _centerOnNode = GraphView._centerOnNode
    _setZoom = GraphView._setZoom
    _zoomAroundPoint = GraphView._zoomAroundPoint
    navigateBackward = GraphView.navigateBackward
    navigateForward = GraphView.navigateForward
    _jumpToSourceNode = GraphView._jumpToSourceNode
    _jumpToTargetNode = GraphView._jumpToTargetNode
    _jumpToSourceNodeInternal = GraphView._jumpToSourceNodeInternal
    _jumpToTargetNodeInternal = GraphView._jumpToTargetNodeInternal
    _navigateAlongRelationshipLink = GraphView._navigateAlongRelationshipLink


class TestInteractionBindings(unittest.TestCase):
    def test_moved_functions_are_bound_by_explicit_assignment(self):
        for name in MOVED:
            with self.subTest(name=name):
                self.assertIs(getattr(GraphView, name), getattr(interaction, name))


class TestPointInNode(unittest.TestCase):
    def _view(self, nodes):
        view = _HitView()
        view.nodes = {node.id: node for node in nodes}
        view._spatialIndex = _SpatialIndex(view.nodes)
        return view

    def test_inside_returns_id(self):
        view = self._view([_Node("/A")])
        self.assertEqual(
            GraphView._getNodeIdAtWorldPos(view, Gf.Vec2d(10.0, 10.0)), "/A"
        )

    def test_outside_returns_empty(self):
        view = self._view([_Node("/A")])
        self.assertEqual(
            GraphView._getNodeIdAtWorldPos(view, Gf.Vec2d(200.0, 200.0)), ""
        )

    def test_topmost_z_order_wins(self):
        view = self._view([_Node("/A", z_order=0), _Node("/B", z_order=1)])
        self.assertEqual(
            GraphView._getNodeIdAtWorldPos(view, Gf.Vec2d(10.0, 10.0)), "/B"
        )

    def test_margin_expands_bounds(self):
        view = self._view([_Node("/A")])
        outside = Gf.Vec2d(-5.0, 10.0)
        self.assertEqual(GraphView._getNodeIdAtWorldPos(view, outside), "")
        self.assertEqual(GraphView._getNodeIdAtWorldPos(view, outside, margin=10.0), "/A")

    def test_unknown_spatial_id_is_ignored(self):
        view = self._view([_Node("/A")])
        view._spatialIndex = SimpleNamespace(
            queryPoint=lambda _pos: (["/Ghost"], []),
            queryRegion=lambda _bounds: (["/Ghost"], []),
        )
        self.assertEqual(
            GraphView._getNodeIdAtWorldPos(view, Gf.Vec2d(10.0, 10.0)), ""
        )


class TestPointInPort(unittest.TestCase):
    def _view(self, hit):
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.defaultRenderer = _Renderer()
        view._cachedPortHitRadiusMultiplier = 2.0
        view._nodeRenderManager = SimpleNamespace(
            portAtPoint=MagicMock(return_value=hit)
        )
        view.nodeGraph = object()
        view._hoveredPort = None
        view._draggingLink = False
        view._dragLinkTempLink = None
        return view

    def test_hit_publishes_port_tuple(self):
        view = self._view(
            {"found": True, "nodeId": "/A", "portName": "in", "isOutput": False}
        )
        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))
        self.assertTrue(changed)
        self.assertEqual(view._hoveredPort, ("/A", "in", False))
        view._nodeRenderManager.portAtPoint.assert_called_once_with(
            view.nodeGraph, 60.0, 50.0, 16.0
        )

    def test_same_hover_reports_no_change(self):
        view = self._view(
            {"found": True, "nodeId": "/A", "portName": "in", "isOutput": False}
        )
        view._hoveredPort = ("/A", "in", False)
        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))
        self.assertFalse(changed)
        self.assertEqual(view._hoveredPort, ("/A", "in", False))

    def test_miss_clears_hover(self):
        view = self._view(None)
        view._hoveredPort = ("/A", "in", False)
        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))
        self.assertTrue(changed)
        self.assertIsNone(view._hoveredPort)

    def test_miss_without_hover_reports_no_change(self):
        view = self._view({"found": False})
        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))
        self.assertFalse(changed)
        self.assertIsNone(view._hoveredPort)

    def test_set_hovered_port_reports_change(self):
        view = _HitView()
        view._hoveredPort = None
        self.assertTrue(GraphView._setHoveredPort(view, ("/A", "x", True)))
        self.assertEqual(view._hoveredPort, ("/A", "x", True))
        self.assertFalse(GraphView._setHoveredPort(view, ("/A", "x", True)))


class TestRowHitMetrics(unittest.TestCase):
    def test_layout_is_the_single_source_of_truth(self):
        node = _Node(port_start_y=30.0, port_line_height=20.0)
        self.assertEqual(
            GraphView._getRowHitMetrics(None, node, _Renderer()), (30.0, 20.0)
        )

    def test_missing_layout_degrades_to_port_width(self):
        node = _Node(port_line_height=0.0)
        start, height = GraphView._getRowHitMetrics(None, node, _Renderer(12.0))
        self.assertEqual(start, 30.0)
        self.assertEqual(height, 24.0)


class TestRowEdgePinCandidates(unittest.TestCase):
    def test_input_side_lists_input_pins(self):
        node = _Node(inputPins=["a", "b"], outputPins=["x"])
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, node, False),
            [("a", 0), ("b", 1)],
        )

    def test_output_side_lists_output_pins(self):
        node = _Node(inputPins=["a"], outputPins=["x", "y"])
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, node, True),
            [("x", 0), ("y", 1)],
        )

    def test_unfolded_header_kind_is_skipped(self):
        node = _Node(
            outputPins=["grp", "grp:child"],
            output_slots=[0, 1],
            output_kinds=[2, 3],
        )
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, node, True), [("grp:child", 1)]
        )

    def test_dual_pin_mirrors_without_double_count(self):
        lazy = _Node(inputPins=["dual"], outputPins=[], dual={"dual"})
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, lazy, True), [("dual", 0)]
        )
        descriptor = _Node(inputPins=["rig1:space"], outputPins=["rig1:space"])
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, descriptor, True),
            [("rig1:space", 0)],
        )
        plain = _Node(inputPins=["a"], outputPins=["x"], dual=set())
        self.assertEqual(
            GraphView._rowEdgePinCandidates(None, plain, True), [("x", 0)]
        )


class TestPinAtRowEdge(unittest.TestCase):
    def _view(self, positions):
        view = _HitView()
        view.defaultRenderer = _Renderer()
        view._getPortPosition = lambda _n, pin, out: positions.get((pin, out))
        return view

    def _node(self):
        return _Node(inputPins=["a", "b"], outputPins=["x"])

    def test_bands_match_the_cached_layout(self):
        view = self._view(
            {("a", False): (0.0, 40.0), ("b", False): (0.0, 60.0), ("x", True): (100.0, 40.0)}
        )
        node = self._node()
        self.assertEqual(
            GraphView._pinAtRowEdge(view, node, (5.0, 40.0), False), "a"
        )
        self.assertEqual(
            GraphView._pinAtRowEdge(view, node, (5.0, 60.0), False), "b"
        )
        self.assertEqual(
            GraphView._pinAtRowEdge(view, node, (95.0, 40.0), True), "x"
        )
        self.assertIsNone(GraphView._pinAtRowEdge(view, node, (5.0, 150.0), False))

    def test_missing_port_position_falls_back_to_band_center(self):
        view = self._view({})
        node = self._node()
        self.assertEqual(
            GraphView._pinAtRowEdge(view, node, (5.0, 55.0), False), "b"
        )

    def test_no_renderer_returns_none(self):
        view = self._view({})
        view.defaultRenderer = None
        self.assertIsNone(
            GraphView._pinAtRowEdge(view, self._node(), (5.0, 40.0), False)
        )


class TestFindRowEdgeDragStart(unittest.TestCase):
    def _view(self):
        view = _HitView()
        view.defaultRenderer = _Renderer()
        view._getPortPosition = lambda _n, pin, out: {
            ("a", False): (0.0, 40.0),
            ("b", False): (0.0, 60.0),
            ("x", True): (100.0, 40.0),
        }.get((pin, out))
        return view

    def _node(self, **overrides):
        return _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(100.0, 200.0),
            inputPins=["a", "b"],
            outputPins=["x"],
            **overrides,
        )

    def test_left_edge_starts_input_drag(self):
        self.assertEqual(
            GraphView._findRowEdgeDragStart(self._view(), self._node(), (5.0, 40.0)),
            ("a", False),
        )

    def test_right_edge_starts_output_drag(self):
        self.assertEqual(
            GraphView._findRowEdgeDragStart(self._view(), self._node(), (95.0, 40.0)),
            ("x", True),
        )

    def test_middle_of_row_is_a_node_drag(self):
        self.assertIsNone(
            GraphView._findRowEdgeDragStart(self._view(), self._node(), (50.0, 40.0))
        )

    def test_collapsed_node_starts_no_drag(self):
        self.assertIsNone(
            GraphView._findRowEdgeDragStart(
                self._view(), self._node(title_collapsed=True), (5.0, 40.0)
            )
        )


class TestRelationshipDragRowHover(unittest.TestCase):
    def _view(self, nodes):
        view = _HitView()
        view._draggingLink = True
        view._dragLinkTempLink = SimpleNamespace(is_relationship_link=True)
        view.nodes = {node.id: node for node in nodes}
        view.nodeIdUnderCursor = nodes[0].id
        view._spatialIndex = _SpatialIndex(view.nodes)
        view.defaultRenderer = _Renderer()
        view._getPortPosition = lambda _n, _pin, _out: None
        return view

    def _node(self):
        node = _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(200.0, 200.0),
            outputPins=["rel", "attr"],
            port_start_y=30.0,
            port_line_height=20.0,
            renderer=_Renderer(),
        )
        node.is_relationship_pin = lambda pin, _out: pin == "rel"
        return node

    def test_relationship_row_is_a_drop_target(self):
        view = self._view([self._node()])
        self.assertEqual(
            GraphView._relationshipDragRowHover(view, (100.0, 40.0), 16.0),
            ("/A", "rel", True),
        )

    def test_property_row_is_a_drop_target(self):
        view = self._view([self._node()])
        self.assertEqual(
            GraphView._relationshipDragRowHover(view, (100.0, 60.0), 16.0),
            ("/A", "attr", True),
        )

    def test_inactive_without_a_relationship_drag(self):
        view = self._view([self._node()])
        view._draggingLink = False
        self.assertIsNone(GraphView._relationshipDragRowHover(view, (100.0, 40.0), 16.0))
        view._draggingLink = True
        view._dragLinkTempLink = SimpleNamespace(is_relationship_link=False)
        self.assertIsNone(GraphView._relationshipDragRowHover(view, (100.0, 40.0), 16.0))

    def test_property_hover_prefers_the_cursor_half(self):
        node = _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(200.0, 200.0),
            inputPins=["p"],
            outputPins=["q"],
            port_start_y=30.0,
            port_line_height=20.0,
        )
        node.is_relationship_pin = lambda _pin, _out: False
        view = self._view([node])
        self.assertEqual(
            GraphView._findPropertyRowHover(view, node, "/A", (150.0, 40.0)),
            ("/A", "q", True),
        )
        self.assertEqual(
            GraphView._findPropertyRowHover(view, node, "/A", (50.0, 40.0)),
            ("/A", "p", False),
        )


class TestGetPropertyAtPoint(unittest.TestCase):
    def _view(self, nodes):
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.defaultRenderer = _Renderer()
        view.nodes = {node.id: node for node in nodes}
        return view

    def test_row_band_matches_cached_centers(self):
        node = _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(100.0, 100.0),
            inputPins=["in"],
            port_start_y=10.0,
            port_line_height=20.0,
            input_centers=[15.0],
        )
        view = self._view([node])
        self.assertEqual(
            GraphView._getPropertyAtPoint(view, QtCore.QPoint(50, 15)), ("/A", "in")
        )
        self.assertIsNone(GraphView._getPropertyAtPoint(view, QtCore.QPoint(50, 60)))

    def test_collapsed_node_is_skipped(self):
        node = _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(100.0, 100.0),
            inputPins=["in"],
            title_collapsed=True,
        )
        view = self._view([node])
        self.assertIsNone(GraphView._getPropertyAtPoint(view, QtCore.QPoint(50, 40)))


class TestArrowheadUnderCursor(unittest.TestCase):
    def _view(self, links):
        view = _HitView()
        view.zoom = 1.0
        view.links = links
        return view

    def test_nearest_arrowhead_wins(self):
        links = [
            _make_link(is_relationship_link=True, end=(100.0, 0.0)),
            _make_link(is_relationship_link=True, end=(104.0, 0.0)),
        ]
        view = self._view(links)
        self.assertEqual(GraphView._findArrowheadUnderCursor(view, (103.0, 0.0)), 1)

    def test_dangling_and_plain_links_are_skipped(self):
        links = [
            _make_link(is_relationship_link=True, isDangling=True, end=(50.0, 0.0)),
            _make_link(is_relationship_link=False, end=(50.0, 0.0)),
        ]
        view = self._view(links)
        self.assertEqual(GraphView._findArrowheadUnderCursor(view, (50.0, 0.0)), -1)

    def test_miss_returns_negative(self):
        view = self._view([_make_link(is_relationship_link=True, end=(100.0, 0.0))])
        self.assertEqual(GraphView._findArrowheadUnderCursor(view, (0.0, 0.0)), -1)


class TestDanglingIndicatorUnderCursor(unittest.TestCase):
    def test_hit_returns_link_index(self):
        view = _HitView()
        view.links = [
            _make_link(end=(0.0, 0.0)),
            _make_link(isDangling=True, end=(100.0, 50.0)),
        ]
        self.assertEqual(
            GraphView._findDanglingIndicatorUnderCursor(view, (103.0, 52.0)), 1
        )

    def test_miss_returns_negative(self):
        view = _HitView()
        view.links = [_make_link(isDangling=True, end=(100.0, 50.0))]
        self.assertEqual(
            GraphView._findDanglingIndicatorUnderCursor(view, (0.0, 0.0)), -1
        )
        view.links = [_make_link(end=(100.0, 50.0))]
        self.assertEqual(
            GraphView._findDanglingIndicatorUnderCursor(view, (100.0, 50.0)), -1
        )


class TestFindDanglingLinkAtPort(unittest.TestCase):
    def _view(self, links):
        view = _HitView()
        view.links = links
        view.nodes = {"/A": _Node("/A")}
        view.defaultRenderer = _Renderer()
        view._cachedPortHitRadiusMultiplier = 2.0
        view._getPortPosition = lambda _n, _pin, _out: (101.0, 51.0)
        return view

    def test_hit_returns_dangling_index(self):
        view = self._view(
            [
                _make_link(),
                _make_link(
                    isDangling=True,
                    sourceNodeId="/A",
                    sourcePort="out",
                    targetNodeId="/Missing",
                    targetPort="in",
                ),
            ]
        )
        self.assertEqual(
            GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(101.0, 51.0)), 1
        )

    def test_miss_returns_negative(self):
        view = self._view(
            [_make_link(isDangling=True, sourceNodeId="/A", sourcePort="out")]
        )
        self.assertEqual(
            GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(200.0, 200.0)), -1
        )


class TestHoverLeavesModelUntouched(unittest.TestCase):
    def _world(self):
        nodes = [_Node("/A"), _Node("/B", position=(200.0, 0.0))]
        links = [
            _make_link(end=(100.0, 10.0)),
            _make_link(is_relationship_link=True, end=(104.0, 10.0)),
            _make_link(
                isDangling=True,
                sourceNodeId="/A",
                sourcePort="out",
                targetNodeId="/Missing",
                targetPort="in",
                end=(100.0, 50.0),
            ),
        ]
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.nodes = {node.id: node for node in nodes}
        view.links = links
        view._spatialIndex = _SpatialIndex(view.nodes)
        view.defaultRenderer = _Renderer()
        view._cachedPortHitRadiusMultiplier = 2.0
        view._nodeRenderManager = SimpleNamespace(portAtPoint=MagicMock(return_value=None))
        view.nodeGraph = object()
        view._hoveredPort = None
        view._draggingLink = False
        view._dragLinkTempLink = None
        view._getPortPosition = lambda _n, _pin, _out: (101.0, 51.0)
        view._profiler = SimpleNamespace(recordTime=MagicMock())
        view.nodeIdUnderCursor = ""
        return view, nodes, links

    def _hit_battery(self, view):
        GraphView._getNodeIdAtWorldPos(view, Gf.Vec2d(10.0, 10.0))
        GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))
        GraphView._updateNodeUnderCursor(view, QtCore.QPoint(10, 10))
        GraphView._findArrowheadUnderCursor(view, (103.0, 10.0))
        GraphView._findDanglingIndicatorUnderCursor(view, (103.0, 52.0))
        GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(101.0, 51.0))
        node = view.nodes["/A"]
        GraphView._getRowHitMetrics(view, node, view.defaultRenderer)
        GraphView._rowEdgePinCandidates(view, node, True)
        GraphView._pinAtRowEdge(view, node, (95.0, 40.0), True)
        GraphView._findRowEdgeDragStart(view, node, (95.0, 40.0))
        GraphView._getPropertyAtPoint(view, QtCore.QPoint(50, 15))

    def test_link_fields_are_unchanged(self):
        view, _nodes, links = self._world()
        before = [copy.deepcopy(vars(link)) for link in links]
        self._hit_battery(view)
        for link, snapshot in zip(links, before):
            self.assertEqual(vars(link), snapshot)

    def test_position_versions_are_unchanged(self):
        view, nodes, _links = self._world()
        before = [(tuple(n.position), tuple(n.size), n.positionVersion) for n in nodes]
        self._hit_battery(view)
        after = [(tuple(n.position), tuple(n.size), n.positionVersion) for n in nodes]
        self.assertEqual(after, before)

    def test_hover_update_never_sets_link_hovered(self):
        view, _nodes, links = self._world()
        view._nodeRenderManager.portAtPoint = MagicMock(
            return_value={"found": True, "nodeId": "/A", "portName": "x", "isOutput": True}
        )
        GraphView._updateHoveredPort(view, QtCore.QPoint(95, 40))
        self.assertEqual(view._hoveredPort, ("/A", "x", True))
        self.assertEqual([link.hovered for link in links], [False, False, False])


class TestUpdateNodeUnderCursor(unittest.TestCase):
    def test_cursor_node_matches_point_query(self):
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.nodes = {"/A": _Node("/A")}
        view._spatialIndex = _SpatialIndex(view.nodes)
        view._profiler = SimpleNamespace(recordTime=MagicMock())
        view.nodeIdUnderCursor = ""
        GraphView._updateNodeUnderCursor(view, QtCore.QPoint(10, 10))
        self.assertEqual(view.nodeIdUnderCursor, "/A")
        view._profiler.recordTime.assert_called_once()
        GraphView._updateNodeUnderCursor(view, QtCore.QPoint(500, 500))
        self.assertEqual(view.nodeIdUnderCursor, "")


class TestViewportJumps(unittest.TestCase):
    def _view(self):
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.width = lambda: 800
        view.height = lambda: 600
        view.minimap = SimpleNamespace(
            getGraphBounds=lambda: SimpleNamespace(IsEmpty=lambda: True)
        )
        view.update = MagicMock()
        return view

    def test_zoom_clamps_to_node_scale(self):
        view = self._view()
        GraphView._zoomAroundPoint(view, 5.0, (400.0, 300.0))
        # A typical node fills a fifth of an 800x600 screen at 1.2x.
        self.assertAlmostEqual(view.zoom, 1.2)
        self.assertAlmostEqual(view.panX, 400.0 * (1.0 - 1.0 / 1.2))
        self.assertAlmostEqual(view.panY, 300.0 * (1.0 - 1.0 / 1.2))

    def test_set_zoom_delegates_through_center(self):
        view = self._view()
        calls = []
        view._zoomAroundPoint = lambda new_zoom, focus: calls.append((new_zoom, focus))
        GraphView._setZoom(view, 2.0)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], 2.0)
        self.assertEqual(tuple(calls[0][1]), (400.0, 300.0))
        view.update.assert_called_once_with()

    def test_center_on_node_immediate(self):
        view = self._view()
        view._panAnimating = True
        node = _Node(position=(100.0, 100.0), size=(200.0, 100.0))
        GraphView._centerOnNode(view, node, animate=False)
        self.assertAlmostEqual(view.panX, 200.0 - 400.0)
        self.assertAlmostEqual(view.panY, 150.0 - 300.0)
        self.assertFalse(view._panAnimating)
        view.update.assert_called_once_with()

    def test_center_on_node_animated_arms_targets(self):
        view = self._view()
        view._cachedPanAnimationSpeed = 3.0
        view._panXAnim = SimpleNamespace(current=0.0, target=0.0, speed=0.0)
        view._panYAnim = SimpleNamespace(current=0.0, target=0.0, speed=0.0)
        view._panAnimating = False
        GraphView._centerOnNode(view, _Node(position=(100.0, 100.0), size=(200.0, 100.0)))
        self.assertTrue(view._panAnimating)
        self.assertAlmostEqual(view._panXAnim.target, 200.0 - 400.0)
        self.assertAlmostEqual(view._panYAnim.target, 150.0 - 300.0)
        self.assertEqual(view.panX, 0.0)
        self.assertEqual(view.panY, 0.0)


class TestNavigateAlongRelationshipLink(unittest.TestCase):
    def _view(self, nodes):
        view = _HitView()
        view.nodes = {node.id: node for node in nodes}
        view._jumpToSourceNodeInternal = MagicMock()
        view._jumpToTargetNodeInternal = MagicMock()
        return view

    def test_jumps_to_the_farthest_end(self):
        view = self._view([_Node("/A"), _Node("/B")])
        link = _make_link(start=(0.0, 0.0), end=(100.0, 0.0))
        self.assertTrue(GraphView._navigateAlongRelationshipLink(view, link, (10.0, 0.0)))
        view._jumpToTargetNodeInternal.assert_called_once_with(link, "/B")
        view._jumpToSourceNodeInternal.assert_not_called()

    def test_falls_back_to_the_reachable_end(self):
        view = self._view([_Node("/A")])
        link = _make_link(start=(0.0, 0.0), end=(100.0, 0.0))
        self.assertTrue(GraphView._navigateAlongRelationshipLink(view, link, (10.0, 0.0)))
        view._jumpToSourceNodeInternal.assert_called_once_with(link, "/A")

    def test_no_reachable_end_reports_false(self):
        view = self._view([])
        link = _make_link(start=(0.0, 0.0), end=(100.0, 0.0))
        self.assertFalse(GraphView._navigateAlongRelationshipLink(view, link, (10.0, 0.0)))


class TestNavigateAndJump(unittest.TestCase):
    def _view(self):
        view = _HitView()
        view._getFirstSelectedLink = MagicMock(return_value=None)
        view._getFirstSelectedNode = MagicMock(return_value=None)
        view._jumpToSourceNode = MagicMock()
        view._jumpToTargetNode = MagicMock()
        view._selectFirstLinkInDirection = MagicMock()
        return view

    def test_backward_from_link_jumps_to_source(self):
        view = self._view()
        link = _make_link()
        view._getFirstSelectedLink = MagicMock(return_value=link)
        GraphView.navigateBackward(view)
        view._jumpToSourceNode.assert_called_once_with(link)
        view._selectFirstLinkInDirection.assert_not_called()

    def test_forward_from_node_selects_output(self):
        view = self._view()
        node = _Node("/A")
        view._getFirstSelectedNode = MagicMock(return_value=node)
        GraphView.navigateForward(view)
        view._selectFirstLinkInDirection.assert_called_once_with(node, "output")

    def test_jump_to_source_completes(self):
        view = _HitView()
        view._showHotkeyPopup = MagicMock()
        view._showPopupMessage = MagicMock()
        view.nodes = {"/A": _Node("/A")}
        view.links = []
        view._lastDanglingLinkIndex = -1
        view._danglingLinkFollowCount = 0
        view._addNodeFromDanglingLink = MagicMock()
        view._jumpToSourceNodeInternal = MagicMock()
        link = _make_link(isDangling=False, data_sourceNodeId="/A")
        GraphView._jumpToSourceNode(view, link)
        self.assertEqual(view._lastJumpDirection, "source")
        view._jumpToSourceNodeInternal.assert_called_once_with(link, "/A")

    def test_dangling_follow_asks_then_adds(self):
        view = _HitView()
        view._showHotkeyPopup = MagicMock()
        view._showPopupMessage = MagicMock()
        view.nodes = {}
        link = _make_link(isDangling=True, data_sourceNodeId="/A")
        view.links = [link]
        view._lastDanglingLinkIndex = -1
        view._danglingLinkFollowCount = 0
        view._addNodeFromDanglingLink = MagicMock(return_value=True)
        view._jumpToSourceNodeInternal = MagicMock()
        GraphView._jumpToSourceNode(view, link)
        view._showPopupMessage.assert_called_with("Follow again to add node")
        view._jumpToSourceNodeInternal.assert_not_called()
        view._addNodeFromDanglingLink.side_effect = (
            lambda _link: view.nodes.update({"/A": _Node("/A")}) or True
        )
        GraphView._jumpToSourceNode(view, link)
        view._jumpToSourceNodeInternal.assert_called_once_with(link, "/A")

    def test_jump_internal_reselects_and_recenters(self):
        view = _HitView()
        node = _Node("/A")
        view.nodes = {"/A": node}
        link = _make_link()
        view.links = [link]
        view.clearSelection = MagicMock()
        view.clearLinkSelection = MagicMock()
        view._selectedNodes = set()
        view._centerOnNode = MagicMock()
        view._syncSelectionToPrimtree = MagicMock()
        view._lastDanglingLinkIndex = 3
        view._danglingLinkFollowCount = 2
        GraphView._jumpToSourceNodeInternal(view, link, "/A")
        self.assertEqual(view._lastOutputLinkIndex, 0)
        self.assertEqual(view._selectedNodes, {"/A"})
        self.assertTrue(node.selected)
        self.assertIs(view._lastSelectedNode, node)
        self.assertEqual(view._lastDanglingLinkIndex, -1)
        self.assertEqual(view._danglingLinkFollowCount, 0)
        view._centerOnNode.assert_called_once_with(node)
        view._syncSelectionToPrimtree.assert_called_once_with()


class TestTooltipThroughLegacyLookup(unittest.TestCase):
    def test_clear_reaches_the_patched_tooltip(self):
        view = _HitView()
        view._tooltipTimer = MagicMock()
        view._tooltipProperty = ("/A", "in")
        with patch("UsdNoodles.graphView.QtWidgets") as mock_qt:
            GraphView._clearPropertyTooltip(view)
            mock_qt.QToolTip.hideText.assert_called_once_with()
        view._tooltipTimer.stop.assert_called_once_with()
        self.assertIsNone(view._tooltipProperty)

    def test_update_and_show_use_current_bindings(self):
        view = _HitView()
        view._tooltipTimer = MagicMock()
        view._tooltipProperty = None
        view._tooltipScreenPos = None
        view._getPropertyAtPoint = lambda _pos: ("/A", "in")
        view.mapToGlobal = lambda pos: ("global", pos)
        with patch("UsdNoodles.graphView.QtWidgets") as mock_qt:
            GraphView._updatePropertyTooltip(view, QtCore.QPoint(10, 20))
            mock_qt.QToolTip.hideText.assert_called_once_with()
        view._tooltipTimer.start.assert_called_once_with()
        self.assertEqual(view._tooltipProperty, ("/A", "in"))
        node = SimpleNamespace(get_pin_documentation=lambda pin: f"doc-for-{pin}")
        view.nodes = {"/A": node}
        with patch("UsdNoodles.graphView.QtWidgets") as mock_qt:
            GraphView._showPropertyTooltip(view)
            mock_qt.QToolTip.showText.assert_called_once_with(
                ("global", QtCore.QPoint(10, 20)), "doc-for-in", view
            )


class TestValueHoverAndCell(unittest.TestCase):
    def _view(self):
        view = _HitView()
        view.zoom = 1.0
        view.panX = 0.0
        view.panY = 0.0
        view.draggingNodes = False
        view._draggingLink = False
        view.spacePressed = False
        view._valueHover = None
        view._valueCursorSet = None
        view._VALUE_CURSORS = {"float": QtCore.Qt.SizeHorCursor}
        view.update = MagicMock()
        view.setCursor = MagicMock()
        return view

    def test_hover_tracks_cell_and_cursor(self):
        view = self._view()
        row = SimpleNamespace(kind="float", editable=True)
        view._valueCellAtPoint = lambda _world: SimpleNamespace(
            nodeId="/A", propertyName="p", comp=0, row=row
        )
        GraphView._updateValueHoverCursor(view, QtCore.QPoint(50, 15))
        self.assertEqual(view._valueHover, ("/A", "p", 0))
        self.assertEqual(view._valueCursorSet, QtCore.Qt.SizeHorCursor)
        view.update.assert_called_once_with()
        view.setCursor.assert_called_once_with(QtCore.Qt.SizeHorCursor)

    def test_drag_owns_the_pointer(self):
        view = self._view()
        view.draggingNodes = True
        view._valueCellAtPoint = MagicMock()
        GraphView._updateValueHoverCursor(view, QtCore.QPoint(50, 15))
        view._valueCellAtPoint.assert_not_called()
        self.assertIsNone(view._valueHover)
        view.update.assert_not_called()
        view.setCursor.assert_not_called()

    def test_cell_hidden_when_values_are_off(self):
        view = _HitView()
        view._cachedShowAttributeValues = False
        self.assertIsNone(GraphView._valueCellAtPoint(view, (5.0, 5.0)))

    def test_cell_resolves_to_nearest_component(self):
        view = _HitView()
        view._cachedShowAttributeValues = True
        view._asWorldXY = lambda point: (float(point[0]), float(point[1]))
        view.nodeIdUnderCursor = "/A"
        node = _Node(position=(0.0, 0.0), size=(400.0, 100.0))
        node.layoutPortLineHeight = 20.0
        view.nodes = {"/A": node}
        view._getNodeIdAtWorldPos = MagicMock(return_value="/A")
        view.zoom = 1.0
        view._cachedValueMinPixelHeight = 4.0
        row = SimpleNamespace(property_name="p")
        rect = valueCell.Rect(300.0, 30.0, 380.0, 50.0)
        subs = [
            valueCell.Rect(300.0, 30.0, 338.0, 50.0),
            valueCell.Rect(342.0, 30.0, 380.0, 50.0),
        ]
        view._valueRowGeometry = MagicMock(return_value=[(row, rect, subs, 20.0)])
        hit = GraphView._valueCellAtPoint(view, (350.0, 40.0))
        self.assertEqual(hit.nodeId, "/A")
        self.assertEqual(hit.primPath, "/A")
        self.assertEqual(hit.propertyName, "p")
        self.assertEqual(hit.comp, 1)
        self.assertEqual(hit.rect, subs[1])
        self.assertIs(hit.row, row)


if __name__ == "__main__":
    unittest.main()
