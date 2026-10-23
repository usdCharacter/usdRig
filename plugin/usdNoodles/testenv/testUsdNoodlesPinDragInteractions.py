#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Production pin hit-testing, cached layout, and renderer state handoff.

View and node doubles provide geometry without constructing a GL widget.
"""

import collections
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from UsdNoodles.graphView import GraphView


# Lightweight fakes (plain objects, NOT MagicMock).
#
# MagicMock is unsafe here: getattr(mock, "_title_collapsed", False) returns a
# truthy Mock rather than the default, which would silently short-circuit the
# row-edge logic. Plain objects give honest attribute defaults.


class _Renderer:
    def __init__(self, port_width=8.0):
        self._port_width = port_width

    def getPortWidth(self):
        return self._port_width


class _Node:
    def __init__(
        self,
        node_id="/N",
        position=(0.0, 0.0),
        size=(100.0, 200.0),
        inputPins=None,
        outputPins=None,
        input_slots=None,
        output_slots=None,
        input_kinds=None,
        output_kinds=None,
        dual=None,
        title_collapsed=False,
        renderer=None,
    ):
        self.id = node_id
        self.position = position
        self.size = size
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
        self._title_collapsed = title_collapsed
        self.renderer = renderer

    inputRowSlots = property(lambda self: self._input_row_slots)
    outputRowSlots = property(lambda self: self._output_row_slots)
    inputRowKinds = property(lambda self: self._input_row_kinds)
    outputRowKinds = property(lambda self: self._output_row_kinds)


# namedtuple (immutable) keeps FLAKE8 B903 quiet for this pure data holder.
_Link = collections.namedtuple(
    "_Link",
    ["end", "is_relationship_link", "isDangling"],
    defaults=(False, False),
)


class _View:
    def __init__(
        self,
        port_start_y=30.0,
        port_line_height=20.0,
        port_positions=None,
        renderer=None,
        zoom=1.0,
        links=None,
    ):
        self._port_start_y = port_start_y
        self._port_line_height = port_line_height
        self._port_positions = dict(port_positions or {})
        self.defaultRenderer = renderer if renderer is not None else _Renderer()
        self.zoom = zoom
        self.links = list(links or [])
        # Drag state consumed by the REQ 3 visibility gate.
        self._draggingLink = False
        self._dragLinkSourceNode = None
        self._dragLinkSourcePort = None
        self._dragLinkSourceIsOutput = None

    _rowEdgePinCandidates = GraphView._rowEdgePinCandidates
    _pinAtRowEdge = GraphView._pinAtRowEdge

    # Unchanged production helpers, stubbed deterministically.
    def _getRowHitMetrics(self, node, renderer):
        return self._port_start_y, self._port_line_height

    def _getPortPosition(self, node, pinName, isOutput):
        return self._port_positions.get((pinName, isOutput))


def _row_edge_pin_candidates(node, is_output):
    return GraphView._rowEdgePinCandidates(None, node, is_output)


_pin_at_row_edge = GraphView._pinAtRowEdge
_find_row_edge_drag_start = GraphView._findRowEdgeDragStart
_find_arrowhead_under_cursor = GraphView._findArrowheadUnderCursor


class TestPortHighlightHandoff(unittest.TestCase):
    def _view(self):
        return SimpleNamespace(
            nodeGraph=object(),
            _renderConfig=lambda: SimpleNamespace(nodePortWidth=10),
            _cachedPortRingThickness=1,
            _cachedPortConnectedFillColor=(1, 1, 1, 1),
            _cachedPortConnectedRingColor=(1, 1, 1, 1),
            _cachedPortDisconnectedFillColor=(0, 0, 0, 1),
            _cachedPortDisconnectedRingColor=(0, 0, 0, 1),
            _cachedPortHoverColor=(1, 1, 0, 1),
            _worldSpaceProjectionMatrix=lambda: SimpleNamespace(data=lambda: [0.0] * 16),
            _hoveredPort=None,
            _portHighlightsDirty=False,
            textChanged=False,
            draggingNodes=False,
            _draggingLink=False,
            _dragLinkSourceNode=None,
            _dragLinkSourcePort=None,
            _dragLinkSourceIsOutput=False,
            _nodeRenderManager=MagicMock(),
        )

    def test_drag_endpoint_reaches_the_authoritative_renderer(self):
        view = self._view()
        view._draggingLink = True
        view._dragLinkSourceNode = "/A"
        view._dragLinkSourcePort = "in"
        GraphView._renderPortHighlightsCpp(view)
        args = view._nodeRenderManager.renderPortHighlightsFromGraph.call_args.args
        self.assertEqual(args[8:12], (True, "/A", "in", False))
        self.assertTrue(args[12])

    def test_idle_frame_keeps_cached_geometry(self):
        view = self._view()
        GraphView._renderPortHighlightsCpp(view)
        args = view._nodeRenderManager.renderPortHighlightsFromGraph.call_args.args
        self.assertFalse(args[8])
        self.assertFalse(args[12])


# REQ 4 - Row-edge drag start


class TestRowEdgePinCandidates(unittest.TestCase):
    """Candidate pins per side, including dual mirrors and header skips."""

    def test_input_side_lists_input_pins(self):
        node = _Node(inputPins=["a", "b"], outputPins=["x"])
        self.assertEqual(_row_edge_pin_candidates(node, False), [("a", 0), ("b", 1)])

    def test_output_side_lists_output_pins(self):
        node = _Node(inputPins=["a"], outputPins=["x", "y"])
        self.assertEqual(_row_edge_pin_candidates(node, True), [("x", 0), ("y", 1)])

    def test_unfolded_header_kind2_is_skipped(self):
        # "grp" is an unfolded header (kind 2); only the child is a candidate.
        node = _Node(
            outputPins=["grp", "grp:child"],
            output_slots=[0, 1],
            output_kinds=[2, 3],
        )
        self.assertEqual(_row_edge_pin_candidates(node, True), [("grp:child", 1)])

    def test_lazy_dual_pin_mirrors_to_output_side(self):
        # Lazy path: dual pin lives only in inputPins, mirrored to output.
        node = _Node(inputPins=["dual"], outputPins=[], dual={"dual"})
        self.assertEqual(_row_edge_pin_candidates(node, True), [("dual", 0)])

    def test_descriptor_dual_not_double_counted(self):
        # Descriptor path: pin already in both lists; dual set empty.
        node = _Node(inputPins=["rig1:space"], outputPins=["rig1:space"])
        self.assertEqual(_row_edge_pin_candidates(node, True), [("rig1:space", 0)])

    def test_non_dual_input_not_mirrored_to_output(self):
        node = _Node(inputPins=["a"], outputPins=["x"], dual=set())
        self.assertEqual(_row_edge_pin_candidates(node, True), [("x", 0)])


class TestPinAtRowEdge(unittest.TestCase):
    """Resolve a press to the pin whose row band contains its Y."""

    def _view(self):
        return _View(
            port_start_y=30.0,
            port_line_height=20.0,
            port_positions={
                ("a", False): (0.0, 40.0),
                ("b", False): (0.0, 60.0),
                ("x", True): (100.0, 40.0),
            },
        )

    def _node(self):
        return _Node(inputPins=["a", "b"], outputPins=["x"])

    def test_first_input_row(self):
        self.assertEqual(
            _pin_at_row_edge(self._view(), self._node(), (5.0, 40.0), False), "a"
        )

    def test_second_input_row(self):
        self.assertEqual(
            _pin_at_row_edge(self._view(), self._node(), (5.0, 60.0), False), "b"
        )

    def test_output_row(self):
        self.assertEqual(
            _pin_at_row_edge(self._view(), self._node(), (95.0, 40.0), True), "x"
        )

    def test_y_outside_any_row_returns_none(self):
        self.assertIsNone(
            _pin_at_row_edge(self._view(), self._node(), (5.0, 150.0), False)
        )

    def test_missing_port_position_falls_back_to_band_center(self):
        # No port_positions -> row_center falls back to row band center; the
        # press still resolves to the row whose band contains it.
        view = _View(port_start_y=30.0, port_line_height=20.0, port_positions={})
        node = _Node(inputPins=["a", "b"])
        self.assertEqual(_pin_at_row_edge(view, node, (5.0, 55.0), False), "b")

    def test_no_renderer_returns_none(self):
        view = _View()
        view.defaultRenderer = None
        self.assertIsNone(_pin_at_row_edge(view, self._node(), (5.0, 40.0), False))


class TestFindRowEdgeDragStart(unittest.TestCase):
    """End-to-end row-edge drag start: press -> (pin, is_output) or None."""

    def _view(self):
        return _View(
            port_start_y=30.0,
            port_line_height=20.0,
            port_positions={
                ("a", False): (0.0, 40.0),
                ("b", False): (0.0, 60.0),
                ("x", True): (100.0, 40.0),
            },
        )

    def _node(self):
        # position (0,0), size (100,200); 10% edge band = 10px on each side.
        return _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(100.0, 200.0),
            inputPins=["a", "b"],
            outputPins=["x"],
        )

    def test_left_edge_starts_input_drag(self):
        self.assertEqual(
            _find_row_edge_drag_start(self._view(), self._node(), (5.0, 40.0)),
            ("a", False),
        )

    def test_right_edge_starts_output_drag(self):
        self.assertEqual(
            _find_row_edge_drag_start(self._view(), self._node(), (95.0, 40.0)),
            ("x", True),
        )

    def test_middle_falls_through_to_node_drag(self):
        # The middle 80% of the row is a node-body drag handle, not a
        # connect-zone, so a press there returns None and the node moves.
        self.assertIsNone(
            _find_row_edge_drag_start(self._view(), self._node(), (50.0, 40.0))
        )

    def test_edge_band_is_outer_tenth(self):
        # Band = 10% of the 100px-wide node = 10px on each side, boundaries
        # inclusive. Just inside the band connects; one pixel past it falls
        # through to a node-body drag.
        view, node = self._view(), self._node()
        self.assertEqual(
            _find_row_edge_drag_start(view, node, (10.0, 40.0)), ("a", False)
        )
        self.assertIsNone(_find_row_edge_drag_start(view, node, (11.0, 40.0)))
        self.assertEqual(
            _find_row_edge_drag_start(view, node, (90.0, 40.0)), ("x", True)
        )
        self.assertIsNone(_find_row_edge_drag_start(view, node, (89.0, 40.0)))

    def test_press_left_of_node_returns_none(self):
        self.assertIsNone(
            _find_row_edge_drag_start(self._view(), self._node(), (-5.0, 40.0))
        )

    def test_press_right_of_node_returns_none(self):
        self.assertIsNone(
            _find_row_edge_drag_start(self._view(), self._node(), (105.0, 40.0))
        )

    def test_press_below_all_rows_returns_none(self):
        # Y is below every row band -> no pin on either side -> None, so the
        # press falls through to node selection/drag.
        self.assertIsNone(
            _find_row_edge_drag_start(self._view(), self._node(), (5.0, 150.0))
        )

    def test_title_collapsed_node_returns_none(self):
        node = self._node()
        node._title_collapsed = True
        self.assertIsNone(_find_row_edge_drag_start(self._view(), node, (5.0, 40.0)))

    def test_no_renderer_returns_none(self):
        view = self._view()
        view.defaultRenderer = None
        self.assertIsNone(_find_row_edge_drag_start(view, self._node(), (5.0, 40.0)))

    def test_dual_row_both_edges_resolve(self):
        view = _View(
            port_start_y=30.0,
            port_line_height=20.0,
            port_positions={
                ("dual", False): (0.0, 40.0),
                ("dual", True): (100.0, 40.0),
            },
        )
        node = _Node(
            node_id="/A",
            position=(0.0, 0.0),
            size=(100.0, 200.0),
            inputPins=["dual"],
            outputPins=[],
            dual={"dual"},
        )
        self.assertEqual(
            _find_row_edge_drag_start(view, node, (5.0, 40.0)), ("dual", False)
        )
        self.assertEqual(
            _find_row_edge_drag_start(view, node, (95.0, 40.0)), ("dual", True)
        )


# REQ 5 - Arrowhead hit-test


class TestFindArrowheadUnderCursor(unittest.TestCase):
    """Grab the arrowhead at a relationship link's target end."""

    def test_hit_within_radius(self):
        view = _View(
            zoom=1.0,
            links=[_Link((100.0, 50.0), is_relationship_link=True)],
        )
        # radius = max(10, 8) = 10; (105,50) is 5 away -> hit.
        self.assertEqual(_find_arrowhead_under_cursor(view, (105.0, 50.0)), 0)

    def test_miss_outside_radius(self):
        view = _View(
            zoom=1.0,
            links=[_Link((100.0, 50.0), is_relationship_link=True)],
        )
        # (115,50) is 15 away > radius 10 -> miss.
        self.assertEqual(_find_arrowhead_under_cursor(view, (115.0, 50.0)), -1)

    def test_non_relationship_link_ignored(self):
        view = _View(
            zoom=1.0,
            links=[_Link((100.0, 50.0), is_relationship_link=False)],
        )
        self.assertEqual(_find_arrowhead_under_cursor(view, (100.0, 50.0)), -1)

    def test_dangling_relationship_link_ignored(self):
        view = _View(
            zoom=1.0,
            links=[_Link((100.0, 50.0), is_relationship_link=True, isDangling=True)],
        )
        self.assertEqual(_find_arrowhead_under_cursor(view, (100.0, 50.0)), -1)

    def test_nearest_arrowhead_wins(self):
        view = _View(
            zoom=1.0,
            links=[
                _Link((100.0, 50.0), is_relationship_link=True),
                _Link((103.0, 50.0), is_relationship_link=True),
            ],
        )
        # Cursor (104,50): link0 dist 4, link1 dist 1 -> link1.
        self.assertEqual(_find_arrowhead_under_cursor(view, (104.0, 50.0)), 1)

    def test_empty_links_returns_minus_one(self):
        self.assertEqual(_find_arrowhead_under_cursor(_View(), (0.0, 0.0)), -1)

    def test_radius_scales_with_zoom(self):
        # zoom 2.0 -> arrow_length max(5,2)=5, radius 5, radius^2 25.
        view = _View(
            zoom=2.0,
            links=[_Link((100.0, 50.0), is_relationship_link=True)],
        )
        self.assertEqual(_find_arrowhead_under_cursor(view, (104.0, 50.0)), 0)
        self.assertEqual(_find_arrowhead_under_cursor(view, (106.0, 50.0)), -1)


class TestCachedRowHitMetrics(unittest.TestCase):
    def test_uses_cached_layout_with_world_position(self):
        node = SimpleNamespace(position=(5, 7), layoutPortStartY=50,
                               layoutPortLineHeight=20)
        self.assertEqual(GraphView._getRowHitMetrics(None, node, _Renderer()), (57, 20))

    def test_uncomputed_line_height_uses_port_width_floor(self):
        node = SimpleNamespace(position=(0, 7), layoutPortStartY=50,
                               layoutPortLineHeight=0)
        self.assertEqual(GraphView._getRowHitMetrics(None, node, _Renderer(12)), (57, 24))

    def test_layout_updates_are_visible_to_hit_testing(self):
        node = SimpleNamespace(position=(0, 7), layoutPortStartY=50,
                               layoutPortLineHeight=20)
        node.layoutPortStartY = 80
        node.layoutPortLineHeight = 30
        self.assertEqual(GraphView._getRowHitMetrics(None, node, _Renderer()), (87, 30))
