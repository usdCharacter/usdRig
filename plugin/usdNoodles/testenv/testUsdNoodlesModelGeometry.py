#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Direct placement and geometry checks against the staged model owner."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from pxr import Gf, Sdf, Usd

from UsdNoodles.graphView import GraphView
from UsdNoodles.graphview import model
from UsdNoodles.models import NodeModel
from UsdNoodles.nodeLayout import layout_graph


class _GeometryView:
    _AUTO_GRID_COLUMNS = 2
    _AUTO_GRID_GAP = 10.0
    _batchExtents = model.batch_extents
    _pushBelowOccupied = model.push_below_occupied
    _nodeHasVisiblePort = model.node_has_visible_port
    _mirrorPortPosition = staticmethod(model.mirror_port_position)
    _getRowHitMetrics = GraphView._getRowHitMetrics
    _valueRowLayout = model.value_row_layout
    _valueRowGeometry = model.value_row_geometry

    def _valueTextBand(self, height):
        return 5.0, height - 10.0


class TestModelGeometry(unittest.TestCase):
    def test_grid_clears_occupied_rows_and_advances_native_versions(self):
        stage = Usd.Stage.CreateInMemory()
        nodes = []
        for name, size in (("/A", (100, 50)), ("/B", (140, 80)), ("/C", (90, 30))):
            stage.DefinePrim(name)
            node = NodeModel(stage=stage, primPath=Sdf.Path(name))
            node.size = Gf.Vec2d(*size)
            node.setDisplayPosition(Gf.Vec2d(0, 0))
            nodes.append(node)

        # The batch hits the first blocker, then the second, but not the third.
        blockers = [
            SimpleNamespace(id=name, position=pos, size=(100, 40))
            for name, pos in (("/First", (10, 20)), ("/Second", (10, 75)), ("/Far", (320, 130)))
        ]
        view = _GeometryView()
        view.nodes = {node.id: node for node in nodes + blockers}
        self.assertEqual(model.push_below_occupied(view, 10, 20, nodes, blockers), 125)
        self.assertEqual(model.push_below_occupied(view, 500, 20, nodes, blockers), 20)

        versions = [node.positionVersion for node in nodes]
        layer = stage.GetRootLayer().ExportToString()
        self.assertEqual(model.grid_place_nodes(view, nodes, anchor=(10, 20)), nodes)
        self.assertEqual([tuple(node.position) for node in nodes], [(10, 125), (160, 125), (10, 215)])
        for node, version in zip(nodes, versions):
            self.assertGreater(node.positionVersion, version)
        self.assertEqual(stage.GetRootLayer().ExportToString(), layer)

    def test_auto_layout_applies_pin_aligned_positions_and_records_replay(self):
        sizes = {"A": (200.0, 100.0), "B": (200.0, 100.0)}
        old = {"A": (10.0, 20.0), "B": (30.0, 40.0)}
        nodes = {key: SimpleNamespace(position=old[key], size=size) for key, size in sizes.items()}
        applied = []
        frames = []
        view = SimpleNamespace(
            nodes=nodes,
            links=[SimpleNamespace(sourceNodeId="A", targetNodeId="B", start=(210, 50), end=(30, 110))],
            _applyNodePositions=applied.append,
            frameAll=lambda: frames.append(True),
        )
        with (
            patch("UsdNoodles.graphView.layout_graph", wraps=layout_graph) as layout,
            patch("UsdNoodles.graphView._push_undo_command") as push,
        ):
            model.auto_layout_nodes(view)
        edges = [("A", "B", 30.0, 70.0)]
        expected = {key: tuple(pos) for key, pos in layout_graph(sizes, edges).items()}
        layout.assert_called_once_with(sizes, edges)
        self.assertEqual(applied, [expected])
        self.assertEqual(frames, [True])
        push.assert_called_once()
        description, redo, undo = push.call_args.args
        self.assertEqual(description, "Auto Layout")
        self.assertIs(push.call_args.kwargs["view"], view)
        restored = []
        replay_view = SimpleNamespace(_restoreNodePositions=restored.append)
        undo(replay_view)
        redo(replay_view)
        self.assertEqual(restored, [old, expected])
        self.assertIs(GraphView._autoLayoutNodes, model.auto_layout_nodes)

    def test_dual_port_and_value_rect_use_node_geometry(self):
        row = SimpleNamespace(property_name="p", count=1, has_swatch=False)
        renderer = SimpleNamespace(
            getPortPosition=lambda node, pin, output, atlas: Gf.Vec2d(118, 190),
            resolvePortPosition=lambda node, pin, output: Gf.Vec2d(682, 190),
            getPortWidth=lambda: 8.0,
        )
        node = SimpleNamespace(
            position=Gf.Vec2d(100, 20), size=Gf.Vec2d(600, 400),
            renderer=renderer, inputPins=["p"], outputPins=[], _dual_pin_names={"p"},
            layoutPortStartY=100.0, layoutPortLineHeight=50.0,
            inputRowSlots=[1], inputRowKinds=[0], outputRowSlots=[], _value_cell_width=200.0,
            value_rows=lambda time: {("p", False): row},
        )
        view = _GeometryView()
        view.fontAtlas = object()
        view._cachedShowAttributeValues = True
        view._currentTimeCode = Usd.TimeCode.Default()
        self.assertEqual(model.get_visible_port_position(view, node, "p", True), Gf.Vec2d(682, 190))
        self.assertEqual(model.get_port_position(view, node, "p", True), Gf.Vec2d(682, 190))
        [(actual_row, rect, (component,), height)] = model.value_row_geometry(view, node)
        self.assertIs(actual_row, row)
        self.assertEqual(height, 50)
        self.assertEqual((rect.left, rect.top, rect.right, rect.bottom), (427.5, 175, 627.5, 215))
        self.assertEqual(model.value_cell_rect_for(view, node, "p"), component)
        node.outputPins = ["occupied"]
        node.outputRowSlots = [1]
        self.assertEqual(list(model.value_row_geometry(view, node)), [])
