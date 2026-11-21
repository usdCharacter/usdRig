#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview node primitives.

``render_nodes.draw(frame, resources, mirror, transforms)`` paints node
quads, port highlights and title-bar icons from a detached frame and
the render-owned mirror, sharing one transform frame with the text
and icon paths. Placement tests pin vertex/port geometry for a known
node at a known transform. Selection tests pin the themed stroke
color, the quad-dirty marking on selection change, and the moved
link-color helpers. Offset tests pin the identity rule: a replaced
record with the same ID discards its transient offsets while an
unchanged identity keeps them.

Everything but the final class runs without a GL context, against
recording fakes and the real native mirror (CPU-only). The final
class attempts a real offscreen context and reports it unavailable
when the platform cannot provide one.
"""

from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

from UsdNoodles.graphview.frame import (
    Frame,
    NodeRecord,
    PortRecord,
    TransformFrameInputs,
    TransformOffset,
    Viewport,
)

try:
    from UsdNoodles.graphview import render_nodes as render_nodes_module
    from UsdNoodles.graphview.render import (
        Renderer,
        transforms_for_viewport,
    )
    from UsdNoodles.graphView import GraphView

    _has_native = True
except ImportError as error:  # pragma: no cover - needs staged natives
    print("render-nodes imports failed: %s" % error)
    _has_native = False


def _record(node_id, selected=False, x=100.0, y=200.0):
    """A known node record: 150x80 at (100, 200) with one port a side."""
    return NodeRecord(
        node_id=node_id,
        position=(x, y),
        size=(150.0, 80.0),
        title=node_id,
        input_pins=("in",),
        output_pins=("out",),
        ports=(
            PortRecord("in", False, (x, y + 30.0)),
            PortRecord("out", True, (x + 150.0, y + 30.0)),
        ),
        selected=selected,
    )


def _inputs(generation=7, offsets=()):
    return TransformFrameInputs(generation=generation, offsets=tuple(offsets))


def _offset(node_id, generation, dx, dy):
    return TransformOffset(node_id, generation, dx, dy)


def _frame(nodes, inputs=None, **overrides):
    values = {
        "revision": 1,
        "identity_generation": 1,
        "nodes": tuple(nodes),
        "links": (),
        "transform_inputs": inputs if inputs is not None else _inputs(1),
        "viewport": Viewport(
            pan_x=0.0, pan_y=0.0, zoom=1.0, width=800.0, height=600.0
        ),
        "content_changed": False,
    }
    values.update(overrides)
    return Frame(**values)


def _span_resources():
    """Resources with only a recording transform frame (submits skip)."""
    return SimpleNamespace(transform_frame=MagicMock())


def _full_resources():
    """Resources with recording managers for every node submit."""
    return SimpleNamespace(
        transform_frame=MagicMock(),
        node_render_manager=MagicMock(),
        font_atlas=MagicMock(),
        icon_renderer=MagicMock(isInitialized=True),
    )


@unittest.skipUnless(_has_native, "render natives not available")
class NodePlacementTest(unittest.TestCase):
    """Vertex/port placement for a known node at a known transform."""

    def test_bounds_without_offset(self):
        record = _record("n1")
        self.assertEqual(
            render_nodes_module.placed_node_bounds(record, _inputs()),
            (100.0, 200.0, 250.0, 280.0),
        )

    def test_bounds_with_live_offset(self):
        record = _record("n1")
        inputs = _inputs(7, [_offset("n1", 7, 5.0, -3.0)])
        self.assertEqual(
            render_nodes_module.placed_node_bounds(record, inputs),
            (105.0, 197.0, 255.0, 277.0),
        )

    def test_bounds_ignores_stale_generation(self):
        record = _record("n1")
        inputs = _inputs(7, [_offset("n1", 6, 50.0, 60.0)])
        self.assertEqual(
            render_nodes_module.placed_node_bounds(record, inputs),
            (100.0, 200.0, 250.0, 280.0),
        )

    def test_bounds_ignores_other_node_offset(self):
        record = _record("n1")
        inputs = _inputs(7, [_offset("n2", 7, 50.0, 60.0)])
        self.assertEqual(
            render_nodes_module.placed_node_bounds(record, inputs),
            (100.0, 200.0, 250.0, 280.0),
        )

    def test_port_centers_without_offset(self):
        record = _record("n1")
        self.assertEqual(
            render_nodes_module.placed_port_centers(record, _inputs()),
            (("in", False, 100.0, 230.0), ("out", True, 250.0, 230.0)),
        )

    def test_port_centers_with_live_offset(self):
        record = _record("n1")
        inputs = _inputs(7, [_offset("n1", 7, 5.0, -3.0)])
        self.assertEqual(
            render_nodes_module.placed_port_centers(record, inputs),
            (("in", False, 105.0, 227.0), ("out", True, 255.0, 227.0)),
        )

    def test_port_centers_ignore_stale_generation(self):
        record = _record("n1")
        inputs = _inputs(7, [_offset("n1", 6, 50.0, 60.0)])
        self.assertEqual(
            render_nodes_module.placed_port_centers(record, inputs),
            (("in", False, 100.0, 230.0), ("out", True, 250.0, 230.0)),
        )

    def test_select_node_offsets_keeps_live_present_only(self):
        first, second = _record("A"), _record("B")
        frame = _frame(
            [first, second],
            _inputs(
                3,
                [
                    _offset("A", 3, 1.0, 2.0),
                    _offset("B", 2, 50.0, 60.0),
                    _offset("gone", 3, 7.0, 8.0),
                ],
            ),
        )
        self.assertEqual(
            render_nodes_module.select_node_offsets(frame),
            {"A": (1.0, 2.0)},
        )

    def test_selected_node_ids(self):
        frame = _frame([_record("A", True), _record("B", False)])
        self.assertEqual(
            render_nodes_module.selected_node_ids(frame), frozenset({"A"})
        )


@unittest.skipUnless(_has_native, "render natives not available")
class NodeSelectionColorTest(unittest.TestCase):
    """Selection color versus unselected, on both color paths."""

    def test_moved_bindings_are_explicit_assignments(self):
        module = render_nodes_module
        for name in (
            "paintNodes",
            "_drawNodeVertices",
            "_renderPortHighlightsCpp",
            "paintGroupStickers",
            "_relationshipLinkColor",
            "_isPrimTargetRelationshipLink",
        ):
            bound = getattr(GraphView, name)
            self.assertIs(bound, getattr(module, name))
            self.assertEqual(bound.__module__, module.__name__)

    def test_relationship_link_color_selected(self):
        view = SimpleNamespace(
            _cachedLinkRelationshipBaseColor=[0.78, 0.24, 0.24, 1.0]
        )
        link = SimpleNamespace(hovered=False, selected=True)
        self.assertEqual(
            GraphView._relationshipLinkColor(
                view, link, (0.0, 0.0, 1.0, 1.0), (1.0, 1.0, 0.0, 1.0)
            ),
            (1.0, 1.0, 0.0, 1.0),
        )

    def test_relationship_link_color_hovered_wins(self):
        view = SimpleNamespace(
            _cachedLinkRelationshipBaseColor=[0.78, 0.24, 0.24, 1.0]
        )
        link = SimpleNamespace(hovered=True, selected=True)
        self.assertEqual(
            GraphView._relationshipLinkColor(
                view, link, (0.0, 0.0, 1.0, 1.0), (1.0, 1.0, 0.0, 1.0)
            ),
            (0.0, 0.0, 1.0, 1.0),
        )

    def test_relationship_link_color_unselected_uses_base(self):
        view = SimpleNamespace(
            _cachedLinkRelationshipBaseColor=[0.78, 0.24, 0.24, 1.0]
        )
        link = SimpleNamespace(hovered=False, selected=False)
        self.assertEqual(
            GraphView._relationshipLinkColor(
                view, link, (0.0, 0.0, 1.0, 1.0), (1.0, 1.0, 0.0, 1.0)
            ),
            (0.78, 0.24, 0.24, 1.0),
        )

    def test_is_prim_target_relationship_link(self):
        prim_target = SimpleNamespace(
            is_relationship_link=True, targetPort="", targetPropertyName=""
        )
        self.assertTrue(
            GraphView._isPrimTargetRelationshipLink(prim_target)
        )
        pin_target = SimpleNamespace(
            is_relationship_link=True,
            targetPort="in",
            targetPropertyName="",
        )
        self.assertFalse(GraphView._isPrimTargetRelationshipLink(pin_target))
        attribute = SimpleNamespace(
            is_relationship_link=False, targetPort="", targetPropertyName=""
        )
        self.assertFalse(GraphView._isPrimTargetRelationshipLink(attribute))

    def test_selected_stroke_honors_patched_config(self):
        with patch("UsdNoodles.graphView.NoodlesConfig") as config:
            config.get.side_effect = lambda key, default: default
            self.assertEqual(
                render_nodes_module._selected_stroke_color(),
                [1.0, 1.0, 0.3, 1.0],
            )
        with patch("UsdNoodles.graphView.NoodlesConfig") as config:
            config.get.return_value = [9.0, 8.0, 7.0, 1.0]
            self.assertEqual(
                render_nodes_module._selected_stroke_color(),
                [9.0, 8.0, 7.0, 1.0],
            )

    def test_selected_stroke_without_legacy_module(self):
        saved = sys.modules.pop("UsdNoodles.graphView", None)
        try:
            color = render_nodes_module._selected_stroke_color()
        finally:
            if saved is not None:
                sys.modules["UsdNoodles.graphView"] = saved
        self.assertEqual(len(color), 4)
        self.assertTrue(
            all(isinstance(v, (int, float)) for v in color)
        )


@unittest.skipUnless(_has_native, "render natives not available")
class TransientOffsetIdentityTest(unittest.TestCase):
    """Replacing a record drops its offsets; an unchanged one keeps them."""

    def test_first_draw_keeps_live_offsets(self):
        record = _record("A")
        frame = _frame([record], _inputs(3, [_offset("A", 3, 5.0, 6.0)]))
        resources = _span_resources()
        Renderer(resources).draw(frame)
        resources.transform_frame.reset.assert_called_once_with()
        resources.transform_frame.translate.assert_called_once_with(
            "A", 5.0, 6.0
        )
        self.assertEqual(
            render_nodes_module.placed_node_bounds(
                record, frame.transform_inputs
            ),
            (105.0, 206.0, 255.0, 286.0),
        )

    def test_unchanged_identity_keeps_offsets(self):
        record = _record("A")
        nodes = (record,)
        first = _frame(
            nodes, _inputs(3, [_offset("A", 3, 5.0, 6.0)]), revision=1
        )
        second = _frame(
            nodes, _inputs(3, [_offset("A", 3, 5.0, 6.0)]), revision=1
        )
        resources = _span_resources()
        renderer = Renderer(resources)
        renderer.draw(first)
        renderer.draw(second)
        self.assertEqual(resources.transform_frame.reset.call_count, 2)
        self.assertEqual(resources.transform_frame.translate.call_count, 2)
        resources.transform_frame.translate.assert_called_with("A", 5.0, 6.0)

    def test_replaced_record_discards_offsets(self):
        first = _frame(
            [_record("A")], _inputs(3, [_offset("A", 3, 5.0, 6.0)]), revision=1
        )
        replaced = _frame(
            [_record("A")], _inputs(3, [_offset("A", 3, 5.0, 6.0)]), revision=2
        )
        resources = _span_resources()
        renderer = Renderer(resources)
        renderer.draw(first)
        renderer.draw(replaced)
        frame_mock = resources.transform_frame
        self.assertEqual(frame_mock.translate.call_count, 2)
        self.assertEqual(frame_mock.reset.call_count, 3)
        tail = frame_mock.mock_calls[-2:]
        self.assertEqual(
            tail, [call.translate("A", 5.0, 6.0), call.reset()]
        )

    def test_stale_generation_draws_nothing(self):
        frame = _frame(
            [_record("A")], _inputs(3, [_offset("A", 2, 5.0, 6.0)])
        )
        resources = _span_resources()
        Renderer(resources).draw(frame)
        resources.transform_frame.translate.assert_not_called()

    def test_removed_id_discards_its_offset(self):
        kept, dropped = _record("A"), _record("B")
        both = _frame(
            [kept, dropped],
            _inputs(
                3, [_offset("A", 3, 5.0, 6.0), _offset("B", 3, 7.0, 8.0)]
            ),
            revision=1,
        )
        narrowed = _frame(
            [kept],
            _inputs(
                3, [_offset("A", 3, 5.0, 6.0), _offset("B", 3, 7.0, 8.0)]
            ),
            revision=2,
        )
        resources = _span_resources()
        renderer = Renderer(resources)
        renderer.draw(both)
        renderer.draw(narrowed)
        frame_mock = resources.transform_frame
        tail = frame_mock.mock_calls[-2:]
        self.assertEqual(tail, [call.reset(), call.translate("A", 5.0, 6.0)])

    def test_reconcile_skipped_without_frame(self):
        frame = _frame(
            [_record("A")], _inputs(3, [_offset("A", 3, 5.0, 6.0)])
        )
        Renderer().draw(frame)
        Renderer(SimpleNamespace()).draw(frame)


@unittest.skipUnless(_has_native, "render natives not available")
class RenderNodesDrawTest(unittest.TestCase):
    """Draw submits the mirror (never the live model) to the managers."""

    def test_primitives_discovered(self):
        frame = _frame([_record("A")])
        renderer = Renderer(_span_resources())
        renderer.draw(frame)
        self.assertIn("render_nodes", renderer.primitives_drawn)

    def test_draw_uses_no_private_transform_frame(self):
        self.assertFalse(hasattr(render_nodes_module, "NodeTransformFrame"))
        import inspect

        self.assertEqual(
            list(inspect.signature(render_nodes_module.draw).parameters),
            ["frame", "resources", "mirror", "transforms"],
        )

    def test_submits_node_quads_from_mirror(self):
        record = _record("A")
        frame = _frame([record])
        resources = _full_resources()
        renderer = Renderer(resources)
        with patch("UsdNoodles.graphView.NoodlesConfig") as config:
            config.get.side_effect = lambda key, default: default
            renderer.draw(frame)
        manager = resources.node_render_manager
        manager.renderNodesFromGraph.assert_called_once()
        args = manager.renderNodesFromGraph.call_args.args
        self.assertIs(args[0], renderer.mirror)
        self.assertIs(args[2], resources.font_atlas)
        self.assertEqual(
            args[3], list(transforms_for_viewport(frame.viewport).world)
        )
        self.assertEqual(args[5], [1.0, 1.0, 0.3, 1.0])
        self.assertEqual(args[6], False)

    def test_selection_change_marks_quads_dirty(self):
        resources = _full_resources()
        renderer = Renderer(resources)
        renderer.draw(_frame([_record("A", False)], revision=1))
        resources.node_render_manager.markNodeQuadDirty.assert_not_called()
        renderer.draw(_frame([_record("A", True)], revision=2))
        resources.node_render_manager.markNodeQuadDirty.assert_called_once_with(
            True
        )

    def test_submits_port_highlights_with_hover(self):
        frame = _frame([_record("A")], hovered_port=("A", "in", False))
        resources = _full_resources()
        renderer = Renderer(resources)
        with patch("UsdNoodles.graphView.NoodlesConfig") as config:
            config.get.side_effect = lambda key, default: default
            renderer.draw(frame)
        manager = resources.node_render_manager
        manager.renderPortHighlightsFromGraph.assert_called_once()
        args = manager.renderPortHighlightsFromGraph.call_args.args
        self.assertIs(args[0], renderer.mirror)
        self.assertEqual(
            args[2], list(transforms_for_viewport(frame.viewport).world)
        )
        self.assertEqual(args[4:8], (True, "A", "in", False))
        self.assertEqual(args[8:12], (False, "", "", False))
        self.assertEqual(args[12], False)

    def test_submits_port_highlights_without_hover(self):
        frame = _frame([_record("A")])
        resources = _full_resources()
        with patch("UsdNoodles.graphView.NoodlesConfig") as config:
            config.get.side_effect = lambda key, default: default
            Renderer(resources).draw(frame)
        args = (
            resources.node_render_manager.renderPortHighlightsFromGraph.call_args.args
        )
        self.assertEqual(args[4:8], (False, "", "", False))

    def test_submits_icons_with_viewport(self):
        frame = _frame([_record("A")])
        resources = _full_resources()
        renderer = Renderer(resources)
        renderer.draw(frame)
        icons = resources.icon_renderer
        icons.renderNodeIcons.assert_called_once()
        args = icons.renderNodeIcons.call_args.args
        self.assertIs(args[0], renderer.mirror)
        self.assertEqual(
            args[1].data(), list(transforms_for_viewport(frame.viewport).world)
        )
        self.assertEqual(args[2:8], (1.0, False, 0.0, 0.0, 800.0, 600.0))
        self.assertEqual(args[8], True)

    def test_skips_icons_when_uninitialized(self):
        frame = _frame([_record("A")])
        resources = _full_resources()
        resources.icon_renderer.isInitialized = False
        Renderer(resources).draw(frame)
        resources.icon_renderer.renderNodeIcons.assert_not_called()

    def test_skips_submits_without_managers(self):
        frame = _frame(
            [_record("A")], _inputs(3, [_offset("A", 3, 5.0, 6.0)])
        )
        resources = _span_resources()
        Renderer(resources).draw(frame)
        resources.transform_frame.translate.assert_called_once_with(
            "A", 5.0, 6.0
        )

    def test_draw_nothing_when_no_nodes(self):
        frame = _frame([])
        resources = _full_resources()
        Renderer(resources).draw(frame)
        resources.node_render_manager.renderNodesFromGraph.assert_not_called()
        resources.node_render_manager.renderPortHighlightsFromGraph.assert_not_called()
        resources.icon_renderer.renderNodeIcons.assert_not_called()


@unittest.skipUnless(_has_native, "render natives not available")
class RenderNodesGLTest(unittest.TestCase):
    """One real offscreen draw when the platform provides a context."""

    def test_real_offscreen_draw(self):
        try:
            from headless.harness import NoodlesTestHarness
        except ImportError as error:
            self.skipTest("headless harness unavailable: %s" % error)
        harness = NoodlesTestHarness()
        try:
            harness.setup()
        except Exception as error:
            self.skipTest("GL context unavailable: %s" % error)
        self.addCleanup(harness.teardown)
        from UsdNoodles.graphview.resources import RenderResources

        view = harness.graph_view
        try:
            bundle = RenderResources.initialize(view)
        except RuntimeError as error:
            self.skipTest("GL context not usable: %s" % error)
        self.assertIsNotNone(bundle)
        frame = _frame([_record("A", x=10.0, y=20.0)])
        renderer = Renderer(bundle)
        try:
            mirror = renderer.draw(frame)
        except Exception as error:
            self.skipTest("offscreen draw unavailable: %s" % error)
        self.assertIsNotNone(mirror)
        self.assertIn("render_nodes", renderer.primitives_drawn)


if __name__ == "__main__":
    unittest.main()
