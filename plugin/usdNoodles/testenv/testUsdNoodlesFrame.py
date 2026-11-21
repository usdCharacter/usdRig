#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview/frame layer.

``FrameSource.capture(viewport, interaction_state, time)`` pins one
immutable ``Frame`` (revision, identity generation, detached node/link
records with explicit dangling records, overlay/hotbox records,
transform inputs, hit candidates) and ``FrameSource.hit(point)`` picks
against the same revision's candidates, transforms and draw ordering.
Capture is available on demand before the first paint; a pan changes
the viewport half only; a stage switch bumps the generation while
retained frames stay hittable.

Most cases run against stub views and a duck-typed fake model state so
the render layer's decoupling (no model/selection/editing imports, no
Qt, no native module) is load-bearing. The ``RealModelInterop`` cases
run the same source against a real ``GraphModelState``.
"""

from __future__ import annotations

import ast
import dataclasses
import threading
import unittest
from types import SimpleNamespace

from UsdNoodles.graphview import frame
from UsdNoodles.graphview.frame import (
    FlashRecord,
    FrameSource,
    HitParams,
    HotboxRecord,
    InteractionState,
    LinkId,
    OverlayState,
    PanAnim,
    PopupRecord,
    TransformFrameInputs,
    TransformOffset,
    ValueCellCandidate,
    Viewport,
)

try:
    from pxr import Gf, Sdf, Usd, UsdGeom

    from UsdNoodles.core import LinkData
    from UsdNoodles.graphview import model
    from UsdNoodles.models import NodeModel
    from UsdNoodles.nodeGraph import NodeGraph
    from UsdNoodles.spatial import SpatialIndex

    _has_real = True
except ImportError as error:  # pragma: no cover - needs staged natives
    print("frame real-model imports failed: %s" % error)
    _has_real = False


def _make_node(
    node_id,
    x=0.0,
    y=0.0,
    w=120.0,
    h=80.0,
    z=0,
    inputs=("in",),
    outputs=("out",),
    line_height=20.0,
    start_y=20.0,
):
    centers_in = [start_y + 10.0 + i * line_height for i in range(len(inputs))]
    centers_out = [
        start_y + 10.0 + i * line_height for i in range(len(outputs))
    ]
    return SimpleNamespace(
        position=(x, y),
        size=(w, h),
        zOrder=z,
        name=node_id,
        inputPins=list(inputs),
        outputPins=list(outputs),
        layoutPortStartY=start_y,
        layoutPortLineHeight=line_height,
        layoutInputCenterY=centers_in,
        layoutOutputCenterY=centers_out,
        inputRowKinds=[0] * len(inputs),
        inputRowSlots=list(range(len(inputs))),
        outputRowKinds=[0] * len(outputs),
        outputRowSlots=list(range(len(outputs))),
        renderer=None,
    )


def _make_link(
    src,
    src_port,
    tgt,
    tgt_port,
    start=(0.0, 0.0),
    end=(10.0, 0.0),
    dangling=False,
    dangling_direction="",
    relationship=False,
    owner="",
):
    return SimpleNamespace(
        sourceNodeId=src,
        sourcePort=src_port,
        targetNodeId=tgt,
        targetPort=tgt_port,
        start=start,
        end=end,
        is_relationship_link=relationship,
        isDangling=dangling,
        danglingDirection=dangling_direction,
        targetPropertyName="",
        propertyName=owner,
    )


class FakeGraph:
    def __init__(self):
        self.nodes = {}
        self.links = []


class FakeChangeSet:
    def __init__(
        self,
        revision,
        changed_nodes=(),
        removed_nodes=(),
        changed_links=(),
        removed_links=(),
        remaps=(),
    ):
        self.revision = revision
        self.topology_revision = revision
        self.changed_nodes = tuple(changed_nodes)
        self.removed_nodes = tuple(removed_nodes)
        self.changed_links = tuple(changed_links)
        self.removed_links = tuple(removed_links)
        self.remaps = tuple(remaps)


class FakeState:
    """Duck-typed GraphModelState: graph, revision, subscribe."""

    def __init__(self, graph=None):
        self.graph = graph if graph is not None else FakeGraph()
        self.revision = 0
        self._subscribers = []

    def subscribe(self, callback):
        self._subscribers.append(callback)

        def unsubscribe():
            try:
                self._subscribers.remove(callback)
            except ValueError:
                pass

        return unsubscribe

    def commit(self, **fields):
        self.revision += 1
        changeset = FakeChangeSet(self.revision, **fields)
        for callback in list(self._subscribers):
            callback(changeset)
        return changeset


def _viewport(**overrides):
    fields = {
        "pan_x": 0.0,
        "pan_y": 0.0,
        "zoom": 1.0,
        "width": 800.0,
        "height": 600.0,
    }
    fields.update(overrides)
    return Viewport(**fields)


def _interaction(**overrides):
    return InteractionState(**overrides)


def _two_node_graph():
    graph = FakeGraph()
    graph.nodes["A"] = _make_node("A", x=0.0, y=0.0, z=1)
    graph.nodes["B"] = _make_node("B", x=200.0, y=0.0, z=2)
    graph.links.append(
        _make_link("A", "out", "B", "in", start=(120.0, 30.0), end=(200.0, 30.0))
    )
    return graph


class FrameCaptureTest(unittest.TestCase):
    def test_capture_before_paint_and_hit_node(self):
        graph = _two_node_graph()
        view = SimpleNamespace(nodeGraph=graph)
        source = FrameSource(view)
        captured = source.capture(_viewport(), _interaction(), 100.0)
        self.assertEqual(captured.revision, 0)
        self.assertEqual(captured.identity_generation, 0)
        self.assertEqual([r.node_id for r in captured.nodes], ["A", "B"])
        self.assertEqual(len(captured.links), 1)
        # Title band of A (above its first row band at y=20..40).
        hit = source.hit((60.0, 10.0))
        self.assertIsNotNone(hit)
        self.assertEqual(hit.kind, "node")
        self.assertEqual(hit.node_id, "A")
        self.assertEqual(hit.revision, 0)
        self.assertEqual(hit.identity_generation, 0)
        self.assertEqual(hit.world, (60.0, 10.0))

    def test_hit_link_midpoint_and_miss(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), _interaction(), 100.0)
        hit = captured.hit((160.0, 30.0))
        self.assertIsNotNone(hit)
        self.assertEqual(hit.kind, "link")
        self.assertEqual(hit.link_index, 0)
        self.assertIsNotNone(hit.link_id)
        self.assertEqual(hit.link_id.source_id, "A")
        self.assertEqual(hit.link_id.target_id, "B")
        self.assertIsNone(captured.hit((160.0, 300.0)))

    def test_hit_topmost_node_first(self):
        graph = FakeGraph()
        graph.nodes["low"] = _make_node("low", x=0.0, y=0.0, z=1)
        graph.nodes["high"] = _make_node("high", x=0.0, y=0.0, z=9)
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        hit = captured.hit((60.0, 10.0))
        self.assertEqual((hit.kind, hit.node_id), ("node", "high"))

    def test_hit_port_bands(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        hit = captured.hit((60.0, 30.0))
        self.assertEqual(
            (hit.kind, hit.node_id, hit.pin, hit.is_output),
            ("port", "A", "in", False),
        )
        hit = source.hit((260.0, 30.0))
        self.assertEqual(
            (hit.kind, hit.node_id, hit.pin, hit.is_output),
            ("port", "B", "in", False),
        )

    def test_collapsed_node_has_no_port_hits(self):
        node = _make_node("A")
        node._title_collapsed = True
        graph = FakeGraph()
        graph.nodes["A"] = node
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        self.assertEqual(captured.nodes[0].ports, ())
        hit = captured.hit((60.0, 30.0))
        self.assertEqual((hit.kind, hit.node_id), ("node", "A"))

    def test_hit_value_cell_and_show_gate(self):
        state = _interaction(
            value_cells=(
                ValueCellCandidate(
                    node_id="A",
                    pin_name="in",
                    component=0,
                    rect=(20.0, 22.0, 100.0, 38.0),
                ),
            )
        )
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), state, 1.0)
        hit = captured.hit((60.0, 30.0))
        self.assertEqual(
            (hit.kind, hit.node_id, hit.pin, hit.component),
            ("value_cell", "A", "in", 0),
        )
        hidden = source.capture(
            _viewport(),
            _interaction(show_values=False, value_cells=state.value_cells),
            2.0,
        )
        # The pinned candidates stay, but the show gate falls through.
        self.assertEqual(hidden.value_cells, captured.value_cells)
        self.assertEqual(hidden.hit((60.0, 30.0)).kind, "port")

    def test_hit_overlay_order(self):
        state = _interaction(
            hotbox=HotboxRecord(
                visible=True, rect=(0.0, 0.0, 400.0, 300.0)
            )
        )
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), state, 1.0)
        hit = captured.hit((60.0, 10.0))
        self.assertEqual(hit.kind, "hotbox")

    def test_hit_links_use_draw_tolerance(self):
        graph = _two_node_graph()
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        wide = source.capture(
            _viewport(), _interaction(link_line_width=12.0), 1.0
        )
        # Tolerance max(0.5*12, 2/1) = 6: 3 units off still hits.
        self.assertEqual(wide.hit((160.0, 33.0)).kind, "link")
        narrow = source.capture(
            _viewport(), _interaction(link_line_width=0.5), 2.0
        )
        # Tolerance max(0.25, 2/1) = 2: 3 units off misses.
        self.assertIsNone(narrow.hit((160.0, 33.0)))
        self.assertEqual(narrow.hit((160.0, 31.0)).kind, "link")

    def test_dangling_links_are_explicit_shared_records(self):
        graph = _two_node_graph()
        graph.links.append(
            _make_link(
                "B",
                "out",
                "ghost",
                "",
                start=(320.0, 30.0),
                end=(380.0, 30.0),
                dangling=True,
                dangling_direction="output",
            )
        )
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        self.assertEqual(len(captured.links), 2)
        self.assertEqual(len(captured.dangling_links), 1)
        dangling = captured.dangling_links[0]
        self.assertIs(dangling, captured.links[1])
        self.assertTrue(dangling.is_dangling)
        self.assertEqual(dangling.link_id.dangling_id, "ghost")
        hit = captured.hit((350.0, 30.0))
        self.assertEqual((hit.kind, hit.detail), ("link", "dangling"))


class FrameSharingTest(unittest.TestCase):
    def test_edit_shares_unchanged_storage(self):
        graph = FakeGraph()
        graph.nodes["A"] = _make_node("A", x=0.0, y=0.0)
        graph.nodes["B"] = _make_node("B", x=200.0, y=0.0)
        graph.nodes["C"] = _make_node("C", x=0.0, y=200.0)
        graph.nodes["D"] = _make_node("D", x=200.0, y=200.0)
        graph.links.append(
            _make_link(
                "A", "out", "B", "in",
                start=(120.0, 30.0), end=(200.0, 30.0),
            )
        )
        graph.links.append(
            _make_link(
                "C", "out", "D", "in",
                start=(120.0, 230.0), end=(200.0, 230.0),
            )
        )
        state = FakeState(graph)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        graph.nodes["B"].position = (220.0, 10.0)
        graph.links[0].end = (220.0, 40.0)
        state.commit(changed_nodes=["B"])
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual((first.revision, second.revision), (0, 1))
        by_id = lambda fr: {r.node_id: r for r in fr.nodes}
        first_nodes, second_nodes = by_id(first), by_id(second)
        self.assertIs(second_nodes["A"], first_nodes["A"])
        self.assertIs(second_nodes["C"], first_nodes["C"])
        self.assertIs(second_nodes["D"], first_nodes["D"])
        self.assertIsNot(second_nodes["B"], first_nodes["B"])
        self.assertEqual(second_nodes["B"].position, (220.0, 10.0))
        # The incident edge rebuilds; the untouched edge is shared.
        self.assertIsNot(second.links[0], first.links[0])
        self.assertIs(second.links[1], first.links[1])

    def test_retained_frame_hits_after_model_moves_on(self):
        graph = _two_node_graph()
        state = FakeState(graph)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        graph.nodes["A"].position = (500.0, 500.0)
        graph.nodes["B"].position = (700.0, 500.0)
        state.commit(changed_nodes=["A", "B"])
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertIsNot(second, first)
        self.assertEqual(first.revision, 0)
        self.assertEqual(second.revision, 1)
        # The retained frame still pins and hits its own revision.
        self.assertEqual(first.nodes[0].position, (0.0, 0.0))
        hit = first.hit((60.0, 10.0))
        self.assertEqual((hit.kind, hit.node_id, hit.revision), ("node", "A", 0))
        moved = second.hit((560.0, 510.0))
        self.assertEqual((moved.kind, moved.node_id), ("node", "A"))

    def test_pan_shares_graph_records(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        first = source.capture(_viewport(), _interaction(), 1.0)
        panned = source.capture(
            _viewport(pan_x=100.0, pan_y=50.0), _interaction(), 2.0
        )
        self.assertIs(panned.nodes, first.nodes)
        self.assertIs(panned.links, first.links)
        self.assertIs(panned.dangling_links, first.dangling_links)
        self.assertIs(panned.hit_candidates, first.hit_candidates)
        self.assertEqual(panned.revision, first.revision)
        self.assertNotEqual(panned.viewport, first.viewport)
        # Same world point through the new transform still hits A.
        hit = panned.hit((-40.0, -40.0))
        self.assertEqual((hit.kind, hit.node_id), ("node", "A"))
        self.assertEqual(hit.world, (60.0, 10.0))

    def test_stage_switch_bumps_generation(self):
        state_a = FakeState(_two_node_graph())
        source = FrameSource(None, state_a)
        first = source.capture(_viewport(), _interaction(), 1.0)
        other = FakeGraph()
        other.nodes["Z"] = _make_node("Z", x=10.0, y=10.0)
        source.attach(FakeState(other))
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual(first.identity_generation, 0)
        self.assertEqual(second.identity_generation, 1)
        self.assertEqual([r.node_id for r in second.nodes], ["Z"])
        # The old generation stays retained and hittable.
        self.assertEqual(
            [r.node_id for r in first.nodes], ["A", "B"]
        )
        hit = first.hit((60.0, 10.0))
        self.assertEqual(
            (hit.kind, hit.node_id, hit.identity_generation),
            ("node", "A", 0),
        )

    def test_view_graph_swap_bumps_generation_without_state(self):
        view = SimpleNamespace(nodeGraph=_two_node_graph())
        source = FrameSource(view)
        first = source.capture(_viewport(), _interaction(), 1.0)
        view.nodeGraph = FakeGraph()
        view.nodeGraph.nodes["Z"] = _make_node("Z")
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual(
            (first.identity_generation, second.identity_generation), (0, 1)
        )
        self.assertEqual([r.node_id for r in second.nodes], ["Z"])

    def test_remap_rebuilds_and_incident_edges(self):
        graph = _two_node_graph()
        state = FakeState(graph)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        node = graph.nodes.pop("B")
        node.name = "B2"
        graph.nodes["B2"] = node
        for link in graph.links:
            if link.targetNodeId == "B":
                link.targetNodeId = "B2"
        state.commit(changed_nodes=["B2"], remaps=[("B", "B2")])
        second = source.capture(_viewport(), _interaction(), 2.0)
        ids = [r.node_id for r in second.nodes]
        self.assertNotIn("B", ids)
        self.assertIn("B2", ids)
        by_id = lambda fr: {r.node_id: r for r in fr.nodes}
        self.assertIs(by_id(second)["A"], by_id(first)["A"])
        self.assertIsNot(second.links[0], first.links[0])
        self.assertEqual(second.links[0].link_id.target_id, "B2")

    def test_remove_node_drops_records_and_links(self):
        graph = _two_node_graph()
        state = FakeState(graph)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        doomed = frame.link_id_for(graph.links[0])
        del graph.nodes["B"]
        del graph.links[0]
        state.commit(removed_nodes=["B"], removed_links=[doomed])
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual([r.node_id for r in second.nodes], ["A"])
        self.assertEqual(second.links, ())
        self.assertEqual(second.dangling_links, ())
        by_id = lambda fr: {r.node_id: r for r in fr.nodes}
        self.assertIs(by_id(second)["A"], by_id(first)["A"])

    def test_selection_change_rebuilds_only_affected(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        first = source.capture(_viewport(), _interaction(), 1.0)
        second = source.capture(
            _viewport(), _interaction(selected_nodes=("B",)), 2.0
        )
        by_id = lambda fr: {r.node_id: r for r in fr.nodes}
        self.assertIs(by_id(second)["A"], by_id(first)["A"])
        self.assertIsNot(by_id(second)["B"], by_id(first)["B"])
        self.assertTrue(by_id(second)["B"].selected)
        self.assertFalse(by_id(second)["A"].selected)
        self.assertIs(second.links[0], first.links[0])

    def test_foreign_link_keys_equate_by_fields(self):
        graph = _two_node_graph()
        state = FakeState(graph)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        foreign = SimpleNamespace(
            kind="attribute",
            owner_property="",
            source_id="A",
            source_pin="out",
            target_id="B",
            target_pin="in",
            dangling_id="",
        )
        second = source.capture(
            _viewport(), _interaction(selected_links=(foreign,)), 2.0
        )
        self.assertTrue(second.links[0].selected)
        self.assertFalse(first.links[0].selected)
        self.assertIsNot(second.links[0], first.links[0])

    def test_value_key_change_rebuilds_value_inputs(self):
        node = _make_node("A")
        seen = []

        def value_rows(time_code):
            seen.append(time_code)
            components = (1.5,) if len(seen) == 1 else (2.5,)
            return {
                ("in", False): SimpleNamespace(
                    property_name="p",
                    kind="float",
                    count=1,
                    components=components,
                    has_swatch=False,
                )
            }

        node.value_rows = value_rows
        graph = FakeGraph()
        graph.nodes["A"] = node
        view = SimpleNamespace(nodeGraph=graph, _currentTimeCode=None)
        source = FrameSource(view)
        first = source.capture(_viewport(), _interaction(), 1.0)
        self.assertEqual(len(first.nodes[0].value_inputs), 1)
        self.assertEqual(
            first.nodes[0].value_inputs[0].components, (1.5,)
        )
        second = source.capture(
            _viewport(), _interaction(time_code=24.0), 2.0
        )
        self.assertIsNot(second.nodes[0], first.nodes[0])
        self.assertEqual(
            second.nodes[0].value_inputs[0].components, (2.5,)
        )
        third = source.capture(
            _viewport(), _interaction(time_code=24.0), 3.0
        )
        self.assertIs(third.nodes[0], second.nodes[0])
        # Rows are read through the view's live time code, never a
        # synthesized one.
        self.assertEqual(seen, [None, None, None])

    def test_transform_offsets_shift_hits(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        plain = source.capture(_viewport(), _interaction(), 1.0)
        shifted = source.capture(
            _viewport(),
            _interaction(
                transform_inputs=TransformFrameInputs(
                    generation=7,
                    offsets=(
                        TransformOffset(
                            node_id="A", generation=7, dx=50.0, dy=0.0
                        ),
                    ),
                )
            ),
            2.0,
        )
        # Records are shared; only the candidates move with the offset.
        self.assertIs(shifted.nodes, plain.nodes)
        self.assertIsNot(
            shifted.hit_candidates, plain.hit_candidates
        )
        candidate = [
            c
            for c in shifted.hit_candidates.nodes
            if c.node_id == "A"
        ][0]
        self.assertEqual(candidate.bounds, (50.0, 0.0, 170.0, 80.0))
        self.assertEqual(shifted.hit((110.0, 10.0)).node_id, "A")
        stale = source.capture(
            _viewport(),
            _interaction(
                transform_inputs=TransformFrameInputs(
                    generation=8,
                    offsets=(
                        TransformOffset(
                            node_id="A", generation=7, dx=50.0, dy=0.0
                        ),
                    ),
                )
            ),
            3.0,
        )
        self.assertEqual(stale.hit((10.0, 10.0)).node_id, "A")


class FrameAnimationTest(unittest.TestCase):
    def test_view_animations_run_before_graph_reads(self):
        log = []

        class RecordingGraph(FakeGraph):
            @property
            def nodes(self):
                log.append("read-nodes")
                return self.__dict__.setdefault("_nodes", {})

            @nodes.setter
            def nodes(self, value):
                self.__dict__["_nodes"] = value

            @property
            def links(self):
                log.append("read-links")
                return self.__dict__.setdefault("_links", [])

            @links.setter
            def links(self, value):
                self.__dict__["_links"] = value

        graph = RecordingGraph()
        graph.nodes["A"] = _make_node("A")
        view = SimpleNamespace(
            nodeGraph=graph,
            _updateAnimator=lambda: log.append("animator"),
            _updatePanAnimation=lambda: log.append("pan"),
        )
        del log[:]
        source = FrameSource(view)
        source.capture(_viewport(), _interaction(), 1.0)
        self.assertEqual(log[:2], ["animator", "pan"])
        self.assertIn("read-nodes", log[2:])

    def test_flash_resolves_against_time(self):
        live = OverlayState(
            flash=FlashRecord(
                node_id="A",
                property_name="p",
                component=0,
                end_time=100.0,
            )
        )
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        early = source.capture(_viewport(), _interaction(overlay=live), 50.0)
        self.assertIsNotNone(early.overlay.flash)
        self.assertIsNotNone(early.animations.flash)
        self.assertIs(early.animations.flash, early.overlay.flash)
        late = source.capture(_viewport(), _interaction(overlay=live), 150.0)
        self.assertIsNone(late.overlay.flash)
        self.assertIsNone(late.animations.flash)
        # The frozen input is never mutated by resolution.
        self.assertIsNotNone(live.flash)

    def test_pan_advances_and_snaps(self):
        running = PanAnim(
            current_x=0.0,
            current_y=0.0,
            target_x=100.0,
            target_y=0.0,
            speed=18.0,
            animating=True,
        )
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        first = source.capture(_viewport(), _interaction(pan=running), 1.0)
        # No elapsed time on the first capture: unmoved, still running.
        self.assertEqual(
            (first.animations.pan_x, first.animations.pan_animating),
            (0.0, True),
        )
        second = source.capture(
            _viewport(), _interaction(pan=running), 1.01
        )
        self.assertGreater(second.animations.pan_x, 0.0)
        self.assertLess(second.animations.pan_x, 100.0)
        self.assertTrue(second.animations.pan_animating)
        near = PanAnim(
            current_x=99.8,
            current_y=0.0,
            target_x=100.0,
            target_y=0.0,
            speed=18.0,
            animating=True,
        )
        landed = source.capture(_viewport(), _interaction(pan=near), 1.02)
        self.assertEqual(
            (landed.animations.pan_x, landed.animations.pan_animating),
            (100.0, False),
        )


class FrameContractTest(unittest.TestCase):
    def test_viewport_math(self):
        viewport = _viewport(pan_x=10.0, pan_y=5.0, zoom=2.0)
        self.assertEqual(
            viewport.screen_to_world((20.0, 10.0)), (20.0, 10.0)
        )
        self.assertEqual(
            viewport.world_to_screen((20.0, 10.0)), (20.0, 10.0)
        )
        world = viewport.screen_to_world((37.0, 91.0))
        back = viewport.world_to_screen(world)
        self.assertAlmostEqual(back[0], 37.0)
        self.assertAlmostEqual(back[1], 91.0)
        self.assertEqual(
            viewport.world_bounds, (10.0, 5.0, 410.0, 305.0)
        )

    def test_records_are_frozen(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.revision = 99
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.nodes[0].position = (1.0, 1.0)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.links[0].start = (1.0, 1.0)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.viewport.pan_x = 9.0
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.overlay.status_node_id = "X"
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.hotbox.visible = True
        with self.assertRaises(dataclasses.FrozenInstanceError):
            captured.transform_inputs.generation = 3
        with self.assertRaises(dataclasses.FrozenInstanceError):
            _interaction().show_values = False

    def test_frame_holds_no_live_handles(self):
        node = _make_node("A")
        node._display_color = (0.5, 0.25, 1.0)
        graph = _two_node_graph()
        graph.nodes["A"] = node
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        allowed_records = {
            getattr(frame, name)
            for name in (
                "Viewport", "LinkId", "PortRecord", "ValueInput",
                "GlyphInputs", "NodeRecord", "LinkRecord", "PopupRecord",
                "ValueEditorRecord", "FlashRecord", "OverlayState",
                "HotboxRecord", "TransformOffset", "TransformFrameInputs",
                "PanAnim", "AnimationOutputs", "NodeHitCandidate",
                "LinkHitCandidate", "HitCandidates", "HitParams",
                "ValueCellCandidate", "InteractionState", "Frame", "Hit",
            )
        }
        allowed_scalars = (str, float, int, bool, type(None))

        def check(value, path):
            if isinstance(value, allowed_scalars):
                return
            if isinstance(value, tuple):
                for i, item in enumerate(value):
                    check(item, "%s[%d]" % (path, i))
                return
            self.assertIn(
                type(value), allowed_records, "live handle at %s" % path
            )
            for f in dataclasses.fields(value):
                check(getattr(value, f.name), "%s.%s" % (path, f.name))

        check(captured, "frame")
        check(source.hit((60.0, 10.0)), "hit")
        record = frame.find_node(captured, "A")
        self.assertIsInstance(record.position, tuple)
        self.assertTrue(
            all(isinstance(v, float) for v in record.position)
        )
        self.assertEqual(record.display_color, (0.5, 0.25, 1.0))

    def test_frame_imports_no_model_writers(self):
        with open(frame.__file__, encoding="utf-8") as handle:
            tree = ast.parse(handle.read())
        forbidden = ("model", "selection", "editing")
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""] + [
                    alias.name for alias in node.names
                ]
            else:
                continue
            for name in names:
                tail = name.split(".")[-1]
                self.assertNotIn(
                    tail,
                    forbidden,
                    "frame.py imports a model writer: %s" % name,
                )

    def test_hit_before_first_paint_captures_on_demand(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        self.assertIsNone(source.latest)
        hit = source.hit((60.0, 10.0))
        self.assertEqual((hit.kind, hit.node_id), ("node", "A"))
        self.assertIsNotNone(source.latest)

    def test_hit_without_view_or_frame_raises(self):
        with self.assertRaises(RuntimeError):
            FrameSource(None).hit((0.0, 0.0))

    def test_capture_is_owner_thread_confined(self):
        source = FrameSource(SimpleNamespace(nodeGraph=_two_node_graph()))
        captured = source.capture(_viewport(), _interaction(), 1.0)
        errors = []

        def attempt():
            try:
                source.capture(_viewport(), _interaction(), 2.0)
                errors.append("capture did not raise")
            except RuntimeError:
                pass
            try:
                # Pure frame hits stay thread-safe.
                hit = captured.hit((60.0, 10.0))
                if hit.node_id != "A":
                    errors.append("wrong hit: %r" % (hit,))
            except Exception as error:  # pragma: no cover
                errors.append("frame hit raised: %r" % (error,))

        worker = threading.Thread(target=attempt)
        worker.start()
        worker.join()
        self.assertEqual(errors, [])

    def test_adapters_read_a_stub_view(self):
        view = SimpleNamespace(
            panX=4.0,
            panY=6.0,
            zoom=2.0,
            width=lambda: 400,
            height=300,
            _selection=SimpleNamespace(
                node_ids={"B", "A"}, link_keys=set()
            ),
            nodeIdUnderCursor="A",
            _linkUnderCursor=3,
            _hoveredPort=("A", "out", True),
            _valueHover=("A", "p", 1),
            _cachedLinkLineWidth=10.0,
            _cachedShowAttributeValues=True,
            _cachedValueMinPixelHeight=5.0,
            textChanged=True,
            _currentTimeCode=None,
            _panXAnim=SimpleNamespace(current=1.0, target=9.0, speed=4.0),
            _panYAnim=SimpleNamespace(current=2.0, target=8.0, speed=4.0),
            _panAnimating=True,
            linkDimming=SimpleNamespace(current=0.5),
            marqueeActive=True,
            marqueeStart=(0.0, 0.0),
            marqueeCurrent=(50.0, 40.0),
            _renamingNodeId="B",
            _valueFlash=("A", "p", 2, 999.0, "commit"),
            nodeCreationHotbox=SimpleNamespace(
                is_visible=True, rect=(1.0, 2.0, 3.0, 4.0)
            ),
        )
        viewport = frame.viewport_from_view(view)
        self.assertEqual(
            (viewport.pan_x, viewport.pan_y, viewport.zoom),
            (4.0, 6.0, 2.0),
        )
        self.assertEqual((viewport.width, viewport.height), (400.0, 300.0))
        state = frame.interaction_state_from_view(view)
        self.assertEqual(state.selected_nodes, ("A", "B"))
        self.assertEqual(state.node_under_cursor, "A")
        self.assertEqual(state.link_under_cursor, 3)
        self.assertEqual(state.hovered_port, ("A", "out", True))
        self.assertEqual(state.value_hover, ("A", "p", 1))
        self.assertEqual(state.link_line_width, 10.0)
        self.assertTrue(state.content_changed)
        self.assertEqual(
            (state.pan.current_x, state.pan.target_x, state.pan.speed),
            (1.0, 9.0, 4.0),
        )
        self.assertTrue(state.pan.animating)
        self.assertEqual(state.link_dimming, 0.5)
        self.assertEqual(
            state.overlay.marquee_rect, (0.0, 0.0, 50.0, 40.0)
        )
        self.assertEqual(state.overlay.rename_node_id, "B")
        self.assertEqual(state.overlay.flash.node_id, "A")
        self.assertTrue(state.hotbox.visible)
        legacy = SimpleNamespace(_valueFlash=("A", "p", 999.0, "commit"))
        adapted = frame.interaction_state_from_view(legacy)
        self.assertEqual(
            (adapted.overlay.flash.component, adapted.overlay.flash.kind),
            (0, "commit"),
        )


@unittest.skipUnless(_has_real, "real model stack not available")
class RealModelInteropTest(unittest.TestCase):
    def _stage(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(stage, "/A")
        UsdGeom.Sphere.Define(stage, "/B")
        return stage

    def _node(self, stage, path, x, y):
        node = NodeModel(stage=stage, primPath=Sdf.Path(path))
        node.setDisplayPosition(Gf.Vec2d(x, y))
        node.size = Gf.Vec2d(120.0, 80.0)
        return node

    def _link(self, src, src_port, tgt, tgt_port, x0, y0, x1, y1):
        link = LinkData()
        link.sourceNodeId = src
        link.sourcePort = src_port
        link.targetNodeId = tgt
        link.targetPort = tgt_port
        link.start = Gf.Vec2d(x0, y0)
        link.end = Gf.Vec2d(x1, y1)
        link.data_sourceNodeId = src
        link.data_targetNodeId = tgt
        link.is_input_link = False
        link.propertyOwnerNodeId = ""
        link.propertyName = ""
        link.is_relationship_link = False
        link.sourcePropertyName = ""
        link.targetPropertyName = ""
        link.isDangling = False
        link.danglingDirection = ""
        link.selected = False
        link.hovered = False
        return link

    def _state(self, stage):
        graph = NodeGraph()
        graph.setStage(stage)
        graph.nodes["/A"] = self._node(stage, "/A", 0.0, 0.0)
        graph.nodes["/B"] = self._node(stage, "/B", 200.0, 0.0)
        graph.links.append(
            self._link(
                "/A", "out", "/B", "in", 120.0, 30.0, 200.0, 30.0
            )
        )
        spatial = SpatialIndex(maxDepth=4, maxItems=20)
        return model.GraphModelState(graph, spatial), graph

    def test_capture_and_hit_against_real_state(self):
        state, _graph = self._state(self._stage())
        source = FrameSource(None, state)
        captured = source.capture(_viewport(), _interaction(), 1.0)
        self.assertEqual(captured.revision, 0)
        self.assertEqual(
            [r.node_id for r in captured.nodes], ["/A", "/B"]
        )
        record = frame.find_node(captured, "/A")
        self.assertIsInstance(record.position, tuple)
        self.assertEqual(record.position, (0.0, 0.0))
        self.assertEqual(record.size, (120.0, 80.0))
        hit = captured.hit((160.0, 30.0))
        self.assertEqual((hit.kind, hit.link_index), ("link", 0))
        self.assertEqual(hit.link_id.source_id, "/A")
        self.assertEqual(hit.link_id.target_id, "/B")

    def test_real_apply_shares_unchanged_records(self):
        state, graph = self._state(self._stage())
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        graph.nodes["/B"].setDisplayPosition(Gf.Vec2d(220.0, 10.0))
        graph.links[0].end = Gf.Vec2d(220.0, 40.0)
        state.apply(
            model.GraphDelta(
                ops=(
                    model.UpdateNode("/B"),
                    model.UpdateLink(
                        model.link_key_for(graph.links[0])
                    ),
                )
            )
        )
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual((first.revision, second.revision), (0, 1))
        by_id = lambda fr: {r.node_id: r for r in fr.nodes}
        self.assertIs(by_id(second)["/A"], by_id(first)["/A"])
        self.assertIsNot(by_id(second)["/B"], by_id(first)["/B"])
        self.assertEqual(
            by_id(second)["/B"].position, (220.0, 10.0)
        )
        # The edge incident to the moved node rebuilds.
        self.assertIsNot(second.links[0], first.links[0])

    def test_real_replace_bumps_generation(self):
        stage = self._stage()
        state, _graph = self._state(stage)
        source = FrameSource(None, state)
        first = source.capture(_viewport(), _interaction(), 1.0)
        replacement = NodeGraph()
        replacement.setStage(stage)
        replacement.nodes["/A"] = self._node(stage, "/A", 5.0, 5.0)
        state.replace(replacement)
        self.assertTrue(state.retired)
        spatial = SpatialIndex(maxDepth=4, maxItems=20)
        source.attach(model.GraphModelState(replacement, spatial))
        second = source.capture(_viewport(), _interaction(), 2.0)
        self.assertEqual(
            (first.identity_generation, second.identity_generation),
            (0, 1),
        )
        self.assertEqual([r.node_id for r in second.nodes], ["/A"])
        self.assertEqual(first.hit((60.0, 10.0)).node_id, "/A")


if __name__ == "__main__":
    unittest.main()
