#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview/model layer.

``GraphModelState`` wraps a real ``NodeGraph`` and ``SpatialIndex`` and
every mutation enters through ``apply``: notification counts,
revision/topology_revision across add, move, remove, replace, failed
applies (rollback leaves the graph and both revisions untouched and
publishes nothing) and no-ops, native adjacency and spatial-index
agreement with the collections after each delta, and subscription
isolation across replacement. The stage-sync remap / validate / adopt /
reload hooks enter through the same ``apply`` rather than a second
path. The moved graph/topology functions stay bound on ``GraphView`` by
explicit assignment and resolve patched module names late.

Only OpenUSD and the native noodles module are treated as optional
dependencies here. The modules under test are product code: an
ImportError from one of them is a broken staged package, so it fails
the run instead of skipping the coverage.
"""

from __future__ import annotations

import dataclasses
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

try:
    from pxr import Gf, Sdf, Usd, UsdGeom

    _has_usd = True
except ImportError as error:  # pragma: no cover - OpenUSD not installed
    print("OpenUSD imports failed: %s" % error)
    _has_usd = False

if _has_usd:
    from UsdNoodles.graphview import legacy as legacy_module
    from UsdNoodles.graphview import model
    from UsdNoodles.graphview import stage_adapter, stage_sync

try:
    from UsdNoodles.core import LinkData
    from UsdNoodles.models import NodeModel
    from UsdNoodles.nodeGraph import NodeGraph
    from UsdNoodles.spatial import SpatialIndex

    _has_native = True
except ImportError as error:  # pragma: no cover - native module not built
    print("native noodles imports failed: %s" % error)
    _has_native = False

try:
    from UsdNoodles.graphView import GraphView

    _has_graph_view = True
except ImportError as error:  # pragma: no cover - Qt bridge not installed
    print("graphView imports failed: %s" % error)
    _has_graph_view = False


def _stage_with(*names):
    stage = Usd.Stage.CreateInMemory()
    for name in names:
        UsdGeom.Sphere.Define(stage, "/" + name)
    return stage


def _make_node(stage, path, x=0.0, y=0.0):
    node = NodeModel(stage=stage, primPath=Sdf.Path(path))
    node.setDisplayPosition(Gf.Vec2d(x, y))
    node.size = Gf.Vec2d(120.0, 80.0)
    return node


def _make_link(
    src, src_port, tgt, tgt_port, x0=0.0, y0=0.0, x1=200.0, y1=0.0
):
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


def _make_state(stage):
    graph = NodeGraph()
    graph.setStage(stage)
    spatial = SpatialIndex(maxDepth=4, maxItems=20)
    return model.GraphModelState(graph, spatial), graph, spatial


def _indexed(spatial):
    node_ids, link_indices = spatial.queryRegion(
        Gf.Range2d(Gf.Vec2d(-1e7, -1e7), Gf.Vec2d(1e7, 1e7))
    )
    return set(node_ids), set(link_indices)


def _connected(graph, node_id):
    return set(graph.getConnectedNodeIds(node_id))


class _ModelView:
    """A view double with a real graph pair and recording view state."""

    def __init__(self, stage):
        self.nodeGraph = NodeGraph()
        self.nodeGraph.setStage(stage)
        self._spatialIndex = SpatialIndex(maxDepth=4, maxItems=20)
        self._stage = stage
        self._selectedNodes = set()
        self._marqueeBaseSelectedNodes = set()
        self._marqueePrevSelectedNodes = set()
        self._reconciledPositionVersions = {}
        self.nodeIdUnderCursor = ""
        self._renamingNodeId = ""
        self._valueEditTarget = None
        self._mungState = None
        self._tokenPopup = None
        self._hoveredPort = None
        self._tooltipProperty = None
        self._dragLinkSourceNode = None
        self._dragLinkSyntheticTarget = None
        self._dragLinkResolvedTarget = None
        self._reconnectOldSourceNodeId = None
        self._reconnectOldTargetNodeId = None
        self._currentPrimPath = None
        self._syncingSelection = False
        self._nextZOrder = 0
        self.fontAtlas = None
        self.linksChanged = False
        self.textChanged = False
        self._backReferenceLinkDataCache = None
        self._invalidateBackReferenceLinkDataCache = MagicMock()
        self._nodeRenderManager = MagicMock()
        self._resetRenderCachesAfterBulkMove = MagicMock()
        self._clearRenderCache = MagicMock()
        self.update = MagicMock()
        self.textRenderer = MagicMock()
        self._nodeFactory = SimpleNamespace(
            create_node_from_prim=self._create_node
        )

    @property
    def nodes(self):
        return self.nodeGraph.nodes

    @property
    def links(self):
        return self.nodeGraph.links

    def _create_node(self, prim, stage):
        return _make_node(stage, str(prim.GetPath()))

    def _graphScopeParentPath(self):
        return GraphView._graphScopeParentPath(self)


@unittest.skipUnless(_has_usd and _has_native, "model layer not available")
class ModelStateOpsTest(unittest.TestCase):
    """apply/subscribe semantics over a real graph pair."""

    def setUp(self):
        self.stage = _stage_with("A", "B", "C")
        self.state, self.graph, self.spatial = _make_state(self.stage)
        self.published = []
        self.state.subscribe(self.published.append)

    def test_insert_nodes_publishes_single_changeset(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        changeset = self.state.apply(
            model.GraphDelta(
                ops=(
                    model.InsertNode("/A", node_a),
                    model.InsertNode("/B", node_b),
                )
            )
        )
        self.assertEqual(len(self.published), 1)
        self.assertIs(changeset, self.published[0])
        self.assertEqual(changeset.revision, 1)
        self.assertEqual(changeset.topology_revision, 1)
        self.assertEqual(changeset.changed_nodes, ("/A", "/B"))
        self.assertEqual(changeset.removed_nodes, ())
        self.assertEqual(self.state.revision, 1)
        self.assertEqual(self.state.topology_revision, 1)
        self.assertIs(self.graph.nodes["/A"], node_a)
        self.assertIs(self.graph.nodes["/B"], node_b)
        node_ids, link_indices = _indexed(self.spatial)
        self.assertEqual(node_ids, {"/A", "/B"})
        self.assertEqual(link_indices, set())
        self.assertEqual(_connected(self.graph, "/A"), set())
        with self.assertRaises(dataclasses.FrozenInstanceError):
            changeset.revision = 99

    def test_insert_duplicate_rejected_without_mutation(self):
        node_a = _make_node(self.stage, "/A")
        self.graph.nodes["/A"] = node_a
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
            )
        self.assertEqual(self.state.revision, 0)
        self.assertEqual(self.state.topology_revision, 0)
        self.assertEqual(self.published, [])
        self.assertEqual(list(self.graph.nodes), ["/A"])

    def test_base_revision_guard(self):
        node_a = _make_node(self.stage, "/A")
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(
                    ops=(model.InsertNode("/A", node_a),),
                    base_revision=99,
                )
            )
        self.assertEqual(self.published, [])
        self.assertEqual(self.graph.nodes, {})
        changeset = self.state.apply(
            model.GraphDelta(
                ops=(model.InsertNode("/A", node_a),), base_revision=0
            )
        )
        self.assertEqual(changeset.revision, 1)
        self.assertEqual(len(self.published), 1)

    def test_empty_delta_is_noop(self):
        changeset = self.state.apply(model.GraphDelta())
        self.assertEqual(changeset.revision, 0)
        self.assertEqual(changeset.topology_revision, 0)
        self.assertEqual(changeset.changed_nodes, ())
        self.assertEqual(self.published, [])
        self.assertEqual(self.state.revision, 0)

    def _abc_wired(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        node_c = _make_node(self.stage, "/C", x=600.0)
        link_ab = _make_link("/A", "out", "/B", "in")
        link_bc = _make_link("/B", "out", "/C", "in", x0=300.0, x1=500.0)
        self.state.apply(
            model.GraphDelta(
                ops=(
                    model.InsertNode("/A", node_a),
                    model.InsertNode("/B", node_b),
                    model.InsertNode("/C", node_c),
                    model.InsertLink(link_ab),
                    model.InsertLink(link_bc),
                )
            )
        )
        self.published.clear()
        return node_a, node_b, node_c, link_ab, link_bc

    def test_remove_node_cascades_incident_links(self):
        node_a, _, node_c, link_ab, link_bc = self._abc_wired()
        changeset = self.state.apply(
            model.GraphDelta(ops=(model.RemoveNode("/B"),))
        )
        self.assertEqual(len(self.published), 1)
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(changeset.removed_nodes, ("/B",))
        self.assertEqual(
            set(changeset.removed_links),
            {model.link_key_for(link_ab), model.link_key_for(link_bc)},
        )
        self.assertEqual(list(self.graph.nodes), ["/A", "/C"])
        self.assertEqual(self.graph.links, [])
        self.assertEqual(node_a.outputLinks, [])
        self.assertEqual(node_c.inputLinks, [])
        node_ids, link_indices = _indexed(self.spatial)
        self.assertEqual(node_ids, {"/A", "/C"})
        self.assertEqual(link_indices, set())
        self.assertEqual(_connected(self.graph, "/A"), set())
        self.assertEqual(_connected(self.graph, "/C"), set())

    def test_failed_apply_rolls_back_entire_delta(self):
        node_a, node_b, node_c, link_ab, link_bc = self._abc_wired()
        revision = self.state.revision
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(
                    ops=(
                        model.RemoveNode("/B"),
                        model.RemoveNode("/Missing"),
                    )
                )
            )
        self.assertEqual(self.published, [])
        self.assertEqual(self.state.revision, revision)
        self.assertEqual(self.state.topology_revision, revision)
        self.assertEqual(list(self.graph.nodes), ["/A", "/B", "/C"])
        self.assertIs(self.graph.nodes["/A"], node_a)
        self.assertIs(self.graph.nodes["/B"], node_b)
        self.assertIs(self.graph.nodes["/C"], node_c)
        self.assertEqual(len(self.graph.links), 2)
        self.assertIs(self.graph.links[0], link_ab)
        self.assertIs(self.graph.links[1], link_bc)
        self.assertEqual(node_a.outputLinks, [link_ab])
        self.assertEqual(node_c.inputLinks, [link_bc])
        node_ids, link_indices = _indexed(self.spatial)
        self.assertEqual(node_ids, {"/A", "/B", "/C"})
        self.assertEqual(link_indices, {0, 1})
        self.assertEqual(_connected(self.graph, "/A"), {"/B"})

    def test_move_nodes_rekeys_and_rewrites_endpoints(self):
        node_a, node_b, _, link_ab, _ = self._abc_wired()
        self.stage.DefinePrim("/A2")
        self.stage.RemovePrim("/A")
        self.assertEqual(
            [link.sourceNodeId for link in self.graph.links],
            ["/A", "/B"],
        )
        changeset = self.state.apply(
            model.GraphDelta(
                ops=(model.MoveNodes(pairs=(("/A", "/A2"),)),)
            )
        )
        self.assertEqual(len(self.published), 1)
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(changeset.remaps, (("/A", "/A2"),))
        self.assertEqual(changeset.changed_nodes, ("/A2",))
        self.assertEqual(changeset.removed_nodes, ())
        self.assertEqual(list(self.graph.nodes), ["/A2", "/B", "/C"])
        self.assertIs(self.graph.nodes["/A2"], node_a)
        self.assertEqual(node_a.id, "/A2")
        self.assertEqual(node_a._primPath, Sdf.Path("/A2"))
        self.assertTrue(node_a._prim.IsValid())
        self.assertEqual(link_ab.sourceNodeId, "/A2")
        self.assertEqual(link_ab.data_sourceNodeId, "/A2")
        node_ids, link_indices = _indexed(self.spatial)
        self.assertEqual(node_ids, {"/A2", "/B", "/C"})
        self.assertEqual(link_indices, {0, 1})
        self.assertEqual(_connected(self.graph, "/A2"), {"/B"})
        self.assertEqual(_connected(self.graph, "/A"), set())
        self.assertEqual(len(node_a.outputLinks), 1)
        self.assertIs(node_a.outputLinks[0], link_ab)
        self.assertEqual(len(node_b.inputLinks), 1)
        self.assertIs(node_b.inputLinks[0], link_ab)

    def test_move_chain_rekeys_simultaneously(self):
        self.stage.DefinePrim("/M3")
        graph = self.graph
        node_m1 = _make_node(self.stage, "/M1")
        node_m2 = _make_node(self.stage, "/M2", x=300.0)
        graph.nodes["/M1"] = node_m1
        graph.nodes["/M2"] = node_m2
        changeset = self.state.apply(
            model.GraphDelta(
                ops=(
                    model.MoveNodes(
                        pairs=(("/M1", "/M2"), ("/M2", "/M3"))
                    ),
                )
            )
        )
        self.assertEqual(changeset.remaps, (("/M1", "/M2"), ("/M2", "/M3")))
        self.assertIs(graph.nodes["/M2"], node_m1)
        self.assertIs(graph.nodes["/M3"], node_m2)
        self.assertEqual(node_m1.id, "/M2")
        self.assertEqual(node_m2.id, "/M3")

    def test_move_validates_targets(self):
        self._abc_wired()
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(
                    ops=(model.MoveNodes(pairs=(("/A", "/B"),)),)
                )
            )
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(
                    ops=(model.MoveNodes(pairs=(("/A", "/A"),)),)
                )
            )
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(ops=(model.MoveNodes(pairs=()),))
            )
        self.assertEqual(self.published, [])
        self.assertEqual(list(self.graph.nodes), ["/A", "/B", "/C"])

    def test_update_node_bumps_revision_only(self):
        node_a = _make_node(self.stage, "/A")
        self.state.apply(
            model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
        )
        self.published.clear()
        found, _ = self.spatial.queryPoint(Gf.Vec2d(10.0, 10.0))
        self.assertIn("/A", found)
        node_a.setDisplayPosition(Gf.Vec2d(500.0, 500.0))
        changeset = self.state.apply(
            model.GraphDelta(ops=(model.UpdateNode("/A"),))
        )
        self.assertEqual(len(self.published), 1)
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 1)
        self.assertEqual(changeset.changed_nodes, ("/A",))
        found, _ = self.spatial.queryPoint(Gf.Vec2d(10.0, 10.0))
        self.assertNotIn("/A", found)
        found, _ = self.spatial.queryPoint(Gf.Vec2d(510.0, 510.0))
        self.assertIn("/A", found)
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(ops=(model.UpdateNode("/Missing"),))
            )

    def test_insert_and_remove_link(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        self.state.apply(
            model.GraphDelta(
                ops=(
                    model.InsertNode("/A", node_a),
                    model.InsertNode("/B", node_b),
                )
            )
        )
        link = _make_link("/A", "out", "/B", "in")
        key = model.link_key_for(link)
        self.assertEqual(key.kind, model.KIND_ATTRIBUTE)
        changeset = self.state.apply(
            model.GraphDelta(ops=(model.InsertLink(link),))
        )
        self.assertEqual(changeset.changed_links, (key,))
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(self.graph.links, [link])
        self.assertEqual(node_a.outputLinks, [link])
        self.assertEqual(node_b.inputLinks, [link])
        self.assertTrue(self.graph.isPortConnected("/A", "out", True))
        self.assertTrue(self.graph.isPortConnected("/B", "in", False))
        self.assertEqual(_connected(self.graph, "/A"), {"/B"})
        _, link_indices = _indexed(self.spatial)
        self.assertEqual(link_indices, {0})
        removed = self.state.apply(
            model.GraphDelta(ops=(model.RemoveLink(key),))
        )
        self.assertEqual(removed.removed_links, (key,))
        self.assertEqual(self.graph.links, [])
        self.assertEqual(node_a.outputLinks, [])
        self.assertEqual(node_b.inputLinks, [])
        self.assertFalse(self.graph.isPortConnected("/A", "out", True))
        self.assertEqual(_connected(self.graph, "/A"), set())
        _, link_indices = _indexed(self.spatial)
        self.assertEqual(link_indices, set())
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(
                model.GraphDelta(ops=(model.RemoveLink(key),))
            )

    def test_nested_apply_coalesces_to_one_changeset(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        with self.state.transaction():
            nested_a = self.state.apply(
                model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
            )
            nested_b = self.state.apply(
                model.GraphDelta(ops=(model.InsertNode("/B", node_b),))
            )
            self.assertIsNone(nested_a)
            self.assertIsNone(nested_b)
            self.assertEqual(self.published, [])
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.revision, 1)
        self.assertEqual(changeset.topology_revision, 1)
        self.assertEqual(changeset.changed_nodes, ("/A", "/B"))

    def test_nested_failure_aborts_outer_transaction(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        with self.state.transaction():
            self.state.apply(
                model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
            )
            with self.assertRaises(model.GraphDeltaError):
                self.state.apply(
                    model.GraphDelta(ops=(model.RemoveNode("/Missing"),))
                )
            with self.assertRaises(model.GraphDeltaError):
                self.state.apply(
                    model.GraphDelta(ops=(model.InsertNode("/B", node_b),))
                )
        self.assertEqual(self.published, [])
        self.assertEqual(self.state.revision, 0)
        self.assertEqual(self.state.topology_revision, 0)
        self.assertEqual(self.graph.nodes, {})

    def test_unsubscribe_stops_delivery(self):
        extra = MagicMock()
        unsubscribe = self.state.subscribe(extra)
        unsubscribe()
        unsubscribe()
        node_a = _make_node(self.stage, "/A")
        self.state.apply(
            model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
        )
        extra.assert_not_called()
        self.assertEqual(len(self.published), 1)

    def test_owner_thread_confinement(self):
        node_a = _make_node(self.stage, "/A")
        errors = []

        def _other_thread():
            try:
                self.state.apply(
                    model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
                )
            except RuntimeError as error:
                errors.append(error)
            try:
                self.state.subscribe(lambda _changeset: None)
            except RuntimeError as error:
                errors.append(error)

        worker = threading.Thread(target=_other_thread)
        worker.start()
        worker.join()
        self.assertEqual(len(errors), 2)
        for error in errors:
            self.assertIsInstance(error, RuntimeError)
        self.assertEqual(self.graph.nodes, {})
        self.assertEqual(self.published, [])

    def test_replace_publishes_final_changeset_and_retires(self):
        node_a = _make_node(self.stage, "/A")
        self.state.apply(
            model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
        )
        self.published.clear()
        new_graph = NodeGraph()
        new_graph.setStage(self.stage)
        new_graph.nodes["/X"] = _make_node(self.stage, "/X")
        changeset = self.state.apply(
            model.GraphDelta(ops=(model.Replace(new_graph),))
        )
        self.assertEqual(len(self.published), 1)
        self.assertIs(changeset, self.published[0])
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(changeset.removed_nodes, ("/A",))
        self.assertEqual(changeset.changed_nodes, ("/X",))
        self.assertTrue(self.state.retired)
        with self.assertRaises(model.GraphDeltaError):
            self.state.apply(model.GraphDelta())
        with self.assertRaises(model.GraphDeltaError):
            self.state.subscribe(lambda _changeset: None)

    def test_replace_identical_pair_is_noop(self):
        node_a = _make_node(self.stage, "/A")
        self.state.apply(
            model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
        )
        self.published.clear()
        changeset = self.state.replace(self.graph, self.spatial)
        self.assertEqual(changeset.revision, 1)
        self.assertEqual(changeset.changed_nodes, ())
        self.assertEqual(self.published, [])
        self.assertFalse(self.state.retired)

    def test_replaced_graph_subscription_sees_new_graph_only(self):
        node_a = _make_node(self.stage, "/A")
        self.state.apply(
            model.GraphDelta(ops=(model.InsertNode("/A", node_a),))
        )
        new_graph = NodeGraph()
        new_graph.setStage(self.stage)
        new_graph.nodes["/X"] = _make_node(self.stage, "/X")
        self.state.replace(new_graph)
        self.assertEqual(len(self.published), 2)
        fresh = model.GraphModelState(new_graph, self.spatial)
        seen = []
        fresh.subscribe(seen.append)
        node_y = _make_node(self.stage, "/C", x=300.0)
        changeset = fresh.apply(
            model.GraphDelta(ops=(model.InsertNode("/Y", node_y),))
        )
        self.assertEqual(changeset.revision, 1)
        self.assertEqual(changeset.changed_nodes, ("/Y",))
        self.assertEqual(len(seen), 1)
        self.assertEqual(len(self.published), 2)
        node_ids, _ = _indexed(self.spatial)
        self.assertEqual(node_ids, {"/X", "/Y"})


@unittest.skipUnless(_has_usd and _has_native, "model layer not available")
class StageSyncApplyTest(unittest.TestCase):
    """The reload/remap hooks enter through apply, not a second path."""

    def setUp(self):
        self.stage = _stage_with("A", "B")
        self.view = _ModelView(self.stage)
        self.state = model.state_for_view(self.view)
        self.published = []
        self.state.subscribe(self.published.append)

    def _wire_ab(self):
        node_a = _make_node(self.stage, "/A")
        node_b = _make_node(self.stage, "/B", x=300.0)
        link = _make_link("/A", "out", "/B", "in")
        self.state.apply(
            model.GraphDelta(
                ops=(
                    model.InsertNode("/A", node_a),
                    model.InsertNode("/B", node_b),
                    model.InsertLink(link),
                )
            )
        )
        self.published.clear()
        return node_a, node_b, link

    def test_remap_hook_publishes_one_changeset(self):
        node_a, _, link = self._wire_ab()
        self.view._selectedNodes = {"/A"}
        self.stage.DefinePrim("/A2")
        self.stage.RemovePrim("/A")
        remapped = stage_adapter.remapMovedNodes(self.view, {"/A": "/A2"})
        self.assertEqual(remapped, {"/A": "/A2"})
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(changeset.remaps, (("/A", "/A2"),))
        self.assertEqual(changeset.changed_nodes, ("/A2",))
        self.assertEqual(list(self.view.nodes), ["/A2", "/B"])
        self.assertIs(self.view.nodes["/A2"], node_a)
        self.assertEqual(link.sourceNodeId, "/A2")
        self.assertEqual(self.view._selectedNodes, {"/A2"})
        node_ids, link_indices = _indexed(self.view._spatialIndex)
        self.assertEqual(node_ids, {"/A2", "/B"})
        self.assertEqual(link_indices, {0})
        self.assertEqual(_connected(self.view.nodeGraph, "/A2"), {"/B"})

    def test_validate_hook_drops_stale_through_apply(self):
        node_a, _, link = self._wire_ab()
        self.view._selectedNodes = {"/A", "/B"}
        self.stage.RemovePrim("/B")
        removed = stage_adapter.validateNodesAgainstStage(self.view)
        self.assertEqual(removed, ["/B"])
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.removed_nodes, ("/B",))
        self.assertEqual(
            changeset.removed_links, (model.link_key_for(link),)
        )
        self.assertEqual(list(self.view.nodes), ["/A"])
        self.assertEqual(self.view.links, [])
        self.assertEqual(node_a.outputLinks, [])
        self.assertEqual(self.view._selectedNodes, {"/A"})

    @unittest.skipUnless(_has_graph_view, "graphView module not available")
    def test_adopt_hook_inserts_through_apply(self):
        self._wire_ab()
        UsdGeom.Sphere.Define(self.stage, "/New")
        added = stage_adapter.adoptPrimsNewToScope(self.view, {"/New"})
        self.assertEqual(added, ["/New"])
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.revision, 2)
        self.assertEqual(changeset.topology_revision, 2)
        self.assertEqual(changeset.changed_nodes, ("/New",))
        self.assertIn("/New", self.view.nodes)
        node_ids, _ = _indexed(self.view._spatialIndex)
        self.assertIn("/New", node_ids)

    @unittest.skipUnless(_has_graph_view, "graphView module not available")
    def test_notice_pass_coalesces_to_single_changeset(self):
        self._wire_ab()
        self.stage.RemovePrim("/B")
        UsdGeom.Sphere.Define(self.stage, "/New")
        stage_adapter.applyUsdChanges(
            self.view, {"/B", "/New"}, set(), None
        )
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.removed_nodes, ("/B",))
        self.assertEqual(changeset.changed_nodes, ("/New",))
        self.assertEqual(
            sorted(self.view.nodes), ["/A", "/New"]
        )
        self.assertTrue(self.view.linksChanged)

    def test_stub_spatial_index_keeps_direct_path(self):
        self.view._spatialIndex = MagicMock()
        add_fast = MagicMock(return_value="/New")
        self.view._addNewNodeFast = add_fast
        self.assertIsNone(model.state_for_view(self.view))
        UsdGeom.Sphere.Define(self.stage, "/New")
        added = stage_adapter.adoptPrimsNewToScope(self.view, {"/New"})
        self.assertEqual(added, ["/New"])
        add_fast.assert_called_once()
        self.assertEqual(self.published, [])

    def test_reload_installs_fresh_state_and_publishes_replace(self):
        node_a, _, _ = self._wire_ab()
        old_graph = self.view.nodeGraph
        new_graph = NodeGraph()
        new_graph.setStage(self.stage)
        new_graph.nodes["/X"] = _make_node(self.stage, "/X")
        self.view.nodeGraph = new_graph
        fresh = model.note_graph_replaced(self.view, self.state, old_graph)
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.removed_nodes, ("/A", "/B"))
        self.assertEqual(changeset.changed_nodes, ("/X",))
        self.assertTrue(self.state.retired)
        self.assertIsNotNone(fresh)
        self.assertIsNot(fresh, self.state)
        self.assertIs(fresh.graph, new_graph)
        self.assertEqual(fresh.revision, 0)
        self.assertIs(self.view._graphModelState, fresh)
        self.assertIs(node_a, old_graph.nodes["/A"])

    def test_load_delta_publishes_replace_through_service_hook(self):
        self._wire_ab()
        new_graph = NodeGraph()
        new_graph.setStage(self.stage)
        new_graph.nodes["/X"] = _make_node(self.stage, "/X")
        self.view.loadStage = lambda _stage: setattr(
            self.view, "nodeGraph", new_graph
        )
        stage_adapter.applyStageDelta(
            self.view,
            stage_sync.Load(3, self.stage, stage_sync.MODE_STAGE, None),
        )
        self.assertEqual(len(self.published), 1)
        changeset = self.published[0]
        self.assertEqual(changeset.removed_nodes, ("/A", "/B"))
        self.assertEqual(changeset.changed_nodes, ("/X",))
        self.assertTrue(self.state.retired)
        fresh = self.view._graphModelState
        self.assertIsNot(fresh, self.state)
        self.assertIs(fresh.graph, new_graph)


@unittest.skipUnless(
    _has_usd and _has_graph_view, "graphView module not available"
)
class MovedBindingsTest(unittest.TestCase):
    """The moved methods stay bound by explicit assignment."""

    _MOVED = {
        "_addBackReferenceDanglingLinks": "add_back_reference_dangling_links",
        "_rebuildLinks": "rebuild_links",
        "_processNodeLinks": "process_node_links",
        "_addNodeFromDanglingLink": "add_node_from_dangling_link",
        "_addLinkToGraph": "add_link_to_graph",
        "_removeLinkFromGraph": "remove_link_from_graph",
        "_surfaceConnectionEndpointPins": (
            "surface_connection_endpoint_pins"
        ),
        "_addNewNodeFast": "add_new_node_fast",
        "_classifyRelationshipLinkFromUsd": (
            "classify_relationship_link_from_usd"
        ),
        "_reconcileMovedNodes": "reconcile_moved_nodes",
        "_updateLinksForMovedNodes": "update_links_for_moved_nodes",
        "_collectSiblingDanglingLinks": "collect_sibling_dangling_links",
        "_addPrimAsNode": "add_prim_as_node",
        "_setupDanglingLinkEndpoint": "setup_dangling_link_endpoint",
        "_getLinksForNodeDirection": "get_links_for_node_direction",
        "_getLinkEndpointPosition": "get_link_endpoint_position",
        "_updateNodeLinkEndpoints": "update_node_link_endpoints",
        "_syncLinksIfNeeded": "sync_links_if_needed",
        "_setupLinkEndpoints": "setup_link_endpoints",
        "_hasExactLink": "has_exact_link",
        "_resolveLinkDisplayPinName": "resolve_link_display_pin_name",
        "_findLinkToInput": "find_link_to_input",
        "_makeLinkFromUsdData": "make_link_from_usd_data",
        "_addNodesFromDanglingLink": "add_nodes_from_dangling_link",
        "_populateNodeLinksFromPrim": "populate_node_links_from_prim",
        "_invalidateBackReferenceLinkDataCache": (
            "invalidate_back_reference_link_data_cache"
        ),
        "_getLinksForNodeFlat": "get_links_for_node_flat",
        "_resolvePortPropertyName": "resolve_port_property_name",
        "_findPinIndex": "find_pin_index",
        "_computeLinkBounds": "compute_link_bounds",
        "_rebuildConnectionCache": "rebuild_connection_cache",
    }

    def test_moved_count(self):
        self.assertEqual(len(self._MOVED), 31)

    def test_explicit_assignment_not_forwarding(self):
        for attribute, function in sorted(self._MOVED.items()):
            with self.subTest(attribute=attribute):
                bound = getattr(GraphView, attribute)
                owned = getattr(model, function)
                self.assertIs(bound, owned)

    def test_legacy_symbol_reads_current_binding(self):
        real = legacy_module.legacy_symbol("collect_links_for_prim")
        self.assertTrue(callable(real))
        sentinel = MagicMock()
        with patch(
            "UsdNoodles.graphView.collect_links_for_prim", sentinel
        ):
            self.assertIs(
                legacy_module.legacy_symbol("collect_links_for_prim"),
                sentinel,
            )
        self.assertIs(
            legacy_module.legacy_symbol("collect_links_for_prim"), real
        )
        with self.assertRaises(AttributeError):
            legacy_module.legacy_symbol("no_such_symbol_xyz")


if __name__ == "__main__":
    unittest.main()
