#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

try:
    from pxr import Gf
    from UsdNoodles.graphView import GraphView, _makeStageSync
    from UsdNoodles.nodeGraphBlueprint import (
        _append_relationship_links as append_blueprint_relationship_links,
    )
    from UsdNoodles.nodeGraphBpContainer import (
        _append_relationship_links as append_container_relationship_links,
    )
    from UsdNoodles.nodeGraphStage import (
        _append_collected_links as append_stage_collected_links,
    )
    from UsdNoodles.pinUtils import RELATIONSHIP_LINK_BASE_COLOR
    from pxr.Usdviewq.qt import QtCore
    from _relationshipFixtures import (
        Projection,
        back_ref_link_data,
        back_ref_stage,
        collect_record,
        fake_node_vertex,
        graph_link,
        make_arrowhead_view,
        make_back_ref_view,
        make_box_node,
        make_change_block_view,
        make_complete_drag_view,
        make_connection_view,
        make_dangling_link,
        make_edge_drag_node,
        make_edge_drag_view,
        make_endpoint_node,
        make_endpoint_pin_link,
        make_endpoint_view,
        make_graph_node,
        make_link_click_link,
        make_link_click_view,
        make_link_drag_view,
        make_mouse_press_event,
        make_node_expansion_view,
        make_node_prim,
        make_paint_links_view,
        make_pinnable_node,
        make_prim_tree_view,
        make_regular_link,
        make_relationship_link,
        make_row_hover_node,
        make_row_hover_view,
        make_row_metrics_renderer,
        make_row_metrics_view,
        make_row_renderer,
        make_sdf_prim,
        make_stage_at_path,
        make_surface_view,
        make_temp_link_render_view,
        make_temp_link_triangle_view,
        make_typed_row_node,
        patched_collect_links,
        patched_graph_symbol,
        patched_module_collect_links,
    )

    _has_graph = True
except ImportError as e:
    print(f"relationship graph imports failed: {e}")
    _has_graph = False


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class GraphViewRelationshipHelpersTest(unittest.TestCase):
    def test_populate_node_links_from_prim_copies_metadata(self):
        node = SimpleNamespace(inputLinks=["stale"], outputLinks=["stale"])
        view = SimpleNamespace(
            _makeLinkFromUsdData=GraphView._makeLinkFromUsdData,
        )

        records = [
            collect_record("/Constraint", "sources", "/Driver", ""),
            collect_record("/Constraint", "affects", "/Driven", ""),
        ]
        with patched_collect_links(return_value=records):
            GraphView._populateNodeLinksFromPrim(view, node, MagicMock())

        self.assertEqual(len(node.inputLinks), 0)
        self.assertEqual(len(node.outputLinks), 2)
        self.assertEqual(node.outputLinks[0].propertyName, "sources")
        self.assertEqual(node.outputLinks[0].sourcePropertyName, "sources")
        self.assertEqual(node.outputLinks[1].sourcePropertyName, "affects")
        self.assertTrue(node.outputLinks[0].is_relationship_link)

    def test_rebuild_links_uses_shared_population_helper(self):
        prim = MagicMock()
        prim.IsValid.return_value = True
        node = SimpleNamespace(
            id="/Node",
            _prim=prim,
            inputLinks=[],
            outputLinks=[],
            position=Gf.Vec2d(0.0, 0.0),
            size=Gf.Vec2d(10.0, 10.0),
        )
        view = SimpleNamespace(
            links=[],
            nodes={"/Node": node},
            nodeGraph=SimpleNamespace(syncLinksFromModels=MagicMock()),
            _populateNodeLinksFromPrim=MagicMock(),
            _surfaceConnectionEndpointPins=MagicMock(),
            _processNodeLinks=MagicMock(),
            _addBackReferenceDanglingLinks=MagicMock(),
            _rebuildConnectionCache=MagicMock(),
            _spatialIndex=SimpleNamespace(
                clear=MagicMock(),
                insertNode=MagicMock(),
                insertLink=MagicMock(),
            ),
            _computeLinkBounds=MagicMock(return_value=MagicMock()),
            _profiler=MagicMock(),
        )

        GraphView._rebuildLinks(view)

        view._populateNodeLinksFromPrim.assert_called_once_with(node, prim)
        view._surfaceConnectionEndpointPins.assert_called_once()
        view._processNodeLinks.assert_called_once_with(node)
        view._addBackReferenceDanglingLinks.assert_called_once()
        # The freshly-rebuilt links are synced into the C++ View Model before the
        # connection cache is rebuilt, so the cache reflects the current links.
        view.nodeGraph.syncLinksFromModels.assert_called_once_with(view.links)

    def test_add_back_reference_swallows_collect_links_runtime_error(self):
        """RuntimeError from collect_links_for_prim must not surface; loop continues."""
        stage = back_ref_stage("/A")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        with patched_collect_links(side_effect=RuntimeError("usd boom")):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])
        # Cache populated to {} so we don't re-traverse on the next call.
        self.assertEqual(view._backReferenceLinkDataCache, {})

    def test_add_back_reference_swallows_collect_links_type_error(self):
        stage = back_ref_stage("/A")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        with patched_collect_links(side_effect=TypeError("bad type")):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])

    def test_add_back_reference_swallows_collect_links_attribute_error(self):
        stage = back_ref_stage("/A")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        with patched_collect_links(side_effect=AttributeError("no attr")):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])

    def test_add_back_reference_present_is_source_branch(self):
        """Absent prim's link with present-prim source → output-side dangling link."""
        stage = back_ref_stage("/Absent")
        present_node = SimpleNamespace(id="/Present")
        view = make_back_ref_view(stage, nodes={"/Present": present_node})

        link_data = back_ref_link_data(
            "/Present",
            "out",
            "/Absent",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[link_data]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(len(view.links), 1)
        added = view.links[0]
        self.assertFalse(added.is_input_link)
        self.assertEqual(added.data_sourceNodeId, "/Present")
        self.assertEqual(added.data_targetNodeId, "/Absent")
        view._setupDanglingLinkEndpoint.assert_called_once_with(
            added, present_node, "out", isOutput=True
        )

    def test_add_back_reference_present_is_target_branch(self):
        """Absent prim's link with present-prim target → input-side dangling link."""
        stage = back_ref_stage("/Absent")
        present_node = SimpleNamespace(id="/Present")
        view = make_back_ref_view(stage, nodes={"/Present": present_node})

        link_data = back_ref_link_data(
            "/Absent",
            "out",
            "/Present",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[link_data]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(len(view.links), 1)
        added = view.links[0]
        self.assertTrue(added.is_input_link)
        self.assertEqual(added.data_sourceNodeId, "/Absent")
        self.assertEqual(added.data_targetNodeId, "/Present")
        view._setupDanglingLinkEndpoint.assert_called_once_with(
            added, present_node, "inp", isOutput=False
        )

    def test_add_back_reference_skips_when_both_endpoints_absent(self):
        """Links between two absent prims are not surfaced."""
        stage = back_ref_stage("/Absent")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        link_data = back_ref_link_data(
            "/OtherAbsent",
            "out",
            "/Absent",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[link_data]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])

    def test_add_back_reference_dedups_against_existing_link(self):
        """A back-ref matching an already-added link's full key is skipped."""
        stage = back_ref_stage("/Absent")
        present_node = SimpleNamespace(id="/Present")
        # Pre-seed an existing forward-direction link with the same key.
        existing = graph_link("/Present", "out", "/Absent", "inp",
                              source_property="outputs:out",
                              target_property="inputs:inp")
        view = make_back_ref_view(
            stage, nodes={"/Present": present_node}, links=[existing]
        )

        link_data = back_ref_link_data(
            "/Present",
            "out",
            "/Absent",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[link_data]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [existing])
        view._setupDanglingLinkEndpoint.assert_not_called()

    def test_add_back_reference_skips_present_prim_at_use_time(self):
        """The use-time loop skips prims in self.nodes; forward pass handled them."""
        present_prim = MagicMock()
        present_prim.GetPath.return_value = "/Present"
        stage = MagicMock()
        stage.Traverse.return_value = [present_prim]
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        # Return non-empty data so the present prim's path IS cached, then
        # verify the use-time loop still skips it (no _setupDangling call,
        # no link appended) because membership is checked per call, not per
        # cache build.
        link_data = back_ref_link_data(
            "/Present",
            "out",
            "/Other",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[link_data]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])
        view._setupDanglingLinkEndpoint.assert_not_called()
        self.assertIn("/Present", view._backReferenceLinkDataCache)

    def test_add_back_reference_no_op_when_stage_missing(self):
        view = SimpleNamespace(
            links=[],
            nodes={},
            nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=None)),
            _backReferenceLinkDataCache=None,
        )
        GraphView._addBackReferenceDanglingLinks(view)
        self.assertEqual(view.links, [])
        # Cache untouched when there's no stage to scan.
        self.assertIsNone(view._backReferenceLinkDataCache)

    def test_add_back_reference_reuses_cache_across_calls(self):
        """Second call must NOT re-traverse the stage (cache is reused)."""
        stage = back_ref_stage("/Absent")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        collect = MagicMock(return_value=[])
        with patch("UsdNoodles.graphView.collect_links_for_prim", collect):
            GraphView._addBackReferenceDanglingLinks(view)
            first_traverse_count = stage.Traverse.call_count
            first_collect_count = collect.call_count
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(stage.Traverse.call_count, first_traverse_count)
        self.assertEqual(collect.call_count, first_collect_count)

    def test_invalidate_back_reference_cache_forces_retraversal(self):
        """After invalidation, the next call must re-traverse."""
        stage = back_ref_stage("/Absent")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        with patched_collect_links(return_value=[]):
            GraphView._addBackReferenceDanglingLinks(view)
            GraphView._invalidateBackReferenceLinkDataCache(view)
            self.assertIsNone(view._backReferenceLinkDataCache)
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(stage.Traverse.call_count, 2)

    def test_add_back_reference_shared_absent_prim_to_multiple_present(self):
        """A single absent prim with links to two different present prims
        produces two distinct back-reference dangling links."""
        stage = back_ref_stage("/Absent")
        present_a = SimpleNamespace(id="/PresentA")
        present_b = SimpleNamespace(id="/PresentB")
        view = make_back_ref_view(
            stage, nodes={"/PresentA": present_a, "/PresentB": present_b}
        )

        link_to_a = back_ref_link_data(
            "/PresentA",
            "out",
            "/Absent",
            "inp_a",
            source_property="outputs:out",
            target_property="inputs:inp_a",
        )
        link_to_b = back_ref_link_data(
            "/PresentB",
            "out",
            "/Absent",
            "inp_b",
            source_property="outputs:out",
            target_property="inputs:inp_b",
        )
        with patched_collect_links(return_value=[link_to_a, link_to_b]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(len(view.links), 2)
        present_nodes_used = {
            id(call.args[1]) for call in view._setupDanglingLinkEndpoint.call_args_list
        }
        self.assertEqual(present_nodes_used, {id(present_a), id(present_b)})

    def test_add_back_reference_self_referencing_absent_prim_is_skipped(self):
        """An absent prim whose connection points at itself (both endpoints
        absent) is not surfaced — there is no present-prim port to attach to."""
        stage = back_ref_stage("/Absent")
        view = make_back_ref_view(stage, nodes={"/Present": SimpleNamespace()})

        self_link = back_ref_link_data(
            "/Absent",
            "out",
            "/Absent",
            "inp",
            source_property="outputs:out",
            target_property="inputs:inp",
        )
        with patched_collect_links(return_value=[self_link]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [])
        view._setupDanglingLinkEndpoint.assert_not_called()

    def test_add_back_reference_self_referencing_present_prim_skipped_by_forward_pass(
        self,
    ):
        """A present prim that self-references is fully handled by the forward
        pass; the back-ref scan must not double-add. We verify by including a
        present-prim self-link in the existing self.links keyset — the scan
        sees the absent prim's view of any data and must dedup correctly."""
        stage = back_ref_stage("/Absent")
        present = SimpleNamespace(id="/Present")
        # forward pass already added the present prim's self-link
        existing_self = graph_link("/Present", "out", "/Present", "inp",
                                   source_property="outputs:out",
                                   target_property="inputs:inp")
        view = make_back_ref_view(
            stage, nodes={"/Present": present}, links=[existing_self]
        )

        # Absent prim has no links of its own; back-ref must add nothing.
        with patched_collect_links(return_value=[]):
            GraphView._addBackReferenceDanglingLinks(view)

        self.assertEqual(view.links, [existing_self])
        view._setupDanglingLinkEndpoint.assert_not_called()

    def test_create_connection_preserves_relationship_property_metadata(self):
        view, library, output_node, input_node = make_connection_view()

        with patched_graph_symbol("_push_undo_command"):
            GraphView._createConnection(
                view,
                "/ConstraintA",
                "sources",
                "/ConstraintB",
                "affects",
                sourcePropertyName="sources",
                targetPropertyName="affects",
            )

        library.create_connection.assert_called_once()
        view._addLinkToGraph.assert_called_once_with(
            "/ConstraintA",
            "sources",
            "/ConstraintB",
            "affects",
            output_node,
            input_node,
            sourcePropertyName="sources",
            targetPropertyName="affects",
        )

    def test_reconnect_continues_when_delete_old_returns_false(self):
        """When _deleteOldConnection returns False (implicit replacement by
        create_connection), the reconnect must still succeed: link added,
        undo pushed, no abort."""
        view, _library, _output_node, _input_node = make_connection_view(
            delete_old_returns=False
        )

        with patched_graph_symbol("_push_undo_command") as mock_push:
            GraphView._createConnection(
                view,
                "/C",
                "out",
                "/D",
                "inp",
                reconnect_state=("/A", "out", "/B", "inp"),
            )

        view._addLinkToGraph.assert_called_once()
        view._abortConnection.assert_not_called()
        view._clearReconnectState.assert_called_once()
        mock_push.assert_called_once()
        self.assertEqual(mock_push.call_args[0][0], "Reconnect")

    def test_create_connection_invalidates_back_reference_cache(self):






        view, _library, _output_node, _input_node = make_connection_view(
            show_popup=False
        )

        with patched_graph_symbol("_push_undo_command"):
            GraphView._createConnection(
                view,
                "/A",
                "material:binding",
                "/B",
                "",
                sourcePropertyName="material:binding",
                targetPropertyName="",
            )

        view._invalidateBackReferenceLinkDataCache.assert_called_once()

    def test_delete_connection_invalidates_back_reference_cache(self):





        library = MagicMock()
        library.delete_connection.return_value = True
        output_prim = MagicMock()
        output_prim.IsValid.return_value = True
        input_prim = MagicMock()
        input_prim.IsValid.return_value = True
        output_node = SimpleNamespace(
            outputLinks=[],
            getUsdPrim=MagicMock(return_value=output_prim),
        )
        input_node = SimpleNamespace(
            inputLinks=[],
            getUsdPrim=MagicMock(return_value=input_prim),
        )
        stage = MagicMock()
        view = SimpleNamespace(
            nodes={"/A": output_node, "/B": input_node},
            nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=stage)),
            _findLibraryForPrim=MagicMock(return_value=library),
            _stageSync=_makeStageSync(None),
            _removeLinkFromGraph=MagicMock(),
            _invalidateBackReferenceLinkDataCache=MagicMock(),
        )

        with patched_graph_symbol("_push_undo_command"):
            GraphView._deleteConnection(
                view,
                "/A",
                "material:binding",
                "/B",
                "",
                sourcePropertyName="material:binding",
                targetPropertyName="",
            )

        library.delete_connection.assert_called_once()
        view._invalidateBackReferenceLinkDataCache.assert_called_once()

    def test_add_node_from_dangling_link_uses_population_helper(self):
        prim = make_node_prim("Missing", type_name="RigJoint")
        node = SimpleNamespace(id="/Missing", inputLinks=[], outputLinks=[], zOrder=0)
        view = make_node_expansion_view(
            stage=make_stage_at_path(prim),
            create_node_from_prim=MagicMock(return_value=node),
            nodes={},
            viewport_anchor=(10.0, 20.0),
            real_placement=False,
        )
        link = SimpleNamespace(
            isDangling=True,
            data_sourceNodeId="/Missing",
            data_targetNodeId="",
        )

        result = GraphView._addNodeFromDanglingLink(view, link)

        self.assertTrue(result)
        view._populateNodeLinksFromPrim.assert_called_once_with(node, prim)
        self.assertIn("/Missing", view.nodes)
        # Expansion places by connection like the add-node path (not the
        # viewport center): positioning and z-order are required for the
        # renderer to compute pin positions reliably, which in turn lets
        # _rebuildLinks resolve the noodle endpoints on the next paint
        # instead of leaving them at the dangling stub.
        view._placeAddedNodes.assert_called_once_with(
            [node], fallback=(10.0, 20.0)
        )
        self.assertEqual(view._nextZOrder, 6)
        self.assertEqual(node.zOrder, 6)

    def test_add_node_from_dangling_link_lands_beside_present_node(self):
        # End to end through the real _placeAddedNodes/place_node: expanding
        # a relationship target must land one column to the RIGHT of the
        # double-clicked source node, not at the viewport anchor.
        present = SimpleNamespace(
            id="/Mesh",
            position=[0.0, 0.0],
            size=[200.0, 100.0],
            inputLinks=[],
            outputLinks=[
                SimpleNamespace(sourceNodeId="/Mesh", targetNodeId="/Mat")
            ],
            setDisplayPosition=MagicMock(),
        )
        placed = {}

        def _record_position(pos):
            placed["pos"] = (float(pos[0]), float(pos[1]))
            node.position = [placed["pos"][0], placed["pos"][1]]

        node = SimpleNamespace(
            id="/Mat",
            position=[0.0, 0.0],
            size=[200.0, 100.0],
            inputLinks=[
                SimpleNamespace(sourceNodeId="/Mesh", targetNodeId="/Mat")
            ],
            outputLinks=[],
            setDisplayPosition=MagicMock(side_effect=_record_position),
        )
        prim = make_node_prim("Mat", type_name="Material")
        view = make_node_expansion_view(
            stage=make_stage_at_path(prim),
            create_node_from_prim=MagicMock(return_value=node),
            nodes={"/Mesh": present},
            viewport_anchor=(5000.0, 5000.0),
        )
        link = SimpleNamespace(
            isDangling=True,
            data_sourceNodeId="/Mesh",
            data_targetNodeId="/Mat",
        )

        result = GraphView._addNodeFromDanglingLink(view, link)

        self.assertTrue(result)
        self.assertIn("pos", placed)
        # Right of the source's right edge, vertically aligned with it --
        # and nowhere near the (5000, 5000) viewport-anchor fallback.
        self.assertGreaterEqual(placed["pos"][0], 200.0)
        self.assertLess(placed["pos"][0], 5000.0)
        self.assertLess(abs(placed["pos"][1] - 0.0), 5000.0)

    def test_add_nodes_from_dangling_link_staggers_siblings(self):
        # A 3-target relationship expansion must stagger the siblings so no
        # two expanded nodes overlap, via the real placement helper.
        present = SimpleNamespace(
            id="/Mesh",
            position=[0.0, 0.0],
            size=[200.0, 300.0],
            inputLinks=[],
            outputLinks=[
                SimpleNamespace(
                    sourceNodeId="/Mesh", targetNodeId=f"/Mat{i}"
                )
                for i in range(3)
            ],
            setDisplayPosition=MagicMock(),
        )
        created = {}

        def _make_node(prim, _stage):
            name = prim.GetName()
            node = SimpleNamespace(
                id=f"/{name}",
                position=[0.0, 0.0],
                size=[200.0, 100.0],
                inputLinks=[
                    SimpleNamespace(
                        sourceNodeId="/Mesh", targetNodeId=f"/{name}"
                    )
                ],
                outputLinks=[],
            )
            node.setDisplayPosition = MagicMock(
                side_effect=lambda pos, _n=node: _n.position.__setitem__(
                    slice(None), [float(pos[0]), float(pos[1])]
                )
            )
            created[node.id] = node
            return node

        view = make_node_expansion_view(
            stage=make_stage_at_path(
                side_effect=lambda path: make_node_prim(
                    str(path).rsplit("/", 1)[-1], type_name="Material"
                )
            ),
            create_node_from_prim=MagicMock(side_effect=_make_node),
            nodes={"/Mesh": present},
            viewport_anchor=(5000.0, 5000.0),
            links=[],
        )
        view._collectSiblingDanglingLinks = lambda seed: (
            GraphView._collectSiblingDanglingLinks(view, seed)
        )
        view._addNodeFromDanglingLink = lambda link: (
            GraphView._addNodeFromDanglingLink(view, link)
        )
        view.links = [
            make_dangling_link(
                "/Mesh", "material:binding", f"/Mat{i}", with_endpoints=True
            )
            for i in range(3)
        ]

        result = GraphView._addNodesFromDanglingLink(view, view.links[0])

        self.assertTrue(result)
        self.assertEqual(len(created), 3)
        rects = [
            (n.position[0], n.position[1], n.size[0], n.size[1])
            for n in created.values()
        ]
        for i in range(3):
            for j in range(i + 1, 3):
                ax, ay, aw, ah = rects[i]
                bx, by, bw, bh = rects[j]
                overlaps = (
                    ax < bx + bw and ax + aw > bx
                    and ay < by + bh and ay + ah > by
                )
                self.assertFalse(
                    overlaps, f"sibling {i} overlaps sibling {j}: {rects}"
                )
        # All three sit right of the source, not at the viewport fallback.
        for x, _y, _w, _h in rects:
            self.assertGreaterEqual(x, 200.0)
            self.assertLess(x, 5000.0)

    def test_collect_sibling_dangling_links_groups_by_present_source_port(self):
        # A USD relationship with three targets becomes three dangling
        # LinkData objects that all share (sourceNodeId, sourcePort) on the
        # present side.  Expansion needs every sibling, not just the first
        # one a port-hit search returns.
        seed = make_dangling_link("/Mesh", "material:binding", "/Mat1")
        sibling_b = make_dangling_link("/Mesh", "material:binding", "/Mat2")
        sibling_c = make_dangling_link("/Mesh", "material:binding", "/Mat3")
        # A dangling link on a DIFFERENT relationship on the same prim must
        # not be folded into this expansion.
        unrelated = make_dangling_link("/Mesh", "proxyPrim", "/Proxy")
        non_dangling = make_dangling_link(
            "/Mesh", "material:binding", "/Mat4", is_dangling=False
        )
        view = SimpleNamespace(
            links=[non_dangling, seed, sibling_b, unrelated, sibling_c],
            nodes={"/Mesh": SimpleNamespace()},
        )

        siblings = GraphView._collectSiblingDanglingLinks(view, seed)

        self.assertEqual(siblings, [seed, sibling_b, sibling_c])

    def test_add_nodes_from_dangling_link_expands_all_relationship_targets(self):
        # Regression: expanding a relationship with multiple targets must
        # bring in every missing target, not just the first one
        # ``_findDanglingLinkAtPort`` happens to return.
        present_node = SimpleNamespace()
        seed = make_dangling_link("/Mesh", "material:binding", "/Mat1")
        sibling_b = make_dangling_link("/Mesh", "material:binding", "/Mat2")
        sibling_c = make_dangling_link("/Mesh", "material:binding", "/Mat3")
        view = SimpleNamespace(
            links=[seed, sibling_b, sibling_c],
            nodes={"/Mesh": present_node},
            _addNodeFromDanglingLink=MagicMock(return_value=True),
            _collectSiblingDanglingLinks=lambda link: GraphView._collectSiblingDanglingLinks(
                view, link
            ),
        )

        result = GraphView._addNodesFromDanglingLink(view, seed)

        self.assertTrue(result)
        self.assertEqual(view._addNodeFromDanglingLink.call_count, 3)
        called_links = [
            call.args[0] for call in view._addNodeFromDanglingLink.call_args_list
        ]
        self.assertEqual(called_links, [seed, sibling_b, sibling_c])

    def test_find_dangling_link_at_port_hits_present_endpoint(self):
        renderer = SimpleNamespace(getPortWidth=MagicMock(return_value=8.0))
        node = SimpleNamespace(renderer=renderer)
        present_link = make_dangling_link(
            "/Missing", "out", "/Present", "inp", with_properties=False
        )
        non_dangling_link = make_dangling_link(
            "/Present",
            "out",
            "/Other",
            "inp",
            with_properties=False,
            is_dangling=False,
        )
        view = SimpleNamespace(
            links=[non_dangling_link, present_link],
            nodes={"/Present": node},
            defaultRenderer=renderer,
            _cachedPortHitRadiusMultiplier=1.5,
            _getPortPosition=MagicMock(return_value=Gf.Vec2d(100.0, 50.0)),
        )

        idx = GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(101.0, 51.0))
        self.assertEqual(idx, 1)
        view._getPortPosition.assert_called_with(node, "inp", False)

        miss_idx = GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(200.0, 200.0))
        self.assertEqual(miss_idx, -1)

    def test_find_dangling_link_at_port_skips_when_both_endpoints_missing(self):
        view = SimpleNamespace(
            links=[
                make_dangling_link(
                    "/MissingA",
                    "out",
                    "/MissingB",
                    "inp",
                    with_properties=False,
                )
            ],
            nodes={},
            defaultRenderer=None,
            _cachedPortHitRadiusMultiplier=1.5,
            _getPortPosition=MagicMock(),
        )

        self.assertEqual(
            GraphView._findDanglingLinkAtPort(view, Gf.Vec2d(0.0, 0.0)),
            -1,
        )

    def test_add_prim_as_node_uses_population_helper(self):
        prim = MagicMock()
        prim.GetPath.return_value = "/Node"
        node = SimpleNamespace(
            id="/Node",
            inputLinks=[],
            outputLinks=[],
            position=Gf.Vec2d(0.0, 0.0),
            size=Gf.Vec2d(20.0, 10.0),
            zOrder=0,
        )
        view = SimpleNamespace(
            nodes={},
            _nodeFactory=SimpleNamespace(
                create_node_from_prim=MagicMock(return_value=node),
            ),
            _populateNodeLinksFromPrim=MagicMock(),
            fontAtlas=None,
            nodeGraph=SimpleNamespace(_calculateNodeSize=MagicMock()),
            textRenderer=SimpleNamespace(calculateTextWidth=MagicMock()),
            _positionNodeInViewport=MagicMock(),
            _nextZOrder=0,
        )
        added_node_ids = []

        result = GraphView._addPrimAsNode(view, prim, MagicMock(), added_node_ids)

        self.assertTrue(result)
        view._populateNodeLinksFromPrim.assert_called_once_with(node, prim)
        self.assertEqual(added_node_ids, ["/Node"])

    def test_add_nodes_from_prim_tree_selection_processes_all_selected(self):
        driver_prim = make_sdf_prim("/Scope/DriverNode")
        driven_prim = make_sdf_prim("/Scope/DrivenNode")
        view, added_paths = make_prim_tree_view([driver_prim, driven_prim])

        GraphView.addNodesFromPrimTreeSelection(view)

        self.assertEqual(added_paths, ["/Scope/DriverNode", "/Scope/DrivenNode"])
        view._showPopupMessage.assert_called_with("Added 2 node(s)")
        view._frameNodeBounds.assert_called_once()

    def test_add_nodes_from_prim_tree_selection_deduplicates(self):
        prim = make_sdf_prim("/Scope/Node")
        view, added_paths = make_prim_tree_view([prim, prim])

        GraphView.addNodesFromPrimTreeSelection(view)

        self.assertEqual(added_paths, ["/Scope/Node"])

    def test_add_nodes_from_prim_tree_selection_uses_cached_when_richer(self):
        # Live API returns only one prim (e.g. focus narrowed),
        # but the cached selection captured both.
        driver_prim = make_sdf_prim("/Scope/DriverNode")
        driven_prim = make_sdf_prim("/Scope/DrivenNode")
        view, added_paths = make_prim_tree_view(
            [driver_prim], last_selection=[driver_prim, driven_prim]
        )

        GraphView.addNodesFromPrimTreeSelection(view)

        self.assertEqual(
            sorted(added_paths),
            ["/Scope/DrivenNode", "/Scope/DriverNode"],
        )

    def test_adding_into_an_existing_graph_leaves_the_camera_alone(self):
        """Adding a prim adds a node. It does not re-aim the camera.

        Reframing on every add throws away the pan and zoom the user chose,
        mid-task, which is only tolerable when there was no view to lose. The
        batch is anchored in the viewport instead, so it is visible without
        moving anything.
        """
        prim = make_sdf_prim("/Scope/NewNode")
        existing = make_graph_node("/Scope/Existing", position=(4000.0, 9000.0))
        view, _added_paths = make_prim_tree_view(
            [prim],
            nodes={"/Scope/Existing": existing},
            viewport_anchor=(120.0, 240.0),
        )

        GraphView.addNodesFromPrimTreeSelection(view)

        self.assertIn("/Scope/NewNode", view.nodes)
        # Liveness first: the method wraps its whole body in a broad
        # try/except, so a raise anywhere before the framing block would make
        # the assertion below pass without the code ever reaching it. This
        # popup is emitted AFTER the framing block.
        view._showPopupMessage.assert_called_with("Added 1 node(s)")
        view._frameNodeBounds.assert_not_called()
        # ...and the batch is placed where the camera is looking, not below
        # content that may be an entire screen away.
        view._viewportAnchor.assert_called_once()
        self.assertEqual(
            view._placeAddedNodes.call_args.kwargs["fallback"], (120.0, 240.0)
        )

    def test_a_node_that_lands_off_screen_is_still_framed(self):
        """Add placement only moves nodes still at the USD default (0, 0). A
        prim carrying an authored ui:nodegraph:node:pos goes back to wherever
        it was last dropped -- possibly nowhere near the current view -- and
        "the camera never moves" would then mean "added, invisible, and
        unfindable". Visibility, not emptiness, is the gate.
        """
        prim = make_sdf_prim("/Scope/FarAway")
        existing = make_graph_node("/Scope/Existing")
        view, _added_paths = make_prim_tree_view(
            [prim],
            nodes={"/Scope/Existing": existing},
            # An authored position, thousands of units from the camera, and
            # outside the viewport.
            added_position=(40000.0, 40000.0),
            is_visible=False,
        )

        GraphView.addNodesFromPrimTreeSelection(view)

        view._showPopupMessage.assert_called_with("Added 1 node(s)")
        view._frameNodeBounds.assert_called_once()
        # ...and it framed the node that was actually added.
        framed = view._frameNodeBounds.call_args[0][0]
        self.assertEqual(tuple(framed.GetMin()), (40000.0, 40000.0))

    def test_update_link_drag_rejects_regular_to_relationship_hover(self):
        temp_link = SimpleNamespace(start=None, end=None)
        view = make_link_drag_view(
            temp_link,
            source_port="translate",
            source_node="/Driver",
            nodes={"constraint": SimpleNamespace()},
            valid_target=True,
            synthetic_target=("old", "sources", False),
            resolved_target=("old", "sources", False, "sources"),
            hovered_port=("constraint", "sources", True),
            _getPortPosition=MagicMock(return_value=Gf.Vec2d(5.0, 6.0)),
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(0, 0))

        self.assertFalse(view._dragLinkValidTarget)
        self.assertIsNone(view._dragLinkSyntheticTarget)
        self.assertIsNone(view._dragLinkResolvedTarget)

    def test_update_link_drag_creates_prim_target_for_relationship_pin(self):
        temp_link = SimpleNamespace(start=None, end=None, is_relationship_link=True)
        view = make_link_drag_view(
            temp_link,
            source_port="sources",
            source_node="/Constraint",
            nodes={"driver": make_box_node()},
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            node_id_under_cursor="driver",
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(1, 2))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertEqual(view._dragLinkSyntheticTarget, ("driver", "", False, ""))
        self.assertEqual(temp_link.end, Gf.Vec2d(25.0, 12.0))

    def test_render_temporary_link_insets_prim_target_noodle_to_arrow_base(self):
        """A snapped whole-prim relationship drag draws the preview noodle to the
        arrowhead base (node_top.y - arrow_length), not the tip, then restores the
        true node top so the arrowhead/hit geometry stay anchored to it. The
        committed path gets this inset from the shader (isPrimTarget); the per-call
        renderLinks path used for the preview does not, so it is applied here."""
        node_top = Gf.Vec2d(25.0, 100.0)
        temp_link = SimpleNamespace(
            start=Gf.Vec2d(0.0, 0.0),
            end=Gf.Vec2d(node_top[0], node_top[1]),
            is_relationship_link=True,
        )
        view, captured = make_temp_link_render_view(temp_link, valid_target=True)

        GraphView._renderTemporaryLink(view)

        arrow_length = max(12.0 / max(view.zoom, 1e-6), 4.0)
        self.assertEqual(
            captured["free_end"], (node_top[0], node_top[1] - arrow_length)
        )
        self.assertEqual((temp_link.end[0], temp_link.end[1]), (25.0, 100.0))

    def test_render_temporary_link_does_not_inset_when_unsnapped(self):
        """With no valid target the preview noodle follows the cursor to its full
        end (the arrowhead faces sideways, not down)."""
        cursor = Gf.Vec2d(40.0, 70.0)
        temp_link = SimpleNamespace(
            start=Gf.Vec2d(0.0, 0.0),
            end=Gf.Vec2d(cursor[0], cursor[1]),
            is_relationship_link=True,
        )
        view, captured = make_temp_link_render_view(temp_link, valid_target=False)

        GraphView._renderTemporaryLink(view)

        self.assertEqual(captured["free_end"], (40.0, 70.0))
        self.assertEqual((temp_link.end[0], temp_link.end[1]), (40.0, 70.0))

    def test_update_link_drag_allows_self_prim_target_for_relationship_pin(self):
        temp_link = SimpleNamespace(start=None, end=None, is_relationship_link=True)
        view = make_link_drag_view(
            temp_link,
            source_port="sources",
            source_node="/Constraint",
            nodes={"/Constraint": make_box_node()},
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            node_id_under_cursor="/Constraint",
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(1, 2))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertEqual(view._dragLinkSyntheticTarget, ("/Constraint", "", False, ""))
        self.assertEqual(temp_link.end, Gf.Vec2d(25.0, 12.0))

    def test_update_link_drag_prefers_property_target_over_prim_target(self):
        temp_link = SimpleNamespace(
            start=Gf.Vec2d(40.0, 50.0),
            end=Gf.Vec2d(40.0, 50.0),
            sourceNodeId="/Constraint",
            sourcePort="sources",
            targetNodeId="",
            targetPort="",
            sourcePropertyName="sources",
            targetPropertyName="",
            is_relationship_link=True,
        )
        view = make_link_drag_view(
            temp_link,
            source_port="sources",
            source_node="/Constraint",
            nodes={
                "/Constraint": make_box_node(
                    position=(10.0, 20.0), size=(60.0, 90.0)
                )
            },
            hovered_port=("/Constraint", "sourceTranslations", False),
            _resolvePortPropertyName=MagicMock(
                return_value="inputs:sourceTranslations"
            ),
            _getLinkEndpointPosition=MagicMock(return_value=Gf.Vec2d(10.0, 72.0)),
            node_id_under_cursor="/Constraint",
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(10, 72))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertIsNone(view._dragLinkSyntheticTarget)
        self.assertEqual(
            view._dragLinkResolvedTarget,
            ("/Constraint", "sourceTranslations", False, "inputs:sourceTranslations"),
        )
        self.assertEqual(temp_link.targetNodeId, "/Constraint")
        self.assertEqual(temp_link.targetPort, "sourceTranslations")
        self.assertEqual(temp_link.targetPropertyName, "inputs:sourceTranslations")
        self.assertEqual(temp_link.end, Gf.Vec2d(10.0, 72.0))

    def test_update_link_drag_recovers_prim_target_when_border_misses_node_hit(self):
        temp_link = SimpleNamespace(start=None, end=None, is_relationship_link=True)
        view = make_link_drag_view(
            temp_link,
            source_port="affects",
            source_node="/Constraint",
            nodes={"driver": make_box_node()},
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            _getNodeIdAtWorldPos=MagicMock(return_value="driver"),
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(25, 12))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertEqual(view._dragLinkSyntheticTarget, ("driver", "", False, ""))
        self.assertEqual(temp_link.end, Gf.Vec2d(25.0, 12.0))
        view._getNodeIdAtWorldPos.assert_called_once()

    def test_update_link_drag_accepts_any_property_for_relationship_pin(self):
        temp_link = SimpleNamespace(start=None, end=None, is_relationship_link=True)
        view = make_link_drag_view(
            temp_link,
            source_port="sources",
            source_node="/Constraint",
            nodes={"driver": SimpleNamespace()},
            hovered_port=("driver", "translate", False),
            _resolvePortPropertyName=MagicMock(return_value="inputs:translate"),
            _getLinkEndpointPosition=MagicMock(return_value=Gf.Vec2d(7.0, 9.0)),
            node_id_under_cursor="driver",
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(1, 2))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertEqual(
            view._dragLinkResolvedTarget,
            ("driver", "translate", False, "inputs:translate"),
        )
        self.assertEqual(temp_link.end, Gf.Vec2d(7.0, 9.0))

    def test_update_link_drag_accepts_visible_relationship_pin_as_target(self):
        temp_link = SimpleNamespace(start=None, end=None, is_relationship_link=True)
        view = make_link_drag_view(
            temp_link,
            source_port="sources",
            source_node="/ConstraintA",
            nodes={"constraintB": SimpleNamespace()},
            hovered_port=("constraintB", "affects", True),
            _resolvePortPropertyName=MagicMock(return_value="affects"),
            _getLinkEndpointPosition=MagicMock(return_value=Gf.Vec2d(11.0, 13.0)),
            node_id_under_cursor="constraintB",
        )

        GraphView._updateLinkDrag(view, QtCore.QPoint(1, 2))

        self.assertTrue(view._dragLinkValidTarget)
        self.assertEqual(
            view._dragLinkResolvedTarget,
            ("constraintB", "affects", False, "affects"),
        )
        self.assertEqual(temp_link.end, Gf.Vec2d(11.0, 13.0))
        self.assertTrue(temp_link.is_relationship_link)
        self.assertEqual(temp_link.targetPropertyName, "affects")

    def test_start_link_drag_marks_temp_link_as_relationship(self):
        temp_link = SimpleNamespace(
            sourceNodeId="",
            targetNodeId="",
            sourcePort="",
            targetPort="",
            sourcePropertyName="",
            targetPropertyName="",
            is_relationship_link=False,
            start=None,
            end=None,
        )
        node = SimpleNamespace(is_relationship_pin=MagicMock(return_value=True))
        view = SimpleNamespace(
            _draggingLink=False,
            _dragLinkSourceNode=None,
            _dragLinkSourcePort=None,
            _dragLinkSourceIsOutput=None,
            _dragLinkSyntheticTarget=("old", "", False),
            _dragLinkResolvedTarget=("old", "", False, ""),
            _dragLinkTempLink=temp_link,
            nodes={"/Constraint": node},
            _getPortPosition=MagicMock(return_value=Gf.Vec2d(7.0, 9.0)),
        )

        GraphView._startLinkDrag(view, "/Constraint", "sources", True)

        self.assertTrue(view._draggingLink)
        self.assertTrue(temp_link.is_relationship_link)
        self.assertEqual(temp_link.sourcePropertyName, "sources")
        self.assertEqual(temp_link.sourcePort, "sources")
        self.assertEqual(temp_link.start, Gf.Vec2d(7.0, 9.0))

    def test_add_link_to_graph_sets_relationship_state_before_endpoint_setup(self):
        setup_states = []
        output_node = SimpleNamespace(outputLinks=[])
        input_node = SimpleNamespace(inputLinks=[])
        view = SimpleNamespace(
            _setupLinkEndpoints=MagicMock(
                side_effect=lambda link, *_args: setup_states.append(
                    (
                        getattr(link, "is_relationship_link", False),
                        str(getattr(link, "sourcePropertyName", "")),
                        str(getattr(link, "targetPropertyName", "")),
                    )
                )
            ),
            links=[],
            _spatialIndex=SimpleNamespace(insertLink=MagicMock()),
            _computeLinkBounds=MagicMock(return_value=MagicMock()),
            _rebuildConnectionCache=MagicMock(),
            _classifyRelationshipLinkFromUsd=staticmethod(
                GraphView._classifyRelationshipLinkFromUsd
            ),
        )

        GraphView._addLinkToGraph(
            view,
            "/ConstraintA",
            "sources",
            "/ConstraintB",
            "affects",
            output_node,
            input_node,
            sourcePropertyName="sources",
            targetPropertyName="affects",
        )

        self.assertEqual(setup_states, [(True, "sources", "affects")])
        self.assertEqual(len(view.links), 1)
        self.assertTrue(view.links[0].is_relationship_link)
        self.assertEqual(view.links[0].propertyName, "sources")

    def test_add_link_to_graph_marks_unknown_relationship_via_usd_fallback(self):
        # Regression: after a reroute drop, an unknown-name relationship like
        # material:binding was being classified as a regular attribute link
        # because get_relationship_link_metadata only recognises the hardcoded
        # sources/affects fast-path names.  That left the persistent prim-target
        # arrowhead unrendered until the next USD-driven _rebuildLinks fired,
        # so the user saw the noodle "ending under the prim" with no arrow on
        # top.  _addLinkToGraph now falls back to a live prim.GetRelationship
        # probe so the link is correctly classified at insert time.
        rel = SimpleNamespace(IsValid=MagicMock(return_value=True))
        source_prim = MagicMock()
        source_prim.IsValid.return_value = True
        source_prim.GetRelationship.return_value = rel
        target_prim = MagicMock()
        target_prim.IsValid.return_value = False
        output_node = SimpleNamespace(
            id="/MeshBody",
            outputLinks=[],
            getUsdPrim=MagicMock(return_value=source_prim),
        )
        input_node = SimpleNamespace(
            id="/Material",
            inputLinks=[],
            getUsdPrim=MagicMock(return_value=target_prim),
        )
        setup_states = []
        view = SimpleNamespace(
            _setupLinkEndpoints=MagicMock(
                side_effect=lambda link, *_args: setup_states.append(
                    getattr(link, "is_relationship_link", False)
                )
            ),
            links=[],
            _spatialIndex=SimpleNamespace(insertLink=MagicMock()),
            _computeLinkBounds=MagicMock(return_value=MagicMock()),
            _rebuildConnectionCache=MagicMock(),
            _classifyRelationshipLinkFromUsd=staticmethod(
                GraphView._classifyRelationshipLinkFromUsd
            ),
        )

        GraphView._addLinkToGraph(
            view,
            "/MeshBody",
            "material:binding",
            "/Material",
            "",
            output_node,
            input_node,
            sourcePropertyName="material:binding",
            targetPropertyName="",
        )

        source_prim.GetRelationship.assert_called_with("material:binding")
        self.assertEqual(setup_states, [True])
        self.assertEqual(len(view.links), 1)
        link = view.links[0]
        self.assertTrue(link.is_relationship_link)
        self.assertEqual(link.propertyOwnerNodeId, "/MeshBody")
        self.assertEqual(link.propertyName, "material:binding")

    def test_update_hovered_port_accepts_relationship_row_hit_during_drag(self):
        node = make_row_hover_node(make_row_renderer())
        view = make_row_hover_view(
            node, port_position=MagicMock(return_value=Gf.Vec2d(130.0, 56.0))
        )

        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(60, 50))

        self.assertTrue(changed)
        self.assertEqual(view._hoveredPort, ("constraintB", "affects", True))

    def test_update_hovered_port_accepts_property_row_hit_during_relationship_drag(
        self,
    ):
        node = make_row_hover_node(
            make_row_renderer(),
            size=(120.0, 100.0),
            input_pins=["inputs", "sourceTranslations"],
            input_row_kinds=[2, 3],
            input_row_slots=[0, 1],
            output_row_slots=[2],
        )
        view = make_row_hover_view(
            node,
            relationship_row_hover=MagicMock(return_value=None),
            property_row_hover=True,
            port_position=MagicMock(
                side_effect=lambda _node_arg, pin_name, is_output: (
                    Gf.Vec2d(10.0, 75.0)
                    if pin_name == "sourceTranslations" and not is_output
                    else (
                        Gf.Vec2d(130.0, 93.0)
                        if pin_name == "affects" and is_output
                        else None
                    )
                )
            ),
        )

        changed = GraphView._updateHoveredPort(view, QtCore.QPoint(40, 75))

        self.assertTrue(changed)
        self.assertEqual(
            view._hoveredPort,
            ("constraintB", "sourceTranslations", False),
        )

    def test_get_link_endpoint_position_uses_top_center_for_prim_target(self):
        node = SimpleNamespace(
            position=Gf.Vec2d(10.0, 20.0),
            size=Gf.Vec2d(30.0, 40.0),
        )
        view = SimpleNamespace(
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            _resolveLinkDisplayPinName=MagicMock(return_value=""),
            _getPropertyOutputSide=GraphView._getPropertyOutputSide,
            _getPortPosition=MagicMock(),
        )

        position = GraphView._getLinkEndpointPosition(view, node, "", False, "")

        self.assertEqual(position, Gf.Vec2d(25.0, 20.0))

    def test_get_link_endpoint_position_mirrors_property_row_across_node(self):
        node = SimpleNamespace(
            position=Gf.Vec2d(10.0, 20.0),
            size=Gf.Vec2d(40.0, 30.0),
        )
        view = SimpleNamespace(
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            _resolveLinkDisplayPinName=MagicMock(return_value="translate"),
            _getPropertyOutputSide=GraphView._getPropertyOutputSide,
            _mirrorPortPosition=GraphView._mirrorPortPosition,
            _getPortPosition=MagicMock(return_value=Gf.Vec2d(10.0, 35.0)),
        )

        position = GraphView._getLinkEndpointPosition(
            view,
            node,
            "translate",
            True,
            "inputs:translate",
        )

        self.assertEqual(position, Gf.Vec2d(50.0, 35.0))

    def test_get_link_endpoint_position_uses_property_side_when_folded_names_overlap(
        self,
    ):
        node = SimpleNamespace(
            position=Gf.Vec2d(10.0, 20.0),
            size=Gf.Vec2d(40.0, 30.0),
            _prim=None,
        )
        node.resolve_pin_name = MagicMock(
            side_effect=lambda pin_name, is_output=None: (
                "outputs" if is_output else "inputs"
            )
        )
        view = SimpleNamespace(
            _getNodeTopCenterPosition=GraphView._getNodeTopCenterPosition,
            _getPropertyOutputSide=GraphView._getPropertyOutputSide,
            _mirrorPortPosition=GraphView._mirrorPortPosition,
            _resolveLinkDisplayPinName=lambda n, p, prop, o: (
                GraphView._resolveLinkDisplayPinName(view, n, p, prop, o)
            ),
            _getPortPosition=MagicMock(
                side_effect=lambda _node_arg, pin_name, is_output: (
                    Gf.Vec2d(50.0, 35.0)
                    if pin_name == "outputs" and is_output
                    else None
                )
            ),
        )

        position = GraphView._getLinkEndpointPosition(
            view,
            node,
            "translation",
            False,
            "outputs:translation",
        )

        self.assertEqual(position, Gf.Vec2d(10.0, 35.0))
        node.resolve_pin_name.assert_called_once_with("translation", is_output=True)

    def test_get_port_position_delegates_to_resolve_port_position(self):
        # _getPortPosition is now a thin shim over the C++ View Model's
        # GraphNodeRenderer.resolvePortPosition (which owns the synthetic / dual /
        # normal geometry; covered by the C++ ResolvePortPositionTest gtests).
        renderer = SimpleNamespace(
            resolvePortPosition=MagicMock(return_value=Gf.Vec2d(50.0, 35.0))
        )
        node = SimpleNamespace(renderer=renderer)
        view = SimpleNamespace(defaultRenderer=None)

        position = GraphView._getPortPosition(view, node, "translation", True)

        self.assertEqual(position, Gf.Vec2d(50.0, 35.0))
        renderer.resolvePortPosition.assert_called_once_with(node, "translation", True)

    def test_get_port_position_falls_back_to_default_renderer(self):
        default_renderer = SimpleNamespace(
            resolvePortPosition=MagicMock(return_value=Gf.Vec2d(1.0, 2.0))
        )
        node = SimpleNamespace(renderer=None)
        view = SimpleNamespace(defaultRenderer=default_renderer)

        position = GraphView._getPortPosition(view, node, "p", False)

        self.assertEqual(position, Gf.Vec2d(1.0, 2.0))
        default_renderer.resolvePortPosition.assert_called_once_with(node, "p", False)

    def test_get_port_position_returns_none_without_a_renderer(self):
        node = SimpleNamespace(renderer=None)
        view = SimpleNamespace(defaultRenderer=None)

        self.assertIsNone(GraphView._getPortPosition(view, node, "x", True))

    def test_get_link_endpoint_position_uses_visible_output_side_for_relationship_source(
        self,
    ):
        node = make_endpoint_node("affects")
        view = make_endpoint_view(node, display_pin="affects")

        position = GraphView._getLinkEndpointPosition(
            view,
            node,
            "affects",
            True,
            "affects",
            preferPropertySide=True,
        )

        self.assertEqual(position, Gf.Vec2d(50.0, 35.0))
        view._getPortPosition.assert_not_called()

    def test_get_link_endpoint_position_uses_left_row_anchor_for_relationship_target(
        self,
    ):
        node = make_endpoint_node("sources")
        view = make_endpoint_view(node, display_pin="sources")

        position = GraphView._getLinkEndpointPosition(
            view,
            node,
            "sources",
            False,
            "sources",
            preferPropertySide=True,
        )

        self.assertEqual(position, Gf.Vec2d(10.0, 35.0))
        view._getPortPosition.assert_not_called()

    def test_get_link_endpoint_position_uses_collapsed_parent_row_for_relationship_target(
        self,
    ):
        node = make_endpoint_node("Group")
        view = make_endpoint_view(node, display_pin="Group")

        position = GraphView._getLinkEndpointPosition(
            view,
            node,
            "sources",
            False,
            "sources",
            preferPropertySide=True,
        )

        self.assertEqual(position, Gf.Vec2d(10.0, 35.0))
        view._getPortPosition.assert_not_called()

    def test_complete_link_drag_creates_relationship_property_link_without_replacement(
        self,
    ):
        view = make_complete_drag_view(
            source_port="affects",
            source_node="/Constraint",
            temp_link=SimpleNamespace(is_relationship_link=True),
            _dragLinkResolvedTarget=(
                "/Driven",
                "translate",
                False,
                "inputs:translate",
            ),
            _hoveredPort=("ignored", "ignored", False),
        )

        GraphView._completeLinkDrag(view)

        view._findLinkToInput.assert_not_called()
        view._createConnection.assert_called_once_with(
            "/Constraint",
            "affects",
            "/Driven",
            "translate",
            sourcePropertyName="affects",
            targetPropertyName="inputs:translate",
        )
        self.assertIsNone(view._dragLinkResolvedTarget)

    def test_complete_link_drag_creates_self_relationship_prim_link(self) -> None:
        view = make_complete_drag_view(
            source_port="sources",
            source_node="/Constraint",
            temp_link=SimpleNamespace(is_relationship_link=True),
            _dragLinkSyntheticTarget=("/Constraint", "", False, ""),
        )

        GraphView._completeLinkDrag(view)

        view._findLinkToInput.assert_not_called()
        view._createConnection.assert_called_once_with(
            "/Constraint",
            "sources",
            "/Constraint",
            "",
            sourcePropertyName="sources",
            targetPropertyName="",
        )
        self.assertIsNone(view._dragLinkSyntheticTarget)

    def test_find_link_to_input_skips_sources_but_keeps_regular_inputs(self):
        regular_link = SimpleNamespace(targetNodeId="/Node", targetPort="weight")
        sources_link = SimpleNamespace(targetNodeId="/Node", targetPort="sources")
        view = SimpleNamespace(links=[sources_link, regular_link])

        self.assertIsNone(GraphView._findLinkToInput(view, "/Node", "sources"))
        self.assertIs(GraphView._findLinkToInput(view, "/Node", "weight"), regular_link)

    def test_render_relationship_arrowheads_draws_triangle_vertices(self):
        link = make_relationship_link()
        view = make_arrowhead_view(link)

        with patched_graph_symbol("NodeVertex", side_effect=fake_node_vertex):
            GraphView._renderRelationshipArrowheads(
                view,
                Projection(),
                (1.0, 1.0, 0.0, 1.0),
                (1.0, 1.0, 0.3, 1.0),
            )

        drawn_vertices = view._drawNodeVertices.call_args.args[0]
        self.assertEqual(len(drawn_vertices), 3)

    def test_navigate_along_relationship_link_jumps_to_target_when_near_source(self):
        link = make_relationship_link()
        # Source-side click — should navigate to the (off-screen) target.
        worldPos = Gf.Vec2d(0.0, 0.0)
        view = SimpleNamespace(
            nodes={"/Constraint": object(), "/Driven": object()},
            _jumpToTargetNodeInternal=MagicMock(),
            _jumpToSourceNodeInternal=MagicMock(),
        )

        result = GraphView._navigateAlongRelationshipLink(view, link, worldPos)

        self.assertTrue(result)
        view._jumpToTargetNodeInternal.assert_called_once_with(link, "/Driven")
        view._jumpToSourceNodeInternal.assert_not_called()

    def test_navigate_along_relationship_link_jumps_to_source_when_near_target(self):
        link = make_relationship_link()
        # Target-side click — should navigate back to the source.
        worldPos = Gf.Vec2d(20.0, 0.0)
        view = SimpleNamespace(
            nodes={"/Constraint": object(), "/Driven": object()},
            _jumpToTargetNodeInternal=MagicMock(),
            _jumpToSourceNodeInternal=MagicMock(),
        )

        result = GraphView._navigateAlongRelationshipLink(view, link, worldPos)

        self.assertTrue(result)
        view._jumpToSourceNodeInternal.assert_called_once_with(link, "/Constraint")
        view._jumpToTargetNodeInternal.assert_not_called()

    def test_navigate_along_relationship_link_falls_back_when_preferred_missing(self):
        link = make_relationship_link()
        # Click is near source so target is preferred; but target node is not
        # present in self.nodes, so we should fall back to the source jump.
        worldPos = Gf.Vec2d(0.0, 0.0)
        view = SimpleNamespace(
            nodes={"/Constraint": object()},
            _jumpToTargetNodeInternal=MagicMock(),
            _jumpToSourceNodeInternal=MagicMock(),
        )

        result = GraphView._navigateAlongRelationshipLink(view, link, worldPos)

        self.assertTrue(result)
        view._jumpToSourceNodeInternal.assert_called_once_with(link, "/Constraint")
        view._jumpToTargetNodeInternal.assert_not_called()

    def test_navigate_along_relationship_link_returns_false_when_no_nodes_known(self):
        link = make_relationship_link()
        view = SimpleNamespace(
            nodes={},
            _jumpToTargetNodeInternal=MagicMock(),
            _jumpToSourceNodeInternal=MagicMock(),
        )

        result = GraphView._navigateAlongRelationshipLink(
            view, link, Gf.Vec2d(10.0, 0.0)
        )

        self.assertFalse(result)
        view._jumpToTargetNodeInternal.assert_not_called()
        view._jumpToSourceNodeInternal.assert_not_called()

    def test_render_relationship_arrowheads_skips_dangling_links(self):
        """Dangling relationship links should NOT draw an arrowhead.

        When the prim on the other end of a relationship is absent from the
        graph, the link is marked dangling. Only the circle indicator should
        be drawn; the noodle body and the arrowhead must both be suppressed
        so the user sees a stub instead of a connection.
        """
        link = make_relationship_link()
        link.isDangling = True
        view = make_arrowhead_view(link)

        with patched_graph_symbol("NodeVertex", side_effect=fake_node_vertex):
            GraphView._renderRelationshipArrowheads(
                view,
                Projection(),
                (1.0, 1.0, 0.0, 1.0),
                (1.0, 1.0, 0.3, 1.0),
            )

        view._drawNodeVertices.assert_not_called()

    def test_relationship_arrowhead_points_down_for_prim_target(self):
        link = make_relationship_link()
        link.targetPort = ""
        link.targetPropertyName = ""
        link.end = Gf.Vec2d(30.0, 18.0)

        tip, base_a, base_b = GraphView._relationshipArrowheadPoints(
            link,
            10.0,
            8.0,
        )

        self.assertEqual(tip, (30.0, 18.0))
        self.assertLess(base_a[1], tip[1])
        self.assertLess(base_b[1], tip[1])
        self.assertLess(base_a[0], tip[0])
        self.assertGreater(base_b[0], tip[0])

    def test_prim_target_relationship_link_detected_for_shader_inset(self):




        link = make_relationship_link()
        link.targetPort = ""
        link.targetPropertyName = ""
        self.assertTrue(GraphView._isPrimTargetRelationshipLink(link))

        link.targetPort = "in"
        self.assertFalse(GraphView._isPrimTargetRelationshipLink(link))

    def test_paint_links_leaves_prim_target_end_unmutated(self):
        # The prim-target end inset moved into the vertex shader, so paintLinks no
        # longer mutates link.end (the old Python prim_target_tips hack). Verify
        # the end is left untouched and rendering goes through the held-vector path
        # (syncLinksFromModels + renderLinksFromGraph), not the per-call renderLinks.
        relationship_link = make_relationship_link()
        relationship_link.targetPort = ""
        relationship_link.targetPropertyName = ""
        relationship_link.end = Gf.Vec2d(30.0, 18.0)

        view, render_from_graph = make_paint_links_view([relationship_link])

        GraphView.paintLinks(view)

        # The wire ribbon's inset is now shader-side; link.end stays the node top.
        self.assertEqual(relationship_link.end, Gf.Vec2d(30.0, 18.0))
        view.nodeGraph.syncLinksFromModels.assert_called_once_with([relationship_link])
        self.assertTrue(render_from_graph.called)

    def test_paint_links_batches_relationship_links_with_red_base_color(self):
        relationship_link = make_relationship_link()
        regular_link = make_regular_link()
        view, render_from_graph = make_paint_links_view(
            [regular_link, relationship_link], height=60
        )

        GraphView.paintLinks(view)

        # The C++ side partitions the held vector; the relationship pass is keyed
        # by relationshipOnly=True with the relationship base color, once per
        # drawSelected pass.
        relationship_calls = [
            call
            for call in render_from_graph.call_args_list
            if call.kwargs.get("relationshipOnly") is True
            and call.kwargs.get("baseColor") == list(RELATIONSHIP_LINK_BASE_COLOR)
        ]
        self.assertEqual(len(relationship_calls), 2)
        view.nodeGraph.syncLinksFromModels.assert_called_once_with(
            [regular_link, relationship_link]
        )
        view._renderRelationshipArrowheads.assert_called_once()

    def test_has_exact_link_returns_true_for_duplicate(self):
        existing = SimpleNamespace(
            sourceNodeId="/Constraint",
            sourcePort="sources",
            targetNodeId="/Driven",
            targetPort="",
            sourcePropertyName="sources",
            targetPropertyName="",
        )
        view = SimpleNamespace(links=[existing])

        self.assertTrue(
            GraphView._hasExactLink(
                view, "/Constraint", "sources", "/Driven", "", "sources", ""
            )
        )

    def test_has_exact_link_returns_false_for_different_property(self):
        existing = SimpleNamespace(
            sourceNodeId="/Constraint",
            sourcePort="sources",
            targetNodeId="/Driven",
            targetPort="",
            sourcePropertyName="sources",
            targetPropertyName="",
        )
        view = SimpleNamespace(links=[existing])

        self.assertFalse(
            GraphView._hasExactLink(
                view, "/Constraint", "sources", "/Driven", "", "OTHER_PROP", ""
            )
        )

    def test_has_exact_link_returns_false_for_empty_links(self):
        view = SimpleNamespace(links=[])

        self.assertFalse(GraphView._hasExactLink(view, "/A", "out", "/B", "in"))

    def test_complete_link_drag_rejects_duplicate_relationship_link(self):
        view = make_complete_drag_view(
            source_port="affects",
            source_node="/Constraint",
            has_exact=True,
            _dragLinkResolvedTarget=(
                "/Driven",
                "translate",
                False,
                "inputs:translate",
            ),
            _hoveredPort=("ignored", "ignored", False),
        )

        GraphView._completeLinkDrag(view)

        view._createConnection.assert_not_called()

    def test_complete_link_drag_cancels_reconnect_on_duplicate(self):
        view = make_complete_drag_view(
            source_port="affects",
            source_node="/Constraint",
            has_exact=True,
            _dragLinkResolvedTarget=(
                "/Driven",
                "translate",
                False,
                "inputs:translate",
            ),
            _hoveredPort=("ignored", "ignored", False),
            _cancelLinkReconnect=MagicMock(),
            _reconnectingLink=True,
            _reconnectOldSourceNodeId="/OldConstraint",
            _reconnectOldSourcePort="old_port",
            _reconnectOldTargetNodeId="/OldDriven",
            _reconnectOldTargetPort="old_target",
        )

        GraphView._completeLinkDrag(view)

        view._createConnection.assert_not_called()
        view._cancelLinkReconnect.assert_called_once()

    def test_cancel_link_reconnect_restores_property_names(self):
        srcNode = SimpleNamespace()
        tgtNode = SimpleNamespace()
        view = SimpleNamespace(
            _reconnectingLink=True,
            _reconnectOldSourceNodeId="/Constraint",
            _reconnectOldSourcePort="sources",
            _reconnectOldTargetNodeId="/Driven",
            _reconnectOldTargetPort="",
            _reconnectOldSourcePropertyName="sources",
            _reconnectOldTargetPropertyName="",
            _clearReconnectState=MagicMock(),
            nodes={"/Constraint": srcNode, "/Driven": tgtNode},
            _addLinkToGraph=MagicMock(),
        )

        GraphView._cancelLinkReconnect(view)

        view._addLinkToGraph.assert_called_once_with(
            "/Constraint",
            "sources",
            "/Driven",
            "",
            srcNode,
            tgtNode,
            sourcePropertyName="sources",
            targetPropertyName="",
        )

    def test_start_link_reconnect_saves_property_names(self):
        link = make_relationship_link()
        view = SimpleNamespace(
            links=[link],
            nodes={"/Constraint": SimpleNamespace()},
            _reconnectingLink=False,
            _reconnectOldSourceNodeId=None,
            _reconnectOldSourcePort=None,
            _reconnectOldTargetNodeId=None,
            _reconnectOldTargetPort=None,
            _reconnectOldSourcePropertyName="",
            _reconnectOldTargetPropertyName="",
            _removeLinkFromGraph=MagicMock(),
            _startLinkDrag=MagicMock(),
            _cancelLinkReconnect=MagicMock(),
            _dragLinkTempLink=SimpleNamespace(start=None, end=None),
        )

        GraphView._startLinkReconnect(view, 0, (10.0, 5.0))

        self.assertEqual(view._reconnectOldSourcePropertyName, "affects")
        self.assertEqual(view._reconnectOldTargetPropertyName, "")

    def test_render_temporary_link_triangle_faces_down_when_valid(self):
        view, render_tri = make_temp_link_triangle_view(
            end_pos=Gf.Vec2d(50.0, 30.0),
            start_pos=Gf.Vec2d(10.0, 10.0),
            valid_target=True,
        )
        GraphView._renderTemporaryLink(view)
        render_tri.assert_called_once()
        args = render_tri.call_args[0]
        tip_y = args[1]
        base1_y = args[3]
        base2_y = args[5]
        self.assertGreater(tip_y, base1_y)
        self.assertGreater(tip_y, base2_y)

    def test_render_temporary_link_triangle_tip_sits_on_prim_top_edge(self):






        view, render_tri = make_temp_link_triangle_view(
            end_pos=Gf.Vec2d(50.0, 30.0),
            start_pos=Gf.Vec2d(10.0, 10.0),
            valid_target=True,
        )
        GraphView._renderTemporaryLink(view)
        args = render_tri.call_args[0]
        tip_x = args[0]
        tip_y = args[1]
        self.assertEqual(tip_x, 50.0)
        self.assertEqual(tip_y, 30.0)

    def test_render_temporary_link_triangle_faces_right_when_dx_positive(self):
        view, render_tri = make_temp_link_triangle_view(
            end_pos=Gf.Vec2d(50.0, 30.0),
            start_pos=Gf.Vec2d(10.0, 30.0),
            valid_target=False,
        )
        GraphView._renderTemporaryLink(view)
        render_tri.assert_called_once()
        args = render_tri.call_args[0]
        tip_x = args[0]
        base1_x = args[2]
        base2_x = args[4]
        self.assertGreater(tip_x, base1_x)
        self.assertGreater(tip_x, base2_x)

    def test_render_temporary_link_triangle_faces_left_when_dx_negative(self):
        view, render_tri = make_temp_link_triangle_view(
            end_pos=Gf.Vec2d(10.0, 30.0),
            start_pos=Gf.Vec2d(50.0, 30.0),
            valid_target=False,
        )
        GraphView._renderTemporaryLink(view)
        render_tri.assert_called_once()
        args = render_tri.call_args[0]
        tip_x = args[0]
        base1_x = args[2]
        base2_x = args[4]
        self.assertLess(tip_x, base1_x)
        self.assertLess(tip_x, base2_x)

    def test_find_connection_authoring_layer_returns_first_match_when_two_layers_author(
        self,
    ):
        layer_a = SimpleNamespace(identifier="/root.usda")
        layer_b = SimpleNamespace(identifier="/overlay.usda")
        stage = MagicMock()
        stage.GetLayerStack.return_value = [layer_a, layer_b]
        input_prim = MagicMock()
        input_prim.IsValid.return_value = True
        input_prim.GetPath.return_value = "/NodeB"
        output_prim = MagicMock()
        output_prim.IsValid.return_value = True
        output_prim.GetPath.return_value = "/NodeA"
        attr_paths = [("/NodeB.inputs:x", "/NodeA")]
        view = SimpleNamespace(
            nodeGraph=SimpleNamespace(getStage=MagicMock(return_value=stage)),
            _buildAttrSearchPaths=MagicMock(return_value=attr_paths),
            _buildRelSearchEntries=MagicMock(return_value=[]),
            _layerAuthorsAttrConnection=MagicMock(return_value=True),
            _layerAuthorsRelTarget=MagicMock(return_value=False),
        )

        result = GraphView._findConnectionAuthoringLayer(
            view, input_prim, "x", output_prim, "result"
        )

        self.assertEqual(result, "/root.usda")
        view._layerAuthorsAttrConnection.assert_called_once_with(layer_a, attr_paths)


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class NodeGraphRelationshipHelpersTest(unittest.TestCase):
    def test_stage_helper_copies_relationship_link_metadata(self):
        node = SimpleNamespace(inputLinks=[], outputLinks=[])
        records = [collect_record("/Constraint", "sources", "/Driver", "")]

        with patched_module_collect_links(
            "UsdNoodles.nodeGraphStage", return_value=records
        ):
            append_stage_collected_links(node, MagicMock())

        self.assertEqual(node.outputLinks[0].propertyOwnerNodeId, "/Constraint")
        self.assertEqual(node.outputLinks[0].sourcePropertyName, "sources")
        self.assertTrue(node.outputLinks[0].is_relationship_link)

    def test_blueprint_helper_filters_root_relationship_links(self):
        node = SimpleNamespace(inputLinks=[], outputLinks=[])
        records = [
            collect_record("/Constraint", "sources", "/Blueprint", ""),
            collect_record("/Constraint", "affects", "/Driven", ""),
        ]

        with patched_module_collect_links(
            "UsdNoodles.nodeGraphBlueprint", return_value=records
        ):
            append_blueprint_relationship_links(node, MagicMock(), "/Blueprint")

        self.assertEqual(len(node.inputLinks), 0)
        self.assertEqual(len(node.outputLinks), 1)
        self.assertEqual(node.outputLinks[0].propertyName, "affects")

    def test_container_helper_filters_root_relationship_links(self):
        node = SimpleNamespace(inputLinks=[], outputLinks=[])
        records = [
            collect_record("/Constraint", "sources", "/Container", ""),
            collect_record("/Constraint", "affects", "/Driven", ""),
        ]

        with patched_module_collect_links(
            "UsdNoodles.nodeGraphBpContainer", return_value=records
        ):
            append_container_relationship_links(node, MagicMock(), "/Container")

        self.assertEqual(len(node.inputLinks), 0)
        self.assertEqual(len(node.outputLinks), 1)
        self.assertEqual(node.outputLinks[0].targetNodeId, "/Driven")


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class LinkClickDispatchTest(unittest.TestCase):
    """Verify isDangling controls whether click selects or starts reconnect."""

    def test_click_dangling_link_selects(self):
        link = make_link_click_link(isDangling=True)
        view = make_link_click_view(link)
        GraphView.mousePressEvent(view, make_mouse_press_event())

        view._selectLink.assert_called_once_with(0, addToSelection=False)
        view._startLinkReconnect.assert_not_called()

    def test_click_regular_link_starts_reconnect(self):
        link = make_link_click_link(isDangling=False)
        view = make_link_click_view(link)
        GraphView.mousePressEvent(view, make_mouse_press_event())

        view._startLinkReconnect.assert_called_once()
        view._selectLink.assert_not_called()


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class RowHitMetricsSchemaTypeTest(unittest.TestCase):
    """_getRowHitMetrics reserves the schema-type subtitle line (Bug 2).

    Drives the real GraphView method unbound (like LinkClickDispatchTest above)
    so the production schema-offset branch is exercised, not just the shim
    mirror in testUsdNoodlesPinDragInteractions. After REQ 1 every USD-prim node
    carries a schemaTypeName; the renderer reserves an extra line for the
    subtitle, so the first row's hit band must shift down to stay aligned with
    the rendered pins/noodles.
    """

    def test_schema_type_shifts_first_row_band_down(self):
        view = make_row_metrics_view()
        renderer = make_row_metrics_renderer()
        typed_start, typed_h = GraphView._getRowHitMetrics(
            view, make_typed_row_node("IrFkController"), renderer
        )
        bare_start, bare_h = GraphView._getRowHitMetrics(
            view, make_typed_row_node(""), renderer
        )
        # Bare baseline: pos.y + (titleFont*(asc-desc) + marginV*2) + marginV
        #             = 0 + (14*1.0 + 8) + 4 = 26.0
        self.assertAlmostEqual(bare_start, 26.0)
        # A schemaTypeName adds a positive subtitle line, pushing the band down.
        self.assertGreater(typed_start, bare_start)
        # Row height itself is unchanged; only the start offset moves.
        self.assertAlmostEqual(typed_h, bare_h)
        self.assertAlmostEqual(bare_h, 10.0 * 1.2 + 8.0 * 1.2 + 2.0)


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class RowEdgeDragStartTest(unittest.TestCase):
    """Pressing a row's outer-tenth edge band starts a connection.

    Drives the real GraphView._findRowEdgeDragStart end-to-end (it calls the
    real _pinAtRowEdge -> _rowEdgePinCandidates / _getRowHitMetrics), unlike the
    shim mirror in testUsdNoodlesPinDragInteractions. It covers REQ 4 for
    pin-less rows: REQ 3 hides a pin glyph until it is connected or dragged, so
    on a freshly added node the row carries no glyph, yet a press in the outer
    tenth of either side must still start a drag (aim near the node edge, not the
    inward-offset label). The middle 80% of the row stays a node-body drag
    handle. Only driving the real method exercises the real schema-subtitle row
    offset together with the edge-band resolution.
    """

    def test_edge_press_on_typed_node_row_starts_drag(self):
        # Band = 10% of the 100px node = 10px on each side. A press in the outer
        # tenth resolves to the row's pin even though no glyph is drawn (REQ 3);
        # the side is the one pressed. y=45 lands in the first row band
        # regardless of the schema-subtitle offset.
        view = make_edge_drag_view()
        node = make_edge_drag_node()
        self.assertEqual(
            view._findRowEdgeDragStart(node, Gf.Vec2d(3.0, 45.0)), ("a", False)
        )
        self.assertEqual(
            view._findRowEdgeDragStart(node, Gf.Vec2d(97.0, 45.0)), ("x", True)
        )

    def test_middle_press_falls_through_to_node_drag(self):
        # The middle 80% of the row is a node-body drag handle, so a press there
        # returns None and the node moves instead of starting a connection.
        view = make_edge_drag_view()
        self.assertIsNone(
            view._findRowEdgeDragStart(make_edge_drag_node(), Gf.Vec2d(50.0, 45.0))
        )

    def test_press_above_first_row_falls_through(self):
        # In the left edge band horizontally, but above the first row band
        # vertically -> no pin -> None (falls through to node selection/drag).
        view = make_edge_drag_view()
        self.assertIsNone(
            view._findRowEdgeDragStart(make_edge_drag_node(), Gf.Vec2d(3.0, 5.0))
        )


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class ChangeBlockBatchingTest(unittest.TestCase):
    """Verify _applyRestoreDeleted and _applyDeleteSelected use Sdf.ChangeBlock."""

    @patch("pxr.Sdf.ChangeBlock")
    def test_applyRestoreDeleted_uses_change_block(self, mock_block):
        view = make_change_block_view()
        GraphView._applyRestoreDeleted(view, [], [])
        mock_block.assert_called_once()
        mock_block.return_value.__enter__.assert_called_once()

    @patch("pxr.Sdf.ChangeBlock")
    def test_applyDeleteSelected_uses_change_block(self, mock_block):
        view = make_change_block_view()
        GraphView._applyDeleteSelected(view, [], [])
        mock_block.assert_called_once()
        mock_block.return_value.__enter__.assert_called_once()


@unittest.skipUnless(_has_graph, "GraphView modules not available")


class SurfaceConnectionEndpointPinsTest(unittest.TestCase):
    """_surfaceConnectionEndpointPins materializes a pin for every endpoint.

    A connection can reference a property that is not an authored/declared
    attribute on its prim (e.g. a typed prim whose schema is not registered, so
    its outputs exist only as the targets of other prims' connections). Such an
    endpoint has no pin row, so the noodle attaches to the bare node edge. This
    pass surfaces each non-relationship endpoint as a dual pin on its node.
    """

    def test_surfaces_producer_output_and_consumer_input(self):
        # The link is stored on the consumer; it names the producer as source.
        link = make_endpoint_pin_link("/FK1", "space", "/Switch1", "rig1:space")
        fk1 = make_pinnable_node("/FK1")
        switch1 = make_pinnable_node("/Switch1", input_links=[link])
        view = make_surface_view([fk1, switch1])

        GraphView._surfaceConnectionEndpointPins(view)

        # Producer FK1 gains the output endpoint "space".
        self.assertEqual(fk1.added_calls, [([], ["space"])])
        # Consumer Switch1 gains the input endpoint "rig1:space".
        self.assertEqual(switch1.added_calls, [(["rig1:space"], [])])

    def test_relationship_links_are_skipped(self):
        link = make_endpoint_pin_link("/A", "affects", "/B", "x", is_rel=True)
        node_a = make_pinnable_node("/A", output_links=[link])
        node_b = make_pinnable_node("/B")
        view = make_surface_view([node_a, node_b])

        GraphView._surfaceConnectionEndpointPins(view)

        self.assertEqual(node_a.added_calls, [])
        self.assertEqual(node_b.added_calls, [])

    def test_endpoint_to_absent_node_is_ignored(self):
        # Source node not in the graph: only the present endpoint is surfaced.
        link = make_endpoint_pin_link("/Absent", "out", "/B", "in")
        node_b = make_pinnable_node("/B", input_links=[link])
        view = make_surface_view([node_b])

        GraphView._surfaceConnectionEndpointPins(view)

        self.assertEqual(node_b.added_calls, [(["in"], [])])

    def test_recomputes_node_size_when_pins_added(self):
        link = make_endpoint_pin_link("/FK1", "space", "/Switch1", "rig1:space")
        fk1 = make_pinnable_node("/FK1")
        switch1 = make_pinnable_node("/Switch1", input_links=[link])
        view = make_surface_view([fk1, switch1])

        GraphView._surfaceConnectionEndpointPins(view)

        self.assertEqual(view.nodeGraph._calculateNodeSize.call_count, 2)

    def test_no_size_recompute_when_nothing_added(self):
        link = make_endpoint_pin_link("/FK1", "space", "/Switch1", "rig1:space")
        fk1 = make_pinnable_node("/FK1", add_returns=False)
        switch1 = make_pinnable_node("/Switch1", input_links=[link], add_returns=False)
        view = make_surface_view([fk1, switch1])

        GraphView._surfaceConnectionEndpointPins(view)

        view.nodeGraph._calculateNodeSize.assert_not_called()

    def test_no_font_atlas_skips_size_recompute_but_still_surfaces(self):
        link = make_endpoint_pin_link("/FK1", "space", "/Switch1", "rig1:space")
        fk1 = make_pinnable_node("/FK1")
        switch1 = make_pinnable_node("/Switch1", input_links=[link])
        view = make_surface_view([fk1, switch1], font_atlas=False)

        GraphView._surfaceConnectionEndpointPins(view)

        view.nodeGraph._calculateNodeSize.assert_not_called()
        self.assertEqual(fk1.added_calls, [([], ["space"])])
