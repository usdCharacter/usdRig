#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview render layer: resources and renderer.

``RenderResources.initialize(context)`` / ``dispose(context)`` own the GL
bundle (managers, font atlas, buffers, shaders): allocation happens only
while the owning context is current, context destruction is hooked, cleanup
is idempotent, disposal queues onto the owner when the context is not
current, and nothing runs from a GC thread. ``Renderer.draw(frame)`` syncs
the render-owned native mirror from frame records -- fresh bound
``NodeData``/``LinkData``, never the live model -- validates it, and swaps
only a fully valid candidate.

Lifecycle tests run against a fake GL owner with recording factories, so
they need no display. Mirror tests run against the real native
``GraphModel`` (CPU-only). One class attempts a real offscreen GL context
and reports it unavailable when the platform cannot provide one.
"""

from __future__ import annotations

import gc
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from UsdNoodles.graphview import frame as frame_module
from UsdNoodles.graphview.frame import (
    Frame,
    FrameSource,
    InteractionState,
    LinkId,
    LinkRecord,
    NodeRecord,
    TransformFrameInputs,
    TransformOffset,
    Viewport,
)

try:
    from UsdNoodles.graphview import render as render_module
    from UsdNoodles.graphview import resources as resources_module
    from UsdNoodles.graphview.render import (
        MirrorBuildError,
        Renderer,
        transforms_for_viewport,
    )
    from UsdNoodles.graphview.resources import (
        GLContextNotCurrentError,
        RenderResources,
    )
    from UsdNoodles.core import GraphModel, LinkData, NodeData
    from UsdNoodles.models import NodeModel
    from pxr import Gf, Sdf, Usd, UsdGeom

    _has_native = True
except ImportError as error:  # pragma: no cover - needs staged natives
    print("render-resources imports failed: %s" % error)
    _has_native = False


# -- fake GL owner -----------------------------------------------------------


class _FakeGLContextHandle:
    def __init__(self):
        self.valid = True

    def isValid(self):
        return self.valid


class _FakeDestroyedSignal:
    def __init__(self):
        self.slots = []

    def connect(self, slot):
        self.slots.append(slot)

    def emit(self):
        for slot in list(self.slots):
            slot()


class FakeGLWidget:
    """Duck-typed GL owner: validity is programmed, GL calls recorded."""

    def __init__(self):
        self._handle = _FakeGLContextHandle()
        self.destroyed = _FakeDestroyedSignal()
        self.make_current_calls = 0
        self.done_current_calls = 0

    def context(self):
        return self._handle

    def makeCurrent(self):
        self.make_current_calls += 1

    def doneCurrent(self):
        self.done_current_calls += 1

    def invalidate(self):
        self._handle.valid = False

    def revalidate(self):
        self._handle.valid = True


class _FakeAtlas:
    def __init__(self, log):
        self._log = log
        self.texture = None
        self.args = None

    def initialize(self, *args):
        self._log.append("atlas.initialize")
        self.args = args
        self.texture = "texture"

    def cleanup(self):
        self._log.append("atlas.cleanup")
        self.texture = None


class _FakeShaders:
    def __init__(self, log):
        self._log = log
        self.args = None

    def initialize(self, *args):
        self._log.append("shaders.initialize")
        self.args = args

    def cleanup(self):
        self._log.append("shaders.cleanup")


class _FakeManager:
    def __init__(self, name, log):
        self._name = name
        self._log = log
        self.args = None
        self.transform_frame = None

    def initialize(self, *args):
        self._log.append("%s.initialize" % self._name)
        self.args = args

    def cleanup(self):
        self._log.append("%s.cleanup" % self._name)

    def setTransformFrame(self, transform_frame):
        self.transform_frame = transform_frame


class _FakeTextRenderer(_FakeManager):
    def __init__(self, log):
        super().__init__("text", log)
        self._cpp = SimpleNamespace(
            cleanup=lambda: log.append("text._cpp.cleanup")
        )


class _FakeIconRenderer(_FakeManager):
    def __init__(self, log):
        super().__init__("icons", log)
        self.isInitialized = False

    def initialize(self, *args):
        super().initialize(*args)
        self.isInitialized = True

    def cleanup(self):
        super().cleanup()
        self.isInitialized = False


def _recording_factories(log, constructed):
    def _counted(name, factory):
        def _make():
            constructed[name] = constructed.get(name, 0) + 1
            return factory()

        return _make

    return {
        "font_atlas": _counted("font_atlas", lambda: _FakeAtlas(log)),
        "shader_library": _counted(
            "shader_library", lambda: _FakeShaders(log)
        ),
        "node_render_manager": _counted(
            "node_render_manager", lambda: _FakeManager("nodes", log)
        ),
        "group_sticker_renderer": _counted(
            "group_sticker_renderer", lambda: _FakeManager("stickers", log)
        ),
        "link_renderer": _counted(
            "link_renderer", lambda: _FakeManager("links", log)
        ),
        "text_renderer": _counted(
            "text_renderer", lambda: _FakeTextRenderer(log)
        ),
        "transform_frame": _counted(
            "transform_frame", lambda: _FakeManager("frame", log)
        ),
        "icon_renderer": _counted(
            "icon_renderer", lambda: _FakeIconRenderer(log)
        ),
    }


def _assets_with_atlas(root):
    fonts = Path(root) / "fonts"
    fonts.mkdir(parents=True, exist_ok=True)
    (fonts / "Poppins-Regular.png").write_bytes(b"fake-atlas")
    (fonts / "Poppins-Regular.json").write_text("{}")
    return Path(root)


@unittest.skipUnless(_has_native, "render natives not available")
class RenderResourcesLifecycleTest(unittest.TestCase):
    """Lifecycle logic against a fake owner: no display needed."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.assets = _assets_with_atlas(self._tmp.name)
        self.widget = FakeGLWidget()
        self.log = []
        self.constructed = {}
        self.factories = _recording_factories(self.log, self.constructed)

    def _initialize(self, widget=None):
        return RenderResources.initialize(
            widget or self.widget,
            assets_path=self.assets,
            factories=self.factories,
        )

    def test_initialize_allocates_once_and_returns_same_bundle(self):
        first = self._initialize()
        second = self._initialize()
        self.assertIs(second, first)
        self.assertTrue(first.initialized)
        for name in self.factories:
            self.assertEqual(self.constructed.get(name), 1, name)
        self.assertEqual(self.log.count("atlas.initialize"), 1)
        self.assertEqual(self.log.count("nodes.initialize"), 1)
        # The bundle wires the shared transform frame like the old path.
        self.assertIs(
            first.node_render_manager.transform_frame,
            first.transform_frame,
        )
        self.assertIs(
            first.text_renderer.transform_frame, first.transform_frame
        )
        self.assertIs(
            first.icon_renderer.transform_frame, first.transform_frame
        )
        # Owner aliases track the bundle.
        self.assertIs(self.widget.fontAtlas, first.font_atlas)
        self.assertIs(self.widget.shaderLibrary, first.shader_library)

    def test_initialize_requires_current_context(self):
        self.widget.invalidate()
        with self.assertRaises(GLContextNotCurrentError):
            self._initialize()
        self.assertEqual(self.constructed, {})
        self.assertEqual(self.log, [])

    def test_dispose_frees_once_and_is_idempotent(self):
        bundle = self._initialize()
        RenderResources.dispose(self.widget)
        self.assertFalse(bundle.initialized)
        self.assertEqual(self.log.count("atlas.cleanup"), 1)
        self.assertEqual(self.log.count("nodes.cleanup"), 1)
        self.assertEqual(self.log.count("text._cpp.cleanup"), 1)
        self.assertIsNone(bundle.font_atlas)
        self.assertIsNone(bundle.shader_library)
        self.assertIsNone(self.widget.fontAtlas)
        self.assertIsNone(self.widget.shaderLibrary)
        # Managers persist as objects; only their GL state cycled.
        self.assertIsNotNone(bundle.node_render_manager)
        before = list(self.log)
        RenderResources.dispose(self.widget)
        RenderResources.dispose(self.widget)
        self.assertEqual(self.log, before)

    def test_dispose_without_initialize_is_noop(self):
        RenderResources.dispose(self.widget)
        self.assertEqual(self.constructed, {})
        self.assertEqual(self.log, [])

    def test_dispose_while_not_current_queues_then_drains(self):
        bundle = self._initialize()
        self.widget.invalidate()
        RenderResources.dispose(self.widget)
        self.assertTrue(bundle.pending_dispose)
        self.assertTrue(bundle.initialized)
        self.assertNotIn("atlas.cleanup", self.log)
        self.widget.revalidate()
        RenderResources.dispose(self.widget)
        self.assertFalse(bundle.pending_dispose)
        self.assertFalse(bundle.initialized)
        self.assertEqual(self.log.count("atlas.cleanup"), 1)

    def test_initialize_rescinds_queued_dispose(self):
        bundle = self._initialize()
        self.widget.invalidate()
        RenderResources.dispose(self.widget)
        self.assertTrue(bundle.pending_dispose)
        self.widget.revalidate()
        same = self._initialize()
        self.assertIs(same, bundle)
        self.assertTrue(bundle.initialized)
        self.assertFalse(bundle.pending_dispose)
        for name in self.factories:
            self.assertEqual(self.constructed.get(name), 1, name)

    def test_initialize_after_pending_frees_staged_set_first(self):
        atlas = _FakeAtlas(self.log)
        atlas.initialize("legacy.png", "legacy.json")
        shaders = _FakeShaders(self.log)
        shaders.initialize("legacy")
        self.widget.fontAtlas = atlas
        self.widget.shaderLibrary = shaders
        self.widget.invalidate()
        RenderResources.dispose(self.widget)
        shell = self.widget._render_resources
        self.assertTrue(shell.pending_dispose)
        self.assertFalse(shell.initialized)
        # Popping the owner aliases must not strand the staged set: the
        # next initialize frees it before allocating its replacement.
        self.widget.fontAtlas = None
        self.widget.shaderLibrary = None
        self.log.clear()
        self.widget.revalidate()
        bundle = self._initialize()
        self.assertIs(bundle, shell)
        self.assertEqual(self.log.count("atlas.cleanup"), 1)
        self.assertEqual(self.log.count("shaders.cleanup"), 1)
        self.assertIsNot(bundle.font_atlas, atlas)
        self.assertTrue(bundle.initialized)

    def test_destroyed_hook_forgets_without_gl(self):
        bundle = self._initialize()
        self.widget.destroyed.emit()
        self.assertFalse(bundle.initialized)
        self.assertIsNone(bundle.font_atlas)
        self.assertIsNone(bundle.node_render_manager)
        self.assertNotIn("atlas.cleanup", self.log)
        self.assertNotIn("nodes.cleanup", self.log)
        RenderResources.dispose(self.widget)
        self.assertEqual(
            [entry for entry in self.log if entry.endswith("cleanup")], []
        )

    def test_no_gl_from_gc(self):
        self._initialize()
        del self.widget._render_resources
        gc.collect()
        self.assertNotIn("atlas.cleanup", self.log)
        self.assertNotIn("nodes.cleanup", self.log)

    def test_reinitialize_after_dispose_reuses_managers(self):
        bundle = self._initialize()
        managers = (
            bundle.node_render_manager,
            bundle.group_sticker_renderer,
            bundle.link_renderer,
            bundle.text_renderer,
            bundle.transform_frame,
            bundle.icon_renderer,
        )
        RenderResources.dispose(self.widget)
        same = self._initialize()
        self.assertIs(same, bundle)
        self.assertIs(bundle.node_render_manager, managers[0])
        self.assertIs(bundle.text_renderer, managers[3])
        self.assertIs(bundle.icon_renderer, managers[5])
        self.assertEqual(self.log.count("nodes.initialize"), 2)
        self.assertEqual(self.log.count("nodes.cleanup"), 1)

    def test_legacy_set_freed_before_replacement(self):
        atlas = _FakeAtlas(self.log)
        atlas.initialize("legacy.png", "legacy.json")
        shaders = _FakeShaders(self.log)
        shaders.initialize("legacy")
        self.widget.fontAtlas = atlas
        self.widget.shaderLibrary = shaders
        self.log.clear()
        bundle = self._initialize()
        self.assertEqual(self.log.count("atlas.cleanup"), 1)
        self.assertEqual(self.log.count("shaders.cleanup"), 1)
        self.assertIsNot(bundle.font_atlas, atlas)
        self.assertIsNot(bundle.shader_library, shaders)

    def test_missing_atlas_warns_and_stays_uninitialized(self):
        empty = Path(self._tmp.name) / "empty"
        empty.mkdir()
        with unittest.mock.patch.object(
            resources_module.Tf, "Warn"
        ) as warn:
            bundle = RenderResources.initialize(
                self.widget, assets_path=empty, factories=self.factories
            )
        self.assertIsNone(bundle)
        warn.assert_called_once()
        shell = self.widget._render_resources
        self.assertFalse(shell.initialized)


@unittest.skipUnless(_has_native, "render natives not available")
class RenderResourcesGLTest(unittest.TestCase):
    """Real offscreen GL when the platform provides a context."""

    def test_real_offscreen_initialize_dispose(self):
        try:
            from headless.harness import (
                GLNotAvailableError,
                NoodlesTestHarness,
            )
        except ImportError as error:
            self.skipTest("headless harness unavailable: %s" % error)
        harness = NoodlesTestHarness()
        try:
            harness.setup()
        except Exception as error:
            self.skipTest("GL context unavailable: %s" % error)
        self.addCleanup(harness.teardown)
        view = harness.graph_view
        try:
            bundle = RenderResources.initialize(view)
        except RuntimeError as error:
            self.skipTest("GL context not usable: %s" % error)
        self.assertIsNotNone(bundle)
        self.assertTrue(bundle.initialized)
        self.assertIsNotNone(view.fontAtlas)
        self.assertIsNotNone(view.shaderLibrary)
        same = RenderResources.initialize(view)
        self.assertIs(same, bundle)
        RenderResources.dispose(view)
        self.assertFalse(bundle.initialized)
        self.assertIsNone(view.fontAtlas)
        self.assertIsNone(view.shaderLibrary)
        RenderResources.dispose(view)
        revived = RenderResources.initialize(view)
        self.assertIs(revived, bundle)
        self.assertTrue(bundle.initialized)


# -- mirror tests --------------------------------------------------------------


def _node(
    node_id,
    x=0.0,
    y=0.0,
    inputs=("in",),
    outputs=("out",),
    **overrides,
):
    fields = {
        "node_id": node_id,
        "position": (x, y),
        "size": (120.0, 80.0),
        "z_order": 1,
        "title": node_id,
        "type_name": "TestNode",
        "node_type": "TestNode",
        "input_pins": inputs,
        "output_pins": outputs,
        "port_start_y": 20.0,
        "port_line_height": 20.0,
        "input_centers": tuple(30.0 + 20.0 * i for i in range(len(inputs))),
        "output_centers": tuple(
            30.0 + 20.0 * i for i in range(len(outputs))
        ),
        "input_row_kinds": (0,) * len(inputs),
        "input_row_slots": tuple(range(len(inputs))),
        "output_row_kinds": (0,) * len(outputs),
        "output_row_slots": tuple(range(len(outputs))),
    }
    fields.update(overrides)
    return NodeRecord(**fields)


def _link(
    source_id,
    source_pin,
    target_id,
    target_pin,
    index=0,
    kind="attribute",
    owner="",
    dangling=False,
    dangling_direction="",
    relationship=False,
    start=(0.0, 0.0),
    end=(10.0, 0.0),
    selected=False,
    hovered=False,
):
    return LinkRecord(
        link_id=LinkId(
            kind=kind,
            owner_property=owner,
            source_id=source_id,
            source_pin=source_pin,
            target_id=target_id,
            target_pin=target_pin,
            dangling_id=(
                target_id
                if dangling and dangling_direction == "output"
                else source_id if dangling else ""
            ),
        ),
        index=index,
        start=start,
        end=end,
        is_dangling=dangling,
        dangling_direction=dangling_direction,
        is_relationship=relationship,
        selected=selected,
        hovered=hovered,
    )


def _frame(nodes, links, revision=0, generation=0, **overrides):
    fields = {
        "revision": revision,
        "identity_generation": generation,
        "viewport": Viewport(
            pan_x=0.0, pan_y=0.0, zoom=1.0, width=800.0, height=600.0
        ),
        "nodes": tuple(nodes),
        "links": tuple(links),
    }
    fields.update(overrides)
    return Frame(**fields)


def _two_node_frame(**overrides):
    a = _node("A", x=0.0, y=0.0, z_order=1)
    b = _node("B", x=200.0, y=0.0, z_order=2)
    link = _link("A", "out", "B", "in", start=(120.0, 30.0), end=(200.0, 30.0))
    return _frame([a, b], [link], **overrides), a, b, link


@unittest.skipUnless(_has_native, "render natives not available")
class RendererMirrorTest(unittest.TestCase):
    """Mirror sync against the real native GraphModel (CPU-only)."""

    def test_draw_builds_native_mirror(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        mirror = renderer.draw(frame)
        self.assertIs(type(mirror), GraphModel)
        self.assertIs(renderer.mirror, mirror)
        self.assertEqual(renderer.mirror_revision, 0)
        self.assertEqual(renderer.mirror_generation, 0)
        self.assertEqual(mirror.getLinkCount(), 1)
        self.assertTrue(mirror.isPortConnected("A", "out", True))
        self.assertTrue(mirror.isPortConnected("B", "in", False))
        self.assertIn("B", mirror.getConnectedNodeIds("A"))
        self.assertEqual(renderer.mirror_node_ids, ("A", "B"))
        self.assertEqual(renderer.copies_last_draw, 3)
        self.assertEqual(renderer.builds_total, 1)
        self.assertEqual(renderer.draws_total, 1)
        self.assertEqual(renderer.primitives_drawn, ())
        self.assertIs(renderer.hit_data, frame.hit_candidates)

    def test_idle_draw_reuses_mirror(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        first = renderer.draw(frame)
        second = renderer.draw(frame)
        self.assertIs(second, first)
        self.assertEqual(renderer.copies_last_draw, 0)
        self.assertEqual(renderer.node_copies_last_draw, 0)
        self.assertEqual(renderer.link_copies_last_draw, 0)
        self.assertEqual(renderer.builds_total, 1)

    def test_pan_draw_reuses_mirror(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        first = renderer.draw(frame)
        panned = _frame(
            frame.nodes,
            frame.links,
            revision=frame.revision,
            generation=frame.identity_generation,
            viewport=Viewport(
                pan_x=100.0, pan_y=50.0, zoom=1.0, width=800.0, height=600.0
            ),
        )
        self.assertIs(panned.nodes, frame.nodes)
        second = renderer.draw(panned)
        self.assertIs(second, first)
        self.assertEqual(renderer.copies_last_draw, 0)
        self.assertEqual(renderer.builds_total, 1)

    def test_moved_node_resyncs_only_that_record(self):
        frame, a, b, link = _two_node_frame()
        renderer = Renderer()
        first = renderer.draw(frame)
        moved_b = _node("B", x=220.0, y=10.0, z_order=2)
        moved = _frame([a, moved_b], [link], revision=1)
        second = renderer.draw(moved)
        self.assertIsNot(second, first)
        self.assertEqual(renderer.node_copies_last_draw, 1)
        self.assertEqual(renderer.link_copies_last_draw, 0)
        self.assertEqual(renderer.mirror_revision, 1)
        self.assertTrue(second.isPortConnected("B", "in", False))

    def test_selection_only_change_resyncs_at_same_revision(self):
        frame, a, b, link = _two_node_frame()
        renderer = Renderer()
        first = renderer.draw(frame)
        selected_b = _node("B", x=200.0, y=0.0, z_order=2, selected=True)
        changed = _frame([a, selected_b], [link], revision=0)
        second = renderer.draw(changed)
        self.assertIsNot(second, first)
        self.assertEqual(renderer.node_copies_last_draw, 1)
        self.assertEqual(renderer.link_copies_last_draw, 0)
        self.assertEqual(renderer.mirror_revision, 0)

    def test_rejected_candidate_keeps_previous_mirror_and_hit_data(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        good = renderer.draw(frame)
        good_hit = renderer.hit_data
        bad_link = _link("A", "out", "ghost", "in")
        bad = _frame(
            frame.nodes,
            [frame.links[0], bad_link],
            revision=1,
            hit_candidates=frame_module.HitCandidates(
                nodes=(
                    frame_module.NodeHitCandidate(node_id="A", bounds=(0.0,) * 4),
                ),
                links=(),
            ),
        )
        self.assertNotEqual(good_hit, bad.hit_candidates)
        with self.assertRaises(MirrorBuildError):
            renderer.draw(bad)
        self.assertIs(renderer.mirror, good)
        self.assertIs(renderer.hit_data, good_hit)
        self.assertEqual(renderer.mirror_revision, 0)
        # The renderer recovers on the next good frame.
        again = _frame(frame.nodes, frame.links, revision=2)
        recovered = renderer.draw(again)
        self.assertIsNot(recovered, good)
        self.assertEqual(renderer.mirror_revision, 2)
        self.assertIs(renderer.hit_data, again.hit_candidates)

    def test_failed_build_staging_never_leaks_into_recovery(self):
        frame, a, b, link = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        # The failed frame replaces B's record object outright; the bad edge
        # rejects the whole candidate.
        replaced_b = _node("B", x=999.0, y=999.0, z_order=2)
        bad_link = _link("A", "out", "ghost", "in")
        with self.assertRaises(MirrorBuildError):
            renderer.draw(_frame([a, replaced_b], [link, bad_link], revision=1))
        # Recovery reuses the ORIGINAL record objects at a new revision: B
        # must repopulate from the original (not reuse the failed staging),
        # and the fresh mirror must build.
        builds = renderer.builds_total
        recovered = renderer.draw(_frame([a, b], [link], revision=2))
        self.assertEqual(renderer.node_copies_last_draw, 1)
        self.assertEqual(renderer.link_copies_last_draw, 0)
        self.assertEqual(renderer.builds_total, builds + 1)
        self.assertEqual(renderer.mirror_revision, 2)
        self.assertIs(renderer.mirror, recovered)

    def test_duplicate_node_ids_rejected(self):
        frame, a, b, link = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        dupe = _frame([a, b, a], [link], revision=1)
        with self.assertRaises(MirrorBuildError):
            renderer.draw(dupe)
        self.assertEqual(renderer.mirror_revision, 0)

    def test_duplicate_link_rejected(self):
        frame, _, _, link = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        dupe = _frame(frame.nodes, [link, link], revision=1)
        with self.assertRaises(MirrorBuildError):
            renderer.draw(dupe)

    def test_unknown_pin_rejected(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        bad = _link("A", "nope", "B", "in")
        with self.assertRaises(MirrorBuildError):
            renderer.draw(_frame(frame.nodes, [bad], revision=1))

    def test_relationship_source_from_input_only_pin_rejected(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        # "sources" is relationship-only: not dual, so it cannot source.
        # The owner is an output pin, so the sources-flip exemption stays out.
        a = _node("A", inputs=("sources",), outputs=("out",))
        b = _node("B", x=200.0, y=0.0)
        bad = _link(
            "A",
            "sources",
            "B",
            "in",
            kind="relationship",
            owner="affects",
            relationship=True,
        )
        with self.assertRaises(MirrorBuildError):
            renderer.draw(_frame([a, b], [bad], revision=1))

    def test_dual_attribute_pins_pass_either_direction(self):
        # Non-relationship rows are mirror-dual: a swapped attribute edge is
        # still well-formed, so the mirror builds instead of rejecting it.
        a = _node("A", inputs=("in",), outputs=("out",))
        b = _node("B", x=200.0, y=0.0, inputs=("in",), outputs=("out",))
        flipped = _link("B", "in", "A", "out")
        renderer = Renderer()
        mirror = renderer.draw(_frame([a, b], [flipped]))
        self.assertEqual(mirror.getLinkCount(), 1)
        self.assertTrue(mirror.isPortConnected("B", "in", True))

    def test_relationship_whole_prim_target_valid(self):
        a = _node("A", inputs=("in",), outputs=("affects",))
        b = _node("B", x=200.0, y=0.0)
        whole = _link(
            "A",
            "affects",
            "B",
            "",
            kind="relationship",
            owner="affects",
            relationship=True,
        )
        renderer = Renderer()
        mirror = renderer.draw(_frame([a, b], [whole]))
        self.assertEqual(mirror.getLinkCount(), 1)
        self.assertTrue(mirror.isPortConnected("A", "affects", True))

    def test_sources_flip_preserved(self):
        # Authored sources-to-sources direction keeps its input-side source.
        a = _node("A", inputs=("sources",), outputs=("out",))
        b = _node("B", x=200.0, y=0.0, inputs=("sources",))
        flipped = _link(
            "A",
            "sources",
            "B",
            "sources",
            kind="relationship",
            owner="sources",
            relationship=True,
        )
        renderer = Renderer()
        mirror = renderer.draw(_frame([a, b], [flipped]))
        self.assertEqual(mirror.getLinkCount(), 1)

    def test_dangling_records_draw_independently(self):
        frame, a, b, link = _two_node_frame()
        dangling = _link(
            "B",
            "out",
            "ghost",
            "",
            index=1,
            dangling=True,
            dangling_direction="output",
            start=(320.0, 30.0),
            end=(380.0, 30.0),
        )
        renderer = Renderer()
        mirror = renderer.draw(_frame([a, b], [link, dangling]))
        self.assertEqual(mirror.getLinkCount(), 2)
        self.assertTrue(mirror.isPortConnected("A", "out", True))

    def test_remove_node_drops_copies(self):
        frame, a, _, _ = _two_node_frame()
        renderer = Renderer()
        renderer.draw(frame)
        pruned = _frame([a], [], revision=1)
        mirror = renderer.draw(pruned)
        self.assertEqual(renderer.mirror_node_ids, ("A",))
        self.assertEqual(mirror.getLinkCount(), 0)
        self.assertFalse(mirror.isPortConnected("B", "in", False))

    def test_draw_leaves_live_model_untouched(self):
        stage = Usd.Stage.CreateInMemory()
        prim_a = UsdGeom.Sphere.Define(stage, "/A")
        prim_b = UsdGeom.Sphere.Define(stage, "/B")
        prim_a.GetPrim().CreateAttribute(
            "outputs:out", Sdf.ValueTypeNames.Float
        )
        prim_b.GetPrim().CreateAttribute("inputs:in", Sdf.ValueTypeNames.Float)
        live_a = NodeModel(stage=stage, primPath=Sdf.Path("/A"))
        live_b = NodeModel(stage=stage, primPath=Sdf.Path("/B"))
        live_a.setDisplayPosition(Gf.Vec2d(0.0, 0.0))
        live_b.setDisplayPosition(Gf.Vec2d(200.0, 0.0))
        live_a.size = Gf.Vec2d(120.0, 80.0)
        live_b.size = Gf.Vec2d(120.0, 80.0)
        live_a.textStartIndex = 11
        live_a.textNumChars = 22
        self.assertIn("out", live_a.outputPins)
        self.assertIn("in", live_b.inputPins)
        live_link = LinkData()
        live_link.sourceNodeId = "A"
        live_link.sourcePort = "out"
        live_link.targetNodeId = "B"
        live_link.targetPort = "in"
        live_link.start = Gf.Vec2d(120.0, 30.0)
        live_link.end = Gf.Vec2d(200.0, 30.0)
        graph = SimpleNamespace(
            nodes={"A": live_a, "B": live_b}, links=[live_link]
        )
        source = FrameSource(SimpleNamespace(nodeGraph=graph))
        captured = source.capture(
            Viewport(
                pan_x=0.0, pan_y=0.0, zoom=1.0, width=800.0, height=600.0
            ),
            InteractionState(link_under_cursor=0),
            1.0,
        )
        self.assertTrue(captured.links[0].hovered)
        version_a = live_a.positionVersion
        version_b = live_b.positionVersion
        renderer = Renderer()
        renderer.draw(captured)
        self.assertEqual(live_a.positionVersion, version_a)
        self.assertEqual(live_b.positionVersion, version_b)
        self.assertEqual(
            (live_a.textStartIndex, live_a.textNumChars), (11, 22)
        )
        self.assertFalse(live_link.hovered)
        self.assertFalse(live_link.selected)

    def test_transform_inputs_applied_to_resources(self):
        frame, _, _, _ = _two_node_frame(
            transform_inputs=TransformFrameInputs(
                generation=3,
                offsets=(
                    TransformOffset("A", 3, 5.0, 6.0),
                    TransformOffset("B", 2, 50.0, 60.0),
                ),
            )
        )
        resources = SimpleNamespace(transform_frame=MagicMock())
        Renderer(resources).draw(frame)
        resources.transform_frame.reset.assert_called_once_with()
        resources.transform_frame.translate.assert_called_once_with(
            "A", 5.0, 6.0
        )

    def test_transform_inputs_skipped_without_resources(self):
        frame, _, _, _ = _two_node_frame(
            transform_inputs=TransformFrameInputs(
                generation=1,
                offsets=(TransformOffset("A", 1, 5.0, 6.0),),
            )
        )
        Renderer().draw(frame)

    def test_draw_returns_mirror_and_counts_draws(self):
        frame, _, _, _ = _two_node_frame()
        renderer = Renderer()
        self.assertIs(renderer.draw(frame), renderer.mirror)
        self.assertEqual(renderer.draws_total, 1)
        renderer.draw(frame)
        self.assertEqual(renderer.draws_total, 2)

    def test_transforms_match_ortho_geometry(self):
        viewport = Viewport(
            pan_x=5.0, pan_y=7.0, zoom=2.0, width=200.0, height=100.0
        )
        transforms = transforms_for_viewport(viewport)
        depth = 100.0
        world = (
            2.0 / 100.0,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 / (7.0 - 57.0),
            0.0,
            0.0,
            0.0,
            0.0,
            -2.0 / (2.0 * depth),
            0.0,
            -(105.0 + 5.0) / 100.0,
            -(7.0 + 57.0) / (7.0 - 57.0),
            0.0,
            1.0,
        )
        screen = (
            2.0 / 200.0,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 / (0.0 - 100.0),
            0.0,
            0.0,
            0.0,
            0.0,
            -2.0 / (2.0 * depth),
            0.0,
            -1.0,
            1.0,
            0.0,
            1.0,
        )
        for got, want in zip(transforms.world, world):
            self.assertAlmostEqual(got, want)
        for got, want in zip(transforms.screen, screen):
            self.assertAlmostEqual(got, want)


if __name__ == "__main__":
    unittest.main()
