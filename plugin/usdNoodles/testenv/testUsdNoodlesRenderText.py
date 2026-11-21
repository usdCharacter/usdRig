#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""
Direct tests for the graphview text primitives.

``render_text.draw(frame, resources, mirror, transforms)`` paints node
text and inline value cells from a detached frame and the render-owned
mirror, sharing one transform frame with the node and icon paths.
Placement tests pin the emitted text and glyph runs for a known row.
Color tests pin the pill colors for every value state. Cache tests pin
invalidation on a value change versus a retained range when nothing
changed. The live-model test pins that a draw mutates neither position
versions nor text fields.

Everything but the final class runs without a GL context, against a
recording fake text renderer and a stubbed quad submitter. The final
class attempts a real offscreen context and reports it unavailable
when the platform cannot provide one.
"""

from __future__ import annotations

import contextlib
import dataclasses
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from UsdNoodles.graphview.frame import (
    FlashRecord,
    Frame,
    GlyphInputs,
    NodeRecord,
    OverlayState,
    TransformFrameInputs,
    TransformOffset,
    ValueEditorRecord,
    ValueInput,
    Viewport,
)

try:
    from UsdNoodles import noodlesValues as nv
    from UsdNoodles.graphview import render_text as render_text_module
    from UsdNoodles.graphview.frame import FrameSource, InteractionState
    from UsdNoodles.graphview.render import (
        Renderer,
        transforms_for_viewport,
    )
    from UsdNoodles.graphView import GraphView
    from UsdNoodles.models import NodeModel
    from UsdNoodles.widgets import valueCell
    from pxr import Gf, Sdf, Usd, UsdGeom

    _has_native = True
except ImportError as error:  # pragma: no cover - needs staged natives
    print("render-text imports failed: %s" % error)
    _has_native = False


_CONFIG = {
    "nodeBgHigh": 90,
    "nodeBgLow": 86,
    "selectedNodeStrokeColor": [1.0, 1.0, 0.117, 1.0],
    "portConnectedFillColor": [0.7, 0.8, 0.0, 1.0],
    "portDisconnectedFillColor": [0.235, 0.235, 0.235, 1.0],
    "valueConnectedColor": [0.45, 0.62, 0.85, 1.0],
    "valueAnimatedColor": [0.95, 0.72, 0.25, 1.0],
    "valueAuthoredColor": [0.93, 0.93, 0.95, 1.0],
    "valueFallbackColor": [0.93, 0.93, 0.95, 0.42],
    "valueFontScale": 0.9,
    "showPinTypeLabels": True,
    "nodePinTypeFontSize": 26.0,
}


@contextlib.contextmanager
def _config():
    """A patched ``NoodlesConfig`` serving the fixed theme above."""
    with patch("UsdNoodles.graphView.NoodlesConfig") as config:
        config.get.side_effect = (
            lambda key, default: _CONFIG.get(key, default)
        )
        yield config


class _FakeTextRenderer:
    """Deterministic stand-in: one 6-vertex quad per character."""

    def __init__(self, atlas):
        self.font_atlas = atlas
        self.gen_calls = 0
        self.width_calls = 0
        self.text_submits = []
        self.node_text_calls = []

    def calculateTextWidth(self, text, size):
        self.width_calls += 1
        return len(text) * size * 0.5

    def generateTextVertices(
        self, text, cx, cy, depth, size, out, node_index=0.0
    ):
        self.gen_calls += 1
        for i, _ch in enumerate(text):
            x0 = cx + i * size * 0.5
            for v in range(6):
                out.extend([x0 + v, cy + v, depth, 0.0, 0.0, node_index])
        return cx, len(text)

    def drawTextVertices(self, vertices, projection, color=(1.0, 1.0, 1.0, 1.0)):
        self.text_submits.append(
            (list(vertices), list(projection.data()), tuple(color))
        )

    def renderNodeText(
        self,
        graph,
        projection,
        zoom,
        text_changed,
        pan_x=0.0,
        pan_y=0.0,
        viewport_width=0.0,
        viewport_height=0.0,
        cull_offscreen=False,
    ):
        self.node_text_calls.append(
            {
                "graph": graph,
                "projection": list(projection.data()),
                "zoom": zoom,
                "text_changed": text_changed,
                "pan": (pan_x, pan_y),
                "size": (viewport_width, viewport_height),
                "cull": cull_offscreen,
            }
        )
        return True


def _atlas():
    return SimpleNamespace(ascender=0.8, descender=-0.2, lineHeight=1.0)


def _value(
    pin="x",
    prop="x",
    kind="float",
    count=1,
    components=(1.5,),
    type_name="float",
    state="authored",
    editable=True,
    swatch=False,
):
    return ValueInput(
        pin_name=pin,
        property_name=prop,
        kind=kind,
        count=count,
        components=components,
        has_swatch=swatch,
        type_name=type_name,
        state=state,
        editable=editable,
    )


def _record(node_id="A", values=None, x=100.0, y=200.0, **overrides):
    """A known node record: 200x80 at (100, 200), one float row at slot 0.

    Row height 24, port area starting 20 below the node top, cell width
    120, port width 12: the cell fits with room to spare.
    """
    fields = {
        "node_id": node_id,
        "position": (x, y),
        "size": (200.0, 80.0),
        "z_order": 3,
        "title": node_id,
        "type_name": "TestNode",
        "node_type": "TestNode",
        "input_pins": ("x",),
        "output_pins": (),
        "input_row_kinds": (0,),
        "input_row_slots": (0,),
        "output_row_kinds": (),
        "output_row_slots": (),
        "port_start_y": 20.0,
        "port_line_height": 24.0,
        "value_inputs": (values if values is not None else _value(),),
        "glyph_inputs": GlyphInputs(
            font_size=16.0, show_values=True, min_pixel_height=9.0
        ),
        "value_cell_width": 120.0,
        "port_width": 12.0,
    }
    fields.update(overrides)
    return NodeRecord(**fields)


def _frame(nodes, **overrides):
    values = {
        "revision": 1,
        "identity_generation": 1,
        "nodes": tuple(nodes),
        "links": (),
        "transform_inputs": TransformFrameInputs(generation=1, offsets=()),
        "viewport": Viewport(
            pan_x=0.0, pan_y=0.0, zoom=1.0, width=800.0, height=600.0
        ),
        "content_changed": True,
    }
    values.update(overrides)
    return Frame(**values)


def _resources(text_renderer=None):
    atlas = _atlas()
    return SimpleNamespace(
        transform_frame=MagicMock(),
        text_renderer=(
            text_renderer if text_renderer is not None
            else _FakeTextRenderer(atlas)
        ),
        font_atlas=atlas,
        shader_library=object(),
    )


def _theme():
    return valueCell.build_theme(
        _CONFIG["nodeBgHigh"],
        _CONFIG["nodeBgLow"],
        _CONFIG["selectedNodeStrokeColor"],
        _CONFIG["portConnectedFillColor"],
        _CONFIG["portDisconnectedFillColor"],
        _CONFIG["valueConnectedColor"],
        _CONFIG["valueAnimatedColor"],
        _CONFIG["valueAuthoredColor"],
        _CONFIG["valueFallbackColor"],
    )


def _draw(frame, resources):
    """Draw through the public entry with a mock mirror and real transforms."""
    mirror = MagicMock()
    transforms = transforms_for_viewport(frame.viewport)
    render_text_module.draw(frame, resources, mirror, transforms)
    return mirror, transforms


@unittest.skipUnless(_has_native, "render natives not available")
class MovedBindingsTest(unittest.TestCase):
    """Every moved method stays an explicit class attribute."""

    def test_bindings_are_explicit_assignments(self):
        module = render_text_module
        for name in (
            "paintText",
            "_paintValueCells",
            "_paintValueEditor",
            "_emitValueText",
            "_valueTextBand",
            "_valueGlyph",
            "_valueBaselineY",
            "_appendBoolSwitch",
            "_appendChevron",
            "_paintNodeRename",
            "_paintMungLadder",
            "_paintTokenPopup",
            "_activeValueFlash",
            "_startValueFlash",
            "paintHotkeyPopup",
            "paintNavigationHotkeyPopup",
            "_paintHotkeyPopups",
        ):
            bound = getattr(GraphView, name)
            self.assertIs(bound, getattr(module, name))
            self.assertEqual(bound.__module__, module.__name__)

    def test_draw_signature(self):
        import inspect

        self.assertEqual(
            list(inspect.signature(render_text_module.draw).parameters),
            ["frame", "resources", "mirror", "transforms"],
        )

    def test_draw_uses_no_private_transform_frame(self):
        self.assertFalse(hasattr(render_text_module, "NodeTransformFrame"))

    def test_primitives_discovered(self):
        frame = _frame([_record()])
        renderer = Renderer(_resources())
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                renderer.draw(frame)
        self.assertIn("render_text", renderer.primitives_drawn)


@unittest.skipUnless(_has_native, "render natives not available")
class NodeTextSubmitTest(unittest.TestCase):
    """Node text reaches the native manager with the frame's flag."""

    def test_submit_passes_mirror_and_content_changed(self):
        for changed in (True, False):
            fake = _FakeTextRenderer(_atlas())
            resources = _resources(fake)
            frame = _frame([_record()], content_changed=changed)
            with _config():
                with patch.object(render_text_module, "render_nodes"):
                    mirror, _ = _draw(frame, resources)
            self.assertEqual(len(fake.node_text_calls), 1)
            call = fake.node_text_calls[0]
            self.assertIs(call["graph"], mirror)
            self.assertEqual(call["text_changed"], changed)
            self.assertEqual(call["zoom"], 1.0)
            self.assertEqual(call["pan"], (0.0, 0.0))
            self.assertEqual(call["size"], (800.0, 600.0))
            self.assertTrue(call["cull"])
            self.assertEqual(
                call["projection"],
                list(transforms_for_viewport(frame.viewport).world),
            )

    def test_skips_without_pieces(self):
        frame = _frame([_record()])
        transforms = transforms_for_viewport(frame.viewport)
        mirror = MagicMock()
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                render_text_module.draw(frame, None, mirror, transforms)
                render_text_module.draw(
                    frame, SimpleNamespace(), mirror, transforms
                )
                render_text_module.draw(
                    _frame([]), resources, mirror, transforms
                )
                no_atlas = _resources(fake)
                no_atlas.font_atlas = None
                render_text_module.draw(frame, no_atlas, mirror, transforms)
        self.assertEqual(fake.node_text_calls, [])
        self.assertEqual(fake.text_submits, [])


@unittest.skipUnless(_has_native, "render natives not available")
class KnownRowEmissionTest(unittest.TestCase):
    """Emitted text and glyph runs for one known float row.

    Record A at (100, 200), row height 24, value font 16: the pill band
    is (0, 19.36), the cell (154, 220, 274, 239.36), the fitted text
    "1.5000" set tabular at x 221.68 on baseline 234.48, depth 0.0035.
    """

    def _drawn(self, frame, resources):
        with _config():
            with patch.object(
                render_text_module, "render_nodes"
            ) as quads:
                _draw(frame, resources)
                return quads

    def test_glyph_run_geometry(self):
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        self._drawn(_frame([_record()]), resources)
        self.assertEqual(len(fake.text_submits), 1)
        vertices, _projection, color = fake.text_submits[0]
        self.assertEqual(color, (0.93, 0.93, 0.95, 1.0))
        # "1.5000": 6 characters, 6 vertices of 6 floats each.
        self.assertEqual(len(vertices), 6 * 6 * 6)
        self.assertAlmostEqual(vertices[0], 221.68)
        self.assertAlmostEqual(vertices[1], 234.48)
        self.assertAlmostEqual(vertices[2], 0.0035)
        # Tabular digits ride one advance: 8 world units at size 16.
        self.assertAlmostEqual(vertices[36], 221.68 + 8.0)

    def test_pill_quad_bucket(self):
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        quads = self._drawn(_frame([_record()]), resources)
        theme = _theme()
        radius = round(valueCell.corner_radius(24.0), 3)
        pill = [
            len(call.args[1])
            for call in quads._drawNodeVertices.call_args_list
            if call.args[3] == radius and call.args[4] == theme.hairline
        ]
        self.assertEqual(len(pill), 1)
        self.assertGreater(pill[0], 0)
        # Quads share the draw's world transform.
        projection = quads._drawNodeVertices.call_args_list[0].args[2]
        self.assertEqual(
            projection.data(),
            list(
                transforms_for_viewport(
                    _frame([_record()]).viewport
                ).world
            ),
        )

    def test_transient_offset_shifts_value_geometry(self):
        frame = _frame([_record()])
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        self._drawn(frame, resources)
        plain_x = fake.text_submits[0][0][0]
        moved = _frame(
            frame.nodes,
            transform_inputs=TransformFrameInputs(
                generation=1,
                offsets=(TransformOffset("A", 1, 5.0, -3.0),),
            ),
        )
        fake.text_submits.clear()
        self._drawn(moved, resources)
        self.assertAlmostEqual(fake.text_submits[0][0][0], plain_x + 5.0)
        self.assertAlmostEqual(fake.text_submits[0][0][1], 234.48 - 3.0)

    def test_stale_generation_applies_no_offset(self):
        frame = _frame(
            [_record()],
            transform_inputs=TransformFrameInputs(
                generation=1,
                offsets=(TransformOffset("A", 0, 50.0, 60.0),),
            ),
        )
        fake = _FakeTextRenderer(_atlas())
        self._drawn(frame, _resources(fake))
        self.assertAlmostEqual(fake.text_submits[0][0][0], 221.68)


@unittest.skipUnless(_has_native, "render natives not available")
class ValueStateColorTest(unittest.TestCase):
    """Pill colors for every value state, from the record alone."""

    def test_each_state(self):
        theme = _theme()
        colors = render_text_module.value_colors_for_state
        self.assertEqual(
            colors("authored", True, theme),
            (theme.fillAuthored, theme.hairline, theme.textNormal, 0.0),
        )
        self.assertEqual(
            colors("fallback", True, theme),
            (None, None, theme.textDim, 0.0),
        )
        animated = colors("animated", True, theme)
        self.assertEqual(
            animated,
            (
                theme.fillAuthored,
                theme.hairline,
                theme.textAnimated,
                0.0,
            ),
        )
        self.assertEqual(
            colors("connected", True, theme),
            (None, None, theme.textConnected, 0.20),
        )
        # Read-only is the editable flag, not the state word: an
        # editable row in any other state keeps its pill.
        self.assertEqual(
            colors("readonly", True, theme),
            (theme.fillAuthored, theme.hairline, theme.textNormal, 0.0),
        )
        self.assertEqual(
            colors("readonly", False, theme),
            (None, None, theme.textDim, 0.20),
        )
        self.assertEqual(
            colors("authored", False, theme),
            (None, None, theme.textDim, 0.20),
        )

    def test_hover_and_active_overrides(self):
        theme = _theme()
        colors = render_text_module.value_colors_for_state
        hovered = colors("authored", True, theme, hovered=True)
        self.assertEqual(hovered[0], theme.fillHover)
        # A pill-less state has nothing to lift on hover.
        self.assertIsNone(
            colors("connected", True, theme, hovered=True)[0]
        )
        active = colors("authored", True, theme, active=True)
        self.assertEqual(active[0], theme.fillAccent)
        self.assertEqual(active[1], theme.accent)

    def test_matches_view_side_state_table(self):
        theme = _theme()
        view = SimpleNamespace()
        for state in (
            "authored",
            "fallback",
            "animated",
            "connected",
            "readonly",
        ):
            for editable in (False, True):
                for hovered in (False, True):
                    for active in (False, True):
                        row = SimpleNamespace(
                            state=state, editable=editable
                        )
                        self.assertEqual(
                            render_text_module.value_colors_for_state(
                                state, editable, theme, hovered, active
                            ),
                            GraphView._valueStateColors(
                                view, row, theme, hovered, active
                            ),
                            (state, editable, hovered, active),
                        )

    def test_theme_honors_patched_config(self):
        with _config():
            self.assertEqual(render_text_module._value_theme(), _theme())

    def test_theme_without_legacy_module(self):
        saved = sys.modules.pop("UsdNoodles.graphView", None)
        try:
            theme = render_text_module._value_theme()
        finally:
            if saved is not None:
                sys.modules["UsdNoodles.graphView"] = saved
        self.assertTrue(hasattr(theme, "fillAuthored"))
        self.assertTrue(hasattr(theme, "textNormal"))


@unittest.skipUnless(_has_native, "render natives not available")
class DrawHelperParityTest(unittest.TestCase):
    """Draw twins match the moved view-side formulas exactly."""

    def test_band_matches_legacy_formula(self):
        atlas = _atlas()
        with _config():
            view = SimpleNamespace(
                textRenderer=SimpleNamespace(font_atlas=atlas),
                _cachedNodePinFontSize=16.0 / 0.9,
                _cachedValueFontScale=0.9,
                _valueTextBandCache={},
            )
            view._valueFontSize = GraphView._valueFontSize.__get__(view)
            legacy = GraphView._valueTextBand(view, 24.0)
            mine = render_text_module.text_band_for(
                24.0, view._valueFontSize(), atlas, 16.0 / 0.9, 26.0
            )
            self.assertEqual(legacy, mine)
            self.assertAlmostEqual(mine[0], 0.0)
            self.assertAlmostEqual(mine[1], 19.36)

    def test_emit_matches_legacy_glyph_math(self):
        for kwargs in (
            {},
            {"tabular": True},
            {"italic": 0.2},
            {"tabular": True, "italic": 0.2},
        ):
            legacy_view = SimpleNamespace(
                _valueGlyphCache={},
                _valueDigitAdvanceCache={},
                textRenderer=_FakeTextRenderer(None),
            )
            legacy_view._valueDigitAdvance = (
                GraphView._valueDigitAdvance.__get__(legacy_view)
            )
            legacy_view._valueGlyph = GraphView._valueGlyph.__get__(
                legacy_view
            )
            out_legacy = []
            width_legacy = GraphView._emitValueText(
                legacy_view,
                "a1.",
                100.0,
                50.0,
                3.0,
                10.0,
                out_legacy,
                **kwargs,
            )
            caches = {
                "atlas": None,
                "glyphs": {},
                "digits": {},
                "fits": {},
                "axes": {},
                "bands": {},
            }
            out_draw = []
            width_draw = render_text_module._emit_text(
                caches,
                _FakeTextRenderer(None),
                "a1.",
                100.0,
                50.0,
                3.0,
                10.0,
                out_draw,
                **kwargs,
            )
            self.assertEqual(out_draw, out_legacy, kwargs)
            self.assertAlmostEqual(width_draw, width_legacy)


@unittest.skipUnless(_has_native, "render natives not available")
class ValueWidgetEmissionTest(unittest.TestCase):
    """Bool switches, token chevrons, flash, hover and editor states."""

    def _drawn(self, frame, resources):
        with _config():
            with patch.object(
                render_text_module, "render_nodes"
            ) as quads:
                _draw(frame, resources)
                return quads

    def _strokes(self, quads):
        return [
            (call.args[3], call.args[4], len(call.args[1]))
            for call in quads._drawNodeVertices.call_args_list
        ]

    def test_bool_switch_on_adds_inset_quad(self):
        counts = {}
        for on in (False, True):
            record = _record(
                values=_value(
                    kind=nv.KIND_BOOL,
                    components=(on,),
                    type_name="bool",
                )
            )
            fake = _FakeTextRenderer(_atlas())
            quads = self._drawn(_frame([record]), _resources(fake))
            counts[on] = sum(
                len(call.args[1])
                for call in quads._drawNodeVertices.call_args_list
            )
            # A switch carries no text run.
            self.assertEqual(fake.text_submits, [])
        self.assertEqual(counts[True] - counts[False], 6)

    def test_token_chevron_and_left_aligned_text(self):
        record = _record(
            values=_value(
                kind=nv.KIND_TOKEN_ENUM,
                components=("CHOICE",),
                type_name="token",
            )
        )
        fake = _FakeTextRenderer(_atlas())
        quads = self._drawn(_frame([record]), _resources(fake))
        self.assertEqual(len(fake.text_submits), 1)
        vertices, _projection, _color = fake.text_submits[0]
        self.assertEqual(len(vertices), 6 * 6 * 6)
        # Token text is left-aligned: cell left plus the inner pad.
        self.assertAlmostEqual(vertices[0], 154.0 + 0.18 * 24.0)
        carets = [
            count
            for radius, _stroke, count in self._strokes(quads)
            if radius == 0.0
        ]
        self.assertEqual(carets, [3])

    def test_flash_tints_cell(self):
        theme = _theme()
        for kind, stroke in (
            ("commit", theme.accent),
            ("error", theme.error),
        ):
            overlay = OverlayState(
                flash=FlashRecord(
                    node_id="A",
                    property_name="x",
                    component=0,
                    end_time=1e9,
                    kind=kind,
                )
            )
            quads = self._drawn(
                _frame([_record()], overlay=overlay),
                _resources(),
            )
            self.assertIn(
                stroke,
                [second for _first, second, _count in self._strokes(quads)],
                kind,
            )

    def test_hover_arms_component_overlay(self):
        radius = round(valueCell.corner_radius(24.0), 3)
        quads = self._drawn(
            _frame([_record()], value_hover=("A", "x", 0)), _resources()
        )
        self.assertIn(
            (radius, (0.0, 0.0, 0.0, 0.0)),
            [
                (first, second)
                for first, second, _count in self._strokes(quads)
            ],
        )
        quads = self._drawn(_frame([_record()]), _resources())
        self.assertNotIn(
            (radius, (0.0, 0.0, 0.0, 0.0)),
            [
                (first, second)
                for first, second, _count in self._strokes(quads)
            ],
        )

    def test_edited_component_is_skipped(self):
        overlay = OverlayState(
            value_editor=ValueEditorRecord(
                node_id="A",
                property_name="x",
                component=0,
                text="9",
                rect=None,
            )
        )
        fake = _FakeTextRenderer(_atlas())
        quads = self._drawn(
            _frame([_record()], overlay=overlay), _resources(fake)
        )
        # The editor draws this one, so the cell pass skips its text.
        self.assertEqual(fake.text_submits, [])
        self.assertIn(
            _theme().accent,
            [second for _first, second, _count in self._strokes(quads)],
        )


@unittest.skipUnless(_has_native, "render natives not available")
class ValueInvalidationTest(unittest.TestCase):
    """A value change refits; an identical redraw retains."""

    def test_identical_redraw_retains_ranges(self):
        record = _record()
        frame = _frame([record])
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                transforms = transforms_for_viewport(frame.viewport)
                mirror = MagicMock()
                render_text_module.draw(frame, resources, mirror, transforms)
                gen_calls, width_calls = fake.gen_calls, fake.width_calls
                self.assertGreater(gen_calls, 0)
                self.assertGreater(width_calls, 0)
                render_text_module.draw(frame, resources, mirror, transforms)
                self.assertEqual(
                    (fake.gen_calls, fake.width_calls),
                    (gen_calls, width_calls),
                )

    def test_value_change_refits(self):
        record = _record()
        frame = _frame([record])
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                transforms = transforms_for_viewport(frame.viewport)
                mirror = MagicMock()
                render_text_module.draw(frame, resources, mirror, transforms)
                gen_calls, width_calls = fake.gen_calls, fake.width_calls
                changed_value = dataclasses.replace(
                    record.value_inputs[0], components=(2.5,)
                )
                changed = dataclasses.replace(
                    record, value_inputs=(changed_value,)
                )
                render_text_module.draw(
                    _frame([changed], content_changed=True),
                    resources,
                    mirror,
                    transforms,
                )
                self.assertGreater(fake.gen_calls, gen_calls)
                self.assertGreater(fake.width_calls, width_calls)

    def test_atlas_replacement_drops_caches(self):
        frame = _frame([_record()])
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                transforms = transforms_for_viewport(frame.viewport)
                mirror = MagicMock()
                render_text_module.draw(frame, resources, mirror, transforms)
                gen_calls = fake.gen_calls
                fresh = _atlas()
                resources.font_atlas = fresh
                fake.font_atlas = fresh
                render_text_module.draw(frame, resources, mirror, transforms)
                self.assertGreater(fake.gen_calls, gen_calls)


@unittest.skipUnless(_has_native, "render natives not available")
class RenderTextDrawTest(unittest.TestCase):
    """A draw touches the mirror only: the live model is unchanged."""

    def test_draw_leaves_live_model_untouched(self):
        from UsdNoodles.core import LinkData

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
        version_a = live_a.positionVersion
        version_b = live_b.positionVersion
        fake = _FakeTextRenderer(_atlas())
        resources = _resources(fake)
        with _config():
            with patch.object(render_text_module, "render_nodes"):
                renderer = Renderer(resources)
                mirror = renderer.draw(captured)
        self.assertIn("render_text", renderer.primitives_drawn)
        self.assertEqual(len(fake.node_text_calls), 1)
        self.assertIs(fake.node_text_calls[0]["graph"], mirror)
        self.assertEqual(live_a.positionVersion, version_a)
        self.assertEqual(live_b.positionVersion, version_b)
        self.assertEqual(
            (live_a.textStartIndex, live_a.textNumChars), (11, 22)
        )
        self.assertFalse(live_link.hovered)
        self.assertFalse(live_link.selected)


@unittest.skipUnless(_has_native, "render natives not available")
class RenderTextGLTest(unittest.TestCase):
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
        self.assertIn("render_text", renderer.primitives_drawn)


if __name__ == "__main__":
    unittest.main()
