"""Paint and hit use the same detached screen/world geometry."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from UsdNoodles.graphview import render_hotbox, render_overlays
from UsdNoodles.graphview.frame import (
    Frame, HotboxRecord, OverlayState, PopupRecord, Viewport,
)
from UsdNoodles.graphview.render import transforms_for_viewport


class TestRenderOverlays(unittest.TestCase):
    def test_marquee_uses_world_projection(self):
        frame = Frame(viewport=Viewport(15, 20, 2, 300, 200),
                      overlay=OverlayState(marquee_rect=(18, 25, 28, 45)))
        transforms = transforms_for_viewport(frame.viewport)
        with patch.object(render_overlays, '_submit_quads') as submit:
            render_overlays.draw(frame, SimpleNamespace(), None, transforms)
        vertices, projection = submit.call_args.args[1:3]
        self.assertEqual(list(projection.data()), list(transforms.world))
        self.assertEqual((vertices[0].x, vertices[0].y), (18, 25))

    def test_hotbox_row_hit_wins_over_overlapping_popup(self):
        box = HotboxRecord(visible=True, rect=(10, 20, 210, 120),
                           entries=('Node',), rows=((0, (20, 60, 200, 84)),),
                           selected_index=0)
        frame = Frame(
            viewport=Viewport(15, 20, 2, 300, 200),
            hotbox=box,
            overlay=OverlayState(hotkey_popup=PopupRecord(True, box.rect)),
        )
        transforms = transforms_for_viewport(frame.viewport)
        with patch.object(render_hotbox, '_submit_quads') as submit:
            render_hotbox.draw(frame, SimpleNamespace(), None, transforms)
        vertices, projection = submit.call_args_list[-1].args[1:3]
        self.assertEqual(list(projection.data()), list(transforms.screen))
        hit = frame.hit((vertices[0].x + 1, vertices[0].y + 1))
        self.assertEqual((hit.kind, hit.component), ('hotbox', 0))
