"""Headless submissions from immutable link records."""
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

from UsdNoodles.graphview import render_links
from UsdNoodles.graphview.frame import Frame, LinkId, LinkRecord, Viewport
from UsdNoodles.graphview.render import Renderer, transforms_for_viewport


class TestRenderLinks(unittest.TestCase):
    def test_retained_frame_does_not_replace_current_mirror(self):
        renderer = Renderer()
        dangling = LinkRecord(link_id=LinkId(dangling_id='/missing'),
                              start=(3, 5), end=(33, 55), is_dangling=True)
        old = Frame(revision=1, links=(dangling,), dangling_links=(dangling,))
        current = Frame(revision=2)
        current_mirror = renderer.draw(current)
        old_mirror = renderer.draw(old)
        self.assertIsNot(old_mirror, current_mirror)
        self.assertEqual(old_mirror.getLinkCount(), 1)
        self.assertEqual(current_mirror.getLinkCount(), 0)
        self.assertIs(renderer.mirror, current_mirror)
        self.assertEqual(renderer.mirror_revision, 2)
        self.assertIs(renderer.hit_data, current.hit_candidates)

    def test_relationship_tip_and_hover_precedence(self):
        link = LinkRecord(start=(3, 5), end=(83, 55), is_relationship=True,
                          vertical_end_tangent=True, selected=True)
        manager = Mock()
        resources = SimpleNamespace(node_render_manager=manager)
        frame = Frame(viewport=Viewport(width=200, height=100, zoom=2),
                      links=(link, replace(link, end=(93, 65), hovered=True)))
        render_links.draw(frame, resources, None,
                          transforms_for_viewport(frame.viewport))
        selected, hovered = manager.renderEndpointTriangle.call_args_list
        self.assertEqual(selected.args[:2], (83, 55))
        self.assertLess(selected.args[3], 55)
        self.assertEqual(hovered.args[:2], (93, 65))
        self.assertNotEqual(selected.args[-1], hovered.args[-1])

    def test_dangling_indicator_without_adjacency(self):
        link = LinkRecord(link_id=LinkId(dangling_id='/missing'),
                          start=(3, 5), end=(33, 55), is_dangling=True)
        manager = Mock()
        resources = SimpleNamespace(node_render_manager=manager)
        frame = Frame(dangling_links=(link,))
        render_links.draw(frame, resources, None,
                          transforms_for_viewport(frame.viewport))
        self.assertEqual(manager.renderEndpointCircle.call_args.args[:3], (33, 55, 6))
