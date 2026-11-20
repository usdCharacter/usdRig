# Copyright (c) Meta Platforms, Inc. and affiliates.
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.

from types import SimpleNamespace
import unittest

from UsdNoodles.graphview.selection import LinkKey, PrimTreeAdapter, Selection


class SelectionTest(unittest.TestCase):
    def setUp(self):
        self.a = LinkKey('attribute', '/A.inputs:x', '/B', 'out', '/A', 'x', '')
        self.b = LinkKey('attribute', '/A.inputs:y', '/Missing', 'out', '/A', 'y', '/Missing')
        self.selection = Selection({'/A'}, {self.a})

    def test_update_modes(self):
        self.selection.update('union', {'/B'}, {self.b})
        self.assertEqual(self.selection.node_ids, {'/A', '/B'})
        self.assertEqual(self.selection.link_keys, {self.a, self.b})
        self.selection.update('toggle', {'/B', '/C'}, {self.a})
        self.assertEqual(self.selection.node_ids, {'/A', '/C'})
        self.assertEqual(self.selection.link_keys, {self.b})
        self.selection.update('replace', {'/D'}, {self.a})
        self.assertEqual(self.selection.node_ids, {'/D'})
        self.assertEqual(self.selection.link_keys, {self.a})

    def test_marquee_uses_gesture_start(self):
        base = self.selection.copy()
        for hits, links, expected_nodes, expected_links in (
            ({'/A', '/B'}, {self.a, self.b}, {'/B'}, {self.b}),
            ({'/A'}, {self.a}, set(), set()),
            (set(), set(), {'/A'}, {self.a}),
        ):
            self.selection.marquee(base, 'toggle', hits, links)
            self.assertEqual(self.selection.node_ids, expected_nodes)
            self.assertEqual(self.selection.link_keys, expected_links)

    def test_keys_distinguish_properties_with_the_same_display_pin(self):
        fields = dict(sourceNodeId='/A', sourcePort='affects', targetNodeId='',
                      targetPort='x', data_targetNodeId='/B', isDangling=True,
                      danglingDirection='output', is_relationship_link=True,
                      propertyOwnerNodeId='/A', propertyName='affects')
        first = LinkKey.from_link(SimpleNamespace(**fields, targetPropertyName='in:x'))
        second = LinkKey.from_link(SimpleNamespace(**fields, targetPropertyName='inputs:x'))
        self.assertNotEqual(first, second)
        self.assertEqual(first.owner_property, '/A.affects')
        self.assertEqual(first.dangling_id, '/B')
        self.selection.update('replace', (), {first})
        self.selection.remap({}, SimpleNamespace(nodes={}, links=[second, first]))
        self.assertEqual(self.selection.link_keys, {first})

    def test_remap_prunes_and_rewrites_owner_endpoints_and_dangling(self):
        self.selection.update('union', {'/A/Child', '/Deleted'}, {self.b})
        live = LinkKey('attribute', '/Group/A.inputs:y', '/NewMissing', 'out',
                       '/Group/A', 'y', '/NewMissing')
        model = SimpleNamespace(nodes={'/Group/A', '/Group/A/Child'}, links=[live])
        self.selection.remap({'/A': '/Group/A', '/Missing': '/NewMissing'}, model)
        self.assertEqual(self.selection.node_ids, {'/Group/A', '/Group/A/Child'})
        self.assertEqual(self.selection.link_keys, {live})


class PrimTreeAdapterTest(unittest.TestCase):
    def test_feedback_settles_in_one_round(self):
        paths = {'/A'}
        writes = []
        selection = Selection()

        def write(new_paths):
            writes.append(set(new_paths))
            paths.clear()
            paths.update(new_paths)
            adapter.from_tree()

        adapter = PrimTreeAdapter(lambda: paths, write, selection)
        adapter.from_tree()
        adapter.to_tree()
        self.assertEqual(selection.node_ids, {'/A'})
        self.assertEqual(writes, [])
        selection.update('replace', {'/B'}, ())
        adapter.to_tree()
        adapter.to_tree()
        self.assertEqual(paths, {'/B'})
        self.assertEqual(writes, [{'/B'}])

    def test_guard_released_after_write_raises(self):
        selection = Selection({'/A'})
        paths = {'/B'}

        def write(_paths):
            adapter.from_tree()
            raise RuntimeError('write failed')

        adapter = PrimTreeAdapter(lambda: paths, write, selection)
        with self.assertRaisesRegex(RuntimeError, 'write failed'):
            adapter.to_tree()
        self.assertEqual(selection.node_ids, {'/A'})
        adapter.from_tree()
        self.assertEqual(selection.node_ids, {'/B'})
