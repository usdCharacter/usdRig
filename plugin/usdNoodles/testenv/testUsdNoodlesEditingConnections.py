# Copyright (c) Meta Platforms, Inc. and affiliates.
# Licensed under the terms set forth in LICENSE.txt in plugin/usdNoodles/.

"""Editor registry and stage history against layered USD opinions."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from pxr import Gf, Sdf, Usd
from UsdNoodles import graphView
from UsdNoodles.graphview import editing, intents
from UsdNoodles.nodeLibs.usdPrimLibrary import UsdPrimLibrary


class View:
    def __init__(self, stage):
        self.nodeGraph = SimpleNamespace(getStage=lambda: stage)
        self.nodes = {}
        self.links = []
        self.history = editing.HistoryBridge(stage)
        self.editor = editing.editor_for_view(self, graphView.GraphView._makeCommandRegistry(self))
        self.reloads = 0

    def _findLibraryForPrim(self, prim):
        return UsdPrimLibrary()

    def _reloadCurrentGraph(self):
        self.reloads += 1

    _rebuildLinks = _reloadCurrentGraph

    def _invalidateBackReferenceLinkDataCache(self):
        pass

    def update(self):
        pass


class EditingConnectionsTest(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.root = self.stage.GetRootLayer()
        self.session = self.stage.GetSessionLayer()
        self.a, self.b, self.c, self.d = (
            self.stage.DefinePrim(f'/{name}', 'Xform') for name in 'ABCD')
        self.view = View(self.stage)
        self.execute = self.view.editor.execute

    def test_typed_default_block_and_missing_spec(self):
        attr = self.a.CreateAttribute('vec', Sdf.ValueTypeNames.Double3)
        attr.Set(Gf.Vec3d(1, 2, 3))
        self.stage.SetEditTarget(self.session)
        path = '/A.vec'
        edit = intents.SetValue(path, (4, 5, 6), 'default', 10)
        self.assertEqual(self.execute(edit), (True, ''))
        self.assertEqual(attr.Get(), Gf.Vec3d(4, 5, 6))
        self.view.history.undo(self.view)
        self.assertFalse(self.session.GetPrimAtPath('/A'))
        self.assertEqual(attr.Get(), Gf.Vec3d(1, 2, 3))
        attr.Block()
        self.assertEqual(self.execute(edit), (True, ''))
        self.view.history.undo(self.view)
        self.assertIsInstance(self.session.GetAttributeAtPath(path).default, Sdf.ValueBlock)
        self.view.history.redo(self.view)
        self.assertEqual(attr.Get(), Gf.Vec3d(4, 5, 6))

    def test_sample_replay_preserves_other_samples_and_target(self):
        attr = self.a.CreateAttribute('weight', Sdf.ValueTypeNames.Float)
        attr.Set(1.0, 12)
        self.stage.SetEditTarget(self.session)
        attr.Set(Sdf.ValueBlock(), 12)
        attr.Set(3.0, 20)
        self.assertEqual(self.execute(intents.SetValue('/A.weight', (9,), 'auto', 12)), (True, ''))
        self.assertEqual(attr.Get(12), 9)
        attr.Set(4.0, 20)
        peer = View(self.stage)
        self.stage.SetEditTarget(self.root)
        peer.history.undo(peer)
        self.assertIsInstance(self.session.QueryTimeSample('/A.weight', 12), Sdf.ValueBlock)
        self.assertEqual(self.session.QueryTimeSample('/A.weight', 20), 4)
        peer.history.redo(peer)
        self.assertEqual(attr.Get(12), 9)
        self.assertEqual(self.root.QueryTimeSample('/A.weight', 12), 1)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)

    def test_token_and_uniform_bool(self):
        for name, type_name, old, new in (
                ('mode', Sdf.ValueTypeNames.Token, 'slow', 'fast'),
                ('enabled', Sdf.ValueTypeNames.Bool, False, True)):
            attr = self.a.CreateAttribute(name, type_name, variability=Sdf.VariabilityUniform)
            attr.Set(old)
            self.assertEqual(self.execute(intents.SetValue(str(attr.GetPath()), (new,), 'animation', 4)),
                             (True, ''))
            self.assertEqual(attr.Get(), new)
            self.assertEqual(attr.GetTimeSamples(), [])
            self.view.history.undo(self.view)
            self.assertEqual(attr.Get(), old)

    def test_token_popup_and_bool_toggle_commit_through_registry(self):
        token = self.a.CreateAttribute('mode', Sdf.ValueTypeNames.Token)
        flag = self.a.CreateAttribute('enabled', Sdf.ValueTypeNames.Bool)
        token.Set('slow')
        flag.Set(False)
        before = self.root.ExportToString()
        self.stage.SetEditTarget(self.session)
        view = self.view
        view._editor = view.editor
        view._cachedValueWriteMode = 'default'
        view._currentTimeCode = Usd.TimeCode(3)
        view._startValueFlash = lambda *_: None
        view._writeValueComponents = lambda *args, **kwargs: editing._writeValueComponents(
            view, *args, **kwargs)
        view._tokenPopup = dict(nodeId='/A', primPath='/A', prop='mode', original=('slow',))
        view._tokenPopupList = SimpleNamespace(get_selected_item=lambda: SimpleNamespace(name='fast'))
        graphView.GraphView._commitTokenPopup(view)
        graphView.GraphView._toggleBoolValue(view, SimpleNamespace(
            nodeId='/A', primPath='/A', propertyName='enabled', row=SimpleNamespace(components=(False,))))
        self.assertEqual((token.Get(), flag.Get()), ('fast', True))
        view.history.undo(view)
        view.history.undo(view)
        self.assertEqual((token.Get(), flag.Get()), ('slow', False))
        self.assertFalse(self.session.GetPrimAtPath('/A'))
        view.history.redo(view)
        view.history.redo(view)
        self.assertEqual((token.Get(), flag.Get()), ('fast', True))
        self.assertEqual(self.root.ExportToString(), before)

    def test_animation_mode_authors_first_sample(self):
        attr = self.a.CreateAttribute('weight', Sdf.ValueTypeNames.Float)
        attr.Set(1.0)
        before = self.root.ExportToString()
        self.stage.SetEditTarget(self.session)
        self.assertEqual(self.execute(intents.SetValue('/A.weight', (9,), 'animation', 8)),
                         (True, ''))
        self.assertEqual(self.session.ListTimeSamplesForPath('/A.weight'), [8.0])
        self.view.history.undo(self.view)
        self.assertFalse(self.session.GetPrimAtPath('/A'))
        self.view.history.redo(self.view)
        self.assertEqual(attr.Get(8), 9)
        self.assertEqual(self.root.ExportToString(), before)

    def test_schema_missing_and_explicit_empty_op(self):
        self.stage.SetEditTarget(self.session)
        cmd = intents.Schema(('/A', '/B'), 'PhysicsRigidBodyAPI', 'apply')
        self.execute(cmd)
        self.view.history.undo(self.view)
        self.assertFalse(self.session.GetPrimAtPath('/A'))
        spec = Sdf.CreatePrimInLayer(self.session, '/A')
        spec.SetInfo('apiSchemas', Sdf.TokenListOp.CreateExplicit([]))
        before = self.session.ExportToString()
        self.execute(cmd)
        after = self.session.ExportToString()
        self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), before)
        self.view.history.redo(self.view)
        self.assertEqual(self.session.ExportToString(), after)
        self.execute(intents.Schema('/A', 'PhysicsRigidBodyAPI', 'remove'))
        self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), after)

    def _connection_fixture(self, relationship):
        for prim in (self.a, self.c, self.d):
            prim.CreateAttribute('outputs:x', Sdf.ValueTypeNames.Float)
        if relationship:
            prop = self.b.CreateRelationship('peer')
            field = 'targetPaths'
            old = intents.LinkEndpoints('/A', 'x', '/B', 'peer', 'outputs:x', 'peer')
            new = intents.LinkEndpoints('/C', 'x', '/B', 'peer', 'outputs:x', 'peer')
        else:
            prop = self.b.CreateAttribute('inputs:y', Sdf.ValueTypeNames.Float)
            prop.Set(2.0, 19)
            field = 'connectionPaths'
            old = intents.LinkEndpoints('/A', 'x', '/B', 'y', 'outputs:x', 'inputs:y')
            new = intents.LinkEndpoints('/C', 'x', '/B', 'y', 'outputs:x', 'inputs:y')
        spec = self.root.GetObjectAtPath(prop.GetPath())
        op = Sdf.PathListOp()
        op.prependedItems = [Sdf.Path('/A.outputs:x'), Sdf.Path('/Dangling.outputs:x')]
        op.appendedItems = [Sdf.Path('/D.outputs:x')]
        op.addedItems = [Sdf.Path('/Another.outputs:x')]
        op.deletedItems = [Sdf.Path('/Deleted.outputs:x')]
        op.orderedItems = [Sdf.Path('/D.outputs:x'), Sdf.Path('/A.outputs:x')]
        editing.set_spec_field(spec, field, op)
        self.stage.SetEditTarget(self.session)
        return old, new, prop, field

    def _assert_replacement(self, relationship, kind):
        old, new, prop, field = self._connection_fixture(relationship)
        before = self.root.ExportToString(), self.session.ExportToString()
        self.execute(kind(old, new))
        after = self.root.ExportToString(), self.session.ExportToString()
        spec = self.session.GetObjectAtPath(prop.GetPath())
        op = spec.GetInfo(field)
        self.assertFalse(op.isExplicit)
        self.assertEqual(op.prependedItems, [Sdf.Path('/C.outputs:x')])
        composed = prop.GetTargets() if relationship else prop.GetConnections()
        self.assertNotIn(Sdf.Path('/A.outputs:x'), composed)
        self.assertIn(Sdf.Path('/Dangling.outputs:x'), composed)
        self.stage.SetEditTarget(self.root)
        peer = View(self.stage)
        peer.history.undo(peer)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        peer.history.redo(peer)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), after)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)

    def test_cross_layer_attribute_replace(self):
        self._assert_replacement(False, intents.Replace)

    def test_cross_layer_relationship_reconnect(self):
        self._assert_replacement(True, intents.Reconnect)

    def test_dangling_reconnect_uses_captured_owner_and_restores_visual_order(self):
        _, new, prop, _ = self._connection_fixture(True)
        view = self.view

        def rebuild():
            view.links[:] = [SimpleNamespace(
                sourceNodeId='/B', sourcePort='missing', data_sourceNodeId=str(path.GetPrimPath()),
                sourcePropertyName=path.name, targetNodeId='/B', targetPort='peer',
                targetPropertyName='peer', propertyOwnerNodeId='/B', propertyName='peer',
                is_relationship_link=True, is_input_link=True, isDangling=True,
                danglingDirection='input') for path in prop.GetTargets()]

        view._rebuildLinks = rebuild
        rebuild()
        view.links.reverse()  # custom pre-gesture order, not USD composition order
        original = tuple(map(editing.LinkKey.from_link, view.links))
        link = next(link for link in view.links if link.data_sourceNodeId == '/Dangling')
        capture = editing._capture_reconnect(view, link)
        view.links.remove(link)  # drag temporarily detaches the visual record
        self.execute(intents.Reconnect(*editing._reconnect_command_args(capture, new)))
        self.assertNotIn(Sdf.Path('/Dangling.outputs:x'), prop.GetTargets())
        after = tuple(map(editing.LinkKey.from_link, view.links))
        view.history.undo(view)
        self.assertEqual(tuple(map(editing.LinkKey.from_link, view.links)), original)
        view.history.redo(view)
        self.assertEqual(tuple(map(editing.LinkKey.from_link, view.links)), after)

    def test_connect_and_disconnect_keep_exact_explicit_list(self):
        _, new, prop, field = self._connection_fixture(True)
        spec = Sdf.CreateRelationshipInLayer(self.session, prop.GetPath(), Sdf.VariabilityUniform, True)
        editing.set_spec_field(spec, field, Sdf.PathListOp.CreateExplicit([]))
        before = self.session.ExportToString()
        self.execute(intents.Connect(new))
        connected = self.session.ExportToString()
        self.assertEqual(prop.GetTargets(), [Sdf.Path('/C.outputs:x')])
        self.execute(intents.Disconnect(new))
        self.assertEqual(self.session.ExportToString(), before)
        self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), connected)
        self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), before)

    def test_second_write_failure_rolls_back_layers_and_history(self):
        old, new, _, _ = self._connection_fixture(True)
        before = self.root.ExportToString(), self.session.ExportToString()
        write = editing._write_connection

        def fail_after_add(operation):
            write(operation)
            if operation[-1]:
                raise RuntimeError('injected add failure')

        with patch.object(editing, '_write_connection', side_effect=fail_after_add):
            with self.assertRaisesRegex(RuntimeError, 'injected add failure'):
                self.execute(intents.Replace(old, new))
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        self.assertFalse(self.view.history.history.canUndo())
        self.assertEqual(self.view.reloads, 0)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)
