# Copyright (c) Meta Platforms, Inc. and affiliates.
# Licensed under the terms set forth in LICENSE.txt in plugin/usdNoodles/.

"""Production command dispatch against local opinions on real USD layers."""

import unittest
from types import SimpleNamespace

from pxr import Sdf, Tf, Usd
from UsdNoodles.graphview import editing, intents


class View:
    def __init__(self, stage):
        self.nodeGraph = SimpleNamespace(getStage=lambda: stage)
        self.history = editing.HistoryBridge(stage)
        self.editor = editing.Editor(
            stage=self.nodeGraph.getStage, view=lambda: self, history=self.history,
        )

    def _remapMovedNodes(self, paths):
        self.remap = paths


class EditingCoreTest(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.root = self.stage.GetRootLayer()
        self.session = self.stage.GetSessionLayer()
        self.stage.DefinePrim('/Parent/A', 'Xform').SetDocumentation('weaker')
        self.stage.DefinePrim('/Parent/B', 'Xform').CreateRelationship('peer').SetTargets(['/Parent/A'])
        self.stage.SetEditTarget(self.session)
        self.view = View(self.stage)

    def test_activation_replays_absent_opinion_at_recorded_layer(self):
        self.view.editor.execute(intents.Activate(('/Parent/A',), False))
        self.assertFalse(self.session.GetPrimAtPath('/Parent/A').active)
        peer = View(self.stage)
        self.stage.SetEditTarget(self.root)
        peer.history.undo(peer)
        self.assertIsNone(self.session.GetPrimAtPath('/Parent/A'))
        self.assertEqual(self.root.GetPrimAtPath('/Parent/A').documentation, 'weaker')
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        peer.history.redo(peer)
        self.assertFalse(self.session.GetPrimAtPath('/Parent/A').active)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)

    def test_create_reuses_allocated_path(self):
        path = self.view.editor.execute(intents.Create('/Parent', 'Xform', (1000, 2000)))
        self.assertEqual(str(path), '/Parent/Xform1')
        self.assertTrue(self.stage.GetPrimAtPath(path).IsActive())
        self.view.history.undo(self.view)
        self.assertFalse(self.session.GetPrimAtPath(path).active)
        self.view.history.redo(self.view)
        self.assertTrue(self.stage.GetPrimAtPath(path).IsActive())
        self.assertEqual(tuple(self.stage.GetPrimAtPath(path).GetAttribute('ui:nodegraph:node:pos').Get()), (1, 2))

    def test_rename_records_fixup_and_remaps_on_replay(self):
        status, _, _ = self.view.editor.execute(intents.Rename('/Parent', 'A', 'C'))
        self.assertEqual(status, 'renamed')
        rel = self.stage.GetRelationshipAtPath('/Parent/B.peer')
        self.assertEqual(rel.GetTargets(), [Sdf.Path('/Parent/C')])
        self.stage.GetPrimAtPath('/Parent/B').SetDocumentation('unrelated')
        peer = View(self.stage)
        peer.history.undo(peer)
        self.assertEqual(rel.GetTargets(), [Sdf.Path('/Parent/A')])
        self.assertEqual(peer.remap, {'/Parent/C': '/Parent/A'})
        peer.history.redo(peer)
        self.assertEqual(rel.GetTargets(), [Sdf.Path('/Parent/C')])
        self.assertEqual(self.stage.GetPrimAtPath('/Parent/B').GetDocumentation(), 'unrelated')

    def test_partial_second_write_is_rolled_back_and_not_recorded(self):
        from unittest.mock import patch

        original = editing._activation_step

        def fail_after_write(edit, value):
            original(edit, value)
            if edit.path.name == 'B' and value is False:
                raise RuntimeError('second write failed')

        before = self.session.ExportToString()
        with patch.object(editing, '_activation_step', fail_after_write):
            with self.assertRaisesRegex(RuntimeError, 'second write failed'):
                self.view.editor.execute(intents.Activate(('/Parent/A', '/Parent/B'), False))
        self.assertEqual(self.session.ExportToString(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)
        self.assertFalse(self.view.history.history.canUndo())

    def test_failed_replay_leaves_the_starting_state_and_entry(self):
        from unittest.mock import patch

        self.view.editor.execute(intents.Activate(('/Parent/A', '/Parent/B'), False))
        self.view.history.undo(self.view)
        before = self.session.ExportToString()
        original = editing._activation_step

        def fail_after_write(edit, value):
            original(edit, value)
            if edit.path.name == 'B' and value is False:
                raise RuntimeError('redo failed')

        self.stage.SetEditTarget(self.root)
        with patch.object(editing, '_activation_step', fail_after_write):
            with self.assertRaisesRegex(RuntimeError, 'redo failed'):
                self.view.history.redo(self.view)
        self.assertEqual(self.session.ExportToString(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        self.assertFalse(self.view.history.history.canUndo())
        self.assertTrue(self.view.history.history.canRedo())

    def test_create_failure_removes_partially_authored_specs(self):
        before = self.session.ExportToString()

        def failed_author(stage, parent, type_name, position):
            editing.PrimAuthor.create_node_prim(stage, parent, type_name, position)
            return None

        with self.view.editor.creation_author(failed_author):
            with self.assertRaisesRegex(RuntimeError, 'Failed to create Xform'):
                self.view.editor.execute(intents.Create('/Parent', 'Xform', (1, 2)))
        self.assertEqual(self.session.ExportToString(), before)
        self.assertFalse(self.view.history.history.canUndo())

    def test_replay_rejects_same_root_on_a_replaced_stage(self):
        self.view.editor.execute(intents.Activate(('/Parent/A',), False))
        other = Usd.Stage.Open(self.root, Sdf.Layer.CreateAnonymous())
        self.view.nodeGraph.getStage = lambda: other
        history = self.view.history.history
        with self.assertRaisesRegex(RuntimeError, 'different stage'):
            history.undo(self.view)
        self.assertTrue(history.canUndo())
        self.assertFalse(history.canRedo())
        self.assertTrue(other.GetPrimAtPath('/Parent/A').IsActive())

    def test_remove_visual_preserves_usd_and_order(self):
        from UsdNoodles.graphview.selection import LinkKey

        a, b, c = '/Parent/A', '/Parent/B', '/Parent/C'
        link = SimpleNamespace(sourceNodeId=a, targetNodeId=b, sourcePort='out',
                               targetPort='in')
        other = SimpleNamespace(sourceNodeId=b, targetNodeId=c, sourcePort='out',
                                targetPort='in')
        self.view.nodes = {k: SimpleNamespace(selected=k == a, inputLinks=[], outputLinks=[])
                           for k in (a, b, c)}
        self.view.nodes[a].outputLinks[:] = [link]
        self.view.nodes[b].inputLinks[:] = [link]
        self.view.nodes[b].outputLinks[:] = [other]
        self.view.links = [link, other]
        before = (self.root.ExportToString(), self.session.ExportToString())
        self.view.editor.execute(intents.RemoveVisual((a,), (LinkKey.from_link(link),)))
        self.assertEqual(list(self.view.nodes), [b, c])
        self.assertEqual(self.view.links, [other])
        peer = View(self.stage)
        d = '/PeerOnly'
        peer_b, peer_c, peer_d = (SimpleNamespace(selected=False, inputLinks=[], outputLinks=[])
                                  for _ in range(3))
        peer_other = SimpleNamespace(**vars(other))
        peer_link = SimpleNamespace(sourceNodeId=c, targetNodeId=d, sourcePort='out',
                                    targetPort='in')
        peer.nodes = {b: peer_b, c: peer_c, d: peer_d}
        peer.links = [peer_other, peer_link]
        peer_b.outputLinks[:] = [peer_other]
        peer_c.outputLinks[:] = [peer_link]
        peer_d.inputLinks[:] = [peer_link]
        peer._selectedNodes = {d}
        peer.history.undo(peer)
        self.assertEqual(list(peer.nodes), [a, b, c, d])
        self.assertIs(peer.nodes[b], peer_b)
        self.assertIs(peer.nodes[d], peer_d)
        self.assertEqual(peer.links, [link, peer_other, peer_link])
        self.assertEqual(peer_b.inputLinks, [link])
        self.assertIs(peer_b.outputLinks[0], peer_other)
        self.assertIs(peer_c.outputLinks[0], peer_link)
        self.assertEqual(peer._selectedNodes, {d})
        peer.history.redo(peer)
        self.assertEqual(list(peer.nodes), [b, c, d])
        self.assertEqual(peer.links, [peer_other, peer_link])
        self.assertEqual(peer_b.inputLinks, [])
        self.assertIs(peer_b.outputLinks[0], peer_other)
        self.assertEqual(list(self.view.nodes), [b, c])
        self.assertEqual(self.view.links, [other])
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)

    def test_activation_uses_variant_mapping_on_replay(self):
        parent = self.stage.GetPrimAtPath('/Parent')
        variants = parent.GetVariantSets().AddVariantSet('choice')
        variants.AddVariant('one')
        variants.SetVariantSelection('one')
        with variants.GetVariantEditContext():
            mapped = self.stage.GetEditTarget().MapToSpecPath('/Parent/A')
            self.view.editor.execute(intents.Activate(('/Parent/A',), False))
        self.assertFalse(self.session.GetPrimAtPath(mapped).active)
        self.stage.SetEditTarget(self.root)
        self.view.history.undo(self.view)
        self.assertIsNone(self.session.GetPrimAtPath(mapped))
        self.view.history.redo(self.view)
        self.assertFalse(self.session.GetPrimAtPath(mapped).active)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)

    def test_removed_layer_replay_is_refused_without_fallback(self):
        local = Sdf.Layer.CreateAnonymous()
        self.root.subLayerPaths = [local.identifier]
        self.stage.SetEditTarget(local)
        self.view.editor.execute(intents.Activate(('/Parent/A',), False))
        self.stage.SetEditTarget(self.session)
        self.root.subLayerPaths = []
        before = self.session.ExportToString()
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), before)
        self.assertFalse(local.GetPrimAtPath('/Parent/A').active)
        self.assertTrue(self.view.history.history.canUndo())
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)

    def test_position_undo_does_not_author_a_composed_default(self):
        from pxr import Gf
        from UsdNoodles.models import NodeModel

        self.view.nodes = {'/Parent/A': NodeModel(stage=self.stage, primPath=Sdf.Path('/Parent/A'))}
        self.view.update = lambda: None
        self.view._resetRenderCachesAfterBulkMove = lambda **_: None
        before = self.session.ExportToString()
        positions = {'/Parent/A': (2050, 650)}
        editing.edit_node_positions(self.view, positions, 'Move Nodes',
                                    lambda: editing._authorNodePosition(self.view, '/Parent/A', (2050, 650)))
        path = '/Parent/A.ui:nodegraph:node:pos'
        self.assertEqual(self.stage.GetAttributeAtPath(path).Get(), Gf.Vec2f(2.05, .65))
        self.view.history.undo(self.view)
        self.assertEqual(self.session.ExportToString(), before)
        self.view.history.redo(self.view)
        self.assertEqual(self.stage.GetAttributeAtPath(path).Get(), Gf.Vec2f(2.05, .65))

    def _delete_fixture(self, link_layer):
        from UsdNoodles.nodeLibs.usdPrimLibrary import UsdPrimLibrary

        a, b = '/Parent/A', '/Parent/B'
        link = SimpleNamespace(sourceNodeId=b, sourcePort='peer', targetNodeId=a,
                               targetPort='', sourcePropertyName='peer', targetPropertyName='')
        view = self.view
        view.nodes = {path: SimpleNamespace(selected=path == a,
                      getUsdPrim=lambda path=path: self.stage.GetPrimAtPath(path))
                      for path in (a, b)}
        view.links = [link]
        view._selectedNodes = {a}
        view._collectSelectedLinkData = lambda: []
        record = dict(sourceNodeId=b, sourcePort='peer', targetNodeId=a, targetPort='',
                      sourcePropertyName='peer', targetPropertyName='')
        view._collectIncidentLinkKeys = lambda *_: [record]
        view._deserializeLinkRecord = lambda data: data
        view._findLibraryForPrim = lambda _: UsdPrimLibrary()
        view._findConnectionAuthoringLayer = lambda *_: link_layer.identifier
        view._buildConnectionKwargs = intents._buildConnectionKwargs
        view._removeLinkFromGraph = lambda *_, **__: view.links.clear()
        view.clearLinkSelection = view._clearRenderCache = view.update = lambda: None
        view._showPopupMessage = lambda _: None
        return view, a, b, link

    def test_delete_notice_is_atomic_on_execution_and_replay_failure(self):
        view, a, b, link = self._delete_fixture(self.root)
        history = view.history.history

        def state():
            return (self.root.ExportToString(), self.session.ExportToString(),
                    tuple(self.stage.GetRelationshipAtPath(b + '.peer').GetTargets()),
                    self.stage.GetPrimAtPath(a).IsActive())

        before = state()
        observed = []
        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)

        def fail_after_writes():
            self.assertTrue(self.session.GetPrimAtPath(a).HasInfo('active'))
            self.assertNotIn(Sdf.Path(a), self.root.GetRelationshipAtPath(b + '.peer').targetPathList.GetAppliedItems())
            raise RuntimeError('model update failed')

        view._clearRenderCache = fail_after_writes
        with self.assertRaisesRegex(RuntimeError, 'model update failed'):
            view.editor.execute(intents.DeleteSelection())
        self.assertTrue(all(snapshot == before for snapshot in observed), observed)
        self.assertEqual(state(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)
        self.assertEqual((len(history._undo), len(history._redo)), (0, 0))
        self.assertEqual(list(view.nodes), [a, b])
        self.assertEqual(view.links, [link])

        view._clearRenderCache = lambda: None
        observed.clear()
        view.editor.execute(intents.DeleteSelection())
        after = state()
        self.assertTrue(observed)
        self.assertTrue(all(snapshot == after for snapshot in observed), observed)
        view.history.undo(view)
        observed.clear()
        self.stage.SetEditTarget(self.root)
        view.update = fail_after_writes
        with self.assertRaisesRegex(RuntimeError, 'model update failed'):
            view.history.redo(view)
        self.assertTrue(all(snapshot == before for snapshot in observed), observed)
        self.assertEqual(state(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        self.assertEqual((len(history._undo), len(history._redo)), (0, 1))
        self.assertEqual(list(view.nodes), [a, b])
        self.assertEqual(view.links, [link])

    def test_delete_undo_removes_introduced_link_spec_and_redo_recreates_it(self):
        view, a, b, _ = self._delete_fixture(self.session)
        path = b + '.peer'
        before = self.session.ExportToString()
        view.editor.execute(intents.DeleteSelection())
        after = self.session.ExportToString()
        self.assertIsNotNone(self.session.GetRelationshipAtPath(path))
        view.history.undo(view)
        self.assertEqual(self.session.ExportToString(), before)
        self.assertEqual(self.stage.GetRelationshipAtPath(path).GetTargets(), [Sdf.Path(a)])
        view.history.redo(view)
        self.assertEqual(self.session.ExportToString(), after)
        self.assertEqual(self.stage.GetRelationshipAtPath(path).GetTargets(), [])

    def test_redo_rejects_an_occupied_relationship_spec(self):
        view, _, b, _ = self._delete_fixture(self.session)
        Sdf.CreatePrimInLayer(self.session, b)
        path = b + '.peer'
        view.editor.execute(intents.DeleteSelection())
        view.history.undo(view)
        self.assertIsNone(self.session.GetRelationshipAtPath(path))
        self.stage.GetRelationshipAtPath(path).SetTargets(['/Intervening'])
        before = self.session.ExportToString()

        with self.assertRaises(RuntimeError) as error:
            view.history.redo(view)

        self.assertIn(path, str(error.exception))
        self.assertIn(self.session.identifier, str(error.exception))
        self.assertEqual(self.session.ExportToString(), before)
        self.assertTrue(view.history.history.canRedo())

    def test_undo_rejects_a_modified_relationship_spec(self):
        view, _, b, _ = self._delete_fixture(self.session)
        Sdf.CreatePrimInLayer(self.session, b)
        path = b + '.peer'
        view.editor.execute(intents.DeleteSelection())
        self.assertIsNotNone(self.session.GetRelationshipAtPath(path))
        self.stage.GetRelationshipAtPath(path).SetTargets(['/Intervening'])
        before = self.session.ExportToString()

        with self.assertRaises(RuntimeError) as error:
            view.history.undo(view)

        self.assertIn(path, str(error.exception))
        self.assertIn(self.session.identifier, str(error.exception))
        self.assertEqual(self.session.ExportToString(), before)
        self.assertTrue(view.history.history.canUndo())

    def test_create_notice_is_atomic_on_model_failure(self):
        before = (self.root.ExportToString(), self.session.ExportToString())
        observed = []

        def state():
            prim = self.stage.GetPrimAtPath('/Parent/Xform1')
            return (self.session.ExportToString(), bool(prim and prim.IsValid()))

        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)

        def fail_after_authoring(view, intent, path, journal):
            self.assertNotEqual(self.session.ExportToString(), before[1])
            raise RuntimeError('model update failed')

        editor = editing.Editor(stage=self.view.nodeGraph.getStage, view=lambda: self.view,
                                history=self.view.history, model=fail_after_authoring)
        with self.assertRaisesRegex(RuntimeError, 'model update failed'):
            editor.execute(intents.Create('/Parent', 'Xform', (1, 2)))
        self.assertTrue(all(snapshot == (before[1], False) for snapshot in observed),
                        observed)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)
        self.assertFalse(self.view.history.history.canUndo())
        self.assertFalse(self.view.history.history.canRedo())

    def test_rename_notice_is_atomic_on_model_failure(self):
        before = (self.root.ExportToString(), self.session.ExportToString())
        self.view.nodes = {}
        self.view._selectedNodes = {'/Parent/A'}
        observed = []

        def state():
            return (self.session.ExportToString(), self.root.ExportToString(),
                    bool(self.stage.GetPrimAtPath('/Parent/C')),
                    tuple(self.stage.GetRelationshipAtPath('/Parent/B.peer').GetTargets()))

        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)

        def fail_after_authoring(view, intent, result, journal):
            self.assertNotEqual((self.root.ExportToString(), self.session.ExportToString()),
                                before)
            view._selectedNodes.add('/Parent/C')
            raise RuntimeError('model update failed')

        editor = editing.Editor(stage=self.view.nodeGraph.getStage, view=lambda: self.view,
                                history=self.view.history, model=fail_after_authoring)
        with self.assertRaisesRegex(RuntimeError, 'model update failed'):
            editor.execute(intents.Rename('/Parent', 'A', 'C'))
        self.assertTrue(all(snapshot == (before[1], before[0], False, (Sdf.Path('/Parent/A'),))
                            for snapshot in observed), observed)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        self.assertEqual(self.view._selectedNodes, {'/Parent/A'})
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)
        self.assertFalse(self.view.history.history.canUndo())
        self.assertFalse(self.view.history.history.canRedo())

    def test_create_replay_failure_is_atomic(self):
        path = self.view.editor.execute(intents.Create('/Parent', 'Xform', (1, 2)))
        self.view.history.undo(self.view)
        before = self.session.ExportToString()
        observed = []

        def state():
            return (self.session.ExportToString(),
                    self.stage.GetPrimAtPath(path).IsActive())

        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)
        original = editing._activation_step

        def fail_after_write(edit, value):
            original(edit, value)
            if value is True:
                raise RuntimeError('redo failed')

        from unittest.mock import patch

        self.stage.SetEditTarget(self.root)
        with patch.object(editing, '_activation_step', fail_after_write):
            with self.assertRaisesRegex(RuntimeError, 'redo failed'):
                self.view.history.redo(self.view)
        self.assertTrue(all(snapshot == (before, False) for snapshot in observed), observed)
        self.assertEqual(self.session.ExportToString(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        self.assertFalse(self.view.history.history.canUndo())
        self.assertTrue(self.view.history.history.canRedo())

    def test_rename_replay_failure_is_atomic(self):
        from unittest.mock import MagicMock

        status, _, _ = self.view.editor.execute(intents.Rename('/Parent', 'A', 'C'))
        self.assertEqual(status, 'renamed')
        before = (self.root.ExportToString(), self.session.ExportToString())
        observed = []

        def state():
            return (self.session.ExportToString(), self.root.ExportToString(),
                    tuple(self.stage.GetRelationshipAtPath('/Parent/B.peer').GetTargets()))

        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)
        self.view._afterUsdviewRefresh = MagicMock(side_effect=RuntimeError('replay visual failed'))
        self.view._syncSelectionToPrimtree = lambda: None
        self.stage.SetEditTarget(self.root)
        with self.assertRaisesRegex(RuntimeError, 'replay visual failed'):
            self.view.history.undo(self.view)
        self.assertTrue(all(snapshot == (before[1], before[0], (Sdf.Path('/Parent/C'),))
                            for snapshot in observed), observed)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        self.assertEqual(self.view.remap, {'/Parent/A': '/Parent/C'})
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        self.assertTrue(self.view.history.history.canUndo())
        self.assertFalse(self.view.history.history.canRedo())

    def test_position_replay_failure_is_atomic(self):
        from pxr import Gf
        from UsdNoodles.models import NodeModel

        self.view.nodes = {'/Parent/A': NodeModel(stage=self.stage, primPath=Sdf.Path('/Parent/A'))}
        self.view.update = lambda: None
        self.view._resetRenderCachesAfterBulkMove = lambda **_: None
        editing.edit_node_positions(self.view, {'/Parent/A': (2050, 650)}, 'Move Nodes',
                                    lambda: editing._authorNodePosition(self.view, '/Parent/A', (2050, 650)))
        before = self.session.ExportToString()
        observed = []
        attr_path = Sdf.Path('/Parent/A.ui:nodegraph:node:pos')

        def state():
            attr = self.stage.GetAttributeAtPath(attr_path)
            value = attr.Get() if attr and attr.HasValue() else None
            return (self.session.ExportToString(),
                    (round(float(value[0]), 4), round(float(value[1]), 4))
                    if value is not None else None)

        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(state()), self.stage)
        self.addCleanup(observation.Revoke)
        node = self.view.nodes['/Parent/A']
        original_set = node.setDisplayPosition
        calls = {'n': 0}

        def flaky(value):
            calls['n'] += 1
            if calls['n'] == 1:
                raise RuntimeError('replay visual failed')
            return original_set(value)

        node.setDisplayPosition = flaky
        self.stage.SetEditTarget(self.root)
        with self.assertRaisesRegex(RuntimeError, 'replay visual failed'):
            self.view.history.undo(self.view)
        self.assertTrue(all(snapshot == (before, (2.05, 0.65)) for snapshot in observed),
                        observed)
        self.assertEqual(self.session.ExportToString(), before)
        self.assertEqual(tuple(node.position), (2050, 650))
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.root)
        self.assertTrue(self.view.history.history.canUndo())
        self.assertFalse(self.view.history.history.canRedo())

    def test_persist_positions_rolls_back_partial_writes(self):
        from pxr import Gf
        from UsdNoodles.models import NodeModel

        nodes = {path: NodeModel(stage=self.stage, primPath=Sdf.Path(path))
                 for path in ('/Parent/A', '/Parent/B')}
        nodes['/Parent/A'].setDisplayPosition(Gf.Vec2d(100, 200))
        nodes['/Parent/B'].setDisplayPosition(Gf.Vec2d(300, 400))
        self.view.nodes = nodes
        before = self.root.ExportToString()
        observed = []
        observation = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                         lambda *_: observed.append(self.root.ExportToString()),
                                         self.stage)
        self.addCleanup(observation.Revoke)

        def flaky(value):
            raise RuntimeError('disk is full')

        nodes['/Parent/B']._writePositionToUsdRaw = flaky
        with self.assertRaisesRegex(RuntimeError, 'disk is full'):
            editing._persistNodePositions(self.view, self.stage, self.root)
        self.assertTrue(all(snapshot == before for snapshot in observed), observed)
        self.assertEqual(self.root.ExportToString(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.session)

    def test_create_visual_build_receives_composed_prim(self):
        view = self.view
        view.nodes = {}
        view.links = []
        view._selectedNodes = set()
        view._nodeRenderManager = SimpleNamespace(invalidateAll=lambda: None)
        view.clearSelection = lambda: (view._selectedNodes.clear(), [
            setattr(node, 'selected', False) for node in view.nodes.values()])
        calls = {}
        view._syncSelectionToPrimtree = lambda: calls.update(synced=True)
        view.update = lambda: calls.update(updated=True)

        def add_fast(prim, stage):
            calls['valid'] = bool(prim and prim.IsValid())
            calls['path'] = str(prim.GetPath()) if calls['valid'] else ''
            node = SimpleNamespace(id='/Parent/Xform1', selected=False,
                                   inputLinks=[], outputLinks=[])
            view.nodes[node.id] = node
            return node.id

        view._addNewNodeFast = add_fast
        editor = editing.Editor(stage=view.nodeGraph.getStage, view=lambda: view,
                                history=view.history, model=editing._apply_view_model)
        path = editor.execute(intents.Create('/Parent', 'Xform', (1, 2)))
        self.assertEqual(str(path), '/Parent/Xform1')
        self.assertTrue(calls['valid'])
        self.assertEqual(calls['path'], '/Parent/Xform1')
        self.assertIn('/Parent/Xform1', view.nodes)
        self.assertEqual(view._selectedNodes, {'/Parent/Xform1'})
        self.assertTrue(calls.get('synced') and calls.get('updated'))
        view.history.undo(view)
        self.assertEqual(list(view.nodes), [])
        view.history.redo(view)
        self.assertTrue(calls['valid'])
        self.assertIn('/Parent/Xform1', view.nodes)

    def test_remap_refresh_repoints_handles_after_commit(self):
        status, _, _ = self.view.editor.execute(intents.Rename('/Parent', 'A', 'C'))
        self.assertEqual(status, 'renamed')
        node = SimpleNamespace(id='/Parent/C',
                               _prim=self.stage.GetPrimAtPath('/Parent/A'))
        view = SimpleNamespace(nodes={'/Parent/C': node},
                               _remapMovedNodes=lambda remap: {'/Parent/A': '/Parent/C'})
        journal = editing.Journal()
        applied = editing._remap_and_refresh(view, self.stage, journal,
                                             {'/Parent/A': '/Parent/C'})
        self.assertEqual(applied, {'/Parent/A': '/Parent/C'})
        self.assertFalse(node._prim.IsValid())
        journal.run_deferred()
        self.assertTrue(node._prim.IsValid())
        self.assertEqual(str(node._prim.GetPath()), '/Parent/C')

    def test_delete_selection_freezes_link_fixup_and_active_opinion(self):
        view, a, b, link = self._delete_fixture(self.root)
        before = self.root.ExportToString(), self.session.ExportToString()
        view.editor.execute(intents.DeleteSelection())
        after = self.root.ExportToString(), self.session.ExportToString()
        self.assertEqual(self.stage.GetRelationshipAtPath(b + '.peer').GetTargets(), [])
        self.assertFalse(self.stage.GetPrimAtPath(a).IsActive())
        view.history.undo(view)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), before)
        self.assertEqual(list(view.nodes), [a, b])
        self.assertEqual(view.links, [link])
        view.history.redo(view)
        self.assertEqual((self.root.ExportToString(), self.session.ExportToString()), after)
        self.assertEqual(list(view.nodes), [b])
