#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Deletion undo restores authored activation, including mapped edit targets."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

from pxr import Sdf, Usd
from UsdNoodles.graphView import GraphView


class _View:
    _applyRestoreDeleted = GraphView._applyRestoreDeleted
    _applyDeleteSelected = GraphView._applyDeleteSelected
    _groupLinksByLayer = staticmethod(GraphView._groupLinksByLayer)

    def __init__(self, stage, path):
        self.nodeGraph = SimpleNamespace(getStage=lambda: stage)
        prim = stage.GetPrimAtPath(path)
        self.nodes = {path: SimpleNamespace(getUsdPrim=lambda: prim)}
        self._selectedNodes = {path}
        self._spatialIndex = MagicMock()
        self._rebuildLinks = MagicMock()
        self.update = MagicMock()
        GraphView._initUndoTracking(self, stage)

    def delete(self):
        records, count = GraphView._deactivateNodesInUsd(self, list(self.nodes.items()))
        GraphView._pushDeleteSelectedUndo(self, [], records)
        return records, count


class DeleteUndoTest(unittest.TestCase):
    def _round_trip(self, stage, path, layers):
        before = [layer.ExportToString() for layer in layers]
        view = _View(stage, path)
        records, count = view.delete()
        self.assertEqual(count, 1)
        self.assertFalse(stage.GetPrimAtPath(path).IsActive())
        stage.SetEditTarget(stage.GetSessionLayer())
        view._undoHistory.undo(view)
        self.assertTrue(stage.GetPrimAtPath(path).IsActive())
        self.assertEqual([layer.ExportToString() for layer in layers], before)
        self.assertEqual(stage.GetEditTarget().GetLayer(), stage.GetSessionLayer())
        view._undoHistory.redo(view)
        self.assertFalse(stage.GetPrimAtPath(path).IsActive())
        view._undoHistory.undo(view)
        self.assertEqual([layer.ExportToString() for layer in layers], before)
        return records

    def test_root_deletion_restores_absent_and_explicit_active_opinions(self):
        for active in (None, True):
            with self.subTest(active=active), TemporaryDirectory() as directory:
                filename = Path(directory) / "rig.usda"
                stage = Usd.Stage.CreateNew(str(filename))
                prim = stage.DefinePrim("/Rig/Node")
                if active is not None:
                    prim.SetActive(active)
                self._round_trip(stage, "/Rig/Node",
                                 [stage.GetRootLayer(), stage.GetSessionLayer()])
                stage.GetRootLayer().Save()
                reopened = Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(filename)))
                self.assertTrue(reopened.GetPrimAtPath("/Rig/Node").IsActive())
                spec = reopened.GetRootLayer().GetPrimAtPath("/Rig/Node")
                self.assertEqual(spec.HasInfo("active"), active is not None)

    def test_sublayer_edit_removes_new_overs_and_preserves_existing_ancestors(self):
        with TemporaryDirectory() as directory:
            root_path = Path(directory) / "rig.usda"
            edits_path = Path(directory) / "edits.usda"
            stage = Usd.Stage.CreateNew(str(root_path))
            stage.DefinePrim("/Rig/Node")
            layer = Sdf.Layer.CreateNew(str(edits_path))
            Sdf.CreatePrimInLayer(layer, "/Rig")
            stage.GetRootLayer().subLayerPaths = [str(edits_path)]
            stage.SetEditTarget(layer)
            records = self._round_trip(stage, "/Rig/Node",
                                      [stage.GetRootLayer(), layer, stage.GetSessionLayer()])
            self.assertEqual(records[0].layer, layer)
            layer.Save()
            stage.GetRootLayer().Save()
            reopened = Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(root_path)))
            self.assertTrue(reopened.GetPrimAtPath("/Rig/Node").IsActive())
            self.assertIsNone(layer.GetPrimAtPath("/Rig/Node"))

    def test_session_deletion_does_not_author_on_root(self):
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Rig/Node")
        stage.SetEditTarget(stage.GetSessionLayer())
        self._round_trip(stage, "/Rig/Node",
                         [stage.GetRootLayer(), stage.GetSessionLayer()])

    def test_variant_edit_target_restores_the_mapped_spec(self):
        stage = Usd.Stage.CreateInMemory()
        rig = stage.DefinePrim("/Rig")
        variants = rig.GetVariantSets().AddVariantSet("mode")
        variants.AddVariant("a")
        variants.SetVariantSelection("a")
        with variants.GetVariantEditContext():
            stage.DefinePrim("/Rig/Node")
            target = stage.GetEditTarget()
        stage.SetEditTarget(target)
        records = self._round_trip(stage, "/Rig/Node",
                                  [stage.GetRootLayer(), stage.GetSessionLayer()])
        self.assertEqual(records[0].path, Sdf.Path("/Rig{mode=a}Node"))

    def test_variant_override_removes_new_spec_but_keeps_variant_container(self):
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Rig/Node")
        rig = stage.GetPrimAtPath("/Rig")
        variants = rig.GetVariantSets().AddVariantSet("mode")
        variants.AddVariant("a")
        variants.SetVariantSelection("a")
        with variants.GetVariantEditContext():
            target = stage.GetEditTarget()
        stage.SetEditTarget(target)
        self._round_trip(stage, "/Rig/Node",
                         [stage.GetRootLayer(), stage.GetSessionLayer()])
        self.assertFalse(stage.GetRootLayer().GetPrimAtPath("/Rig{mode=a}Node"))
        self.assertEqual(variants.GetVariantNames(), ["a"])


if __name__ == "__main__":
    unittest.main()
