"""Exercise the gesture lifecycle without importing Qt or usdview."""
import sys
import types
import unittest

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd

import gizmoMath
import gizmoModel
import gizmoScreen
import gizmoSettings
import rigExecUndo


class GizmoModelTest(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.prim = self.stage.DefinePrim('/Control')
        # Deliberately not path-sorted; updating a value must not move it.
        self.attrs = [self.prim.CreateAttribute(name, Sdf.ValueTypeNames.Double)
                      for name in ('z', 'a')]
        for attr in self.attrs:
            attr.Set(0.)
        self.writer = gizmoMath.Writer(self.stage, Usd.TimeCode.Default(),
                                       gizmoMath.WRITE_DEFAULT)
        self.stack = rigExecUndo.UndoStack()
        self.events = []
        self.model = gizmoModel.GizmoModel(
            stage=lambda: self.stage, pushUndo=self.stack.Push,
            author=self._Author,
            previewPush=lambda pending, **kw: self.events.append(
                ('preview', list(pending.items()))),
            previewEnd=lambda **kw: self.events.append(('end', kw['publish'])))
        self.settings = gizmoSettings.GizmoSettings()
        self.tool = gizmoScreen.TOOL_TRANSLATE
        self.handle = types.SimpleNamespace(
            grabbable=True, center=(0., 0.), kind='center',
            worldCenter=Gf.Vec3d(0), worldOrigin=Gf.Vec3d(0))
        self.target = types.SimpleNamespace(
            prim=self.prim, writer=self.writer, label='Control',
            supportsPreserveChildren=False, SetPreserveChildren=lambda _: None,
            AttributePaths=lambda: [a.GetPath() for a in self.attrs],
            BeginDrag=self._BeginStructure, ApplyTranslate=self._Collect,
            RefreshDuringDrag=lambda: None, Refresh=lambda: None)

    def _BeginStructure(self):
        # Press-time structure lands inside the same recorder as the values.
        self.attrs[0].Set(10.)

    def _Collect(self, delta, **kwargs):
        for index, attr in enumerate(self.attrs):
            self.writer.Set(attr, delta[0] + index)

    def _Author(self, writer):
        with Sdf.ChangeBlock():
            paths = []
            for path, value in writer.Pending().items():
                self.assertIs(self.model.committing[0], self.target)
                self.events.append(('write', path.name))
                self.stage.GetAttributeAtPath(path).Set(value)
                paths.append(path)
        writer.Clear()
        return paths

    def _Drag(self, x):
        camera = Gf.Camera()
        camera.transform = Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(0, 0, 10))
        self.assertTrue(self.model.Begin(
            self.tool, self.target, self.handle, (0., 0.), camera,
            (0, 0, 800, 600), self.settings))
        self.model.Update((x / 2, 0.), self.settings)
        self.model.Update((x, 0.), self.settings)
        self.assertEqual([a.Get() for a in self.attrs], [10., 0.])
        self.assertEqual([event[0] for event in self.events[-2:]],
                         ['preview', 'preview'])

    def test_cancel_discards_preview_and_restores_structure(self):
        self._Drag(50.)
        self.model.Cancel()
        self.assertEqual(self.events[-1], ('end', True))
        self.assertFalse(self.writer.HasPending())
        self.assertFalse(gizmoMath.PreviewValues(self.stage))
        self.assertFalse(self.stack.CanUndo())
        self.assertEqual([a.Get() for a in self.attrs], [0., 0.])
        self.assertIsNone(self.model.drag)
        self.assertIsNone(self.model.Release())
        self.assertEqual([event[0] for event in self.events],
                         ['preview', 'preview', 'end'])

    def test_release_commits_in_channel_order_once_per_gesture(self):
        self._Drag(50.)
        first = list(self.writer.Pending().values())
        token = self.model.Release()
        self.assertIs(self.model.releasedEdit, token)
        self.assertFalse(self.stack.CanUndo())
        self.assertTrue(gizmoMath.PreviewValues(self.stage))
        self.model.Commit()
        self.model.Commit()
        self.assertEqual(self.events[-3:],
                         [('end', False), ('write', 'z'), ('write', 'a')])
        self.assertFalse(gizmoMath.PreviewValues(self.stage))
        self.assertEqual([a.Get() for a in self.attrs], first)
        self.model.Begin(self.tool, self.target, self.handle, token.press,
                         token.camera, token.viewport, self.settings)
        self.model.Update((100., 0.), self.settings)
        second = list(self.writer.Pending().values())
        self.assertNotEqual(first, second)
        self.model.Release()
        self.model.Commit()
        self.assertEqual([a.Get() for a in self.attrs], second)
        self.stack.Undo()
        self.assertEqual([a.Get() for a in self.attrs], first)
        self.stack.Undo()
        self.assertEqual([a.Get() for a in self.attrs], [0., 0.])
        self.assertFalse(self.stack.CanUndo())
        self.stack.Redo()
        self.stack.Redo()
        self.assertEqual([a.Get() for a in self.attrs], second)


if __name__ == '__main__':
    assert not any(name.startswith(('pxr.Usdviewq', 'PySide'))
                   for name in sys.modules)
    unittest.main()
