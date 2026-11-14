"""A gizmo drag routed by an editor extension's DocumentEdit records one undo
entry in the layer it wrote, with the xformOps it created in that layer."""
import types
import unittest

from test_gizmo_deferred_release import APP, Controller
from pxr import Gf, Sdf, Usd, UsdGeom
import editorExtension
import gizmoDrag
import gizmoMath
import gizmoPreview
import gizmoSettings
import gizmoUI


class Routing(editorExtension.EditorExtension):
    def __init__(self, stage, layer):
        self.stage = stage
        self.layer = layer

    def DocumentEdit(self, usdviewApi):
        return Usd.EditContext(self.stage, self.layer)


def _Specs(layer):
    found = []
    layer.Traverse(Sdf.Path.absoluteRootPath,
                   lambda path: found.append(path) if path.IsPropertyPath()
                   else None)
    return sorted(str(path) for path in found)


class DocumentEditDrag(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.docA = Sdf.Layer.CreateAnonymous('docA.usda')
        self.docB = Sdf.Layer.CreateAnonymous('docB.usda')
        root = self.stage.GetRootLayer()
        root.subLayerPaths.append(self.docA.identifier)
        root.subLayerPaths.append(self.docB.identifier)
        self.box = UsdGeom.Xform.Define(self.stage, '/Box').GetPrim()
        self.stage.SetEditTarget(Usd.EditTarget(root))
        self.rootBefore = _Specs(root)
        self.routing = Routing(self.stage, self.docA)
        editorExtension.Register(self.routing)
        self.controller = self._Controller()
        gizmoPreview.SetSink(None, session=self.controller.usdviewApi)
        self.applyDrag = gizmoDrag.ApplyDrag
        gizmoUI.gizmoDrag.ApplyDrag = lambda drag, *args, **kw: \
            drag.target.ApplyTranslate(Gf.Vec3d(1, 2, 3))

    def tearDown(self):
        gizmoUI.gizmoDrag.ApplyDrag = self.applyDrag
        editorExtension.Unregister(self.routing)
        self.controller.Detach()
        self.controller.window.close()
        APP.processEvents()

    def _Controller(self):
        controller = Controller(self.stage)
        controller.settings = gizmoSettings.GizmoSettings()
        controller._tool = gizmoUI.TOOL_TRANSLATE
        controller._gimbalAxes = None
        controller._ctrl = False
        controller._holdSnap = controller._holdGrid = False
        controller._holdPoint = controller._holdEdge = False
        controller._holdPreserve = False
        controller._SwitchPresentedLimbs = lambda: False
        controller._Camera = lambda: (object(), object(), None)
        controller._RigWrittenSets = lambda stage, root: (set(), {},
                                                          set(), {})
        controller._RigRoot = lambda target: None
        writer = gizmoMath.Writer(self.stage, Usd.TimeCode.Default(),
                                  gizmoMath.WRITE_DEFAULT)
        target, reason = gizmoMath.MakeTarget(
            self.stage, self.box, gizmoMath.CHANNELS_POSE, writer)
        self.assertIsNotNone(target, reason)
        controller._target = target
        return controller

    def _Drag(self, switchTo=None):
        handle = types.SimpleNamespace(grabbable=True, center=(0, 0), kind='arrow')
        self.assertTrue(self.controller._BeginDrag(handle, (0, 0)))
        self.controller._UpdateDrag((5, 5))
        if switchTo is not None:
            self.routing.layer = switchTo
        self.controller._EndDrag()
        APP.processEvents()

    def _Translate(self):
        return UsdGeom.XformCommonAPI(self.box).GetXformVectors(
            Usd.TimeCode.Default())[0]

    def test_ops_and_values_land_in_the_routed_layer(self):
        self._Drag()
        self.assertEqual(self._Translate(), Gf.Vec3d(1, 2, 3))
        self.assertEqual(_Specs(self.stage.GetRootLayer()), self.rootBefore)
        self.assertEqual(_Specs(self.docA),
                         ['/Box.xformOp:translate', '/Box.xformOpOrder'])
        stack = self.controller.undoStack
        self.assertEqual(len(stack._undo), 1)
        stack.Undo()
        self.assertEqual(_Specs(self.docA), [])
        self.assertEqual(self._Translate(), Gf.Vec3d(0, 0, 0))
        stack.Redo()
        self.assertEqual(self._Translate(), Gf.Vec3d(1, 2, 3))

    def test_layer_switched_before_commit_is_one_entry(self):
        self._Drag(switchTo=self.docB)
        # The ops were created at the press; only the value follows the
        # switch.
        self.assertEqual(_Specs(self.docA),
                         ['/Box.xformOp:translate', '/Box.xformOpOrder'])
        self.assertEqual(_Specs(self.docB), ['/Box.xformOp:translate'])
        self.assertEqual(self._Translate(), Gf.Vec3d(1, 2, 3))
        stack = self.controller.undoStack
        self.assertEqual(len(stack._undo), 1)
        stack.Undo()
        self.assertEqual(_Specs(self.docA), [])
        self.assertEqual(_Specs(self.docB), [])
        stack.Redo()
        self.assertEqual(self._Translate(), Gf.Vec3d(1, 2, 3))


if __name__ == '__main__':
    unittest.main()
