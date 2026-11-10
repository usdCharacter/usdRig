"""Run with testusdview and examples/11_VolumeWeights.usda."""
import ctypes
import math
import time
from pxr import Gf, Sdf, Usd, UsdGeom, UsdUtils
from pxr.Usdviewq.qt import QtCore, QtWidgets
from pxr.Usdviewq._usdviewq import HydraObserver
import volumeWeightUI as ui
import gizmoPreview
import rigExecUndo
from rigExecUsdview import ImagingLibraryPath


class Event:
    def __init__(self, x, y=10, modifiers=QtCore.Qt.NoModifier, kind=None):
        self.point = QtCore.QPointF(x, y)
        self.mods = modifiers
        self.kind = kind
    def type(self): return self.kind
    def position(self): return self.point
    def button(self): return QtCore.Qt.LeftButton
    def buttons(self): return QtCore.Qt.LeftButton
    def modifiers(self): return self.mods


def testUsdviewInputFunction(app):
    api = app._usdviewApi
    stage = api.stage
    weight = next(p for p in stage.Traverse() if p.GetTypeName() == "RigExecSphereWeight")
    stage.SetEditTarget(stage.GetSessionLayer())
    app._dataModel.selection.setPrimPath(weight.GetPath())
    undo = rigExecUndo.UndoStack()
    dll = ctypes.CDLL(ImagingLibraryPath())
    dll.RigExecImaging_SetWeightOverlayForStage.argtypes = [ctypes.c_longlong,
                                                            ctypes.c_char_p]
    def overlay(path):
        # Re-read the id per call: the container caches the stage when it
        # activates the example's rig.
        stageId = UsdUtils.StageCache.Get().GetId(stage)
        cacheId = stageId.ToLongInt() if stageId.IsValid() else -1
        return dll.RigExecImaging_SetWeightOverlayForStage(
            cacheId, path.encode()) == 0
    panel = ui.OpenVolumeWeightPanel(api, overlay, lambda: True, undo)
    panel._overlayCheckBox.setChecked(True)
    def pump():
        deadline = time.perf_counter()+.06
        while time.perf_counter() < deadline:
            app._processEvents()
        QtWidgets.QApplication.processEvents()
    pump()
    assert gizmoPreview.HasSink(api), "real Hydra preview sink is required"
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(HydraObserver.GetRegisteredSceneIndexNames()[-1])
    def points():
        result = []
        for i in range(12):
            _, ds = observer.GetPrim(weight.GetPath().AppendChild("rigGuideVol_%d" % i))
            if ds:
                result.append(str(ds.Get("xform").Get("matrix").GetValue(0)))
                pv = ds.Get("primvars").Get("points")
                if pv:
                    result.append(str(pv.Get("primvarValue").GetValue(0)))
        return result
    def color():
        target = weight.GetRelationship("rigExec:weightTarget").GetTargets()[0].GetPrimPath()
        _, ds = observer.GetPrim(target)
        return str(ds.Get("primvars").Get("displayColor").Get("primvarValue").GetValue(0))
    initialGuide = points()
    initialColor = color()
    assert initialGuide
    edit = next(w for w in panel.findChildren(ui.AttributeValueWidget)
                if w._usdAttribute.GetName() == "inputs:falloffMax")
    attr = edit._usdAttribute
    original = attr.Get(app._dataModel.currentFrame)
    before = stage.GetSessionLayer().ExportToString()
    edit.mousePressEvent(Event(10))
    for i in range(12): edit.mouseMoveEvent(Event(11+i*2))
    pump()
    assert stage.GetSessionLayer().ExportToString() == before, "drag authored the stage"
    assert points() != initialGuide, "Hydra guide did not resize"
    assert color() != initialColor, "surface influence did not follow the preview"
    assert len(undo._undo) == 0
    edit.mouseReleaseEvent(Event(35))
    assert stage.GetSessionLayer().ExportToString() == before, "release committed synchronously"
    pump()
    assert len(undo._undo) == 1
    assert attr.Get(app._dataModel.currentFrame) != original
    undo.Undo(); pump()
    assert attr.Get(app._dataModel.currentFrame) == original
    edit.mousePressEvent(Event(10)); edit.mouseMoveEvent(Event(40)); pump()
    edit._gesture.Cancel(); edit._gesture = None
    pump()
    assert attr.Get(app._dataModel.currentFrame) == original
    def opacity():
        _, ds = observer.GetPrim(weight.GetPath().AppendChild("rigGuideVol_0"))
        return str(ds.Get("primvars").Get("displayOpacity").Get("primvarValue").GetValue(0))
    originalOpacity = opacity()
    held = stage.GetSessionLayer().ExportToString()
    gesture = ui.WeightGesture(api, weight.GetAttribute("guide:displayOpacity"))
    gesture.Update(weight.GetAttribute("guide:displayOpacity"), .91)
    pump()
    assert opacity() != originalOpacity, "display-only override was not published"
    assert stage.GetSessionLayer().ExportToString() == held
    gesture.Cancel(); pump()
    assert opacity() == originalOpacity
    # Pick real rendered guides, including Storm's owner-path remapping.
    import gizmoMath, gizmoScreen
    controller = panel._viewportController
    view = controller.view
    camera, _ = view.resolveCamera()
    projection = gizmoScreen.ViewProjection(camera)
    viewport = view.computeWindowViewport()
    ratio = float(view.devicePixelRatioF())
    frame = gizmoMath._ReadPublishedControlFrame(stage, weight.GetPath(), app._dataModel.currentFrame)
    root = gizmoMath.FindRigRoot(weight).GetParent()
    world = frame * UsdGeom.XformCache(app._dataModel.currentFrame).GetLocalToWorldTransform(root)
    for shift in (False, True):
        grabbed = False
        for i in range(48):
            angle = i * math.pi / 24
            p = world.Transform(Gf.Vec3d(original*math.cos(angle), original*math.sin(angle), 0))
            screen = gizmoScreen.ProjectPoint(projection, viewport, p)
            if screen is None: continue
            event = Event(screen[0]/ratio, screen[1]/ratio,
                          QtCore.Qt.ShiftModifier if shift else QtCore.Qt.NoModifier)
            if not controller.Begin(event): continue
            grabbed = True
            name = controller.drag[1].GetName()
            assert name.startswith("inputs:scale") if shift else name.startswith("inputs:falloff")
            if shift:
                undo.Clear()
                held = stage.GetSessionLayer().ExportToString()
                scaleAttr = controller.drag[1]
                scaleBefore = scaleAttr.Get(app._dataModel.currentFrame)
                direction = controller.drag[4]
                length = math.hypot(*direction)
                moved = Event(event.point.x()+20*direction[0]/length,
                              event.point.y()+20*direction[1]/length,
                              kind=QtCore.QEvent.MouseMove)
                controller.eventFilter(view, moved)
                pump()
                assert stage.GetSessionLayer().ExportToString() == held
                moved.kind = QtCore.QEvent.MouseButtonRelease
                controller.eventFilter(view, moved)
                assert stage.GetSessionLayer().ExportToString() == held
                pump()
                assert scaleAttr.Get(app._dataModel.currentFrame) > scaleBefore
                assert len(undo._undo) == 1
                undo.Undo(); pump()
            else:
                # A press and release without a drag selects, and authors nothing.
                api.ClearPrimSelection(); pump()
                undo.Clear()
                picked = controller._dragPrim
                held = stage.GetSessionLayer().ExportToString()
                controller.eventFilter(view, Event(event.point.x(), event.point.y(),
                                                   kind=QtCore.QEvent.MouseButtonRelease))
                pump()
                assert controller.drag is None
                assert api.selectedPrims == [picked], "a click on a guide did not select its weight"
                assert stage.GetSessionLayer().ExportToString() == held
                assert len(undo._undo) == 0
                app._dataModel.selection.setPrimPath(weight.GetPath()); pump()
            break
        assert grabbed, "could not grab a rendered weight guide"
    panel._onPreset("linear"); pump(); undo.Clear()
    curve = panel._curveWidget
    curveAttr = curve._usdAttribute
    originalSpline = curveAttr.GetSpline()
    held = stage.GetSessionLayer().ExportToString()
    originalColor = color()
    start = curve._ToPixels(curve._PlotRect(), 0, 0)
    curve.mousePressEvent(Event(start.x(), start.y()))
    curve.mouseMoveEvent(Event(start.x(), start.y()-20))
    pump()
    assert stage.GetSessionLayer().ExportToString() == held
    previewColor = color()
    assert previewColor != originalColor, "curve drag did not update surface influence"
    curve.mouseReleaseEvent(Event(start.x(), start.y()-20))
    assert stage.GetSessionLayer().ExportToString() == held
    pump()
    assert len(undo._undo) == 1
    assert curveAttr.GetSpline() != originalSpline
    assert color() == previewColor, "surface jumped when committing the curve"
    assert not weight.GetAttribute("inputs:falloffSamples").HasAuthoredValueOpinion()
    undo.Undo(); pump()
    assert curveAttr.GetSpline() == originalSpline
    assert color() == originalColor
    curve.mousePressEvent(Event(start.x(), start.y()))
    curve.mouseMoveEvent(Event(start.x(), start.y()-30)); pump()
    assert color() != originalColor
    curve._CancelEdit(); pump()
    assert color() == originalColor
    assert curveAttr.GetSpline() == originalSpline
    # Closing the panel must leave selection changes free of editor work.
    panel.close()
    built = panel._builtSelection
    app._dataModel.selection.setPrimPath(Sdf.Path.absoluteRootPath)
    pump()
    assert panel._builtSelection == built
    print("Weight editor: Hydra preview, queued commit, undo, cancel and hidden panel passed")
