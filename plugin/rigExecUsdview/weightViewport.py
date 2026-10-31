"""Direct manipulation of the published weight guides."""
import ctypes
import math
from pxr import Gf, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtWidgets
import gizmoMath
import gizmoScreen
import gizmoUI


def NearestOnCurve(point, points):
    best, distance = Gf.Vec3d(0), float("inf")
    for a, b in zip(points, points[1:]):
        a, b = Gf.Vec3d(a), Gf.Vec3d(b)
        edge = b - a
        length = Gf.Dot(edge, edge)
        t = max(0.0, min(1.0, Gf.Dot(point - a, edge) / length)) if length else 0.0
        candidate = a + edge * t
        d = (point - candidate).GetLength()
        if d < distance:
            best, distance = candidate, d
    return best


def DragValue(base, delta, direction, minimum=0.0, maximum=None):
    length = sum(v * v for v in direction)
    if length < 1e-8:
        return base
    value = base + sum(a * b for a, b in zip(delta, direction)) / length
    value = max(minimum, value)
    return min(maximum, value) if maximum is not None else value


class WeightEscapeFilter(QtCore.QObject):
    def __init__(self, controller):
        super().__init__(controller)
        self.controller = controller

    def eventFilter(self, obj, event):
        if event.type() != QtCore.QEvent.KeyPress or event.key() != QtCore.Qt.Key_Escape:
            return False
        from volumeWeightUI import _weightGestures, AttributeValueWidget
        controller = self.controller
        active = QtWidgets.QApplication.activeWindow()
        if active not in (controller.api.qMainWindow, controller.panel):
            return False
        gesture = _weightGestures.Get(controller.api)
        if gesture is None or gesture.closed:
            return False
        gesture.Cancel()
        controller.Cancel()
        controller.panel._curveWidget._CancelEdit()
        for widget in controller.panel.findChildren(AttributeValueWidget):
            widget._gesture = None
            widget._mungBaseMousePos = None
        return True


class WeightViewportController(QtCore.QObject):
    """Drags weight guides while the weight panel is open.

    `drag` is [gesture, attr, base, origin, direction, lower, upper]. The
    gesture starts once the cursor passes the drag distance, so a plain
    click on a guide still selects its weight.
    """

    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel
        self.api = panel._usdviewApi
        self.view = gizmoUI.StageView(self.api)
        self.drag = None
        self._dragPrim = None
        self._dragAdds = False
        self._imaging = None
        self._escape = WeightEscapeFilter(self)
        self._attached = False
        if self.view is not None:
            self.view.installEventFilter(self)
        self.api.dataModel.signalStageReplaced.connect(self.Cancel)
        self.api.dataModel.currentFrameChanged.connect(self.Cancel)

    def Attach(self):
        """Install the application Escape filter while the panel is shown."""
        if not self._attached:
            QtWidgets.QApplication.instance().installEventFilter(self._escape)
            self._attached = True

    def Detach(self):
        if self._attached:
            QtWidgets.QApplication.instance().removeEventFilter(self._escape)
            self._attached = False

    def Cancel(self, *unused):
        if self.drag is not None:
            if self.drag[0] is not None:
                self.drag[0].Cancel()
            self.drag = None
            self._dragPrim = None
            self.view.releaseMouse()

    def _Imaging(self):
        stage = self.api.stage
        if self._imaging is not None and self._imaging.stage is stage:
            return self._imaging
        try:
            import imagingHandle
            from rigExecUsdview import ImagingLibraryPath
            self._imaging = imagingHandle.ImagingHandle(
                ctypes.CDLL(ImagingLibraryPath()), stage)
        except Exception:
            self._imaging = None
        return self._imaging

    def _MovedFloat(self, path):
        """The value a mover published at `path` this generation, else None."""
        imaging = self._Imaging()
        if imaging is None:
            return None
        out = (ctypes.c_float * 1)()
        try:
            found = int(imaging.GetMovedFloats(str(path).encode("utf-8"), out, 1))
        except Exception:
            return None
        return float(out[0]) if found == 1 else None

    def _Click(self):
        """A press and release without a drag selects the weight."""
        prim = self._dragPrim
        if prim is None or not prim:
            return
        selection = self.api.dataModel.selection
        if self._dragAdds:
            selection.addPrimPath(prim.GetPath())
        else:
            selection.setPrimPath(prim.GetPath())
        self.api.UpdateGUI()

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Escape:
            if self.drag is not None:
                self.Cancel()
                return True
        if self.drag is not None:
            if kind == QtCore.QEvent.MouseMove:
                gesture, attr, base, origin, direction, lower, upper = self.drag
                pos = event.position() if hasattr(event, "position") else event.pos()
                delta = (pos.x() - origin[0], pos.y() - origin[1])
                if gesture is None:
                    if (math.hypot(*delta) <
                            QtWidgets.QApplication.startDragDistance()):
                        return True
                    from volumeWeightUI import WeightGesture
                    gesture = self.drag[0] = WeightGesture(self.api, attr)
                    self.panel._selectionLabel.setText(
                        "Dragging %s: %s" % (self._dragPrim.GetName(),
                                             attr.GetName().split(":")[-1]))
                gesture.Update(attr, DragValue(base, delta, direction, lower, upper))
                return True
            if kind == QtCore.QEvent.MouseButtonRelease:
                gesture = self.drag[0]
                self.drag = None
                self.view.releaseMouse()
                if gesture is not None:
                    gesture.Commit()
                else:
                    self._Click()
                self._dragPrim = None
                return True
        if (kind != QtCore.QEvent.MouseButtonPress or not self.panel.isVisible()
                or event.button() != QtCore.Qt.LeftButton
                or event.modifiers() & (QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier
                                        | QtCore.Qt.ControlModifier)):
            return False
        return self.Begin(event)

    def Begin(self, event):
        from volumeWeightUI import PLACED_WEIGHT_TYPE_NAMES
        pos = event.position() if hasattr(event, "position") else event.pos()
        ratio = float(self.view.devicePixelRatioF())
        inside, frustum = self.view.computePickFrustum(pos.x()*ratio, pos.y()*ratio)
        hits = self.view.pick(frustum) if inside else []
        if not hits:
            return False
        hit = hits[0]
        path = hit.hitPrimPath
        # Storm may map a synthetic guide back to its owning USD prim.
        if path.name.startswith("rigGuideVol_"):
            path = path.GetParentPath()
        prim = self.api.stage.GetPrimAtPath(path)
        if not prim or prim.GetTypeName() not in PLACED_WEIGHT_TYPE_NAMES:
            return False
        time = self.api.dataModel.currentFrame
        frame = gizmoMath._ReadPublishedControlFrame(self.api.stage, prim.GetPath(), time)
        if frame is None:
            return False
        root = gizmoMath.FindRigRoot(prim).GetParent()
        asset = UsdGeom.XformCache(time).GetLocalToWorldTransform(root)
        world = frame * asset
        local = world.GetInverse().Transform(Gf.Vec3d(hit.hitPoint))
        # Shape the hit test from the values the guides were drawn with:
        # a property mover's output stands in for the authored value.
        def read(name, default=1.0):
            moved = self._MovedFloat(prim.GetPath().AppendProperty(name))
            if moved is not None:
                return moved
            attr = prim.GetAttribute(name)
            value = attr.Get(time) if attr else None
            return default if value is None else float(value)
        inner, outer = read("inputs:falloffMin", 0), read("inputs:falloffMax")
        shape = prim.GetTypeName()
        shift = bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        lower, upper = 0.0, None
        if shape == "RigExecPlaneWeight":
            axis = "xyz".index(str(prim.GetAttribute("rigExec:planeAxis").Get()))
            distance = local[axis]
            direction = Gf.Vec3d(0); direction[axis] = 1
            if shift and prim.GetAttribute("rigExec:planeBounds").Get() == "bounded":
                axes = ((axis+1)%3, (axis+2)%3)
                which = max(range(2), key=lambda i: abs(local[axes[i]]) / max(read("inputs:extent"+"UV"[i]), 1e-6))
                axis = axes[which]
                name = "inputs:extent" + "UV"[which]
                direction = Gf.Vec3d(0); direction[axis] = 1 if local[axis] >= 0 else -1
                lower = 1e-4
            else:
                name = "inputs:falloffMin" if abs(distance-inner) < abs(distance-outer) else "inputs:falloffMax"
                lower = -float("inf")
        else:
            scales = Gf.Vec3d(*[read("inputs:scale"+a) for a in "XYZ"])
            if shape == "RigExecSphereWeight":
                for i, a in enumerate("XYZ"):
                    scales[i] *= read("inputs:scale"+a+("Pos" if local[i]>=0 else "Neg"))
            if any(s <= 0 for s in scales):
                return False
            normalized = Gf.Vec3d(*[local[i]/scales[i] for i in range(3)])
            center = Gf.Vec3d(0)
            if shape == "RigExecCurveWeight":
                targets = prim.GetRelationship("rigExec:curve").GetTargets()
                if not targets:
                    return False
                attr = self.api.stage.GetAttributeAtPath(targets[0]) if targets[0].IsPropertyPath() else self.api.stage.GetPrimAtPath(targets[0]).GetAttribute("points")
                points = attr.Get(time) if attr else []
                if points is None or len(points) < 2:
                    return False
                inverse = frame.GetInverse()
                localPoints = [inverse.Transform(Gf.Vec3d(p)) for p in points]
                localPoints = [Gf.Vec3d(*[p[i]/scales[i] for i in range(3)]) for p in localPoints]
                center = NearestOnCurve(normalized, localPoints)
            radial = normalized-center
            distance = radial.GetLength()
            if distance < 1e-6:
                return False
            direction = Gf.Vec3d(*[radial[i]*scales[i]/distance for i in range(3)])
            name = "inputs:falloffMin" if abs(distance-inner) < abs(distance-outer) else "inputs:falloffMax"
            if shift and shape == "RigExecSphereWeight":
                axis = max(range(3), key=lambda i: abs(normalized[i]))
                name = "inputs:scale"+"XYZ"[axis]+("Pos" if local[axis]>=0 else "Neg")
                direction = Gf.Vec3d(0)
                direction[axis] = local[axis]/max(read(name), 1e-6)
                lower = 1e-4
        if name == "inputs:falloffMin":
            upper = outer-1e-4
        elif name == "inputs:falloffMax":
            lower = max(lower, inner+1e-4)
        attr = prim.GetAttribute(name)
        # A connected or mover-driven input would not follow its authored value.
        if (not attr or attr.HasAuthoredConnections() or
                self._MovedFloat(attr.GetPath()) is not None):
            return False
        camera, _ = self.view.resolveCamera()
        projection = gizmoScreen.ViewProjection(camera)
        viewport = self.view.computeWindowViewport()
        a = gizmoScreen.ProjectPoint(projection, viewport, world.Transform(local))
        b = gizmoScreen.ProjectPoint(projection, viewport, world.Transform(local+direction))
        if a is None or b is None:
            return False
        screenDirection = ((b[0]-a[0])/ratio, (b[1]-a[1])/ratio)
        if math.hypot(*screenDirection) < 1e-3:
            # A face-on shell normal has no screen projection. Horizontal
            # movement remains usable at the same world-units-per-pixel scale.
            units = gizmoScreen.WorldPerPixel(camera, viewport, world.Transform(local))
            if units is None or units <= 0:
                return False
            screenDirection = (world.TransformDir(direction).GetLength()/units/ratio, 0.0)
        self.drag = [None, attr, read(name), (pos.x(), pos.y()),
                     screenDirection, lower, upper]
        self._dragPrim = prim
        self._dragAdds = bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        self.view.grabMouse()
        return True
