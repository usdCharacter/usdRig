#
# RigExec usdview: the control points of a path curve, as gizmo handles.
#
# A RigExecFloatMathMover whose rigExec:operation is "path" samples the
# UsdGeomBasisCurves its rigExec:path names, reading the curve's points as
# authored (libs/rigExecSchema/schema.usda). Selecting such a curve with the
# Move tool puts a handle on every control point. Dragging one moves that
# point in the curve's horizontal plane -- its local Y kept, the plane the
# path's heading turns in -- as an ordinary gizmo edit of the one points
# attribute: gizmoDrag maps the mouse through the handle's plane, the whole
# array goes to Hydra through gizmoPreview so the rig follows while the
# mouse is down, and gizmoMath.Writer authors it on release under the
# drag's undo recorder.
#
# Qt-free: which curves are paths, where their handles are, and what a drag
# does to the points are decided here. gizmoUI builds the target and paints
# the handles; gizmoScreen.HitTest picks the one nearest the cursor.
#
import os
import sys
from collections import OrderedDict

from pxr import Gf, UsdGeom, Vt

try:
    import gizmoMath
    import gizmoScreen
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath
    import gizmoScreen

PATH_MOVER_TYPE = "RigExecFloatMathMover"
PATH_OPERATION = "path"
OPERATION_ATTR = "rigExec:operation"
PATH_REL = "rigExec:path"

# A control point handle: gizmoScreen picks it by its centre, gizmoDrag moves
# it in the plane its worldNormal names, gizmoSnap constrains it there.
HANDLE_KIND = "point"
# The drawn square's side, in LOGICAL pixels.
POINT_PIXELS = 9.0
COLOR_POINT = (0.92, 0.92, 0.92)


def IsPathMover(prim):
    """Whether `prim` is a float math mover running the path operation."""
    if not prim or prim.GetTypeName() != PATH_MOVER_TYPE:
        return False
    operation = prim.GetAttribute(OPERATION_ATTR)
    return bool(operation) and operation.Get() == PATH_OPERATION


def NamesCurve(mover, curvePath):
    """Whether path mover `mover` samples the curve at `curvePath`."""
    relationship = mover.GetRelationship(PATH_REL)
    return bool(relationship) and curvePath in relationship.GetTargets()


def FindPathCurves(stage):
    """
    {curve path: [mover, ...]} for every path mover inside a RigExecRoot
    whose rigExec:path names a UsdGeomBasisCurves prim, in traversal order.

    usdview activates every RigExecRoot on the stage, so a mover in any rig
    counts. One outside a rig is never evaluated and makes no path.
    """
    curves = OrderedDict()
    if not stage:
        return curves
    for prim in stage.Traverse():
        if not IsPathMover(prim) or gizmoMath.FindRigRoot(prim) is None:
            continue
        relationship = prim.GetRelationship(PATH_REL)
        if not relationship:
            continue
        for path in relationship.GetTargets():
            curve = stage.GetPrimAtPath(path)
            if curve and curve.IsA(UsdGeom.BasisCurves):
                curves.setdefault(path, []).append(prim)
    return curves


def WriteMode(attr, requested):
    """
    How a drag of `attr`'s points authors: their default, unless the points
    are keyed -- then `requested` (the toolbar's mode), which gizmoMath.Writer
    applies to keyed points as to any keyed attribute.
    """
    if attr and attr.GetNumTimeSamples() > 0:
        return requested
    return gizmoMath.WRITE_DEFAULT


def HorizontalNormal(localToWorld):
    """
    The world normal of the curve's horizontal plane: the plane of constant
    local Y, the one the path's heading turns in. A transformed plane's
    normal, so a sheared or non-uniformly scaled curve is still exact.
    """
    plane = Gf.Plane(Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 0))
    plane.Transform(localToWorld)
    return plane.GetNormal()


def MovePoint(points, index, worldDelta, localToWorld, snapStep=None):
    """
    A copy of `points` with point `index` moved by `worldDelta` taken into
    the curve's local space, in its horizontal plane: the local Y is the
    authored float, kept bit for bit, whatever the delta's vertical part.

    `snapStep` (Step Snap, J) rounds the local x and z travel to whole steps,
    so a point that started off the grid stays off it.
    """
    local = localToWorld.GetInverse().TransformDir(Gf.Vec3d(worldDelta))
    dx, dz = local[0], local[2]
    if snapStep:
        dx = gizmoScreen.SnapRelative(dx, snapStep)
        dz = gizmoScreen.SnapRelative(dz, snapStep)
    moved = Vt.Vec3fArray(points)
    point = moved[index]
    moved[index] = Gf.Vec3f(point[0] + dx, point[1], point[2] + dz)
    return moved


def ControlPointHandles(points, localToWorld, camera, viewport, pixelRatio):
    """
    One "point" handle per control point in front of the camera, named
    "point<i>" and carrying `pointIndex`: drawn and picked at the projected
    point (physical pixels), with the world point as its centre and origin
    and the horizontal plane's normal as its worldNormal. No handles for a
    curve whose transform has collapsed.
    """
    if abs(localToWorld.GetDeterminant()) < 1e-12:
        return []
    viewProjection = gizmoScreen.ViewProjection(camera)
    normal = HorizontalNormal(localToWorld)
    size = POINT_PIXELS * pixelRatio
    handles = []
    for index, point in enumerate(points):
        world = localToWorld.Transform(Gf.Vec3d(point))
        screen = gizmoScreen.ProjectPoint(viewProjection, viewport, world)
        if screen is None:
            continue
        handle = gizmoScreen.Handle(
            "point%d" % index, HANDLE_KIND, None, [screen], None, 0.0,
            COLOR_POINT, screen, worldCenter=world, worldOrigin=world,
            worldCenterScreen=screen, worldNormal=normal, sizePixels=size)
        handle.pointIndex = index
        handles.append(handle)
    return handles


class PathCurveTarget(gizmoMath.Target):
    """
    A path curve's control points as one gizmo target: Move only, one point
    per drag (Grab names it), the whole points array written through the
    writer so the preview and the commit carry the curve as the movers read
    it.
    """

    kind = "path curve"
    supportsRotate = False
    supportsScale = False

    def __init__(self, stage, prim, writer, movers, requestedMode=None):
        super(PathCurveTarget, self).__init__(stage, prim, writer)
        self.movers = list(movers)
        self.attr = UsdGeom.BasisCurves(prim).GetPointsAttr()
        root = gizmoMath.FindRigRoot(self.movers[0]) if self.movers else None
        self._rigRoot = root.GetPath() if root is not None else None
        self._requestedMode = requestedMode or writer.mode
        self._curveName = prim.GetName()
        self.points = Vt.Vec3fArray()
        self.localToWorld = Gf.Matrix4d(1.0)
        self.grabbed = None
        self._basePoints = None
        self.Refresh()

    def Refresh(self, frameCache=None):
        """
        Re-read the points (the previewed ones while a drag holds them) and
        the curve's transform. Raises ValueError once no path mover samples
        the curve any more, so the controller resolves the selection again.
        """
        path = self.prim.GetPath() if self.prim.IsValid() else None
        if path is None or not any(
                mover.IsValid() and IsPathMover(mover) and
                NamesCurve(mover, path) for mover in self.movers):
            raise ValueError("%s is no longer a path mover's curve"
                             % self._curveName)
        value = gizmoMath.PreviewValues(self.stage).get(self.attr.GetPath())
        if value is None:
            value = self.attr.Get(self.time)
        self.points = (Vt.Vec3fArray(value) if value is not None
                       else Vt.Vec3fArray())
        self.localToWorld = UsdGeom.Xformable(
            self.prim).ComputeLocalToWorldTransform(self.time)

    def RigRootPath(self):
        """The rig of the first mover that samples the curve."""
        return self._rigRoot

    def GizmoMatrix(self):
        """The curve's frame, placed at the grabbed point (else the first)."""
        matrix = Gf.Matrix4d(self.localToWorld)
        if len(self.points):
            index = self.grabbed if self.grabbed is not None else 0
            matrix.SetTranslateOnly(
                self.localToWorld.Transform(Gf.Vec3d(self.points[index])))
        return matrix

    def ChannelFrame(self):
        return self.GizmoMatrix()

    def AttributePaths(self):
        return [self.attr.GetPath()]

    def BuildHandles(self, camera, viewport, pixelRatio):
        """The control point handles, in place of the manipulator's."""
        return ControlPointHandles(self.points, self.localToWorld, camera,
                                   viewport, pixelRatio)

    def Grab(self, handle):
        """The control point the next drag moves: `handle`'s, at the press."""
        self.grabbed = getattr(handle, "pointIndex", None)
        if self.grabbed is not None:
            self.label = "%s point %d" % (self._curveName, self.grabbed)

    def BeginDrag(self, frameCache=None):
        # Decided per drag, from whether the points are keyed now.
        self.writer.mode = WriteMode(self.attr, self._requestedMode)
        self.Refresh()
        self._basePoints = Vt.Vec3fArray(self.points)

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        """
        Move the grabbed point by a WORLD delta, from the drag base, in its
        horizontal plane. `snapAbsolute` has no channel grid to land on here
        and is ignored; the world grid arrives as a plain delta.
        """
        base = self._basePoints
        if base is None or self.grabbed is None or \
                not 0 <= self.grabbed < len(base):
            return
        self.points = MovePoint(base, self.grabbed, worldDelta,
                                self.localToWorld, snapStep)
        self.writer.Set(self.attr, self.points)


def MakeTarget(stage, prim, time, writeMode):
    """
    A PathCurveTarget for `prim`, or None when it is not a curve some rig's
    path mover samples. `writeMode` is the toolbar's; see WriteMode.
    """
    if not stage or not prim or not prim.IsValid() or \
            not prim.IsA(UsdGeom.BasisCurves):
        return None
    movers = FindPathCurves(stage).get(prim.GetPath())
    if not movers:
        return None
    attr = UsdGeom.BasisCurves(prim).GetPointsAttr()
    writer = gizmoMath.Writer(stage, time, WriteMode(attr, writeMode))
    return PathCurveTarget(stage, prim, writer, movers, writeMode)
