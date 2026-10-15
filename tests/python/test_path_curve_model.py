#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/pathCurveModel.py: dragging a path
curve's control points.

Which curves are paths (a path mover inside a rig names them), one handle
per control point at its projection, the nearest handle picked, a drag that
moves the grabbed point in the curve's horizontal plane under the cursor
with its local Y kept bit for bit, the whole points array previewed and
nothing authored until the release, which authors the default -- or a key,
when the points are keyed -- under the drag's undo recorder.

The drag runs through the same gizmoDrag.ApplyDrag the manipulator uses,
with a synthetic Gf.Camera at (0, 8, 8) looking at the origin, so the
ground plane is seen at 45 degrees. Usage: test_path_curve_model.py
[ignored]
"""
import math
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd, UsdGeom, Vt  # noqa: E402

import gizmoDrag  # noqa: E402
import gizmoMath  # noqa: E402
import gizmoPreview  # noqa: E402
import gizmoScreen  # noqa: E402
import gizmoSettings  # noqa: E402
import pathCurveModel  # noqa: E402
import rigExecUndo  # noqa: E402

VIEWPORT = (0, 0, 800, 600)
RIG = "/Mech/Rig"
PATH = Sdf.Path("/Mech/Rig/Path")
POINTS = PATH.AppendProperty("points")
AUTHORED = [(0, 0, 0), (0.5, 0.25, 1), (0, 0.5, 2), (-0.5, 0, 3),
            (0.5, 0, 4)]
FRAME = Usd.TimeCode(5)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(0, 8, 8), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 1, 0)).GetInverse()
    # Conformed to the viewport, as usdview's StageView does before drawing.
    camera.verticalAperture = (camera.horizontalAperture
                               * VIEWPORT[3] / float(VIEWPORT[2]))
    return camera


def _Points(points):
    return Vt.Vec3fArray([Gf.Vec3f(*p) for p in points])


def _Curve(stage, path, points):
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr().Set(UsdGeom.Tokens.cubic)
    curve.CreateBasisAttr().Set(UsdGeom.Tokens.catmullRom)
    curve.CreateWrapAttr().Set(UsdGeom.Tokens.pinned)
    curve.CreateCurveVertexCountsAttr().Set(Vt.IntArray([len(points)]))
    curve.CreatePointsAttr().Set(_Points(points))
    return curve


def _Mover(stage, path, operation, curve=None, target=None):
    """A float math mover; authored, not schema fallbacks, so the test
    reads the same with or without the schema plugin registered."""
    mover = stage.DefinePrim(path, "RigExecFloatMathMover")
    mover.CreateAttribute("rigExec:operation", Sdf.ValueTypeNames.Token,
                          False, Sdf.VariabilityUniform).Set(operation)
    if target is not None:
        mover.CreateRelationship("rigExec:moves").SetTargets([target])
    if curve is not None:
        mover.CreateRelationship("rigExec:path").SetTargets([curve])
    return mover


def _Stage(parent=None):
    """
    /Mech (an Xform, `parent` its matrix) holds the rig. Path is sampled by
    the X and Z movers; Other only by a curve-operation mover; Loose by a
    path mover outside any rig; and a third path mover names a control.
    """
    stage = Usd.Stage.CreateInMemory()
    mech = UsdGeom.Xform.Define(stage, "/Mech")
    if parent is not None:
        mech.AddTransformOp().Set(parent)
    stage.DefinePrim(RIG, "RigExecRoot")
    root = stage.DefinePrim(RIG + "/Controls/Root", "RigExecControl")
    tx = root.CreateAttribute("avars:tx", Sdf.ValueTypeNames.Double)
    tz = root.CreateAttribute("avars:tz", Sdf.ValueTypeNames.Double)
    _Curve(stage, PATH, AUTHORED)
    _Curve(stage, RIG + "/Other", AUTHORED)
    _Curve(stage, "/Loose/Curve", AUTHORED)
    _Mover(stage, RIG + "/Movers/X", "path", PATH, tx.GetPath())
    _Mover(stage, RIG + "/Movers/Z", "path", PATH, tz.GetPath())
    _Mover(stage, RIG + "/Movers/Keys", "curve", RIG + "/Other",
           tx.GetPath())
    _Mover(stage, RIG + "/Movers/OnControl", "path", root.GetPath(),
           tx.GetPath())
    _Mover(stage, "/Loose/Mover", "path", "/Loose/Curve")
    return stage


def _Target(stage, mode=gizmoMath.WRITE_ANIMATION, time=FRAME):
    return pathCurveModel.MakeTarget(stage, stage.GetPrimAtPath(PATH), time,
                                     mode)


def _Drag(target, index, offset, camera=None):
    """Grab point `index` and drag it `offset` pixels: what gizmoUI does."""
    camera = camera or _Camera()
    handle = target.BuildHandles(camera, VIEWPORT, 1.0)[index]
    target.Grab(handle)
    target.BeginDrag()
    state = gizmoDrag.DragState(gizmoScreen.TOOL_TRANSLATE, handle, target,
                                handle.center, camera, VIEWPORT)
    current = (handle.center[0] + offset[0], handle.center[1] + offset[1])
    gizmoDrag.ApplyDrag(state, current, gizmoSettings.ToolDefaults(
        gizmoScreen.TOOL_TRANSLATE))
    return current


def _Screen(camera, world):
    return gizmoScreen.ProjectPoint(gizmoScreen.ViewProjection(camera),
                                    VIEWPORT, world)


def TestFindPathCurves():
    stage = _Stage()
    curves = pathCurveModel.FindPathCurves(stage)
    _Check(list(curves) == [PATH],
           "only the curve a rig's path mover samples: %s" % list(curves))
    _Check([m.GetName() for m in curves[PATH]] == ["X", "Z"],
           "both movers that sample it: %s" % curves[PATH])

    target = _Target(stage)
    _Check(isinstance(target, pathCurveModel.PathCurveTarget),
           "the selected path curve is a control point target")
    _Check(target.supportsTranslate and not target.supportsRotate and
           not target.supportsScale, "Move only")
    _Check(target.AttributePaths() == [POINTS], "the undo set is the points")
    _Check(target.RigRootPath() == Sdf.Path(RIG), "the movers' rig")
    _Check(list(target.points) == list(_Points(AUTHORED)),
           "the authored points: %s" % target.points)
    for path in (RIG + "/Other", "/Loose/Curve", RIG + "/Controls/Root"):
        _Check(pathCurveModel.MakeTarget(
            stage, stage.GetPrimAtPath(path), FRAME,
            gizmoMath.WRITE_ANIMATION) is None,
            "%s is no path curve" % path)

    # Once no path mover samples it, a refresh refuses, so the controller
    # resolves the selection again.
    stage.GetPrimAtPath(RIG + "/Movers/X").GetRelationship(
        "rigExec:path").SetTargets([RIG + "/Other"])
    target.Refresh()
    stage.GetPrimAtPath(RIG + "/Movers/Z").GetAttribute(
        "rigExec:operation").Set("clamp")
    try:
        target.Refresh()
    except ValueError:
        pass
    else:
        raise AssertionError("a curve no mover samples still refreshed")


def TestWriteMode():
    stage = _Stage()
    _Check(_Target(stage).writer.mode == gizmoMath.WRITE_DEFAULT,
           "unkeyed points take their default under Animation")
    _Check(_Target(stage, gizmoMath.WRITE_DEFAULT).writer.mode ==
           gizmoMath.WRITE_DEFAULT, "and under Default")
    attr = stage.GetAttributeAtPath(POINTS)
    attr.Set(_Points(AUTHORED), Usd.TimeCode(1))
    _Check(_Target(stage).writer.mode == gizmoMath.WRITE_ANIMATION,
           "keyed points take a key under Animation")
    _Check(_Target(stage, gizmoMath.WRITE_DEFAULT).writer.mode ==
           gizmoMath.WRITE_DEFAULT, "and their default under Default")


def TestHandles():
    camera = _Camera()
    parent = (Gf.Matrix4d().SetScale(Gf.Vec3d(2.0, 1.5, 0.5)) *
              Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 30)) *
              Gf.Matrix4d().SetTranslate(Gf.Vec3d(1, 0.5, -2)))
    target = _Target(_Stage(parent))
    handles = target.BuildHandles(camera, VIEWPORT, 2.0)
    _Check([h.name for h in handles] == ["point%d" % i for i in range(5)],
           "one handle per control point: %s" % handles)
    for index, handle in enumerate(handles):
        world = parent.Transform(Gf.Vec3d(*AUTHORED[index]))
        screen = _Screen(camera, world)
        _Check(handle.kind == "point" and handle.pointIndex == index,
               "a point handle that knows its index")
        _Check(Gf.IsClose(handle.worldCenter, world, 1e-6) and
               Gf.IsClose(handle.worldOrigin, world, 1e-6),
               "centred on the drawn point: %s" % handle.worldCenter)
        _Check(math.hypot(handle.center[0] - screen[0],
                          handle.center[1] - screen[1]) < 1e-6,
               "at its projection: %s %s" % (handle.center, screen))
        _Check(Gf.IsClose(handle.worldNormal, Gf.Vec3d(0, 1, 0), 1e-9),
               "a curve turned about Y stays horizontal: %s"
               % handle.worldNormal)
        _Check(abs(handle.sizePixels - 2.0 * pathCurveModel.POINT_PIXELS)
               < 1e-9, "sized in physical pixels")

    # A tilted curve drags in its own tilted plane.
    tilt = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 20))
    normal = pathCurveModel.HorizontalNormal(tilt)
    _Check(Gf.IsClose(normal, tilt.TransformDir(Gf.Vec3d(0, 1, 0)), 1e-9),
           "the tilted horizontal plane: %s" % normal)
    # A point behind the camera has no handle; the rest keep their index.
    behind = list(AUTHORED) + [(0, 16, 16)]
    handles = pathCurveModel.ControlPointHandles(
        _Points(behind), Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)
    _Check([h.pointIndex for h in handles] == list(range(5)),
           "a point behind the eye is skipped: %s" % handles)
    _Check(pathCurveModel.ControlPointHandles(
        _Points(AUTHORED), Gf.Matrix4d().SetScale(Gf.Vec3d(1, 0, 1)),
        camera, VIEWPORT, 1.0) == [], "a collapsed curve has no handles")


def TestPickNearest():
    """gizmoScreen.HitTest picks the control point nearest the cursor."""
    camera = _Camera()
    close = [(0, 0, 0), (0.02, 0, 0), (2, 0, 0)]
    handles = pathCurveModel.ControlPointHandles(
        _Points(close), Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)
    a, b = handles[0].center, handles[1].center
    _Check(math.hypot(b[0] - a[0], b[1] - a[1]) <
           gizmoScreen.HIT_PIXELS, "two points within one pick radius")
    for share, expected in ((0.3, 0), (0.7, 1)):
        x = a[0] + share * (b[0] - a[0])
        y = a[1] + share * (b[1] - a[1]) + 2.0
        hit = gizmoScreen.HitTest(handles, x, y, gizmoScreen.HIT_PIXELS)
        _Check(hit is not None and hit.pointIndex == expected,
               "%.1f of the way picks point %d: %s" % (share, expected, hit))
    far = handles[2].center
    _Check(gizmoScreen.HitTest(handles, far[0] + 4.0, far[1] - 3.0,
                               gizmoScreen.HIT_PIXELS).pointIndex == 2,
           "a press beside a point picks it")
    _Check(gizmoScreen.HitTest(handles, far[0] + 30.0, far[1],
                               gizmoScreen.HIT_PIXELS) is None,
           "a press away from every point picks none")


def _AssertDragged(label, target, camera, index, current, parent):
    points = target.writer.Pending()[POINTS]
    authored = _Points(AUTHORED)
    _Check(len(points) == len(authored), "%s: the whole array" % label)
    for i in range(len(authored)):
        if i != index:
            _Check(points[i] == authored[i],
                   "%s: point %d did not move" % (label, i))
    _Check(points[index][1] == authored[index][1],
           "%s: the local Y is kept exactly: %s" % (label, points[index]))
    _Check(points[index] != authored[index], "%s: the point moved" % label)
    world = parent.Transform(Gf.Vec3d(points[index]))
    screen = _Screen(camera, world)
    _Check(math.hypot(screen[0] - current[0], screen[1] - current[1]) < 1e-3,
           "%s: the point stays under the cursor: %s %s"
           % (label, screen, current))


def TestDragFollowsCursorInPlane():
    camera = _Camera()
    for label, parent in (
            ("identity", Gf.Matrix4d(1.0)),
            ("scaled and turned",
             Gf.Matrix4d().SetScale(Gf.Vec3d(2.0, 1.5, 0.5)) *
             Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), 30)) *
             Gf.Matrix4d().SetTranslate(Gf.Vec3d(1, 0.5, -2))),
            ("tilted",
             Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 15)))):
        stage = _Stage(parent)
        target = _Target(stage)
        before = stage.GetRootLayer().ExportToString()
        current = _Drag(target, 2, (40.0, 25.0), camera)
        _AssertDragged(label, target, camera, 2, current, parent)
        _Check(target.label == "Path point 2",
               "%s: the edit names the point: %s" % (label, target.label))
        _Check(stage.GetRootLayer().ExportToString() == before,
               "%s: a drag authors nothing" % label)
        # The handles follow the previewed points.
        target.RefreshDuringDrag()
        moved = target.BuildHandles(camera, VIEWPORT, 1.0)[2].center
        _Check(math.hypot(moved[0] - current[0], moved[1] - current[1])
               < 1e-3, "%s: the handle is redrawn under the cursor: %s"
               % (label, moved))


def TestReleaseAuthorsTheDefault():
    stage = _Stage()
    target = _Target(stage)
    attr = stage.GetAttributeAtPath(POINTS)
    recorder = rigExecUndo.EditRecorder(stage, target.AttributePaths())
    recorder.Begin()
    _Drag(target, 1, (-30.0, 10.0))
    dragged = target.writer.Pending()[POINTS]
    _Check(target.writer.CommitToStage() == [POINTS], "the points authored")
    edit = recorder.Commit(target.label)
    _Check(attr.Get() == dragged and attr.GetNumTimeSamples() == 0,
           "as the default, no key: %s" % attr.GetTimeSamples())
    _Check(not target.writer.Warnings(), "with nothing outranking it")
    _Check(POINTS not in gizmoMath.PreviewValues(stage),
           "and no longer previewed")
    _Check(edit is not None and edit.label == "Path point 1",
           "one undo entry for the drag")
    edit.Undo()
    _Check(attr.Get() == _Points(AUTHORED), "undo restores the points")
    edit.Redo()
    _Check(attr.Get() == dragged, "redo moves them again")


def TestKeyedPointsTakeAKey():
    stage = _Stage()
    attr = stage.GetAttributeAtPath(POINTS)
    later = Vt.Vec3fArray(_Points(AUTHORED))
    later[3] = Gf.Vec3f(-1.5, 0, 3)
    attr.Set(_Points(AUTHORED), Usd.TimeCode(1))
    attr.Set(later, Usd.TimeCode(10))
    target = _Target(stage)
    _Check(target.points == attr.Get(FRAME), "the points at the frame")
    _Drag(target, 3, (25.0, -15.0))
    dragged = target.writer.Pending()[POINTS]
    target.writer.CommitToStage()
    _Check(attr.GetTimeSamples() == [1.0, 5.0, 10.0],
           "a key at the frame, the others kept: %s" % attr.GetTimeSamples())
    _Check(attr.Get(FRAME) == dragged and attr.Get(10) == later,
           "holding the dragged points")

    # Under Default the writer authors the default and says it is hidden.
    target = _Target(stage, gizmoMath.WRITE_DEFAULT)
    _Drag(target, 3, (5.0, 5.0))
    target.writer.CommitToStage()
    _Check(any("outranked" in w for w in target.writer.Warnings()),
           "the outranked default is reported: %s"
           % target.writer.Warnings())


def TestSnapStep():
    points = _Points(AUTHORED)
    moved = pathCurveModel.MovePoint(points, 1, Gf.Vec3d(0.74, 3.0, -0.26),
                                     Gf.Matrix4d(1.0), snapStep=0.5)
    _Check(moved[1] == Gf.Vec3f(1.0, 0.25, 0.5),
           "x and z travel in whole steps, Y kept: %s" % moved[1])
    _Check(points == _Points(AUTHORED), "the input array is untouched")
    moved = pathCurveModel.MovePoint(points, 1, Gf.Vec3d(0.74, 3.0, -0.26),
                                     Gf.Matrix4d(1.0))
    _Check(moved[1] == Gf.Vec3f(1.24, 0.25, 0.74), "unsnapped: %s"
           % moved[1])


class _Sink(object):
    """Expects 3 doubles per declared point, 1 per other slot."""

    def __init__(self):
        self.declarations = []
        self.samples = []

    def Begin(self, packedPaths):
        self.declarations.append(packedPaths)
        return sum(3 * int(line.split("\t")[1]) if "\t" in line else 1
                   for line in packedPaths.split("\n"))

    def Update(self, values):
        self.samples.append(list(values))
        return True

    def End(self, publish=True):
        return True


def TestPreviewCarriesThePoints():
    stage = _Stage()
    target = _Target(stage)
    sink = _Sink()
    gizmoPreview.SetSink(sink)
    try:
        _Drag(target, 2, (10.0, 0.0))
        pending = target.writer.Pending()
        _Check(gizmoPreview.Push(pending, stage=stage),
               "the sink took the points")
        _Check(sink.declarations == ["%s\t5" % POINTS],
               "declared with their count: %s" % sink.declarations)
        _Check(sink.samples[-1] == [float(c) for p in pending[POINTS]
                                    for c in p],
               "x, y, z per point: %s" % sink.samples[-1])
        gizmoPreview.End(stage=stage)
        _Check(POINTS not in gizmoMath.PreviewValues(stage),
               "the end drops the previewed points")
        # An abort drops the values; the stage still holds the authored ones.
        target.writer.Clear()
        target.Refresh()
        _Check(target.points == _Points(AUTHORED),
               "an abandoned drag rereads the authored points")
    finally:
        gizmoPreview.SetSink(None)
        gizmoMath.SetPreviewValues({})


def TestNoGrabMovesNothing():
    target = _Target(_Stage())
    target.BeginDrag()
    target.ApplyTranslate(Gf.Vec3d(1, 0, 0))
    _Check(not target.writer.HasPending(), "no point grabbed, no edit")


def main():
    groups = (
        ("path curves", TestFindPathCurves),
        ("write mode", TestWriteMode),
        ("handles", TestHandles),
        ("pick the nearest point", TestPickNearest),
        ("drag in the plane, under the cursor",
         TestDragFollowsCursorInPlane),
        ("release authors the default", TestReleaseAuthorsTheDefault),
        ("keyed points take a key", TestKeyedPointsTakeAKey),
        ("snap step", TestSnapStep),
        ("preview carries the points", TestPreviewCarriesThePoints),
        ("no grab, no edit", TestNoGrabMovesNothing),
    )
    for name, fn in groups:
        try:
            fn()
        finally:
            gizmoMath.SetPreviewValues({})
        print("  ok: %s" % name)
    print("PATH_CURVE_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
