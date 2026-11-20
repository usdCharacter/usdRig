#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoTargets.py.

Usage: test_gizmo_targets.py [<generated schema resources dir>]

Exercises the targets owner directly: a group transform delta over an
in-memory stage, a time sample authored through the writer, and the
facade aliases (SetAnimated/AnimatedWrites resolve to the same objects
from gizmoMath and gizmoTargets).
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Usd, UsdGeom  # noqa: E402

import gizmoFrames  # noqa: E402
import gizmoMath  # noqa: E402
import gizmoTargets  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _Rot(axis, deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, deg))
    return m


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


def _PairStage():
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    first = stage.DefinePrim("/Asset/Rig/Controls/First", "RigExecControl")
    space = _Rot(Gf.Vec3d(0, 0, 1), 90.0)
    space.SetTranslateOnly(Gf.Vec3d(0, 5, 0))
    first.GetAttribute("rest:space").Set(space)
    second = stage.DefinePrim("/Asset/Rig/Controls/Second", "RigExecControl")
    sideSpace = _Rot(Gf.Vec3d(0, 1, 0), 40.0)
    sideSpace.SetTranslateOnly(Gf.Vec3d(0, -8, 12))
    second.GetAttribute("rest:space").Set(sideSpace)
    second.GetAttribute("avars:ty").Set(2.0)
    return stage, first, second


def _WorldOrigin(stage, prim, time):
    frames = gizmoFrames.ComputeRigFrames(stage, prim, time)
    return (frames.posed * frames.assetToWorld).ExtractTranslation()


def TestFacadeAliases():
    # The rv-gizmo-ui contract names first: the gesture state may import
    # these from either module, and both resolve to the same objects.
    for name in ("SetAnimated", "AnimatedWrites", "Writer", "MakeTarget",
                 "MakeGroupTarget", "GroupTarget", "RigPoseTarget",
                 "RigPivotTarget", "XformPoseTarget", "XformPivotTarget",
                 "WRITE_ANIMATION", "WRITE_DEFAULT", "CHANNELS_POSE",
                 "CHANNELS_PIVOT", "GROUP_PIVOT_CENTER", "GROUP_PIVOT_LEAD",
                 "GROUP_PIVOT_INDIVIDUAL"):
        _Check(getattr(gizmoMath, name) is getattr(gizmoTargets, name),
               "gizmoMath.%s is the targets owner's" % name)


def TestGroupTransformDelta():
    stage, first, second = _PairStage()
    time = Usd.TimeCode.Default()
    writer = gizmoTargets.Writer(stage, time, gizmoTargets.WRITE_DEFAULT)
    group, reason = gizmoTargets.MakeGroupTarget(
        stage, [first, second], gizmoTargets.CHANNELS_POSE, writer)
    _Check(group is not None, "pair refused: %s" % reason)
    before = {p.GetName(): _WorldOrigin(stage, p, time)
              for p in (first, second)}
    delta = Gf.Vec3d(2.5, -3.25, 7.0)
    group.BeginDrag()
    group.ApplyTranslate(delta)
    _Check(writer.HasPending(), "the drag collects before the release")
    writer.CommitToStage()
    group.Refresh()
    for prim in (first, second):
        after = _WorldOrigin(stage, prim, time)
        start = before[prim.GetName()]
        _Check(all(_Close(after[i], start[i] + delta[i], 1e-9)
                   for i in range(3)),
               "%s moved by the group delta: %s vs %s"
               % (prim.GetName(), Gf.Vec3d(after) - Gf.Vec3d(start), delta))
    for prim in (first, second):
        for name in gizmoFrames.AVAR_T:
            _Check(prim.GetAttribute(name).HasAuthoredValue(),
                   "%s authored %s" % (prim.GetName(), name))


def TestWriterAuthorsTimeSample():
    stage, first, _second = _PairStage()
    frame = Usd.TimeCode(1001.0)
    anim = gizmoTargets.Writer(stage, frame, gizmoTargets.WRITE_ANIMATION)
    box = UsdGeom.Xform.Define(stage, "/Asset/Box")
    op = box.AddTranslateOp()
    anim.Set(op.GetAttr(), Gf.Vec3d(1, 2, 3))
    _Check(op.GetAttr().GetNumTimeSamples() == 0,
           "collecting authors nothing")
    authored = anim.CommitToStage()
    _Check(authored == [op.GetAttr().GetPath()],
           "the commit reports what it authored")
    _Check(op.GetAttr().GetNumTimeSamples() == 1,
           "a vector attribute authors one time sample")
    sample = op.GetAttr().Get(frame)
    _Check(all(_Close(sample[i], want) for i, want in enumerate((1, 2, 3))),
           "sample value: %s" % (sample,))
    # A scalar avar through the same writer lands on a spline knot, not a
    # sample: that is the SetAnimated split the writer implements.
    attr = first.GetAttribute("avars:tx")
    anim.Set(attr, 4.0)
    anim.CommitToStage()
    _Check(attr.HasSpline() and len(attr.GetSpline().GetKnots()) == 1,
           "a scalar avar authors one spline knot")
    _Check(_Close(attr.Get(frame), 4.0), "knot value")


def main():
    _RegisterSchema()
    groups = [
        ("facade aliases", TestFacadeAliases),
        ("group transform delta", TestGroupTransformDelta),
        ("writer authors a time sample", TestWriterAuthorsTimeSample),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_TARGETS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
