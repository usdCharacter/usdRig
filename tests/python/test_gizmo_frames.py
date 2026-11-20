#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoFrames.py.

Usage: test_gizmo_frames.py [<generated schema resources dir>]

Exercises the frames owner directly: rest/default/posed space conversion
on an in-memory stage, and the facade aliases that keep every name
importable from gizmoMath.
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom  # noqa: E402

import gizmoFrames  # noqa: E402
import gizmoMath  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _MatClose(a, b, tol=1e-6):
    return all(_Close(a[r][c], b[r][c], tol)
               for r in range(4) for c in range(4))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


def _Rot(axis, deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, deg))
    return m


def _ChainStage():
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    parent = stage.DefinePrim("/Asset/Rig/Controls/Parent", "RigExecControl")
    child = stage.DefinePrim(
        "/Asset/Rig/Controls/Parent/Child", "RigExecControl")
    space = _Rot(Gf.Vec3d(0, 0, 1), 90.0)
    space.SetTranslateOnly(Gf.Vec3d(0, 5, 0))
    parent.GetAttribute("rest:space").Set(space)
    parent.GetAttribute("avars:tx").Set(1.0)
    parent.GetAttribute("avars:rz").Set(30.0)
    parent.GetAttribute("avars:sx").Set(2.0)
    child.GetAttribute("rest:space").Set(space)
    child.GetAttribute("rest:tx").Set(3.0)
    child.GetAttribute("rest:ry").Set(45.0)
    child.GetAttribute("avars:ty").Set(0.5)
    child.GetAttribute("avars:rx").Set(20.0)
    child.GetAttribute("avars:rotationOrder").Set("ZYX")
    return stage, parent, child


def TestFacadeAliases():
    for name in ("ComputeRigFrames", "RestSpace", "RestLocal", "AvarsMatrix",
                 "FindRigRoot", "ScalarAvar", "RigFrames", "AVAR_T", "AVAR_R",
                 "AVAR_S", "REST_T", "REST_R", "REST_SPACE", "POSED_SPACE",
                 "DEFAULT_T", "DEFAULT_R", "DEFAULT_SPACE"):
        _Check(getattr(gizmoMath, name) is getattr(gizmoFrames, name),
               "gizmoMath.%s is the frames owner's" % name)


def TestPosedVersusRest():
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    pf = gizmoFrames.ComputeRigFrames(stage, parent, t)
    cf = gizmoFrames.ComputeRigFrames(stage, child, t)
    _Check(pf.reason == "" and cf.reason == "", "chain is editable")
    _Check(pf.rigRoot.GetPath() == Sdf.Path("/Asset/Rig"), "root found")
    # Posed is avars composed over the rig-space frame; rest is the frame
    # with the avars out. With nonzero avars the two differ.
    parentAvars = gizmoFrames.AvarsMatrix(parent, t)
    _Check(_MatClose(pf.posed, parentAvars * pf.rest), "parent composition")
    _Check(not _MatClose(pf.posed, pf.rest), "posed differs from rest")
    _Check(_MatClose(pf.rest, gizmoFrames.RestSpace(parent, t)),
           "parent rest frame")
    expectedP = cf.rest * pf.rest.GetInverse() * pf.posed
    _Check(_MatClose(cf.P, expectedP), "child P")
    _Check(_MatClose(cf.posed, gizmoFrames.AvarsMatrix(child, t) * cf.P),
           "child composition")
    _Check(not _MatClose(cf.posed, cf.P), "child posed differs from P")
    _Check(_Close(cf.assetToWorld[3][0], 100.0), "asset placement")


def TestIdentityAvarsLeaveRestAlone():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Asset")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    control = stage.DefinePrim("/Asset/Rig/Still", "RigExecControl")
    space = _Rot(Gf.Vec3d(1, 0, 0), 25.0)
    space.SetTranslateOnly(Gf.Vec3d(1, 2, 3))
    control.GetAttribute("rest:space").Set(space)
    t = Usd.TimeCode.Default()
    frames = gizmoFrames.ComputeRigFrames(stage, control, t)
    _Check(frames.reason == "", "still control is editable")
    _Check(_MatClose(frames.posed, frames.rest),
           "identity avars pose nothing")
    _Check(_MatClose(gizmoFrames.AvarsMatrix(control, t),
                     Gf.Matrix4d(1.0)), "identity avar matrix")


def main():
    _RegisterSchema()
    groups = [
        ("facade aliases", TestFacadeAliases),
        ("posed versus rest", TestPosedVersusRest),
        ("identity avars leave rest alone", TestIdentityAvarsLeaveRestAlone),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_FRAMES_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
