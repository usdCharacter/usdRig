#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoRotation.py.

Usage: test_gizmo_rotation.py

Pure Gf checks, no stage: Euler round trips in every rotation order,
nontrivial gimbal axes, and the world-axis solve. The rotation owner
depends on nothing but math and Gf, so this test registers no schema.
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import gizmoMath  # noqa: E402
import gizmoRotation  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _MatClose(a, b, tol=1e-6):
    return all(_Close(a[r][c], b[r][c], tol)
               for r in range(4) for c in range(4))


def _Rot(axis, deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, deg))
    return m


def TestFacadeAliases():
    for name in ("ROTATION_ORDERS", "RotationFromEuler", "DecomposeEuler",
                 "SolveWorldRotation", "GimbalAxes"):
        _Check(getattr(gizmoMath, name) is getattr(gizmoRotation, name),
               "gizmoMath.%s is the rotation owner's" % name)


def TestEulerRoundTripAllOrders():
    # Nonzero angles on every channel: a decomposition that dropped or
    # swapped one could not reproduce the matrix.
    angles = (20.0, -35.0, 50.0)
    for order in gizmoRotation.ROTATION_ORDERS:
        composed = gizmoRotation.RotationFromEuler(order, *angles)
        back = gizmoRotation.DecomposeEuler(composed, order, hint=angles)
        _Check(all(_Close(b, a, 1e-9) for a, b in zip(angles, back)),
               "%s round trip with hint: %s" % (order, (back,)))
        unhinted = gizmoRotation.DecomposeEuler(composed, order)
        rebuilt = gizmoRotation.RotationFromEuler(order, *unhinted)
        _Check(_MatClose(rebuilt.GetOrthonormalized(False),
                         composed.GetOrthonormalized(False), 1e-9),
               "%s round trip without hint reproduces the rotation" % order)


def TestGimbalAxesAreNontrivial():
    # ZYX with all channels nonzero, carried by a rotated channel frame:
    # every ring axis must be a unit vector, the last-applied ring must
    # stay on its own channel axis, and the first two must be off-axis.
    order = "ZYX"
    angles = (20.0, -35.0, 50.0)
    channel = _Rot(Gf.Vec3d(0, 1, 0), 40.0)
    axes = gizmoRotation.GimbalAxes(order, *angles, channel)
    _Check(len(axes) == 3, "one axis per channel")
    for index, axis in enumerate(axes):
        _Check(_Close(Gf.Vec3d(axis).GetLength(), 1.0, 1e-9),
               "ring %d is unit: %s" % (index, axis))
    world = channel.GetOrthonormalized(False)
    wantLast = world.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
    _Check(all(_Close(axes[0][i], wantLast[i], 1e-9) for i in range(3)),
           "the X ring stays on X in channel space: %s" % (axes[0],))
    offAxis = [i for i in (1, 2)
               if sum(1 for c in axes[i] if abs(c) > 1e-6) > 1]
    _Check(offAxis == [1, 2],
           "the Y and Z rings are off-axis: %s" % (axes,))
    # A different order draws different rings for the same angles.
    other = gizmoRotation.GimbalAxes("XYZ", *angles, channel)
    _Check(any(abs(a[i] - b[i]) > 1e-6
               for a, b in zip(axes, other) for i in range(3)),
           "ring axes depend on the order")


def TestSolveWorldRotation():
    base = (_Rot(Gf.Vec3d(1, 0, 0), 20.0)
            * _Rot(Gf.Vec3d(0, 1, 0), -35.0))
    channel = _Rot(Gf.Vec3d(0, 0, 1), 50.0) * _Rot(Gf.Vec3d(1, 0, 0), 15.0)
    axis = Gf.Vec3d(1, 1, 0).GetNormalized()
    turn = _Rot(axis, 25.0)
    result = gizmoRotation.SolveWorldRotation(base, channel, axis, 25.0)
    current = (base * channel).GetOrthonormalized(False)
    want = current * turn
    got = (result * channel).GetOrthonormalized(False)
    _Check(_MatClose(got, want, 1e-6),
           "the solved frame is the drawn frame turned:\\n%s\\n%s"
           % (got, want))
    _Check(_MatClose(result.GetOrthonormalized(False), result, 1e-9),
           "the answer is a rotation")
    # A zero drag solves to the base it started from.
    same = gizmoRotation.SolveWorldRotation(base, channel, axis, 0.0)
    _Check(_MatClose(same, base, 1e-9), "a zero drag holds still")
    # A sheared channel frame still answers with a rotation: under
    # non-uniform scale no closed form exists, so this is the Newton
    # path, and the contract is only that it stays on SO(3).
    sheared = channel * Gf.Matrix4d(1.0).SetScale(Gf.Vec3d(1, 2, 3))
    rough = gizmoRotation.SolveWorldRotation(base, sheared, axis, 25.0)
    _Check(_MatClose(rough.GetOrthonormalized(False), rough, 1e-6),
           "the sheared answer is a rotation")
    residual = (rough * sheared).GetOrthonormalized(False).GetInverse() \
        * (base * sheared).GetOrthonormalized(False) * turn
    twist = abs(residual.ExtractRotation().GetAngle())
    _Check(twist < 5.0, "the sheared answer turns the right way: %g" % twist)


def main():
    groups = [
        ("facade aliases", TestFacadeAliases),
        ("euler round trip, all orders", TestEulerRoundTripAllOrders),
        ("gimbal axes are nontrivial", TestGimbalAxesAreNontrivial),
        ("world rotation solve", TestSolveWorldRotation),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_ROTATION_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
