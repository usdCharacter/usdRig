#!/usr/bin/env python
"""
Interactive overrides of a curve mover's float2[] inputs:keys table.

A manipulation can preview a whole keys table beside a dragged avar, with
nothing authored: the table is a (prim, "inputs:keys", float2[]) entry in the
same interactive override set the avar travels in.
The binding must deliver it as a VtVec2fArray, and the evaluator must place
it so the frame it drives is exactly the frame of the same table authored.

The rig: a constant avar (lid.avars:ty) is added into pivot.avars:rx, and a
float curve mover then maps rx through inputs:keys, so the keys turn the
pivot control's frame.

Usage: test_rigexec_curve_key_overrides.py [<generated schema resources dir>]
Requires the native _rigexec binding (build/python on PYTHONPATH).
"""
import sys

# The suite's shared bootstrap: sys.path (including the build tree's python
# directory, found from the resources argument), DLL directories and
# PXR_PLUGINPATH_NAME.
from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Gf, Plug, Usd, Vt  # noqa: E402

import rigexec  # noqa: E402

_RIG = "/Asset/Rig"
_LID = _RIG + "/Controls/lid"
_PIVOT = _RIG + "/Controls/pivot"
_CURVE = _RIG + "/Movers/lid_to_rx"
_RX = _PIVOT + ".avars:rx"
_TIME = 1.0

# (input, output) keys, strictly increasing in input. Each table bends the
# curve differently around the evaluated inputs, so no two share an answer.
AUTHORED = [(-10.0, 40.0), (0.0, 0.0), (10.0, -40.0)]
K = [(-10.0, 25.5), (0.0, -3.25), (10.0, 61.75)]
K2 = [(-12.0, -7.5), (-1.0, 12.125), (6.0, 33.0), (14.0, -9.0)]
TY = 3.7
TY_DRAG = -6.3


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Stage(keys=AUTHORED, ty=TY):
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim(_RIG, "RigExecRoot")
    stage.DefinePrim(_RIG + "/Joints", "Scope")
    joint = stage.DefinePrim(_RIG + "/Joints/Root", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    stage.DefinePrim(_RIG + "/Controls", "Scope")
    lid = stage.DefinePrim(_LID, "RigExecControl")
    lid.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    lid.GetAttribute("avars:ty").Set(ty)
    pivot = stage.DefinePrim(_PIVOT, "RigExecControl")
    pivot.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    rx = pivot.GetAttribute("avars:rx").GetPath()
    curve = stage.DefinePrim(_CURVE, "RigExecFloatMathMover")
    curve.AddAppliedSchema("RigExecMoverAPI")
    curve.GetRelationship("rigExec:moves").SetTargets([rx])
    curve.GetAttribute("rigExec:operation").Set("curve")
    curve.GetAttribute("inputs:keys").Set(
        Vt.Vec2fArray([Gf.Vec2f(*k) for k in keys]))
    add = stage.DefinePrim(_CURVE + "_0_add", "RigExecFloatMathMover")
    add.AddAppliedSchema("RigExecMoverAPI")
    add.GetRelationship("rigExec:moves").SetTargets([rx])
    add.GetAttribute("rigExec:operation").Set("add")
    add.GetAttribute("inputs:value").SetConnections(
        [lid.GetAttribute("avars:ty").GetPath()])
    # Movers run bottom to top: the add first, then the curve.
    stage.GetPrimAtPath(_RIG + "/Movers").SetChildrenReorder(
        ["lid_to_rx", "lid_to_rx_0_add"])
    return stage


def _Open(keys=AUTHORED, ty=TY):
    stage = _Stage(keys, ty)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    # Held: the Rig does not own the stage.
    return stage, rig


def _Answer(pose):
    """The driven numbers, compared exactly: rx and the pivot's frame."""
    assert pose.valid, "pose invalid: %s" % (list(pose.diagnostics),)
    frame = pose.control_frame(_PIVOT)
    return (pose.moved_property(_RX), tuple(frame.origin),
            tuple(frame.x_axis), tuple(frame.y_axis), tuple(frame.z_axis))


def _Authored(keys=AUTHORED, ty=TY):
    """What a fresh rig on a stage that AUTHORS keys and ty evaluates to."""
    _, rig = _Open(keys, ty)
    return _Answer(rig.evaluate(_TIME))


def _AssertSame(label, got, want):
    assert got == want, "%s:\n  got  %r\n  want %r" % (label, got, want)


def _AssertDiffer(label, a, b):
    assert a[0] != b[0] and a[1:] != b[1:], (
        "%s: the two answers coincide, so the comparison proves nothing: %r"
        % (label, a))


def _Keys(keys):
    return (_CURVE, "inputs:keys", keys)


def TestKeysOverrideIsAuthoring():
    """An overridden table evaluates bit-identically to the table authored.

    Both spellings the editor can hand over -- a list of tuples and a
    Vt.Vec2fArray -- and both a rig that evaluated before the drag and one
    whose first evaluate already carries the override.
    """
    authored = _Authored()
    want = _Authored(K)
    _AssertDiffer("K against the authored keys", want, authored)

    _, rig = _Open()
    _AssertSame("before the override", _Answer(rig.evaluate(_TIME)),
                authored)
    rig.set_interactive_overrides([_Keys(K)])
    assert rig.has_interactive_overrides
    _AssertSame("keys override (list of tuples)",
                _Answer(rig.evaluate(_TIME)), want)

    rig.clear_interactive_overrides()
    rig.set_interactive_overrides(
        [_Keys(Vt.Vec2fArray([Gf.Vec2f(*k) for k in K]))])
    _AssertSame("keys override (Vt.Vec2fArray)",
                _Answer(rig.evaluate(_TIME)), want)

    _, cold = _Open()
    cold.set_interactive_overrides([_Keys(K)])
    _AssertSame("keys override before the first evaluate",
                _Answer(cold.evaluate(_TIME)), want)


def TestNextTableIsPlacedAgain():
    """Each push is the table that evaluates; nothing of the last one stays."""
    want = _Authored(K)
    want2 = _Authored(K2)
    _AssertDiffer("K2 against K", want2, want)

    _, rig = _Open()
    rig.evaluate(_TIME)
    for label, keys, expected in (("K", K, want), ("K2", K2, want2),
                                  ("K again", K, want)):
        rig.set_interactive_overrides([_Keys(keys)])
        _AssertSame("override %s" % label, _Answer(rig.evaluate(_TIME)),
                    expected)


def TestClearRestoresTheAuthoredRig():
    """Releasing the drag is the authored table again, exactly."""
    authored = _Authored()
    _, rig = _Open()
    rig.evaluate(_TIME)
    for keys in (K, K2):
        rig.set_interactive_overrides([_Keys(keys)])
        rig.evaluate(_TIME)
    rig.clear_interactive_overrides()
    assert not rig.has_interactive_overrides
    _AssertSame("after clear", _Answer(rig.evaluate(_TIME)), authored)


def TestAvarAndKeysTogether():
    """A dragged avar and a previewed table in one set both take effect."""
    want = _Authored(K, TY_DRAG)
    _AssertDiffer("both against keys alone", want, _Authored(K, TY))
    _AssertDiffer("both against the avar alone", want,
                  _Authored(AUTHORED, TY_DRAG))

    _, rig = _Open()
    rig.evaluate(_TIME)
    rig.set_interactive_overrides([(_LID, "avars:ty", TY_DRAG), _Keys(K)])
    _AssertSame("avar and keys overrides", _Answer(rig.evaluate(_TIME)),
                want)
    # The order within the set does not matter.
    rig.set_interactive_overrides([_Keys(K), (_LID, "avars:ty", TY_DRAG)])
    _AssertSame("keys and avar overrides", _Answer(rig.evaluate(_TIME)),
                want)


def TestValueTypes():
    """Values are typed like the attribute, or refused with TypeError.

    A refused call replaces nothing: the set pushed before it still stands.
    """
    want = _Authored(K)
    _, rig = _Open()
    rig.set_interactive_overrides([_Keys(K)])

    refused = (
        ("a sequence for a double avar", (_LID, "avars:ty", [(1.0, 2.0)])),
        ("a flat sequence for a double avar", (_LID, "avars:ty", [1.0])),
        ("a sequence for a float input", (_CURVE, "inputs:value", [0.5])),
        ("a string for a double avar", (_LID, "avars:ty", "3")),
        ("a number for float2[] keys", _Keys(1.0)),
        ("a string for float2[] keys", _Keys("ab")),
        ("a three-element key", _Keys([(0.0, 1.0, 2.0)])),
        ("a one-element key", _Keys([(0.0,)])),
        ("a non-numeric key component", _Keys([(0.0, "1")])),
        ("a scalar key", _Keys([0.0, 1.0])),
    )
    for label, entry in refused:
        try:
            rig.set_interactive_overrides([entry])
        except TypeError:
            pass
        else:
            raise AssertionError("%s was accepted" % label)
    _AssertSame("the set standing after the refusals",
                _Answer(rig.evaluate(_TIME)), want)

    # An empty table is a table, not a malformed value.
    rig.set_interactive_overrides([_Keys([])])
    assert rig.has_interactive_overrides
    rig.set_interactive_overrides([_Keys(Vt.Vec2fArray())])
    assert rig.has_interactive_overrides
    rig.clear_interactive_overrides()

    # Integer components are numbers like any other.
    rig.set_interactive_overrides([_Keys([(-10, 40), (0, 0), (10, -40)])])
    _AssertSame("integer keys equal to the authored table",
                _Answer(rig.evaluate(_TIME)), _Authored())


def main():
    _RegisterSchema()
    groups = [
        ("a keys override is the authored table",
         TestKeysOverrideIsAuthoring),
        ("the next table is placed again", TestNextTableIsPlacedAgain),
        ("clear restores the authored rig", TestClearRestoresTheAuthoredRig),
        ("an avar and a keys override together", TestAvarAndKeysTogether),
        ("values are typed like the attribute", TestValueTypes),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_CURVE_KEY_OVERRIDES_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
