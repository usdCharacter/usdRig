#!/usr/bin/env python
"""
Interactive overrides of a path curve's point3f[] points.

Dragging a control point of the curve a path mover samples previews the
whole points array with nothing authored: it is a (curve, "points",
point3f[]) entry in the interactive override set. The binding must deliver
it as a VtVec3fArray, and the evaluator must sample it so every value the
path drives is bit-identical to the same points authored.

The rig: three path movers place a control's tx, tz and ry on a pinned
catmullRom curve at the distance each avar holds, with a local offset and a
heading offset so every component of the sample reaches the answer.

Usage: test_rigexec_path_point_overrides.py [<generated schema resources dir>]
Requires the native _rigexec binding (build/python on PYTHONPATH).
"""
import sys

# The suite's shared bootstrap: sys.path (including the build tree's python
# directory, found from the resources argument), DLL directories and
# PXR_PLUGINPATH_NAME.
from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Gf, Plug, Usd, UsdGeom, Vt  # noqa: E402

import rigexec  # noqa: E402

_RIG = "/Rig"
_ROOT = _RIG + "/Controls/Root"
_PATH = _RIG + "/Path"
_AVARS = ("avars:tx", "avars:tz", "avars:ry")
_TIME = 1.0

AUTHORED = [(0, 0, 0), (0.5, 0, 1), (0, 0, 2), (-0.5, 0, 3), (0.5, 0, 4)]
# The span the movers sample, bent two ways, and a curve of another length.
MOVED = [(0, 0, 0), (1.25, 0, 0.75), (-0.3, 0, 2.2), (-0.5, 0, 3),
         (0.5, 0, 4)]
MOVED2 = [(0, 0, 0), (-0.75, 0.5, 1.5), (0.4, 0, 2.5), (-0.5, 0, 3),
          (0.5, 0, 4)]
SHORTER = [(0, 0, 0), (1.0, 0, 1.0), (1.5, 0, 2.5), (1.0, 0, 4.0)]
DISTANCE = 1.5
DISTANCE_DRAG = 2.75


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Points(points):
    return Vt.Vec3fArray([Gf.Vec3f(*p) for p in points])


def _Stage(points=AUTHORED, distance=DISTANCE):
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim(_RIG, "RigExecRoot")
    stage.DefinePrim(_RIG + "/Joints", "Scope")
    joint = stage.DefinePrim(_RIG + "/Joints/J", "RigExecJoint")
    joint.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    stage.DefinePrim(_RIG + "/Controls", "Scope")
    root = stage.DefinePrim(_ROOT, "RigExecControl")
    root.GetAttribute("rest:space").Set(Gf.Matrix4d(1.0))
    curve = UsdGeom.BasisCurves.Define(stage, _PATH)
    curve.CreateTypeAttr().Set(UsdGeom.Tokens.cubic)
    curve.CreateBasisAttr().Set(UsdGeom.Tokens.catmullRom)
    curve.CreateWrapAttr().Set(UsdGeom.Tokens.pinned)
    curve.CreateCurveVertexCountsAttr().Set(Vt.IntArray([len(points)]))
    curve.CreatePointsAttr().Set(_Points(points))
    stage.DefinePrim(_RIG + "/Movers", "Scope")
    for avar, component in zip(_AVARS, ("x", "z", "heading")):
        root.GetAttribute(avar).Set(distance)
        mover = stage.DefinePrim(_RIG + "/Movers/" + component,
                                 "RigExecFloatMathMover")
        mover.AddAppliedSchema("RigExecMoverAPI")
        mover.GetRelationship("rigExec:moves").SetTargets(
            [root.GetAttribute(avar).GetPath()])
        mover.GetAttribute("rigExec:operation").Set("path")
        mover.GetRelationship("rigExec:path").SetTargets([_PATH])
        mover.GetAttribute("rigExec:pathComponent").Set(component)
        mover.GetAttribute("rigExec:pathOffset").Set(Gf.Vec2f(0.1, 0.2))
        mover.GetAttribute("rigExec:pathHeadingOffset").Set(30.0)
        mover.GetAttribute("inputs:value").Set(0.25)
    return stage


def _Open(points=AUTHORED, distance=DISTANCE):
    stage = _Stage(points, distance)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    # Held: the Rig does not own the stage.
    return stage, rig


def _Answer(pose):
    """The driven numbers, compared exactly: the avars and Root's frame."""
    assert pose.valid, "pose invalid: %s" % (list(pose.diagnostics),)
    frame = pose.control_frame(_ROOT)
    return (tuple(pose.moved_property(_ROOT + "." + avar)
                  for avar in _AVARS),
            tuple(frame.origin), tuple(frame.x_axis), tuple(frame.y_axis),
            tuple(frame.z_axis))


def _Authored(points=AUTHORED, distance=DISTANCE):
    """What a fresh rig on a stage that AUTHORS the points evaluates to."""
    _, rig = _Open(points, distance)
    return _Answer(rig.evaluate(_TIME))


def _AssertSame(label, got, want):
    assert got == want, "%s:\n  got  %r\n  want %r" % (label, got, want)


def _AssertDiffer(label, a, b):
    assert all(x != y for x, y in zip(a[0], b[0])) and a[1:] != b[1:], (
        "%s: the two answers coincide, so the comparison proves nothing: %r"
        % (label, a))


def _PointsEntry(points):
    return (_PATH, "points", points)


def TestPointsOverrideIsAuthoring():
    """Overridden points evaluate bit-identically to the points authored.

    Both spellings the editor can hand over -- a list of tuples and a
    Vt.Vec3fArray -- and both a rig that evaluated before the drag and one
    whose first evaluate already carries the override.
    """
    authored = _Authored()
    want = _Authored(MOVED)
    _AssertDiffer("MOVED against the authored points", want, authored)

    _, rig = _Open()
    _AssertSame("before the override", _Answer(rig.evaluate(_TIME)),
                authored)
    rig.set_interactive_overrides([_PointsEntry(MOVED)])
    assert rig.has_interactive_overrides
    _AssertSame("points override (list of tuples)",
                _Answer(rig.evaluate(_TIME)), want)

    rig.clear_interactive_overrides()
    rig.set_interactive_overrides([_PointsEntry(_Points(MOVED))])
    _AssertSame("points override (Vt.Vec3fArray)",
                _Answer(rig.evaluate(_TIME)), want)

    _, cold = _Open()
    cold.set_interactive_overrides([_PointsEntry(MOVED)])
    _AssertSame("points override before the first evaluate",
                _Answer(cold.evaluate(_TIME)), want)


def TestEachDragSampleIsPlaced():
    """Each push is the curve that evaluates, whatever its point count, and
    clearing it is the authored curve again, exactly."""
    authored = _Authored()
    want = _Authored(MOVED)
    want2 = _Authored(MOVED2)
    shorter = _Authored(SHORTER)
    _AssertDiffer("MOVED2 against MOVED", want2, want)
    _AssertDiffer("SHORTER against MOVED", shorter, want)

    _, rig = _Open()
    rig.evaluate(_TIME)
    for label, points, expected in (("MOVED", MOVED, want),
                                    ("MOVED2", MOVED2, want2),
                                    ("SHORTER", SHORTER, shorter),
                                    ("MOVED again", MOVED, want)):
        rig.set_interactive_overrides([_PointsEntry(points)])
        _AssertSame("override %s" % label, _Answer(rig.evaluate(_TIME)),
                    expected)
    rig.clear_interactive_overrides()
    assert not rig.has_interactive_overrides
    _AssertSame("after clear", _Answer(rig.evaluate(_TIME)), authored)


def TestAvarAndPointsTogether():
    """A dragged distance and previewed points in one set both take effect."""
    want = _Authored(MOVED, DISTANCE_DRAG)
    _AssertDiffer("both against the points alone", want, _Authored(MOVED))
    _AssertDiffer("both against the distance alone", want,
                  _Authored(AUTHORED, DISTANCE_DRAG))

    _, rig = _Open()
    rig.evaluate(_TIME)
    distances = [(_ROOT, avar, DISTANCE_DRAG) for avar in _AVARS]
    rig.set_interactive_overrides(distances + [_PointsEntry(MOVED)])
    _AssertSame("distance and points overrides",
                _Answer(rig.evaluate(_TIME)), want)
    rig.set_interactive_overrides([_PointsEntry(MOVED)] + distances)
    _AssertSame("points and distance overrides",
                _Answer(rig.evaluate(_TIME)), want)


def TestValueTypes():
    """Points are (x, y, z) triples, or refused with TypeError.

    A refused call replaces nothing: the set pushed before it still stands.
    """
    want = _Authored(MOVED)
    _, rig = _Open()
    rig.set_interactive_overrides([_PointsEntry(MOVED)])

    refused = (
        ("a number for point3f[] points", _PointsEntry(1.0)),
        ("a string for point3f[] points", _PointsEntry("xyz")),
        ("a flat sequence of numbers", _PointsEntry([0.0, 1.0, 2.0])),
        ("a two-element point", _PointsEntry([(0.0, 1.0)])),
        ("a four-element point", _PointsEntry([(0.0, 1.0, 2.0, 3.0)])),
        ("a non-numeric point component", _PointsEntry([(0.0, "1", 2.0)])),
        ("a string point", _PointsEntry(["abc"])),
        ("float2 pairs for point3f[] points",
         _PointsEntry(Vt.Vec2fArray([(0, 1), (2, 3)]))),
        ("a points sequence for a double avar", (_ROOT, "avars:tx", MOVED)),
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

    # An empty array is an array, not a malformed value.
    rig.set_interactive_overrides([_PointsEntry([])])
    assert rig.has_interactive_overrides
    rig.set_interactive_overrides([_PointsEntry(Vt.Vec3fArray())])
    assert rig.has_interactive_overrides
    rig.clear_interactive_overrides()

    # Integer components and Gf.Vec3d elements are numbers like any other.
    rig.set_interactive_overrides(
        [_PointsEntry([(x, int(y), int(z)) for x, y, z in AUTHORED])])
    _AssertSame("integer components equal to the authored points",
                _Answer(rig.evaluate(_TIME)), _Authored())
    rig.set_interactive_overrides(
        [_PointsEntry([Gf.Vec3d(*p) for p in MOVED])])
    _AssertSame("Gf.Vec3d points", _Answer(rig.evaluate(_TIME)), want)


def main():
    _RegisterSchema()
    groups = [
        ("a points override is the authored curve",
         TestPointsOverrideIsAuthoring),
        ("each drag sample is placed", TestEachDragSampleIsPlaced),
        ("a distance and a points override together",
         TestAvarAndPointsTogether),
        ("points are (x, y, z) triples", TestValueTypes),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_PATH_POINT_OVERRIDES_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
