#!/usr/bin/env python
"""
Step tables on RigExecAnimatedMaps, through rigexec.Rig.

A smart-bone action is one RigExecAnimatedMaps on a dial angle whose
RigExecAnimatedMap children share slope-0 rows: one cell per sample value
(outer cells to +-FLT_MAX, so the end frames hold) after a gate row for the
side the action does not serve. Each map's cut is its value at the cell's
frame. This holds the contract on a USDA rig with a dial at samples
[0, 15, 30, 45, 60]:

  * `weight` publishes float32(k) / float32(N), `active` 0 at frame 0 and 1
    after, at every probe angle, ties to the row listed first;
  * an offset map and a value map with rigExec:clampOutput false publish
    values outside [0, 1] as they are (30, 45, -1, 2);
  * an `add` mover reading the offset map at `final` adds it to its
    property, a `blend` mover reading the value map with `active` as its
    envelope switches its property, and both match a float32 reference;
  * inputs:enabled false publishes 0 on every map, so the add adds nothing
    and the blend passes its property through, without a recompile;
  * a mover input connected to a map without a declared read phase reads
    the map's authored 0.

Usage: test_rigexec_step_maps.py [<generated schema resources dir>]
"""
import sys

from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

import numpy  # noqa: E402
from pxr import Plug, Sdf, Usd  # noqa: E402

import rigexec  # noqa: E402

RIG = "/Rig"
DIAL = "/Rig/Controls/Dial"
TARGET = "/Rig/Controls/Target"
OTHER = "/Rig/Controls/Other"
ACTION = "/Rig/Actions/Act"
SAMPLES = [0.0, 15.0, 30.0, 45.0, 60.0]
OFFSET = [0.0, 10.0, 30.0, 30.0, 45.0]      # an offset channel, degrees
VALUE = [-1.0, 0.0, 1.0, 2.0, 2.0]          # a value channel (switch index)
TY_BASE = 5.0
OTHER_BASE = 3.0
FLT_MAX = float(numpy.finfo(numpy.float32).max)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _F32(value):
    return float(numpy.float32(value))


def StepRows(samples, side):
    """[(from, to, frame)]: the gate row, then the nearest-sample cells of
    the unique sample values (earlier ties: the lowest frame of a value,
    cells by frame ascending so a shared bound goes to the lower frame).
    The midpoints here are floats, so the bounds are exact."""
    frame = {}
    for k, value in enumerate(samples):
        frame.setdefault(value, k)
    values = sorted(frame)
    s0 = _F32(samples[0])
    rows = [(-FLT_MAX, s0, 0) if side > 0 else (s0, FLT_MAX, 0)]
    cells = []
    for j, value in enumerate(values):
        lo = -FLT_MAX if j == 0 else _F32((values[j - 1] + value) / 2.0)
        hi = FLT_MAX if j + 1 == len(values) else \
            _F32((value + values[j + 1]) / 2.0)
        cells.append((lo, hi, frame[value]))
    cells.sort(key=lambda cell: cell[2])
    return rows + cells


def StepFrame(rows, x):
    """The frame of the first row whose closed interval holds x, else 0
    (NaN included): one same-input group, slope 0."""
    for lo, hi, frame in rows:
        if lo <= x <= hi:
            return frame
    return 0


def _Cuts(rows, cut):
    return [_F32(cut(frame)) for _lo, _hi, frame in rows]


def _Map(name, rows, cut, clamp=True):
    def Floats(values):
        return "[" + ", ".join(repr(float(v)) for v in values) + "]"
    text = ('            def RigExecAnimatedMap "%s"\n            {\n'
            "                uniform int[] rigExec:rowInputs = [%s]\n"
            "                uniform float[] rigExec:rowFrom = %s\n"
            "                uniform float[] rigExec:rowTo = %s\n"
            "                uniform float[] rigExec:rowSlope = %s\n"
            "                uniform float[] rigExec:rowCut = %s\n"
            % (name, ", ".join("0" for _r in rows),
               Floats(r[0] for r in rows), Floats(r[1] for r in rows),
               Floats(0.0 for _r in rows), Floats(_Cuts(rows, cut))))
    if not clamp:
        text += "                uniform bool rigExec:clampOutput = false\n"
    return text + "            }\n"


def _Mover(name, target, operation, value, weight=None, final=True):
    phase = ' (\n                rigExecReadPhase = "final"\n            )' \
        if final else ""
    text = ('        def RigExecFloatMathMover "%s" (\n'
            '            prepend apiSchemas = ["RigExecMoverAPI"]\n'
            "        )\n        {\n"
            "            rel rigExec:moves = <%s>\n"
            '            uniform token rigExec:operation = "%s"\n'
            "            float inputs:value%s\n"
            "            float inputs:value.connect = <%s>\n"
            % (name, target, operation, phase, value))
    if weight:
        text += ("            float inputs:defaultWeight%s\n"
                 "            float inputs:defaultWeight.connect = <%s>\n"
                 % (phase, weight))
    return text + "        }\n"


def _Control(name, body):
    return ('        def RigExecControl "%s" (\n'
            '            prepend apiSchemas = ["RigExecControlAPI"]\n'
            "        )\n        {\n%s        }\n" % (name, body))


def _Stage(rows):
    n = len(SAMPLES) - 1
    last = len(OFFSET) - 1
    maps = (
        _Map("weight", rows,
             lambda k: numpy.float32(k) / numpy.float32(n)) +
        _Map("active", rows, lambda k: 1.0 if k > 0 else 0.0) +
        _Map("Off", rows, lambda k: OFFSET[min(k, last)] - OFFSET[0],
             clamp=False) +
        _Map("Val", rows, lambda k: VALUE[min(k, last)], clamp=False))
    text = (
        "#usda 1.0\n"
        'def RigExecRoot "Rig"\n{\n'
        '    def Scope "Controls"\n    {\n' +
        _Control("Dial", "            double avars:rz = 0\n") +
        _Control("Target", "            double avars:tx = 0\n"
                           "            double avars:ty = %r\n" % TY_BASE) +
        _Control("Other", "            double avars:tx = %r\n" % OTHER_BASE) +
        "    }\n"
        '    def Scope "Actions"\n    {\n'
        '        def RigExecAnimatedMaps "Act"\n        {\n'
        "            rel rigExec:driverAttributes = <%s.avars:rz> (\n"
        '                rigExecReadPhase = "final"\n            )\n' % DIAL +
        maps +
        "        }\n    }\n"
        '    def Scope "Movers"\n    {\n' +
        _Mover("OffTx", TARGET + ".avars:tx", "add",
               ACTION + "/Off.outputs:value") +
        _Mover("ValTy", TARGET + ".avars:ty", "blend",
               ACTION + "/Val.outputs:value",
               weight=ACTION + "/active.outputs:value") +
        # Undeclared: reads the map's authored outputs:value, 0.
        _Mover("Undeclared", OTHER + ".avars:tx", "add",
               ACTION + "/Off.outputs:value", final=False) +
        "    }\n}\n")
    stage = Usd.Stage.CreateInMemory("step_maps.usda")
    _Check(stage.GetRootLayer().ImportFromString(text),
           "the step-table rig parses")
    return stage


def _Evaluate(rig, angle):
    rig.set_interactive_overrides([(DIAL, "avars:rz", float(angle))])
    pose = rig.evaluate(1.0)
    _Check(pose.valid, "pose at %r is valid: %s" % (angle, pose.diagnostics))
    return pose


def _Read(pose, path):
    return pose.moved_property(path)


ANGLES = [-10.0, 0.0, 5.0, 7.5, 7.6, 15.0, 22.5, 22.6, 30.0, 37.5, 45.0,
          52.5, 52.6, 60.0, 100.0, 1e30]
# The old rule's frames at those angles (earlier ties; s_0 and below: 0;
# past 60 the last frame holds).
FRAMES = [0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4]


def TestStepTable(stage, rig, rows):
    n = len(SAMPLES) - 1
    last = len(OFFSET) - 1
    for angle, frame in zip(ANGLES, FRAMES):
        _Check(StepFrame(rows, angle) == frame,
               "reference rows give frame %d at %r, not %d"
               % (frame, angle, StepFrame(rows, angle)))
        pose = _Evaluate(rig, angle)
        k = frame
        weight = _F32(numpy.float32(k) / numpy.float32(n))
        active = 1.0 if k > 0 else 0.0
        offset = _F32(OFFSET[min(k, last)] - OFFSET[0])
        value = _F32(VALUE[min(k, last)])
        got = {
            "weight": _Read(pose, ACTION + "/weight.outputs:value"),
            "active": _Read(pose, ACTION + "/active.outputs:value"),
            "Off": _Read(pose, ACTION + "/Off.outputs:value"),
            "Val": _Read(pose, ACTION + "/Val.outputs:value"),
        }
        want = {"weight": weight, "active": active, "Off": offset,
                "Val": value}
        for name in want:
            _Check(got[name] == want[name],
                   "%s at %r (frame %d): %r, expected %r"
                   % (name, angle, k, got[name], want[name]))
        # The add mover adds the unclamped offset; the blend switches to the
        # unclamped value while active and passes the base through at 0.
        tx = _Read(pose, TARGET + ".avars:tx")
        ty = _Read(pose, TARGET + ".avars:ty")
        _Check(tx == offset, "tx at %r: %r, expected %r" % (angle, tx, offset))
        expected_ty = value if active else TY_BASE
        _Check(ty == expected_ty,
               "ty at %r: %r, expected %r" % (angle, ty, expected_ty))
        # An undeclared read is the map's authored 0, whatever the dial.
        other = _Read(pose, OTHER + ".avars:tx")
        _Check(other == OTHER_BASE,
               "an undeclared read at %r added %r" % (angle, other - OTHER_BASE))
    # Values outside [0, 1] reached the movers unclamped.
    _Check(max(_F32(OFFSET[k] - OFFSET[0]) for k in range(n + 1)) == 45.0 and
           min(VALUE) == -1.0, "the probe reaches beyond [0, 1]")


def TestDisable(stage, rig):
    signature = rig.compiled_rig_signature()
    stage.GetPrimAtPath(ACTION).CreateAttribute(
        "inputs:enabled", Sdf.ValueTypeNames.Bool).Set(False)
    pose = _Evaluate(rig, 45.0)
    _Check(rig.compiled_rig_signature() == signature,
           "inputs:enabled is a value edit, not a recompile")
    for name in ("weight", "active", "Off", "Val"):
        got = _Read(pose, ACTION + "/%s.outputs:value" % name)
        _Check(got == 0.0, "disabled %s published %r" % (name, got))
    _Check(_Read(pose, TARGET + ".avars:tx") == 0.0,
           "a disabled table's offset adds nothing")
    _Check(_Read(pose, TARGET + ".avars:ty") == TY_BASE,
           "a disabled table's blend passes the base through")
    stage.GetPrimAtPath(ACTION).GetAttribute("inputs:enabled").Set(True)
    pose = _Evaluate(rig, 45.0)
    _Check(_Read(pose, ACTION + "/Val.outputs:value") == 2.0,
           "re-enabled, the value map publishes frame 3's 2")


def main():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    rows = StepRows(SAMPLES, +1)
    stage = _Stage(rows)
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    skipped = rig.skipped_operations()
    _Check(not skipped, "nothing is set aside: %s" % dict(skipped))
    TestStepTable(stage, rig, rows)
    print("  ok: weight, active and unclamped offset/value maps step with "
          "the dial; add and blend movers read them at final")
    TestDisable(stage, rig)
    print("  ok: a disabled table publishes 0 on every map without a "
          "recompile")
    print("RIGEXEC_STEP_MAPS_OK (2 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
