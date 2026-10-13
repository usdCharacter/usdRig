"""The rig's IK/FK match reproduces the limb's joints.

Headless test for RigExecIkFkMatchConstraint on the biped and for
plugin/rigExecUsdview/ikfkMatch.py, which authors what the constraint
publishes:

  1. Discovery finds the four limbs from their match constraints: the
     switch, both halves, the pole and the channels the match writes.
  2. On random poses -- stretched, twisted and pinned -- the published
     values and the switch move none of the limb's joints, on all four
     limbs, both ways.
  3. The half that is not driving is presented where those values put it:
     once switched, it drives from exactly where it was drawn.
  4. Nothing in the rig reads the presentation: every joint is where the
     rig without its match constraints puts it.
  5. A round trip FK -> IK -> FK returns the FK channels it started with.
  6. A stretched IK limb lands in FK on its stretch channels, never scale.
  7. A foot's roll channels are kept, not written, when a leg goes to IK.
  8. Rig-written channels are never written.
  9. Plan switches to the half that is not driving.

The pose under test is applied as interactive overrides, never authored.

Usage: test_ikfk_match.py [<generated schema resources dir>]
"""
import os
import random
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

from pxr import Gf, Usd  # noqa: E402

import ikfkMatch  # noqa: E402

_failures = []
# The FK chain lengthens through the rig's float32 stretch chain (a rest
# offset times float(stretch)), so a joint the FK half places is exact to
# that chain's precision: a few 1e-6 at the biped's 25-unit bones. An arm
# switched to IK lands as exactly as the auto clavicle's fixed point, which
# turns its root from the IK estimate rather than the FK direction.
_POSITION = 1e-5
_IK_POSITION = 1e-5


def _Check(condition, message):
    if not condition:
        _failures.append(message)
        print("FAIL: %s" % message)


class _Session(object):
    """A compiled biped and an evaluate(overrides) over a base pose."""

    def __init__(self):
        import _rigexec
        self.stage = Usd.Stage.Open(os.path.join(
            _ROOT, "examples", "biped", "Biped_stack.usda"))
        self.rig = _rigexec.Rig(self.stage, "/Biped/Rig")
        self.rig.compile()
        self.rig.publish_weight_fields = False
        self.base = {}

    def Evaluate(self, overrides, time=None):
        merged = dict(self.base)
        merged.update(overrides)
        self.rig.set_interactive_overrides(
            [(str(p.GetPrimPath()), p.name, float(v))
             for p, v in merged.items()])
        return self.rig.evaluate(TIME.GetValue())

    def Read(self):
        return ikfkMatch.PoseValueReader(self.Evaluate({}))


TIME = Usd.TimeCode(1.0)


def _FrameError(a, b):
    """Worst distance over the frame's origin and handle tips."""
    worst = (a.ExtractTranslation() - b.ExtractTranslation()).GetLength()
    for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)):
        worst = max(worst, (a.Transform(axis) - b.Transform(axis)).GetLength())
    return worst


def _JointsMoved(session, limb, values):
    before = session.Evaluate({})
    after = session.Evaluate(values)
    return max(_FrameError(ikfkMatch.JointFrame(before, j),
                           ikfkMatch.JointFrame(after, j))
               for j in limb.joints)


def _Hinge(limb):
    # Every biped limb bends its middle joint about rz, negative bending it
    # the way it already bends (a two-bone IK bends one way only).
    return limb.fkControls[1].AppendProperty("avars:rz"), -1.0


def _RandomPose(limb, toIk, rng, trial=0):
    """A pose on the half that is driving, the switch on that half, with the
    limb's stretch, twist and pin varied."""
    pose = {limb.switchPath: limb.fkValue if toIk else limb.ikValue}
    sc = limb.switchControl
    pose[sc.AppendProperty("avars:twist")] = rng.uniform(-30.0, 30.0)
    pose[sc.AppendProperty("avars:pvPin")] = [0.0, 0.4, 1.0][trial % 3]
    if toIk:
        for control in (limb.fkControls[0], limb.fkControls[2]):
            for r in ("rx", "ry", "rz"):
                pose[control.AppendProperty("avars:" + r)] = \
                    rng.uniform(-40.0, 40.0)
        hinge, sign = _Hinge(limb)
        pose[hinge] = sign * rng.uniform(5.0, 70.0)
        pose[sc.AppendProperty("avars:stretchTop")] = rng.uniform(0.8, 1.3)
        pose[sc.AppendProperty("avars:stretchBottom")] = rng.uniform(0.8, 1.3)
    else:
        for t in ("tx", "ty", "tz"):
            pose[limb.ikControl.AppendProperty("avars:" + t)] = \
                rng.uniform(-12.0, 12.0)
            pose[limb.pole.AppendProperty("avars:" + t)] = \
                rng.uniform(-15.0, 15.0)
        for r in ("rx", "ry", "rz"):
            pose[limb.ikControl.AppendProperty("avars:" + r)] = \
                rng.uniform(-40.0, 40.0)
    return pose


def TestDiscovery(session):
    limbs = ikfkMatch.FindLimbs(session.stage)
    found = {l.switchControl.name: (l.ikControl.name, l.pole.name,
                                    [c.name for c in l.fkControls])
             for l in limbs}
    expected = {
        "L_Arm": ("L_ArmIK", "L_ArmPV", ["L_UpArm", "L_LoArm", "L_Hand"]),
        "R_Arm": ("R_ArmIK", "R_ArmPV", ["R_UpArm", "R_LoArm", "R_Hand"]),
        "L_Leg": ("L_LegIK", "L_LegPV", ["L_UpLeg", "L_LoLeg", "L_Foot"]),
        "R_Leg": ("R_LegIK", "R_LegPV", ["R_UpLeg", "R_LoLeg", "R_Foot"]),
    }
    _Check(found == expected, "discovery: %r" % found)
    for limb in limbs:
        _Check(limb.upperScale is not None and limb.lowerScale is not None
               and limb.pin is not None and limb.twist is not None,
               "%s: the stretch, pin and twist channels are found"
               % limb.switchControl.name)
    leg = [l for l in limbs if l.switchControl.name == "L_Leg"][0]
    roll = session.stage.GetPrimAtPath(leg.effector).GetParent().GetPath()
    _Check(ikfkMatch.LimbFor(limbs, roll) is leg,
           "a control in the foot's roll stack belongs to its leg")
    return limbs


def TestRandomPoses(session, limbs):
    rng = random.Random(11)
    written = ikfkMatch.RigWritten(session.stage)
    for limb in limbs:
        for trial in range(6):
            for toIk in (True, False):
                session.base = _RandomPose(limb, toIk, rng, trial)
                read = session.Read()
                values, chosen = ikfkMatch.Plan(
                    limb, session.stage, TIME, read, toIk=toIk,
                    written=written, current=session.base)
                _Check(chosen == toIk, "plan honours toIk")
                before = session.Evaluate({})
                follow = [limb.ikControl, limb.pole] if toIk else \
                    list(limb.fkControls)
                shown = {c: ikfkMatch.ControlFrame(before, c) for c in follow}
                after = session.Evaluate(values)
                moved = max(_FrameError(ikfkMatch.JointFrame(before, j),
                                        ikfkMatch.JointFrame(after, j))
                            for j in limb.joints)
                drawn = max(_FrameError(shown[c],
                                        ikfkMatch.ControlFrame(after, c))
                            for c in follow)
                limit = _IK_POSITION if toIk else _POSITION
                label = "%s %s trial %d" % (
                    limb.switchControl.name, "FK->IK" if toIk else "IK->FK",
                    trial)
                _Check(moved < limit, "%s: joints moved %.3g" % (label, moved))
                _Check(drawn < _POSITION,
                       "%s: drove from %.3g away from where it was drawn"
                       % (label, drawn))


def TestPresentationIsReadByNothing(session, limbs):
    """The match revises nothing the rig reads: every joint is where the
    same rig without its match constraints puts it, on poses in either
    half."""
    import _rigexec
    plain = Usd.Stage.Open(os.path.join(_ROOT, "examples", "biped",
                                        "Biped_stack.usda"))
    # Muted on this stage only: the layers are shared with the session's.
    plain.MuteLayer(os.path.join(_ROOT, "examples", "biped",
                                 "Biped_ikfk_match.usda"))
    rig = _rigexec.Rig(plain, "/Biped/Rig")
    rig.compile()
    rig.publish_weight_fields = False
    joints = [p.GetPath() for p in session.stage.Traverse()
              if p.GetTypeName() == "RigExecJoint"]
    rng = random.Random(17)
    for limb in limbs:
        for toIk in (True, False):
            session.base = _RandomPose(limb, toIk, rng)
            a = session.Evaluate({})
            rig.set_interactive_overrides(
                [(str(p.GetPrimPath()), p.name, float(v))
                 for p, v in session.base.items()])
            b = rig.evaluate(TIME.GetValue())
            moved = 0.0
            for path in joints:
                try:
                    moved = max(moved, _FrameError(
                        ikfkMatch.JointFrame(a, path),
                        ikfkMatch.JointFrame(b, path)))
                except KeyError:
                    pass
            _Check(moved == 0.0, "%s %s: a joint differs from the rig "
                   "without its matches by %.3g" % (
                       limb.switchControl.name, "FK" if toIk else "IK",
                       moved))


def TestRoundTrip(session, limbs):
    rng = random.Random(23)
    for limb in limbs:
        start = _RandomPose(limb, True, rng)
        start[limb.switchControl.AppendProperty("avars:twist")] = 0.0
        start[limb.switchControl.AppendProperty("avars:pvPin")] = 0.0
        session.base = dict(start)
        toIk, _ = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                                 toIk=True, current=session.base)
        session.base.update(toIk)
        back, _ = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                                 toIk=False, current=session.base)
        worst = 0.0
        for path, value in start.items():
            if path == limb.switchPath or path not in back:
                continue
            # Angles may come back a full turn away; the rig cannot tell.
            delta = (back[path] - value + 180.0) % 360.0 - 180.0
            worst = max(worst, abs(delta))
        _Check(worst < 1e-4, "%s round trip FK->IK->FK drifts %.3g"
               % (limb.switchControl.name, worst))


def TestStretch(session, limbs):
    limb = [l for l in limbs if l.switchControl.name == "L_Arm"][0]
    # Far beyond reach: the IK stretches the limb.
    session.base = {limb.switchPath: limb.ikValue,
                    limb.ikControl.AppendProperty("avars:tx"): 25.0,
                    limb.ikControl.AppendProperty("avars:ty"): -20.0}
    values, _ = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                               toIk=False, current=session.base)
    moved = _JointsMoved(session, limb, values)
    _Check(moved < _POSITION, "stretched IK -> FK moved joints %.3g" % moved)
    scales = [p for p in values if p.name in ("avars:sx", "avars:sy",
                                              "avars:sz")]
    _Check(not scales, "a match never writes scale: %r" % scales)
    top = values.get(limb.upperScale, 1.0)
    _Check(top > 1.01, "the stretch lands on stretchTop (%.4f)" % top)
    moved = values.get(limb.fkControls[1].AppendProperty("avars:tx"), 0.0)
    _Check(abs(moved) < 1e-4,
           "the FK translation stays put (tx %.3g)" % moved)


def TestFootRollKept(session, limbs):
    limb = [l for l in limbs if l.switchControl.name == "L_Leg"][0]
    rng = random.Random(5)
    session.base = _RandomPose(limb, True, rng)
    roll = limb.switchControl.AppendProperty("avars:footRoll")
    session.base[roll] = 25.0
    values, _ = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                               toIk=True, current=session.base)
    moved = _JointsMoved(session, limb, values)
    _Check(moved < _IK_POSITION,
           "leg with foot roll -> IK moved joints %.3g" % moved)
    touched = [p for p in values if p.GetPrimPath() not in
               (limb.ikControl, limb.pole, limb.switchControl)]
    _Check(not touched and roll not in values,
           "only the IK control, pole and switch control are written: %r"
           % touched)


def TestRigWrittenUntouched(session, limbs):
    written = ikfkMatch.RigWritten(session.stage)
    rng = random.Random(3)
    for limb in limbs:
        for toIk in (True, False):
            session.base = _RandomPose(limb, toIk, rng)
            values, _ = ikfkMatch.Plan(limb, session.stage, TIME,
                                       session.Read(), toIk=toIk,
                                       current=session.base)
            overlap = [p for p in values if p in written]
            _Check(not overlap, "%s writes rig-written %r"
                   % (limb.switchControl.name, overlap))


def TestPlanPicksTheOtherHalf(session, limbs):
    limb = limbs[0]
    session.base = {limb.switchPath: limb.fkValue}
    values, toIk = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                                  current=session.base)
    _Check(toIk and values[limb.switchPath] == limb.ikValue,
           "from FK, Plan switches to IK")
    session.base = {limb.switchPath: limb.ikValue}
    values, toIk = ikfkMatch.Plan(limb, session.stage, TIME, session.Read(),
                                  current=session.base)
    _Check(not toIk and values[limb.switchPath] == limb.fkValue,
           "from IK, Plan switches to FK")


def TestPresentedHalves(session, limbs):
    """The gizmo's view of what each match presents: the IK side (with the
    foot roll under it) on the FK value, the FK side on the IK value,
    nothing between."""
    import gizmoMath
    posed = gizmoMath.SolverPosedPaths(
        session.stage.GetPrimAtPath("/Biped/Rig"))
    _Check(len(posed.matchHalves) == 4,
           "four match halves found (%d)" % len(posed.matchHalves))
    leg = [l for l in limbs if l.switchControl.name == "L_Leg"][0]
    half = [h for h in posed.matchHalves if h.switch == leg.switchPath][0]
    heel = [p for p in half.ikSide if p.name == "L_Heel"]
    _Check(leg.ikControl in half.ikSide and leg.pole in half.ikSide and heel,
           "the IK side holds the IK control, the pole and the foot roll")
    _Check(set(leg.fkControls) <= half.fkSide, "the FK side holds the FK controls")
    switch = session.stage.GetAttributeAtPath(leg.switchPath)
    time = Usd.TimeCode(1.0)
    session.stage.SetEditTarget(session.stage.GetSessionLayer())
    try:
        for value, expected in ((0.0, half.ikSide), (1.0, half.fkSide),
                                (0.5, set())):
            switch.Set(value, time)
            _Check(half.Presented(session.stage, time) == expected,
                   "switch %g presents the expected side" % value)
            presenter = gizmoMath.PresentedBy(session.stage, leg.ikControl,
                                              time, posed)
            _Check((presenter is half) == (expected is half.ikSide),
                   "PresentedBy agrees at switch %g" % value)
    finally:
        switch.Clear()
        session.stage.SetEditTarget(session.stage.GetRootLayer())


def main():
    session = _Session()
    limbs = TestDiscovery(session)
    TestRandomPoses(session, limbs)
    TestPresentationIsReadByNothing(session, limbs)
    TestRoundTrip(session, limbs)
    TestStretch(session, limbs)
    TestFootRollKept(session, limbs)
    TestRigWrittenUntouched(session, limbs)
    TestPlanPicksTheOtherHalf(session, limbs)
    TestPresentedHalves(session, limbs)
    if _failures:
        print("%d failure(s)" % len(_failures))
        return 1
    print("test_ikfk_match: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
