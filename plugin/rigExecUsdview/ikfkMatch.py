"""
IK/FK switching: hand a limb's joints to its other half without moving them.

Qt-free. The rig does the matching. A RigExecIkFkMatchConstraint on each
limb keeps the half the switch is not on presented on the joints, and
publishes every frame, as outputs:ikValues and outputs:fkValues, the
channel values that half needs to drive the joints unchanged. A switch
reads those values and authors them with the switch value, as one edit;
nothing is solved here and no rig is evaluated for it.

A limb is found from its match constraint, so nothing here knows the
biped's names. The values are read from a published pose
(PoseValueReader), or in a host from the generation it last drew
(SetPublishedValuesReader; rigExecUsdview installs one per session).

Layouts, as the schema states them:

    outputs:ikValues = [ikControl tx ty tz rx ry rz, poleControl tx ty tz,
                        upperScale, lowerScale]
    outputs:fkValues = [fkControls[0..2] tx ty tz rx ry rz,
                        upperScale, lowerScale]

A switch to FK also sets the pin and the twist to zero: both are the IK
solve's own, and an FK pose has neither.

Conventions: USD row vectors (a point maps as p * M, a child's world frame
is local * parent); frames are asset space, as rigexec.Pose reports them.
"""

import math

from pxr import Gf, Sdf, Usd

import gizmoMath
import sessionRegistry

MATCH = "RigExecIkFkMatchConstraint"
CONTROL = "RigExecControl"
RIG_ROOT = "RigExecRoot"

# The channels a match may write: translate and rotate. Scale is never
# written -- an FK scale shears everything below it, so a stretched limb is
# matched with its stretch channels, the way the rig lengthens FK.
_TRANSLATE = ("tx", "ty", "tz")
_ROTATE = ("rx", "ry", "rz")
_IDENTITY = {"tx": 0.0, "ty": 0.0, "tz": 0.0, "rx": 0.0, "ry": 0.0,
             "rz": 0.0, "rspin": 0.0, "sx": 1.0, "sy": 1.0, "sz": 1.0}
IK_VALUES = "outputs:ikValues"
FK_VALUES = "outputs:fkValues"


class Limb(object):
    """One switchable limb, found from its match constraint."""

    def __init__(self, match, switchControl, switchAttr, ikValue, joints,
                 fkControls, ikControl, effector, pole, rigRoot,
                 upperScale=None, lowerScale=None, pin=None, twist=None):
        self.match = match                  # Sdf.Path of the constraint
        self.switchControl = switchControl  # Sdf.Path
        self.switchAttr = switchAttr        # "avars:ikfk"
        self.ikValue = ikValue              # switch value that selects IK
        self.joints = joints                # [Sdf.Path] x 3, root to end
        self.fkControls = fkControls        # [Sdf.Path] x 3
        self.ikControl = ikControl          # Sdf.Path, the animator's IK
        self.effector = effector            # Sdf.Path, the solver target
        self.pole = pole                    # Sdf.Path
        self.rigRoot = rigRoot              # Sdf.Path
        # The channels the match's stretch, pin and twist inputs read
        # (attribute paths), or None where it reads none.
        self.upperScale = upperScale
        self.lowerScale = lowerScale
        self.pin = pin
        self.twist = twist
        # The channel paths a switch may write, settled at discovery: the rig
        # writes or drives none of them (Writable).
        self.writable = None

    @property
    def fkValue(self):
        return 1.0 - self.ikValue

    @property
    def switchPath(self):
        return self.switchControl.AppendProperty(self.switchAttr)

    def Controls(self):
        """Every control this limb's switch concerns."""
        return [self.switchControl] + list(self.fkControls) + \
            [self.ikControl, self.pole]

    def IsIk(self, stage, time, current=None):
        """Whether the IK half drives the joints now (the switch is nearer
        the IK value)."""
        if current and self.switchPath in current:
            value = float(current[self.switchPath])
        else:
            value = _Get(stage, self.switchPath, time, self.fkValue)
        return abs(value - self.ikValue) < abs(value - self.fkValue)

    def __repr__(self):
        return "<Limb %s>" % self.switchControl.name


# -- discovery ------------------------------------------------------------

def _Targets(prim, name):
    rel = prim.GetRelationship(name)
    return list(rel.GetTargets()) if rel else []


def _One(prim, name):
    targets = _Targets(prim, name)
    return targets[0] if len(targets) == 1 else None


def _Source(prim, name):
    """The attribute `name` reads through its one connection, or None."""
    attr = prim.GetAttribute(name)
    sources = attr.GetConnections() if attr else []
    return sources[0] if len(sources) == 1 else None


def _RigRoot(prim):
    walk = prim
    while walk and walk.GetPath() != Sdf.Path.absoluteRootPath:
        if walk.GetTypeName() == RIG_ROOT:
            return walk.GetPath()
        walk = walk.GetParent()
    return None


def FindLimbs(stage):
    """Every switchable limb on the stage: one per match constraint, with
    the channels a switch may write settled once."""
    limbs = []
    written = None
    for prim in stage.Traverse():
        if prim.GetTypeName() != MATCH:
            continue
        switch = _Source(prim, "inputs:switch")
        ik, pole = _One(prim, "rigExec:ikControl"), _One(prim,
                                                       "rigExec:poleControl")
        effector = _One(prim, "rigExec:effector")
        fk = _Targets(prim, "rigExec:fkControls")
        joints = _Targets(prim, "rigExec:limbJoints")
        if switch is None or ik is None or pole is None or effector is None \
                or len(fk) != 3 or len(joints) != 3:
            continue
        ikValue = prim.GetAttribute("rigExec:ikValue")
        ikValue = float(ikValue.Get()) if ikValue and \
            ikValue.Get() is not None else 1.0
        limbs.append(Limb(
            prim.GetPath(), switch.GetPrimPath(), switch.name, ikValue,
            joints, fk, ik, effector, pole, _RigRoot(prim),
            _Source(prim, "inputs:upperScale"),
            _Source(prim, "inputs:lowerScale"),
            _Source(prim, "inputs:pin"), _Source(prim, "inputs:twist")))
        if written is None:
            written = RigWritten(stage)
        limbs[-1].writable = Writable(stage, limbs[-1], written)
    return limbs


def Writable(stage, limb, written):
    """The set of channel paths a switch of `limb` may write: those that
    exist, are not connected, and are written by no mover."""
    out = set()
    for control, names in ([(limb.ikControl, _TRANSLATE + _ROTATE),
                            (limb.pole, _TRANSLATE)] +
                           [(c, _TRANSLATE + _ROTATE)
                            for c in limb.fkControls]):
        for name in _Writable(stage, control, names, written):
            out.add(control.AppendProperty("avars:" + name))
    for path in (limb.upperScale, limb.lowerScale, limb.pin, limb.twist):
        attr = stage.GetAttributeAtPath(path) if path is not None else None
        if attr and path not in written and not attr.HasAuthoredConnections():
            out.add(path)
    return out


def LimbFor(limbs, path):
    """The limb a selected prim belongs to: its switch control, an FK or IK
    control, the pole, or anything under the IK control (a foot's roll
    controls)."""
    path = Sdf.Path(str(path))
    for limb in limbs:
        if path in limb.Controls() or path.HasPrefix(limb.ikControl):
            return limb
    return None


# -- presentation ---------------------------------------------------------

class MatchHalf(object):
    """
    What one RigExecIkFkMatchConstraint presents: while its switch sits on
    the FK value the IK control, the pole and every control under them are
    drawn on the joints; on the IK value the FK controls and every control
    under them are. Their avars still drive nothing until the switch
    changes, so a manipulator on one is drawn where it is shown
    (gizmoMath._PreferPublishedFrame), and grabbing it switches the limb first.
    """

    def __init__(self, switch, ikValue, ikSide, fkSide):
        self.switch = switch        # Sdf.Path of the switch channel
        self.ikValue = ikValue
        self.ikSide = ikSide        # set of control paths
        self.fkSide = fkSide

    @staticmethod
    def From(prim):
        attr = prim.GetAttribute("inputs:switch")
        sources = attr.GetConnections() if attr else []
        if len(sources) != 1:
            return None
        ikValue = prim.GetAttribute("rigExec:ikValue")
        ikValue = float(ikValue.Get()) if ikValue and \
            ikValue.Get() is not None else 1.0

        def side(names):
            roots = []
            for name in names:
                rel = prim.GetRelationship(name)
                roots += list(rel.GetTargets()) if rel else []
            controls = set()
            stage = prim.GetStage()
            for root in roots:
                start = stage.GetPrimAtPath(root)
                if not start:
                    continue
                for under in Usd.PrimRange(start):
                    if under.GetTypeName() == "RigExecControl":
                        controls.add(under.GetPath())
            return controls
        return MatchHalf(sources[0], ikValue,
                         side(("rigExec:ikControl", "rigExec:poleControl")),
                         side(("rigExec:fkControls",)))

    def Presented(self, stage, time):
        """The controls presented now: one side, or none between values."""
        attr = stage.GetAttributeAtPath(self.switch)
        value = attr.Get(time) if attr else None
        if value is None:
            return set()
        value = float(value)
        if abs(value - (1.0 - self.ikValue)) <= 1e-9:
            return self.ikSide
        if abs(value - self.ikValue) <= 1e-9:
            return self.fkSide
        return set()


def PresentedBy(stage, path, time, halves):
    """The MatchHalf of `halves` presenting `path` at `time`, or None."""
    for half in halves:
        if path in half.Presented(stage, time):
            return half
    return None


# -- published values -----------------------------------------------------

def PoseValueReader(pose):
    """read(path) -> [float] or None over one rigexec.Pose."""
    def read(path):
        try:
            return list(pose.moved_property(str(path)))
        except (KeyError, TypeError):
            return None
    return read


# The hosts' readers, one per usdview session: (stage, property path, time)
# -> [float]/None from a generation of the stage as it stands at that time.
# Each answers for its own session's stage only, so a read asks them all and
# takes the answer. Filed under the session, and dropped with it.
_publishedValuesReaders = sessionRegistry.SessionRegistry(
    "ikfkMatch published values readers")


def SetPublishedValuesReader(reader, session):
    """Install `session`'s reader; None removes it."""
    if reader is None:
        _publishedValuesReaders.Pop(session)
    else:
        _publishedValuesReaders.Set(session, reader)


def PublishedValueReader(stage, time):
    """read(path) -> [float] or None from whichever host session draws
    `stage`, at `time`; None when no session does."""
    readers = list(_publishedValuesReaders.Values())
    if not readers:
        return None

    def read(path):
        for reader in readers:
            values = reader(stage, path, time)
            if values is not None:
                return values
        return None
    return read


def Values(limb, read, toIk):
    """The published values of the half named by `toIk`, or raise."""
    name = IK_VALUES if toIk else FK_VALUES
    values = read(limb.match.AppendProperty(name))
    count = 11 if toIk else 20
    if values is None or len(values) != count or \
            not all(math.isfinite(v) for v in values):
        raise RuntimeError("%s published no %s" % (limb.match, name))
    return values


def Switch(limb, toIk):
    """{attribute path: value} handing the limb's joints to the named
    half."""
    return {limb.switchPath: limb.ikValue if toIk else limb.fkValue}


def Match(limb, stage, read, toIk, written=None):
    """{attribute path: value} posing the half named by `toIk` onto the
    limb's current joints, from the values the rig published. Channels the
    rig writes or drives, or that do not exist, are left alone."""
    writable = limb.writable
    if writable is None or written is not None:
        writable = Writable(stage, limb,
                            written if written is not None
                            else RigWritten(stage))
    values = Values(limb, read, toIk)
    out = {}

    def put(control, names, numbers):
        for name, number in zip(names, numbers):
            path = control.AppendProperty("avars:" + name)
            if path in writable:
                out[path] = float(number)

    def channel(path, value):
        if path is not None and path in writable:
            out[path] = float(value)

    if toIk:
        put(limb.ikControl, _TRANSLATE + _ROTATE, values[0:6])
        put(limb.pole, _TRANSLATE, values[6:9])
        channel(limb.upperScale, values[9])
        channel(limb.lowerScale, values[10])
    else:
        for i, control in enumerate(limb.fkControls):
            put(control, _TRANSLATE + _ROTATE, values[i * 6:i * 6 + 6])
        channel(limb.upperScale, values[18])
        channel(limb.lowerScale, values[19])
        channel(limb.pin, 0.0)
        channel(limb.twist, 0.0)
    return out


def Plan(limb, stage, time, read, toIk=None, written=None, current=None):
    """Match then Switch, as one set of values: what a "Switch to IK / FK"
    command authors. `toIk` defaults to the half that is not driving now.
    """
    if toIk is None:
        toIk = not limb.IsIk(stage, time, current)
    values = Match(limb, stage, read, toIk, written)
    values.update(Switch(limb, toIk))
    return values, toIk


# -- frames and channels --------------------------------------------------

def _Get(stage, attrPath, time, fallback):
    attr = stage.GetAttributeAtPath(attrPath)
    value = attr.Get(time) if attr else None
    return float(value) if value is not None else fallback


def _Matrix(values):
    return Gf.Matrix4d(*[values[i * 4:i * 4 + 4] for i in range(4)])


def JointFrame(pose, path):
    return _Matrix(pose.joint_frame(str(path), True).to_matrix4())


def ControlFrame(pose, path):
    return _Matrix(pose.control_frame(str(path)).to_matrix4())


def _Frame(pose, path):
    """A control's or joint's posed frame, whichever the pose has."""
    try:
        return ControlFrame(pose, path)
    except KeyError:
        return JointFrame(pose, path)


class _Channels(object):
    """One control's channel values at a time, and its avar matrix."""

    def __init__(self, stage, path, time, overrides=None):
        prim = stage.GetPrimAtPath(path)
        self.path = path
        self.prim = prim
        self.order = str(prim.GetAttribute("avars:rotationOrder").Get()
                         or "XYZ")
        self.sign = gizmoMath.RotationSign(prim)
        self.values = {}
        for name, fallback in _IDENTITY.items():
            attrPath = path.AppendProperty("avars:" + name)
            if overrides is not None and attrPath in overrides:
                self.values[name] = float(overrides[attrPath])
            else:
                self.values[name] = _Get(stage, attrPath, time, fallback)

    def Effective(self):
        """Rotation angles as the rig applies them (raw * sign)."""
        return tuple(self.values[r] * s for r, s in zip(_ROTATE, self.sign))

    def Matrix(self, values=None):
        v = dict(self.values, **(values or {}))
        rx, ry, rz = (v[r] * s for r, s in zip(_ROTATE, self.sign))
        return gizmoMath.ComposeAvarMatrix(
            v["tx"], v["ty"], v["tz"], v["sx"], v["sy"], v["sz"],
            rx, ry, rz, v["rspin"] * self.sign[0], self.order)

    def Solve(self, target, parent):
        """Translate and rotate values that put this control's posed frame
        on `target`, with `parent` the frame its avars compose onto."""
        local = target * parent.GetInverse()
        translate = local.ExtractTranslation()
        # local = S * R * Rspin * T: take the translation off, then the
        # scale (which a match never writes) and spin (likewise) from
        # either side of the rotation.
        linear = Gf.Matrix4d(local)
        linear.SetTranslateOnly(Gf.Vec3d(0.0))
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(Gf.Vec3d(
            gizmoMath.NormalizeAvarScale(self.values["sx"]),
            gizmoMath.NormalizeAvarScale(self.values["sy"]),
            gizmoMath.NormalizeAvarScale(self.values["sz"])))
        spin = Gf.Matrix4d(1.0)
        if self.values["rspin"]:
            spin.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0),
                                       self.values["rspin"] * self.sign[0]))
        rotation = scale.GetInverse() * linear * spin.GetInverse()
        effective = gizmoMath.DecomposeEuler(rotation, self.order,
                                             hint=self.Effective())
        solved = {t: float(translate[i]) for i, t in enumerate(_TRANSLATE)}
        for r, angle, sign in zip(_ROTATE, effective, self.sign):
            solved[r] = float(angle * sign)
        return solved

    def SolveTranslate(self, point, parent):
        """Translate values that put this control's origin on `point`; its
        rotation and scale stay as they are."""
        local = parent.GetInverse().Transform(point)
        return {t: float(local[i]) for i, t in enumerate(_TRANSLATE)}


def _Writable(stage, path, names, written):
    """The channels of `names` this control has and the rig does not write
    or drive."""
    prim = stage.GetPrimAtPath(path)
    keep = []
    for name in names:
        attr = prim.GetAttribute("avars:" + name) if prim else None
        if not attr or attr.GetPath() in written or \
                attr.HasAuthoredConnections():
            continue
        keep.append(name)
    return keep


def RigWritten(stage):
    """Attribute paths a mover writes: overriding one would freeze the rig,
    so a match never does."""
    written = set()
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:moves")
        if rel:
            written.update(rel.GetTargets())
    return written


def Residual(limb, evaluate, time, values):
    """How far the limb's joints move when `values` are applied: (worst
    position error, worst rotation error in degrees) over its three
    joints. A switch that matched moves nothing."""
    before = evaluate({}, time)
    after = evaluate(values, time)
    worstPosition = worstRotation = 0.0
    for joint in limb.joints:
        a, b = JointFrame(before, joint), JointFrame(after, joint)
        worstPosition = max(worstPosition, (a.ExtractTranslation() -
                                            b.ExtractTranslation())
                            .GetLength())
        qa = a.GetOrthonormalized().ExtractRotationQuat()
        qb = b.GetOrthonormalized().ExtractRotationQuat()
        dot = min(1.0, abs(Gf.Dot(qa, qb)))
        worstRotation = max(worstRotation, math.degrees(2.0 * math.acos(dot)))
    return worstPosition, worstRotation


def Evaluator(stage, rigPath):
    """An explicit matching evaluator retaining the caller's exact stage.

    The returned callable owns the evaluator; no process-wide cache retains
    stages after the caller releases it.
    """
    import rigexec
    rig = rigexec.Rig(stage, str(rigPath))
    rig.compile()
    rig.publish_weight_fields = False

    def evaluate(overrides, time):
        rig.set_interactive_overrides(
            [(str(p.GetPrimPath()), p.name, float(v))
             for p, v in overrides.items()])
        try:
            return rig.evaluate(-1.0 if time.IsDefault()
                                else time.GetValue())
        finally:
            rig.clear_interactive_overrides()

    evaluate.rig = rig
    return evaluate

# -- applying a switch -----------------------------------------------------

def PlanSwitch(stage, limbs, time, read=None):
    """({attribute path: value}, [(limb, toIk, channels)]) that switch every
    limb in `limbs` to its other half, matched, without authoring. `read`
    defaults to the host session drawing `stage`."""
    if read is None:
        read = PublishedValueReader(stage, time)
    if read is None:
        raise RuntimeError("no rig publishes IK/FK values for this stage")
    values = {}
    done = []
    for limb in limbs:
        planned, toIk = Plan(limb, stage, time, read)
        values.update(planned)
        done.append((limb, toIk, len(planned) - 1))
    return values, done


def SwitchLabel(done):
    """The undo label for PlanSwitch's `done`."""
    return "Switch %s" % ", ".join(
        "%s to %s" % (limb.switchControl.name, "IK" if toIk else "FK")
        for limb, toIk, _ in done)


def SwitchLimbs(stage, limbs, time, mode, undoStack=None, read=None):
    """Switch every limb in `limbs` to its other half, matched, authored as
    ONE edit and one undo entry. `mode` is a gizmoMath write mode, so a
    switch lands the way a drag does (keyed in animation mode). Returns
    [(limb, toIk, channels)] for the limbs switched."""
    values, done = PlanSwitch(stage, limbs, time, read)
    if not values:
        return []
    label = SwitchLabel(done)
    scope = None
    if undoStack is not None:
        import rigExecUndo
        scope = rigExecUndo.SpecScope(stage, list(values), undoStack, label)
        scope.__enter__()
    try:
        writer = gizmoMath.Writer(stage, time, mode)
        for path, value in values.items():
            attr = stage.GetAttributeAtPath(path)
            if attr:
                writer.Set(attr, value)
        writer.CommitToStage()
    except Exception as error:
        if scope is not None:
            scope.__exit__(type(error), error, None)
        raise
    if scope is not None:
        scope.__exit__(None, None, None)
    return done
