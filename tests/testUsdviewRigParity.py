#
# 2D rig evaluation parity, end to end (testusdview, a GUI session).
#
# Run by bin/test/run_testusdview_rig_parity.sh. Every authoring step below is
# plain USD written to the root layer and recorded on a local undo stack; the
# evaluated bone frames are read back from the frames Hydra publishes
# (rigExecUsdview.ContainerFor(api).ReadPublishedControlFrame). Any failure
# raises, so testusdview exits non-zero. Offsets, targets and controls also round-trip through undo/redo.
#
# Held splines (the index/step channels) are asserted at their key frames
# only; between-key stepping is covered by the C++ stage tests, which author
# .spline syntax directly.
#
# The stage (written by the runner into $TMPDIR, startTimeCode 0 so the setup
# frame is reachable, endTimeCode 24): /Doc (Xform) with /Doc/BoneLayer
# (Xform) and a curve stroke /Doc/BoneLayer/Art. Step 1 makes the layer a
# bone layer (skeleton + nested bones + bind mover). Bones nest under their
# parents with parent-relative rests; a bone's tip is its first joint child,
# an End joint for a leaf with a length; an animated parent is a
# RigExecSpaceSwitch on the bone.
#
import contextlib
import json
import math
import os
import sys
import tempfile
import time
import traceback

from pxr import Gf, Sdf, Usd

LAYER = "/Doc/BoneLayer"
ART = LAYER + "/Art"
# The rig root a bone layer carries (RigExec activates RigExecRoot prims
# only): skeleton, bones and movers live under it, the art beside it.
RIG = LAYER + "/Rig"
SKEL = RIG + "/Skeleton"
BONES = RIG + "/Bones"
SPACES = RIG + "/Spaces"

# (name, parent, x, y, angle, length): absolute setup frames; a bone with
# children runs to its first child, a leaf with a length gets an End joint.
BONE_TABLE = [
    ("H1", "", 0, 0.7, -90, 0.2),
    ("T", "H1", 0, 0.5, -90, 0.4),
    ("S", "T", 0, 0.1, -90, 0.4),
    ("F", "S", 0, -0.3, 0, 0.2),
    ("P", "", 0.3, -0.2, 0, 0),
    ("D", "", -0.8, 0.5, 0, 0.1),
    ("M", "", -0.5, 0.5, 0, 0.1),
    ("HA", "H1", 0, 0.9, 0, 0.2),
    ("JA", "HA", 0, 1.1, 0, 0.1),
    ("HB", "H1", 0, 0.9, 0, 0.2),
    ("X", "S", 0, -0.1, 0, 0.1),
]
_PARENT = {row[0]: row[1] for row in BONE_TABLE}
_LENGTH = {row[0]: row[5] for row in BONE_TABLE}


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _ReportPath():
    return os.environ.get("RIGEXEC_RIGPARITY_REPORT",
                          os.path.join(tempfile.gettempdir(), "rigparity.json"))


def _ShotsDir():
    return os.environ.get(
        "RIGEXEC_RIGPARITY_SHOTS",
        os.path.join(tempfile.gettempdir(), "rigparity-shots"))


def _Wrap180(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def _Near(actual, expected, tolerance, what):
    _Check(abs(actual - expected) <= tolerance,
           "%s: %r, expected %r (+-%g)" % (what, actual, expected, tolerance))


def _SetAttr(prim, name, value, valueType, time=None):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, valueType)
    if time is None:
        _Check(attr.Set(value), "set %s" % prim.GetPath().AppendProperty(name))
    else:
        _Check(attr.Set(value, Usd.TimeCode(float(time))),
               "set %s at %s" % (prim.GetPath().AppendProperty(name), time))
    return attr


class _Report(object):
    """Named checks written to JSON; prints one line per check."""

    def __init__(self, path, prefix):
        self.path = path
        self.prefix = prefix
        self.checks = []
        self.infos = {}
        self.started = time.time()

    def check(self, name, fn):
        started = time.time()
        try:
            ok = fn() is not False
            error = None if ok else "returned False"
        except Exception as exc:  # noqa: BLE001
            ok = False
            error = "%s: %s\n%s" % (type(exc).__name__, exc,
                                    traceback.format_exc())
        self.checks.append({"name": name, "ok": ok, "error": error,
                            "seconds": round(time.time() - started, 3)})
        print("%s %s: %s%s" % (self.prefix, "ok" if ok else "FAIL", name,
                               "" if ok else "\n" + (error or "")))
        return ok

    def info(self, key, value):
        self.infos[key] = value

    @property
    def ok(self):
        return bool(self.checks) and all(c["ok"] for c in self.checks)

    def write(self):
        data = {"prefix": self.prefix, "ok": self.ok,
                "passed": sum(1 for c in self.checks if c["ok"]),
                "failed": sum(1 for c in self.checks if not c["ok"]),
                "checks": self.checks, "info": self.infos,
                "seconds": round(time.time() - self.started, 3)}
        folder = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(folder, exist_ok=True)
        with open(self.path, "w") as f:
            json.dump(data, f, indent=1, sort_keys=True, default=str)
        return self.ok


class _Pose(object):
    """One bone's evaluated placement in asset space, from a published
    row-vector frame."""

    def __init__(self, matrix, length):
        xx, xy = matrix[0][0], matrix[0][1]
        yx, yy = matrix[1][0], matrix[1][1]
        self.baseX = matrix[3][0]
        self.baseY = matrix[3][1]
        self.xAxis = (xx, xy)
        self.yAxis = (yx, yy)
        self.length = length
        self.angle = math.degrees(math.atan2(xy, xx))
        self.parity = -1 if xx * yy - xy * yx < 0 else 1

    def Tip(self):
        return (self.baseX + self.length * self.xAxis[0],
                self.baseY + self.length * self.xAxis[1])


class _Session(object):
    def __init__(self, appController, report):
        import rigExecUndo
        self.appController = appController
        self.report = report
        self.undo = rigExecUndo
        self.stack = rigExecUndo.UndoStack()
        self.api = appController._usdviewApi
        self.view = appController._stageView

    @property
    def stage(self):
        return self.api.stage

    def ActivateWindow(self):
        window = self.api.qMainWindow
        if window is not None:
            window.raise_()
            window.activateWindow()
        for _ in range(10):
            self.appController._processEvents()

    @contextlib.contextmanager
    def Txn(self, label):
        """Author to the root layer; record the layer subtree before/after
        on the local undo stack as one edit."""
        layer = self.stage.GetRootLayer()
        before = self.undo.SpecSnapshot.Capture(layer, Sdf.Path(LAYER))
        with Usd.EditContext(self.stage, layer):
            yield
        after = self.undo.SpecSnapshot.Capture(layer, Sdf.Path(LAYER))
        self.stack.Push(self.undo.Edit(label, [
            self.undo.EditEntry(layer, Sdf.Path(LAYER), before, after)]))

    def Settle(self, rounds=2):
        for _ in range(rounds):
            try:
                self.api.UpdateViewport()
            except Exception:  # noqa: BLE001
                pass
            self.appController._processEvents()

    def WaitFor(self, condition, seconds=10.0):
        end = time.time() + seconds
        while time.time() < end:
            self.appController._processEvents()
            if condition():
                return True
            time.sleep(0.05)
        return condition()

    def SetFrame(self, frame):
        # The root data model's currentFrame drives usdview's time (view
        # settings have no frame; assigning one only adds an attribute).
        self.api.dataModel.currentFrame = Usd.TimeCode(float(frame))
        self.Settle()

    def BonePath(self, name):
        parent = _PARENT.get(name, "")
        return (self.BonePath(parent) if parent else BONES) + "/" + name

    def Snap(self, frame):
        """{bone name: _Pose} of the frames Hydra published at `frame`; None
        until every bone answers."""
        import rigExecUsdview
        container = rigExecUsdview.ContainerFor(self.api)
        if container is None or not self.stage.GetPrimAtPath(BONES):
            return None
        out = {}
        for name, _parent, _x, _y, _angle, length in BONE_TABLE:
            matrix = container.ReadPublishedControlFrame(
                self.stage, Sdf.Path(self.BonePath(name)),
                Usd.TimeCode(float(frame)))
            if matrix is None:
                return None
            out[name] = _Pose(matrix, length)
        return out

    def Published(self, frame, seconds=10.0):
        # The published frames describe the displayed time only: show it.
        if self.api.dataModel.currentFrame != Usd.TimeCode(float(frame)):
            self.SetFrame(frame)
        box = {}

        def _Read():
            box["snap"] = self.Snap(frame)
            return box["snap"] is not None
        _Check(self.WaitFor(_Read, seconds),
               "no evaluated snapshot at %s" % frame)
        return box["snap"]

    def Pose(self, snap, name):
        _Check(name in snap, "no bone %s in snapshot" % name)
        return snap[name]

    def Shot(self, name):
        path = os.path.join(_ShotsDir(), name)
        self.Settle(3)
        pixmap = self.view.grab()
        _Check(not pixmap.isNull() and pixmap.save(path),
               "could not grab the viewport to %s" % path)
        _Check(os.path.getsize(path) > 0, "empty grab %s" % path)
        self.report.info("shot " + name, path)
        return path

    def UndoRedo(self, check):
        label = self.stack.UndoText()
        _Check(self.stack.Undo(), "undo")
        self.Settle()
        check("after undo")
        _Check(self.stack.Redo(), "redo")
        self.Settle()
        check("after redo")
        _Check(self.stack.UndoText() == label, "entry restored")


def _RestMatrix(x, y, angle):
    rad = math.radians(angle)
    c, s = math.cos(rad), math.sin(rad)
    return Gf.Matrix4d(c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, x, y, 0, 1)


def _LocalRest(row, parentRow):
    """The parent-relative rest:space of a bone given absolute frames."""
    _name, _parent, x, y, angle, _length = row
    if parentRow is None:
        return _RestMatrix(x, y, angle)
    _pn, _pp, px, py, pangle, _pl = parentRow
    rad = math.radians(pangle)
    c, s = math.cos(rad), math.sin(rad)
    dx, dy = x - px, y - py
    return _RestMatrix(dx * c + dy * s, -dx * s + dy * c, angle - pangle)


def _Steps(s):
    report = s.report
    stage = s.stage

    # -- 1. the rig ------------------------------------------------------
    def BuildRig():
        with s.Txn("New Bone Layer"):
            stage.DefinePrim(RIG, "RigExecRoot")
            stage.DefinePrim(SKEL, "RigExecSkeleton2d")
            stage.DefinePrim(BONES, "Scope")
            rows = {row[0]: row for row in BONE_TABLE}
            parents = set(_PARENT.values())
            joints = []
            # Parents before children, each child's first: a bone's first
            # joint child is its tip.
            for row in BONE_TABLE:
                name, parent, _x, _y, _angle, length = row
                joint = stage.DefinePrim(s.BonePath(name), "RigExecJoint")
                joint.AddAppliedSchema("RigExecBone2dAPI")
                _SetAttr(joint, "rest:space",
                         _LocalRest(row, rows.get(parent)),
                         Sdf.ValueTypeNames.Matrix4d)
                if name not in parents and length > 0:
                    end = stage.DefinePrim(
                        joint.GetPath().AppendChild("End"), "RigExecJoint")
                    _SetAttr(end, "rest:space", _RestMatrix(length, 0, 0),
                             Sdf.ValueTypeNames.Matrix4d)
                joints.append(Sdf.Path(s.BonePath(name)))
            stage.GetPrimAtPath(SKEL).CreateRelationship(
                "rigExec:joints").SetTargets(joints)
        with s.Txn("Bind Art"):
            # A capsule inside every bone owns its region; the skin's rows
            # are generated from the capsules, as the binding tools do.
            for row in BONE_TABLE:
                capsule = stage.DefinePrim(
                    s.BonePath(row[0]) + "/Influence", "RigExecCapsuleWeight")
                _SetAttr(capsule, "inputs:falloffMax", 0.15,
                         Sdf.ValueTypeNames.Float)
            mover = stage.DefinePrim(RIG + "/BindArt", "RigExecSkinMover")
            mover.AddAppliedSchema("RigExecMoverAPI")
            mover.CreateRelationship("rigExec:influences").SetTargets(joints)
            mover.CreateRelationship("rigExec:moves").SetTargets(
                [Sdf.Path(ART + ".points")])
            import rigexec
            rows = rigexec.binding.generate_skin_weights(stage, RIG + "/BindArt")
            _Check(rows["rows_written"] == len(
                stage.GetPrimAtPath(ART).GetAttribute("points").Get()),
                "every art point has a row")
    report.check("1. rig builds in two edits", BuildRig)
    s.SetFrame(0)

    def BonesLive():
        snap = s.Published(0)
        _Check(len(snap) == 11, "eleven bones, got %d" % len(snap))
    report.check("1a. eleven evaluated bones at frame 0", BonesLive)

    # -- 2. offsets -------------------------------------------------------
    def AuthorOffset():
        with s.Txn("Offset Bone"):
            bone = stage.GetPrimAtPath(s.BonePath("F"))
            _SetAttr(bone, "bone2d:offsetX", 0.0, Sdf.ValueTypeNames.Double, 0)
            _SetAttr(bone, "bone2d:offsetX", 0.3, Sdf.ValueTypeNames.Double, 1)
            _SetAttr(bone, "bone2d:offsetY", 0.0, Sdf.ValueTypeNames.Double, 0)
            _SetAttr(bone, "bone2d:offsetY", 0.1, Sdf.ValueTypeNames.Double, 1)
    report.check("2. offset authors in one edit", AuthorOffset)

    def OffsetApart():
        pose = s.Pose(s.Published(0), "F")
        _Near(pose.baseX, 0.0, 1e-6, "frame 0 apart x")
        _Near(pose.baseY, -0.3, 1e-6, "frame 0 apart y")
    report.check("2a. frame 0 shows the bone apart", OffsetApart)

    def OffsetExpected(snap):
        # bone2d:offset is in the parent's frame (schema): F's
        # rest base (0, -0.3) plus (0.3, 0.1) along S's posed axes.
        parent = s.Pose(snap, "S")
        return (0.0 + 0.3 * parent.xAxis[0] + 0.1 * parent.yAxis[0],
                -0.3 + 0.3 * parent.xAxis[1] + 0.1 * parent.yAxis[1])

    def OffsetAssembled():
        s.SetFrame(10)
        snap = s.Published(10)
        pose = s.Pose(snap, "F")
        x, y = OffsetExpected(snap)
        _Near(pose.baseX, x, 1e-6, "frame 10 assembled x")
        _Near(pose.baseY, y, 1e-6, "frame 10 assembled y")
        s.Shot("rigparity-offset.png")
    report.check("2b. frame 10 assembles the bone", OffsetAssembled)

    def OffsetUndoCheck(when):
        s.SetFrame(10)
        snap = s.Published(10)
        pose = s.Pose(snap, "F")
        x = 0.0 if when == "after undo" else OffsetExpected(snap)[0]
        _Near(pose.baseX, x, 1e-6, "offset %s" % when)
    report.check("2c. offset undo/redo", lambda: s.UndoRedo(OffsetUndoCheck))

    # -- 3. IK targets ----------------------------------------------------
    def AuthorTarget():
        with s.Txn("Assign Target"):
            bone = stage.GetPrimAtPath(s.BonePath("S"))
            bone.CreateRelationship("bone2d:targetCandidates").SetTargets(
                [Sdf.Path(s.BonePath("P"))])
            _SetAttr(bone, "bone2d:targetIndex", 0.0,
                     Sdf.ValueTypeNames.Double)
    report.check("3. target assigns in one edit", AuthorTarget)

    def TargetTracks():
        s.SetFrame(10)
        snap = s.Published(10)
        tip = s.Pose(snap, "S").Tip()
        pin = s.Pose(snap, "P")
        _Check(math.hypot(tip[0] - pin.baseX, tip[1] - pin.baseY) <= 1e-3,
               "tip on target: %r vs (%r, %r)" % (tip, pin.baseX, pin.baseY))
        s.Shot("rigparity-target.png")
    report.check("3a. the shin tip tracks the pin", TargetTracks)

    def TargetFollows():
        with s.Txn("Move Target"):
            _SetAttr(stage.GetPrimAtPath(s.BonePath("P")), "avars:tx", -0.2,
                     Sdf.ValueTypeNames.Double, 20)
        s.SetFrame(20)
        snap = s.Published(20)
        tip = s.Pose(snap, "S").Tip()
        pin = s.Pose(snap, "P")
        _Check(math.hypot(tip[0] - pin.baseX, tip[1] - pin.baseY) <= 1e-3,
               "tip follows: %r vs (%r, %r)" % (tip, pin.baseX, pin.baseY))
    report.check("3b. the tip follows the animated pin", TargetFollows)

    # -- 4. animated reparenting ------------------------------------------
    def AuthorReparent():
        # A space switch on X, in the authoring tools' shape: source 0 the
        # namespace parent S, source 1 H1, the index X's own held
        # avars:space. The snapshot, as the reparent tool writes it: X's
        # placement at frame 9 (on S, which the pin of step 3 already bends)
        # expressed as a local under H1 at frame 10, where H1 has moved to
        # tx 0.5. Locals are the rest-local under the new parent plus the
        # channels, in the parent's frame: rest-local of X under H1 is
        # (0.8, 0) at 90 degrees (X rests at (0, -0.1) angle 0, H1 at
        # (0, 0.7) angle -90).
        held = s.Pose(s.Published(9), "X")
        parentBase, parentAngle = (0.5, 0.7), -90.0
        rad = math.radians(parentAngle)
        xAxis, yAxis = (math.cos(rad), math.sin(rad)), (-math.sin(rad), math.cos(rad))
        dx, dy = held.baseX - parentBase[0], held.baseY - parentBase[1]
        tx = dx * xAxis[0] + dy * xAxis[1] - 0.8
        ty = dx * yAxis[0] + dy * yAxis[1] - 0.0
        rz = (held.angle - parentAngle) - 90.0
        with s.Txn("Reparent Bone"):
            bone = stage.GetPrimAtPath(s.BonePath("X"))
            stage.DefinePrim(SPACES, "Scope")
            switch = stage.DefinePrim(SPACES + "/X", "RigExecSpaceSwitch")
            switch.CreateRelationship("rigExec:target").SetTargets(
                [bone.GetPath()])
            # Index 0 keeps X on S, index 1 moves it to H1.
            switch.CreateRelationship("rigExec:sources").SetTargets(
                [Sdf.Path(s.BonePath("S")), Sdf.Path(s.BonePath("H1"))])
            switch.CreateRelationship("rigExec:activeSpaceAttribute").SetTargets(
                [bone.GetPath().AppendProperty("avars:space")])
            _SetAttr(bone, "avars:space", 0.0, Sdf.ValueTypeNames.Double)
            _SetAttr(bone, "avars:space", 0.0, Sdf.ValueTypeNames.Double, 9)
            _SetAttr(bone, "avars:space", 1.0, Sdf.ValueTypeNames.Double, 10)
            _SetAttr(stage.GetPrimAtPath(s.BonePath("H1")), "avars:tx", 0.0,
                     Sdf.ValueTypeNames.Double, 9)
            _SetAttr(stage.GetPrimAtPath(s.BonePath("H1")), "avars:tx", 0.5,
                     Sdf.ValueTypeNames.Double, 10)
            for attr in ("avars:tx", "avars:ty", "avars:rz"):
                _SetAttr(bone, attr, 0.0, Sdf.ValueTypeNames.Double, 9)
            _SetAttr(bone, "avars:tx", tx, Sdf.ValueTypeNames.Double, 10)
            _SetAttr(bone, "avars:ty", ty, Sdf.ValueTypeNames.Double, 10)
            _SetAttr(bone, "avars:rz", rz, Sdf.ValueTypeNames.Double, 10)
    report.check("4. reparent authors in one edit", AuthorReparent)

    def ReparentSnapshot():
        nine = s.Published(9)
        ten = s.Published(10)
        before, after = s.Pose(nine, "X"), s.Pose(ten, "X")
        _Check(math.hypot(after.baseX - before.baseX,
                          after.baseY - before.baseY) <= 1e-6,
               "snapshot keeps placement: X at 9 (%r, %r) angle %r, at 10 "
               "(%r, %r) angle %r; H1 at 10 (%r, %r) angle %r; S at 9 (%r, %r)"
               % (before.baseX, before.baseY, before.angle, after.baseX,
                  after.baseY, after.angle, s.Pose(ten, "H1").baseX,
                  s.Pose(ten, "H1").baseY, s.Pose(ten, "H1").angle,
                  s.Pose(nine, "S").baseX, s.Pose(nine, "S").baseY))
        with s.Txn("Move New Parent"):
            _SetAttr(stage.GetPrimAtPath(s.BonePath("H1")), "avars:tx", 1.0,
                     Sdf.ValueTypeNames.Double, 20)
        s.SetFrame(20)
        moved = s.Pose(s.Published(20), "X")
        _Near(moved.baseX, after.baseX + 0.5, 1e-6, "follows the new parent")
        s.Shot("rigparity-reparent.png")
    report.check("4a. snapshot then follows", ReparentSnapshot)

    # -- 5. bone groups ---------------------------------------------------
    def AuthorGroup():
        with s.Txn("Bone Groups"):
            group = stage.DefinePrim(SKEL + "/G", "Scope")
            group.AddAppliedSchema("RigExecBone2dGroupAPI")
            group.CreateRelationship("collection:members:includes").SetTargets(
                [Sdf.Path(s.BonePath("HA")), Sdf.Path(s.BonePath("HB"))])
            stage.GetPrimAtPath(SKEL).CreateRelationship(
                "rigExec:boneGroups").SetTargets([group.GetPath()])
            _SetAttr(group, "rigExec:activeBone", 0.0,
                     Sdf.ValueTypeNames.Double, 1)
            _SetAttr(group, "rigExec:activeBone", 1.0,
                     Sdf.ValueTypeNames.Double, 10)
            for head in ("HA", "HB"):
                _SetAttr(stage.GetPrimAtPath(s.BonePath(head)), "avars:rz",
                         30.0, Sdf.ValueTypeNames.Double)
    report.check("5. group authors in one edit", AuthorGroup)

    def GroupSwitches():
        first = s.Published(1)
        _Near(_Wrap180(s.Pose(first, "HA").angle), 30, 1e-6, "HA active at 1")
        _Near(_Wrap180(s.Pose(first, "HB").angle), 0, 1e-6, "HB rests at 1")
        second = s.Published(10)
        _Near(_Wrap180(s.Pose(second, "HA").angle), 0, 1e-6, "HA rests at 10")
        _Near(_Wrap180(s.Pose(second, "HB").angle), 30, 1e-6, "HB active at 10")
        s.SetFrame(10)
        s.Shot("rigparity-groups.png")
    report.check("5a. the active member switches", GroupSwitches)

    # -- 6. control bones -------------------------------------------------
    def AuthorControl():
        with s.Txn("Bone Control"):
            _SetAttr(stage.GetPrimAtPath(s.BonePath("D")), "avars:rz", 40.0,
                     Sdf.ValueTypeNames.Double, 10)
            muscle = stage.GetPrimAtPath(s.BonePath("M"))
            muscle.CreateRelationship("bone2d:angleControl").SetTargets(
                [Sdf.Path(s.BonePath("D"))])
            _SetAttr(muscle, "bone2d:angleControlFactor", 0.5,
                     Sdf.ValueTypeNames.Double)
            _SetAttr(muscle, "avars:rz", 10.0, Sdf.ValueTypeNames.Double)
    report.check("6. control authors in one edit", AuthorControl)

    def ControlResponds():
        s.SetFrame(10)
        snap = s.Published(10)
        _Near(_Wrap180(s.Pose(snap, "M").angle), 30, 1e-6, "muscle follows")
        s.Shot("rigparity-control.png")
    report.check("6a. the muscle answers the dial", ControlResponds)

    def ControlUndoCheck(when):
        s.SetFrame(10)
        angle = _Wrap180(s.Pose(s.Published(10), "M").angle)
        # Undo reverts the whole edit, the muscle's own rz 10 with the
        # link: it rests (0); redo brings back 10 + 0.5 * the dial's 40.
        _Near(angle, 0 if when == "after undo" else 30, 1e-6,
              "control %s" % when)
    report.check("6b. control undo/redo", lambda: s.UndoRedo(ControlUndoCheck))

    # -- 7. hygiene --------------------------------------------------------
    def Hygiene():
        _Check(stage.GetEditTarget() == stage.GetSessionLayer(),
               "edit target is the session layer")
    report.check("7. edit target untouched", Hygiene)


def testUsdviewInputFunction(appController):
    from pxr.Usdviewq.qt import QtCore

    report = _Report(_ReportPath(), "RIGEXEC_RIGPARITY")
    exceptions = []

    def _Handler(mode, context, message):
        sys.stderr.write(str(message) + "\n")
    QtCore.qInstallMessageHandler(_Handler)
    previousHook = sys.excepthook

    def _ExceptHook(kind, value, tb):
        exceptions.append("%s: %s" % (kind.__name__, value))
        previousHook(kind, value, tb)
    sys.excepthook = _ExceptHook

    s = _Session(appController, report)
    api = s.api
    try:
        api.viewerMode = True
        ratio = s.view.devicePixelRatioF()
        s.view.SetPhysicalWindowSize(int(1200 * ratio), int(800 * ratio))
        api.qMainWindow.resize(1300, 1000)
    except Exception:  # noqa: BLE001
        pass
    appController._processEvents()
    s.ActivateWindow()
    try:
        api.dataModel.viewSettings.showHUD = False
        api.dataModel.viewSettings.showBBoxes = False
    except Exception:  # noqa: BLE001
        pass
    if not os.path.isdir(_ShotsDir()):
        os.makedirs(_ShotsDir())
    report.info("stage", s.stage.GetRootLayer().identifier)
    try:
        _Steps(s)
    finally:
        sys.excepthook = previousHook
        QtCore.qInstallMessageHandler(None)

    def NoExceptions():
        _Check(not exceptions, "Python exceptions in Qt slots: %s"
               % exceptions)
    report.check("no exceptions in Qt slots", NoExceptions)
    ok = report.write()
    print("report: %s" % _ReportPath())
    if not ok:
        failed = [c["name"] for c in report.checks if not c["ok"]]
        raise AssertionError("rig parity session failed: %s" % failed)
    print("RIGEXEC_RIGPARITY_OK %d checks" % len(report.checks))
