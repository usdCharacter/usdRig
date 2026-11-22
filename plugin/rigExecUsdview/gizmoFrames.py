#
# RigExec usdview gizmo frames: rig-root discovery, pose providers and the
# rest / default / posed frames of a control, in asset space, computed the way
# the evaluator computes them.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): everything here is
# exercised headlessly by tests/python/test_gizmo_frames.py, including a
# comparison against the native evaluator through the _rigexec binding.
#
# Conventions (libs/rigExec/internal/computations.cpp:144-175, 196-215, 284-285),
# row-vector, leftmost factor applies first:
#
#   avars = S * R(rotationOrder) * Rspin(+X) * T
#   rest  = orthonormalize(compose(rest:t, rest:r, XYZ) * rest:space)
#           * parentRest
#   default = defaultOffsets * rest * parentRest^-1 * parentDefault
#   posed = avars * posedDefault * parentDefault^-1 * parentPosed
#
# where "parent" is the nearest namespace-ancestor RigExecXformable and the
# whole rig is in ASSET space: the world transform of the RigExecRoot's
# parent prim places it (libs/rigExec/api/rigEvaluator.h:79-84).
#
import math

from pxr import Gf, Tf, Usd, UsdGeom

try:
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import sessionRegistry

from gizmoRotation import _IDENTITY, _AxisRotation, RotationFromEuler

# Attribute names, kept as constants so the targets and the tests agree.
AVAR_T = ("avars:tx", "avars:ty", "avars:tz")

AVAR_R = ("avars:rx", "avars:ry", "avars:rz")

AVAR_S = ("avars:sx", "avars:sy", "avars:sz")

AVAR_RSPIN = "avars:rspin"

AVAR_ORDER = "avars:rotationOrder"

AVAR_ROTATION_SIGN = "avars:rotationSign"

REST_T = ("rest:tx", "rest:ty", "rest:tz")

REST_R = ("rest:rx", "rest:ry", "rest:rz")

REST_SPACE = "rest:space"

POSED_SPACE = "posed:space"

DEFAULT_T = ("default:tx", "default:ty", "default:tz")

DEFAULT_R = ("default:rx", "default:ry", "default:rz")

DEFAULT_SPACE = "default:space"

AVAR_DEFAULT_SPACE = "avars:defaultSpace"

POSED_DEFAULT_SPACE = "posed:defaultSpace"

PARENT_SPACE = "parent:space"

PARENT_DEFAULT_SPACE = "parent:defaultSpace"

AVAR_UNIT_SCALE = "avars:unitScaleFactor"

_COMPUTED_SPACES = (DEFAULT_SPACE, AVAR_DEFAULT_SPACE, POSED_DEFAULT_SPACE,
                    PARENT_SPACE, PARENT_DEFAULT_SPACE)


# The concrete RigExecVolumeWeight subclasses. Used only as the fallback
# for ReadsScaleAvars and IsRigXformable when the schema plugin is not
# registered and Tf cannot answer an IsA question.
VOLUME_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
)


# Avar values and their composition

def NormalizeAvarScale(value):
    """
    The evaluator's floor (libs/rigExecMath/avarScale.h:34-44): finite
    magnitudes below 1e-4 become signed 1e-4, non-finite becomes 1.
    Applied before writing so the stage holds what the viewport shows.

    copysign is deliberately unguarded for zero: the native contract keeps
    the sign of NEGATIVE zero (negative scale is a supported reflection),
    and -0.0 == 0.0 in Python, so a `value != 0.0` guard would silently
    flip a reflected axis back to positive and diverge from the evaluator.
    """
    if not math.isfinite(value):
        return 1.0
    if abs(value) < 1e-4:
        return math.copysign(1e-4, value)
    return value


def ComposeAvarMatrix(tx, ty, tz, sx, sy, sz, rx, ry, rz, rspin, order):
    """Mirror of _ComposeAvars: S * R(order) * Rspin * T."""
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(NormalizeAvarScale(sx), NormalizeAvarScale(sy),
                        NormalizeAvarScale(sz)))
    m = m * RotationFromEuler(order, rx, ry, rz)
    if rspin != 0.0:
        m = m * _AxisRotation(0, rspin)
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(tx, ty, tz))
    return m * t


def RotationSign(prim):
    """
    avars:rotationSign as a 3-tuple of +1.0/-1.0 (RigExecNormalizeRotation-
    Sign: negative selects -1, everything else +1). A mirrored limb
    declares it so the same avar value turns both sides the same way; the
    sign multiplies the avar, so the gizmo reads the EFFECTIVE angle as
    raw * sign and writes raw = effective * sign back (the sign is its own
    inverse). rspin takes the x sign, being a rotation about +X.
    """
    attr = prim.GetAttribute(AVAR_ROTATION_SIGN) if prim else None
    value = attr.Get() if attr and attr.IsValid() else None
    if value is None:
        return (1.0, 1.0, 1.0)
    return tuple(-1.0 if float(v) < 0.0 else 1.0 for v in value)


# Rig frames (asset space)

def IsRigXformable(prim):
    xformable = Tf.Type.FindByName("RigExecXformable")
    if xformable.isUnknown:
        return prim.GetTypeName() in (
            ("RigExecControl", "RigExecJoint") + VOLUME_WEIGHT_TYPE_NAMES)
    return prim.IsA(xformable)


def ReadsScaleAvars(prim):
    """
    Whether the evaluator feeds this prim's avars:sx/sy/sz into its frame.

    False for RigExecVolumeWeight: computations.cpp registers it with
    readScaleAvars = false on purpose, because volume shape is owned
    exclusively by inputs:scaleX/Y/Z and transform-scale avars would be
    silent no-ops. Reading them here anyway would give a volume weight --
    and everything parented under one -- a replica frame the evaluator
    disagrees with, so the gizmo would draw and edit in the wrong place.
    """
    volume = Tf.Type.FindByName("RigExecVolumeWeight")
    if volume.isUnknown:
        return prim.GetTypeName() not in VOLUME_WEIGHT_TYPE_NAMES
    return not prim.IsA(volume)


class _EvalContext(object):
    """
    Everything ONE top-level ComputeRigFrames call reads, read once.

    The frames of a prim are a pure function of (stage, time, preview
    values), and none of the three moves while a single computation is
    running: a drag step installs its preview values (SetPreviewValues)
    and only then asks for frames, and stage edits arrive between steps.
    So a context lives for exactly one invocation and NOTHING survives
    across calls -- a memo that outlived its call would draw the gizmo
    at the previous mouse position.

    Why it exists: RestSpace recurses to the rig root, and was called
    once per ancestor from _ComputeRigFrames and twice more from
    _ComputedSpace, so a joint at depth d paid O(d^2) rest walks -- 312
    ms per mouse move on the biped's depth-20 brow joints, against 5.7
    ms now. The XformCache is shared for the same reason:
    InterveningXform built a fresh one per call, and a fresh cache
    re-walks the namespace for every question asked of it.

    Memoising cannot change the arithmetic: every entry is the result of
    the same operations in the same order, handed back as a copy so each
    caller still owns its matrix, and a computation that RAISED is never
    recorded -- a cyclic connection or a _PoseAuthorityError has to
    raise again the next time it is asked for.

    Keying reads by (path, name) is safe even though a previewed
    attribute stands in for itself and suppresses its connection (see
    ScalarAvar): _previewValues is fixed for the invocation, so a
    previewed and an unpreviewed read of one attribute cannot both
    happen inside one context.
    """

    def __init__(self, time, frames=None):
        self.time = time


        self.frames = {} if frames is None else frames
        self.rigRoots = {}
        self.parents = {}
        self.scalars = {}
        self.matrices = {}
        self.restLocals = {}
        self.defaultLocals = {}
        self.intervening = {}
        self.restSpaces = {}
        self.spaces = {}
        self._xformCache = None

    def XformCache(self):
        """The one UsdGeomXformCache every query in this call shares."""
        if self._xformCache is None:
            self._xformCache = UsdGeom.XformCache(self.time)
        return self._xformCache


# The MemoScopes open on the call stack, innermost last. Not anybody's
# state between calls: a scope is only ever open for the duration of a
# `with` block. Each serves only the stage it was opened for (see MemoScope),
# so a scope one session opened can never answer another session's stage --
# the same prim paths in two stages are exactly what a shared memo would mix
# up.
_openScopes = []


def _AmbientScope(prim):
    """The innermost open MemoScope that serves `prim`'s stage, or None."""
    if not _openScopes:
        return None
    stage = None
    for scope in reversed(_openScopes):
        if scope.stage is None:
            return scope
        if stage is None:
            if prim is None:
                continue
            try:
                stage = prim.GetStage()
            except Exception:
                return None
        if scope.stage == stage:
            return scope
    return None


def _Context(cache, time, prim=None):
    """
    The evaluation context for a call: `cache` if it already is one, a
    throwaway around it otherwise.

    This is what keeps every entry point callable exactly as before --
    with no context, with None, or with the plain dict that _frameCache
    used to be, which then goes on holding the frames.

    `prim` is the prim the call is about; an open MemoScope bound to a
    stage shares its context only with calls about that stage.
    """
    if isinstance(cache, _EvalContext):
        # None of the memos key on time -- a context IS one time code --
        # so a context handed on to a second time would answer from the
        # first one's reads and nothing would say so. Refuse it instead.
        if Usd.TimeCode(cache.time) != Usd.TimeCode(time):
            raise ValueError(
                "gizmo evaluation context built for time %s reused at %s"
                % (cache.time, time))
        return cache
    scope = _AmbientScope(prim)
    if scope is not None:
        shared = scope.Context(time, cache)
        if shared is not None:
            return shared
    return _EvalContext(time, cache)


class MemoScope(object):
    """
    Hold ONE evaluation context open across SEVERAL ComputeRigFrames calls.

    ComputeRigFrames builds a private _EvalContext per top-level call and
    drops it on return, which is right for one control and wasteful for a
    hundred: the marquee (gizmoMarquee.ScreenPositions) projects every
    control on the stage in one pass, and they share ancestors all the
    way to the rig root. Measured on examples/biped/Biped.usda, its 111
    selectable controls cost 0.844 s with a context per call and 0.262 s
    inside one scope.

    Safe only where nothing is authored between the calls -- a context is
    a pure function of (stage, time, preview values) and a projection
    pass changes none of them. A drag, which writes between requests,
    must NOT wrap itself in one; nothing on the drag path does.

    A scope holds one context, made on the first call inside it and
    handed to every later call at the SAME time code that passes no
    context of its own (or the plain dict that context already uses as
    its frame memo). A call at another time, or with another dict, gets
    a throwaway context exactly as it would outside the scope: the scope
    is a convenience and never a reason to answer from the wrong time,
    which is the contract _Context enforces for an explicit context.

    `stage` binds the scope to one stage: only calls about prims of that
    stage share its context. Several usdview sessions can run in one
    process, and a memo keyed by prim path must never answer for another
    session's stage. Without a stage the scope serves any call, as it
    always did.
    """

    def __init__(self, stage=None):
        self.stage = stage
        self._context = None

    def Context(self, time, cache):
        """The shared context for `time`, or None when a call cannot use it."""
        if self._context is None:
            self._context = _EvalContext(time, cache)
            return self._context
        ctx = self._context
        if Usd.TimeCode(ctx.time) != Usd.TimeCode(time):
            return None
        if cache is not None and cache is not ctx.frames:
            return None
        return ctx

    def __enter__(self):
        _openScopes.append(self)
        return self

    def __exit__(self, *args):
        # Removed by identity, not popped: exits always nest, but a scope
        # that is not on top must still never be left behind.
        for index in range(len(_openScopes) - 1, -1, -1):
            if _openScopes[index] is self:
                del _openScopes[index]
                break
        self._context = None
        return False


def _MemoMatrix(memo, key, compute):
    """
    `compute()` once per key, handed back as a matrix the caller owns.

    The copy preserves today's contract that every call returns a fresh
    matrix; `memo` is None when there is no context, and then this is
    just the call.
    """
    if memo is None:
        return compute()
    if key not in memo:
        memo[key] = compute()
    return Gf.Matrix4d(memo[key])


def FindRigRoot(prim, _ctx=None):
    """
    The enclosing RigExecRoot, or None.

    An INVALID prim is refused here with a clear message rather than being
    allowed through to `GetParent()`, which raises "Accessed invalid null
    prim" from inside the context's memo several frames down and says
    nothing about where the bad prim came from. It got here once already:
    a test held a hardcoded control path, the rig moved that control, and
    `GetPrimAtPath` handed back an invalid prim that travelled a long way
    before failing. The lookup that produced it is the bug; this says so.
    """
    if not prim or not prim.IsValid():
        raise ValueError(
            "FindRigRoot got an invalid prim -- the lookup that produced it "
            "failed, most likely a path that no longer exists on this stage")
    memo = None if _ctx is None else _ctx.rigRoots
    key = prim.GetPath()
    if memo is not None and key in memo:
        return memo[key]
    root = None
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if parent.GetTypeName() == "RigExecRoot":
            root = parent
            break
        parent = parent.GetParent()
    if memo is not None:
        memo[key] = root
    return root


def _FindParentXformable(prim, rigRoot, _ctx=None):
    memo = None if _ctx is None else _ctx.parents
    key = (prim.GetPath(), rigRoot.GetPath() if rigRoot else None)
    if memo is not None and key in memo:
        return memo[key]
    found = None
    parent = prim.GetParent()
    while parent and parent != rigRoot and not parent.IsPseudoRoot():
        if IsRigXformable(parent):
            found = parent
            break
        parent = parent.GetParent()
    if memo is not None:
        memo[key] = found
    return found


# The relationships SolverPosedPaths reads and SolverPosedCache watches
# for invalidation. Named once so the two cannot drift apart.
SOLVER_JOINTS_REL = "rigExec:joints"

MOVER_MOVES_REL = "rigExec:moves"


# A space switch names its target with its own relationship, not
# rigExec:moves, so reading the two above cannot see it.
SPACE_TARGET_REL = "rigExec:target"

RIG_WRITTEN_RELS = (SOLVER_JOINTS_REL, MOVER_MOVES_REL, SPACE_TARGET_REL)


# Mover types that OVERWRITE their target's frame rather than composing
# onto it. Only these belong in SolverPosedPaths; see its docstring for
# the measurement that separates them from the rest.
OVERWRITING_MOVER_TYPES = ("RigExecParentConstraint",)


# Types that re-parent their target through SPACE_TARGET_REL.
SPACE_SWITCH_TYPES = ("RigExecSpaceSwitch",)


class _PosedPaths(set):
    """The posed set, plus which of its members a SPACE SWITCH claimed.

    One set answers two different questions and they part company on a
    switched control. "Whose frame must come from the published pose?"
    includes a switch target -- that is what stops a switched IK hand's
    manipulator sitting 25 cm from the hand. "Whose avars are inert, so
    refuse to manipulate it?" does NOT: a switch re-parents its target,
    it does not drive it, and the animator still poses it in the
    selected space.

    A subclass rather than a second return value or a map, because every
    caller tests membership against this and a plain `set` is still
    exactly what they get.
    """

    def __init__(self, *args):
        super(_PosedPaths, self).__init__(*args)
        self.spaceSwitched = set()
        # One record per IK/FK match (ikfkMatch.MatchHalf): the controls it
        # presents on the joints while its switch is on the other half.
        self.matchHalves = []


def SolverPosedPaths(rigRoot):
    """
    Every prim the rig poses OUTRIGHT: all rigExec:joints targets, plus
    the rigExec:moves targets of the movers that overwrite a frame. The
    evaluator injects those frames as value overrides
    (computations.cpp:223-227), so their avars are inert.

    WHY rigExec:moves is here at all, and why only for some movers.
    Reading rigExec:joints alone missed the biped's finger FK roots,
    which are parent-constrained to the wrist
    (/Biped/Rig/Movers/hand_?_follow), and the gizmo drew them from a
    composition that knows nothing about that constraint -- 39.5-42.2 cm
    and 45 degrees away from the hand with the arm posed. But
    rigExec:moves by ITSELF is too blunt in both directions, so the
    filter is on the mover's type:

      * measured, of the 41 controls that plain rigExec:moves would
        have claimed on Biped.usda, only 15 are avar-inert. All 15 are
        RigExecParentConstraint targets;
      * the other 26 are the foot roll rig (heel_?, toe_?, ballRoll_?,
        toeBend_?, the ?_ikTargets and the roll twins), reached through
        RigExecAimConstraint / RotationConstraint / PositionConstraint /
        FloatMathMover. Every one of those honours its own avars --
        avars:tx=5 moves it 5.0000 cm and carries real joints with it --
        and none of them is mis-placed by the composition (measured 0.000
        cm before any fix). Claiming them would have cost the animator
        the whole foot and corrected nothing.

    The separation is total, not a judgement call: 15 of 15 inert
    controls are parent-constraint targets and 0 of 26 live ones are.
    A mover type that turns out to overwrite as well belongs in
    OVERWRITING_MOVER_TYPES, with the measurement that says so.

    AND SPACE SWITCHES, which neither relationship above can reach: a
    RigExecSpaceSwitch names what it re-parents with rigExec:target. A
    switched control is posed from its ACTIVE source, and the
    composition here walks the NAMESPACE parent, so the two part company
    the moment the active space is not the namespace one.

    Measured on Biped_stack.usda with the character posed (M_Body
    tx=25 ty=-8, M_Head ry=35 rx=15, M_Chest rz=20), comparing the
    composed frame against pose.control_frame() for all thirteen
    targets:

        space 0 ..... 0.0000 cm, every one of them
        space 1 ..... 25.06 to 34.20 cm

    Zero at index 0 because that is the composition's own parent, which
    is exactly why this went unseen until an animator switched a space:
    "we also need controls manips to move where they are when dealing
    with spaces. They seem to be getting left behind."

    These targets are NOT avar-inert -- an animator still translates a
    switched IK hand, and must -- but that is not what the set means to
    _PreferPublishedFrame, which keeps posed == avars * P by deriving
    P = avars^-1 * published. Inertness is what made the foot roll
    pointless to claim (0.000 cm wrong either way); being mis-placed is
    what makes these worth claiming.
    """
    paths = _PosedPaths()
    for prim in Usd.PrimRange(rigRoot):
        rel = prim.GetRelationship(SOLVER_JOINTS_REL)
        if rel:
            paths.update(rel.GetTargets())
        rel = prim.GetRelationship(MOVER_MOVES_REL)
        if rel and prim.GetTypeName() in OVERWRITING_MOVER_TYPES:
            paths.update(rel.GetTargets())
        rel = prim.GetRelationship(SPACE_TARGET_REL)
        if rel and prim.GetTypeName() in SPACE_SWITCH_TYPES:
            switched = set(rel.GetTargets())
            paths.update(switched)
            paths.spaceSwitched.update(switched)
        if prim.GetTypeName() == "RigExecIkFkMatchConstraint":
            import ikfkMatch
            half = ikfkMatch.MatchHalf.From(prim)
            if half is not None:
                paths.matchHalves.append(half)
    return paths


def PresentedBy(stage, path, time, solverPosed):
    """The IK/FK match presenting `path` at `time` (ikfkMatch.MatchHalf), or
    None: the halves come with the memoised posed set."""
    import ikfkMatch
    return ikfkMatch.PresentedBy(stage, path, time,
                                 getattr(solverPosed, "matchHalves", ()))


class SolverPosedCache(object):
    """
    SolverPosedPaths memoised per rig root.

    Every Refresh() of a rig target re-derives its frames, and that walks
    the WHOLE rig looking for rigExec:joints and rigExec:moves. Outside
    a drag the controller refreshes on every relevant stage notice, so
    without this an avar change costs a full traversal for an answer that
    cannot have moved.

    The set changes in two notice shapes, not one, and missing either
    leaves the memo handing out an editable target for a control a
    solver has taken over. Probed on this USD build:

      SetTargets on a NEW relationship     resync   /Rig/Ik.rigExec:joints
      SetTargets on an existing one        INFO     /Rig/Ik.rigExec:joints
      ClearTargets                         resync   /Rig/Ik.rigExec:joints
      a new prim carrying the relationship resync   /Rig/Other

    So retargeting -- the common case, and what a panel or a script
    does -- is info-only. InvalidateChanged covers it; InvalidateResynced
    covers the rest.

    Invalidation is the caller's job -- both methods on a notice, Clear
    when the stage is replaced -- because only the caller sees notices.
    """

    def __init__(self):
        self._byRoot = {}

    def For(self, rigRoot):
        """The memoised SolverPosedPaths(rigRoot); empty without a root."""
        if not rigRoot:
            return set()
        path = rigRoot.GetPath()
        paths = self._byRoot.get(path)
        if paths is None:
            paths = SolverPosedPaths(rigRoot)
            self._byRoot[path] = paths
        return paths

    def Clear(self):
        self._byRoot = {}

    def InvalidateResynced(self, resyncedPaths):
        """
        Drop every rig root a resynced path could have changed: one
        INSIDE the rig (a joints relationship was authored, a prim added
        or removed) and one AT or ABOVE it (the rig itself was replaced
        or its composition changed).
        """
        for root in list(self._byRoot):
            for path in resyncedPaths:
                prim = path.GetPrimPath()
                if prim.HasPrefix(root) or root.HasPrefix(prim):
                    del self._byRoot[root]
                    break

    def InvalidateChanged(self, changedPaths):
        """
        Drop every rig root whose solver joints or mover targets were
        RE-TARGETED.

        SetTargets on a relationship that already has targets is an
        info-only change (see the class docstring's probe table), so a
        memo invalidated on resync alone would keep answering with the
        joints from before the edit. The names matched here are the ones
        SolverPosedPaths reads, so the two cannot drift apart.
        """
        for root in list(self._byRoot):
            for path in changedPaths:
                if (path.IsPropertyPath()
                        and path.name in RIG_WRITTEN_RELS
                        and path.GetPrimPath().HasPrefix(root)):
                    del self._byRoot[root]
                    break


def NoticeAffectsTarget(resyncedPaths, changedPaths, targetPath,
                        rigRootPath=None):
    """
    Whether an ObjectsChanged notice can have moved the gizmo's target.

    True when any resynced or changed-info path IS the target prim or
    one of its ancestors (an ancestor's transform carries it), and -- for
    a rig target -- when any of them lies under `rigRootPath`, because
    the evaluator composes a control's frame from every avar above it
    and a solver can pose it from a joint anywhere else in the rig.

    A property path answers for the prim that owns it: an avar edit
    arrives as </Rig/Arm.avars:tx>, and what moved is /Rig/Arm.

    Without the filter every notice on the stage -- a volume weight edit
    or a timeline scrub -- costs a full rig walk,
    a resolveCamera() (which is NOT side-effect free, see gizmoUI._Camera)
    and a reprojection of the whole manipulator, for a target that
    usually did not change. A target-less controller is always affected:
    the notice may be the very edit that gives the focus prim a target.
    """
    if targetPath is None:
        return True
    for paths in (resyncedPaths, changedPaths):
        for path in paths:
            prim = path.GetPrimPath()
            if targetPath.HasPrefix(prim):
                return True
            if rigRootPath is not None and prim.HasPrefix(rigRootPath):
                return True
    return False


# Uncommitted manipulation values, per STAGE, by attribute path:
# {stage: {Sdf.Path: value}}.
#
# A drag in progress is not authored (see Writer and the design note at
# docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md), so
# the stage still holds the pre-drag value for as long as the mouse is down.
# Every read below consults this first, for one reason: the gizmo has to draw
# its handles where it is dragging them. Hydra is given the same values through
# the evaluator, and both sides reading the same place is what keeps the
# manipulator on the geometry it is moving.
#
# Module-level because the frame maths is a tree of free functions that no
# drag object is threaded through -- but keyed by the stage the values belong
# to, because several usdview sessions can share this module in one process
# and the same prim path in two of their stages is two different attributes.
# A read only ever sees its own stage's values. The None key holds values
# installed with no stage (SetPreviewValues(values)), which stand in for
# every stage.
_previewValues = {}


def _PreviewBucket(stage):
    """The {path: value} map for `stage`, created on first use."""
    bucket = _previewValues.get(stage)
    if bucket is None:
        bucket = {}
        _previewValues[stage] = bucket
    return bucket


def _DropFromPreview(stage, paths):
    bucket = _previewValues.get(stage)
    if bucket is None:
        return
    for path in paths:
        bucket.pop(path, None)
    if not bucket:
        del _previewValues[stage]


def SetPreviewValues(values, stage=None):
    """
    Install `stage`'s uncommitted values; {} drops them.

    Without a stage, this is the old whole-process call: {} drops every
    preview, and a non-empty map replaces every preview with values that
    stand in for any stage.
    """
    if stage is None:
        _previewValues.clear()
        if values:
            _previewValues[None] = dict(values)
        return
    if values:
        _previewValues[stage] = dict(values)
    else:
        _previewValues.pop(stage, None)


def SetPreviewValue(path, value, stage=None):
    """Add one uncommitted value for `stage` (None: for every stage)."""
    _PreviewBucket(stage)[path] = value


def PreviewValues(stage=None):
    """
    `stage`'s uncommitted values (plus any installed with no stage), or,
    without a stage, every uncommitted value in the process.
    """
    if stage is None:
        merged = {}
        for bucket in _previewValues.values():
            merged.update(bucket)
        return merged
    merged = dict(_previewValues.get(None, {}))
    merged.update(_previewValues.get(stage, {}))
    return merged


def _Previewed(attr):
    """The uncommitted value for `attr` in ITS stage, or None."""
    if not _previewValues or not attr:
        return None
    path = attr.GetPath()
    stage = None
    for key, bucket in _previewValues.items():
        if path not in bucket:
            continue
        if key is None:
            return bucket[path]
        if stage is None:
            stage = attr.GetStage()
        if key == stage:
            return bucket[path]
    return None


def _ScalarAvarValue(prim, name, time):
    """
    ScalarAvar's stage read, WITHOUT the fallback: the float the
    attribute (or the end of its connection chain) resolves to, or None
    when nothing resolves.

    Split out so the memo can key on (path, name) alone. The fallback is
    the caller's, not the attribute's -- avars:sx falls back to 1.0 and
    rest:tx to 0.0 -- so folding it in would either key on it too or
    hand one caller another's default.
    """
    attr = prim.GetAttribute(name)
    visiting = set()
    while attr and attr.GetPath() not in visiting:
        visiting.add(attr.GetPath())
        # A previewed value stands in for the attribute ITSELF, so the
        # connection is not followed past it: a dragged channel is the value,
        # and its upstream author is what it is standing in for.
        previewed = _Previewed(attr)
        if previewed is not None:
            return float(previewed)
        connections = attr.GetConnections()
        if len(connections) != 1:
            break
        source = prim.GetStage().GetAttributeAtPath(connections[0])
        if not source:
            break
        attr = source
    if attr:
        previewed = _Previewed(attr)
        if previewed is not None:
            return float(previewed)
        value = attr.Get(time)
        if value is not None:
            return float(value)
    return None


def ScalarAvar(prim, name, time, fallback, _ctx=None):
    memo = None if _ctx is None else _ctx.scalars
    key = (prim.GetPath(), name)
    if memo is None:
        value = _ScalarAvarValue(prim, name, time)
    elif key in memo:
        value = memo[key]
    else:
        value = _ScalarAvarValue(prim, name, time)
        memo[key] = value
    return fallback if value is None else value


def _MatrixAttr(prim, name, time, _ctx=None):
    def Read():
        attr = prim.GetAttribute(name)
        if attr:
            value = _Previewed(attr)
            if value is None:
                value = attr.Get(time)
            if value is not None:
                return Gf.Matrix4d(value)
        return Gf.Matrix4d(1.0)
    memo = None if _ctx is None else _ctx.matrices
    return _MemoMatrix(memo, (prim.GetPath(), name), Read)


def RestLocal(prim, time, _ctx=None):
    """compose(rest:t, rest:r) -- XYZ order, no scale, no spin."""
    def Compose():
        return ComposeAvarMatrix(
            ScalarAvar(prim, REST_T[0], time, 0.0, _ctx),
            ScalarAvar(prim, REST_T[1], time, 0.0, _ctx),
            ScalarAvar(prim, REST_T[2], time, 0.0, _ctx),
            1.0, 1.0, 1.0,
            ScalarAvar(prim, REST_R[0], time, 0.0, _ctx),
            ScalarAvar(prim, REST_R[1], time, 0.0, _ctx),
            ScalarAvar(prim, REST_R[2], time, 0.0, _ctx),
            0.0, "XYZ")
    memo = None if _ctx is None else _ctx.restLocals
    return _MemoMatrix(memo, prim.GetPath(), Compose)


def InterveningXform(prim, time, rigRoot=None, _ctx=None):
    """
    The transform of any plain Xformable lying between `prim` and the
    frame provider above it -- the quantity the evaluator folds into its
    frames (RigExecRigEvaluator::_ComposeInterveningXforms).

    _FindParentXformable walks past a `Scope` and past an `Xform` alike,
    because neither is a RigExec provider. Skipping the Scope is right --
    it has no transform -- and dropping the Xform is the defect this
    answers. Without it the manipulator would draw and drag a joint at
    the place it sat before its Xform was applied, while the pose put it
    somewhere else.

    Computed through UsdGeomXformCache relative to the same anchor the
    evaluator uses, rather than by multiplying local matrices here: the
    two agreeing is the whole point, and a second hand-rolled walk is how
    they would stop agreeing.

    Identity across a `!resetXformStack!`, which detaches the provider
    from the anchor entirely -- again matching the evaluator, which
    reports that case rather than guessing at it.
    """
    _ctx = _Context(_ctx, time, prim)
    rigRoot = FindRigRoot(prim, _ctx) if rigRoot is None else rigRoot
    if rigRoot is None:
        return Gf.Matrix4d(1.0)

    def Compute():
        anchor = _FindParentXformable(prim, rigRoot, _ctx)
        if anchor is None:
            # No RigExec ancestor: the chain is anchored at the asset root,
            # which is the rig root's parent.
            anchor = rigRoot.GetParent()
        parent = prim.GetParent()
        if not anchor or not parent or parent == anchor:
            return Gf.Matrix4d(1.0)
        matrix, resets = _ctx.XformCache().ComputeRelativeTransform(
            parent, anchor)
        return Gf.Matrix4d(1.0) if resets else matrix

    # The rig root is part of the key: it is an argument, and a caller
    # that passes one other than FindRigRoot's answer is asking about a
    # different anchor.
    return _MemoMatrix(_ctx.intervening,
                       (prim.GetPath(), rigRoot.GetPath()), Compute)


def RestSpace(prim, time, _ctx=None):
    """
    Mirror of _JointRestSpace: the local rest carried into the parent's
    rest frame, orthonormalize(restLocal * rest:space) * X * parentRest.

    X is InterveningXform, and it sits AFTER the orthonormalize for the
    same reason the evaluator does not orthonormalize it either: it is
    the author's own transform, and scale they put there is theirs to
    keep. Orthonormalizing the local factor first is what keeps an
    invalid ancestor's NaN from being scrubbed by the multiply.
    """
    _ctx = _Context(_ctx, time, prim)

    def Compute():
        rigRoot = FindRigRoot(prim, _ctx)
        rest = RestLocal(prim, time, _ctx) * _MatrixAttr(
            prim, REST_SPACE, time, _ctx)
        rest = rest.GetOrthonormalized(False) * InterveningXform(
            prim, time, rigRoot, _ctx)
        parent = _FindParentXformable(prim, rigRoot, _ctx)
        return rest * RestSpace(parent, time, _ctx) if parent else rest

    # The memo that turns the module from quadratic to linear: the
    # recursion above walks to the rig root, and every ancestor's frames
    # ask for the same walk again.
    return _MemoMatrix(_ctx.restSpaces, prim.GetPath(), Compute)


def DefaultLocal(prim, time, _ctx=None):
    def Compose():
        return ComposeAvarMatrix(
            *(tuple(ScalarAvar(prim, n, time, 0.0, _ctx) for n in DEFAULT_T)
              + (1.0, 1.0, 1.0)
              + tuple(ScalarAvar(prim, n, time, 0.0, _ctx) for n in DEFAULT_R)
              + (0.0, "XYZ")))
    memo = None if _ctx is None else _ctx.defaultLocals
    return _MemoMatrix(memo, prim.GetPath(), Compose)


class _PoseAuthorityError(ValueError):
    """
    A parent:space that cannot be resolved because the POSE authority
    upstream is not the avars -- a solver-posed ancestor, or one with an
    authored or connected posed:space.

    Distinguished from every other ValueError raised here (a cyclic
    space connection, a singular matrix) because it says nothing about
    the rest side: rest:t/r upstream is still authored data, still read,
    and a rest frame reads no namespace ancestor at all. It refuses Pose
    only. See _ComputeRigFrames.
    """


def _ComputedSpace(prim, name, time, solverPosed, visiting=None,
                   frameCache=None):
    """
    Evaluate the schema space expressions, including connected identity.

    Memoised per (prim, name) for the invocation. The memo is consulted
    BEFORE the cycle guard, and that cannot hide a cycle: a key only
    enters `visiting` after the memo has missed, and only enters the
    memo once its computation has RETURNED, so no key is ever in both.
    """
    ctx = _Context(frameCache, time, prim)
    key = (prim.GetPath(), name)
    if key in ctx.spaces:
        return Gf.Matrix4d(ctx.spaces[key])
    visiting = set() if visiting is None else visiting
    path = prim.GetPath().AppendProperty(name)
    if path in visiting:
        raise ValueError("cyclic space connection at %s" % path)
    visiting.add(path)
    try:
        space = _ComputeSpace(prim, name, time, solverPosed, visiting, ctx)
    finally:
        visiting.remove(path)
    # Recorded only on the way out, so a cyclic connection and a
    # _PoseAuthorityError raise again for every caller that asks.
    ctx.spaces[key] = space
    return Gf.Matrix4d(space)


def _ComputeSpace(prim, name, time, solverPosed, visiting, ctx):
    """_ComputedSpace's body, inside its cycle guard."""
    attr = prim.GetAttribute(name)
    connections = attr.GetConnections() if attr else []
    if len(connections) == 1:
        source = prim.GetStage().GetAttributeAtPath(connections[0])
        if source:
            return _ComputedSpace(source.GetPrim(), source.GetName(),
                                  time, solverPosed, visiting, ctx)
    raw = _MatrixAttr(prim, name, time, ctx)
    if name not in _COMPUTED_SPACES or not IsRigXformable(prim):
        return raw
    if raw != _IDENTITY:
        return raw
    if name == AVAR_DEFAULT_SPACE:
        return _ComputedSpace(prim, DEFAULT_SPACE, time, solverPosed,
                              visiting, ctx)
    if name == POSED_DEFAULT_SPACE:
        return _ComputedSpace(prim, AVAR_DEFAULT_SPACE, time, solverPosed,
                              visiting, ctx)
    parent = _FindParentXformable(prim, FindRigRoot(prim, ctx), ctx)
    if name == PARENT_DEFAULT_SPACE:
        return (_ComputedSpace(parent, DEFAULT_SPACE, time, solverPosed,
                               visiting, ctx)
                if parent else Gf.Matrix4d(1.0))
    if name == PARENT_SPACE:
        if not parent:
            return Gf.Matrix4d(1.0)
        parentFrames = ComputeRigFrames(prim.GetStage(), parent, time,
                                        solverPosed, ctx)
        if parentFrames.reason and not parentFrames.published:
            # Published means the frame IS resolved -- the evaluator
            # handed it over -- so there is no authority to refuse.
            # See the matching guard in _ComputeRigFrames.
            raise _PoseAuthorityError("space source %s: %s" % (
                parent.GetPath(), parentFrames.reason))
        return parentFrames.posed
    parentRest = RestSpace(parent, time, ctx) if parent else Gf.Matrix4d(1.0)
    return (DefaultLocal(prim, time, ctx) * RestSpace(prim, time, ctx)
            * parentRest.GetInverse()
            * _ComputedSpace(prim, PARENT_DEFAULT_SPACE, time, solverPosed,
                             visiting, ctx))


def AvarsMatrix(prim, time, _ctx=None):
    order = prim.GetAttribute(AVAR_ORDER)
    orderValue = order.Get(time) if order else None
    # Mirrors _ComputeXformablePointFrame's readScaleAvars argument: a
    # volume weight substitutes identity scale rather than reading avars.
    scaled = ReadsScaleAvars(prim)
    units = ScalarAvar(prim, AVAR_UNIT_SCALE, time, 1.0, _ctx)
    sign = RotationSign(prim)
    return ComposeAvarMatrix(
        ScalarAvar(prim, AVAR_T[0], time, 0.0, _ctx) * units,
        ScalarAvar(prim, AVAR_T[1], time, 0.0, _ctx) * units,
        ScalarAvar(prim, AVAR_T[2], time, 0.0, _ctx) * units,
        ScalarAvar(prim, AVAR_S[0], time, 1.0, _ctx) if scaled else 1.0,
        ScalarAvar(prim, AVAR_S[1], time, 1.0, _ctx) if scaled else 1.0,
        ScalarAvar(prim, AVAR_S[2], time, 1.0, _ctx) if scaled else 1.0,
        ScalarAvar(prim, AVAR_R[0], time, 0.0, _ctx) * sign[0],
        ScalarAvar(prim, AVAR_R[1], time, 0.0, _ctx) * sign[1],
        ScalarAvar(prim, AVAR_R[2], time, 0.0, _ctx) * sign[2],
        ScalarAvar(prim, AVAR_RSPIN, time, 0.0, _ctx) * sign[0],
        orderValue or "XYZ")


class RigFrames(object):
    """
    Everything the gizmo needs about one RigExecXformable at one time,
    in ASSET space (multiply by assetToWorld for world):

      rest       orthonormal rest frame (the evaluator's computeRestFrame)
      posed      avars * P   (the evaluator's computePointFrame)
      P          posedDefault * parentDefault^-1 * parentPosed: the frame the avars
                 are expressed in -- Pose mode edits happen relative to it
      Qrest      orthonormalize(rest:space): the frame the rest offsets
                 are ACTUALLY expressed in, and what Pivot mode edits
                 relative to. computeRestFrame composes rest:t/rest:r
                 against rest:space and reads no namespace ancestor
                 (computations.cpp:200-219), so a rest frame is absolute
                 in asset space and owes nothing to the parent's pose.
                 restLocal * Qrest is therefore `rest` itself, and the
                 pivot manipulator sits on the frame its own channels
                 define -- unmoved by an animated ancestor, and unmoved
                 by a solver posing this joint.
      Q          orthonormalize(rest:space) * parentRest^-1 * parentPosed:
                 the frame the rest offsets are carried into for POSE
                 purposes. Pivot mode does NOT use it. Rest editing acts
                 in the bind frame, before local default offsets: with
                 identity default channels, restLocal * Q reproduces P,
                 while a default offset moves the avar origin and leaves
                 the rest pivot in the bind frame. Raw rest:space is
                 orthonormalized here, matching the evaluator's rigid
                 rest-frame convention -- an approximation only for a
                 rest:space carrying scale or shear, which the evaluator
                 throws away too.
      restLocal  compose(rest:t, rest:r)
      default    computed default:space, including connected overrides
      unitScale  distance per translation avar unit; inverted on drag writes
      published  True when `posed` came from the rig's evaluated snapshot
                 rather than from composing avars onto P. The frame is
                 then authoritative, so a CHILD of this prim must not
                 inherit its pose reason -- see _PreferPublishedFrame.
      reason     "" when the prim is editable through its avars, else why
                 not (solver-posed, posed:space authority, no rig root)
      pivotReason
                 "" when the prim is editable through its REST offsets,
                 else why not. Much narrower than `reason`: everything
                 that only redirects the POSE authority away from the
                 avars -- a solver, an authored or connected posed:space,
                 an ancestor with either -- leaves rest:t/r authored,
                 read and honoured, so it is no reason to refuse a pivot.
                 Only a prim outside a RigExecRoot is refused here, and
                 for a hard reason: the frames are never computed, so
                 there is nothing to draw on.
    """

    def __init__(self, prim):
        self.prim = prim
        self.rigRoot = None
        self.reason = ""
        self.pivotReason = ""
        self.published = False
        self.rest = Gf.Matrix4d(1.0)
        self.posed = Gf.Matrix4d(1.0)
        self.P = Gf.Matrix4d(1.0)
        self.translationP = None
        self.translationEnabled = True
        self.Q = Gf.Matrix4d(1.0)
        self.Qrest = Gf.Matrix4d(1.0)
        self.restLocal = Gf.Matrix4d(1.0)
        self.parentRest = Gf.Matrix4d(1.0)
        self.parentPosed = Gf.Matrix4d(1.0)
        self.parentDefault = Gf.Matrix4d(1.0)
        self.default = Gf.Matrix4d(1.0)
        self.unitScale = 1.0
        self.assetToWorld = Gf.Matrix4d(1.0)


def _RefuseBothModes(frames, message):
    """
    Record a cause that refuses Pose AND Pivot.

    Pivot normally answers only to `pivotReason`, because a solver or a
    posed:space redirects the POSE authority and leaves rest:t/r live.
    That reasoning needs frames to exist: every site that gives up before
    building Qrest/restLocal, and every prim whose data is broken rather
    than merely solver-driven, refuses both modes through here.
    """
    frames.reason = message
    frames.pivotReason = message
    return frames


# The hosts' snapshot readers, one per usdview session: (stage, path, time)
# -> matrix/None. Each reader answers for its own session's stage only and
# None for any other, so a read asks them all and takes the answer; with the
# same prim path in two sessions' stages, only the reader whose stage it is
# answers. Filed under the session's main window, and dropped with it.
_publishedControlFrameReaders = sessionRegistry.SessionRegistry(
    "gizmoMath published control frame readers")


# A reader installed with no session: what a headless caller
# (tests/python/test_gizmo_math.py) makes.
_keylessControlFrameReader = None


def SetPublishedControlFrameReader(reader, session=None):
    """
    Install `session`'s snapshot reader: (stage, path, time) -> matrix/None.

    `session` is the usdview api (or its main window) the reader belongs
    to; None removes it. Without a session, installs the one keyless
    reader, as this always did.
    """
    global _keylessControlFrameReader
    if session is None:
        _keylessControlFrameReader = reader
        return
    if reader is None:
        _publishedControlFrameReaders.Pop(session)
    else:
        _publishedControlFrameReaders.Set(session, reader)


def _HasPublishedControlFrameReader():
    return (_keylessControlFrameReader is not None or
            len(_publishedControlFrameReaders) > 0)


def _ReadPublishedControlFrame(stage, path, time):
    """The published frame of `path` in `stage`, from whichever session's
    reader owns that stage; None when none does."""
    for reader in _publishedControlFrameReaders.Values():
        matrix = reader(stage, path, time)
        if matrix is not None:
            return matrix
    reader = _keylessControlFrameReader
    if reader is not None:
        return reader(stage, path, time)
    return None


def ComputeRigFrames(stage, prim, time, solverPosed=None, _frameCache=None):
    """
    The frames of one RigExecXformable, in asset space.

    `_frameCache` carries the recursion's shared state. A top-level call
    passes nothing and gets a private _EvalContext for the duration --
    one invocation, one context, nothing kept afterwards, because the
    preview values and the stage both move between drag steps. A plain
    dict is still accepted and is still used as the frame memo, so the
    older contract holds for anything that passes one.
    """
    ctx = _Context(_frameCache, time, prim)
    memo = ctx.frames
    if prim.GetPath() in memo:
        cached = memo[prim.GetPath()]
        if cached is None:
            raise ValueError("cyclic posed-space dependency at %s" % prim.GetPath())
        return cached
    memo[prim.GetPath()] = None
    try:
        frames = _ComputeRigFrames(stage, prim, time, solverPosed, ctx)
        memo[prim.GetPath()] = frames
        return frames
    except ValueError:
        del memo[prim.GetPath()]
        raise


def _PreferPublishedFrame(stage, prim, time, solverPosed, frames, ctx):
    """
    Replace a rig-written prim's COMPOSED pose with the evaluated one.

    Everything above this composes `posed` from authored USD alone --
    rest:space, the default:space family, and the namespace parent's
    posed frame. That reproduces the evaluator exactly for a prim whose
    pose IS its avars, which is nearly every control, and not at all for
    one the rig writes: a solver or a mover hands that prim a frame this
    module has no way to recompute, and the composition silently answers
    with the REST frame instead.

    Measured on examples/biped/Biped.usda with the left arm's FK
    shoulder at avars:rz=45, the finger FK roots (parent-constrained to
    wrist_?_bind) drew 39.5-42.2 cm and 45 degrees away from where the
    rig had actually put them, and their namespace descendants, which
    compose off that wrong parent, 44.5-51.6 cm away; on
    examples/biped/Biped_anim.usda at frame 12, 33.0-39.2 cm, the gizmo
    sitting ~35 cm BELOW the hand it belonged to while the control
    circles -- drawn by Hydra from the evaluated frame -- were correct.
    Every control the rig does not write measured 0.0000 cm both before
    and after, which is why this went unnoticed for so long: for them the
    two answers agree exactly. With this in place the worst residual
    against pose.control_frame() over all 116 controls is 1.1e-12 cm.

    The published frame is the host's snapshot of the evaluated control
    (RigExecImaging_GetControlFrameAssetSpace, installed by
    rigExecUsdview._ReadPublishedControlFrame), in asset space -- the
    same space `posed` is in, so it substitutes directly. P is recovered
    as avars^-1 * posed, preserving posed == avars * P so the channel
    frames, gimbal frames, and drag maths stay consistent.

    Gated on `solverPosed` -- the prims the rig writes outright -- rather
    than asked for every prim. Three reasons, in order of weight. It is
    exactly the set whose composed answer is known to be wrong: measured
    on Biped.usda with the arm posed, 22 of 116 controls were mis-placed
    and every one of them was a RigExecParentConstraint target or a
    namespace descendant of one, so the other 94 have nothing to gain.
    The set is already memoised and invalidated (SolverPosedCache). And a
    composed frame reads the uncommitted drag values (_previewValues)
    synchronously, whereas the published frame arrives through the host's
    preview channel, so keeping composition wherever it is already right
    keeps a drag off that round trip.

    The reader returns None for anything it has no published frame for --
    a joint, a mover-written prim that is not a control, a frame where
    RigExec is not active -- and a headless or un-activated session has
    no reader at all; every one of those falls back to the composition,
    which is the behaviour that was correct for them all along.
    """
    if solverPosed is None or not _HasPublishedControlFrameReader():
        return frames
    if prim.GetPath() not in solverPosed and \
            PresentedBy(stage, prim.GetPath(), time, solverPosed) is None:
        return frames
    published = _ReadPublishedControlFrame(stage, prim.GetPath(), time)
    if published is None:
        return frames
    published = Gf.Matrix4d(published)
    if not all(math.isfinite(published[r][c])
               for r in range(4) for c in range(4)):
        return frames
    avars = AvarsMatrix(prim, time, ctx)
    if abs(avars.GetDeterminant()) < 1e-12:
        # P = avars^-1 * posed is the only route back to a consistent
        # pair, so a singular avar matrix keeps the composed frames
        # rather than handing out a P that does not reproduce posed.
        return frames
    frames.posed = published
    frames.P = avars.GetInverse() * published
    frames.published = True
    return frames


def _ApplyChannelSpaces(stage, prim, time, frames):
    rel = prim.GetRelationship("rigExec:channelSpaces")
    paths = rel.GetTargets() if rel else []
    if not paths:
        return
    frames.reason = "channel spaces require valid published control frames"
    frames.pivotReason = "channel-space controls do not support pivot editing"
    if (len(paths) != 2 or prim.GetTypeName() != 'RigExecControl'
            or not math.isfinite(frames.unitScale) or abs(frames.unitScale) < 1e-12):
        return
    if any(prim.GetAttribute(n).HasAuthoredConnections() for n in AVAR_T + AVAR_R + AVAR_S):
        return
    matrices = []
    for path in [prim.GetPath()] + paths:
        source = stage.GetPrimAtPath(path)
        if not source or source.GetTypeName() != 'RigExecControl' or FindRigRoot(source) != frames.rigRoot:
            return
        value = _ReadPublishedControlFrame(stage, path, time)
        if value is None:
            return
        matrix = Gf.Matrix4d(value)
        if not all(math.isfinite(matrix[r][c]) for r in range(4) for c in range(4)) or abs(matrix.GetDeterminant()) < 1e-12:
            return
        matrices.append(matrix)
    frames.posed, frames.P, frames.translationP = matrices
    enabled = prim.GetAttribute('rigExec:translationEnabled')
    frames.translationEnabled = not enabled or enabled.Get(time) is not False
    frames.published = True
    frames.reason = ""


def _ComputeRigFrames(stage, prim, time, solverPosed, ctx):
    frames = RigFrames(prim)
    frames.rigRoot = FindRigRoot(prim, ctx)
    if frames.rigRoot is None:
        # Returns before any frame is computed, so rest/Qrest/assetToWorld
        # stay identity and a pivot built from them would draw at the
        # world origin.
        return _RefuseBothModes(
            frames, "%s is not under a RigExecRoot" % prim.GetName())
    if solverPosed is None:
        solverPosed = SolverPosedPaths(frames.rigRoot)

    parent = _FindParentXformable(prim, frames.rigRoot, ctx)
    if parent is not None:
        parentFrames = ComputeRigFrames(stage, parent, time, solverPosed, ctx)
        parentSpace = prim.GetAttribute(PARENT_SPACE)
        explicitParent = parentSpace and (parentSpace.HasAuthoredConnections()
                                         or _MatrixAttr(prim, PARENT_SPACE,
                                                        time, ctx)
                                         != _IDENTITY)
        # `published` breaks the inheritance, and has to. A pose reason
        # propagates because an ancestor whose pose is not its avars
        # leaves THIS prim composed off a stale parent frame -- but a
        # published parent frame is not stale, it is the evaluator's own
        # answer, so nothing downstream of it is in doubt.
        if (parentFrames.reason and not explicitParent
                and not parentFrames.published):
            frames.reason = "parent %s: %s" % (
                parent.GetName(), parentFrames.reason)
        frames.parentRest = parentFrames.rest
        frames.parentPosed = parentFrames.posed

    if (prim.GetPath() in solverPosed
            and prim.GetPath() not in getattr(solverPosed, "spaceSwitched",
                                              ())):
        # A SPACE SWITCH target is in the posed set and is NOT refused: the
        # switch re-parents the control, it does not pose it, and its avars
        # still apply in the selected space. Refusing them took the
        # manipulator off all thirteen switched controls on the biped --
        # both IK hands, both pole vectors, both legs, the look, the neck,
        # the skull and both upper arms.
        #
        # One message for the two authorities that DO pose outright, whose
        # artist-facing fact is the same either way: something in the rig
        # writes this frame, so the avars here are not what moves it.
        frames.reason = ("%s is posed by a solver or an overwriting mover "
                         "(rigExec:joints / rigExec:moves); its avars "
                         "are ignored" % prim.GetName())
    posed = prim.GetAttribute(POSED_SPACE)
    if posed:
        if posed.HasAuthoredConnections():
            frames.reason = ("%s has a connected posed:space; its avars "
                             "are ignored" % prim.GetName())
        elif _MatrixAttr(prim, POSED_SPACE, time, ctx) != _IDENTITY:
            frames.reason = ("%s has an authored posed:space; its avars "
                             "are ignored" % prim.GetName())

    frames.restLocal = RestLocal(prim, time, ctx)
    frames.rest = RestSpace(prim, time, ctx)
    # Built here, before anything on the pose side can fail: Pivot needs
    # Qrest and restLocal and nothing else (RigPivotTarget._Qw), so a
    # pose-side give-up must not be allowed to return past this.
    frames.Qrest = _MatrixAttr(prim, REST_SPACE, time, ctx)\
        .GetOrthonormalized(False)
    try:
        frames.default = _ComputedSpace(prim, DEFAULT_SPACE, time,
                                        solverPosed, frameCache=ctx)
        frames.parentDefault = _ComputedSpace(prim, PARENT_DEFAULT_SPACE, time,
                                              solverPosed, frameCache=ctx)
        effectiveDefault = _ComputedSpace(prim, POSED_DEFAULT_SPACE, time,
                                          solverPosed, frameCache=ctx)
    except ValueError as error:
        # The default-space family. Pivot answers to these too -- the
        # loop below refuses a pivot on an authored default space -- so a
        # broken one refuses both modes.
        return _RefuseBothModes(frames, str(error))
    try:
        frames.parentPosed = _ComputedSpace(prim, PARENT_SPACE, time,
                                            solverPosed, frameCache=ctx)
    except _PoseAuthorityError as error:
        # An ancestor whose pose comes from somewhere other than its
        # avars: a solver, or an authored posed:space. That is a POSE
        # reason. The rest frames are already built above, and a rest
        # frame reads no namespace ancestor (computations.cpp:200-219),
        # so the pivot below such an ancestor is untouched by it -- which
        # is the case that matters, since a TwoBoneIk MEASURES its bone
        # lengths from these rests. Falls through with P/Q/posed built
        # from the parent's stale posed frame; Pose is refused, and
        # nothing else reads them.
        frames.reason = frames.reason or str(error)
    except ValueError as error:
        return _RefuseBothModes(frames, str(error))
    frames.unitScale = ScalarAvar(prim, AVAR_UNIT_SCALE, time, 1.0, ctx)
    if not math.isfinite(frames.unitScale) or abs(frames.unitScale) < 1e-12:
        # Only RigPoseTarget divides by it, but a prim carrying a broken
        # unit scale is not one to author against in either mode.
        _RefuseBothModes(
            frames,
            "%s has a zero or non-finite translation unit scale" % prim.GetName())
    toParent = frames.parentDefault.GetInverse() * frames.parentPosed
    frames.P = effectiveDefault * toParent
    # Rest editing still acts in the bind frame before local default
    # offsets. No parentRest^-1 here: rest:t/r are parent-relative now, so
    # restLocal is already in the parent's frame and dividing it out again
    # would land the pivot an ancestor offset away from the joint.
    # Qrest reads no namespace ancestor, so the intervening Xform has to
    # be inserted here rather than arriving through it -- the pivot draws
    # in the bind basis, and the bind basis moved with the Xform.
    frames.Q = (frames.Qrest
                * InterveningXform(prim, time, frames.rigRoot, ctx)
                * frames.parentDefault * toParent)
    for name in (DEFAULT_SPACE, AVAR_DEFAULT_SPACE, POSED_DEFAULT_SPACE):
        attr = prim.GetAttribute(name)
        if attr and (attr.HasAuthoredConnections()
                     or _MatrixAttr(prim, name, time, ctx) != _IDENTITY):
            frames.pivotReason = ("%s.%s selects a default space independently "
                                  "of rest; edit that source for pivot changes"
                                  % (prim.GetName(), name))
    frames.posed = AvarsMatrix(prim, time, ctx) * frames.P
    frames = _PreferPublishedFrame(stage, prim, time, solverPosed, frames,
                                   ctx)

    _ApplyChannelSpaces(stage, prim, time, frames)

    assetRoot = frames.rigRoot.GetParent()
    if assetRoot and not assetRoot.IsPseudoRoot():
        frames.assetToWorld = ctx.XformCache()\
            .GetLocalToWorldTransform(assetRoot)
    ctx.frames[prim.GetPath()] = frames
    return frames


def _PoseProvider(prim, time):
    """
    The one prim whose POSE carries `prim`, or None.

    Mirrors what _ComputeRigFrames actually resolves, rather than
    assuming the namespace parent: frames.parentPosed comes from
    _ComputedSpace(prim, PARENT_SPACE), which follows a single authored
    CONNECTION to its source prim, treats an authored non-identity
    matrix as an absolute space with nothing upstream, and only then
    falls back to _FindParentXformable. Getting this wrong would not be
    cosmetic -- see PoseProviderPaths for what it decides.
    """
    attr = prim.GetAttribute(PARENT_SPACE)
    if attr:
        connections = attr.GetConnections()
        if len(connections) == 1:
            source = prim.GetStage().GetPrimAtPath(
                connections[0].GetPrimPath())
            return source if source and source.IsValid() else None
        if _MatrixAttr(prim, PARENT_SPACE, time) != _IDENTITY:
            # An absolute parent space: nothing above it carries this prim.
            return None
    return _FindParentXformable(prim, FindRigRoot(prim))


def PoseProviderChain(prim, time):
    """
    The prims whose pose carries `prim`, NEAREST FIRST: its pose parent,
    that prim's pose parent, and so on out of the rig and up the asset's
    xforms.

    Used for two things by a group drag, and for NEITHER of them as an
    exclusion rule. It orders the members so an ancestor's contribution
    is in place before a descendant's is computed, and it names each
    member's nearest selected ancestor so the descendant can be given
    the REMAINDER rather than a second copy of the same motion (see
    GroupTarget._Solve).

    That distinction is the whole design. This rig nests its FK controls
    in a namespace chain -- rigExec:controlSpace = "parentRelative" --
    where another rig would use constraints, so L_UpArm
    > elbow > wrist is one line of descent and so is every finger, leg
    and spine chain. An earlier version SKIPPED a member whose ancestor
    was also selected, on the grounds that the ancestor's rigid motion
    already carries it. That is true for a rigid pivot and false for
    Individual Origins, and either way it let an implementation detail
    -- how THIS rig happens to be wired -- decide what the animator's
    selection does. Measured on Biped.usda, a three-control selection of
    any FK chain had a delta reach 1 member of 3 in every pivot mode.
    The remainder solve replaces the rule with arithmetic: a member that
    is already exactly where it should be gets an identity remainder and
    authors nothing, and that is a RESULT rather than a policy.

    Ordered rather than a set because the order is what makes the walk
    topological; PoseProviderPaths is the set form, for membership tests.
    """
    chain = []
    seen = set()
    rigRoot = FindRigRoot(prim)
    if rigRoot is None:
        # A plain xformable: its namespace ancestors carry it, up to a
        # !resetXformStack! -- which is exactly what _XformTarget.Refresh
        # honours when it reads the parent-to-world transform.
        xformable = UsdGeom.Xformable(prim)
        if xformable and xformable.GetResetXformStack():
            return chain
        parent = prim.GetParent()
        while parent and not parent.IsPseudoRoot():
            chain.append(parent.GetPath())
            xformable = UsdGeom.Xformable(parent)
            if xformable and xformable.GetResetXformStack():
                return chain
            parent = parent.GetParent()
        return chain
    current = prim
    while current is not None:
        provider = _PoseProvider(current, time)
        if provider is None or provider.GetPath() in seen:
            break
        seen.add(provider.GetPath())
        chain.append(provider.GetPath())
        current = provider
    # Out of the rig: assetToWorld is the rig root's PARENT's
    # local-to-world (see the module banner), so those xforms carry the
    # whole rig and a selected one of them drives every control under it.
    parent = rigRoot.GetParent()
    while parent and not parent.IsPseudoRoot():
        chain.append(parent.GetPath())
        parent = parent.GetParent()
    return chain


def PoseProviderPaths(prim, time):
    """PoseProviderChain as a set, for membership tests."""
    return set(PoseProviderChain(prim, time))
