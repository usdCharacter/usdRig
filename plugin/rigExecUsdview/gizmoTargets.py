#
# RigExec usdview gizmo targets: what a world-space gizmo delta does to the
# channels of one control or of a group, and the writer that decides whether a
# value lands on the animation (spline knot at the current frame) or on the
# default.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): everything here is
# exercised headlessly by tests/python/test_gizmo_targets.py.
#
import math
from collections import OrderedDict

from pxr import Gf, Sdf, Tf, UsdGeom

from gizmoFrames import (
    AVAR_ORDER, AVAR_R, AVAR_RSPIN, AVAR_S, AVAR_T, AvarsMatrix,
    ComputeRigFrames, FindRigRoot, IsRigXformable, NormalizeAvarScale,
    PoseProviderChain, ReadsScaleAvars, REST_R, REST_SPACE, REST_T, RestSpace,
    RotationSign, ScalarAvar, _Context, _DropFromPreview, _MatrixAttr,
    _PreviewBucket, _Previewed, _previewValues,
)
from gizmoRotation import (
    _AXES, _AxisRotation, _IDENTITY, _Linear, _NormalizeOrder, _RotationOnly,
    _WorldRotation, DecomposeEuler, RotationFromEuler, SolveWorldRotation,
)

# Writing values: animation (spline knot at the frame) or default

WRITE_ANIMATION = "animation"

WRITE_DEFAULT = "default"

CHANNELS_POSE = "pose"

CHANNELS_PIVOT = "pivot"


# Where a MULTI-selection turns and scales about. Transform
# Pivot Point, restated for a rig:
#
#   GROUP_PIVOT_CENTER      the centroid of the selected controls'
#                           evaluated origins. THE DEFAULT, because the
#                           thing an animator means by "rotate these
#                           together" is about their middle -- not about
#                           whichever one they happened to click last.
#   GROUP_PIVOT_LEAD        the last-selected control's origin (the
#                           "Active Element"), for
#                           swinging a group about one of its members.
#   GROUP_PIVOT_INDIVIDUAL  each control about its OWN origin: they all
#                           turn by the same angle and none of them
#                           moves. "Individual Origins", and
#                           the only one of the three that is not a
#                           rigid motion of the selection.
#
# This is independent of the AXIS ORIENTATION (gizmoSettings.ORIENT_*),
# which says which way the rings point, not what they turn about.
GROUP_PIVOT_CENTER = "center"

GROUP_PIVOT_LEAD = "lead"

GROUP_PIVOT_INDIVIDUAL = "individual"

GROUP_PIVOT_MODES = (GROUP_PIVOT_CENTER, GROUP_PIVOT_LEAD,
                     GROUP_PIVOT_INDIVIDUAL)


def SetAnimated(attr, value, time):
    """
    volumeWeightUI.SetAtTime's rule (volumeWeightUI.py:68-111), restated
    here so this module stays Qt-free (volumeWeightUI imports Qt at
    module scope): default time or existing time samples or a
    spline-incapable type -> Set(value, time); otherwise a
    curve-interpolated knot on the attribute's spline.

    The knot itself comes from graphModel.AuthorKnot, so a gizmo drag
    and a graph-editor insert produce the SAME key -- the default new
    key, AutoEase on both tangents (graph editor design spec 1.2) -- and
    a key the artist has already shaped keeps its tangents.
    """
    writes = AnimatedWrites(attr, value, time)
    # One notice: a first key creates the spec and then sets the spline.
    with Sdf.ChangeBlock():
        for write in writes:
            write()


def AnimatedWrites(attr, value, time, adopt=True):
    """
    SetAnimated's reads done now, its writes returned as callables, so a
    caller can author several attributes in one Sdf.ChangeBlock after
    reading them all. `adopt` False skips copying a weaker layer's
    samples (the caller already copies them).
    """
    from pxr import Ts
    if attr.GetVariability() == Sdf.VariabilityUniform:
        return [lambda: attr.Set(value)]
    valueType = attr.GetTypeName().type
    if (time.IsDefault() or attr.GetNumTimeSamples() > 0 or
            not Ts.Spline.IsSupportedValueType(valueType)):
        writes = []
        if adopt and not time.IsDefault():
            writes.extend(_AdoptionWrites(attr))
        writes.append(lambda: attr.Set(value, time))
        return writes
    # Imported inside the function: graphModel imports THIS module at
    # module scope, so a top-level import here would be a cycle.
    import graphModel
    spline = graphModel.SplineFor(attr)
    graphModel.AuthorKnot(spline, time.GetValue(), value)
    return [lambda: attr.SetSpline(spline)]


def _AdoptionWrites(attr):
    """
    Copies of `attr`'s composed time samples into the edit target when they
    come from another layer, as the spline branch copies the composed spline:
    a layer's samples replace a weaker layer's outright, so keying one frame
    there would leave a single key.
    """
    target = attr.GetStage().GetEditTarget()
    if not attr.GetNumTimeSamples() or target.GetLayer().GetNumTimeSamplesForPath(
            target.MapToSpecPath(attr.GetPath())):
        return []
    return [lambda t=t, sample=attr.Get(t): attr.Set(sample, t)
            for t in attr.GetTimeSamples()]


class Writer(object):
    """
    Where a gizmo value lands, and WHEN.

    A drag COLLECTS; a release AUTHORS. While a manipulation is in progress
    Set() records the value and nothing reaches the stage -- the viewport is
    kept correct by handing the same values to Hydra instead (see the design
    note at docs/superpowers/specs/
    2026-09-10-hydra-preview-manipulation-design.md) -- and CommitToStage()
    authors all of them in one pass when the artist lets go.

    That is the only write path. It used to author on every mouse sample,
    which invalidated exec through the authoring stage, rewrote a layer spec,
    and notified every observer of the stage, all for a value the artist had
    not committed to -- and in WRITE_ANIMATION mode it authored a spline knot
    per sample to arrive at one knot.

    WRITE_ANIMATION authors at `time` through SetAnimated; WRITE_DEFAULT
    authors the default and records a warning for every attribute whose
    spline or time samples will outrank it (the default is then invisible in
    the viewport, and the toolbar says so rather than letting the drag look
    broken). Those warnings now appear when the value is authored, which is
    the same moment the artist finds out, because it is the same moment the
    value becomes real.
    """

    def __init__(self, stage, time, mode):
        self.stage = stage
        self.time = time
        self.mode = mode
        self._warnings = []
        # {Sdf.Path: value} in the order they were first set. Ordered so
        # CommitToStage authors deterministically, which keeps the undo
        # recorder's before/after diff reproducible.
        self._pending = OrderedDict()

    def Set(self, attr, value):
        """
        Record `value` for `attr`. Authors nothing; see CommitToStage.

        The value also goes straight into the module's preview map, which is
        what every read above consults first. That is not a convenience for
        the viewport -- it is a correctness requirement. A Preserve Children
        pivot drag writes the parent and then computes the children's
        compensation FROM the parent's new state, and a rotate reads the spin
        it just set; while those writes were authored immediately a plain stage
        read saw them, and now the uncommitted value has to be visible in the
        same way or the second half of such a drag computes from the first
        half's pre-drag values.
        """
        if not attr:
            return
        path = attr.GetPath()
        self._pending[path] = value
        _PreviewBucket(self.stage)[path] = value

    def Pending(self):
        """{Sdf.Path: value} collected so far, for the preview channel."""
        return OrderedDict(self._pending)

    def Clear(self):
        """Drop the collected values without authoring any of them."""
        _DropFromPreview(self.stage, list(self._pending))
        self._pending.clear()

    def HasPending(self):
        """Whether CommitToStage would author anything."""
        return bool(self._pending)

    def CommitToStage(self):
        """
        Author every collected value, once, inside one change block.

        Returns the paths authored. One SdfChangeBlock for the whole commit
        rather than one per attribute: a release is a single edit, and the
        observers watching the stage should see it as one.
        """
        if not self._pending:
            return []
        with Sdf.ChangeBlock():
            for path, value in self._pending.items():
                attr = self.stage.GetAttributeAtPath(path)
                if attr:
                    self._Author(attr, value)
        authored = list(self._pending.keys())
        # The stage holds these now, so the preview has nothing left to stand
        # in for. Dropped here rather than left for the caller: a value that
        # stayed in the preview map would shadow the authored one, and the next
        # reader could not tell which it was looking at.
        _DropFromPreview(self.stage, authored)
        self._pending.clear()
        return authored

    def _Author(self, attr, value):
        if self.mode == WRITE_DEFAULT:
            if attr.HasSpline() or attr.GetNumTimeSamples() > 0:
                message = ("%s: the default is outranked by its %s" % (
                    attr.GetPath(),
                    "spline" if attr.HasSpline() else "time samples"))
                if message not in self._warnings:
                    self._warnings.append(message)
            attr.Set(value)
        else:
            SetAnimated(attr, value, self.time)

    def Warnings(self):
        return list(self._warnings)


def _FrameAt(matrix, origin):
    """`matrix`'s rotation, moved to `origin`: a drawable handle frame."""
    m = _RotationOnly(matrix)
    m.SetTranslateOnly(Gf.Vec3d(origin))
    return m


def _SnapValue(value, step):
    """
    `value` rounded to the nearest multiple of `step` (Step Snap),
    or `value` unchanged when `step` is None or 0.

    Halves go AWAY from zero. Python's round() sends them to even, which
    makes a slow drag stick unevenly -- 0.5 snapping to 0 but 1.5 to 2 --
    and the asymmetry is visible on a step grid.
    """
    if not step:
        return value
    steps = math.floor(abs(value) / abs(step) + 0.5)
    return math.copysign(steps * abs(step), value)


def _SnapTranslation(base, delta, step, absolute):
    """
    base + delta with Step Snap applied where the channel values live.

    Relative (`J` hold) quantises the DELTA, so a drag advances in
    whole steps from wherever it started; absolute quantises the RESULT,
    so the channel value lands on the grid however the drag started.
    Snapping the WORLD delta instead would put the channels off the grid
    whenever the channel frame is rotated, which is the whole reason this
    lives here and not in the controller. No UI binds the absolute form
    any more: the world grid goes through gizmoSnap.GridPoint and a plain
    ApplyTranslate with no keyword (snapping design, sections 1-2), and
    this parameter stays only because it is still correct and tested.
    """
    if not step:
        return [base[i] + delta[i] for i in range(3)]
    if absolute:
        return [_SnapValue(base[i] + delta[i], step) for i in range(3)]
    return [base[i] + _SnapValue(delta[i], step) for i in range(3)]


def _ScaleAxes(axisIndex):
    """
    Which scale channels a drag touches: None for all three (the centre
    cube), an int for one (an axis handle), or ANY iterable of ints for a
    planar handle -- (0, 1) is the XY square.

    The iterable case is duck-typed rather than a list of accepted
    classes: the controller hands over whatever its handle description
    carries, and an isinstance whitelist turned a perfectly good
    generator or sequence-like object into a TypeError deep inside a
    drag. Only the iteration is guarded, so a genuinely bad element
    still raises where it is written rather than being read as a
    single-axis index.
    """
    if axisIndex is None:
        return (0, 1, 2)
    try:
        axes = tuple(axisIndex)
    except TypeError:
        return (int(axisIndex),)
    return tuple(int(i) for i in axes)


class Target(object):
    """
    One prim being edited by the gizmo. Subclasses know which attributes
    they own and how a WORLD-space delta maps back onto them. Every
    Apply* is computed from the values captured by BeginDrag(), never
    incrementally, so a drag cannot accumulate rounding drift and an
    aborted drag has one well-defined state to return to.

    Three frames, all orthonormal and all translated to the gizmo origin,
    feed the Axis Orientation option (design spec 8.2):

      ObjectFrame()   "Object": the target's own posed orientation
      ChannelFrame()  "Parent": the space the channels are written
                      in (P or Q for a rig prim, the parent xform
                      otherwise)
      GimbalFrame()   the space the ROTATE channels compose in; equal to
                      ChannelFrame() except on a rig pose target, where
                      avars:rspin sits between the Euler product and P

    World orientation needs no frame: the controller uses the identity.
    """

    kind = ""
    supportsTranslate = True
    supportsRotate = True
    supportsScale = True
    supportsPreserveChildren = False
    preserveChildrenReason = ""

    def __init__(self, stage, prim, writer):
        self.stage = stage
        self.prim = prim
        self.writer = writer
        self.label = prim.GetName()
        self.preserveChildren = False
        # Children a Preserve Children drag will NOT hold still, one
        # sentence each, for the toolbar to show: silently leaving a
        # child behind looks like a bug from the viewport.
        self.skippedChildren = []
        self._base = {}

    @property
    def time(self):
        return self.writer.time

    def Advisory(self):
        """
        A note about an edit that will succeed and change nothing
        visible, or "" when there is none. Sibling of the
        `the default is outranked by its spline` warning: the drag is
        not refused, it is explained.
        """
        return ""

    def RefreshDuringDrag(self):
        """
        Re-read what a drag sample can have moved, as cheaply as is exact.

        A drag sample changes only the previewed values of THIS target's own
        channels, so for most targets nothing but its posed frame can have
        moved. The default is the full Refresh; subclasses that can prove a
        narrower answer override it.
        """
        self.Refresh()

    def Refresh(self, frameCache=None):
        """
        Re-read the stage; call after a frame change or an undo.

        `frameCache` is the optional {path: RigFrames} dict
        ComputeRigFrames threads through its own recursion, offered here
        so a GROUP can refresh several targets against ONE walk. It has
        to be fresh for each round of refreshes -- a cache kept across
        mouse samples would hand out the previous sample's frames -- so
        the group builds a new one every time and nothing here keeps it.
        Ignored by targets with no rig frames to compute.
        """

    def RigRootPath(self):
        """
        The enclosing RigExecRoot's path, or None for a plain xform.

        The controller needs it to decide whether a stage notice can
        have moved this target: a rig control's frame is composed from
        every avar above it and can be posed by a solver reading a joint
        elsewhere in the rig, so anything under the root is relevant
        (NoticeAffectsTarget), while an xform only follows its own
        ancestors.
        """
        return None

    def GizmoMatrix(self):
        raise NotImplementedError

    def ObjectFrame(self):
        return self.GizmoMatrix()

    def ChannelFrame(self):
        raise NotImplementedError

    def GimbalFrame(self):
        return self.ChannelFrame()

    def RotationState(self):
        """
        (rotation order, [rx, ry, rz]) for the Euler channels this target
        writes, or None when it has none. Read live from the stage, so
        the controller can draw gimbal rings outside a drag.
        """
        return None

    def SetPreserveChildren(self, enabled):
        """
        "Preserve Children". A target that cannot honour it stays
        off however often it is asked, so a stale toolbar checkbox can
        never make a drag silently skip the compensation.
        """
        self.preserveChildren = bool(enabled) and \
            self.supportsPreserveChildren

    def AttributePaths(self):
        """
        Every attribute this target may write, for the undo recorder.

        STATEFUL on the xform pose target: with Preserve Children on it
        also returns the compensated children's channels. Call it AFTER
        SetPreserveChildren() and BEFORE the recorder's Begin(), or an
        undo will move the parent back and leave the children behind.
        """
        raise NotImplementedError

    def BeginDrag(self, frameCache=None):
        self.Refresh(frameCache)
        self._base = {name: ScalarAvar(self.prim, name, self.time, fallback)
                      for name, fallback in self._ScalarChannels()}

    def _ScalarChannels(self):
        return []

    def _Write(self, name, value):
        self.writer.Set(self.prim.GetAttribute(name), float(value))

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        """
        Move by a WORLD delta, optionally with Step Snap.

        `snapStep` quantises in CHANNEL space, where the values that get
        written live; see _SnapTranslation for why that is not the same
        as quantising the world delta, and for what `snapAbsolute` picks
        between. Passing no snapStep leaves the delta exactly as given.
        """
        raise NotImplementedError

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        """
        Turn the drawn world frame by `degrees` about a WORLD axis.

        `snapStep` quantises the ANGLE, relative to the drag base, so a
        snapped drag advances in whole steps (`J` hold, default
        step 15 degrees).
        """
        raise NotImplementedError

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        """
        Add `degrees` to ONE Euler channel (0 = X, 1 = Y, 2 = Z), the
        others held at their drag base. `snapStep` quantises the angle
        exactly as it does for ApplyRotate.

        This is Gimbal mode (design spec 8.3, "each ring changes
        exactly one Euler channel"), and it needs its own entry point
        because ApplyRotate cannot deliver it: ApplyRotate takes a world
        axis and promises the drawn frame turns by the dragged angle,
        and under a sheared channel frame (an ancestor with non-uniform
        scale) the rotation that does that moves all three channels.
        Here the guarantee is exact whatever the channel frame is,
        because the Euler product isolates the channel by construction.
        """
        raise NotImplementedError

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        """
        Multiply the scale channels by `factor`.

        `axisIndex` picks them: None for all three (the centre cube), an
        int for one (an axis handle), or a tuple or list for a planar
        handle -- (0, 1) is the XY square. `snapStep` quantises the
        RESULTING value of each channel the drag touches, before the
        evaluator's 1e-4 floor. Channels the drag does not touch keep
        their authored value rather than being nudged onto the grid.
        """
        raise NotImplementedError


class _RigTarget(Target):
    """Shared frame bookkeeping for the two RigExecXformable modes."""

    # A control's children are placed by the evaluator from the parent's
    # published frame, so there is nothing to re-author: compensating
    # them here would fight the rig on the next evaluation.
    preserveChildrenReason = ("children of a rig control are evaluated by "
                              "the rig")

    def __init__(self, stage, prim, writer, solverPosed=None,
                 frameCache=None):
        Target.__init__(self, stage, prim, writer)
        # A SolverPosedCache, or None to walk the rig on every Refresh.
        self._solverPosed = solverPosed
        self.frames = None
        # `frameCache` is used for THIS walk and then dropped: it belongs
        # to whoever is building targets right now (MakeGroupTarget
        # builds several off one walk), and keeping it would hand out
        # frames composed from a stage that has since moved.
        self.Refresh(frameCache)

    def Refresh(self, frameCache=None):
        posed = None
        if self._solverPosed is not None:
            posed = self._solverPosed.For(FindRigRoot(self.prim))
        self.frames = ComputeRigFrames(self.stage, self.prim, self.time,
                                       posed, frameCache)

    def RefreshDuringDrag(self):
        """
        The posed frame only, for a pose drag on this one control.

        Measured on the biped: a full Refresh per mouse move recomputed every
        ancestor's frame in Python -- ~18 ms per sample for a brow control
        twenty levels deep, more than the rig's own evaluation. During a pose
        drag of a single control, only ITS avars are previewed, so everything
        the recursion produces is unchanged except `posed`:

          P, Q, rest, Qrest, default   depend on the parent and on rest/default
                                       channels, none of which a pose drag
                                       writes
          posed                        = AvarsMatrix * P, exactly the line
                                       _ComputeRigFrames uses

        Anything that breaks that argument takes the full Refresh: a frame
        the rig published (a solver-posed control, whose pose is not its
        avars), a refused target, or no frames yet.
        """
        frames = self.frames
        if (frames is None or frames.rigRoot is None or frames.published or
                frames.reason):
            self.Refresh()
            return
        context = _Context(None, self.time, self.prim)
        frames.posed = AvarsMatrix(self.prim, self.time, context) * frames.P

    def RigRootPath(self):
        root = self.frames.rigRoot if self.frames is not None else None
        return root.GetPath() if root else None

    def _WriteVector(self, names, values):
        # No change block: these collect rather than author (Writer.Set),
        # so there is no edit to batch. Writer.CommitToStage wraps the
        # whole release in one block.
        for name, value in zip(names, values):
            self._Write(name, value)


class RigPoseTarget(_RigTarget):
    """Edits avars:t/r/s relative to P (see RigFrames)."""

    kind = "rig-pose"

    def Refresh(self, frameCache=None):
        super().Refresh(frameCache)
        if self.frames.translationP is not None:
            self.supportsTranslate = self.frames.translationEnabled

    def _ScalarChannels(self):
        return ([(n, 0.0) for n in AVAR_T] + [(n, 0.0) for n in AVAR_R]
                + [(n, 1.0) for n in AVAR_S] + [(AVAR_RSPIN, 0.0)])

    def _Order(self):
        attr = self.prim.GetAttribute(AVAR_ORDER)
        value = attr.Get(self.time) if attr else None
        return _NormalizeOrder(value)

    def _Sign(self):
        # avars:rotationSign: the drag works in EFFECTIVE angles, which is
        # what the limb actually turns by and what the gizmo draws, and
        # signs them back on the way to the stage.
        return RotationSign(self.prim)

    def AttributePaths(self):
        return [self.prim.GetPath().AppendProperty(n)
                for n in AVAR_T + AVAR_R + AVAR_S]

    def GizmoMatrix(self):
        world = self.frames.posed * self.frames.assetToWorld
        return world.GetOrthonormalized(False)

    def _Pw(self):
        return self.frames.P * self.frames.assetToWorld

    def ChannelFrame(self):
        return _FrameAt(self._Pw(), self.GizmoMatrix().ExtractTranslation())

    def GimbalFrame(self):
        spin = _AxisRotation(0, ScalarAvar(
            self.prim, AVAR_RSPIN, self.time, 0.0) * self._Sign()[0])
        return _FrameAt(spin * self._Pw(),
                        self.GizmoMatrix().ExtractTranslation())

    def RotationState(self):
        sign = self._Sign()
        return (self._Order(),
                [ScalarAvar(self.prim, n, self.time, 0.0) * sign[i]
                 for i, n in enumerate(AVAR_R)])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        if not self.supportsTranslate:
            return
        translation = self.frames.translationP if self.frames.translationP is not None else self.frames.P
        local = _Linear(translation * self.frames.assetToWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta)) / self.frames.unitScale
        base = [self._base[n] for n in AVAR_T]
        self._WriteVector(AVAR_T, _SnapTranslation(
            base, local, snapStep, snapAbsolute))

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        sign = self._Sign()
        base = [self._base[n] * sign[i] for i, n in enumerate(AVAR_R)]
        order = self._Order()
        # World linear = S * R * spin * linear(P*assetToWorld); only R is
        # ours to move, so spin * P is the channel frame it turns inside.
        channel = _AxisRotation(
            0, self._base[AVAR_RSPIN] * sign[0]) * self._Pw()
        rNew = SolveWorldRotation(RotationFromEuler(order, *base), channel,
                                  worldAxis, _SnapValue(degrees, snapStep))
        solved = DecomposeEuler(rNew, order, hint=base)
        self._WriteVector(AVAR_R, [v * sign[i]
                                   for i, v in enumerate(solved)])

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        # The channel handle turns the limb by `degrees` about its own
        # effective axis, so a mirrored channel takes the negated step.
        values = [self._base[n] for n in AVAR_R]
        values[axisIndex] = (values[axisIndex] +
                             _SnapValue(degrees, snapStep) *
                             self._Sign()[axisIndex])
        self._WriteVector(AVAR_R, values)

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        axes = _ScaleAxes(axisIndex)
        values = []
        for i, name in enumerate(AVAR_S):
            value = self._base[name]
            if i in axes:
                value = _SnapValue(value * factor, snapStep)
            values.append(NormalizeAvarScale(value))
        self._WriteVector(AVAR_S, values)


class RigPivotTarget(_RigTarget):
    """Edits rest:t/r relative to Q (see RigFrames); no scale."""

    kind = "rig-pivot"

    def RefreshDuringDrag(self):
        # A pivot drag previews REST channels, which move rest, Qrest and
        # everything composed from them -- the posed-only shortcut does not
        # hold here, so every sample takes the full walk.
        self.Refresh()
    supportsScale = False
    # Rest offsets are parent-relative, so a pivot drag carries the whole
    # subtree with it. Preserve Children is therefore meaningful here in a
    # way it is not for a rig POSE edit: the compensation is authored into
    # each child's own rest channels, which the evaluator does read, rather
    # than into xformOps, which it ignores.
    supportsPreserveChildren = True
    preserveChildrenReason = ""

    def _ChildProviders(self):
        """
        Immediate child frame providers.

        Immediate only: rest is parent-relative, so a grandchild rides on
        a child this already holds still, and compensating it too would
        double-count the correction.
        """
        return [child for child in self.prim.GetChildren()
                if IsRigXformable(child)]

    def _ScalarChannels(self):
        return [(n, 0.0) for n in REST_T] + [(n, 0.0) for n in REST_R]

    def AttributePaths(self):
        paths = [self.prim.GetPath().AppendProperty(n)
                 for n in REST_T + REST_R]
        self.skippedChildren = []
        if self.preserveChildren:
            for child in self._ChildProviders():
                paths.extend(child.GetPath().AppendProperty(n)
                             for n in REST_T + REST_R)
        return paths

    def BeginDrag(self, frameCache=None):
        _RigTarget.BeginDrag(self, frameCache)
        # The world rest each child must be put back onto. Captured before
        # the first write, so a multi-event drag compensates against the
        # start of the drag rather than accumulating per event.
        self._preserved = [(child, RestSpace(child, self.time))
                           for child in self._ChildProviders()] \
            if self.preserveChildren else []

    def _RestoreChildren(self):
        """
        Put every captured child back on its starting world rest.

        restLocal(child) = restWorld(child) * restWorld(self)^-1, then the
        rest:t/r scalars that produce it against the child's own untouched
        rest:space -- so the bind basis stays the author's and only the
        offset channels move.
        """
        preserved = getattr(self, "_preserved", [])
        if not preserved:
            return
        inverse = RestSpace(self.prim, self.time).GetInverse()
        # No change block: collecting authors nothing (Writer.Set), so there
        # is no edit to batch. The release batches, in CommitToStage.
        for child, worldRest in preserved:
            scalars = (worldRest * inverse
                       * _MatrixAttr(child, REST_SPACE,
                                     self.time).GetInverse())
            scalars = scalars.GetOrthonormalized(False)
            hint = [ScalarAvar(child, n, self.time, 0.0) for n in REST_R]
            for name, value in zip(REST_T, scalars.ExtractTranslation()):
                self.writer.Set(child.GetAttribute(name), float(value))
            for name, value in zip(
                    REST_R, DecomposeEuler(scalars, "XYZ", hint=hint)):
                self.writer.Set(child.GetAttribute(name), float(value))

    def Advisory(self):
        # A bone length can no longer be authored -- rigExec:upperLength /
        # rigExec:lowerLength are computed from the joints' rests on every
        # evaluation and Compile rejects an authored opinion. So a pivot
        # drag always reaches the solve, and there is nothing to warn about.
        return ""

    def _Qw(self):
        # Qrest AND the parent's REST, not Q: rest:t/r are expressed
        # against rest:space carried into the parent's rest frame
        # (computations.cpp:230), so the parent's rest belongs in the frame
        # this drag inverts through and draws on -- but its POSE still does
        # not, which is what keeps a pivot still under an animated ancestor.
        return self.frames.Qrest * self.frames.parentRest \
            * self.frames.assetToWorld

    def GizmoMatrix(self):
        return (self.frames.restLocal * self._Qw()).GetOrthonormalized(False)

    def ChannelFrame(self):
        return _FrameAt(self._Qw(), self.GizmoMatrix().ExtractTranslation())

    def RotationState(self):
        # RestLocal composes rest:r in XYZ with no spin (computations.cpp
        # :196-215), so the pivot rings are always XYZ rings.
        return ("XYZ",
                [ScalarAvar(self.prim, n, self.time, 0.0) for n in REST_R])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self._Qw()).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        base = [self._base[n] for n in REST_T]
        self._WriteVector(REST_T, _SnapTranslation(
            base, local, snapStep, snapAbsolute))
        self._RestoreChildren()

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        base = [self._base[n] for n in REST_R]
        rNew = SolveWorldRotation(RotationFromEuler("XYZ", *base),
                                  self._Qw(), worldAxis,
                                  _SnapValue(degrees, snapStep))
        self._WriteVector(REST_R, DecomposeEuler(rNew, "XYZ", hint=base))
        self._RestoreChildren()

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        values = [self._base[n] for n in REST_R]
        values[axisIndex] = values[axisIndex] + _SnapValue(degrees, snapStep)
        self._WriteVector(REST_R, values)
        self._RestoreChildren()

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        pass

_XFORM_ORDER_NAMES = {
    UsdGeom.XformCommonAPI.RotationOrderXYZ: "XYZ",
    UsdGeom.XformCommonAPI.RotationOrderXZY: "XZY",
    UsdGeom.XformCommonAPI.RotationOrderYXZ: "YXZ",
    UsdGeom.XformCommonAPI.RotationOrderYZX: "YZX",
    UsdGeom.XformCommonAPI.RotationOrderZXY: "ZXY",
    UsdGeom.XformCommonAPI.RotationOrderZYX: "ZYX",
}

_ZERO3F = Gf.Vec3f(0.0, 0.0, 0.0)


def _XformCommonMatrix(t, p, r, s, order):
    """
    The local matrix of an XformCommonAPI op stack, composed from its
    five components WITHOUT reading the stage.

    Preserve Children compensates the children against the parent's NEW
    world transform, and the write that produces it now shares one
    Sdf.ChangeBlock with the children's own writes -- reading a
    recomposed value back from inside the block that authored it is the
    documented Sdf.ChangeBlock hazard. Composing it here instead is what
    lets a whole Apply* be a single change notification.

    Probed against GetLocalTransformation and covered by the "xform
    local matrix" group in test_gizmo_math.py: the stack
    [translate, pivot, rotate, scale, !invert!pivot] composes row-vector
    as pivot^-1 * S * R * pivot * T -- ops apply to points last-to-first
    -- and R is RotationFromEuler in the op's own rotation order. The
    two pure translations collapse into one because pivot and translate
    commute.
    """
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(-Gf.Vec3d(p))
    scale = Gf.Matrix4d(1.0)
    scale.SetScale(Gf.Vec3d(s))
    m = m * scale * RotationFromEuler(order, r[0], r[1], r[2])
    back = Gf.Matrix4d(1.0)
    back.SetTranslate(Gf.Vec3d(p) + Gf.Vec3d(t))
    return m * back


class _PreservedChild(object):
    """
    One child of a Preserve Children drag: its world transform at the
    press, plus the values needed to re-author it afterwards.

    The Euler hint is the child's angles at the press, so the
    compensation stays on the same branch as whatever an animator
    already authored instead of jumping to an equivalent triple.
    """

    def __init__(self, prim, api, vectors, cache):
        self.prim = prim
        self.api = api
        self.order = vectors[4]
        self.angles = [float(v) for v in vectors[1]]
        self.world = cache.GetLocalToWorldTransform(prim)

    def AttributePaths(self):
        prefix = self.prim.GetPath()
        rotateName = "xformOp:rotate" + _XFORM_ORDER_NAMES[self.order]
        return [prefix.AppendProperty(n) for n in (
            "xformOp:translate", rotateName, "xformOp:scale",
            "xformOpOrder")]


class _XformTarget(Target):
    """
    A plain UsdGeomXformable through UsdGeomXformCommonAPI, whose op
    stack is [translate, pivot, rotate, scale, !invert!pivot]: ops apply
    to points last-to-first, so scale then rotation happen about the
    pivot and the translate is in PARENT space -- which is why world
    deltas are mapped through the parent's transform, not the prim's.
    """

    def __init__(self, stage, prim, writer):
        Target.__init__(self, stage, prim, writer)
        self.api = UsdGeom.XformCommonAPI(prim)
        self.vectors = None
        self.parentWorld = Gf.Matrix4d(1.0)
        self._preserved = []
        self.Refresh()

    def Refresh(self, frameCache=None):
        cache = UsdGeom.XformCache(self.time)
        # An op stack beginning with !resetXformStack! ignores its
        # ancestors, and XformCommonAPI accepts one. GetParentToWorld-
        # Transform is just the parent's CTM (xformCache.cpp:37-41) and
        # does not consult the flag, so without this the gizmo would be
        # drawn and the children compensated against a parent frame the
        # prim does not actually sit in.
        if UsdGeom.Xformable(self.prim).GetResetXformStack():
            self.parentWorld = Gf.Matrix4d(1.0)
        else:
            self.parentWorld = cache.GetParentToWorldTransform(self.prim)
        self.vectors = self.api.GetXformVectors(self.time)
        self.vectors = self._PreviewedVectors(self.vectors)

    def _PreviewedVectors(self, vectors):
        """
        `vectors` with any uncommitted op value substituted in.

        GetXformVectors reads the stage through XformCommonAPI, which a
        preview cannot reach -- nothing is authored for it to read -- so the
        substitution happens here, on the same four slots the API reports.
        Without it the handles would snap back to the pre-drag transform on
        every mouse sample while the geometry moved, which is the same defect
        the preview-aware reads above answer for a rig control.

        The ops are matched the way XformCommonAPI itself interprets them: a
        plain translate is the translation, a translate named `pivot` is the
        pivot, any rotate is the rotation, and scale is the scale.
        """
        if not _previewValues:
            return vectors
        t, r, sc, p, order = vectors
        for op in UsdGeom.Xformable(self.prim).GetOrderedXformOps():
            value = _Previewed(op.GetAttr())
            if value is None:
                continue
            opType = op.GetOpType()
            name = op.GetOpName()
            if opType == UsdGeom.XformOp.TypeTranslate:
                # HasSuffix is C++-only; the op NAME carries the suffix and is
                # what XformCommonAPI keys the pivot off.
                if name.endswith(":pivot"):
                    p = Gf.Vec3f(value)
                else:
                    t = Gf.Vec3d(value)
            elif opType == UsdGeom.XformOp.TypeScale:
                sc = Gf.Vec3f(value)
            elif name.startswith("xformOp:rotate"):
                r = Gf.Vec3f(value)
        return (t, r, sc, p, order)

    def BeginDrag(self, frameCache=None):
        self.Refresh(frameCache)
        t, r, s, p, order = self.vectors
        self._base = {"t": Gf.Vec3d(t), "r": Gf.Vec3f(r), "s": Gf.Vec3f(s),
                      "p": Gf.Vec3f(p), "order": order}
        self._preserved = self._PreserveCandidates()

    def ChannelFrame(self):
        return _FrameAt(self.parentWorld,
                        self.GizmoMatrix().ExtractTranslation())

    def _Ops(self, *which):
        """
        Create ONLY the ops named in `which`, and answer all five slots.

        CreateXformOps always returns a 5-tuple in fixed slots --
        translate, pivot, rotate, scale, !invert!pivot -- filling the
        slots it was not asked for with invalid ops, and it leaves an
        already-authored op alone (probed: asking for translate, rotate
        and scale on a prim that already has a pivot keeps the pivot and
        still reports it in slot 1).

        Each caller asks for the one op it is about to write, so a drag
        adds no scene description beyond what it changes. Asking for all
        four instead gave a prim that had none a zero pivot pair and two
        more ops on the first translate, none of which the user asked
        for and all of which then had to be undone.
        """
        return self.api.CreateXformOps(self._base["order"], *which)

    def _PreserveCandidates(self):
        """
        The children a Preserve Children drag can hold still, and as a
        side effect the sentences in self.skippedChildren explaining the
        ones it cannot.

        A RigExec prim is never a candidate even though the schema makes
        it a UsdGeomXformable with an empty (so XformCommonAPI-
        compatible) op stack: RigExecXformable inherits Boundable
        (libs/rigExecSchema/schema.usda:200-201). The evaluator places
        those prims from their avars and never reads an xformOp, so
        authoring one would not hold the child still AND would break the
        project rule that RigExec prims carry no xformOps. FindRigRoot
        catches plain xforms grouped inside a rig too, whose children
        are the evaluator's business either way.

        The rest of the filter: an XformCommonAPI-compatible stack, a
        zero pivot (a non-zero pivot makes the local decomposition of
        the compensated matrix ambiguous) and a non-reflecting linear
        part (row lengths cannot recover a negative scale, so a mirrored
        child would silently come back unmirrored).
        """
        self.skippedChildren = []
        if not self.preserveChildren:
            return []
        cache = UsdGeom.XformCache(self.time)
        found = []
        for child in self.prim.GetChildren():
            name = child.GetName()
            if IsRigXformable(child) or FindRigRoot(child) is not None:
                self.skippedChildren.append(
                    "%s is placed by the rig, not by xformOps" % name)
                continue
            if not child.IsA(UsdGeom.Xformable):
                continue
            api = UsdGeom.XformCommonAPI(child)
            if not api:
                self.skippedChildren.append(
                    "%s: xformOp stack is not XformCommonAPI-compatible"
                    % name)
                continue
            vectors = api.GetXformVectors(self.time)
            if Gf.Vec3f(vectors[3]) != _ZERO3F:
                self.skippedChildren.append(
                    "%s has a non-zero pivot" % name)
                continue
            local = UsdGeom.Xformable(child).GetLocalTransformation(self.time)
            if _Linear(local).GetDeterminant() <= 0.0:
                self.skippedChildren.append(
                    "%s is mirrored or collapsed" % name)
                continue
            found.append(_PreservedChild(child, api, vectors, cache))
        return found

    def _ChildCompensation(self, parentWorld):
        """
        [(child, translation, angles, scale)] putting every recorded
        child back on its captured world matrix against the parent's NEW
        world transform: childLocal = childWorld * newParent^-1, split
        into row lengths (scale), the orthonormalized linear part
        (rotation) and the translation row -- exactly the decomposition
        XformCommonAPI recomposes from.

        Computation only, authoring nothing, so the caller can put every
        write of one Apply* -- the target's op and all 3N child channels
        -- inside a single Sdf.ChangeBlock.
        """
        if not self._preserved:
            return []
        if abs(_Linear(parentWorld).GetDeterminant()) < 1e-12:
            # A collapsed parent (a scale drag through zero) has no
            # inverse; Gf answers with FLT_MAX rather than raising, so
            # compensating here would author garbage on the children.
            # Leave them where they are and let the next event recover.
            return []
        inverse = parentWorld.GetInverse()
        writes = []
        for entry in self._preserved:
            local = entry.world * inverse
            linear = _Linear(local)
            scale = [Gf.Vec3d(linear[r][0], linear[r][1],
                              linear[r][2]).GetLength() for r in range(3)]
            angles = DecomposeEuler(linear, _XFORM_ORDER_NAMES[entry.order],
                                    hint=entry.angles)
            writes.append((entry, local.ExtractTranslation(),
                           Gf.Vec3f(*angles), Gf.Vec3f(*scale)))
        return writes

    def _WriteOp(self, opType, slot, value, t=None, r=None, s=None, p=None):
        """
        Author one XformCommonAPI op and every Preserve Children
        compensation, with all the VALUES in one Sdf.ChangeBlock, so a
        mouse-move costs the stage one change notification (design spec
        3.3 and 4.2).

        The RigExec evaluator republishes synchronously on
        ObjectsChanged, so the unbatched version of this cost 1 + 3N
        recompositions per event with N preserved children, all thrown
        away but the last.

        `t`/`r`/`s`/`p` name the component this call changes; the others
        come from the drag base. They are needed rather than read back
        because the children are compensated against the parent's new
        world transform, which _XformCommonMatrix composes instead of
        the stage recomposing it -- see that function for why.

        Op CREATION stays outside the block, deliberately. AddXformOp
        validates the op it just created with a composed stage query
        (xformable.cpp:200-207), and inside a change block a prim whose
        specs come only from a reference cannot see the spec that was
        just authored -- its index has not been rescanned -- so the op
        reads back undefined, AddXformOp posts a coding error, and the
        Apply* raises before it has written anything. Creating the ops
        first costs one extra notification on the first event of a drag
        that has an op to create, and nothing on any event after it,
        which is once per drag rather than once per mouse-move.
        """
        children = []
        if self._preserved:
            base = self._base
            local = _XformCommonMatrix(
                base["t"] if t is None else t,
                base["p"] if p is None else p,
                base["r"] if r is None else r,
                base["s"] if s is None else s,
                _XFORM_ORDER_NAMES[base["order"]])
            children = self._ChildCompensation(local * self.parentWorld)
        api = UsdGeom.XformCommonAPI
        target = self._Ops(opType)[slot].GetAttr()
        childOps = [entry.api.CreateXformOps(
            entry.order, api.OpTranslate, api.OpRotate, api.OpScale)
            for entry, _, _, _ in children]
        # No change block, for the same reason: the writes below collect, and
        # CommitToStage batches the release. Op CREATION above still authors,
        # and still has to stay outside any block -- see the class comment.
        self.writer.Set(target, value)
        for ops, (_, translation, angles, scale) in zip(childOps, children):
            self.writer.Set(ops[0].GetAttr(), translation)
            self.writer.Set(ops[2].GetAttr(), angles)
            self.writer.Set(ops[3].GetAttr(), scale)

    def AttributePaths(self):
        prefix = self.prim.GetPath()
        _, _, _, _, order = self.vectors
        rotateName = "xformOp:rotate" + _XFORM_ORDER_NAMES[order]
        paths = [prefix.AppendProperty(n) for n in (
            "xformOp:translate", "xformOp:translate:pivot", rotateName,
            "xformOp:scale", "xformOpOrder")]
        # The compensated children are re-authored by the same drag, so
        # they belong to the same undo entry as the target itself.
        for entry in self._PreserveCandidates():
            paths.extend(entry.AttributePaths())
        return paths


class XformPoseTarget(_XformTarget):
    kind = "xform-pose"
    supportsPreserveChildren = True

    def GizmoMatrix(self):
        """
        The object's orientation, drawn at the PIVOT (design spec 8.2).

        All three manipulators are centred on the point the rotate and
        scale ops turn about, which for this op stack is `pivot +
        translate` in parent space. Drawing at the prim's local origin
        instead would put the rotate rings off the point the object
        turns about, and the ring centre would orbit during the drag.
        """
        local = UsdGeom.Xformable(self.prim).GetLocalTransformation(self.time)
        m = (local * self.parentWorld).GetOrthonormalized(False)
        t, _, _, p, _ = self.vectors
        m.SetTranslateOnly(self.parentWorld.Transform(Gf.Vec3d(p) + t))
        return m

    def RotationState(self):
        # Read through the API rather than the cached self.vectors: the
        # base class promises a live answer, and the controller asks for
        # one between the writes of a drag to redraw the rings.
        _, r, _, _, order = self.api.GetXformVectors(self.time)
        return (_XFORM_ORDER_NAMES[order], [float(v) for v in r])

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        base = self._base["t"]
        values = _SnapTranslation([base[i] for i in range(3)], local,
                                  snapStep, snapAbsolute)
        t = Gf.Vec3d(*values)
        self._WriteOp(UsdGeom.XformCommonAPI.OpTranslate, 0, t, t=t)

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        order = _XFORM_ORDER_NAMES[self._base["order"]]
        base = [float(v) for v in self._base["r"]]
        rNew = SolveWorldRotation(RotationFromEuler(order, *base),
                                  self.parentWorld, worldAxis,
                                  _SnapValue(degrees, snapStep))
        angles = Gf.Vec3f(*DecomposeEuler(rNew, order, hint=base))
        self._WriteOp(UsdGeom.XformCommonAPI.OpRotate, 2, angles, r=angles)

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        values = [float(v) for v in self._base["r"]]
        values[axisIndex] = values[axisIndex] + _SnapValue(degrees, snapStep)
        angles = Gf.Vec3f(*values)
        self._WriteOp(UsdGeom.XformCommonAPI.OpRotate, 2, angles, r=angles)

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        axes = _ScaleAxes(axisIndex)
        s = Gf.Vec3f(self._base["s"])
        for i in range(3):
            if i in axes:
                s[i] = _SnapValue(s[i] * factor, snapStep)
        self._WriteOp(UsdGeom.XformCommonAPI.OpScale, 3, s, s=s)


class XformPivotTarget(_XformTarget):
    kind = "xform-pivot"
    supportsRotate = False
    supportsScale = False
    # Children DO move with a pivot edit whenever the prim carries a
    # rotate or a scale, since the pivot is part of the local matrix.
    # The brief still puts Preserve Children out of scope for pivot
    # mode (design spec 1.2: a pivot edit is uncompensated by design,
    # like moving any pivot without compensation), so say that
    # rather than claiming the children hold still on their own.
    preserveChildrenReason = ("pivot edits are not compensated, on this "
                              "prim or its children (spec 1.2)")

    def GizmoMatrix(self):
        t, r, s, p, order = self.vectors
        m = _RotationOnly(self.parentWorld)
        m.SetTranslateOnly(self.parentWorld.Transform(Gf.Vec3d(p) + t))
        return m

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        # Both sides are rounded to float32 before the sum, so the
        # unsnapped result is the same value the plain Vec3f addition
        # this replaced produced.
        base = [float(v) for v in self._base["p"]]
        delta = [float(v) for v in Gf.Vec3f(local)]
        p = Gf.Vec3f(*_SnapTranslation(base, delta, snapStep, snapAbsolute))
        self._WriteOp(UsdGeom.XformCommonAPI.OpPivot, 1, p, p=p)

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        pass

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        pass

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        pass


def _ConnectedAvar(prim, names):
    for name in names:
        attr = prim.GetAttribute(name)
        if attr and attr.HasAuthoredConnections():
            return name
    return None


# The only rig prims the gizmo edits (design spec assumption 1.3). Volume
# weights also inherit RigExecXformable, but they carry no scale avars
# and have their own panel, so they are declined by concrete type rather
# than through IsRigXformable.
RIG_TARGET_TYPE_NAMES = ("RigExecControl", "RigExecJoint")


def _IsRigTargetType(prim):
    for name in RIG_TARGET_TYPE_NAMES:
        schemaType = Tf.Type.FindByName(name)
        if schemaType.isUnknown:
            # No schema plugin registered: fall back to the type name,
            # matching IsRigXformable's own fallback.
            if prim.GetTypeName() == name:
                return True
        elif prim.IsA(schemaType):
            return True
    return False


def MakeTarget(stage, prim, channels, writer, solverPosed=None,
               frameCache=None):
    """
    (target, "") or (None, reason) for usdview's focus prim.

    `solverPosed` is an optional SolverPosedCache, shared with the
    target so a Refresh() reuses the rig walk instead of repeating it.

    `frameCache` is the {path: RigFrames} dict ComputeRigFrames threads
    through its own recursion. One is made here when none is given, so
    the refusal check below and the target's own first Refresh() cost
    ONE walk rather than two; MakeGroupTarget passes one in so the whole
    selection costs one. Measured on examples/biped/Biped.usda, building
    targets for {fk shoulder, fk elbow, fk wrist, M_BodyGimbal} is
    262.5 ms of walking with a cache each and 120.1 ms sharing one --
    and a selection change runs this.
    """
    if not prim or not prim.IsValid():
        return None, "nothing selected"
    frameCache = {} if frameCache is None else frameCache
    if IsRigXformable(prim):
        if not _IsRigTargetType(prim):
            if not ReadsScaleAvars(prim):
                return None, ("%s is a volume weight; use the Volume "
                              "Weight panel" % prim.GetName())
            return None, "%s (%s) is not a control or a joint" % (
                prim.GetName(), prim.GetTypeName() or "untyped")
        posed = (solverPosed.For(FindRigRoot(prim))
                 if solverPosed is not None else None)
        frames = ComputeRigFrames(stage, prim, writer.time, posed,
                                  frameCache)
        # Pose answers to `reason`. Pivot edits rest:t/r, which no solver
        # and no posed:space takes away -- a TwoBoneIk measures its bone
        # lengths FROM the bound joints' rest frames -- so it answers to
        # `pivotReason` alone: the causes that leave the frames unusable
        # and a default space that selects the pivot independently of rest.
        blocking = (frames.pivotReason if channels == CHANNELS_PIVOT
                    else frames.reason)
        if blocking:
            return None, blocking
        names = (AVAR_T + AVAR_R + AVAR_S if channels == CHANNELS_POSE
                 else REST_T + REST_R)
        connected = _ConnectedAvar(prim, names)
        if connected:
            return None, "%s.%s is connected; edit its source instead" % (
                prim.GetName(), connected)
        if channels == CHANNELS_PIVOT:
            return RigPivotTarget(stage, prim, writer, solverPosed,
                                  frameCache), ""
        return RigPoseTarget(stage, prim, writer, solverPosed,
                             frameCache), ""
    if prim.IsA(UsdGeom.Xformable):
        if not UsdGeom.XformCommonAPI(prim):
            return None, ("%s: xformOp stack is not XformCommonAPI-"
                          "compatible" % prim.GetName())
        if channels == CHANNELS_PIVOT:
            return XformPivotTarget(stage, prim, writer), ""
        return XformPoseTarget(stage, prim, writer), ""
    return None, "%s (%s) has no transform to edit" % (
        prim.GetName(), prim.GetTypeName() or "untyped")


# Group edits: one gizmo over several selected controls
# How small a remainder counts as nothing. Degrees for a turn, world
# units for a move: a member already carried into place by a selected
# ancestor lands within floating-point noise of where it belongs, and
# authoring that noise would put a key on every control in the chain.
_GROUP_EPSILON_DEGREES = 1e-9

_GROUP_EPSILON_LENGTH = 1e-9


def _IsIdentity(matrix, tolerance=1e-12):
    return all(abs(matrix[r][c] - _IDENTITY[r][c]) <= tolerance
               for r in range(4) for c in range(4))


def _Centroid(points):
    total = Gf.Vec3d(0.0, 0.0, 0.0)
    for point in points:
        total += Gf.Vec3d(point)
    return total / float(len(points)) if points else total


class GroupTarget(Target):
    """
    One gizmo for several selected controls, moved and turned together.

    THE PIVOT IS THE CENTROID of the members' evaluated origins, and a
    rotate or a scale turns the whole selection about it. That is the
    default and the point of the feature: "rotate these together" means
    about their middle, not about whichever control was clicked last.
    SetPivotMode() offers the other two a application has -- GROUP_PIVOT_LEAD
    (the last-selected control's origin) and GROUP_PIVOT_INDIVIDUAL
    (each about its own) -- but neither is what a fresh selection does.

    THE AXES ARE A SEPARATE QUESTION, and the controller answers it, not
    this class: gizmoUI._Orientation reads the per-tool Axis Orientation
    (Global/World, Local/Object, Parent, Gimbal) and feeds the frame to
    gizmoScreen. ObjectFrame / ChannelFrame / GimbalFrame below answer
    with the LEAD control's frames moved onto the group pivot, which is
    the usual meaning of "Local" for a multi-selection -- an
    orientation averaged over the members points nowhere in particular,
    and the first-selected control's frame would move the gizmo whenever
    the artist added to the selection from the other end.

    THE DELTA IS EXPRESSED PER MEMBER, and it has to be. The members sit
    at different depths and in different spaces -- a finger control's
    avars are relative to a wrist-constrained root, a hips control's to
    the asset -- so one world delta is one set of channel values only
    after it has been inverted through that member's OWN channel frame.
    There is no second copy of that maths here: every member's
    ApplyTranslate already does exactly that inversion, and the group
    calls it. What the group adds is the ORBIT -- a member away from the
    pivot must translate as well as turn -- and the REMAINDER.

    THE REMAINDER IS THE WHOLE DESIGN (_Solve). Each member is given
    its intended world motion MINUS whatever a selected ancestor has
    already carried it through. A member already exactly where it
    belongs gets an identity remainder and authors nothing; a member
    half-way there authors the difference; a member with no selected
    ancestor authors all of it. One code path, no modes, and no rule
    about hierarchies -- which matters because the hierarchy is OUR
    business and not the animator's. This rig nests its FK controls
    (rigExec:controlSpace = "parentRelative") where another rig would
    constrain them, and an earlier version SKIPPED a member whose
    ancestor was also selected. That skip was right for a rigid pivot,
    wrong for Individual Origins, and in both cases it let our wiring
    decide what a selection does: measured on Biped.usda, a three-
    control selection of any FK chain -- fingers, arm, leg -- reached
    exactly one member in every pivot mode.

    WHY THE TURN AND THE ORBIT DO NOT INTERFERE. Every composition in
    this module puts the translation LAST (avars = S * R * Rspin * T,
    restLocal = compose(rest:t, rest:r), and XformCommonAPI's translate
    op, whose gizmo origin is parentWorld.Transform(pivot + translate)),
    so in row-vector form a target's own origin is its translation
    channel carried by its channel frame and NOTHING else. Rotating or
    scaling a member therefore cannot move its own origin, and the orbit
    delta can be computed once from the drag base instead of being
    re-measured after the turn -- which would have cost a rig walk per
    member per mouse sample. Each Apply* recomputes from BeginDrag()'s
    base, so the two writes compose rather than accumulate.

    WHAT IS DELIBERATELY NOT HERE. Preserve Children is a single-target
    option: with several members the compensation would have to agree
    about children that two of them both claim, and nothing asks for it.
    Gimbal rings are declined the same way (see RotationState): a gimbal
    ring IS one Euler channel of one prim, and a group has no shared
    channel to put an angle on.
    """

    kind = "group"
    supportsPreserveChildren = False
    preserveChildrenReason = "Preserve Children edits one target at a time"

    def __init__(self, members, lead, writer, skipped=None):
        Target.__init__(self, lead.stage, lead.prim, writer)
        self.members = list(members)
        self.lead = lead
        # [(prim name, reason)] for everything in the selection the
        # gizmo will not touch, so the status line can say so rather
        # than letting a control sit still and look broken.
        self.skipped = list(skipped or [])
        self.label = ("%d controls" % len(self.members)
                      if len(self.members) != 1 else self.members[0].label)
        # ANY, not all: a member that cannot scale is skipped by
        # ApplyScale, and greying out the whole tool because one member
        # in eight is a pivot target would be worse than dropping it.
        self.supportsTranslate = any(m.supportsTranslate for m in members)
        self.supportsRotate = any(m.supportsRotate for m in members)
        self.supportsScale = any(m.supportsScale for m in members)
        self.pivotMode = GROUP_PIVOT_CENTER
        self._origins = []
        self._order = []
        self._ancestor = []
        self._pivot = Gf.Vec3d(0.0, 0.0, 0.0)
        self._frame = Gf.Matrix4d(1.0)
        self._notes = []
        self.axisOrientation = "object"
        self._axes = Gf.Matrix4d(1.0)
        self._memberAxes = []

    def SetAxisOrientation(self, orientation):
        """
        Which frame the handles are laid out on: "world", "object",
        "parent" or "gimbal" (gizmoSettings.ORIENT_*, restated as plain
        strings so this module stays free of gizmoSettings).

        Only Individual Origins reads it. A drag along the group's X handle
        then moves or scales every member along ITS OWN matching axis, and
        "its own matching axis" is that member's frame of the same kind:
        its object frame when the handles are on the lead's object frame,
        its parent frame for Parent, and the world for World. Gimbal has
        no group form (RotationState is None) and reads as Object.
        """
        self.axisOrientation = orientation or "object"

    def _AxesOf(self, target):
        """The rotation `target`'s handles would use for axisOrientation."""
        if self.axisOrientation == "world":
            return Gf.Matrix4d(1.0)
        if self.axisOrientation == "parent":
            return _RotationOnly(target.ChannelFrame())
        return _RotationOnly(target.GizmoMatrix())

    def SetPivotMode(self, mode):
        """
        Choose what a rotate or a scale turns about (GROUP_PIVOT_*).

        Unknown values fall back to the centre rather than raising: this
        is fed from a settings field a stale panel can write, and the
        centre is the answer that is never surprising.
        """
        self.pivotMode = (mode if mode in GROUP_PIVOT_MODES
                          else GROUP_PIVOT_CENTER)

    # -- frames ---------------------------------------------------------

    def Origins(self):
        return [m.GizmoMatrix().ExtractTranslation() for m in self.members]

    def Pivot(self):
        """
        Where the gizmo is drawn, and -- except in Individual mode --
        what a rotate or a scale turns about.

        Individual origins draw at the centroid too. There is one
        manipulator and it has to be somewhere; the editor puts it on the
        median for this mode as well, and putting it on one member would
        say that member was special when the whole point is that none of
        them is.
        """
        if self.pivotMode == GROUP_PIVOT_LEAD:
            return Gf.Vec3d(self.lead.GizmoMatrix().ExtractTranslation())
        return _Centroid(self.Origins())

    def GizmoMatrix(self):
        return _FrameAt(self.lead.GizmoMatrix(), self.Pivot())

    def ObjectFrame(self):
        return self.GizmoMatrix()

    def ChannelFrame(self):
        return _FrameAt(self.lead.ChannelFrame(), self.Pivot())

    def GimbalFrame(self):
        return _FrameAt(self.lead.GimbalFrame(), self.Pivot())

    def RotationState(self):
        # None disables the Gimbal orientation for a real group, which
        # is what makes gizmoDrag take the world-axis route: _Orientation
        # falls back to Object when this is None, and DragState then has
        # no gimbal base to quantise against. A group of ONE is not a
        # group -- it keeps its member's rings.
        if len(self.members) == 1:
            return self.members[0].RotationState()
        return None

    def RigRootPath(self):
        return self.lead.RigRootPath()

    def Refresh(self, frameCache=None):
        """
        Re-read every member against ONE rig walk.

        The members of a real group share most of their ancestry -- the
        biped's three FK arm controls share five frame providers out of
        six -- and ComputeRigFrames recomposes the whole chain for each
        prim asked about unless it is handed the cache it uses
        internally. Measured on Biped.usda, one refresh of
        {fk shoulder, fk elbow, fk wrist, M_BodyGimbal}: 262.5 ms with
        a walk each, 120.1 ms sharing one cache -- and this runs on
        every mouse sample of a drag, so it is 2.2x of the whole
        interactive budget, not of a setup step.

        The cache is built FRESH here every time and kept by nobody: the
        frames it holds are composed from the preview values of one
        sample (gizmoMath._previewValues), and reusing it across samples
        would redraw the manipulator on the previous sample's pose.
        """
        cache = {} if frameCache is None else frameCache
        for member in self.members:
            member.Refresh(cache)

    def Advisory(self):
        notes = ["%s: %s" % (name, reason) for name, reason in self.skipped]
        notes.extend(self._notes)
        return "; ".join(notes)

    def AttributePaths(self):
        """
        Every member's channels, in selection order, de-duplicated.

        ONE list for ONE undo entry: the controller hands this to a
        single rigExecUndo.EditRecorder at the press, so the whole group
        drag takes back in one step however many controls it moved.
        """
        paths = []
        seen = set()
        for member in self.members:
            for path in member.AttributePaths():
                if path not in seen:
                    seen.add(path)
                    paths.append(path)
        return paths

    def SetPreserveChildren(self, enabled):
        Target.SetPreserveChildren(self, False)

    # -- drag -----------------------------------------------------------

    def BeginDrag(self, frameCache=None):
        cache = {} if frameCache is None else frameCache
        for member in self.members:
            member.BeginDrag(cache)
        self._origins = self.Origins()
        self._pivot = Gf.Vec3d(self.Pivot())
        self._frame = _RotationOnly(self.lead.GizmoMatrix())
        self._axes = self._AxesOf(self.lead)
        self._memberAxes = [self._AxesOf(m) for m in self.members]
        self._order, self._ancestor = self._Hierarchy()
        self._notes = []

    def _Note(self, text):
        """
        A one-sentence explanation of something a drag could NOT do,
        for Advisory() and the status line. De-duplicated: these are
        raised per mouse sample, and the artist wants the fact once.
        """
        if text not in self._notes:
            self._notes.append(text)

    def _Hierarchy(self):
        """
        (order, ancestor): the member indices SHALLOWEST FIRST, and each
        member's nearest selected ancestor (an index, or None).

        Ordered by how many selected ancestors a member has, which is a
        topological order for a forest and needs no graph. Computed once
        at the press -- it walks PoseProviderChain per member, the same
        walk the old driver filter did -- and then the whole drag is
        arithmetic.
        """
        byPath = dict((m.prim.GetPath(), i)
                      for i, m in enumerate(self.members))
        ancestor = []
        depth = []
        for member in self.members:
            nearest = None
            count = 0
            for path in PoseProviderChain(member.prim, self.time):
                if path in byPath:
                    count += 1
                    if nearest is None:
                        nearest = byPath[path]
            ancestor.append(nearest)
            depth.append(count)
        order = sorted(range(len(self.members)), key=lambda i: depth[i])
        return order, ancestor

    def _Solve(self, intended):
        """
        Walk the members shallowest first and hand each one the
        REMAINDER of its intended motion: what is left after everything
        it has already inherited from a selected ancestor.

        `intended(index, origin, inherited, inheritedOrigin)` answers
        with the member's intended TOTAL world motion. Yields
        (member, remainder, inheritedLinear, inheritedOrigin,
        intendedOrigin).

        THE WHOLE POINT. A rig control's world frame is
        avars * C * parentWorld, with the parent's frame the RIGHTMOST
        factor, so moving an ancestor right-multiplies every descendant
        by the same world transform -- exactly, and for free. If a
        member's intended motion IS what it inherited, the remainder is
        the identity and it authors nothing; if it is not, the member
        authors the difference. No rule about hierarchies, no mode to
        get wrong: the two cases the old skip hard-coded both fall out
        of one subtraction.

        NO FRAME WALK PER MEMBER PER SAMPLE. `inherited` is composed
        from the ancestors' own intended motions, which this loop
        already holds, rather than re-derived from the stage --
        ComputeRigFrames costs 7.2 ms on a spine control, 49.7 on the FK
        shoulder and 120.0 on the FK wrist (measured on Biped.usda), and
        a drag samples at mouse rate. The price is that each member's
        own frames are STALE by exactly its inherited motion when its
        Apply* is called, which _Author corrects for in closed form.
        """
        applied = {}
        for index in self._order:
            member = self.members[index]
            origin = Gf.Vec3d(self._origins[index])
            parent = self._ancestor[index]
            inherited = (applied[parent] if parent is not None
                         else Gf.Matrix4d(1.0))
            inheritedOrigin = inherited.Transform(origin)
            total = intended(index, origin, inherited, inheritedOrigin)
            applied[index] = total
            yield (member, inherited.GetInverse() * total,
                   _Linear(inherited), inheritedOrigin,
                   total.Transform(origin))

    def _Author(self, member, remainder, inheritedLinear, inheritedOrigin,
                intendedOrigin):
        """
        Write one member's remainder through its OWN entry points.

        The remainder is a world motion of this member's frame, so it is
        two calls: turn the frame by the remainder's rotation (which
        leaves the origin alone -- every composition here puts the
        translation last), then move the origin the rest of the way.
        Each member's ApplyRotate / ApplyTranslate does the conversion
        into that member's own channel space, which is the one copy of
        that maths there is.

        THE COMPENSATION. Those entry points read the member's channel
        frame from self.frames, and _Solve does not refresh it, so when
        an ancestor has already moved they are working in a frame that
        is stale by exactly the ancestor's linear part, L. The member's
        live frame is Pw*L, so a world vector v handed over unchanged
        would land as v*L. Handing over v*L^-1 lands it as v -- exact,
        and free, against a refresh that would cost a rig walk per
        member per mouse sample. The rotation AXIS takes the same
        correction, from the same algebra: conjugating a row-vector
        rotation, R(n, a) becomes R(n*L^-1, a).
        """
        inverse = inheritedLinear.GetInverse()
        turn = _RotationOnly(remainder).ExtractRotation()
        angle = turn.GetAngle()
        if abs(angle) > _GROUP_EPSILON_DEGREES:
            if member.supportsRotate:
                member.ApplyRotate(
                    inverse.TransformDir(Gf.Vec3d(turn.GetAxis())), angle)
            else:
                self._Note("%s cannot rotate: it stays as it was rather "
                           "than turning with the group" % member.label)
        delta = Gf.Vec3d(intendedOrigin) - Gf.Vec3d(inheritedOrigin)
        if delta.GetLength() > _GROUP_EPSILON_LENGTH:
            if member.supportsTranslate:
                member.ApplyTranslate(inverse.TransformDir(delta))
            else:
                self._Note("%s cannot translate: it turns in place rather "
                           "than following the group pivot" % member.label)

    def _About(self, motion, pivot):
        """`motion` applied about `pivot` instead of about the origin."""
        to = Gf.Matrix4d(1.0).SetTranslate(-Gf.Vec3d(pivot))
        back = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(pivot))
        return to * motion * back

    def ApplyTranslate(self, worldDelta, *, snapStep=None,
                       snapAbsolute=False):
        """
        Every member ends up moved by the SAME world delta.

        Which ones AUTHOR it is not decided here: a member that a
        selected ancestor has already carried the whole way has an
        identity remainder and writes nothing, and that falls out of
        _Solve rather than out of a rule about hierarchies.

        Step Snap is quantised HERE, once, on the world delta, and the
        members are then handed an exact vector (snapStep=None). Letting
        each member quantise in its own channel space -- which is what
        the single-target path does, and rightly, since that is where
        the value lands -- would move the members by different amounts
        and tear the group apart the moment two of them sit in spaces
        rotated relative to each other, which on the biped is every
        pair.
        """
        delta = Gf.Vec3d(worldDelta)
        if (self.pivotMode == GROUP_PIVOT_INDIVIDUAL
                and len(self.members) > 1):
            self._TranslateIndividually(delta, snapStep)
            return
        if snapStep and len(self.members) > 1:
            base = [self._pivot[i] for i in range(3)]
            landed = _SnapTranslation(base, [delta[i] for i in range(3)],
                                      snapStep, snapAbsolute)
            delta = Gf.Vec3d(*landed) - self._pivot
            snapStep = None
        elif snapStep:
            member = self.members[0]
            if member.supportsTranslate:
                member.ApplyTranslate(delta, snapStep=snapStep,
                                      snapAbsolute=snapAbsolute)
            return
        move = Gf.Matrix4d(1.0).SetTranslate(delta)
        for entry in self._Solve(lambda *args: move):
            self._Author(*entry)

    def _TranslateIndividually(self, delta, snapStep):
        """
        Individual Origins for a move: every member moves the same amount
        along its OWN axes.

        The drag's world delta is read back as amounts along the handle
        frame (the lead's, for Object), and those same amounts are laid
        along each member's frame of the same kind. Dragging the X arrow
        of three fingers therefore slides each finger along its own X,
        rather than all three along the lead's. A member under a selected
        ancestor rides the ancestor's move and adds its own on top, as a
        rotation does in this mode.

        Step Snap quantises the amounts once, here, so every member lands
        on the same step.
        """
        local = self._axes.GetInverse().TransformDir(delta)
        if snapStep:
            local = Gf.Vec3d(*[round(local[i] / snapStep) * snapStep
                               for i in range(3)])
        moves = [Gf.Matrix4d(1.0).SetTranslate(axes.TransformDir(local))
                 for axes in self._memberAxes]

        def intended(index, origin, inherited, inheritedOrigin):
            return inherited * moves[index]

        for entry in self._Solve(intended):
            self._Author(*entry)

    def ApplyRotate(self, worldAxis, degrees, *, snapStep=None):
        """
        Turn the selection by the same world rotation, about whatever
        the group pivot mode says (gizmoMath.GROUP_PIVOT_*).

        Centre and Last Selected are RIGID motions of the whole
        selection about one point, so every member's intended motion is
        the same transform. Individual Origins is not rigid and is not
        meant to be: each member turns about where IT now is, on top of
        anything it inherited, so selecting a finger chain and dragging
        the ring curls the finger instead of swinging it.

        The angle is snapped once here so the members cannot land on
        different angles; each member's own ApplyRotate then takes it
        exactly (snapStep=None).
        """
        angle = _SnapValue(degrees, snapStep)
        rotation = _WorldRotation(worldAxis, angle)
        rigid = self._About(rotation, self._pivot)

        def intended(index, origin, inherited, inheritedOrigin):
            if self.pivotMode == GROUP_PIVOT_INDIVIDUAL:
                return inherited * self._About(rotation, inheritedOrigin)
            return rigid

        for entry in self._Solve(intended):
            self._Author(*entry)

    def ApplyRotateChannel(self, axisIndex, degrees, *, snapStep=None):
        """
        A gimbal ring on a group. Unreachable while RotationState()
        answers None for a real group -- the rings are then laid out on
        world or object axes and gizmoDrag takes ApplyRotate -- but a
        group of one still carries its member's rings, and a caller with
        its own handles can still arrive here. Both are served by
        turning the group about the named axis of the GROUP frame, which
        is the frame the ring the artist grabbed was drawn on.
        """
        if len(self.members) == 1:
            self.members[0].ApplyRotateChannel(axisIndex, degrees,
                                               snapStep=snapStep)
            return
        axis = _RotationOnly(self._frame).TransformDir(_AXES[axisIndex])
        self.ApplyRotate(axis, degrees, snapStep=snapStep)

    def ApplyScale(self, axisIndex, factor, *, snapStep=None):
        """
        Scale the selection about the group pivot, along the axes of the
        GROUP frame -- the frame the handles the artist grabbed were
        drawn on.

        Same remainder walk as the other two, with one difference worth
        naming: a scale is not a rigid motion, so a member whose
        ancestor was scaled has inherited that scale in its own channels
        already (the evaluator composes scale down the chain) and its
        remainder is the identity, exactly as it is for a rigid move.
        Where the remainder is NOT the identity the member takes the
        whole factor, because the factor is the same for every member of
        the group; only the orbit differs.
        """
        axes = _ScaleAxes(axisIndex)
        frame = _RotationOnly(self._frame)
        inverseFrame = frame.GetInverse()
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(Gf.Vec3d(*[factor if i in axes else 1.0
                                  for i in range(3)]))
        aligned = inverseFrame * scale * frame

        def intended(index, origin, inherited, inheritedOrigin):
            if self.pivotMode == GROUP_PIVOT_INDIVIDUAL:
                # Along the member's OWN axes, about its own origin.
                own = self._memberAxes[index] if self._memberAxes else frame
                along = own.GetInverse() * scale * own
                return inherited * self._About(along, inheritedOrigin)
            return self._About(aligned, self._pivot)

        for (member, remainder, inheritedLinear, inheritedOrigin,
             intendedOrigin) in self._Solve(intended):
            if _IsIdentity(remainder):
                continue
            if member.supportsScale:
                member.ApplyScale(axisIndex, factor, snapStep=snapStep)
            delta = Gf.Vec3d(intendedOrigin) - Gf.Vec3d(inheritedOrigin)
            if delta.GetLength() > _GROUP_EPSILON_LENGTH:
                if member.supportsTranslate:
                    member.ApplyTranslate(
                        inheritedLinear.GetInverse().TransformDir(delta))
                else:
                    self._Note("%s cannot translate: it scales in place "
                               "rather than following the group pivot"
                               % member.label)


def MakeGroupTarget(stage, prims, channels, writer, solverPosed=None):
    """
    (target, "") or (None, reason) for a MULTI-prim selection, `prims`
    in selection order with the LEAD last.

    Every prim goes through MakeTarget, so a group inherits every
    refusal the single-target path already makes -- and that is the
    whole of the "do not manipulate what the rig overwrites" rule. No
    second policy, and no second place for the two to disagree.

    Measured on examples/biped/Biped.usda, of its 116 controls:

      15  refused because a RigExecParentConstraint overwrites their
          frame (SolverPosedPaths / OVERWRITING_MOVER_TYPES -- the ten
          constrained finger roots, arm_?_params, leg_?_params and
          spine_mid_follow). Their avars are inert: a drag would author
          values the next evaluation throws away.
       2  refused for a connected avar (toeBend_?_fk.avars:rx).
      33  more, but ONLY in a session with no published-frame reader:
          headless, the descendants of those 15 inherit the refusal
          (_PreferPublishedFrame explains why a published frame breaks
          that inheritance), so the count is 50 under the tests and 17
          in a live usdview with RigExec active.

    A refused prim is left out of the group and NAMED in the target's
    Advisory(), rather than dropped in silence -- a control that sits
    still while its neighbours move reads as a bug from the viewport.

    A selection with one survivor still returns a GroupTarget, so those
    refusals stay visible; a group of one behaves exactly like the bare
    target it wraps -- its pivot is that member's own origin, its orbit
    deltas are zero, and RotationState() passes the member's rings
    through.
    """
    members = []
    skipped = []
    lead = None
    # ONE rig walk for the whole selection: the members share most of
    # their ancestry, and ComputeRigFrames recomposes the entire chain
    # per prim asked about unless it is handed this.
    frameCache = {}
    for prim in prims:
        target, reason = MakeTarget(stage, prim, channels, writer,
                                    solverPosed, frameCache)
        if target is None:
            name = prim.GetName() if prim and prim.IsValid() else "?"
            skipped.append((name, reason))
            continue
        members.append(target)
        # The lead is the last ACCEPTED prim, not simply the last one: a
        # selection whose focus prim is one of the refused controls still
        # has to orient its gizmo on something real.
        lead = target
    if not members:
        if skipped:
            # The LEAD's refusal, not the first one: it is the prim the
            # artist picked last, and the one the status line named
            # before the rest of the selection arrived.
            return None, skipped[-1][1]
        return None, "nothing selected"
    return GroupTarget(members, lead, writer, skipped), ""
