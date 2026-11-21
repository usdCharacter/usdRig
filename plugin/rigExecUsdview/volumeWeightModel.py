#
# RigExec usdview plugin: Qt-free authoring rules for volumetric weight
# objects (RigExecSphereWeight, RigExecPlaneWeight, RigExecCurveWeight
# and RigExecCombineWeight).
#
# The attribute widgets at the top of this file -- SetAtTime,
# AttributeValueMayHaveChanged, AttributeValueWidget and
# AttributeValueSelectWidget -- are adapted from Pixar's OpenUSD sample
# plugin at
#   extras/exec/examples/invertibleRigsExample/
#       invertibleRigsExampleUsdviewPlugin/plugin.py
# Copyright 2026 Pixar, licensed under the terms set forth in the
# LICENSE.txt file available at https://openusd.org/license.  The
# additions here are the 'sensitivity' knob on the munging line edit, the
# uniform-variability guard in SetAtTime, and Detach() so a rebuilt panel
# can revoke its notice keys before Qt deletes the C++ widgets.
#
# SetAtTime and AttributeValueMayHaveChanged live here; the widgets and
# Detach() stay in volumeWeightUI.py, which re-exports the names below.
# Nothing in this file touches Qt.
#
import os
import sys

from pxr import Gf, Sdf, Tf, Ts, Usd, UsdGeom

try:
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import rigExecUndo


# The concrete volume weight types this panel can create and edit.  The
# order is the order of the Create buttons.
VOLUME_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
    "RigExecCombineWeight",
)

# RigExecCombineWeight inherits RigExecWeightObject, not RigExecXformable,
# so it has no placement avars and no falloff of its own.  Keeping the
# distinction in one place stops every section of the panel from
# re-deriving it.
PLACED_WEIGHT_TYPE_NAMES = (
    "RigExecSphereWeight",
    "RigExecPlaneWeight",
    "RigExecCurveWeight",
)

# Anything that may legally appear in rigExec:inputWeights.  The two
# authored weight types predate the volumetric ones but fold identically.
WEIGHT_OBJECT_TYPE_NAMES = VOLUME_WEIGHT_TYPE_NAMES + (
    "RigExecStaticWeight",
    "RigExecDynamicWeight",
)

# The named analytic profiles EnsureDefaultSpline can bake into a spline.
# 'curve' is excluded: it is the token that *reads* the spline, so it is
# not itself a shape.
SPLINE_PROFILE_NAMES = (
    "linear",
    "smooth",
    "easeIn",
    "easeOut",
    "constant",
)


# Attribute authoring helpers (no Qt)

# --------------------------------------------------------------------
# Undo
#
# Every write in this file goes through SetAtTime or SetVisibleAtTime, so
# recording happens THERE rather than at two dozen call sites. The panel
# hands the shared stack to SetUndoStack when it opens.
#
# Recording() nests: an outer scope over a prim already covers whatever
# its attributes do inside, so an inner call folds into it instead of
# pushing a second entry. That is what makes "create a sphere weight" --
# which authors a prim, a relationship and half a dozen placement avars
# -- arrive as one Ctrl+Z, and it is also what lets a slider hold one
# scope open across a whole drag.
# --------------------------------------------------------------------

_UNDO = None
_OPEN = 0


def SetUndoStack(stack):
    """The stack Recording() pushes onto. None disables recording."""
    global _UNDO
    _UNDO = stack


class _Scope(object):
    def __init__(self, inner):
        self._inner = inner

    def __enter__(self):
        global _OPEN
        _OPEN += 1
        if self._inner is not None:
            self._inner.__enter__()
        return self

    def __exit__(self, excType, exc, tb):
        global _OPEN
        _OPEN -= 1
        if self._inner is not None:
            self._inner.__exit__(excType, exc, tb)
        return False


def Recording(stage, paths, label):
    """One undo entry covering `paths`, unless one is already open."""
    if _UNDO is None or stage is None or not paths or _OPEN:
        return _Scope(None)
    return _Scope(rigExecUndo.SpecScope(stage, paths, _UNDO, label))


def _AttributeScope(attr, label):
    """Recording() for one attribute, keyed on its OWNING PRIM.

    The prim, not the property: authoring a placement avar can create the
    attribute spec, and undoing has to remove the spec rather than leave
    an empty one behind. A prim snapshot does that and costs no more here.
    """
    path = attr.GetPath()
    return Recording(attr.GetStage(), [path.GetPrimPath()],
                     "%s %s" % (label, path.name))


def SetAtTime(attr, value, time):
    """
    Sets a value on an attribute at a specified time.

    If setting at the default time, a default value is written.  If time
    samples have already been authored on the attribute (which would
    override a spline), or if the attribute's value type isn't supported
    by splines, authors a time sample at the given time.  Otherwise the
    value is written as a knot in a Ts.Spline.  If no knot exists at that
    time, creates a new knot with curve interpolation for the segment
    that follows it.

    Adapted from Pixar's invertibleRigsExample usdview plugin.  The one
    behavioural addition is the uniform guard: half of what this panel
    edits is `uniform token` (falloffProfile, planeAxis, rangePolicy,
    drawMode), and a uniform attribute cannot legally carry
    a time sample, so those always go to the default.
    """
    with _AttributeScope(attr, "Set"):
        _SetAtTime(attr, value, time)


def _SetAtTime(attr, value, time):
    if attr.GetVariability() == Sdf.VariabilityUniform:
        attr.Set(value)
        return

    valueType = attr.GetTypeName().type

    # If we're setting at the default time, or if the attribute's value
    # type isn't supported by splines, or if the attribute already has
    # time samples, use Set to write to the default or to time samples.
    if (time.IsDefault() or attr.GetNumTimeSamples() > 0 or
            not Ts.Spline.IsSupportedValueType(valueType)):
        attr.Set(value, time)
        return

    # Otherwise, write the value as a knot on the attribute's spline.
    frame = time.GetValue()
    spline = attr.GetSpline()
    knot = spline.GetKnot(frame)
    if knot:
        knot.SetValue(value)
    else:
        knot = Ts.Knot(
            typeName=valueType.typeName, time=frame, value=value,
            nextInterp=Ts.InterpCurve)
    spline.SetKnot(knot)
    # One notice: a first key creates the spec and then sets the spline.
    with Sdf.ChangeBlock():
        attr.SetSpline(spline)


def SetVisibleAtTime(attr, value, time=None):
    """
    Authors value so that it is what the attribute actually RESOLVES to
    at `time`, and writes the plainest scene description that achieves
    that.

    This exists because usdview's currentFrame is never the default:
    appController.py:1409 sets it to Usd.TimeCode(0.0) for a stage with
    no time samples and to the first sample otherwise.  So the moment an
    artist scrubs avars:tx, SetAtTime gives that attribute a spline knot
    at frame 0 -- and a spline outranks a default in value resolution.
    A later attr.Set(value), which writes the default, would then be
    completely invisible: "Snap to selection" would silently do nothing.

    So: write the default while the attribute is still a plain default
    (a freshly created weight, which should not be born animated), and
    write at `time` once the attribute has a spline or time samples to
    outrank.
    """
    if attr is None or not attr:
        return False

    with _AttributeScope(attr, "Set"):
        return _SetVisibleAtTime(attr, value, time)


def _SetVisibleAtTime(attr, value, time):
    if time is None or time.IsDefault():
        attr.Set(value)
        return True

    hasSpline = attr.HasSpline() if hasattr(attr, "HasSpline") else False
    if hasSpline or attr.GetNumTimeSamples() > 0:
        SetAtTime(attr, value, time)
    else:
        attr.Set(value)
    return True


def AttributeValueMayHaveChanged(attrPath, notice):
    """
    Returns True if the given notice indicates that the value of the
    attribute at attrPath may have changed.

    Adapted from Pixar's invertibleRigsExample usdview plugin.
    """
    if attrPath in notice.GetChangedInfoOnlyPaths():
        return True

    for path in notice.GetResyncedPaths():
        if attrPath.HasPrefix(path):
            return True

    return False


# Falloff spline authoring (no Qt)
#
# The falloff spline is a shape over the NORMALIZED band parameter, not
# over time: x = 0 is the outer end of the band (inputs:falloffMax) and
# x = 1 is the inner end (inputs:falloffMin).  Ts.Spline.Eval() is happy
# at arbitrary x and extrapolates held outside the authored knots, which
# is exactly the saturating behaviour a falloff wants, so the engine can
# evaluate it without clamping first.

def _MakeKnot(x, y, interp=None, preSlope=0.0, preWidth=0.0,
              postSlope=0.0, postWidth=0.0):
    """
    Builds a float-typed Ts.Knot at (x, y).

    Ts.Spline() with no arguments defaults to a DOUBLE value type and
    rejects float knots, so every knot this module makes is constructed
    with an explicit float typeName to match rigExec:falloffCurve.
    """
    knot = Ts.Knot(
        typeName="float", time=float(x), value=float(y),
        nextInterp=Ts.InterpCurve if interp is None else interp)
    knot.SetPreTanSlope(float(preSlope))
    knot.SetPreTanWidth(float(preWidth))
    knot.SetPostTanSlope(float(postSlope))
    knot.SetPostTanWidth(float(postWidth))
    return knot


def _MakeProfileSpline(profile):
    """
    Returns a fresh float Ts.Spline carrying the named analytic profile.

    The curved profiles are shaped with TANGENTS rather than with extra
    knots: two knots the artist can grab is a far friendlier starting
    point than a six-knot approximation of a smoothstep, and the bezier
    reproduces the analytic shape closely enough that switching profile
    to `curve` is visually a no-op.
    """
    spline = Ts.Spline("float")

    if profile == "linear":
        spline.SetKnot(_MakeKnot(0.0, 0.0, Ts.InterpLinear))
        spline.SetKnot(_MakeKnot(1.0, 1.0, Ts.InterpLinear))
    elif profile == "smooth":
        # Zero slope at both ends with 1/3 widths is smoothstep to
        # within a thousandth: 0.157 / 0.500 / 0.843 at the quarters.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postWidth=0.33))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preWidth=0.33))
    elif profile == "easeIn":
        # Flat departure, steep arrival: the whole curve sits below the
        # diagonal, so the field stays near zero well into the band.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postWidth=0.55))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preSlope=1.5, preWidth=0.2))
    elif profile == "easeOut":
        # The mirror of easeIn: steep departure, flat arrival, so the
        # curve sits above the diagonal.
        spline.SetKnot(_MakeKnot(0.0, 0.0, postSlope=1.5, postWidth=0.2))
        spline.SetKnot(_MakeKnot(1.0, 1.0, preWidth=0.55))
    elif profile == "constant":
        # Held knots at full weight: the band is a hard region rather
        # than a ramp, which is what a mask wants.
        spline.SetKnot(_MakeKnot(0.0, 1.0, Ts.InterpHeld))
        spline.SetKnot(_MakeKnot(1.0, 1.0, Ts.InterpHeld))
    else:
        raise ValueError("unknown falloff profile %r" % (profile,))

    return spline


def EnsureDefaultSpline(attr, profile, force=False):
    """
    Authors a starting spline for the named profile onto a float
    attribute (rigExec:falloffCurve).

    Returns the spline that is now on the attribute.  With force=False
    an existing spline that already has at least two knots is left
    alone, so merely opening the panel never stomps an artist's curve;
    the preset buttons pass force=True because overwriting is the whole
    point of pressing one.
    """
    if attr is None or not attr:
        raise ValueError("EnsureDefaultSpline needs a valid attribute")

    if not force:
        existing = attr.GetSpline()
        if existing and len(existing.GetKnots()) >= 2:
            return existing

    spline = _MakeProfileSpline(profile)
    attr.SetSpline(spline)
    return attr.GetSpline()


def GetSplineKnotPoints(attr):
    """
    Returns the attribute's spline knots as a list of (x, y) tuples
    sorted by x.

    Ts.Spline.GetKnots() returns a KnotMap that iterates as float TIMES,
    not as knot objects, so the times have to be turned back into knots
    one at a time.
    """
    if attr is None or not attr:
        return []

    spline = attr.GetSpline()
    if not spline:
        return []

    points = []
    for time in spline.GetKnots():
        knot = spline.GetKnot(time)
        if knot:
            points.append((float(knot.GetTime()), float(knot.GetValue())))
    points.sort(key=lambda point: point[0])
    return points


def InsertSplineKnot(attr, x, y):
    """
    Inserts a curve-interpolated knot at (x, y), clamped into the unit
    square.  Returns the x the knot actually landed on.
    """
    x = min(max(float(x), 0.0), 1.0)
    y = min(max(float(y), 0.0), 1.0)

    spline = attr.GetSpline()
    if not spline:
        spline = Ts.Spline("float")
    spline.SetKnot(_MakeKnot(x, y))
    attr.SetSpline(spline)
    return x


def MoveSplineKnot(attr, oldX, newX, newY, pinEndpoints=True):
    """
    Moves the knot at oldX to (newX, newY), clamped into the unit
    square.  Returns the x the knot ended up at, or None if there was no
    knot at oldX.

    With pinEndpoints the first and last knots keep their x: they define
    where the band starts and ends, and letting them slide inward would
    silently change what falloffMin/falloffMax mean.  Their y is still
    free, which is how an artist authors a ramp that never reaches full
    strength.
    """
    spline = attr.GetSpline()
    if not spline:
        return None

    knot = spline.GetKnot(float(oldX))
    if not knot:
        return None

    times = sorted(float(t) for t in spline.GetKnots())
    newX = min(max(float(newX), 0.0), 1.0)
    newY = min(max(float(newY), 0.0), 1.0)

    if pinEndpoints and times and float(oldX) in (times[0], times[-1]):
        newX = float(oldX)
    else:
        # Two knots at the same time is not a thing a KnotMap can hold;
        # nudge off any occupied slot rather than silently eating the
        # neighbour.
        for time in times:
            if time != float(oldX) and abs(time - newX) < 1e-6:
                newX = float(oldX)
                break

    if newX != float(oldX):
        spline.RemoveKnot(float(oldX))
        knot.SetTime(newX)
    knot.SetValue(newY)
    spline.SetKnot(knot)
    attr.SetSpline(spline)
    return newX


def DeleteSplineKnot(attr, x, minimumKnots=2):
    """
    Removes the knot at x.  Returns True if it was removed.

    Refuses to go below minimumKnots: a spline with one knot evaluates
    to a constant everywhere and there is no way back to a ramp by
    dragging, so the floor keeps the editor usable.
    """
    spline = attr.GetSpline()
    if not spline:
        return False

    times = sorted(float(t) for t in spline.GetKnots())
    if len(times) <= minimumKnots:
        return False
    if float(x) not in times:
        return False

    spline.RemoveKnot(float(x))
    attr.SetSpline(spline)
    return True


# Prim creation and placement (no Qt)

def FindRigPrim(stage):
    """
    Returns the first RigExecRoot prim on the stage, or None.

    Traversal by typeName rather than by schema type mirrors what
    rigExecUsdview.py already does to decide whether to engage: the
    generated schema may or may not be registered in this process, and
    the type name is authored either way.
    """
    if stage is None:
        return None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "RigExecRoot":
            return prim
    return None


def FindWeightsScopeAncestor(prim):
    """
    Returns the nearest ancestor (or the prim itself) that looks like a
    .../Rig/Weights scope, or None.

    'Looks like' is a name match on Weights with a RigExec-typed
    ancestor: rigs in the wild nest their weights differently, and the
    panel only needs somewhere sensible to hang a new prim.
    """
    if not prim:
        return None

    walker = prim
    while walker and not walker.IsPseudoRoot():
        if walker.GetName() == "Weights":
            ancestor = walker.GetParent()
            while ancestor and not ancestor.IsPseudoRoot():
                if str(ancestor.GetTypeName()).startswith("RigExec"):
                    return walker
                ancestor = ancestor.GetParent()
            return walker
        walker = walker.GetParent()
    return None


def ChooseWeightParentPath(stage, selectedPrims):
    """Where ChooseWeightParentPrim WILL put a new weight, creating nothing.

    The undo scope has to name a path before the thing at it exists, and
    ChooseWeightParentPrim defines the Weights scope as a side effect of
    answering -- so the answer is separated from the act here, and the
    two share this function to keep them from drifting.
    """
    for prim in selectedPrims or ():
        if prim and prim.GetTypeName() == "RigExecJoint":
            return prim.GetPath(), True
    for prim in selectedPrims or ():
        scope = FindWeightsScopeAncestor(prim)
        if scope:
            return scope.GetPath(), True
    rig = FindRigPrim(stage)
    path = (rig.GetPath().AppendChild("Weights") if rig
            else Sdf.Path("/Weights"))
    return path, bool(stage.GetPrimAtPath(path))


def ChooseWeightParentPrim(stage, selectedPrims):
    """
    Decides where a newly created volume weight should live and returns
    that prim, defining it if necessary.

    The order is deliberately "obey the artist first":
      1. a selected RigExecJoint -- weights authored while a joint is
         selected almost always belong to that joint, and parenting them
         there means the joint's transform carries the volume;
      2. a selected prim already inside a .../Rig/Weights scope -- reuse
         the scope the artist is clearly working in;
      3. a Weights scope under the stage's RigExecRoot, created on
         demand;
      4. /Weights as a last resort on a stage with no rig at all.
    """
    for prim in selectedPrims or ():
        if prim and prim.GetTypeName() == "RigExecJoint":
            return prim

    for prim in selectedPrims or ():
        scope = FindWeightsScopeAncestor(prim)
        if scope:
            return scope

    path, existed = ChooseWeightParentPath(stage, selectedPrims)
    return (stage.GetPrimAtPath(path) if existed
            else stage.DefinePrim(path, "Scope"))


def MakeUniquePrimName(parentPrim, baseName):
    """
    Returns baseName, or baseName with the lowest free integer suffix.
    """
    if not parentPrim or not parentPrim.GetChild(baseName):
        return baseName
    index = 1
    while parentPrim.GetChild("%s%d" % (baseName, index)):
        index += 1
    return "%s%d" % (baseName, index)


def DefaultPrimNameForType(typeName):
    """
    Returns the base prim name used for a newly created weight type,
    e.g. RigExecSphereWeight -> SphereWeight.
    """
    if typeName.startswith("RigExec"):
        return typeName[len("RigExec"):]
    return typeName


def FindPointsPropertyPath(prim):
    """
    Returns the .points property path of a point-based prim, or None.

    rigExec:weightTarget names the exact property being weighted rather
    than the prim, so the field and the points it grabs cannot drift
    apart when a prim carries more than one point set.
    """
    if not prim:
        return None
    if not UsdGeom.PointBased(prim):
        return None
    return prim.GetPath().AppendProperty(UsdGeom.Tokens.points)


def ComputeCentreAndSize(prim, time=None):
    """
    Returns (Gf.Vec3d centre, float size) for a prim: the midpoint of
    its world bound and the largest axis of that bound.

    Falls back to the prim's world-space origin with a unit size when
    the prim has no computable extent, and to the world origin when
    there is no prim at all, so callers never have to special-case an
    empty selection.
    """
    if time is None:
        time = Usd.TimeCode.Default()

    if not prim:
        return Gf.Vec3d(0.0, 0.0, 0.0), 1.0

    # Guides are included so that snapping to an existing weight volume
    # (whose purpose is "guide") lands on the volume, not on the origin.
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide]
    try:
        bboxCache = UsdGeom.BBoxCache(time, purposes)
        bound = bboxCache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not bound.IsEmpty():
            centre = Gf.Vec3d(bound.GetMidpoint())
            extent = bound.GetSize()
            size = max(float(extent[0]), float(extent[1]),
                       float(extent[2]))
            if size > 0.0:
                return centre, size
            return centre, 1.0
    except Tf.ErrorException:
        pass

    try:
        xformCache = UsdGeom.XformCache(time)
        matrix = xformCache.GetLocalToWorldTransform(prim)
        return Gf.Vec3d(matrix.ExtractTranslation()), 1.0
    except Tf.ErrorException:
        return Gf.Vec3d(0.0, 0.0, 0.0), 1.0


def GetWeightTargetPrim(prim):
    """
    Returns the prim that owns the property rigExec:weightTarget points
    at, or None.
    """
    if not prim:
        return None
    relationship = prim.GetRelationship("rigExec:weightTarget")
    if not relationship:
        return None
    stage = prim.GetStage()
    for path in relationship.GetTargets():
        target = stage.GetPrimAtPath(path.GetPrimPath())
        if target:
            return target
    return None


def ComputeScrubScale(prim, time=None):
    """
    Returns the stage-unit scale a scrub on this weight's parameters
    should be measured against.

    Prefer the WEIGHTED GEOMETRY's size, not the weight guide's. Volume
    weights now have a registered authored-fallback extent so they remain
    framable before evaluation, but that extent changes with the very
    falloff/scale parameter being scrubbed. Feeding it back into sensitivity
    would make one continuous drag accelerate as the guide grows. The target
    instead supplies a stable scene scale -- and avoids the old resource-only
    fallback of 0.005 units per pixel on characters measured in tens of
    units. An unbound weight still falls back to its own guide size (or 1.0
    when no compute-extent plugin is available).
    """
    target = GetWeightTargetPrim(prim)
    if target is not None:
        _, size = ComputeCentreAndSize(target, time)
        if size > 1e-6:
            return float(size)

    _, size = ComputeCentreAndSize(prim, time)
    return float(size)


def SetPlacementFromPoint(prim, centre, time=None):
    """
    Authors avars:tx/ty/tz so a placed volume weight sits at centre.

    These are the animatable placement avars rather than xformOps: a
    RigExecXformable is posed through its avars, and authoring an
    xformOp here would put the volume somewhere the engine does not
    look.

    Authored through SetVisibleAtTime so that snapping a weight whose
    avars the artist has already scrubbed (and so turned into splines)
    actually moves it -- see that function.
    """
    for name, value in (("avars:tx", centre[0]),
                        ("avars:ty", centre[1]),
                        ("avars:tz", centre[2])):
        attr = prim.GetAttribute(name)
        if attr:
            SetVisibleAtTime(attr, float(value), time)


def SeedFalloffForSize(prim, size, time=None):
    """
    Authors inputs:falloffMin/falloffMax proportional to a target's
    bounding box.

    Without this a new sphere weight is a unit radius lost inside a
    character and the artist's first act is always the same corrective
    scrub.  A quarter of the largest bbox axis is a blob that reads
    immediately at character scale.

    A plane weight gets a SIGNED band straddling zero instead, because
    its distance is the signed coordinate along rigExec:planeAxis: the
    useful default there is a gradient across the plane, not a slab
    hugging one side of it.

    Its inputs:extentU/extentV are seeded here too, and have to be: the
    plane guide is sized by the extents alone now (it used to derive a
    half-size from the band), so a new plane on a character measured in
    tens of units would otherwise draw a one-unit square nobody can see.
    Half-extents of size/2 give a square spanning the target's largest
    bbox axis.
    """
    radius = max(float(size) * 0.25, 1e-4)

    minAttr = prim.GetAttribute("inputs:falloffMin")
    maxAttr = prim.GetAttribute("inputs:falloffMax")
    if prim.GetTypeName() == "RigExecPlaneWeight":
        SetVisibleAtTime(minAttr, float(-radius), time)
        SetVisibleAtTime(maxAttr, float(radius), time)
        extent = max(float(size) * 0.5, 1e-4)
        for name in ("inputs:extentU", "inputs:extentV"):
            SetVisibleAtTime(prim.GetAttribute(name), float(extent), time)
    else:
        SetVisibleAtTime(minAttr, 0.0, time)
        SetVisibleAtTime(maxAttr, float(radius), time)
    return radius


def CreateVolumeWeightPrim(stage, typeName, selectedPrims=None, time=None):
    """
    Creates one volume weight prim, wires it to the selection and
    returns it.

    Everything authored here lands in the stage's CURRENT EDIT TARGET,
    which in usdview is the session layer until the artist picks a
    different one -- see the note in VolumeWeightPanel.

    The rules, all of which exist so a freshly created volume is
    immediately useful rather than an invisible unit sphere at the
    origin:
      * parent scope per ChooseWeightParentPrim;
      * uniquely named after its type;
      * rigExec:weightTarget bound to the selected mesh's .points when
        exactly one point-based prim is selected;
      * placed at that prim's bbox centre through the placement avars;
      * falloff seeded from that prim's bbox size.
    """
    if typeName not in VOLUME_WEIGHT_TYPE_NAMES:
        raise ValueError("not a volume weight type: %r" % (typeName,))
    if time is None:
        time = Usd.TimeCode.Default()

    selectedPrims = [p for p in (selectedPrims or ()) if p]

    parent = ChooseWeightParentPrim(stage, selectedPrims)
    name = MakeUniquePrimName(parent, DefaultPrimNameForType(typeName))
    prim = stage.DefinePrim(parent.GetPath().AppendChild(name), typeName)

    # Exactly one point-based prim: unambiguous, so bind it.  Two would
    # be a guess, and a wrong weightTarget is worse than none.
    pointBased = [p for p in selectedPrims if UsdGeom.PointBased(p)]
    targetPrim = pointBased[0] if len(pointBased) == 1 else None

    if targetPrim is not None:
        pointsPath = FindPointsPropertyPath(targetPrim)
        relationship = prim.GetRelationship("rigExec:weightTarget")
        if relationship and pointsPath:
            relationship.SetTargets([pointsPath])

    # A curve weight is useless without a curve; if the artist selected
    # one alongside the mesh, bind it too.
    if typeName == "RigExecCurveWeight":
        curvePrims = [p for p in selectedPrims
                      if p.GetTypeName() == "BasisCurves" and
                      p is not targetPrim]
        if len(curvePrims) == 1:
            curveRel = prim.GetRelationship("rigExec:curve")
            curvePath = FindPointsPropertyPath(curvePrims[0])
            if curveRel and curvePath:
                curveRel.SetTargets([curvePath])

    # RigExecCombineWeight has no placement and no falloff of its own;
    # it only folds what its inputs already computed.
    if typeName in PLACED_WEIGHT_TYPE_NAMES:
        sizeSource = targetPrim if targetPrim is not None else (
            selectedPrims[0] if selectedPrims else None)
        centre, size = ComputeCentreAndSize(sizeSource, time)
        SetPlacementFromPoint(prim, centre, time)
        SeedFalloffForSize(prim, size, time)

    return prim


def SnapWeightToPrim(weightPrim, sourcePrim, time=None):
    """
    Moves a placed volume weight onto another prim's bbox centre.
    Returns the centre it moved to, or None when it could not.
    """
    if not weightPrim or weightPrim.GetTypeName() not in \
            PLACED_WEIGHT_TYPE_NAMES:
        return None
    if not sourcePrim:
        return None

    centre, _ = ComputeCentreAndSize(sourcePrim, time)
    SetPlacementFromPoint(weightPrim, centre, time)
    return centre


# Combine weight input list (no Qt)

def GetInputWeightPaths(prim):
    """
    Returns the ordered rigExec:inputWeights targets as Sdf.Paths.
    """
    if not prim:
        return []
    relationship = prim.GetRelationship("rigExec:inputWeights")
    if not relationship:
        return []
    return list(relationship.GetTargets())


def SetInputWeightPaths(prim, paths):
    """
    Replaces rigExec:inputWeights wholesale.  Order matters for the
    non-commutative combine modes (subtract, overlay), so the list is
    authored as a list rather than merged.
    """
    relationship = prim.GetRelationship("rigExec:inputWeights")
    if not relationship:
        return False
    return relationship.SetTargets([Sdf.Path(p) for p in paths])


def AddInputWeightPaths(prim, paths):
    """
    Appends paths to rigExec:inputWeights, skipping duplicates and the
    combine prim itself.  Returns the number appended.
    """
    current = GetInputWeightPaths(prim)
    existing = set(current)
    added = 0
    for path in paths:
        path = Sdf.Path(path)
        if path in existing or path == prim.GetPath():
            continue
        current.append(path)
        existing.add(path)
        added += 1
    if added:
        SetInputWeightPaths(prim, current)
    return added


def RemoveInputWeightPaths(prim, paths):
    """
    Removes paths from rigExec:inputWeights.  Returns the number
    removed.
    """
    doomed = set(Sdf.Path(p) for p in paths)
    current = GetInputWeightPaths(prim)
    kept = [p for p in current if p not in doomed]
    removed = len(current) - len(kept)
    if removed:
        SetInputWeightPaths(prim, kept)
    return removed


def IsVolumeWeightPrim(prim):
    """True for the four concrete types this panel edits."""
    return bool(prim) and prim.GetTypeName() in VOLUME_WEIGHT_TYPE_NAMES


def IsWeightObjectPrim(prim):
    """True for anything that may feed rigExec:inputWeights."""
    return bool(prim) and prim.GetTypeName() in WEIGHT_OBJECT_TYPE_NAMES
