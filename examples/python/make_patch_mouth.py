#!/usr/bin/env python3
"""Build examples/18_PatchMouth.usda: two lip curves on one NURBS patch.

The upper and lower lip are the two curves of one BasisCurves, parted a
little at rest and meeting at the corners. One RigExecPatchMover sticks
them to a flat NURBS patch and mixes 27 UV shapes into their UV points, one
per ARKit jaw and mouth blend shape, weighted by the Mouth control's avars
of the same names.

The lips never cross, under any mix of weights:

  * Point i of the upper lip and point i of the lower lip form one column:
    every shape moves both by the same u, so every column keeps one u.
  * The mover's contact pairs (rigExec:uvContactLower/Upper) pair the two
    points of each column, so the lower one never stands above the upper
    one in v: past contact the two meet at their midpoint. That is what
    lets mouthClose close the lips at any jawOpen: it closes the rest gap
    and jawOpen's gap, half from each side, and lips it closes past each
    other meet instead.
  * Within a lip, u never folds: u is linear in the weights, so the
    narrowest a column spacing can get over all weights in [0, 1] is the
    neutral spacing plus every shape's narrowing at once. build() asserts
    that margin, and that every column stays on the patch.

The patch is a bilinear plane at z = 0, with x linear in u and y linear in
v, so seen from the front each lip is a function of x and the upper lip is
never below the lower one.

Usage:
    python make_patch_mouth.py [output.usda]

The output defaults to examples/18_PatchMouth.usda. Only pxr is needed: run
it with the environment bin/_env.sh sets up, from the repository root:

    . bin/_env.sh && "$PY" examples/python/make_patch_mouth.py
"""

import pathlib
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

ROOT = "/MouthAsset"
RIG = ROOT + "/Rig"
CONTROL = RIG + "/Controls/Mouth"
MOVER = RIG + "/Movers/Geometry/Lips"
GEOM = ROOT + "/Geom"
PATCH = GEOM + "/Face"
SKIN = GEOM + "/Skin"
LIPS = GEOM + "/Lips"
LIP_UVS = GEOM + "/LipUVs"
SHAPES = GEOM + "/LipShapes"

# The patch: a bilinear plane (order 2, 2x2 CVs) over x in [-X, X] and
# y in [Y0, Y1] (cm), at z = 0.
X = 5.5
Y0, Y1 = -4.5, 4.0

# The lips: N columns from the character's right corner (p = -1, screen
# left) to the left corner (p = +1). At rest they sit PART above and below
# a seam at the middle column and meet at the corners.
N = 17
HALF_WIDTH = 2.4
SEAM_DROP = 0.06
PART = 0.2


def to_uv(x, y):
    return ((x + X) / (2.0 * X), (y - Y0) / (Y1 - Y0))


def patch_cvs():
    """The 4 CVs, u-fastest (rows of constant v)."""
    return [(x, y, 0.0) for y in (Y0, Y1) for x in (-X, X)]


def patch_point(u, v):
    """The patch at (u, v): exactly the surface the mover evaluates."""
    return (-X + 2.0 * X * u, Y0 + (Y1 - Y0) * v, 0.0)


# Shape profiles over a column's p in [-1, 1]. Each is zero at both corners
# where a shape opens the lips, so the corner columns stay closed.

def _smoothstep(e0, e1, x):
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def lens(p):
    """The opening of a parted mouth: 1 at the middle, 0 at the corners."""
    return (1.0 - p * p) ** 0.75


def rounded(p):
    """A rounder opening, for the funnel's O."""
    return (1.0 - p * p) ** 0.5


def corner(p):
    """The character's left corner: 1 there, 0 at the right corner."""
    return ((1.0 + p) / 2.0) ** 2.5


def cheek(p):
    """A broader left-corner falloff, for lateral pulls."""
    return ((1.0 + p) / 2.0) ** 1.5


def side(p):
    """The character's left half."""
    return _smoothstep(-0.7, 0.7, p)


def mirrored(shape):
    """The right-side twin of a left-side shape."""
    def right(p):
        dx, up, lo = shape(-p)
        return (-dx, up, lo)
    return right


# Each shape is p -> (dx, upper dy, lower dy) in cm, added to the neutral.
# dx is shared by both lips, so every column keeps one u.
def _jaw_left(p):
    return (0.5 - 0.15 * p * p, 0.0, -0.15 * lens(p))


def _mouth_left(p):
    return (0.6, 0.06 * p, 0.06 * p)


def _smile_left(p):
    return (0.45 * corner(p), 0.62 * corner(p), 0.62 * corner(p))


def _frown_left(p):
    return (0.15 * corner(p), -0.55 * corner(p), -0.55 * corner(p))


def _dimple_left(p):
    return (0.4 * cheek(p), 0.08 * corner(p), 0.08 * corner(p))


def _stretch_left(p):
    return (0.6 * cheek(p), -0.28 * corner(p), -0.28 * corner(p))


def _press_left(p):
    return (0.08 * corner(p), -(PART + 0.1) * lens(p) * side(p),
            (PART + 0.25) * lens(p) * side(p))


def _lower_down_left(p):
    return (0.04 * corner(p), 0.0, -0.75 * lens(p) * side(p))


def _upper_up_left(p):
    return (0.04 * corner(p), 0.62 * lens(p) * side(p), 0.0)


# jawOpen drops the corners by JAW_CORNER and parts the lips a further
# (JAW_UP + JAW_DOWN) lens(p). mouthClose closes the rest gap and jawOpen's,
# half from each side, so at any jawOpen up to 1 the lips meet at the mean
# of where they would be without it.
JAW_CORNER, JAW_UP, JAW_DOWN = 0.6, 0.6, 1.3
CLOSE = PART + 0.5 * (JAW_UP + JAW_DOWN)

# ARKit's jaw and mouth blend shapes, in Apple's documented order. The
# shapes that bring the lips together (mouthClose, mouthPucker, the rolls,
# mouthShrugLower, the presses) close the rest gap, PART from each lip.
SHAPE_TABLE = [
    ("jawForward", lambda p: (-0.08 * p, 0.05 * lens(p), -0.15 * lens(p))),
    ("jawLeft", _jaw_left),
    ("jawRight", mirrored(_jaw_left)),
    ("jawOpen", lambda p: (-0.2 * p, -JAW_CORNER + JAW_UP * lens(p),
                           -JAW_CORNER - JAW_DOWN * lens(p))),
    ("mouthClose", lambda p: (0.0, -CLOSE * lens(p), CLOSE * lens(p))),
    ("mouthFunnel", lambda p: (-0.55 * p, 0.38 * rounded(p),
                               -0.5 * rounded(p))),
    ("mouthPucker", lambda p: (-0.9 * p,
                               0.06 * (1.0 - p * p) - PART * lens(p),
                               0.06 * (1.0 - p * p) + PART * lens(p))),
    ("mouthLeft", _mouth_left),
    ("mouthRight", mirrored(_mouth_left)),
    ("mouthSmileLeft", _smile_left),
    ("mouthSmileRight", mirrored(_smile_left)),
    ("mouthFrownLeft", _frown_left),
    ("mouthFrownRight", mirrored(_frown_left)),
    ("mouthDimpleLeft", _dimple_left),
    ("mouthDimpleRight", mirrored(_dimple_left)),
    ("mouthStretchLeft", _stretch_left),
    ("mouthStretchRight", mirrored(_stretch_left)),
    ("mouthRollLower", lambda p: (-0.06 * p, 0.0,
                                  (2.0 * PART + 0.2) * lens(p))),
    ("mouthRollUpper", lambda p: (-0.06 * p, -(2.0 * PART + 0.2) * lens(p),
                                  0.0)),
    ("mouthShrugLower", lambda p: (0.0, 0.3 * lens(p),
                                   (2.0 * PART + 0.4) * lens(p))),
    ("mouthShrugUpper", lambda p: (0.0, 0.28 * lens(p), 0.2 * lens(p))),
    ("mouthPressLeft", _press_left),
    ("mouthPressRight", mirrored(_press_left)),
    ("mouthLowerDownLeft", _lower_down_left),
    ("mouthLowerDownRight", mirrored(_lower_down_left)),
    ("mouthUpperUpLeft", _upper_up_left),
    ("mouthUpperUpRight", mirrored(_upper_up_left)),
]
NAMES = [name for name, _ in SHAPE_TABLE]

# The animation: a tour of every shape alone, then poses that mix them.
# Each key pose names the avars it sets; the others are 0 there.
TOUR_START, TOUR_STEP = 1007, 6
MIX_START, MIX_STEP = 1179, 10
MIXES = [
    {"jawOpen": 1.0},
    {"jawOpen": 1.0, "mouthClose": 1.0},
    {"mouthClose": 1.0},
    {"jawOpen": 0.5, "mouthSmileLeft": 1.0, "mouthSmileRight": 1.0},
    {"mouthFrownLeft": 1.0, "mouthFrownRight": 1.0, "mouthShrugLower": 1.0},
    {"mouthPucker": 1.0, "mouthFunnel": 0.4},
    {"jawOpen": 0.35, "mouthFunnel": 1.0},
    {"jawOpen": 0.2, "mouthUpperUpLeft": 1.0, "mouthUpperUpRight": 1.0,
     "mouthLowerDownLeft": 1.0, "mouthLowerDownRight": 1.0,
     "mouthStretchLeft": 1.0, "mouthStretchRight": 1.0},
    {name: 1.0 for name in NAMES},
]


def key_poses():
    poses = [(1001, {})]
    for k, name in enumerate(NAMES):
        poses.append((TOUR_START + TOUR_STEP * k, {name: 1.0}))
    poses.append((TOUR_START + TOUR_STEP * len(NAMES), {}))
    for m, mix in enumerate(MIXES):
        poses.append((MIX_START + MIX_STEP * m, mix))
    poses.append((MIX_START + MIX_STEP * len(MIXES), {}))
    return poses


def columns():
    return [-1.0 + 2.0 * i / (N - 1) for i in range(N)]


def neutral():
    """The rest lips in cm: (x, upper y, lower y) per column, PART above
    and below the seam at the middle and meeting at the corners."""
    return [(HALF_WIDTH * p, -SEAM_DROP * p * p + PART * lens(p),
             -SEAM_DROP * p * p - PART * lens(p)) for p in columns()]


def lip_uvs(rest, delta):
    """UV points: the upper lip's N columns, then the lower lip's."""
    upper, lower = [], []
    for (x, yu, yl), (dx, du, dl) in zip(rest, delta):
        upper.append(to_uv(x + dx, yu + du))
        lower.append(to_uv(x + dx, yl + dl))
    return upper + lower


def check(rest, deltas):
    """Assert the guarantees the module docstring states; return margins."""
    ps = columns()
    for name, delta in deltas.items():
        for end in (0, N - 1):
            assert abs(delta[end][1] - delta[end][2]) < 1e-12, (
                "%s opens the corner %d" % (name, end))
    spacing = 2.0 * HALF_WIDTH / (N - 1)
    narrowest = spacing
    for i in range(N - 1):
        worst = spacing + sum(
            min(0.0, d[i + 1][0] - d[i][0]) for d in deltas.values())
        narrowest = min(narrowest, worst)
    assert narrowest > 0.1 * spacing, (
        "a lip can fold in u: narrowest spacing %g" % narrowest)
    # Every column stays on the patch, at every weight in [0, 1].
    for i, p in enumerate(ps):
        x = rest[i][0]
        lo = x + sum(min(0.0, d[i][0]) for d in deltas.values())
        hi = x + sum(max(0.0, d[i][0]) for d in deltas.values())
        assert -X < lo and hi < X, ("column %d leaves the patch in x" % i)
        for lip in (1, 2):
            y = rest[i][lip]
            lo = y + sum(min(0.0, d[i][lip]) for d in deltas.values())
            hi = y + sum(max(0.0, d[i][lip]) for d in deltas.values())
            assert Y0 < lo and hi < Y1, ("column %d leaves the patch in y" % i)
    return narrowest / spacing


DOC = """
    RigExec example 18 - two lip curves on one NURBS patch, mixed by the
    ARKit jaw and mouth blend shapes, never crossing.

    Generated by examples/python/make_patch_mouth.py; edit the generator,
    not this file.

    Lips is one BasisCurves holding two 17-point curves, the upper lip and
    the lower lip, parted a little at rest and meeting at the corners. One
    RigExecPatchMover sticks both to the Face patch, a flat plane:
    rigExec:uvShapes lists 27 UV shapes and
    inputs:uvWeights connects, in the same order, to the Mouth control's
    27 float avars, named after ARKit's blend shapes (jawForward, jawLeft,
    jawRight, jawOpen, mouthClose, mouthFunnel, mouthPucker, mouthLeft,
    mouthRight, then the Left/Right pairs of mouthSmile, mouthFrown,
    mouthDimple, mouthStretch, mouthPress, mouthLowerDown and mouthUpperUp,
    and mouthRollLower/Upper and mouthShrugLower/Upper). Left is the
    character's left, +x.

    The lips cannot cross. Point i of each lip is one column: every shape
    moves both points of a column by the same u. The mover's contact pairs
    (rigExec:uvContactLower/Upper) pair the two points of each column, so
    the lower lip never stands above the upper one -- past contact they
    meet at their midpoint. mouthClose closes the rest gap and jawOpen's,
    half from each side, so it seals the lips at any jawOpen, and alone it
    closes the parted lips on the rest seam. The pucker, rolls, presses
    and mouthShrugLower close the rest gap too and push one lip against
    the other. No mix of weights folds a lip in u either: the generator
    checks the narrowest column spacing over every weight in [0, 1].

    FACS action units are combinations of these avars. AU8 lips toward each
    other: mouthClose with jawOpen. AU10 upper lip raiser: mouthUpperUp
    Left and Right. AU12 lip corner puller: mouthSmile. AU14 dimpler:
    mouthDimple. AU15 lip corner depressor: mouthFrown. AU16 lower lip
    depressor: mouthLowerDown. AU17 chin raiser: mouthShrugLower. AU18 lip
    pucker: mouthPucker. AU20 lip stretcher: mouthStretch. AU22 lip
    funneler: mouthFunnel. AU23 lip tightener has no ARKit shape; mouthPress
    with a little mouthPucker comes close. AU24 lip pressor: mouthPress.
    AU25 lips part: the rest pose, parted further by mouthLowerDown and
    mouthUpperUp at about 0.3 or a little jawOpen. AU26 jaw drop: jawOpen near 0.5, with mouthClose when
    the lips stay together. AU27 mouth stretch: jawOpen at 1. AU28 lip
    suck: mouthRollUpper and mouthRollLower. AU29 jaw thrust: jawForward.
    AU30 jaw sideways: jawLeft or jawRight.

    Frames 1007-1163 show each shape alone, in the order above, six frames
    apart. From 1179, ten frames apart: jawOpen; jawOpen with mouthClose;
    mouthClose alone, which closes the lips; an open smile (AU12+25+26); a
    sad mouth (AU15+17); a kiss (AU18+22); an O (AU22+25+26); a snarl
    (AU10+16+20+25); and every avar at 1 at once.

    The Face patch is a bilinear plane at z = 0, a guide so it does not
    draw through the lips; Skin is the same plane set back a little, to
    look at. Patch points run u-fastest (rows of constant v).
    """


def build(out_path):
    rest = neutral()
    ps = columns()
    deltas = {name: [shape(p) for p in ps] for name, shape in SHAPE_TABLE}
    margin = check(rest, deltas)

    stage = Usd.Stage.CreateInMemory()
    layer = stage.GetRootLayer()
    layer.documentation = DOC
    stage.SetStartTimeCode(1001)
    poses = key_poses()
    stage.SetEndTimeCode(poses[-1][0])
    stage.SetTimeCodesPerSecond(24)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    stage.SetDefaultPrim(root)

    rig = stage.DefinePrim(RIG, "RigExecRoot")
    rig.CreateAttribute("rigExec:partition", Sdf.ValueTypeNames.Token,
                        False, Sdf.VariabilityUniform).Set("MouthAsset")

    def icon(prim, name):
        prim.AddAppliedSchema("NodeGraphNodeAPI")
        prim.CreateAttribute(
            "ui:nodegraph:node:icon", Sdf.ValueTypeNames.Asset, False,
            Sdf.VariabilityUniform).Set(Sdf.AssetPath("../icons/" + name))

    # The control: one float avar per shape, keyed through the key poses.
    stage.DefinePrim(RIG + "/Controls", "Scope")
    control = stage.DefinePrim(CONTROL, "RigExecControl")
    control.AddAppliedSchema("RigExecControlAPI")
    icon(control, "control.png")
    for name in NAMES:
        avar = control.CreateAttribute("avars:" + name,
                                       Sdf.ValueTypeNames.Float, False)
        avar.Set(0.0)
        values = [pose.get(name, 0.0) for _, pose in poses]
        keyed = set()
        for k, value in enumerate(values):
            if value != 0.0:
                keyed.update(j for j in (k - 1, k, k + 1)
                             if 0 <= j < len(poses))
        for k in sorted(keyed):
            avar.Set(values[k], Usd.TimeCode(poses[k][0]))
    control.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(
        Gf.Matrix4d(1.0))

    # The mover.
    stage.DefinePrim(RIG + "/Movers", "Scope")
    stage.DefinePrim(RIG + "/Movers/Geometry", "Scope")
    mover = stage.DefinePrim(MOVER, "RigExecPatchMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    icon(mover, "surface_mover.png")
    mover.CreateRelationship("rigExec:moves", False).SetTargets(
        [Sdf.Path(LIPS + ".points")])
    mover.CreateRelationship("rigExec:patch", False).SetTargets(
        [Sdf.Path(PATCH)])
    mover.CreateRelationship("rigExec:uvPoints", False).SetTargets(
        [Sdf.Path(LIP_UVS)])
    mover.CreateRelationship("rigExec:uvShapes", False).SetTargets(
        [Sdf.Path(SHAPES + "/" + name) for name in NAMES])
    mover.CreateAttribute("inputs:uvWeights", Sdf.ValueTypeNames.FloatArray,
                          False).SetConnections(
        [Sdf.Path(CONTROL + ".avars:" + name) for name in NAMES])
    mover.CreateAttribute("rigExec:uvContactLower",
                          Sdf.ValueTypeNames.IntArray, False,
                          Sdf.VariabilityUniform).Set(
        Vt.IntArray(list(range(N, 2 * N))))
    mover.CreateAttribute("rigExec:uvContactUpper",
                          Sdf.ValueTypeNames.IntArray, False,
                          Sdf.VariabilityUniform).Set(
        Vt.IntArray(list(range(N))))

    # The patch: a guide, since the lips lie in its plane.
    stage.DefinePrim(GEOM, "Scope")
    patch = UsdGeom.NurbsPatch.Define(stage, PATCH)
    cvs = patch_cvs()
    patch.CreatePointsAttr([Gf.Vec3f(*c) for c in cvs])
    patch.CreateUVertexCountAttr(2)
    patch.CreateVVertexCountAttr(2)
    patch.CreateUOrderAttr(2)
    patch.CreateVOrderAttr(2)
    patch.CreateUKnotsAttr([0, 0, 1, 1])
    patch.CreateVKnotsAttr([0, 0, 1, 1])
    patch.CreateUFormAttr(UsdGeom.Tokens.open)
    patch.CreateVFormAttr(UsdGeom.Tokens.open)
    patch.CreatePurposeAttr(UsdGeom.Tokens.guide)
    patch.CreateExtentAttr(_extent(cvs))

    # The surface the lips ride, to look at.
    _author_skin(stage)

    # The UV carriers: the neutral, then one per shape.
    neutral_uv = lip_uvs(rest, [(0.0, 0.0, 0.0)] * N)
    carrier = UsdGeom.Points.Define(stage, LIP_UVS)
    carrier.CreatePointsAttr([Gf.Vec3f(u, v, 0.0) for u, v in neutral_uv])
    carrier.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    stage.DefinePrim(SHAPES, "Scope")
    for name in NAMES:
        shape = UsdGeom.Points.Define(stage, SHAPES + "/" + name)
        shape.CreatePointsAttr([Gf.Vec3f(u, v, 0.0)
                                for u, v in lip_uvs(rest, deltas[name])])
        shape.CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    # The lips, authored at rest: the patch at the neutral UVs.
    lips = UsdGeom.BasisCurves.Define(stage, LIPS)
    rest_points = [patch_point(u, v) for u, v in neutral_uv]
    lips.CreatePointsAttr([Gf.Vec3f(*q) for q in rest_points])
    lips.CreateCurveVertexCountsAttr([N, N])
    lips.CreateTypeAttr(UsdGeom.Tokens.linear)
    lips.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    lips.CreateWidthsAttr([0.09])
    lips.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    colors = lips.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    colors.Set([Gf.Vec3f(0.78, 0.16, 0.2), Gf.Vec3f(0.62, 0.1, 0.17)])
    lips.CreateExtentAttr(_extent(rest_points, 0.045))

    layer.Export(str(out_path))
    return margin


def _extent(points, pad=0.0):
    lo = [min(p[k] for p in points) - pad for k in range(3)]
    hi = [max(p[k] for p in points) + pad for k in range(3)]
    return [Gf.Vec3f(*lo), Gf.Vec3f(*hi)]


def _author_skin(stage, setback=0.05):
    points = [(x, y, -setback) for x, y, _ in patch_cvs()]
    skin = UsdGeom.Mesh.Define(stage, SKIN)
    skin.CreatePointsAttr([Gf.Vec3f(*q) for q in points])
    skin.CreateFaceVertexCountsAttr([4])
    skin.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
    skin.CreateNormalsAttr([Gf.Vec3f(0.0, 0.0, 1.0)])
    skin.SetNormalsInterpolation(UsdGeom.Tokens.constant)
    skin.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    skin.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(
        [Gf.Vec3f(0.86, 0.67, 0.57)])
    skin.CreateExtentAttr(_extent(points))


def main():
    here = pathlib.Path(__file__).resolve().parent
    out = (pathlib.Path(sys.argv[1]) if len(sys.argv) > 1
           else here.parent / "18_PatchMouth.usda")
    margin = build(out)
    print("wrote %s (narrowest column spacing %.0f%% of rest)"
          % (out, 100.0 * margin))


if __name__ == "__main__":
    main()
