"""rigexec.binding.generate_skin_weights: skin rows from volume weights.

Two nested bones, each holding one sphere weight stretched along its segment
(at the midpoint, inputs:scaleX = 1 + L/(2S), inputs:falloffMin =
max(S/4, (L/2) S/(L/2 + S))), over a six-point strip (inside one sphere, in
the overlap, outside both), with rows from the sphere field formula; partial
regeneration, an influence subset, several spheres on one bone, a pin bone's
sphere and the setup transform; the other volume shapes from their field
formulas: a rotated bone's sphere with half-axis scales, a plane (unbounded
and bounded) and a curve under an Xform; then the checked-in volumeSkin
fixtures and the docs example must hold exactly the rows the generator
writes, and the F8 fixture evaluates its rigid and released rows as
authored.

Usage: test_generate_skin_weights.py [<generated schema resources dir>]
"""

import math
import os
import pathlib
import sys
import unittest

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Gf, Sdf, Usd, UsdGeom, Vt  # noqa: E402
import rigexec  # noqa: E402

_RESOURCES = None
for _argument in list(sys.argv[1:]):
    if (pathlib.Path(_argument) / "plugInfo.json").is_file():
        _RESOURCES = _argument
        sys.argv.remove(_argument)
rigexec.load_schema_plugin(_RESOURCES)

_REPO = pathlib.Path(__file__).resolve().parents[2]
_FIXTURES = _REPO / "tests" / "fixtures" / "volumeSkin"
_EXAMPLE = _REPO / "docs" / "examples" / "skeleton_2d.usda"

BONES = "/Char/Rig/Bones"
A = BONES + "/A"
B = A + "/B"
MOVER = "/Char/Rig/Movers/Bind"
STRIP = "/Char/Art/Strip"
# Inside A only, inside A beside its midpoint, the overlap at B's base, the
# overlap past it, B's tip, and outside both spheres.
STRIP_POINTS = [(0.5, 0, 0), (1, 0.5, 0), (2, 0.5, 0), (2.5, 0.25, 0),
                (3, 0, 0), (5, 0, 0)]
# Each bone's segment in its own frame (to its first joint child) and its
# sphere's centre in asset space: A (0,0) -> (2,0), B (2,0) -> (3,0).
SEGMENTS = {A: (2.0, 0.0), B: (1.0, 0.0)}
CENTRES = {A: (1.0, 0.0, 0.0), B: (2.5, 0.0, 0.0)}


def _translate(x, y, z=0.0):
    return Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(x, y, z))


def _linear_field(distance, falloff_max, falloff_min=0.0):
    """A linear-profile volume weight: 1 at falloffMin, 0 at falloffMax."""
    u = min(max((distance - falloff_min) / (falloff_max - falloff_min), 0.0), 1.0)
    return 1.0 - u


def _sphere_field(p, center, angle, scales, falloff_min, falloff_max,
                  positive=(1.0, 1.0, 1.0), negative=(1.0, 1.0, 1.0)):
    """A linear sphere weight at \\p center, its local x axis at \\p angle
    degrees about z: d = |local / scales|, each half-axis further divided
    by its Pos or Neg factor."""
    a = math.radians(angle)
    dx, dy, dz = p[0] - center[0], p[1] - center[1], p[2] - center[2]
    local = [dx * math.cos(a) + dy * math.sin(a),
             -dx * math.sin(a) + dy * math.cos(a), dz]
    for axis in range(3):
        local[axis] /= scales[axis]
        local[axis] /= negative[axis] if local[axis] < 0 else positive[axis]
    return _linear_field(math.sqrt(sum(c * c for c in local)), falloff_max, falloff_min)


def _segment_distance(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    length = ax * ax + ay * ay
    t = 0.0 if length == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / length))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def _rows(stage, mover=MOVER):
    prim = stage.GetPrimAtPath(mover)
    size = prim.GetAttribute("rigExec:elementSize").Get()
    indices = list(prim.GetAttribute("rigExec:jointIndices").Get())
    weights = list(prim.GetAttribute("rigExec:jointWeights").Get())
    rows = []
    for p in range(len(indices) // size):
        rows.append([(indices[p * size + k], weights[p * size + k])
                     for k in range(size) if weights[p * size + k] != 0.0])
    return size, rows, indices, weights


def _volume(stage, parent, type_name, falloff_max, name="Influence",
            falloff_min=0.0, **inputs):
    """A linear-profile volume weight of any type under \\p parent; extra
    keyword arguments set inputs:<name> floats or rigExec:<name> tokens."""
    volume = stage.DefinePrim(parent + "/" + name, type_name)
    volume.GetAttribute("inputs:falloffMin").Set(falloff_min)
    volume.GetAttribute("inputs:falloffMax").Set(falloff_max)
    volume.GetAttribute("rigExec:falloffProfile").Set("linear")
    for key, value in inputs.items():
        if isinstance(value, str):
            volume.GetAttribute("rigExec:" + key).Set(value)
        else:
            volume.GetAttribute("inputs:" + key).Set(value)
    return volume


def _place(sphere, length, strength):
    """The stretched sphere's shape for a segment of \\p length and
    \\p strength S: falloffMax = S, scaleX = 1 + L/(2S), falloffMin =
    max(S/4, (L/2) S/(L/2 + S))."""
    half = length / 2.0
    sphere.GetAttribute("inputs:falloffMax").Set(strength)
    sphere.GetAttribute("inputs:falloffMin").Set(
        max(0.25 * strength, half * strength / (half + strength)))
    sphere.GetAttribute("inputs:scaleX").Set(1.0 + length / (2.0 * strength))


def _stretched_sphere(stage, bone, t, strength, use_tx=False):
    """The sphere that stands in for a bone's segment: at the midpoint of
    the segment to its child joint at \\p t (bone frame), its x axis along
    the segment, shaped by _place. Placed by rest:space, or, for a segment
    along +x with \\p use_tx, by rest:tx = L/2 alone. Returns the authored
    float values (rounded to float as the generator reads them) and the
    placement."""
    length = math.hypot(t[0], t[1])
    angle = math.degrees(math.atan2(t[1], t[0]))
    sphere = _volume(stage, bone, "RigExecSphereWeight", strength)
    _place(sphere, length, strength)
    if use_tx:
        assert t[1] == 0.0 and t[0] > 0.0
        sphere.GetAttribute("rest:tx").Set(length / 2.0)
    else:
        rest = Gf.Matrix4d(1.0)
        rest.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), angle))
        rest.SetTranslateOnly(Gf.Vec3d(t[0] / 2.0, t[1] / 2.0, 0))
        sphere.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(rest)
    return _sphere_info(stage, sphere.GetPath().pathString, angle)


def _sphere_info(stage, path, angle=0.0):
    get = stage.GetPrimAtPath(path).GetAttribute
    return {"min": get("inputs:falloffMin").Get(), "max": get("inputs:falloffMax").Get(),
            "scaleX": get("inputs:scaleX").Get(), "angle": angle}


def _stage(art_xform=None, char_xform=False):
    """Two nested bones A -> B -> End, a stretched sphere of strength 1 in
    both (A's placed by rest:space, B's by rest:tx), a skin on a strip."""
    stage = Usd.Stage.CreateInMemory()
    char = UsdGeom.Xform.Define(stage, "/Char")
    if char_xform:  # shared by rig and art: it cancels
        char.AddTranslateOp().Set(Gf.Vec3d(7, 8, 9))
    stage.DefinePrim("/Char/Rig", "RigExecRoot")
    stage.DefinePrim(BONES, "Scope")
    for path, x in ((A, 0), (B, 2), (B + "/End", 1)):
        joint = stage.DefinePrim(path, "RigExecJoint")
        if not path.endswith("End"):
            joint.AddAppliedSchema("RigExecBone2dAPI")
        joint.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(_translate(x, 0))
    _stretched_sphere(stage, A, SEGMENTS[A], 1.0)
    _stretched_sphere(stage, B, SEGMENTS[B], 1.0, use_tx=True)
    skeleton = stage.DefinePrim("/Char/Rig/Skeleton", "RigExecSkeleton2d")
    skeleton.GetRelationship("rigExec:joints").SetTargets([A, B])
    art = UsdGeom.Xform.Define(stage, "/Char/Art")
    points = [Gf.Vec3d(*p) for p in STRIP_POINTS]
    if art_xform is not None:
        art.AddTransformOp().Set(art_xform)
        inverse = art_xform.GetInverse()
        points = [inverse.Transform(p) for p in points]
    strip = UsdGeom.Points.Define(stage, STRIP)
    strip.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(p) for p in points]))
    mover = stage.DefinePrim(MOVER, "RigExecSkinMover")
    mover.AddAppliedSchema("RigExecMoverAPI")
    mover.GetRelationship("rigExec:moves").SetTargets([STRIP + ".points"])
    mover.GetRelationship("rigExec:influences").SetTargets([A, B])
    return stage


def _bone_field(stage, bone, p):
    """The raw field of \\p bone's stretched sphere (along +x) at \\p p."""
    info = _sphere_info(stage, bone + "/Influence")
    return _sphere_field(p, CENTRES[bone], 0.0, (info["scaleX"], 1, 1),
                         info["min"], info["max"])


def _normalised(fields):
    """One row from raw per-influence fields: clamp each to 1, normalise."""
    fields = [min(f, 1.0) for f in fields]
    total = sum(fields)
    return [] if total == 0 else [(j, f / total) for j, f in enumerate(fields) if f > 0]


def _expected_two_bones(stage, points=STRIP_POINTS):
    """Normalised (A, B) rows per point, from the authored spheres."""
    return [_normalised((_bone_field(stage, A, p), _bone_field(stage, B, p)))
            for p in points]


def _authored_names(prim):
    return sorted(str(p.GetName()) for p in prim.GetAuthoredProperties())


def _copy(path):
    """An anonymous, editable copy of a checked-in layer."""
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.TransferContent(Sdf.Layer.FindOrOpen(str(path)))
    return Usd.Stage.Open(layer)


class GenerateSkinWeightsTest(unittest.TestCase):

    def assert_rows(self, actual, expected, places=6):
        self.assertEqual(len(actual), len(expected))
        for point, (got, want) in enumerate(zip(actual, expected)):
            self.assertEqual([j for j, _ in got], [j for j, _ in want],
                             "point %d: %s != %s" % (point, got, want))
            for (_, w), (_, v) in zip(got, want):
                self.assertAlmostEqual(w, v, places=places, msg="point %d" % point)

    def test_two_nested_bones(self):
        stage = _stage()
        mover = stage.GetPrimAtPath(MOVER)
        before = _authored_names(mover)
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result, {"rows_written": 6, "element_size": 2,
                                  "influences_without_volumes": []})
        size, rows, indices, weights = _rows(stage)
        self.assertEqual(size, 2)
        self.assert_rows(rows, _expected_two_bones(stage))
        # Inside A alone (on its axis and beside its midpoint), the
        # overlap, B's tip, and the zero row.
        self.assertEqual(rows[0], [(0, 1.0)])
        self.assertEqual(rows[1], [(0, 1.0)])
        self.assertEqual([j for j, _ in rows[2]], [0, 1])
        self.assertEqual(rows[4], [(1, 1.0)])
        self.assertEqual(rows[5], [])
        # Padding is index 0, weight 0.
        self.assertEqual((indices[1], weights[1]), (0, 0.0))
        self.assertEqual((indices[10], weights[10], indices[11], weights[11]),
                         (0, 0.0, 0, 0.0))
        # Only the three layout properties are authored.
        self.assertEqual(
            sorted(set(_authored_names(mover)) - set(before)),
            ["rigExec:elementSize", "rigExec:jointIndices", "rigExec:jointWeights"])
        self.assertEqual(rigexec.generate_skin_weights, rigexec.binding.generate_skin_weights)
        self.assertIn("binding", rigexec.__all__)
        self.assertIn("generate_skin_weights", rigexec.__all__)

    def test_rows_compile(self):
        stage = _stage()
        rigexec.binding.generate_skin_weights(stage, MOVER)
        rig = rigexec.Rig(stage, "/Char/Rig")
        rig.compile()
        pose = rig.evaluate(-1)
        self.assertTrue(pose.valid, pose.diagnostics)
        moved = pose.moved_property(STRIP + ".points")
        for got, want in zip(moved, STRIP_POINTS):
            for g, w in zip(got, want):
                self.assertAlmostEqual(g, w, places=5)

    def test_point_indices(self):
        stage = _stage()
        rigexec.binding.generate_skin_weights(stage, MOVER)
        _, before, _, _ = _rows(stage)
        # B's strength raised to 2.5 and its sphere re-placed.
        _place(stage.GetPrimAtPath(B + "/Influence"), 1.0, 2.5)
        result = rigexec.binding.generate_skin_weights(stage, MOVER, [3, 5, 3])
        self.assertEqual(result["rows_written"], 2)
        self.assertEqual(result["element_size"], 2)
        _, rows, _, _ = _rows(stage)
        widened = _expected_two_bones(stage)
        for point in (0, 1, 2, 4):
            self.assertEqual(rows[point], before[point], "point %d kept" % point)
        self.assert_rows([rows[3], rows[5]], [widened[3], widened[5]])
        self.assertEqual(rows[5], [(1, 1.0)])

    def test_point_indices_recompute_element_size(self):
        stage = _stage()
        prim = stage.GetPrimAtPath(MOVER)
        # Every point rigid to A, one slot per row.
        prim.GetAttribute("rigExec:elementSize").Set(1)
        prim.GetAttribute("rigExec:jointIndices").Set(Vt.IntArray([0] * 6))
        prim.GetAttribute("rigExec:jointWeights").Set(Vt.FloatArray([1.0] * 6))
        result = rigexec.binding.generate_skin_weights(stage, MOVER, [2])
        self.assertEqual(result["element_size"], 2)
        size, rows, indices, weights = _rows(stage)
        self.assertEqual(size, 2)
        self.assert_rows([rows[2]], [_expected_two_bones(stage)[2]])
        for point in (0, 1, 3, 4, 5):
            self.assertEqual(rows[point], [(0, 1.0)])
            self.assertEqual((indices[2 * point + 1], weights[2 * point + 1]), (0, 0.0))
        # Shrinks again when the wide row is released and the rest stay single.
        prim.GetAttribute("rigExec:jointWeights").Set(
            Vt.FloatArray([w if i not in (4, 5) else 0.0 for i, w in enumerate(weights)]))
        result = rigexec.binding.generate_skin_weights(stage, MOVER, [0])
        self.assertEqual(result["element_size"], 1)
        self.assertEqual(_rows(stage)[1], [[(0, 1.0)], [(0, 1.0)], [], [(0, 1.0)],
                                           [(0, 1.0)], [(0, 1.0)]])

    def test_point_indices_without_layout(self):
        stage = _stage()
        result = rigexec.binding.generate_skin_weights(stage, MOVER, [0, 4])
        self.assertEqual(result, {"rows_written": 2, "element_size": 1,
                                  "influences_without_volumes": []})
        self.assertEqual(_rows(stage)[1], [[(0, 1.0)], [], [], [], [(1, 1.0)], []])

    def test_bad_point_indices(self):
        stage = _stage()
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER, [6])
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER, [-1])
        prim = stage.GetPrimAtPath(MOVER)
        self.assertFalse(prim.GetAttribute("rigExec:jointIndices").HasAuthoredValue())
        # A layout written for another point count cannot be merged.
        prim.GetAttribute("rigExec:jointIndices").Set(Vt.IntArray([0] * 4))
        prim.GetAttribute("rigExec:jointWeights").Set(Vt.FloatArray([1.0] * 4))
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER, [0])

    def test_influence_subset(self):
        stage = _stage()
        rel = stage.GetPrimAtPath(MOVER).GetRelationship("rigExec:influences")
        rel.SetTargets([B])
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result["element_size"], 1)
        self.assertEqual(_rows(stage)[1], [[], [], [(0, 1.0)], [(0, 1.0)], [(0, 1.0)], []])
        # Slots index the list as given, not the skeleton's joints.
        rel.SetTargets([B, A])
        rigexec.binding.generate_skin_weights(stage, MOVER)
        swapped = [[(1 - j, w) for j, w in reversed(row)]
                   for row in _expected_two_bones(stage)]
        self.assert_rows(_rows(stage)[1], swapped)
        # An influence without a volume weight contributes nothing and is
        # reported.
        rel.SetTargets([A, B, B + "/End"])
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result["influences_without_volumes"], [B + "/End"])
        self.assert_rows(_rows(stage)[1], _expected_two_bones(stage))

    def test_volumes_sum_and_clamp(self):
        stage = _stage()
        # A second, wide sphere about A's origin: summed with the stretched
        # one, clamped to 1.
        _volume(stage, A, "RigExecSphereWeight", 6.0, name="Wide")
        rigexec.binding.generate_skin_weights(stage, MOVER)
        expected = []
        raw = []
        for p in STRIP_POINTS:
            a = min(_bone_field(stage, A, p) + _linear_field(math.dist(p, (0, 0, 0)), 6.0), 1.0)
            b = _bone_field(stage, B, p)
            raw.append((a, b))
            expected.append(_normalised((a, b)))
        self.assert_rows(_rows(stage)[1], expected)
        # The overlap clamps A to 1 against B's own field.
        self.assertEqual(raw[2][0], 1.0)
        self.assertAlmostEqual(_rows(stage)[1][2][0][1], 1.0 / (1.0 + raw[2][1]), places=6)
        self.assertEqual(_rows(stage)[1][5], [(0, 1.0)])

    def test_pin_bone_sphere(self):
        """A pin bone (no joint child, L = 0): an unscaled sphere about its
        origin, falloffMin S/4."""
        stage = _stage()
        for name, x in (("P", 10.0), ("Q", 11.0)):
            pin = stage.DefinePrim(BONES + "/" + name, "RigExecJoint")
            pin.AddAppliedSchema("RigExecBone2dAPI")
            pin.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(_translate(x, 0))
            info = _stretched_sphere(stage, pin.GetPath().pathString, (0.0, 0.0), 1.0)
            self.assertEqual((info["scaleX"], info["min"], info["max"]), (1.0, 0.25, 1.0))
        points = [(10.25, 0, 0), (10.5, 0.5, 0), (10, 0.9, 0), (10, -0.9, 0),
                  (10, 0, 0.5), (10.9, 0, 0)]
        stage.GetPrimAtPath(STRIP).GetAttribute("points").Set(
            Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        stage.GetPrimAtPath(MOVER).GetRelationship("rigExec:influences").SetTargets(
            [BONES + "/P", BONES + "/Q"])
        rigexec.binding.generate_skin_weights(stage, MOVER)
        expected = []
        for p in points:
            fp = _linear_field(math.dist(p, (10, 0, 0)), 1.0, 0.25)
            fq = _linear_field(math.dist(p, (11, 0, 0)), 1.0, 0.25)
            expected.append([(j, w / (fp + fq)) for j, w in ((0, fp), (1, fq)) if w > 0])
        self.assert_rows(_rows(stage)[1], expected)
        self.assertEqual(_rows(stage)[1][2], [(0, 1.0)])
        self.assertEqual(_rows(stage)[1][3], [(0, 1.0)])
        self.assert_rows([_rows(stage)[1][0]], [[(0, 0.75), (1, 0.25)]])

    def test_stretched_spheres_under_bones(self):
        """The base stage's spheres: A (0,0) -> (2,0) placed by rest:space
        and B (2,0) -> (3,0) placed by rest:tx = L/2, strength 1."""
        stage = _stage()
        self.assertFalse(stage.GetPrimAtPath(B + "/Influence")
                         .GetAttribute("rest:space").HasAuthoredValue())
        a = _sphere_info(stage, A + "/Influence")
        b = _sphere_info(stage, B + "/Influence")
        # The rule's numbers: L/2 = 1 gives scaleX 2 and falloffMin 1/2;
        # L/2 = 1/2 gives scaleX 1.5 and falloffMin max(1/4, 1/3).
        self.assertEqual((a["scaleX"], a["min"], a["max"]), (2.0, 0.5, 1.0))
        self.assertAlmostEqual(b["scaleX"], 1.5, places=6)
        self.assertAlmostEqual(b["min"], 1.0 / 3.0, places=6)
        points = STRIP_POINTS + [(0, 0, 0), (2, 0, 0), (-1, 0, 0), (1, 1, 0)]
        stage.GetPrimAtPath(STRIP).GetAttribute("points").Set(
            Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result["influences_without_volumes"], [])
        raw = [(_bone_field(stage, A, p), _bone_field(stage, B, p)) for p in points]
        # The inner ellipse passes through both joints of A: raw weight 1 at
        # (0,0) and (2,0); the off surface sits S beyond each joint on the
        # axis, (-1,0); across the midpoint it is S, (1,1).
        self.assertAlmostEqual(raw[6][0], 1.0, places=6)
        self.assertAlmostEqual(raw[7][0], 1.0, places=6)
        self.assertAlmostEqual(raw[8][0], 0.0, places=6)
        self.assertAlmostEqual(raw[9][0], 0.0, places=6)
        # Hand values: (2, 0.5) reads A at d = |(0.5, 0.5)| = 0.7071 and B
        # at d = |(-1/3, 0.5)| = 0.6009.
        self.assertAlmostEqual(raw[2][0], 1.0 - (math.sqrt(0.5) - 0.5) / 0.5, places=6)
        self.assertAlmostEqual(
            raw[2][1], 1.0 - (math.sqrt(1.0 / 9.0 + 0.25) - 1.0 / 3.0) / (2.0 / 3.0),
            places=6)
        self.assert_rows(_rows(stage)[1], [_normalised(f) for f in raw])
        self.assertEqual(_rows(stage)[1][0], [(0, 1.0)])
        self.assertEqual(_rows(stage)[1][5], [])

    def test_rotated_sphere_half_axes(self):
        """A bone whose child sits off its x axis: the sphere turns with
        the segment; Pos/Neg half-axis factors are read too."""
        stage = _stage()
        stage.RemovePrim(A + "/Influence")
        stage.RemovePrim(B + "/Influence")
        pin = stage.DefinePrim(BONES + "/P", "RigExecJoint")
        pin.AddAppliedSchema("RigExecBone2dAPI")
        pin.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(_translate(10, 0))
        tip = stage.DefinePrim(BONES + "/P/Tip", "RigExecJoint")
        tip.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(_translate(0, 2))
        p = _stretched_sphere(stage, BONES + "/P", (0.0, 2.0), 1.0)
        self.assertAlmostEqual(p["angle"], 90.0, places=9)
        # A plain sphere on A at (0,0), its negative x half-axis halved.
        _volume(stage, A, "RigExecSphereWeight", 2.0, scaleXNeg=0.5)
        points = [(10, 1, 0), (10.5, 1, 0), (10, 2.5, 0), (10, 3.2, 0),
                  (9.5, 2, 0), (1, 0, 0), (-0.75, 0, 0), (-0.25, 0.5, 0)]
        stage.GetPrimAtPath(STRIP).GetAttribute("points").Set(
            Vt.Vec3fArray([Gf.Vec3f(*q) for q in points]))
        stage.GetPrimAtPath(MOVER).GetRelationship("rigExec:influences").SetTargets(
            [A, BONES + "/P"])
        rigexec.binding.generate_skin_weights(stage, MOVER)
        raw = []
        for q in points:
            fa = _sphere_field(q, (0, 0, 0), 0.0, (1, 1, 1), 0.0, 2.0,
                               negative=(0.5, 1, 1))
            fp = _sphere_field(q, (10, 1, 0), p["angle"], (p["scaleX"], 1, 1),
                               p["min"], p["max"])
            raw.append((fa, fp))
        # (10, 2.5) is 1.5 along the segment from the midpoint: 0.75 after
        # the stretch, halfway through the band [0.5, 1].
        self.assertAlmostEqual(raw[2][1], 0.5, places=6)
        # (-0.75, 0) on A's negative half-axis reads as 1.5 of 2.
        self.assertAlmostEqual(raw[6][0], 0.25, places=6)
        self.assert_rows(_rows(stage)[1], [_normalised(f) for f in raw])

    def test_plane(self):
        """A plane weight on A (its x axis, band [0, 4]) beside B's sphere;
        then bounded to |y| <= 0.3."""
        stage = _stage()
        stage.RemovePrim(A + "/Influence")
        plane = _volume(stage, A, "RigExecPlaneWeight", 4.0, planeAxis="x")
        rigexec.binding.generate_skin_weights(stage, MOVER)

        def rows(bounded):
            out = []
            for p in STRIP_POINTS:
                fa = _linear_field(p[0], 4.0)
                if bounded and abs(p[1]) > 0.3:
                    fa = 0.0
                out.append(_normalised((fa, _bone_field(stage, B, p))))
            return out
        self.assert_rows(_rows(stage)[1], rows(False))
        # (0.5, 0): plane 0.875, B's sphere 0.
        self.assertEqual(_rows(stage)[1][0], [(0, 1.0)])
        plane.GetAttribute("rigExec:planeBounds").Set("bounded")
        plane.GetAttribute("inputs:extentU").Set(0.3)
        plane.GetAttribute("inputs:extentV").Set(1.0)
        rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assert_rows(_rows(stage)[1], rows(True))
        self.assertEqual(_rows(stage)[1][1], [])
        # A plane on another axis: y, the schema's fallback.
        plane.GetAttribute("rigExec:planeAxis").Clear()
        plane.GetAttribute("rigExec:planeBounds").Set("unbounded")
        rigexec.binding.generate_skin_weights(stage, MOVER)
        expected = [_normalised((_linear_field(p[1], 4.0), _bone_field(stage, B, p)))
                    for p in STRIP_POINTS]
        self.assert_rows(_rows(stage)[1], expected)
        # An unknown axis is no field.
        plane.GetAttribute("rigExec:planeAxis").Set("w")
        with self.assertRaises(RuntimeError):
            rigexec.binding.generate_skin_weights(stage, MOVER)

    def test_curve(self):
        """A curve weight on B (scaleY 2) over a polyline that sits under a
        translated Xform; the curve's points are carried into asset space
        like the skin's, and both into B's scaled local frame."""
        stage = _stage()
        stage.RemovePrim(B + "/Influence")
        holder = UsdGeom.Xform.Define(stage, "/Char/Art/Guides")
        holder.AddTranslateOp().Set(Gf.Vec3d(0, 5, 0))
        polyline = [(2, 0, 0), (4, 0, 0), (4, 2, 0)]
        curve = UsdGeom.BasisCurves.Define(stage, "/Char/Art/Guides/Path")
        curve.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(x, y - 5, z) for x, y, z in polyline]))
        curve.CreateCurveVertexCountsAttr(Vt.IntArray([3]))
        curve.CreateTypeAttr("linear")
        volume = _volume(stage, B, "RigExecCurveWeight", 1.0, scaleY=2.0)
        volume.GetRelationship("rigExec:curve").SetTargets([curve.GetPath()])
        points = STRIP_POINTS + [(4, 1, 0), (4.5, 1, 0), (3, 1.5, 0)]
        stage.GetPrimAtPath(STRIP).GetAttribute("points").Set(
            Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result["influences_without_volumes"], [])

        def curve_field(p):
            # B's local frame is a translation by (2, 0); scaleY 2 divides y.
            def local(q):
                return ((q[0] - 2.0), (q[1] - 0.0) / 2.0)
            lp = local(p)
            lc = [local(c) for c in polyline]
            d = min(_segment_distance(lp, lc[k], lc[k + 1]) for k in range(len(lc) - 1))
            return _linear_field(d, 1.0)
        raw = [(_bone_field(stage, A, p), curve_field(p)) for p in points]
        # (3, 1.5) is 1.5 above the first segment, 0.75 after scaleY.
        self.assertAlmostEqual(raw[8][1], 0.25, places=6)
        self.assertAlmostEqual(raw[6][1], 1.0, places=6)
        self.assert_rows(_rows(stage)[1], [_normalised(f) for f in raw])
        # The curve source can be the points property itself.
        volume.GetRelationship("rigExec:curve").SetTargets(
            [curve.GetPath().AppendProperty("points")])
        rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assert_rows(_rows(stage)[1], [_normalised(f) for f in raw])
        # Exactly one points source.
        volume.GetRelationship("rigExec:curve").SetTargets([])
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER)
        volume.GetRelationship("rigExec:curve").SetTargets([curve.GetPath(), STRIP])
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER)
        volume.GetRelationship("rigExec:curve").SetTargets([MOVER])
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER)

    def test_setup_transform(self):
        reference = _stage()
        rigexec.binding.generate_skin_weights(reference, MOVER)
        placed = Gf.Matrix4d(1.0)
        placed.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
        placed.SetTranslateOnly(Gf.Vec3d(10, -3, 0))
        stage = _stage(art_xform=placed, char_xform=True)
        rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assert_rows(_rows(stage)[1], _rows(reference)[1], places=5)

    def test_rejections(self):
        stage = _stage()
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, STRIP)
        with self.assertRaises(TypeError):
            rigexec.binding.generate_skin_weights(None, MOVER)
        # A non-positive scale is no field.
        stage.GetPrimAtPath(A + "/Influence").GetAttribute("inputs:scaleX").Set(0.0)
        with self.assertRaises(RuntimeError):
            rigexec.binding.generate_skin_weights(stage, MOVER)
        prim = stage.GetPrimAtPath(MOVER)
        self.assertFalse(prim.GetAttribute("rigExec:jointIndices").HasAuthoredValue())

    def test_fixtures_hold_generated_rows(self):
        for name, mover in (("chain_2d", "/Char/Rig/Movers/Bind"),
                            ("arm_3d", "/Arm/Rig/Movers/Bind")):
            stage = _copy(_FIXTURES / (name + ".usda"))
            authored = _rows(stage, mover)
            rigexec.binding.generate_skin_weights(stage, mover)
            self.assertEqual(_rows(stage, mover), authored, name)
        stage = _copy(_EXAMPLE)
        mover = "/Skeleton2dAsset/Rig/Movers/Bind"
        authored = _rows(stage, mover)
        rigexec.binding.generate_skin_weights(stage, mover)
        self.assertEqual(_rows(stage, mover), authored, "docs example")

    def test_fixture_spheres_follow_the_rule(self):
        """Every Influence in the fixtures and the docs example is a sphere
        weight placed by the stretched-sphere rule for its bone's segment to
        its first joint child (rest:tx = L/2; none for a pin)."""
        for path in (_FIXTURES / "chain_2d.usda", _FIXTURES / "arm_3d.usda",
                     _FIXTURES / "arm_3d_regenerated.usda", _EXAMPLE):
            stage = Usd.Stage.Open(str(path))
            spheres = [p for p in stage.Traverse() if p.GetName() == "Influence"]
            self.assertTrue(spheres, path.name)
            for sphere in spheres:
                self.assertEqual(str(sphere.GetTypeName()), "RigExecSphereWeight")
                bone = sphere.GetParent()
                ends = [c for c in bone.GetChildren()
                        if str(c.GetTypeName()) == "RigExecJoint"]
                length = (ends[0].GetAttribute("rest:space").Get()
                          .ExtractTranslation().GetLength() if ends else 0.0)
                strength = sphere.GetAttribute("inputs:falloffMax").Get()
                half = length / 2.0
                self.assertEqual(sphere.GetAttribute("rest:tx").Get(), half)
                self.assertAlmostEqual(
                    sphere.GetAttribute("inputs:scaleX").Get(),
                    1.0 + length / (2.0 * strength), places=6)
                self.assertAlmostEqual(
                    sphere.GetAttribute("inputs:falloffMin").Get(),
                    max(0.25 * strength, half * strength / (half + strength)),
                    places=6)

    def test_regenerated_fixture(self):
        """F8: widen Mid, author rigid and released rows, regenerate the rest."""
        mover = "/Arm/Rig/Movers/Bind"
        stage = _copy(_FIXTURES / "arm_3d.usda")
        size, _, indices, weights = _rows(stage, mover)
        # Mid's strength 0.9 -> 1.4, its sphere re-placed for its 2-unit
        # segment.
        _place(stage.GetPrimAtPath("/Arm/Rig/Joints/Root/Mid/Influence"), 2.0, 1.4)
        for point in (0, 1, 10, 11):
            for k in range(size):
                indices[point * size + k] = 0
                weights[point * size + k] = 1.0 if (point < 2 and k == 0) else 0.0
        prim = stage.GetPrimAtPath(mover)
        prim.GetAttribute("rigExec:jointIndices").Set(Vt.IntArray(indices))
        prim.GetAttribute("rigExec:jointWeights").Set(Vt.FloatArray(weights))
        count = len(stage.GetPrimAtPath("/Arm/Art/Stroke").GetAttribute("points").Get())
        flexi = [p for p in range(count) if p not in (0, 1, 2, 3, 10, 11)]
        rigexec.binding.generate_skin_weights(stage, mover, flexi)
        fixture = Usd.Stage.Open(str(_FIXTURES / "arm_3d_regenerated.usda"))
        self.assertEqual(_rows(stage, mover), _rows(fixture, mover))
        _, f4, _, _ = _rows(_copy(_FIXTURES / "arm_3d.usda"), mover)
        _, f8, _, _ = _rows(fixture, mover)
        self.assertEqual(f8[0], [(0, 1.0)])
        self.assertEqual(f8[2:4], f4[2:4])
        self.assertEqual(f8[10:12], [[], []])
        self.assertNotEqual(f8[6], f4[6])
        # Released points hold their rest; rigid ones ride Root alone.
        rig = rigexec.Rig(fixture, "/Arm/Rig")
        rig.compile()
        pose = rig.evaluate(2)
        self.assertTrue(pose.valid, pose.diagnostics)
        rest = fixture.GetPrimAtPath("/Arm/Art/Stroke").GetAttribute("points").Get()
        moved = pose.moved_property("/Arm/Art/Stroke.points")
        for point in (10, 11):
            self.assertEqual(tuple(moved[point]), tuple(rest[point]))
        root = Gf.Matrix4d(*pose.joint_matrix("/Arm/Rig/Joints/Root"))
        for point in (0, 1):
            want = root.Transform(Gf.Vec3d(rest[point]))
            for g, w in zip(moved[point], want):
                self.assertAlmostEqual(g, w, places=5)
        self.assertGreater((Gf.Vec3d(*moved[6]) - Gf.Vec3d(rest[6])).GetLength(), 1e-3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
