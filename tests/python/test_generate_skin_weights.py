"""rigexec.binding.generate_skin_weights: skin rows from capsule weights.

Hand-computed rows for two nested bones over a six-point strip (inside one
capsule, in the overlap, outside both), partial regeneration, an influence
subset, several capsules on one bone, a pin bone's spherical capsule and the
setup transform; then the checked-in capsuleSkin fixtures and the docs
example must hold exactly the rows the generator writes, and the F8 fixture
evaluates its rigid and released rows as authored.

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
_FIXTURES = _REPO / "tests" / "fixtures" / "capsuleSkin"
_EXAMPLE = _REPO / "docs" / "examples" / "skeleton_2d.usda"

BONES = "/Char/Rig/Bones"
A = BONES + "/A"
B = A + "/B"
MOVER = "/Char/Rig/Movers/Bind"
STRIP = "/Char/Art/Strip"
# Inside A only, A's half band, the overlap at B's base, the overlap past
# it, B's tip, and outside both capsules.
STRIP_POINTS = [(0.5, 0, 0), (1, 0.5, 0), (2, 0.5, 0), (2.5, 0.25, 0),
                (3, 0, 0), (5, 0, 0)]


def _translate(x, y, z=0.0):
    return Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(x, y, z))


def _linear_field(distance, falloff_max, falloff_min=0.0):
    """A linear-profile volume weight: 1 at falloffMin, 0 at falloffMax."""
    u = min(max((distance - falloff_min) / (falloff_max - falloff_min), 0.0), 1.0)
    return 1.0 - u


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


def _stage(art_xform=None, char_xform=False):
    """Two nested bones A -> B -> End, capsules in both, a skin on a strip."""
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
    for bone in (A, B):
        _capsule(stage, bone, 1.0)
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


def _capsule(stage, parent, falloff_max, name="Capsule", falloff_min=0.0):
    capsule = stage.DefinePrim(parent + "/" + name, "RigExecCapsuleWeight")
    capsule.GetAttribute("inputs:falloffMin").Set(falloff_min)
    capsule.GetAttribute("inputs:falloffMax").Set(falloff_max)
    capsule.GetAttribute("rigExec:falloffProfile").Set("linear")
    return capsule


def _expected_two_bones(b_max=1.0):
    """Normalised (A, B) fields per strip point, from the segments."""
    rows = []
    for p in STRIP_POINTS:
        a = _linear_field(_segment_distance(p, (0, 0), (2, 0)), 1.0)
        b = _linear_field(_segment_distance(p, (2, 0), (3, 0)), b_max)
        total = a + b
        rows.append([] if total == 0 else
                    [(j, w / total) for j, w in ((0, a), (1, b)) if w > 0])
    return rows


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
                                  "influences_without_capsules": []})
        size, rows, indices, weights = _rows(stage)
        self.assertEqual(size, 2)
        self.assert_rows(rows, _expected_two_bones())
        # Inside A alone, the overlap, B's tip, and the zero row.
        self.assertEqual(rows[0], [(0, 1.0)])
        self.assertEqual(rows[2], [(0, 0.5), (1, 0.5)])
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
        stage.GetPrimAtPath(B + "/Capsule").GetAttribute("inputs:falloffMax").Set(2.5)
        result = rigexec.binding.generate_skin_weights(stage, MOVER, [3, 5, 3])
        self.assertEqual(result["rows_written"], 2)
        self.assertEqual(result["element_size"], 2)
        _, rows, _, _ = _rows(stage)
        widened = _expected_two_bones(2.5)
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
        self.assertEqual(rows[2], [(0, 0.5), (1, 0.5)])
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
                                  "influences_without_capsules": []})
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
        swapped = [[(1 - j, w) for j, w in reversed(row)] for row in _expected_two_bones()]
        self.assert_rows(_rows(stage)[1], swapped)
        # An influence without a capsule contributes nothing and is reported.
        rel.SetTargets([A, B, B + "/End"])
        result = rigexec.binding.generate_skin_weights(stage, MOVER)
        self.assertEqual(result["influences_without_capsules"], [B + "/End"])
        self.assert_rows(_rows(stage)[1], _expected_two_bones())

    def test_capsules_sum_and_clamp(self):
        stage = _stage()
        # A second, wide capsule in A: summed with the first, clamped to 1.
        _capsule(stage, A, 6.0, name="Wide")
        rigexec.binding.generate_skin_weights(stage, MOVER)
        expected = []
        for p in STRIP_POINTS:
            d = _segment_distance(p, (0, 0), (2, 0))
            a = min(_linear_field(d, 1.0) + _linear_field(d, 6.0), 1.0)
            b = _linear_field(_segment_distance(p, (2, 0), (3, 0)), 1.0)
            expected.append([(j, w / (a + b)) for j, w in ((0, a), (1, b)) if w > 0])
        self.assert_rows(_rows(stage)[1], expected)
        self.assertAlmostEqual(_rows(stage)[1][2][0][1], 1.0 / 1.5, places=6)
        self.assertEqual(_rows(stage)[1][5], [(0, 1.0)])

    def test_pin_bone_sphere(self):
        stage = _stage()
        for name, x in (("P", 10.0), ("Q", 11.0)):
            pin = stage.DefinePrim(BONES + "/" + name, "RigExecJoint")
            pin.AddAppliedSchema("RigExecBone2dAPI")
            pin.CreateAttribute("rest:space", Sdf.ValueTypeNames.Matrix4d).Set(_translate(x, 0))
            _capsule(stage, pin.GetPath().pathString, 1.0)
        points = [(10.25, 0, 0), (10.5, 0.5, 0), (10, 0.9, 0), (10, -0.9, 0),
                  (10, 0, 0.5), (10.9, 0, 0)]
        stage.GetPrimAtPath(STRIP).GetAttribute("points").Set(
            Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        stage.GetPrimAtPath(MOVER).GetRelationship("rigExec:influences").SetTargets(
            [BONES + "/P", BONES + "/Q"])
        rigexec.binding.generate_skin_weights(stage, MOVER)
        expected = []
        for p in points:
            fp = _linear_field(math.dist(p, (10, 0, 0)), 1.0)
            fq = _linear_field(math.dist(p, (11, 0, 0)), 1.0)
            expected.append([(j, w / (fp + fq)) for j, w in ((0, fp), (1, fq)) if w > 0])
        self.assert_rows(_rows(stage)[1], expected)
        self.assertEqual(_rows(stage)[1][2], [(0, 1.0)])
        self.assertEqual(_rows(stage)[1][3], [(0, 1.0)])
        self.assert_rows([_rows(stage)[1][0]], [[(0, 0.75), (1, 0.25)]])

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
        capsule = stage.GetPrimAtPath(A + "/Capsule")
        capsule.GetRelationship("rigExec:segmentEnd").SetTargets([B, B + "/End"])
        with self.assertRaises(ValueError):
            rigexec.binding.generate_skin_weights(stage, MOVER)
        capsule.GetRelationship("rigExec:segmentEnd").SetTargets([B + "/End"])
        # An explicit end: A's capsule now spans (0,0) -> (3,0).
        rigexec.binding.generate_skin_weights(stage, MOVER)
        a = _linear_field(_segment_distance(STRIP_POINTS[4], (0, 0), (3, 0)), 1.0)
        self.assert_rows([_rows(stage)[1][4]], [[(0, a / (a + 1.0)), (1, 1.0 / (a + 1.0))]])
        capsule.GetAttribute("inputs:scaleX").Set(0.0)
        with self.assertRaises(RuntimeError):
            rigexec.binding.generate_skin_weights(stage, MOVER)

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

    def test_regenerated_fixture(self):
        """F8: widen Mid, author rigid and released rows, regenerate the rest."""
        mover = "/Arm/Rig/Movers/Bind"
        stage = _copy(_FIXTURES / "arm_3d.usda")
        size, _, indices, weights = _rows(stage, mover)
        stage.GetPrimAtPath("/Arm/Rig/Joints/Root/Mid/Influence").GetAttribute(
            "inputs:falloffMax").Set(1.4)
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
