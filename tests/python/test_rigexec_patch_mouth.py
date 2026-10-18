"""Two lips on one patch never cross, under any mix of the ARKit avars.

examples/18_PatchMouth.usda sticks an upper and a lower lip curve, parted
at rest, to one flat NURBS patch and mixes 27 UV shapes into them, weighted
by avars named after
ARKit's jaw and mouth blend shapes. The lips must touch but never cross, and
each lip must stay a function of x, at every weight in [0, 1] -- not only at
the poses the example animates.

The structural half reads the stage. Every UV point is in one contact pair,
and the two points of a pair share one u in the neutral and in every shape.
u is linear in the weights (contacts move v only), so the narrowest a lip's
column spacing can get is the neutral spacing plus every shape's narrowing
at once; that has to stay positive.

The evaluated half poses the rig -- every shape alone, the mixes ARKit
leans on (jawOpen with mouthClose), every avar at 1, and seeded random
mixes -- with the cpu reference on, so the engine and the reference agree
on each pose, and looks at the lips from the front, where the patch maps u
to x and v to y: the two points of a column share x, the upper one is never
below the lower one, and x rises along each lip.

Usage:
    python test_rigexec_patch_mouth.py [schema_resources_dir]
"""

import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_REPO = pathlib.Path(__file__).resolve().parents[2]
_STAGE = _REPO / "examples" / "18_PatchMouth.usda"
_RIG = "/MouthAsset/Rig"
_MOVER = "/MouthAsset/Rig/Movers/Geometry/Lips"
_LIPS = "/MouthAsset/Geom/Lips.points"
# float evaluation of the patch: x and y are exact in u and v up to rounding
_TOL = 1e-5
_RANDOM_MIXES = 300


def _structure(stage):
    """The contact pairs, after checking the stage can never fold or cross."""
    mover = stage.GetPrimAtPath(_MOVER)
    lower = list(mover.GetAttribute("rigExec:uvContactLower").Get())
    upper = list(mover.GetAttribute("rigExec:uvContactUpper").Get())
    uv_target = mover.GetRelationship("rigExec:uvPoints").GetTargets()[0]
    neutral = stage.GetPrimAtPath(uv_target).GetAttribute("points").Get()
    shapes = [stage.GetPrimAtPath(t).GetAttribute("points").Get()
              for t in mover.GetRelationship("rigExec:uvShapes").GetTargets()]
    assert len(shapes) == 27, len(shapes)
    assert sorted(lower + upper) == list(range(len(neutral))), (
        "every lip point is in exactly one contact pair")
    for array in [neutral] + shapes:
        assert len(array) == len(neutral)
        for a, b in zip(lower, upper):
            assert abs(array[a][0] - array[b][0]) < 1e-6, (
                "contact %d-%d does not share u" % (a, b))
    for lip in (upper, lower):
        for a, b in zip(lip, lip[1:]):
            worst = (neutral[b][0] - neutral[a][0]) + sum(
                min(0.0, (s[b][0] - neutral[b][0]) - (s[a][0] - neutral[a][0]))
                for s in shapes)
            assert worst > 0.0, "points %d-%d can fold in u (%g)" % (
                a, b, worst)
    return lower, upper


def _crossings(points, lower, upper):
    """What is wrong with one evaluated pose, seen from the front."""
    wrong = []
    for a, b in zip(lower, upper):
        below, above = points[a], points[b]
        if abs(below[0] - above[0]) > _TOL:
            wrong.append("column %d-%d splits in x: %g vs %g"
                         % (a, b, below[0], above[0]))
        if above[1] < below[1] - _TOL:
            wrong.append("column %d-%d crosses: upper y %g below lower y %g"
                         % (a, b, above[1], below[1]))
    for lip in (upper, lower):
        for a, b in zip(lip, lip[1:]):
            if points[b][0] <= points[a][0]:
                wrong.append("lip folds between points %d and %d" % (a, b))
    return wrong


def main():
    _setup_environment()
    from pxr import Sdf, Usd
    import rigexec
    resources = sys.argv[1] if len(sys.argv) > 1 else None
    rigexec.load_schema_plugin(resources)

    stage = Usd.Stage.Open(str(_STAGE))
    assert stage, _STAGE
    lower, upper = _structure(stage)

    # Session-layer defaults outweigh the example's own time samples.
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    mover = stage.GetPrimAtPath(_MOVER)
    sources = mover.GetAttribute("inputs:uvWeights").GetConnections()
    avars = [stage.GetAttributeAtPath(path) for path in sources]
    names = [path.name.split(":", 1)[1] for path in sources]
    rig = rigexec.Rig(stage, _RIG)
    rig.cpu_reference = True
    rig.compile()

    def evaluate(weights):
        with Sdf.ChangeBlock():
            for avar, weight in zip(avars, weights):
                avar.Set(float(weight))
        pose = rig.evaluate(1001)
        assert pose.valid, [d for d in pose.diagnostics
                            if not d.startswith("warning:")]
        assert pose.reference_mismatches == 0, pose.diagnostics
        return [tuple(v) for v in pose.moved_property(_LIPS)]

    def at(**named):
        return [named.get(name, 0.0) for name in names]

    poses = [("neutral", at())]
    poses += [(name, at(**{name: 1.0})) for name in names]
    poses += [
        ("jawOpen+mouthClose", at(jawOpen=1.0, mouthClose=1.0)),
        ("half jawOpen+mouthClose", at(jawOpen=0.5, mouthClose=1.0)),
        ("every avar at 1", [1.0] * len(names)),
    ]
    rng = random.Random(18)
    for k in range(_RANDOM_MIXES):
        # Sparse mixes look like faces; dense ones push every shape at once.
        density = rng.choice((0.15, 0.4, 1.0))
        poses.append(("random mix %d" % k,
                      [rng.random() if rng.random() < density else 0.0
                       for _ in names]))

    failures = []
    for label, weights in poses:
        wrong = _crossings(evaluate(weights), lower, upper)
        if wrong:
            failures.append("%s: %s" % (label, "; ".join(wrong[:3])))

    # The lips are parted at rest and meet at the corners. The contacts
    # carry ARKit's mouthClose: over an open jaw it seals the lips, and
    # alone it closes the parted lips on the rest seam.
    rest = evaluate(at())
    sealed = evaluate(at(jawOpen=1.0, mouthClose=1.0))
    alone = evaluate(at(mouthClose=1.0))
    opened = evaluate(at(jawOpen=1.0))
    middle = (upper[len(upper) // 2], lower[len(lower) // 2])

    def gap(points, column):
        return points[upper[column]][1] - points[lower[column]][1]

    if gap(rest, len(upper) // 2) < 0.3:
        failures.append("the rest lips part by only %g"
                        % gap(rest, len(upper) // 2))
    if any(abs(gap(rest, end)) > _TOL for end in (0, len(upper) - 1)):
        failures.append("the rest lips do not meet at the corners")
    if gap(opened, len(upper) // 2) < 1.0:
        failures.append("jawOpen parts the lips by only %g"
                        % gap(opened, len(upper) // 2))
    if abs(gap(sealed, len(upper) // 2)) > _TOL:
        failures.append("jawOpen+mouthClose leaves the lips apart")
    seam = 0.5 * (rest[middle[0]][1] + rest[middle[1]][1])
    if sealed[middle[0]][1] > seam - 0.5:
        failures.append("jawOpen+mouthClose does not keep the jaw down")
    for column, (a, b) in enumerate(zip(upper, lower)):
        if (abs(gap(alone, column)) > _TOL or
                abs(alone[a][1] - 0.5 * (rest[a][1] + rest[b][1])) > _TOL):
            failures.append("mouthClose alone does not close column %d "
                            "on the rest seam" % column)
            break

    if failures:
        for line in failures[:20]:
            print("FAIL", line)
        print("%d of %d poses failed" % (len(failures), len(poses)))
        sys.exit(1)
    print("OK: %d poses, lips touch but never cross" % len(poses))


if __name__ == "__main__":
    main()
