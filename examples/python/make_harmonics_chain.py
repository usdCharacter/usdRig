#!/usr/bin/env python3
"""Build examples/19_HarmonicsChain.usda: harmonics on a chain, read as rotation.

A four-segment stalk (joints Seg0..Seg3 and an end joint Tip, skinned to a
tapered tube with a ball on top) stands on a pad that the Base control
dashes about. Each segment's joint aims at a target control that rides the
pad one segment higher (Aim1..Aim4), and a RigExecHarmonicsConstraint rings
each target with the pad's motion over the frames before the current one.
The aims turn the swinging targets into joint rotation.

The rings are tuned so the stalk bends like a whip rather than tilting as
one piece or thrashing:

  * Displacement grows about with the square of the height: amplitude
    times waveLength (matchVelocity scales a ring by its period) goes
    roughly 1 : 4 : 9 : 16 up the targets, so the base barely tilts and
    the top segment swings most.
  * The period lengthens up the stalk (8 to 11 frames), so the upper
    targets peak later and the bend travels upward.
  * Two waves with a decay of 3 settle each ring inside the 18-frame holds
    between dashes.

Usage:
    python make_harmonics_chain.py [output.usda]

The output defaults to examples/19_HarmonicsChain.usda. Only the Python
standard library is needed.
"""

import math
import pathlib
import sys

SEG = 6.0          # segment length
NSEG = 4           # Seg0..Seg3 rotate; Tip is the end joint
HEIGHT = SEG * NSEG
BLEND = 1.5        # half width of the skin blend around each inner joint
SIDES = 12
RING_STEP = 0.75
# Per target Aim1..Aim4: frames per period (at rigExec:frequency 1) and
# inputs:amplitude.
WAVE_LENGTH = [8.0, 9.0, 10.0, 11.0]
AMPLITUDE = [0.1, 0.35, 0.7, 1.1]
WAVES = 2.0
DECAY = 3.0

ROOT = "/AntennaAsset"
RIG = ROOT + "/Rig"
CTL = RIG + "/Controls"
JNT = RIG + "/Joints"
JOINT_PATHS = []
_p = JNT
for name in ["Seg0", "Seg1", "Seg2", "Seg3", "Tip"]:
    _p = _p + "/" + name
    JOINT_PATHS.append(_p)


def fmt(v):
    s = "%.6g" % v
    return "0" if s == "-0" else s


def vec(v):
    return "(" + ", ".join(fmt(c) for c in v) + ")"


def wrap(items, per_line, indent):
    lines = []
    for i in range(0, len(items), per_line):
        lines.append(indent + ", ".join(items[i:i + per_line]) + ",")
    return "\n".join(lines)


def translate(x, y, z):
    return "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (%s, %s, %s, 1) )" % (fmt(x), fmt(y), fmt(z))


# ---- meshes --------------------------------------------------------------
def stalk():
    rings = int(round(HEIGHT / RING_STEP)) + 1
    pts, idx, wts = [], [], []
    for r in range(rings):
        h = r * RING_STEP
        radius = 1.2 - 0.6 * (h / HEIGHT)
        # Skin: one influence inside a segment, a smoothstep pair across
        # an inner joint.
        k = min(int(h // SEG), NSEG - 1)
        pair = (k, 0, 1.0, 0.0)
        for j in range(1, NSEG):
            yj = j * SEG
            if abs(h - yj) < BLEND:
                t = (h - (yj - BLEND)) / (2 * BLEND)
                s = t * t * (3 - 2 * t)
                pair = (j - 1, j, 1.0 - s, s)
        for i in range(SIDES):
            a = 2 * math.pi * i / SIDES
            pts.append((radius * math.cos(a), h, -radius * math.sin(a)))
            idx += [pair[0], pair[1]]
            wts += [pair[2], pair[3]]
    counts, verts = [], []
    for r in range(rings - 1):
        for i in range(SIDES):
            a = r * SIDES + i
            b = r * SIDES + (i + 1) % SIDES
            counts.append(4)
            verts += [a, b, b + SIDES, a + SIDES]
    return pts, counts, verts, idx, wts


def box(x0, x1, y0, y1, z0, z1):
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1),
           (x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)]
    verts = [0, 1, 2, 3, 4, 7, 6, 5, 3, 2, 6, 7, 0, 4, 5, 1, 0, 3, 7, 4, 1, 5, 6, 2]
    return pts, [4] * 6, verts


def sphere(center, radius, rings=8, sides=12):
    cx, cy, cz = center
    pts = [(cx, cy + radius, cz)]
    for r in range(1, rings):
        phi = math.pi * r / rings
        for i in range(sides):
            a = 2 * math.pi * i / sides
            pts.append((cx + radius * math.sin(phi) * math.cos(a),
                        cy + radius * math.cos(phi),
                        cz - radius * math.sin(phi) * math.sin(a)))
    pts.append((cx, cy - radius, cz))
    bottom = len(pts) - 1
    counts, verts = [], []
    for i in range(sides):
        counts.append(3)
        verts += [0, 1 + i, 1 + (i + 1) % sides]
    for r in range(rings - 2):
        for i in range(sides):
            a = 1 + r * sides + i
            b = 1 + r * sides + (i + 1) % sides
            counts.append(4)
            verts += [a, a + sides, b + sides, b]
    last = 1 + (rings - 2) * sides
    for i in range(sides):
        counts.append(3)
        verts += [bottom, last + (i + 1) % sides, last + i]
    return pts, counts, verts


def mesh(name, pts, counts, verts, color, extra=""):
    return """        def Mesh "%s"
        {
            int[] faceVertexCounts = [
%s
            ]
            int[] faceVertexIndices = [
%s
            ]
            point3f[] points = [
%s
            ]
            color3f[] primvars:displayColor = [%s]
            uniform token subdivisionScheme = "none"%s
        }
""" % (name, wrap([str(c) for c in counts], 24, "                "),
       wrap([str(v) for v in verts], 24, "                "),
       wrap([vec(p) for p in pts], 4, "                "), vec(color), extra)


stalk_pts, stalk_counts, stalk_verts, skin_idx, skin_wts = stalk()
pad_pts, pad_counts, pad_verts = box(-4, 4, -1.5, 0, -4, 4)
ball_pts, ball_counts, ball_verts = sphere((0, HEIGHT, 0), 1.8)

# ---- animation -----------------------------------------------------------
TX = {1001: 0, 1006: 0, 1012: 16, 1030: 16, 1036: -8, 1054: -8, 1060: 0}
TZ = {1001: 0, 1030: 0, 1036: 8, 1054: 8, 1060: 0}


def samples(d):
    return "{ " + ", ".join("%d: %s" % (k, fmt(v)) for k, v in sorted(d.items())) + " }"


# ---- rig -----------------------------------------------------------------
def target_control(k):
    return """                def RigExecControl "Aim%d" (
                    prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
                )
                {
                    double guide:scaleX = 0.25
                    double guide:scaleY = 0.25
                    double guide:scaleZ = 0.25
                    uniform token guide:shape = "sphere"
                    matrix4d rest:space = %s
                    uniform asset ui:nodegraph:node:icon = @../icons/control.png@
                }
""" % (k, translate(0, SEG * k, 0))


def joints():
    names = ["Seg0", "Seg1", "Seg2", "Seg3", "Tip"]
    out = []
    indent = "            "
    for i, name in enumerate(names):
        y = 0 if i == 0 else SEG
        out.append('%sdef RigExecJoint "%s" (\n%s    prepend apiSchemas = ["NodeGraphNodeAPI"]\n%s)\n%s{\n'
                   % (indent, name, indent, indent, indent))
        out.append("%s    double guide:radius = 0.35\n" % indent)
        out.append("%s    matrix4d rest:space = %s\n" % (indent, translate(0, y, 0)))
        out.append("%s    uniform asset ui:nodegraph:node:icon = @../icons/joint.png@\n" % indent)
        indent += "    "
    for i in range(len(names)):
        indent = indent[:-4]
        out.append("%s}\n" % indent)
    return "".join(out)


def jiggle(k):
    return """                    def RigExecHarmonicsConstraint "Jiggle%d" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        rel rigExec:moves = <%s/Base/Aim%d>
                        uniform double rigExec:waves = %s
                        uniform double rigExec:waveLength = %s
                        double inputs:amplitude = %s
                        double inputs:decay = %s
                        double3 inputs:axisAmplitude = (1, 0, 1)
                    }
""" % (k, CTL, k, fmt(WAVES), fmt(WAVE_LENGTH[k - 1]), fmt(AMPLITUDE[k - 1]), fmt(DECAY))


def aim(k):
    return """                    def RigExecAimConstraint "AimSeg%d" (
                        prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
                    )
                    {
                        double3 inputs:aimVector = (0, 1, 0)
                        double3 inputs:upVector = (0, 0, 1)
                        double3 inputs:worldUpVector = (0, 0, 1)
                        rel rigExec:moves = <%s>
                        rel rigExec:sources = <%s/Base/Aim%d>
                        uniform token rigExec:worldUpType = "vector"
                        uniform asset ui:nodegraph:node:icon = @../icons/aim_constraint.png@
                    }
""" % (k, JOINT_PATHS[k], CTL, k + 1)


DOC = '''
    RigExec example 19 - harmonics on a chain, read as rotation.

    A four-segment stalk stands on a pad that dashes right, then back and
    across. Each segment's joint aims at a target control that rides the
    pad one segment higher, and a RigExecHarmonicsConstraint rings each
    target with the pad's motion over the frames before the current one.
    The rings grow longer and larger up the stalk, so after every stop the
    targets swing past and settle late, and the aim constraints turn that
    swinging translation into joint rotation: the stalk bends behind the
    pad, whips past it and settles, with its segment lengths unchanged.

    Stack order, bottom first: the jiggles ring the targets, the parent
    constraint carries Seg0 with the pad, and the aims run from the base
    up, so each segment aims from where the one below it left it. The skin
    and the two matrix movers read the joints at final.
    '''

aims = "".join(aim(k) for k in reversed(range(NSEG)))
jiggles = "".join(jiggle(k) for k in reversed(range(1, NSEG + 1)))
influences = ",\n".join("                        <%s>" % p for p in JOINT_PATHS[:NSEG])

text = '''#usda 1.0
(
    """%s"""
    defaultPrim = "AntennaAsset"
    endTimeCode = 1082
    metersPerUnit = 0.01
    startTimeCode = 1001
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "AntennaAsset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "AntennaAsset"

        def Scope "Controls"
        {
            def RigExecControl "Base" (
                prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
            )
            {
                double avars:tx = 0
                double avars:tx.timeSamples = %s
                double avars:tz = 0
                double avars:tz.timeSamples = %s
                double guide:scaleX = 5
                double guide:scaleY = 0.4
                double guide:scaleZ = 5
                uniform token guide:shape = "box"
                matrix4d rest:space = %s
                uniform asset ui:nodegraph:node:icon = @../icons/control.png@

%s            }
        }

        def Scope "Joints"
        {
%s        }

        def Scope "Weights"
        {
            def RigExecStaticWeight "PadW" (
                prepend apiSchemas = ["NodeGraphNodeAPI"]
            )
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </AntennaAsset/Geom/Pad.points>
                uniform asset ui:nodegraph:node:icon = @../icons/static_weight.png@
            }

            def RigExecStaticWeight "BallW" (
                prepend apiSchemas = ["NodeGraphNodeAPI"]
            )
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </AntennaAsset/Geom/Ball.points>
                uniform asset ui:nodegraph:node:icon = @../icons/static_weight.png@
            }
        }

        def Scope "Movers"
        {
            # The hierarchy is the displayed stack: bottom executes first.
            # Pose sits below Geometry so the skin reads the final joints.
            reorder nameChildren = ["Geometry", "Pose"]
            def Scope "Pose"
            {
                # Jiggle rings the targets, Follow carries the base joint,
                # then Aim turns each segment toward its rung target.
                reorder nameChildren = ["Aim", "Follow", "Jiggle"]
                def Scope "Aim"
                {
                    # Bottom first: Seg0 aims before Seg1 reads where it is.
%s                }

                def Scope "Follow"
                {
                    def RigExecParentConstraint "PadFollow" (
                        prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
                    )
                    {
                        rel rigExec:moves = <%s>
                        rel rigExec:sources = <%s/Base>
                        uniform asset ui:nodegraph:node:icon = @../icons/parent_constraint.png@
                    }
                }

                def Scope "Jiggle"
                {
%s                }
            }

            def Scope "Geometry"
            {
                def RigExecSkinMover "StalkSkin" (
                    prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
                )
                {
                    uniform int rigExec:elementSize = 2
                    rel rigExec:influences = [
%s,
                    ] (
                        rigExecReadPhase = "final"
                    )
                    int[] rigExec:jointIndices = [
%s
                    ]
                    float[] rigExec:jointWeights = [
%s
                    ]
                    rel rigExec:moves = </AntennaAsset/Geom/Stalk.points>
                    uniform token rigExec:skinningMethod = "dualQuaternion"
                    uniform asset ui:nodegraph:node:icon = @../icons/skin_mover.png@
                }

                def RigExecMatrixMover "PadSkin" (
                    prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
                )
                {
                    rel rigExec:moves = </AntennaAsset/Geom/Pad.points>
                    rel rigExec:transform = <%s/Base> (
                        rigExecReadPhase = "final"
                    )
                    rel rigExec:weightObject = </AntennaAsset/Rig/Weights/PadW>
                    uniform asset ui:nodegraph:node:icon = @../icons/matrix_mover.png@
                }

                def RigExecMatrixMover "BallSkin" (
                    prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
                )
                {
                    rel rigExec:moves = </AntennaAsset/Geom/Ball.points>
                    rel rigExec:transform = <%s> (
                        rigExecReadPhase = "final"
                    )
                    rel rigExec:weightObject = </AntennaAsset/Rig/Weights/BallW>
                    uniform asset ui:nodegraph:node:icon = @../icons/matrix_mover.png@
                }
            }
        }
    }

    def Scope "Geom"
    {
%s
%s
%s    }
}
''' % (DOC, samples(TX), samples(TZ), translate(0, 0, 0),
       "\n".join(target_control(k) for k in range(1, NSEG + 1)),
       joints(), aims, JOINT_PATHS[0], CTL, jiggles, influences,
       wrap([str(i) for i in skin_idx], 24, "                        "),
       wrap([fmt(w) for w in skin_wts], 12, "                        "),
       CTL, JOINT_PATHS[-1],
       mesh("Stalk", stalk_pts, stalk_counts, stalk_verts, (0.35, 0.65, 0.4)),
       mesh("Pad", pad_pts, pad_counts, pad_verts, (0.3, 0.35, 0.45)),
       mesh("Ball", ball_pts, ball_counts, ball_verts, (0.9, 0.4, 0.2)))


def main():
    default = pathlib.Path(__file__).resolve().parents[1] / "19_HarmonicsChain.usda"
    out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else default
    with open(out, "w", newline="\n") as f:
        f.write(text)
    print("wrote", out)


if __name__ == "__main__":
    main()
