#
# RigExec usdview gizmo rotation: the Euler composition of avars, the
# decomposition of a world matrix back into avar angles, and the world-axis
# solving that keeps a drag on the axis the user grabbed. Depends on nothing
# but math and Gf, so frames and targets both build on it.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): everything here is
# exercised headlessly by tests/python/test_gizmo_rotation.py.
#
# Row-vector convention, leftmost factor applies first, matching the
# evaluator (libs/rigExec/computations.cpp:144-175).
#
import math

from pxr import Gf

ROTATION_ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))

_CYCLIC = ("XYZ", "YZX", "ZXY")

_IDENTITY = Gf.Matrix4d(1.0)


def _NormalizeOrder(order):
    order = str(order or "XYZ").upper()
    return order if order in ROTATION_ORDERS else "XYZ"


def _AxisRotation(axisIndex, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(_AXES[axisIndex], degrees))
    return m


def RotationFromEuler(order, rx, ry, rz):
    """Rx/Ry/Rz applied in `order` sequence, row-vector (X first for XYZ)."""
    angles = (rx, ry, rz)
    m = Gf.Matrix4d(1.0)
    for axis in _NormalizeOrder(order):
        index = "XYZ".index(axis)
        if angles[index] != 0.0:
            m = m * _AxisRotation(index, angles[index])
    return m


def _Unwrap(angle, hint):
    """`angle` shifted by a multiple of 360 to land nearest `hint`."""
    return angle + 360.0 * round((hint - angle) / 360.0)


def DecomposeEuler(matrix, order, hint=None):
    """
    Euler angles (degrees) whose RotationFromEuler(order, ...) reproduces
    the rotation part of `matrix`. Of the two solutions the one nearest
    `hint` (a 3-tuple of degrees) wins, and each angle is unwrapped to
    the hint's turn, so a drag never flips 180 degrees between events.

    Derivation: with column-vector M = R^T the product is
    M = Rk(g) * Rj(b) * Ri(a) for order (i, j, k); parity eps is +1 for
    cyclic orders. Then sin(b) = -eps*M[k][i], a = atan2(eps*M[k][j],
    M[k][k]) and g = atan2(eps*M[j][i], M[i][i]) -- verified for XYZ and
    XZY by hand, and for all six orders by the round-trip test.
    """
    order = _NormalizeOrder(order)
    i, j, k = ("XYZ".index(axis) for axis in order)
    eps = 1.0 if order in _CYCLIC else -1.0
    rot = Gf.Matrix4d(matrix).GetOrthonormalized(False)

    def M(r, c):
        # Column-vector element = transpose of the row-vector matrix.
        return rot[c][r]

    cosBeta = math.hypot(M(k, j), M(k, k))
    beta = math.atan2(-eps * M(k, i), cosBeta)
    if cosBeta > 1e-9:
        alpha = math.atan2(eps * M(k, j), M(k, k))
        gamma = math.atan2(eps * M(j, i), M(i, i))
    else:
        # Gimbal lock: alpha is free; keep the hint's and solve gamma from
        # the residual rotation about axis k.
        alpha = math.radians(hint[i]) if hint is not None else 0.0
        partial = _AxisRotation(i, math.degrees(alpha)) * \
            _AxisRotation(j, math.degrees(beta))
        residual = partial.GetInverse() * rot
        gamma = math.atan2(eps * residual[i][j], residual[i][i])

    first = [0.0, 0.0, 0.0]
    first[i], first[j], first[k] = (
        math.degrees(alpha), math.degrees(beta), math.degrees(gamma))
    if hint is None:
        return tuple(first)
    second = [0.0, 0.0, 0.0]
    second[i] = first[i] + 180.0
    second[j] = 180.0 - first[j]
    second[k] = first[k] + 180.0
    best = None
    for candidate in (first, second):
        unwrapped = [_Unwrap(c, h) for c, h in zip(candidate, hint)]
        distance = sum(abs(u - h) for u, h in zip(unwrapped, hint))
        if best is None or distance < best[0]:
            best = (distance, unwrapped)
    return tuple(best[1])


# Matrix helpers shared by the targets, and solving a
# world-axis rotation for the axis the user grabbed

def _Linear(matrix):
    """The 3x3 part as a 4x4 with zero translation."""
    m = Gf.Matrix4d(matrix)
    m.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    return m


def _RotationOnly(matrix):
    return _Linear(matrix).GetOrthonormalized(False)


def _WorldRotation(axis, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(axis).GetNormalized(), degrees))
    return m


def _RotationVector(matrix):
    """A rotation as axis * radians, the 3 free parameters of SO(3)."""
    rotation = matrix.ExtractRotation()
    axis = rotation.GetAxis()
    angle = math.radians(rotation.GetAngle())
    return [axis[i] * angle for i in range(3)]


def _FromRotationVector(vector):
    length = math.sqrt(sum(v * v for v in vector))
    if length < 1e-15:
        return Gf.Matrix4d(1.0)
    return _WorldRotation(Gf.Vec3d(*vector) / length, math.degrees(length))


# Newton refinement budget for SolveWorldRotation. The closed-form first
# guess is already exact for a rigid channel frame, so these steps only
# ever run for a sheared one. The tolerance is a residual in radians and
# cannot usefully go below ~1e-8: _RotationVector reads the angle back
# through Gf.Rotation, whose acos loses half the mantissa near identity.
_ROTATION_SOLVE_STEPS = 6

_ROTATION_SOLVE_TOLERANCE = 1e-7

_ROTATION_SOLVE_STEP = 1e-6


def SolveWorldRotation(base, channel, worldAxis, degrees):
    """
    The channel rotation R' whose ORTHONORMALIZED world frame is the
    current one turned by `degrees` about `worldAxis`, where the world
    frame is `base * channel` and only `base` may be edited.

    `channel` is the full linear part that follows the rotation channels
    (spin * P, Q, or the parent xform), NOT orthonormalized. When it is
    rigid -- the ordinary case -- the answer is the closed form of design
    spec section 2, R' = base * Cr * Rw * Cr^-1, and the loop below exits
    on its first residual check without touching it.

    When an ancestor carries non-uniform scale the channel frame is
    sheared and that conjugate is no longer a rotation, so no closed form
    exists. What the viewport shows is Gf's orthonormalization of a
    sheared matrix, and that operator is measurably neither Gram-Schmidt
    nor the polar factor, so it cannot be inverted algebraically. It is
    exactly invariant under a positive diagonal on the left and exactly
    equivariant under a rotation on the right, which is all the two lines
    above rely on. Newton on the three rotation parameters recovers the
    answer instead. The best iterate is kept rather than the last one, so
    the two escapes below (the step budget, and a singular Jacobian)
    return the closest orientation actually found, never a diverged one.
    A rigid REFLECTION is not one of those cases: the closed form is
    exact there too, since M C Rw C^-1 stays a rotation.

    The left invariance is why the avar scale S is absent here: uniform
    or not, a POSITIVE scale cannot change the drawn orientation. A
    negative one is a reflection and does; the gizmo turns the
    unreflected frame in that case.
    """
    channel = _Linear(channel)
    base = _Linear(base)
    rigid = channel.GetOrthonormalized(False)
    turn = _WorldRotation(worldAxis, degrees)
    target = (base * channel).GetOrthonormalized(False) * turn
    result = base * rigid * turn * rigid.GetInverse()

    def Residual(candidate):
        current = (candidate * channel).GetOrthonormalized(False)
        return _RotationVector(current.GetInverse() * target)

    best = None
    for _ in range(_ROTATION_SOLVE_STEPS):
        residual = Residual(result)
        length = math.sqrt(sum(v * v for v in residual))
        if best is None or length < best[0]:
            best = (length, Gf.Matrix4d(result))
        if length < _ROTATION_SOLVE_TOLERANCE:
            break
        columns = []
        for axis in range(3):
            nudge = [0.0, 0.0, 0.0]
            nudge[axis] = _ROTATION_SOLVE_STEP
            moved = Residual(result * _FromRotationVector(nudge))
            columns.append([(moved[r] - residual[r]) / _ROTATION_SOLVE_STEP
                            for r in range(3)])
        jacobian = Gf.Matrix3d(columns[0][0], columns[1][0], columns[2][0],
                               columns[0][1], columns[1][1], columns[2][1],
                               columns[0][2], columns[1][2], columns[2][2])
        if abs(jacobian.GetDeterminant()) < 1e-9:
            break
        inverse = jacobian.GetInverse()
        step = [sum(inverse[r][c] * -residual[c] for c in range(3))
                for r in range(3)]
        result = (result * _FromRotationVector(step))\
            .GetOrthonormalized(False)
    return best[1]


def GimbalAxes(order, rx, ry, rz, channelRotation):
    """
    The world unit axes of the X / Y / Z gimbal rings (design spec 8.3
    "Rotate Axis: Gimbal"), returned indexed by AXIS: [0] turns rx, [1]
    turns ry, [2] turns rz, whatever `order` is.

    For order (i, j, k) applied i first the composition is R = Ri*Rj*Rk
    (row-vector). Moving angle j alone gives R' = Ri*Rj*dj*Rk =
    R * (Rk^-1 dj Rk), and conjugation maps M^-1 d(x) M to d(x*M), so in
    CHANNEL space the j ring turns about e_j * Rk; likewise the k ring
    about e_k and the i ring about e_i * Rj * Rk (= e_i * R, since a
    rotation fixes its own axis). `channelRotation` carries those into
    world -- pass the target's GimbalFrame(), which is the frame the
    ROTATE channels compose in (it includes avars:rspin, which sits
    between the Euler product and P; ChannelFrame() does not).

    These axes are for DRAWING the rings and measuring the swept screen
    angle. To apply the drag, Gimbal mode calls the target's
    ApplyRotateChannel(axisIndex, degrees), which moves exactly the one
    channel whatever the channel frame is; every other Rotate Axis mode
    calls ApplyRotate(worldAxis, degrees). Feeding these axes back into
    ApplyRotate agrees with ApplyRotateChannel only while the channel
    frame is rigid -- see ApplyRotateChannel and SolveWorldRotation.
    """
    order = _NormalizeOrder(order)
    angles = (rx, ry, rz)
    i, j, k = ("XYZ".index(axis) for axis in order)
    rk = _AxisRotation(k, angles[k])
    rj = _AxisRotation(j, angles[j])
    axes = [None, None, None]
    axes[k] = _AXES[k]
    axes[j] = rk.TransformDir(_AXES[j])
    axes[i] = (rj * rk).TransformDir(_AXES[i])
    world = _RotationOnly(channelRotation)
    return [world.TransformDir(axis).GetNormalized() for axis in axes]
