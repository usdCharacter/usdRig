#ifndef RIGEXEC_MATH_MOVER_TRANSFORM_KERNEL_H
#define RIGEXEC_MATH_MOVER_TRANSFORM_KERNEL_H

/// A mover's transform measured in rigExec:transformSpace, from its neutral
/// reference, and carried into the frame its geometry's points are in. The
/// native, runtime and op evaluators instantiate these one definitions over
/// Gf or the runtime types, so they agree bit for bit.

namespace rigExec {

/// The length of each column of \p frame's linear part: the scale a frame
/// under a scaled master carries (world = local * parent). A zero-length
/// column reports 1.
template <class Vec3, class Mat4>
Vec3
RigExecFrameScaleT(const Mat4 &frame)
{
    Vec3 scale(1.0, 1.0, 1.0);
    for (int c = 0; c < 3; ++c) {
        const double length =
            Vec3(frame[0][c], frame[1][c], frame[2][c]).GetLength();
        if (length > 0.0) {
            scale[c] = length;
        }
    }
    return scale;
}

/// Sets the projective column exactly: a product with an affine inverse is
/// affine, but not to the last bit, and the matrix mover refuses a transform
/// whose last column is not exactly (0, 0, 0, 1).
template <class Mat4>
void
RigExecSetAffineColumnT(Mat4 *m)
{
    (*m)[0][3] = 0.0;
    (*m)[1][3] = 0.0;
    (*m)[2][3] = 0.0;
    (*m)[3][3] = 1.0;
}

/// transform * space^-1: the driver's local matrix in its space.
template <class Mat4>
Mat4
RigExecMeasureInSpaceT(const Mat4 &transform, const Mat4 &space)
{
    Mat4 m = transform * space.GetInverse();
    RigExecSetAffineColumnT(&m);
    return m;
}

/// reference^-1 * transform: an explicit neutral solve removed.
template <class Mat4>
Mat4
RigExecMeasureFromReferenceT(const Mat4 &transform, const Mat4 &reference)
{
    Mat4 m = reference.GetInverse() * transform;
    RigExecSetAffineColumnT(&m);
    return m;
}

/// space^-1 * transform: the same offset conjugated into the space's current
/// frame, so it rides the space. Identity when the transform sits at its
/// space, as RigExecMeasureInSpaceT.
template <class Mat4>
Mat4
RigExecMeasureInPosedSpaceT(const Mat4 &transform, const Mat4 &space)
{
    Mat4 m = space.GetInverse() * transform;
    RigExecSetAffineColumnT(&m);
    return m;
}

/// The offset \p m in the frame posed points are in: conjugated by a NAMED
/// \p carry (rigExec:space's rest -> pose map), else by \p space's scale.
/// The two never compose. Unposed points, or an unscaled space with no
/// carry, return \p m bit for bit; a named carry at its rest does not.
template <class Vec3, class Mat4>
Mat4
RigExecClusterInPointFrameT(const Mat4 &m, const Mat4 &space, bool posedPoints,
                            const Mat4 *carry)
{
    if (!posedPoints) {
        return m;
    }
    Mat4 out;
    if (carry) {
        out = carry->GetInverse() * m * *carry;
    } else {
        const Vec3 k = RigExecFrameScaleT<Vec3>(space);
        if (k == Vec3(1.0, 1.0, 1.0)) {
            return m;
        }
        Mat4 scale(1.0), unscale(1.0);
        scale.SetScale(k);
        unscale.SetScale(Vec3(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        out = unscale * m * scale;
    }
    RigExecSetAffineColumnT(&out);
    return out;
}

/// A mover's transform as a geometry revision applies it: measured from its
/// \p reference, then in its \p space (itself measured from
/// \p referenceSpace) and carried into the point frame. A null pointer is an
/// input the mover does not name, never an identity.
template <class Vec3, class Mat4>
Mat4
RigExecMatrixInPointFrameT(const Mat4 &transform, const Mat4 *space,
                           const Mat4 *reference, const Mat4 *referenceSpace,
                           bool posedPoints, const Mat4 *carry)
{
    Mat4 result = reference
                      ? RigExecMeasureFromReferenceT(transform, *reference)
                      : transform;
    if (space) {
        const Mat4 measured =
            referenceSpace
                ? RigExecMeasureFromReferenceT(*space, *referenceSpace)
                : *space;
        result = RigExecClusterInPointFrameT<Vec3>(
            RigExecMeasureInSpaceT(result, measured), measured, posedPoints,
            carry);
    }
    return result;
}

} // namespace rigExec

#endif
