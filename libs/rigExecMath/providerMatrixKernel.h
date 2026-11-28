#ifndef RIGEXEC_MATH_PROVIDER_MATRIX_KERNEL_H
#define RIGEXEC_MATH_PROVIDER_MATRIX_KERNEL_H

/// A provider's rest -> final and rest -> base matrices, which geometry
/// revisions and joint publication read. The baked step, the .rigexec
/// runtime and the ProviderMatrix op instantiate these one definitions.

#include "switchMathKernel.h"

#include <array>

namespace rigExec {

/// One point frame's landmarks and the two flags the matrices read.
template <class Vec3> struct RigExecProviderFrameT {
    const std::array<Vec3, 4> &points;
    bool valid;
    bool degenerate;
};

template <class Vec3>
bool
RigExecProviderFrameUsableT(const RigExecProviderFrameT<Vec3> &frame)
{
    return RigExecFrameUsableT(frame.points, frame.valid, frame.degenerate);
}

/// rest -> posed where both are usable and the rest is not singular. Where
/// only the posed frame is usable: rest -> base (identity unless both are
/// flagged valid), and for a joint then base -> posed. Identity where the
/// posed frame is not usable.
template <class Mat3, class Vec3, class Mat4>
Mat4
RigExecProviderFinalMatrixT(const RigExecProviderFrameT<Vec3> &rest,
                            const RigExecProviderFrameT<Vec3> &posed,
                            const RigExecProviderFrameT<Vec3> &base, bool joint)
{
    Mat4 matrix;
    matrix.SetIdentity();
    const bool posedUsable = RigExecProviderFrameUsableT(posed);
    if (RigExecProviderFrameUsableT(rest) && posedUsable &&
        RigExecPointsToMatrixT<Mat3>(rest.points, posed.points, &matrix))
        return matrix;
    if (!posedUsable)
        return matrix;
    matrix.SetIdentity();
    if (rest.valid && base.valid)
        RigExecPointsToMatrixT<Mat3>(rest.points, base.points, &matrix);
    if (joint) {
        Mat4 delta;
        delta.SetIdentity();
        if (RigExecPointsToMatrixT<Mat3>(base.points, posed.points, &delta))
            matrix = matrix * delta;
    }
    return matrix;
}

/// rest -> base where both are usable; identity otherwise.
template <class Mat3, class Vec3, class Mat4>
Mat4
RigExecProviderBaseMatrixT(const RigExecProviderFrameT<Vec3> &rest,
                           const RigExecProviderFrameT<Vec3> &base)
{
    Mat4 matrix;
    matrix.SetIdentity();
    if (RigExecProviderFrameUsableT(rest) && RigExecProviderFrameUsableT(base))
        RigExecPointsToMatrixT<Mat3>(rest.points, base.points, &matrix);
    return matrix;
}

} // namespace rigExec

#endif
