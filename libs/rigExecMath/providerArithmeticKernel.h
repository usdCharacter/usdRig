#ifndef RIGEXEC_MATH_PROVIDER_ARITHMETIC_KERNEL_H
#define RIGEXEC_MATH_PROVIDER_ARITHMETIC_KERNEL_H
#include "rigExecSchema/schemaNames.h"
#include "avarScale.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string_view>
namespace rigExec {
/// Math supplies Matrix/Vector/Rotation/Frame and PointsToMatrix. Gf and the
/// zero-USD mirror use this exact arithmetic and matrix multiplication order.
template <class Math>
typename Math::Matrix
RigExecProviderComposeAvars(double tx, double ty, double tz, double sx,
                            double sy, double sz, double rx, double ry,
                            double rz, double spin, std::string_view order)
{
    using M = typename Math::Matrix;
    using V = typename Math::Vector;
    using R = typename Math::Rotation;
    const V axes[3] = {V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    const std::string_view sequence =
        order.size() == 3 ? order : std::string_view(SchemaValueText::xyz);
    M matrix(1.0);
    matrix.SetScale(V(RigExecNormalizeAvarScale(sx),
                      RigExecNormalizeAvarScale(sy),
                      RigExecNormalizeAvarScale(sz)));
    for (char axis : sequence) {
        const int i = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[i] != 0.0)
            matrix = matrix * M(R(axes[i], angles[i]), V(0));
    }
    if (spin != 0.0)
        matrix = matrix * M(R(axes[0], spin), V(0));
    M translation(1.0);
    translation.SetTranslate(V(tx, ty, tz));
    return matrix * translation;
}
template <class Math>
typename Math::Frame
RigExecProviderFrameFromMatrix(const typename Math::Matrix &matrix)
{
    using V = typename Math::Vector;
    const std::array<V, 4> identity = {V(0), V(1, 0, 0), V(0, 1, 0),
                                       V(0, 0, 1)};
    typename Math::Frame frame;
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = matrix.TransformAffine(identity[i]);
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite(frame.points[i][a])) {
                frame.flags = 2u;
                return frame;
            }
    }
    frame.flags = 1u;
    return frame;
}
template <class Math>
typename Math::Matrix
RigExecProviderSpaceFromFrame(const typename Math::Frame *frame)
{
    using V = typename Math::Vector;
    using M = typename Math::Matrix;
    const std::array<V, 4> identity = {V(0), V(1, 0, 0), V(0, 1, 0),
                                       V(0, 0, 1)};
    M result(1.0);
    if (frame && (!frame->IsValid() || frame->IsDegenerate() ||
                  !Math::PointsToMatrix(identity, frame->points, &result)))
        result[3][0] = std::numeric_limits<double>::quiet_NaN();
    return result;
}
} // namespace rigExec
#endif
