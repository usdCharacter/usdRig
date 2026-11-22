// The SRT decomposition of a four-point frame and its inverse, shared by
// the native (Gf) frame math, the dual-quaternion split, the zero-USD
// runtime and the ops. Header-only and USD-free; the host types come from
// RigExecDualQuatMath<Point, Double>.
#ifndef RIGEXEC_MATH_FRAME_PARAMS_KERNEL_H
#define RIGEXEC_MATH_FRAME_PARAMS_KERNEL_H

#include <algorithm>
#include <array>
#include <cmath>

namespace rigExec {

template <class Point, class Double> struct RigExecDualQuatMath;

/// Jacobi eigen decomposition of a symmetric 3x3 matrix: eigenvalues descending, orthonormal eigenvectors as
// the columns of V (column convention), hemisphere- and repeated-eigenspace
// canonicalized so the pose path and this split cannot drift apart.
inline void
RigExecSymmetricEigen3Kernel(const double m[3][3], double eval[3],
                             double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            a[i][j] = m[i][j];

    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-15)
            break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18)
                    continue;
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t =
                    (theta >= 0 ? 1.0 : -1.0) /
                    (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double cth = 1.0 / std::sqrt(t * t + 1);
                const double sth = t * cth;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = cth * akp - sth * akq;
                    a[k][q] = sth * akp + cth * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = cth * apk - sth * aqk;
                    a[q][k] = sth * apk + cth * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = cth * vkp - sth * vkq;
                    v[k][q] = sth * vkp + cth * vkq;
                }
            }
        }
    }

    int order[3] = {0, 1, 2};
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    std::sort(order, order + 3, [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(evec[k][i]) > std::abs(evec[maxK][i])) {
                maxK = k;
            }
        }
        if (evec[maxK][i] < 0) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = -evec[k][i];
            }
        }
    }

    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                evec[k][i] = (k == i) ? 1.0 : 0.0;
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;
        const int other = eq01 ? 2 : 0;
        const double n[3] = {evec[0][other], evec[1][other], evec[2][other]};
        int bestAxis = 0;
        double bestLen = -1.0;
        double b1[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            double p[3];
            for (int k = 0; k < 3; ++k) {
                p[k] = ((k == axis) ? 1.0 : 0.0) - n[k] * n[axis];
            }
            const double len =
                std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (len > bestLen) {
                bestLen = len;
                bestAxis = axis;
                for (int k = 0; k < 3; ++k)
                    b1[k] = p[k] / len;
            }
        }
        (void)bestAxis;
        double b2[3] = {n[1] * b1[2] - n[2] * b1[1],
                        n[2] * b1[0] - n[0] * b1[2],
                        n[0] * b1[1] - n[1] * b1[0]};
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK]))
                maxK = k;
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k)
                b2[k] = -b2[k];
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

template <class Point, class Double> struct RigExecTransformParamsKernelT {
    typename RigExecDualQuatMath<Point, Double>::Quat rotation{1.0};
    Double translation{0.0, 0.0, 0.0};
    Double scale{1.0, 1.0, 1.0};
    Double shear{0.0, 0.0, 0.0};
    int reflectionAxis = 2;
};

/// The affine map rest->pose split into translation, rotation, scale and
/// shear by SVD. False for a singular reference or posed frame.
template <class Point, class Double>
bool
RigExecPointsToParamsKernel(
    const std::array<Double, 4> &restPoints,
    const std::array<Double, 4> &posePoints, int reflectionAxis,
    RigExecTransformParamsKernelT<Point, Double> *params)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    typename Math::Mat4 m;
    if (!Math::PointsToMatrix(restPoints, posePoints, &m)) {
        return false;
    }

    double L[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            L[i][j] = m[j][i];

    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += L[k][i] * L[k][j];
            ltl[i][j] = sum;
        }
    }
    double s2[3], V[3][3];
    RigExecSymmetricEigen3Kernel(ltl, s2, V);

    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL = L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
                        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
                        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false; // singular posed frame: no SRT decomposition
    }

    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += L[i][k] * V[k][j];
            U[i][j] = sum / sigma[j];
        }
    }

    const double delta = (detL >= 0) ? 1.0 : -1.0;

    int k = 2;
    if (delta < 0) {
        const int axis = reflectionAxis;
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }

    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                const double dkk = (kk == k) ? delta : 1.0;
                sum += U[i][kk] * dkk * V[j][kk];
            }
            R[i][j] = sum;
        }
    }

    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk)
                sum += R[kk][i] * L[kk][j];
            H[i][j] = sum;
        }
    }

    typename Math::Mat3 rowR(R[0][0], R[1][0], R[2][0], R[0][1], R[1][1],
                             R[2][1], R[0][2], R[1][2], R[2][2]);
    Math::Orthonormalize(&rowR);
    const auto q = Math::ExtractRotationQuaternion(rowR);

    params->translation = Double(m[3][0], m[3][1], m[3][2]);
    params->rotation = q.GetNormalized();
    params->scale = Double(H[0][0], H[1][1], H[2][2]);
    params->shear =
        Double(H[0][1] / H[0][0], H[0][2] / H[0][0], H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

/// The inverse of RigExecPointsToParamsKernel: L = R(q) H(s, h) with the
/// translation in row 3. H is the symmetric polar stretch, so the round trip
/// keeps the whole stretch.
template <class Point, class Double>
typename RigExecDualQuatMath<Point, Double>::Mat4
RigExecParamsToMatrixKernel(
    const RigExecTransformParamsKernelT<Point, Double> &params)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    typename Math::Mat3 rowR;
    Math::SetRotate(&rowR, params.rotation);
    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = rowR[j][i];

    const Double &sc = params.scale;
    const Double &sh = params.shear;
    double H[3][3] = {
        {sc[0], sh[0] * sc[0], sh[1] * sc[0]},
        {sh[0] * sc[0], sc[1], sh[2] * sc[1]},
        {sh[1] * sc[0], sh[2] * sc[1], sc[2]},
    };

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += R[i][k] * H[k][j];
            L[i][j] = sum;
        }
    }

    return typename Math::Mat4(L[0][0], L[1][0], L[2][0], 0.0, L[0][1], L[1][1],
                               L[2][1], 0.0, L[0][2], L[1][2], L[2][2], 0.0,
                               params.translation[0], params.translation[1],
                               params.translation[2], 1.0);
}

} // namespace rigExec

#endif
