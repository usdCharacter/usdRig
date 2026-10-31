// The radial partial's float twin: fractional-rotation rows without the
// double matrix. Header-only and USD-free: the native sparse walk and the
// zero-USD runtime include one definition, so both walks agree bit for bit.
#ifndef RIGEXEC_MATH_PARTIAL_ROWS_KERNEL_H
#define RIGEXEC_MATH_PARTIAL_ROWS_KERNEL_H

#include <cmath>

namespace rigExec {

// The fractional-rotation rows for a clamped weight \p w in (0, 1): the
// float twin of RigExecApplyPartialDecomposition, writing the 16 floats
// (four rows of three plus padding) the sparse walk's affine reads,
// without building the double matrix or narrowing it. The inputs are the
// decomposition's plain doubles -- scale factors, unit screw axis, angle
// in degrees, translation -- as both the native and the runtime
// decomposition spell them, so one definition serves both walks.
//
// The formula is the GPU deformer's (gpuDeformProgram.h Radial):
// axis-angle to quaternion to row-vector rotation, rows scaled, pivot
// translation; small angles take the linear translation. Endpoints are
// the caller's (identity rows at 0, the transform's rows at 1), as with
// the double form. A new float formulation, so its last bits differ from
// the narrowed double matrix; every float walker takes it together.
inline void
RigExecPartialRowsFloat(const double *scale, const double *axis, double angle,
                        const double *translation, float w, float *rows)
{
    constexpr float pi = 3.14159265358979323846f;
    const float radians = float(angle) * (pi / 180.0f);
    const float ax = float(axis[0]);
    const float ay = float(axis[1]);
    const float az = float(axis[2]);
    // The rotation to apply: axis-angle (axis, angle*w) as a quaternion.
    const float applied = radians * w;
    const float h = 0.5f * applied;
    const float qw = std::cos(h);
    const float s = std::sin(h);
    const float qx = ax * s;
    const float qy = ay * s;
    const float qz = az * s;
    // Row-vector rotation: the transpose of the column form, as the GPU
    // deformer's Radial spells it.
    const float qxx = qx * qx;
    const float qyy = qy * qy;
    const float qzz = qz * qz;
    const float qxy = qx * qy;
    const float qxz = qx * qz;
    const float qyz = qy * qz;
    const float qwx = qw * qx;
    const float qwy = qw * qy;
    const float qwz = qw * qz;
    float m00 = 1.0f - 2.0f * (qyy + qzz);
    float m01 = 2.0f * (qxy + qwz);
    float m02 = 2.0f * (qxz - qwy);
    float m10 = 2.0f * (qxy - qwz);
    float m11 = 1.0f - 2.0f * (qxx + qzz);
    float m12 = 2.0f * (qyz + qwx);
    float m20 = 2.0f * (qxz + qwy);
    float m21 = 2.0f * (qyz - qwx);
    float m22 = 1.0f - 2.0f * (qxx + qyy);
    // scale * rotate, row-vector: row r scales by 1 + (s_r - 1) w.
    const float k0 = 1.0f + (float(scale[0]) - 1.0f) * w;
    const float k1 = 1.0f + (float(scale[1]) - 1.0f) * w;
    const float k2 = 1.0f + (float(scale[2]) - 1.0f) * w;
    m00 *= k0;
    m01 *= k0;
    m02 *= k0;
    m10 *= k1;
    m11 *= k1;
    m12 *= k1;
    m20 *= k2;
    m21 *= k2;
    m22 *= k2;
    const float tx = float(translation[0]);
    const float ty = float(translation[1]);
    const float tz = float(translation[2]);
    float ux, uy, uz;
    // Below about a tenth of a degree there is no meaningful axis to turn
    // about and the chord and the arc agree to within float noise, so the
    // straight translation is both correct and better conditioned.
    if (std::fabs(std::sin(0.5f * radians)) < 1e-4f) {
        ux = tx * w;
        uy = ty * w;
        uz = tz * w;
    } else {
        // The translation's part along the axis (a screw's own travel,
        // linear in w) and across it, which is (I - R) applied to the
        // pivot and so names where the pivot is.
        const float along = tx * ax + ty * ay + tz * az;
        const float cx = tx - ax * along;
        const float cy = ty - ay * along;
        const float cz = tz - az * along;
        const float half = 0.5f / std::tan(0.5f * radians);
        // across x axis, then pivot = across*0.5 + (axis x across)*half.
        const float xx = ay * cz - az * cy;
        const float xy = az * cx - ax * cz;
        const float xz = ax * cy - ay * cx;
        const float px = cx * 0.5f + xx * half;
        const float py = cy * 0.5f + xy * half;
        const float pz = cz * 0.5f + xz * half;
        // p' = R_w (p - pivot) + pivot + w * along * axis, so the
        // translation is pivot - R_w * pivot + axis * (along * w) with
        // the scaled rotation above.
        const float dx = px * m00 + py * m10 + pz * m20;
        const float dy = px * m01 + py * m11 + pz * m21;
        const float dz = px * m02 + py * m12 + pz * m22;
        const float aw = along * w;
        ux = px - dx + ax * aw;
        uy = py - dy + ay * aw;
        uz = pz - dz + az * aw;
    }
    rows[0] = m00;
    rows[1] = m01;
    rows[2] = m02;
    rows[3] = 0.0f;
    rows[4] = m10;
    rows[5] = m11;
    rows[6] = m12;
    rows[7] = 0.0f;
    rows[8] = m20;
    rows[9] = m21;
    rows[10] = m22;
    rows[11] = 0.0f;
    rows[12] = ux;
    rows[13] = uy;
    rows[14] = uz;
    rows[15] = 0.0f;
}

}  // namespace rigExec

#endif
