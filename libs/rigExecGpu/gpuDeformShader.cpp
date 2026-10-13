#include "rigExecGpu/gpuDeformShader.h"

#include <string>

namespace rigExec {

namespace {

// GLSL statement for statement with RigExecRunGpuDeformProgram
// (rigExecMath/gpuDeformProgram.h), which is its CPU judge: the same op
// table, the same envelope endpoints, the same gather order. The host
// defines the accessors:
//   RX_OP(i)  int    op table
//   RX_SF(i)  float  static floats
//   RX_SI(i)  int    static ints
//   RX_FR(i)  float  frame floats
//   RX_OP_COUNT      number of ops
const char *const _body = R"GLSL(
bool rxFailed = false;

vec3 rxEnvelope(vec3 preceding, vec3 full, float w)
{
    if (w <= 0.0) return preceding;
    if (w >= 1.0) return full;
    return preceding + (full - preceding) * w;
}

vec3 rxAffine(int f, vec3 p)
{
    return vec3(p.x * RX_FR(f + 0) + p.y * RX_FR(f + 3) + p.z * RX_FR(f + 6) + RX_FR(f + 9),
                p.x * RX_FR(f + 1) + p.y * RX_FR(f + 4) + p.z * RX_FR(f + 7) + RX_FR(f + 10),
                p.x * RX_FR(f + 2) + p.y * RX_FR(f + 5) + p.z * RX_FR(f + 8) + RX_FR(f + 11));
}

vec3 rxRadial(int r, float w, vec3 p)
{
    vec3 scale = vec3(RX_FR(r + 12), RX_FR(r + 13), RX_FR(r + 14));
    vec3 axis = vec3(RX_FR(r + 15), RX_FR(r + 16), RX_FR(r + 17));
    float angle = RX_FR(r + 18) * w;
    vec3 translation = vec3(RX_FR(r + 19), RX_FR(r + 20), RX_FR(r + 21));
    float along = RX_FR(r + 22);
    vec3 pivot = vec3(RX_FR(r + 23), RX_FR(r + 24), RX_FR(r + 25));
    bool smallAngle = RX_FR(r + 26) != 0.0;
    float h = 0.5 * angle;
    float qw = cos(h);
    float s = sin(h);
    float qx = axis.x * s, qy = axis.y * s, qz = axis.z * s;
    // Row-vector rotation (rows of the transpose), each row scaled.
    vec3 r0 = vec3(1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy + qw * qz), 2.0 * (qx * qz - qw * qy));
    vec3 r1 = vec3(2.0 * (qx * qy - qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz + qw * qx));
    vec3 r2 = vec3(2.0 * (qx * qz + qw * qy), 2.0 * (qy * qz - qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
    r0 *= 1.0 + (scale.x - 1.0) * w;
    r1 *= 1.0 + (scale.y - 1.0) * w;
    r2 *= 1.0 + (scale.z - 1.0) * w;
    vec3 t = translation * w;
    if (!smallAngle) {
        vec3 dp = pivot.x * r0 + pivot.y * r1 + pivot.z * r2;
        t = pivot - dp + axis * (along * w);
    }
    return p.x * r0 + p.y * r1 + p.z * r2 + t;
}

vec3 rxDeform(int i, vec3 p)
{
    for (int o = 0; o < RX_OP_COUNT; ++o) {
        int e = o * 8;
        int kind = RX_OP(e + 0);
        int env = RX_OP(e + 1);
        int fb = RX_OP(e + 2);
        if (RX_FR(fb) == 0.0) continue;
        float w = env < 0 ? 1.0 : RX_SF(env + i);
        int a = RX_OP(e + 3);
        int b = RX_OP(e + 4);
        int c = RX_OP(e + 5);
        int d = RX_OP(e + 6);
        if (kind == 1) {
            if (w > 0.0) p = rxEnvelope(p, rxAffine(fb + 1, p), w);
        } else if (kind == 2) {
            if (w >= 1.0) p = rxAffine(fb + 1, p);
            else if (w > 0.0) p = rxRadial(fb + 1, w, p);
        } else if (kind == 3) {
            if (w <= 0.0) continue;
            vec3 sum = vec3(0.0);
            float total = 0.0;
            for (int k = 0; k < c; ++k) {
                float jw = RX_SF(b + i * c + k);
                if (jw == 0.0) continue;
                int m = fb + 1 + RX_SI(a + i * c + k) * 12;
                sum += rxAffine(m, p) * jw;
                total += jw;
            }
            p = rxEnvelope(p, p * (1.0 - total) + sum, w);
        } else if (kind == 4) {
            if (w <= 0.0) continue;
            int pivot = 0;
            float pivotWeight = -1.0;
            float total = 0.0;
            for (int k = 0; k < c; ++k) {
                float jw = RX_SF(b + i * c + k);
                if (pivotWeight < jw) { pivotWeight = jw; pivot = k; }
                total += jw;
            }
            vec4 real = vec4(0.0);
            vec4 dual = vec4(0.0);
            vec4 ref = vec4(0.0);
            bool haveRef = false;
            bool allRigid = true;
            float absSum = 0.0;
            float wSum = 0.0;
            mat3 stretch = mat3(0.0);
            // Slots: the pivot, the others in order, then the complement
            // against the identity entry at index d.
            for (int slot = -1; slot <= c; ++slot) {
                int index;
                float jw;
                if (slot == -1) {
                    index = RX_SI(a + i * c + pivot);
                    jw = RX_SF(b + i * c + pivot);
                } else if (slot < c) {
                    if (slot == pivot) continue;
                    index = RX_SI(a + i * c + slot);
                    jw = RX_SF(b + i * c + slot);
                } else {
                    index = d;
                    jw = 1.0 - total;
                }
                if (jw == 0.0) continue;
                int q = fb + 1 + index * 18;
                vec4 r = vec4(RX_FR(q + 0), RX_FR(q + 1), RX_FR(q + 2), RX_FR(q + 3));
                float sgn = 1.0;
                if (!haveRef) { ref = r; haveRef = true; }
                else if (dot(r, ref) < 0.0) sgn = -1.0;
                float sw = sgn * jw;
                real += r * sw;
                dual += vec4(RX_FR(q + 4), RX_FR(q + 5), RX_FR(q + 6), RX_FR(q + 7)) * sw;
                absSum += abs(jw);
                wSum += jw;
                if (RX_FR(q + 17) == 0.0) allRigid = false;
                // Row-major rows; GLSL mat3 is column-major, so stretch[r]
                // holds row r here.
                stretch[0] += vec3(RX_FR(q + 8), RX_FR(q + 9), RX_FR(q + 10)) * jw;
                stretch[1] += vec3(RX_FR(q + 11), RX_FR(q + 12), RX_FR(q + 13)) * jw;
                stretch[2] += vec3(RX_FR(q + 14), RX_FR(q + 15), RX_FR(q + 16)) * jw;
            }
            float norm = sqrt(dot(real, real));
            float eps = 1e-9 * max(absSum, 1.0);
            if (!haveRef || !(norm > eps)) { rxFailed = true; continue; }
            real /= norm;
            dual /= norm;
            dual -= real * dot(real, dual);
            vec3 q3 = p;
            if (!allRigid) {
                if (!(abs(wSum) > eps)) { rxFailed = true; continue; }
                float k = 1.0 / wSum;
                q3 = (p.x * stretch[0] + p.y * stretch[1] + p.z * stretch[2]) * k;
            }
            // Components are (w, x, y, z).
            vec3 u = real.yzw;
            vec3 cu = cross(u, q3);
            vec3 rotated = q3 + (cu * real.x + cross(u, cu)) * 2.0;
            vec3 dv = dual.yzw;
            vec3 t = (dv * real.x - u * dual.x + cross(u, dv)) * 2.0;
            p = rxEnvelope(p, rotated + t, w);
        } else if (kind == 5) {
            if (w <= 0.0) continue;
            vec3 delta = vec3(0.0);
            int end = RX_SI(a + i + 1);
            for (int n = RX_SI(a + i); n < end; ++n) {
                float s = RX_FR(fb + 1 + RX_SI(b + n));
                delta += vec3(RX_SF(c + 3 * n), RX_SF(c + 3 * n + 1), RX_SF(c + 3 * n + 2)) * s;
            }
            p = rxEnvelope(p, p + delta, w);
        } else if (kind == 6) {
            if (w <= 0.0) continue;
            vec3 delta = vec3(0.0);
            int end = RX_SI(a + i + 1);
            for (int n = RX_SI(a + i); n < end; ++n) {
                int dc = fb + 1 + 3 * RX_SI(b + n);
                delta += vec3(RX_FR(dc), RX_FR(dc + 1), RX_FR(dc + 2)) * RX_SF(c + n);
            }
            p = rxEnvelope(p, p + delta, w);
        }
    }
    return p;
}
)GLSL";

}  // namespace

const char *
RigExecGpuDeformKernelBody()
{
    return _body;
}

const char *
RigExecGpuDeformStormKernel()
{
    static const std::string kernel =
        std::string(R"GLSL(bool rxPrevious = false;
#define RX_OP(i) HdGet_rxOps(i)
#define RX_SF(i) HdGet_rxStaticF(i)
#define RX_SI(i) HdGet_rxStaticI(i)
#define RX_FR(i) (rxPrevious ? HdGet_rxPrevFrame(i) : HdGet_rxFrame(i))
#define RX_OP_COUNT HdGet_rxOpCount()
)GLSL") +
        _body +
        R"GLSL(
void compute(int index)
{
    vec3 base = HdGet_rxBase(index);
    vec3 p = rxDeform(index, base);
    HdSet_points(index, p);
    rxPrevious = true;
    vec3 q = rxDeform(index, base);
    HdSet_velocities(index, (p - q) * HdGet_rxVelocityScale());
}
)GLSL";
    return kernel.c_str();
}

}  // namespace rigExec
