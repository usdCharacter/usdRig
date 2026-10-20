#include "rigExecGpu/clothShader.h"

namespace rigExec {

// Each function restates the CPU pass of the same name in
// rigExecMath/clothKernel.h, with the same expressions in the same order;
// the results differ from the CPU only by float rounding (GLSL may fuse a
// multiply-add or round a division differently).
//
// One dispatch runs one pass over the whole cloth, so a solve spreads over
// every compute unit; the executor issues the passes in the CPU solve's
// order with a storage barrier between them:
//   Init      x and v from the first two ticks, then Predict (tick 1, sub 1)
//   Color     one constraint color of one iteration
//   Collider  the collider at this substep: each vertex where it stood a
//             substep before, and each triangle's corners now and its unit
//             normal, one 64-byte record a contact test reads at once
//   Particle  tether, goal and collision; after the last iteration also
//             Velocity, then Predict for the next substep (or the output)
// A color's first iteration starts its multipliers at zero, as the CPU's
// per-substep reset does. A tether reads its kinematic anchor where Predict
// placed it, recomputed, since the anchor's own thread may already have
// moved it on to the next substep.
//
// Buffers (the executor fills them, rigExecGpu/gpuDeformExecutor.cpp):
//   meshI   constraint ends (2C), color starts (K+1), tether anchors (N),
//           collider triangles (3T), constraint kinds (C)
//   meshF   constraint rest lengths (C), inverse masses (N), pins (N),
//           tether rest lengths (N)
//   tickP   per tick: entering points (3N), then collider points (3M)
//   params  per tick: the ten RigExecClothParameters floats
//   grid    per tick from the second: bucket starts, then bucket triangles
//   head    per tick: bucket-start offset, triangle offset, mask, cell bits
//   state   x (3N), previous (3N), velocity (3N), lambda (C), collider
//           vertices a substep before (3M), triangle records (16T: three
//           corners, the normal, and whether it has one)
const char *
RigExecGpuClothKernelSource()
{
    return R"GLSL(#version 450
layout(local_size_x = 64) in;
layout(std430, binding = 0) readonly buffer RxMeshI { int meshI[]; };
layout(std430, binding = 1) readonly buffer RxMeshF { float meshF[]; };
layout(std430, binding = 2) readonly buffer RxTickP { float tickP[]; };
layout(std430, binding = 3) readonly buffer RxParams { float params[]; };
layout(std430, binding = 4) readonly buffer RxGrid { uint grid[]; };
layout(std430, binding = 5) readonly buffer RxHead { uint head[]; };
layout(std430, binding = 6) buffer RxState { float state[]; };
layout(std430, binding = 7) writeonly buffer RxOut { float outPoints[]; };

uniform int N;
uniform int M;
uniform int C;
uniform int T;
uniform int tickCount;
uniform int substeps;
uniform float frameSeconds;
uniform float tetherSlack;
uniform int oEnds;
uniform int oColor;
uniform int oAnchor;
uniform int oTris;
uniform int oKind;
uniform int oRest;
uniform int oInv;
uniform int oPin;
uniform int oTether;
// The pass and where in the solve it stands.
uniform int pass;
uniform int tick;
uniform int sub;
uniform int color;
uniform int firstIteration;
uniform int lastIteration;

const int kInit = 0, kColor = 1, kParticle = 2, kCollider = 3;

int stride() { return 3 * N + 3 * M; }
vec3 xAt(int i) { return vec3(state[3 * i], state[3 * i + 1], state[3 * i + 2]); }
void xSet(int i, vec3 p) { state[3 * i] = p.x; state[3 * i + 1] = p.y; state[3 * i + 2] = p.z; }
vec3 prevAt(int i) { int b = 3 * N + 3 * i; return vec3(state[b], state[b + 1], state[b + 2]); }
void prevSet(int i, vec3 p) { int b = 3 * N + 3 * i; state[b] = p.x; state[b + 1] = p.y; state[b + 2] = p.z; }
vec3 velAt(int i) { int b = 6 * N + 3 * i; return vec3(state[b], state[b + 1], state[b + 2]); }
void velSet(int i, vec3 v) { int b = 6 * N + 3 * i; state[b] = v.x; state[b + 1] = v.y; state[b + 2] = v.z; }
vec3 entering(int k, int i) { int b = k * stride() + 3 * i; return vec3(tickP[b], tickP[b + 1], tickP[b + 2]); }
vec3 colliderAt(int k, int v) { int b = k * stride() + 3 * N + 3 * v; return vec3(tickP[b], tickP[b + 1], tickP[b + 2]); }
int oThen() { return 9 * N + C; }
int oTri() { return 9 * N + C + 3 * M; }
vec3 thenAt(int v) { int b = oThen() + 3 * v; return vec3(state[b], state[b + 1], state[b + 2]); }
vec3 corner(int tri, int c) { int b = oTri() + 16 * tri + 3 * c; return vec3(state[b], state[b + 1], state[b + 2]); }
vec3 lerp3(vec3 a, vec3 b, float t) { return a + (b - a) * t; }
float len3(vec3 d) { return sqrt(d.x * d.x + d.y * d.y + d.z * d.z); }
float dot3(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
vec3 cross3(vec3 a, vec3 b) {
    return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

struct Step {
    float h, t, t0, alphaStretch, alphaBend, goalExponent, damp, attract,
          thickness, depth, friction;
    vec3 gravity;
};

// RigExecClothStepFor.
Step stepFor(int k, int s0) {
    int b = 10 * k;
    Step s;
    s.h = frameSeconds / float(substeps);
    s.t = float(s0) / float(substeps);
    s.t0 = float(s0 - 1) / float(substeps);
    s.alphaStretch = max(0.0, params[b]) / (s.h * s.h);
    s.alphaBend = max(0.0, params[b + 1]) / (s.h * s.h);
    s.goalExponent = 1.0 / float(substeps);
    s.damp = max(0.0, 1.0 - max(0.0, params[b + 5]) * s.h);
    s.attract = min(1.0, max(0.0, params[b + 6]));
    s.thickness = max(0.0, params[b + 7]);
    s.depth = max(0.0, params[b + 8]);
    s.friction = max(0.0, params[b + 9]);
    s.gravity = vec3(params[b + 2], params[b + 3], params[b + 4]);
    return s;
}

// RigExecClothPredict.
void predict(Step s, int k, int i) {
    vec3 p = xAt(i);
    prevSet(i, p);
    if (meshF[oInv + i] == 0.0) {
        xSet(i, lerp3(entering(k - 1, i), entering(k, i), s.t));
        return;
    }
    vec3 v = velAt(i);
    v = v + s.gravity * s.h;
    p = p + v * s.h;
    velSet(i, v);
    xSet(i, p);
}

// RigExecClothProjectConstraint.
void project(Step s, int c) {
    int a = meshI[oEnds + 2 * c];
    int b = meshI[oEnds + 2 * c + 1];
    float wa = meshF[oInv + a];
    float wb = meshF[oInv + b];
    float alpha = meshI[oKind + c] == 0 ? s.alphaStretch : s.alphaBend;
    vec3 pa = xAt(a), pb = xAt(b);
    vec3 d = pa - pb;
    float len = len3(d);
    float denom = wa + wb + alpha;
    float lambda = firstIteration != 0 ? 0.0 : state[9 * N + c];
    if (!(len > 0.0) || !(denom > 0.0)) {
        state[9 * N + c] = lambda;
        return;
    }
    float Cc = len - meshF[oRest + c];
    float dl = (-Cc - alpha * lambda) / denom;
    state[9 * N + c] = lambda + dl;
    float ka = wa * dl / len;
    float kb = wb * dl / len;
    xSet(a, pa + d * ka);
    xSet(b, pb - d * kb);
}

// cloth_detail::ClosestBarycentric (Ericson 5.1.5).
vec3 closestBarycentric(vec3 p, vec3 a, vec3 b, vec3 c) {
    vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot3(ab, ap), d2 = dot3(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return vec3(1, 0, 0);
    vec3 bp = p - b;
    float d3 = dot3(ab, bp), d4 = dot3(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return vec3(0, 1, 0);
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        float v = d1 / (d1 - d3);
        return vec3(1.0 - v, v, 0.0);
    }
    vec3 cp = p - c;
    float d5 = dot3(ab, cp), d6 = dot3(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return vec3(0, 0, 1);
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        float v = d2 / (d2 - d6);
        return vec3(1.0 - v, 0.0, v);
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return vec3(0.0, 1.0 - v, v);
    }
    float denom = 1.0 / (va + vb + vc);
    float v = vb * denom, u = vc * denom;
    return vec3(1.0 - v - u, v, u);
}

int cellOf(float v, float cell) { return int(floor(v / cell)); }

// One candidate triangle of RigExecClothFindContact: kept when it is the
// closest violated one so far, a tie going to the lower triangle.
void testTriangle(Step s, vec3 p, float reach, int tri, inout float bestDistance,
                  inout int bestTri, inout float violation, inout vec3 normal,
                  inout vec3 weights) {
    int rb = oTri() + 16 * tri;
    if (!(state[rb + 12] > 0.0)) return;
    vec3 v0 = vec3(state[rb], state[rb + 1], state[rb + 2]);
    vec3 v1 = vec3(state[rb + 3], state[rb + 4], state[rb + 5]);
    vec3 v2 = vec3(state[rb + 6], state[rb + 7], state[rb + 8]);
    vec3 n = vec3(state[rb + 9], state[rb + 10], state[rb + 11]);
    vec3 w = closestBarycentric(p, v0, v1, v2);
    vec3 q = vec3(w.x * v0.x + w.y * v1.x + w.z * v2.x,
                  w.x * v0.y + w.y * v1.y + w.z * v2.y,
                  w.x * v0.z + w.y * v1.z + w.z * v2.z);
    vec3 d = p - q;
    float signedDistance = dot3(d, n);
    float distance = len3(d);
    if (distance > reach) return;
    if (signedDistance >= s.thickness || signedDistance < -s.depth) return;
    if (signedDistance >= 0.0 && distance >= s.thickness) return;
    if (distance < bestDistance || (distance == bestDistance && tri < bestTri)) {
        bestDistance = distance;
        bestTri = tri;
        violation = s.thickness - signedDistance;
        normal = n;
        weights = w;
    }
}

// RigExecClothFindContact over tick k's grid: the one bucket of p's own
// cell, whose triangles are each listed once.
bool findContact(Step s, int k, vec3 p, out int bestTri, out float violation,
                 out vec3 normal, out vec3 weights) {
    bestTri = -1;
    violation = 0.0;
    normal = vec3(0);
    weights = vec3(0);
    uint h = uint(4 * k);
    uint starts = head[h], tris = head[h + 1u], mask = head[h + 2u];
    float cell = uintBitsToFloat(head[h + 3u]);
    float reach = max(s.thickness, s.depth);
    float bestDistance = uintBitsToFloat(0x7f800000u);
    int x = cellOf(p.x, cell), y = cellOf(p.y, cell), z = cellOf(p.z, cell);
    uint bucket = ((uint(x) * 73856093u) ^ (uint(y) * 19349663u) ^ (uint(z) * 83492791u)) & mask;
    uint begin = grid[starts + bucket], end = grid[starts + bucket + 1u];
    for (uint e = begin; e < end; ++e)
        testTriangle(s, p, reach, int(grid[tris + e]), bestDistance, bestTri, violation,
                     normal, weights);
    return bestTri >= 0;
}

// RigExecClothParticlePass.
void particlePass(Step s, int k, int i, bool first) {
    if (meshF[oInv + i] == 0.0) return;
    vec3 p = xAt(i);
    int anchor = meshI[oAnchor + i];
    if (anchor >= 0) {
        // The anchor is kinematic: where Predict put it this substep.
        vec3 d = p - lerp3(entering(k - 1, anchor), entering(k, anchor), s.t);
        float len = len3(d);
        float limit = meshF[oTether + i] * (1.0 + tetherSlack);
        if (len > limit && len > 0.0) {
            float kk = (len - limit) / len;
            p = p - d * kk;
        }
    }
    if (first) {
        float goal = 1.0 - (1.0 - s.attract) * (1.0 - meshF[oPin + i]);
        if (goal > 0.0) {
            float g = 1.0 - pow(1.0 - goal, s.goalExponent);
            vec3 target = lerp3(entering(k - 1, i), entering(k, i), s.t);
            p = p + (target - p) * g;
        }
    }
    if (T > 0 && M > 0) {
        int tri;
        float violation;
        vec3 n, w;
        if (findContact(s, k, p, tri, violation, n, w)) {
            p = p + n * violation;
            if (s.friction > 0.0) {
                vec3 now = vec3(0), then = vec3(0);
                for (int c = 0; c < 3; ++c) {
                    vec3 a1 = corner(tri, c);
                    vec3 a0 = thenAt(meshI[oTris + 3 * tri + c]);
                    now = now + w[c] * a1;
                    then = then + w[c] * a0;
                }
                vec3 rel = (p - prevAt(i)) - (now - then);
                float along = dot3(rel, n);
                vec3 tangent = rel - n * along;
                float slide = len3(tangent);
                if (slide > 0.0) {
                    float kk = min(1.0, s.friction * violation / slide);
                    p = p - tangent * kk;
                }
            }
        }
    }
    xSet(i, p);
}

// RigExecClothVelocity.
void velocity(Step s, int i) {
    if (meshF[oInv + i] == 0.0) return;
    vec3 d = xAt(i) - prevAt(i);
    velSet(i, d / s.h * s.damp);
}

void main() {
    int g = int(gl_GlobalInvocationID.x);
    if (pass == kColor) {
        int c = meshI[oColor + color] + g;
        if (c < meshI[oColor + color + 1]) project(stepFor(tick, sub), c);
        return;
    }
    if (pass == kCollider) {
        Step s = stepFor(tick, sub);
        if (g < M) {
            vec3 a0 = lerp3(colliderAt(tick - 1, g), colliderAt(tick, g), s.t0);
            int c = oThen() + 3 * g;
            state[c] = a0.x; state[c + 1] = a0.y; state[c + 2] = a0.z;
        }
        if (g < T) {
            // The vertices again, by the same expression, so this needs no
            // pass before it.
            int i0 = meshI[oTris + 3 * g], i1 = meshI[oTris + 3 * g + 1], i2 = meshI[oTris + 3 * g + 2];
            vec3 v0 = lerp3(colliderAt(tick - 1, i0), colliderAt(tick, i0), s.t);
            vec3 v1 = lerp3(colliderAt(tick - 1, i1), colliderAt(tick, i1), s.t);
            vec3 v2 = lerp3(colliderAt(tick - 1, i2), colliderAt(tick, i2), s.t);
            vec3 n = cross3(v1 - v0, v2 - v0);
            float len = len3(n);
            if (len > 0.0) n = vec3(n.x / len, n.y / len, n.z / len);
            int rb = oTri() + 16 * g;
            state[rb] = v0.x; state[rb + 1] = v0.y; state[rb + 2] = v0.z;
            state[rb + 3] = v1.x; state[rb + 4] = v1.y; state[rb + 5] = v1.z;
            state[rb + 6] = v2.x; state[rb + 7] = v2.y; state[rb + 8] = v2.z;
            state[rb + 9] = n.x; state[rb + 10] = n.y; state[rb + 11] = n.z;
            state[rb + 12] = len > 0.0 ? 1.0 : 0.0;
        }
        return;
    }
    if (g >= N) return;
    if (pass == kInit) {
        for (int a = 0; a < 3; ++a) {
            float e0 = tickP[3 * g + a];
            float e1 = tickP[stride() + 3 * g + a];
            state[3 * g + a] = e0;
            state[6 * N + 3 * g + a] = (e1 - e0) / frameSeconds;
        }
        predict(stepFor(1, 1), 1, g);
        return;
    }
    // kParticle
    Step s = stepFor(tick, sub);
    particlePass(s, tick, g, firstIteration != 0);
    if (lastIteration == 0) return;
    velocity(s, g);
    int nextTick = sub < substeps ? tick : tick + 1;
    int nextSub = sub < substeps ? sub + 1 : 1;
    if (nextTick < tickCount) {
        predict(stepFor(nextTick, nextSub), nextTick, g);
    } else {
        vec3 p = xAt(g);
        outPoints[3 * g] = p.x;
        outPoints[3 * g + 1] = p.y;
        outPoints[3 * g + 2] = p.z;
    }
}
)GLSL";
}

}  // namespace rigExec
