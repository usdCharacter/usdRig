// Runtime aggregate solvers: FK, two-bone IK, spline IK, ribbon, and twist.

#include "rigExecSchema/schemaNames.h"
#include "poseInternal.h"
#include "dualQuatMath.h"
#include "rigExecMath/chainSolverKernel.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecMath/splineIkKernel.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Solver kernels (rigExecMath/solvers.cpp, splineIk.cpp): Gf -> Rr, the
// arithmetic untouched.

// RigExecSolveFkChain (rigExecMath/chainSolverKernel.h).
std::vector<RrPointFrame>
_RrSolveFkChain(const std::vector<RrPoseFkElement> &elements)
{
    return RigExecSolveFkChainKernel<RrVec3f, RrVec3d, RrPointFrame>(elements);
}

// RigExecTwoBoneIkLengths.
void
_RrTwoBoneIkLengths(const std::array<std::array<RrVec3d, 4>, 3> &restPoints,
                    const RrMat4d &space, double upperOffset,
                    double lowerOffset, double *upperLength,
                    double *lowerLength)
{
    RigExecTwoBoneIkLengthsKernel(restPoints, space, upperOffset, lowerOffset,
                                  upperLength, lowerLength);
}

// RigExecSolveTwoBoneIk.
std::array<RrPointFrame, 3>
_RrSolveTwoBoneIk(const RrPointFrame &rootFrame,
                  const RrPointFrame &effectorFrame,
                  const RrPointFrame &poleFrame,
                  const std::array<std::array<RrVec3d, 4>, 3> &restPoints,
                  const RrPoseTwoBoneIkParams &params)
{
    return RigExecSolveTwoBoneIkKernel<RrVec3f, RrVec3d>(
        rootFrame, effectorFrame, poleFrame, restPoints, params);
}

// RigExecSplineIkMakeRest (rigExecMath/splineIkKernel.h).
RrPoseSplineIkRest
_RrSplineIkMakeRest(const std::vector<RrPointFrame> &joints,
                    const RrPointFrame &rootControl,
                    const RrPointFrame &midControl,
                    const RrPointFrame &endControl,
                    const std::vector<double> &volumeWeights,
                    uint8_t restLengthChain)
{
    RrPoseSplineIkRest rest;
    RigExecSplineIkMakeRestKernel<RrVec3d>(joints, rootControl, midControl,
                                           endControl, volumeWeights,
                                           restLengthChain != 0, &rest);
    return rest;
}

// Rebuilds one solver's rest description from the ladder this run
// composed (RefreshSolverRests, bakedPose.cpp).
bool
_RrRefreshSolverRests(RrProgram *program, size_t step, size_t solver,
                      std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSolver &wire = program->poses->solvers[solver];
    RrPoseSolverState &s = scratch->solvers[solver];
    const auto liveRest = [&](size_t k) {
        return k < wire.restIsLive.size() && wire.restIsLive[k] &&
               k < wire.restReads.size() &&
               size_t(wire.restReads[k]) < store.fin.size();
    };
    for (size_t k = 0; k < wire.restRefs.size() && k < s.jointRests.size();
         ++k) {
        const int slot = wire.restRefs[k].first;
        if (liveRest(k)) {
            s.jointRests[k] = store.fin[size_t(wire.restReads[k])].points;
        } else if (slot >= 0 && size_t(slot) < scratch->restPts.size()) {
            s.jointRests[k] = scratch->restPts[size_t(slot)];
        } else {
            if (error) {
                *error = _RrStepHead(program, step) + " names no rest slot";
            }
            return false;
        }
    }
    // The space target's rest, read where the file's space_rest was read:
    // the composed rest table, never a live ref.
    const auto refreshSpaceRest = [&]() {
        if (wire.spaceSlot < 0) {
            return true;
        }
        if (size_t(wire.spaceSlot) >= scratch->restPts.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no rest slot";
            }
            return false;
        }
        s.spaceRest = scratch->restPts[size_t(wire.spaceSlot)];
        return true;
    };
    const std::string type = program->TextOrEmpty(wire.type);
    // A 2D skeleton's controls are its outside parent spaces.
    if (type == SchemaTypeName::RigExecFkChain ||
        type == SchemaTypeName::RigExecSkeleton2d) {
        for (size_t k = 0; k < wire.controls.size(); ++k) {
            if (k >= s.controlRests.size() || wire.controls[k] < 0 ||
                size_t(wire.controls[k]) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) + " names no rest slot";
                }
                return false;
            }
            s.controlRests[k] = scratch->restPts[size_t(wire.controls[k])];
        }
        if (wire.start >= 0) {
            if (size_t(wire.start) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) + " names no rest slot";
                }
                return false;
            }
            s.startRest = scratch->restPts[size_t(wire.start)];
        }
    } else if (type == SchemaTypeName::RigExecTwoBoneIk) {
        for (size_t k = 0; k < wire.restRefs.size(); ++k) {
            const int slot = wire.restRefs[k].first;
            const int element = wire.restRefs[k].second;
            if (element >= 0 && element < 3) {
                if (liveRest(k)) {
                    s.ikRests[size_t(element)] =
                        store.fin[size_t(wire.restReads[k])].points;
                } else if (slot >= 0 &&
                           size_t(slot) < scratch->restPts.size()) {
                    s.ikRests[size_t(element)] = scratch->restPts[size_t(slot)];
                } else {
                    if (error) {
                        *error =
                            _RrStepHead(program, step) + " names no rest slot";
                    }
                    return false;
                }
            }
        }
        s.upperLengthBase = (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
        s.lowerLengthBase = (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
        // The constant arm's lengths, measured in the folded spaceMatrix as
        // the baked rest refresh measures them.
        const std::array<int32_t, RrSolverFieldCount> &reads =
            program->solverRead[solver];
        const auto constant = [&](int field) {
            return program->RegisteredConstant(reads[size_t(field)]);
        };
        _RrTwoBoneIkLengths(s.ikRests, constant(RrSolverIkSpace).matrix,
                            constant(RrSolverUpperOffset).f64,
                            constant(RrSolverLowerOffset).f64,
                            &s.ikParams.upperLength, &s.ikParams.lowerLength);
        if (!refreshSpaceRest()) {
            return false;
        }
    } else if (type == SchemaTypeName::RigExecSplineIk) {
        std::vector<RrPointFrame> restJoints(size_t(wire.splineCount));
        for (size_t k = 0; k < wire.restRefs.size(); ++k) {
            const int slot = wire.restRefs[k].first;
            const int element = wire.restRefs[k].second;
            if (element >= 0 && size_t(element) < size_t(wire.splineCount)) {
                if (liveRest(k)) {
                    restJoints[size_t(element)] =
                        store.fin[size_t(wire.restReads[k])];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            store.fin[size_t(wire.restReads[k])].points;
                    }
                } else if (slot >= 0 &&
                           size_t(slot) < scratch->restPts.size() &&
                           size_t(slot) < scratch->restFrames.size()) {
                    restJoints[size_t(element)] =
                        scratch->restFrames[size_t(slot)];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            scratch->restPts[size_t(slot)];
                    }
                } else {
                    if (error) {
                        *error =
                            _RrStepHead(program, step) + " names no rest slot";
                    }
                    return false;
                }
            }
        }
        const RrPointFrame noFrame;
        s.splineRest = _RrSplineIkMakeRest(
            restJoints,
            wire.root >= 0 ? scratch->restFrames[size_t(wire.root)] : noFrame,
            wire.mid >= 0 ? scratch->restFrames[size_t(wire.mid)] : noFrame,
            wire.end >= 0 ? scratch->restFrames[size_t(wire.end)] : noFrame,
            wire.splineRestWeights, wire.splineRestMode);
        if (!refreshSpaceRest()) {
            return false;
        }
    } else if (type == SchemaTypeName::RigExecTwistDistribution) {
        if (wire.root >= 0 && wire.end >= 0 &&
            size_t(wire.root) < scratch->restPts.size() &&
            size_t(wire.end) < scratch->restPts.size()) {
            s.twistStartRest = scratch->restPts[size_t(wire.root)];
            s.twistEndRest = scratch->restPts[size_t(wire.end)];
        }
    }
    return true;
}

RrPointFrameArray
_RrSolveTwistDistribution(const RrPointFrame &start, const RrPointFrame &end,
                          const std::array<RrVec3d, 4> &startRest,
                          const std::array<RrVec3d, 4> &endRest,
                          const std::vector<double> &weights, double twistTurns,
                          const std::vector<std::array<RrVec3d, 4>> &jointRests,
                          const std::vector<char> &jointRestLive);

RrPointFrameArray
_RrSampleRibbonFrames(const std::vector<RrVec3f> &posed,
                      const std::vector<RrVec3f> &rest, int sampleCount,
                      const std::vector<std::array<RrVec3d, 4>> &jointRests,
                      const std::vector<char> &jointRestLive);

struct _RrSplineIkControls {
    RrPointFrame root;
    RrPointFrame mid;
    RrPointFrame end;
};

struct _RrSplineIkParams {
    double preserveVolume = 1.0;
    double midFollowWeight = 0.5;
    double roll = 0.0;
    double twist = 0.0;
    double minLengthRatio = 0.0;
    bool aimRootTangent = false;
};

struct _RrSplineIkJoint {
    RrPointFrame frame;
    RrVec3d scale{1.0, 1.0, 1.0};
    double arcDistance = 0.0;
    double arcParam = 0.0;
    double twist = 0.0;
};

struct _RrSplineIkResult {
    std::array<RrVec3d, 4> cvs{};
    double arcLength = 0.0;
    double ratio = 1.0;
    double roll = 0.0;
    double twist = 0.0;
    std::vector<_RrSplineIkJoint> joints;
};

bool _RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                      const _RrSplineIkControls &controls,
                      const _RrSplineIkParams &params,
                      _RrSplineIkResult *result);

// RigExecDistributeTwist (solvers.cpp).
std::vector<RrPointFrame>
_RrDistributeTwist(const RrPointFrame &start, const RrPointFrame &end,
                   const std::array<RrVec3d, 4> &startRest,
                   const std::array<RrVec3d, 4> &endRest,
                   const std::vector<double> &weights, double twistTurns)
{
    return RigExecDistributeTwistKernel<RrVec3f, RrVec3d>(
        start, end, startRest, endRest, weights, twistTurns);
}

// Ribbon substrate (geometryKernels.cpp, solverKernels.cpp).

struct _RrCurveFrameSamples {
    std::vector<RrVec3f> positions;
    std::vector<RrVec3f> tangents;
    std::vector<RrVec3f> normals;
    std::vector<RrVec3f> binormals;
    std::vector<float> parameters;

    size_t GetSize() const
    {
        return positions.size();
    }
};

RrVec3f
_RrBsplinePoint(const RrVec3f &p0, const RrVec3f &p1, const RrVec3f &p2,
                const RrVec3f &p3, float t)
{
    const float t2 = t * t, t3 = t2 * t;
    const float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
    const float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
    const float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
    const float b3 = t3 / 6.0f;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

// RigExecSampleCurveRMF: a cubic uniform B-spline at sampleCount
// arc-length parameters, rotation-minimizing frames by the
// double-reflection method.
_RrCurveFrameSamples
_RrSampleCurveRMF(const std::vector<RrVec3f> &controlPoints, int sampleCount)
{
    _RrCurveFrameSamples samples;
    if (sampleCount < 2 || controlPoints.size() < 2) {
        return samples;
    }

    const int dense = std::max(sampleCount * 16, 64);
    std::vector<RrVec3f> densePoints;
    densePoints.reserve(size_t(dense) + 1);
    const int spans = int(controlPoints.size()) - 3;
    for (int i = 0; i <= dense; ++i) {
        const float u = float(i) / float(dense);
        if (spans >= 1) {
            const float s = u * spans;
            const int span = std::min(spans - 1, int(s));
            const float t = s - span;
            densePoints.push_back(_RrBsplinePoint(
                controlPoints[size_t(span)], controlPoints[size_t(span) + 1],
                controlPoints[size_t(span) + 2],
                controlPoints[size_t(span) + 3], t));
        } else {
            const float s = u * (controlPoints.size() - 1);
            const int seg = std::min(int(controlPoints.size()) - 2, int(s));
            densePoints.push_back(
                controlPoints[size_t(seg)] +
                (controlPoints[size_t(seg) + 1] - controlPoints[size_t(seg)]) *
                    (s - seg));
        }
    }
    std::vector<float> arcLength(densePoints.size(), 0.0f);
    for (size_t i = 1; i < densePoints.size(); ++i) {
        arcLength[i] = arcLength[i - 1] +
                       (densePoints[i] - densePoints[i - 1]).GetLength();
    }
    const float total = arcLength.back();
    if (total <= 1e-12f) {
        return samples;
    }

    samples.positions.reserve(size_t(sampleCount));
    samples.parameters.reserve(size_t(sampleCount));
    size_t cursor = 0;
    for (int k = 0; k < sampleCount; ++k) {
        const float target = total * float(k) / float(sampleCount - 1);
        while (cursor + 1 < arcLength.size() &&
               arcLength[cursor + 1] < target) {
            ++cursor;
        }
        const float span = arcLength[cursor + 1] - arcLength[cursor];
        const float t =
            span > 1e-12f ? (target - arcLength[cursor]) / span : 0.0f;
        samples.positions.push_back(
            densePoints[cursor] +
            (densePoints[cursor + 1] - densePoints[cursor]) * t);
        samples.parameters.push_back(float(k) / float(sampleCount - 1));
    }

    samples.tangents.resize(size_t(sampleCount));
    for (int k = 0; k < sampleCount; ++k) {
        const RrVec3f &prev = samples.positions[size_t(std::max(0, k - 1))];
        const RrVec3f &next =
            samples.positions[size_t(std::min(sampleCount - 1, k + 1))];
        RrVec3f tangent = next - prev;
        const float len = tangent.GetLength();
        samples.tangents[size_t(k)] =
            len > 1e-12f ? tangent / len : RrVec3f(1, 0, 0);
    }
    samples.normals.resize(size_t(sampleCount));
    samples.binormals.resize(size_t(sampleCount));
    {
        const RrVec3f t0 = samples.tangents[0];
        RrVec3f candidate(1, 0, 0);
        float best = 2.0f;
        for (const RrVec3f axis :
             {RrVec3f(1, 0, 0), RrVec3f(0, 1, 0), RrVec3f(0, 0, 1)}) {
            const float align = std::abs(RrDot(axis, t0));
            if (align < best) {
                best = align;
                candidate = axis;
            }
        }
        RrVec3f n0 = candidate - t0 * RrDot(t0, candidate);
        n0.Normalize();
        samples.normals[0] = n0;
        samples.binormals[0] = RrCross(t0, n0);
    }
    for (int k = 0; k + 1 < sampleCount; ++k) {
        const RrVec3f v1 =
            samples.positions[size_t(k) + 1] - samples.positions[size_t(k)];
        const float c1 = RrDot(v1, v1);
        if (c1 <= 1e-20f) {
            samples.normals[size_t(k) + 1] = samples.normals[size_t(k)];
            samples.binormals[size_t(k) + 1] = samples.binormals[size_t(k)];
            continue;
        }
        const RrVec3f nL =
            samples.normals[size_t(k)] -
            v1 * (2.0f / c1) * RrDot(v1, samples.normals[size_t(k)]);
        const RrVec3f tL =
            samples.tangents[size_t(k)] -
            v1 * (2.0f / c1) * RrDot(v1, samples.tangents[size_t(k)]);
        const RrVec3f v2 = samples.tangents[size_t(k) + 1] - tL;
        const float c2 = RrDot(v2, v2);
        RrVec3f n = c2 > 1e-20f ? nL - v2 * (2.0f / c2) * RrDot(v2, nL) : nL;
        n -= samples.tangents[size_t(k) + 1] *
             RrDot(samples.tangents[size_t(k) + 1], n);
        const float len = n.GetLength();
        samples.normals[size_t(k) + 1] =
            len > 1e-12f ? n / len : samples.normals[size_t(k)];
        samples.binormals[size_t(k) + 1] = RrCross(
            samples.tangents[size_t(k) + 1], samples.normals[size_t(k) + 1]);
    }
    return samples;
}

RrVec3d
_RrWiden(const RrVec3f &v)
{
    return RrVec3d(double(v[0]), double(v[1]), double(v[2]));
}

// Re-bases one aggregate element onto a joint's rest reference
// (solverKernels.cpp).
bool
_RrRebaseElement(const std::array<RrVec3d, 4> &ownRest,
                 const std::array<RrVec3d, 4> &jointRest, RrPointFrame *frame)
{
    RrMat4d map = _RrIdentity();
    if (!RrPointsToMatrix(ownRest, frame->points, &map)) {
        return false;
    }
    const uint32_t flags = frame->flags;
    *frame = RrMatrixToPoints(jointRest, map);
    frame->flags = flags;
    return true;
}

// RigExecSampleRibbonFrames (solverKernels.cpp).
RrPointFrameArray
_RrSampleRibbonFrames(const std::vector<RrVec3f> &posed,
                      const std::vector<RrVec3f> &rest, int sampleCount,
                      const std::vector<std::array<RrVec3d, 4>> &jointRests,
                      const std::vector<char> &jointRestLive)
{
    RrPointFrameArray result;
    if (posed.empty() || rest.empty() || sampleCount < 2) {
        return result;
    }
    const _RrCurveFrameSamples posedSamples =
        _RrSampleCurveRMF(posed, sampleCount);
    const _RrCurveFrameSamples restSamples =
        _RrSampleCurveRMF(rest, sampleCount);
    if (posedSamples.GetSize() != size_t(sampleCount) ||
        restSamples.GetSize() != size_t(sampleCount)) {
        return result;
    }
    result.frames.reserve(size_t(sampleCount));
    result.rests.reserve(size_t(sampleCount));
    for (int k = 0; k < sampleCount; ++k) {
        RrPointFrame frame;
        frame.points = {_RrWiden(posedSamples.positions[size_t(k)]),
                        _RrWiden(posedSamples.positions[size_t(k)] +
                                 posedSamples.tangents[size_t(k)]),
                        _RrWiden(posedSamples.positions[size_t(k)] +
                                 posedSamples.normals[size_t(k)]),
                        _RrWiden(posedSamples.positions[size_t(k)] +
                                 posedSamples.binormals[size_t(k)])};
        frame.flags = RrPointFrameValid;
        std::array<RrVec3d, 4> restPoints = {
            _RrWiden(restSamples.positions[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.tangents[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.normals[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.binormals[size_t(k)])};
        if (size_t(k) < jointRests.size() && size_t(k) < jointRestLive.size() &&
            jointRestLive[size_t(k)] &&
            _RrRebaseElement(restPoints, jointRests[size_t(k)], &frame)) {
            restPoints = jointRests[size_t(k)];
        }
        result.frames.push_back(frame);
        result.rests.push_back(restPoints);
    }
    return result;
}

// RigExecSolveTwistDistribution (solverKernels.cpp): every frame paired
// with the START landmarks, re-based where a lower step wrote.
RrPointFrameArray
_RrSolveTwistDistribution(const RrPointFrame &start, const RrPointFrame &end,
                          const std::array<RrVec3d, 4> &startRest,
                          const std::array<RrVec3d, 4> &endRest,
                          const std::vector<double> &weights, double twistTurns,
                          const std::vector<std::array<RrVec3d, 4>> &jointRests,
                          const std::vector<char> &jointRestLive)
{
    RrPointFrameArray result;
    result.frames =
        _RrDistributeTwist(start, end, startRest, endRest, weights, twistTurns);
    result.rests.assign(result.frames.size(), startRest);
    for (size_t k = 0; k < result.frames.size(); ++k) {
        if (k < jointRests.size() && k < jointRestLive.size() &&
            jointRestLive[k] &&
            _RrRebaseElement(startRest, jointRests[k], &result.frames[k])) {
            result.rests[k] = jointRests[k];
        }
    }
    return result;
}

// RigExecSolveSplineIk (rigExecMath/splineIkKernel.h).
bool
_RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                 const _RrSplineIkControls &controls,
                 const _RrSplineIkParams &params, _RrSplineIkResult *result)
{
    return RigExecSolveSplineIkKernel<RrVec3f, RrVec3d, RrPointFrame>(
        rest, controls, params, result);
}

} // namespace

namespace runtimePoseDetail {

bool _RrRunSolveStep(RrProgram *program, size_t step, std::string *error);

bool _RrRunSolverCommitStep(RrProgram *program, size_t step,
                            std::string *error);

bool
_RrRunSolveStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->solvers.size() ||
        size_t(wire.object) >= scratch->solvers.size() ||
        size_t(wire.object) >= store.aggregates.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    const RigExecWireSolver &ws = program->poses->solvers[size_t(wire.object)];
    RrPoseSolverState &s = scratch->solvers[size_t(wire.object)];
    const std::vector<std::array<RrVec3d, 4>> &liveRests = s.jointRests;
    std::vector<char> liveFlags;
    if (ws.hasLiveRest) {
        liveFlags.assign(s.jointRests.size(), 0);
        for (size_t k = 0; k < ws.restIsLive.size() && k < liveFlags.size();
             ++k) {
            liveFlags[k] = ws.restIsLive[k];
        }
    }
    bool restMoved = false;
    for (const int slot : ws.restSlots) {
        restMoved = restMoved ||
                    (slot >= 0 && size_t(slot) < store.restChanged.size() &&
                     store.restChanged[size_t(slot)]);
    }
    if (restMoved || ws.hasLiveRest) {
        if (!_RrRefreshSolverRests(program, step, size_t(wire.object), error)) {
            return false;
        }
    }
    RrPointFrameArray &aggregate = store.aggregates[size_t(wire.object)];
    aggregate.frames.clear();
    aggregate.rests.clear();
    const std::string type = program->TextOrEmpty(ws.type);
    if (size_t(wire.object) >= program->solverJointGroupRead.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    {
        const std::vector<int32_t> &groupReads =
            program->solverJointGroupRead[size_t(wire.object)];
        s.jointGroupSelectors.resize(groupReads.size());
        for (size_t g = 0; g < groupReads.size(); ++g) {
            s.jointGroupSelectors[g] =
                program->ReadRegistered(groupReads[g]).f64;
        }
    }
    const auto finAt = [&](uint32_t version, const RrPointFrame **out) {
        if (size_t(version) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no fin version";
            }
            return false;
        }
        *out = &store.fin[size_t(version)];
        return true;
    };
    if (ws.degenerate) {
    } else if (type == SchemaTypeName::RigExecFkChain) {
        const size_t base = ws.start >= 0 ? 1 : 0;
        if (s.elements.size() != ws.controls.size() + base ||
            ws.controlReads.size() != ws.controls.size()) {
            if (error) {
                *error =
                    _RrStepHead(program, step) + " names no solver control";
            }
            return false;
        }
        const bool jointBasis = s.jointRests.size() == ws.controls.size();
        aggregate.rests = s.controlRests;
        if (base) {
            s.elements[0].restPoints = s.startRest;
            const RrPointFrame *startPose = nullptr;
            if (!finAt(ws.startRead, &startPose)) {
                return false;
            }
            s.elements[0].posePoints = startPose->points;
            s.elements[0].hasOutRest = false;
            s.elements[0].parentIndex = -1;
        }
        for (size_t k = 0; k < ws.controls.size(); ++k) {
            if (k >= s.controlRests.size()) {
                if (error) {
                    *error =
                        _RrStepHead(program, step) + " names no solver control";
                }
                return false;
            }
            const size_t e = k + base;
            s.elements[e].restPoints = s.controlRests[k];
            const RrPointFrame *pose = nullptr;
            if (!finAt(ws.controlReads[k], &pose)) {
                return false;
            }
            s.elements[e].posePoints = pose->points;
            const bool live =
                jointBasis && k < ws.restIsLive.size() && ws.restIsLive[k];
            s.elements[e].hasOutRest = live;
            if (live) {
                if (k >= s.jointRests.size() || k >= aggregate.rests.size()) {
                    if (error) {
                        *error =
                            _RrStepHead(program, step) + " names no live rest";
                    }
                    return false;
                }
                s.elements[e].outRestPoints = s.jointRests[k];
                aggregate.rests[k] = s.jointRests[k];
            }
            s.elements[e].parentIndex =
                ws.parentRelative ? int(base) - 1 : int(k) + int(base) - 1;
        }
        aggregate.frames = _RrSolveFkChain(s.elements);
        if (ws.scaleSegments)
            RigExecScaleFkSegmentsKernel<RrVec3d>(s.elements,
                                                  &aggregate.frames);
        if (base && !aggregate.frames.empty()) {
            aggregate.frames.erase(aggregate.frames.begin());
        }
        if (!s.jointGroups.empty()) {
            // RigExecRunSolver's FkChain groups: a joint under an inactive
            // member publishes its output rest through the start delta.
            if (s.jointGroupSelectors.size() != s.jointGroups.size()) {
                if (error) {
                    *error =
                        _RrStepHead(program, step) + " names no joint group";
                }
                return false;
            }
            const size_t joints = s.jointGroupParents.size();
            s.jointGroupActive.resize(joints);
            s.jointGroupSlots.resize(s.jointGroups.size());
            RigExecResolveJointGroups(
                joints, [&s](size_t i) { return s.jointGroupParents[i]; },
                s.jointGroups.data(), s.jointGroups.size(),
                s.jointGroupMembers.data(), s.jointGroupSelectors.data(),
                s.jointGroupSlots.data(), s.jointGroupActive.data());
            RrMat4d start = _RrIdentity();
            if (base) {
                RrPointsToMatrix(s.elements[0].restPoints,
                                 s.elements[0].posePoints, &start);
            }
            for (size_t k = 0; k < joints && k < aggregate.frames.size(); ++k) {
                if (s.jointGroupActive[k]) {
                    continue;
                }
                const RrPoseFkElement &element = s.elements[k + base];
                aggregate.frames[k] =
                    RrMatrixToPoints(element.hasOutRest ? element.outRestPoints
                                                        : element.restPoints,
                                     start);
            }
        }
    } else if (type == SchemaTypeName::RigExecTwoBoneIk) {
        RrPoseTwoBoneIkParams params = s.ikParams;
        const size_t solver = size_t(wire.object);
        // rigExec:spaceMatrix, composed after rigExec:space's rest -> pose
        // map when a space is named (spaceSlot, not spaceRead: an unnamed
        // space reads no frame).
        RrMat4d ikSpace = program->ReadSolver(solver, RrSolverIkSpace).matrix;
        bool spaceMoved = false;
        if (ws.spaceSlot >= 0) {
            const RrPointFrame *spaceFrame = nullptr;
            if (!finAt(ws.spaceRead, &spaceFrame)) {
                return false;
            }
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(s.spaceRest, *spaceFrame, &delta)) {
                ikSpace = delta * ikSpace;
                spaceMoved = true;
            }
        }
        {
            params.preferredBendRadians =
                program->ReadSolver(solver, RrSolverBend).f64;
            params.stretch =
                double(program->ReadSolver(solver, RrSolverStretch).f32);
            params.softness =
                double(program->ReadSolver(solver, RrSolverSoftness).f32);
            params.space = ikSpace;
            _RrTwoBoneIkLengths(
                s.ikRests, ikSpace,
                program->ReadSolver(solver, RrSolverUpperOffset).f64,
                program->ReadSolver(solver, RrSolverLowerOffset).f64,
                &params.upperLength, &params.lowerLength);
        }
        params.softDistancePolicy = ws.softDistancePolicy;
        params.scaleSegments = ws.ikScaleSegments;
        params.limb.stretch = params.stretch;
        params.limb.pin =
            ws.pin ? program->ReadSolver(solver, RrSolverPin).f32 : 0;
        params.limb.upperScale =
            ws.upperScale ? program->ReadSolver(solver, RrSolverUpperScale).f64
                          : 1;
        params.limb.lowerScale =
            ws.lowerScale ? program->ReadSolver(solver, RrSolverLowerScale).f64
                          : 1;
        params.limb.scaleCalibration = ws.scaleCalibration;
        const double spaceFactor =
            RigExecTwoBoneIkSpaceFactorKernel(s.ikRests, ikSpace);
        params.limb.softDistance =
            ws.softDistance
                ? program->ReadSolver(solver, RrSolverSoftDistance).f32 *
                      spaceFactor
                : 0;
        params.twistRadians =
            ws.limbTwist ? program->ReadSolver(solver, RrSolverLimbTwist).f32 *
                               _RrPi / 180.0
                         : 0;
        const RrPointFrame *root = nullptr;
        const RrPointFrame *end = nullptr;
        const RrPointFrame *pole = nullptr;
        if (!finAt(ws.rootRead, &root) || !finAt(ws.endRead, &end) ||
            !finAt(ws.poleRead, &pole)) {
            return false;
        }
        const std::array<RrPointFrame, 3> frames =
            _RrSolveTwoBoneIk(*root, *end, *pole, s.ikRests, params);
        aggregate.frames.assign(frames.begin(), frames.end());
        aggregate.rests.assign(s.ikRests.begin(), s.ikRests.end());
    } else if (type == SchemaTypeName::RigExecBlendPointFrames) {
        if ((ws.inA >= 0 && size_t(ws.inA) >= store.aggregates.size()) ||
            (ws.inB >= 0 && size_t(ws.inB) >= store.aggregates.size())) {
            if (error) {
                *error =
                    _RrStepHead(program, step) + " names no solver aggregate";
            }
            return false;
        }
        const RrPointFrameArray *a =
            ws.inA >= 0 ? &store.aggregates[size_t(ws.inA)] : nullptr;
        const RrPointFrameArray *b =
            ws.inB >= 0 ? &store.aggregates[size_t(ws.inB)] : nullptr;
        if (!a) {
            if (b) {
                aggregate = *b;
            }
        } else if (!b) {
            aggregate = *a;
        } else if (ws.blendRotationRejected) {
        } else {
            const size_t n = a->frames.size();
            const double w =
                std::min(std::max(double(program
                                             ->ReadSolver(size_t(wire.object),
                                                          RrSolverBlendWeight)
                                             .f32),
                                  0.0),
                         1.0);
            if (n == b->frames.size() && a->rests.size() == n) {
                aggregate.frames.reserve(n);
                aggregate.rests.reserve(n);
                for (size_t k = 0; k < n; ++k) {
                    const bool live = k < liveFlags.size() && liveFlags[k] &&
                                      k < liveRests.size();
                    aggregate.frames.push_back(_RrBlendFrames(
                        a->frames[k], b->frames[k], a->rests[k], w,
                        ws.scaleMode == 0, live ? &liveRests[k] : nullptr));
                    aggregate.rests.push_back(live ? liveRests[k]
                                                   : a->rests[k]);
                }
            }
        }
    } else if (type == SchemaTypeName::RigExecTwistDistribution) {
        const RrPointFrame *start = nullptr;
        const RrPointFrame *end = nullptr;
        if (!finAt(ws.rootRead, &start) || !finAt(ws.endRead, &end)) {
            return false;
        }
        aggregate = _RrSolveTwistDistribution(
            *start, *end, s.twistStartRest, s.twistEndRest, s.twistWeights,
            program->ReadSolver(size_t(wire.object), RrSolverTwistTurns).f64,
            liveRests, liveFlags);
    } else if (type == SchemaTypeName::RigExecRibbon) {
        if (size_t(wire.object) >= store.ribbonConstant.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no solver";
            }
            return false;
        }
        aggregate = _RrSampleRibbonFrames(
            store.ribbonConstant[size_t(wire.object)], s.ribbonRestPoints,
            program->ReadSolver(size_t(wire.object), RrSolverRibbonSampleCount)
                .i32,
            liveRests, liveFlags);
    } else if (type == SchemaTypeName::RigExecSkeleton2d) {
        // The native body (RigExecRunSolver's Skeleton2d arm): rests and
        // outputs in rigExec:joints order, the kernel in solve order in the
        // start frame's plane, setup frames for every bone when the solve
        // fails. A rest is the bone's setup frame or the frame a pose step
        // below the skeleton wrote; the outside parent spaces are the
        // row's controls.
        const size_t n = s.bone2dOrder.size();
        const size_t object = size_t(wire.object);
        if (object >= program->solverBone2dRead.size() ||
            object >= program->solverBone2dSelectorRead.size() ||
            s.jointRests.size() != n || s.bone2dSetup.size() != n ||
            ws.controlReads.size() != ws.controls.size() ||
            s.controlRests.size() != ws.controls.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D skeleton";
            }
            return false;
        }
        const std::vector<int32_t> &reads = program->solverBone2dRead[object];
        if (reads.size() != n * kRigExecBone2dChannelCount) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D skeleton";
            }
            return false;
        }
        const std::vector<int32_t> &selectorReads =
            program->solverBone2dSelectorRead[object];
        s.bone2dSelectors.resize(selectorReads.size());
        for (size_t g = 0; g < selectorReads.size(); ++g) {
            s.bone2dSelectors[g] =
                program->ReadRegistered(selectorReads[g]).f64;
        }
        RigExecSkeleton2dLandmarkInputs<std::array<RrVec3d, 4>> frames;
        if (ws.start >= 0) {
            const RrPointFrame *startPose = nullptr;
            if (!finAt(ws.startRead, &startPose)) {
                return false;
            }
            s.bone2dStartPose = startPose->points;
            frames.startRest = &s.startRest;
            frames.startPose = &s.bone2dStartPose;
        }
        s.bone2dExternalPoses.resize(ws.controls.size());
        for (size_t e = 0; e < ws.controls.size(); ++e) {
            const RrPointFrame *posed = nullptr;
            if (!finAt(ws.controlReads[e], &posed)) {
                return false;
            }
            s.bone2dExternalPoses[e] = posed->points;
        }
        frames.externalRests = s.controlRests.data();
        frames.externalPoses = s.bone2dExternalPoses.data();
        frames.externalCount = s.bone2dExternalPoses.size();
        frames.selectors = s.bone2dSelectors.data();
        frames.selectorCount = s.bone2dSelectors.size();
        frames.groupSelectors = s.jointGroupSelectors.data();
        frames.groupSelectorCount = s.jointGroupSelectors.size();
        const RigExecSkeleton2dStatic extra = RigExecSkeleton2dStaticOf(
            s.bone2dSpaceSources, s.bone2dTargetCandidates, s.jointGroups,
            s.jointGroupMembers);
        const RigExecSkeleton2dResult result = RigExecSolveSkeleton2dJoints(
            s.bone2dOrder, s.jointRests, s.bone2dSetup, extra, frames,
            [&](size_t j, RigExecBone2dChannels *channels) {
                for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                    const RrInputValue value = program->ReadRegistered(
                        reads[j * kRigExecBone2dChannelCount + c]);
                    RigExecBone2dSetChannel(channels, c,
                                            value.tag == RrInputTag::Bool
                                                ? (value.boolean ? 1.0 : 0.0)
                                                : value.f64);
                }
            },
            RrPointFrameValid, &s.bone2d, &aggregate.frames);
        if (result == RigExecSkeleton2dResult::Malformed) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D bone";
            }
            return false;
        }
        aggregate.rests = s.jointRests;
    } else if (type == SchemaTypeName::RigExecSplineIk) {
        _RrSplineIkParams params;
        params.preserveVolume = ws.splineParams.preserveVolume;
        params.midFollowWeight = ws.splineParams.midFollowWeight;
        params.roll = ws.splineParams.roll;
        params.twist = ws.splineParams.twist;
        params.minLengthRatio = ws.splineParams.minLengthRatio;
        params.aimRootTangent = ws.splineParams.aimRootTangent;
        if (ws.splineParamsVary ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverPreserveVolume) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMidFollowWeight) ||
            _RrLiveSolver(program, size_t(wire.object), RrSolverRoll) ||
            _RrLiveSolver(program, size_t(wire.object), RrSolverTwist) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMinLengthRatio)) {
            params.preserveVolume =
                program->ReadSolver(size_t(wire.object), RrSolverPreserveVolume)
                    .f64;
            params.midFollowWeight =
                program
                    ->ReadSolver(size_t(wire.object), RrSolverMidFollowWeight)
                    .f64;
            params.roll = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object), RrSolverRoll).f64);
            params.twist = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object), RrSolverTwist).f64);
            params.minLengthRatio =
                program->ReadSolver(size_t(wire.object), RrSolverMinLengthRatio)
                    .f64;
        }
        // The rest rebuilt in rigExec:space once the space has moved
        // (bakedPose.cpp): the bind-time rest is measured at identity and
        // the placement ratio is arcLength / restArcLength, so a scaled
        // space would otherwise read as stretch. s.splineRest holds the
        // rest frames and the root, mid and end rests it was made from.
        const RrPoseSplineIkRest *restForSolve = &s.splineRest;
        RrPoseSplineIkRest spacedRest;
        if (ws.spaceSlot >= 0) {
            const RrPointFrame *spaceFrame = nullptr;
            if (!finAt(ws.spaceRead, &spaceFrame)) {
                return false;
            }
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(s.spaceRest, *spaceFrame, &delta) &&
                delta != RrMat4d(1.0)) {
                std::vector<RrPointFrame> spaced;
                spaced.reserve(s.splineRest.joints.size());
                for (const RrPointFrame &f : s.splineRest.joints) {
                    spaced.push_back(_RrTransformFrame(f, delta));
                }
                spacedRest = _RrSplineIkMakeRest(
                    spaced, _RrTransformFrame(s.splineRest.rootControl, delta),
                    _RrTransformFrame(s.splineRest.midControl, delta),
                    _RrTransformFrame(s.splineRest.endControl, delta),
                    ws.splineRestWeights, ws.splineRestMode);
                restForSolve = &spacedRest;
            }
        }
        const RrPointFrame *root = nullptr;
        const RrPointFrame *mid = nullptr;
        const RrPointFrame *end = nullptr;
        if (!finAt(ws.rootRead, &root) || !finAt(ws.midRead, &mid) ||
            !finAt(ws.endRead, &end)) {
            return false;
        }
        _RrSplineIkControls controls;
        controls.root = *root;
        controls.mid = *mid;
        controls.end = *end;
        _RrSplineIkResult solved;
        _RrSolveSplineIk(*restForSolve, controls, params, &solved);
        if (solved.joints.size() == size_t(ws.splineCount)) {
            aggregate.frames.reserve(size_t(ws.splineCount));
            for (const auto &joint : solved.joints) {
                aggregate.frames.push_back(joint.frame);
            }
            aggregate.rests = s.splineJointRests;
        }
    }
    if (size_t(wire.object) >= store.solverOutFrames.size() ||
        size_t(wire.object) >= store.solverOutPresent.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    if (store.solverOutFrames[size_t(wire.object)].size() !=
            ws.outputs.size() ||
        store.solverOutPresent[size_t(wire.object)].size() !=
            ws.outputs.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    for (size_t k = 0; k < ws.outputs.size(); ++k) {
        const int32_t slot = ws.outputs[k].first;
        const int32_t element = ws.outputs[k].second;
        if (element < 0 || size_t(element) >= aggregate.frames.size()) {
            store.solverOutPresent[size_t(wire.object)][k] = 0;
            (void)slot;
            continue;
        }
        store.solverOutFrames[size_t(wire.object)][k] =
            RrExtractElementFrame(&aggregate, size_t(element));
        store.solverOutPresent[size_t(wire.object)][k] = 1;
    }
    return true;
}

bool
_RrRunSolverCommitStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    std::fill(commit.present.begin(), commit.present.end(), 0);
    for (const int si :
         program->poses->walkSteps[size_t(wire.object)].batchSolvers) {
        if (si < 0 || size_t(si) >= program->poses->solvers.size() ||
            size_t(si) >= store.solverOutFrames.size() ||
            size_t(si) >= store.solverOutPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no solver";
            }
            return false;
        }
        const RigExecWireSolver &s = program->poses->solvers[size_t(si)];
        if (store.solverOutFrames[size_t(si)].size() != s.outputs.size() ||
            store.solverOutPresent[size_t(si)].size() != s.outputs.size() ||
            s.outPosition.size() != s.outputs.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no solver";
            }
            return false;
        }
        for (size_t k = 0; k < s.outputs.size(); ++k) {
            if (!store.solverOutPresent[size_t(si)][k] ||
                s.outPosition[k] < 0) {
                continue;
            }
            if (size_t(s.outPosition[k]) >= commit.frames.size() ||
                size_t(s.outPosition[k]) >= commit.present.size()) {
                if (error) {
                    *error =
                        _RrStepHead(program, step) + " names no candidate slot";
                }
                return false;
            }
            commit.frames[size_t(s.outPosition[k])] =
                store.solverOutFrames[size_t(si)][k];
            commit.present[size_t(s.outPosition[k])] = 1;
        }
    }
    commit.abandoned = std::find(commit.present.begin(), commit.present.end(),
                                 1) == commit.present.end();
    if (wireCommit.split) {
        return true;
    }
    if (!commit.abandoned) {
        if (!_RrComputeCommitDeltas(program, step, wireCommit, &commit,
                                    error)) {
            return false;
        }
        if (!_RrStageCommitPairs(program, step, wireCommit, &commit, 0,
                                 wireCommit.propagate.size(), error)) {
            return false;
        }
    }
    return _RrFinishCommit(program, step, size_t(wire.object), error);
}

} // namespace runtimePoseDetail

} // namespace rigExec
