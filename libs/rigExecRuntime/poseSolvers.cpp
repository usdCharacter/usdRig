// Runtime hosts of the solver op nodes: their lookups and stored frames.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "poseInternal.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/chainSolverKernel.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecMath/skeleton2dKernel.h"
#include "rigExecMath/splineIkKernel.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

bool
_RrSolverFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Whether \p type holds one \p T, or with \p array an array of them.
template <class T>
bool
_RrSolverType(const RigExecTypeDesc &type, bool array)
{
    if (!array)
        return !type.element && type.size == sizeof(T);
    return type.element && type.resize && type.data &&
           RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size ==
               sizeof(T);
}

/// \p v into \p value, a scalar of \p type.
template <class T>
bool
_RrSolverScalar(const RigExecTypeDesc &type, T v, void *value,
                std::string *error)
{
    if (type.element || type.size != sizeof(T))
        return _RrSolverFail(error, "a solver scalar of another type");
    std::memcpy(value, &v, sizeof(T));
    return true;
}

rigExecRig::PointLandmarks
_RrSolverLandmarks(const std::array<RrVec3d, 4> &points)
{
    rigExecRig::PointLandmarks out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = rigExecStd::Vec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

rigExecRig::PointFrame
_RrSolverFrame(const RrPointFrame &frame)
{
    rigExecRig::PointFrame out;
    out.points = _RrSolverLandmarks(frame.points);
    out.flags = frame.flags;
    return out;
}

/// \p s's rigExec:space map from its rest frame, when the space names a
/// slot and moved it; false otherwise.
bool
_RrSplineSpace(const RrProgram &p, const RigExecWireSolver &s, RrMat4d *delta)
{
    const auto &restPts =
        static_cast<const RrPoseScratch *>(p.pose.get())->restPts;
    *delta = RrMat4d(1.0);
    return s.spaceSlot >= 0 && size_t(s.spaceSlot) < restPts.size() &&
           size_t(s.spaceRead) < p.store.fin.size() &&
           RrPointsToMatrix(restPts[size_t(s.spaceSlot)],
                            p.store.fin[size_t(s.spaceRead)], delta) &&
           *delta != RrMat4d(1.0);
}

std::array<RrVec3d, 4>
_RrFromSolverLandmarks(const rigExecRig::PointLandmarks &points)
{
    std::array<RrVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = RrVec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

} // namespace

int64_t
RrSolveOf(const RrProgram *program, const RigExecWireStep &step)
{
    return step.kind == RigExecWireStepKind::OpNode && step.object >= 0 &&
                   size_t(step.object) < program->solverOfNode.size()
               ? program->solverOfNode[size_t(step.object)]
               : -1;
}

bool
RrLookUpSolver(const RrProgram *program, const eval::OpLookup &lookup,
               const RigExecTypeDesc &type, void *value, std::string *error)
{
    using Read = eval::OpSolverRead;
    const RrProgram &p = *program;
    if (!p.poses || lookup.object >= p.poses->solvers.size())
        return _RrSolverFail(error, "solver " + std::to_string(lookup.object) +
                                        " is no solver of the program");
    const RigExecWireSolver &s = p.poses->solvers[lookup.object];
    const RrPoseScratch &scratch =
        *static_cast<const RrPoseScratch *>(p.pose.get());
    const std::vector<std::array<RrVec3d, 4>> &restPts = scratch.restPts;
    const std::vector<RrPointFrame> &fin = p.store.fin;
    const size_t base = s.start >= 0 ? 1 : 0, n = s.controls.size();
    const auto rest = [&](int slot, rigExecRig::PointLandmarks *out) {
        if (slot < 0 || size_t(slot) >= restPts.size())
            return _RrSolverFail(
                error, "solver " + std::to_string(lookup.object) +
                           " names no rest slot " + std::to_string(slot));
        *out = _RrSolverLandmarks(restPts[size_t(slot)]);
        return true;
    };
    const auto frame = [&](uint32_t version, rigExecRig::PointFrame *out) {
        if (version >= fin.size())
            return _RrSolverFail(error, "pose version " +
                                            std::to_string(version) +
                                            " is no pose frame of the "
                                            "program");
        *out = _RrSolverFrame(fin[version]);
        return true;
    };
    switch (Read(lookup.part)) {
    case Read::FkControls: {
        if (!_RrSolverType<rigExecRig::PointFrameArray>(type, false) ||
            s.controlReads.size() != n)
            break;
        auto &out = *static_cast<rigExecRig::PointFrameArray *>(value);
        out.frames.Resize(n + base);
        out.rests.Resize(n + base);
        if (base && (!frame(s.startRead, &out.frames[0]) ||
                     !rest(s.start, &out.rests[0])))
            return false;
        for (size_t k = 0; k < n; ++k)
            if (!frame(s.controlReads[k], &out.frames[k + base]) ||
                !rest(s.controls[k], &out.rests[k + base]))
                return false;
        return true;
    }
    case Read::FkOutRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true))
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(n + base);
        if (base && !rest(s.start, &out[0]))
            return false;
        const bool jointBasis = s.jointRests.size() == n;
        for (size_t k = 0; k < n; ++k) {
            if (!jointBasis || k >= s.restIsLive.size() || !s.restIsLive[k]) {
                if (!rest(s.controls[k], &out[k + base]))
                    return false;
                continue;
            }
            if (k >= s.restReads.size() || s.restReads[k] >= fin.size())
                return _RrSolverFail(
                    error, "solver " + std::to_string(lookup.object) +
                               " names no live rest " + std::to_string(k));
            out[k + base] = _RrSolverLandmarks(fin[s.restReads[k]].points);
        }
        return true;
    }
    case Read::StartRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.start, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::IkRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(3);
        for (size_t e = 0; e < 3; ++e)
            out[e] =
                _RrSolverLandmarks(scratch.solvers[lookup.object].ikRests[e]);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element < 0 || element >= 3)
                continue;
            if (k < s.restIsLive.size() && s.restIsLive[k]) {
                if (k >= s.restReads.size() || s.restReads[k] >= fin.size())
                    return _RrSolverFail(
                        error, "solver " + std::to_string(lookup.object) +
                                   " names no live rest " + std::to_string(k));
                out[size_t(element)] =
                    _RrSolverLandmarks(fin[s.restReads[k]].points);
            } else if (!rest(slot, &out[size_t(element)])) {
                return false;
            }
        }
        return true;
    }
    case Read::IkSpace: {
        if (!_RrSolverType<RrMat4d>(type, false))
            break;
        RrMat4d space = p.ReadSolver(lookup.object, RrSolverIkSpace).matrix;
        if (s.spaceSlot >= 0) {
            if (size_t(s.spaceSlot) >= restPts.size() ||
                size_t(s.spaceRead) >= fin.size())
                return _RrSolverFail(error, "solver " +
                                                std::to_string(lookup.object) +
                                                " names no space frame");
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(restPts[size_t(s.spaceSlot)],
                                 fin[size_t(s.spaceRead)], &delta))
                space = delta * space;
        }
        std::memcpy(value, &space, sizeof(RrMat4d));
        return true;
    }
    case Read::Bend:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverBend).f64, value, error);
    case Read::UpperOffset:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverUpperOffset).f64, value,
            error);
    case Read::LowerOffset:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverLowerOffset).f64, value,
            error);
    case Read::Stretch:
        return _RrSolverScalar(type,
                               p.ReadSolver(lookup.object, RrSolverStretch).f32,
                               value, error);
    case Read::Softness:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverSoftness).f32, value,
            error);
    case Read::Pin:
        return _RrSolverScalar(
            type, s.pin ? p.ReadSolver(lookup.object, RrSolverPin).f32 : 0.0f,
            value, error);
    case Read::UpperScale:
        return _RrSolverScalar(
            type,
            s.upperScale ? p.ReadSolver(lookup.object, RrSolverUpperScale).f64
                         : 1.0,
            value, error);
    case Read::LowerScale:
        return _RrSolverScalar(
            type,
            s.lowerScale ? p.ReadSolver(lookup.object, RrSolverLowerScale).f64
                         : 1.0,
            value, error);
    case Read::SoftDistance:
        return _RrSolverScalar(
            type,
            s.softDistance
                ? p.ReadSolver(lookup.object, RrSolverSoftDistance).f32
                : 0.0f,
            value, error);
    case Read::Twist:
        return _RrSolverScalar(
            type,
            s.limbTwist ? p.ReadSolver(lookup.object, RrSolverLimbTwist).f32
                        : 0.0f,
            value, error);
    case Read::JointRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        const RrPoseSolverState &state = scratch.solvers[lookup.object];
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(state.jointRests.size());
        for (size_t k = 0; k < state.jointRests.size(); ++k) {
            if (k < s.restIsLive.size() && s.restIsLive[k] &&
                k < s.restReads.size() && s.restReads[k] < fin.size())
                out[k] = _RrSolverLandmarks(fin[s.restReads[k]].points);
            else if (k >= s.restRefs.size())
                out[k] = _RrSolverLandmarks(state.jointRests[k]);
            else if (!rest(s.restRefs[k].first, &out[k]))
                return false;
        }
        return true;
    }
    case Read::JointRestLive: {
        if (!_RrSolverType<uint8_t>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        const size_t joints = scratch.solvers[lookup.object].jointRests.size();
        auto &out = *static_cast<rigExecStd::RigExecArray<uint8_t> *>(value);
        out.Resize(joints);
        for (size_t k = 0; k < joints; ++k)
            out[k] = k < s.restIsLive.size() && s.restIsLive[k] ? 1 : 0;
        return true;
    }
    case Read::TwistStartRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.root, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistEndRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.end, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistTurns:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverTwistTurns).f64, value,
            error);
    case Read::DriverPoints: {
        if (!_RrSolverType<rigExecStd::Vec3f>(type, true) ||
            lookup.object >= p.store.ribbonConstant.size())
            break;
        const std::vector<RrVec3f> &points =
            p.store.ribbonConstant[lookup.object];
        auto &out =
            *static_cast<rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(value);
        out.Resize(points.size());
        for (size_t i = 0; i < points.size(); ++i)
            out[i] =
                rigExecStd::Vec3f(points[i][0], points[i][1], points[i][2]);
        return true;
    }
    case Read::SplineRestJoints: {
        if (!_RrSolverType<rigExecRig::PointFrame>(type, true))
            break;
        RrMat4d delta(1.0);
        const bool spaced = _RrSplineSpace(p, s, &delta);
        const size_t count = size_t(s.splineCount);
        auto &out =
            *static_cast<rigExecStd::RigExecArray<rigExecRig::PointFrame> *>(
                value);
        out.Resize(count);
        for (size_t e = 0; e < count; ++e)
            out[e] =
                _RrSolverFrame(spaced ? _RrTransformFrame(RrPointFrame(), delta)
                                      : RrPointFrame());
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element < 0 || size_t(element) >= count)
                continue;
            const bool live = k < s.restIsLive.size() && s.restIsLive[k] &&
                              k < s.restReads.size() &&
                              s.restReads[k] < fin.size();
            if (!live &&
                (slot < 0 || size_t(slot) >= scratch.restFrames.size()))
                return _RrSolverFail(
                    error, "solver " + std::to_string(lookup.object) +
                               " names no rest slot " + std::to_string(slot));
            const RrPointFrame &frame =
                live ? fin[s.restReads[k]] : scratch.restFrames[size_t(slot)];
            out[size_t(element)] = _RrSolverFrame(
                spaced ? _RrTransformFrame(frame, delta) : frame);
        }
        return true;
    }
    case Read::SplineRestRoot:
    case Read::SplineRestMid:
    case Read::SplineRestEnd: {
        if (!_RrSolverType<rigExecRig::PointFrame>(type, false))
            break;
        const int slot = Read(lookup.part) == Read::SplineRestRoot  ? s.root
                         : Read(lookup.part) == Read::SplineRestMid ? s.mid
                                                                    : s.end;
        if (slot >= 0 && size_t(slot) >= scratch.restFrames.size())
            return _RrSolverFail(
                error, "solver " + std::to_string(lookup.object) +
                           " names no rest slot " + std::to_string(slot));
        const RrPointFrame frame =
            slot >= 0 ? scratch.restFrames[size_t(slot)] : RrPointFrame();
        RrMat4d delta(1.0);
        *static_cast<rigExecRig::PointFrame *>(value) = _RrSolverFrame(
            _RrSplineSpace(p, s, &delta) ? _RrTransformFrame(frame, delta)
                                         : frame);
        return true;
    }
    case Read::SplineJointRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        const RrPoseSolverState &state = scratch.solvers[lookup.object];
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(state.splineJointRests.size());
        for (size_t e = 0; e < state.splineJointRests.size(); ++e)
            out[e] = _RrSolverLandmarks(state.splineJointRests[e]);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element < 0 || size_t(element) >= out.Size())
                continue;
            if (k < s.restIsLive.size() && s.restIsLive[k] &&
                k < s.restReads.size() && s.restReads[k] < fin.size())
                out[size_t(element)] =
                    _RrSolverLandmarks(fin[s.restReads[k]].points);
            else if (!rest(slot, &out[size_t(element)]))
                return false;
        }
        return true;
    }
    case Read::PreserveVolume:
    case Read::MidFollow:
    case Read::Roll:
    case Read::SplineTwist:
    case Read::MinLengthRatio: {
        const size_t object = lookup.object;
        const bool live = s.splineParamsVary ||
                          _RrLiveSolver(&p, object, RrSolverPreserveVolume) ||
                          _RrLiveSolver(&p, object, RrSolverMidFollowWeight) ||
                          _RrLiveSolver(&p, object, RrSolverRoll) ||
                          _RrLiveSolver(&p, object, RrSolverTwist) ||
                          _RrLiveSolver(&p, object, RrSolverMinLengthRatio);
        const auto &folded = s.splineParams;
        const auto leaf = [&](int field) {
            return p.ReadSolver(object, field).f64;
        };
        double v = 0.0;
        switch (Read(lookup.part)) {
        case Read::PreserveVolume:
            v = live ? leaf(RrSolverPreserveVolume) : folded.preserveVolume;
            break;
        case Read::MidFollow:
            v = live ? leaf(RrSolverMidFollowWeight) : folded.midFollowWeight;
            break;
        case Read::Roll:
            v = live ? RrDegreesToRadians(leaf(RrSolverRoll)) : folded.roll;
            break;
        case Read::SplineTwist:
            v = live ? RrDegreesToRadians(leaf(RrSolverTwist)) : folded.twist;
            break;
        default:
            v = live ? leaf(RrSolverMinLengthRatio) : folded.minLengthRatio;
            break;
        }
        return _RrSolverScalar(type, v, value, error);
    }
    case Read::StartPose:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false) ||
            s.startRead >= fin.size())
            break;
        *static_cast<rigExecRig::PointLandmarks *>(value) =
            _RrSolverLandmarks(fin[s.startRead].points);
        return true;
    case Read::ControlRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true))
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(s.controls.size());
        for (size_t k = 0; k < s.controls.size(); ++k)
            if (!rest(s.controls[k], &out[k]))
                return false;
        return true;
    }
    case Read::ControlPoses: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            s.controlReads.size() != s.controls.size())
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(s.controls.size());
        for (size_t k = 0; k < s.controls.size(); ++k) {
            if (s.controlReads[k] >= fin.size())
                return _RrSolverFail(
                    error, "pose version " + std::to_string(s.controlReads[k]) +
                               " is no pose frame of the program");
            out[k] = _RrSolverLandmarks(fin[s.controlReads[k]].points);
        }
        return true;
    }
    case Read::Bone2dChannels: {
        if (!_RrSolverType<double>(type, true) ||
            lookup.object >= p.solverBone2dRead.size())
            break;
        const std::vector<int32_t> &reads = p.solverBone2dRead[lookup.object];
        const size_t bones = reads.size() / kRigExecBone2dChannelCount;
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(bones * kRigExecBone2dChannelCount);
        for (size_t k = 0; k < bones; ++k) {
            RigExecBone2dChannels ch;
            for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                const RrInputValue v =
                    p.ReadRegistered(reads[k * kRigExecBone2dChannelCount + c]);
                RigExecBone2dSetChannel(&ch, c,
                                        v.tag == RrInputTag::Bool
                                            ? (v.boolean ? 1.0 : 0.0)
                                            : v.f64);
            }
            const double values[] = {ch.tx,
                                     ch.ty,
                                     ch.rz,
                                     ch.sx,
                                     double(ch.sideFlip),
                                     double(ch.endFlip),
                                     double(ch.lock),
                                     ch.lockX,
                                     ch.lockY,
                                     ch.lockAngle,
                                     ch.offsetX,
                                     ch.offsetY,
                                     ch.targetIndex};
            std::memcpy(out.Data() + k * kRigExecBone2dChannelCount, values,
                        sizeof(values));
        }
        return true;
    }
    case Read::BlendWeight:
        return _RrSolverScalar(
            type, double(p.ReadSolver(lookup.object, RrSolverBlendWeight).f32),
            value, error);
    case Read::Bone2dSelectors: {
        if (!_RrSolverType<double>(type, true) ||
            lookup.object >= p.solverBone2dSelectorRead.size())
            break;
        const std::vector<int32_t> &reads =
            p.solverBone2dSelectorRead[lookup.object];
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(reads.size());
        for (size_t g = 0; g < reads.size(); ++g)
            out[g] = p.ReadRegistered(reads[g]).f64;
        return true;
    }
    case Read::GroupSelectors: {
        if (!_RrSolverType<double>(type, true) ||
            lookup.object >= p.solverJointGroupRead.size())
            break;
        const std::vector<int32_t> &reads =
            p.solverJointGroupRead[lookup.object];
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(reads.size());
        for (size_t g = 0; g < reads.size(); ++g)
            out[g] = p.ReadRegistered(reads[g]).f64;
        return true;
    }
    }
    return _RrSolverFail(
        error, "solver " + std::to_string(lookup.object) + " has no input " +
                   std::to_string(lookup.part) + " of that type");
}

bool
RrStoreSolver(RrProgram *program, size_t step, const eval::OpHostOutput &output,
              const void *value, std::string *error)
{
    RrProgram &p = *program;
    RrStore &store = p.store;
    if (!p.poses || output.object >= p.poses->solvers.size() ||
        output.object >= store.aggregates.size() ||
        output.object >= store.solverOutFrames.size() ||
        output.object >= store.solverOutPresent.size())
        return _RrSolverFail(error, "solver " + std::to_string(output.object) +
                                        " is no solver of the program");
    const RigExecWireSolver &s = p.poses->solvers[output.object];
    if (output.target == uint8_t(eval::OpOutputTarget::SolverPlanar)) {
        if (step >= store.stepOutputs.size())
            return _RrSolverFail(error, "stores a solver result outside a "
                                        "step");
        if (!*static_cast<const uint8_t *>(value))
            store.stepOutputs[step].diagnostics.push_back(
                "diag " + p.TextOrEmpty(s.path) + ": " +
                kRigExecSkeleton2dNotPlanar);
        return true;
    }
    const auto &out = *static_cast<const rigExecRig::PointFrameArray *>(value);
    const size_t first =
        p.opNodes && output.object < p.opNodes->solverLeading.size()
            ? p.opNodes->solverLeading[output.object]
            : 0;
    RrPointFrameArray &aggregate = store.aggregates[output.object];
    const size_t frames =
        out.frames.Size() > first ? out.frames.Size() - first : 0;
    const size_t rests =
        out.rests.Size() > first ? out.rests.Size() - first : 0;
    aggregate.frames.resize(frames);
    aggregate.rests.resize(rests);
    for (size_t k = 0; k < frames; ++k) {
        aggregate.frames[k].points =
            _RrFromSolverLandmarks(out.frames[k + first].points);
        aggregate.frames[k].flags = out.frames[k + first].flags;
    }
    for (size_t k = 0; k < rests; ++k)
        aggregate.rests[k] = _RrFromSolverLandmarks(out.rests[k + first]);
    std::vector<RrPointFrame> &outFrames = store.solverOutFrames[output.object];
    std::vector<char> &outPresent = store.solverOutPresent[output.object];
    if (outFrames.size() != s.outputs.size() ||
        outPresent.size() != s.outputs.size())
        return _RrSolverFail(error, "solver " + std::to_string(output.object) +
                                        " publishes no outputs");
    for (size_t k = 0; k < s.outputs.size(); ++k) {
        const int32_t element = s.outputs[k].second;
        if (element < 0 || size_t(element) >= aggregate.frames.size()) {
            outPresent[k] = 0;
            continue;
        }
        outFrames[k] = RrExtractElementFrame(&aggregate, size_t(element));
        outPresent[k] = 1;
    }
    return true;
}

std::string
RrSolverFailure(const RrProgram *program, uint32_t solver,
                const std::string &error)
{
    const RrProgram &p = *program;
    if (!p.poses || solver >= p.poses->solvers.size())
        return error;
    return p.TextOrEmpty(p.poses->solvers[solver].path) + ": " + error;
}

namespace {

/// The versions list \p part (OpCommitList) of \p commit names, and
/// whether they are base versions; null for no list.
const std::vector<uint32_t> *
_RrCommitVersions(const RigExecWireCommit &commit, int32_t part, bool *base)
{
    using L = eval::OpCommitList;
    *base = part == int32_t(L::BaseCarry) ||
            part == int32_t(L::DescendantBaseCarry);
    switch (L(part)) {
    case L::Entering:
        return &commit.slotReads;
    case L::Carry:
        return &commit.slotCarry;
    case L::BaseCarry:
        return &commit.slotBaseCarry;
    case L::Current:
        return &commit.descendantReads;
    case L::Closest:
        return &commit.closestReads;
    case L::DescendantCarry:
        return &commit.descendantCarry;
    case L::DescendantBaseCarry:
        return &commit.descendantBaseCarry;
    }
    return nullptr;
}

bool
_RrCommitFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Resizes the array \p value of \p type to \p count elements of
/// \p size bytes, its first in \p data; false when \p type holds no such
/// array.
template <class T>
bool
_RrCommitElements(const RigExecTypeDesc &type, void *value, size_t count,
                  T **data)
{
    if (!type.element || !type.resize || !type.data ||
        RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size !=
            sizeof(T))
        return false;
    type.resize(value, count);
    *data = static_cast<T *>(type.data(value));
    return *data || !count;
}

/// Calls \p visit with each candidate position solver batch \p w's
/// solvers published and its frame, the last in batch order winning.
template <class Visit>
void
_RrEachSolverCandidate(const RrProgram &p, size_t w, Visit visit)
{
    for (const int si : p.poses->walkSteps[w].batchSolvers) {
        if (si < 0 || size_t(si) >= p.poses->solvers.size() ||
            size_t(si) >= p.store.solverOutFrames.size() ||
            size_t(si) >= p.store.solverOutPresent.size())
            continue;
        const RigExecWireSolver &s = p.poses->solvers[size_t(si)];
        const auto &frames = p.store.solverOutFrames[size_t(si)];
        const auto &present = p.store.solverOutPresent[size_t(si)];
        for (size_t k = 0; k < s.outPosition.size() && k < frames.size() &&
                           k < present.size();
             ++k)
            if (present[k] && s.outPosition[k] >= 0)
                visit(size_t(s.outPosition[k]), frames[k]);
    }
}

/// Whether candidate \p pos of commit \p w is present this frame, and its
/// frame in \p frame when it is.
bool
_RrCandidateAt(const RrProgram &p, size_t w, size_t pos, RrPointFrame *frame)
{
    const RigExecWireCommit &commit = p.poses->commits[w];
    const RrCommitScratch &scratch = p.store.commits[w];
    if (!commit.solverOutput) {
        if (pos < scratch.frames.size())
            *frame = scratch.frames[pos];
        return pos < scratch.present.size() && scratch.present[pos];
    }
    bool present = false;
    _RrEachSolverCandidate(p, w, [&](size_t at, const RrPointFrame &f) {
        if (at == pos) {
            *frame = f;
            present = true;
        }
    });
    return present;
}

/// The candidate position whose write descendant \p k of \p commit
/// carries, or -1 when it carries a version written before the commit.
int
_RrCarriedCandidate(const RigExecWireCommit &commit, size_t k, bool base)
{
    const std::vector<uint32_t> &writes =
        base ? commit.slotBaseWrites : commit.slotWrites;
    const std::vector<uint32_t> &carry =
        base ? commit.descendantBaseCarry : commit.descendantCarry;
    for (size_t pos = 0; pos < commit.slots.size() && pos < writes.size();
         ++pos)
        if (commit.slots[pos] == commit.propagate[k].first &&
            writes[pos] == carry[k])
            return int(pos);
    return -1;
}

} // namespace

bool
RrLookUpCommit(const RrProgram *program, const eval::OpLookup &lookup,
               const RigExecTypeDesc &type, void *value, std::string *error)
{
    const RrProgram &p = *program;
    if (lookup.object >= p.poses->commits.size() ||
        lookup.object >= p.store.commits.size())
        return _RrCommitFail(error, "commit " + std::to_string(lookup.object) +
                                        " is no commit of the program");
    const RigExecWireCommit &commit = p.poses->commits[lookup.object];
    const RrCommitScratch &scratch = p.store.commits[lookup.object];
    const size_t slots = commit.slots.size();
    if (lookup.source == uint8_t(eval::OpLookupSource::CommitVersions)) {
        bool base = false;
        const std::vector<uint32_t> *versions =
            _RrCommitVersions(commit, lookup.part, &base);
        const std::vector<RrPointFrame> &store =
            base ? p.store.base : p.store.fin;
        RrPointFrame *out = nullptr;
        if (!versions ||
            !_RrCommitElements(type, value, versions->size(), &out))
            return _RrCommitFail(error, "commit " +
                                            std::to_string(lookup.object) +
                                            " names no frame list " +
                                            std::to_string(lookup.part));
        const bool descendants =
            lookup.part == int32_t(eval::OpCommitList::DescendantCarry) ||
            lookup.part == int32_t(eval::OpCommitList::DescendantBaseCarry);
        for (size_t i = 0; i < versions->size(); ++i) {
            const int pos =
                descendants ? _RrCarriedCandidate(commit, i, base) : -1;
            if (pos >= 0 &&
                _RrCandidateAt(p, lookup.object, size_t(pos), &out[i]))
                continue;
            const uint32_t version =
                pos >= 0 ? (base ? commit.slotBaseCarry
                                 : commit.slotCarry)[size_t(pos)]
                         : (*versions)[i];
            if (version >= store.size())
                return _RrCommitFail(error, "pose version " +
                                                std::to_string(version) +
                                                " is no pose frame of the "
                                                "program");
            out[i] = store[version];
        }
        return true;
    }
    if (lookup.part == 0) {
        RrPointFrame *out = nullptr;
        if (!_RrCommitElements(type, value, slots, &out) ||
            (!commit.solverOutput && scratch.frames.size() < slots))
            return _RrCommitFail(error, "the candidates of commit " +
                                            std::to_string(lookup.object) +
                                            " are no frame list");
        if (!commit.solverOutput) {
            for (size_t pos = 0; pos < slots; ++pos)
                out[pos] = scratch.frames[pos];
            return true;
        }
        for (size_t pos = 0; pos < slots; ++pos)
            out[pos] = RrPointFrame();
        _RrEachSolverCandidate(p, lookup.object,
                               [&](size_t pos, const RrPointFrame &frame) {
                                   if (pos < slots)
                                       out[pos] = frame;
                               });
        return true;
    }
    uint8_t *out = nullptr;
    if (lookup.part != 1 || !_RrCommitElements(type, value, slots, &out) ||
        (!commit.solverOutput && scratch.present.size() < slots))
        return _RrCommitFail(error, "commit " + std::to_string(lookup.object) +
                                        " names no candidate list " +
                                        std::to_string(lookup.part));
    for (size_t pos = 0; pos < slots; ++pos)
        out[pos] = commit.solverOutput ? 0 : uint8_t(scratch.present[pos] != 0);
    if (commit.solverOutput)
        _RrEachSolverCandidate(p, lookup.object,
                               [&](size_t pos, const RrPointFrame &) {
                                   if (pos < slots)
                                       out[pos] = 1;
                               });
    return true;
}

bool
RrStoreCommit(RrProgram *program, const eval::OpHostOutput &output,
              const void *value, std::string *error)
{
    RrProgram &p = *program;
    if (output.object >= p.poses->commits.size() ||
        output.object >= p.store.commits.size())
        return _RrCommitFail(error, "commit " + std::to_string(output.object) +
                                        " is no commit of the program");
    const RigExecWireCommit &commit = p.poses->commits[output.object];
    RrCommitScratch &scratch = p.store.commits[output.object];
    using T = eval::OpOutputTarget;
    const auto &array =
        *static_cast<const rigExecStd::RigExecArray<uint8_t> *>(value);
    if (output.target == uint8_t(T::CommitPresent)) {
        if (array.Size() != scratch.present.size())
            return _RrCommitFail(
                error, "commit " + std::to_string(output.object) + " has " +
                           std::to_string(scratch.present.size()) +
                           " candidates, not " + std::to_string(array.Size()));
        bool any = false;
        for (size_t pos = 0; pos < array.Size(); ++pos) {
            scratch.present[pos] = array[pos] ? 1 : 0;
            any = any || array[pos];
        }
        scratch.abandoned = !any;
        return true;
    }
    const bool base = output.target == uint8_t(T::CommitBaseSlots) ||
                      output.target == uint8_t(T::CommitBaseDescendants);
    const std::vector<uint32_t> *versions =
        output.target == uint8_t(T::CommitSlots) ? &commit.slotWrites
        : output.target == uint8_t(T::CommitDescendants)
            ? &commit.descendantWrites
        : output.target == uint8_t(T::CommitBaseSlots) ? &commit.slotBaseWrites
        : output.target == uint8_t(T::CommitBaseDescendants)
            ? &commit.descendantBaseWrites
            : nullptr;
    std::vector<RrPointFrame> &store = base ? p.store.base : p.store.fin;
    const auto &frames =
        *static_cast<const rigExecStd::RigExecArray<RrPointFrame> *>(value);
    if (!versions || frames.Size() != versions->size())
        return _RrCommitFail(
            error, "commit " + std::to_string(output.object) + " writes no " +
                       std::to_string(frames.Size()) + " frames at target " +
                       std::to_string(output.target));
    for (size_t i = 0; i < frames.Size(); ++i) {
        if ((*versions)[i] >= store.size())
            return _RrCommitFail(error, "pose version " +
                                            std::to_string((*versions)[i]) +
                                            " is no pose frame of the program");
        store[(*versions)[i]] = frames[i];
    }
    return true;
}

std::string
RrCommitFailure(const RrProgram *program, uint32_t commit,
                const std::string &error)
{
    const RrProgram &p = *program;
    if (commit >= p.poses->commits.size())
        return error;
    const RigExecWireCommit &c = p.poses->commits[commit];
    std::string mover = p.TextOrEmpty(c.moverPath);
    if (mover.empty() && commit < p.poses->walkSteps.size()) {
        const auto &batch = p.poses->walkSteps[commit].batchSolvers;
        if (!batch.empty() && batch.front() >= 0 &&
            size_t(batch.front()) < p.poses->solvers.size())
            mover = p.TextOrEmpty(p.poses->solvers[size_t(batch.front())].path);
    }
    const auto &meta = *p.slotMeta;
    return RigExecCommitFailureText(mover, error, [&](size_t k) {
        return k < c.propagate.size() &&
                       size_t(c.propagate[k].first) < meta.paths.size()
                   ? p.TextOrEmpty(meta.paths[size_t(c.propagate[k].first)])
                   : "descendant " + std::to_string(k);
    });
}

} // namespace rigExec
