// Runtime pose scratch allocation, input prologue, and ladder composition.

#include "rigExecMath/avarScale.h"
#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "poseInternal.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

RrMat4d
_RrWireMatrix(const RigExecWireMatrix4d &wire)
{
    RrMat4d m;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            m[r][c] = wire[r * 4 + c];
        }
    }
    return m;
}

std::array<RrVec3d, 4>
_RrWireLandmarks(const std::array<RigExecWireVec3d, 4> &wire)
{
    std::array<RrVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i) {
        out[i] = RrVec3d(wire[i][0], wire[i][1], wire[i][2]);
    }
    return out;
}

} // namespace

namespace runtimePoseDetail {

RrMat4d
_RrIdentity()
{
    RrMat4d m;
    m.SetIdentity();
    return m;
}

// RigExecBakedComposeAvars (bakedProgramImpl.h), written the same way
// rather than algebraically simplified. `order` is the resolved token
// text; anything but a 3-letter sequence composes XYZ.
RrMat4d
_RrComposeAvars(double tx, double ty, double tz, double sx, double sy,
                double sz, double rx, double ry, double rz, double rspin,
                const std::string &order)
{
    const RrVec3d axes[3] = {RrVec3d(1, 0, 0), RrVec3d(0, 1, 0),
                             RrVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order;
    if (sequence.size() != 3) {
        sequence = SchemaValueText::xyz;
    }
    RrMat4d m;
    m.SetIdentity();
    m[0][0] = RigExecNormalizeAvarScale(sx);
    m[1][1] = RigExecNormalizeAvarScale(sy);
    m[2][2] = RigExecNormalizeAvarScale(sz);
    for (const char axis : sequence) {
        const int index = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[index] != 0.0) {
            m = m * RrMat4d(RrRotation(axes[index], angles[index]), RrVec3d(0));
        }
    }
    if (rspin != 0.0) {
        m = m * RrMat4d(RrRotation(axes[0], rspin), RrVec3d(0));
    }
    RrMat4d t;
    t.SetIdentity();
    t.SetTranslateOnly(RrVec3d(tx, ty, tz));
    return m * t;
}

// The step body's `live`: whether a baked parameter has to be re-read
// at all (bakedPose.cpp): read per run, or an override stands on its walk.
bool
_RrLive(const RrProgram *program, const RigExecWireInput &read)
{
    if ((read.flags & (uint8_t(RigExecWireInputReadFlags::Varying) |
                       uint8_t(RigExecWireInputReadFlags::SourceBacked))) !=
        0) {
        return true;
    }
    return read.overrideIndex >= 0 &&
           size_t(read.overrideIndex) < program->store.overridden.size() &&
           program->store.overridden[size_t(read.overrideIndex)];
}

bool
_RrLiveSolver(const RrProgram *program, size_t solver, int field)
{
    return _RrLive(program, program->RegisteredInput(
                                program->solverRead[solver][size_t(field)]));
}

RrPoseScratch *
_RrScratch(RrProgram *program)
{
    return static_cast<RrPoseScratch *>(program->pose.get());
}

} // namespace runtimePoseDetail

const std::vector<RrPointFrame> &
RrPoseRestFrames(const RrProgram *program)
{
    return static_cast<const RrPoseScratch *>(program->pose.get())->restFrames;
}

bool
RrPoseSizeScratch(RrProgram *program, std::string *error)
{
    if (!program || !program->slotMeta || !program->constants ||
        !program->poses || !program->geometry) {
        if (error) {
            *error = "pose scratch needs the program tables";
        }
        return false;
    }
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireConstants &constants = *program->constants;
    const RigExecWireDomainPose &poses = *program->poses;
    const size_t slots = meta.paths.size();
    if (constants.restM.size() != slots || constants.restPts.size() != slots ||
        constants.restFrames.size() != slots ||
        constants.selfD.size() != slots || constants.posedD.size() != slots ||
        constants.parentSpaceM.size() != slots ||
        constants.parentSpaceAuthored.size() != slots ||
        constants.parentDinv.size() != slots ||
        constants.rotOrder.size() != slots ||
        constants.restRoundTrip.size() != slots ||
        constants.defaultRoundTrip.size() != slots ||
        constants.posedAuthored.size() != slots ||
        constants.posedAuthoredM.size() != slots ||
        constants.noScaleAvars.size() != slots) {
        if (error) {
            *error = "pose constants do not cover the slots";
        }
        return false;
    }
    auto scratch = std::make_shared<RrPoseScratch>();
    const RrMat4d identity = _RrIdentity();
    scratch->switchInputs.resize(poses.spaceSwitches.size());
    for (size_t i = 0; i < poses.spaceSwitches.size(); ++i)
        scratch->switchInputs[i].sources.resize(
            poses.spaceSwitches[i].sourceSlots.size());
    scratch->restM.assign(slots, identity);
    scratch->selfD.assign(slots, identity);
    scratch->posedD.assign(slots, identity);
    scratch->parentSpaceM.assign(slots, identity);
    scratch->parentSpaceAuthored.assign(slots, 0);
    scratch->parentDinv.assign(slots, identity);
    scratch->posedAuthoredM.assign(slots, identity);
    scratch->restRoundTrip.assign(slots, identity);
    scratch->defaultRoundTrip.assign(slots, identity);
    scratch->restPts.assign(slots, RrIdentityLandmarks());
    scratch->restFrames.assign(slots, RrPointFrame());
    scratch->rotOrder.assign(slots, 0);
    scratch->posedAuthored.assign(slots, 0);
    scratch->noScaleAvars.assign(slots, 0);
    // Empty on a minor-1 binary, which is every axis +1.
    scratch->rotationSign.assign(slots, 0);
    for (size_t i = 0; i < slots; ++i) {
        scratch->restM[i] = _RrWireMatrix(constants.restM[i]);
        scratch->selfD[i] = _RrWireMatrix(constants.selfD[i]);
        scratch->posedD[i] = _RrWireMatrix(constants.posedD[i]);
        scratch->parentSpaceM[i] = _RrWireMatrix(constants.parentSpaceM[i]);
        scratch->parentSpaceAuthored[i] =
            constants.parentSpaceAuthored[i] ? 1 : 0;
        scratch->parentDinv[i] = _RrWireMatrix(constants.parentDinv[i]);
        scratch->posedAuthoredM[i] = _RrWireMatrix(constants.posedAuthoredM[i]);
        scratch->restRoundTrip[i] = _RrWireMatrix(constants.restRoundTrip[i]);
        scratch->defaultRoundTrip[i] =
            _RrWireMatrix(constants.defaultRoundTrip[i]);
        scratch->restPts[i] = _RrWireLandmarks(constants.restPts[i]);
        scratch->restFrames[i] = RrWireToFrame(constants.restFrames[i]);
        scratch->rotOrder[i] = constants.rotOrder[i];
        scratch->posedAuthored[i] = constants.posedAuthored[i] ? 1 : 0;
        scratch->noScaleAvars[i] = constants.noScaleAvars[i] ? 1 : 0;
        scratch->rotationSign[i] =
            i < constants.rotationSign.size()
                ? (unsigned char)(constants.rotationSign[i] & 7u)
                : 0;
    }
    scratch->lastRestM = scratch->restM;
    scratch->lastSelfD = scratch->selfD;
    scratch->lastParentDinv = scratch->parentDinv;
    scratch->lastPosedAuthoredM = scratch->posedAuthoredM;
    scratch->lastPosedAuthored = scratch->posedAuthored;
    scratch->lastRotOrder = scratch->rotOrder;
    scratch->lastPosedD = scratch->posedD;
    scratch->lastParentSpaceM = scratch->parentSpaceM;
    scratch->lastParentSpaceAuthored = scratch->parentSpaceAuthored;
    scratch->lastRotationSign = scratch->rotationSign;
    scratch->solvers.resize(poses.solvers.size());
    for (size_t s = 0; s < poses.solvers.size(); ++s) {
        const RigExecWireSolver &wire = poses.solvers[s];
        RrPoseSolverState &state = scratch->solvers[s];
        for (const auto &rest : wire.jointRests) {
            state.jointRests.push_back(_RrWireLandmarks(rest));
        }
        for (size_t k = 0; k < 3; ++k) {
            state.ikRests[k] = _RrWireLandmarks(wire.ikRests[k]);
        }
        for (const auto &rest : wire.splineJointRests) {
            state.splineJointRests.push_back(_RrWireLandmarks(rest));
        }
    }
    scratch->recordAfter.assign(poses.commits.size(), 1);
    scratch->recordEveryTarget.assign(poses.commits.size(), 1);
    scratch->weightScratch.assign(poses.constraints.size(), {});
    scratch->weightError.assign(poses.constraints.size(), {});
    scratch->deltaValues.assign(program->geometry->deltaBasePaths.size(),
                                identity);
    scratch->deltaPresent.assign(program->geometry->deltaBasePaths.size(), 0);
    // Per-commit IK chains, sized from the wire commit the way Build
    // sizes them: the IK write-back and the atomic check walk these.
    RrStore &store = program->store;
    for (size_t c = 0; c < poses.commits.size() && c < store.commits.size();
         ++c) {
        const size_t targets = poses.commits[c].targetReads.size();
        store.commits[c].ikChain.assign(targets, RrPointFrame());
        store.commits[c].ikPrepared.assign(targets, RrPointFrame());
        store.commits[c].ikRest.assign(targets, RrPointFrame());
        store.commits[c].ikSolved.clear();
    }
    program->pose = scratch;
    return true;
}

void
RrProloguePose(RrProgram *program)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // The validator checked the static tables' sizes against the program.
    const RrStatic &statics = program->statics;
    const RrInputState &inputs = program->inputState;
    // Xform-derived slots, seeded from the stage values the bake captured.
    const size_t captured =
        program->requiredStageFramesAdmission.admitted
            ? meta.xformSlots.size()
            : size_t(program->requiredStageFramesAdmission.firstBadTarget);
    for (size_t k = 0; k < captured; ++k) {
        const int slot = meta.xformSlots[k];
        const RrMat4d matrix = _RrWireMatrix(statics.XformBase(k));
        store.xformBase[k] = matrix;
        const RrPointFrame frame = RrWireToFrame(poses.xformFrames[k]);
        store.base[size_t(slot)] = frame;
        store.fin[size_t(slot)] = frame;
    }
    if (!program->requiredStageFramesAdmission.admitted)
        return;
    // The target transform each geometry-domain constraint measures
    // its delta against.
    for (size_t k = 0; k < geometry.deltaBasePaths.size(); ++k) {
        store.deltaBaseOk[k] = statics.DeltaBaseOk(k) ? 1 : 0;
        store.deltaBaseMatrix[k] = _RrWireMatrix(statics.DeltaBase(k));
    }
    // The plain Xformables a constraint reads as a source. ok is derived
    // from the frame's own usability and the file's NativeSource.ok is not
    // read: a store read that failed has no frame to seed.
    for (size_t k = 0; k < poses.nativeSources.size(); ++k) {
        const RrPointFrame frame = RrWireToFrame(statics.NativeFrame(k));
        store.nativeFrames[k] = frame;
        store.nativeFrameOk[k] = RrFrameUsable(frame) ? 1 : 0;
    }
    // Constraint array inputs are copied by the input sampler. The owning
    // Constraint operation validates cardinality and expands neutral defaults.
}

} // namespace rigExec
