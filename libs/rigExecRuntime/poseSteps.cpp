// Runtime pose-step dispatch, commit deltas, and propagation.

#include "poseInternal.h"
#include "poseEvalHost.h"
#include "spaces.h"
#include "rigExecRuntime/spaceSwitchMath.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecRigTypes/rigTypes.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

struct RrPosePropagateMath {
    static bool Usable(const RrPointFrame &frame)
    {
        return RrFrameUsable(frame);
    }
    static RrPointFrame Carry(const RrPointFrame &frame, const RrMat4d &delta)
    {
        return RrMatrixToPoints(frame.points, delta);
    }
};

// Propagation outcomes, in RigExecBakedPropagateOutcome order.
enum _RrPropagateOutcome : uint8_t {
    _RrStaged = 0,
    _RrSkipped = 1,
    _RrUnusableDescendant = 3,
    _RrSingularDelta = 4,
    _RrInvalidResult = 5,
};

std::string
_RrKindName(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::ComposeSubtree:
        return "ComposeSubtree";
    case RigExecWireStepKind::Solve:
        return "Solve";
    case RigExecWireStepKind::ReservedSolverCommit:
        return "ReservedSolverCommit";
    case RigExecWireStepKind::Constraint:
        return "Constraint";
    case RigExecWireStepKind::ReservedCommitDelta:
        return "ReservedCommitDelta";
    case RigExecWireStepKind::ReservedPropagateChunk:
        return "ReservedPropagateChunk";
    case RigExecWireStepKind::ReservedCommitApply:
        return "ReservedCommitApply";
    case RigExecWireStepKind::SnapshotFinals:
        return "SnapshotFinals";
    case RigExecWireStepKind::PoseInterpolator:
        return "PoseInterpolator";
    default:
        return "Unknown";
    }
}

// Whether slot \p i's avars and ladder rows are there to compose.
bool
_RrComposable(const RrProgram &program, const RrPoseScratch &scratch, size_t i)
{
    return i * 11 + 11 <= program.store.avars.size() &&
           i < scratch.noScaleAvars.size() && i < scratch.rotOrder.size() &&
           i < scratch.posedD.size() && i < scratch.parentSpaceM.size() &&
           i < scratch.parentSpaceAuthored.size() &&
           i < scratch.parentDinv.size() && i < scratch.posedAuthored.size() &&
           i < scratch.posedAuthoredM.size();
}

// Slot \p i's avars as one matrix, as the program composes them
// (_ComposeAvarsOf, bakedPose.cpp).
RrMat4d
_RrComposeAvarsOf(const RrProgram &program, const RrPoseScratch &scratch,
                  size_t i)
{
    const double *a = &program.store.avars[i * 11];
    const double units = a[10];
    const bool noScale = scratch.noScaleAvars[i] != 0;
    // avars:rotationSign, applied to the avar where the other two paths
    // apply it (computations.cpp, bakedPose.cpp).
    const unsigned sign =
        i < scratch.rotationSign.size() ? scratch.rotationSign[i] : 0u;
    const double signX = RigExecRotationSignFromMask(sign, 0);
    return _RrComposeAvars(
        a[0] * units, a[1] * units, a[2] * units, noScale ? 1.0 : a[3],
        noScale ? 1.0 : a[4], noScale ? 1.0 : a[5], a[6] * signX,
        a[7] * RigExecRotationSignFromMask(sign, 1),
        a[8] * RigExecRotationSignFromMask(sign, 2), a[9] * signX,
        program.TextOrEmpty(scratch.rotOrder[i]));
}

// _SpaceOfFrame: an unusable frame selects the NaN sentinel, so the
// failure survives into every descendant instead of being scrubbed into a
// plausible identity.
RrMat4d
_RrSpaceOfFrame(const RrPointFrame &frame)
{
    RrMat4d space = _RrIdentity();
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &space)) {
        space = _RrIdentity();
        space[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return space;
}

// Whether every slot \p read names is there to read or recompose.
bool
_RrVersionReadable(const RrProgram &program, const RrPoseScratch &scratch,
                   const RigExecWireFrameVersion &read)
{
    if (read.context >= 0) {
        return size_t(read.context) < program.store.switchFrames.size();
    }
    if (read.anchor >= 0 &&
        size_t(read.anchor) >= program.store.posedM.size()) {
        return false;
    }
    return read.recompose.empty();
}

// Read an explicit checkpoint output or the bound current provider matrix.
RrMat4d
_RrReadFrameVersion(const RrProgram &program, const RrPoseScratch &scratch,
                    const RigExecWireFrameVersion &read)
{
    if (read.context >= 0) {
        return program.store.switchFrames[size_t(read.context)];
    }
    RrMat4d posed = read.anchor >= 0 ? program.store.posedM[size_t(read.anchor)]
                                     : _RrIdentity();
    return posed;
}

} // namespace

namespace runtimePoseDetail {

std::string
_RrStepHead(const RrProgram *program, size_t step)
{
    const RigExecWireStep &wire = (*program->steps)[step];
    return "pose step " + _RrKindName(wire.kind) + " object " +
           std::to_string(wire.object) + " part " + std::to_string(wire.part);
}

// An IK/FK match's pole: the frame its space switch composes avars onto,
// with each source flagged in \p rides carried by \p moved (the IK
// control's move). The switch's inputs are gathered as the compose step
// gathers them, with the avars at identity. False when they cannot be read.
bool
_RrMatchPoleParent(RrProgram *program, int switchIndex,
                   const std::vector<uint8_t> &rides, const RrMat4d &moved,
                   RrMat4d *out)
{
    const RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    if (switchIndex < 0 ||
        size_t(switchIndex) >= program->poses->spaceSwitches.size()) {
        return false;
    }
    const RigExecWireSpaceSwitch &sw =
        program->poses->spaceSwitches[size_t(switchIndex)];
    const size_t i = size_t(sw.slot);
    if (sw.slot < 0 || i >= meta.parent.size() || i >= scratch->posedD.size() ||
        i >= scratch->parentDinv.size() ||
        sw.sourceReads.size() != sw.sourceSlots.size() ||
        !_RrVersionReadable(*program, *scratch, *sw.parentRead) ||
        !_RrVersionReadable(*program, *scratch, *sw.spaceRead)) {
        return false;
    }
    for (const RigExecWireFrameVersion &read : sw.sourceReads) {
        if (!_RrVersionReadable(*program, *scratch, read))
            return false;
    }
    const int parent = meta.parent[i];
    if (parent >= 0 && size_t(parent) >= scratch->defaultRoundTrip.size()) {
        return false;
    }
    RigExecSpaceSwitchRecordT<RrVec3d> record;
    record.filters.assign(sw.filters.begin(), sw.filters.end());
    record.twistAxis =
        RrVec3d(sw.twistAxis[0], sw.twistAxis[1], sw.twistAxis[2]);
    std::copy(sw.affectTranslation.begin(), sw.affectTranslation.end(),
              record.affectTranslation.begin());
    std::copy(sw.affectRotation.begin(), sw.affectRotation.end(),
              record.affectRotation.begin());
    std::copy(sw.affectScale.begin(), sw.affectScale.end(),
              record.affectScale.begin());
    RigExecSpaceSwitchInputsT<RrMat4d> input;
    input.avars = _RrIdentity();
    input.posedDefault = scratch->posedD[i];
    input.parentDefaultInverse = scratch->parentDinv[i];
    input.parentPosed = _RrReadFrameVersion(*program, *scratch, *sw.parentRead);
    input.parentDefault =
        parent >= 0 ? scratch->defaultRoundTrip[size_t(parent)] : _RrIdentity();
    const auto selector = program->ReadSpaceSwitch(size_t(switchIndex));
    input.active = selector.f64;
    if (sw.tokenIndex) {
        input.active = 0.0;
        const auto text = program->TextOrEmpty(selector.token);
        const auto found = std::find(sw.labels.begin(), sw.labels.end(), text);
        if (found != sw.labels.end()) {
            input.active = double(found - sw.labels.begin());
        }
    }
    input.hasCarry = sw.spaceSlot >= 0;
    if (input.hasCarry) {
        input.spaceDefault = scratch->defaultRoundTrip[size_t(sw.spaceSlot)];
        input.spacePosed =
            _RrReadFrameVersion(*program, *scratch, *sw.spaceRead);
    }
    for (size_t source = 0; source < sw.sourceSlots.size(); ++source) {
        RigExecSpaceSwitchSourceT<RrMat4d> value;
        const int slot = sw.sourceSlots[source];
        value.world = slot < 0;
        if (slot >= 0) {
            value.defaultSpace = scratch->defaultRoundTrip[size_t(slot)];
            value.posedSpace =
                _RrReadFrameVersion(*program, *scratch, sw.sourceReads[source]);
            if (source < rides.size() && rides[source]) {
                value.posedSpace = value.posedSpace * moved;
            }
        }
        input.sources.push_back(value);
    }
    RrMat4d output;
    if (!RigExecRunSpaceSwitchArithmetic<RrSpaceSwitchMath>(record, input,
                                                            &output)) {
        return false;
    }
    // The compose publishes the frame of this matrix; read it back the same
    // way, so the parent is the one a frame read of the pole would give.
    const RrPointFrame frame = RrFrameFromMatrix(output);
    return RrFrameUsable(frame) &&
           RrPointsToMatrix(RrIdentityLandmarks(), frame.points, out);
}

RigExecIkFkMatchChannels
_RrMatchChannelsOf(const RrProgram &program, const RrPoseScratch &scratch,
                   size_t slot)
{
    RigExecIkFkMatchChannels channels;
    const double *a = &program.store.avars[slot * 11];
    const bool noScale = scratch.noScaleAvars[slot] != 0;
    const unsigned sign =
        slot < scratch.rotationSign.size() ? scratch.rotationSign[slot] : 0u;
    for (int k = 0; k < 3; ++k) {
        channels.t[k] = a[k];
        channels.s[k] = noScale ? 1.0 : a[3 + k];
        channels.r[k] = a[6 + k];
        channels.sign[k] = (sign & (1u << k)) ? -1.0 : 1.0;
    }
    channels.spin = a[9];
    channels.units = a[10];
    const std::string order = program.TextOrEmpty(scratch.rotOrder[slot]);
    if (order.size() == 3) {
        std::copy(order.begin(), order.end(), channels.order);
    }
    return channels;
}

// A switch's space match (spaceMatchKernel.h), with the inputs its compose
// just ran on: the channels per source index that keep the control on the
// frame it composed.
static void
_RrRunSpaceMatch(RrProgram *program, const RrPoseScratch &scratch,
                 size_t switchIndex, size_t slot,
                 const RigExecSpaceSwitchRecordT<RrVec3d> &record,
                 const RigExecSpaceSwitchInputsT<RrMat4d> &input,
                 const RrPointFrame &posed,
                 std::vector<std::string> *diagnostics)
{
    RrStore &store = program->store;
    if (switchIndex >= store.spaceMatchValid.size())
        return;
    store.spaceMatchValid[switchIndex] = 0;
    const size_t spaces = input.sources.size();
    RrMat4d target;
    if (!spaces ||
        !RrPointsToMatrix(RrIdentityLandmarks(), posed.points, &target)) {
        return;
    }
    RigExecSpaceSwitchInputsT<RrMat4d> trial = input;
    const auto compose = [&](const double *avars, double active,
                             RrMat4d *composed) {
        std::copy(avars, avars + 16, &trial.avars._mtx[0][0]);
        trial.active = active;
        return RigExecRunSpaceSwitchArithmetic<RrSpaceSwitchMath>(record, trial,
                                                                  composed);
    };
    std::vector<double> &values = store.spaceMatchValues[switchIndex];
    values.resize(spaces * RigExecSpaceMatchStride);
    const RigExecSpaceMatchOutcome outcome =
        RigExecSpaceMatchT<RrMat3d, RrVec3d>(
            target, _RrMatchChannelsOf(*program, scratch, slot), spaces,
            compose, values.data());
    store.spaceMatchValid[switchIndex] =
        outcome == RigExecSpaceMatchOutcome::Matched ? 1 : 0;
    if (outcome == RigExecSpaceMatchOutcome::NotFinite && diagnostics &&
        slot < program->slotMeta->paths.size()) {
        diagnostics->push_back(
            program->TextOrEmpty(program->slotMeta->paths[slot]) +
            kRigExecSpaceMatchNotFinite);
    }
}

} // namespace runtimePoseDetail

bool
RrRunPoseStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    if (step >= program->steps->size() || step >= store.stepOutputs.size()) {
        if (error) {
            *error = "pose step " + std::to_string(step) + " names no step";
        }
        return false;
    }
    const RigExecWireStep &wire = (*program->steps)[step];
    RrStepOutput &output = store.stepOutputs[step];
    (void)output;

    switch (wire.kind) {
    case RigExecWireStepKind::ComposeSubtree: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->composeGroups.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no compose group";
            }
            return false;
        }
        const RigExecWireComposeGroup &group =
            program->poses->composeGroups[size_t(wire.object)];
        const RigExecWireSlotMeta &meta = *program->slotMeta;
        const auto noSlot = [&]() {
            if (error) {
                *error = _RrStepHead(program, step) + " names no slot";
            }
            return false;
        };
        RrPoseHost host(program);
        for (int i = group.begin; i < group.end; ++i) {
            if (i < 0 || !host.inputs().composeSlotBounds(size_t(i))) {
                return noSlot();
            }
            if (!host.inputs().isPoseSlot(size_t(i))) {
                continue;
            }
            if (!host.inputs().providerActive(size_t(i))) {
                eval::PublishComposedSlotUnavailable(host, size_t(i));
                continue;
            }
            // One int per slot, and only on a rig that has a switch
            // at all: the ordinary compose falls straight through.
            const int switchIndex = program->spaceSwitchBySlot.empty()
                                        ? -1
                                        : program->spaceSwitchBySlot[size_t(i)];
            if (scratch->posedAuthored[size_t(i)]) {
                store.base[size_t(i)] =
                    RrFrameFromMatrix(scratch->posedAuthoredM[size_t(i)]);
            } else if (switchIndex < 0) {
                if (size_t(i) >= meta.parent.size()) {
                    return noSlot();
                }
                const int parent = host.inputs().parent(size_t(i));
                RrMat4d parentPosed;
                if (parent < 0) {
                    parentPosed = _RrIdentity();
                } else if (!host.inputs().posedMatrix(size_t(parent),
                                                      &parentPosed)) {
                    return noSlot();
                }
                std::array<RrVec3d, 4> points;
                unsigned flags = 0;
                if (!eval::ComposeSlotFrame(host, size_t(i), parentPosed,
                                            &points, &flags)) {
                    return noSlot();
                }
                host.state().writeBase(size_t(i), points, flags);
            } else {
                if (!_RrComposable(*program, *scratch, size_t(i)) ||
                    size_t(i) >= scratch->defaultRoundTrip.size() ||
                    size_t(i) >= meta.parent.size()) {
                    return noSlot();
                }
                const RrMat4d avars =
                    _RrComposeAvarsOf(*program, *scratch, size_t(i));
                const int parent = meta.parent[size_t(i)];
                // A switched slot composes against the SELECTED
                // source's pair of spaces instead of its namespace
                // parent's. Both halves come from one source, so at
                // rest they cancel and no space moves the rig
                // standing still. Every provider frame it reads is
                // read at the version the program bound at Build,
                // never as whatever the slot holds now: the step
                // order does not place the namespace parent's
                // posedM before this step.
                const RigExecWireSpaceSwitch &sw =
                    program->poses->spaceSwitches[size_t(switchIndex)];
                bool readable =
                    sw.sourceReads.size() == sw.sourceSlots.size() &&
                    _RrVersionReadable(*program, *scratch, *sw.parentRead) &&
                    _RrVersionReadable(*program, *scratch, *sw.spaceRead);
                for (const RigExecWireFrameVersion &read : sw.sourceReads) {
                    readable = readable &&
                               _RrVersionReadable(*program, *scratch, read);
                }
                if (!readable ||
                    (parent >= 0 &&
                     size_t(parent) >= scratch->defaultRoundTrip.size())) {
                    return noSlot();
                }
                // `local` is avars * default:space reached the long
                // way round -- compose the UNSWITCHED world, then
                // divide the namespace parent back out -- rather than
                // as `avars * selfD`, which is the same quantity in
                // exact arithmetic and NOT the same in doubles. The
                // program can only reach it this way, the two answers
                // are compared bit for bit, and 4e-15 of disagreement
                // here propagates to every descendant. The parent is
                // read before its own switch when that resolves in
                // this switch's round or later.
                const RrMat4d parentPosed =
                    _RrReadFrameVersion(*program, *scratch, *sw.parentRead);
                const RrMat4d parentDefault =
                    parent >= 0 ? scratch->defaultRoundTrip[size_t(parent)]
                                : _RrIdentity();
                RigExecSpaceSwitchRecordT<RrVec3d> record;
                record.filters.assign(sw.filters.begin(), sw.filters.end());
                record.twistAxis =
                    RrVec3d(sw.twistAxis[0], sw.twistAxis[1], sw.twistAxis[2]);
                std::copy(sw.affectTranslation.begin(),
                          sw.affectTranslation.end(),
                          record.affectTranslation.begin());
                std::copy(sw.affectRotation.begin(), sw.affectRotation.end(),
                          record.affectRotation.begin());
                std::copy(sw.affectScale.begin(), sw.affectScale.end(),
                          record.affectScale.begin());
                RigExecSpaceSwitchInputsT<RrMat4d> input;
                input.avars = avars;
                input.posedDefault = scratch->posedD[size_t(i)];
                input.parentDefaultInverse = scratch->parentDinv[size_t(i)];
                input.parentPosed = parentPosed;
                input.parentDefault = parentDefault;
                const auto selector =
                    program->ReadSpaceSwitch(size_t(switchIndex));
                input.active = selector.f64;
                if (sw.tokenIndex) {
                    input.active = 0.0;
                    const auto text = program->TextOrEmpty(selector.token);
                    const auto found =
                        std::find(sw.labels.begin(), sw.labels.end(), text);
                    if (found != sw.labels.end())
                        input.active = double(found - sw.labels.begin());
                }
                input.hasCarry = sw.spaceSlot >= 0;
                if (input.hasCarry) {
                    input.spaceDefault =
                        scratch->defaultRoundTrip[size_t(sw.spaceSlot)];
                    input.spacePosed =
                        _RrReadFrameVersion(*program, *scratch, *sw.spaceRead);
                }
                for (size_t source = 0; source < sw.sourceSlots.size();
                     ++source) {
                    RigExecSpaceSwitchSourceT<RrMat4d> value;
                    const int slot = sw.sourceSlots[source];
                    value.world = slot < 0;
                    if (slot >= 0) {
                        value.defaultSpace =
                            scratch->defaultRoundTrip[size_t(slot)];
                        value.posedSpace = _RrReadFrameVersion(
                            *program, *scratch, sw.sourceReads[source]);
                    }
                    input.sources.push_back(value);
                }
                RrMat4d output;
                if (!RigExecRunSpaceSwitchArithmetic<RrSpaceSwitchMath>(
                        record, input, &output)) {
                    if (error)
                        *error = _RrStepHead(program, step) + " names no space";
                    return false;
                }
                store.base[size_t(i)] = RrFrameFromMatrix(output);
                _RrRunSpaceMatch(program, *scratch, size_t(switchIndex),
                                 size_t(i), record, input,
                                 store.base[size_t(i)],
                                 &store.stepOutputs[step].diagnostics);
            }
            for (size_t acIndex = 0;
                 acIndex < program->poses->autoClavicles.size(); ++acIndex) {
                const auto &ac = program->poses->autoClavicles[acIndex];
                if (ac.slot != i)
                    continue;
                std::array<RrMat4d, 13> matrices;
                matrices[0] = _RrSpaceOfFrame(store.base[size_t(i)]);
                bool usable =
                    (store.base[size_t(i)].flags & RrPointFrameValid) &&
                    !(store.base[size_t(i)].flags & RrPointFrameDegenerate);
                for (size_t k = 1; k < 13; ++k) {
                    const auto &read = ac.frames[k];
                    if (read.slot < 0)
                        continue;
                    if (read.computation == 1)
                        matrices[k] =
                            scratch->defaultRoundTrip[size_t(read.slot)];
                    else if (read.computation == 2)
                        matrices[k] = _RrSpaceOfFrame(
                            scratch->restFrames[size_t(read.slot)]);
                    else if (!read.recompose.empty() || read.slot == i) {
                        matrices[k] = matrices[0];
                        for (int child : read.recompose) {
                            if (scratch->posedAuthored[size_t(child)])
                                matrices[k] =
                                    scratch->posedAuthoredM[size_t(child)];
                            else
                                matrices[k] = RrSpaceSwitchMath::RoundTrip(
                                    _RrComposeAvarsOf(*program, *scratch,
                                                      size_t(child)) *
                                    scratch->posedD[size_t(child)] *
                                    scratch->parentDinv[size_t(child)] *
                                    (scratch->parentSpaceAuthored[size_t(child)]
                                         ? scratch->parentSpaceM[size_t(child)]
                                         : matrices[k]));
                        }
                    } else
                        matrices[k] =
                            _RrSpaceOfFrame(store.fin[size_t(read.slot)]);
                    for (int row = 0; row < 4; ++row)
                        for (int col = 0; col < 4; ++col)
                            usable =
                                usable && std::isfinite(matrices[k][row][col]);
                }
                if (!usable)
                    continue;
                RigExecAutoClavicleConstants constants;
                std::copy(ac.basis.begin(), ac.basis.end(), constants.basis);
                constants.ikValue = ac.ikValue;
                constants.gain = ac.gain;
                constants.kernel = ac.kernel;
                constants.normalize = ac.normalize;
                constants.swings = ac.swings;
                constants.widths = ac.widths;
                constants.gains = ac.gains;
                constants.weights = ac.weights;
                const auto scalar = [&](size_t k, double fallback) {
                    const int index = ac.scalarIndices[k];
                    if (index < 0)
                        return fallback;
                    const auto value = program->ReadRegistered(
                        program->autoClavicleRead[acIndex][size_t(index)]);
                    return value.tag == RrInputTag::Float ? double(value.f32)
                                                          : value.f64;
                };
                RigExecAutoClavicleFrames frames;
                frames.targetPosed = &matrices[0]._mtx[0][0];
                frames.pivotPosed = &matrices[1]._mtx[0][0];
                frames.anchorPosed = &matrices[2]._mtx[0][0];
                frames.anchorDefault = &matrices[3]._mtx[0][0];
                frames.fkPosed = &matrices[4]._mtx[0][0];
                for (size_t k = 0; k < 3; ++k)
                    frames.fkDefault[k] = &matrices[5 + k]._mtx[0][0];
                if (ac.frames[8].slot >= 0)
                    frames.ikTargetPosed = &matrices[8]._mtx[0][0];
                if (ac.frames[9].slot >= 0)
                    frames.polePosed = &matrices[9]._mtx[0][0];
                frames.ikBlend = scalar(0, 1 - constants.ikValue);
                frames.amount = scalar(1, 1);
                frames.hasLimb = ac.hasLimb;
                if (frames.hasLimb) {
                    frames.limb.stretch = scalar(2, 1);
                    frames.limb.pin = scalar(3, 0);
                    frames.limb.upperScale = scalar(4, 1);
                    frames.limb.lowerScale = scalar(5, 1);
                    frames.limb.softDistance = scalar(6, 0);
                    frames.limb.scaleCalibration = scalar(7, 0);
                    frames.twistRadians = scalar(8, 0) * _RrPi / 180.0;
                    frames.limbRestUpper = (matrices[11].ExtractTranslation() -
                                            matrices[10].ExtractTranslation())
                                               .GetLength() +
                                           scalar(9, 0);
                    frames.limbRestLower = (matrices[12].ExtractTranslation() -
                                            matrices[11].ExtractTranslation())
                                               .GetLength() +
                                           scalar(10, 0);
                }
                double delta[3];
                RigExecAutoClavicleShift(constants, frames, delta);
                if (delta[0] != 0 || delta[1] != 0 || delta[2] != 0) {
                    auto shifted = matrices[0];
                    shifted.SetTranslateOnly(
                        shifted.ExtractTranslation() +
                        RrVec3d(delta[0], delta[1], delta[2]));
                    store.base[size_t(i)] = RrFrameFromMatrix(shifted);
                }
            }
            eval::PublishFinPosedM(host, size_t(i));
        }
        return true;
    }

    case RigExecWireStepKind::Solve: {
        return runtimePoseDetail::_RrRunSolveStep(program, step, error);
    }

    case RigExecWireStepKind::Constraint: {
        return runtimePoseDetail::_RrRunConstraintStep(program, step, error);
    }

    case RigExecWireStepKind::SpaceCheckpoint: {
        RrPoseHost host(program);
        eval::StepRef ref;
        ref.kind = uint32_t(wire.kind);
        ref.object = wire.object;
        ref.part = wire.part;
        ref.slot = step;
        return eval::RunValueStep(host, ref).outcome == eval::Outcome::Applied;
    }
    case RigExecWireStepKind::AvarInputs: {
        RrPoseHost host(program);
        eval::StepRef ref;
        ref.kind = uint32_t(wire.kind);
        ref.object = wire.object;
        ref.part = wire.part;
        ref.slot = step;
        return eval::RunValueStep(host, ref).outcome == eval::Outcome::Applied;
    }

    case RigExecWireStepKind::ProviderRefresh:
        return RrRunProviderRefresh(program, wire, error);

    case RigExecWireStepKind::SpaceExpression:
        return RrRunSpaceExpression(program, wire, error);

    case RigExecWireStepKind::PoseInterpolator: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->poseInterpolators.size() ||
            size_t(wire.object) >= scratch->interpEnabled.size() ||
            size_t(wire.object) >= scratch->interpSolvers.size() ||
            size_t(wire.object) >= scratch->interpScratch.size()) {
            if (error) {
                *error =
                    _RrStepHead(program, step) + " names no pose interpolator";
            }
            return false;
        }
        const RigExecWirePoseInterpolator &interp =
            program->poses->poseInterpolators[size_t(wire.object)];
        // Resolve current producer outputs in the consuming graph body.
        const size_t object = size_t(wire.object);
        scratch->interpEnabled[object] =
            program->ReadInterp(object).boolean ? 1 : 0;
        for (size_t v = 0; v < interp.valueInputs.size(); ++v) {
            const RrInputValue value = program->ReadInterpValue(object, v);
            scratch->interpValues[object][v] =
                value.tag == RrInputTag::Float ? double(value.f32) : value.f64;
        }
        if (interp.network) {
            scratch->interpMasks[object] =
                double(program->ReadInterpMask(object).f32);
        }
        for (int slot = interp.weightBegin; slot < interp.weightEnd; ++slot) {
            if (slot < 0 || size_t(slot) >= store.poseWeights.size()) {
                if (error) {
                    *error =
                        _RrStepHead(program, step) + " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(slot)] = 0.0f;
        }
        if (!scratch->interpEnabled[size_t(wire.object)] ||
            interp.poseSlots.empty()) {
            return true;
        }
        // A NUMERIC driver reads dials, not a frame: it names no driver
        // prim and takes no slot, and its rotation channel is the
        // identity, so every frame lookup below is skipped. An ML mover
        // may read both.
        const bool numeric =
            !interp.valueInputs.empty() && interp.driverSlot < 0;
        if (!numeric &&
            (interp.driverSlot < 0 ||
             size_t(interp.driverSlot) >= store.finLast.size() ||
             size_t(interp.driverSlot) >= scratch->restFrames.size() ||
             size_t(interp.driverSlot) >= program->slotMeta->paths.size() ||
             (interp.parentSlot >= 0 &&
              (size_t(interp.parentSlot) >= store.finLast.size() ||
               size_t(interp.parentSlot) >= scratch->restFrames.size())))) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no driver slot";
            }
            return false;
        }
        const size_t d = size_t(numeric ? 0 : interp.driverSlot);
        if (!numeric && size_t(store.finLast[d]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no fin version";
            }
            return false;
        }
        RigExecPoseInterpolatorInputsT<RrPointFrame> input;
        input.enabled = true;
        input.numeric = scratch->interpValues[object];
        input.mask = scratch->interpMasks[object];
        if (!numeric) {
            input.driverFinal = &store.fin[size_t(store.finLast[d])];
            input.driverRest = &scratch->restFrames[d];
            if (interp.parentSlot >= 0) {
                const size_t p = size_t(interp.parentSlot);
                if (size_t(store.finLast[p]) >= store.fin.size()) {
                    if (error)
                        *error = _RrStepHead(program, step) +
                                 " names no fin version";
                    return false;
                }
                input.parentFinal = &store.fin[size_t(store.finLast[p])];
                input.parentRest = &scratch->restFrames[p];
            }
            const size_t extras = interp.extraDriverSlots.size();
            if (extras != interp.extraParentSlots.size() ||
                (extras && !interp.method)) {
                if (error)
                    *error =
                        _RrStepHead(program, step) + " names no driver slot";
                return false;
            }
            input.extraDrivers.resize(extras);
            for (size_t e = 0; e < extras; ++e) {
                const int slots[2] = {interp.extraDriverSlots[e],
                                      interp.extraParentSlots[e]};
                const RrPointFrame *finals[2] = {nullptr, nullptr},
                                   *rests[2] = {nullptr, nullptr};
                for (int k = 0; k < 2; ++k) {
                    if (slots[k] < 0 && k == 1)
                        continue;
                    if (slots[k] < 0 ||
                        size_t(slots[k]) >= store.finLast.size() ||
                        size_t(slots[k]) >= scratch->restFrames.size() ||
                        size_t(store.finLast[size_t(slots[k])]) >=
                            store.fin.size()) {
                        if (error)
                            *error = _RrStepHead(program, step) +
                                     " names no driver slot";
                        return false;
                    }
                    finals[k] =
                        &store.fin[size_t(store.finLast[size_t(slots[k])])];
                    rests[k] = &scratch->restFrames[size_t(slots[k])];
                }
                input.extraDrivers[e] = {finals[0], rests[0], finals[1],
                                         rests[1]};
            }
        }
        auto &solver = scratch->interpSolvers[object];
        RigExecPoseInterpolatorRecordT<
            decltype(solver), const RigExecRbfMethodTable &,
            const RigExecMlNetwork *, const RigExecAnimatedMapsTable *>
            record{solver,
                   interp.enableTranslation,
                   interp.allowNegativeWeights,
                   interp.poseSlots.size(),
                   bool(interp.method),
                   scratch->interpMethods[object],
                   bool(interp.network),
                   &scratch->interpNetworks[object],
                   bool(interp.maps),
                   &scratch->interpMaps[object]};
        std::vector<double> &interpScratch = scratch->interpScratch[object];
        const auto status = RigExecRunPoseInterpolatorArithmetic<
            RrPoseInterpolatorMathT<RrPointFrame>>(record, input,
                                                   &interpScratch);
        if (status == RigExecPoseInterpolatorStatus::UnusableRotation) {
            output.diagnostics.push_back(
                "pose interpolator " + program->TextOrEmpty(interp.path) +
                " has no usable frame for its driver " +
                program->TextOrEmpty(program->slotMeta->paths[d]) +
                " after the pose walk; its weights are zero this generation");
            return true;
        }
        if (status == RigExecPoseInterpolatorStatus::UnusableTranslation) {
            output.diagnostics.push_back(
                "pose interpolator " + program->TextOrEmpty(interp.path) +
                " could not measure its driver's translation; its weights are "
                "zero this generation");
            return true;
        }
        if (interpScratch.size() != interp.poseSlots.size()) {
            output.diagnostics.push_back(
                "pose interpolator " + program->TextOrEmpty(interp.path) +
                " solved " + std::to_string(interpScratch.size()) +
                " weights for " + std::to_string(interp.poseSlots.size()) +
                " poses");
            return true;
        }
        for (size_t i = 0; i < interp.poseSlots.size(); ++i) {
            if (interp.poseSlots[i] < 0 ||
                size_t(interp.poseSlots[i]) >= store.poseWeights.size()) {
                if (error) {
                    *error =
                        _RrStepHead(program, step) + " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(interp.poseSlots[i])] =
                static_cast<float>(interpScratch[i]);
        }
        // Open validated these against the weight range and the poses.
        for (size_t j = 0; j < interp.primvarSlots.size(); ++j) {
            store.poseWeights[size_t(interp.primvarSlots[j])] =
                store.poseWeights[size_t(
                    interp.poseSlots[size_t(interp.primvarSources[j])])];
        }
        return true;
    }

    default:
        break;
    }

    if (error) {
        *error = _RrStepHead(program, step) + " not implemented yet";
    }
    return false;
}

bool
RrLookUpSpace(const RrProgram *program, const eval::OpLookup &lookup,
              const RigExecTypeDesc &type, void *out, std::string *error)
{
    const auto fail = [error](std::string why) {
        if (error)
            *error = std::move(why);
        return false;
    };
    const auto *scratch =
        static_cast<const RrPoseScratch *>(program->pose.get());
    if (lookup.source == uint8_t(eval::OpLookupSource::RestFrame)) {
        if (!scratch || type.size != sizeof(rigExecRig::PointFrame) ||
            lookup.object >= scratch->restFrames.size())
            return fail("slot " + std::to_string(lookup.object) +
                        " has no rest frame");
        const RrPointFrame &frame = scratch->restFrames[lookup.object];
        auto &value = *static_cast<rigExecRig::PointFrame *>(out);
        for (size_t i = 0; i < frame.points.size(); ++i)
            value.points[i] = rigExecStd::Vec3d(
                frame.points[i][0], frame.points[i][1], frame.points[i][2]);
        value.flags = frame.flags;
        return true;
    }
    const RrStore &store = program->store;
    if (!scratch ||
        lookup.source != uint8_t(eval::OpLookupSource::CommitRecordGate) ||
        lookup.object >= store.commits.size() ||
        lookup.object >= scratch->recordAfter.size() ||
        lookup.object >= scratch->recordEveryTarget.size() || lookup.read > 2)
        return fail("commit " + std::to_string(lookup.object) +
                    " holds no such record gate");
    if (lookup.read == 0) {
        if (!type.borrow)
            return fail("a commit's presence is an array");
        const std::vector<char> &present = store.commits[lookup.object].present;
        type.borrow(out, present.data(), present.size());
        return true;
    }
    if (type.size != sizeof(uint8_t))
        return fail("a commit's record gate is a Bool");
    const std::vector<char> &gates =
        lookup.read == 1 ? scratch->recordAfter : scratch->recordEveryTarget;
    *static_cast<uint8_t *>(out) = gates[lookup.object] != 0 ? 1 : 0;
    return true;
}

bool
RrStoreSpace(RrProgram *program, const eval::OpHostOutput &output,
             const void *value, std::string *error)
{
    using T = eval::OpOutputTarget;
    RrStore &store = program->store;
    const T target = T(output.target);
    if (target == T::FrameMatrixValid) {
        if (output.object >= store.frameMatrixValid.size()) {
            if (error)
                *error = "frame record " + std::to_string(output.object) +
                         " has no slot";
            return false;
        }
        store.frameMatrixValid[output.object] =
            *static_cast<const uint8_t *>(value) != 0;
        return true;
    }
    std::vector<RrMat4d> &matrices =
        target == T::ProviderFinalMatrix  ? store.finalMatrix
        : target == T::ProviderBaseMatrix ? store.baseMatrix
                                          : store.frameMatrix;
    if (output.object >= matrices.size()) {
        if (error)
            *error = "no matrix " + std::to_string(output.object) + " to store";
        return false;
    }
    matrices[output.object] = *static_cast<const RrMat4d *>(value);
    return true;
}

} // namespace rigExec
