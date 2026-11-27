// Runtime pose-step dispatch, commit deltas, and propagation.

#include "poseInternal.h"
#include "poseEvalHost.h"
#include "spaces.h"
#include "rigExecRuntime/spaceSwitchMath.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"
#include <algorithm>
#include <cstring>
#include <variant>
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
    case RigExecWireStepKind::ReservedSolve:
        return "ReservedSolve";
    case RigExecWireStepKind::ReservedSolverCommit:
        return "ReservedSolverCommit";
    case RigExecWireStepKind::ReservedCommitDelta:
        return "ReservedCommitDelta";
    case RigExecWireStepKind::ReservedPropagateChunk:
        return "ReservedPropagateChunk";
    case RigExecWireStepKind::ReservedCommitApply:
        return "ReservedCommitApply";
    case RigExecWireStepKind::SnapshotFinals:
        return "SnapshotFinals";
    case RigExecWireStepKind::ReservedPoseInterpolator:
        return "ReservedPoseInterpolator";
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
                if (size_t(switchIndex) >= scratch->switchInputs.size() ||
                    scratch->switchInputs[size_t(switchIndex)].sources.size() !=
                        sw.sourceSlots.size()) {
                    return noSlot();
                }
                RigExecSpaceSwitchInputsT<RrMat4d> &input =
                    scratch->switchInputs[size_t(switchIndex)];
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
                    RigExecSpaceSwitchSourceT<RrMat4d> &value =
                        input.sources[source];
                    const int slot = sw.sourceSlots[source];
                    value.world = slot < 0;
                    if (slot >= 0) {
                        value.defaultSpace =
                            scratch->defaultRoundTrip[size_t(slot)];
                        value.posedSpace = _RrReadFrameVersion(
                            *program, *scratch, sw.sourceReads[source]);
                    }
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
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderInputs))
        return RrFillProviderInputs(program, lookup, type, out, error);
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderCandidates))
        return RrProviderCandidates(program, lookup, type, out, error);
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
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderFrame)) {
        const auto &refreshes = program->poses->providerRefreshes;
        const uint64_t id =
            lookup.object >= refreshes.size() ? UINT64_MAX
            : lookup.read ? uint64_t(refreshes[lookup.object].currentValue)
                          : uint64_t(refreshes[lookup.object].baseValue);
        const auto *value = id < store.providerValues.size() &&
                                    store.providerValues[id].initialized &&
                                    !store.providerValues[id].blocked
                                ? std::get_if<RigExecProviderPlainFrame>(
                                      &store.providerValues[id].value)
                                : nullptr;
        if (!value || type.size != sizeof(RrPointFrame))
            return fail(lookup.read ? "connected final pose input incomplete"
                                    : "connected base pose input incomplete");
        auto &frame = *static_cast<RrPointFrame *>(out);
        for (size_t p = 0; p < 4; ++p)
            for (size_t a = 0; a < 3; ++a)
                frame.points[p][a] = value->points[p][a];
        frame.flags = value->flags;
        return true;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderParentRaw)) {
        if (type.size != sizeof(RrMat4d) ||
            lookup.object >= program->ladderRead.size())
            return fail("slot " + std::to_string(lookup.object) +
                        " has no parent space");
        const RrWireValue raw =
            RrReadProviderRefreshBlocker(program, int(lookup.object));
        std::memcpy(out, raw.matrix.data(), sizeof(RrMat4d));
        return true;
    }
    if (!scratch ||
        lookup.source != uint8_t(eval::OpLookupSource::CommitRecordGate) ||
        lookup.object >= store.commits.size() ||
        lookup.object >= scratch->recordAfter.size() ||
        lookup.object >= scratch->recordEveryTarget.size() || lookup.read > 3)
        return fail("commit " + std::to_string(lookup.object) +
                    " holds no such record gate");
    if (lookup.read == 3) {
        const auto &commit = store.commits[lookup.object];
        if (type.size != sizeof(uint8_t) || lookup.part < 0 ||
            size_t(lookup.part) >= commit.present.size())
            return fail("commit " + std::to_string(lookup.object) +
                        " has no output " + std::to_string(lookup.part));
        *static_cast<uint8_t *>(out) =
            !commit.abandoned && commit.present[size_t(lookup.part)] ? 1 : 0;
        return true;
    }
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

namespace {

template <class T, class At>
bool
_RrFillList(const RigExecTypeDesc &type, void *value, size_t count, At &&at)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, count);
    T *out = static_cast<T *>(type.data(value));
    for (size_t k = 0; k < count; ++k)
        out[k] = at(k);
    return true;
}

bool
_RrFillMatrices(const RigExecTypeDesc &type, void *value,
                const std::vector<int32_t> &slots,
                const std::vector<RrMat4d> &table)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, slots.size());
    auto *out = static_cast<double *>(type.data(value));
    for (size_t k = 0; k < slots.size(); ++k)
        std::memcpy(out + 16 * k, table[size_t(slots[k])]._mtx,
                    16 * sizeof(double));
    return true;
}

// Input \p read of space checkpoint \p object's SpaceCheckpoint node, from
// the slots it recomposes (RigExecBakedLookUpCompose's twin).
bool
_RrLookUpCheckpoint(const RrProgram *program, uint32_t object,
                    eval::OpCheckpointRead read, const RigExecTypeDesc &type,
                    void *value)
{
    using R = eval::OpCheckpointRead;
    const auto &context = program->poses->spaceCheckpoints[object];
    const std::vector<int32_t> &slots = context.recompose;
    const RrStore &store = program->store;
    const RrPoseScratch &scratch =
        *_RrScratch(const_cast<RrProgram *>(program));
    const auto flags = [&](const std::vector<char> &table) {
        return _RrFillList<uint8_t>(type, value, slots.size(), [&](size_t k) {
            return uint8_t(table[size_t(slots[k])] != 0);
        });
    };
    switch (read) {
    case R::Avars:
        return _RrFillList<double>(
            type, value, slots.size() * kRigExecSlotAvars, [&](size_t k) {
                return store.avars[size_t(slots[k / kRigExecSlotAvars]) *
                                       kRigExecSlotAvars +
                                   k % kRigExecSlotAvars];
            });
    case R::RotOrders:
        return _RrFillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const std::string order =
                program->TextOrEmpty(scratch.rotOrder[size_t(slots[k])]);
            return RigExecRotOrderCode(order.data(), order.size());
        });
    case R::RotationSigns:
        return _RrFillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const size_t s = size_t(slots[k]);
            return s < scratch.rotationSign.size()
                       ? uint32_t(scratch.rotationSign[s])
                       : 0u;
        });
    case R::PosedDefault:
        return _RrFillMatrices(type, value, slots, scratch.posedD);
    case R::ParentDefaultInverse:
        return _RrFillMatrices(type, value, slots, scratch.parentDinv);
    case R::ParentExpression:
        return _RrFillMatrices(type, value, slots, scratch.parentSpaceM);
    case R::ParentExpressionAuthored:
        return flags(scratch.parentSpaceAuthored);
    case R::PosedAuthored:
        return flags(scratch.posedAuthored);
    case R::PosedAuthoredMatrix:
        return _RrFillMatrices(type, value, slots, scratch.posedAuthoredM);
    case R::Anchor:
        if (context.anchor < 0 || type.size != 16 * sizeof(double))
            return false;
        std::memcpy(value, store.posedM[size_t(context.anchor)]._mtx,
                    16 * sizeof(double));
        return true;
    }
    return false;
}

// Input \p read of pose interpolator \p object's PoseInterpolator node
// (RigExecBakedLookUpCompose's twin).
bool
_RrLookUpInterpolator(const RrProgram *program, uint32_t object,
                      eval::OpInterpolatorRead read,
                      const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpInterpolatorRead;
    using Frame = rigExecRig::PointFrame;
    static_assert(sizeof(Frame) == sizeof(RrPointFrame),
                  "op and runtime frames share a layout");
    const auto &interp = program->poses->poseInterpolators[object];
    const RrStore &store = program->store;
    const RrPoseScratch &scratch =
        *_RrScratch(const_cast<RrProgram *>(program));
    const auto put = [&](int slot, bool final, Frame *out) {
        if (slot < 0 || size_t(slot) >= store.finLast.size() ||
            size_t(store.finLast[size_t(slot)]) >= store.fin.size() ||
            size_t(slot) >= scratch.restFrames.size())
            return false;
        std::memcpy(out,
                    final ? &store.fin[size_t(store.finLast[size_t(slot)])]
                          : &scratch.restFrames[size_t(slot)],
                    sizeof(Frame));
        return true;
    };
    const auto frame = [&](int slot, bool final) {
        return type.size == sizeof(Frame) &&
               put(slot, final, static_cast<Frame *>(value));
    };
    const auto frames = [&](bool parents, bool final) {
        if (!type.resize || !type.data)
            return false;
        size_t count = 0;
        for (size_t e = 0; e < interp.extraDriverSlots.size(); ++e)
            count += !parents || interp.extraParentSlots[e] >= 0;
        type.resize(value, count);
        Frame *out = static_cast<Frame *>(type.data(value));
        for (size_t e = 0, k = 0; e < interp.extraDriverSlots.size(); ++e) {
            const int slot = parents ? interp.extraParentSlots[e]
                                     : interp.extraDriverSlots[e];
            if (slot >= 0 && !put(slot, final, &out[k++]))
                return false;
        }
        return true;
    };
    switch (read) {
    case R::Enabled:
        if (type.size != sizeof(uint8_t))
            return false;
        *static_cast<uint8_t *>(value) =
            program->ReadInterp(object).boolean ? 1 : 0;
        return true;
    case R::Numeric:
        return _RrFillList<double>(
            type, value, interp.valueInputs.size(), [&](size_t v) {
                const RrInputValue read = program->ReadInterpValue(object, v);
                return read.tag == RrInputTag::Float ? double(read.f32)
                                                     : read.f64;
            });
    case R::Mask:
        if (type.size != sizeof(double))
            return false;
        *static_cast<double *>(value) =
            double(program->ReadInterpMask(object).f32);
        return true;
    case R::DriverFinal:
        return frame(interp.driverSlot, true);
    case R::DriverRest:
        return frame(interp.driverSlot, false);
    case R::ParentFinal:
        return frame(interp.parentSlot, true);
    case R::ParentRest:
        return frame(interp.parentSlot, false);
    case R::ExtraFinals:
        return frames(false, true);
    case R::ExtraRests:
        return frames(false, false);
    case R::ExtraParentFinals:
        return frames(true, true);
    case R::ExtraParentRests:
        return frames(true, false);
    }
    return false;
}

bool
_RrPutMatrix(const RigExecTypeDesc &type, void *value, const RrMat4d &matrix)
{
    if (type.size != 16 * sizeof(double))
        return false;
    std::memcpy(value, matrix._mtx, 16 * sizeof(double));
    return true;
}

// Input \p read of compose group \p object's RestCompose node
// (RigExecBakedLookUpCompose's twin).
bool
_RrLookUpRest(const RrProgram *program, uint32_t object, eval::OpRestRead read,
              const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpRestRead;
    const RrPoseInputs inputs(program);
    const size_t slot = size_t(program->poses->composeGroups[object].begin);
    switch (read) {
    case R::RestAvars:
        return _RrFillList<double>(
            type, value, 6, [&](size_t k) { return inputs.restAvar(slot, k); });
    case R::RestSpace:
        return _RrPutMatrix(
            type, value, inputs.ladderSpace(slot, eval::kPoseLadderRestSpace));
    case R::Intervening:
        return _RrPutMatrix(type, value, inputs.interveningSpace(slot));
    case R::ParentRest: {
        const int parent = inputs.parent(slot);
        RrMat4d rest;
        return parent >= 0 && inputs.restRoundTrip(size_t(parent), &rest) &&
               _RrPutMatrix(type, value, rest);
    }
    }
    return false;
}

// Output \p port of compose group \p object's RestCompose node into its
// slot's rest tables (RigExecBakedStoreCompose's twin).
bool
_RrStoreRest(RrProgram *program, uint32_t object, uint32_t port,
             const void *value)
{
    using P = eval::OpRestOutput;
    RrPoseScratch &scratch = *_RrScratch(program);
    const size_t slot = size_t(program->poses->composeGroups[object].begin);
    switch (P(port)) {
    case P::RestMatrix: {
        RrMat4d &restM = scratch.restM[slot];
        std::memcpy(restM._mtx, value, 16 * sizeof(double));
        if (std::memcmp(restM._mtx, scratch.lastRestM[slot]._mtx,
                        16 * sizeof(double)) != 0) {
            scratch.lastRestM[slot] = restM;
            program->store.restChanged[slot] = 1;
        }
        return true;
    }
    case P::Rest: {
        const auto &frame = *static_cast<const rigExecRig::PointFrame *>(value);
        scratch.restPts[slot] = frame.points;
        scratch.restFrames[slot] =
            RrPoseMath::MakeFrame(frame.points, frame.flags);
        return true;
    }
    case P::RoundTrip:
        std::memcpy(scratch.restRoundTrip[slot]._mtx, value,
                    16 * sizeof(double));
        return true;
    }
    return false;
}

} // namespace

bool
RrLookUpCompose(const RrProgram *program, const eval::OpLookup &lookup,
                const RigExecTypeDesc &type, void *value, std::string *error)
{
    const RrInputState &state = program->inputState;
    if (lookup.source == uint8_t(eval::OpLookupSource::RestInputs)) {
        if (lookup.object < program->poses->composeGroups.size() &&
            _RrLookUpRest(program, lookup.object, eval::OpRestRead(lookup.read),
                          type, value))
            return true;
        if (error)
            *error = "op nodes: compose group " +
                     std::to_string(lookup.object) + " has no rest input " +
                     std::to_string(lookup.read);
        return false;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::InterpolatorInputs)) {
        if (lookup.object < program->poses->poseInterpolators.size() &&
            _RrLookUpInterpolator(program, lookup.object,
                                  eval::OpInterpolatorRead(lookup.read), type,
                                  value))
            return true;
        if (error)
            *error = "op nodes: pose interpolator " +
                     std::to_string(lookup.object) + " has no input " +
                     std::to_string(lookup.read);
        return false;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::CheckpointInputs)) {
        if (lookup.object < program->poses->spaceCheckpoints.size() &&
            _RrLookUpCheckpoint(program, lookup.object,
                                eval::OpCheckpointRead(lookup.read), type,
                                value))
            return true;
        if (error)
            *error = "op nodes: space checkpoint " +
                     std::to_string(lookup.object) + " has no input " +
                     std::to_string(lookup.read);
        return false;
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::AvarBindings) ||
        !type.resize || !type.data) {
        if (error)
            *error = "op nodes: no avar bindings of slot " +
                     std::to_string(lookup.object);
        return false;
    }
    const auto range = RrAvarReadRange(state, int64_t(lookup.object));
    type.resize(value, size_t(range.second - range.first));
    auto *out = static_cast<double *>(type.data(value));
    for (uint32_t i = range.first; i < range.second; ++i)
        out[i - range.first] =
            program->ReadRegistered(int32_t(state.avarReads[i])).f64;
    return true;
}

bool
RrStoreCompose(RrProgram *program, const eval::OpHostOutput &output,
               const void *value, std::string *error)
{
    if (output.target == uint8_t(eval::OpOutputTarget::Rest)) {
        if (output.object < program->poses->composeGroups.size() &&
            _RrStoreRest(program, output.object, output.port, value))
            return true;
        if (error)
            *error = "op nodes: compose group " +
                     std::to_string(output.object) + " stores no rest port " +
                     std::to_string(output.port);
        return false;
    }
    if (output.target == uint8_t(eval::OpOutputTarget::PoseWeights)) {
        const auto &weights =
            *static_cast<const rigExecStd::RigExecArray<float> *>(value);
        const auto &interps = program->poses->poseInterpolators;
        std::vector<float> &poseWeights = program->store.poseWeights;
        if (output.object >= interps.size() ||
            weights.Size() != interps[output.object].poseSlots.size()) {
            if (error)
                *error = "op nodes: pose interpolator " +
                         std::to_string(output.object) + " stores no " +
                         std::to_string(weights.Size()) + " weights";
            return false;
        }
        const auto &interp = interps[output.object];
        // Open validated the pose and primvar slots against the range.
        for (int slot = interp.weightBegin; slot < interp.weightEnd; ++slot)
            poseWeights[size_t(slot)] = 0.0f;
        for (size_t i = 0; i < interp.poseSlots.size(); ++i)
            poseWeights[size_t(interp.poseSlots[i])] = weights[i];
        for (size_t j = 0; j < interp.primvarSlots.size(); ++j)
            poseWeights[size_t(interp.primvarSlots[j])] = poseWeights[size_t(
                interp.poseSlots[size_t(interp.primvarSources[j])])];
        return true;
    }
    if (output.target == uint8_t(eval::OpOutputTarget::SwitchFrame)) {
        if (output.object >= program->store.switchFrames.size()) {
            if (error)
                *error = "op nodes: no space checkpoint " +
                         std::to_string(output.object);
            return false;
        }
        std::memcpy(program->store.switchFrames[output.object]._mtx, value,
                    16 * sizeof(double));
        return true;
    }
    const RrInputState &state = program->inputState;
    const auto &values =
        *static_cast<const rigExecStd::RigExecArray<double> *>(value);
    const auto range = RrAvarReadRange(state, int64_t(output.object));
    const unsigned channels = RigExecBoundAvarChannels(
        size_t(range.second - range.first), [&](size_t k) {
            return program->registeredReads[state.avarReads[range.first + k]]
                .avar;
        });
    std::vector<double> &avars = program->store.avars;
    const size_t first = size_t(output.object) * kRigExecSlotAvars;
    if (output.target != uint8_t(eval::OpOutputTarget::Avars) ||
        first + kRigExecSlotAvars > avars.size() ||
        values.Size() != RigExecAvarElement(channels, kRigExecSlotAvars)) {
        if (error)
            *error = "op nodes: slot " + std::to_string(output.object) +
                     " stores no " + std::to_string(values.Size()) + " avars";
        return false;
    }
    size_t element = 0;
    for (size_t c = 0; c < kRigExecSlotAvars; ++c)
        if ((channels >> c) & 1u)
            avars[first + c] = values[element++];
    return true;
}

int64_t
RrProviderOpOf(const RrProgram *program, const RigExecWireStep &step)
{
    return step.kind == RigExecWireStepKind::OpNode && step.object >= 0 &&
                   size_t(step.object) < program->providerOpOfNode.size()
               ? program->providerOpOfNode[size_t(step.object)]
               : -1;
}

int64_t
RrProviderAnswerOf(const RrProgram *program, const RigExecWireStep &step)
{
    return step.kind == RigExecWireStepKind::OpNode && step.object >= 0 &&
                   size_t(step.object) < program->providerAnswerOfNode.size()
               ? program->providerAnswerOfNode[size_t(step.object)]
               : -1;
}

int64_t
RrRefreshOf(const RrProgram *program, const RigExecWireStep &step)
{
    return step.kind == RigExecWireStepKind::OpNode && step.object >= 0 &&
                   size_t(step.object) < program->refreshOfNode.size()
               ? program->refreshOfNode[size_t(step.object)]
               : -1;
}

bool
RrStoreSpace(RrProgram *program, size_t step, const eval::OpHostOutput &output,
             const void *value, std::string *error)
{
    using T = eval::OpOutputTarget;
    RrStore &store = program->store;
    const T target = T(output.target);
    if (target >= T::ProviderAnswer && target <= T::ProviderAnswerBlocked)
        return RrStoreProviderAnswer(program, step, output, value, error);
    if (target == T::ProviderValueMatrix || target == T::ProviderValueFrame ||
        target == T::ProviderValueAuthority ||
        target == T::ProviderValueUnavailable)
        return RrStoreProviderValue(program, output, value, error);
    if (target >= T::RefreshBase) {
        if (output.object >= program->poses->providerRefreshes.size()) {
            if (error)
                *error = "no provider refresh " + std::to_string(output.object);
            return false;
        }
        const auto &refresh = program->poses->providerRefreshes[output.object];
        if (target == T::RefreshBase || target == T::RefreshFinal) {
            (target == T::RefreshBase ? store.base[refresh.baseWrite]
                                      : store.fin[refresh.finWrite]) =
                *static_cast<const RrPointFrame *>(value);
            return true;
        }
        const auto &frames = *static_cast<
            const rigExecStd::RigExecArray<rigExecRig::PointFrame> *>(value);
        if (frames.Size() != refresh.carries.size()) {
            if (error)
                *error = "provider refresh " + std::to_string(output.object) +
                         " carries " + std::to_string(refresh.carries.size()) +
                         " descendants";
            return false;
        }
        static_assert(sizeof(rigExecRig::PointFrame) == sizeof(RrPointFrame),
                      "a rig frame is a runtime frame");
        for (size_t k = 0; k < frames.Size(); ++k) {
            const auto &carry = refresh.carries[k];
            std::memcpy(target == T::RefreshCarryBase
                            ? &store.base[carry.baseWrite]
                            : &store.fin[carry.finWrite],
                        &frames[k], sizeof(RrPointFrame));
        }
        return true;
    }
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
