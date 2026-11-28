// Runtime pose-step dispatch, commit deltas, and propagation.

#include "poseInternal.h"
#include "poseEvalHost.h"
#include "spaces.h"
#include "rigExecRuntime/spaceSwitchMath.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecEvalCore/constraintLookups.h"
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

} // namespace

namespace runtimePoseDetail {

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

} // namespace runtimePoseDetail

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

// Input \p read of auto clavicle \p object's node: a frame outside its
// subtree or a scalar (LookUpClavicle's twin, bakedComposeOps.cpp).
bool
_RrLookUpClavicle(const RrProgram *program, uint32_t object, uint8_t read,
                  const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpClavicleRead;
    const auto &ac = program->poses->autoClavicles[object];
    if (read >= uint8_t(R::ScalarBegin)) {
        const size_t k = size_t(read - uint8_t(R::ScalarBegin));
        if (k >= ac.scalarIndices.size() || type.size != sizeof(double))
            return false;
        const int j = ac.scalarIndices[k];
        double scalar = k == 0 ? 1.0 - ac.ikValue : 1.0;
        if (j >= 0) {
            if (object >= program->autoClavicleRead.size() ||
                size_t(j) >= program->autoClavicleRead[object].size())
                return false;
            const RrInputValue v = program->ReadRegistered(
                program->autoClavicleRead[object][size_t(j)]);
            scalar = v.tag == RrInputTag::Float ? double(v.f32) : v.f64;
        } else if (k > 1) {
            return false;
        }
        *static_cast<double *>(value) = scalar;
        return true;
    }
    const RrStore &store = program->store;
    const RrPoseScratch &scratch =
        *_RrScratch(const_cast<RrProgram *>(program));
    const auto matrix = [&](size_t k, double *out) {
        if (k >= ac.frames.size() || ac.frames[k].slot < 0)
            return false;
        const auto &frame = ac.frames[k];
        const size_t slot = size_t(frame.slot);
        RrMat4d m;
        if (frame.computation == 1) {
            if (slot >= scratch.defaultRoundTrip.size())
                return false;
            m = scratch.defaultRoundTrip[slot];
        } else {
            const std::vector<RrPointFrame> &table =
                frame.computation == 2 ? scratch.restFrames : store.fin;
            if (slot >= table.size())
                return false;
            m = eval::pose_ops::SpaceOfFrame<RrPoseMath>(table[slot].points,
                                                         table[slot].flags);
        }
        std::memcpy(out, &m[0][0], 16 * sizeof(double));
        return true;
    };
    if (read == uint8_t(R::FkDefaults) || read == uint8_t(R::LimbRests)) {
        if (!type.resize || !type.data)
            return false;
        type.resize(value, 3);
        auto *out = static_cast<double *>(type.data(value));
        for (size_t e = 0; e < 3; ++e)
            if (!matrix(size_t(read) + e, out + 16 * e))
                return false;
        return true;
    }
    return type.size == 16 * sizeof(double) &&
           matrix(size_t(read), static_cast<double *>(value));
}

// A compose group node's output into its slot's first pose and posed
// matrix, or its switch's match values and status (StoreComposeGroup's
// twin, bakedComposeOps.cpp).
bool
_RrStoreComposeGroup(RrProgram *program, size_t step,
                     const eval::OpHostOutput &output, const void *value,
                     std::string *error)
{
    using T = eval::OpOutputTarget;
    RrStore &store = program->store;
    const T target = T(output.target);
    const auto fail = [&](const std::string &why) {
        if (error)
            *error = "op nodes: " + why;
        return false;
    };
    if (target == T::SpaceMatchValues || target == T::SpaceMatchStatus) {
        const size_t sw = output.object;
        if (sw >= store.spaceMatchValid.size() ||
            sw >= store.spaceMatchValues.size() ||
            sw >= program->poses->spaceSwitches.size())
            return fail("no space switch " + std::to_string(sw));
        if (target == T::SpaceMatchValues) {
            const auto &values =
                *static_cast<const rigExecStd::RigExecArray<double> *>(value);
            store.spaceMatchValues[sw].assign(values.Data(),
                                              values.Data() + values.Size());
            return true;
        }
        const int slot = program->poses->spaceSwitches[sw].slot;
        const RrPoseScratch &scratch = *_RrScratch(program);
        const bool authored = slot >= 0 &&
                              size_t(slot) < scratch.posedAuthored.size() &&
                              scratch.posedAuthored[size_t(slot)];
        const uint32_t status = *static_cast<const uint32_t *>(value);
        store.spaceMatchValid[sw] =
            status == uint32_t(RigExecSpaceMatchStatus::Matched) && !authored
                ? 1
                : 0;
        if (status == uint32_t(RigExecSpaceMatchStatus::NotFinite)) {
            if (step >= store.stepOutputs.size() || slot < 0 ||
                size_t(slot) >= program->slotMeta->paths.size())
                return fail("no step to report space switch " +
                            std::to_string(sw));
            store.stepOutputs[step].diagnostics.push_back(
                program->TextOrEmpty(program->slotMeta->paths[size_t(slot)]) +
                kRigExecSpaceMatchNotFinite);
        }
        return true;
    }
    const size_t slot = output.object;
    if (slot >= store.base.size() || slot >= store.fin.size() ||
        slot >= store.posedM.size())
        return fail("no slot " + std::to_string(slot) + " to compose");
    if (target == T::ComposePosed) {
        std::memcpy(&store.posedM[slot][0][0], value, 16 * sizeof(double));
        return true;
    }
    static_assert(sizeof(rigExecRig::PointFrame) == sizeof(RrPointFrame),
                  "a rig frame is a runtime frame");
    std::memcpy(&store.base[slot], value, sizeof(RrPointFrame));
    store.fin[slot] = store.base[slot];
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

// Input \p read of compose group \p object's LadderCompose node
// (RigExecBakedLookUpCompose's twin).
bool
_RrLookUpLadder(const RrProgram *program, uint32_t object,
                eval::OpLadderRead read, const RigExecTypeDesc &type,
                void *value)
{
    using R = eval::OpLadderRead;
    const RrPoseInputs inputs(program);
    const size_t slot = size_t(program->poses->composeGroups[object].begin);
    const int parent = inputs.parent(slot);
    RrMat4d matrix;
    switch (read) {
    case R::Spaces: {
        if (!type.resize || !type.data)
            return false;
        type.resize(value, 6);
        auto *out = static_cast<double *>(type.data(value));
        for (int c = 0; c < 6; ++c)
            std::memcpy(out + 16 * c, inputs.ladderSpace(slot, c + 1)._mtx,
                        16 * sizeof(double));
        return true;
    }
    case R::Authored:
        return _RrFillList<uint8_t>(type, value, 6, [&](size_t k) {
            const int channel = int(k) + 1;
            const bool defaultChannel =
                channel == eval::kPoseLadderDefaultSpace ||
                channel == eval::kPoseLadderParentDefaultSpace ||
                channel == eval::kPoseLadderAvarDefaultSpace ||
                channel == eval::kPoseLadderPosedDefaultSpace;
            return uint8_t(
                (defaultChannel && inputs.ladderChannelReady(slot, channel)) ||
                inputs.ladderAuthoritative(slot, channel));
        });
    case R::ParentSpaceLeaf:
        return _RrPutMatrix(type, value, inputs.ladderParentSpaceLeaf(slot));
    case R::RotationSign: {
        if (type.size != 3 * sizeof(double))
            return false;
        const RrVec3d sign = inputs.ladderRotationSign(slot);
        const double v[3] = {sign[0], sign[1], sign[2]};
        std::memcpy(value, v, sizeof(v));
        return true;
    }
    case R::DefaultAvars:
        return _RrFillList<double>(type, value, 6, [&](size_t k) {
            return inputs.defaultAvar(slot, k);
        });
    case R::OwnRest:
        return inputs.restRoundTrip(slot, &matrix) &&
               _RrPutMatrix(type, value, matrix);
    case R::ParentRest:
        return parent >= 0 && inputs.restRoundTrip(size_t(parent), &matrix) &&
               _RrPutMatrix(type, value, matrix);
    case R::ParentDefault:
        return parent >= 0 &&
               inputs.defaultRoundTrip(size_t(parent), &matrix) &&
               _RrPutMatrix(type, value, matrix);
    }
    return false;
}

// Output \p port of compose group \p object's LadderCompose node into its
// slot's ladder tables (RigExecBakedStoreCompose's twin). The last port
// takes the rotation order, publishes the live ladder echo and marks a
// moved ladder.
bool
_RrStoreLadder(RrProgram *program, uint32_t object, uint32_t port,
               const void *value)
{
    using P = eval::OpLadderOutput;
    RrPoseScratch &s = *_RrScratch(program);
    const size_t slot = size_t(program->poses->composeGroups[object].begin);
    const auto matrix = [&](std::vector<RrMat4d> &table) {
        std::memcpy(table[slot]._mtx, value, 16 * sizeof(double));
        return true;
    };
    const auto flag = [&](std::vector<char> &table) {
        table[slot] = *static_cast<const uint8_t *>(value) ? 1 : 0;
        return true;
    };
    switch (P(port)) {
    case P::SelfDefault:
        return matrix(s.selfD);
    case P::ParentDefaultInverse:
        return matrix(s.parentDinv);
    case P::DefaultRoundTrip:
        return matrix(s.defaultRoundTrip);
    case P::PosedDefault:
        return matrix(s.posedD);
    case P::ParentSpace:
        return matrix(s.parentSpaceM);
    case P::PosedAuthoredMatrix:
        return matrix(s.posedAuthoredM);
    case P::PosedAuthored:
        return flag(s.posedAuthored);
    case P::ParentSpaceAuthored:
        return flag(s.parentSpaceAuthored);
    case P::RotationSign:
        break;
    default:
        return false;
    }
    s.rotationSign[slot] =
        static_cast<unsigned char>(*static_cast<const uint32_t *>(value));
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    if (!meta.providerActive.empty() && !meta.providerActive[slot]) {
        s.rotOrder[slot] = 0;
        return true;
    }
    s.rotOrder[slot] = program->ReadLadder(slot, RrLadderRotationOrder).token;
    RrStore &store = program->store;
    if (slot < store.ladders.size()) {
        RrLadderLive &live = store.ladders[slot];
        live.restSpace = program->ReadLadder(slot, RrLadderRestSpace).matrix;
        live.defaultSpace =
            program->ReadLadder(slot, RrLadderDefaultSpace).matrix;
        live.posedSpace = program->ReadLadder(slot, RrLadderPosedSpace).matrix;
        for (int k = 0; k < 6; ++k) {
            live.restAvars[size_t(k)] =
                program->ReadLadder(slot, RrLadderRestAvar0 + k).f64;
            live.defaultAvars[size_t(k)] =
                program->ReadLadder(slot, RrLadderDefaultAvar0 + k).f64;
        }
        live.rotationOrder = s.rotOrder[slot];
    }
    const auto same = [](const RrMat4d &a, const RrMat4d &b) {
        return std::memcmp(a._mtx, b._mtx, 16 * sizeof(double)) == 0;
    };
    if (same(s.selfD[slot], s.lastSelfD[slot]) &&
        same(s.parentDinv[slot], s.lastParentDinv[slot]) &&
        s.posedAuthored[slot] == s.lastPosedAuthored[slot] &&
        same(s.posedAuthoredM[slot], s.lastPosedAuthoredM[slot]) &&
        s.rotOrder[slot] == s.lastRotOrder[slot] &&
        same(s.posedD[slot], s.lastPosedD[slot]) &&
        same(s.parentSpaceM[slot], s.lastParentSpaceM[slot]) &&
        s.parentSpaceAuthored[slot] == s.lastParentSpaceAuthored[slot] &&
        s.rotationSign[slot] == s.lastRotationSign[slot])
        return true;
    s.lastSelfD[slot] = s.selfD[slot];
    s.lastParentDinv[slot] = s.parentDinv[slot];
    s.lastPosedAuthored[slot] = s.posedAuthored[slot];
    s.lastPosedAuthoredM[slot] = s.posedAuthoredM[slot];
    s.lastRotOrder[slot] = s.rotOrder[slot];
    s.lastPosedD[slot] = s.posedD[slot];
    s.lastParentSpaceM[slot] = s.parentSpaceM[slot];
    s.lastParentSpaceAuthored[slot] = s.parentSpaceAuthored[slot];
    s.lastRotationSign[slot] = s.rotationSign[slot];
    store.ladderChanged[slot] = 1;
    return true;
}

} // namespace

bool
RrLookUpCompose(const RrProgram *program, const eval::OpLookup &lookup,
                const RigExecTypeDesc &type, void *value, std::string *error)
{
    using L = eval::OpLookupSource;
    const auto missing = [&](const char *what) {
        if (error)
            *error = "op nodes: " + std::string(what) + " " +
                     std::to_string(lookup.object) + " has no input " +
                     std::to_string(lookup.read);
        return false;
    };
    if (lookup.source == uint8_t(L::ComposeInputs))
        return eval::compose_lookups::LookUpComposeInput<RrPoseMath>(
                   RrPoseInputs(program), lookup.object, lookup.read, type,
                   value) ||
               missing("slot");
    if (lookup.source == uint8_t(L::SwitchInputs))
        return (lookup.object < program->poses->spaceSwitches.size() &&
                eval::compose_lookups::LookUpSwitchInput<RrPoseMath>(
                    RrPoseInputs(program), lookup.object, lookup.read, type,
                    value)) ||
               missing("space switch");
    if (lookup.source == uint8_t(L::ClavicleInputs))
        return (lookup.object < program->poses->autoClavicles.size() &&
                _RrLookUpClavicle(program, lookup.object, lookup.read, type,
                                  value)) ||
               missing("auto clavicle");
    const RrInputState &state = program->inputState;
    if (lookup.source == uint8_t(eval::OpLookupSource::LadderInputs)) {
        if (lookup.object < program->poses->composeGroups.size() &&
            _RrLookUpLadder(program, lookup.object,
                            eval::OpLadderRead(lookup.read), type, value))
            return true;
        if (error)
            *error = "op nodes: compose group " +
                     std::to_string(lookup.object) + " has no ladder input " +
                     std::to_string(lookup.read);
        return false;
    }
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
RrStoreCompose(RrProgram *program, size_t step,
               const eval::OpHostOutput &output, const void *value,
               std::string *error)
{
    if (output.target >= uint8_t(eval::OpOutputTarget::ComposeBase) &&
        output.target <= uint8_t(eval::OpOutputTarget::SpaceMatchStatus))
        return _RrStoreComposeGroup(program, step, output, value, error);
    if (output.target == uint8_t(eval::OpOutputTarget::InterpolatorStatus)) {
        const auto &interps = program->poses->poseInterpolators;
        if (output.object >= interps.size() ||
            step >= program->store.stepOutputs.size()) {
            if (error)
                *error = "op nodes: no pose interpolator " +
                         std::to_string(output.object) + " status to store";
            return false;
        }
        const auto &interp = interps[output.object];
        const std::string line = RigExecPoseInterpolatorStatusLine(
            RigExecPoseInterpolatorStatus(
                *static_cast<const uint32_t *>(value)),
            program->TextOrEmpty(interp.path), [&] {
                return interp.driverSlot >= 0
                           ? program->TextOrEmpty(
                                 program->slotMeta
                                     ->paths[size_t(interp.driverSlot)])
                           : std::string();
            });
        if (!line.empty())
            program->store.stepOutputs[step].diagnostics.push_back(line);
        return true;
    }
    if (output.target == uint8_t(eval::OpOutputTarget::Ladder)) {
        if (output.object < program->poses->composeGroups.size() &&
            _RrStoreLadder(program, output.object, output.port, value))
            return true;
        if (error)
            *error = "op nodes: compose group " +
                     std::to_string(output.object) + " stores no ladder port " +
                     std::to_string(output.port);
        return false;
    }
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
