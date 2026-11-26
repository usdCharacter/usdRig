#include "rigExec/internal/bakedProgramImpl.h"

#include "rigExec/api/rigDesc.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecOps/space/spaceOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"

#include "pxr/base/gf/matrix4d.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace rigExec {

namespace {

bool
SpaceFail(std::string *error, std::string why)
{
    if (error)
        *error = std::move(why);
    return false;
}

void
PutFrame(const RigExecPointFrame &frame, rigExecRig::PointFrame *out)
{
    for (size_t i = 0; i < 4; ++i)
        out->points[i] = rigExecStd::Vec3d(
            frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    out->flags = frame.flags;
}

RigExecPointFrame
TakeFrame(const rigExecRig::PointFrame &frame)
{
    RigExecPointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] =
            GfVec3d(frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    out.flags = frame.flags;
    return out;
}

} // namespace

bool
RigExecBakedAddSpaceOp(const RigExecBakedProgramImpl &B,
                       RigExecRigDescBuilder *builder,
                       eval::OpStepLayout *layout, size_t o, std::string *error)
{
    using namespace rigExecOps;
    using S = eval::OpLookupSource;
    using T = eval::OpOutputTarget;
    const auto lookUp = [&](RigExecNodeId node, const RigExecPortHandle &port,
                            S source, uint32_t object, int32_t part,
                            uint8_t read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(source), object, part, read});
    };
    const auto store = [&](RigExecNodeId node, const RigExecPortHandle &port,
                           T target, uint32_t object) {
        builder->BindOutput(node, port);
        layout->hostOutputs.push_back(
            {node, port.Index(), uint8_t(target), object});
    };
    const auto lookUpList = [&](RigExecNodeId node,
                                const RigExecPortHandle &port,
                                const std::vector<eval::OpLookup> &elements) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(S::List),
                                   uint32_t(layout->lookups.size()) + 1,
                                   int32_t(elements.size()), 0});
        layout->lookups.insert(layout->lookups.end(), elements.begin(),
                               elements.end());
    };
    const auto numbered = [&](RigExecNodeId node, size_t origin) {
        return node == origin ||
               SpaceFail(error, "op node " + std::to_string(origin) +
                                    " was numbered " + std::to_string(node));
    };
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    if (origin.family == RigExecBakedOpFamily::ProviderMatrix) {
        const uint32_t slot = uint32_t(origin.object);
        ProviderMatrixPayload payload;
        payload.posed = origin.part == 1;
        payload.joint = std::find(B.jointSlots.begin(), B.jointSlots.end(),
                                  int(slot)) != B.jointSlots.end();
        const RigExecNodeId node = builder->AddOp(ProviderMatrix(), &payload);
        if (!numbered(node, o))
            return false;
        lookUp(node, ProviderMatrixRest(), S::RestFrame, slot, -1, 0);
        if (payload.posed)
            lookUp(node, ProviderMatrixFinal(), S::PoseVersion,
                   uint32_t(B.finLast[slot]), 0, 0);
        lookUp(node, ProviderMatrixBase(), S::PoseVersion,
               uint32_t(B.baseLast[slot]), 1, 0);
        store(node, ProviderMatrixOut(),
              payload.posed ? T::ProviderFinalMatrix : T::ProviderBaseMatrix,
              slot);
    } else if (origin.family == RigExecBakedOpFamily::FrameMatrix) {
        const RigExecBakedFrameRecord &record =
            B.frameRecords[size_t(origin.object)];
        if (record.commit < 0 || size_t(record.commit) >= B.commits.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(origin.object) +
                                        " names no commit");
        if (record.slot < 0 || size_t(record.slot) >= B.restFrames.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(origin.object) +
                                        " names no slot");
        FrameMatrixPayload payload;
        payload.solverOutput = B.commits[size_t(record.commit)].solverOutput;
        payload.position = record.position;
        payload.target = record.target;
        const RigExecNodeId node = builder->AddOp(FrameMatrix(), &payload);
        if (!numbered(node, o))
            return false;
        const uint32_t commit = uint32_t(record.commit);
        lookUp(node, FrameMatrixRest(), S::RestFrame, uint32_t(record.slot), -1,
               0);
        lookUp(node, FrameMatrixFrame(), S::PoseVersion, record.version, 0, 0);
        if (payload.solverOutput) {
            lookUp(node, FrameMatrixPresent(), S::CommitRecordGate, commit, -1,
                   0);
        } else {
            lookUp(node, FrameMatrixRecordAfter(), S::CommitRecordGate, commit,
                   -1, 1);
            lookUp(node, FrameMatrixRecordEveryTarget(), S::CommitRecordGate,
                   commit, -1, 2);
        }
        store(node, FrameMatrixOut(), T::FrameMatrix, uint32_t(origin.object));
        store(node, FrameMatrixValid(), T::FrameMatrixValid,
              uint32_t(origin.object));
    } else if (origin.family == RigExecBakedOpFamily::ProviderRefresh) {
        const uint32_t index = uint32_t(origin.object);
        const auto &refresh = B.providerRefreshes[index];
        ProviderRefreshPayload payload;
        std::vector<eval::OpLookup> carryBase, carryFinal, parents;
        for (const auto &carry : refresh.carries) {
            payload.blockingCounts.push_back(
                uint32_t(carry.blockingSlots.size()));
            carryBase.push_back(
                {uint8_t(S::PoseVersion), carry.baseRead, 1, 0});
            carryFinal.push_back(
                {uint8_t(S::PoseVersion), carry.finRead, 0, 0});
            for (const int slot : carry.blockingSlots)
                parents.push_back(
                    {uint8_t(S::ProviderParentRaw), uint32_t(slot), -1, 0});
        }
        const RigExecNodeId node = builder->AddOp(ProviderRefresh(), &payload);
        if (!numbered(node, o))
            return false;
        lookUp(node, ProviderRefreshBase(), S::PoseVersion, refresh.baseRead, 1,
               0);
        lookUp(node, ProviderRefreshFinal(), S::PoseVersion, refresh.finRead, 0,
               0);
        lookUp(node, ProviderRefreshRaw(), S::ProviderFrame, index, -1, 0);
        lookUp(node, ProviderRefreshCurrent(), S::ProviderFrame, index, -1, 1);
        if (!refresh.priorConstraints.empty()) {
            std::vector<eval::OpLookup> committed;
            for (const auto &[commit, pos] : refresh.priorConstraints)
                committed.push_back(
                    {uint8_t(S::CommitRecordGate), commit, int32_t(pos), 3});
            lookUpList(node, ProviderRefreshConstrained(), committed);
        }
        lookUpList(node, ProviderRefreshCarryBase(), carryBase);
        lookUpList(node, ProviderRefreshCarryFinal(), carryFinal);
        if (!parents.empty())
            lookUpList(node, ProviderRefreshBlockingParents(), parents);
        store(node, ProviderRefreshBaseOut(), T::RefreshBase, index);
        store(node, ProviderRefreshFinalOut(), T::RefreshFinal, index);
        store(node, ProviderRefreshCarryBaseOut(), T::RefreshCarryBase, index);
        store(node, ProviderRefreshCarryFinalOut(), T::RefreshCarryFinal,
              index);
    }
    return true;
}

int
RigExecBakedProviderRefreshOf(const RigExecBakedProgramImpl &B,
                              const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::ProviderRefresh
               ? int(origin->object)
               : -1;
}

bool
RigExecBakedLookUpSpace(const RigExecBakedProgramImpl &B,
                        const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *out,
                        std::string *error)
{
    if (lookup.source == uint8_t(eval::OpLookupSource::RestFrame)) {
        if (type.size != sizeof(rigExecRig::PointFrame) ||
            lookup.object >= B.restFrames.size())
            return SpaceFail(error, "slot " + std::to_string(lookup.object) +
                                        " has no rest frame");
        PutFrame(B.restFrames[lookup.object],
                 static_cast<rigExecRig::PointFrame *>(out));
        return true;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderFrame)) {
        const RigExecPointFrame *frame =
            type.size == sizeof(rigExecRig::PointFrame) &&
                    lookup.object < B.providerRefreshes.size()
                ? B.providerValues.Read<RigExecPointFrame>(
                      lookup.read
                          ? B.providerRefreshes[lookup.object].currentValue
                          : B.providerRefreshes[lookup.object].baseValue)
                : nullptr;
        if (!frame)
            return SpaceFail(error, lookup.read ? "connected final pose input "
                                                  "incomplete"
                                                : "connected base pose input "
                                                  "incomplete");
        PutFrame(*frame, static_cast<rigExecRig::PointFrame *>(out));
        return true;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderParentRaw)) {
        if (type.size != sizeof(GfMatrix4d) ||
            lookup.object >= B.providerParentRawLeaves.size())
            return SpaceFail(error, "slot " + std::to_string(lookup.object) +
                                        " has no parent space");
        const GfMatrix4d raw =
            RigExecBakedProviderParentRaw(B, int(lookup.object));
        std::memcpy(out, raw.data(), sizeof(GfMatrix4d));
        return true;
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::CommitRecordGate) ||
        lookup.object >= B.commits.size() || lookup.read > 3)
        return SpaceFail(error, "commit " + std::to_string(lookup.object) +
                                    " holds no such record gate");
    const RigExecBakedCommit &commit = B.commits[lookup.object];
    if (lookup.read == 3) {
        if (type.size != sizeof(uint8_t) || lookup.part < 0 ||
            size_t(lookup.part) >= commit.present.size())
            return SpaceFail(error, "commit " + std::to_string(lookup.object) +
                                        " has no output " +
                                        std::to_string(lookup.part));
        *static_cast<uint8_t *>(out) =
            !commit.abandoned && commit.present[size_t(lookup.part)] ? 1 : 0;
        return true;
    }
    if (lookup.read == 0) {
        if (!type.borrow)
            return SpaceFail(error, "a commit's presence is an array");
        type.borrow(out, commit.present.data(), commit.present.size());
        return true;
    }
    if (type.size != sizeof(uint8_t))
        return SpaceFail(error, "a commit's record gate is a Bool");
    const bool gate =
        lookup.read == 1 ? commit.recordAfter : commit.recordEveryTarget;
    *static_cast<uint8_t *>(out) = gate ? 1 : 0;
    return true;
}

bool
RigExecBakedStoreSpace(RigExecBakedProgramImpl &B,
                       const eval::OpHostOutput &output, const void *value,
                       std::string *error)
{
    using T = eval::OpOutputTarget;
    const T target = T(output.target);
    if (target >= T::RefreshBase) {
        if (output.object >= B.providerRefreshes.size())
            return SpaceFail(error, "no provider refresh " +
                                        std::to_string(output.object));
        const auto &refresh = B.providerRefreshes[output.object];
        if (target == T::RefreshBase || target == T::RefreshFinal) {
            const auto &frame =
                *static_cast<const rigExecRig::PointFrame *>(value);
            (target == T::RefreshBase ? B.base[refresh.baseWrite]
                                      : B.fin[refresh.finWrite]) =
                TakeFrame(frame);
            return true;
        }
        const auto &frames = *static_cast<
            const rigExecStd::RigExecArray<rigExecRig::PointFrame> *>(value);
        if (frames.Size() != refresh.carries.size())
            return SpaceFail(error, "provider refresh " +
                                        std::to_string(output.object) +
                                        " carries " +
                                        std::to_string(refresh.carries.size()) +
                                        " descendants");
        for (size_t k = 0; k < frames.Size(); ++k) {
            const auto &carry = refresh.carries[k];
            (target == T::RefreshCarryBase ? B.base[carry.baseWrite]
                                           : B.fin[carry.finWrite]) =
                TakeFrame(frames[k]);
        }
        return true;
    }
    if (target == T::FrameMatrixValid) {
        if (output.object >= B.frameMatrixValid.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(output.object) +
                                        " has no slot");
        B.frameMatrixValid[output.object] =
            *static_cast<const uint8_t *>(value) != 0;
        return true;
    }
    std::vector<GfMatrix4d> &matrices =
        target == T::ProviderFinalMatrix  ? B.finalMatrix
        : target == T::ProviderBaseMatrix ? B.baseMatrix
                                          : B.frameMatrix;
    if (output.object >= matrices.size())
        return SpaceFail(error, "no matrix " + std::to_string(output.object) +
                                    " to store");
    std::memcpy(matrices[output.object].data(), value, sizeof(GfMatrix4d));
    return true;
}

} // namespace rigExec
