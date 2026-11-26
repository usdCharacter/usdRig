#include "rigExec/internal/bakedProgramImpl.h"

#include "rigExec/api/rigDesc.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecOps/space/spaceOps.h"
#include "rigExecRigTypes/rigTypes.h"

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
    }
    return true;
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
        const RigExecPointFrame &frame = B.restFrames[lookup.object];
        auto &value = *static_cast<rigExecRig::PointFrame *>(out);
        for (size_t i = 0; i < 4; ++i)
            value.points[i] = rigExecStd::Vec3d(
                frame.points[i][0], frame.points[i][1], frame.points[i][2]);
        value.flags = frame.flags;
        return true;
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::CommitRecordGate) ||
        lookup.object >= B.commits.size() || lookup.read > 2)
        return SpaceFail(error, "commit " + std::to_string(lookup.object) +
                                    " holds no such record gate");
    const RigExecBakedCommit &commit = B.commits[lookup.object];
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
