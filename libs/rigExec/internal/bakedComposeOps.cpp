#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/poseEvalHost.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/avarScale.h"
#include "rigExecOps/constraint/autoClavicleOps.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <string>
#include <vector>

namespace rigExec {

namespace {

using Doubles = rigExecStd::RigExecArray<double>;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Provider slot \p slot's avar bindings: the varying ones, then the
/// constant ones.
struct AvarBindings {
    const RigExecBakedProgramImpl &B;
    std::pair<size_t, size_t> varying, constant;

    AvarBindings(const RigExecBakedProgramImpl &program, uint32_t slot)
        : B(program), varying(RigExecBakedAvarBindingRange(
                          program.avarBindingBegin, int64_t(slot))),
          constant(RigExecBakedAvarBindingRange(
              program.avarConstantBindingBegin, int64_t(slot)))
    {
    }
    size_t Count() const
    {
        return (varying.second - varying.first) +
               (constant.second - constant.first);
    }
    const RigExecBakedProgramImpl::AvarBinding &At(size_t k) const
    {
        const size_t n = varying.second - varying.first;
        return k < n ? B.avarBindings[varying.first + k]
                     : B.avarConstantBindings[constant.first + (k - n)];
    }
    unsigned Channels() const
    {
        return RigExecBoundAvarChannels(Count(),
                                        [&](size_t k) { return At(k).slot; });
    }
};

/// Resizes list \p value to \p count elements of \p T and fills element k
/// with \p at(k).
template <class T, class At>
bool
FillList(const RigExecTypeDesc &type, void *value, size_t count, At &&at)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, count);
    T *out = static_cast<T *>(type.data(value));
    for (size_t k = 0; k < count; ++k)
        out[k] = at(k);
    return true;
}

/// Fills Mat4d list \p value with \p table's entries at \p slots.
bool
FillMatrices(const RigExecTypeDesc &type, void *value,
             const std::vector<int> &slots,
             const std::vector<GfMatrix4d> &table)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, slots.size());
    auto *out = static_cast<double *>(type.data(value));
    for (size_t k = 0; k < slots.size(); ++k)
        std::memcpy(out + 16 * k, table[size_t(slots[k])].data(),
                    16 * sizeof(double));
    return true;
}

/// Answers input \p read of space checkpoint \p object's SpaceCheckpoint
/// node from the slots it recomposes.
bool
LookUpCheckpoint(const RigExecBakedProgramImpl &B, uint32_t object,
                 eval::OpCheckpointRead read, const RigExecTypeDesc &type,
                 void *value)
{
    using R = eval::OpCheckpointRead;
    const auto &context = B.switchFrameContexts[object];
    const std::vector<int> &slots = context.recompose;
    const auto flags = [&](const std::vector<char> &table) {
        return FillList<uint8_t>(type, value, slots.size(), [&](size_t k) {
            return uint8_t(table[size_t(slots[k])] != 0);
        });
    };
    switch (read) {
    case R::Avars:
        return FillList<double>(
            type, value, slots.size() * kRigExecSlotAvars, [&](size_t k) {
                return B.avars[size_t(slots[k / kRigExecSlotAvars]) *
                                   kRigExecSlotAvars +
                               k % kRigExecSlotAvars];
            });
    case R::RotOrders:
        return FillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const TfToken &order = B.rotOrder[size_t(slots[k])];
            return RigExecRotOrderCode(order.GetText(), order.size());
        });
    case R::RotationSigns:
        return FillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const size_t s = size_t(slots[k]);
            return s < B.rotationSign.size() ? uint32_t(B.rotationSign[s]) : 0u;
        });
    case R::PosedDefault:
        return FillMatrices(type, value, slots, B.posedD);
    case R::ParentDefaultInverse:
        return FillMatrices(type, value, slots, B.parentDinv);
    case R::ParentExpression:
        return FillMatrices(type, value, slots, B.parentSpaceM);
    case R::ParentExpressionAuthored:
        return flags(B.parentSpaceAuthored);
    case R::PosedAuthored:
        return flags(B.posedAuthored);
    case R::PosedAuthoredMatrix:
        return FillMatrices(type, value, slots, B.posedAuthoredM);
    case R::Anchor:
        if (context.anchor < 0 || type.size != 16 * sizeof(double))
            return false;
        std::memcpy(value, B.posedM[size_t(context.anchor)].data(),
                    16 * sizeof(double));
        return true;
    }
    return false;
}

/// The SpaceCheckpoint node of space checkpoint \p object: every recomposed
/// slot's inputs and the anchor's posed matrix, into its switch frame.
bool
AddCheckpointOp(const RigExecBakedProgramImpl &B,
                RigExecRigDescBuilder *builder, eval::OpStepLayout *layout,
                size_t o, uint32_t object, std::string *error)
{
    using namespace rigExecOps;
    using R = eval::OpCheckpointRead;
    const auto &context = B.switchFrameContexts[object];
    SpaceCheckpointPayload payload;
    payload.noScale.reserve(context.recompose.size());
    for (const int slot : context.recompose)
        payload.noScale.push_back(B.noScaleAvars[size_t(slot)] != 0);
    const RigExecNodeId node = builder->AddOp(SpaceCheckpoint(), &payload);
    if (node != o)
        return Fail(error, "space checkpoint " + std::to_string(object) +
                               " was added as node " + std::to_string(node));
    const auto feed = [&](const RigExecPortHandle &port, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::CheckpointInputs), object, -1,
             uint8_t(read)});
    };
    feed(SpaceCheckpointAvars(), R::Avars);
    feed(SpaceCheckpointRotOrders(), R::RotOrders);
    feed(SpaceCheckpointRotationSigns(), R::RotationSigns);
    feed(SpaceCheckpointPosedDefault(), R::PosedDefault);
    feed(SpaceCheckpointParentDefaultInverse(), R::ParentDefaultInverse);
    feed(SpaceCheckpointParentExpression(), R::ParentExpression);
    feed(SpaceCheckpointParentExpressionAuthored(),
         R::ParentExpressionAuthored);
    feed(SpaceCheckpointPosedAuthored(), R::PosedAuthored);
    feed(SpaceCheckpointPosedAuthoredMatrix(), R::PosedAuthoredMatrix);
    if (context.anchor >= 0)
        feed(SpaceCheckpointAnchor(), R::Anchor);
    builder->BindOutput(node, SpaceCheckpointOut());
    layout->hostOutputs.push_back({node, SpaceCheckpointOut().Index(),
                                   uint8_t(eval::OpOutputTarget::SwitchFrame),
                                   object});
    return true;
}

/// Pose interpolator \p interp's solve and tables as its PoseInterpolator
/// payload.
void
InterpolatorPayload(const RigExecBakedProgramImpl::PoseInterpolator &interp,
                    rigExecOps::PoseInterpolatorPayload *payload)
{
    using rigExecOps::PoseSolve;
    payload->poseCount = interp.poseSlots.size();
    payload->enableTranslation = interp.enableTranslation;
    payload->allowNegativeWeights = interp.allowNegativeWeights;
    const auto vector = [](const GfVec3d &v) {
        return rigExecStd::Vec3d(v[0], v[1], v[2]);
    };
    if (interp.mapsSolve && interp.maps) {
        payload->solve = PoseSolve::Maps;
        const RigExecAnimatedMapsTable &table = *interp.maps;
        payload->mapsInputCount = table.inputCount;
        for (const RigExecConditionalRow &row : table.rows) {
            payload->mapRowInputs.push_back(int32_t(row.input));
            payload->mapRowFrom.push_back(row.from);
            payload->mapRowTo.push_back(row.to);
            payload->mapRowSlope.push_back(row.slope);
            payload->mapRowCut.push_back(row.cut);
        }
        for (const size_t begin : table.mapBegin)
            payload->mapBegin.push_back(begin);
        for (size_t m = 0; m < table.MapCount(); ++m)
            payload->mapUnclamped.push_back(table.Clamps(m) ? 0 : 1);
    } else if (interp.mlSolve && interp.network) {
        payload->solve = PoseSolve::Network;
        const RigExecMlNetwork &net = *interp.network;
        payload->networkInputCount = net.inputCount;
        for (const RigExecMlLayer &layer : net.layers) {
            payload->networkLayerSizes.push_back(int32_t(layer.outputCount));
            payload->networkActivations.push_back(uint32_t(layer.activation));
            payload->networkAlphas.push_back(float(layer.alpha));
        }
        const size_t biasBegin =
            net.layers.empty() ? 0 : net.layers.front().biasOffset;
        payload->networkWeights.assign(net.parameters.begin(),
                                       net.parameters.begin() + biasBegin);
        payload->networkBiases.assign(net.parameters.begin() + biasBegin,
                                      net.parameters.end());
        payload->networkInputMean = net.inputMean;
        payload->networkInputScale = net.inputScale;
        payload->networkDefaults = net.defaults;
    } else if (interp.rigLogicSolve) {
        payload->solve = PoseSolve::RbfMethod;
        const RigExecRbfMethodTable &table = interp.method;
        payload->methodSolverType = uint32_t(table.solverType);
        payload->methodDistance = uint32_t(table.distanceMethod);
        payload->methodWeightFunction = uint32_t(table.weightFunction);
        payload->methodNormalize = uint32_t(table.normalizeMethod);
        payload->methodTwistAxis = uint32_t(table.twistAxis);
        payload->methodRawCount = table.rawCount;
        payload->methodTargetCount = table.targetCount;
        payload->methodRadius = table.radius;
        payload->methodWeightThreshold = table.weightThreshold;
        payload->methodTargets = table.targets;
        payload->methodDistanceTargets = table.distanceTargets;
        payload->methodTargetScales = table.targetScales;
        payload->methodCoefficients = table.coefficients;
    } else {
        const RigExecRbfSolver &solver = interp.solver;
        for (const GfVec3d &p : solver.GetPoses())
            payload->rbfPoses.push_back(vector(p));
        for (const GfVec3d &t : solver.GetTranslations())
            payload->rbfTranslations.push_back(vector(t));
        for (const RigExecRbfPoseType type : solver.GetPoseTypes())
            payload->rbfPoseTypes.push_back(uint32_t(type));
        payload->rbfTwistAxis = vector(solver.GetTwistAxis());
        payload->rbfKernel = uint32_t(solver.GetKernel());
        payload->rbfRadius = solver.GetRadius();
        payload->rbfTranslationRadius = solver.GetTranslationRadius();
        payload->rbfRadii = solver.GetRadii();
        payload->rbfTranslationRadii = solver.GetTranslationRadii();
        payload->rbfNormalize = solver.GetNormalize();
        payload->rbfEnableRotation = solver.GetEnableRotation();
        payload->rbfEnableTranslation = solver.GetEnableTranslation();
        payload->rbfWeightRows = solver.GetWeights().size();
        for (const auto &row : solver.GetWeights())
            payload->rbfWeights.insert(payload->rbfWeights.end(), row.begin(),
                                       row.end());
    }
    if (payload->solve == PoseSolve::RbfMethod ||
        payload->solve == PoseSolve::Network)
        for (const int parent : interp.extraParentSlots)
            payload->extraParents.push_back(parent >= 0);
}

/// Whether pose interpolator \p interp's node reads extra driver frames.
bool
ReadsExtraDrivers(const RigExecBakedProgramImpl::PoseInterpolator &interp)
{
    return (interp.rigLogicSolve || (interp.mlSolve && interp.network)) &&
           !(interp.mapsSolve && interp.maps) &&
           !interp.extraDriverSlots.empty();
}

void
PutFrame(const RigExecPointFrame &frame, rigExecRig::PointFrame *out)
{
    for (size_t i = 0; i < 4; ++i)
        out->points[i] = rigExecStd::Vec3d(
            frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    out->flags = frame.flags;
}

/// Answers input \p read of pose interpolator \p object's node: its leaves
/// as the prologue sampled them, its drivers' last fin versions and rest
/// frames.
bool
LookUpInterpolator(const RigExecBakedProgramImpl &B, uint32_t object,
                   eval::OpInterpolatorRead read, const RigExecTypeDesc &type,
                   void *value)
{
    using R = eval::OpInterpolatorRead;
    using Frame = rigExecRig::PointFrame;
    const auto &interp = B.poseInterpolators[object];
    const auto frame = [&](int slot, bool final) {
        if (slot < 0 || type.size != sizeof(Frame))
            return false;
        PutFrame(final ? B.fin[size_t(B.finLast[size_t(slot)])]
                       : B.restFrames[size_t(slot)],
                 static_cast<Frame *>(value));
        return true;
    };
    const auto frames = [&](bool parents, bool final) {
        size_t count = 0;
        for (size_t e = 0; e < interp.extraDriverSlots.size(); ++e)
            count += !parents || interp.extraParentSlots[e] >= 0;
        if (!type.resize || !type.data)
            return false;
        type.resize(value, count);
        Frame *out = static_cast<Frame *>(type.data(value));
        for (size_t e = 0, k = 0; e < interp.extraDriverSlots.size(); ++e) {
            const int slot = parents ? interp.extraParentSlots[e]
                                     : interp.extraDriverSlots[e];
            if (slot < 0)
                continue;
            PutFrame(final ? B.fin[size_t(B.finLast[size_t(slot)])]
                           : B.restFrames[size_t(slot)],
                     &out[k++]);
        }
        return true;
    };
    switch (read) {
    case R::Enabled:
        if (type.size != sizeof(uint8_t))
            return false;
        *static_cast<uint8_t *>(value) =
            RigExecBakedLeafRead(B, interp.enabled) ? 1 : 0;
        return true;
    case R::Numeric:
        return FillList<double>(
            type, value, interp.valueInputs.size(), [&](size_t i) {
                return i < interp.valueIsFloat.size() && interp.valueIsFloat[i]
                           ? double(RigExecBakedLeafRead(
                                 B, interp.floatValueInputs[i]))
                           : RigExecBakedLeafRead(B, interp.valueInputs[i]);
            });
    case R::Mask:
        if (type.size != sizeof(double))
            return false;
        *static_cast<double *>(value) =
            double(RigExecBakedLeafRead(B, interp.mask));
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

/// The PoseInterpolator node of pose interpolator \p object: its leaves and
/// driver frames in, its weights out.
bool
AddInterpolatorOp(const RigExecBakedProgramImpl &B,
                  RigExecRigDescBuilder *builder, eval::OpStepLayout *layout,
                  size_t o, uint32_t object, std::string *error)
{
    using namespace rigExecOps;
    using R = eval::OpInterpolatorRead;
    const auto &interp = B.poseInterpolators[object];
    PoseInterpolatorPayload payload;
    InterpolatorPayload(interp, &payload);
    const RigExecNodeId node = builder->AddOp(PoseInterpolator(), &payload);
    if (node != o)
        return Fail(error, "pose interpolator " + std::to_string(object) +
                               " was added as node " + std::to_string(node));
    const auto feed = [&](const RigExecPortHandle &port, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::InterpolatorInputs), object, -1,
             uint8_t(read)});
    };
    feed(PoseInterpolatorEnabled(), R::Enabled);
    if (!interp.valueInputs.empty())
        feed(PoseInterpolatorNumeric(), R::Numeric);
    if (payload.solve == PoseSolve::Network)
        feed(PoseInterpolatorMask(), R::Mask);
    if (interp.driverSlot >= 0) {
        feed(PoseInterpolatorDriverFinal(), R::DriverFinal);
        feed(PoseInterpolatorDriverRest(), R::DriverRest);
        if (interp.parentSlot >= 0) {
            feed(PoseInterpolatorParentFinal(), R::ParentFinal);
            feed(PoseInterpolatorParentRest(), R::ParentRest);
        }
    }
    if (ReadsExtraDrivers(interp)) {
        feed(PoseInterpolatorExtraFinals(), R::ExtraFinals);
        feed(PoseInterpolatorExtraRests(), R::ExtraRests);
        if (std::any_of(interp.extraParentSlots.begin(),
                        interp.extraParentSlots.end(),
                        [](int parent) { return parent >= 0; })) {
            feed(PoseInterpolatorExtraParentFinals(), R::ExtraParentFinals);
            feed(PoseInterpolatorExtraParentRests(), R::ExtraParentRests);
        }
    }
    builder->BindOutput(node, PoseInterpolatorOut());
    layout->hostOutputs.push_back({node, PoseInterpolatorOut().Index(),
                                   uint8_t(eval::OpOutputTarget::PoseWeights),
                                   object});
    builder->BindOutput(node, PoseInterpolatorStatus());
    layout->hostOutputs.push_back(
        {node, PoseInterpolatorStatus().Index(),
         uint8_t(eval::OpOutputTarget::InterpolatorStatus), object});
    return true;
}

/// The slot compose group \p object composes.
size_t
GroupSlot(const RigExecBakedProgramImpl &B, uint32_t object)
{
    return size_t(B.composeGroups[object].begin);
}

bool
PutMatrix(const RigExecTypeDesc &type, void *value, const GfMatrix4d &matrix)
{
    if (type.size != 16 * sizeof(double))
        return false;
    std::memcpy(value, matrix.data(), 16 * sizeof(double));
    return true;
}

/// Answers input \p read of compose group \p object's RestCompose node.
bool
LookUpRest(const RigExecBakedProgramImpl &B, uint32_t object,
           eval::OpRestRead read, const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpRestRead;
    const BakedPoseInputs inputs(&B);
    const size_t slot = GroupSlot(B, object);
    switch (read) {
    case R::RestAvars:
        return FillList<double>(
            type, value, 6, [&](size_t k) { return inputs.restAvar(slot, k); });
    case R::RestSpace:
        return PutMatrix(type, value,
                         inputs.ladderSpace(slot, eval::kPoseLadderRestSpace));
    case R::Intervening:
        return PutMatrix(type, value, inputs.interveningSpace(slot));
    case R::ParentRest: {
        const int parent = inputs.parent(slot);
        GfMatrix4d rest;
        return parent >= 0 && inputs.restRoundTrip(size_t(parent), &rest) &&
               PutMatrix(type, value, rest);
    }
    }
    return false;
}

/// The RestCompose node of compose group \p object: its slot's rest
/// channels and its parent's rest in, the slot's rest tables out.
bool
AddRestOp(const RigExecBakedProgramImpl &B, RigExecRigDescBuilder *builder,
          eval::OpStepLayout *layout, size_t o, uint32_t object,
          std::string *error)
{
    using namespace rigExecOps;
    using R = eval::OpRestRead;
    using P = eval::OpRestOutput;
    if (RestComposeRestMatrix().Index() != uint32_t(P::RestMatrix) ||
        RestComposeRest().Index() != uint32_t(P::Rest) ||
        RestComposeRoundTrip().Index() != uint32_t(P::RoundTrip))
        return Fail(error, "RestCompose's outputs are not ports 4 to 6");
    const size_t slot = GroupSlot(B, object);
    SlotActivityPayload payload;
    payload.active = BakedPoseInputs(&B).providerActive(slot);
    const RigExecNodeId node = builder->AddOp(RestCompose(), &payload);
    if (node != o)
        return Fail(error, "compose group " + std::to_string(object) +
                               " was added as node " + std::to_string(node));
    const auto feed = [&](const RigExecPortHandle &port, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(eval::OpLookupSource::RestInputs),
                                   object, -1, uint8_t(read)});
    };
    feed(RestComposeRestAvars(), R::RestAvars);
    feed(RestComposeRestSpace(), R::RestSpace);
    feed(RestComposeIntervening(), R::Intervening);
    if (B.parent[slot] >= 0)
        feed(RestComposeParentRest(), R::ParentRest);
    for (const RigExecPortHandle *port :
         {&RestComposeRestMatrix(), &RestComposeRest(),
          &RestComposeRoundTrip()}) {
        builder->BindOutput(node, *port);
        layout->hostOutputs.push_back(
            {node, port->Index(), uint8_t(eval::OpOutputTarget::Rest), object});
    }
    return true;
}

/// Stores output \p port of compose group \p object's RestCompose node
/// into its slot's rest tables; a rest matrix that moved marks the slot's
/// rest changed.
bool
StoreRest(RigExecBakedProgramImpl &B, uint32_t object, uint32_t port,
          const void *value)
{
    using P = eval::OpRestOutput;
    const size_t slot = GroupSlot(B, object);
    switch (P(port)) {
    case P::RestMatrix: {
        GfMatrix4d &restM = B.restM[slot];
        std::memcpy(restM.data(), value, 16 * sizeof(double));
        if (std::memcmp(restM.data(), B.lastRestM[slot].data(),
                        16 * sizeof(double)) != 0) {
            B.lastRestM[slot] = restM;
            B.restChanged[slot] = 1;
        }
        return true;
    }
    case P::Rest: {
        const auto &frame = *static_cast<const rigExecRig::PointFrame *>(value);
        std::array<GfVec3d, 4> points;
        for (size_t i = 0; i < 4; ++i)
            points[i] = GfVec3d(frame.points[i][0], frame.points[i][1],
                                frame.points[i][2]);
        B.restPts[slot] = points;
        B.restFrames[slot] = BakedPoseMath::MakeFrame(points, frame.flags);
        return true;
    }
    case P::RoundTrip:
        std::memcpy(B.restRoundTrip[slot].data(), value, 16 * sizeof(double));
        return true;
    }
    return false;
}

/// Answers input \p read of compose group \p object's LadderCompose node.
bool
LookUpLadder(const RigExecBakedProgramImpl &B, uint32_t object,
             eval::OpLadderRead read, const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpLadderRead;
    const BakedPoseInputs inputs(&B);
    const size_t slot = GroupSlot(B, object);
    const int parent = inputs.parent(slot);
    switch (read) {
    case R::Spaces: {
        if (!type.resize || !type.data)
            return false;
        type.resize(value, 6);
        auto *out = static_cast<double *>(type.data(value));
        for (int c = 0; c < 6; ++c)
            std::memcpy(out + 16 * c, inputs.ladderSpace(slot, c + 1).data(),
                        16 * sizeof(double));
        return true;
    }
    case R::Authored:
        return FillList<uint8_t>(type, value, 6, [&](size_t c) {
            return uint8_t(RigExecBakedLadderAuthored(B, slot, int(c) + 1));
        });
    case R::ParentSpaceLeaf:
        return PutMatrix(type, value, inputs.ladderParentSpaceLeaf(slot));
    case R::RotationSign: {
        if (type.size != 3 * sizeof(double))
            return false;
        const GfVec3d sign = inputs.ladderRotationSign(slot);
        std::memcpy(value, sign.data(), 3 * sizeof(double));
        return true;
    }
    case R::DefaultAvars:
        return FillList<double>(type, value, 6, [&](size_t k) {
            return inputs.defaultAvar(slot, k);
        });
    case R::OwnRest:
        return PutMatrix(type, value, B.restRoundTrip[slot]);
    case R::ParentRest:
        return parent >= 0 &&
               PutMatrix(type, value, B.restRoundTrip[size_t(parent)]);
    case R::ParentDefault:
        return parent >= 0 &&
               PutMatrix(type, value, B.defaultRoundTrip[size_t(parent)]);
    }
    return false;
}

/// The LadderCompose node of compose group \p object: its slot's ladder
/// channels and the rests and parent ladder composed before it in, the
/// slot's ladder tables out.
bool
AddLadderOp(const RigExecBakedProgramImpl &B, RigExecRigDescBuilder *builder,
            eval::OpStepLayout *layout, size_t o, uint32_t object,
            std::string *error)
{
    using namespace rigExecOps;
    using R = eval::OpLadderRead;
    using P = eval::OpLadderOutput;
    const RigExecPortHandle *outputs[] = {
        &LadderComposeSelfDefault(),      &LadderComposeParentDefaultInverse(),
        &LadderComposeDefaultRoundTrip(), &LadderComposePosedDefault(),
        &LadderComposeParentSpace(),      &LadderComposePosedAuthoredMatrix(),
        &LadderComposePosedAuthored(),    &LadderComposeParentSpaceAuthored(),
        &LadderComposeOutRotationSign()};
    for (size_t k = 0; k < std::size(outputs); ++k)
        if (outputs[k]->Index() != uint32_t(P::SelfDefault) + k)
            return Fail(error, "LadderCompose's outputs are not ports 8 to 16");
    const size_t slot = GroupSlot(B, object);
    SlotActivityPayload payload;
    payload.active = BakedPoseInputs(&B).providerActive(slot);
    const RigExecNodeId node = builder->AddOp(LadderCompose(), &payload);
    if (node != o)
        return Fail(error, "compose group " + std::to_string(object) +
                               " was added as node " + std::to_string(node));
    const auto feed = [&](const RigExecPortHandle &port, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(eval::OpLookupSource::LadderInputs),
                                   object, -1, uint8_t(read)});
    };
    feed(LadderComposeSpaces(), R::Spaces);
    feed(LadderComposeAuthored(), R::Authored);
    feed(LadderComposeParentSpaceLeaf(), R::ParentSpaceLeaf);
    feed(LadderComposeRotationSign(), R::RotationSign);
    feed(LadderComposeDefaultAvars(), R::DefaultAvars);
    feed(LadderComposeOwnRest(), R::OwnRest);
    if (B.parent[slot] >= 0) {
        feed(LadderComposeParentRest(), R::ParentRest);
        feed(LadderComposeParentDefault(), R::ParentDefault);
    }
    for (const RigExecPortHandle *port : outputs) {
        builder->BindOutput(node, *port);
        layout->hostOutputs.push_back({node, port->Index(),
                                       uint8_t(eval::OpOutputTarget::Ladder),
                                       object});
    }
    return true;
}

/// Stores output \p port of compose group \p object's LadderCompose node
/// into its slot's ladder tables. The last port, the rotation sign, also
/// takes the slot's rotation order and, for an active slot whose ladder
/// moved, marks it changed.
bool
StoreLadder(RigExecBakedProgramImpl &B, uint32_t object, uint32_t port,
            const void *value)
{
    using P = eval::OpLadderOutput;
    const size_t slot = GroupSlot(B, object);
    const auto matrix = [&](std::vector<GfMatrix4d> &table) {
        std::memcpy(table[slot].data(), value, 16 * sizeof(double));
        return true;
    };
    const auto flag = [&](std::vector<char> &table) {
        table[slot] = *static_cast<const uint8_t *>(value) ? 1 : 0;
        return true;
    };
    switch (P(port)) {
    case P::SelfDefault:
        return matrix(B.selfD);
    case P::ParentDefaultInverse:
        return matrix(B.parentDinv);
    case P::DefaultRoundTrip:
        return matrix(B.defaultRoundTrip);
    case P::PosedDefault:
        return matrix(B.posedD);
    case P::ParentSpace:
        return matrix(B.parentSpaceM);
    case P::PosedAuthoredMatrix:
        return matrix(B.posedAuthoredM);
    case P::PosedAuthored:
        return flag(B.posedAuthored);
    case P::ParentSpaceAuthored:
        return flag(B.parentSpaceAuthored);
    case P::RotationSign:
        break;
    default:
        return false;
    }
    B.rotationSign[slot] =
        static_cast<unsigned char>(*static_cast<const uint32_t *>(value));
    if (!BakedPoseInputs(&B).providerActive(slot)) {
        B.rotOrder[slot] = B.xyzToken;
        return true;
    }
    B.rotOrder[slot] = RigExecBakedLadderRotOrder(B, slot);
    const auto same = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return std::memcmp(a.data(), b.data(), 16 * sizeof(double)) == 0;
    };
    if (same(B.selfD[slot], B.lastSelfD[slot]) &&
        same(B.parentDinv[slot], B.lastParentDinv[slot]) &&
        B.posedAuthored[slot] == B.lastPosedAuthored[slot] &&
        same(B.posedAuthoredM[slot], B.lastPosedAuthoredM[slot]) &&
        B.rotOrder[slot] == B.lastRotOrder[slot] &&
        same(B.posedD[slot], B.lastPosedD[slot]) &&
        same(B.parentSpaceM[slot], B.lastParentSpaceM[slot]) &&
        B.parentSpaceAuthored[slot] == B.lastParentSpaceAuthored[slot] &&
        B.rotationSign[slot] == B.lastRotationSign[slot])
        return true;
    B.lastSelfD[slot] = B.selfD[slot];
    B.lastParentDinv[slot] = B.parentDinv[slot];
    B.lastPosedAuthored[slot] = B.posedAuthored[slot];
    B.lastPosedAuthoredM[slot] = B.posedAuthoredM[slot];
    B.lastRotOrder[slot] = B.rotOrder[slot];
    B.lastPosedD[slot] = B.posedD[slot];
    B.lastParentSpaceM[slot] = B.parentSpaceM[slot];
    B.lastParentSpaceAuthored[slot] = B.parentSpaceAuthored[slot];
    B.lastRotationSign[slot] = B.rotationSign[slot];
    B.ladderChanged[slot] = 1;
    return true;
}

/// Frame \p k of auto clavicle \p ac's 13, or null where the op reads none.
const RigExecBakedProgramImpl::AutoClavicle::FrameRead *
ClavicleFrame(const RigExecBakedProgramImpl::AutoClavicle &ac, size_t k)
{
    if (k >= ac.operation.frames.size() || ac.operation.frames[k] == UINT64_MAX)
        return nullptr;
    for (const auto &read : ac.frames)
        if (read.value == ac.operation.frames[k])
            return &read;
    return nullptr;
}

/// Whether \p read is a frame of the clavicle's own subtree, which the op
/// reads from the slot's unshifted compose.
bool
InClavicleSubtree(const RigExecBakedProgramImpl::AutoClavicle::FrameRead &read,
                  int slot)
{
    return read.computation.empty() &&
           (!read.recompose.empty() || read.slot == slot);
}

/// The clavicle frames a single Mat4d port reads, by frame number.
constexpr size_t kClavicleFrames[] = {1, 2, 3, 4, 8, 9};

/// The op nodes compose group \p g runs as, in origin order: its compose,
/// its space match, the recomposes of its clavicle's subtree frames, its
/// auto clavicle and its shift.
struct ComposeNodes {
    int slot = -1;
    int switchIndex = -1;
    int clavicle = -1;
    bool match = false;
    /// Per recompose node: the clavicle frame it ends or continues and the
    /// slot it composes.
    std::vector<std::pair<size_t, int>> recompose;

    size_t Count() const
    {
        return 1 + size_t(match) + recompose.size() + (clavicle >= 0 ? 2 : 0);
    }
    size_t Offset(int part) const
    {
        if (part == 0)
            return 0;
        if (part == 1)
            return 1;
        if (part == 2)
            return 1 + size_t(match) + recompose.size();
        if (part >= 3)
            return 1 + size_t(match) + size_t(part - 3);
        return clavicle >= 0 ? Count() - 1 : 0;
    }
    int Part(size_t n) const
    {
        if (clavicle < 0)
            return n == 0 ? -1 : 1;
        if (n + 1 == Count())
            return -1;
        if (n + 2 == Count())
            return 2;
        if (n == 0)
            return 0;
        if (match && n == 1)
            return 1;
        return int(n - 1 - size_t(match)) + 3;
    }
};

ComposeNodes
ComposeNodesOf(const RigExecBakedProgramImpl &B, size_t g)
{
    ComposeNodes nodes;
    nodes.slot = B.composeGroups[g].begin;
    const size_t slot = size_t(nodes.slot);
    const bool active = BakedPoseInputs(&B).providerActive(slot);
    nodes.switchIndex =
        slot < B.spaceSwitchBySlot.size() ? B.spaceSwitchBySlot[slot] : -1;
    nodes.match = active && nodes.switchIndex >= 0;
    nodes.clavicle = active && slot < B.autoClavicleBySlot.size()
                         ? B.autoClavicleBySlot[slot]
                         : -1;
    if (nodes.clavicle < 0)
        return nodes;
    const auto &ac = B.autoClavicles[size_t(nodes.clavicle)];
    for (const size_t k : kClavicleFrames)
        if (const auto *read = ClavicleFrame(ac, k);
            read && InClavicleSubtree(*read, ac.slot))
            for (const int child : read->recompose)
                nodes.recompose.emplace_back(k, child);
    return nodes;
}

/// The SpaceSwitchSetup of space switch \p sw.
rigExecOps::SpaceSwitchSetup
SwitchSetupOf(const RigExecBakedProgramImpl::SpaceSwitch &sw)
{
    rigExecOps::SpaceSwitchSetup setup;
    for (const auto filter : sw.kernelRecord.filters)
        setup.filters.push_back(int(filter));
    setup.twistAxis = rigExecStd::Vec3d(sw.kernelRecord.twistAxis[0],
                                        sw.kernelRecord.twistAxis[1],
                                        sw.kernelRecord.twistAxis[2]);
    setup.affectTranslation = sw.kernelRecord.affectTranslation;
    setup.affectRotation = sw.kernelRecord.affectRotation;
    setup.affectScale = sw.kernelRecord.affectScale;
    setup.hasCarry = sw.spaceSlot >= 0;
    for (const int source : sw.sourceSlots)
        setup.world.push_back(source < 0 ? 1 : 0);
    return setup;
}

/// The index into \p ac.scalars of the clavicle op's scalar \p k, or -1
/// where it reads none.
int
ClavicleScalar(const RigExecBakedProgramImpl::AutoClavicle &ac, size_t k)
{
    const auto &read = ac.operation.scalars[k];
    if (read.hops.empty())
        return -1;
    for (size_t j = 0; j < ac.scalars.size(); ++j)
        if (ac.scalars[j].value == read.hops.front().raw)
            return int(j);
    return -1;
}

/// Answers input \p read of auto clavicle \p object's node: a frame outside
/// its subtree, or a scalar.
bool
LookUpClavicle(const RigExecBakedProgramImpl &B, uint32_t object, uint8_t read,
               const RigExecTypeDesc &type, void *value)
{
    using R = eval::OpClavicleRead;
    const auto &ac = B.autoClavicles[object];
    if (read >= uint8_t(R::ScalarBegin)) {
        const size_t k = size_t(read - uint8_t(R::ScalarBegin));
        if (k >= ac.operation.scalars.size() || type.size != sizeof(double))
            return false;
        const int j = ClavicleScalar(ac, k);
        double scalar = k == 0 ? 1.0 - ac.operation.constants.ikValue : 1.0;
        if (j >= 0)
            scalar = ac.scalars[size_t(j)].isFloat
                         ? double(RigExecBakedLeafRead(
                               B, ac.scalars[size_t(j)].narrow))
                         : RigExecBakedLeafRead(B, ac.scalars[size_t(j)].wide);
        else if (k > 1)
            return false;
        *static_cast<double *>(value) = scalar;
        return true;
    }
    const auto matrix = [&](size_t k, double *out) {
        const auto *frame = ClavicleFrame(ac, k);
        if (!frame || frame->slot < 0)
            return false;
        const size_t slot = size_t(frame->slot);
        GfMatrix4d m;
        if (frame->computation == SchemaComputationText::computeDefaultFrame)
            m = B.defaultRoundTrip[slot];
        else {
            const RigExecPointFrame &f =
                frame->computation == SchemaComputationText::computeRestFrame
                    ? B.restFrames[slot]
                    : B.fin[slot];
            m = eval::pose_ops::SpaceOfFrame<BakedPoseMath>(f.points, f.flags);
        }
        std::memcpy(out, m.data(), 16 * sizeof(double));
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

/// Op node \p o of compose group \p g (origin part \p part): its compose,
/// space match, a recompose, its auto clavicle or its shift.
bool
AddComposeGroupOp(const RigExecBakedProgramImpl &B,
                  RigExecRigDescBuilder *builder, eval::OpStepLayout *layout,
                  size_t o, size_t g, int part, std::string *error)
{
    using namespace rigExecOps;
    using L = eval::OpLookupSource;
    using C = eval::OpComposeRead;
    using S = eval::OpSwitchRead;
    using T = eval::OpOutputTarget;
    const ComposeNodes nodes = ComposeNodesOf(B, g);
    const size_t offset = nodes.Offset(part);
    if (offset > o || offset >= nodes.Count())
        return Fail(error, "compose group " + std::to_string(g) +
                               " has no node part " + std::to_string(part));
    const RigExecNodeId first = RigExecNodeId(o - offset);
    const uint32_t slot = uint32_t(nodes.slot);
    const BakedPoseInputs inputs(&B);
    RigExecNodeId node = RigExecNodeId(kRigExecInvalidIndex);
    const auto look = [&](const RigExecPortHandle &port, L source,
                          uint32_t object, uint8_t read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(source), object, -1, read});
    };
    const auto compose = [&](const RigExecPortHandle &port, uint32_t of,
                             C read) {
        look(port, L::ComposeInputs, of, uint8_t(read));
    };
    const auto added = [&](RigExecNodeId id) {
        node = id;
        return id == o ||
               Fail(error, "compose group " + std::to_string(g) + " node " +
                               std::to_string(o) + " was added as node " +
                               std::to_string(id));
    };
    const auto store = [&](const RigExecPortHandle &base,
                           const RigExecPortHandle &posed) {
        builder->BindOutput(node, base);
        layout->hostOutputs.push_back(
            {node, base.Index(), uint8_t(T::ComposeBase), slot});
        builder->BindOutput(node, posed);
        layout->hostOutputs.push_back(
            {node, posed.Index(), uint8_t(T::ComposePosed), slot});
    };
    const bool switched = nodes.switchIndex >= 0;
    const RigExecPortHandle &composeBase =
        switched ? SwitchedComposeSlotBase() : ComposeSlotBase();
    const RigExecPortHandle &composePosed =
        switched ? SwitchedComposeSlotPosedMatrix() : ComposeSlotPosedMatrix();
    const auto switchInputs = [&](const RigExecPortHandle *const ports[9],
                                  bool carry) {
        const S reads[] = {S::PosedDefault, S::ParentDefaultInverse,
                           S::ParentPosed,  S::ParentDefault,
                           S::Active,       S::SpaceDefault,
                           S::SpacePosed,   S::SourceDefaults,
                           S::SourcePosed};
        for (size_t k = 0; k < 9; ++k)
            if (ports[k] && (carry || (reads[k] != S::SpaceDefault &&
                                       reads[k] != S::SpacePosed)))
                look(*ports[k], L::SwitchInputs, uint32_t(nodes.switchIndex),
                     uint8_t(reads[k]));
    };
    if (offset == 0) {
        const bool active = inputs.providerActive(slot);
        const bool noScale = B.noScaleAvars[slot] != 0;
        if (!switched) {
            ComposeSlotPayload payload;
            payload.active = active;
            payload.noScale = noScale;
            if (!added(builder->AddOp(ComposeSlot(), &payload)))
                return false;
            compose(ComposeSlotAvars(), slot, C::Avars);
            compose(ComposeSlotRotOrder(), slot, C::RotOrder);
            compose(ComposeSlotRotationSign(), slot, C::RotationSign);
            compose(ComposeSlotPosedAuthored(), slot, C::PosedAuthored);
            compose(ComposeSlotPosedAuthoredMatrix(), slot,
                    C::PosedAuthoredMatrix);
            compose(ComposeSlotPosedDefault(), slot, C::PosedDefault);
            compose(ComposeSlotParentDefaultInverse(), slot,
                    C::ParentDefaultInverse);
            compose(ComposeSlotParentSpace(), slot, C::ParentSpace);
            compose(ComposeSlotParentSpaceAuthored(), slot,
                    C::ParentSpaceAuthored);
            if (inputs.parent(slot) >= 0)
                compose(ComposeSlotParentPosed(), slot, C::ParentPosed);
        } else {
            const auto &sw = B.spaceSwitches[size_t(nodes.switchIndex)];
            SwitchedComposeSlotPayload payload;
            payload.active = active;
            payload.noScale = noScale;
            payload.space = SwitchSetupOf(sw);
            if (!added(builder->AddOp(SwitchedComposeSlot(), &payload)))
                return false;
            compose(SwitchedComposeSlotAvars(), slot, C::Avars);
            compose(SwitchedComposeSlotRotOrder(), slot, C::RotOrder);
            compose(SwitchedComposeSlotRotationSign(), slot, C::RotationSign);
            compose(SwitchedComposeSlotPosedAuthored(), slot, C::PosedAuthored);
            compose(SwitchedComposeSlotPosedAuthoredMatrix(), slot,
                    C::PosedAuthoredMatrix);
            const RigExecPortHandle *const ports[9] = {
                &SwitchedComposeSlotPosedDefault(),
                &SwitchedComposeSlotParentDefaultInverse(),
                &SwitchedComposeSlotParentPosed(),
                &SwitchedComposeSlotParentDefault(),
                &SwitchedComposeSlotActive(),
                &SwitchedComposeSlotSpaceDefault(),
                &SwitchedComposeSlotSpacePosed(),
                &SwitchedComposeSlotSourceDefaults(),
                &SwitchedComposeSlotSourcePosed()};
            switchInputs(ports, payload.space.hasCarry);
        }
        if (nodes.clavicle < 0)
            store(composeBase, composePosed);
        return true;
    }
    if (part == 1) {
        const auto &sw = B.spaceSwitches[size_t(nodes.switchIndex)];
        SpaceMatchPayload payload;
        payload.space = SwitchSetupOf(sw);
        if (!added(builder->AddOp(SpaceMatch(), &payload)))
            return false;
        builder->Connect(first, composePosed, node, SpaceMatchTarget());
        compose(SpaceMatchChannels(), slot, C::MatchChannels);
        const RigExecPortHandle *const ports[9] = {
            &SpaceMatchPosedDefault(),
            &SpaceMatchParentDefaultInverse(),
            &SpaceMatchParentPosed(),
            &SpaceMatchParentDefault(),
            nullptr,
            &SpaceMatchSpaceDefault(),
            &SpaceMatchSpacePosed(),
            &SpaceMatchSourceDefaults(),
            &SpaceMatchSourcePosed()};
        switchInputs(ports, payload.space.hasCarry);
        for (const auto &[port, target] :
             {std::make_pair(&SpaceMatchValues(), T::SpaceMatchValues),
              std::make_pair(&SpaceMatchStatus(), T::SpaceMatchStatus)}) {
            builder->BindOutput(node, *port);
            layout->hostOutputs.push_back({node, port->Index(), uint8_t(target),
                                           uint32_t(nodes.switchIndex)});
        }
        return true;
    }
    if (part >= 3) {
        const size_t r = size_t(part - 3);
        const auto [frame, child] = nodes.recompose[r];
        ComposeSlotPayload payload;
        payload.noScale = B.noScaleAvars[size_t(child)] != 0;
        if (!added(builder->AddOp(ComposeSlot(), &payload)))
            return false;
        const uint32_t of = uint32_t(child);
        compose(ComposeSlotAvars(), of, C::Avars);
        compose(ComposeSlotRotOrder(), of, C::RotOrder);
        compose(ComposeSlotRotationSign(), of, C::RotationSign);
        compose(ComposeSlotPosedAuthored(), of, C::PosedAuthored);
        compose(ComposeSlotPosedAuthoredMatrix(), of, C::PosedAuthoredMatrix);
        compose(ComposeSlotPosedDefault(), of, C::PosedDefault);
        compose(ComposeSlotParentDefaultInverse(), of, C::ParentDefaultInverse);
        compose(ComposeSlotParentSpace(), of, C::ParentSpace);
        compose(ComposeSlotParentSpaceAuthored(), of, C::ParentSpaceAuthored);
        if (r > 0 && nodes.recompose[r - 1].first == frame)
            builder->Connect(node - 1, ComposeSlotPosedMatrix(), node,
                             ComposeSlotParentPosed());
        else
            builder->Connect(first, composePosed, node,
                             ComposeSlotParentPosed());
        return true;
    }
    const auto &ac = B.autoClavicles[size_t(nodes.clavicle)];
    const std::string path = (*B.pathTexts)[slot];
    if (part == 2) {
        AutoClavicleOpPayload payload;
        payload.constants = ac.operation.constants;
        payload.hasLimb = ac.operation.hasLimb;
        if (!added(builder->AddOp(AutoClavicle(), &payload)))
            return false;
        builder->Connect(first, composePosed, node, AutoClavicleTarget());
        const RigExecPortHandle *const ports[] = {
            &AutoClaviclePivot(),         &AutoClavicleAnchor(),
            &AutoClavicleAnchorDefault(), &AutoClavicleFkPosed(),
            &AutoClavicleIkTarget(),      &AutoClaviclePole()};
        using R = eval::OpClavicleRead;
        for (size_t f = 0; f < std::size(kClavicleFrames); ++f) {
            const size_t k = kClavicleFrames[f];
            const auto *read = ClavicleFrame(ac, k);
            if (!read) {
                if (k < 8)
                    return Fail(error, path +
                                           ": its auto clavicle reads no "
                                           "frame " +
                                           std::to_string(k));
                continue;
            }
            if (!InClavicleSubtree(*read, ac.slot)) {
                look(*ports[f], L::ClavicleInputs, uint32_t(nodes.clavicle),
                     uint8_t(k));
                continue;
            }
            RigExecNodeId from = first;
            const RigExecPortHandle *out = &composePosed;
            for (size_t r = 0; r < nodes.recompose.size(); ++r)
                if (nodes.recompose[r].first == k) {
                    from = first + RigExecNodeId(1 + size_t(nodes.match) + r);
                    out = &ComposeSlotPosedMatrix();
                }
            builder->Connect(from, *out, node, *ports[f]);
        }
        for (const R list : {R::FkDefaults, R::LimbRests}) {
            if (list == R::LimbRests && !ac.operation.hasLimb)
                continue;
            for (size_t e = 0; e < 3; ++e) {
                const auto *read = ClavicleFrame(ac, size_t(list) + e);
                if (!read || InClavicleSubtree(*read, ac.slot))
                    return Fail(error, path +
                                           ": its auto clavicle reads "
                                           "frame " +
                                           std::to_string(size_t(list) + e) +
                                           " from its own subtree");
            }
            look(list == R::FkDefaults ? AutoClavicleFkDefault()
                                       : AutoClavicleLimbRests(),
                 L::ClavicleInputs, uint32_t(nodes.clavicle), uint8_t(list));
        }
        const RigExecPortHandle *const scalars[] = {
            &AutoClavicleIkBlend(),      &AutoClavicleAmount(),
            &AutoClavicleStretch(),      &AutoClaviclePin(),
            &AutoClavicleUpperScale(),   &AutoClavicleLowerScale(),
            &AutoClavicleSoftDistance(), &AutoClavicleScaleCalibration(),
            &AutoClavicleTwist(),        &AutoClavicleUpperOffset(),
            &AutoClavicleLowerOffset()};
        for (size_t k = 0; k < std::size(scalars); ++k)
            if (k < 2 || (ac.operation.hasLimb && ClavicleScalar(ac, k) >= 0))
                look(*scalars[k], L::ClavicleInputs, uint32_t(nodes.clavicle),
                     uint8_t(size_t(R::ScalarBegin) + k));
        return true;
    }
    if (!added(builder->AddOp(ShiftedComposeSlot(), nullptr)))
        return false;
    builder->Connect(first, composeBase, node, ShiftedComposeSlotBase());
    builder->Connect(node - 1, AutoClavicleDelta(), node,
                     ShiftedComposeSlotDelta());
    builder->Connect(node - 1, AutoClavicleOut(), node,
                     ShiftedComposeSlotShifted());
    store(ShiftedComposeSlotOut(), ShiftedComposeSlotPosedMatrix());
    return true;
}

/// Stores output \p output of a compose group's node: the slot's first
/// pose and posed matrix, or its switch's match values and status.
bool
StoreComposeGroup(RigExecBakedProgramImpl &B, RigExecBakedStep *step,
                  const eval::OpHostOutput &output, const void *value,
                  std::string *error)
{
    using T = eval::OpOutputTarget;
    const T target = T(output.target);
    if (target == T::SpaceMatchValues || target == T::SpaceMatchStatus) {
        if (output.object >= B.spaceSwitches.size())
            return Fail(error,
                        "no space switch " + std::to_string(output.object));
        auto &sw = B.spaceSwitches[output.object];
        if (target == T::SpaceMatchValues) {
            const auto &values = *static_cast<const Doubles *>(value);
            sw.matchValues.assign(values.Data(), values.Data() + values.Size());
            return true;
        }
        const uint32_t status = *static_cast<const uint32_t *>(value);
        sw.matchValid = status == uint32_t(RigExecSpaceMatchStatus::Matched) &&
                        !B.posedAuthored[size_t(sw.slot)];
        if (status == uint32_t(RigExecSpaceMatchStatus::NotFinite)) {
            if (!step)
                return Fail(error, "stores a space match status outside a "
                                   "step");
            step->diagnostics.push_back((*B.pathTexts)[size_t(sw.slot)] +
                                        kRigExecSpaceMatchNotFinite);
        }
        return true;
    }
    const size_t slot = output.object;
    if (slot >= B.base.size() || slot >= B.posedM.size())
        return Fail(error, "no slot " + std::to_string(slot) + " to compose");
    if (target == T::ComposePosed) {
        std::memcpy(B.posedM[slot].data(), value, 16 * sizeof(double));
        return true;
    }
    const auto &frame = *static_cast<const rigExecRig::PointFrame *>(value);
    std::array<GfVec3d, 4> points;
    for (size_t i = 0; i < 4; ++i)
        points[i] =
            GfVec3d(frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    B.base[slot] = BakedPoseMath::MakeFrame(points, frame.flags);
    B.fin[slot] = B.base[slot];
    return true;
}

} // namespace

bool
RigExecBakedAddComposeOp(const RigExecBakedProgramImpl &B,
                         RigExecRigDescBuilder *builder,
                         eval::OpStepLayout *layout, size_t o,
                         std::string *error)
{
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    if (origin.family == RigExecBakedOpFamily::SpaceCheckpoint &&
        origin.object >= 0 &&
        size_t(origin.object) < B.switchFrameContexts.size())
        return AddCheckpointOp(B, builder, layout, o, uint32_t(origin.object),
                               error);
    if (origin.family == RigExecBakedOpFamily::PoseInterpolator &&
        origin.object >= 0 &&
        size_t(origin.object) < B.poseInterpolators.size())
        return AddInterpolatorOp(B, builder, layout, o, uint32_t(origin.object),
                                 error);
    if (origin.family == RigExecBakedOpFamily::RestCompose &&
        origin.object >= 0 && size_t(origin.object) < B.composeGroups.size())
        return AddRestOp(B, builder, layout, o, uint32_t(origin.object), error);
    if (origin.family == RigExecBakedOpFamily::LadderCompose &&
        origin.object >= 0 && size_t(origin.object) < B.composeGroups.size())
        return AddLadderOp(B, builder, layout, o, uint32_t(origin.object),
                           error);
    if (origin.family == RigExecBakedOpFamily::Compose && origin.object >= 0 &&
        size_t(origin.object) < B.composeGroups.size())
        return AddComposeGroupOp(B, builder, layout, o, size_t(origin.object),
                                 origin.part, error);
    return Fail(error, "node " + std::to_string(o) + " has no compose origin");
}

void
RigExecBakedPlanComposeOps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (size_t slot = 0; slot < B.paths.size(); ++slot) {
        RigExecBakedOpOrigin origin;
        origin.family = RigExecBakedOpFamily::AvarInputs;
        origin.kind = RigExecBakedStepKind::AvarInputs;
        origin.object = int32_t(slot);
        B.opNodeOrigins.push_back(origin);
    }
}

size_t
RigExecBakedComposeNodeCount(const RigExecBakedProgramImpl &B, size_t g)
{
    return ComposeNodesOf(B, g).Count();
}

int
RigExecBakedComposeNodePart(const RigExecBakedProgramImpl &B, size_t g,
                            size_t n)
{
    return ComposeNodesOf(B, g).Part(n);
}

bool
RigExecBakedAddComposeOps(const RigExecBakedProgramImpl &B,
                          RigExecRigDescBuilder *builder,
                          eval::OpStepLayout *layout, std::string *error)
{
    using namespace rigExecOps;
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o) {
        const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
        if (origin.family != RigExecBakedOpFamily::AvarInputs)
            continue;
        const uint32_t slot = uint32_t(origin.object);
        const AvarBindings bindings(B, slot);
        const unsigned channels = bindings.Channels();
        AvarInputsPayload payload;
        payload.avarCount = RigExecAvarElement(channels, kRigExecSlotAvars);
        payload.targets.reserve(bindings.Count());
        for (size_t k = 0; k < bindings.Count(); ++k)
            payload.targets.push_back(RigExecAvarElement(
                channels, bindings.At(k).slot % kRigExecSlotAvars));
        const RigExecNodeId node = builder->AddOp(AvarInputs(), &payload);
        if (node != o)
            return Fail(error, "the avar inputs of node " + std::to_string(o) +
                                   " were added as node " +
                                   std::to_string(node));
        builder->BindInput(node, AvarInputsValues());
        layout->hostInputs.push_back({node, AvarInputsValues().Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::AvarBindings), slot, -1, 0});
        builder->BindOutput(node, AvarInputsOut());
        layout->hostOutputs.push_back({node, AvarInputsOut().Index(),
                                       uint8_t(eval::OpOutputTarget::Avars),
                                       slot});
    }
    return true;
}

bool
RigExecBakedLookUpCompose(const RigExecBakedProgramImpl &B,
                          const eval::OpLookup &lookup,
                          const RigExecTypeDesc &type, void *value,
                          std::string *error)
{
    using L = eval::OpLookupSource;
    if (lookup.source == uint8_t(L::ComposeInputs)) {
        if (lookup.object < B.paths.size() &&
            eval::compose_lookups::LookUpComposeInput<BakedPoseMath>(
                BakedPoseInputs(&B), lookup.object, lookup.read, type, value))
            return true;
        return Fail(error, "slot " + std::to_string(lookup.object) +
                               " has no compose input " +
                               std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(L::SwitchInputs)) {
        if (lookup.object < B.spaceSwitches.size() &&
            eval::compose_lookups::LookUpSwitchInput<BakedPoseMath>(
                BakedPoseInputs(&B), lookup.object, lookup.read, type, value))
            return true;
        return Fail(error, "space switch " + std::to_string(lookup.object) +
                               " has no input " + std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(L::ClavicleInputs)) {
        if (lookup.object < B.autoClavicles.size() &&
            LookUpClavicle(B, lookup.object, lookup.read, type, value))
            return true;
        return Fail(error, "auto clavicle " + std::to_string(lookup.object) +
                               " has no input " + std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::InterpolatorInputs)) {
        if (lookup.object < B.poseInterpolators.size() &&
            LookUpInterpolator(B, lookup.object,
                               eval::OpInterpolatorRead(lookup.read), type,
                               value))
            return true;
        return Fail(error, "pose interpolator " +
                               std::to_string(lookup.object) +
                               " has no input " + std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::LadderInputs)) {
        if (lookup.object < B.composeGroups.size() &&
            LookUpLadder(B, lookup.object, eval::OpLadderRead(lookup.read),
                         type, value))
            return true;
        return Fail(error, "compose group " + std::to_string(lookup.object) +
                               " has no ladder input " +
                               std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::RestInputs)) {
        if (lookup.object < B.composeGroups.size() &&
            LookUpRest(B, lookup.object, eval::OpRestRead(lookup.read), type,
                       value))
            return true;
        return Fail(error, "compose group " + std::to_string(lookup.object) +
                               " has no rest input " +
                               std::to_string(lookup.read));
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::CheckpointInputs)) {
        if (lookup.object < B.switchFrameContexts.size() &&
            LookUpCheckpoint(B, lookup.object,
                             eval::OpCheckpointRead(lookup.read), type, value))
            return true;
        return Fail(error, "space checkpoint " + std::to_string(lookup.object) +
                               " has no input " + std::to_string(lookup.read));
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::AvarBindings) ||
        lookup.object >= B.paths.size() || !type.resize || !type.data)
        return Fail(error, "no avar bindings of slot " +
                               std::to_string(lookup.object));
    const AvarBindings bindings(B, lookup.object);
    type.resize(value, bindings.Count());
    auto *out = static_cast<double *>(type.data(value));
    for (size_t k = 0; k < bindings.Count(); ++k)
        out[k] = RigExecBakedLeafRead(B, bindings.At(k).input);
    return true;
}

bool
RigExecBakedStoreCompose(RigExecBakedProgramImpl &B, RigExecBakedStep *step,
                         const eval::OpHostOutput &output, const void *value,
                         std::string *error)
{
    if (output.target >= uint8_t(eval::OpOutputTarget::ComposeBase) &&
        output.target <= uint8_t(eval::OpOutputTarget::SpaceMatchStatus))
        return StoreComposeGroup(B, step, output, value, error);
    if (output.target == uint8_t(eval::OpOutputTarget::InterpolatorStatus)) {
        if (output.object >= B.poseInterpolators.size())
            return Fail(error, "no pose interpolator " +
                                   std::to_string(output.object));
        const std::string line = RigExecPoseInterpolatorStatusLine(
            RigExecPoseInterpolatorStatus(
                *static_cast<const uint32_t *>(value)),
            B.poseInterpolators[output.object].pathText, [&] {
                const int d = B.poseInterpolators[output.object].driverSlot;
                return d >= 0 ? (*B.pathTexts)[size_t(d)] : std::string();
            });
        if (!line.empty()) {
            if (!step)
                return Fail(error, "stores a pose interpolator status outside "
                                   "a step");
            step->diagnostics.push_back(line);
        }
        return true;
    }
    if (output.target == uint8_t(eval::OpOutputTarget::PoseWeights)) {
        const auto &weights =
            *static_cast<const rigExecStd::RigExecArray<float> *>(value);
        if (output.object >= B.poseInterpolators.size() ||
            weights.Size() !=
                B.poseInterpolators[output.object].poseSlots.size())
            return Fail(error, "pose interpolator " +
                                   std::to_string(output.object) +
                                   " stores no " +
                                   std::to_string(weights.Size()) + " weights");
        const auto &interp = B.poseInterpolators[output.object];
        for (int slot = interp.weightBegin; slot < interp.weightEnd; ++slot)
            B.poseWeights[size_t(slot)] = 0.0f;
        for (size_t i = 0; i < interp.poseSlots.size(); ++i)
            B.poseWeights[size_t(interp.poseSlots[i])] = weights[i];
        for (size_t j = 0; j < interp.primvarSlots.size(); ++j)
            B.poseWeights[size_t(interp.primvarSlots[j])] =
                B.poseWeights[size_t(
                    interp.poseSlots[size_t(interp.primvarSources[j])])];
        return true;
    }
    if (output.target == uint8_t(eval::OpOutputTarget::Ladder)) {
        if (output.object < B.composeGroups.size() &&
            StoreLadder(B, output.object, output.port, value))
            return true;
        return Fail(error, "compose group " + std::to_string(output.object) +
                               " stores no ladder port " +
                               std::to_string(output.port));
    }
    if (output.target == uint8_t(eval::OpOutputTarget::Rest)) {
        if (output.object < B.composeGroups.size() &&
            StoreRest(B, output.object, output.port, value))
            return true;
        return Fail(error, "compose group " + std::to_string(output.object) +
                               " stores no rest port " +
                               std::to_string(output.port));
    }
    if (output.target == uint8_t(eval::OpOutputTarget::SwitchFrame)) {
        if (output.object >= B.switchFrames.size())
            return Fail(error,
                        "no space checkpoint " + std::to_string(output.object));
        std::memcpy(B.switchFrames[output.object].data(), value,
                    16 * sizeof(double));
        return true;
    }
    const Doubles &values = *static_cast<const Doubles *>(value);
    const unsigned channels = AvarBindings(B, output.object).Channels();
    const size_t first = size_t(output.object) * kRigExecSlotAvars;
    if (output.target != uint8_t(eval::OpOutputTarget::Avars) ||
        first + kRigExecSlotAvars > B.avars.size() ||
        values.Size() != RigExecAvarElement(channels, kRigExecSlotAvars))
        return Fail(error, "slot " + std::to_string(output.object) +
                               " stores no " + std::to_string(values.Size()) +
                               " avars");
    size_t element = 0;
    for (size_t c = 0; c < kRigExecSlotAvars; ++c)
        if ((channels >> c) & 1u)
            B.avars[first + c] = values[element++];
    return true;
}

} // namespace rigExec
