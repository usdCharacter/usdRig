#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/poseEvalHost.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/avarScale.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"

#include <algorithm>
#include <cstring>
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
RigExecBakedStoreCompose(RigExecBakedProgramImpl &B,
                         const eval::OpHostOutput &output, const void *value,
                         std::string *error)
{
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
