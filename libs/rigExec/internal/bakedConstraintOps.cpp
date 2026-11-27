#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/poseEvalHost.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/constraint/harmonicsOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/constraint/matrixMoverOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"

#include <string>
#include <string_view>

namespace rigExec {

namespace {

using Constraint = RigExecBakedProgramImpl::Constraint;
using L = eval::OpLookupSource;
using R = eval::OpConstraintRead;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = why;
    return false;
}

const Constraint *
ConstraintAt(const RigExecBakedProgramImpl &B, size_t w)
{
    if (w >= B.walkSteps.size() || w >= B.commits.size() ||
        B.walkSteps[w].solverBatch || B.walkSteps[w].index < 0 ||
        size_t(B.walkSteps[w].index) >= B.constraints.size())
        return nullptr;
    return &B.constraints[size_t(B.walkSteps[w].index)];
}

} // namespace

const RigExecPortHandle &
RigExecBakedConstraintOut(const RigExecBakedProgramImpl &B, size_t w)
{
    using namespace rigExecOps;
    const Constraint &c = B.constraints[size_t(B.walkSteps[w].index)];
    const TfToken &type = c.type;
    if (c.harmonics)
        return HarmonicsOut();
    if (type == SchemaTypeName::RigExecMatrixMover)
        return MatrixMoverOut();
    if (type == SchemaTypeName::RigExecPositionConstraint)
        return PositionConstraintOut();
    if (type == SchemaTypeName::RigExecRotationConstraint)
        return RotationConstraintOut();
    if (type == SchemaTypeName::RigExecScaleConstraint)
        return ScaleConstraintOut();
    if (type == SchemaTypeName::RigExecParentConstraint)
        return ParentConstraintOut();
    return AimConstraintOut();
}

bool
RigExecBakedConstraintRunsAsOps(const RigExecBakedProgramImpl &B, size_t w)
{
    const Constraint *c = ConstraintAt(B, w);
    return c && (c->target >= 0 || c->ikFkMatch);
}

bool
RigExecBakedConstraintStoresPresence(const RigExecBakedProgramImpl &B, size_t w)
{
    const Constraint *c = ConstraintAt(B, w);
    return c && c->harmonics;
}

bool
RigExecBakedConstraintStoresFrames(const RigExecBakedProgramImpl &B, size_t w)
{
    const Constraint *c = ConstraintAt(B, w);
    return c && c->singleChainIk;
}

bool
RigExecBakedAddConstraintOp(const RigExecBakedProgramImpl &B,
                            RigExecRigDescBuilder *builder,
                            eval::OpStepLayout *layout, size_t o,
                            std::string *error)
{
    using namespace rigExecOps;
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    const size_t w = size_t(origin.object);
    const Constraint *found = ConstraintAt(B, w);
    if (!found || !RigExecBakedConstraintRunsAsOps(B, w))
        return Fail(error, "op nodes: walk step " + std::to_string(w) +
                               " commits no constraint its op nodes solve");
    const Constraint &c = *found;
    const RigExecBakedCommit &commit = B.commits[w];
    const auto look = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          L source, int32_t part, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(source), uint32_t(w), part, uint8_t(read)});
    };
    const auto axes = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          R first) {
        builder->BindInput(node, port);
        const uint32_t list = uint32_t(layout->lookups.size());
        layout->hostInputs.push_back(
            {node, port.Index(), eval::OpHostSource::Lookup, list});
        layout->lookups.push_back({uint8_t(L::List), list + 1, 3, 0});
        for (uint8_t a = 0; a < 3; ++a)
            layout->lookups.push_back({uint8_t(L::ConstraintLeaf), uint32_t(w),
                                       -1, uint8_t(uint8_t(first) + a)});
    };
    const auto entering = [&](RigExecNodeId node,
                              const RigExecPortHandle &port) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(L::PoseVersion), commit.targetRead, 0, 0});
    };
    if (origin.part == 0) {
        if (c.deltaBase < 0 || o == 0 ||
            B.opNodeOrigins[o - 1].family != RigExecBakedOpFamily::Constraint ||
            size_t(B.opNodeOrigins[o - 1].object) != w)
            return Fail(error, "op nodes: " + c.pathText +
                                   " measures a delta it does not solve for");
        const RigExecNodeId node = builder->AddOp(ConstraintDelta(), nullptr);
        if (node != o)
            return Fail(error, "op nodes: the constraint delta of node " +
                                   std::to_string(o) + " was added as node " +
                                   std::to_string(node));
        builder->Connect(RigExecNodeId(o - 1), RigExecBakedConstraintOut(B, w),
                         node, ConstraintDeltaSolved());
        look(node, ConstraintDeltaBase(), L::ConstraintDeltaBase, -1, R(0));
        look(node, ConstraintDeltaBaseOk(), L::ConstraintDeltaBase, -1, R(1));
        look(node, ConstraintDeltaWeight(), L::ConstraintEnvelope, -1,
             R::DeltaWeight);
        builder->BindOutput(node, ConstraintDeltaDelta());
        builder->BindOutput(node, ConstraintDeltaMeasured());
        layout->hostOutputs.push_back(
            {node, ConstraintDeltaDelta().Index(),
             uint8_t(eval::OpOutputTarget::ConstraintDelta), uint32_t(w)});
        layout->hostOutputs.push_back(
            {node, ConstraintDeltaMeasured().Index(),
             uint8_t(eval::OpOutputTarget::ConstraintDeltaMeasured),
             uint32_t(w)});
        return true;
    }
    RigExecNodeId node = RigExecNodeId(kRigExecInvalidIndex);
    const RigExecPortHandle *out = nullptr;
    const auto sources = [&](const RigExecPortHandle &frames,
                             const RigExecPortHandle &weights) {
        look(node, frames, L::ConstraintSources, -1, R(0));
        look(node, weights, L::ConstraintArrays, -1, R::SourceWeights);
    };
    ConstraintPayload payload;
    payload.rotationOrder = ConstraintEulerOrder(int(c.order));
    payload.blendShear = c.blendShear;
    payload.preserveInputUp = c.preserveInputUp;
    if (c.ikFkMatch) {
        using M = eval::OpIkFkMatchRead;
        static_assert(eval::constraint_lookups::kMatchChannelStride ==
                              kMatchChannelStride &&
                          kIkFkMatchMatched == 1u << 5,
                      "the host reads IkFkMatch's channels and status");
        if (c.sources.size() != 10)
            return Fail(error,
                        c.pathText + ": names an incomplete IK/FK match");
        IkFkMatchPayload match;
        match.ikValue = c.matchIkValue;
        match.scaleCalibration = c.matchScaleCalibration;
        match.upperLengthOffset = c.matchUpperOffset;
        match.lowerLengthOffset = c.matchLowerOffset;
        match.stretchLimb = c.matchStretchLimb;
        if (c.matchPoleSwitch >= 0) {
            const auto &sw = B.spaceSwitches[size_t(c.matchPoleSwitch)];
            const RigExecSpaceSwitchRecord &record = sw.kernelRecord;
            match.poleSwitch.filters = record.filters;
            match.poleSwitch.twistAxis = rigExecStd::Vec3d(
                record.twistAxis[0], record.twistAxis[1], record.twistAxis[2]);
            match.poleSwitch.affectTranslation = record.affectTranslation;
            match.poleSwitch.affectRotation = record.affectRotation;
            match.poleSwitch.affectScale = record.affectScale;
            match.poleSwitch.hasCarry = sw.spaceSlot >= 0;
            for (const int slot : sw.sourceSlots)
                match.poleSwitch.world.push_back(slot < 0 ? 1 : 0);
            match.poleSourceRides.assign(c.matchPoleSourceRides.begin(),
                                         c.matchPoleSourceRides.end());
        }
        node = builder->AddOp(IkFkMatch(), &match);
        if (node != o)
            return Fail(error, "op nodes: the IK/FK match of node " +
                                   std::to_string(o) + " was added as node " +
                                   std::to_string(node));
        const auto input = [&](const RigExecPortHandle &port, M read) {
            look(node, port, L::IkFkMatchInput, -1, R(uint8_t(read)));
        };
        input(IkFkMatchEnabled(), M::Readable);
        input(IkFkMatchWeight(), M::Weight);
        input(IkFkMatchSwitch(), M::Switch);
        input(IkFkMatchStretch(), M::Stretch);
        input(IkFkMatchSoftDistance(), M::SoftDistance);
        input(IkFkMatchUpperScale(), M::UpperScale);
        input(IkFkMatchLowerScale(), M::LowerScale);
        input(IkFkMatchPin(), M::Pin);
        input(IkFkMatchTwist(), M::Twist);
        input(IkFkMatchSources(), M::Sources);
        input(IkFkMatchRests(), M::Rests);
        input(IkFkMatchChannels(), M::Channels);
        input(IkFkMatchHasSpace(), M::HasSpace);
        input(IkFkMatchSpaceRest(), M::SpaceRest);
        input(IkFkMatchSpacePosed(), M::SpacePosed);
        input(IkFkMatchPoleMode(), M::PoleMode);
        input(IkFkMatchPosedDefault(), M::PolePosedDefault);
        input(IkFkMatchParentDefaultInverse(), M::PoleParentDefaultInverse);
        input(IkFkMatchParentPosed(), M::PoleParentPosed);
        input(IkFkMatchParentDefault(), M::PoleParentDefault);
        if (match.poleSwitch.hasCarry) {
            input(IkFkMatchSwitchSpaceDefault(), M::PoleSpaceDefault);
            input(IkFkMatchSwitchSpacePosed(), M::PoleSpacePosed);
        }
        input(IkFkMatchSourceDefaults(), M::PoleSourceDefaults);
        input(IkFkMatchSourcePosed(), M::PoleSourcePosed);
        input(IkFkMatchActive(), M::PoleActive);
        const auto store = [&](const RigExecPortHandle &port,
                               eval::OpOutputTarget target) {
            builder->BindOutput(node, port);
            layout->hostOutputs.push_back(
                {node, port.Index(), uint8_t(target), uint32_t(w)});
        };
        store(IkFkMatchShownMask(), eval::OpOutputTarget::ConstraintStatus);
        store(IkFkMatchIkValues(), eval::OpOutputTarget::IkFkMatchIkValues);
        store(IkFkMatchFkValues(), eval::OpOutputTarget::IkFkMatchFkValues);
        store(IkFkMatchShown(), eval::OpOutputTarget::IkFkMatchShown);
        return true;
    }
    if (c.singleChainIk) {
        using S = eval::OpSingleChainIkRead;
        const bool rotatePlane =
            c.ikMode == RigExecSingleChainIkMode::RotatePlane;
        if (rotatePlane && c.poleModeObject && c.poleObjects.empty())
            return Fail(error, c.pathText + ": uses object pole mode with no "
                                            "pole-vector objects");
        SingleChainIkPayload ik;
        ik.singleChain = c.ikMode == RigExecSingleChainIkMode::SingleChain;
        ik.preserveJointOrientation = c.preserveJointOrientation;
        node = builder->AddOp(SingleChainIk(), &ik);
        if (node != o)
            return Fail(error, "op nodes: the single-chain IK of node " +
                                   std::to_string(o) + " was added as node " +
                                   std::to_string(node));
        const auto input = [&](const RigExecPortHandle &port, S read) {
            look(node, port, L::SingleChainIkInput, -1, R(uint8_t(read)));
        };
        input(SingleChainIkChain(), S::Chain);
        if (!c.useAnimatedTs)
            input(SingleChainIkRests(), S::Rests);
        input(SingleChainIkEffector(), S::Effector);
        if (rotatePlane) {
            input(SingleChainIkPole(), S::Pole);
            input(SingleChainIkTwist(), S::Twist);
        }
        look(node, SingleChainIkWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        input(SingleChainIkStretch(), S::Stretch);
        builder->BindOutput(node, SingleChainIkStatus());
        builder->BindOutput(node, SingleChainIkOut());
        layout->hostOutputs.push_back(
            {node, SingleChainIkStatus().Index(),
             uint8_t(eval::OpOutputTarget::ConstraintStatus), uint32_t(w)});
        layout->hostOutputs.push_back(
            {node, SingleChainIkOut().Index(),
             uint8_t(eval::OpOutputTarget::SingleChainIkFrames), uint32_t(w)});
        return true;
    }
    if (c.harmonics) {
        using H = eval::OpHarmonicsRead;
        if (c.window < 0 || size_t(c.window) >= B.windowedOps.size())
            return Fail(error, c.pathText + ": has no history window");
        HarmonicsPayload harmonics;
        harmonics.settings = c.harmonicsSettings;
        harmonics.matchVelocity = c.harmonicsMatchVelocity;
        harmonics.hasSpace = c.spaceSlot >= 0;
        harmonics.frames = B.windowedOps[size_t(c.window)].frames;
        node = builder->AddOp(Harmonics(), &harmonics);
        if (node != o)
            return Fail(error, "op nodes: the harmonics of node " +
                                   std::to_string(o) + " was added as node " +
                                   std::to_string(node));
        const auto input = [&](const RigExecPortHandle &port, H read) {
            look(node, port, L::HarmonicsInput, -1, R(uint8_t(read)));
        };
        entering(node, HarmonicsIn());
        if (c.spaceSlot >= 0)
            input(HarmonicsSpace(), H::Space);
        input(HarmonicsEnabled(), H::Open);
        look(node, HarmonicsWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        input(HarmonicsAmplitude(), H::Amplitude);
        input(HarmonicsDecay(), H::Decay);
        input(HarmonicsTermination(), H::Termination);
        input(HarmonicsAxisAmplitude(), H::AxisAmplitude);
        builder->BindOutput(node, HarmonicsStatus());
        layout->hostOutputs.push_back(
            {node, HarmonicsStatus().Index(),
             uint8_t(eval::OpOutputTarget::ConstraintStatus), uint32_t(w)});
        return true;
    }
    if (c.type == SchemaTypeName::RigExecMatrixMover) {
        if (c.sources.empty())
            return Fail(error, c.pathText +
                                   ": a MatrixMover with no rigExec:transform "
                                   "source");
        MatrixMoverPayload mover;
        mover.radialBlend = c.radialBlend;
        node = builder->AddOp(MatrixMover(), &mover);
        entering(node, MatrixMoverIn());
        look(node, MatrixMoverTransform(), L::ConstraintSources, 0, R(0));
        if (c.sources[0] >= 0)
            look(node, MatrixMoverTransformRest(), L::ConstraintSources, 0,
                 R(1));
        if (c.sources.size() > 1) {
            look(node, MatrixMoverSpace(), L::ConstraintSources, 1, R(0));
            if (c.sources[1] >= 0)
                look(node, MatrixMoverSpaceRest(), L::ConstraintSources, 1,
                     R(1));
        }
        look(node, MatrixMoverWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        out = &MatrixMoverOut();
    } else if (c.type == SchemaTypeName::RigExecPositionConstraint) {
        node = builder->AddOp(PositionConstraint(), &payload);
        entering(node, PositionConstraintIn());
        sources(PositionConstraintSources(), PositionConstraintSourceWeights());
        look(node, PositionConstraintWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        look(node, PositionConstraintOffset(), L::ConstraintLeaf, -1,
             R::Offset);
        axes(node, PositionConstraintAffect(), R::AffectX);
        out = &PositionConstraintOut();
    } else if (c.type == SchemaTypeName::RigExecRotationConstraint) {
        node = builder->AddOp(RotationConstraint(), &payload);
        entering(node, RotationConstraintIn());
        sources(RotationConstraintSources(), RotationConstraintSourceWeights());
        look(node, RotationConstraintWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        if (c.spaceSlot >= 0)
            look(node, RotationConstraintSpace(), L::ConstraintSpace, -1, R(0));
        look(node, RotationConstraintOffset(), L::ConstraintLeaf, -1,
             R::Offset);
        axes(node, RotationConstraintAffect(), R::AffectX);
        out = &RotationConstraintOut();
    } else if (c.type == SchemaTypeName::RigExecScaleConstraint) {
        node = builder->AddOp(ScaleConstraint(), &payload);
        entering(node, ScaleConstraintIn());
        sources(ScaleConstraintSources(), ScaleConstraintSourceWeights());
        look(node, ScaleConstraintWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        look(node, ScaleConstraintOffset(), L::ConstraintLeaf, -1, R::Offset);
        axes(node, ScaleConstraintAffect(), R::AffectX);
        out = &ScaleConstraintOut();
    } else if (c.type == SchemaTypeName::RigExecParentConstraint) {
        node = builder->AddOp(ParentConstraint(), &payload);
        entering(node, ParentConstraintIn());
        sources(ParentConstraintSources(), ParentConstraintSourceWeights());
        look(node, ParentConstraintWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        if (c.spaceSlot >= 0)
            look(node, ParentConstraintSpace(), L::ConstraintSpace, -1, R(0));
        axes(node, ParentConstraintTranslationAxes(), R::TranslateX);
        axes(node, ParentConstraintRotationAxes(), R::RotateX);
        axes(node, ParentConstraintScaleAxes(), R::ScaleX);
        look(node, ParentConstraintTranslationOffsets(), L::ConstraintArrays,
             -1, R::TranslationOffsets);
        look(node, ParentConstraintRotationOffsets(), L::ConstraintArrays, -1,
             R::RotationOffsets);
        out = &ParentConstraintOut();
    } else {
        node = builder->AddOp(AimConstraint(), &payload);
        entering(node, AimConstraintIn());
        sources(AimConstraintSources(), AimConstraintSourceWeights());
        look(node, AimConstraintWeight(), L::ConstraintEnvelope, -1,
             R::SolveWeight);
        look(node, AimConstraintOffset(), L::ConstraintLeaf, -1,
             R::RotationOffset);
        axes(node, AimConstraintAffect(), R::AffectX);
        look(node, AimConstraintAimVector(), L::ConstraintLeaf, -1,
             R::AimVector);
        look(node, AimConstraintUpVector(), L::ConstraintLeaf, -1, R::UpVector);
        const std::string_view type(c.worldUpType.GetText(),
                                    c.worldUpType.size());
        using U = eval::OpConstraintWorldUp;
        const U mode = type == "sceneUp"            ? U::Scene
                       : type == "vector"           ? U::Vector
                       : type == "objectUp"         ? U::Object
                       : type == "objectRotationUp" ? U::ObjectRotation
                                                    : U::None;
        if (mode != U::None)
            look(node, AimConstraintWorldUp(), L::ConstraintWorldUp,
                 int32_t(mode), R(0));
        out = &AimConstraintOut();
    }
    if (node != o)
        return Fail(error, "op nodes: the constraint of node " +
                               std::to_string(o) + " was added as node " +
                               std::to_string(node));
    builder->BindOutput(node, *out);
    layout->hostOutputs.push_back(
        {node, out->Index(), uint8_t(eval::OpOutputTarget::ConstraintCandidate),
         uint32_t(w)});
    return true;
}

bool
RigExecBakedLookUpConstraint(RigExecBakedProgramImpl &B,
                             eval::ConstraintStepCache &cache,
                             const eval::OpLookup &lookup,
                             const RigExecTypeDesc &type, void *value,
                             std::string *error)
{
    BakedPoseHost host(&B, nullptr);
    return eval::LookUpConstraint(host, cache, lookup, type, value, error);
}

bool
RigExecBakedStoreConstraint(RigExecBakedProgramImpl &B, RigExecBakedStep *step,
                            eval::ConstraintStepCache &cache,
                            const eval::OpHostOutput &output, const void *value,
                            std::string *error)
{
    if (!step)
        return Fail(error, "stores a constraint result outside a step");
    BakedPoseHost host(&B, step);
    return eval::StoreConstraint(host, cache, output, value, error);
}

bool
RigExecBakedLendConstraintWindow(RigExecBakedProgramImpl &B,
                                 RigExecRunState &run, uint32_t node,
                                 std::string *error)
{
    const Constraint *c =
        node < B.opNodeOrigins.size() &&
                B.opNodeOrigins[node].family ==
                    RigExecBakedOpFamily::Constraint &&
                B.opNodeOrigins[node].object >= 0
            ? ConstraintAt(B, size_t(B.opNodeOrigins[node].object))
            : nullptr;
    if (!c || c->window < 0 || size_t(c->window) >= B.windowedOps.size() ||
        size_t(c->window) >= B.windowRings.size() ||
        size_t(c->window) >= B.windowCaptures.size())
        return Fail(error, "op nodes: node " + std::to_string(node) +
                               " keeps no history window");
    const size_t w = size_t(c->window);
    return run.LendWindow(
        RigExecNodeId(node), B.windowRings[w], B.windowCaptures[w],
        B.historyTick && !B.windowedOps[w].feedsWindow, error);
}

std::string
RigExecBakedConstraintFailure(const RigExecBakedProgramImpl &B, int w,
                              const std::string &error)
{
    const Constraint *c = w >= 0 ? ConstraintAt(B, size_t(w)) : nullptr;
    return c ? c->pathText + ": " + error : error;
}

} // namespace rigExec
