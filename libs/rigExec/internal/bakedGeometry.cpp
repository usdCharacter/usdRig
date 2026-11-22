#include "rigExecSchema/schemaNames.h"
#include "rigExecScene/geometryProgram.h"
#include "rigExecMath/blendLayoutKernel.h"
#include "rigExecMath/blendRangeKernel.h"
#include "rigExec/internal/weightField.h"
#include "rigExec/internal/gpuCloth.h"
// The baked program's geometry half: the chain/revision bake and the frame
// path that runs those revisions and the derived maintenance behind them.
// Geometry-domain bake and frame path; must agree on captured vs re-read
// state. The evaluator is not visible here -- the compiled chains arrive
// restated as RigExecBakedChainSpec, and the caches the frame path shares
// with the dynamic walk were captured into RigExecBakedProgramImpl at Build.
#include "rigExec/internal/bakedOpValues.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/bakedSchedule.h"

#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/api/types.h"
#include "frameExtraction.h"
#include "moverGraph.h"
#include "parallel.h"
#include "pathText.h"
#include "rigExecEvalCore/chain.h"
#include "rigExecEvalCore/envelope.h"
#include "rigExecEvalCore/gate.h"
#include "rigExecEvalCore/skin.h"

#include "rigExecMath/dualQuat.h"
#include "rigExecMath/skinAcceptance.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/pointRanges.h"
#include "rigExecMath/simdKernels.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/usdSkel/blendShape.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// The two names a step body needs a TOKEN for rather than a comparison: the
// status a failed revision publishes, and the attribute the defaultWeight
// diagnostic re-reads. Hoisted because TfToken(const char *) takes the token
// registry's spin lock on every construction, and a step body may take no
// lock (libs/rigExec/internal/bodyPurity.h).
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((moverFailed, "moverFailed"))
    ((defaultWeight, rigExec::SchemaAttrText::inputsDefaultWeight))
    ((classicLinear, "classicLinear"))
    ((dualQuaternion, "dualQuaternion"))
    ((jointIndices, rigExec::SchemaAttrText::rigExecJointIndices))
    ((elementSize, rigExec::SchemaAttrText::rigExecElementSize))
    ((denseRepresentation, rigExec::SchemaValueText::dense))
    ((sparseRepresentation, rigExec::SchemaValueText::sparse))
    ((staticWeight, rigExec::SchemaTypeName::RigExecStaticWeight))
    ((weightDefault, rigExec::SchemaAttrText::rigExecDefaultWeight))
    ((targetSpace, "target"))
    ((operationCycle, "operation cycle"))
    ((legacyLattice, "legacy"))
    ((rigExecJointWeights, rigExec::SchemaAttrText::rigExecJointWeights))
    ((rigExecSkinningMethod, rigExec::SchemaAttrText::rigExecSkinningMethod))
    ((inputsWeight, rigExec::SchemaAttrText::inputsWeight))
    ((rigExecActivation, rigExec::SchemaAttrText::rigExecActivation))
    (offsets)
    (pointIndices)
    ((rigExecWeightTarget, rigExec::SchemaAttrText::rigExecWeightTarget))
);

namespace rigExec {

void
RigExecBakedGeometryTouchTokens()
{
    (void)_tokens.Get();
}

// Bake.

namespace {

/// Binds every phased point read of the program to the chain versions the
/// dynamic walk answers it from (RigExecBakedPointsBinding): its phased-read
/// store, or for `preceding` on the reader's own chain the points entering
/// the reader.
///
/// "Before the reader" is program order, which RigExecBakedBuildGeometrySteps
/// emits chain by chain in `chains` order: every step of an earlier chain,
/// and on the reader's own chain the fuses of the revisions before it -- or,
/// for a derived reader, every fuse and the chain's status step. The walk
/// records a revision only where `snapshotAfter` is set and a chain's final
/// after its status sweep, and both only when the chain read a base.
void
BindPointReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedCaptureCrossDomainOrder(&B);
    int nextId = 0;
    const auto bind = [&](const SdfPath &input, const RigExecReadPhase &phase,
                          const SdfPath &reader, size_t, size_t, bool) {
        auto out = RigExecBakedBindPointInput(B, input, phase, reader);
        out.id = nextId++;
        return out;
    };
    const auto bindRevision =
        [&](RigExecBakedProgramImpl::GeomRevision *revision, size_t chain,
            size_t before, bool afterStatus) {
            revision->pointBindings.clear();
            for (const auto &[input, phase] : revision->binding.phases) {
                RigExecBakedPointsBinding bound =
                    bind(input, phase, revision->moverPath, chain, before,
                         afterStatus);
                bound.diagnoseMiss =
                    phase.kind != RigExecReadPhaseKind::Preceding;
                if (bound.diagnoseMiss) {
                    bound.missDiagnostic =
                        "diag " + revision->moverPath.GetString() +
                        ": read phase '" + bound.phase.GetAsString() +
                        "' for " + bound.input.GetString() +
                        " resolved to nothing; read the authored base";
                }
                revision->pointBindings.push_back(std::move(bound));
            }
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision->blendChannels) {
                for (auto &sample : channel.samples) {
                    sample.pointBinding = RigExecBakedPointsBinding();
                    if (sample.phase.IsBase() || !sample.blendShape.IsEmpty()) {
                        continue;
                    }
                    sample.pointBinding =
                        bind(sample.pointsPath, sample.phase,
                             revision->moverPath, chain, before, afterStatus);
                }
            }
        };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            bindRevision(&chain.revisions[r], c, r, false);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            bindRevision(&derived.revision, c, chain.revisions.size(), true);
        }
    }
    B.pointBindingCount = nextId;
}

} // namespace

void
RigExecBakedBuildGeometry(RigExecBakedBuildContext *ctx,
                          const std::vector<RigExecBakedChainSpec> &chains)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx->Refuse(what, where);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx->SlotOf(path); };
    RigExecPathText pathText;

    auto bakeRevision = [&](const RigExecBakedRevisionSpec &r) {
        RigExecBakedProgramImpl::GeomRevision out;
        out.moverPath = r.moverPath;
        out.moverPathText = pathText(r.moverPath);
        out.target = r.target;
        out.moverPrim = B.stage->GetPrimAtPath(r.moverPath);
        out.op = r.op;
        out.binding = r.binding;
        out.finalPhase = r.transformFinalPhase;
        out.posedPoints = r.transformPosedPoints;
        out.skinTopologyFixed = r.skinTopologyFixed;
        out.snapshotAfter = r.snapshotAfter;
        // The geometry-domain constraint whose measured delta IS this
        // revision's transform. The join key is the MOVER path, which is
        // what the dynamic walk's own constraintDeltas map is keyed by; the
        // pose half is baked first, so the constraint is already here.
        for (const RigExecBakedProgramImpl::Constraint &constraint :
             B.constraints) {
            if (constraint.deltaBase >= 0 && constraint.path == r.moverPath) {
                out.constraintDelta = constraint.deltaBase;
                break;
            }
        }
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // A windowed external mover (windowHistory.h): its chunk body
        // captures and solves over the window the handler names.
        if (r.op == RigExecRevisionOp::External && out.moverPrim) {
            const RigExecMoverHandler *handler =
                RigExecFindMoverHandler(out.moverPrim.GetTypeName());
            const uint32_t frames = handler && handler->windowFrames
                                        ? handler->windowFrames(out.moverPrim)
                                        : 0;
            if (frames > 0) {
                RigExecBakedProgramImpl::WindowedOp window;
                window.frames = frames;
                window.path = r.moverPath;
                out.window = int(B.windowedOps.size());
                B.windowedOps.push_back(std::move(window));
            }
        }
        // The packet is assembled live from this prim every frame through the
        // generation's resolved inputs: nothing of it is captured, so a value
        // edit needs no rebuild and an override places itself. The layout
        // arrays are still NAMED: IsBakeable requires unconnected layout
        // arrays; connecting one resyncs. The weights do not belong in
        // `rebuild`: the SkinTopology op holds the layout, and its layout
        // leaves are filed under these paths, so a weight-paint edit reaches
        // the next generation through the re-read rather than through a rebuild
        // of this program. The indices and element size are topology, and go to
        // `rebuild` with the other topology reads below.
        B.prims.insert(r.moverPath);
        for (const SdfPath &structural : r.binding.externalStructure) {
            if (structural.IsPropertyPath())
                B.named.insert(structural);
            B.prims.insert(structural.GetPrimPath());
        }
        B.resolvedRoutedPrims.insert(r.moverPath);
        for (const TfToken &name :
             {_tokens->jointIndices, _tokens->rigExecJointWeights,
              _tokens->elementSize, _tokens->rigExecSkinningMethod}) {
            B.named.insert(r.moverPath.AppendProperty(name));
        }
        // Every path an operation's per-frame assembler reads, whatever the
        // operation is: the cage a lattice deforms through, the surface a
        // projection lands on, the topology a smooth walks, the bind
        // coordinates a ribbon rides. None of them is folded into the
        // program -- they are re-read on every frame through the
        // generation's resolved inputs -- so they belong in `named`, where a
        // RESYNC (the property appearing, disappearing or being retargeted,
        // which also invalidates any retained query) finds them, and not in
        // `rebuild`, which is for values the program captured -- except the
        // topology among them, which is compiled rig state (below).
        // Named unconditionally rather than per operation: the binding
        // carries exactly the paths the operation resolved, so an empty one
        // is an operation that does not read it and an authored one is a
        // path some assembler will ask for.
        const auto name = [&](const SdfPath &path) {
            if (path.IsEmpty()) {
                return;
            }
            B.named.insert(path);
            B.prims.insert(path.GetPrimPath());
        };
        name(r.binding.base);
        name(r.binding.topologyCounts);
        name(r.binding.topologyIndices);
        name(r.binding.cagePoints);
        name(r.binding.surfacePoints);
        name(r.binding.bindCoords);
        name(r.binding.driverCurvePoints);
        name(r.binding.driverCurveOrder);
        name(r.binding.driverCurveKnots);
        name(r.binding.widths);
        // A phased input reads the chain version it is bound to when a
        // candidate answers and the resolved input -- the stage -- otherwise,
        // so the path is one the bake asked about either way.
        for (const auto &[input, phase] : r.binding.phases) {
            name(input);
        }
        // The blend channels, in the order the accumulation is defined over.
        // Only the HANDLES are captured: every channel weight, every
        // activation and every target shape is re-read per frame through the
        // generation's resolved inputs, which is what lets an animator drag a
        // channel weight and see it -- so the input and sample prims go into
        // `resolvedRoutedPrims` beside the mover's, where an override on one
        // of their properties places with nothing else to do.
        for (const SdfPath &input : r.binding.blendInputs) {
            RigExecBakedProgramImpl::GeomBlendChannel channel;
            B.prims.insert(input);
            B.resolvedRoutedPrims.insert(input);
            const SdfPath weightPath =
                input.AppendProperty(_tokens->inputsWeight);
            B.named.insert(weightPath);
            channel.weight = B.stage->GetAttributeAtPath(weightPath);
            channel.weightPath = weightPath;
            // A weight connected to a pose interpolator's output reads the
            // interpolator's slot. The walk is the one RigExecResolvedInputs::
            // GetAttribute makes -- single authored connections, followed
            // until one lands on a property the program publishes -- so the
            // slot answers exactly where the dynamic walk's in-memory lookup
            // would have.
            {
                std::set<SdfPath> visiting;
                UsdAttribute a = channel.weight;
                while (a && visiting.insert(a.GetPath()).second) {
                    const auto found = B.poseWeightIndex.find(a.GetPath());
                    if (found != B.poseWeightIndex.end()) {
                        channel.poseWeight = found->second;
                        break;
                    }
                    SdfPathVector connections;
                    if (a.HasAuthoredConnections()) {
                        a.GetConnections(&connections);
                    }
                    if (connections.size() != 1) {
                        break;
                    }
                    a = B.stage->GetAttributeAtPath(connections[0]);
                }
            }
            const auto samples = r.binding.blendSamples.find(input);
            if (samples != r.binding.blendSamples.end()) {
                for (const RigExecBlendSampleBinding &binding :
                     samples->second) {
                    RigExecBakedProgramImpl::GeomBlendChannel::Sample sample;
                    B.prims.insert(binding.sample);
                    B.resolvedRoutedPrims.insert(binding.sample);
                    const SdfPath activationPath =
                        binding.sample.AppendProperty(
                            _tokens->rigExecActivation);
                    B.named.insert(activationPath);
                    sample.activation =
                        B.stage->GetAttributeAtPath(activationPath);
                    sample.activationPath = activationPath;
                    sample.samplePath = binding.sample;
                    sample.pointsPath = binding.points;
                    sample.phase = binding.phase;
                    sample.blendShape = binding.blendShape;
                    if (!sample.blendShape.IsEmpty()) {
                        const UsdSkelBlendShape shape(B.stage->GetPrimAtPath(
                            sample.blendShape.GetPrimPath()));
                        sample.shapeValid = bool(shape);
                        sample.layoutRefused =
                            shape &&
                            (shape.GetOffsetsAttr().HasAuthoredConnections() ||
                             shape.GetPointIndicesAttr()
                                 .HasAuthoredConnections());
                    }
                    if (binding.blendShape.IsEmpty()) {
                        name(binding.points);
                        sample.points =
                            B.stage->GetAttributeAtPath(binding.points);
                    } else {
                        // A sparse sample's shape is read from its leaves;
                        // what is named here is the shape prim, so a resync
                        // under it rebuilds. Its offsets and point indices
                        // are topology, so an edit to either rebuilds too
                        // (the compiled rig reads below).
                        B.prims.insert(binding.blendShape.GetPrimPath());
                    }
                    channel.samples.push_back(std::move(sample));
                }
            }
            out.blendChannels.push_back(std::move(channel));
        }
        if (!r.binding.transform.IsEmpty()) {
            out.transformSlot = slotOf(r.binding.transform);
            if (out.transformSlot < 0) {
                refuse("mover transform provider is not a pose provider",
                       r.binding.transform);
            }
        }
        if (!r.binding.transformSpace.IsEmpty()) {
            out.transformSpaceSlot = slotOf(r.binding.transformSpace);
            if (out.transformSpaceSlot < 0) {
                refuse("mover transform space is not a pose provider",
                       r.binding.transformSpace);
            }
        }
        // rigExec:space: the rig's carry. Refused on miss: an unresolved
        // carry must not become identity.
        if (!r.binding.carrySpace.IsEmpty()) {
            out.carrySpaceSlot = slotOf(r.binding.carrySpace);
            if (out.carrySpaceSlot < 0) {
                refuse("mover carry space is not a pose provider",
                       r.binding.carrySpace);
            }
        }
        // The solver whose aggregate supplies values.driverFrames, resolved
        // once here: the walk's solver table is built before the geometry
        // half, so the per-revision tap the dynamic path reads becomes an
        // index into it.
        if (!r.binding.driverFrames.IsEmpty()) {
            B.prims.insert(r.binding.driverFrames);
            const auto solver = B.solverIndex.find(r.binding.driverFrames);
            if (solver == B.solverIndex.end()) {
                // Compile admitted this revision against its aggregate
                // batch. A miss here is a translation correspondence failure,
                // not the optional-driver admission policy.
                refuse("admitted driver frames solver is absent from the "
                       "native table",
                       r.binding.driverFrames);
            } else {
                out.driverFramesSolver = solver->second;
            }
        }
        for (const SdfPath &influence : r.binding.influences) {
            const int slot = slotOf(influence);
            if (slot < 0)
                refuse("skin influence is not a provider", influence);
            out.influenceSlots.push_back(slot);
        }
        // The weight object, baked once however many revisions bind it: what
        // this revision holds is its index in the shared table, and the
        // packet itself is built once per frame by that object's own step.
        out.weightObject =
            RigExecBakedBakeWeightObject(ctx, r.binding.weightObject);
        if (out.weightObject >= 0) {
            // WHAT the published field says it weights, decided here because
            // it is a relationship read and a relationship is compiled rig state.
            // The condition is the dynamic path's, verbatim: exactly one
            // declared target, and it is this mover, means the field weights
            // the OPERATION and carries one element; anything else weights
            // the chain's points.
            const UsdPrim weightPrim =
                B.stage->GetPrimAtPath(r.binding.weightObject);
            const SdfPathVector declared =
                ctx->Targets(weightPrim, _tokens->rigExecWeightTarget);
            out.weightOperationDomain =
                declared.size() == 1 && declared[0] == r.moverPath;
            out.weightFieldTarget =
                out.weightOperationDomain ? r.moverPath : r.target;
            out.weightCurrentPhase =
                B.currentPhaseWeights.count(r.binding.weightObject) > 0;
        }
        // The packet assembly's reads as path leaves, sampled in the
        // prologue: the assembler's, for an operation the leaves assemble,
        // and the blend gather's, which reads the generation's resolved
        // inputs and not the phase overlay.
        RigExecRevisionLeafDecl &decl = out.leaves.decl;
        RigExecDeclareRevisionLeaves(out.op, out.moverPath, out.binding, &decl);
        if (decl.assembles) {
            using Type = RigExecRevisionLeafType;
            using Flavour = RigExecRevisionLeafFlavour;
            const RigExecRevisionLeafTime at = RigExecRevisionLeafTime::AtTime;
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 out.blendChannels) {
                channel.weightHeldLeaf =
                    decl.Add({channel.weightPath, Type::Bool, at,
                              Flavour::Present, VtValue(false)});
                channel.weightLeaf = decl.Add(
                    {channel.weightPath, Type::Float, at, Flavour::ResolvedOnly,
                     VtValue(RigExecBlendChannel().weight)});
                for (auto &sample : channel.samples) {
                    sample.activationLeaf = decl.Add(
                        {sample.activationPath, Type::Float, at,
                         Flavour::ResolvedOnly,
                         VtValue(RigExecBlendSampleData().activation)});
                    if (sample.blendShape.IsEmpty()) {
                        sample.pointsLeaf = decl.Add(
                            {sample.pointsPath, Type::Vec3fArray, at,
                             Flavour::ResolvedOnly, VtValue(VtVec3fArray())});
                    } else {
                        // Sparse layouts are sampled at Default in the prologue;
                        // their exact source arrays must also reach readiness.
                        const SdfPath shape = sample.blendShape.GetPrimPath();
                        sample.offsetsLeaf =
                            decl.Add({shape.AppendProperty(_tokens->offsets),
                                      Type::Vec3fArray,
                                      RigExecRevisionLeafTime::AtDefault,
                                      Flavour::Raw, VtValue(VtVec3fArray())});
                        sample.indicesLeaf = decl.Add(
                            {shape.AppendProperty(_tokens->pointIndices),
                             Type::IntArray, RigExecRevisionLeafTime::AtDefault,
                             Flavour::Raw, VtValue(VtIntArray())});
                    }
                }
            }
        }
        // Topology is compiled rig state: the reads the role table marks
        // (RigExecRevisionLeafRoleIsTopology) and a sparse sample's offsets
        // and point indices. An authored edit to one -- a value, a time
        // sample, a connection -- rebuilds the program into a new compiled rig
        // instead of reaching the next generation through a re-read, so
        // inside one program such a read moves only with the time or an
        // override (RigExecBakedSamplePathLeaves).
        const auto compiledRig = [&](int k) {
            if (k < 0 || size_t(k) >= decl.keys.size() ||
                !decl.keys[size_t(k)].path.IsPropertyPath()) {
                return;
            }
            const SdfPath &path = decl.keys[size_t(k)].path;
            B.rebuild.insert(path);
            B.named.insert(path);
            B.prims.insert(path.GetPrimPath());
        };
        for (size_t role = 0; role < decl.roles.size(); ++role) {
            if (RigExecRevisionLeafRoleIsTopology(
                    RigExecRevisionLeafRole(role))) {
                compiledRig(decl.roles[role]);
            }
        }
        for (const auto &channel : out.blendChannels) {
            for (const auto &sample : channel.samples) {
                compiledRig(sample.offsetsLeaf);
                compiledRig(sample.indicesLeaf);
            }
        }
        out.kernelRecord.op = out.op;
        out.kernelRecord.binding = out.binding;
        out.kernelRecord.leaves = decl;
        // Detached translation consumes the already captured scene facts. USD
        // handles above remain confined to the native sampling adapter.
        if (B.sceneDescriptors) {
            std::string error;
            if (!RigExecDispatchTranslateSceneGeometry(
                    *B.sceneDescriptors, out.moverPath, out.target,
                    &out.sceneGeometry, &error, &out.kernelRecord))
                refuse(error, out.moverPath);
        }
        return out;
    };
    // A revision's leaves bound, its sparse samples' offsets and point
    // indices marked compiled rig keys beside the role table's.
    const auto bindRevisionLeaves =
        [&](RigExecBakedProgramImpl::GeomRevision *revision) {
            RigExecBakedBindPathLeaves(B.stage, &revision->leaves);
            std::vector<char> &marks = revision->leaves.compiledRig;
            for (const auto &channel : revision->blendChannels) {
                for (const auto &sample : channel.samples) {
                    for (const int k :
                         {sample.offsetsLeaf, sample.indicesLeaf}) {
                        if (k >= 0 && size_t(k) < marks.size()) {
                            marks[size_t(k)] = 1;
                        }
                    }
                }
            }
        };
    for (const RigExecBakedChainSpec &spec : chains) {
        const SdfPath &target = spec.target;
        RigExecBakedProgramImpl::GeomChain chain;
        chain.target = target;
        // The base points are read per frame through a query, so only the
        // query's own validity depends on this surviving a resync.
        B.prims.insert(target.GetPrimPath());
        B.named.insert(target);
        if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
            chain.baseQuery = UsdAttributeQuery(a);
        } else {
            refuse("chain target has no attribute", target);
        }
        for (const RigExecBakedRevisionSpec &revision : spec.revisions) {
            chain.revisions.push_back(bakeRevision(revision));
            // RevisionStatic's own defaultWeight read, which the fuse's
            // diagnostic is about: the generation's resolved inputs alone.
            RigExecBakedProgramImpl::GeomRevision &baked =
                chain.revisions.back();
            baked.defaultWeightLeaf = baked.leaves.decl.Add(
                {baked.moverPath.AppendProperty(_tokens->defaultWeight),
                 RigExecRevisionLeafType::Float,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly, VtValue(1.0f)});
            bindRevisionLeaves(&baked);
        }
        for (const RigExecBakedRevisionSpec &derived : spec.derived) {
            RigExecBakedProgramImpl::GeomChain::Derived d;
            d.target = derived.target;
            d.targetText = pathText(derived.target);
            d.matrixTarget = RigExecIsDerivedMatrixOp(derived.op);
            B.prims.insert(derived.target.GetPrimPath());
            B.named.insert(derived.target);
            if (d.matrixTarget) {
                // A projector's primvar is published, never authored, so
                // there is no base to read; its inputs are the chain's.
            } else if (const UsdAttribute a =
                           B.stage->GetAttributeAtPath(derived.target)) {
                d.baseQuery = UsdAttributeQuery(a);
            } else {
                refuse("derived target has no attribute", derived.target);
            }
            d.revision = bakeRevision(derived);
            bindRevisionLeaves(&d.revision);
            chain.derived.push_back(std::move(d));
        }
        B.chains.push_back(std::move(chain));
    }
    BindPointReads(&B);
}

// The vertex partition.
// A skin revision's per-vertex work is separable -- point i reads the indices
// and weights at i * elementSize, the influence matrices they name and its
// own incoming position, and nothing else -- so a contiguous range of
// vertices is an independent sub-problem whose result is bit-identical to the
// same vertices computed as part of the whole array. What is NOT separable is
// everything around that body, which is why the range is cut here and the
// decisions stay whole.
// The cut is by vertex COUNT and the key follows from it, rather than the
// other way round: the vertex order is the mesh's and is never permuted, so
// two body regions that happen to share a range simply wait for both their
// joints -- and the other ranges still start when their own joints land,
// which is what the whole exercise buys. The schedule report prints |key| per
// chunk so that trade-off is measured per asset rather than assumed.

namespace {

// Natural chunk producer identities are independent of any scheduling order.
using SkinProducerSet = std::vector<std::pair<uint8_t, int>>;

SkinProducerSet
ChunkProducers(const RigExecBakedProgramImpl::GeomRevision &revision,
               const RigExecBakedProgramImpl::GeomChunk &chunk, bool chunked)
{
    SkinProducerSet producers;
    const uint8_t domain = uint8_t(RigExecBakedOwnMatrixDomain(revision));
    const size_t count =
        chunked ? chunk.key.size() : revision.influenceSlots.size();
    for (size_t k = 0; k < count; ++k) {
        const size_t position = chunked ? size_t(chunk.key[k]) : k;
        if (position < revision.influenceSlots.size() &&
            revision.influenceSlots[position] >= 0)
            producers.emplace_back(domain, revision.influenceSlots[position]);
    }
    std::sort(producers.begin(), producers.end());
    producers.erase(std::unique(producers.begin(), producers.end()),
                    producers.end());
    return producers;
}

bool
ChunkAlways()
{
    return TfGetenvInt("RIGEXEC_BAKED_CHUNK_ALWAYS", 0) != 0;
}

/// Whether every element of ascending \p a appears in ascending \p b.
bool
IsSubset(const std::vector<int> &a, const std::vector<int> &b)
{
    size_t j = 0;
    for (const int value : a) {
        while (j < b.size() && b[j] < value) {
            ++j;
        }
        if (j >= b.size() || b[j] != value) {
            return false;
        }
    }
    return true;
}

} // namespace

/// How many vertices one chunk covers before the cap takes over.
///
/// The default is the point count at which the geometry kernels start
/// splitting work themselves, because a range smaller than that is a range
/// the deformation was never worth spreading over. Read at Build into
/// chunkVertexTarget.
size_t
RigExecBakedChunkVertexTargetFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_CHUNK_VERTS",
                                     int(RigExecGeometryParallelThreshold));
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// The most chunks one revision is cut into; the range grows to meet it.
///
/// A cap rather than a target: chunk count decides STEP count, and a mesh
/// with a million vertices must not add two hundred steps to the program for
/// a machine that can run twenty of them at once. Read at Build into
/// chunkCap.
size_t
RigExecBakedChunkCapFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32);
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// Whether a chain of more than chunkVertexTarget points runs its Matrix,
/// Wire and Lattice revisions as one step per point range. Read at Build
/// into rangeChains.
bool
RigExecBakedRangeChainsFromEnvironment()
{
    return TfGetenvBool("RIGEXEC_BAKED_RANGE_CHAINS", true);
}

/// How many points one vertex group of a range chain covers before the cap
/// takes over. Read at Build into groupVertexTarget.
size_t
RigExecBakedGroupVertexTargetFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_GROUP_VERTS", 1024);
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// The most vertex groups one range chain is cut into. Read at Build into
/// groupCap.
size_t
RigExecBakedGroupCapFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_GROUP_CAP",
                                     int(RigExecBakedDefaultGroupCap));
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// Whether a Range revision over a static sparse zero-default weight writes
/// only the groups its indices touch. Read at Build into groupGates.
bool
RigExecBakedGroupGatesFromEnvironment()
{
    return TfGetenvBool("RIGEXEC_BAKED_GROUP_GATES", true);
}

void
RigExecBakedPartitionRevision(const RigExecBakedProgramImpl &program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const VtIntArray &indices, int elementSize)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    const size_t indexCount = indices.size();
    const size_t slots = elementSize < 1 ? 0 : size_t(elementSize);
    const size_t points = slots ? indexCount / slots : 0;
    const size_t influences = revision->influenceSlots.size();
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indexCount;
    revision->partitionPointCount = points;

    // How many ranges, and how long.
    size_t target = program.chunkVertexTarget;
    size_t count = std::max<size_t>(1, (points + target - 1) / target);
    if (count > program.chunkCap) {
        count = program.chunkCap;
        target = std::max<size_t>(1, (points + count - 1) / count);
        count = std::max<size_t>(1, (points + target - 1) / target);
    }

    std::vector<GeomChunk> chunks;
    chunks.reserve(count);
    // One pass over the indices, and one byte per influence to remember what
    // this range has already claimed -- the key is a SET of the range's
    // influence positions, and the vertices arrive in no particular order.
    std::vector<char> claimed(influences, 0);
    for (size_t c = 0; c < count; ++c) {
        GeomChunk chunk;
        chunk.begin = int(std::min(points, c * target));
        chunk.end =
            int(c + 1 == count ? points : std::min(points, (c + 1) * target));
        for (size_t point = size_t(chunk.begin); point < size_t(chunk.end);
             ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices.cdata()[point * slots + slot];
                // An index outside the table is a layout the packet will
                // reject; leaving it out of the key costs nothing, because
                // the revision never applies.
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunks.push_back(std::move(chunk));
    }

    // Adjacent ranges merge while they stay under the vertex target and one
    // key contains the other: a range waiting for a superset of its
    // neighbour's joints is not waiting any longer for holding both, and one
    // range is one step's worth of scheduling instead of two.
    for (size_t i = 0; i + 1 < chunks.size();) {
        const size_t merged = size_t(chunks[i + 1].end - chunks[i].begin);
        const bool contains = IsSubset(chunks[i].key, chunks[i + 1].key);
        const bool contained = IsSubset(chunks[i + 1].key, chunks[i].key);
        if (merged <= target && (contains || contained)) {
            chunks[i].end = chunks[i + 1].end;
            if (contains) {
                chunks[i].key = chunks[i + 1].key; // the union
            }
            chunks.erase(chunks.begin() + long(i) + 1);
        } else {
            ++i;
        }
    }

    // The table every chunk skins against: identity everywhere, with its own
    // influences copied in per run. Filled here so that a run writes only the
    // |key| entries that moved, and so that the rows a SIMD kernel loads are
    // never out of step with the matrices beside them.
    const bool keyed = chunks.size() > 1;
    for (GeomChunk &chunk : chunks) {
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.palette.clear();
        if (!keyed) {
            // One chunk is the whole array, and the whole array is what the
            // revision's own folded table already holds.
            chunk.key.clear();
            chunk.transforms.clear();
            chunk.rows.clear();
            continue;
        }
        chunk.transforms.assign(influences, GfMatrix4d(1.0));
        chunk.rows.assign(influences * RigExecSkinRowStride, 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(chunk.transforms[t],
                                  &chunk.rows[t * RigExecSkinRowStride]);
        }
    }
    revision->chunks = std::move(chunks);
    revision->chunked = keyed;
    revision->partitionIndices = keyed ? indices : VtIntArray();
}

void
RigExecBakedAdoptPartition(RigExecBakedProgramImpl::GeomRevision *revision)
{
    // EvalCore's one adoption decision (rigExecEvalCore/chain.h) over
    // this adapter's Build inputs: a null handle is a refusal (the
    // packet reads the arrays per frame and RevisionStatic runs the
    // revision whole), and a handle is adopted only when it has the
    // arrays the keys were cut from; a weight-only edit keeps the
    // chunks running.
    eval::AdoptPartitionBody(
        revision->chunked, revision->topology != nullptr,
        revision->topology == revision->partitionTopology,
        [revision] {
            const RigExecSkinTopology &topology = *revision->topology;
            const VtIntArray &indices = revision->partitionIndices;
            return topology.elementSize == revision->partitionElementSize &&
                   topology.indices.size() == indices.size() &&
                   std::equal(topology.indices.begin(), topology.indices.end(),
                              indices.cdata());
        },
        [revision] { revision->partitionTopology = revision->topology; });
}

// Build: the geometry half of the program, in program order.

namespace {

RigExecBakedStep &
AddGeometryStep(RigExecBakedProgramImpl *program, RigExecBakedStepKind kind,
                int object, int part = -1)
{
    RigExecBakedStep step;
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 0;
    program->steps.push_back(std::move(step));
    return program->steps.back();
}

/// Declares the matrices \p revision reads, which are the slots the pose half
/// published for it: the same enumeration the ProviderMatrix steps were
/// created from.
void
DeclareMatrixReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                   RigExecBakedStep *step)
{
    RigExecBakedForEachMatrixRead(
        revision, [step](RigExecBakedSlotDomain domain, int slot) {
            step->reads.push_back(RigExecBakedOne(domain, slot));
        });
}

/// Declares a read of point version \p version: the authored base for 0, and
/// otherwise the RevisionDone and ChainDirty slots of revision
/// first + version - 1, whose only writer is that revision's fuse. The buffer
/// the version resolves to is not declared: whichever earlier chunk or fuse
/// filled it precedes that fuse.
void
DeclarePointVersionRead(const RigExecBakedProgramImpl &B,
                        RigExecBakedPointVersion version,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    RigExecBakedDeclarePointVersion(B, version.chain, version.version, reads);
}

/// Declares what \p binding can read: each candidate version, or the
/// chain's published points for a final read. The tail is the resolved
/// input, which is prologue state.
void
DeclarePointBindingReads(const RigExecBakedProgramImpl &B,
                         const RigExecBakedPointsBinding &binding,
                         std::vector<RigExecBakedSlotRange> *reads)
{
    RigExecBakedDeclarePointInput(B, binding, reads);
}

/// The frame records an AtPrim transform phase of \p revision can read,
/// for its transform and for each influence entry. Their FrameMatrix steps
/// are the only writers, and all of them are pose steps.
void
DeclareFrameRecordReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    for (const int record : revision.transformRecords) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
    }
    for (const std::vector<int> &records : revision.influenceRecords) {
        for (const int record : records) {
            reads->push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
        }
    }
}

/// The point bindings of \p revision: its declared input phases and its
/// blend samples'.
void
DeclareRevisionPointReads(const RigExecBakedProgramImpl &B,
                          const RigExecBakedProgramImpl::GeomRevision &revision,
                          std::vector<RigExecBakedSlotRange> *reads)
{
    for (const RigExecBakedPointsBinding &binding : revision.pointBindings) {
        DeclarePointBindingReads(B, binding, reads);
    }
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
         revision.blendChannels) {
        for (const auto &sample : channel.samples) {
            DeclarePointBindingReads(B, sample.pointBinding, reads);
        }
    }
}

} // namespace

namespace {

/// Cuts \p revision at Build, from the layout the stage authors.
///
/// Only a SKIN revision whose layout the compiled rig fixed is cut into more than
/// one chunk. Two reasons, and they are the same reason twice: a layout that
/// can move within the compiled rig is one whose keys a Build-time cut cannot
/// promise anything about, and a mover with such a layout already re-reads
/// and re-validates every element of it once per frame, which is far more
/// than a partition would save. Everything else -- every other operation, and
/// a skin whose arrays are animated, connected or written by a property chain
/// -- is one chunk over the whole array, which is the degenerate case of the
/// same step.
void
PartitionAtBuild(const RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
    revision->chunked = false;
    revision->partitionCandidates = 0;
    revision->partitionProducerMin = 0;
    revision->partitionProducerMax = 0;
    revision->partitionDistinctReads = 0;
    revision->partitionProducerSets.clear();
    if (revision->op != RigExecRevisionOp::Skin ||
        !revision->skinTopologyFixed || !revision->moverPrim) {
        return;
    }
    // Read directly, at Default: Build holds no layout handle yet. Fixed
    // means the arrays do not vary in time, so the default-time read IS the
    // compiled rig's. A handle whose arrays disagree with it (an override standing
    // on the indices at Build) is never adopted, and the revision runs whole
    // (RigExecBakedAdoptPartition).
    VtIntArray indices;
    int elementSize = 1;
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->jointIndices)) {
        a.Get(&indices, UsdTimeCode::Default());
    }
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->elementSize)) {
        a.Get(&elementSize, UsdTimeCode::Default());
    }
    if (elementSize < 1 || indices.empty() ||
        indices.size() % size_t(elementSize) != 0) {
        return;
    }
    RigExecBakedPartitionRevision(B, revision, indices, elementSize);
    revision->partitionCandidates = revision->chunks.size();

    // The vertex target and cap bound the work per operation. Distinct exact
    // producer sets let the common graph release independent ranges; no pose
    // scheduling information is needed to make the cut.
    std::set<SkinProducerSet> distinct;
    int producerMin = -1, producerMax = 0;
    for (const auto &chunk : revision->chunks) {
        auto producers = ChunkProducers(*revision, chunk, revision->chunked);
        const int count = int(producers.size());
        producerMin = producerMin < 0 ? count : std::min(producerMin, count);
        producerMax = std::max(producerMax, count);
        distinct.insert(producers);
        revision->partitionProducerSets.push_back(std::move(producers));
    }
    revision->partitionProducerMin = std::max(0, producerMin);
    revision->partitionProducerMax = producerMax;
    revision->partitionDistinctReads = distinct.size();
    if (revision->chunked && distinct.size() < 2 && !ChunkAlways()) {
        // Back to the degenerate cut, which is the shape every other
        // operation already has: one range over the whole array, no keys and
        // no per-chunk tables, so the fuse's whole-array path runs the
        // revision through the self-parallelising kernel. The partition
        // FIELDS stay where the cut left them -- the point and index counts
        // are the layout's, not the cut's, and the report reads them -- but
        // the indices, which only an adoption reads, are let go.
        revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
        revision->chunked = false;
        revision->partitionIndices = VtIntArray();
    }
}

/// Declares WeightFrames[slot] for every volume in weight object \p object's
/// composition closure (its baseWeight and inputWeights, recursively): the
/// placements the oracle reads when it resolves that object. A closure
/// member outside the table, or a volume slot that has no placement step,
/// should be unreachable; it falls back to every volume slot.
void
DeclarePlacementReads(const RigExecBakedProgramImpl &B, int object,
                      std::vector<RigExecBakedSlotRange> *reads)
{
    std::vector<char> seen(B.weightObjects.size(), 0);
    std::vector<int> stack{object};
    std::vector<int> slots;
    bool complete = true;
    while (!stack.empty()) {
        const int o = stack.back();
        stack.pop_back();
        if (o < 0 || size_t(o) >= B.weightObjects.size()) {
            complete = false;
            continue;
        }
        if (seen[size_t(o)]) {
            continue;
        }
        seen[size_t(o)] = 1;
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[size_t(o)];
        if (weight.providerSlot >= 0) {
            if (size_t(weight.providerSlot) < B.noScaleAvars.size() &&
                B.noScaleAvars[size_t(weight.providerSlot)]) {
                slots.push_back(weight.providerSlot);
            } else {
                complete = false;
            }
        }
        if (weight.base >= 0) {
            stack.push_back(weight.base);
        }
        stack.insert(stack.end(), weight.inputs.begin(), weight.inputs.end());
    }
    if (!TF_VERIFY(complete, "a weight object's closure is outside the "
                             "program's volume table")) {
        slots.clear();
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i]) {
                slots.push_back(int(i));
            }
        }
    }
    for (const int slot : slots) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, slot));
    }
}

/// The point count Build cuts \p chain's ranges from, read on the owning
/// thread in the prologue's order: the upstream value at the target, else the
/// authored base at Default, else at the earliest time; 0 when none reads.
/// Only the partition depends on it: a run clips the ranges to the count it
/// applies to, and any partition gives a per-point kernel the same bits.
size_t
BuildPointCount(const RigExecBakedProgramImpl &B,
                const RigExecBakedProgramImpl::GeomChain &chain)
{
    const auto upstream = B.upstream.find(chain.target);
    if (upstream != B.upstream.end() &&
        upstream->second.IsHolding<VtVec3fArray>()) {
        return upstream->second.UncheckedGet<VtVec3fArray>().size();
    }
    if (!chain.baseQuery.IsValid()) {
        return 0;
    }
    VtVec3fArray points;
    if (!chain.baseQuery.Get(&points, UsdTimeCode::Default()) ||
        points.empty()) {
        points = VtVec3fArray();
        chain.baseQuery.Get(&points, UsdTimeCode::EarliestTime());
    }
    return points.size();
}

/// How many ranges a chain of \p points points is cut into: the shared count,
/// less the trailing ranges the ceil-sized bounds would leave empty (only
/// when the cap or a tiny vertex target makes the ranges nearly as many as
/// the points), so every range is non-empty. 1 is no pipelining.
size_t
ChainRangeCount(const RigExecBakedProgramImpl &B, size_t points)
{
    if (!B.rangeChains) {
        return 1;
    }
    size_t ranges =
        RigExecPointRangeCount(points, B.chunkVertexTarget, B.chunkCap);
    while (ranges > 1 &&
           RigExecPointRangeBound(points, ranges, ranges - 1) >= points) {
        --ranges;
    }
    return ranges;
}

/// How many vertex groups a range chain of \p points points is cut into:
/// the group target and cap, trimmed like ChainRangeCount so every group is
/// non-empty. Live and Export alike.
size_t
ChainGroupCount(const RigExecBakedProgramImpl &B, size_t points)
{
    if (!B.rangeChains) {
        return 1;
    }
    size_t groups =
        RigExecPointRangeCount(points, B.groupVertexTarget, B.groupCap);
    while (groups > 1 &&
           RigExecPointRangeBound(points, groups, groups - 1) >= points) {
        --groups;
    }
    return groups;
}

using GeomRole = RigExecBakedRevisionRole;
using RolePin = RigExecBakedRolePin;

/// What Build's role classification reads, the way the geometry prologue
/// will read it: the stage, the evaluator's standing interactive overrides
/// and the admitted upstream values (B.upstream). Owning thread, Build only.
struct RoleReads {
    explicit RoleReads(const RigExecBakedProgramImpl &program)
        : B(program),
          exportMode(program.roleMode == RigExecBakedRoleMode::Export)
    {
        if (B.interactiveOverrides) {
            for (const RigExecValueOverride &o : *B.interactiveOverrides) {
                const SdfPath path = o.prim.AppendProperty(
                    o.attribute.IsEmpty() ? o.computation : o.attribute);
                overlay.SetProperty(path, o.value);
                overrides.emplace_back(path, o.value);
            }
        }
    }

    /// Whether a standing override or an admitted upstream value stands on
    /// one of \p hops.
    bool Reaches(const std::vector<SdfPath> &hops) const
    {
        for (const SdfPath &hop : hops) {
            if (B.upstream.count(hop)) {
                return true;
            }
            for (const auto &entry : overrides) {
                if (entry.first == hop) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Whether \p path may never become a file constant: a property-chain
    /// target, an admitted upstream path or a path the bake keeps listed.
    bool KeptListed(const SdfPath &path) const
    {
        return B.chainTargets.count(path) || B.upstream.count(path) ||
               B.exportKeep.count(path);
    }

    /// Live: key \p k's read cannot move with the time -- no hop varies or
    /// holds a time sample, so an animated method or space is never pinned --
    /// and its walk meets no value the region computes.
    bool Pinnable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        if (k < 0 || size_t(k) >= leaves.decl.keys.size() ||
            size_t(k) >= leaves.hops.size()) {
            return false;
        }
        if (size_t(k) < leaves.walks.size() && leaves.walks[size_t(k)] >= 0) {
            return false;
        }
        if (RigExecBakedLeafVaryingNow(B, leaves, size_t(k))) {
            return false;
        }
        for (const SdfPath &hop : leaves.hops[size_t(k)]) {
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(hop)) {
                if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Export: pinnable, a walk of exactly the head (no connection), and
    /// never kept listed.
    bool Bakeable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        if (!Pinnable(leaves, k)) {
            return false;
        }
        const std::vector<SdfPath> &hops = leaves.hops[size_t(k)];
        const SdfPath &head = leaves.decl.keys[size_t(k)].path;
        if (hops.size() > 1 || (hops.size() == 1 && hops[0] != head) ||
            KeptListed(head)) {
            return false;
        }
        const UsdAttribute &a = size_t(k) < leaves.attributes.size()
                                    ? leaves.attributes[size_t(k)]
                                    : UsdAttribute();
        return !(a && a.HasAuthoredConnections());
    }

    /// The rule of the mode Build runs in.
    bool Usable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        return exportMode ? Bakeable(leaves, k) : Pinnable(leaves, k);
    }

    /// Key \p k's value as the prologue will sample it. A usable key does not
    /// vary, so Default reads what every time reads.
    TfToken Token(const RigExecBakedPathLeaves &leaves, int k) const
    {
        const VtValue value = RigExecSampleRevisionLeaf(
            leaves.decl.keys[size_t(k)], leaves.attributes[size_t(k)], &overlay,
            UsdTimeCode::Default(), &B.upstream);
        return value.IsHolding<TfToken>() ? value.UncheckedGet<TfToken>()
                                          : TfToken();
    }

    /// Key \p k's int array, sampled as Token samples; empty for k < 0.
    VtIntArray Ints(const RigExecBakedPathLeaves &leaves, int k) const
    {
        if (k < 0) {
            return VtIntArray();
        }
        const VtValue value = RigExecSampleRevisionLeaf(
            leaves.decl.keys[size_t(k)], leaves.attributes[size_t(k)], &overlay,
            UsdTimeCode::Default(), &B.upstream);
        return value.IsHolding<VtIntArray>() ? value.UncheckedGet<VtIntArray>()
                                             : VtIntArray();
    }

    /// Key \p k's int, sampled as Token samples; \p fallback when it holds
    /// none.
    int Int(const RigExecBakedPathLeaves &leaves, int k, int fallback) const
    {
        if (k < 0) {
            return fallback;
        }
        const VtValue value = RigExecSampleRevisionLeaf(
            leaves.decl.keys[size_t(k)], leaves.attributes[size_t(k)], &overlay,
            UsdTimeCode::Default(), &B.upstream);
        return value.IsHolding<int>() ? value.UncheckedGet<int>() : fallback;
    }

    const RigExecBakedProgramImpl &B;
    const bool exportMode;
    RigExecResolvedInputs overlay;
    std::vector<std::pair<SdfPath, VtValue>> overrides;
};

/// A skin mover's layout arrays as PartitionAtBuild reads them, at Default;
/// false when they cannot be cut.
bool
ReadSkinLayoutAtBuild(const RigExecBakedProgramImpl::GeomRevision &revision,
                      VtIntArray *indices, int *elementSize)
{
    *elementSize = 1;
    if (!revision.moverPrim) {
        return false;
    }
    if (const UsdAttribute a =
            revision.moverPrim.GetAttribute(_tokens->jointIndices)) {
        a.Get(indices, UsdTimeCode::Default());
    }
    if (const UsdAttribute a =
            revision.moverPrim.GetAttribute(_tokens->elementSize)) {
        a.Get(elementSize, UsdTimeCode::Default());
    }
    return *elementSize >= 1 && !indices->empty() &&
           indices->size() % size_t(*elementSize) == 0;
}

/// Cuts a skin revision of a range chain at the chain's group bounds: chunk
/// g is group g, keyed by the union of its vertices' joint indices (zero-
/// weight slots included) and cut exactly at the bounds -- no merge, no
/// degenerate fallback, RIGEXEC_BAKED_CHUNK_ALWAYS not consulted -- with its
/// tables filled as a keyed chunk's and the indices kept for adoption.
/// False, leaving the revision as it was, when the layout cannot be cut.
bool
PartitionSkinGroups(RigExecBakedProgramImpl::GeomRevision *revision,
                    const std::vector<int> &bounds)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    VtIntArray indices;
    int elementSize = 1;
    if (bounds.size() < 2 ||
        !ReadSkinLayoutAtBuild(*revision, &indices, &elementSize)) {
        return false;
    }
    const size_t slots = size_t(elementSize);
    const size_t layoutPoints = indices.size() / slots;
    const size_t influences = revision->influenceSlots.size();
    std::vector<GeomChunk> chunks(bounds.size() - 1);
    std::vector<char> claimed(influences, 0);
    for (size_t g = 0; g + 1 < bounds.size(); ++g) {
        GeomChunk &chunk = chunks[g];
        chunk.begin = bounds[g];
        chunk.end = bounds[g + 1];
        const size_t end = std::min(size_t(chunk.end), layoutPoints);
        for (size_t point = size_t(chunk.begin); point < end; ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices.cdata()[point * slots + slot];
                // As RigExecBakedPartitionRevision: an index outside the
                // table is a layout the packet rejects.
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.transforms.assign(influences, GfMatrix4d(1.0));
        chunk.rows.assign(influences * RigExecSkinRowStride, 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(chunk.transforms[t],
                                  &chunk.rows[t * RigExecSkinRowStride]);
        }
    }
    revision->chunks = std::move(chunks);
    revision->chunked = true;
    revision->partitionIndices = indices;
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indices.size();
    revision->partitionPointCount = layoutPoints;
    revision->partitionCandidates = revision->chunks.size();
    revision->partitionProducerSets.clear();
    std::set<SkinProducerSet> distinct;
    int producerMin = -1, producerMax = 0;
    for (const GeomChunk &chunk : revision->chunks) {
        SkinProducerSet producers = ChunkProducers(*revision, chunk, true);
        const int count = int(producers.size());
        producerMin = producerMin < 0 ? count : std::min(producerMin, count);
        producerMax = std::max(producerMax, count);
        distinct.insert(producers);
        revision->partitionProducerSets.push_back(std::move(producers));
    }
    revision->partitionProducerMin = std::max(0, producerMin);
    revision->partitionProducerMax = producerMax;
    revision->partitionDistinctReads = distinct.size();
    return true;
}

/// The role \p revision of an eligible chain takes, from what Build reads,
/// with the pins it rests on. A role a usable method or space leaf
/// chose by value is pinned, whichever it chose; a Range skin also pins that
/// no override or upstream value reaches its layout topology. Export adds
/// the Range skin's reads to \p pinned.
GeomRole
ClassifyRevision(const RoleReads &reads,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 std::vector<SdfPath> *pinned)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->leaves;
    const auto pinToken = [&](int k, const TfToken &token) {
        RolePin pin;
        pin.kind = RolePin::Kind::LeafToken;
        pin.leaf = k;
        pin.token = token;
        revision->pins.push_back(pin);
    };
    switch (revision->op) {
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Wire:
        return GeomRole::Range;
    case RigExecRevisionOp::Lattice: {
        // Only the Bernstein evaluation splits by vertex group; a regular
        // grid, or an evaluation that can move, runs whole. A file records
        // a usable evaluation as a constant, as a Range skin's method.
        const int evaluation = leaves.decl.Role(Role::LatticeEvaluation);
        if (!reads.Usable(leaves, evaluation)) {
            return GeomRole::Whole;
        }
        const TfToken value = reads.Token(leaves, evaluation);
        pinToken(evaluation, value);
        if (value != _tokens->legacyLattice) {
            return GeomRole::Whole;
        }
        if (reads.exportMode) {
            pinned->push_back(leaves.decl.keys[size_t(evaluation)].path);
        }
        return GeomRole::Range;
    }
    case RigExecRevisionOp::BlendShape: {
        // rigExec:deltaSpace never enters the file, so Export pins it as
        // Live does.
        const int space = leaves.decl.Role(Role::DeltaSpace);
        if (!reads.Pinnable(leaves, space)) {
            return GeomRole::Whole;
        }
        const TfToken value = reads.Token(leaves, space);
        pinToken(space, value);
        return value == _tokens->targetSpace ? GeomRole::Range
                                             : GeomRole::Whole;
    }
    case RigExecRevisionOp::Skin: {
        // The keys of a Range skin rest on a compiled rig layout; without
        // one the method decides nothing and nothing is pinned.
        VtIntArray indices;
        int elementSize = 1;
        if (!revision->skinTopologyFixed ||
            !RigExecSkinLayoutTopologyIsFixed(revision->moverPrim) ||
            !ReadSkinLayoutAtBuild(*revision, &indices, &elementSize)) {
            return GeomRole::Whole;
        }
        const int method = leaves.decl.Role(Role::SkinningMethod);
        const int jointIndices = leaves.decl.Role(Role::JointIndices);
        const int size = leaves.decl.Role(Role::ElementSize);
        if (!reads.Usable(leaves, method) || jointIndices < 0 || size < 0 ||
            size_t(jointIndices) >= leaves.hops.size() ||
            size_t(size) >= leaves.hops.size()) {
            return GeomRole::Whole;
        }
        // A layout override or upstream value standing at Build keeps the
        // stale-partition fallback of a Whole fuse.
        if (reads.Reaches(leaves.hops[size_t(jointIndices)]) ||
            reads.Reaches(leaves.hops[size_t(size)]) ||
            (reads.exportMode && (!reads.Bakeable(leaves, jointIndices) ||
                                  !reads.Bakeable(leaves, size)))) {
            return GeomRole::Whole;
        }
        const TfToken value = reads.Token(leaves, method);
        pinToken(method, value);
        if (value != _tokens->classicLinear &&
            value != _tokens->dualQuaternion) {
            return GeomRole::Whole;
        }
        for (const int k : {jointIndices, size}) {
            RolePin pin;
            pin.kind = RolePin::Kind::NoLayoutOverride;
            pin.leaf = k;
            revision->pins.push_back(pin);
        }
        if (reads.exportMode) {
            for (const int k : {method, jointIndices, size}) {
                pinned->push_back(leaves.decl.keys[size_t(k)].path);
            }
        }
        return GeomRole::Range;
    }
    case RigExecRevisionOp::Smooth: {
        // Its group steps read the groups its edge neighbours lie in, cut
        // from the topology Build reads; a topology that can move runs whole.
        const int counts = leaves.decl.Role(Role::TopologyCounts);
        const int indices = leaves.decl.Role(Role::TopologyIndices);
        for (const int k : {counts, indices}) {
            if (k < 0 || size_t(k) >= leaves.hops.size() ||
                !reads.Usable(leaves, k) ||
                reads.Reaches(leaves.hops[size_t(k)])) {
                return GeomRole::Whole;
            }
        }
        for (const int k : {counts, indices}) {
            RolePin pin;
            pin.kind = RolePin::Kind::NoLayoutOverride;
            pin.leaf = k;
            revision->pins.push_back(pin);
            if (reads.exportMode) {
                pinned->push_back(leaves.decl.keys[size_t(k)].path);
            }
        }
        return GeomRole::Range;
    }
    case RigExecRevisionOp::DeltaMush: {
        // Its group steps read the region its smoothing reaches, cut from
        // the topology, the explicit edges and the iteration count Build
        // reads; any of them that can move runs it whole.
        const std::array<int, 4> keys = {
            leaves.decl.Role(Role::TopologyCounts),
            leaves.decl.Role(Role::TopologyIndices),
            leaves.decl.Role(Role::MushEdges), leaves.decl.scalarBegin};
        for (size_t j = 0; j < keys.size(); ++j) {
            const int k = keys[j];
            if (k < 0 && j == 2) {
                continue;
            }
            if (k < 0 || size_t(k) >= leaves.hops.size() ||
                !reads.Usable(leaves, k) ||
                reads.Reaches(leaves.hops[size_t(k)])) {
                return GeomRole::Whole;
            }
        }
        for (const int k : keys) {
            if (k < 0) {
                continue;
            }
            RolePin pin;
            pin.kind = RolePin::Kind::NoLayoutOverride;
            pin.leaf = k;
            revision->pins.push_back(pin);
            if (reads.exportMode) {
                pinned->push_back(leaves.decl.keys[size_t(k)].path);
            }
        }
        return GeomRole::Range;
    }
    default:
        return GeomRole::Whole;
    }
}

/// Whether \p revision's Range role rests on NoLayoutOverride pins of both
/// its topology counts and indices; RigExecBakedRolesStand rebuilds when
/// one fails, so its kernel memo may key that topology by point count.
bool
TopologyPinned(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    using Role = RigExecRevisionLeafRole;
    const int counts = revision.leaves.decl.Role(Role::TopologyCounts);
    const int indices = revision.leaves.decl.Role(Role::TopologyIndices);
    if (!revision.rangeRole || counts < 0 || indices < 0) {
        return false;
    }
    bool countsPinned = false, indicesPinned = false;
    for (const RolePin &pin : revision.pins) {
        if (pin.kind == RolePin::Kind::NoLayoutOverride) {
            countsPinned = countsPinned || pin.leaf == counts;
            indicesPinned = indicesPinned || pin.leaf == indices;
        }
    }
    return countsPinned && indicesPinned;
}

/// The halo of the Range DeltaMush \p revision: per group of \p chain, the
/// other groups holding a point the group's results read, from the topology,
/// edges and iteration count Build reads (the classification pinned them).
/// The graph is the rest builder's own, over rest points that weigh nothing.
void
BuildMushHalo(const RoleReads &reads,
              const RigExecBakedProgramImpl::GeomChain &chain,
              RigExecBakedProgramImpl::GeomRevision *revision, size_t points)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->leaves;
    const size_t groups =
        chain.groupBounds.empty() ? 0 : chain.groupBounds.size() - 1;
    const VtIntArray counts =
        reads.Ints(leaves, leaves.decl.Role(Role::TopologyCounts));
    const VtIntArray indices =
        reads.Ints(leaves, leaves.decl.Role(Role::TopologyIndices));
    const VtIntArray edges =
        reads.Ints(leaves, leaves.decl.Role(Role::MushEdges));
    const int iterations = reads.Int(leaves, leaves.decl.scalarBegin, 10);
    RigExecDeltaMushSettings settings;
    settings.onlySmooth = true;
    settings.edges.assign(edges.cbegin(), edges.cend());
    RigExecDeltaMushRestData<GfVec3f, GfVec3d> graph;
    revision->haloGroups.assign(groups, {});
    if (RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(
            std::vector<GfVec3f>(points, GfVec3f(0.0f)),
            std::vector<int>(counts.cbegin(), counts.cend()),
            std::vector<int>(indices.cbegin(), indices.cend()), iterations, 0.5,
            false, 0.0, settings, &graph)) {
        std::vector<int> readList;
        for (size_t g = 0; g < groups; ++g) {
            const size_t begin = size_t(chain.groupBounds[g]);
            const size_t end = size_t(chain.groupBounds[g + 1]);
            if (!RigExecDeltaMushGroupReads(graph, begin, end, &readList)) {
                continue;
            }
            for (const int i : readList) {
                const size_t h =
                    size_t(std::upper_bound(chain.groupBounds.begin(),
                                            chain.groupBounds.end(), i) -
                           chain.groupBounds.begin()) -
                    1;
                std::vector<int> &halo = revision->haloGroups[g];
                if (h < groups && h != g &&
                    (halo.empty() || halo.back() != int(h))) {
                    halo.push_back(int(h));
                }
            }
        }
    }
    revision->haloPoints.assign(groups,
                                std::vector<const GfVec3f *>(groups, nullptr));
    revision->mushGroups.assign(groups,
                                RigExecDeltaMushGroupState<GfVec3f, GfVec3d>());
}

/// The halo of the Range Smooth \p revision: per group of \p chain, the
/// other groups its points' edge neighbours lie in, from the topology Build
/// reads (the classification pinned it), and its steps' pointer tables.
void
BuildSmoothHalo(const RoleReads &reads,
                const RigExecBakedProgramImpl::GeomChain &chain,
                RigExecBakedProgramImpl::GeomRevision *revision, size_t points)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->leaves;
    const size_t groups =
        chain.groupBounds.empty() ? 0 : chain.groupBounds.size() - 1;
    const VtIntArray counts =
        reads.Ints(leaves, leaves.decl.Role(Role::TopologyCounts));
    const VtIntArray indices =
        reads.Ints(leaves, leaves.decl.Role(Role::TopologyIndices));
    RigExecMeshAdjacency adjacency;
    RigExecBuildMeshAdjacency(points, counts.cdata(), counts.size(),
                              indices.cdata(), indices.size(), &adjacency);
    RigExecSmoothHaloGroups(adjacency, chain.groupBounds,
                            &revision->haloGroups);
    revision->haloPoints.assign(groups,
                                std::vector<const GfVec3f *>(groups, nullptr));
}

/// Static sparse weight object \p object's resolved unlisted value -- its
/// defaultWeight, the only read that enters it (types.cpp) -- as the
/// prologue will read it, when a gate may rest on it: the read is pinnable
/// (Live) or a bakeable constant (Export). \p head receives the read's
/// attribute path.
bool
GateDefault(const RoleReads &reads, int object, float *value, SdfPath *head)
{
    const RigExecBakedProgramImpl &B = reads.B;
    if (object < 0 || size_t(object) >= B.weightObjects.size()) {
        return false;
    }
    const RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(object)];
    if (weight.type != _tokens->staticWeight ||
        weight.representation != _tokens->sparseRepresentation) {
        return false;
    }
    const RigExecBakedInput<float> &input = weight.defaultWeight;
    if (input.varying || input.walk >= 0) {
        return false;
    }
    SdfPathVector walk;
    UsdAttribute selected;
    if (input.head) {
        bool viaChain = false, varying = false;
        RigExecBakedClassifyInput<float>(input.head, UsdTimeCode::Default(),
                                         B.chainTargets, &viaChain, &varying,
                                         &selected, &walk);
        if (viaChain || varying) {
            return false;
        }
        *head = input.head.GetPath();
        if (reads.exportMode &&
            (walk.size() > 1 || input.head.HasAuthoredConnections())) {
            return false;
        }
    } else {
        *head = weight.path.AppendProperty(_tokens->weightDefault);
    }
    if (reads.exportMode) {
        if (reads.KeptListed(*head)) {
            return false;
        }
        for (const SdfPath &hop : walk) {
            if (reads.KeptListed(hop)) {
                return false;
            }
        }
    }
    // The value the walk selects -- it does not vary, so Default reads what
    // every time reads -- unless a standing override or upstream value
    // answers first along the walk.
    *value = input.constant;
    const auto number = [value](const VtValue &v) {
        if (v.IsHolding<float>()) {
            *value = v.UncheckedGet<float>();
        } else if (v.IsHolding<double>()) {
            *value = float(v.UncheckedGet<double>());
        } else {
            return false;
        }
        return true;
    };
    VtValue authored;
    if (selected && selected.Get(&authored, UsdTimeCode::Default())) {
        number(authored);
    }
    for (const SdfPath &hop : walk) {
        for (const auto &entry : reads.overrides) {
            if (entry.first == hop) {
                return number(entry.second);
            }
        }
        const auto upstream = B.upstream.find(hop);
        if (upstream != B.upstream.end()) {
            return number(upstream->second);
        }
    }
    return true;
}

/// Gates a Range \p revision: a static sparse weight object with an
/// exactly-zero resolved default the pins can hold, a packet the kernel half
/// (RigExecRevisionGateHolds) admits, and no current-phase or operation-
/// domain weighting. Writes the groups the authored indices inside
/// [0, N0) touch into \p written, pins the default and, in Export, adds its
/// read to \p pinned.
bool
GateRevision(const RoleReads &reads,
             const RigExecBakedProgramImpl::GeomChain &chain,
             RigExecBakedProgramImpl::GeomRevision *revision,
             std::vector<char> *written, std::vector<SdfPath> *pinned)
{
    const RigExecBakedProgramImpl &B = reads.B;
    if (!B.groupGates || revision->weightObject < 0 ||
        revision->weightCurrentPhase || revision->weightOperationDomain) {
        return false;
    }
    float value = 1.0f;
    SdfPath head;
    if (!GateDefault(reads, revision->weightObject, &value, &head) ||
        value != 0.0f) {
        return false;
    }
    const RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(revision->weightObject)];
    // The packet a run assembles from this object, as far as the gate reads
    // it: the static weight's arrays at the default the pin holds.
    RigExecMoverParameters probe;
    probe.enabled = true;
    probe.valid = true;
    probe.weights.representation = weight.representation;
    probe.weights.rangePolicy = weight.rangePolicy;
    probe.weights.values = weight.values;
    probe.weights.indices = weight.indices;
    probe.weights.defaultWeight = 0.0f;
    probe.weights.valid = true;
    probe.skinningMethod = _tokens->classicLinear;
    probe.blendSurfaceFrame = false;
    if (!RigExecRevisionGateHolds(revision->op, probe)) {
        return false;
    }
    const std::vector<int> &bounds = chain.groupBounds;
    written->assign(bounds.size() - 1, 0);
    for (const int index : weight.indices) {
        if (index < 0 || size_t(index) >= chain.groupPointCount) {
            continue;
        }
        const size_t g =
            size_t(std::upper_bound(bounds.begin(), bounds.end(), index) -
                   bounds.begin()) -
            1;
        if (g < written->size()) {
            (*written)[g] = 1;
        }
    }
    RolePin pin;
    pin.kind = RolePin::Kind::WeightDefaultZero;
    pin.weightObject = revision->weightObject;
    revision->pins.push_back(pin);
    if (reads.exportMode) {
        pinned->push_back(head);
    }
    return true;
}

/// Gives every revision of \p chain its role, groups, gates, pins and
/// chunks. A chain is range-pipelined when chunking would split
/// it, it holds two or more groups and one of its revisions takes the Range
/// role; every other chain keeps its Legacy chunks. The skinning methods and
/// delta spaces of an eligible chain are compiled rig keys: an authored edit to one
/// rebuilds, so only an override or upstream value flips a role within the compiled rig.
void
AssignChainRoles(RigExecBakedProgramImpl *program, const RoleReads &reads,
                 RigExecBakedProgramImpl::GeomChain *chain, size_t points)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t groups = ChainGroupCount(B, points);
    const bool eligible = ChainRangeCount(B, points) >= 2 && groups >= 2;
    chain->groupBounds.clear();
    chain->groupPointCount = 0;
    chain->baseOwner.reset();
    chain->baseGroups.clear();
    chain->resultIds.clear();
    chain->spareIds.clear();
    std::vector<GeomRole> candidate(chain->revisions.size(), GeomRole::Legacy);
    std::vector<std::vector<SdfPath>> pinned(chain->revisions.size());
    bool anyRange = false;
    for (size_t r = 0; r < chain->revisions.size(); ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision = chain->revisions[r];
        revision.role = GeomRole::Legacy;
        revision.rangeRole = false;
        revision.blendChunked = false;
        revision.groupWritten.clear();
        revision.enteringWriter.clear();
        revision.groups.clear();
        revision.groupIds.clear();
        revision.haloGroups.clear();
        revision.haloPoints.clear();
        revision.mushGroups.clear();
        revision.pins.clear();
        revision.surfaceCache.PinTopology(false);
        PartitionAtBuild(B, &revision);
        if (!eligible) {
            continue;
        }
        // What cuts a halo (a smooth's topology; a delta mush's topology,
        // edges and iteration count) is a compiled rig key: an authored edit
        // rebuilds.
        const bool smooth = revision.op == RigExecRevisionOp::Smooth;
        const bool mush = revision.op == RigExecRevisionOp::DeltaMush;
        const auto rebuildOn = [&B](const SdfPath &path) {
            if (path.IsPropertyPath()) {
                B.rebuild.insert(path);
                B.named.insert(path);
                B.prims.insert(path.GetPrimPath());
            }
        };
        for (const RigExecRevisionLeafRole role :
             {RigExecRevisionLeafRole::SkinningMethod,
              RigExecRevisionLeafRole::DeltaSpace,
              RigExecRevisionLeafRole::LatticeEvaluation,
              RigExecRevisionLeafRole::TopologyCounts,
              RigExecRevisionLeafRole::TopologyIndices,
              RigExecRevisionLeafRole::MushEdges}) {
            const bool topology =
                role == RigExecRevisionLeafRole::TopologyCounts ||
                role == RigExecRevisionLeafRole::TopologyIndices;
            if ((topology && !smooth && !mush) ||
                (role == RigExecRevisionLeafRole::MushEdges && !mush)) {
                continue;
            }
            const int k = revision.leaves.decl.Role(role);
            if (k >= 0) {
                rebuildOn(revision.leaves.decl.keys[size_t(k)].path);
            }
        }
        if (mush && revision.leaves.decl.scalarBegin >= 0) {
            rebuildOn(revision.leaves.decl
                          .keys[size_t(revision.leaves.decl.scalarBegin)]
                          .path);
        }
        candidate[r] = ClassifyRevision(reads, &revision, &pinned[r]);
        anyRange = anyRange || candidate[r] == GeomRole::Range;
    }
    if (!anyRange) {
        // Legacy. A skin or blend that a usable leaf made Whole keeps its
        // LeafToken pin, so a flip to Range rebuilds into a range chain.
        return;
    }
    chain->groupPointCount = points;
    chain->groupBounds.resize(groups + 1);
    for (size_t g = 0; g <= groups; ++g) {
        chain->groupBounds[g] = int(RigExecPointRangeBound(points, groups, g));
    }
    chain->baseGroups.resize(groups);
    chain->resultIds.assign(groups, RigExecGroupSource());
    chain->spareIds.assign(groups, RigExecGroupSource());
    std::vector<int> lastWriter(groups, -1);
    for (size_t r = 0; r < chain->revisions.size(); ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision = chain->revisions[r];
        revision.role = candidate[r];
        revision.rangeRole = revision.role == GeomRole::Range;
        revision.groupWritten.assign(groups, 1);
        const bool skin = revision.op == RigExecRevisionOp::Skin;
        if (revision.rangeRole && skin &&
            !PartitionSkinGroups(&revision, chain->groupBounds)) {
            // Unreachable: the classification read the same layout.
            revision.role = GeomRole::Whole;
            revision.rangeRole = false;
            pinned[r].clear();
        }
        revision.surfaceCache.PinTopology(TopologyPinned(revision));
        if (revision.rangeRole) {
            if (!skin) {
                revision.chunks.assign(groups,
                                       RigExecBakedProgramImpl::GeomChunk());
                for (size_t g = 0; g < groups; ++g) {
                    revision.chunks[g].begin = chain->groupBounds[g];
                    revision.chunks[g].end = chain->groupBounds[g + 1];
                }
                revision.chunked = false;
                revision.partitionIndices = VtIntArray();
            }
            if (B.gpuDeformMode == RigExecGpuDeformMode::Off &&
                revision.op == RigExecRevisionOp::BlendShape &&
                !revision.blendChannels.empty()) {
                bool fixed = true;
                std::vector<std::vector<int>> keys(groups);
                for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
                    std::vector<char> touches(groups, 0);
                    for (const auto &sample :
                         revision.blendChannels[c].samples) {
                        if (sample.blendShape.IsEmpty() || !sample.shapeValid ||
                            sample.layoutRefused ||
                            !reads.Usable(revision.leaves,
                                          sample.indicesLeaf) ||
                            !reads.Usable(revision.leaves,
                                          sample.offsetsLeaf) ||
                            reads.Reaches(revision.leaves.hops[size_t(
                                sample.indicesLeaf)]) ||
                            reads.Reaches(revision.leaves.hops[size_t(
                                sample.offsetsLeaf)])) {
                            fixed = false;
                            break;
                        }
                        const auto indices =
                            reads.Ints(revision.leaves, sample.indicesLeaf);
                        const auto &leaves = revision.leaves;
                        const auto value = RigExecSampleRevisionLeaf(
                            leaves.decl.keys[size_t(sample.offsetsLeaf)],
                            leaves.attributes[size_t(sample.offsetsLeaf)],
                            &reads.overlay, UsdTimeCode::Default(),
                            &B.upstream);
                        const auto offsets =
                            value.IsHolding<VtVec3fArray>()
                                ? value.UncheckedGet<VtVec3fArray>()
                                : VtVec3fArray();
                        RigExecBlendSampleLayout layout;
                        if (!RigExecBuildGeometryBlendLayout(offsets, indices,
                                                             points, &layout)) {
                            fixed = false;
                            break;
                        }
                        if (indices.empty())
                            std::fill(touches.begin(), touches.end(), 1);
                        for (int i : indices)
                            if (i >= 0 && size_t(i) < points) {
                                const size_t g =
                                    size_t(std::upper_bound(
                                               chain->groupBounds.begin(),
                                               chain->groupBounds.end(), i) -
                                           chain->groupBounds.begin() - 1);
                                touches[g] = 1;
                            }
                    }
                    if (!fixed)
                        break;
                    for (size_t g = 0; g < groups; ++g)
                        if (touches[g])
                            keys[g].push_back(int(c));
                }
                if (fixed &&
                    std::any_of(keys.begin(), keys.end(),
                                [](const auto &key) { return !key.empty(); })) {
                    revision.blendChunked = true;
                    for (size_t g = 0; g < groups; ++g) {
                        revision.chunks[g].key = std::move(keys[g]);
                        revision.chunks[g].blendDeltas.resize(size_t(
                            chain->groupBounds[g + 1] - chain->groupBounds[g]));
                    }
                    for (const auto &channel : revision.blendChannels)
                        for (const auto &sample : channel.samples)
                            for (int k :
                                 {sample.indicesLeaf, sample.offsetsLeaf}) {
                                RolePin pin;
                                pin.kind = RolePin::Kind::NoLayoutOverride;
                                pin.leaf = k;
                                revision.pins.push_back(pin);
                                if (reads.exportMode)
                                    pinned[r].push_back(
                                        revision.leaves.decl.keys[size_t(k)]
                                            .path);
                            }
                }
            }
            std::vector<char> written;
            if (GateRevision(reads, *chain, &revision, &written, &pinned[r])) {
                revision.groupWritten = std::move(written);
            }
            if (revision.op == RigExecRevisionOp::Smooth) {
                BuildSmoothHalo(reads, *chain, &revision, points);
            } else if (revision.op == RigExecRevisionOp::DeltaMush) {
                BuildMushHalo(reads, *chain, &revision, points);
            }
            B.exportPinnedPaths.insert(pinned[r].begin(), pinned[r].end());
        } else if (skin && revision.skinTopologyFixed &&
                   PartitionSkinGroups(&revision, chain->groupBounds)) {
            // A Whole keyed skin: one speculative chunk per group.
        } else {
            // A Whole one-chunk revision: PartitionAtBuild's single chunk,
            // which a skin's whole fuse runs unkeyed.
            revision.chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
            revision.chunked = false;
            revision.partitionIndices = VtIntArray();
        }
        revision.enteringWriter = lastWriter;
        revision.groups.assign(groups, RigExecGroupState<GfVec3f>());
        revision.groupIds.assign(groups, RigExecGroupSource());
        for (size_t g = 0; g < groups; ++g) {
            if (!revision.groupWritten[g]) {
                continue;
            }
            lastWriter[g] = int(r);
            // The two buffers its writer alternates between, sized to the
            // group here, on the owning thread, so a run allocates nothing.
            const size_t size =
                size_t(chain->groupBounds[g + 1] - chain->groupBounds[g]);
            for (auto &own : revision.groups[g].own) {
                own = std::make_shared<std::vector<GfVec3f>>(size);
            }
        }
    }
}

/// The chain index of the revision whose slot holds group \p g of point
/// version \p version of \p chain, or -1 for the base: the last revision
/// before the version that writes g, unless a revision a cycle set aside
/// (which passes the base through) comes first.
int
GroupWriter(const RigExecBakedProgramImpl::GeomChain &chain, size_t version,
            size_t g)
{
    for (size_t q = std::min(version, chain.revisions.size()); q > 0; --q) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            chain.revisions[q - 1];
        if (revision.rangeSetAside) {
            return -1;
        }
        if (g < revision.groupWritten.size() && revision.groupWritten[g]) {
            return int(q - 1);
        }
    }
    return -1;
}

/// Appends RevisionOut reads of `chunkBase + g` for every written group g,
/// one range per run of consecutive written groups.
void
DeclareWrittenGroupReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                         std::vector<RigExecBakedSlotRange> *reads)
{
    const size_t groups = revision.groupWritten.size();
    for (size_t g = 0; g < groups;) {
        if (!revision.groupWritten[g]) {
            ++g;
            continue;
        }
        size_t end = g + 1;
        while (end < groups && revision.groupWritten[end]) {
            ++end;
        }
        reads->push_back(RigExecBakedRange(RigExecBakedSlotDomain::RevisionOut,
                                           revision.chunkBase + int(g),
                                           revision.chunkBase + int(end)));
        g = end;
    }
}

} // namespace

void
RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.gpuDeformMode = RigExecGpuDeformModeFromEnv();
    B.chainChunkBegin.assign(B.chains.size(), 0);
    B.chainChunkEnd.assign(B.chains.size(), 0);
    B.chunkRevision.clear();
    B.exportPinnedPaths.clear();
    const RoleReads roleReads(B);
    int nextChunk = 0;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        // The chain's vertex groups, roles, gates and pins: every Range
        // revision of it carries the group bounds as its chunks.
        AssignChainRoles(&B, roleReads, &chain, BuildPointCount(B, chain));
        const size_t groups =
            chain.groupBounds.empty() ? 0 : chain.groupBounds.size() - 1;
        RigExecBakedStep &input =
            AddGeometryStep(&B, RigExecBakedStepKind::ChainInputs, int(c));
        input.maxDiagnostics = 0;
        input.reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainInput, int(c)));
        input.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
        // Revision ids are handed out chain by chain, so one chain's
        // revisions are a contiguous RANGE: point version v of the chain is
        // revision first + v - 1's, and the status sweep reads the whole
        // chain as one declared range. The chunk ids inside them are
        // contiguous for the same reason and at the same grain.
        // Build preindexed every revision before WeightField capture;
        // all readers and operations consume those same immutable ids.
        B.chainChunkBegin[c] = nextChunk;
        const int first = B.chainRevisionBegin[c];
        const int last = B.chainRevisionEnd[c];
        B.revisionFuseStep.resize(size_t(last), -1);
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            const int id = first + int(r);
            // The points entering this revision, which its chunks, its fuse
            // and a current-phase assemble read.
            const RigExecBakedPointVersion entering{int(c), int(r)};
            const bool skin = revision.op == RigExecRevisionOp::Skin;
            revision.influences.assign(revision.influenceSlots.size(),
                                       GfMatrix4d(1.0));
            // The table the PACKET carries for a skin revision, and never
            // anything else: identity, sized to the influence count and
            // written once, here.
            // The packet is assembled before the matrices are folded, so it
            // cannot carry them -- that is what lets a chunk start on its own
            // joints without waiting for every joint of the rig. What the
            // assembler does with the table is check its SHAPE and its
            // elements' finiteness, and identities pass both; the real
            // table's finite/affine check is the fold's, and the fuse ANDs it
            // in exactly where the assembler's would have landed.
            revision.packetInfluences.assign(revision.influenceSlots.size(),
                                             GfMatrix4d(1.0));
            revision.rangeInputs = RigExecRevisionRangeInputs();
            revision.joinSeen.clear();
            revision.rangeRefusals = 0;
            const GeomRole role = revision.role;
            // RevisionOut ids: a Range revision's G groups (an unwritten one
            // has no writer and no reader); a Whole one's chunks, then the G
            // groups its fuse publishes; a Legacy one's chunks.
            const size_t own = revision.chunks.size();
            const size_t published = role == GeomRole::Range   ? groups
                                     : role == GeomRole::Whole ? own + groups
                                                               : own;
            revision.chunkBase = nextChunk;
            nextChunk += int(published);
            B.revisionChunkBase.push_back(revision.chunkBase);
            B.revisionChunkCount.push_back(int(published));
            B.chunkRevision.insert(B.chunkRevision.end(), published, id);

            // The fold comes FIRST for every operation but a skin, because
            // every other operation's packet carries the matrix it was folded
            // from. A skin revision's does not, so its fold comes after the
            // assemble -- which is also where it learns the skinning method,
            // and so which form of the table the chunks will want.
            const auto addFold = [&] {
                RigExecBakedStep &fold = AddGeometryStep(
                    &B, RigExecBakedStepKind::InfluenceFold, id);
                // The body gates folding on this chain's current base availability.
                fold.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
                DeclareMatrixReads(revision, &fold);
                // The delta a geometry-domain constraint measured for this
                // mover IS this revision's transform, so the fold waits for
                // the constraint step that writes it.
                if (revision.constraintDelta >= 0) {
                    fold.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::ConstraintDelta,
                                        revision.constraintDelta));
                }
                // A pose-walk read phase reads the frame records its lists
                // name, each written by one FrameMatrix step.
                DeclareFrameRecordReads(revision, &fold.reads);
                if (skin) {
                    fold.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionPacket, id));
                }
                fold.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
            };
            if (revision.weightField >= 0) {
                RigExecBakedStep &field =
                    AddGeometryStep(&B, RigExecBakedStepKind::WeightField,
                                    revision.weightField);
                field.maxDiagnostics = 0;
                RigExecBakedDeclareWeightField(&B, &field);
            }
            const auto addStatic = [&] {
                RigExecBakedStep &assemble = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionStatic, id);
                // One line per declared phase that resolved to nothing. The
                // defaultWeight line moved to the fuse, which is the first
                // step that knows both halves of "the packet is valid".
                assemble.maxDiagnostics = revision.binding.phases.size();
                if (!skin) {
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                assemble.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket,
                                        revision.weightObject));
                }
                if (revision.weightField >= 0)
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightField,
                                        revision.weightField));
                if (revision.weightCurrentPhase)
                    assemble.maxDiagnostics += 1;
                if (revision.driverFramesSolver >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Aggregate,
                                        revision.driverFramesSolver));
                }
                // A channel driven by a pose interpolator waits for the
                // interpolator's step, which is the edge that makes a drag
                // that cannot reach the driver leave this revision alone.
                for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     revision.blendChannels) {
                    if (!revision.blendChunked && channel.poseWeight >= 0) {
                        assemble.reads.push_back(
                            RigExecBakedOne(RigExecBakedSlotDomain::PoseWeight,
                                            channel.poseWeight));
                    }
                }
                // The versions its phased inputs and blend samples are bound
                // to. The AtPrim transform phase is the fold's read.
                DeclareRevisionPointReads(B, revision, &assemble.reads);
                assemble.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
            };
            if (skin) {
                addStatic();
                addFold();
            } else {
                addFold();
                addStatic();
            }
            // A Range revision's group steps and a Whole keyed skin's
            // speculative chunks are one per group and read group g of the
            // entering version alone; every other chunk reads the version.
            const bool perGroup = role == GeomRole::Range ||
                                  (role == GeomRole::Whole && revision.chunked);
            for (size_t k = 0; k < revision.chunks.size(); ++k) {
                if (role == GeomRole::Range && !revision.groupWritten[k]) {
                    // A gated group: the entering group is this revision's.
                    continue;
                }
                RigExecBakedStep &chunk = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionChunk, id, int(k));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                chunk.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.blendChunked) {
                    for (int channel : revision.chunks[k].key) {
                        const int weight =
                            revision.blendChannels[size_t(channel)].poseWeight;
                        if (weight >= 0)
                            chunk.reads.push_back(RigExecBakedOne(
                                RigExecBakedSlotDomain::PoseWeight, weight));
                    }
                }
                if (role == GeomRole::Range && skin) {
                    // The fold's validity, which the group step's decision
                    // shares with the join.
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                if (revision.chunked) {
                    // Waits only on its own influences; the fuse decides use.
                    const RigExecBakedSlotDomain domain =
                        RigExecBakedOwnMatrixDomain(revision);
                    for (const int position : revision.chunks[k].key) {
                        const int slot =
                            revision.influenceSlots[size_t(position)];
                        if (slot >= 0) {
                            chunk.reads.push_back(
                                RigExecBakedOne(domain, slot));
                        }
                    }
                } else if (skin) {
                    // One chunk is the whole array, so there is nothing to
                    // speculate about: it skins against the folded table.
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                if (perGroup) {
                    // The pipelining edge: group k of the entering version,
                    // in its last writer's slot, and nothing of any join.
                    RigExecBakedDeclareGroupRead(B, int(c), r, k, &chunk.reads);
                    if (role == GeomRole::Range &&
                        k < revision.haloGroups.size()) {
                        for (const int h : revision.haloGroups[k]) {
                            RigExecBakedDeclareGroupRead(
                                B, int(c), r, size_t(h), &chunk.reads);
                        }
                    }
                } else {
                    // Which buffer holds the entering points is decided by
                    // the previous fuse, so the read is that fuse's version.
                    DeclarePointVersionRead(B, entering, &chunk.reads);
                }
                chunk.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut,
                                    revision.chunkBase + int(k)));
            }
            {
                RigExecBakedStep &fuse =
                    AddGeometryStep(&B, RigExecBakedStepKind::RevisionFuse, id);
                // The one the defaultWeight check can emit.
                fuse.maxDiagnostics = 1;
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
                fuse.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    // The packet the MoverFailed arm consults. The assemble
                    // declares it too and the fuse waits on the assemble
                    // either way, so this changes no order. Every slot a
                    // step touches must be declared by that step.
                    fuse.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket,
                                        revision.weightObject));
                }
                // Range summaries read group metadata directly; no previous
                // summary lies on the path to this revision's consumers.
                if (role == GeomRole::Range) {
                    DeclareWrittenGroupReads(revision, &fuse.reads);
                } else {
                    fuse.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut, revision.chunkBase,
                        revision.chunkBase + int(revision.chunks.size())));
                }
                if (role == GeomRole::Range) {
                    for (size_t g = 0; g < groups; ++g) {
                        if (!revision.groupWritten[g]) {
                            RigExecBakedDeclareGroupRead(B, int(c), r, g,
                                                         &fuse.reads);
                        }
                    }
                } else {
                    DeclarePointVersionRead(B, entering, &fuse.reads);
                }
                if (role == GeomRole::Whole) {
                    // A Whole fuse publishes every group: an applied group
                    // from its chunks, a refused one passed from the
                    // entering group it reads here.
                    for (size_t g = 0; g < groups; ++g) {
                        RigExecBakedDeclareGroupRead(B, int(c), r, g,
                                                     &fuse.reads);
                    }
                    fuse.writes.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut,
                        revision.chunkBase + int(revision.chunks.size()),
                        revision.chunkBase + int(revision.chunks.size()) +
                            int(groups)));
                }
                fuse.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone, id));
                fuse.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, id));
                B.revisionFuseStep[size_t(id)] = int(B.steps.size()) - 1;
            }
        }
        B.chainChunkEnd[c] = nextChunk;
        {
            RigExecBakedStep &status =
                AddGeometryStep(&B, RigExecBakedStepKind::ChainStatus, int(c));
            // One MoverFailed line per revision of the chain, which is the
            // sweep's whole output.
            status.maxDiagnostics = chain.revisions.size();
            status.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // Every revision's published status; the last one's
            // `currentSource` among them names version n, the final points.
            if (last > first) {
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionDone, first, last));
            }
            status.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[d];
            const int id = int(B.derivedIndex.size());
            B.derivedIndex.emplace_back(int(c), int(d));
            derived.revision.influences.assign(
                derived.revision.influenceSlots.size(), GfMatrix4d(1.0));
            RigExecBakedStep &step =
                AddGeometryStep(&B, RigExecBakedStepKind::Derived, id);
            step.maxDiagnostics = derived.revision.binding.phases.size() + 1;
            // Final chain points: recomputeNormals and recomputeExtent
            // describe the geometry as published. The
            // target's own authored base is prologue state, read beside the
            // chain's.
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedBase, id));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // A derived revision's binding carries phases like any other, so
            // it reads the versions they are bound to. Its fold runs inside
            // this step, so it reads the frame records an AtPrim transform
            // phase names too.
            DeclareRevisionPointReads(B, derived.revision, &step.reads);
            DeclareFrameRecordReads(derived.revision, &step.reads);
            DeclareMatrixReads(derived.revision, &step);
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedOut, id));
        }
    }
}

// The vertex groups of a range chain (bakedProgramImpl.h).

namespace {
// What RigExecBakedGroupAt answers for a group the chain does not have.
// Constant-initialized, so no reader meets an initialization guard.
const RigExecPointsRef<GfVec3f> kNoGroup{};
} // namespace

const RigExecPointsRef<GfVec3f> &
RigExecBakedGroupAt(const RigExecBakedProgramImpl::GeomChain &chain,
                    size_t version, size_t g)
{
    // EvalCore's one writer resolution (rigExecEvalCore/chain.h); the
    // group refs themselves stay in this adapter's chain.
    return eval::ChainGroupAt<GfVec3f>(
        chain.revisions.size(), version, g,
        [&chain](size_t revision, size_t group) {
            return revision < chain.revisions.size() &&
                   group < chain.revisions[revision].groupWritten.size() &&
                   chain.revisions[revision].groupWritten[group] != 0;
        },
        [&chain](size_t revision) {
            return chain.revisions[revision].rangeSetAside;
        },
        [&chain](size_t group, const RigExecPointsRef<GfVec3f> &none)
            -> const RigExecPointsRef<GfVec3f> & {
            return group < chain.baseGroups.size()
                       ? chain.baseGroups[group].published
                       : none;
        },
        [&chain](size_t writer, size_t group,
                 const RigExecPointsRef<GfVec3f> &none)
            -> const RigExecPointsRef<GfVec3f> & {
            const auto &groups = chain.revisions[writer].groups;
            return group < groups.size() ? groups[group].published : none;
        },
        [&chain](size_t group) {
            RigExecGroupSource source;
            source.slot = -1 - int64_t(group);
            source.version = group < chain.baseGroups.size()
                                 ? chain.baseGroups[group].version
                                 : 0;
            return source;
        },
        [&chain](size_t writer, size_t group) {
            RigExecGroupSource source;
            source.version = group < chain.revisions[writer].groups.size()
                                 ? chain.revisions[writer].groups[group].version
                                 : 0;
            return source;
        },
        nullptr, kNoGroup);
}

RigExecGroupSource
RigExecBakedGroupSourceAt(const RigExecBakedProgramImpl &B, int chain,
                          size_t version, size_t g)
{
    RigExecGroupSource source;
    if (chain < 0 || size_t(chain) >= B.chains.size()) {
        return source;
    }
    const RigExecBakedProgramImpl::GeomChain &geom = B.chains[size_t(chain)];
    const int writer = GroupWriter(geom, version, g);
    if (writer < 0) {
        source.slot = -1 - int64_t(g);
        source.version =
            g < geom.baseGroups.size() ? geom.baseGroups[g].version : 0;
        return source;
    }
    const RigExecBakedProgramImpl::GeomRevision &revision =
        geom.revisions[size_t(writer)];
    source.slot = RigExecBakedGroupSlot(
        B, B.chainRevisionBegin[size_t(chain)] + writer, g);
    source.version =
        g < revision.groups.size() ? revision.groups[g].version : 0;
    return source;
}

void
RigExecBakedDeclareGroupRead(const RigExecBakedProgramImpl &B, int chain,
                             size_t version, size_t g,
                             std::vector<RigExecBakedSlotRange> *reads)
{
    if (chain < 0 || size_t(chain) >= B.chains.size()) {
        return;
    }
    const int writer = GroupWriter(B.chains[size_t(chain)], version, g);
    const int slot =
        writer < 0 ? -1
                   : RigExecBakedGroupSlot(
                         B, B.chainRevisionBegin[size_t(chain)] + writer, g);
    reads->push_back(
        slot < 0 ? RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, chain)
                 : RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut, slot));
}

int
RigExecBakedGroupSlot(const RigExecBakedProgramImpl &B, int id, size_t g)
{
    if (id < 0 || size_t(id) >= B.revisionIndex.size()) {
        return -1;
    }
    const auto &[chain, r] = B.revisionIndex[size_t(id)];
    const RigExecBakedProgramImpl::GeomRevision &revision =
        B.chains[size_t(chain)].revisions[size_t(r)];
    if (g >= revision.groupWritten.size()) {
        return -1;
    }
    switch (revision.role) {
    case RigExecBakedRevisionRole::Range:
        return revision.groupWritten[g] ? revision.chunkBase + int(g) : -1;
    case RigExecBakedRevisionRole::Whole:
        return revision.chunkBase + int(revision.chunks.size()) + int(g);
    case RigExecBakedRevisionRole::Legacy:
        break;
    }
    return -1;
}

bool
RigExecBakedRolesStand(const RigExecBakedProgramImpl &B)
{
    // Worker-safe (the frozen worker calls it on its clone): reads sampled
    // leaves and the clone's own tables, compares tokens and paths by
    // identity, builds no token or path and posts no diagnostic.
    const auto reached = [&B](const std::vector<SdfPath> &hops) {
        for (const SdfPath &hop : hops) {
            if (B.interactiveOverrides) {
                for (const RigExecValueOverride &o : *B.interactiveOverrides) {
                    const TfToken &name =
                        o.attribute.IsEmpty() ? o.computation : o.attribute;
                    if (name == hop.GetNameToken() &&
                        o.prim == hop.GetPrimPath()) {
                        return true;
                    }
                }
            }
            for (const auto &entry : B.upstream) {
                if (entry.first == hop) {
                    return true;
                }
            }
        }
        return false;
    };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        // Only this run's sample counts (the prologue just took it):
        // `haveBase` is the previous run's, false on a program's first run.
        if (chain.groupPointCount > 0 && chain.sampledHaveBase &&
            chain.sampledBase.size() != chain.groupPointCount) {
            return false;
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            const RigExecBakedPathLeaves &leaves = revision.leaves;
            for (const RigExecBakedRolePin &pin : revision.pins) {
                switch (pin.kind) {
                case RigExecBakedRolePin::Kind::LeafToken:
                    if (leaves.sampled &&
                        leaves.Value<TfToken>(pin.leaf, TfToken()) !=
                            pin.token) {
                        return false;
                    }
                    break;
                case RigExecBakedRolePin::Kind::WeightDefaultZero:
                    if (pin.weightObject < 0 ||
                        size_t(pin.weightObject) >= B.weightObjects.size() ||
                        RigExecBakedLeaf(
                            B, B.weightObjects[size_t(pin.weightObject)]
                                   .defaultWeight) != 0.0f) {
                        return false;
                    }
                    break;
                case RigExecBakedRolePin::Kind::NoLayoutOverride:
                    if (pin.leaf >= 0 &&
                        size_t(pin.leaf) < leaves.hops.size() &&
                        reached(leaves.hops[size_t(pin.leaf)])) {
                        return false;
                    }
                    break;
                }
            }
        }
    }
    return true;
}

// One frame, geometry half.
// The packet is assembled by the same RigExecAssembleParameters the dynamic
// path calls and the kernel is the same shared kernel; only the VdfNetwork
// around them is baked away, so the accounting it performed -- a revision
// runs when its inputs changed, and not otherwise -- is performed here
// instead, by the four steps a revision is made of.

namespace {

/// The points of version \p version of \p chain (RigExecBakedPointVersion):
/// 0 is the authored base, v > 0 what revision v - 1's fuse left.
///
/// Not a buffer of its own: revision v - 1's `currentSource` names the
/// revision whose output buffer holds the value, so a pass-through version
/// names an earlier revision's buffer, or the base when it is -1.
void
PointsAt(const RigExecBakedProgramImpl::GeomChain &chain, size_t version,
         const GfVec3f **points, size_t *count)
{
    const int source =
        version == 0 ? -1 : chain.revisions[version - 1].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

bool
SamePoints(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b)
{
    return eval::PointsBitsEqualBody(a.data(), a.size(), b.data(), b.size());
}

bool
SamePoints(const VtVec3fArray &a, const VtVec3fArray &b)
{
    return eval::PointsBitsEqualBody(a.cdata(), a.size(), b.cdata(), b.size());
}

/// The influence matrices of \p revision, out of the tables the pose half's
/// ProviderMatrix steps published. Returns whether the table MOVED.
bool
FoldInfluences(const RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    bool changed = false;
    revision->haveTransform = revision->transformSlot >= 0;
    if (revision->haveTransform) {
        revision->transform =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSlot)]
                : B.baseMatrix[size_t(revision->transformSlot)];
    }
    // rigExec:space, out of the same table at the same phase, so this reads
    // exactly what the dynamic path's carry tap reads. Held on the revision
    // for the assemble, which may not read the tables itself: the fold is
    // the step that declared them.
    revision->haveCarry = revision->carrySpaceSlot >= 0;
    if (revision->haveCarry) {
        const GfMatrix4d &carry =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->carrySpaceSlot)]
                : B.baseMatrix[size_t(revision->carrySpaceSlot)];
        if (revision->carry != carry) {
            revision->carry = carry;
            changed = true;
        }
    }
    // A geometry-domain constraint's delta, if one was measured for this
    // mover: the pose walk solved the constraint and stashed the map from the
    // target's authored transform to the solved one, and THIS is the Matrix
    // revision that same constraint contributes. Find-guarded, and here
    // rather than in the assemble because the transform table is what this
    // step declares -- the assemble only READS it, and a step that writes a
    // slot it declared as a read is the declaration the executor trusts
    // being wrong.
    // The dynamic path applies it before the final-phase substitution rather
    // than after. The two orders can only disagree for a revision that has
    // both a bound transform provider and a delta, which cannot arise: a
    // geometry-domain constraint's binding.transform is empty -- that is what
    // makes the delta the only source of the matrix.
    if (revision->constraintDelta >= 0 &&
        B.deltaPresent[size_t(revision->constraintDelta)]) {
        revision->transform = B.deltaValues[size_t(revision->constraintDelta)];
        revision->haveTransform = true;
    }
    // A read phase naming a POINT IN THE POSE WALK, which is the general
    // form `base`, `preceding` and `final` abbreviate: the provider's matrix
    // as it stood immediately after one named constraint, out of the frame
    // records bound at Build instead of out of the dense tables.
    // AFTER the delta, exactly as the dynamic path orders the two, and the
    // two can no more both apply here than they can there: an AtPrim phase
    // needs a bound rigExec:transform to name a frame chain of, and a
    // geometry-domain constraint's binding.transform is empty -- which is
    // what makes its delta the only source of the matrix.
    // The first valid record of the list answers: the last record the walk
    // made under the phase's prim. A list with none valid leaves the
    // dense-table value standing and says nothing, as the dynamic path does:
    // every named constraint under the prim declined to record this run (a
    // geometry-domain exit, an unusable frame).
    const auto phased = [&B](const std::vector<int> &records,
                             GfMatrix4d *matrix) {
        for (const int record : records) {
            if (B.frameMatrixValid[size_t(record)]) {
                *matrix = B.frameMatrix[size_t(record)];
                return true;
            }
        }
        return false;
    };
    const bool atPrim =
        revision->binding.transformPhase.kind == RigExecReadPhaseKind::AtPrim;
    if (atPrim && phased(revision->transformRecords, &revision->transform)) {
        // Set rather than left alone: the dynamic path binds its matrix
        // pointer whenever the store answered, whatever the tap held.
        revision->haveTransform = true;
    }
    const bool hasReference = revision->op == RigExecRevisionOp::Matrix &&
                              !revision->influenceSlots.empty();
    const auto referenceMatrix = [&B, revision](size_t index) {
        const size_t slot = size_t(revision->influenceSlots[index]);
        return revision->finalPhase ? B.finalMatrix[slot] : B.baseMatrix[slot];
    };
    if (revision->haveTransform) {
        GfMatrix4d space(1.0), reference(1.0), referenceSpace(1.0);
        const bool hasSpace = revision->transformSpaceSlot >= 0;
        if (hasSpace)
            space = revision->finalPhase
                        ? B.finalMatrix[size_t(revision->transformSpaceSlot)]
                        : B.baseMatrix[size_t(revision->transformSpaceSlot)];
        if (hasReference)
            reference = referenceMatrix(0);
        const bool hasReferenceSpace =
            hasReference && revision->influenceSlots.size() > 1;
        if (hasSpace && hasReferenceSpace)
            referenceSpace = referenceMatrix(1);
        revision->transform = RigExecGeometryMatrixInPointFrame(
            revision->transform, hasSpace ? &space : nullptr,
            hasReference ? &reference : nullptr,
            hasReferenceSpace ? &referenceSpace : nullptr,
            revision->posedPoints,
            revision->haveCarry ? &revision->carry : nullptr);
    }
    for (size_t k = 0; k < revision->influenceSlots.size(); ++k) {
        const size_t slot = size_t(revision->influenceSlots[k]);
        GfMatrix4d matrix =
            revision->finalPhase ? B.finalMatrix[slot] : B.baseMatrix[slot];
        // Every influence shares the one declared phase, so the substitution
        // is per ENTRY and the provider is the entry's own. A skin never
        // gets here: compile refuses an AtPrim phase whose binding.transform
        // is empty, as a skin's is. A matrix mover's influences are its
        // reference providers; the Matrix op does not read them, but its
        // packet carries this table into the executed decision, as the
        // walk's fold does.
        if (atPrim && k < revision->influenceRecords.size()) {
            phased(revision->influenceRecords[k], &matrix);
        }
        if (revision->influences[k] != matrix) {
            revision->influences[k] = matrix;
            changed = true;
        }
    }
    return changed;
}

/// EvalCore's skin bodies (rigExecEvalCore/skin.h) over the core's Gf
/// numbers: the SIMD blend and narrowed rows are the rigExecMath kernels,
/// the split is the Gf dual-quaternion palette.
struct BakedSkinMath {
    using Vec3f = GfVec3f;
    using Vec3d = GfVec3d;
    using Mat4 = GfMatrix4d;
    using ScaledDualQuat = RigExecScaledDualQuat;
    static Vec3f ToFloat(const Vec3d &v)
    {
        return Vec3f(v);
    }
    static Vec3d ToDouble(const Vec3f &v)
    {
        return Vec3d(v);
    }
    static void NarrowSkinRows(const Mat4 &t, float *rows)
    {
        RigExecNarrowSkinRows(t, rows);
    }
    static void
    ApplyLinearBlendSkinSimd(const Vec3f *in, Vec3f *out,
                             const eval::SkinLayoutViewT<BakedSkinMath> &part,
                             const float *rows)
    {
        RigExecSkinLayout layout;
        layout.transforms = part.transforms;
        layout.transformCount = part.transformCount;
        layout.indices = part.indices;
        layout.weights = part.weights;
        layout.indexCount = part.indexCount;
        layout.elementSize = part.elementSize;
        layout.pointCount = part.pointCount;
        RigExecApplyLinearBlendSkinSimd(in, out, layout, rows);
    }
};

/// The packet as EvalCore's SkinPacketView: the method classified from
/// its token, the layout arrays borrowed from the handle or the packet.
eval::SkinPacketView
BakedSkinPacket(const RigExecMoverParameters &p)
{
    eval::SkinPacketView v;
    v.method = p.skinningMethod == _tokens->classicLinear
                   ? eval::kSkinMethodClassicLinear
                   : (p.skinningMethod == _tokens->dualQuaternion
                          ? eval::kSkinMethodDualQuaternion
                          : eval::kSkinMethodUnknown);
    if (const RigExecSkinTopology *const topology = p.skinTopology.get()) {
        v.hasTopology = true;
        v.topologyValidated = topology->validated;
        v.topologyInfluenceCount = topology->influenceCount;
        v.topologyIndices = topology->indices.data();
        v.topologyWeights = topology->weights.data();
        v.topologyIndexCount = topology->indices.size();
        v.topologyElementSize = topology->elementSize;
    }
    v.indices = p.skinIndices.data();
    v.weights = p.skinWeights.data();
    v.indexCount = p.skinIndices.size();
    v.weightCount = p.skinWeights.size();
    v.elementSize = p.skinElementSize;
    return v;
}

/// The revision's own folded table, in the forms the kernels want it in.
///
/// What a chunk of a revision cut into ONE chunk skins against, and what the
/// fuse falls back to when the partition no longer describes the layout. A
/// chunked revision's chunks keep their own tables beside these and never
/// read them; the fold writes them anyway, because they are part of the
/// RevisionTransforms slot and the fuse may only READ that slot.
eval::SkinTransformsViewT<BakedSkinMath>
WholeTransformsView(RigExecBakedProgramImpl::GeomRevision *revision)
{
    return eval::SkinWholeTransforms<BakedSkinMath>(
        revision->influences.data(), revision->influences.size(),
        revision->rows.data(), !revision->rows.empty(),
        revision->palette.data(), revision->palette.size());
}

/// Narrows and splits \p revision's folded table for the method its packet
/// names, once per run rather than once per range.
///
/// Part of what InfluenceFold writes, for every skin revision and not only
/// an unchunked one: the whole-array fallback the fuse falls back to reads
/// these, and a step may not write a slot it declared as a read.
void
FoldTransformForms(RigExecBakedProgramImpl::GeomRevision *revision,
                   bool useSimd)
{
    eval::FoldSkinTransformForms<BakedSkinMath>(
        BakedSkinPacket(revision->parameters).method,
        revision->influences.data(), revision->influences.size(), useSimd,
        &revision->rows, &revision->palette);
}

/// Copies \p chunk's OWN influences out of the matrix slots, keeping the
/// narrowed and split forms beside them, and records whether any moved.
///
/// Only |key| entries, never the whole table: that is what makes a chunk's
/// per-run cost proportional to the joints its vertices actually reference.
/// The entries outside the key stay identity, and linear blend skinning
/// reaches an entry only through an index of one of the chunk's own vertices,
/// so they are never read.
void
GatherChunkTransforms(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      RigExecBakedProgramImpl::GeomChunk *chunk)
{
    const auto matrixAt = [&](int position) -> const GfMatrix4d * {
        if (position < 0 ||
            size_t(position) >= revision.influenceSlots.size()) {
            return nullptr;
        }
        const int slot = revision.influenceSlots[size_t(position)];
        if (slot < 0) {
            return nullptr;
        }
        return revision.finalPhase ? &B.finalMatrix[size_t(slot)]
                                   : &B.baseMatrix[size_t(slot)];
    };
    eval::GatherSkinChunkTransforms<BakedSkinMath>(
        BakedSkinPacket(revision.parameters).method, chunk->key, matrixAt,
        &chunk->transforms, &chunk->rows, &chunk->palette, &chunk->keyChanged);
}

/// The view onto \p chunk's own tables.
eval::SkinTransformsViewT<BakedSkinMath>
ChunkTransformsView(const RigExecBakedProgramImpl::GeomChunk &chunk,
                    bool useSimd)
{
    return eval::SkinChunkTransforms<BakedSkinMath>(
        chunk.transforms.data(), chunk.transforms.size(), chunk.rows.data(),
        !chunk.rows.empty(), useSimd, chunk.palette.data(),
        chunk.palette.size());
}

/// The skin kernel's validation over the half RevisionStatic holds: the
/// packet, the layout against the entering count and the envelope it
/// resolved (the fold's `influencesValid` is the matrix half, which the fuse
/// ANDs in). One definition for the revision's decision and every chunk's
/// guard.
bool
SkinPacketIsUsable(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return eval::SkinPacketIsUsable(revision.parameters.valid,
                                    revision.parameters.kind == "skin",
                                    revision.layoutUsable, revision.envelopeOk);
}

/// A skin revision's apply-or-fail answer from that half: past it a linear
/// blend cannot fail. Range DQ checks blends in InfluenceFold; Whole DQ
/// defers that decision until its speculative chunks finish.
RigExecRevisionAcceptance
SkinAcceptance(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return static_cast<RigExecRevisionAcceptance>(
        eval::SkinAcceptance(SkinPacketIsUsable(revision),
                             BakedSkinPacket(revision.parameters).method,
                             revision.role == RigExecBakedRevisionRole::Range));
}

/// One vertex range of a skin revision: seed the revision's own output buffer
/// from the preceding points, deform in place, blend the envelope back.
///
/// The three pieces RigExecRunRevisionKernel performs for the whole array,
/// performed over a range -- with every whole-array decision (the packet
/// check, the layout validation, the envelope's resolution, the size check)
/// already made by the steps that own them.
///
/// \p whole runs the full-range kernel instead, which is the same body
/// inside the kernel's own parallel loop; a revision cut into one chunk uses
/// it so that an unpartitioned mesh keeps the same threading.
/// \p target is the buffer written, at the full count: staging for a chunk.
bool
SkinRange(RigExecBakedProgramImpl::GeomRevision *revision,
          const GfVec3f *preceding,
          const eval::SkinTransformsViewT<BakedSkinMath> &view, size_t begin,
          size_t end, bool whole, bool useSimd, std::vector<GfVec3f> *target)
{
    // "Apply once", over the same range: the envelope was resolved at the
    // full count by RevisionStatic, because resolving it is atomic over the
    // whole array, and every array here is indexed absolutely.
    return eval::SkinRange<BakedSkinMath>(
        BakedSkinPacket(revision->parameters), revision->fullStrength,
        revision->envelope.data(), preceding, view, begin, end, whole, useSimd,
        !revision->parameters.skinTopology, RigExecGeometryRanges(), target);
}

/// The test capture (RigExecBakedProgramTesting::CapturePointReads): the
/// binding's answer for \p binding as its reader took it this run.
void
CapturePointRead(RigExecBakedProgramImpl *program,
                 const RigExecBakedPointsBinding &binding,
                 const GfVec3f *points, size_t count, bool answered)
{
    RigExecBakedProgramImpl &B = *program;
    if (!B.capturePointReads || binding.id < 0 ||
        size_t(binding.id) >= B.pointCaptures.size()) {
        return;
    }
    RigExecBakedPointCapture &capture = B.pointCaptures[size_t(binding.id)];
    capture.read = true;
    capture.bindingAnswered = answered;
    capture.bound =
        answered ? VtVec3fArray(points, points + count) : VtVec3fArray();
}

} // namespace

void
RigExecBakedVersionPoints(const RigExecBakedProgramImpl::GeomChain &chain,
                          size_t version, std::vector<GfVec3f> *scratch,
                          const GfVec3f **points, size_t *count)
{
    // The base, and every version of a chain without groups: one buffer.
    if (version == 0 || chain.groupBounds.size() < 2) {
        PointsAt(chain, version, points, count);
        return;
    }
    // Group g of the version is the ref its writer published. Consecutive
    // slices of one owner are one buffer, read in place; anything else is
    // gathered into the reader's own scratch.
    const size_t groups = chain.groupBounds.size() - 1;
    size_t total = 0;
    bool contiguous = true;
    const void *owner = nullptr;
    const GfVec3f *start = nullptr;
    const GfVec3f *next = nullptr;
    for (size_t g = 0; g < groups; ++g) {
        const RigExecPointsRef<GfVec3f> &ref =
            RigExecBakedGroupAt(chain, version, g);
        total += ref.count;
        if (!contiguous || ref.count == 0) {
            continue;
        }
        if (!start) {
            start = ref.data;
            owner = ref.owner.get();
        } else if (ref.owner.get() != owner || ref.data != next) {
            contiguous = false;
            continue;
        }
        next = ref.data + ref.count;
    }
    if (contiguous) {
        *points = start;
        *count = total;
        return;
    }
    scratch->resize(total);
    GfVec3f *out = scratch->data();
    for (size_t g = 0; g < groups; ++g) {
        const RigExecPointsRef<GfVec3f> &ref =
            RigExecBakedGroupAt(chain, version, g);
        if (ref.count) {
            std::memcpy(out, ref.data, ref.count * sizeof(GfVec3f));
        }
        out += ref.count;
    }
    *points = scratch->data();
    *count = total;
}

bool
RigExecBakedResolvePoints(const RigExecBakedProgramImpl &B,
                          const RigExecBakedPointsBinding &binding,
                          const GfVec3f **points, size_t *count)
{
    // EvalCore's one candidate walk (rigExecEvalCore/chain.h); the
    // version gather into the binding's own buffer stays adapter-local.
    const void *raw = nullptr;
    const bool answered = eval::ResolvePointsBody(
        binding.candidates, binding.finalRead,
        [](const RigExecBakedPointVersion &candidate) {
            return int64_t(candidate.chain);
        },
        [](const RigExecBakedPointVersion &candidate) {
            return int64_t(candidate.version);
        },
        [&B](size_t chain, bool finalRead) {
            return chain < B.chains.size() &&
                   (finalRead ? B.chains[chain].haveResult
                              : B.chains[chain].haveBase);
        },
        [&B](size_t chain, const void **out, size_t *outCount) {
            *out = B.chains[chain].result.cdata();
            *outCount = B.chains[chain].result.size();
            return true;
        },
        [&B, &binding](size_t chain, size_t version, const void **out,
                       size_t *outCount) {
            const GfVec3f *versionPoints = nullptr;
            size_t versionCount = 0;
            RigExecBakedVersionPoints(B.chains[chain], version, &binding.gather,
                                      &versionPoints, &versionCount);
            *out = versionPoints;
            *outCount = versionCount;
            return true;
        },
        &raw, count);
    *points = static_cast<const GfVec3f *>(raw);
    return answered;
}

void
RigExecBakedOverlayPointReads(RigExecBakedProgramImpl *program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const RigExecResolvedInputs &resolved,
                              std::vector<std::string> *diagnostics)
{
    if (revision->binding.phases.empty()) {
        return;
    }
    RigExecBakedProgramImpl &B = *program;
    revision->revisionInputs.SetChainedBase(&resolved);
    for (const RigExecBakedPointsBinding &binding : revision->pointBindings) {
        const GfVec3f *points = nullptr;
        size_t count = 0;
        const bool answered =
            RigExecBakedResolvePoints(B, binding, &points, &count);
        CapturePointRead(&B, binding, points, count, answered);
        if (answered) {
            // A final read shares the chain's published buffer, as the
            // walk's record of it does.
            revision->revisionInputs.SetProperty(
                binding.input,
                binding.finalRead
                    ? VtValue(
                          B.chains[size_t(binding.candidates[0].chain)].result)
                    : VtValue(VtVec3fArray(points, points + count)));
        } else if (binding.diagnoseMiss) {
            // A `preceding` tail is silent, as it is in the dynamic walk.
            diagnostics->push_back(binding.missDiagnostic);
        }
    }
}

namespace {

// Chain revisions read the authored base; derived revisions read final
// points.
//
// The provider values both assemblies take: everything but the stage reads.
RigExecProviderValues
RevisionValues(RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision,
               const GfVec3f *basePoints, size_t basePointCount)
{
    RigExecProviderValues values;
    // The fold decided whether there is a matrix at all -- a bound transform
    // provider, or a geometry-domain constraint's delta -- and wrote it.
    // Skin packets carry no matrix: the skin's static step precedes the fold,
    // and RigExecAssembleSkinParameters takes no transform.
    if (revision->haveTransform && revision->op != RigExecRevisionOp::Skin) {
        values.transform = &revision->transform;
    }
    // The carry the fold read beside the transform, for the wire assembler.
    // A skin's fold runs after its assemble, but a skin never names one.
    if (revision->haveCarry) {
        values.carry = &revision->carry;
    }
    // A skin revision's packet carries the IDENTITY table and never the
    // folded one: it is assembled before the matrices are folded, which is
    // what lets its chunks start on their own joints. Skin is the only
    // operation that binds influences at all, so every other operation's
    // packet carries the identity table the fold writes it.
    const std::vector<GfMatrix4d> &table =
        revision->op == RigExecRevisionOp::Skin ? revision->packetInfluences
                                                : revision->influences;
    if (!table.empty()) {
        values.influenceTransforms = &table;
    }
    // The weight object's packet, shared with every other mover that binds
    // it and built by its own step earlier in the frame. A mover copies EXEC
    // here (see bakedWeights.cpp): this is the packet the tap would have
    // carried. `inputs:defaultWeight` is used only when no packet is bound.
    if (revision->weightObject >= 0) {
        values.weights = revision->weightCurrentPhase
                             ? &revision->currentPhasePacket
                             : &B.weightPackets[size_t(revision->weightObject)];
    }
    // Only the operations that measure against the authored base read it; a
    // matrix, skin or wire revision never does.
    if (revision->op != RigExecRevisionOp::Matrix &&
        revision->op != RigExecRevisionOp::Skin &&
        revision->op != RigExecRevisionOp::Wire) {
        if (revision->op == RigExecRevisionOp::External) {
            // API4 exposes an owning vector view to plugins.
            if (basePointCount)
                values.basePoints.assign(basePoints,
                                         basePoints + basePointCount);
        } else {
            values.borrowsBasePoints = true;
            values.borrowedBasePoints = basePoints;
            values.borrowedBaseCount = basePointCount;
        }
    }
    // The driver solver's aggregate, which is what the dynamic path's
    // per-revision tap resolves to: exec computes one and the pose walk then
    // OVERRIDES the tap with its own solve, so the program's table -- the
    // same solve, by the same kernel -- is the authoritative value on both
    // paths. Declared as a read of the Aggregate slot, which is the edge
    // from the solver's Solve step to this revision.
    if (revision->driverFramesSolver >= 0) {
        values.driverFrames =
            &B.aggregates[size_t(revision->driverFramesSolver)];
    }
    // Compiled rig layouts were adopted in the PROLOGUE from the SkinTopology
    // op, which builds them by the dynamic cache's own rules
    // (RigExecBuildSkinTopology) -- so the two paths cannot hold different
    // arrays, and no step body reads one off the stage.
    if (revision->topologyResolved) {
        values.skinTopology = &revision->topology;
    }
    return values;
}

// The blend channels. Gathered here rather than inside the assembler
// because the dynamic walk gathers them here too, and the ORDER is the
// whole contract: channels in `binding.blendInputs` order, which compile
// sorted, and each channel's samples stable-sorted by activation. Float
// addition is not associative, so a different order is a different last
// bit of every point.
// The reads are the generation-wide resolved inputs' and NOT this
// revision's phase overlay's: a sample's own phase is looked up directly,
// which is what lets one blend sample read another chain while the rest of
// the channel reads the stage. \p read answers each one: the stage gather
// through the resolved inputs, the leaf gather from the leaves the prologue
// sampled through them. \p record writes each dense sample's `lastPoints`.
template <class Read>
void
GatherBlendChannels(RigExecBakedProgramImpl &B,
                    RigExecBakedProgramImpl::GeomRevision *revision,
                    const Read &read, bool record,
                    RigExecProviderValues *values)
{
    if (revision->blendChannels.empty()) {
        return;
    }
    std::vector<RigExecBlendChannel> channels;
    channels.reserve(revision->blendChannels.size());
    for (size_t c = 0; c < revision->blendChannels.size(); ++c) {
        RigExecBakedProgramImpl::GeomBlendChannel &bound =
            revision->blendChannels[c];
        RigExecBlendChannel channel;
        // A pose-driven weight is this run's slot -- unless an override
        // stands on the weight itself, which the dynamic walk's lookup
        // finds before it follows the connection and which the resolved
        // inputs therefore answer here too.
        if (revision->blendChunked && record) {
            channel.weight = 0.0f;
        } else if (bound.poseWeight >= 0 && !read.WeightHeld(bound)) {
            channel.weight = B.poseWeights[size_t(bound.poseWeight)];
        } else {
            read.Weight(bound, &channel.weight);
        }
        channel.weight = RigExecBlendInputWeight(channel.weight);
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            auto &boundSample = bound.samples[s];
            RigExecBlendSampleData sample;
            read.Activation(boundSample, &sample.activation);
            if (!boundSample.blendShape.IsEmpty()) {
                // Sparse: the shape the prologue resolved, shared by
                // pointer. The packet compares layouts by pointer, so an
                // unchanged sample costs one compare and not an array.
                sample.layout = boundSample.layout;
                channel.samples.push_back(std::move(sample));
                continue;
            }
            // A phased sample reads the version it is bound to, and the
            // resolved input when that chain read no base: silently,
            // as the dynamic gather falls back.
            const GfVec3f *phased = nullptr;
            size_t phasedCount = 0;
            const bool answered =
                boundSample.pointBinding.id >= 0 &&
                RigExecBakedResolvePoints(B, boundSample.pointBinding, &phased,
                                          &phasedCount);
            if (boundSample.pointBinding.id >= 0) {
                CapturePointRead(&B, boundSample.pointBinding, phased,
                                 phasedCount, answered);
            }
            if (record) {
                // The consumer needs an immutable view; export retention
                // changes only when the actual sample bytes/count changed.
                if (answered) {
                    const bool same =
                        boundSample.lastPoints.size() == phasedCount &&
                        (!phasedCount ||
                         !std::memcmp(boundSample.lastPoints.data(), phased,
                                      phasedCount * sizeof(GfVec3f)));
                    if (!same) {
                        if (phasedCount)
                            boundSample.lastPoints.assign(phased,
                                                          phased + phasedCount);
                        else
                            boundSample.lastPoints.clear();
                    }
                } else {
                    read.Points(boundSample, &boundSample.lastPoints);
                }
                sample.borrowsPoints = true;
                sample.borrowedPoints = boundSample.lastPoints.data();
                sample.borrowedCount = boundSample.lastPoints.size();
            } else if (answered) {
                if (phasedCount)
                    sample.points.assign(phased, phased + phasedCount);
            } else {
                read.Points(boundSample, &sample.points);
            }
            channel.samples.push_back(std::move(sample));
        }
        std::stable_sort(channel.samples.begin(), channel.samples.end(),
                         [](const RigExecBlendSampleData &a,
                            const RigExecBlendSampleData &b) {
                             return a.activation < b.activation;
                         });
        channels.push_back(std::move(channel));
    }
    if (revision->blendChunked && record) {
        if (RigExecBlendChannelsAreUsable(channels, values->BasePointCount())) {
            values->blendDeltas.assign(values->BasePointCount(), GfVec3f(0));
        } else {
            values->blendDeltas.clear();
        }
        if (revision->blendSamples != channels) {
            revision->blendSamples = std::move(channels);
            ++revision->deltasVersion;
        }
        return;
    }
    if (revision->gpuOwned) {
        // The program sums the channels on the device. The packet keeps the
        // sum's shape, zero deltas, so its validity and status are the
        // CPU path's; a structural failure empties it as below.
        if (RigExecBlendChannelsAreUsable(channels, values->BasePointCount())) {
            values->blendDeltas.assign(values->BasePointCount(), GfVec3f(0));
        } else {
            values->blendDeltas.clear();
        }
        revision->gpuChannels = std::move(channels);
        return;
    }
    // A structural failure leaves blendDeltas empty, which is what makes
    // the assembled packet invalid -- the same atomic MoverFailed
    // pass-through the kernel produces.
    if (!RigExecGeometryBlendDeltas(channels, values->BasePointData(),
                                    values->BasePointCount(),
                                    &values->blendDeltas)) {
        values->blendDeltas.clear();
    }
}

// The blend gather's reads, off the stage through the resolved inputs.
struct _StageBlendReads {
    const RigExecResolvedInputs &R;
    UsdTimeCode time;

    bool
    WeightHeld(const RigExecBakedProgramImpl::GeomBlendChannel &bound) const
    {
        return R.Find(bound.weightPath) != nullptr;
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const
    {
        R.GetAttribute(bound.weight, time, out);
    }
    void
    Activation(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
               float *out) const
    {
        R.GetAttribute(sample.activation, time, out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const
    {
        VtVec3fArray points;
        R.GetAttribute(sample.points, time, &points);
        out->assign(points.cbegin(), points.cend());
    }
};

// The same reads, from the revision's path leaves.
struct _LeafBlendReads {
    const std::vector<VtValue> &values;
    template <class T> T Value(int key, const T &fallback) const
    {
        return key >= 0 && size_t(key) < values.size() &&
                       values[size_t(key)].IsHolding<T>()
                   ? values[size_t(key)].UncheckedGet<T>()
                   : fallback;
    }

    bool
    WeightHeld(const RigExecBakedProgramImpl::GeomBlendChannel &bound) const
    {
        return Value<bool>(bound.weightHeldLeaf, false);
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const
    {
        *out = Value<float>(bound.weightLeaf, *out);
    }
    void
    Activation(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
               float *out) const
    {
        *out = Value<float>(sample.activationLeaf, *out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const
    {
        const VtVec3fArray points =
            Value<VtVec3fArray>(sample.pointsLeaf, VtVec3fArray());
        const bool same =
            out->size() == points.size() &&
            (points.empty() || !std::memcmp(out->data(), points.cdata(),
                                            points.size() * sizeof(GfVec3f)));
        if (!same) {
            if (points.empty())
                out->clear();
            else
                out->assign(points.cbegin(), points.cend());
        }
    }
};

// The packet off the stage, through \p R (or the revision's phase overlay
// over it), with RigExecAssembleParameters: a plugin revision bound to a
// value the region computes, and the shadow test's reference for the rest.
RigExecMoverParameters
AssembleRevisionFromStage(RigExecBakedProgramImpl &B,
                          const RigExecResolvedInputs &R,
                          RigExecBakedProgramImpl::GeomRevision *revision,
                          const GfVec3f *basePoints, size_t basePointCount,
                          UsdTimeCode time,
                          std::vector<std::string> *diagnostics, bool record,
                          RigExecExternalRevision *external)
{
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    if (values.borrowsBasePoints) {
        values.CopyBasePoints(&values.basePoints);
        values.borrowsBasePoints = false;
    }
    values.resolved = &R;
    // One overlay per REVISION: the generation-wide resolved inputs, plus
    // whatever this revision's declared phases resolve to through their
    // bindings. The assembler reads inputs by path and never learns a phase
    // exists, which is what lets a phase apply to any input. Built only for
    // a revision that declares one, and owned by that revision, because one
    // buffer shared by the walk is state two steps could be inside at once.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, R, diagnostics);
        values.resolved = &revision->revisionInputs;
    }
    // Independent host-side reference reads via stage Get; must not reuse the
    // production layout handle.
    for (auto &channel : revision->blendChannels)
        for (auto &sample : channel.samples) {
            if (sample.blendShape.IsEmpty())
                continue;
            auto layout = std::make_shared<RigExecBlendSampleLayout>();
            RigExecResolveBlendSampleLayout(B.stage, sample.blendShape,
                                            basePointCount, layout.get());
            sample.layout = std::move(layout);
        }
    GatherBlendChannels(B, revision, _StageBlendReads{R, time}, record,
                        &values);
    return RigExecAssembleParameters(revision->moverPrim, revision->op,
                                     revision->binding, values, time, external);
}

} // namespace

RigExecMoverParameters
RigExecBakedAssembleFromLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               const GfVec3f *basePoints, size_t basePointCount,
                               std::vector<std::string> *diagnostics,
                               std::vector<std::string> *missing,
                               size_t layoutPointCount,
                               RigExecExternalRevision *external)
{
    RigExecBakedProgramImpl &B = *program;
    if (layoutPointCount == size_t(-1))
        layoutPointCount = basePointCount;
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    // Two arrays the packet takes over rather than copies: the summed blend
    // deltas `values` owns, and a Range lattice's held rest (the chain base
    // at the version RevisionStatic checked, `restBaseHeld`), which the last
    // packet gives up because this assembly replaces it.
    values.lendBlendDeltas = true;
    if (revision->op == RigExecRevisionOp::Lattice && revision->restBaseHeld &&
        revision->parameters.restPoints.size() == basePointCount) {
        values.retainedRest = &revision->parameters.restPoints;
    }
    RigExecRevisionLeafView view;
    view.decl = &revision->leaves.decl;
    if (revision->blendChunked) {
        auto &values = revision->leaves.consumedValues;
        values.resize(revision->leaves.decl.keys.size());
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::RevisionStatic;
        for (size_t k = 0; k < values.size(); ++k)
            values[k] =
                RigExecBakedGeometryReadsLeaf(*revision, step, k)
                    ? RigExecBakedResolvePathLeaf(B, revision->leaves, k)
                    : revision->leaves.decl.keys[k].fallback;
        view.values = &values;
    } else {
        view.values = &RigExecBakedResolvePathLeaves(B, revision->leaves);
    }
    view.missing = missing;
    if (revision->op == RigExecRevisionOp::External) {
        view.external = &revision->externalPayload;
    }
    // The phase overlay, as the stage assembly builds it: a phased points
    // read takes the version it is bound to before its leaf.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, *B.resolvedInputs,
                                      diagnostics);
        view.phased = &revision->revisionInputs;
    }
    // Derived normalization consumes only the declared sparse RawDefault
    // leaves and this target's raw base cardinality, inside the selected body.
    // A layout is a pure function of those, so it is rebuilt only when the
    // leaves' content versions or the point count moved since it was built;
    // a leaf a walk or a produced value answers is not keyed so, and
    // rebuilds every time.
    const RigExecBakedPathLeaves &sources = revision->leaves;
    const auto rawVersion = [&sources, &view](int k, uint64_t *version) {
        const auto at = [k](const std::vector<int> &v) {
            return size_t(k) < v.size() ? v[size_t(k)] : -1;
        };
        if (k < 0 || size_t(k) >= sources.versions.size() ||
            size_t(k) >= view.values->size() || at(sources.walks) >= 0 ||
            at(sources.exactVersions) >= 0 ||
            at(sources.exactRecordIndices) >= 0) {
            return false;
        }
        *version = sources.versions[size_t(k)];
        return true;
    };
    for (auto &channel : revision->blendChannels)
        for (auto &sample : channel.samples) {
            if (sample.blendShape.IsEmpty())
                continue;
            uint64_t offsetsVersion = 0, indicesVersion = 0;
            const bool keyed =
                rawVersion(sample.offsetsLeaf, &offsetsVersion) &&
                rawVersion(sample.indicesLeaf, &indicesVersion);
            if (keyed && sample.layoutKeyed && sample.layout &&
                sample.layoutOffsetsVersion == offsetsVersion &&
                sample.layoutIndicesVersion == indicesVersion &&
                sample.layoutPointCount == layoutPointCount) {
                continue;
            }
            sample.layoutKeyed = keyed;
            sample.layoutOffsetsVersion = offsetsVersion;
            sample.layoutIndicesVersion = indicesVersion;
            sample.layoutPointCount = layoutPointCount;
            ++sample.layoutBuilds;
            const auto &raw = *view.values;
            const VtVec3fArray offsets =
                sample.offsetsLeaf >= 0 &&
                        size_t(sample.offsetsLeaf) < raw.size() &&
                        raw[size_t(sample.offsetsLeaf)]
                            .IsHolding<VtVec3fArray>()
                    ? raw[size_t(sample.offsetsLeaf)]
                          .UncheckedGet<VtVec3fArray>()
                    : VtVec3fArray();
            const VtIntArray indices =
                sample.indicesLeaf >= 0 &&
                        size_t(sample.indicesLeaf) < raw.size() &&
                        raw[size_t(sample.indicesLeaf)].IsHolding<VtIntArray>()
                    ? raw[size_t(sample.indicesLeaf)].UncheckedGet<VtIntArray>()
                    : VtIntArray();
            auto layout = std::make_shared<RigExecBlendSampleLayout>();
            layout->pointCount = layoutPointCount;
            if (sample.shapeValid)
                RigExecBuildGeometryBlendLayout(offsets, indices,
                                                layoutPointCount, layout.get());
            if (!sample.layout ||
                !RigExecSameBlendLayout(*sample.layout, *layout))
                sample.layout = std::move(layout);
        }
    // The sum reuses the buffer the packet before last held: it assigns
    // zeros before it accumulates, and every failure leaves it empty. Only
    // where channels exist, which is where the sum always runs.
    if (revision->op == RigExecRevisionOp::BlendShape &&
        !revision->blendChannels.empty()) {
        values.blendDeltas.swap(revision->deltasSpare);
    }
    GatherBlendChannels(B, revision, _LeafBlendReads{*view.values},
                        /*record=*/true, &values);
    return RigExecAssembleGeometry(revision->kernelRecord.op,
                                   revision->kernelRecord.binding, view, values,
                                   external);
}

namespace {

// The packet of \p revision this run: from its path leaves wherever they
// assemble it, off the stage for a plugin revision bound to region values.
RigExecMoverParameters
AssembleRevision(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 const GfVec3f *basePoints, size_t basePointCount,
                 UsdTimeCode time, RigExecBakedStep *step,
                 size_t layoutPointCount,
                 RigExecExternalRevision *external = nullptr)
{
    if (revision->leaves.decl.assembles) {
        return RigExecBakedAssembleFromLeaves(
            &B, revision, basePoints, basePointCount, &step->diagnostics,
            nullptr, layoutPointCount, external);
    }
    RigExecMoverParameters invalid;
    invalid.valid = false;
    step->diagnostics.push_back(
        revision->moverPathText +
        ": mover has no declared stage-free API4 assembly");
    return invalid;
}

/// The packet as EvalCore's EnvelopePacketView: representation and range
/// policy classified to its integer codes, the arrays borrowed.
eval::EnvelopePacketView
BakedEnvelopeView(const RigExecWeightPacket &w)
{
    eval::EnvelopePacketView v;
    v.valid = w.valid;
    v.defaultWeight = w.defaultWeight;
    v.values = w.values.data();
    v.valueCount = w.values.size();
    v.indices = w.indices.data();
    v.indexCount = w.indices.size();
    v.rangePolicyEmpty = w.rangePolicy.IsEmpty();
    v.representation =
        w.representation == rigExec::SchemaValueText::constant
            ? eval::kWeightConstant
            : (w.representation == rigExec::SchemaValueText::sparse
                   ? eval::kWeightSparse
                   : (w.representation == rigExec::SchemaValueText::dense
                          ? eval::kWeightDense
                          : eval::kWeightRepresentationUnknown));
    v.rangePolicy = w.rangePolicy == rigExec::SchemaValueText::strict
                        ? eval::kWeightPolicyStrict
                        : (w.rangePolicy == rigExec::SchemaValueText::clamp
                               ? eval::kWeightPolicyClamp
                               : eval::kWeightPolicyUnknown);
    return v;
}

/// The packet's envelope resolved at the full \p count, as ResolveAll into
/// `envelope` would leave it (unchanged when it fails), with
/// `envelopeVersion` moved exactly when the bytes did.
void
ResolveEnvelope(RigExecBakedProgramImpl::GeomRevision *revision, size_t count)
{
    revision->envelopeOk = eval::ResolveEnvelopeInto(
        revision->parameters.weights, count, &revision->envelope,
        &revision->resolveScratch,
        [revision](std::vector<float> *field, std::vector<float> *scratch) {
            RigExecBakedNoteFloats(field, scratch, &revision->envelopeVersion);
        });
}

/// The fuse's one diagnostic, about an enabled revision whose packet is
/// invalid: the out-of-range scalar weight, or the invalid common envelope
/// of a bound weight object. A whole revision's fuse and a range-pipelined
/// revision's join emit it in the same place.
void
EmitInvalidPacketLine(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      bool packetValid, std::vector<std::string> *diagnostics)
{
    if (revision.parameters.enabled && !packetValid &&
        revision.weightObject < 0) {
        // Read by RevisionStatic, which always runs: a step body that
        // went to the stage for a value would be a step outside its own
        // declarations, and the cone could not tell when it moved.
        const float scalar = revision.defaultWeight;
        if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
            diagnostics->push_back(
                "MoverFailed " + revision.moverPathText +
                ": inputs:defaultWeight must be finite and in "
                "[0, 1]; revision passed through");
        }
    } else if (revision.parameters.enabled && !packetValid &&
               revision.weightObject >= 0 &&
               !(revision.weightCurrentPhase
                     ? revision.currentPhasePacket
                     : B.weightPackets[size_t(revision.weightObject)])
                    .valid) {
        // The other half of the same sentence, and the reason the arms
        // are exclusive: with a weight object bound, the assembler never
        // reads inputs:defaultWeight, so a rig whose scalar is out of
        // range and whose object is fine must stay silent.
        diagnostics->push_back(
            "MoverFailed " + revision.moverPathText +
            ": rigExec:weightObject produced an invalid common "
            "envelope; revision passed through");
    }
}

/// The version a fuse a cycle set aside publishes, the base passed
/// through (`currentSource` -1, which the set-aside reset writes before the
/// region): it follows \p base against the copy of it last carried.
void
PassBaseThrough(const VtVec3fArray &base,
                RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (!RigExecBakedSamePoints(base.cdata(), base.size(),
                                revision->passedPoints.data(),
                                revision->passedPoints.size())) {
        ++revision->doneVersion;
        revision->passedPoints.assign(base.cbegin(), base.cend());
    }
}

/// A gated packet that failed RigExecRevisionGateHolds while applying, or a
/// Range group step that met a stale partition or a count other than its
/// group's: a pin hole, counted for the owner to report after the region.
void
NoteGateViolation(RigExecBakedProgramImpl &B)
{
    B.gateViolations.fetch_add(1, std::memory_order_relaxed);
}

/// The "apply once" envelope RevisionStatic resolved at the full count, for
/// a group kernel that blends separately below full strength; else null.
const float *
GroupEnvelope(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return !revision.fullStrength && revision.envelopeOk
               ? revision.envelope.data()
               : nullptr;
}

/// Group \p g of the chain base as a content id (pointBlocks.h).
RigExecGroupSource
BaseGroupSource(const RigExecBakedProgramImpl::GeomChain &chain, size_t g)
{
    RigExecGroupSource source;
    source.slot = -1 - int64_t(g);
    source.version =
        g < chain.baseGroups.size() ? chain.baseGroups[g].version : 0;
    return source;
}

/// Makes \p revision's `groupIds[g]` \p source and returns whether it moved.
bool
NoteGroupId(RigExecBakedProgramImpl::GeomRevision *revision, size_t g,
            const RigExecGroupSource &source)
{
    if (revision->groupIds[g] == source) {
        return false;
    }
    revision->groupIds[g] = source;
    return true;
}

/// Sizes \p revision's `groupIds` to its groups (Build sizes them; a size
/// change is a publication that moved).
bool
SizeGroupIds(RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (revision->groupIds.size() == revision->groups.size()) {
        return false;
    }
    revision->groupIds.assign(revision->groups.size(), RigExecGroupSource());
    return true;
}

/// The version a join or Whole fuse of a revision a cycle set aside
/// publishes: the base groups passed through, by their content ids;
/// `doneVersion` moves only when those ids do.
void
PassBaseGroups(const RigExecBakedProgramImpl::GeomChain &chain,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->ran = false;
    revision->executed = false;
    revision->resultStatus = _tokens->operationCycle;
    bool moved = SizeGroupIds(revision);
    for (size_t g = 0; g < revision->groupIds.size(); ++g) {
        moved = NoteGroupId(revision, g, BaseGroupSource(chain, g)) || moved;
    }
    if (moved) {
        ++revision->doneVersion;
    }
}

/// The halo table of group \p g of the Range \p revision: a smooth or
/// a delta mush reads its halo groups of the entering version beside
/// its own, which its step declares (the scheduler ran their writers
/// first). Fills `haloPoints[g]`; false refuses the group.
bool
BakedChainGroupHalo(RigExecBakedProgramImpl &B,
                    const RigExecBakedProgramImpl::GeomChain &chain,
                    RigExecBakedProgramImpl::GeomRevision *revision,
                    size_t revisionIndex, size_t g,
                    const RigExecPointsRef<GfVec3f> &entering)
{
    const bool mush = revision->op == RigExecRevisionOp::DeltaMush;
    if (revision->op != RigExecRevisionOp::Smooth && !mush) {
        return true;
    }
    const size_t groups = chain.groupBounds.size() - 1;
    if (g >= revision->haloGroups.size() || g >= revision->haloPoints.size() ||
        revision->haloPoints[g].size() != groups ||
        (mush && g >= revision->mushGroups.size())) {
        return false;
    }
    std::vector<const GfVec3f *> &table = revision->haloPoints[g];
    std::fill(table.begin(), table.end(), nullptr);
    table[g] = entering.data;
    for (const int h : revision->haloGroups[g]) {
        const RigExecPointsRef<GfVec3f> &group =
            RigExecBakedGroupAt(chain, revisionIndex, size_t(h));
        if (group.count != size_t(chain.groupBounds[size_t(h) + 1] -
                                  chain.groupBounds[size_t(h)])) {
            return false;
        }
        table[size_t(h)] = group.data;
    }
    return true;
}

/// The chunked blend arm of a group step: the group's deltas summed
/// from its key's channels (blendRangeKernel.h), blended back over the
/// entering points through the envelope RevisionStatic resolved.
void
BakedChainBlendGroup(RigExecBakedProgramImpl &B,
                     RigExecBakedProgramImpl::GeomRevision *revision, size_t g,
                     size_t begin, size_t end,
                     const RigExecPointsRef<GfVec3f> &entering,
                     RigExecGroupState<GfVec3f> *state)
{
    const int k = RigExecGroupScratch(state, end - begin);
    auto &chunk = revision->chunks[g];
    auto &deltas = chunk.blendDeltas;
    deltas.assign(end - begin, GfVec3f(0));
    for (int c : chunk.key) {
        const auto &channel = revision->blendChannels[size_t(c)];
        const auto held = RigExecBakedResolvePathLeaf(
            B, revision->leaves, size_t(channel.weightHeldLeaf));
        float weight = 0.0f;
        if (channel.poseWeight >= 0 &&
            !(held.IsHolding<bool>() && held.UncheckedGet<bool>())) {
            weight = B.poseWeights[size_t(channel.poseWeight)];
        } else {
            const auto value = RigExecBakedResolvePathLeaf(
                B, revision->leaves, size_t(channel.weightLeaf));
            if (value.IsHolding<float>())
                weight = value.UncheckedGet<float>();
        }
        RigExecAccumulateBlendRange(revision->blendSamples[size_t(c)].samples,
                                    weight, begin, end, deltas.data());
    }
    const bool full = eval::EnvelopeIsFullStrength(
        BakedEnvelopeView(revision->parameters.weights));
    auto &out = *state->own[size_t(k)];
    for (size_t i = 0; i < end - begin; ++i)
        out[i] = RigExecBlendEnvelope(
            entering.data[i], entering.data[i] + deltas[i],
            full ? 1.0f : revision->rangeInputs.blendWeights[begin + i]);
    state->ok = true;
    RigExecPublishOwnGroup(state, k);
}

/// Speculative chunk \p g of the Whole keyed skin \p revision: group g
/// of the entering version skinned against the chunk's own key-filled
/// table into a buffer of the group's state, kept as its computed
/// result (RigExecNoteComputedGroup) for the fuse to publish or
/// discard. Like a Legacy keyed chunk it does not wait for the fold,
/// and its `ok` is sticky across runs it sits out.
bool
BakedChainWholeSkinChunk(const RigExecBakedProgramImpl &B,
                         const RigExecBakedProgramImpl::GeomChain &chain,
                         RigExecBakedProgramImpl::GeomRevision *revision,
                         size_t revisionIndex, size_t g)
{
    if (g >= revision->chunks.size() || g >= revision->groups.size() ||
        g + 1 >= chain.groupBounds.size()) {
        return true;
    }
    RigExecBakedProgramImpl::GeomChunk &chunk = revision->chunks[g];
    RigExecGroupState<GfVec3f> &state = revision->groups[g];
    chunk.keyChanged = false;
    const RigExecPointsRef<GfVec3f> &entering =
        RigExecBakedGroupAt(chain, revisionIndex, g);
    const size_t begin = size_t(chain.groupBounds[g]);
    const size_t end = size_t(chain.groupBounds[g + 1]);
    if (!revision->status.AllowsApply() || !SkinPacketIsUsable(*revision) ||
        revision->partitionStale ||
        revision->precedingCount != chain.groupPointCount || end < begin ||
        entering.count != end - begin) {
        state.ok = false;
        chunk.ok = false;
        return true;
    }
    GatherChunkTransforms(B, *revision, &chunk);
    const int k = RigExecGroupScratch(&state, end - begin);
    const bool ok = eval::ApplySkinGroupRange<BakedSkinMath>(
        BakedSkinPacket(revision->parameters), revision->fullStrength,
        revision->envelope.data(), ChunkTransformsView(chunk, B.useSimd),
        entering.data, revision->precedingCount, begin, end, B.useSimd,
        state.own[size_t(k)]->data());
    if (ok) {
        RigExecNoteComputedGroup(&state, k);
    }
    state.ok = ok;
    chunk.ok = ok;
    return true;
}

/// Whether chunk \p k of a Whole or Legacy revision holds an answer: a keyed
/// Whole skin's in its group state (with a computed result to publish), any
/// other chunk's in the chunk.
bool
ChunkHoldsAnswer(const RigExecBakedProgramImpl::GeomRevision &revision,
                 size_t k)
{
    if (revision.role == RigExecBakedRevisionRole::Whole && revision.chunked) {
        return k < revision.groups.size() && revision.groups[k].ok &&
               revision.groups[k].ownComputed >= 0;
    }
    return revision.chunks[k].ok;
}

} // namespace

namespace {
void
ResetGeometryRevision(RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->created = true;
    revision->ran = false;
    revision->output.clear();
    revision->currentSource = -1;
    revision->lastParameters = RigExecMoverParameters();
    revision->lastAuxPoints = VtVec3fArray();
    revision->lastStatus = RigExecMoverStatus();
    // No baseline survives: the next publication bumps the version anyway.
    revision->stagingFresh = false;
    revision->passedPoints.clear();
    // Nor a published group: each group and the join or fuse publish a new
    // version, above the one they last published (versions never go down).
    for (auto &group : revision->groups) {
        RigExecResetGroup(&group);
        group.ok = false;
    }
    std::fill(revision->groupIds.begin(), revision->groupIds.end(),
              RigExecGroupSource());
}
} // namespace
void
RigExecBakedAdoptRevisionLayout(RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (revision->op != RigExecRevisionOp::Skin)
        return;
    revision->topology = revision->layoutHandle;
    revision->topologyResolved = true;
    RigExecBakedAdoptPartition(revision);
}
namespace {
/// ChainInputs, a chain cut into groups: `lastBase` as groups. One immutable
/// owner shares the array's buffer and each base group is a ref into it, so
/// publishing the base copies no point. Each group's version moves exactly
/// when its bytes differ from the ones it last published, or on its first
/// publication since a reset. While the bytes stand (\p moved false) the
/// held refs already hold them and nothing is rebuilt.
void
PublishBaseGroups(RigExecBakedProgramImpl::GeomChain *chain, bool moved)
{
    // EvalCore's one publication body (rigExecEvalCore/chain.h); the
    // owner copy and its reads stay in this adapter's types.
    eval::PublishBaseGroupsBody<GfVec3f>(
        chain->groupBounds, moved, chain->baseGroups, chain->baseOwner,
        [chain] {
            return std::make_shared<const VtVec3fArray>(chain->lastBase);
        },
        [](const std::shared_ptr<const VtVec3fArray> &owner) {
            return owner->cdata();
        },
        [](const std::shared_ptr<const VtVec3fArray> &owner) {
            return owner->size();
        });
}

/// Group \p g of the base as a set-aside writer passes it through: \p state
/// shares the base group's ref, its id the base group's.
void
PassSetAsideBaseGroup(const RigExecBakedProgramImpl::GeomChain &chain, size_t g,
                      RigExecGroupState<GfVec3f> *state)
{
    if (g >= chain.baseGroups.size()) {
        return;
    }
    const RigExecGroupState<GfVec3f> &base = chain.baseGroups[g];
    RigExecGroupSource from;
    from.slot = -1 - int64_t(g);
    from.version = base.version;
    RigExecPublishPassedGroup(state, base.published, from);
}

/// What a set-aside join or Whole fuse publishes without running (owner
/// thread, before the region): every group it writes passes the base group
/// through, and its ids are the base groups'. RevisionDone's version moves
/// only when those ids differ from the ones it last published.
void
PassSetAsideBaseGroups(const RigExecBakedProgramImpl::GeomChain &chain,
                       RigExecBakedProgramImpl::GeomRevision *revision)
{
    const size_t groups = chain.groupBounds.size() - 1;
    bool moved = revision->groupIds.size() != groups;
    revision->groupIds.resize(groups);
    for (size_t g = 0; g < groups; ++g) {
        if (g < revision->groupWritten.size() && revision->groupWritten[g] &&
            g < revision->groups.size()) {
            PassSetAsideBaseGroup(chain, g, &revision->groups[g]);
        }
        RigExecGroupSource id;
        id.slot = -1 - int64_t(g);
        id.version =
            g < chain.baseGroups.size() ? chain.baseGroups[g].version : 0;
        if (revision->groupIds[g] != id) {
            revision->groupIds[g] = id;
            moved = true;
        }
    }
    if (moved) {
        ++revision->doneVersion;
    }
}
} // namespace
// EvalCore chain host (rigExecEvalCore/chain.h) over the baked
// program's Gf storage: the chain bodies own the decisions, this
// adapter binds its chains, revisions and counters.
struct BakedChainInputs {
    RigExecBakedProgramImpl *B = nullptr;
    bool chainIndexForObject(int32_t object, size_t *chain) const
    {
        if (object < 0 || size_t(object) >= B->chains.size()) {
            return false;
        }
        *chain = size_t(object);
        return true;
    }
    bool revisionIndexForObject(int32_t object, size_t *chain,
                                size_t *revision) const
    {
        if (object < 0 || size_t(object) >= B->revisionIndex.size()) {
            return false;
        }
        const auto &index = B->revisionIndex[size_t(object)];
        *chain = size_t(index.first);
        *revision = size_t(index.second);
        return *chain < B->chains.size() &&
               *revision < B->chains[*chain].revisions.size();
    }
    bool derivedIndexForObject(int32_t object, size_t *chain,
                               size_t *derived) const
    {
        if (object < 0 || size_t(object) >= B->derivedIndex.size()) {
            return false;
        }
        const auto &index = B->derivedIndex[size_t(object)];
        *chain = size_t(index.first);
        *derived = size_t(index.second);
        return *chain < B->chains.size() &&
               *derived < B->chains[*chain].derived.size();
    }
    bool chainHasBase(size_t chain) const
    {
        return B->chains[chain].haveBase;
    }
    int opCode(size_t chain, size_t revision) const
    {
        return int(B->chains[chain].revisions[revision].op);
    }
    bool useSimd() const
    {
        return B->useSimd;
    }
};

struct BakedChainState {
    RigExecBakedProgramImpl *B = nullptr;
    RigExecBakedStep *step = nullptr;
    UsdTimeCode time;
    VtVec3fArray derivedPublished;
    RigExecMoverParameters derivedParameters;
    RigExecMoverStatus derivedStatus;
    std::vector<GfVec3f> versionScratch;
    using Chain = RigExecBakedProgramImpl::GeomChain;
    using Revision = RigExecBakedProgramImpl::GeomRevision;
    Chain &chainAt(size_t chain)
    {
        return B->chains[chain];
    }
    Revision &revisionAt(size_t chain, size_t revision)
    {
        return B->chains[chain].revisions[revision];
    }
    eval::ChainInputsAccessT<GfVec3f, VtVec3fArray, VtVec3fArray>
    chainInputs(size_t chain)
    {
        Chain &c = chainAt(chain);
        eval::ChainInputsAccessT<GfVec3f, VtVec3fArray, VtVec3fArray> access;
        access.sampledHave = c.sampledHaveBase;
        access.sampled = &c.sampledBase;
        access.last = &c.lastBase;
        access.haveBase = &c.haveBase;
        access.haveResult = &c.haveResult;
        access.result = &c.result;
        access.resultVersion = &c.resultVersion;
        access.baseVersion = &c.baseVersion;
        access.scheduleDirty = &c.scheduleDirty;
        access.revisionCount = c.revisions.size();
        return access;
    }
    void resetChainRevision(size_t chain, size_t revision)
    {
        ResetGeometryRevision(&revisionAt(chain, revision));
    }
    bool revisionRangeNotAside(size_t chain, size_t revision)
    {
        const Revision &rev = revisionAt(chain, revision);
        return rev.rangeRole && !rev.rangeSetAside;
    }
    void setRevisionSource(size_t chain, size_t revision, int source)
    {
        revisionAt(chain, revision).currentSource = source;
    }
    void resetBaseGroups(size_t chain)
    {
        Chain &c = chainAt(chain);
        for (auto &group : c.baseGroups) {
            RigExecResetGroup(&group);
        }
        c.baseOwner.reset();
    }
    void clearChainResultIds(size_t chain)
    {
        chainAt(chain).resultIds.clear();
        chainAt(chain).spareIds.clear();
    }
    void countChainCreated(size_t chain)
    {
        for (auto &revision : chainAt(chain).revisions) {
            if (revision.created) {
                ++step->counters.revisionsCreated;
                revision.created = false;
            }
        }
    }
    void noteChainSchedules(size_t)
    {
        ++step->counters.schedulesBuilt;
    }
    void noteChainBaseDirty(size_t chain, bool dirty)
    {
        chainAt(chain).baseDirty = dirty;
    }
    void publishChainBaseGroups(size_t chain, bool moved)
    {
        PublishBaseGroups(&chainAt(chain), moved);
    }
    bool chainPointsEqual(size_t, const VtVec3fArray &a, const VtVec3fArray &b)
    {
        return SamePoints(a, b);
    }
    bool chainBaseValueEqual(size_t, const VtVec3fArray &a,
                             const VtVec3fArray &b)
    {
        return RigExecBakedHeadValueSame(VtValue(a), VtValue(b));
    }
    bool foldInfluences(size_t chain, size_t revision, bool *changed)
    {
        *changed = FoldInfluences(*B, &revisionAt(chain, revision));
        return true;
    }
    void setInfluencesChanged(size_t chain, size_t revision, bool changed)
    {
        revisionAt(chain, revision).influencesChanged = changed;
    }
    void setInfluencesValid(size_t chain, size_t revision, bool valid)
    {
        revisionAt(chain, revision).influencesValid = valid;
    }
    bool skinTransformsUsable(size_t chain, size_t revision)
    {
        const Revision &rev = revisionAt(chain, revision);
        return eval::SkinTransformsAreUsable<BakedSkinMath>(
            rev.influences.data(), rev.influences.size());
    }
    void foldTransformForms(size_t chain, size_t revision)
    {
        FoldTransformForms(&revisionAt(chain, revision), B->useSimd);
    }
    bool skinPacketUsable(size_t chain, size_t revision)
    {
        return SkinPacketIsUsable(revisionAt(chain, revision));
    }
    bool dualQuatPaletteSafe(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        const auto &palette = rev.palette;
        const auto layout = eval::SkinLayoutForPacket<BakedSkinMath>(
            BakedSkinPacket(rev.parameters), WholeTransformsView(&rev),
            rev.precedingCount);
        return (rev.dqWeightsSafe &&
                RigExecDualQuatPaletteSafe(palette.data(), palette.size())) ||
               RigExecDualQuatBlendsAccept(layout, [&](const int *indices,
                                                       const double *weights,
                                                       size_t count) {
                   RigExecScaledDualQuat blend;
                   return RigExecBlendScaledDualQuats(palette.data(),
                                                      palette.size(), indices,
                                                      weights, count, &blend);
               });
    }
    bool revisionIsRange(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).role ==
               RigExecBakedRevisionRole::Range;
    }
    bool revisionIsDualQuat(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).parameters.skinningMethod ==
               _tokens->dualQuaternion;
    }
    bool revisionGpuOwned(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).gpuOwned;
    }
    void noteRevisionGpuTouched(size_t chain, size_t revision)
    {
        revisionAt(chain, revision).gpuTouched = true;
    }
    void adoptRevisionLayout(size_t chain, size_t revision)
    {
        RigExecBakedAdoptRevisionLayout(&revisionAt(chain, revision));
    }
    bool applyCurrentPhasePacket(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        if (rev.weightCurrentPhase && rev.weightObject >= 0) {
            rev.currentPhasePacket = B->weightPackets[size_t(rev.weightObject)];
            const auto &weightField = B->weightFields[size_t(rev.weightField)];
            if (weightField.ok) {
                rev.currentPhasePacket.representation =
                    _tokens->denseRepresentation;
                rev.currentPhasePacket.values = weightField.values;
                rev.currentPhasePacket.indices.clear();
                rev.currentPhasePacket.defaultWeight = 0.0f;
                rev.currentPhasePacket.valid = true;
            } else {
                rev.currentPhasePacket = RigExecWeightPacket();
                step->diagnostics.push_back("current-phase weight failed: " +
                                            weightField.error);
            }
        }
        return true;
    }
    void assembleRevision(size_t chain, size_t revision)
    {
        Chain &c = chainAt(chain);
        Revision &rev = revisionAt(chain, revision);
        const bool rangeChain = rev.role != RigExecBakedRevisionRole::Legacy;
        const bool rangeLattice = rev.role == RigExecBakedRevisionRole::Range &&
                                  rev.op == RigExecRevisionOp::Lattice;
        if (rev.restBaseHeld &&
            (!rangeLattice || rev.restBaseVersion != c.baseVersion)) {
            rev.restBaseHeld = false;
        }
        RigExecExternalRevision external;
        RigExecMoverParameters assembled =
            AssembleRevision(*B, &rev, c.lastBase.cdata(), c.lastBase.size(),
                             time, step, c.lastBase.size(), &external);
        if (rangeChain && !rev.blendChunked &&
            rev.op == RigExecRevisionOp::BlendShape &&
            !RigExecBakedSamePoints(assembled.blendDeltas.data(),
                                    assembled.blendDeltas.size(),
                                    rev.parameters.blendDeltas.data(),
                                    rev.parameters.blendDeltas.size())) {
            ++rev.deltasVersion;
        }
        if (rev.op == RigExecRevisionOp::BlendShape) {
            rev.deltasSpare = std::move(rev.parameters.blendDeltas);
        }
        rev.parameters = std::move(assembled);
        rev.external = std::move(external);
        if (rangeLattice) {
            rev.restBaseHeld =
                !rev.parameters.restPoints.empty() &&
                rev.parameters.restPoints.size() == c.lastBase.size();
            rev.restBaseVersion = c.baseVersion;
        }
    }
    void publishAssembledStatus(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        rev.status =
            RigExecStatusForParameters(rev.parameters, rev.moverPathText);
        step->counters.revisionsBuilt = 1;
        const VtValue diagnosticWeight = RigExecBakedResolvePathLeaf(
            *B, rev.leaves, size_t(rev.defaultWeightLeaf));
        rev.defaultWeight = diagnosticWeight.IsHolding<float>()
                                ? diagnosticWeight.UncheckedGet<float>()
                                : 1.0f;
        rev.staticDirty = true;
        rev.lastDefaultWeight = rev.defaultWeight;
        rev.weightFieldPublished = false;
        Chain &c = chainAt(chain);
        if (rev.weightObject >= 0 && B->publishWeightFields) {
            const RigExecWeightPacket &packet =
                rev.weightCurrentPhase
                    ? rev.currentPhasePacket
                    : B->weightPackets[size_t(rev.weightObject)];
            if (packet.valid) {
                const size_t logicalCount =
                    rev.weightOperationDomain ? size_t(1) : c.lastBase.size();
                const bool shared =
                    !rev.weightCurrentPhase && rev.weightPacketValue >= 0 &&
                    size_t(rev.weightPacketValue) < B->opAdapter.values.size();
                const uint64_t packetRevision =
                    shared ? B->opAdapter.values[size_t(rev.weightPacketValue)]
                                 .revision
                           : 0;
                if (!shared || !rev.weightValuesHeld ||
                    rev.weightValuesPacketRevision != packetRevision ||
                    rev.weightValuesCount != logicalCount) {
                    std::vector<float> &resolved = rev.resolveScratch;
                    if (!packet.ResolveAll(logicalCount, &resolved)) {
                        resolved.assign(logicalCount, 0.0f);
                        for (size_t i = 0; i < logicalCount; ++i) {
                            const float w = packet.Resolve(i, logicalCount);
                            resolved[i] = w < 0.0f ? 0.0f : w;
                        }
                    }
                    RigExecBakedNoteFloats(&rev.publishedWeightValues,
                                           &resolved, &rev.weightValuesVersion);
                    rev.weightValuesHeld = shared;
                    rev.weightValuesPacketRevision = packetRevision;
                    rev.weightValuesCount = logicalCount;
                }
                rev.weightFieldPublished = true;
            }
        }
    }
    void finishRevisionStaticHead(size_t, size_t)
    {
    }
    void gpuRevisionStatic(size_t chain, size_t revision)
    {
        Chain &c = chainAt(chain);
        Revision &rev = revisionAt(chain, revision);
        const bool skin = rev.op == RigExecRevisionOp::Skin;
        const size_t count = c.lastBase.size();
        rev.precedingCount = count;
        rev.envelopeOk = true;
        rev.fullStrength = true;
        if (skin) {
            rev.layoutUsable =
                eval::SkinLayoutIsUsable(BakedSkinPacket(rev.parameters), count,
                                         rev.parameters.skinTransforms.size());
            rev.dqWeightsSafe =
                rev.layoutUsable &&
                rev.parameters.skinningMethod == _tokens->dualQuaternion &&
                RigExecDualQuatWeightsSafe(
                    eval::SkinLayoutForPacket<BakedSkinMath>(
                        BakedSkinPacket(rev.parameters),
                        eval::SkinTransformsOf<BakedSkinMath>(
                            rev.parameters.skinTransforms.data(),
                            rev.parameters.skinTransforms.size()),
                        count));
            rev.fullStrength = eval::EnvelopeIsFullStrength(
                BakedEnvelopeView(rev.parameters.weights));
            if (!rev.fullStrength) {
                ResolveEnvelope(&rev, count);
            }
            rev.acceptance = SkinAcceptance(rev);
        } else {
            rev.acceptance = RigExecRevisionKernelAcceptance(
                rev.op, rev.parameters, count, &rev.envelopeOk);
        }
    }
    void sizeRevisionStaging(size_t chain, size_t revision, size_t count)
    {
        Revision &rev = revisionAt(chain, revision);
        const bool staged =
            rev.role == RigExecBakedRevisionRole::Legacy ||
            (rev.role == RigExecBakedRevisionRole::Whole && !rev.chunked);
        if (staged && rev.stagingOutput.size() != count) {
            rev.stagingOutput.resize(count);
            rev.staticDirty = true;
        }
        rev.layoutUsable = false;
        rev.envelopeOk = true;
        rev.fullStrength = true;
        rev.partitionStale = false;
    }
    void setPrecedingCount(size_t chain, size_t revision, size_t count)
    {
        revisionAt(chain, revision).precedingCount = count;
    }
    size_t chainBaseCount(size_t chain)
    {
        return chainAt(chain).lastBase.size();
    }
    eval::EnvelopePacketView envelopeView(size_t chain, size_t revision)
    {
        return BakedEnvelopeView(
            revisionAt(chain, revision).parameters.weights);
    }
    void setFullStrength(size_t chain, size_t revision, bool full)
    {
        revisionAt(chain, revision).fullStrength = full;
    }
    void resolveRevisionEnvelope(size_t chain, size_t revision, size_t count)
    {
        ResolveEnvelope(&revisionAt(chain, revision), count);
    }
    void setAcceptance(size_t chain, size_t revision, int acceptance)
    {
        revisionAt(chain, revision).acceptance =
            RigExecRevisionAcceptance(acceptance);
    }
    int prepareRevisionRanges(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        return int(RigExecPrepareRevisionRanges(
            rev.op, rev.parameters, chainAt(chain).lastBase.size(),
            &rev.wireBasis, &rev.surfaceCache, &rev.rangeInputs,
            rev.restBaseHeld ? rev.restBaseVersion : 0));
    }
    int revisionKernelAcceptance(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        return int(RigExecRevisionKernelAcceptance(
            rev.op, rev.parameters, chainAt(chain).lastBase.size(),
            &rev.envelopeOk));
    }
    int skinRevisionAcceptance(size_t chain, size_t revision)
    {
        return int(SkinAcceptance(revisionAt(chain, revision)));
    }
    bool skinLayoutUsable(size_t chain, size_t revision, size_t count)
    {
        const Revision &rev = revisionAt(chain, revision);
        return eval::SkinLayoutIsUsable(BakedSkinPacket(rev.parameters), count,
                                        rev.parameters.skinTransforms.size());
    }
    void setLayoutUsable(size_t chain, size_t revision, bool usable)
    {
        revisionAt(chain, revision).layoutUsable = usable;
    }
    bool dualQuatWeightsSafe(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        return rev.layoutUsable &&
               rev.parameters.skinningMethod == _tokens->dualQuaternion &&
               RigExecDualQuatWeightsSafe(
                   eval::SkinLayoutForPacket<BakedSkinMath>(
                       BakedSkinPacket(rev.parameters),
                       eval::SkinTransformsOf<BakedSkinMath>(
                           rev.parameters.skinTransforms.data(),
                           rev.parameters.skinTransforms.size()),
                       chainAt(chain).lastBase.size()));
    }
    void setDualQuatWeightsSafe(size_t chain, size_t revision, bool safe)
    {
        revisionAt(chain, revision).dqWeightsSafe = safe;
    }
    bool revisionPartitionStale(size_t chain, size_t revision, size_t count)
    {
        Revision &rev = revisionAt(chain, revision);
        if (!rev.chunked) {
            return rev.partitionStale;
        }
        const RigExecSkinTopology *const topology =
            rev.parameters.skinTopology.get();
        const size_t indexCount = topology ? topology->indices.size()
                                           : rev.parameters.skinIndices.size();
        const int elementSize =
            topology ? topology->elementSize : rev.parameters.skinElementSize;
        return !rev.parameters.skinTopology ||
               rev.parameters.skinTopology != rev.partitionTopology ||
               indexCount != rev.partitionIndexCount ||
               elementSize != rev.partitionElementSize ||
               count != rev.partitionPointCount;
    }
    void setPartitionStale(size_t chain, size_t revision, bool stale)
    {
        revisionAt(chain, revision).partitionStale = stale;
    }
    bool gateCheckApplies(size_t chain, size_t revision)
    {
        const Revision &rev = revisionAt(chain, revision);
        if (rev.role != RigExecBakedRevisionRole::Range ||
            std::find(rev.groupWritten.begin(), rev.groupWritten.end(),
                      char(0)) == rev.groupWritten.end()) {
            return false;
        }
        return rev.parameters.valid && rev.status.AllowsApply() &&
               rev.acceptance == RigExecRevisionAcceptance::Applies;
    }
    bool groupGateHolds(size_t chain, size_t revision, size_t count)
    {
        const Revision &rev = revisionAt(chain, revision);
        const auto &owner = chainAt(chain);
        eval::GroupGateFacts facts;
        facts.enabled = true;
        facts.op = int(rev.op);
        facts.envelope = BakedEnvelopeView(rev.parameters.weights);
        facts.skinMethod = BakedSkinPacket(rev.parameters).method;
        facts.blendSurfaceFrame = rev.parameters.blendSurfaceFrame;
        facts.regularGrid = rev.parameters.latticeSettings.regularGrid;
        facts.hasWrittenProof = true;
        facts.groupBounds = owner.groupBounds.data();
        facts.groupBoundCount = owner.groupBounds.size();
        facts.groupWritten = rev.groupWritten.data();
        facts.groupWrittenCount = rev.groupWritten.size();
        facts.pointCount = count;
        return eval::GroupGateHolds(facts);
    }
    void noteGateViolation(size_t, size_t)
    {
        NoteGateViolation(*B);
    }
    bool revisionIsSkin(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).op == RigExecRevisionOp::Skin;
    }
    bool revisionWholeChunked(size_t chain, size_t revision)
    {
        const Revision &rev = revisionAt(chain, revision);
        return rev.role == RigExecBakedRevisionRole::Whole && rev.chunked;
    }
    bool runSingleChunk(size_t chain, size_t revision, int32_t part);
    eval::GroupStepAccessT<GfVec3f> groupStep(size_t chain, size_t revision,
                                              size_t g)
    {
        Chain &c = chainAt(chain);
        Revision &rev = revisionAt(chain, revision);
        eval::GroupStepAccessT<GfVec3f> access;
        access.state = g < rev.groups.size() ? &rev.groups[g] : nullptr;
        access.rangeSetAside = rev.rangeSetAside;
        access.applies = RigExecBakedRevisionApplies(rev);
        access.skin = rev.op == RigExecRevisionOp::Skin;
        access.blendChunked = rev.blendChunked;
        access.partitionStale = rev.partitionStale;
        access.fullStrength = rev.fullStrength;
        access.precedingCount = rev.precedingCount;
        access.chainPointCount = c.groupPointCount;
        access.chunksSize = rev.chunks.size();
        return access;
    }
    void groupRange(size_t chain, size_t, size_t g, size_t, size_t *begin,
                    size_t *end)
    {
        Chain &c = chainAt(chain);
        *begin = g < c.groupBounds.size() ? size_t(c.groupBounds[g]) : 0;
        *end = g + 1 < c.groupBounds.size() ? size_t(c.groupBounds[g + 1]) : 0;
    }
    const RigExecPointsRef<GfVec3f> &enteringGroup(size_t chain,
                                                   size_t revision, size_t g,
                                                   RigExecGroupSource *from)
    {
        Chain &c = chainAt(chain);
        *from = RigExecBakedGroupSourceAt(*B, int(chain), revision, g);
        return RigExecBakedGroupAt(c, revision, g);
    }
    void passBaseGroup(size_t chain, size_t, size_t g,
                       RigExecGroupState<GfVec3f> *state)
    {
        Chain &c = chainAt(chain);
        if (g < c.baseGroups.size()) {
            RigExecPublishPassedGroup(state, c.baseGroups[g].published,
                                      BaseGroupSource(c, g));
        } else {
            RigExecPublishPassedGroup(state, kNoGroup, BaseGroupSource(c, g));
        }
    }
    bool gatherChunkTransforms(size_t chain, size_t revision, size_t g)
    {
        Revision &rev = revisionAt(chain, revision);
        if (g >= rev.chunks.size()) {
            return false;
        }
        rev.chunks[g].keyChanged = false;
        GatherChunkTransforms(*B, rev, &rev.chunks[g]);
        return true;
    }
    bool runSkinGroup(size_t chain, size_t revision, size_t g,
                      const GfVec3f *in, GfVec3f *out, size_t count,
                      size_t begin, size_t end, bool *untouched)
    {
        Revision &rev = revisionAt(chain, revision);
        auto &chunk = rev.chunks[g];
        RigExecSkinTransformsView view;
        view.transforms = chunk.transforms.data();
        view.transformCount = chunk.transforms.size();
        if (B->useSimd && !chunk.rows.empty()) {
            view.rows = chunk.rows.data();
        }
        if (!chunk.palette.empty()) {
            view.palette = chunk.palette.data();
            view.paletteSize = chunk.palette.size();
        }
        return RigExecRunRevisionGroup(
            rev.op, rev.parameters, rev.rangeInputs, &view, in, out, count,
            begin, end, GroupEnvelope(rev), B->useSimd, untouched,
            &rev.groups[g].walk, nullptr, nullptr);
    }
    void runBlendGroup(size_t chain, size_t revision, size_t g, size_t begin,
                       size_t end, const RigExecPointsRef<GfVec3f> &entering,
                       RigExecGroupState<GfVec3f> *state)
    {
        BakedChainBlendGroup(*B, &revisionAt(chain, revision), g, begin, end,
                             entering, state);
    }
    const float *groupEnvelope(size_t chain, size_t revision)
    {
        return GroupEnvelope(revisionAt(chain, revision));
    }
    bool prepareGroupHalo(size_t chain, size_t revision, size_t g,
                          const RigExecPointsRef<GfVec3f> &entering)
    {
        return BakedChainGroupHalo(*B, chainAt(chain),
                                   &revisionAt(chain, revision), revision, g,
                                   entering);
    }
    bool runRevisionGroup(size_t chain, size_t revision, size_t g,
                          const GfVec3f *in, GfVec3f *out, size_t count,
                          size_t begin, size_t end, const float *envelope,
                          bool *untouched)
    {
        Chain &c = chainAt(chain);
        Revision &rev = revisionAt(chain, revision);
        const bool mush = rev.op == RigExecRevisionOp::DeltaMush;
        const bool smooth = rev.op == RigExecRevisionOp::Smooth || mush;
        RigExecGroupPoints<GfVec3f> halo;
        if (smooth) {
            halo.bounds = c.groupBounds.data();
            halo.groups = c.groupBounds.size() - 1;
            halo.points = rev.haloPoints[g].data();
        }
        return RigExecRunRevisionGroup(
            rev.op, rev.parameters, rev.rangeInputs, nullptr, in, out, count,
            begin, end, envelope, B->useSimd, untouched, &rev.groups[g].walk,
            smooth ? &halo : nullptr, mush ? &rev.mushGroups[g] : nullptr);
    }
    bool runWholeSkinChunk(size_t chain, size_t revision, size_t g)
    {
        return BakedChainWholeSkinChunk(
            *B, chainAt(chain), &revisionAt(chain, revision), revision, g);
    }
    eval::FuseRevisionAccessT<GfVec3f> fuseRevision(size_t chain,
                                                    size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        eval::FuseRevisionAccessT<GfVec3f> access;
        access.ran = &rev.ran;
        access.currentSource = &rev.currentSource;
        access.output = &rev.output;
        access.stagingOutput = &rev.stagingOutput;
        access.stagingFresh = &rev.stagingFresh;
        access.passedPoints = &rev.passedPoints;
        access.doneVersion = &rev.doneVersion;
        access.groups = &rev.groups;
        access.groupIds = &rev.groupIds;
        access.groupWritten = &rev.groupWritten;
        access.rangeRefusals = &rev.rangeRefusals;
        access.precedingCount = rev.precedingCount;
        return access;
    }
    bool revisionRangeSetAside(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).rangeSetAside;
    }
    void passSetAsideFuse(size_t chain, size_t revision)
    {
        PassBaseGroups(chainAt(chain), &revisionAt(chain, revision));
    }
    bool revisionIsRangeRole(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).role ==
               RigExecBakedRevisionRole::Range;
    }
    bool revisionIsWholeRole(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).role ==
               RigExecBakedRevisionRole::Whole;
    }
    bool fusePacketValid(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        return rev.parameters.valid &&
               (rev.op != RigExecRevisionOp::Skin || rev.influencesValid);
    }
    void emitFuseDiagnostic(size_t chain, size_t revision, bool packetValid)
    {
        EmitInvalidPacketLine(*B, revisionAt(chain, revision), packetValid,
                              &step->diagnostics);
    }
    bool revisionStatusAllows(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).status.AllowsApply();
    }
    int revisionAcceptanceValue(size_t chain, size_t revision)
    {
        return int(revisionAt(chain, revision).acceptance);
    }
    bool revisionChunkedValue(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).chunked;
    }
    bool revisionPartitionStaleValue(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).partitionStale;
    }
    size_t revisionChunkCount(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).chunks.size();
    }
    bool chunkOk(size_t chain, size_t revision, size_t k)
    {
        Revision &rev = revisionAt(chain, revision);
        return k < rev.chunks.size() && rev.chunks[k].ok;
    }
    bool chunkHoldsAnswer(size_t chain, size_t revision, size_t k)
    {
        return ChunkHoldsAnswer(revisionAt(chain, revision), k);
    }
    void setRevisionExecuted(size_t chain, size_t revision, bool value)
    {
        revisionAt(chain, revision).executed = value;
    }
    void noteFuseExecuted(size_t, size_t)
    {
        step->counters.revisionsExecuted = 1;
    }
    void setFuseResultFromStatus(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        rev.resultStatus = rev.status.state;
    }
    void setFuseResultFailed(size_t chain, size_t revision)
    {
        revisionAt(chain, revision).resultStatus = _tokens->moverFailed;
    }
    void saveFuseLastStatus(size_t chain, size_t revision)
    {
        Revision &rev = revisionAt(chain, revision);
        rev.lastStatus = rev.status;
    }
    void publishFuseState(size_t, size_t, int32_t)
    {
    }
    void noteWholeFuseSource(size_t, size_t, int)
    {
    }
    int precedingCurrentSource(size_t chain, size_t revision)
    {
        if (revision == 0) {
            return -1;
        }
        return chainAt(chain).revisions[revision - 1].currentSource;
    }
    bool wholeFuseSized(size_t chain, size_t revision)
    {
        Chain &c = chainAt(chain);
        Revision &rev = revisionAt(chain, revision);
        const size_t count = rev.precedingCount;
        return count == c.groupPointCount &&
               c.groupBounds.size() == rev.groups.size() + 1 &&
               (rev.chunked || rev.stagingOutput.size() == count);
    }
    bool legacyWholeStagingOk(size_t chain, size_t revision, size_t count)
    {
        return revisionAt(chain, revision).stagingOutput.size() == count;
    }
    int64_t fuseGroupSlot(size_t, size_t, size_t g)
    {
        return int64_t(RigExecBakedGroupSlot(*B, step->object, g));
    }
    RigExecGroupSource fuseVersionSource(size_t chain, size_t version, size_t g)
    {
        return RigExecBakedGroupSourceAt(*B, int(chain), version, g);
    }
    void enteringVersionPoints(size_t chain, size_t version,
                               const GfVec3f **points, size_t *count)
    {
        Chain &c = chainAt(chain);
        if (version == 0 || c.groupBounds.size() < 2) {
            PointsAt(c, version, points, count);
            return;
        }
        std::vector<GfVec3f> *scratch = &versionScratch;
        if (version - 1 < c.revisions.size()) {
            scratch = &c.revisions[version - 1].wholeEntering;
            if (version <= c.revisions.size() && version - 1 != version &&
                version < c.revisions.size() + 1) {
                // The fusing revision's own scratch, as the fuse held it.
                const size_t fuseRevision =
                    step->object >= 0 &&
                            size_t(step->object) < B->revisionIndex.size()
                        ? size_t(B->revisionIndex[size_t(step->object)].second)
                        : version - 1;
                if (fuseRevision < c.revisions.size()) {
                    scratch = &c.revisions[fuseRevision].wholeEntering;
                }
            }
        }
        RigExecBakedVersionPoints(c, version, scratch, points, count);
    }
    bool revisionLayoutUsableValue(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).layoutUsable;
    }
    bool revisionEnvelopeOkValue(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).envelopeOk;
    }
    size_t fusePrecedingCount(size_t chain, size_t revision)
    {
        return revisionAt(chain, revision).precedingCount;
    }
    bool skinWholeRange(size_t chain, size_t revision, const GfVec3f *points,
                        size_t count, std::vector<GfVec3f> *fused)
    {
        Revision &rev = revisionAt(chain, revision);
        return SkinRange(&rev, points, WholeTransformsView(&rev), 0, count,
                         true, B->useSimd, fused);
    }
    eval::ChainStatusAccessT<GfVec3f> chainStatus(size_t chain)
    {
        Chain &c = chainAt(chain);
        eval::ChainStatusAccessT<GfVec3f> access;
        access.haveResult = &c.haveResult;
        access.resultVersion = &c.resultVersion;
        access.resultIds = &c.resultIds;
        return access;
    }
    void clearChainStatusPublish(size_t)
    {
    }
    void emitChainStatusDiagnostics(size_t chain)
    {
        for (const Revision &rev : chainAt(chain).revisions) {
            if (rev.resultStatus == "moverFailed") {
                step->diagnostics.push_back(
                    "MoverFailed " + rev.moverPathText +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
    }
    void noteChainBuilt(size_t)
    {
        step->counters.chainsBuilt = 1;
    }
    bool chainIsGpu(size_t chain)
    {
        return chainAt(chain).gpu;
    }
    void noteChainGpuPending(size_t chain)
    {
        chainAt(chain).gpuPending = true;
    }
    bool chainHasGroups(size_t chain)
    {
        return chainAt(chain).groupBounds.size() >= 2;
    }
    size_t chainRevisionCount(size_t chain)
    {
        return chainAt(chain).revisions.size();
    }
    size_t chainGroupCount(size_t chain)
    {
        Chain &c = chainAt(chain);
        return c.groupBounds.size() >= 1 ? c.groupBounds.size() - 1 : 0;
    }
    RigExecGroupSource chainStatusSource(size_t chain, size_t version, size_t g)
    {
        return RigExecBakedGroupSourceAt(*B, int(chain), version, g);
    }
    const RigExecPointsRef<GfVec3f> &chainStatusGroup(size_t chain,
                                                      size_t version, size_t g)
    {
        return RigExecBakedGroupAt(chainAt(chain), version, g);
    }
    void publishChainGroups(size_t chain, const std::vector<GfVec3f> &gathered,
                            const std::vector<RigExecGroupSource> &ids)
    {
        Chain &c = chainAt(chain);
        c.spare.clear();
        c.spare.resize(gathered.size(), [&gathered](GfVec3f *out, GfVec3f *) {
            std::copy(gathered.begin(), gathered.end(), out);
        });
        if (!SamePoints(c.spare, c.result)) {
            ++c.resultVersion;
        }
        c.result.swap(c.spare);
        c.spareIds.swap(c.resultIds);
        c.resultIds = ids;
        c.haveResult = true;
    }
    void chainFinalPoints(size_t chain, size_t version, const GfVec3f **points,
                          size_t *count)
    {
        PointsAt(chainAt(chain), version, points, count);
    }
    void publishChainFinal(size_t chain, const std::vector<GfVec3f> &gathered)
    {
        Chain &c = chainAt(chain);
        if (!eval::PointsBitsEqualBody(gathered.data(), gathered.size(),
                                       c.result.cdata(), c.result.size())) {
            ++c.resultVersion;
        }
        c.spare.clear();
        c.spare.resize(gathered.size(), [&gathered](GfVec3f *out, GfVec3f *) {
            std::copy(gathered.begin(), gathered.end(), out);
        });
        c.result.swap(c.spare);
        c.haveResult = true;
    }
    void captureDerivedPublished(size_t chain, size_t derived)
    {
        derivedPublished = chainAt(chain).derived[derived].result;
    }
    void prepareDerivedBase(size_t chain, size_t derived)
    {
        RigExecBakedPrepareDerivedBase(&chainAt(chain).derived[derived], step);
    }
    bool derivedHasBase(size_t chain, size_t derived)
    {
        return chainAt(chain).derived[derived].haveBase;
    }
    void adoptDerivedLayout(size_t chain, size_t derived)
    {
        RigExecBakedAdoptRevisionLayout(
            &chainAt(chain).derived[derived].revision);
    }
    bool derivedIsMatrix(size_t chain, size_t derived)
    {
        return chainAt(chain).derived[derived].matrixTarget;
    }
    bool runDerivedMatrix(size_t chain, size_t derived)
    {
        Chain &c = chainAt(chain);
        auto &d = c.derived[derived];
        step->counters.revisionsBuilt = 1;
        d.haveMatrix = RigExecBakedRunProjectorTarget(
            *B, c, d.revision, &d.matrix, &step->diagnostics);
        d.haveResult = true;
        step->counters.chainsBuilt = 1;
        return true;
    }
    bool foldDerivedInfluences(size_t chain, size_t derived)
    {
        FoldInfluences(*B, &chainAt(chain).derived[derived].revision);
        return true;
    }
    void assembleDerived(size_t chain, size_t derived)
    {
        Chain &c = chainAt(chain);
        Revision &rev = c.derived[derived].revision;
        derivedParameters =
            AssembleRevision(*B, &rev, c.result.cdata(), c.result.size(), time,
                             step, c.lastBase.size());
        derivedStatus =
            RigExecStatusForParameters(derivedParameters, rev.moverPathText);
    }
    void noteDerivedBuilt(size_t, size_t)
    {
        step->counters.revisionsBuilt = 1;
    }
    void noteDerivedExecuted(size_t, size_t)
    {
        step->counters.revisionsExecuted = 1;
    }
    bool derivedStatusAllows(size_t, size_t)
    {
        return derivedStatus.AllowsApply();
    }
    bool runDerivedKernel(size_t chain, size_t derived,
                          std::vector<GfVec3f> *values)
    {
        Chain &c = chainAt(chain);
        auto &d = c.derived[derived];
        return RigExecRunGeometryDerived(d.revision.op, derivedParameters,
                                         d.lastBase.cdata(), d.lastBase.size(),
                                         values);
    }
    void derivedLastBasePoints(size_t chain, size_t derived,
                               std::vector<GfVec3f> *values)
    {
        const auto &base = chainAt(chain).derived[derived].lastBase;
        values->assign(base.cbegin(), base.cend());
    }
    void setDerivedResultFromStatus(size_t chain, size_t derived)
    {
        chainAt(chain).derived[derived].revision.resultStatus =
            derivedStatus.state;
    }
    void setDerivedResultFailed(size_t chain, size_t derived)
    {
        chainAt(chain).derived[derived].revision.resultStatus =
            _tokens->moverFailed;
    }
    void saveDerivedRevision(size_t chain, size_t derived,
                             std::vector<GfVec3f> &&values)
    {
        Chain &c = chainAt(chain);
        Revision &rev = c.derived[derived].revision;
        rev.output = std::move(values);
        derivedParameters.auxPoints.clear();
        derivedParameters.auxPoints.shrink_to_fit();
        rev.lastParameters = std::move(derivedParameters);
        rev.lastAuxPoints = c.result;
        rev.lastStatus = derivedStatus;
        rev.ran = true;
    }
    bool derivedResultIsFailed(size_t chain, size_t derived)
    {
        return chainAt(chain).derived[derived].revision.resultStatus ==
               "moverFailed";
    }
    void emitDerivedDiagnostic(size_t chain, size_t derived)
    {
        step->diagnostics.push_back(
            "MoverFailed " + chainAt(chain).derived[derived].targetText +
            ": derived geometry input/cardinality validation failed");
    }
    void noteDerivedEarlyVersion(size_t chain, size_t derived)
    {
        auto &d = chainAt(chain).derived[derived];
        if (!SamePoints(derivedPublished, d.result)) {
            ++d.resultVersion;
        }
    }
    void publishDerivedResult(size_t chain, size_t derived)
    {
        Chain &c = chainAt(chain);
        auto &d = c.derived[derived];
        Revision &rev = d.revision;
        d.spare.resize(rev.output.size());
        std::copy(rev.output.begin(), rev.output.end(), d.spare.data());
        d.result.swap(d.spare);
        if (!SamePoints(derivedPublished, d.result)) {
            ++d.resultVersion;
        }
        d.haveResult = true;
        step->counters.chainsBuilt = 1;
    }
    void skipChainInputs(size_t chain)
    {
        chainAt(chain).baseDirty = false;
    }
    void skipDerived(size_t chain, size_t derived)
    {
        chainAt(chain).derived[derived].revision.influencesChanged = false;
    }
    void skipInfluenceFold(size_t chain, size_t revision)
    {
        revisionAt(chain, revision).influencesChanged = false;
    }
    void skipRevisionStatic(size_t chain, size_t revision, int32_t)
    {
        revisionAt(chain, revision).staticDirty = false;
    }
    void skipRevisionChunk(size_t chain, size_t revision, int32_t part)
    {
        Revision &rev = revisionAt(chain, revision);
        if (part >= 0 && size_t(part) < rev.chunks.size()) {
            rev.chunks[size_t(part)].keyChanged = false;
        }
    }
    void skipRevisionFuse(size_t chain, size_t revision)
    {
        revisionAt(chain, revision).executed = false;
    }
};

struct BakedChainHost {
    using Math = BakedSkinMath;
    using Inputs = BakedChainInputs;
    using State = BakedChainState;
    using ChainPoints = VtVec3fArray;
    BakedChainHost(Inputs inputs, State state) : _inputs(inputs), _state(state)
    {
    }
    const Inputs &inputs() const
    {
        return _inputs;
    }
    State &state()
    {
        return _state;
    }
    const RigExecRangeRunner *ranges() const
    {
        return RigExecGeometryRanges();
    }
    Inputs _inputs;
    State _state;
};

bool
BakedChainState::runSingleChunk(size_t chain, size_t revision, int32_t part)
{
    Chain &c = chainAt(chain);
    Revision &rev = revisionAt(chain, revision);
    const bool skin = rev.op == RigExecRevisionOp::Skin;
    auto &chunk = rev.chunks[size_t(part)];
    const GfVec3f *points = nullptr;
    size_t count = 0;
    if (rev.role == RigExecBakedRevisionRole::Whole) {
        RigExecBakedVersionPoints(c, revision, &rev.wholeEntering, &points,
                                  &count);
    } else {
        PointsAt(c, revision, &points, &count);
    }
    const bool sized =
        count == rev.precedingCount && rev.stagingOutput.size() == count;
    if (!rev.chunked) {
        chunk.ok = false;
        rev.devicePending = false;
        if (!rev.status.AllowsApply()) {
            rev.deviceTicks.clear();
            if (rev.window >= 0 &&
                size_t(rev.window) < B->windowCaptures.size()) {
                B->windowCaptures[size_t(rev.window)].reset();
            }
            return true;
        }
        if (!skin) {
            rev.stagingOutput.resize(count);
            rev.stagingFresh = true;
            if (rev.window >= 0 && size_t(rev.window) < B->windowedOps.size()) {
                const size_t w = size_t(rev.window);
                rev.windowRun.ring = &B->windowRings[w];
                rev.windowRun.capture = &B->windowCaptures[w];
                rev.windowRun.captureOnly =
                    B->historyTick && !B->windowedOps[w].feedsWindow;
                rev.windowRun.cache = &rev.windowCache;
                rev.windowRun.deferred = rev.windowDevice && !B->historyTick
                                             ? &rev.deviceTicks
                                             : nullptr;
                rev.windowRun.solveTicks = nullptr;
                rev.windowRun.device = false;
                if (rev.windowRun.deferred) {
                    rev.deviceTicks.clear();
                }
                rev.external.window = &rev.windowRun;
            }
            chunk.ok = geometryDetail::RunDiscardableGeometry(
                rev.op, rev.parameters, points, count, &rev.stagingOutput,
                B->useSimd, &rev.wireBasis, &rev.surfaceCache,
                !rev.fullStrength && rev.envelopeOk ? &rev.envelope : nullptr,
                &rev.external);
            if (rev.windowRun.deferred) {
                rev.devicePending = chunk.ok && !rev.deviceTicks.empty();
                if (!chunk.ok) {
                    rev.deviceTicks.clear();
                }
                rev.windowRun.deferred = nullptr;
            }
            rev.external.window = nullptr;
            return true;
        }
        if (!SkinPacketIsUsable(rev) || !rev.influencesValid || !sized) {
            return true;
        }
        rev.stagingFresh = true;
        chunk.ok = SkinRange(&rev, points, WholeTransformsView(&rev), 0, count,
                             true, B->useSimd, &rev.stagingOutput);
        return true;
    }
    chunk.keyChanged = false;
    if (!rev.status.AllowsApply() || !SkinPacketIsUsable(rev) ||
        rev.partitionStale || !sized) {
        chunk.ok = false;
        return true;
    }
    GatherChunkTransforms(*B, rev, &chunk);
    chunk.ok = SkinRange(&rev, points, ChunkTransformsView(chunk, B->useSimd),
                         size_t(chunk.begin), size_t(chunk.end), false,
                         B->useSimd, &rev.stagingOutput);
    return true;
}

void
RigExecBakedRunChainInputs(RigExecBakedProgramImpl *program,
                           RigExecBakedStep *step)
{
    // EvalCore's one ChainInputs body (rigExecEvalCore/chain.h) over
    // this adapter's host.
    RigExecBakedProgramImpl &B = *program;
    BakedChainHost host(BakedChainInputs{&B},
                        BakedChainState{&B, step, UsdTimeCode::Default()});
    eval::RunChainInputsStep(host,
                             eval::StepRef{uint32_t(step->kind), step->object,
                                           step->part, size_t(step->object)});
}
void
RigExecBakedPrepareDerivedBase(
    RigExecBakedProgramImpl::GeomChain::Derived *derived,
    RigExecBakedStep *step)
{
    if (derived->matrixTarget) {
        derived->haveBase = true;
        derived->baseDirty = true;
        derived->revision.created = false;
        return;
    }
    derived->haveBase = derived->sampledHaveBase;
    if (!derived->haveBase) {
        derived->baseDirty = false;
        return;
    }
    if (derived->haveResult &&
        derived->sampledBase.size() != derived->lastBase.size()) {
        derived->haveResult = false;
        derived->result = VtVec3fArray();
        ResetGeometryRevision(&derived->revision);
    }
    if (derived->revision.created) {
        ++step->counters.revisionsCreated;
        ++step->counters.schedulesBuilt;
        derived->revision.created = false;
    }
    derived->baseDirty =
        !derived->haveResult ||
        !RigExecBakedHeadValueSame(VtValue(derived->sampledBase),
                                   VtValue(derived->lastBase));
    derived->lastBase = derived->sampledBase;
}
void
RigExecBakedShareLatticeBinds(RigExecBakedProgramImpl *program)
{
    RigExecLatticeBindSharing<GfVec3f> binds;
    for (auto &chain : program->chains) {
        for (auto &revision : chain.revisions) {
            binds.Offer(&revision.surfaceCache);
            // A range-pipelined lattice's range steps read the basis its
            // RevisionStatic retained, through `rangeInputs`, which holds
            // that bind alive for a step that follows a skipped
            // RevisionStatic. When the cache now holds an equal shared bind,
            // name that one, so the one it replaced may be released.
            if (revision.rangeInputs.latticeBind) {
                const auto &bind = revision.surfaceCache.RetainedLatticeBind();
                if (bind && bind != revision.rangeInputs.latticeBind) {
                    revision.rangeInputs.latticeBind = bind;
                    revision.rangeInputs.latticeBasis = &bind->value;
                }
            }
        }
        for (auto &derived : chain.derived) {
            binds.Offer(&derived.revision.surfaceCache);
        }
    }
}
void
RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                UsdTimeCode time, RigExecRigPose *, bool all)
{
    auto &B = *program;
    // Source capture only. No head output, topology adoption, node reset or
    // generation accounting is consumed before the declared graph operations.
    for (auto &object : B.weightObjects) {
        RigExecBakedSamplePathLeaves(&B, &object.pointLeaves, time, all);
        RigExecBakedSamplePathLeaves(&B, &object.oracleLeaves, time, all);
    }
    for (auto &chain : B.chains) {
        chain.sampledBase = VtVec3fArray();
        const auto upstream = B.upstream.find(chain.target);
        if (upstream != B.upstream.end() &&
            upstream->second.IsHolding<VtVec3fArray>()) {
            chain.sampledBase = upstream->second.UncheckedGet<VtVec3fArray>();
            chain.sampledHaveBase = true;
        } else {
            chain.sampledHaveBase =
                chain.baseQuery.IsValid() &&
                chain.baseQuery.Get(&chain.sampledBase, time);
        }
        for (auto &revision : chain.revisions) {
            RigExecBakedSamplePathLeaves(&B, &revision.leaves, time, all);
        }
        for (auto &derived : chain.derived) {
            derived.sampledBase = VtVec3fArray();
            derived.sampledHaveBase =
                derived.matrixTarget ||
                (derived.baseQuery.IsValid() &&
                 derived.baseQuery.Get(&derived.sampledBase, time));
            RigExecBakedSamplePathLeaves(&B, &derived.revision.leaves, time,
                                         all);
        }
    }
}
void
RigExecBakedSkipGeometryStep(RigExecBakedProgramImpl *program,
                             RigExecBakedStep *step)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::WeightField) {
        // Field IDs are not revision IDs; a clean skip preserves the field.
        return;
    }
    // EvalCore's one skip switch (rigExecEvalCore/chain.h) over this
    // adapter's host; the WeightField arm above stays in this half.
    BakedChainHost host(BakedChainInputs{&B},
                        BakedChainState{&B, step, UsdTimeCode::Default()});
    eval::RunSkipGeometryStep(host,
                              eval::StepRef{uint32_t(step->kind), step->object,
                                            step->part, size_t(step->object)});
}

namespace {

// A GPU chain's revisions as the translation reads them: the packets this
// generation assembled, the folded skin tables, the unsummed channels and
// each fuse's decision.
void
TranslateGpuChain(const RigExecBakedProgramImpl &B,
                  RigExecBakedProgramImpl::GeomChain *chain)
{
    std::vector<RigExecGpuRevisionInput> inputs;
    inputs.reserve(chain->revisions.size());
    for (RigExecBakedProgramImpl::GeomRevision &revision : chain->revisions) {
        RigExecGpuRevisionInput in;
        in.op = revision.op;
        in.parameters = &revision.parameters;
        if (revision.op == RigExecRevisionOp::Skin) {
            in.influences = &revision.influences;
        }
        if (revision.op == RigExecRevisionOp::BlendShape) {
            in.channels = &revision.gpuChannels;
        }
        in.applies = revision.gpuApplies;
        // The shared packet's op value revision names it exactly.
        if (revision.weightObject >= 0 && !revision.weightCurrentPhase &&
            revision.weightPacketValue >= 0 &&
            size_t(revision.weightPacketValue) < B.opAdapter.values.size()) {
            in.weightsObject = revision.weightObject;
            in.weightsRevision =
                B.opAdapter.values[size_t(revision.weightPacketValue)].revision;
        }
        inputs.push_back(in);
    }
    chain->gpuTranslated = chain->gpuTranslation.Translate(
        inputs, chain->lastBase.size(), &chain->gpuFrame);
    chain->gpuPending = true;
}

// The chain through its CPU revision kernels, in order, on the decisions
// its fuses made: the boundary's answer where no device ran the program or
// the device refused it (a degenerate dual-quaternion blend fails its
// revision atomically here, as on the CPU path).
void
RunGpuChainOnCpu(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomChain *chain,
                 std::vector<GfVec3f> *points)
{
    points->assign(chain->lastBase.cbegin(), chain->lastBase.cend());
    std::vector<GfVec3f> before;
    for (RigExecBakedProgramImpl::GeomRevision &revision : chain->revisions) {
        if (!revision.gpuApplies) {
            continue;
        }
        RigExecMoverParameters p = revision.parameters;
        if (revision.op == RigExecRevisionOp::Skin) {
            p.skinTransforms = revision.influences;
        } else if (revision.op == RigExecRevisionOp::BlendShape &&
                   !RigExecGeometryBlendDeltas(
                       revision.gpuChannels, chain->lastBase.cdata(),
                       chain->lastBase.size(), &p.blendDeltas)) {
            p.blendDeltas.clear();
        }
        before = *points;
        if (!RigExecRunGeometry(revision.op, p, points, B.useSimd,
                                &revision.wireBasis, &revision.surfaceCache,
                                &revision.external)) {
            points->swap(before);
            if (revision.status.AllowsApply()) {
                revision.resultStatus = _tokens->moverFailed;
            }
        }
    }
}

// The shader-mode boundary: each translated chain publishes its program in
// place of its points, and its derived extents are measured from the
// extent sample. A chain that did not translate runs its CPU kernels and
// publishes points as always.
void
ResolveShaderChains(RigExecBakedProgramImpl &B)
{
    std::vector<GfVec3f> points;
    std::vector<RigExecBakedProgramImpl::GeomChain *> measure;
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.gpu || !chain.gpuPending)
            continue;
        chain.gpuPending = false;
        if (!chain.haveBase)
            continue;
        if (!chain.gpuTranslated || !chain.gpuFrame.layout) {
            RunGpuChainOnCpu(B, &chain, &points);
            chain.spare.clear();
            chain.spare.resize(
                points.size(), [&points](GfVec3f *out, GfVec3f *end) {
                    std::uninitialized_copy(points.begin(),
                                            points.begin() + (end - out), out);
                });
            if (!SamePoints(chain.spare, chain.result))
                ++chain.resultVersion;
            chain.result.swap(chain.spare);
            chain.haveResult = true;
            chain.gpuPublication.reset();
            chain.gpuLastSerial = 0;
            continue;
        }
        const RigExecGpuDeformLayout &layout = *chain.gpuFrame.layout;
        const bool same = chain.gpuPublication &&
                          layout.serial == chain.gpuLastSerial &&
                          chain.gpuLastBaseVersion == chain.baseVersion &&
                          chain.gpuFrame.frame == chain.gpuLastFrame;
        if (same)
            continue;
        if (chain.gpuFrame.frame != chain.gpuLastFrame ||
            layout.serial != chain.gpuLastSerial) {
            ++chain.gpuFrameVersion;
        }
        auto publication = std::make_shared<RigExecGpuDeformPublication>();
        publication->layout = chain.gpuFrame.layout;
        publication->frame =
            std::make_shared<const std::vector<float>>(chain.gpuFrame.frame);
        publication->base = chain.lastBase;
        publication->baseVersion = chain.baseVersion;
        publication->frameVersion = chain.gpuFrameVersion;
        chain.gpuPublication = std::move(publication);
        chain.gpuLastFrame = chain.gpuFrame.frame;
        chain.gpuLastSerial = layout.serial;
        chain.gpuLastBaseVersion = chain.baseVersion;
        ++chain.resultVersion;
        chain.haveResult = true;
        if (chain.derived.empty())
            continue;
        if (chain.gpuExtentSerial != layout.serial ||
            chain.gpuExtentBaseVersion != chain.baseVersion) {
            chain.gpuExtentSample = RigExecGpuDeformExtentSample(
                layout, chain.lastBase.cdata(), chain.lastBase.size());
            chain.gpuExtentSerial = layout.serial;
            chain.gpuExtentBaseVersion = chain.baseVersion;
        }
        measure.push_back(&chain);
    }
    // The extents: every measured chain's sample in pieces, side by side
    // (on one thread on a frozen worker), each piece growing its own box.
    struct Piece {
        RigExecBakedProgramImpl::GeomChain *chain;
        size_t begin, end;
        GfVec3f lo, hi;
        bool ok;
    };
    constexpr size_t kPiece = 128;
    const GfVec3f wide((std::numeric_limits<float>::max)());
    std::vector<Piece> pieces;
    for (RigExecBakedProgramImpl::GeomChain *chain : measure) {
        const size_t n = chain->gpuExtentSample.size();
        for (size_t b = 0; b < n; b += kPiece)
            pieces.push_back(
                {chain, b, std::min(n, b + kPiece), wide, -wide, false});
    }
    const auto runPieces = [&pieces](size_t begin, size_t end) {
        for (size_t k = begin; k < end; ++k) {
            Piece &piece = pieces[k];
            piece.ok = RigExecGpuDeformSampleBounds(
                piece.chain->gpuFrame, piece.chain->lastBase.cdata(),
                piece.chain->gpuExtentSample, piece.begin, piece.end, &piece.lo,
                &piece.hi);
        }
    };
    // Printed by the owning thread only; a frozen worker never prints.
    const bool stats = B.gpuDeformStats && !RigExecFrozenSerialActive();
    const auto t0 = std::chrono::steady_clock::now();
    if (pieces.size() > 1 && RigExecParallelEvaluationEnabled() &&
        !RigExecFrozenSerialActive()) {
        WorkParallelForN(pieces.size(), runPieces, 1);
    } else {
        runPieces(0, pieces.size());
    }
    if (stats) {
        std::fprintf(stderr,
                     "GPUSHADER extents %zu chains %zu pieces %.3f ms\n",
                     measure.size(), pieces.size(),
                     std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t0)
                         .count());
    }
    size_t k = 0;
    for (RigExecBakedProgramImpl::GeomChain *chain : measure) {
        GfVec3f lo = wide, hi = -wide;
        bool measured = !chain->gpuExtentSample.empty();
        for (; k < pieces.size() && pieces[k].chain == chain; ++k) {
            measured = measured && pieces[k].ok;
            for (int axis = 0; axis < 3; ++axis) {
                lo[axis] = std::min(lo[axis], pieces[k].lo[axis]);
                hi[axis] = std::max(hi[axis], pieces[k].hi[axis]);
            }
        }
        if (measured)
            RigExecGpuDeformPadExtent(&lo, &hi);
        for (const int index : chain->gpuDerivedSteps) {
            RigExecBakedStep &step = B.steps[size_t(index)];
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain->derived[size_t(derivedIndex)];
            step.diagnostics.clear();
            RigExecBakedPrepareDerivedBase(&derived, &step);
            if (!derived.haveBase || !measured)
                continue;
            VtVec3fArray extent(2);
            extent[0] = lo;
            extent[1] = hi;
            if (!SamePoints(extent, derived.result))
                ++derived.resultVersion;
            derived.result = extent;
            derived.haveResult = true;
        }
    }
}

// The graph's keys for what the boundary wrote -- a chain's points or
// program and its derived outputs -- published from their current contents,
// so a snapshot of the program finds every key standing. This run's changes
// have propagated already: the flags stay clear.
void
RekeyGpuChains(RigExecBakedProgramImpl &B,
               const std::vector<RigExecBakedProgramImpl::GeomChain *> &chains)
{
    const auto sample = [&B](uint32_t d, uint32_t slot, std::string *key) {
        RigExecBakedOpValueKey(B, RigExecBakedSlotDomain(d), slot, key);
    };
    for (const RigExecBakedProgramImpl::GeomChain *chain : chains) {
        for (const uint32_t c : chain->gpuKeyOps) {
            if (c >= B.opGraph.ops.size())
                continue;
            for (const RigExecValueId id : B.opGraph.ops[c].descriptor.writes) {
                if (size_t(id) >= B.opAdapter.values.size())
                    continue;
                RigExecOpValueState &value = B.opAdapter.values[size_t(id)];
                if (!value.initialized)
                    continue;
                if (!RigExecBakedPublishSmallValue(B, &value)) {
                    RigExecOpPublishValue(&value, sample);
                }
                value.changed = 0;
            }
        }
    }
}

// Which chain a slot of a point-carrying domain belongs to, or -1.
int
GpuSlotChain(const RigExecBakedProgramImpl &B, RigExecBakedSlotDomain domain,
             uint32_t slot)
{
    switch (domain) {
    case RigExecBakedSlotDomain::ChainPoints:
        return slot < B.chains.size() ? int(slot) : -1;
    case RigExecBakedSlotDomain::RevisionOut:
        if (slot < B.chunkRevision.size()) {
            const int id = B.chunkRevision[slot];
            if (id >= 0 && size_t(id) < B.revisionIndex.size())
                return B.revisionIndex[size_t(id)].first;
        }
        return -1;
    case RigExecBakedSlotDomain::RevisionDone:
    case RigExecBakedSlotDomain::ChainDirty:
        return slot < B.revisionIndex.size() ? B.revisionIndex[slot].first : -1;
    default:
        return -1;
    }
}

// The chain a geometry step works on, or -1 for every other step.
int
GpuStepChain(const RigExecBakedProgramImpl &B, const RigExecBakedStep &step)
{
    switch (step.kind) {
    case RigExecBakedStepKind::ChainInputs:
    case RigExecBakedStepKind::ChainStatus:
        return step.object;
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return size_t(step.object) < B.revisionIndex.size()
                   ? B.revisionIndex[size_t(step.object)].first
                   : -1;
    case RigExecBakedStepKind::Derived:
        return size_t(step.object) < B.derivedIndex.size()
                   ? B.derivedIndex[size_t(step.object)].first
                   : -1;
    default:
        return -1;
    }
}

} // namespace

void
RigExecBakedSelectGpuChains(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // Only a consumer that draws the program is handed one; any other
    // reads points, so its chains read the points back.
    if (B.gpuDeformMode == RigExecGpuDeformMode::Shader &&
        !(B.evaluator && B.evaluator->GetAcceptsGpuDeformPublications())) {
        B.gpuDeformMode = RigExecGpuDeformMode::Readback;
    }
    for (auto &chain : B.chains) {
        chain.gpu = false;
        chain.gpuDerivedSteps.clear();
        for (auto &revision : chain.revisions)
            revision.gpuOwned = false;
    }
    if (B.gpuDeformMode == RigExecGpuDeformMode::Off) {
        return;
    }
    const bool shader = B.gpuDeformMode == RigExecGpuDeformMode::Shader;
    // A chain whose points, versions or statuses a step outside it reads
    // must produce them inside the graph, which a GPU chain does not.
    std::vector<char> blocked(B.chains.size(), 0);
    for (const RigExecBakedStep &step : B.steps) {
        const int own = GpuStepChain(B, step);
        for (const RigExecBakedSlotRange &read : step.reads) {
            for (uint32_t slot = read.begin; slot < read.end; ++slot) {
                const int chain = GpuSlotChain(B, read.domain, slot);
                if (chain >= 0 && chain != own)
                    blocked[size_t(chain)] = 1;
            }
        }
    }
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        bool eligible = !blocked[c] && !chain.revisions.empty();
        for (const auto &revision : chain.revisions) {
            if (!eligible)
                break;
            switch (revision.op) {
            case RigExecRevisionOp::Matrix:
            case RigExecRevisionOp::Skin:
            case RigExecRevisionOp::Wire:
                break;
            case RigExecRevisionOp::BlendShape:
                // Sparse samples only: a dense sample's delta is measured
                // against a base the program does not hold.
                for (const auto &channel : revision.blendChannels)
                    for (const auto &sample : channel.samples)
                        if (sample.blendShape.IsEmpty())
                            eligible = false;
                break;
            default:
                eligible = false;
            }
            // A current-phase weight field or a snapshot reads the points
            // entering a revision.
            if (revision.snapshotAfter || revision.weightCurrentPhase)
                eligible = false;
        }
        // A shader chain's points never come back: a derived step that
        // reads them all (normals, a surface projector) keeps the chain on
        // its CPU kernels. An extent is measured from a sample.
        for (const auto &derived : chain.derived) {
            if (shader &&
                (derived.matrixTarget ||
                 derived.revision.op != RigExecRevisionOp::RecomputeExtent))
                eligible = false;
        }
        if (!eligible)
            continue;
        chain.gpu = true;
        for (auto &revision : chain.revisions)
            revision.gpuOwned = true;
    }
    // The derived steps are indexed at the first boundary: compiling the
    // graph reorders the steps after this.
    B.gpuDerivedIndexed = false;
}

void
RigExecBakedSelectWindowDevices(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (auto &chain : B.chains) {
        chain.windowDeviceDerivedSteps.clear();
        for (auto &revision : chain.revisions) {
            revision.windowDevice = false;
            revision.devicePending = false;
            revision.deviceTicks.clear();
        }
    }
    if (B.windowedOps.empty() || !RigExecGpuClothEnabled())
        return;
    B.windowDeviceMinPoints = RigExecGpuClothMinPoints();
    // A chain whose points, versions or statuses a step outside it reads
    // must produce them inside the graph, which a boundary solve does not.
    std::vector<char> blocked(B.chains.size(), 0);
    for (const RigExecBakedStep &step : B.steps) {
        const int own = GpuStepChain(B, step);
        for (const RigExecBakedSlotRange &read : step.reads) {
            for (uint32_t slot = read.begin; slot < read.end; ++slot) {
                const int chain = GpuSlotChain(B, read.domain, slot);
                if (chain >= 0 && chain != own)
                    blocked[size_t(chain)] = 1;
            }
        }
    }
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        if (blocked[c] || chain.revisions.empty() || chain.gpu)
            continue;
        RigExecBakedProgramImpl::GeomRevision &last = chain.revisions.back();
        if (last.op != RigExecRevisionOp::External || last.window < 0 ||
            size_t(last.window) >= B.windowedOps.size() ||
            B.windowedOps[size_t(last.window)].feedsWindow || !last.moverPrim) {
            continue;
        }
        const RigExecMoverHandler *handler =
            RigExecFindMoverHandler(last.moverPrim.GetTypeName());
        if (!handler || !handler->applyWindowDevice)
            continue;
        last.windowDevice = true;
        for (size_t i = 0; i < B.steps.size(); ++i) {
            const RigExecBakedStep &step = B.steps[i];
            if (step.kind == RigExecBakedStepKind::Derived &&
                GpuStepChain(B, step) == int(c)) {
                chain.windowDeviceDerivedSteps.push_back(int(i));
            }
        }
    }
}

namespace {

// The boundary half of a device revision (RigExecBakedSelectWindowDevices):
// the window its chunk body captured, solved on the device -- the CPU kernel
// when none solves it -- through the chunk's own kernel runner, published as
// the chain's points; then the chain's Derived steps again on those points.
void
ResolveWindowDevices(RigExecBakedProgramImpl &B, UsdTimeCode time)
{
    std::vector<int> derived;
    std::vector<GfVec3f> solved;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        if (chain.revisions.empty() || !chain.revisions.back().windowDevice ||
            !chain.haveBase) {
            continue;
        }
        const size_t r = chain.revisions.size() - 1;
        RigExecBakedProgramImpl::GeomRevision &revision = chain.revisions[r];
        // The chain published the points entering the solve: its chunk
        // deferred this run, or ChainStatus published again without it.
        const bool republished =
            chain.resultVersion != revision.deviceResultVersion;
        if (!revision.devicePending && !republished)
            continue;
        revision.devicePending = false;
        if (revision.deviceTicks.empty()) {
            revision.deviceResultVersion = chain.resultVersion;
            continue;
        }
        const GfVec3f *points = nullptr;
        size_t count = 0;
        RigExecBakedVersionPoints(chain, r, &revision.wholeEntering, &points,
                                  &count);
        RigExecWindowRun run;
        run.cache = &revision.windowCache;
        run.solveTicks = &revision.deviceTicks;
        run.device = count >= B.windowDeviceMinPoints;
        revision.external.window = &run;
        solved.clear();
        const bool ok = geometryDetail::RunDiscardableGeometry(
            revision.op, revision.parameters, points, count, &solved, B.useSimd,
            &revision.wireBasis, &revision.surfaceCache,
            !revision.fullStrength && revision.envelopeOk ? &revision.envelope
                                                          : nullptr,
            &revision.external);
        revision.external.window = nullptr;
        // A failed solve keeps the points entering it, as the fuse does.
        if (!ok || solved.size() != count)
            solved.assign(points, points + count);
        chain.spare.clear();
        chain.spare.resize(
            solved.size(), [&solved](GfVec3f *out, GfVec3f *end) {
                std::uninitialized_copy(solved.begin(),
                                        solved.begin() + (end - out), out);
            });
        if (!SamePoints(chain.spare, chain.result))
            ++chain.resultVersion;
        chain.result.swap(chain.spare);
        chain.haveResult = true;
        revision.deviceResultVersion = chain.resultVersion;
        derived.insert(derived.end(), chain.windowDeviceDerivedSteps.begin(),
                       chain.windowDeviceDerivedSteps.end());
    }
    for (const int i : derived) {
        RigExecBakedStep &step = B.steps[size_t(i)];
        step.diagnostics.clear();
        RigExecBakedRunGeometryStep(&B, &step, time);
    }
}

} // namespace

void
RigExecBakedResolveGpuChains(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    ResolveWindowDevices(B, time);
    if (B.gpuDeformMode != RigExecGpuDeformMode::Off && !B.gpuDerivedIndexed) {
        for (auto &chain : B.chains) {
            chain.gpuDerivedSteps.clear();
            chain.gpuKeyOps.clear();
        }
        for (size_t i = 0; i < B.steps.size(); ++i) {
            const RigExecBakedStep &step = B.steps[i];
            if (step.kind != RigExecBakedStepKind::Derived)
                continue;
            const int chain = GpuStepChain(B, step);
            if (chain >= 0 && B.chains[size_t(chain)].gpu)
                B.chains[size_t(chain)].gpuDerivedSteps.push_back(int(i));
        }
        for (uint32_t c = 0; c < B.opGraph.ops.size(); ++c) {
            const RigExecBakedStep &step =
                B.steps[B.opGraph.ops[c].originalIndex];
            if (step.kind != RigExecBakedStepKind::ChainStatus &&
                step.kind != RigExecBakedStepKind::Derived)
                continue;
            const int chain = GpuStepChain(B, step);
            if (chain >= 0 && B.chains[size_t(chain)].gpu)
                B.chains[size_t(chain)].gpuKeyOps.push_back(c);
        }
        B.gpuDerivedIndexed = true;
    }
    // The chains a fuse touched this run, translated from their revisions'
    // persisted packets here rather than by ChainStatus: whether a run
    // reaches ChainStatus depends on the history its skips branch from, a
    // frozen clone's included. Side by side, except on a frozen worker.
    std::vector<RigExecBakedProgramImpl::GeomChain *> translate;
    for (auto &chain : B.chains) {
        if (!chain.gpu)
            continue;
        for (auto &revision : chain.revisions) {
            chain.gpuPending = chain.gpuPending || revision.gpuTouched;
            revision.gpuTouched = false;
        }
        if (chain.gpuPending && chain.haveBase) {
            translate.push_back(&chain);
        }
    }
    const auto translateRange = [&B, &translate](size_t begin, size_t end) {
        for (size_t k = begin; k < end; ++k)
            TranslateGpuChain(B, translate[k]);
    };
    if (translate.size() > 1 && RigExecParallelEvaluationEnabled() &&
        !RigExecFrozenSerialActive()) {
        WorkParallelForN(translate.size(), translateRange, 1);
    } else {
        translateRange(0, translate.size());
    }
    if (B.gpuDeformMode == RigExecGpuDeformMode::Shader) {
        ResolveShaderChains(B);
        RekeyGpuChains(B, translate);
        return;
    }
    if (B.gpuDeformMode != RigExecGpuDeformMode::Readback) {
        return;
    }
    std::vector<size_t> device, cpu, moved;
    std::vector<RigExecGpuDeformJob> jobs;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        if (!chain.gpu || !chain.gpuPending)
            continue;
        chain.gpuPending = false;
        if (!chain.haveBase)
            continue;
        const bool unchanged =
            chain.gpuTranslated && chain.haveResult && chain.gpuFrame.layout &&
            chain.gpuFrame.layout->serial == chain.gpuLastSerial &&
            chain.gpuLastBaseVersion == chain.baseVersion &&
            chain.gpuFrame.frame == chain.gpuLastFrame;
        if (unchanged)
            continue;
        if (!chain.gpuTranslated) {
            cpu.push_back(c);
            continue;
        }
        chain.gpuScratch.resize(chain.lastBase.size());
        RigExecGpuDeformJob job;
        // The chain's key names its device buffers: a frozen clone holds
        // its chains apart from the live program's.
        job.chain = chain.gpuDeviceKey.Use();
        job.frame = &chain.gpuFrame;
        job.base = reinterpret_cast<const float *>(chain.lastBase.cdata());
        job.baseVersion = chain.baseVersion;
        job.out = reinterpret_cast<float *>(chain.gpuScratch.data());
        jobs.push_back(job);
        device.push_back(c);
    }
    const RigExecGpuDeformRunner runner = RigExecGetGpuDeformRunner();
    const auto t0 = std::chrono::steady_clock::now();
    const bool ran = !jobs.empty() && runner && runner(&jobs);
    const auto t1 = std::chrono::steady_clock::now();
    const auto publish = [&B, &moved](size_t c,
                                      const std::vector<GfVec3f> &points) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        chain.spare.clear();
        chain.spare.resize(
            points.size(), [&points](GfVec3f *out, GfVec3f *end) {
                std::uninitialized_copy(points.begin(),
                                        points.begin() + (end - out), out);
            });
        if (!SamePoints(chain.spare, chain.result))
            ++chain.resultVersion;
        chain.result.swap(chain.spare);
        chain.haveResult = true;
        if (chain.gpuFrame.layout) {
            chain.gpuLastFrame = chain.gpuFrame.frame;
            chain.gpuLastSerial = chain.gpuFrame.layout->serial;
        }
        chain.gpuLastBaseVersion = chain.baseVersion;
        moved.push_back(c);
    };
    for (size_t j = 0; j < device.size(); ++j) {
        if (ran && jobs[j].ok) {
            publish(device[j], B.chains[device[j]].gpuScratch);
        } else {
            cpu.push_back(device[j]);
        }
    }
    std::vector<GfVec3f> points;
    for (const size_t c : cpu) {
        RunGpuChainOnCpu(B, &B.chains[c], &points);
        publish(c, points);
        // Not this frame's program: the next one deforms again.
        B.chains[c].gpuLastSerial = 0;
    }
    // The derived steps, on the published points, as the graph would have
    // run them after ChainStatus.
    // Each writes only its own target's state, as in the graph, so they run
    // side by side -- except on a frozen worker, which owns one thread.
    std::vector<int> derived;
    for (const size_t c : moved) {
        derived.insert(derived.end(), B.chains[c].gpuDerivedSteps.begin(),
                       B.chains[c].gpuDerivedSteps.end());
    }
    B.gpuBoundary = true;
    const auto runDerived = [&B, &derived, time](size_t begin, size_t end) {
        for (size_t k = begin; k < end; ++k) {
            RigExecBakedStep &step = B.steps[size_t(derived[k])];
            step.diagnostics.clear();
            RigExecBakedRunGeometryStep(&B, &step, time);
        }
    };
    if (derived.size() > 1 && RigExecParallelEvaluationEnabled() &&
        !RigExecFrozenSerialActive()) {
        WorkParallelForN(derived.size(), runDerived, 1);
    } else {
        runDerived(0, derived.size());
    }
    B.gpuBoundary = false;
    RekeyGpuChains(B, translate);
    // Printed by the owning thread only; a frozen worker never prints.
    if (B.gpuDeformStats && !RigExecFrozenSerialActive()) {
        RigExecPathText pathText;
        const auto t2 = std::chrono::steady_clock::now();
        std::fprintf(
            stderr,
            "GPUSTATS device %zu cpu %zu moved %zu run %.3f ms after %.3f ms\n",
            device.size(), cpu.size(), moved.size(),
            std::chrono::duration<double, std::milli>(t1 - t0).count(),
            std::chrono::duration<double, std::milli>(t2 - t1).count());
        for (const size_t c : device)
            std::fprintf(
                stderr,
                "GPUSTATS chain %s builds %zu points %zu ops %zu staticF %zu\n",
                pathText(B.chains[c].target).c_str(),
                B.chains[c].gpuTranslation.LayoutBuilds(),
                B.chains[c].lastBase.size(),
                B.chains[c].gpuFrame.layout
                    ? B.chains[c].gpuFrame.layout->OpCount()
                    : 0,
                B.chains[c].gpuFrame.layout
                    ? B.chains[c].gpuFrame.layout->staticF.size()
                    : 0);
        for (size_t j = 0; j < device.size(); ++j)
            if (!(ran && jobs[j].ok))
                std::fprintf(stderr, "GPUSTATS fallback %s (ran %d)\n",
                             pathText(B.chains[device[j]].target).c_str(),
                             int(ran));
    }
}

void
RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    // A GPU chain's Derived points arrive at the publication boundary,
    // which runs this step then (RigExecBakedResolveGpuChains); the
    // deferral stays with this adapter around the shared body.
    if (step->kind == RigExecBakedStepKind::Derived && step->object >= 0 &&
        size_t(step->object) < B.derivedIndex.size()) {
        const auto &[chainIndex, derivedIndex] =
            B.derivedIndex[size_t(step->object)];
        if (size_t(chainIndex) < B.chains.size() &&
            B.chains[size_t(chainIndex)].gpu && !B.gpuBoundary) {
            return;
        }
    }
    // A GPU-owned fuse makes its status decision without points here;
    // the boundary translates the chain from it.
    if (step->kind == RigExecBakedStepKind::RevisionFuse && step->object >= 0 &&
        size_t(step->object) < B.revisionIndex.size()) {
        const auto &[chainIndex, revisionIndex] =
            B.revisionIndex[size_t(step->object)];
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.gpuOwned) {
            const bool skin = revision.op == RigExecRevisionOp::Skin;
            const bool packetValid = revision.parameters.valid &&
                                     (!skin || revision.influencesValid);
            EmitInvalidPacketLine(B, revision, packetValid, &step->diagnostics);
            revision.executed = true;
            step->counters.revisionsExecuted = 1;
            revision.resultStatus = revision.status.state;
            revision.gpuApplies =
                packetValid && revision.status.AllowsApply() &&
                revision.acceptance != RigExecRevisionAcceptance::Refuses;
            if (!revision.gpuApplies && revision.status.AllowsApply()) {
                revision.resultStatus = _tokens->moverFailed;
            }
            ++revision.doneVersion;
            revision.gpuTouched = true;
            revision.lastStatus = revision.status;
            revision.ran = true;
            return;
        }
    }
    // EvalCore's one chain bodies (rigExecEvalCore/chain.h) over this
    // adapter's host; no second CPU step switch remains here.
    BakedChainHost host(BakedChainInputs{&B}, BakedChainState{&B, step, time});
    eval::RunGeometryStep(host,
                          eval::StepRef{uint32_t(step->kind), step->object,
                                        step->part, size_t(step->object)});
}

// The report.

namespace {

/// Two decimals, so a ratio in the report lines up with the scheduler's.
std::string
Fixed2(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return buffer;
}

/// Whether \p step is one of the four steps that make up revision \p id.
bool
IsStepOfRevision(const RigExecBakedStep &step, int id)
{
    if (step.object != id) {
        return false;
    }
    switch (step.kind) {
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return true;
    default:
        return false;
    }
}

/// The cost of revision \p id run one step at a time, and the cost of its
/// longest dependency chain -- the floor the chunks cannot go below however
/// many threads there are. The longest path is taken over the revision's own
/// steps only: an edge leaving the revision is the rest of the frame's
/// business, and what this number answers is "how much of this skin is
/// width and how much is depth".
void
SkinCosts(const RigExecBakedProgramImpl &B, int id, double *serial,
          double *criticalPath)
{
    *serial = 0.0;
    *criticalPath = 0.0;
    // Steps are in topological order, so one forward pass settles the
    // longest path into each of them.
    std::vector<double> through(B.steps.size(), 0.0);
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        if (!IsStepOfRevision(step, id)) {
            continue;
        }
        double before = 0.0;
        for (const int pred : step.preds) {
            if (IsStepOfRevision(B.steps[size_t(pred)], id)) {
                before = std::max(before, through[size_t(pred)]);
            }
        }
        through[index] = before + step.cost;
        *serial += step.cost;
        *criticalPath = std::max(*criticalPath, through[index]);
    }
}

} // namespace

std::string
RigExecBakedGeometryReport(const RigExecBakedProgramImpl &B)
{
    std::string out;
    char line[256];
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            const int id = B.chainRevisionBegin[c] + int(r);
            const size_t influences = revision.influenceSlots.size();
            const size_t chunks = revision.chunks.size();
            size_t vertexMin = size_t(-1), vertexMax = 0, vertexSum = 0;
            size_t keyMin = size_t(-1), keyMax = 0, keySum = 0, wide = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                 revision.chunks) {
                // An unpartitioned revision is one chunk over everything,
                // which is every influence and every vertex it has.
                const size_t vertices = revision.chunked
                                            ? size_t(chunk.end - chunk.begin)
                                            : revision.partitionPointCount;
                const size_t key =
                    revision.chunked ? chunk.key.size() : influences;
                vertexMin = std::min(vertexMin, vertices);
                vertexMax = std::max(vertexMax, vertices);
                vertexSum += vertices;
                keyMin = std::min(keyMin, key);
                keyMax = std::max(keyMax, key);
                keySum += key;
                if (influences && key * 2 >= influences) {
                    ++wide;
                }
            }
            std::snprintf(
                line, sizeof(line),
                "  %s: %zu chunk(s), %zu vertex(es), %zu influence(s)%s\n",
                revision.moverPathText.c_str(), chunks, vertexSum, influences,
                revision.chunked ? "" : " [not partitioned]");
            out += line;
            std::snprintf(line, sizeof(line),
                          "    vertices/chunk min %zu mean %.1f max %zu\n",
                          vertexMin == size_t(-1) ? 0 : vertexMin,
                          chunks ? double(vertexSum) / double(chunks) : 0.0,
                          vertexMax);
            out += line;
            std::snprintf(
                line, sizeof(line),
                "    |key| min %zu mean %.1f max %zu; %zu/%zu chunk(s) reach "
                "half the influences\n",
                keyMin == size_t(-1) ? 0 : keyMin,
                chunks ? double(keySum) / double(chunks) : 0.0, keyMax, wide,
                chunks);
            out += line;
            // What the partition bought, in the scheduler's own units: the
            // revision's serial cost against its longest dependency chain.
            double skinSerial = 0.0, skinPath = 0.0;
            SkinCosts(B, id, &skinSerial, &skinPath);
            std::snprintf(
                line, sizeof(line),
                "    skin serial %.2fus critical path %.2fus%s\n", skinSerial,
                skinPath,
                skinPath > 0.0
                    ? (" (" + Fixed2(skinSerial / skinPath) + "x)").c_str()
                    : "");
            out += line;
            for (size_t k = 0; k < chunks; ++k) {
                const auto &chunk = revision.chunks[k];
                const auto producers =
                    ChunkProducers(revision, chunk, revision.chunked);
                std::snprintf(line, sizeof(line), "    [%zu] %d..%d producers",
                              k, chunk.begin, chunk.end);
                out += line;
                for (const auto &producer : producers)
                    out += " " + std::to_string(producer.first) + ":" +
                           std::to_string(producer.second);
                out += "\n";
            }
            std::snprintf(
                line, sizeof(line),
                "    natural producer count min %d max %d distinct sets %zu\n",
                revision.partitionProducerMin, revision.partitionProducerMax,
                revision.partitionDistinctReads);
            out += line;
        }
    }
    return out;
}

void
RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                            RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedEnsureEpilogueIndex(&B);
    // Chain by chain, in chain order, and inside a chain exactly where the
    // straight line put each line: every revision's assemble diagnostics
    // first, then the status sweep's, then each derived target's. Program
    // order is that order.
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Geometry,
                                &pose->diagnostics);
    // The chains' points, weight fields and derived targets, in the same
    // program order: the maps are keyed, so the order shows only where two
    // steps name one key, and the later step's value stands as it did.
    for (const uint32_t index : B.epilogue.geometrySteps) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            // `haveBase` for the same reason the two arms below test it: a
            // chain whose points stopped reading at this time drives
            // nothing, and the assemble that would have refreshed the field
            // returned before it ever looked at the packet. Its last answer
            // is then last FRAME's, and publishing that is the one shape of
            // staleness a value comparison cannot catch -- the dynamic path
            // publishes no field at all for such a chain.
            if (B.publishWeightFields && chain.haveBase &&
                revision.weightFieldPublished) {
                RigExecResolvedWeightField &field =
                    pose->weightFields
                        [B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                // Shared, never copied: RevisionStatic replaces the array
                // rather than writing into it.
                field.weights = revision.publishedWeightValues;
            }
        } else if (step.kind == RigExecBakedStepKind::ChainStatus) {
            RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            if (chain.haveBase) {
                pose->movedProperties[chain.target] =
                    RigExecBakedChainPublication(chain);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[size_t(derivedIndex)];
            if (chain.haveBase && derived.haveBase) {
                if (!derived.matrixTarget) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.result);
                } else if (derived.haveMatrix) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.matrix);
                }
            }
        }
    }
}

VtValue
RigExecBakedChainPublication(const RigExecBakedProgramImpl::GeomChain &chain)
{
    if (chain.gpu && chain.gpuPublication) {
        return VtValue(*chain.gpuPublication);
    }
    return VtValue(chain.result);
}

RigExecSurfaceProjectorFrames
RigExecBakedProjectorFrames(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision)
{
    // Asset frames: each provider's rest times its base and final
    // computeMatrix, the tables every geometry revision reads.
    RigExecSurfaceProjectorFrames frames;
    const SdfPath providers[3] = {revision.binding.transform,
                                  revision.binding.transformSpace,
                                  revision.binding.carrySpace};
    const int slots[3] = {revision.transformSlot, revision.transformSpaceSlot,
                          revision.carrySpaceSlot};
    for (int k = 0; k < 3; ++k) {
        if (providers[k].IsEmpty())
            continue;
        frames.named[k] = true;
        const int slot = slots[k];
        if (slot < 0 || !B.restFrames[size_t(slot)].IsValid())
            continue;
        const auto &rest = B.restFrames[size_t(slot)].points;
        frames.base[k] = RigExecWorldFromRest(rest, B.baseMatrix[size_t(slot)]);
        frames.final[k] =
            RigExecWorldFromRest(rest, B.finalMatrix[size_t(slot)]);
        frames.resolved[k] = true;
    }
    return frames;
}

bool
RigExecBakedRunProjectorTarget(const RigExecBakedProgramImpl &B,
                               const RigExecBakedProgramImpl::GeomChain &chain,
                               RigExecBakedProgramImpl::GeomRevision &revision,
                               GfMatrix4d *matrix,
                               std::vector<std::string> *diagnostics)
{
    RigExecRevisionLeafView view;
    view.decl = &revision.leaves.decl;
    view.values = &RigExecBakedResolvePathLeaves(B, revision.leaves);
    RigExecProjectorReads reads;
    RigExecReadProjectorTargetFromLeaves(revision.op, revision.binding, view,
                                         &reads);
    return RigExecRunGeometryMatrix(
        revision.op, revision.binding, RigExecBakedProjectorFrames(B, revision),
        reads,
        revision.surfaceCache.PointSamples(false, chain.lastBase.cdata(),
                                           chain.lastBase.size()),
        revision.surfaceCache.PointSamples(true, chain.result.cdata(),
                                           chain.result.size()),
        revision.moverPathText, matrix, diagnostics, &revision.surfaceCache);
}

void
RigExecBakedBindPathLeaves(const UsdStageRefPtr &stage,
                           RigExecBakedPathLeaves *leaves)
{
    const size_t n = leaves->decl.keys.size();
    leaves->attributes.assign(n, UsdAttribute());
    leaves->hops.assign(n, std::vector<SdfPath>());
    leaves->varying.assign(n, 0);
    leaves->values.clear();
    leaves->values.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        if (stage && key.path.IsPropertyPath()) {
            leaves->attributes[k] = stage->GetAttributeAtPath(key.path);
        }
        bool varying = false;
        RigExecRevisionLeafHops(key, leaves->attributes[k], &leaves->hops[k],
                                &varying);
        leaves->varying[k] = varying ? 1 : 0;
        leaves->values.push_back(key.fallback);
    }
    leaves->compiledRig.assign(n, 0);
    leaves->overrideReached.assign(n, 0);
    for (size_t role = 0; role < leaves->decl.roles.size(); ++role) {
        const int k = leaves->decl.roles[role];
        if (k >= 0 && size_t(k) < n &&
            RigExecRevisionLeafRoleIsTopology(RigExecRevisionLeafRole(role))) {
            leaves->compiledRig[size_t(k)] = 1;
        }
    }
    leaves->changed.assign(n, 0);
    leaves->mustSample.assign(n, 0);
    leaves->sampled = false;
    RigExecBakedResetPathLeafVersions(leaves);
}

namespace {
// No run has read the key since its versions restarted.
constexpr uint64_t kPathLeafUnread = std::numeric_limits<uint64_t>::max();
} // namespace

void
RigExecBakedResetPathLeafVersions(RigExecBakedPathLeaves *leaves)
{
    const size_t n = leaves->values.size();
    leaves->versions.assign(n, 0);
    leaves->observed.assign(n, VtValue());
    leaves->observedVersions.assign(n, 0);
    leaves->observedRuns.assign(n, kPathLeafUnread);
}

bool
RigExecBakedSetPathLeaf(RigExecBakedPathLeaves *leaves, size_t k, VtValue value,
                        uint64_t run)
{
    if (k >= leaves->values.size()) {
        return false;
    }
    const bool changed = !RigExecBakedHeadValueSame(value, leaves->values[k]);
    // RigExecBakedResetPathLeafVersions sizes the four tables together;
    // unsized, every key over them is inexact (InputKey), so a version not
    // kept here can never pass for an unchanged one.
    if (k < leaves->versions.size() && k < leaves->observed.size() &&
        k < leaves->observedVersions.size() &&
        k < leaves->observedRuns.size()) {
        // The first write since a run read the key: what it held is what
        // that run saw.
        if (leaves->observedRuns[k] != run) {
            leaves->observed[k] = leaves->values[k];
            leaves->observedVersions[k] = leaves->versions[k];
            leaves->observedRuns[k] = run;
        }
        // At the observed version the held value has the observed bytes.
        const bool moved =
            leaves->versions[k] == leaves->observedVersions[k]
                ? changed
                : !RigExecBakedHeadValueSame(value, leaves->observed[k]);
        leaves->versions[k] = leaves->observedVersions[k] + (moved ? 1 : 0);
    }
    leaves->values[k] = std::move(value);
    return changed;
}

bool
RigExecBakedHasExternalInputRoute(const RigExecBakedProgramImpl &B,
                                  const SdfPath &path)
{
    const auto hasRoute = [&](const auto &revision) {
        if (revision.op != RigExecRevisionOp::External)
            return false;
        const auto &binding = revision.binding;
        const auto &leaves = revision.leaves;
        for (size_t i = 0; i < binding.externalInputs.size(); ++i) {
            const auto &key = binding.externalInputs[i];
            using Flavour = RigExecRevisionLeafFlavour;
            if (key.time != RigExecRevisionLeafTime::AtTime ||
                (key.flavour != Flavour::Resolved &&
                 key.flavour != Flavour::ResolvedOnly &&
                 key.flavour != Flavour::OverlayThenRaw))
                continue;
            if (key.path == path)
                return true;
            if (leaves.decl.externalBegin < 0)
                continue;
            const size_t slot = size_t(leaves.decl.externalBegin) + i;
            if (slot < leaves.hops.size() &&
                std::find(leaves.hops[slot].begin(), leaves.hops[slot].end(),
                          path) != leaves.hops[slot].end())
                return true;
        }
        return false;
    };
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions)
            if (hasRoute(revision))
                return true;
        for (const auto &derived : chain.derived)
            if (hasRoute(derived.revision))
                return true;
    }
    return false;
}

VtValue
RigExecBakedResolvePathLeaf(const RigExecBakedProgramImpl &B,
                            const RigExecBakedPathLeaves &leaves, size_t k)
{
    if (k >= leaves.decl.keys.size())
        return VtValue();
    VtValue value = k < leaves.values.size() ? leaves.values[k] : VtValue();
    const auto &key = leaves.decl.keys[k];
    // The compiled typed walk owns overlay selection and the tail-first
    // authored fallback. A direct version shortcut must not
    // discard that fallback when a produced opinion is absent or mistyped.
    if (key.flavour != RigExecRevisionLeafFlavour::Present &&
        k < leaves.walks.size() && leaves.walks[k] >= 0)
        return RigExecBakedSampleWalkedPathLeaf(B, key, leaves.walks[k]);
    const int version =
        k < leaves.exactVersions.size() ? leaves.exactVersions[k] : -1;
    const int recordIndex = k < leaves.exactRecordIndices.size()
                                ? leaves.exactRecordIndices[k]
                                : -1;
    const auto exactDoubleFallback = [&]() {
        if (k < leaves.walks.size() && leaves.walks[k] >= 0)
            return RigExecBakedSampleWalkedPathLeaf(B, key, leaves.walks[k]);
        return value.IsHolding<double>() ? value : key.fallback;
    };
    if (recordIndex >= 0 && size_t(recordIndex) < B.propertyRecords.size()) {
        const size_t r = size_t(recordIndex);
        const auto &record = B.propertyRecords[r];
        const auto &chain = B.propertyChains[record.chain];
        const size_t source = chain.versionBase +
                              std::min(record.applied, chain.revisions.size());
        const bool present = !B.recordStoodAside[r] &&
                             source < B.propertyVersionValid.size() &&
                             B.propertyVersionValid[source];
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            const bool rawPresent = k < leaves.values.size() &&
                                    leaves.values[k].IsHolding<bool>() &&
                                    leaves.values[k].UncheckedGet<bool>();
            value = VtValue(rawPresent || present);
        } else if (present) {
            if (key.type == RigExecRevisionLeafType::Double &&
                !B.recordValues[r].IsHolding<double>())
                return exactDoubleFallback();
            value = B.recordValues[r];
            if (key.type == RigExecRevisionLeafType::Dial &&
                value.IsHolding<float>())
                value = VtValue(double(value.UncheckedGet<float>()));
            else if (key.type == RigExecRevisionLeafType::Float &&
                     value.IsHolding<double>())
                value = VtValue(float(value.UncheckedGet<double>()));
        } else if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly) {
            value = key.fallback;
        }
        return value;
    }
    if (version >= 0 && size_t(version) < B.propertyVersionValid.size()) {
        const bool present = B.propertyVersionValid[size_t(version)] != 0;
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            const bool rawPresent = k < leaves.values.size() &&
                                    leaves.values[k].IsHolding<bool>() &&
                                    leaves.values[k].UncheckedGet<bool>();
            value = VtValue(rawPresent || present);
        } else if (present) {
            const auto &v = B.propertyValues[size_t(version)];
            using Arm = RigExecBakedPropertyChain::Arm;
            const int type = k < leaves.exactValueTypes.size()
                                 ? leaves.exactValueTypes[k]
                                 : -1;
            const bool asFloat = type == int(Arm::Float);
            const bool numeric = asFloat || type == int(Arm::Double);
            const bool compatible =
                ((key.type == RigExecRevisionLeafType::Float ||
                  key.type == RigExecRevisionLeafType::Dial) &&
                 numeric) ||
                (key.type == RigExecRevisionLeafType::Double &&
                 type == int(Arm::Double)) ||
                (key.type == RigExecRevisionLeafType::Matrix4d &&
                 type == int(Arm::Matrix4d)) ||
                (key.type == RigExecRevisionLeafType::Vec3f &&
                 type == int(Arm::Vec3f));
            if (!compatible) {
                if (key.type == RigExecRevisionLeafType::Double)
                    return exactDoubleFallback();
                if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly)
                    value = key.fallback;
                return value;
            }
            switch (key.type) {
            case RigExecRevisionLeafType::Float:
                value = VtValue(asFloat ? v.f : float(v.d));
                break;
            case RigExecRevisionLeafType::Double:
                value = VtValue(v.d);
                break;
            case RigExecRevisionLeafType::Dial: {
                value = VtValue(asFloat ? double(v.f) : v.d);
                break;
            }
            case RigExecRevisionLeafType::Matrix4d:
                value = VtValue(v.m);
                break;
            case RigExecRevisionLeafType::Vec3f:
                value = VtValue(v.v);
                break;
            default:
                break;
            }
        } else if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly) {
            value = key.fallback;
        }
    } else if (k < leaves.walks.size() && leaves.walks[k] >= 0) {
        value = RigExecBakedSampleWalkedPathLeaf(B, key, leaves.walks[k]);
    }
    return value;
}

const std::vector<VtValue> &
RigExecBakedResolvePathLeaves(const RigExecBakedProgramImpl &B,
                              const RigExecBakedPathLeaves &leaves)
{
    leaves.consumedValues.resize(leaves.decl.keys.size());
    for (size_t k = 0; k < leaves.decl.keys.size(); ++k)
        leaves.consumedValues[k] = RigExecBakedResolvePathLeaf(B, leaves, k);
    return leaves.consumedValues;
}

bool
RigExecBakedGeometryReadsLeaf(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const RigExecBakedStep &step, size_t key)
{
    const bool assemble = step.kind == RigExecBakedStepKind::RevisionStatic;
    if (!revision.blendChunked)
        return assemble;
    for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
        const auto &channel = revision.blendChannels[c];
        if (int(key) != channel.weightLeaf &&
            int(key) != channel.weightHeldLeaf)
            continue;
        if (assemble || step.part < 0 ||
            size_t(step.part) >= revision.chunks.size())
            return false;
        const auto &channels = revision.chunks[size_t(step.part)].key;
        return std::binary_search(channels.begin(), channels.end(), int(c));
    }
    return assemble;
}

bool
RigExecRevisionLeafHopsComplete(const RigExecRevisionLeafKey &key)
{
    // A Dial read walks the connections through the overlay whatever its
    // flavour; RigExecRevisionLeafHops lists that walk for these two only.
    return key.type != RigExecRevisionLeafType::Dial ||
           key.flavour == RigExecRevisionLeafFlavour::Resolved ||
           key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly;
}

void
RigExecBakedPrepareOverridePaths(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    std::vector<SdfPath> &paths = B.overridePathsSorted;
    paths.clear();
    if (!B.interactiveOverrides) {
        return;
    }
    for (const auto &overrideValue : *B.interactiveOverrides) {
        paths.push_back(
            overrideValue.attribute.IsEmpty()
                ? overrideValue.prim.AppendProperty(overrideValue.computation)
                : overrideValue.prim.AppendProperty(overrideValue.attribute));
    }
    std::sort(paths.begin(), paths.end());
    paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
}

void
RigExecBakedSamplePathLeaves(RigExecBakedProgramImpl *program,
                             RigExecBakedPathLeaves *leaves, UsdTimeCode time,
                             bool all, const std::vector<int> &skip)
{
    RigExecBakedProgramImpl &B = *program;
    const bool overrides =
        B.interactiveOverrides && !B.interactiveOverrides->empty();
    all = all || !leaves->sampled;
    const bool overridden = overrides || leaves->overrides;
    const bool chainsMoved = leaves->chainSerial != B.pathLeafChainSerial;
    const bool timeMoved = !leaves->sampled || time != leaves->time;
    // Default and a numeric time read different opinions of an attribute
    // that holds both, whatever its variance.
    const bool defaultMoved =
        timeMoved && (time.IsDefault() || leaves->time.IsDefault());
    // The overlay the reads consult, built by the first key that reads.
    RigExecResolvedInputs sourceOnly;
    bool sourceOnlyBuilt = false;
    const auto overlay = [&]() -> const RigExecResolvedInputs * {
        if (!sourceOnlyBuilt) {
            sourceOnlyBuilt = true;
            if (B.interactiveOverrides) {
                for (const auto &overrideValue : *B.interactiveOverrides) {
                    const SdfPath path =
                        overrideValue.attribute.IsEmpty()
                            ? overrideValue.prim.AppendProperty(
                                  overrideValue.computation)
                            : overrideValue.prim.AppendProperty(
                                  overrideValue.attribute);
                    sourceOnly.SetProperty(path, overrideValue.value);
                }
            }
        }
        return &sourceOnly;
    };
    // Whether a standing override is at a path key \p k's read can reach:
    // the overlay the read consults holds nothing anywhere else.
    const auto overrideReaches = [&](size_t k) {
        if (!overrides || k >= leaves->hops.size()) {
            return false;
        }
        const std::vector<SdfPath> &paths = B.overridePathsSorted;
        for (const SdfPath &hop : leaves->hops[k]) {
            if (std::binary_search(paths.begin(), paths.end(), hop)) {
                return true;
            }
        }
        return false;
    };
    // Key \p k's value as a sample reads it now.
    const auto read = [&](const RigExecRevisionLeafKey &key, size_t k) {
        VtValue value = RigExecSampleRevisionLeaf(key, leaves->attributes[k],
                                                  overlay(), time, &B.upstream);
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            bool present = false;
            if (B.interactiveOverrides) {
                for (const auto &overrideValue : *B.interactiveOverrides) {
                    const SdfPath path =
                        overrideValue.attribute.IsEmpty()
                            ? overrideValue.prim.AppendProperty(
                                  overrideValue.computation)
                            : overrideValue.prim.AppendProperty(
                                  overrideValue.attribute);
                    if (path == key.path && !overrideValue.value.IsEmpty())
                        present = true;
                }
            }
            value = VtValue(present);
        }
        return value;
    };
    std::vector<SdfPath> hops;
    for (size_t k = 0; k < leaves->decl.keys.size(); ++k) {
        leaves->changed[k] = 0;
        if (std::find(skip.begin(), skip.end(), int(k)) != skip.end()) {
            leaves->mustSample[k] = 1;
            continue;
        }
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        const bool rebind = all || leaves->VarianceStale(B.programStamp, k);
        const bool atTime = key.time == RigExecRevisionLeafTime::AtTime;
        // A standing override moves a gated key only where one stands on its
        // hops, now or at its last sample: a topology key (compiled rig state)
        // always, any key whose hops list every overlay read while
        // RIGEXEC_PATH_LEAF_GATING is on.
        const bool compiledRig =
            k < leaves->compiledRig.size() && leaves->compiledRig[k] != 0;
        const bool gated =
            compiledRig ||
            (B.pathLeafGating && RigExecRevisionLeafHopsComplete(key));
        bool reached = gated && overrideReaches(k);
        const bool overrideMoves =
            overridden && (!gated || reached ||
                           (k < leaves->overrideReached.size() &&
                            leaves->overrideReached[k] != 0));
        if (!rebind && !overrideMoves && !chainsMoved &&
            !(atTime && timeMoved && (leaves->varying[k] || defaultMoved))) {
            // RIGEXEC_VERIFY_PATH_LEAF_GATING: a key only the gate kept
            // reads what it holds.
            if (B.verifyPathLeafGating && overridden && gated && !compiledRig &&
                !RigExecBakedHeadValueSame(read(key, k), leaves->values[k])) {
                ++B.pathLeafGateMismatches;
                TF_VERIFY(false,
                          "path leaf %s skipped by its override gate reads "
                          "another value",
                          key.path.GetText());
            }
            continue;
        }
        if (rebind) {
            // An edit can author time samples where there were none.
            bool varying = false;
            RigExecRevisionLeafHops(key, leaves->attributes[k], &hops,
                                    &varying);
            leaves->varying[k] = varying ? 1 : 0;
            if (gated && k < leaves->hops.size()) {
                leaves->hops[k] = hops;
                reached = overrideReaches(k);
            }
        }
        if (gated && k < leaves->overrideReached.size()) {
            leaves->overrideReached[k] = reached ? 1 : 0;
        }
        leaves->mustSample[k] = 0;
        ++B.pathLeafSamples;
        // Sampling publishes source opinions only. Produced values are resolved
        // by the owning body after its graph dependencies complete.
        const int walk = k < leaves->walks.size() ? leaves->walks[k] : -1;
        VtValue value = read(key, k);
        leaves->changed[k] =
            RigExecBakedSetPathLeaf(leaves, k, std::move(value), B.pathLeafRun)
                ? 1
                : 0;
        if (walk >= 0 && leaves->changed[k]) {
            B.readerWalkChanged[size_t(walk)] = 1;
        }
    }
    leaves->sampled = true;
    leaves->time = time;
    leaves->stamp = B.programStamp;
    leaves->overrides = overrides;
    leaves->chainSerial = B.pathLeafChainSerial;
}

bool
RigExecBakedLeafVaryingNow(const RigExecBakedProgramImpl &B,
                           const RigExecBakedPathLeaves &leaves, size_t k)
{
    if (k >= leaves.decl.keys.size() || k >= leaves.attributes.size()) {
        return false;
    }
    if (!leaves.VarianceStale(B.programStamp, k)) {
        return k < leaves.varying.size() && leaves.varying[k] != 0;
    }
    std::vector<SdfPath> hops;
    bool varying = false;
    RigExecRevisionLeafHops(leaves.decl.keys[k], leaves.attributes[k], &hops,
                            &varying);
    return varying;
}

// The skin layouts.

RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(RigExecBakedProgramImpl *program, size_t r)
{
    RigExecBakedProgramImpl &B = *program;
    if (r < B.revisionIndex.size()) {
        const auto &[chain, revision] = B.revisionIndex[r];
        return &B.chains[size_t(chain)].revisions[size_t(revision)];
    }
    r -= B.revisionIndex.size();
    if (r < B.derivedIndex.size()) {
        const auto &[chain, derived] = B.derivedIndex[r];
        return &B.chains[size_t(chain)].derived[size_t(derived)].revision;
    }
    return nullptr;
}

const RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(const RigExecBakedProgramImpl &program, size_t r)
{
    return RigExecBakedLayoutRevision(
        const_cast<RigExecBakedProgramImpl *>(&program), r);
}

void
RigExecBakedBuildLayoutSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
    for (size_t r = 0; r < count; ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            *RigExecBakedLayoutRevision(&B, r);
        if (revision.op != RigExecRevisionOp::Skin) {
            continue;
        }
        revision.layoutLeaves = RigExecBakedPathLeaves();
        RigExecDeclareSkinLayoutLeaves(revision.moverPath,
                                       &revision.layoutLeaves.decl);
        RigExecBakedBindPathLeaves(B.stage, &revision.layoutLeaves);
        revision.layoutOverlay.assign(revision.layoutLeaves.decl.keys.size(),
                                      VtValue());
        revision.layoutTopologyFixed =
            RigExecSkinLayoutTopologyIsFixed(revision.moverPrim);
        revision.layoutFixed =
            revision.layoutTopologyFixed &&
            RigExecSkinLayoutWeightsAreFixed(revision.moverPrim);
        revision.layoutFixedChanged = false;
        revision.layoutRan = false;
        revision.layoutHandle = nullptr;
        RigExecBakedStep step;
        step.isHead = true;
        step.part = 0;
        step.kind = RigExecBakedStepKind::SkinTopology;
        step.object = int(r);
        step.label = "SkinTopology " + revision.moverPathText;
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        B.steps.push_back(std::move(step));
    }
}

void
RigExecBakedDeclareLayoutReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        size_t r = 0;
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse:
            r = size_t(step.object);
            break;
        case RigExecBakedStepKind::Derived:
            r = B.revisionIndex.size() + size_t(step.object);
            break;
        default:
            continue;
        }
        const RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(B, r);
        if (revision && revision->op == RigExecRevisionOp::Skin) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        }
    }
}

void
RigExecBakedSampleLayoutLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               UsdTimeCode time, bool all)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    revision->layoutFixedChanged = false;
    // The weights' variance can change on a retained program after an
    // authored weights edit; an edit to the indices or the element size
    // rebuilds it, so the topology half stands from Build. Capture source
    // metadata only when its owning-thread bindings refresh.
    revision->layoutFixed =
        all || !leaves.sampled
            ? revision->layoutTopologyFixed &&
                  RigExecSkinLayoutWeightsAreFixed(revision->moverPrim)
            : RigExecBakedLayoutFixedNow(B, *revision);
    RigExecBakedSamplePathLeaves(&B, &leaves, time, all);
}

bool
RigExecBakedLayoutFixedNow(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision)
{
    if (!revision.layoutTopologyFixed) {
        return false;
    }
    const RigExecBakedPathLeaves &leaves = revision.layoutLeaves;
    const int weights = leaves.decl.Role(RigExecRevisionLeafRole::JointWeights);
    const bool stale =
        weights >= 0 ? leaves.VarianceStale(B.programStamp, size_t(weights))
                     : leaves.AnyVarianceStale(B.programStamp);
    return stale ? RigExecSkinLayoutWeightsAreFixed(revision.moverPrim)
                 : revision.layoutFixed;
}

void
RigExecBakedRunLayoutOp(const RigExecBakedProgramImpl &B,
                        RigExecBakedProgramImpl::GeomRevision *revision)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    const auto &values = RigExecBakedResolvePathLeaves(B, leaves);
    const auto value = [&](Role role) -> const VtValue * {
        const int key = leaves.decl.Role(role);
        return key >= 0 && size_t(key) < values.size() ? &values[size_t(key)]
                                                       : nullptr;
    };
    const VtValue *indicesValue = value(Role::JointIndices);
    const VtValue *weightsValue = value(Role::JointWeights);
    const VtValue *sizeValue = value(Role::ElementSize);
    const VtIntArray indices =
        indicesValue && indicesValue->IsHolding<VtIntArray>()
            ? indicesValue->UncheckedGet<VtIntArray>()
            : VtIntArray();
    const VtFloatArray weights =
        weightsValue && weightsValue->IsHolding<VtFloatArray>()
            ? weightsValue->UncheckedGet<VtFloatArray>()
            : VtFloatArray();
    const int elementSize = sizeValue && sizeValue->IsHolding<int>()
                                ? sizeValue->UncheckedGet<int>()
                                : 1;
    auto built = std::make_shared<RigExecSkinTopology>();
    built->indices.assign(indices.cdata(), indices.cdata() + indices.size());
    built->weights.assign(weights.cdata(), weights.cdata() + weights.size());
    built->elementSize = elementSize;
    built->influenceCount = revision->influenceSlots.size();
    const eval::SkinLayoutBuild build = eval::BuildSkinLayout(
        built->indices.data(), built->indices.size(), built->weights.data(),
        built->weights.size(), elementSize, built->influenceCount);
    built->pointCount = build.pointCount;
    built->validated = build.validated;
    // The same layout keeps its object, so the packet that carries it still
    // compares equal and the kernel does not re-run.
    if (revision->layoutCandidate && *revision->layoutCandidate == *built) {
        revision->layoutHandle = revision->layoutCandidate;
        return;
    }
    revision->layoutHandle = std::move(built);
    revision->layoutCandidate = revision->layoutHandle;
    // `layoutSerial` names `layoutCandidate`: the SkinTopology and
    // RevisionPacket keys carry it in place of the layout's bytes. Only a
    // newly built layout -- which differs from the candidate it replaces --
    // moves it; handing the candidate back keeps it.
    ++revision->layoutSerial;
}

namespace {

// The packet fields two assemblies disagree on, by name.
std::string
PacketDifference(const RigExecMoverParameters &a,
                 const RigExecMoverParameters &b)
{
    std::string out;
    const auto field = [&out](bool same, const char *name) {
        if (!same) {
            out += out.empty() ? name : std::string(", ") + name;
        }
    };
    field(a.kind == b.kind, "kind");
    field(a.enabled == b.enabled, "enabled");
    field(a.valid == b.valid, "valid");
    field(a.transform == b.transform, "transform");
    field(a.radialWeight == b.radialWeight, "radialWeight");
    field(a.weights == b.weights, "weights");
    field(a.blendDeltas == b.blendDeltas, "blendDeltas");
    field(a.blendSurfaceFrame == b.blendSurfaceFrame, "blendSurfaceFrame");
    field(a.referenceVolume == b.referenceVolume, "referenceVolume");
    field(a.strength == b.strength, "strength");
    field(a.topologyCounts == b.topologyCounts, "topologyCounts");
    field(a.topologyIndices == b.topologyIndices, "topologyIndices");
    field(a.restPoints == b.restPoints, "restPoints");
    field(a.auxPoints == b.auxPoints, "auxPoints");
    field(a.bindCoords == b.bindCoords, "bindCoords");
    field(a.widths == b.widths, "widths");
    field(a.skinTransforms == b.skinTransforms, "skinTransforms");
    field(a.skinIndices == b.skinIndices, "skinIndices");
    field(a.skinWeights == b.skinWeights, "skinWeights");
    field(a.skinElementSize == b.skinElementSize, "skinElementSize");
    field(a.skinningMethod == b.skinningMethod, "skinningMethod");
    field(a.skinTopology == b.skinTopology, "skinTopology");
    return out.empty() ? std::string("another field") : out;
}

} // namespace

std::vector<std::string>
RigExecBakedProgramTesting::ShadowAssembly(
    const RigExecBakedProgram &program, const RigExecResolvedInputs &resolved,
    UsdTimeCode time,
    const std::function<void(const SdfPath &, RigExecRevisionLeafDecl *)> &edit,
    size_t *compared)
{
    RigExecBakedProgramImpl &B = *program._impl;
    std::vector<std::string> report;
    size_t count = 0;
    // Both assemblies over the caller's resolved inputs, each on a copy of
    // the revision, so the program keeps the state the run left.
    RigExecResolvedInputs local = resolved;
    RigExecResolvedInputs *const saved = B.resolvedInputs;
    B.resolvedInputs = &local;
    const auto shadow = [&](const RigExecBakedProgramImpl::GeomRevision &live,
                            const GfVec3f *base, size_t baseCount) {
        if (!live.leaves.decl.assembles) {
            return;
        }
        ++count;
        RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
        RigExecBakedProgramImpl::GeomRevision fromStage = live;
        fromLeaves.blendChunked = fromStage.blendChunked = false;
        if (edit) {
            edit(live.moverPath, &fromLeaves.leaves.decl);
        }
        std::vector<std::string> missing, leafLines, stageLines;
        RigExecExternalRevision externalA, externalB;
        const RigExecMoverParameters a = RigExecBakedAssembleFromLeaves(
            &B, &fromLeaves, base, baseCount, &leafLines, &missing, size_t(-1),
            &externalA);
        const RigExecMoverParameters b = AssembleRevisionFromStage(
            B, local, &fromStage, base, baseCount, time, &stageLines,
            /*record=*/false, &externalB);
        const std::string &where = live.moverPathText;
        for (const std::string &name : missing) {
            report.push_back(where + ": undeclared read " + name);
        }
        if (!(a == b && externalA == externalB)) {
            report.push_back(where + ": packets differ (" +
                             PacketDifference(a, b) + ")");
        }
        if (RigExecStatusForParameters(a, live.moverPath) !=
            RigExecStatusForParameters(b, live.moverPath)) {
            report.push_back(where + ": statuses differ");
        }
        if (leafLines != stageLines) {
            report.push_back(where + ": diagnostic lines differ");
        }
    };
    // A projector's matrix target: its reads from leaves and off the stage.
    const auto projector =
        [&](const RigExecBakedProgramImpl::GeomRevision &live) {
            ++count;
            RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
            if (edit) {
                edit(live.moverPath, &fromLeaves.leaves.decl);
            }
            std::vector<std::string> missing;
            RigExecRevisionLeafView view;
            view.decl = &fromLeaves.leaves.decl;
            view.values = &fromLeaves.leaves.values;
            view.missing = &missing;
            RigExecProjectorReads a, b;
            RigExecReadProjectorTargetFromLeaves(live.op, live.binding, view,
                                                 &a);
            RigExecReadProjectorTarget(live.moverPrim, live.op, live.binding,
                                       &local, time, &b);
            const std::string &where = live.moverPathText;
            for (const std::string &name : missing) {
                report.push_back(where + ": undeclared read " + name);
            }
            if (a.rayOrigin != b.rayOrigin ||
                a.rayDirection != b.rayDirection || a.rayUp != b.rayUp ||
                a.shaderOffset != b.shaderOffset ||
                a.reproject != b.reproject || a.dials != b.dials ||
                a.faceVertexCounts != b.faceVertexCounts ||
                a.faceVertexIndices != b.faceVertexIndices) {
                report.push_back(where + ": projector reads differ");
            }
        };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            shadow(revision, chain.lastBase.cdata(), chain.lastBase.size());
        }
        if (!chain.haveResult) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (!derived.matrixTarget && derived.haveBase) {
                shadow(derived.revision, chain.result.cdata(),
                       chain.result.size());
            }
            if (derived.matrixTarget) {
                projector(derived.revision);
            }
        }
    }
    B.resolvedInputs = saved;
    if (compared) {
        *compared = count;
    }
    return report;
}

size_t
RigExecBakedVerifyRangeChains(RigExecBakedProgramImpl *program)
{
    // An independent judge, never a fallback: each range-pipelined revision
    // of a chain that read a base, run whole by the shared kernel over the
    // points entering it, with no caches so nothing the program holds moves.
    // Its ranges hold that answer whether or not they ran this run.
    RigExecBakedProgramImpl &B = *program;
    size_t mismatches = 0;
    std::vector<GfVec3f> whole, enteringGather, publishedGather;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            // A revision a cycle set aside passes the base through: no
            // range result to judge.
            if (!revision.rangeRole || revision.rangeSetAside) {
                continue;
            }
            const GfVec3f *entering = nullptr;
            size_t count = 0;
            RigExecBakedVersionPoints(chain, r, &enteringGather, &entering,
                                      &count);
            whole.assign(entering, entering + count);
            const bool decided = RigExecBakedRevisionApplies(revision);
            bool applied = false;
            if (revision.op == RigExecRevisionOp::Skin) {
                // The packet carries identities for a skin's influences, so
                // the judge skins against the table the fold wrote, then
                // blends the envelope RevisionStatic resolved; the decision
                // is the revision's own.
                if (decided) {
                    const auto view = eval::SkinTransformsOf<BakedSkinMath>(
                        revision.influences.data(), revision.influences.size());
                    applied =
                        eval::ApplySkinKernelWithTransforms<BakedSkinMath>(
                            BakedSkinPacket(revision.parameters), view, &whole,
                            B.useSimd, !revision.parameters.skinTopology,
                            RigExecGeometryRanges());
                    if (applied && !revision.fullStrength) {
                        applied = revision.envelopeOk &&
                                  revision.envelope.size() == count;
                        if (applied) {
                            eval::BlendEnvelopeAll(
                                entering, revision.envelope.data(), count,
                                whole.data(), RigExecGeometryRanges());
                        }
                    }
                }
            } else {
                auto parameters = revision.parameters;
                if (revision.blendChunked) {
                    auto channels = revision.blendSamples;
                    for (size_t c = 0; c < channels.size(); ++c) {
                        const auto &bound = revision.blendChannels[c];
                        const auto held = RigExecBakedResolvePathLeaf(
                            B, revision.leaves, size_t(bound.weightHeldLeaf));
                        if (bound.poseWeight >= 0 &&
                            !(held.IsHolding<bool>() &&
                              held.UncheckedGet<bool>())) {
                            channels[c].weight =
                                B.poseWeights[size_t(bound.poseWeight)];
                        } else {
                            const auto value = RigExecBakedResolvePathLeaf(
                                B, revision.leaves, size_t(bound.weightLeaf));
                            channels[c].weight =
                                value.IsHolding<float>()
                                    ? value.UncheckedGet<float>()
                                    : 0.0f;
                        }
                    }
                    RigExecGeometryBlendDeltas(channels, entering, count,
                                               &parameters.blendDeltas);
                }
                applied = parameters.valid && revision.status.AllowsApply() &&
                          RigExecRunRevisionKernel(revision.op, parameters,
                                                   &whole, B.useSimd, nullptr,
                                                   nullptr, &revision.external);
            }
            if (!applied) {
                whole.assign(entering, entering + count);
            }
            // The version the revision left, gathered from its groups: the
            // written ones and, through the gates, the entering ones.
            const GfVec3f *published = nullptr;
            size_t publishedCount = 0;
            RigExecBakedVersionPoints(chain, r + 1, &publishedGather,
                                      &published, &publishedCount);
            const bool same = applied == decided &&
                              revision.rangeRefusals == 0 &&
                              RigExecBakedSamePoints(whole.data(), whole.size(),
                                                     published, publishedCount);
            if (!same) {
                ++mismatches;
                TF_VERIFY(false,
                          "%s: the range-pipelined revision's ranges "
                          "disagree with the revision run whole",
                          revision.moverPathText.c_str());
            }
        }
    }
    // Gate and partition holes the bodies counted this run, taken so the
    // next run's judge counts only its own.
    const uint64_t holes =
        B.gateViolations.exchange(0, std::memory_order_relaxed);
    if (holes) {
        mismatches += size_t(holes);
        TF_VERIFY(false,
                  "%llu gated or Range-skin group step(s) met a pin "
                  "hole",
                  static_cast<unsigned long long>(holes));
    }
    B.rangeVerifyMismatches += mismatches;
    return mismatches;
}

void
RigExecBakedResetSetAsideGeometryValue(RigExecBakedProgramImpl *program,
                                       RigExecBakedSlotDomain domain,
                                       uint32_t slot)
{
    auto &B = *program;
    if (domain == RigExecBakedSlotDomain::ChainBase ||
        domain == RigExecBakedSlotDomain::ChainPoints) {
        if (slot >= B.chains.size())
            return;
        auto &chain = B.chains[slot];
        // An excluded writer never runs, so clearing is the only way the
        // points move: the content version moves with them.
        if (domain == RigExecBakedSlotDomain::ChainBase) {
            chain.haveBase = false;
            if (!chain.lastBase.empty())
                ++chain.baseVersion;
            chain.lastBase.clear();
            // The base groups go with it; their versions never go down.
            for (auto &group : chain.baseGroups) {
                RigExecResetGroup(&group);
                group.published = RigExecPointsRef<GfVec3f>();
            }
            chain.baseOwner.reset();
        } else {
            chain.haveResult = false;
            if (!chain.result.empty())
                ++chain.resultVersion;
            chain.result.clear();
            chain.resultIds.clear();
        }
        return;
    }
    if (domain == RigExecBakedSlotDomain::DerivedOut) {
        if (slot >= B.derivedIndex.size())
            return;
        const auto [chain, part] = B.derivedIndex[slot];
        auto &derived = B.chains[size_t(chain)].derived[size_t(part)];
        derived.haveResult = false;
        derived.haveMatrix = false;
        if (!derived.result.empty())
            ++derived.resultVersion;
        derived.result.clear();
        derived.revision.output.clear();
        derived.revision.ran = false;
        return;
    }
    if (domain == RigExecBakedSlotDomain::SkinTopology &&
        slot >= B.revisionIndex.size()) {
        const size_t derivedSlot = slot - B.revisionIndex.size();
        if (derivedSlot >= B.derivedIndex.size())
            return;
        const auto [chain, part] = B.derivedIndex[derivedSlot];
        auto &revision = B.chains[size_t(chain)].derived[size_t(part)].revision;
        revision.layoutHandle.reset();
        revision.topology.reset();
        revision.topologyResolved = false;
        revision.layoutUsable = false;
        return;
    }
    if (domain == RigExecBakedSlotDomain::RevisionOut) {
        for (size_t revisionIndex = 0; revisionIndex < B.revisionIndex.size();
             ++revisionIndex) {
            const int first = B.revisionChunkBase[revisionIndex];
            const int count = B.revisionChunkCount[revisionIndex];
            if (slot < uint32_t(first) || slot >= uint32_t(first + count))
                continue;
            const auto [chain, part] = B.revisionIndex[revisionIndex];
            const auto &owner = B.chains[size_t(chain)];
            auto &revision = B.chains[size_t(chain)].revisions[size_t(part)];
            const size_t at = size_t(slot - uint32_t(first));
            if (revision.rangeRole) {
                // A set-aside group makes the revision pass the base through,
                // as an excluded whole fuse does: every reader of a version
                // past it resolves to the base groups (RigExecBakedGroupAt),
                // never to this slot. The slot itself holds the base group,
                // refused; its join publishes the base ids and
                // ChainInputs leaves the source alone.
                if (at < revision.groups.size()) {
                    revision.groups[at].ok = false;
                    PassSetAsideBaseGroup(owner, at, &revision.groups[at]);
                }
                if (at < revision.chunks.size())
                    revision.chunks[at].ok = false;
                revision.rangeSetAside = true;
                revision.currentSource = -1;
                return;
            }
            if (owner.groupBounds.size() >= 2 && at >= revision.chunks.size()) {
                // A Whole revision's published group: its fuse is set aside,
                // and passes the base groups through as a set-aside join does.
                const size_t g = at - revision.chunks.size();
                if (g < revision.groups.size()) {
                    PassSetAsideBaseGroup(owner, g, &revision.groups[g]);
                }
                revision.rangeSetAside = true;
                revision.currentSource = -1;
                return;
            }
            revision.stagingOutput.clear();
            revision.stagingFresh = false;
            // The excluded writer owns the full retained staging output;
            // invalidate every chunk opinion along with its cleared bytes,
            // and a Whole revision's speculative group answers with them.
            for (auto &chunk : revision.chunks)
                chunk.ok = false;
            if (owner.groupBounds.size() >= 2) {
                for (auto &group : revision.groups)
                    group.ok = false;
            }
            return;
        }
        return;
    }
    if (slot >= B.revisionIndex.size())
        return;
    const auto [chain, part] = B.revisionIndex[slot];
    auto &revision = B.chains[size_t(chain)].revisions[size_t(part)];
    switch (domain) {
    case RigExecBakedSlotDomain::RevisionPacket:
        revision.parameters.valid = false;
        revision.acceptance = RigExecRevisionAcceptance::Refuses;
        break;
    case RigExecBakedSlotDomain::RevisionTransforms:
        revision.influencesValid = false;
        break;
    case RigExecBakedSlotDomain::RevisionOut:
        revision.output.clear();
        break;
    case RigExecBakedSlotDomain::RevisionDone:
    case RigExecBakedSlotDomain::ChainDirty: {
        revision.ran = false;
        revision.executed = false;
        revision.resultStatus = _tokens->operationCycle;
        revision.currentSource = -1;
        const auto &owner = B.chains[size_t(chain)];
        if (owner.groupBounds.size() >= 2) {
            // A join or Whole fuse of a chain cut into groups, set aside:
            // readers past it resolve to the base groups, and its version
            // follows their ids, which is the base it passes through.
            revision.rangeSetAside = true;
            PassSetAsideBaseGroups(owner, &revision);
            break;
        }
        // A range-pipelined revision set aside passes the base through
        // exactly as a whole one does; its ranges' buffer is then read by
        // no one, and ChainInputs leaves the source alone.
        if (revision.rangeRole)
            revision.rangeSetAside = true;
        // An excluded fuse never runs, so the version follows the base it
        // passes through, against the copy of the base it last carried.
        PassBaseThrough(owner.lastBase, &revision);
        break;
    }
    case RigExecBakedSlotDomain::SkinTopology:
        revision.layoutHandle.reset();
        revision.topology.reset();
        revision.topologyResolved = false;
        revision.layoutUsable = false;
        break;
    default:
        break;
    }
}
} // namespace rigExec
