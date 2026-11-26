#include "rigExec/internal/bakedProgramImpl.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/opFields.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/compile/rigDescData.h"
#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/propertyLookups.h"
#include "rigExecStdTypes/array.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <tuple>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

using D = RigExecBakedSlotDomain;
using K = RigExecBakedStepKind;

std::function<bool(const RigExecBakedProgramImpl &, RigExecBakedOpInjection *)>
    gOpInjector;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// What the op steps read from the baked program and store in it.
struct BakedOpHost {
    RigExecBakedProgramImpl &B;

    const void *Matrix(eval::OpHostSource source, uint32_t slot) const
    {
        const std::vector<GfMatrix4d> &matrices =
            source == eval::OpHostSource::FinalMatrix ? B.finalMatrix
                                                      : B.baseMatrix;
        return slot < matrices.size() ? matrices[slot].data() : nullptr;
    }
    const void *PresentMatrix(const eval::OpLookup &lookup) const
    {
        const bool record =
            lookup.source == uint8_t(eval::OpLookupSource::FrameRecord);
        const std::vector<GfMatrix4d> &matrices =
            record ? B.frameMatrix : B.deltaValues;
        const std::vector<char> &present =
            record ? B.frameMatrixValid : B.deltaPresent;
        return lookup.object < matrices.size() &&
                       lookup.object < present.size() && present[lookup.object]
                   ? matrices[lookup.object].data()
                   : nullptr;
    }
    const void *ChainGroupPoints(uint32_t chain, uint32_t g) const
    {
        if (chain >= B.chains.size())
            return nullptr;
        const RigExecBakedProgramImpl::GeomChain &c = B.chains[chain];
        return RigExecBakedGroupAt(c, c.revisions.size(), g).data;
    }
    const void *ChainVersionGroupPoints(const eval::OpLookup &lookup,
                                        uint32_t g) const
    {
        if (lookup.object >= B.chains.size() || lookup.part < 0 ||
            size_t(lookup.part) > B.chains[lookup.object].revisions.size())
            return nullptr;
        return RigExecBakedGroupAt(B.chains[lookup.object], size_t(lookup.part),
                                   g)
            .data;
    }
    bool LookUpProperty(uint32_t index, const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *value,
                        std::string *error) const
    {
        return RigExecBakedLookUpProperty(B, lookup, index, type, value, error);
    }
    bool StorePropertyVersion(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RigExecBakedStorePropertyVersion(B, output, value, error);
    }
    bool LookUpPoseVersion(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RigExecBakedLookUpPoseVersion(B, lookup, type, value, error);
    }
    bool StoreVolumePlacement(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RigExecBakedStoreVolumePlacement(B, output, value, error);
    }
    bool LookUpWeight(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RigExecBakedLookUpWeight(B, lookup, type, value, error);
    }
    bool StoreWeightPacket(const eval::OpHostOutput &output, const void *value,
                           std::string *error) const
    {
        return RigExecBakedStoreWeightPacket(B, output, value, error);
    }
    bool LookUpChainVersion(uint32_t index, const eval::OpLookup &lookup,
                            const RigExecTypeDesc &type, void *value,
                            std::string *error) const
    {
        return RigExecBakedLookUpChainVersion(B, index, lookup, type, value,
                                              error);
    }
    bool LookUpWeightField(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RigExecBakedLookUpWeightField(B, lookup, type, value, error);
    }
    bool StoreWeightField(const eval::OpHostOutput &output, const void *value,
                          std::string *error) const
    {
        return RigExecBakedStoreWeightField(B, output, value, error);
    }
    bool LookUpSpace(uint32_t, const eval::OpLookup &lookup,
                     const RigExecTypeDesc &type, void *value,
                     std::string *error) const
    {
        return RigExecBakedLookUpSpace(B, lookup, type, value, error);
    }
    bool StoreSpace(const eval::OpHostOutput &output, const void *value,
                    std::string *error) const
    {
        return RigExecBakedStoreSpace(B, output, value, error);
    }
    bool StoreGroup(const eval::OpHostOutput &output, uint32_t g,
                    const void *elements, size_t count,
                    std::string *error) const
    {
        return RigExecBakedStoreChainGroup(B, output, g, elements, count,
                                           error);
    }
};

template <class T>
void
Bytes(const T *data, size_t count, std::vector<uint8_t> *out)
{
    const auto *bytes = reinterpret_cast<const uint8_t *>(data);
    out->assign(bytes, bytes + count * sizeof(T));
}

/// \p value's bytes as an input of standard type \p type reads them; false
/// when \p value holds another type.
bool
ConstantBytes(const VtValue &value, const RigExecTypeDesc &type,
              std::vector<uint8_t> *out)
{
    const std::string name =
        std::string(type.nameSpace) == "std" ? type.name : "";
    if (name == "Float" && value.IsHolding<float>()) {
        Bytes(&value.UncheckedGet<float>(), 1, out);
    } else if (name == "Double" && value.IsHolding<double>()) {
        Bytes(&value.UncheckedGet<double>(), 1, out);
    } else if (name == "Int" && value.IsHolding<int>()) {
        const int32_t v = value.UncheckedGet<int>();
        Bytes(&v, 1, out);
    } else if (name == "Bool" && value.IsHolding<bool>()) {
        const uint8_t v = value.UncheckedGet<bool>() ? 1 : 0;
        Bytes(&v, 1, out);
    } else if (name == "Mat4d" && value.IsHolding<GfMatrix4d>()) {
        Bytes(value.UncheckedGet<GfMatrix4d>().data(), 16, out);
    } else if (name == "IntArray" && value.IsHolding<VtIntArray>()) {
        const VtIntArray &v = value.UncheckedGet<VtIntArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "FloatArray" && value.IsHolding<VtFloatArray>()) {
        const VtFloatArray &v = value.UncheckedGet<VtFloatArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "DoubleArray" && value.IsHolding<VtDoubleArray>()) {
        const VtDoubleArray &v = value.UncheckedGet<VtDoubleArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "Vec3fArray" && value.IsHolding<VtVec3fArray>()) {
        const VtVec3fArray &v = value.UncheckedGet<VtVec3fArray>();
        Bytes(v.cdata(), v.size(), out);
    } else {
        return false;
    }
    return true;
}

/// Which op step reads: a node's, a chain's head, or a range step over
/// one vertex group or, for a member that runs whole, all of them.
struct OpReader {
    uint32_t chain = kRigExecInvalidIndex;
    uint32_t segment = kRigExecInvalidIndex;
    uint32_t group = kRigExecInvalidIndex;
    bool head = false;
    bool whole = false;
};

/// What port \p p of node \p n reads in \p reader's step.
void
DeclarePortReads(const RigExecBakedProgramImpl &B,
                 const RigExecBakedOpProgram &op, uint32_t n, uint32_t p,
                 const OpReader &reader,
                 std::vector<RigExecBakedSlotRange> *reads)
{
    const eval::OpStepLayout &layout = op.layout;
    const uint32_t chain = reader.chain, segment = reader.segment,
                   group = reader.group;
    const bool whole = reader.whole;
    const RigExecNodeDesc &d = layout.nodes[n];
    const uint32_t flags = d.op->ports[p].flags;
    const auto value = [&](uint32_t v, bool listed) {
        if (v == kRigExecInvalidIndex)
            return;
        const uint32_t producer = layout.producerOf[v];
        if (producer == kRigExecInvalidIndex)
            return;
        const uint32_t from = layout.chainOf[producer];
        if (from == kRigExecInvalidIndex) {
            reads->push_back(RigExecBakedOne(D::OpValue, int(v)));
            return;
        }
        const uint32_t base = layout.groupBase[v];
        if (from == chain) {
            if (reader.head ||
                layout.chainDescs[chain].segments[layout.memberOf[producer]] ==
                    segment)
                return;
            if (!listed && !whole && (flags & kRigExecPortPointwise)) {
                reads->push_back(
                    RigExecBakedOne(D::OpGroupValue, int(base + group)));
                return;
            }
            if (!listed && !whole && (flags & kRigExecPortNeighbourhood)) {
                for (const uint32_t h :
                     op.halo[chain][layout.memberOf[n]][group])
                    reads->push_back(
                        RigExecBakedOne(D::OpGroupValue, int(base + h)));
                return;
            }
        }
        reads->push_back(
            RigExecBakedRange(D::OpGroupValue, int(base),
                              int(base + layout.chains[from].Groups())));
    };
    value(d.values[p], false);
    for (const uint32_t element : d.lists[p])
        value(element, true);
    const auto matrix = [&](const eval::OpLookup &lookup) {
        if (lookup.source == uint8_t(eval::OpLookupSource::Matrix))
            reads->push_back(
                RigExecBakedOne(lookup.part ? D::BaseMatrix : D::FinalMatrix,
                                int(lookup.object)));
        else if (lookup.source == uint8_t(eval::OpLookupSource::FrameRecord))
            reads->push_back(
                RigExecBakedOne(D::FrameMatrix, int(lookup.object)));
        else if (lookup.source ==
                 uint8_t(eval::OpLookupSource::ConstraintDelta))
            reads->push_back(
                RigExecBakedOne(D::ConstraintDelta, int(lookup.object)));
    };
    const auto matrices = [&](const eval::OpLookup &lookup) {
        matrix(lookup);
        if (lookup.source == uint8_t(eval::OpLookupSource::Selected))
            for (int32_t e = 0; e < lookup.part; ++e)
                matrix(layout.lookups[lookup.object + uint32_t(e)]);
    };
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const eval::OpHostInput &input = layout.hostInputs[i];
        if (input.node != n || input.port != p)
            continue;
        if (input.source == eval::OpHostSource::FinalMatrix) {
            reads->push_back(
                RigExecBakedOne(D::FinalMatrix, int(input.object)));
        } else if (input.source == eval::OpHostSource::BaseMatrix) {
            reads->push_back(RigExecBakedOne(D::BaseMatrix, int(input.object)));
        } else if (layout.grouped[i]) {
            if (reader.head || whole)
                continue;
            const bool latest = input.source == eval::OpHostSource::ChainPoints;
            const uint32_t from =
                latest ? input.object : layout.lookups[input.object].object;
            const size_t version =
                latest ? B.chains[from].revisions.size()
                       : size_t(layout.lookups[input.object].part);
            RigExecBakedDeclareGroupRead(B, int(from), version, group, reads);
            if (layout.grouped[i] == 2)
                for (const uint32_t h :
                     op.halo[chain][layout.memberOf[n]][group])
                    if (h != group)
                        RigExecBakedDeclareGroupRead(B, int(from), version, h,
                                                     reads);
        } else if (input.source == eval::OpHostSource::Lookup &&
                   (reader.head || chain == kRigExecInvalidIndex)) {
            const eval::OpLookup &lookup = layout.lookups[input.object];
            matrices(lookup);
            if (lookup.source == uint8_t(eval::OpLookupSource::List))
                for (int32_t e = 0; e < lookup.part; ++e)
                    matrices(layout.lookups[lookup.object + uint32_t(e)]);
        }
    }
}

void
DeclareNodeReads(const RigExecBakedProgramImpl &B,
                 const RigExecBakedOpProgram &op, uint32_t n,
                 const OpReader &reader,
                 std::vector<RigExecBakedSlotRange> *reads)
{
    const RigExecNodeDesc &d = op.layout.nodes[n];
    for (uint32_t p = 0; p < d.values.size(); ++p)
        if (d.op->ports[p].flags & kRigExecPortInput)
            DeclarePortReads(B, op, n, p, reader, reads);
}

/// Per chain, member reading an earlier segment through a neighbourhood
/// input, and vertex group: the groups that group reads, from the
/// member's reads after a head over the values Build has; every group when
/// the head cannot run before the first frame.
void
FindHaloGroups(RigExecBakedOpProgram *op, RigExecRunState &run)
{
    const eval::OpStepLayout &layout = op->layout;
    op->halo.assign(layout.chains.size(),
                    std::vector<std::vector<std::vector<uint32_t>>>());
    for (uint32_t c = 0; c < layout.chains.size(); ++c) {
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const std::vector<int> &bounds = layout.chains[c].bounds;
        const uint32_t groups = layout.chains[c].Groups();
        std::string why;
        const bool known = run.RunChainHead(c, bounds.data(), groups, &why);
        op->halo[c].assign(desc.members.size(),
                           std::vector<std::vector<uint32_t>>());
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            bool halo = false;
            for (const RigExecChainDesc::Feed &feed : desc.feeds[m])
                halo = halo || feed.neighbourhood;
            for (size_t i = 0; i < layout.hostInputs.size(); ++i)
                halo = halo || (layout.grouped[i] == 2 &&
                                layout.hostInputs[i].node == desc.members[m]);
            if (!halo)
                continue;
            op->halo[c][m].assign(groups, std::vector<uint32_t>());
            for (uint32_t g = 0; g < groups; ++g) {
                size_t lo = 0, hi = SIZE_MAX;
                if (known)
                    run.ChainReads(c, m, g, &lo, &hi);
                for (uint32_t h = 0; h < groups; ++h)
                    if (size_t(bounds[h]) < hi && size_t(bounds[h + 1]) > lo)
                        op->halo[c][m][g].push_back(h);
            }
        }
    }
}

/// Removes the RevisionChunk steps of the revisions \p stored names, whose
/// groups op chains store instead, keeping B's step indices current.
void
DropRevisionChunks(RigExecBakedProgramImpl *B, std::vector<int> stored)
{
    std::sort(stored.begin(), stored.end());
    const auto dropped = [&](const RigExecBakedStep &step) {
        return step.kind == K::RevisionChunk &&
               std::binary_search(stored.begin(), stored.end(), step.object);
    };
    std::vector<int> moved(B->steps.size(), -1);
    int kept = 0;
    for (size_t i = 0; i < B->steps.size(); ++i)
        if (!dropped(B->steps[i]))
            moved[i] = kept++;
    B->steps.erase(std::remove_if(B->steps.begin(), B->steps.end(), dropped),
                   B->steps.end());
    for (int &step : B->revisionFuseStep)
        if (step >= 0)
            step = moved[size_t(step)];
}

RigExecBakedStep &
AddOpStep(RigExecBakedProgramImpl *B, K kind, int object, int part)
{
    B->steps.emplace_back();
    RigExecBakedStep &step = B->steps.back();
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 1;
    return step;
}

void
EmitOpSteps(RigExecBakedProgramImpl *B, const RigExecBakedOpProgram &op)
{
    const eval::OpStepLayout &layout = op.layout;
    const auto outputs = [&](uint32_t n, auto &&visit) {
        const RigExecNodeDesc &d = layout.nodes[n];
        for (size_t p = 0; p < d.values.size(); ++p)
            if ((d.op->ports[p].flags & kRigExecPortOutput) &&
                d.values[p] != kRigExecInvalidIndex)
                visit(d.values[p]);
    };
    for (RigExecBakedStep &step : B->steps) {
        const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(*B, step);
        if (!origin || origin->family != RigExecBakedOpFamily::WeightField)
            continue;
        const uint32_t n = uint32_t(step.object);
        DeclareNodeReads(*B, op, n, OpReader(), &step.reads);
        outputs(n, [&](uint32_t v) {
            step.writes.push_back(RigExecBakedOne(D::OpValue, int(v)));
        });
    }
    for (uint32_t n = 0; n < layout.nodes.size(); ++n) {
        if (layout.chainOf[n] != kRigExecInvalidIndex ||
            n < B->opNodeOrigins.size())
            continue;
        RigExecBakedStep &step = AddOpStep(B, K::OpNode, int(n), -1);
        DeclareNodeReads(*B, op, n, OpReader(), &step.reads);
        outputs(n, [&](uint32_t v) {
            step.writes.push_back(RigExecBakedOne(D::OpValue, int(v)));
        });
    }
    for (uint32_t c = 0; c < layout.chains.size(); ++c) {
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const uint32_t groups = layout.chains[c].Groups();
        {
            RigExecBakedStep &head = AddOpStep(B, K::OpChainHead, int(c), -1);
            OpReader reader;
            reader.chain = c;
            reader.head = true;
            for (const uint32_t n : desc.members)
                DeclareNodeReads(*B, op, n, reader, &head.reads);
            head.writes.push_back(
                RigExecBakedOne(D::OpValue, int(layout.PlanSlot(c))));
        }
        for (uint32_t s = 0; s < desc.members.size(); ++s) {
            if (desc.segments[s] != s)
                continue;
            const bool whole = desc.whole[s] != 0;
            for (uint32_t g = 0; g < (whole ? 1 : groups); ++g) {
                if (!eval::OpSegmentWrites(layout.chains[c], desc, s, g))
                    continue;
                RigExecBakedStep &step =
                    AddOpStep(B, K::OpChainRange, int(c), int(s * groups + g));
                step.reads.push_back(
                    RigExecBakedOne(D::OpValue, int(layout.PlanSlot(c))));
                for (uint32_t m = s;
                     m < desc.members.size() && desc.segments[m] == s; ++m) {
                    const uint32_t n = desc.members[m];
                    OpReader reader;
                    reader.chain = c;
                    reader.segment = s;
                    reader.group = g;
                    reader.whole = whole;
                    DeclareNodeReads(*B, op, n, reader, &step.reads);
                    outputs(n, [&](uint32_t v) {
                        const uint32_t base = layout.groupBase[v];
                        step.writes.push_back(
                            whole
                                ? RigExecBakedRange(D::OpGroupValue, int(base),
                                                    int(base + groups))
                                : RigExecBakedOne(D::OpGroupValue,
                                                  int(base + g)));
                    });
                    for (const uint32_t o : layout.hostOutputsOf[n]) {
                        const eval::OpHostOutput &output =
                            layout.hostOutputs[o];
                        for (uint32_t h = whole ? 0 : g;
                             h < (whole ? groups : g + 1); ++h) {
                            const int slot = RigExecBakedGroupSlot(
                                *B, int(output.object), h);
                            if (slot >= 0)
                                step.writes.push_back(
                                    RigExecBakedOne(D::RevisionOut, slot));
                        }
                    }
                }
                std::sort(step.reads.begin(), step.reads.end(),
                          [](const RigExecBakedSlotRange &a,
                             const RigExecBakedSlotRange &b) {
                              return std::make_tuple(a.domain, a.begin, a.end) <
                                     std::make_tuple(b.domain, b.begin, b.end);
                          });
            }
        }
    }
}

} // namespace

const RigExecBakedOpOrigin *
RigExecBakedOriginOf(const RigExecBakedProgramImpl &B,
                     const RigExecBakedStep &step)
{
    if (step.kind != K::OpNode || step.object < 0 ||
        size_t(step.object) >= B.opNodeOrigins.size())
        return nullptr;
    return &B.opNodeOrigins[size_t(step.object)];
}

RigExecBakedStepIdentity
RigExecBakedIdentityOf(const RigExecBakedProgramImpl &B,
                       const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    if (origin && origin->kind != K::OpNode)
        return {origin->kind, origin->object, origin->part};
    return {step.kind, step.object, step.part};
}

void
RigExecBakedProgramTesting::SetOpInjector(
    std::function<bool(const RigExecBakedProgramImpl &,
                       RigExecBakedOpInjection *)>
        injector)
{
    gOpInjector = std::move(injector);
}

bool
RigExecBakedBuildOpSteps(RigExecBakedProgramImpl *program, std::string *error)
{
    RigExecBakedProgramImpl &B = *program;
    B.opProgram.reset();
    B.opState.reset();
    auto op = std::make_shared<RigExecBakedOpProgram>();
    eval::OpStepLayout &layout = op->layout;
    if (!B.opNodeOrigins.empty() && !RigExecEnsureBuiltinRegistry(error))
        return false;
    RigExecRigDescBuilder builder;
    if (!RigExecBakedAddPropertyOps(B, &builder, &layout, error) ||
        !RigExecBakedAddWeightFieldOps(B, &builder, &layout, false, error) ||
        !RigExecBakedAddSpaceOps(B, &builder, &layout, error))
        return false;
    RigExecBakedAddVolumePlacementOps(B, &builder, &layout);
    if (!RigExecBakedAddWeightPacketOps(B, &builder, &layout, error) ||
        !RigExecBakedAddWeightFieldOps(B, &builder, &layout, true, error))
        return false;
    RigExecBakedOpInjection injection;
    injection.builder = &builder;
    const bool injected = gOpInjector && gOpInjector(B, &injection);
    if (B.opNodeOrigins.empty() && !injected)
        return true;
    std::string why;
    const std::shared_ptr<const RigExecRigDesc> desc = builder.Build(&why);
    if (!desc)
        return Fail(error, why);
    RigExecCompileOptions options;
    options.hostOrder = true;
    options.useSimd = B.useSimd;
    layout.program = RigExecCompile(*desc, options, &why);
    if (!layout.program)
        return Fail(error, why);
    const uint32_t lookupBase = uint32_t(layout.lookups.size());
    for (eval::OpHostInput input : injection.hostInputs) {
        if (input.source == eval::OpHostSource::Lookup)
            input.object += lookupBase;
        layout.hostInputs.push_back(input);
    }
    for (eval::OpLookup lookup : injection.lookups) {
        if (lookup.source == uint8_t(eval::OpLookupSource::List) ||
            lookup.source == uint8_t(eval::OpLookupSource::Selected))
            lookup.object += lookupBase;
        layout.lookups.push_back(lookup);
    }
    layout.hostOutputs.insert(layout.hostOutputs.end(),
                              injection.hostOutputs.begin(),
                              injection.hostOutputs.end());
    const size_t chains = RigExecProgramChainCount(*layout.program);
    layout.chains.assign(chains, eval::OpChainLayout());
    std::vector<uint32_t> chainOfNode(RigExecProgramNodeCount(*layout.program),
                                      kRigExecInvalidIndex);
    std::vector<uint32_t> memberOfNode(chainOfNode.size(),
                                       kRigExecInvalidIndex);
    std::vector<size_t> members(chains, 0);
    for (size_t c = 0; c < chains; ++c) {
        RigExecChainDesc desc;
        RigExecProgramChainAt(*layout.program, c, &desc);
        members[c] = desc.members.size();
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            chainOfNode[desc.members[m]] = uint32_t(c);
            memberOfNode[desc.members[m]] = m;
        }
    }
    std::vector<char> placed(chains, 0);
    for (const eval::OpHostInput &input : layout.hostInputs) {
        const uint32_t c = input.node < chainOfNode.size()
                               ? chainOfNode[input.node]
                               : kRigExecInvalidIndex;
        uint32_t from = input.object;
        if (input.source == eval::OpHostSource::Lookup) {
            RigExecNodeDesc node;
            if (c == kRigExecInvalidIndex ||
                input.object >= layout.lookups.size() ||
                layout.lookups[input.object].source !=
                    uint8_t(eval::OpLookupSource::ChainVersion) ||
                !RigExecProgramNodeAt(*layout.program, input.node, &node) ||
                input.port >= node.values.size() ||
                !(node.op->ports[input.port].flags &
                  (kRigExecPortPointwise | kRigExecPortNeighbourhood)))
                continue;
            from = layout.lookups[input.object].object;
        } else if (input.source != eval::OpHostSource::ChainPoints) {
            continue;
        }
        if (c == kRigExecInvalidIndex)
            return Fail(error, "node " + std::to_string(input.node) +
                                   " reads chain points outside a range chain");
        if (from >= B.chains.size() || B.chains[from].groupBounds.size() < 2)
            return Fail(error, "point chain " + std::to_string(from) +
                                   " is not cut into vertex groups");
        if (placed[c] && layout.chains[c].pointChain != from)
            return Fail(error, "range chain " + std::to_string(c) +
                                   " reads two point chains");
        layout.chains[c].pointChain = from;
        layout.chains[c].bounds = B.chains[from].groupBounds;
        placed[c] = 1;
    }
    for (size_t c = 0; c < chains; ++c)
        if (!placed[c])
            return Fail(error, "range chain " + std::to_string(c) +
                                   " reads no point chain");
    std::vector<int> stored;
    for (const eval::OpHostOutput &output : layout.hostOutputs) {
        if (output.target != uint8_t(eval::OpOutputTarget::ChainGroup) ||
            output.node >= chainOfNode.size() ||
            chainOfNode[output.node] == kRigExecInvalidIndex)
            continue;
        eval::OpChainLayout &chain = layout.chains[chainOfNode[output.node]];
        const size_t groups = chain.Groups();
        if (output.object >= B.revisionIndex.size() ||
            uint32_t(B.revisionIndex[output.object].first) != chain.pointChain)
            return Fail(error, "node " + std::to_string(output.node) +
                                   " stores into revision " +
                                   std::to_string(output.object) +
                                   ", which is not of its point chain");
        const auto &[pointChain, r] = B.revisionIndex[output.object];
        const std::vector<char> &written =
            B.chains[size_t(pointChain)].revisions[size_t(r)].groupWritten;
        if (written.size() != groups)
            return Fail(error, "revision " + std::to_string(output.object) +
                                   " is cut into " +
                                   std::to_string(written.size()) +
                                   " groups, its range chain into " +
                                   std::to_string(groups));
        if (chain.writes.empty())
            chain.writes.assign(members[chainOfNode[output.node]] * groups, 1);
        const size_t row = size_t(memberOfNode[output.node]) * groups;
        for (size_t g = 0; g < groups; ++g)
            chain.writes[row + g] = chain.writes[row + g] && written[g];
        stored.push_back(int(output.object));
    }
    for (eval::OpChainLayout &chain : layout.chains)
        if (std::find(chain.writes.begin(), chain.writes.end(), 0) ==
            chain.writes.end())
            chain.writes.clear();
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const RigExecBakedOpInjection::Constant &constant :
         injection.constants) {
        RigExecNodeDesc node;
        RigExecProgramNodeAt(*layout.program, constant.node, &node);
        eval::OpConstant bytes;
        bytes.node = constant.node;
        bytes.port = constant.port;
        if (!node.op || constant.port >= node.values.size() ||
            !ConstantBytes(
                constant.value,
                *registry.TypeAt(node.op->ports[constant.port].type->Index())
                     .desc,
                &bytes.bytes))
            return Fail(error, "the value fixed for input " +
                                   std::to_string(constant.port) + " of node " +
                                   std::to_string(constant.node) +
                                   " is not of its type");
        layout.constants.push_back(std::move(bytes));
    }
    if (!eval::LayOutOpSteps(&layout, error))
        return false;
    B.propertyKeyHold.assign(layout.lookups.size(), VtArray<GfVec2f>());
    B.propertyPointHold.assign(layout.lookups.size(), VtVec3fArray());
    B.chainVersionGather.assign(layout.lookups.size(), {});
    op->outputs = injection.outputs;
    for (const RigExecDescNode &node : desc->Data().nodes) {
        const RigExecOpDesc &desc = *registry.OpAt(node.op).desc;
        std::vector<uint8_t> bytes;
        bool savable = true;
        if (desc.payloadFieldCount)
            RigExecEncodePayloadFields(desc, node.fields, bytes);
        else if (node.hasPayload && desc.payload.encode)
            desc.payload.encode(node.payload.Data(), bytes);
        else
            savable = !node.hasPayload && !desc.payload.size;
        op->payloads.push_back(std::move(bytes));
        op->savable.push_back(savable ? 1 : 0);
    }
    for (const auto &[name, output] : op->outputs) {
        RigExecValueRef ref;
        const RigExecTypeDesc *type =
            RigExecResolveOutput(*layout.program, output, &ref) && ref.type
                ? registry.TypeAt(ref.type->Index()).desc
                : nullptr;
        if (!type || std::string(type->nameSpace) != "std" ||
            std::string(type->name) != "Vec3fArray")
            return Fail(error, "output " + name + " is not a point array");
    }
    auto state = std::make_shared<eval::OpStepState>();
    state->run = RigExecRunState::Create(layout.program, {}, &why);
    if (!state->run)
        return Fail(error, why);
    const BakedOpHost host{B};
    if (!eval::BindOpStepState(layout, host, state.get(), error))
        return false;
    FindHaloGroups(op.get(), *state->run);
    if (!stored.empty())
        DropRevisionChunks(&B, stored);
    EmitOpSteps(&B, *op);
    state->run->ReleaseOwner();
    B.opProgram = std::move(op);
    B.opState = std::move(state);
    return true;
}

void
RigExecBakedRunOpStep(RigExecBakedProgramImpl *program, RigExecBakedStep *step)
{
    RigExecBakedProgramImpl &B = *program;
    step->failed = false;
    step->diagnostics.clear();
    if (!B.opProgram || !B.opState) {
        step->failed = true;
        step->diagnostics.push_back("op nodes: the program holds none");
        return;
    }
    const BakedOpHost host{B};
    eval::OpStepState &state = *B.opState;
    std::string error;
    size_t chain = 0, part = 0;
    const bool property = RigExecBakedPropertyPartOf(B, *step, &chain, &part);
    if (property) {
        step->lines.clear();
        RigExecBakedPropertyInputLines(B, chain, part, &step->lines);
    }
    const eval::OpStepKind kind = eval::OpStepKind(uint8_t(step->kind));
    if (eval::RunOpStep(B.opProgram->layout, host, &state, kind, step->object,
                        step->part, &error))
        return;
    step->failed = true;
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, *step);
    if (property)
        step->diagnostics.push_back(
            B.propertyChains[chain].revisions[part - 1].linePrefix + ": " +
            error);
    else if (origin && origin->family == RigExecBakedOpFamily::WeightPacket)
        step->diagnostics.push_back(
            B.weightProgram[size_t(origin->object)].name + ": " + error);
    else if (origin && origin->family == RigExecBakedOpFamily::WeightField)
        step->diagnostics.push_back(
            B.weightProgram[size_t(B.weightFields[size_t(origin->object)]
                                       .nodeObjects[size_t(origin->part)])]
                .name +
            ": " + error);
    else
        step->diagnostics.push_back(error);
}

bool
RigExecBakedCloneOpState(const RigExecBakedProgramImpl &source,
                         RigExecBakedProgramImpl *clone, std::string *error)
{
    clone->opProgram = source.opProgram;
    clone->opState.reset();
    if (!source.opProgram || !source.opState)
        return true;
    auto state = std::make_shared<eval::OpStepState>();
    state->run = source.opState->run->Clone();
    state->versions = source.opState->versions;
    state->groupVersions = source.opState->groupVersions;
    state->haloReads = source.opState->haloReads;
    const RigExecBakedOpProgram &op = *source.opProgram;
    const BakedOpHost host{*clone};
    if (!eval::BindOpStepState(op.layout, host, state.get(), error))
        return false;
    state->run->ReleaseOwner();
    clone->opState = std::move(state);
    return true;
}

void
RigExecBakedPublishOpOutputs(const RigExecBakedProgramImpl &B,
                             RigExecRigPose *pose)
{
    if (!B.opProgram || !B.opState)
        return;
    for (const auto &[name, output] : B.opProgram->outputs) {
        RigExecValueRef ref;
        if (!RigExecResolveOutput(*B.opProgram->layout.program, output, &ref))
            continue;
        const auto *points =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                B.opState->run->GetValue(ref));
        if (!points)
            continue;
        VtVec3fArray value(points->Size());
        if (points->Size())
            std::memcpy(value.data(), points->Data(),
                        points->Size() * sizeof(GfVec3f));
        pose->opOutputs[name] = VtValue(value);
    }
}

bool
RigExecBakedOpValueVersion(const RigExecBakedProgramImpl &B,
                           RigExecBakedSlotDomain domain, uint32_t slot,
                           uint64_t *version)
{
    if (!B.opState)
        return false;
    const std::vector<uint64_t> &versions =
        domain == D::OpValue ? B.opState->versions : B.opState->groupVersions;
    if (slot >= versions.size())
        return false;
    *version = versions[slot];
    return true;
}

} // namespace rigExec
