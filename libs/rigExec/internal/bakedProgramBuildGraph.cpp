// Building a baked program: the invalidation index, the step graph and the
// program digest.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecOperation, SchemaAttrText::rigExecOperation))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
);

// RIGEXEC_BAKED_PROGRAM_DIGEST: a fingerprint of what Build produced.
// The parts of Build that resolve in parallel promise a program identical to
// a serial Build's, byte for byte -- the same override numbering, the same
// invalidation index, the same bound constants. Nothing in a frame's OUTPUT
// can check that promise: two numberings that differ still pose the rig the
// same, and a path missing from `named` shows up only on the edit that
// should have rebuilt. So the tables are hashed here, table by table, and a
// run with RIGEXEC_ENABLE_PARALLEL_EVAL=0 (or a build from before a change)
// must print the same lines. One line per table rather than one hash, so a
// mismatch names the table it is in.

const bool _kProgramDigestRequested =
    TfGetenvBool("RIGEXEC_BAKED_PROGRAM_DIGEST", false);

bool
_ProgramDigestRequested()
{
    return _kProgramDigestRequested;
}

// FNV-1a over the serialized bytes, plus a count of what was hashed.
struct _DigestTable {
    uint64_t hash = 1469598103934665603ull;
    size_t count = 0;
    RigExecPathText pathText;

    void Bytes(const void *data, size_t size)
    {
        const unsigned char *p = static_cast<const unsigned char *>(data);
        for (size_t i = 0; i < size; ++i) {
            hash ^= p[i];
            hash *= 1099511628211ull;
        }
    }
    void Str(const std::string &s)
    {
        const uint64_t n = s.size();
        Bytes(&n, sizeof(n));
        Bytes(s.data(), s.size());
    }
    void Path(const SdfPath &p)
    {
        Str(pathText(p));
    }
    void Int(int64_t v)
    {
        Bytes(&v, sizeof(v));
    }
    void Double(double v)
    {
        Bytes(&v, sizeof(v));
    }
    void Matrix(const GfMatrix4d &m)
    {
        Bytes(m.GetArray(), sizeof(double) * 16);
    }
    void Value(double v)
    {
        Double(v);
    }
    void Value(bool v)
    {
        Int(v ? 1 : 0);
    }
    void Value(const TfToken &v)
    {
        Str(v.GetString());
    }
    void Value(const GfMatrix4d &v)
    {
        Matrix(v);
    }
    void Value(const GfVec3d &v)
    {
        for (int i = 0; i < 3; ++i)
            Double(v[i]);
    }
    template <class T> void Input(const RigExecBakedInput<T> &input)
    {
        Value(input.constant);
        Int(input.sourceBacked ? 1 : 0);
        if (input.sourceBacked)
            Value(input.sourceFallback);
        Path(input.query.IsValid() ? input.query.GetAttribute().GetPath()
                                   : SdfPath());
        Path(input.resolvedAttr ? input.resolvedAttr.GetPath() : SdfPath());
        Path(input.head ? input.head.GetPath() : SdfPath());
        Int(input.varying ? 1 : 0);
        Int(input.overrideIndex);
    }
    void PathSet(const std::set<SdfPath> &paths)
    {
        for (const SdfPath &p : paths) {
            Path(p);
        }
        count = paths.size();
    }
};

void
_PrintProgramDigest(const RigExecBakedProgramImpl &B)
{
    std::vector<std::pair<const char *, _DigestTable>> tables;
    const auto table = [&tables](const char *name) -> _DigestTable & {
        tables.emplace_back(name, _DigestTable());
        return tables.back().second;
    };
    table("rebuild").PathSet(B.rebuild);
    table("liveInputs").PathSet(B.liveInputPaths);
    table("named").PathSet(B.named);
    table("prims").PathSet(B.prims);
    table("xformPrims").PathSet(B.xformPrims);
    table("folded").PathSet(B.folded);
    table("resolvedRoutedPrims").PathSet(B.resolvedRoutedPrims);
    {
        _DigestTable &t = table("overridden");
        for (char c : B.overridden) {
            t.Int(c);
        }
        t.count = B.overridden.size();
    }
    {
        _DigestTable &t = table("overridableInputs");
        for (const auto &[path, indices] : B.overridableInputs) {
            t.Path(path);
            t.Int(int64_t(indices.size()));
            for (int index : indices) {
                t.Int(index);
            }
        }
        t.count = B.overridableInputs.size();
    }
    {
        _DigestTable &t = table("inputCounts");
        t.Int(int64_t(B.boundInputs));
        t.Int(int64_t(B.varyingInputs));
        t.count = 2;
    }
    {
        _DigestTable &t = table("slots");
        for (size_t i = 0; i < B.paths.size(); ++i) {
            t.Path(B.paths[i]);
            t.Int(int64_t(B.slotKind[i]));
            t.Int(B.providerActive.empty() ? 1 : int(B.providerActive[i]));
            t.Int(B.parent[i]);
            t.Int(B.propParent[i]);
        }
        for (int slot : B.xformSlots) {
            t.Int(slot);
        }
        t.count = B.paths.size();
    }
    {
        _DigestTable &t = table("ladders");
        for (const RigExecBakedProgramImpl::Ladder &ladder : B.ladders) {
            t.Input(ladder.posedSpace);
            t.Input(ladder.restSpace);
            t.Input(ladder.defaultSpace);
            t.Input(ladder.parentSpace);
            t.Input(ladder.parentDefaultSpace);
            t.Input(ladder.avarDefaultSpace);
            t.Input(ladder.posedDefaultSpace);
            t.Input(ladder.rotationSign);
            t.Input(ladder.rotationOrder);
            for (int c = 0; c < 6; ++c) {
                t.Input(ladder.restAvars[c]);
                t.Input(ladder.defaultAvars[c]);
            }
        }
        t.Int(B.ladderVarying ? 1 : 0);
        for (int index : B.ladderOverrides) {
            t.Int(index);
        }
        for (char c : B.restChainVaries) {
            t.Int(c);
        }
        t.count = B.ladders.size();
    }
    {
        _DigestTable &t = table("ladderComposed");
        for (size_t i = 0; i < B.restM.size(); ++i) {
            t.Matrix(B.restM[i]);
            t.Matrix(B.selfD[i]);
            t.Matrix(B.parentDinv[i]);
            t.Matrix(B.posedAuthoredM[i]);
            t.Int(B.posedAuthored[i]);
            t.Str(B.rotOrder[i].GetString());
            t.Int(i < B.rotationSign.size() ? B.rotationSign[i] : 0);
        }
        t.count = B.restM.size();
    }
    {
        _DigestTable &t = table("avarTable");
        for (double v : B.avarConstants) {
            t.Double(v);
        }
        for (const auto *bindings :
             {&B.avarBindings, &B.avarConstantBindings}) {
            t.Int(int64_t(bindings->size()));
            for (const RigExecBakedProgramImpl::AvarBinding &b : *bindings) {
                t.Int(int64_t(b.slot));
                t.Input(b.input);
            }
        }
        for (const auto &[path, index] : B.patchableAvars) {
            t.Path(path);
            t.Int(int64_t(index));
        }
        t.count = B.avarConstants.size();
    }
    {
        // Per leaf, in id order: its pool, its owner's head and the paths
        // it is filed under.
        _DigestTable &t = table("leaves");
        std::vector<std::vector<const SdfPath *>> filed(B.leafRefs.size());
        for (const auto &[path, ids] : B.leafByPath) {
            for (const uint32_t id : ids) {
                if (id < filed.size()) {
                    filed[id].push_back(&path);
                }
            }
        }
        size_t id = 0;
        frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
            const RigExecBakedLeafRef ref =
                id < B.leafRefs.size() ? B.leafRefs[id] : RigExecBakedLeafRef();
            t.Int(int64_t(ref.type));
            t.Int(int64_t(ref.index));
            t.Int(input.leaf);
            t.Path(input.head ? input.head.GetPath() : SdfPath());
            if (id < filed.size()) {
                t.Int(int64_t(filed[id].size()));
                for (const SdfPath *path : filed[id]) {
                    t.Path(*path);
                }
            }
            ++id;
        });
        t.count = B.leafRefs.size();
    }
    {
        // Per path leaf, in id order: its owner, its key and every path its
        // read can reach.
        _DigestTable &t = table("pathLeaves");
        for (const RigExecBakedPathLeafRef &ref : B.pathLeafRefs) {
            t.Int(int64_t(ref.owner));
            t.Int(int64_t(ref.a));
            t.Int(int64_t(ref.b));
            t.Int(int64_t(ref.key));
            const RigExecBakedPathLeaves *leaves =
                RigExecBakedPathLeavesOf(B, ref);
            if (!leaves || ref.key >= leaves->decl.keys.size()) {
                continue;
            }
            const RigExecRevisionLeafKey &key = leaves->decl.keys[ref.key];
            t.Path(key.path);
            t.Int(int64_t(key.type));
            t.Int(int64_t(key.time));
            t.Int(int64_t(key.flavour));
            if (ref.key < leaves->hops.size()) {
                t.Int(int64_t(leaves->hops[ref.key].size()));
                for (const SdfPath &hop : leaves->hops[ref.key]) {
                    t.Path(hop);
                }
            }
        }
        t.count = B.pathLeafRefs.size();
    }
    {
        // Per property chain: its target, arm and version base, each
        // revision's operation, weight object and walks (every hop with its
        // override slot, candidates and leaf), and its records.
        _DigestTable &t = table("propertyChains");
        const auto walk = [&t](const RigExecBakedWalk &w) {
            t.Int(int64_t(w.flavour));
            t.Int(int64_t(w.type));
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&w.hops, &w.doubleHops}) {
                t.Int(int64_t(hops->size()));
                for (const RigExecBakedWalkHop &hop : *hops) {
                    t.Path(hop.path);
                    t.Int(hop.overrideSlot);
                    t.Int(hop.chain);
                    t.Int(hop.record);
                    t.Int(hop.leaf);
                }
            }
        };
        for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
            t.Path(chain.target);
            t.Int(chain.targetExists ? 1 : 0);
            t.Int(int64_t(chain.arm));
            t.Int(chain.ownLeaf);
            t.Int(chain.targetSlot);
            t.Int(int64_t(chain.versionBase));
            for (const RigExecBakedPropertyChain::Revision &r :
                 chain.revisions) {
                t.Path(r.mover);
                t.Int(r.moverExists ? 1 : 0);
                t.Int(r.opValid ? int64_t(r.op) : -1);
                t.Path(r.weightObject);
                t.Int(r.hasTangents ? 1 : 0);
                t.Int(r.path.valid ? 1 : 0);
                t.Int(int64_t(r.path.basis));
                t.Int(r.path.pinned ? 1 : 0);
                t.Int(r.path.planar ? 1 : 0);
                t.Int(int64_t(r.path.component));
                for (const float f :
                     {r.path.offsetX, r.path.offsetZ, r.path.headingOffset}) {
                    uint32_t bits = 0;
                    std::memcpy(&bits, &f, sizeof(bits));
                    t.Int(int64_t(bits));
                }
                for (const RigExecBakedWalk *w :
                     {&r.enabled, &r.defaultWeight, &r.value, &r.minimum,
                      &r.maximum, &r.keys, &r.tangents, &r.pathPoints}) {
                    walk(*w);
                }
            }
            for (const uint32_t index : chain.records) {
                const RigExecBakedPropertyRecord &record =
                    B.propertyRecords[index];
                t.Path(record.consumer);
                t.Str(record.consumerType.GetAsToken().GetString());
                t.Int(int64_t(record.applied));
                t.Int(int64_t(record.id));
                for (const int slot : record.hopSlots) {
                    t.Int(slot);
                }
            }
        }
        for (const RigExecBakedHeadLeaf &leaf : B.headLeaves) {
            t.Path(leaf.path);
            t.Int(int64_t(leaf.type));
            t.Int(leaf.typeMatches ? 1 : 0);
            t.Path(leaf.frozenKey);
        }
        for (const auto &[path, slot] : B.headOverrideSlots) {
            t.Path(path);
            t.Int(int64_t(slot));
        }
        t.count = B.propertyChains.size();
    }
    {
        // Per SkinTopology index: whether the revision's layout is fixed at
        // compile and at Build, and its layout leaves' paths (the leaves'
        // hops are in pathLeaves).
        _DigestTable &t = table("skinLayouts");
        const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
        for (size_t r = 0; r < count; ++r) {
            const RigExecBakedProgramImpl::GeomRevision *revision =
                RigExecBakedLayoutRevision(B, r);
            t.Path(revision->moverPath);
            t.Int(revision->skinTopologyFixed ? 1 : 0);
            t.Int(revision->layoutFixed ? 1 : 0);
            for (const RigExecRevisionLeafKey &key :
                 revision->layoutLeaves.decl.keys) {
                t.Path(key.path);
                t.Int(int64_t(key.type));
                t.Int(int64_t(key.flavour));
            }
        }
        t.count = count;
    }
    {
        // Per chunked revision: the arrays Build cut its keys from, which
        // decide whether a frame adopts a layout handle.
        _DigestTable &t = table("partitions");
        size_t count = 0;
        for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
            for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                if (!revision.chunked) {
                    continue;
                }
                t.Path(revision.moverPath);
                t.Int(revision.partitionElementSize);
                t.Int(int64_t(revision.partitionIndices.size()));
                t.Bytes(revision.partitionIndices.cdata(),
                        sizeof(int) * revision.partitionIndices.size());
                ++count;
            }
        }
        t.count = count;
    }
    {
        // The reads after the head tier that resolve walks: each walk's head,
        // hops (slot, candidates, leaf), raw leaf, declared versions and
        // shadows; which path leaves and steps read them; what each step
        // and avar slot declares.
        _DigestTable &t = table("readerWalks");
        for (const RigExecBakedReaderWalk &reader : B.readerWalks) {
            t.Path(reader.head);
            t.Int(int64_t(reader.walk.flavour));
            t.Int(int64_t(reader.walk.type));
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&reader.walk.hops, &reader.walk.doubleHops}) {
                t.Int(int64_t(hops->size()));
                for (const RigExecBakedWalkHop &hop : *hops) {
                    t.Path(hop.path);
                    t.Int(hop.overrideSlot);
                    t.Int(hop.chain);
                    t.Int(hop.record);
                    t.Int(hop.leaf);
                }
            }
            t.Int(reader.rawLeaf);
            for (const uint32_t id : reader.versions) {
                t.Int(int64_t(id));
            }
            for (const auto &[version, record] : reader.shadowed) {
                t.Int(int64_t(version));
                t.Int(int64_t(record));
            }
        }
        for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
            for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                for (const int walk : revision.leaves.walks) {
                    t.Int(walk);
                }
            }
            for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
                for (const int walk : derived.revision.leaves.walks) {
                    t.Int(walk);
                }
            }
        }
        frozenDetail::_ForEachPatchableInput(
            B, [&t](const auto &input) { t.Int(input.walk); });
        for (size_t i = 0; i < B.steps.size(); ++i) {
            const RigExecBakedStep &step = B.steps[i];
            if (step.readerWalks.empty()) {
                continue;
            }
            t.Int(int64_t(i));
            for (const int walk : step.readerWalks) {
                t.Int(walk);
            }
            for (const RigExecBakedSlotRange &range : step.reads) {
                t.Int(int64_t(range.begin));
                t.Int(int64_t(range.end));
            }
            for (const auto &[version, record] : step.shadowedReads) {
                t.Int(int64_t(version));
                t.Int(int64_t(record));
            }
        }
        for (size_t slot = 0; slot < B.avarHeadReads.size(); ++slot) {
            for (const uint32_t id : B.avarHeadReads[slot]) {
                t.Int(int64_t(slot));
                t.Int(int64_t(id));
            }
        }
        t.count = B.readerWalks.size();
    }
    {
        _DigestTable &t = table("schedule");
        t.Str(RigExecBakedScheduleReport(B));
        t.count = B.steps.size();
    }
    _DigestTable total;
    std::string out;
    for (const auto &[name, t] : tables) {
        total.Str(name);
        total.Int(int64_t(t.count));
        total.Bytes(&t.hash, sizeof(t.hash));
        out += TfStringPrintf(
            "RIGEXEC_BAKED_PROGRAM_DIGEST %-22s n=%-6zu %016llx\n", name,
            t.count, static_cast<unsigned long long>(t.hash));
    }
    out += TfStringPrintf("RIGEXEC_BAKED_PROGRAM_DIGEST %-22s        %016llx\n",
                          "total", static_cast<unsigned long long>(total.hash));
    std::fwrite(out.data(), 1, out.size(), stderr);
}

} // namespace

bool
RigExecBakedProgram::_BuildInvalidationIndex(_BuildAttempt &build)
{
    RigExecRigEvaluator &E = build.E;
    auto &nativeCheckpoints = build.nativeCheckpoints;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    auto &N = build.N;
    // Property chains run INSIDE the program, off the authored stage through
    // the generation's resolved inputs, so their movers are read live like a
    // geometry mover: nothing captured, and an override on one places itself.
    for (const auto &[target, revisions] : E._propertyChains) {
        B.prims.insert(target.GetPrimPath());
        B.resolvedRoutedPrims.insert(target.GetPrimPath());
        // Named: the head tier binds the target's existence and type, which
        // the compiled rig signature does not hash, so a resync or a type edit of it
        // rebuilds. Its value edits still route through the routed prim.
        B.named.insert(target);
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
             revisions) {
            B.prims.insert(revision.moverPath);
            B.resolvedRoutedPrims.insert(revision.moverPath);
            // A path's curve points are read the same way, through the
            // revision's walk, so its curve is routed like the mover.
            const UsdPrim mover = E._stage->GetPrimAtPath(revision.moverPath);
            TfToken operation;
            if (const UsdAttribute a =
                    mover ? mover.GetAttribute(_tokens->rigExecOperation)
                          : UsdAttribute()) {
                a.Get(&operation, UsdTimeCode::Default());
            }
            RigExecPropertyOp op;
            RigExecPathBinding binding;
            if (RigExecParsePropertyOp(operation, &op) &&
                op == RigExecPropertyOp::Path &&
                (RigExecBindPathOperation(mover, &binding),
                 !binding.curve.IsEmpty())) {
                B.prims.insert(binding.curve);
                B.resolvedRoutedPrims.insert(binding.curve);
            }
        }
    }
    // The property revisions as head-tier ops, ordered by what they read and
    // refused when that order cannot be trusted.
    // Envelope objects retain their own weight field consumers and do not produce
    // mover packets. Register their bindings after the bindings registered
    // above.
    const size_t packetObjects = B.weightObjects.size();
    for (const auto &constraint : B.constraints)
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty())
            RigExecBakedBakeWeightObject(&ctx, constraint.weightObject);
    for (const auto &[target, revisions] : E._propertyChains)
        for (const auto &revision : revisions) {
            const auto prim = B.stage->GetPrimAtPath(revision.moverPath);
            const auto rel = prim.GetRelationship(_tokens->rigExecWeightObject);
            SdfPathVector targets;
            if (rel && rel.GetTargets(&targets) && !targets.empty())
                RigExecBakedBakeWeightObject(&ctx, targets[0]);
        }
    for (size_t object = packetObjects; object < B.weightObjects.size();
         ++object)
        B.weightObjects[object].envelopeOnly = true;
    if (!ctx.ok)
        return false;
    _BindPropertyChains(E, &B);
    {
        std::string error;
        if (!RigExecBakedBindOwnPropertySpaces(&B, &error)) {
            ctx.Refuse("own property space delivery could not bind: " + error,
                       E._rigPath);
            return false;
        }
    }
    RigExecBakedBuildPropertySteps(&B);
    // Geometry value identities must exist before a WeightField binds its
    // entering points and phased sources. Operations are emitted later;
    // assigning these authored revision identities performs no scheduling.
    B.revisionIndex.clear();
    B.chainRevisionBegin.resize(B.chains.size());
    B.chainRevisionEnd.resize(B.chains.size());
    for (size_t c = 0; c < B.chains.size(); ++c) {
        B.chainRevisionBegin[c] = int(B.revisionIndex.size());
        for (size_t r = 0; r < B.chains[c].revisions.size(); ++r)
            B.revisionIndex.emplace_back(int(c), int(r));
        B.chainRevisionEnd[c] = int(B.revisionIndex.size());
    }
    {
        std::string invalid;
        if (!RigExecBakedBuildWeightFields(E, &B, &invalid)) {
            ctx.Refuse(invalid, E._rigPath);
            return false;
        }
    }
    // Bakeability accepted every intervening-Xform candidate BECAUSE the Xforms
    // between it and its anchor compose to the identity and do not animate.
    // The index records those Xforms here so that moving one rebuilds the
    // program.
    {
        const UsdPrim assetRoot =
            B.stage->GetPrimAtPath(E._rigPath.GetParentPath());
        for (const SdfPath &path : E._interveningXformProviders) {
            const auto anchorIt = E._poseProviderAnchors.find(path);
            const UsdPrim anchor =
                anchorIt == E._poseProviderAnchors.end() ||
                        anchorIt->second.IsEmpty()
                    ? assetRoot
                    : B.stage->GetPrimAtPath(anchorIt->second);
            for (UsdPrim walk = B.stage->GetPrimAtPath(path.GetParentPath());
                 walk && walk != anchor; walk = walk.GetParent()) {
                B.xformPrims.insert(walk.GetPath());
            }
        }
    }

    B.posedM.assign(N, GfMatrix4d(1.0));
    B.volumePlacement.assign(N, GfMatrix4d(1.0));
    B.volumePlacementBase.assign(N, GfMatrix4d(1.0));
    B.base.resize(N);
    B.fin.resize(N);
    if (!ctx.ok) {
        return false;
    }

    // The volumes this program places and publishes: the walk's, at their
    // slots. A discovered volume with no slot is refused here as well, so a
    // forced bake cannot skip IsBakeable's refusal.
    {
        std::set<SdfPath> programVolumes;
        B.placedVolumes.assign(B.noScaleAvars.size(), 0);
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i]) {
                programVolumes.insert(B.paths[i]);
                B.placedVolumes[i] =
                    E._nativeVolumeProviders.count(B.paths[i]) ? 1 : 0;
            }
        }
        for (const SdfPath &path : _WalkVolumesWithoutASlot(
                 programVolumes, E._nativeVolumeProviders)) {
            ctx.Refuse("a volume weight the walk places has no program slot",
                       path);
        }
    }
    // The solver-checkpoint guard again, so a forced bake cannot skip it.
    for (const SdfPath &solver : _SolverCheckpointsWithoutAnOutput(
             nativeCheckpoints, E._solverDependencies, E._solverJoints,
             _BatchedSolvers(E._solverBatches))) {
        ctx.Refuse(
            "read phase names a solver checkpoint the walk does not commit "
            "for that joint",
            solver);
    }
    if (!ctx.ok)
        return false;

    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions)
            if (!revision.leaves.decl.assembles)
                ctx.Refuse("mover has no declared stage-free API4 assembly",
                           revision.moverPath);
        for (const auto &derived : chain.derived)
            if (!derived.revision.leaves.decl.assembles)
                ctx.Refuse("mover has no declared stage-free API4 assembly",
                           derived.revision.moverPath);
    }
    if (!ctx.ok)
        return false;
    return true;
}

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::_BuildStepGraph(_BuildAttempt &build)
{
    RigExecRigEvaluator &E = build.E;
    auto &impl = build.impl;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    // Last, because it is derived from everything above: the steps are the
    // straight line's pieces in the straight line's order, and the edges
    // between them follow from the slot ranges each piece declares. Built
    // once here, so that a frame costs the graph nothing.
    // Its parts are marked one level down, under the phase this one opened,
    // so the trace says which of them the graph's time goes to.
    RigExecProfilePhases graphPhases(&E._profiler, "compile");
    graphPhases.Next("Bake.the_step_graph.pose_steps");
    RigExecBakedBuildPoseSteps(&B);
    if (!B.providerRefreshError.empty()) {
        ctx.Refuse("connected provider refresh could not bind: " +
                       B.providerRefreshError,
                   E._rigPath);
        return nullptr;
    }
    // The rest and ladder composes as head ops, one pair per compose group,
    // ordered with the property revisions by what they read.
    RigExecBakedBuildRestSteps(&B);
    // Between the two halves, which is where a weight object belongs in
    // program order: its placement comes from the pose walk and its packet is
    // what a revision assembles against.
    graphPhases.Next("Bake.the_step_graph.weight_steps");
    RigExecBakedBuildWeightSteps(&B);
    graphPhases.Next("Bake.the_step_graph.geometry_steps");
    RigExecBakedBuildGeometrySteps(&B);
    graphPhases.Close();
    // The skin layouts as head ops, one per fixed skin revision, now that
    // the geometry steps number the revisions; ordered after the rest and
    // ladder ops, and declared by the steps that read the handle.
    RigExecBakedBuildLayoutSteps(&B);
    RigExecBakedBuildAvarSteps(&B);
    RigExecBakedDeclareInputDependencies(&B);
    RigExecBakedDeclareLayoutReads(&B);
    // Every reader of a provider value outside the provider program is
    // declared now; the provider steps none of them reaches go before their
    // leaves are numbered and their steps built.
    {
        std::string invalid;
        if (!RigExecBakedPruneProviderProgram(&B, &invalid)) {
            ctx.Refuse("provider pruning failed: " + invalid, E._rigPath);
            return nullptr;
        }
    }
    // The complete prefix changes every region index. Rebuild all shared
    // edges, levels, clusters and cones from the final declarations.

    // One leaf per binding, now that every binding exists. The first run
    // samples them all (RigExecBakedSampleLeaves, rule 1).
    RigExecBakedNumberLeaves(&B);
    RigExecBakedNoteRestLeaves(&B);
    RigExecBakedBuildSpaceSteps(&B);
    {
        std::string invalid;
        if (!RigExecBakedFinalizeCrossDomainReads(&B, &invalid)) {
            ctx.Refuse("cross-domain input could not bind: " + invalid,
                       E._rigPath);
            return nullptr;
        }
    }
    for (auto &step : B.steps)
        if (step.kind == RigExecBakedStepKind::SpaceExpression &&
            step.part == 3)
            RigExecBakedDeclareCrossDomainRead(
                B, B.providerRoutedReads[size_t(step.object)], &step.reads);
    // Keep authored revision identity stable when an earlier optional
    // revision was withdrawn. Runtime array indices remain dense and local;
    // only the compiler descriptor key uses the authored position.
    const auto authoredRevision = [&](const SdfPath &chain,
                                      const SdfPath &owner,
                                      const SdfPath &target, bool derived) {
        const auto &chains = derived ? E._graphDerivedChains : E._graphChains;
        const auto found = chains.find(chain);
        int result = -1;
        if (found != chains.end())
            for (size_t i = 0; i < found->second.size(); ++i) {
                const auto &revision = found->second[i];
                if (revision.moverPath != owner || revision.target != target)
                    continue;
                if (result >= 0) {
                    ctx.Refuse("authored revision identity is ambiguous",
                               owner);
                    return -1;
                }
                result = int(i);
            }
        if (result < 0)
            ctx.Refuse("authored revision identity is missing", owner);
        return result;
    };
    // Keys describe authored owners and domain-local revisions, independently
    // of the descriptor emission or eventual execution order.
    RigExecPathText labelText;
    for (auto &step : B.steps) {
        using K = RigExecBakedStepKind;
        SdfPath owner, target;
        int revision = -1, ordinal = -1;
        std::string expression;
        switch (step.kind) {
        case K::ComposeSubtree:
        case K::RestCompose:
        case K::LadderCompose:
            owner = B.paths[size_t(B.composeGroups[size_t(step.object)].begin)];
            break;
        case K::AvarInputs:
        case K::ProviderMatrix:
        case K::VolumePlacements:
            owner = B.paths[size_t(step.object)];
            break;
        case K::Solve:
            owner = B.solvers[size_t(step.object)].path;
            break;
        case K::SolverCommit:
        case K::Constraint:
        case K::CommitDelta:
        case K::PropagateChunk:
        case K::CommitApply: {
            const auto &commit = B.commits[size_t(step.object)];
            owner = commit.moverPath;
            if (owner.IsEmpty() && size_t(step.object) < B.walkSteps.size()) {
                const auto &walk = B.walkSteps[size_t(step.object)];
                if (!walk.batchSolvers.empty())
                    owner = B.solvers[size_t(walk.batchSolvers.front())].path;
            }
            if (!commit.slots.empty())
                target = B.paths[size_t(commit.slots.front())];
            break;
        }
        case K::PropertyRevision: {
            const auto &chain = B.propertyChains[size_t(step.object)];
            target = chain.target;
            revision = step.part;
            owner = step.part > 0 ? chain.revisions[size_t(step.part - 1)].mover
                                  : chain.target;
            break;
        }
        case K::SkinTopology: {
            if (step.object < 0) {
                ctx.Refuse("layout has no authored revision", E._rigPath);
                return nullptr;
            }
            const size_t layout = size_t(step.object);
            if (layout < B.revisionIndex.size()) {
                const auto &entry = B.revisionIndex[layout];
                const auto &chain = B.chains[size_t(entry.first)];
                owner = chain.revisions[size_t(entry.second)].moverPath;
                target = chain.target;
                revision = authoredRevision(chain.target, owner, target, false);
            } else if (layout - B.revisionIndex.size() <
                       B.derivedIndex.size()) {
                const auto &entry =
                    B.derivedIndex[layout - B.revisionIndex.size()];
                const auto &chain = B.chains[size_t(entry.first)];
                const auto &derived = chain.derived[size_t(entry.second)];
                owner = derived.revision.moverPath;
                target = derived.target;
                revision = authoredRevision(chain.target, owner, target, true);
            } else {
                ctx.Refuse("layout has no authored revision", E._rigPath);
                return nullptr;
            }
            break;
        }
        case K::InfluenceFold:
        case K::RevisionStatic:
        case K::RevisionChunk:
        case K::RevisionFuse: {
            const auto &entry = B.revisionIndex[size_t(step.object)];
            const auto &chain = B.chains[size_t(entry.first)];
            owner = chain.revisions[size_t(entry.second)].moverPath;
            target = chain.target;
            revision = authoredRevision(chain.target, owner, target, false);
            break;
        }
        case K::Derived: {
            const auto &entry = B.derivedIndex[size_t(step.object)];
            const auto &derived =
                B.chains[size_t(entry.first)].derived[size_t(entry.second)];
            owner = derived.revision.moverPath;
            target = derived.target;
            revision = authoredRevision(B.chains[size_t(entry.first)].target,
                                        owner, derived.target, true);
            break;
        }
        case K::ChainInputs:
        case K::ChainStatus:
            owner = B.chains[size_t(step.object)].target;
            break;
        case K::PoseInterpolator:
            owner = B.poseInterpolators[size_t(step.object)].path;
            break;
        case K::FrameMatrix: {
            const auto &record = B.frameRecords[size_t(step.object)];
            owner = record.mover;
            target = B.paths[size_t(record.slot)];
            break;
        }
        case K::WeightPacket:
            owner = B.weightObjects[size_t(step.object)].path;
            break;
        case K::WeightField: {
            const auto &field = B.weightFields[size_t(step.object)];
            owner = B.weightObjects[size_t(field.object)].path;
            revision = field.part;
            if (field.form == RigExecBakedProgramImpl::WeightField::Form::
                                  EnvelopeConstraint) {
                if (field.consumer < 0 ||
                    size_t(field.consumer) >= B.walkSteps.size() ||
                    B.walkSteps[size_t(field.consumer)].solverBatch ||
                    B.walkSteps[size_t(field.consumer)].index < 0 ||
                    size_t(B.walkSteps[size_t(field.consumer)].index) >=
                        B.constraints.size()) {
                    ctx.Refuse("weight field has no constraint owner", owner);
                    return nullptr;
                }
                target = B
                             .constraints[size_t(
                                 B.walkSteps[size_t(field.consumer)].index)]
                             .path;
            } else if (field.form ==
                       RigExecBakedProgramImpl::WeightField::Form::Revision) {
                const auto &entry = B.revisionIndex[size_t(field.consumer)];
                target = B.chains[size_t(entry.first)].target;
                const auto &geometry = B.chains[size_t(entry.first)]
                                           .revisions[size_t(entry.second)];
                revision =
                    authoredRevision(target, geometry.moverPath, target, false);
            } else
                target = B.propertyChains[size_t(field.consumer)].target;
            break;
        }
        case K::SpaceCheckpoint:
            expression = B.switchFrameContexts[size_t(step.object)].key;
            break;
        case K::ProviderRefresh:
            expression = B.providerRefreshes[size_t(step.object)].key;
            owner =
                B.paths[size_t(B.providerRefreshes[size_t(step.object)].slot)];
            break;
        case K::SpaceExpression: {
            RigExecValueId value = RigExecNoProviderValue;
            const size_t object = size_t(step.object);
            if (step.part == 0)
                value = B.GetProviderProgram().ops[object].output;
            else if (step.part == 1)
                value = B.GetProviderProgram().externalInputs[object].value;
            else if (step.part == 2)
                value = B.GetProviderProgram().sampled[object].value;
            else if (step.part == 3)
                value = B.GetProviderProgram().routedInputs[object].value;
            else if (step.part == 4)
                value = B.providerFrameInputs[object].value;
            expression = B.GetProviderProgram().valueKeys[size_t(value)];
            break;
        }
        case K::SnapshotFinals:
            expression = "retired";
            break;
        }
        const auto position = E._nativePoseOrdinals.find(owner);
        if (position != E._nativePoseOrdinals.end())
            ordinal = position->second;
        step.label = expression.empty() ? labelText(owner) : expression;
        step.descriptorKey = step.label +
                             "/category:" + std::to_string(int(step.kind)) +
                             "/stack:" + std::to_string(ordinal) +
                             "/target:" + labelText(target) +
                             "/revision:" + std::to_string(revision) +
                             "/subindex:" + std::to_string(step.part);
    }
    if (!ctx.ok)
        return nullptr;
    // Bind declared solver relationship prerequisites to canonical identities
    // after all numerical descriptors exist. No authored stack order or
    // generated scheduling predecessor is used as semantic authority.
    for (const auto &step : B.steps)
        if (step.kind == RigExecBakedStepKind::Solve)
            B.solvers[size_t(step.object)].solveDescriptorKey =
                step.descriptorKey;
    for (auto &step : B.steps)
        if (step.kind == RigExecBakedStepKind::Solve) {
            for (const auto &requirement :
                 B.solvers[size_t(step.object)].relationshipRequirements) {
                const int source = requirement.second;
                if (source < 0 || size_t(source) >= B.solvers.size() ||
                    B.solvers[size_t(source)].solveDescriptorKey.empty()) {
                    ctx.Refuse(
                        "solver relationship has no compiled Solve descriptor",
                        B.solvers[size_t(step.object)].path);
                    return nullptr;
                }
                step.semanticPredecessorKeys.push_back(
                    B.solvers[size_t(source)].solveDescriptorKey);
            }
            std::sort(step.semanticPredecessorKeys.begin(),
                      step.semanticPredecessorKeys.end());
            step.semanticPredecessorKeys.erase(
                std::unique(step.semanticPredecessorKeys.begin(),
                            step.semanticPredecessorKeys.end()),
                step.semanticPredecessorKeys.end());
        }
    // Every read is declared: the chains that run as one GPU program.
    RigExecBakedSelectGpuChains(&B);
    // Canonical graph order is the stored and exported order. Clusters and
    // cones are translated once from that result at the production grain.
    {
        std::string invalid;
        if (!RigExecBakedCompileOpGraph(&B, &invalid)) {
            ctx.Refuse("native operation graph is invalid: " + invalid,
                       E._rigPath);
            return nullptr;
        }
        for (size_t i = 0; i < B.steps.size(); ++i) {
            int level = 0;
            for (int predecessor : B.steps[i].preds)
                level = std::max(level, B.steps[size_t(predecessor)].level + 1);
            B.steps[i].level = level;
        }
        RigExecBakedAssignStepCosts(&B);
        if (!RigExecBakedTranslateOpGraph(&B, &invalid)) {
            ctx.Refuse("native operation clusters could not translate: " +
                           invalid,
                       E._rigPath);
            return nullptr;
        }
        RigExecBakedBuildCones(&B);
        if (!RigExecBakedSettleWindowedOps(&B, &invalid)) {
            ctx.Refuse("windowed operations could not settle: " + invalid,
                       E._rigPath);
            return nullptr;
        }
    }

    if (B.scheduleReport) {
        const std::string report = RigExecBakedScheduleReport(B);
        std::fwrite(report.data(), 1, report.size(), stderr);
    }
    if (_ProgramDigestRequested()) {
        _PrintProgramDigest(B);
    }
    B.compiledProviderProgram = std::make_shared<const RigExecProviderProgram>(
        std::move(B.providerProgram));
    return std::unique_ptr<RigExecBakedProgram>(
        std::make_unique<RigExecBakedProgram>(_FactoryTag(0), std::move(impl)));
}

} // namespace rigExec
