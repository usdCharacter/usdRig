// Helpers the baked program's build, bakeability and run files share.
#ifndef RIGEXEC_BAKED_PROGRAM_SHARED_H
#define RIGEXEC_BAKED_PROGRAM_SHARED_H

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <cstring>
#include <map>
#include <set>
#include <vector>

namespace rigExec {

namespace bakedProgramDetail {

inline SdfPathVector
_Targets(const UsdPrim &prim, const TfToken &name)
{
    SdfPathVector targets;
    if (prim) {
        if (const UsdRelationship rel = prim.GetRelationship(name)) {
            rel.GetTargets(&targets);
        }
    }
    return targets;
}

// The three placeable weight shapes.
inline bool
_IsVolumeWeightTypeName(const TfToken &type)
{
    return type == SchemaTypeName::RigExecSphereWeight ||
           type == SchemaTypeName::RigExecPlaneWeight ||
           type == SchemaTypeName::RigExecCurveWeight;
}

// The volumes the walk places (_nativeVolumeProviders) that have no
// volume-typed provider slot, so the program could not place or publish
// them. Expected empty: compile seeds every discovered volume as a provider
// (seedProvider over the taps, rigEvaluatorCompile.cpp). The converse is
// not checked because it does not hold: a constraint can name a volume
// outside the rig as a source, which makes it a provider without a tap,
// and the program then holds a slot the walk never places (placedVolumes).
inline std::vector<SdfPath>
_WalkVolumesWithoutASlot(const std::set<SdfPath> &programVolumes,
                         const std::set<SdfPath> &walkVolumes)
{
    std::vector<SdfPath> missing;
    for (const SdfPath &path : walkVolumes) {
        if (!programVolumes.count(path)) {
            missing.push_back(path);
        }
    }
    return missing;
}

// The solvers the walk runs in a batch.
template <class Batches>
std::set<SdfPath>
_BatchedSolvers(const Batches &batches)
{
    std::set<SdfPath> batched;
    for (const auto &batch : batches) {
        for (const SdfPath &solverPath : batch.solvers) {
            batched.insert(solverPath);
        }
    }
    return batched;
}

// The read-phase solver checkpoints no batched solver commits: a pair
// (joint, solver) the compile named whose solver is not batched or does not
// bind the joint. "Is an aggregate solver" is _solverDependencies membership:
// every discovered solver is a key there.
inline std::vector<SdfPath>
_SolverCheckpointsWithoutAnOutput(
    const std::map<SdfPath, std::set<SdfPath>> &snapshots,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> &solverJoints,
    const std::set<SdfPath> &batched)
{
    std::vector<SdfPath> missing;
    for (const auto &[joint, movers] : snapshots) {
        for (const SdfPath &mover : movers) {
            if (!solverDependencies.count(mover)) {
                continue;
            }
            bool bound = false;
            const auto joints = solverJoints.find(mover);
            if (batched.count(mover) && joints != solverJoints.end()) {
                for (const auto &[path, element] : joints->second) {
                    bound = bound || path == joint;
                }
            }
            if (!bound) {
                missing.push_back(mover);
            }
        }
    }
    return missing;
}

// A numeric probe time. Selection along a connection chain must not depend on
// which time code the caller asks for, and Default cannot read an attribute
// that carries only time samples, so stability is checked against both.
inline UsdTimeCode
_ProbeTime(const UsdStageRefPtr &stage)
{
    return stage && stage->HasAuthoredTimeCodeRange()
               ? UsdTimeCode(stage->GetStartTimeCode())
               : UsdTimeCode(0.0);
}

inline void
_SampleIntervening(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    auto &B = *program;
    if (B.interveningSlots.empty())
        return;
    UsdGeomXformCache cache(time);
    for (size_t k = 0; k < B.interveningSlots.size(); ++k) {
        const size_t slot = size_t(B.interveningSlots[k]);
        auto &input = B.ladders[slot].interveningSpace;
        const UsdPrim parent =
            B.stage->GetPrimAtPath(B.paths[slot].GetParentPath());
        const UsdPrim anchor = B.stage->GetPrimAtPath(B.interveningAnchors[k]);
        bool reset = false;
        input.constant =
            parent && anchor && parent != anchor
                ? cache.ComputeRelativeTransform(parent, anchor, &reset)
                : GfMatrix4d(1.0);
        B.ladders[slot].interveningReset = reset;
        if (reset)
            input.constant = GfMatrix4d(1.0);
        if (input.leaf >= 0) {
            auto &pool = B.leaves.Of<GfMatrix4d>();
            const size_t leaf = size_t(input.leaf);
            // Bitwise, as every typed leaf compares: -0 is not 0.
            RigExecBakedNoteLeafWrite(&pool, leaf,
                                      std::memcmp(pool.value[leaf].GetArray(),
                                                  input.constant.GetArray(),
                                                  16 * sizeof(double)) != 0);
            pool.value[leaf] = input.constant;
        }
    }
}

} // namespace bakedProgramDetail

} // namespace rigExec

#endif
