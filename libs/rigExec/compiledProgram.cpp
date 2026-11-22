#include "compiledProgram.h"
#include "bakedProgramImpl.h"

namespace rigExec {

namespace {
// Namespace scope, so reading it takes no initialization guard: a program
// that spelled no path texts answers from here.
const std::vector<std::string> kRigExecNoPathTexts;
} // namespace

RigExecCompiledProgram::RigExecCompiledProgram(
    _VersionTag, std::shared_ptr<const RigExecBakedProgramImpl> compiled)
    : _compiled(std::move(compiled))
{
    // Built once from an impl that has finished compiling, so this pass sees
    // every record the counts and facts queries answer for.
    const RigExecBakedProgramImpl &B = *_compiled;
    _chainFacts.reserve(B.chains.size());
    _chainRevisionFacts.resize(B.chains.size());
    for (size_t chainIndex = 0; chainIndex < B.chains.size(); ++chainIndex) {
        const RigExecBakedGeomChain &chain = B.chains[chainIndex];
        _chainFacts.push_back({chain.target});
        std::vector<RigExecCompiledRevisionFacts> &facts =
            _chainRevisionFacts[chainIndex];
        facts.reserve(chain.revisions.size());
        for (const RigExecBakedGeomRevision &revision : chain.revisions) {
            facts.push_back({revision.moverPath, revision.target, revision.op});
        }
    }
    _weightFacts.reserve(B.weightObjects.size());
    for (const RigExecBakedWeightObject &weight : B.weightObjects) {
        _weightFacts.push_back({weight.path, weight.type});
    }
}

size_t
RigExecCompiledProgram::GetStepCount() const noexcept
{
    return _compiled->steps.size();
}

const RigExecBakedStep &
RigExecCompiledProgram::GetStep(size_t index) const noexcept
{
    return _compiled->steps[index];
}

size_t
RigExecCompiledProgram::GetExcludedStepCount() const noexcept
{
    return _compiled->excludedSteps.size();
}

const RigExecBakedStep &
RigExecCompiledProgram::GetExcludedStep(size_t index) const noexcept
{
    return _compiled->excludedSteps[index];
}

size_t
RigExecCompiledProgram::GetCommitCount() const noexcept
{
    return _compiled->commits.size();
}

const RigExecBakedCommit &
RigExecCompiledProgram::GetCommit(size_t index) const noexcept
{
    return _compiled->commits[index];
}

size_t
RigExecCompiledProgram::GetComposeGroupCount() const noexcept
{
    return _compiled->composeGroups.size();
}

const RigExecBakedComposeGroup &
RigExecCompiledProgram::GetComposeGroup(size_t index) const noexcept
{
    return _compiled->composeGroups[index];
}

size_t
RigExecCompiledProgram::GetProviderSlotCount() const noexcept
{
    return _compiled->paths.size();
}

size_t
RigExecCompiledProgram::GetLadderCount() const noexcept
{
    return _compiled->ladders.size();
}

const RigExecBakedLadder &
RigExecCompiledProgram::GetLadder(size_t slot) const noexcept
{
    return _compiled->ladders[slot];
}

size_t
RigExecCompiledProgram::GetSpaceSwitchCount() const noexcept
{
    return _compiled->spaceSwitches.size();
}

const RigExecBakedSpaceSwitch &
RigExecCompiledProgram::GetSpaceSwitch(size_t index) const noexcept
{
    return _compiled->spaceSwitches[index];
}

size_t
RigExecCompiledProgram::GetChainCount() const noexcept
{
    return _chainFacts.size();
}

size_t
RigExecCompiledProgram::GetChainRevisionCount(size_t chainIndex) const noexcept
{
    return _chainRevisionFacts[chainIndex].size();
}

const RigExecCompiledChainFacts &
RigExecCompiledProgram::GetChainFacts(size_t index) const noexcept
{
    return _chainFacts[index];
}

const RigExecCompiledRevisionFacts &
RigExecCompiledProgram::GetChainRevisionFacts(
    size_t chainIndex, size_t revisionIndex) const noexcept
{
    return _chainRevisionFacts[chainIndex][revisionIndex];
}

const RigExecCompiledWeightFacts &
RigExecCompiledProgram::GetWeightObjectFacts(size_t index) const noexcept
{
    return _weightFacts[index];
}

size_t
RigExecCompiledProgram::GetRevisionCount() const noexcept
{
    return _compiled->revisionIndex.size();
}

std::pair<int, int>
RigExecCompiledProgram::GetRevisionIdentity(size_t index) const noexcept
{
    return _compiled->revisionIndex[index];
}

size_t
RigExecCompiledProgram::GetDerivedTargetCount() const noexcept
{
    return _compiled->derivedIndex.size();
}

std::pair<int, int>
RigExecCompiledProgram::GetDerivedIdentity(size_t index) const noexcept
{
    return _compiled->derivedIndex[index];
}

size_t
RigExecCompiledProgram::GetWeightObjectCount() const noexcept
{
    return _weightFacts.size();
}

size_t
RigExecCompiledProgram::GetConstraintCount() const noexcept
{
    return _compiled->constraints.size();
}

const RigExecBakedConstraint &
RigExecCompiledProgram::GetConstraint(size_t index) const noexcept
{
    return _compiled->constraints[index];
}

size_t
RigExecCompiledProgram::GetSolverArrayCount() const noexcept
{
    return _compiled->solverArrays.size();
}

const std::vector<std::pair<SdfPath, int>> &
RigExecCompiledProgram::GetSolverArrays() const noexcept
{
    return _compiled->solverArrays;
}

size_t
RigExecCompiledProgram::GetConstraintArrayCount() const noexcept
{
    return _compiled->constraintArrays.size();
}

const RigExecBakedConstraintArray &
RigExecCompiledProgram::GetConstraintArray(size_t index) const noexcept
{
    return _compiled->constraintArrays[index];
}

size_t
RigExecCompiledProgram::GetPropertyChainCount() const noexcept
{
    return _compiled->propertyChains.size();
}

const RigExecBakedPropertyChain &
RigExecCompiledProgram::GetPropertyChain(size_t index) const noexcept
{
    return _compiled->propertyChains[index];
}

size_t
RigExecCompiledProgram::GetPropertyRecordCount() const noexcept
{
    return _compiled->propertyRecords.size();
}

const RigExecBakedPropertyRecord &
RigExecCompiledProgram::GetPropertyRecord(size_t index) const noexcept
{
    return _compiled->propertyRecords[index];
}

size_t
RigExecCompiledProgram::GetFrameRecordCount() const noexcept
{
    return _compiled->frameRecords.size();
}

const RigExecBakedFrameRecord &
RigExecCompiledProgram::GetFrameRecord(size_t index) const noexcept
{
    return _compiled->frameRecords[index];
}

size_t
RigExecCompiledProgram::GetClusterCount() const noexcept
{
    return _compiled->clustering.clusters.size();
}

const RigExecBakedCluster &
RigExecCompiledProgram::GetCluster(size_t index) const noexcept
{
    return _compiled->clustering.clusters[index];
}

size_t
RigExecCompiledProgram::GetPointBindingCount() const noexcept
{
    return size_t(_compiled->pointBindingCount);
}

size_t
RigExecCompiledProgram::GetNativeSourceCount() const noexcept
{
    return _compiled->nativeSources.size();
}

const RigExecBakedNativeXformSource &
RigExecCompiledProgram::GetNativeSource(size_t index) const noexcept
{
    return _compiled->nativeSources[index];
}

size_t
RigExecCompiledProgram::GetXformSlotCount() const noexcept
{
    return _compiled->xformSlots.size();
}

int
RigExecCompiledProgram::GetXformSlot(size_t index) const noexcept
{
    return _compiled->xformSlots[index];
}

const UsdPrim &
RigExecCompiledProgram::GetXformPrim(size_t index) const noexcept
{
    return _compiled->xformPrimsBySlot[index];
}

size_t
RigExecCompiledProgram::GetReaderWalkCount() const noexcept
{
    return _compiled->readerWalks.size();
}

const RigExecBakedReaderWalk &
RigExecCompiledProgram::GetReaderWalk(size_t index) const noexcept
{
    return _compiled->readerWalks[index];
}

std::vector<RigExecOpGraphNode>
RigExecCompiledProgram::GetOpGraph() const
{
    return RigExecBakedOpGraph(*_compiled);
}

const std::vector<RigExecBakedWindowedOp> &
RigExecCompiledProgram::GetWindowedOps() const noexcept
{
    return _compiled->windowedOps;
}

const std::vector<RigExecBakedSlotRange> &
RigExecCompiledProgram::GetStepReads(size_t stepIndex) const noexcept
{
    return _compiled->steps[stepIndex].reads;
}

const std::vector<RigExecBakedSlotRange> &
RigExecCompiledProgram::GetStepWrites(size_t stepIndex) const noexcept
{
    return _compiled->steps[stepIndex].writes;
}

const RigExecBakedCones &
RigExecCompiledProgram::GetCones() const noexcept
{
    return _compiled->cones;
}

const std::vector<RigExecCrossDomainRead> &
RigExecCompiledProgram::GetCrossDomainReads() const noexcept
{
    return _compiled->crossDomainReads;
}

// GetConstantDigest is defined in frozenDigest.cpp, beside the byte-hash the
// frame-cache fingerprints fold with, so the fold exists once.
uint64_t
RigExecCompiledProgram::GetProgramDigest(
    uint64_t compiledRigSignature) const noexcept
{
    return compiledRigSignature ^
           (uint64_t(_compiled->boundInputs) * 1099511628211ull) ^
           (uint64_t(_compiled->varyingInputs) * 16777619ull);
}

const std::vector<double> &
RigExecCompiledProgram::GetAvarConstants() const noexcept
{
    return _compiled->avarConstants;
}

const std::vector<RigExecBakedAvarBinding> &
RigExecCompiledProgram::GetAvarBindings() const noexcept
{
    return _compiled->avarBindings;
}

const std::vector<RigExecBakedAvarBinding> &
RigExecCompiledProgram::GetAvarConstantBindings() const noexcept
{
    return _compiled->avarConstantBindings;
}

const std::vector<size_t> &
RigExecCompiledProgram::GetPromotedAvars() const noexcept
{
    return _compiled->promotedAvars;
}

size_t
RigExecCompiledProgram::GetAvarSlotCount() const noexcept
{
    return _compiled->avars.size();
}

const RigExecRequiredStageFramesAdmission &
RigExecCompiledProgram::GetRequiredStageFramesAdmission() const noexcept
{
    return _compiled->requiredStageFramesAdmission;
}

const std::vector<SdfPath> &
RigExecCompiledProgram::GetDeltaBasePaths() const noexcept
{
    return _compiled->deltaBasePaths;
}

size_t
RigExecCompiledProgram::GetAvarConstantCount() const noexcept
{
    return _compiled->avarConstants.size();
}

size_t
RigExecCompiledProgram::GetAvarConstantBindingCount() const noexcept
{
    return _compiled->avarConstantBindings.size();
}

size_t
RigExecCompiledProgram::GetAvarBindingCount() const noexcept
{
    return _compiled->avarBindings.size();
}

const std::vector<SdfPath> &
RigExecCompiledProgram::GetBoundPaths() const noexcept
{
    return _compiled->paths;
}

const std::vector<std::string> &
RigExecCompiledProgram::GetPathTexts() const noexcept
{
    const auto &texts = _compiled->pathTexts;
    return texts ? *texts : kRigExecNoPathTexts;
}

const SdfPath &
RigExecCompiledProgram::GetAssetRootPath() const noexcept
{
    return _compiled->assetRootPath;
}

const std::vector<SdfPath> &
RigExecCompiledProgram::GetJointPaths() const noexcept
{
    return _compiled->jointPaths;
}

const std::vector<SdfPath> &
RigExecCompiledProgram::GetControlPaths() const noexcept
{
    return _compiled->controlPaths;
}

const std::vector<SdfPath> &
RigExecCompiledProgram::GetPoseWeightPaths() const noexcept
{
    return _compiled->poseWeightPaths;
}

const std::set<SdfPath> &
RigExecCompiledProgram::GetChainTargets() const noexcept
{
    return _compiled->chainTargets;
}

size_t
RigExecCompiledProgram::GetBoundInputCount() const noexcept
{
    return _compiled->boundInputs;
}

size_t
RigExecCompiledProgram::GetVaryingInputCount() const noexcept
{
    return _compiled->varyingInputs;
}

const std::vector<char> &
RigExecCompiledProgram::GetOverridden() const noexcept
{
    return _compiled->overridden;
}

const std::map<SdfPath, std::vector<int>> &
RigExecCompiledProgram::GetOverridableInputs() const noexcept
{
    return _compiled->overridableInputs;
}

const std::map<SdfPath, uint32_t> &
RigExecCompiledProgram::GetHeadOverrideSlots() const noexcept
{
    return _compiled->headOverrideSlots;
}

const std::map<SdfPath, std::vector<float>> &
RigExecCompiledProgram::GetFalloffLuts() const noexcept
{
    return _compiled->falloffLuts;
}

const std::optional<RigExecOraclePublicationContext> &
RigExecCompiledProgram::GetOraclePublications() const noexcept
{
    return _compiled->oraclePublications;
}

const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> *
RigExecCompiledProgram::GetJointSolverBinding() const noexcept
{
    return _compiled->jointSolverBinding;
}

bool
RigExecCompiledProgram::IsSameVersionAs(
    const RigExecCompiledProgram &other) const noexcept
{
    return _compiled == other._compiled;
}

} // namespace rigExec
