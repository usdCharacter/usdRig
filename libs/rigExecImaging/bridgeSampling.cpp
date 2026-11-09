// Chain-sample bindings and live input sampling owned by the imaging bridge.
#include "bridgeSampling.h"

#include <algorithm>

namespace rigExec {

RigExecBridgeSampling::RigExecBridgeSampling(
    const RigExecRigEvaluator &evaluator)
    : _evaluator(evaluator)
{
}

void
RigExecBridgeSampling::NoteChainEdits(const UsdNotice::ObjectsChanged &notice)
{
    if (!_liveChainValid) {
        return;
    }
    // Whether an edit to `path` -- a value, a connection, or a property spec
    // created or removed -- can move a bound chain: any property of a chain
    // mover, a chain target itself, or anything under a weight object. A
    // mover's connected input reads its source live, so a value beside a
    // target on the same prim (a switch on the control a chain writes to)
    // moves no binding; prim-level info on a mover or a target's prim
    // still does.
    const auto touchesChain = [this](const SdfPath &path) {
        const SdfPath prim = path.GetPrimPath();
        const auto has = [](const std::vector<SdfPath> &list,
                            const SdfPath &key) {
            return std::binary_search(list.begin(), list.end(), key);
        };
        if (path.IsPropertyPath()
                ? has(_liveChainMovers, prim) || has(_liveChainTargets, path)
                : has(_liveChainPrims, prim)) {
            return true;
        }
        for (const SdfPath &weight : _liveChainWeights) {
            if (prim.HasPrefix(weight)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        // A PRIM resync can add, remove or retype a mover or a target before
        // the compiled rig signature has been recomputed: rebind on the next sample. A
        // PROPERTY resync is a spec created or removed -- the first time an
        // avar is authored in a session is one -- and is judged like a value
        // edit, by its prim.
        if (!path.IsPropertyPath() || touchesChain(path)) {
            _liveChainValid = false;
            return;
        }
    }
    // A value or connection edit on a chain-relevant prim can change a
    // folded constant, a connection, or a weight object's answer -- none of
    // which moves the compiled rig signature. Anything else (an avar on a control, a
    // pose edit) leaves every bound chain exactly as it was.
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (touchesChain(path)) {
            _liveChainValid = false;
            return;
        }
    }
}

bool
RigExecBridgeSampling::_EnsureChainBindings() const
{
    const size_t compiledRig = _evaluator.GetCompiledRigSignature();
    if (_liveChainValid && _liveChainCompiledRig == compiledRig) {
        return true;
    }
    _liveChainValid = false;
    _liveChainPrims.clear();
    _liveChainMovers.clear();
    _liveChainTargets.clear();
    _liveChainWeights.clear();
    RigExecChainSampleBindings bound;
    if (!RigExecBindChainSampleInputs(_evaluator, &bound)) {
        return false;
    }
    for (const RigExecChainSampleChain &chain : bound.chains) {
        _liveChainPrims.push_back(chain.targetPath.GetPrimPath());
        _liveChainTargets.push_back(chain.targetPath);
        for (const RigExecChainSampleRevision &revision : chain.revisions) {
            _liveChainPrims.push_back(revision.moverPath);
            _liveChainMovers.push_back(revision.moverPath);
            _liveChainWeights.insert(_liveChainWeights.end(),
                                     revision.weightObjects.begin(),
                                     revision.weightObjects.end());
        }
    }
    for (std::vector<SdfPath> *list :
         {&_liveChainPrims, &_liveChainMovers, &_liveChainTargets,
          &_liveChainWeights}) {
        std::sort(list->begin(), list->end());
        list->erase(std::unique(list->begin(), list->end()), list->end());
    }
    _liveChainBindings = std::move(bound);
    _liveChainCompiledRig = compiledRig;
    _liveChainValid = true;
    ++_liveChainGeneration;
    return true;
}

const RigExecChainSampleBindings *
RigExecBridgeSampling::AcquireChainBindings(uint64_t *generation) const
{
    if (!_EnsureChainBindings()) {
        return nullptr;
    }
    if (generation) {
        *generation = _liveChainGeneration;
    }
    return &_liveChainBindings;
}

bool
RigExecBridgeSampling::SampleLive(
    UsdTimeCode time, const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    RigExecFrameInputs *out) const
{
    if (_EnsureChainBindings() &&
        RigExecSampleFrameInputsWithTrustedChainBindings(
            _evaluator, time, overrides, upstream, _liveChainBindings, out)) {
        return true;
    }
    // The pinned route declined (a bind that failed, an override it cannot
    // place, a hook that refused): the self-binding sampler answers exactly
    // as it always has, at its own cost.
    return RigExecSampleFrameInputs(_evaluator, time, overrides, upstream, out);
}

bool
RigExecBridgeSampling::CanSampleCacheInputs()
{
    const RigExecBakedProgram *program = _evaluator.GetBakedProgram();
    if (!program || _evaluator.cpuReference) {
        return false;
    }
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(_evaluator);
    // Whether the program freezes is a question about connections and
    // prims, which no value edit moves.
    const uint64_t serial = _evaluator.GetStageStructureSerial();
    if (!_sampleSupportValid || _sampleSupportProgram != program ||
        _sampleSupportCompiledRig != compiledRig ||
        _sampleSupportSerial != serial) {
        _sampleSupport = RigExecCanFreezeProgram(_evaluator);
        _sampleSupportProgram = program;
        _sampleSupportCompiledRig = compiledRig;
        _sampleSupportSerial = serial;
        _sampleSupportValid = true;
    }
    return _sampleSupport;
}

void
RigExecBridgeSampling::InvalidateSampleSupport()
{
    _sampleSupportValid = false;
}

} // namespace rigExec
