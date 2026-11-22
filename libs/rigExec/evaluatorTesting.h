#ifndef RIGEXEC_EVALUATOR_TESTING_H
#define RIGEXEC_EVALUATOR_TESTING_H

// Evaluator state that only tests read. Not part of the facade.

#include "rigExec/rigEvaluator.h"

#include <cstddef>

namespace rigExec {

struct RigExecEvaluatorTesting {
    /// Skin layouts held by the dynamic walk's topology cache.
    static size_t SkinTopologyCacheSize(const RigExecRigEvaluator &e)
    {
        return e.GetSkinTopologyCacheSize();
    }
    /// Blend sample shapes held by the dynamic walk's cache.
    static size_t BlendSampleCacheSize(const RigExecRigEvaluator &e)
    {
        return e.GetBlendSampleCacheSize();
    }
    /// History ticks run so far.
    static size_t HistoryTickCount(const RigExecRigEvaluator &e)
    {
        return e.GetHistoryTickCountForTesting();
    }
};

} // namespace rigExec

#endif
