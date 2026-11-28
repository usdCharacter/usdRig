// Shared exact published-value comparisons.
// History tests compare maps and ordered diagnostics; executor work traces
// are checked separately because cached and fresh runs can execute differently.
// Helpers report through the caller-owned failure counter.
#ifndef RIGEXEC_TESTS_POSE_COMPARE_H
#define RIGEXEC_TESTS_POSE_COMPARE_H

#include "rigExec/api/frameReader.h"
#include "rigExec/api/poseFrameReader.h"
#include "rigExec/api/rigEvaluator.h"

#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExecTest {

/// \p n values of \p a and \p b bit for bit, except that any NaN equals any
/// NaN: which NaN an operation yields (its sign and payload) depends on the
/// instruction the compiler picks, so two code paths agree only that it is
/// NaN. Prints both bit patterns of the first mismatch, after \p what.
template <class T>
bool
SameBitsOrNaN(const T *a, const T *b, size_t n, const char *what)
{
    static_assert(sizeof(T) <= sizeof(uint64_t), "a scalar");
    for (size_t i = 0; i < n; ++i) {
        if (std::isnan(a[i]) && std::isnan(b[i]))
            continue;
        if (std::memcmp(&a[i], &b[i], sizeof(T)) != 0) {
            uint64_t x = 0, y = 0;
            std::memcpy(&x, &a[i], sizeof(T));
            std::memcpy(&y, &b[i], sizeof(T));
            std::printf("    %s[%zu]: %g (0x%016llx) vs %g (0x%016llx)\n", what,
                        i, double(a[i]), (unsigned long long)x, double(b[i]),
                        (unsigned long long)y);
            return false;
        }
    }
    return true;
}

/// Exact equality on one published frame: the flags and every point.
///
/// Exact, not a tolerance, for the reason the whole baked mode is: "close"
/// between two implementations of one rig is a second rig.
inline bool
SameFrame(const rigExec::RigExecPointFrame &a,
          const rigExec::RigExecPointFrame &b)
{
    return a.flags == b.flags && a.points == b.points;
}

/// One published map domain, key by key in BOTH directions.
///
/// A key only \p reference has and a key only \p baked has are each a
/// failure: a program that publishes half the joints agrees perfectly about
/// the half it published.
template <class Map, class Equal>
void
CompareMaps(int *failures, const char *what, const std::string &where,
            const Map &reference, const Map &baked, Equal equal)
{
    for (const auto &[path, value] : reference) {
        const auto found = baked.find(path);
        if (found == baked.end()) {
            ++*failures;
            std::printf("FAIL %s: actual pose published no %s for %s\n",
                        where.c_str(), what, path.GetText());
        } else if (!equal(value, found->second)) {
            ++*failures;
            std::printf("FAIL %s: %s differs at %s\n", where.c_str(), what,
                        path.GetText());
        }
    }
    for (const auto &[path, value] : baked) {
        if (!reference.count(path)) {
            ++*failures;
            std::printf("FAIL %s: actual pose published an extra %s at %s\n",
                        where.c_str(), what, path.GetText());
        }
    }
}

/// One published domain, path by path in BOTH directions, as CompareMaps;
/// \p equal takes the reference index and the actual index.
template <class Equal>
void
CompareDomain(int *failures, const char *what, const std::string &where,
              const rigExec::RigExecFrameReader &reference,
              const rigExec::RigExecFrameReader &baked,
              rigExec::RigExecFrameDomain domain, Equal equal)
{
    const size_t referenceCount = reference.GetCount(domain);
    for (size_t index = 0; index < referenceCount; ++index) {
        const SdfPath &path = reference.GetPath(domain, index);
        size_t found = 0;
        if (!baked.FindIndex(domain, path, &found)) {
            ++*failures;
            std::printf("FAIL %s: actual pose published no %s for %s\n",
                        where.c_str(), what, path.GetText());
        } else if (!equal(index, found)) {
            ++*failures;
            std::printf("FAIL %s: %s differs at %s\n", where.c_str(), what,
                        path.GetText());
        }
    }
    const size_t bakedCount = baked.GetCount(domain);
    for (size_t index = 0; index < bakedCount; ++index) {
        const SdfPath &path = baked.GetPath(domain, index);
        if (!reference.Has(domain, path)) {
            ++*failures;
            std::printf("FAIL %s: actual pose published an extra %s at %s\n",
                        where.c_str(), what, path.GetText());
        }
    }
}

inline bool
SameSolverFrames(TfSpan<const rigExec::RigExecPointFrame> a,
                 TfSpan<const rigExec::RigExecPointFrame> b)
{
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), SameFrame);
}

/// Every published MAP domain RigExecComparePoses compares.
///
/// The weight domains are empty on both paths while the features that fill
/// them refuse the bake; they are compared anyway, for the same reason the
/// comparator compares them -- so the first generation a weight object bakes
/// is measured instead of waved through.
inline void
CompareEveryMap(int *failures, const std::string &where,
                const rigExec::RigExecFrameReader &reference,
                const rigExec::RigExecFrameReader &baked)
{
    using rigExec::RigExecFrameDomain;
    const auto sameFrame = [&](RigExecFrameDomain domain) {
        return [&, domain](size_t a, size_t b) {
            return SameFrame(reference.GetPointFrame(domain, a),
                             baked.GetPointFrame(domain, b));
        };
    };
    const auto sameMatrix = [&](RigExecFrameDomain domain) {
        return [&, domain](size_t a, size_t b) {
            return reference.GetMatrix(domain, a) == baked.GetMatrix(domain, b);
        };
    };
    CompareDomain(failures, "base joint frame", where, reference, baked,
                  RigExecFrameDomain::JointFrameBase,
                  sameFrame(RigExecFrameDomain::JointFrameBase));
    CompareDomain(failures, "final joint frame", where, reference, baked,
                  RigExecFrameDomain::JointFrameFinal,
                  sameFrame(RigExecFrameDomain::JointFrameFinal));
    CompareDomain(failures, "joint matrix", where, reference, baked,
                  RigExecFrameDomain::JointMatrix,
                  sameMatrix(RigExecFrameDomain::JointMatrix));
    CompareDomain(failures, "control frame", where, reference, baked,
                  RigExecFrameDomain::ControlFrame,
                  sameFrame(RigExecFrameDomain::ControlFrame));
    CompareDomain(failures, "provider transform", where, reference, baked,
                  RigExecFrameDomain::ProviderTransform,
                  sameMatrix(RigExecFrameDomain::ProviderTransform));
    CompareDomain(failures, "provider base transform", where, reference, baked,
                  RigExecFrameDomain::ProviderBaseTransform,
                  sameMatrix(RigExecFrameDomain::ProviderBaseTransform));
    CompareDomain(failures, "moved property", where, reference, baked,
                  RigExecFrameDomain::MovedProperty, [&](size_t a, size_t b) {
                      return reference.GetMovedValue(a) ==
                             baked.GetMovedValue(b);
                  });
    CompareDomain(failures, "solver frames", where, reference, baked,
                  RigExecFrameDomain::SolverFrames, [&](size_t a, size_t b) {
                      return SameSolverFrames(reference.GetSolverFrames(a),
                                              baked.GetSolverFrames(b));
                  });
    CompareDomain(failures, "weight field", where, reference, baked,
                  RigExecFrameDomain::WeightField, [&](size_t a, size_t b) {
                      return reference.GetWeightFieldTarget(a) ==
                                 baked.GetWeightFieldTarget(b) &&
                             reference.GetWeightFieldWeights(a) ==
                                 baked.GetWeightFieldWeights(b);
                  });
    CompareDomain(failures, "weight frame", where, reference, baked,
                  RigExecFrameDomain::WeightFrame,
                  sameMatrix(RigExecFrameDomain::WeightFrame));
}

inline void
CompareEveryMap(int *failures, const std::string &where,
                const rigExec::RigExecRigPose &reference,
                const rigExec::RigExecRigPose &baked)
{
    CompareEveryMap(failures, where, rigExec::RigExecPoseFrameReader(reference),
                    rigExec::RigExecPoseFrameReader(baked));
}

/// The published SCALARS of a generation, the diagnostics included.
///
/// Compare ordered published diagnostics. Work counts are inspected through
/// the actual executor trace by scheduling tests and can differ with cache history.
inline void
CompareGenerationScalars(int *failures, const std::string &where,
                         const rigExec::RigExecFrameReader &reference,
                         const rigExec::RigExecFrameReader &baked)
{
    const size_t referenceCount = reference.GetDiagnosticCount();
    const size_t bakedCount = baked.GetDiagnosticCount();
    bool same = referenceCount == bakedCount;
    for (size_t i = 0; same && i < referenceCount; ++i)
        same = reference.GetDiagnostic(i) == baked.GetDiagnostic(i);
    if (!same) {
        ++*failures;
        std::printf("FAIL %s: diagnostics differ (%zu vs %zu)\n", where.c_str(),
                    referenceCount, bakedCount);
        for (size_t i = 0; i < std::max(referenceCount, bakedCount); ++i) {
            const char *a = i < referenceCount
                                ? reference.GetDiagnostic(i).c_str()
                                : "<none>";
            const char *b =
                i < bakedCount ? baked.GetDiagnostic(i).c_str() : "<none>";
            if (std::string(a) != b)
                std::printf("    %s\n  vs%s\n", a, b);
        }
    }
}

inline void
CompareGenerationScalars(int *failures, const std::string &where,
                         const rigExec::RigExecRigPose &reference,
                         const rigExec::RigExecRigPose &baked)
{
    CompareGenerationScalars(failures, where,
                             rigExec::RigExecPoseFrameReader(reference),
                             rigExec::RigExecPoseFrameReader(baked));
}

/// The whole published generation: every map, then every compared scalar.
///
/// The full surface of RigExecComparePoses, for a caller whose two
/// evaluators are at the same point in their lives.
inline void
ComparePose(int *failures, const std::string &where,
            const rigExec::RigExecFrameReader &reference,
            const rigExec::RigExecFrameReader &baked)
{
    CompareEveryMap(failures, where, reference, baked);
    CompareGenerationScalars(failures, where, reference, baked);
}

inline void
ComparePose(int *failures, const std::string &where,
            const rigExec::RigExecRigPose &reference,
            const rigExec::RigExecRigPose &baked)
{
    ComparePose(failures, where, rigExec::RigExecPoseFrameReader(reference),
                rigExec::RigExecPoseFrameReader(baked));
}

/// One evaluator input: a time and the interactive overrides standing on it.
struct EvaluationState {
    UsdTimeCode time = UsdTimeCode::Default();
    std::vector<rigExec::RigExecValueOverride> overrides;
};

/// A generation depends only on its own inputs: \p after evaluated right
/// after \p before on one evaluator publishes exactly what a fresh evaluator
/// publishes for \p after alone. A step that read a value the previous
/// generation left -- a blend reading last run's input aggregate -- differs
/// here. Only the maps are compared, because the counters depend on how warm
/// the evaluator is. \p make builds one stage per evaluator; \p before must
/// publish different solver frames from \p after, or the check is vacuous.
inline void
CheckHistoryIndependent(int *failures, const std::string &what,
                        const std::function<UsdStageRefPtr()> &make,
                        const SdfPath &rigPath, const EvaluationState &before,
                        const EvaluationState &after)
{
    const std::string where = what + " (history independence)";
    const UsdStageRefPtr warmStage = make();
    const UsdStageRefPtr freshStage = make();
    if (!warmStage || !freshStage) {
        ++*failures;
        std::printf("FAIL %s: the fixture does not open\n", where.c_str());
        return;
    }
    rigExec::RigExecRigEvaluator warm(warmStage, rigPath);
    rigExec::RigExecRigEvaluator fresh(freshStage, rigPath);
    std::vector<std::string> errors;
    if (!warm.Compile(&errors) || !fresh.Compile(&errors)) {
        ++*failures;
        std::printf("FAIL %s: the fixture does not compile\n", where.c_str());
        return;
    }
    const auto evaluate = [](rigExec::RigExecRigEvaluator &evaluator,
                             const EvaluationState &state) {
        if (state.overrides.empty()) {
            evaluator.ClearInteractiveOverrides();
        } else {
            evaluator.SetInteractiveOverrides(state.overrides);
        }
        return evaluator.Evaluate(state.time);
    };
    const rigExec::RigExecRigPose firstPose = evaluate(warm, before);
    const rigExec::RigExecRigPose secondPose = evaluate(warm, after);
    const rigExec::RigExecRigPose alonePose = evaluate(fresh, after);
    const rigExec::RigExecPoseFrameReader first(firstPose);
    const rigExec::RigExecPoseFrameReader second(secondPose);
    const rigExec::RigExecPoseFrameReader alone(alonePose);
    if (!first.IsValid() || !second.IsValid() || !alone.IsValid()) {
        ++*failures;
        std::printf("FAIL %s: a generation is invalid\n", where.c_str());
        return;
    }
    const auto solvers = rigExec::RigExecFrameDomain::SolverFrames;
    bool sameSolvers = first.GetCount(solvers) == second.GetCount(solvers);
    for (size_t i = 0; sameSolvers && i < first.GetCount(solvers); ++i)
        sameSolvers = first.GetPath(solvers, i) == second.GetPath(solvers, i) &&
                      SameSolverFrames(first.GetSolverFrames(i),
                                       second.GetSolverFrames(i));
    if (sameSolvers) {
        ++*failures;
        std::printf("FAIL %s: the two states publish the same solver "
                    "frames\n",
                    where.c_str());
    }
    CompareEveryMap(failures, where + " after a previous generation", alone,
                    second);
    if (warm.GetBakedGenerationCount() != 2 ||
        fresh.GetBakedGenerationCount() != 1) {
        ++*failures;
        std::printf("FAIL %s: %zu of 2 and %zu of 1 generation(s) came from "
                    "the program\n",
                    where.c_str(), warm.GetBakedGenerationCount(),
                    fresh.GetBakedGenerationCount());
    }
}

} // namespace rigExecTest

#endif // RIGEXEC_TESTS_POSE_COMPARE_H
