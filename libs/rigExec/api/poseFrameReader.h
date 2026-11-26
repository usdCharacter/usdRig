// A RigExecFrameReader over the maps of a RigExecRigPose.
#ifndef RIGEXEC_API_POSE_FRAME_READER_H
#define RIGEXEC_API_POSE_FRAME_READER_H

#include "rigExec/api/frameReader.h"
#include "rigExec/api/rigEvaluator.h"

#include <algorithm>
#include <array>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecPoseFrameReader final : public RigExecFrameReader {
private:
    // Helper templates are defined before their uses: gcc and clang need it.
    template <class F>
    decltype(auto) _Visit(RigExecFrameDomain domain, F &&visit) const
    {
        switch (domain) {
        case RigExecFrameDomain::JointFrameBase:
            return visit(_pose.jointFramesBase);
        case RigExecFrameDomain::JointFrameFinal:
            return visit(_pose.jointFramesFinal);
        case RigExecFrameDomain::JointMatrix:
            return visit(_pose.jointMatricesFinal);
        case RigExecFrameDomain::ControlFrame:
            return visit(_pose.controlFrames);
        case RigExecFrameDomain::ProviderTransform:
            return visit(_pose.providerXforms);
        case RigExecFrameDomain::ProviderBaseTransform:
            return visit(_pose.providerBaseXforms);
        case RigExecFrameDomain::SolverFrames:
            return visit(_pose.solverFrames);
        case RigExecFrameDomain::MovedProperty:
            return visit(_pose.movedProperties);
        case RigExecFrameDomain::WeightField:
            return visit(_pose.weightFields);
        case RigExecFrameDomain::WeightFrame:
            break;
        }
        return visit(_pose.weightFrames);
    }

    template <class Map>
    const typename Map::value_type &
    _Entry(const Map &, RigExecFrameDomain domain, size_t index) const
    {
        return *static_cast<const typename Map::value_type *>(
            _entries[_begin[size_t(domain)] + index]);
    }

    const VtValue &_Moved(size_t index) const
    {
        return _Entry(_pose.movedProperties, RigExecFrameDomain::MovedProperty,
                      index)
            .second;
    }

    template <class T> bool _Held(size_t index, T *value) const
    {
        const VtValue &held = _Moved(index);
        if (!held.IsHolding<T>())
            return false;
        *value = held.UncheckedGet<T>();
        return true;
    }

public:
    /// Borrows \p pose, which must outlive the reader.
    explicit RigExecPoseFrameReader(const RigExecRigPose &pose) : _pose(pose)
    {
        _Index();
    }

    /// Shares ownership of \p pose, which must not be null.
    explicit RigExecPoseFrameReader(std::shared_ptr<const RigExecRigPose> pose)
        : _owned(std::move(pose)), _pose(*_owned)
    {
        _Index();
    }

    RigExecPoseFrameReader(const RigExecPoseFrameReader &) = delete;
    RigExecPoseFrameReader &operator=(const RigExecPoseFrameReader &) = delete;

    const RigExecRigPose &GetPose() const
    {
        return _pose;
    }

    bool IsValid() const override
    {
        return _pose.valid;
    }
    UsdTimeCode GetTime() const override
    {
        return _pose.time;
    }
    size_t GetDiagnosticCount() const override
    {
        return _pose.diagnostics.size();
    }
    const std::string &GetDiagnostic(size_t index) const override
    {
        return _pose.diagnostics[index];
    }
    RigExecFrameCounters GetCounters() const override
    {
        RigExecFrameCounters counters;
        counters.referenceMismatches = _pose.referenceMismatches;
        counters.referenceAgreements = _pose.referenceAgreements;
        counters.comparisonMismatches = _pose.comparisonMismatches;
        counters.executedOpCount = _pose.executedOpCount;
        return counters;
    }

    size_t GetCount(RigExecFrameDomain domain) const override
    {
        const size_t d = size_t(domain);
        return _begin[d + 1] - _begin[d];
    }

    const SdfPath &GetPath(RigExecFrameDomain domain,
                           size_t index) const override
    {
        return _Visit(domain, [&](const auto &map) -> const SdfPath & {
            return _Entry(map, domain, index).first;
        });
    }

    bool FindIndex(RigExecFrameDomain domain, const SdfPath &path,
                   size_t *index) const override
    {
        return _Visit(domain, [&](const auto &map) {
            using Entry = typename std::decay_t<decltype(map)>::value_type;
            const auto first = _entries.begin() + _begin[size_t(domain)];
            const auto last = _entries.begin() + _begin[size_t(domain) + 1];
            const auto found = std::lower_bound(
                first, last, path, [](const void *entry, const SdfPath &key) {
                    return static_cast<const Entry *>(entry)->first < key;
                });
            if (found == last ||
                static_cast<const Entry *>(*found)->first != path)
                return false;
            *index = size_t(found - first);
            return true;
        });
    }

    RigExecPointFrame GetPointFrame(RigExecFrameDomain domain,
                                    size_t index) const override
    {
        switch (domain) {
        case RigExecFrameDomain::JointFrameBase:
            return _Entry(_pose.jointFramesBase, domain, index).second;
        case RigExecFrameDomain::JointFrameFinal:
            return _Entry(_pose.jointFramesFinal, domain, index).second;
        case RigExecFrameDomain::ControlFrame:
            return _Entry(_pose.controlFrames, domain, index).second;
        default:
            return RigExecPointFrame();
        }
    }

    GfMatrix4d GetMatrix(RigExecFrameDomain domain, size_t index) const override
    {
        switch (domain) {
        case RigExecFrameDomain::JointMatrix:
            return _Entry(_pose.jointMatricesFinal, domain, index).second;
        case RigExecFrameDomain::ProviderTransform:
            return _Entry(_pose.providerXforms, domain, index).second;
        case RigExecFrameDomain::ProviderBaseTransform:
            return _Entry(_pose.providerBaseXforms, domain, index).second;
        case RigExecFrameDomain::WeightFrame:
            return _Entry(_pose.weightFrames, domain, index).second;
        default:
            return GfMatrix4d(1.0);
        }
    }

    TfSpan<const RigExecPointFrame> GetSolverFrames(size_t index) const override
    {
        const auto &frames =
            _Entry(_pose.solverFrames, RigExecFrameDomain::SolverFrames, index)
                .second;
        return TfSpan<const RigExecPointFrame>(frames.data(), frames.size());
    }

    const SdfPath &GetWeightFieldTarget(size_t index) const override
    {
        return _Entry(_pose.weightFields, RigExecFrameDomain::WeightField,
                      index)
            .second.target;
    }

    VtFloatArray GetWeightFieldWeights(size_t index) const override
    {
        return _Entry(_pose.weightFields, RigExecFrameDomain::WeightField,
                      index)
            .second.weights;
    }

    RigExecMovedValueKind GetMovedKind(size_t index) const override
    {
        const VtValue &value = _Moved(index);
        if (value.IsEmpty())
            return RigExecMovedValueKind::Empty;
        if (value.IsHolding<VtVec3fArray>())
            return RigExecMovedValueKind::Vec3fArray;
        if (value.IsHolding<float>())
            return RigExecMovedValueKind::Float;
        if (value.IsHolding<double>())
            return RigExecMovedValueKind::Double;
        if (value.IsHolding<GfMatrix4d>())
            return RigExecMovedValueKind::Matrix;
        if (value.IsHolding<VtDoubleArray>())
            return RigExecMovedValueKind::DoubleArray;
        return RigExecMovedValueKind::Other;
    }

    VtValue GetMovedValue(size_t index) const override
    {
        return _Moved(index);
    }
    bool GetMovedValue(size_t index, VtVec3fArray *value) const override
    {
        return _Held(index, value);
    }
    bool GetMovedValue(size_t index, float *value) const override
    {
        return _Held(index, value);
    }
    bool GetMovedValue(size_t index, double *value) const override
    {
        return _Held(index, value);
    }
    bool GetMovedValue(size_t index, GfMatrix4d *value) const override
    {
        return _Held(index, value);
    }
    bool GetMovedValue(size_t index, VtDoubleArray *value) const override
    {
        return _Held(index, value);
    }

private:
    void _Index()
    {
        size_t total = 0;
        for (size_t d = 0; d < RigExecFrameDomainCount; ++d)
            total += _Visit(RigExecFrameDomain(d),
                            [](const auto &map) { return map.size(); });
        _entries.reserve(total);
        for (size_t d = 0; d < RigExecFrameDomainCount; ++d) {
            _begin[d] = _entries.size();
            _Visit(RigExecFrameDomain(d), [&](const auto &map) {
                for (const auto &entry : map)
                    _entries.push_back(&entry);
            });
        }
        _begin[RigExecFrameDomainCount] = _entries.size();
    }

    std::shared_ptr<const RigExecRigPose> _owned;
    const RigExecRigPose &_pose;
    // Map entries of every domain, in domain then path order.
    std::vector<const void *> _entries;
    std::array<size_t, RigExecFrameDomainCount + 1> _begin{};
};

} // namespace rigExec

#endif
