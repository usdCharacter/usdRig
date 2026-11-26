// Read access to one evaluated generation, domain by domain.
#ifndef RIGEXEC_API_FRAME_READER_H
#define RIGEXEC_API_FRAME_READER_H

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/span.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The keyed result domains of a generation.
enum class RigExecFrameDomain {
    JointFrameBase,
    JointFrameFinal,
    JointMatrix,
    ControlFrame,
    ProviderTransform,
    ProviderBaseTransform,
    SolverFrames,
    MovedProperty,
    WeightField,
    WeightFrame,
};

inline constexpr size_t RigExecFrameDomainCount = 10;

/// The type a moved property's value holds.
enum class RigExecMovedValueKind {
    Empty,
    Vec3fArray,
    Float,
    Double,
    Matrix,
    DoubleArray,
    Other,
};

struct RigExecFrameCounters {
    size_t referenceMismatches = 0;
    size_t referenceAgreements = 0;
    size_t comparisonMismatches = 0;
    size_t executedOpCount = 0;
};

/// One evaluated generation, read by domain.
///
/// Each domain enumerates by index in [0, GetCount) in SdfPath order, and
/// FindIndex answers one path. Paths, diagnostics and spans are borrowed for
/// the reader's lifetime; array values share the generation's storage.
/// A getter called with a domain it does not serve returns a default value.
class RigExecFrameReader {
public:
    virtual ~RigExecFrameReader() = default;

    virtual bool IsValid() const = 0;
    virtual UsdTimeCode GetTime() const = 0;
    virtual size_t GetDiagnosticCount() const = 0;
    virtual const std::string &GetDiagnostic(size_t index) const = 0;
    virtual RigExecFrameCounters GetCounters() const = 0;

    virtual size_t GetCount(RigExecFrameDomain domain) const = 0;
    virtual const SdfPath &GetPath(RigExecFrameDomain domain,
                                   size_t index) const = 0;
    virtual bool FindIndex(RigExecFrameDomain domain, const SdfPath &path,
                           size_t *index) const = 0;

    /// JointFrameBase, JointFrameFinal and ControlFrame.
    virtual RigExecPointFrame GetPointFrame(RigExecFrameDomain domain,
                                            size_t index) const = 0;
    /// JointMatrix, ProviderTransform, ProviderBaseTransform and WeightFrame.
    virtual GfMatrix4d GetMatrix(RigExecFrameDomain domain,
                                 size_t index) const = 0;
    virtual TfSpan<const RigExecPointFrame>
    GetSolverFrames(size_t index) const = 0;
    /// The canonical points property a weight field weights.
    virtual const SdfPath &GetWeightFieldTarget(size_t index) const = 0;
    virtual VtFloatArray GetWeightFieldWeights(size_t index) const = 0;

    virtual RigExecMovedValueKind GetMovedKind(size_t index) const = 0;
    virtual VtValue GetMovedValue(size_t index) const = 0;
    /// False, leaving the output alone, when the value holds another type.
    virtual bool GetMovedValue(size_t index, VtVec3fArray *value) const = 0;
    virtual bool GetMovedValue(size_t index, float *value) const = 0;
    virtual bool GetMovedValue(size_t index, double *value) const = 0;
    virtual bool GetMovedValue(size_t index, GfMatrix4d *value) const = 0;
    virtual bool GetMovedValue(size_t index, VtDoubleArray *value) const = 0;

    bool Has(RigExecFrameDomain domain, const SdfPath &path) const
    {
        size_t index = 0;
        return FindIndex(domain, path, &index);
    }

    bool FindPointFrame(RigExecFrameDomain domain, const SdfPath &path,
                        RigExecPointFrame *frame) const
    {
        size_t index = 0;
        if (!FindIndex(domain, path, &index))
            return false;
        *frame = GetPointFrame(domain, index);
        return true;
    }

    bool FindMatrix(RigExecFrameDomain domain, const SdfPath &path,
                    GfMatrix4d *matrix) const
    {
        size_t index = 0;
        if (!FindIndex(domain, path, &index))
            return false;
        *matrix = GetMatrix(domain, index);
        return true;
    }

    bool FindSolverFrames(const SdfPath &path,
                          TfSpan<const RigExecPointFrame> *frames) const
    {
        size_t index = 0;
        if (!FindIndex(RigExecFrameDomain::SolverFrames, path, &index))
            return false;
        *frames = GetSolverFrames(index);
        return true;
    }

    bool FindMovedValue(const SdfPath &path, VtValue *value) const
    {
        size_t index = 0;
        if (!FindIndex(RigExecFrameDomain::MovedProperty, path, &index))
            return false;
        *value = GetMovedValue(index);
        return true;
    }

    /// False when \p path has no moved value of type T.
    template <class T> bool FindMovedValue(const SdfPath &path, T *value) const
    {
        size_t index = 0;
        return FindIndex(RigExecFrameDomain::MovedProperty, path, &index) &&
               GetMovedValue(index, value);
    }
};

} // namespace rigExec

#endif
