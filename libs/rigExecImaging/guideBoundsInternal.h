// RigExec imaging guide geometry queries: the bounds one published prim
// draws. Internal to the rigExecImaging target; shared by the extent
// callback (guideExtent.cpp) and the C guide-bounds surface
// (registryCapi.cpp).
// Not installed.
#ifndef RIGEXEC_IMAGING_GUIDE_BOUNDS_INTERNAL_H
#define RIGEXEC_IMAGING_GUIDE_BOUNDS_INTERNAL_H

#include "snapshotStore.h"

#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"

#include <cmath>

namespace rigExec {

// Bounds are allowed to be too big and are never allowed to be too small,
// but never to a NaN or an infinity: a reader thread framing a camera needs
// a representable box, and one overflowing authored value must not poison
// the whole range.
template <typename Vec>
inline bool
_AllFinite(const Vec &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

PXR_NS::GfRange3d _CustomControlRange(const PXR_NS::VtVec3fArray &points,
                                      const PXR_NS::VtIntArray &counts);

bool _AccumulateGuideBounds(const rigExec::RigExecPublishedPrim &published,
                            PXR_NS::GfRange3d *range);

bool
_AccumulateSubtreeGuideBounds(const rigExec::RigExecImagingSnapshot &snapshot,
                              const PXR_NS::SdfPath &path,
                              const PXR_NS::TfToken &purpose,
                              PXR_NS::GfRange3d *range);

void _WriteBounds(const PXR_NS::GfRange3d &range, double outMinMax[6]);

} // namespace rigExec

#endif
