// RigExec imaging guide geometry queries: the bounds one published prim
// draws. Internal to the rigExecImaging target; shared by the extent
// callback (guideExtent.cpp) and the C guide-bounds surface (registry.cpp).
// Not installed.
#ifndef RIGEXEC_IMAGING_GUIDE_BOUNDS_INTERNAL_H
#define RIGEXEC_IMAGING_GUIDE_BOUNDS_INTERNAL_H

#include "snapshotStore.h"

#include "pxr/base/gf/range3d.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"

namespace rigExec {

// Weight guide draw-mode tokens. Defined once with the bounds helpers in
// guideBounds.cpp; the rest-pose fallback in guideExtent.cpp compares
// against them.
extern const TfToken _tWeightCage, _tWeightBrackets, _tWeightShell,
    _tWeightWire, _tWeightGeometry, _tWeightFalloff;

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
