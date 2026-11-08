// RigExec guide geometry queries: the bounds a published prim draws and the
// subtree accumulation an ancestor's extent answers with.
#include "guideBoundsInternal.h"

#include "sceneIndices.h"

#include "pxr/base/gf/bbox3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <cmath>
#include <cstddef>

namespace rigExec {

const TfToken _tWeightCage("cage"), _tWeightBrackets("brackets"),
    _tWeightShell("shell"), _tWeightWire("wire"), _tWeightGeometry("geometry"),
    _tWeightFalloff("falloff");

PXR_NS::GfRange3d
_CustomControlRange(const PXR_NS::VtVec3fArray &points,
                    const PXR_NS::VtIntArray &counts)
{
    PXR_NS::GfRange3d range;
    size_t total = 0;
    if (counts.empty())
        return range;
    for (int count : counts) {
        if (count < 2 || size_t(count) > points.size() - total)
            return range;
        total += size_t(count);
    }
    if (total != points.size())
        return range;
    for (const PXR_NS::GfVec3f &p : points) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(p[axis]))
                return PXR_NS::GfRange3d();
        }
        range.UnionWith(PXR_NS::GfVec3d(p));
    }
    return range;
}

// Grows \p range to cover everything one published prim draws, in ASSET
// space. Returns whether it contributed anything at all.
bool
_AccumulateGuideBounds(const rigExec::RigExecPublishedPrim &published,
                       PXR_NS::GfRange3d *range)
{
    bool any = false;

    // Joint and solver guides: a sphere where requested plus a cone reaching
    // guideLength along that frame's +X axis. Joint link frames and lengths
    // are derived from evaluated parent/child origins; solver elements derive
    // them from frame landmarks. Bounding the cone by spheres at both ends is
    // conservative by design and also covers sphere-less branch links.
    for (size_t i = 0; i < published.guideFrames.size(); ++i) {
        const PXR_NS::GfMatrix4d &frame = published.guideFrames[i];
        const double radius =
            i < published.guideRadii.size() ? published.guideRadii[i] : 1.0;
        const double length =
            i < published.guideLengths.size() ? published.guideLengths[i] : 0.0;
        const PXR_NS::GfVec3d origin = frame.ExtractTranslation();
        // Row-vector convention: row 0 is the frame's X basis, and the
        // frame is orthonormalized upstream, so it is already unit length.
        const PXR_NS::GfVec3d tip = origin + length * frame.GetRow3(0);
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);
        range->UnionWith(tip - extent);
        range->UnionWith(tip + extent);
        any = true;
    }

    // Built-in control guides are documented as unit-sized and
    // centred on the frame origin with half-extent 1
    // extension), so the unit cube bounds all six of them. Deliberately
    // NOT the exact per-shape extent: that table lives in the scene index,
    // which this registry must not depend on, and using it would make the
    // framing distance jump around as an author retypes guide:shape
    // between a sphere and a flat circle.
    // It must bound what is actually DRAWN, though, so it asks the scene
    // index's own predicate whether anything is: an unrecognized
    // shape/drawMode pair synthesizes no prim, and reporting a box for it
    // would frame a host's camera on empty space.
    if (published.hasControlGuide && published.guideOpacity > 0.0f &&
        rigExec::RigExecControlGuideIsDrawn(published.controlGuideShape,
                                            published.controlGuideDrawMode)) {
        // Wire curves are drawn with a width, in the guide's own local
        // pre-scale units, so the drawn geometry reaches half a width
        // past the unit shape BEFORE the per-axis scale applies.
        double halfWidth = 0.0;
        if (published.controlGuideDrawMode == PXR_NS::TfToken("wire") &&
            std::isfinite(published.controlGuideWireWidth) &&
            published.controlGuideWireWidth > 0.0) {
            halfWidth = published.controlGuideWireWidth * 0.5;
        }
        PXR_NS::GfRange3d local =
            published.controlGuideShape == PXR_NS::TfToken("custom")
                ? _CustomControlRange(published.controlGuidePoints,
                                      published.controlGuideCounts)
                : PXR_NS::GfRange3d(PXR_NS::GfVec3d(-1), PXR_NS::GfVec3d(1));
        if (!local.IsEmpty()) {
            const PXR_NS::GfVec3d pad(halfWidth);
            local =
                PXR_NS::GfRange3d(local.GetMin() - pad, local.GetMax() + pad);
            PXR_NS::GfMatrix4d scale(1.0);
            scale.SetScale(published.controlGuideScale);
            range->UnionWith(
                PXR_NS::GfBBox3d(local, scale * published.controlGuideFrame)
                    .ComputeAlignedRange());
            any = true;
        }
    }

    // Volume weight iso-surfaces. Their points are already the drawn
    // geometry in the element's own local space, so the exact bound is one
    // transformed range -- no per-shape table and no unit-size convention
    // to keep in step, which is what the control arm above has to settle
    // for. The implicits (a solid sphere iso-surface) carry no points and
    // are the unit sphere by construction, so ±1 is exact for them too.
    for (const rigExec::RigExecVolumeGuideElement &element :
         published.volumeGuides) {
        PXR_NS::GfRange3d local;
        for (const PXR_NS::GfVec3f &p : element.points) {
            local.UnionWith(PXR_NS::GfVec3d(p));
        }
        if (local.IsEmpty()) {
            local = PXR_NS::GfRange3d(PXR_NS::GfVec3d(-1, -1, -1),
                                      PXR_NS::GfVec3d(1, 1, 1));
        }
        // Wire curves are drawn with a width in the element's own local
        // units, so the drawn geometry reaches half a width past the
        // points -- the same correction the control arm applies.
        if (element.wireWidth > 0.0 && std::isfinite(element.wireWidth)) {
            const PXR_NS::GfVec3d pad(element.wireWidth * 0.5);
            local =
                PXR_NS::GfRange3d(local.GetMin() - pad, local.GetMax() + pad);
        }
        range->UnionWith(
            PXR_NS::GfBBox3d(local, element.xform).ComputeAlignedRange());
        any = true;
    }
    return any;
}

// The union of everything published beneath \p path that shares its
// PURPOSE, including \p path itself.
// A Boundable's extent is authoritative for its whole subtree:
// UsdGeomBBoxCache stops descending at one ("Boundables should always
// provide their own extent and do not require participation from
// descendants", bboxCache.cpp). RigExec nests providers as a matter of
// course -- a joint chain is joints under joints -- so an extent covering
// only its own guide silently drops every descendant from any ancestor's
// bound, and framing a rig framed its first joint.
// Purpose-scoped, though, because one extent carries ONE purpose: the
// cache files this box under the boundable's own resolved purpose. Folding
// a default-purpose control nested under a guide-purpose joint into that
// box would file the control's bounds under `guide`, so a viewer with
// guides off would frame around geometry it is not showing -- and a viewer
// with guides on would frame around it twice. Differing-purpose
// descendants are excluded here and warned about at compile.
bool
_AccumulateSubtreeGuideBounds(const rigExec::RigExecImagingSnapshot &snapshot,
                              const PXR_NS::SdfPath &path,
                              const PXR_NS::TfToken &purpose,
                              PXR_NS::GfRange3d *range)
{
    bool any = false;
    // The published set is path-keyed and sorted, so the subtree is one
    // contiguous run beginning at the prim itself.
    for (auto it = snapshot.prims.lower_bound(path);
         it != snapshot.prims.end() && it->first.HasPrefix(path); ++it) {
        if (it->second.guidePurpose != purpose) {
            continue;
        }
        any = _AccumulateGuideBounds(it->second, range) || any;
    }
    return any;
}

// Writes \p range out as min xyz then max xyz.
void
_WriteBounds(const PXR_NS::GfRange3d &range, double outMinMax[6])
{
    const PXR_NS::GfVec3d min = range.GetMin();
    const PXR_NS::GfVec3d max = range.GetMax();
    for (int i = 0; i < 3; ++i) {
        outMinMax[i] = min[i];
        outMinMax[i + 3] = max[i];
    }
}

} // namespace rigExec
