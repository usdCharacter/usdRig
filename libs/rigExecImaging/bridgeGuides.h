// Guide rendering owned by the imaging bridge. Built after the evaluator
// and destroyed before it; the stage, serials, overrides and selection
// arrive on every call (RigExecGuideCall). _FillVolumeGuides and
// _FillWeightOverlay are the second TU, bridgeVolumeGuides.cpp.
#ifndef RIGEXEC_IMAGING_BRIDGE_GUIDES_H
#define RIGEXEC_IMAGING_BRIDGE_GUIDES_H

#include "rigExecMath/clothModel.h"

#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

PXR_NAMESPACE_USING_DIRECTIVE

#include <map>
#include <unordered_map>
#include <vector>

namespace rigExec {

struct RigExecImagingSnapshot;
struct RigExecMoverRecord;
struct RigExecPublishedPrim;
struct RigExecRigPose;
struct RigExecValueOverride;

/// The bridge-owned state a guide fill reads, built at the call site and
/// borrowed for the duration of the call. The component keeps no copy: an
/// override or selection that moves takes effect on the next call. The
/// mover order and the display overrides are required borrows: references,
/// non-null by construction.
struct RigExecGuideCall {
    RigExecGuideCall(const UsdStageRefPtr &stage, uint64_t stageEditSerial,
                     size_t compiledRigSignature,
                     const std::vector<RigExecMoverRecord> &moverOrder,
                     const std::vector<RigExecValueOverride> &displayOverrides,
                     const SdfPath &weightOverlay)
        : stage(stage), stageEditSerial(stageEditSerial),
          compiledRigSignature(compiledRigSignature), moverOrder(moverOrder),
          displayOverrides(displayOverrides), weightOverlay(weightOverlay)
    {
    }

    UsdStageRefPtr stage;
    uint64_t stageEditSerial = 0;
    size_t compiledRigSignature = 0;
    const std::vector<RigExecMoverRecord> &moverOrder;
    const std::vector<RigExecValueOverride> &displayOverrides;
    SdfPath weightOverlay;
};

class RigExecBridgeGuides {
public:
    explicit RigExecBridgeGuides(const SdfPath &rigPath);

    /// Forgets every cached guide input. Called for any stage notice that
    /// touches the rig and on every recompile: the caches hold AUTHORED
    /// styling, and only an edit can move that.
    void InvalidateGuideCaches();

    /// The registry's per-notice entry: drops only the guide-cache entries
    /// \p notice can change -- everything on a resync, one prim's styling
    /// on a guide:* edit, a subtree's on a purpose edit (it inherits) --
    /// and leaves the rest standing. A pose edit (avars, rest channels,
    /// mover inputs) changes no guide styling and no hierarchy, so it drops
    /// nothing, so a release does not re-read every provider's styling.
    /// The first call also records that notices ARE being forwarded, which
    /// retires the coarse stage-edit-serial clear in _SyncGuideCaches; a
    /// bridge driven directly keeps that.
    void NoteGuideEdits(const UsdNotice::ObjectsChanged &notice);

    /// Publishes guide payloads (joints and aggregate solvers draw as
    /// guide geometry like OpenExec's IrJointScope) into the snapshot.
    void _FillGuides(const RigExecGuideCall &call, const RigExecRigPose &pose,
                     RigExecImagingSnapshot *snapshot);

    /// Publishes the single synthesized guide shape each control draws at
    /// its posed frame, and the final joint frames pose readers and
    /// manipulators consume.
    void _FillControlGuides(const RigExecGuideCall &call,
                            const RigExecRigPose &pose,
                            RigExecImagingSnapshot *snapshot);

    /// Publishes the cloth constraint attachments as wire links between
    /// the posed source and target samples.
    void _FillClothConstraintGuides(const RigExecGuideCall &call,
                                    const RigExecRigPose &pose,
                                    RigExecImagingSnapshot *snapshot);

    /// Publishes the wire (or solid) falloffMin/falloffMax iso-surfaces
    /// every placed influence volume draws, so a rigger can see the shape
    /// being placed rather than infer it from the deformation.
    void _FillVolumeGuides(const RigExecGuideCall &call,
                           const RigExecRigPose &pose,
                           RigExecImagingSnapshot *snapshot);

    /// Copies the selected weight object's resolved field onto the
    /// geometry prim it weights.
    void _FillWeightOverlay(const RigExecGuideCall &call,
                            const RigExecRigPose &pose,
                            RigExecImagingSnapshot *snapshot);

private:
    /// A prim's guide styling, read once and republished every generation.
    ///
    /// Rebuilding the guides would otherwise re-read every one of these off
    /// the stage on EVERY mouse move of a drag. They are authored values
    /// that do not move while a control is dragged, so each is read once. An
    /// attribute that CAN move between two generations -- connected (the
    /// opacity an IK/FK switch drives) or time-varying -- is marked live and
    /// read every generation.
    struct _GuideInputs {
        UsdPrim prim;
        bool styleReady = false;
        TfToken purpose;
        UsdAttribute colorAttr;
        bool colorLive = false;
        bool hasColor = false;
        GfVec3f color;
        UsdAttribute opacityAttr;
        bool opacityLive = false;
        bool hasOpacity = false;
        float opacity = 1.0f;
        /// A connected opacity's static half (_ReadGuideStyle's rule), read
        /// once: the source it reads and the invert and floor applied to
        /// it. `opacityRuleHeld` is false where any of those can move
        /// without a notice, and the whole rule is then read every time.
        bool opacityRuleReady = false;
        bool opacityRuleHeld = false;
        UsdAttribute opacitySource;
        bool opacityInvert = false;
        double opacityFloor = 0.0;
        bool controlReady = false;
        bool controlLive = false;
        TfToken shape;
        TfToken drawMode;
        /// guide:planeNormal -- X, Y or Z; Y is the historical drawing.
        TfToken planeNormal;
        GfVec3d scale;
        double wireWidth = 0.05;
        GfVec3d offset = GfVec3d(0.0);
        /// guide:orient -- the shape's rotation in the control's frame.
        GfQuatf orient = GfQuatf(1.0f);
        SdfPath guideSource;
        VtVec3fArray customPoints;
        VtIntArray customCounts;
        bool radiusReady = false;
        bool radiusLive = false;
        double radius = 1.0;
    };

    _GuideInputs &_GuideInputsFor(const RigExecGuideCall &call,
                                  const SdfPath &path);
    void _ReadGuideStyleCached(_GuideInputs &inputs, const RigExecRigPose &pose,
                               RigExecPublishedPrim *published);
    double _GuideRadius(_GuideInputs &inputs, UsdTimeCode time);
    void _ReadOpacityRule(_GuideInputs &inputs, UsdTimeCode time);
    float _HeldOpacity(const _GuideInputs &inputs, const RigExecRigPose &pose,
                       float fallback);
    const std::map<SdfPath, std::vector<SdfPath>> &
    _JointChildren(const RigExecGuideCall &call, const RigExecRigPose &pose);

    /// Drops the guide caches if any stage edit has happened since they
    /// were filled. Called at the top of every fill that reads them.
    void _SyncGuideCaches(const RigExecGuideCall &call);

    struct _ClothGuideBinding {
        SdfPath constraint, sourceMesh, targetMesh;
        std::vector<RigExecClothSample> source, target;
    };

    SdfPath _rigPath;
    std::unordered_map<SdfPath, _GuideInputs, SdfPath::Hash> _guideInputs;
    std::map<SdfPath, std::vector<SdfPath>> _jointChildren;
    size_t _jointChildrenKey = 0;
    bool _jointChildrenValid = false;
    /// The evaluator's stage-edit serial the guide caches were filled under.
    uint64_t _guideCacheSerial = 0;
    /// True once NoteGuideEdits has run: the registry forwards every notice
    /// that reaches the rig, so the serial clear is no longer needed.
    bool _guideNoticesForwarded = false;
    std::vector<_ClothGuideBinding> _clothGuideBindings;
    uint64_t _clothGuideSerial = ~uint64_t(0);
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_GUIDES_H
