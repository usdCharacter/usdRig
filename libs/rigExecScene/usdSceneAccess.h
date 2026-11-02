#ifndef RIGEXEC_GRAPH_USD_SCENE_ACCESS_H
#define RIGEXEC_GRAPH_USD_SCENE_ACCESS_H
#include "sceneAccess.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/attribute.h"
namespace rigExec {
/// USD frontend for the same composed descriptor input used by scene databases.
/// Captures values only; compiled graph callbacks must not retain this adapter.
class RigExecUsdSceneAccess final : public RigExecSceneAccess {
public:
    explicit RigExecUsdSceneAccess(const UsdStageRefPtr &stage) : _stage(stage) {}
    SdfPathVector Prims(const SdfPath &root) const override;
    bool Prim(const SdfPath &, RigExecScenePrim *) const override;
    SdfPathVector Attributes(const SdfPath &prim) const override;
    SdfPathVector Relationships(const SdfPath &prim) const override;
    bool Attribute(const SdfPath &, RigExecSceneAttribute *) const override;
    // Host-only, per-property capture binding. No handle enters descriptors.
    UsdAttribute BindAttribute(const SdfPath &) const;
    // Capture-only. \p clipped tells whether value clips above the prim
    // could contribute opinions; callers that do not track it pass the
    // default and always sample. \p subsetMetadata keeps only the metadata
    // keys scene translation reads (rigExecReadPhase, rigExecInputElement,
    // interpolation); the default keeps the complete facts, as
    // Attribute(SdfPath) does.
    bool Attribute(const UsdAttribute &, RigExecSceneAttribute *,
                   bool clipped = true, bool subsetMetadata = false) const;
    bool Resolve(const UsdAttribute &, UsdTimeCode, VtValue *) const;
    // Resolve where the caller already established the prim is active;
    // identical to Resolve otherwise. Capture-only.
    bool ResolveActive(const UsdAttribute &, UsdTimeCode, VtValue *) const;
    // Prim facts plus its property paths in one prim lookup; the same
    // queries Prim, Attributes, and Relationships run, in the same order.
    // When boundAttributes is given, it receives the attribute objects
    // parallel to the attribute paths, so capture binds no path twice.
    // Transient handles for the caller; none enters descriptors.
    // \p subsetMetadata keeps only the metadata keys scene translation
    // reads (none: the complete facts never carry the probed clips key);
    // the default keeps complete facts.
    bool PrimProperties(const SdfPath &, RigExecScenePrim *, SdfPathVector *,
        SdfPathVector *, std::vector<UsdAttribute> *boundAttributes = nullptr,
        bool subsetMetadata = false) const;
    bool ValueBlocked(const UsdAttribute &, UsdTimeCode) const;
    bool Relationship(const SdfPath &, RigExecSceneRelationship *) const override;
    // Capture-only. Relationship with capture-subset metadata
    // (rigExecReadPhase); complete facts stay on Relationship.
    bool RelationshipCaptured(const SdfPath &,
                              RigExecSceneRelationship *) const;
    bool Resolve(const SdfPath &, UsdTimeCode, VtValue *) const override;
    bool ValueBlocked(const SdfPath &, UsdTimeCode) const override;
    bool HasIdentity(UsdTimeCode time) const override;
    double TimeCodesPerSecond() const override;
    double FramesPerSecond() const override;
    TfToken Interpolation() const override;
    TfToken UpAxis() const override;
private:
    UsdStageRefPtr _stage;
};
}
#endif
