// A target's setup (Default) transform to the rig's asset space, the frame of
// the rig root's parent where joint frames live: T with p_asset = p * T, and
// T^-1, composed from the xform operations of the unshared parts of the
// target's and the asset root's ancestry, through any reader with the stage
// reader's Exists/Name/Value (the stage, or the oracle's captured facts).
// Compile and authoring time only (owning thread); never in an operation
// body. Header-only: the skin-weight generator in rigExecAuthoring places the
// target's points with it.
#ifndef RIGEXEC_SCENE_SETUP_TRANSFORM_H
#define RIGEXEC_SCENE_SETUP_TRANSFORM_H

#include "rigExec/pathText.h"
#include "rigExecScene/providerProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cmath>
#include <string>
#include <string_view>

namespace rigExec {

/// Reads the xform facts of one prim at Default from the stage. Reads take a
/// path: no reader lists a prim's attributes, so no SdfPath is built for an
/// attribute the transform does not use.
struct RigExecStageXformReader {
    const PXR_NS::UsdStage *stage;
    bool Exists(const PXR_NS::SdfPath &attribute) const
    {
        return bool(stage->GetAttributeAtPath(attribute));
    }
    /// The existing property token of \p prim spelled \p name, or empty.
    PXR_NS::TfToken Name(const PXR_NS::SdfPath &prim, std::string_view name) const
    {
        const PXR_NS::UsdPrim p = stage->GetPrimAtPath(prim);
        if (p)
            for (const PXR_NS::TfToken &candidate : p.GetPropertyNames())
                if (candidate.GetString() == name) return candidate;
        return PXR_NS::TfToken();
    }
    bool Value(const PXR_NS::SdfPath &attribute, PXR_NS::VtValue *value, std::string *) const
    {
        if (const PXR_NS::UsdAttribute a = stage->GetAttributeAtPath(attribute))
            a.Get(value, PXR_NS::UsdTimeCode::Default());
        return true;
    }
};

namespace setup_transform_detail {

// Operation lookup for RigExecComposeLocalXform over one prim: the
// operations are the ones xformOpOrder names, found by the order's own
// tokens.
template <class Reader>
struct OpSource {
    const Reader &reader;
    const PXR_NS::SdfPath &prim;
    const PXR_NS::VtTokenArray &order;
    PXR_NS::SdfPathVector *structure;
    std::string *error;
    PXR_NS::VtValue slot;
    bool failed = false;
    static bool Find(void *context, std::string_view name, int *type,
                     const PXR_NS::VtValue **value)
    {
        auto &self = *static_cast<OpSource *>(context);
        PXR_NS::TfToken token;
        for (const PXR_NS::TfToken &candidate : self.order) {
            if (candidate.GetString() == name) {
                token = candidate;
                break;
            }
        }
        // An operation the order names only inverted has no token there: take
        // the prim's existing property token, so no token is built from text.
        if (token.IsEmpty()) token = self.reader.Name(self.prim, name);
        if (token.IsEmpty()) return false;
        const PXR_NS::SdfPath path = self.prim.AppendProperty(token);
        if (!self.reader.Exists(path)) return false;
        self.structure->push_back(path);
        self.slot = PXR_NS::VtValue();
        if (!self.reader.Value(path, &self.slot, self.error)) self.failed = true;
        *type = -1;
        *value = self.slot.IsEmpty() ? nullptr : &self.slot;
        return true;
    }
};

// The composed local transform of \p prim and whether it resets the stack.
// Records every attribute the result depends on into \p structure: the
// xformOpOrder (whether or not it exists yet) and the operations it names.
template <class Reader>
bool LocalTransform(const Reader &reader, const PXR_NS::SdfPath &prim,
                    PXR_NS::GfMatrix4d *local, bool *resets,
                    PXR_NS::SdfPathVector *structure, std::string *why)
{
    const PXR_NS::SdfPath orderPath =
        prim.AppendProperty(PXR_NS::UsdGeomTokens->xformOpOrder);
    structure->push_back(orderPath);
    PXR_NS::VtTokenArray order;
    PXR_NS::VtValue value;
    if (!reader.Value(orderPath, &value, why)) return false;
    if (value.IsHolding<PXR_NS::VtTokenArray>())
        order = value.UncheckedGet<PXR_NS::VtTokenArray>();
    std::string reason;
    OpSource<Reader> ops{reader, prim, order, structure, why, PXR_NS::VtValue()};
    PXR_NS::TfToken failedOp;
    if (!RigExecComposeLocalXform(order, {&ops, &OpSource<Reader>::Find}, true, local,
                                  resets, &reason, &failedOp)) {
        std::string text;
        if (!RigExecSpellPathText(prim, &text)) text = prim.GetAsString();
        *why = reason + ": " + text + " " + failedOp.GetString();
        return false;
    }
    return !ops.failed;
}

// The chain from \p prim up to, not including, the absolute root.
inline PXR_NS::SdfPathVector Chain(PXR_NS::SdfPath prim)
{
    PXR_NS::SdfPathVector chain;
    for (; !prim.IsEmpty() && !prim.IsAbsoluteRootPath(); prim = prim.GetParentPath())
        chain.push_back(prim);
    return chain;
}

// The product of the chain's locals, leaf first; stops where one resets the stack.
template <class Reader>
bool ComposeChain(const Reader &reader, const PXR_NS::SdfPathVector &chain,
                  PXR_NS::GfMatrix4d *result, bool *anyReset,
                  PXR_NS::SdfPathVector *structure, std::string *why)
{
    *result = PXR_NS::GfMatrix4d(1.0);
    for (const PXR_NS::SdfPath &prim : chain) {
        PXR_NS::GfMatrix4d local(1.0);
        bool resets = false;
        if (!LocalTransform(reader, prim, &local, &resets, structure, why)) return false;
        *result = *result * local;
        if (resets) {
            *anyReset = true;
            break;
        }
    }
    return true;
}

}  // namespace setup_transform_detail

/// \p toAsset = the target's Default local-to-asset map T (p_asset = p * T),
/// \p fromAsset = T^-1, both resolved once in double precision. Only the
/// unshared parts of the target's and the asset root's ancestry are composed
/// (the shared part cancels); a reset of the xform stack in either composes
/// the full chains. \p structure collects every xformOpOrder and operation
/// attribute the result depends on. False with \p why for a facts failure
/// or a singular or non-finite T.
template <class Reader>
bool RigExecSetupTransform(const Reader &reader, const PXR_NS::SdfPath &target,
                           const PXR_NS::SdfPath &assetRoot, PXR_NS::GfMatrix4d *toAsset,
                           PXR_NS::GfMatrix4d *fromAsset, PXR_NS::SdfPathVector *structure,
                           std::string *why)
{
    using namespace setup_transform_detail;
    using PXR_NS::GfMatrix4d;
    const PXR_NS::SdfPathVector targetChain = Chain(target), assetChain = Chain(assetRoot);
    PXR_NS::SdfPathVector targetOnly = targetChain, assetOnly = assetChain;
    while (!targetOnly.empty() && !assetOnly.empty() && targetOnly.back() == assetOnly.back()) {
        targetOnly.pop_back();
        assetOnly.pop_back();
    }
    GfMatrix4d local(1.0), asset(1.0);
    bool anyReset = false;
    PXR_NS::SdfPathVector paths;
    if (!ComposeChain(reader, targetOnly, &local, &anyReset, &paths, why) ||
        !ComposeChain(reader, assetOnly, &asset, &anyReset, &paths, why)) {
        structure->insert(structure->end(), paths.begin(), paths.end());
        return false;
    }
    if (anyReset) {
        paths.clear();
        anyReset = false;
        if (!ComposeChain(reader, targetChain, &local, &anyReset, &paths, why) ||
            !ComposeChain(reader, assetChain, &asset, &anyReset, &paths, why)) {
            structure->insert(structure->end(), paths.begin(), paths.end());
            return false;
        }
    }
    structure->insert(structure->end(), paths.begin(), paths.end());
    const char *degenerate = "the target's setup (Default) transform to the rig's asset "
                             "space is degenerate";
    GfMatrix4d assetInverse(1.0);
    if (asset != GfMatrix4d(1.0)) {
        double det = 0.0;
        assetInverse = asset.GetInverse(&det);
        if (!std::isfinite(det) || det == 0.0) { *why = degenerate; return false; }
    }
    const GfMatrix4d t = local * assetInverse;
    double det = 0.0;
    const GfMatrix4d inverse = t == GfMatrix4d(1.0) ? GfMatrix4d(1.0) : t.GetInverse(&det);
    bool finite = true;
    for (int i = 0; i < 16; ++i)
        finite = finite && std::isfinite(t.data()[i]) && std::isfinite(inverse.data()[i]);
    if (!finite || (t != GfMatrix4d(1.0) && (!std::isfinite(det) || det == 0.0))) {
        *why = degenerate;
        return false;
    }
    *toAsset = t;
    *fromAsset = inverse;
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_SCENE_SETUP_TRANSFORM_H
