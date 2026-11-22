#ifndef RIGEXEC_TESTS_STANDALONE_CAPTURE_H
#define RIGEXEC_TESTS_STANDALONE_CAPTURE_H

// Test-only producer of a standalone scene database from a stage.

#include "rigExecStandalone/sceneAccess.h"
#include "rigExecStandalone/system.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/resolveInfo.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"

#include <string>
#include <vector>

namespace rigExecTest {

PXR_NAMESPACE_USING_DIRECTIVE

inline VtDictionary
Metadata(const UsdObject &object)
{
    VtDictionary result;
    for (const auto &[key, value] : object.GetAllMetadata())
        result[key.GetString()] = value;
    return result;
}

// Production pack export independently traverses USD and serializes the
// same public Db contract; the runtime sees only these rows. \p error is
// the database's validation error, empty when it validates.
inline rigExec::RigExecSceneDb
Capture(const UsdStageRefPtr &stage, const std::vector<UsdTimeCode> &times,
        std::string *error)
{
    using namespace rigExec;
    RigExecSceneDb db;
    db.upAxis = UsdGeomGetStageUpAxis(stage);
    for (const auto time : times)
        db.identities.insert(RigExecStandaloneTimeKey(time));
    db.prims[SdfPath::AbsoluteRootPath()] = {};
    for (const auto &child : stage->GetPseudoRoot().GetAllChildren())
        db.prims[SdfPath::AbsoluteRootPath()].children.push_back(
            child.GetPath());
    for (const UsdPrim &prim : stage->Traverse()) {
        db.prims[prim.GetPath()] = {
            prim.GetTypeName(), prim.GetAppliedSchemas(), Metadata(prim), true};
        for (const auto &child : prim.GetAllChildren())
            db.prims[prim.GetPath()].children.push_back(child.GetPath());
        for (const auto &attribute : prim.GetAttributes()) {
            RigExecStandaloneAttribute row;
            row.type = attribute.GetTypeName();
            row.metadata = Metadata(attribute);
            if (attribute.HasSpline())
                row.spline = VtValue(attribute.GetSpline());
            attribute.GetConnections(&row.connections);
            row.hasValue = attribute.HasValue();
            row.variability = attribute.GetVariability();
            row.hasAuthoredValue = attribute.HasAuthoredValueOpinion();
            row.hasAuthoredReadableValue = attribute.HasAuthoredValue();
            row.hasAuthoredConnections = attribute.HasAuthoredConnections();
            row.mightBeTimeVarying = attribute.ValueMightBeTimeVarying();
            attribute.GetTimeSamples(&row.sampleTimes);
            for (const auto &spec :
                 attribute.GetPropertyStack(UsdTimeCode::Default())) {
                if (!spec->HasInfo(TfToken("default")))
                    continue;
                row.hasAuthoredDefault = true;
                row.authoredDefault = spec->GetInfo(TfToken("default"));
                row.defaultBlocked =
                    row.authoredDefault.IsHolding<SdfValueBlock>();
                break;
            }
            for (const auto time : times) {
                VtValue value;
                attribute.Get(&value, time);
                if (attribute.GetResolveInfo(time).ValueIsBlocked())
                    row.blockedIdentities.insert(
                        RigExecStandaloneTimeKey(time));
                row.resolved[RigExecStandaloneTimeKey(time)] = value;
            }
            db.attributes[attribute.GetPath()] = row;
        }
        for (const auto &relationship : prim.GetRelationships()) {
            RigExecStandaloneRelationship row;
            row.metadata = Metadata(relationship);
            relationship.GetTargets(&row.targets);
            db.relationships[relationship.GetPath()] = row;
        }
    }
    error->clear();
    if (!db.Validate(error) && error->empty())
        *error = "database does not validate";
    return db;
}

} // namespace rigExecTest

#endif
