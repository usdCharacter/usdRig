// Read-only descriptions of a compiled rig for oracles and reports.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecBaseWeight, SchemaAttrText::rigExecBaseWeight))
    ((rigExecOperation, SchemaAttrText::rigExecOperation))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
    ((rigExecCombineWeight, SchemaTypeName::RigExecCombineWeight))
    ((rigExecCurveWeight, SchemaTypeName::RigExecCurveWeight))
    ((rigExecPlaneWeight, SchemaTypeName::RigExecPlaneWeight))
    ((rigExecDynamicWeight, SchemaTypeName::RigExecDynamicWeight))
    ((rigExecCombineMode, SchemaAttrText::rigExecCombineMode))
    (multiply)
    ((rigExecWeightTarget, SchemaAttrText::rigExecWeightTarget))
    ((rigExecSampleSource, SchemaAttrText::rigExecSampleSource))
    ((rigExecCurve, SchemaAttrText::rigExecCurve))
    ((rigExecPlaneAxis, SchemaAttrText::rigExecPlaneAxis))
    (y)
    ((rigExecPlaneBounds, SchemaAttrText::rigExecPlaneBounds))
    (unbounded)
    ((rigExecRepresentation, SchemaAttrText::rigExecRepresentation))
    (constant)
    ((rigExecRangePolicy, SchemaAttrText::rigExecRangePolicy))
    (strict)
    ((rigExecIndices, SchemaAttrText::rigExecIndices))
    ((rigExecValues, SchemaAttrText::rigExecValues))
    ((rigExecDefaultWeight, SchemaAttrText::rigExecDefaultWeight))
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsDefaultWeight, SchemaAttrText::inputsDefaultWeight))
    ((inputsValue, SchemaAttrText::inputsValue))
    ((inputsMin, SchemaAttrText::inputsMin))
    ((inputsMax, SchemaAttrText::inputsMax))
    ((inputsKeys, SchemaAttrText::inputsKeys))
    ((inputsTangents, SchemaAttrText::inputsTangents))
);

} // namespace

// The structural half of RigExecRigEvaluator::_ResolveWeights and
// _ResolveVolumeWeights (rigEvaluatorGeometry.cpp), statement for
// statement: the same reads, the same fallbacks, the same error text. What
// depends on a run -- placement, counts, scalar values, recursion into the
// objects this one composes -- is left to the port.
void
RigExecBakedProgram::DescribeWeightOracle(const RigExecRigEvaluator &evaluator,
                                          const SdfPath &path, UsdTimeCode time,
                                          RigExecWeightOracleFacts *facts)
{
    const RigExecRigEvaluator &E = evaluator;
    *facts = RigExecWeightOracleFacts();
    const UsdPrim prim = E._stage->GetPrimAtPath(path);
    if (!prim) {
        facts->staticError = "missing weight object " + path.GetString();
        return;
    }
    const TfToken typeName = prim.GetTypeName();
    if (!evaluatorDetail::_IsWeightObjectType(typeName)) {
        facts->staticError = "unknown weight object type " +
                             typeName.GetString() + " on " + path.GetString();
        return;
    }
    const auto readToken = [&](const TfToken &name, const TfToken &fallback) {
        TfToken value = fallback;
        if (const UsdAttribute a = prim.GetAttribute(name)) {
            a.Get(&value, time);
        }
        return value;
    };
    const auto who = [&prim] { return prim.GetPath().GetAsString(); };
    if (typeName == _tokens->rigExecCombineWeight) {
        const TfToken mode =
            readToken(_tokens->rigExecCombineMode, _tokens->multiply);
        if (mode != SchemaValueText::multiply && mode != SchemaValueText::add &&
            mode != SchemaValueText::subtract && mode != SchemaValueText::max &&
            mode != SchemaValueText::min && mode != SchemaValueText::average &&
            mode != SchemaValueText::overlay) {
            facts->staticError =
                who() + ": unknown rigExec:combineMode " + mode.GetString();
        }
        return;
    }
    if (evaluatorDetail::_IsVolumeWeightType(typeName)) {
        bool inFlight = false;
        std::string why;
        if (!evaluatorDetail::_VolumeWeightSamplesInFlight(prim, &inFlight,
                                                           &why)) {
            facts->phaseError = who() + ": " + why;
            return;
        }
        RigExecReadPhase targetPhase;
        std::string targetPhaseError;
        if (!RigExecResolveReadPhase(
                prim.GetRelationship(_tokens->rigExecWeightTarget),
                &targetPhase, &targetPhaseError)) {
            facts->phaseError = who() + ": " + targetPhaseError;
            return;
        }
        facts->samplesInFlight =
            targetPhase.kind == RigExecReadPhaseKind::Preceding;
        // What the facts below hold at `time` only, while the oracle reads
        // it at every evaluation time.
        const auto noteAnimated = [&](const UsdAttribute &a) {
            if (facts->timeVarying.IsEmpty() && a &&
                (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0)) {
                facts->timeVarying = a.GetPath();
            }
        };
        // The attribute _ReadTargetPoints reads through a relationship.
        const auto pointsSource = [&](const TfToken &relationship) {
            SdfPathVector targets;
            if (const UsdRelationship rel =
                    prim.GetRelationship(relationship)) {
                rel.GetTargets(&targets);
            }
            return targets.size() == 1
                       ? E._stage->GetAttributeAtPath(
                             evaluatorDetail::_ResolveGeometryInput(E._stage,
                                                                    targets[0]))
                       : UsdAttribute();
        };
        if (!inFlight) {
            const bool fromSampleSource = E._ReadTargetPoints(
                prim, _tokens->rigExecSampleSource, time, &facts->samples);
            facts->haveSamples =
                fromSampleSource ||
                E._ReadTargetPoints(prim, _tokens->rigExecWeightTarget, time,
                                    &facts->samples);
            if (!facts->haveSamples) {
                facts->samples.clear();
            }
            noteAnimated(pointsSource(_tokens->rigExecSampleSource));
            if (!fromSampleSource) {
                noteAnimated(pointsSource(_tokens->rigExecWeightTarget));
            }
        }
        if (typeName == _tokens->rigExecCurveWeight) {
            facts->haveCurve = E._ReadTargetPoints(prim, _tokens->rigExecCurve,
                                                   time, &facts->curve) &&
                               !facts->curve.empty();
            if (!facts->haveCurve) {
                facts->curve.clear();
            }
            noteAnimated(pointsSource(_tokens->rigExecCurve));
        }
        if (typeName == _tokens->rigExecPlaneWeight) {
            facts->planeAxis = readToken(_tokens->rigExecPlaneAxis, _tokens->y);
            facts->planeBounds =
                readToken(_tokens->rigExecPlaneBounds, _tokens->unbounded);
            noteAnimated(prim.GetAttribute(_tokens->rigExecPlaneAxis));
            noteAnimated(prim.GetAttribute(_tokens->rigExecPlaneBounds));
        }
        return;
    }

    const TfToken representation =
        readToken(_tokens->rigExecRepresentation, _tokens->constant);
    const TfToken rangePolicy =
        readToken(_tokens->rigExecRangePolicy, _tokens->strict);
    if (rangePolicy != SchemaValueText::strict &&
        rangePolicy != SchemaValueText::clamp) {
        facts->staticError = "unknown rangePolicy on " + path.GetString();
        return;
    }
    if (typeName == _tokens->rigExecDynamicWeight) {
        if (readToken(_tokens->rigExecOperation, _tokens->multiply) !=
            SchemaValueText::multiply) {
            facts->staticError =
                "unknown dynamic-weight operation on " + path.GetString();
            return;
        }
        SdfPathVector baseTargets;
        if (const UsdRelationship rel =
                prim.GetRelationship(_tokens->rigExecBaseWeight)) {
            rel.GetTargets(&baseTargets);
        }
        if (baseTargets.size() > 1) {
            facts->staticError =
                "rigExec:baseWeight must have at most one target on " +
                path.GetString();
            return;
        }
        if (baseTargets.empty()) {
            if (representation != SchemaValueText::constant) {
                facts->staticError =
                    "no-base dynamic weight must be constant on " +
                    path.GetString();
            }
            return;
        }
        const UsdPrim basePrim = E._stage->GetPrimAtPath(baseTargets[0]);
        if (!basePrim) {
            facts->staticError =
                "missing base weight object on " + path.GetString();
            return;
        }
        const auto canonicalWeightTarget = [&](const UsdPrim &p) {
            SdfPathVector t;
            if (const UsdRelationship rel =
                    p.GetRelationship(_tokens->rigExecWeightTarget)) {
                rel.GetTargets(&t);
            }
            return t.size() == 1
                       ? evaluatorDetail::_ResolveGeometryInput(E._stage, t[0])
                       : SdfPath();
        };
        if (canonicalWeightTarget(prim) != canonicalWeightTarget(basePrim) ||
            canonicalWeightTarget(prim).IsEmpty()) {
            facts->staticError =
                "dynamic/base weight target mismatch on " + path.GetString();
            return;
        }
        TfToken baseRepresentation = _tokens->constant;
        if (const UsdAttribute a =
                basePrim.GetAttribute(_tokens->rigExecRepresentation)) {
            a.Get(&baseRepresentation, time);
        }
        if (baseRepresentation != representation) {
            facts->staticError =
                "dynamic/base representation mismatch on " + path.GetString();
            return;
        }
        if (representation == SchemaValueText::sparse) {
            VtIntArray mine;
            if (const UsdAttribute a =
                    prim.GetAttribute(_tokens->rigExecIndices)) {
                a.Get(&mine, time);
            }
            if (!mine.empty()) {
                VtIntArray theirs;
                if (const UsdAttribute a =
                        basePrim.GetAttribute(_tokens->rigExecIndices)) {
                    a.Get(&theirs, time);
                }
                const std::set<int> mySupport(mine.begin(), mine.end());
                const std::set<int> baseSupport(theirs.begin(), theirs.end());
                if (mySupport != baseSupport) {
                    facts->staticError =
                        "dynamic/base sparse support mismatch on " +
                        path.GetString();
                }
            }
        }
        return;
    }
    for (const TfToken &field :
         {_tokens->rigExecValues, _tokens->rigExecIndices,
          _tokens->rigExecDefaultWeight, _tokens->rigExecRepresentation,
          _tokens->rigExecRangePolicy}) {
        const UsdAttribute a = prim.GetAttribute(field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            facts->staticError =
                std::string("static weight field ") + field.GetString() +
                " has time samples or connections on " + path.GetString();
            return;
        }
    }
}

bool
RigExecBakedProgram::ComposeEnvelopeObjects(
    const RigExecRigEvaluator &evaluator, const RigExecCompiledProgram &program,
    std::vector<RigExecBakedEnvelopeObject> *objects,
    std::map<SdfPath, int> *index, std::string *error)
{
    return ComposeEnvelopeObjects(evaluator, program.Storage(), objects, index,
                                  error);
}

bool
RigExecBakedProgram::ComposeEnvelopeObjects(
    const RigExecRigEvaluator &evaluator,
    const RigExecBakedProgramImpl &program,
    std::vector<RigExecBakedEnvelopeObject> *objects,
    std::map<SdfPath, int> *index, std::string *error)
{
    objects->clear();
    *index = program.weightIndex;
    const auto require = [&](const SdfPath &path) {
        if (path.IsEmpty())
            return true;
        const auto found = program.weightIndex.find(path);
        if (found != program.weightIndex.end() && found->second >= 0)
            return true;
        if (error)
            *error = "weight envelope has no compiled descriptor: " +
                     path.GetString();
        return false;
    };
    for (const auto &constraint : program.constraints)
        if (constraint.pointsTarget.IsEmpty() &&
            !require(constraint.weightObject))
            return false;
    for (const auto &chain : program.propertyChains)
        for (const auto &revision : chain.revisions)
            if (!require(revision.weightObject))
                return false;
    return true;
}

// _CompilePropertyChains' results as _EvaluatePropertyChains binds them:
// the chain list, each revision's mover inputs (_BindInput), operation
// (_ReadOperation) and weight object, and the phased connections.
bool
RigExecBakedProgram::DescribePropertyChains(
    const RigExecRigEvaluator &evaluator,
    std::vector<RigExecBakedPropertyChainDesc> *chains, std::string *error)
{
    using Desc = RigExecBakedPropertyChainDesc;
    const RigExecRigEvaluator &E = evaluator;
    chains->clear();
    const auto fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    // The runChain arm and RigExecPhasedConsumerValue compare type names.
    const auto valueType = [](const SdfValueTypeName &type) {
        return type == SdfValueTypeNames->Float      ? Desc::ValueType::Float
               : type == SdfValueTypeNames->Double   ? Desc::ValueType::Double
               : type == SdfValueTypeNames->Matrix4d ? Desc::ValueType::Matrix4d
                                                     : Desc::ValueType::Vec3f;
    };
    const TfToken &enabledName = _tokens->inputsEnabled;
    const TfToken &defaultWeightName = _tokens->inputsDefaultWeight;
    const TfToken &operationName = _tokens->rigExecOperation;
    const TfToken &valueName = _tokens->inputsValue;
    const TfToken &minName = _tokens->inputsMin;
    const TfToken &maxName = _tokens->inputsMax;
    const TfToken &keysName = _tokens->inputsKeys;
    const TfToken &tangentsName = _tokens->inputsTangents;
    for (const SdfPath &orderedTarget : E._propertyChainOrder) {
        const auto found = E._propertyChains.find(orderedTarget);
        if (found == E._propertyChains.end()) {
            continue;
        }
        Desc chain;
        chain.target = found->first;
        chain.targetAttr = E._stage->GetAttributeAtPath(chain.target);
        if (!chain.targetAttr) {
            return fail("property chain " + chain.target.GetString() +
                        ": target attribute disappeared");
        }
        chain.valueType = valueType(chain.targetAttr.GetTypeName());
        const bool floatArm = chain.valueType == Desc::ValueType::Float ||
                              chain.valueType == Desc::ValueType::Double;
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
             found->second) {
            Desc::Revision r;
            r.mover = revision.moverPath;
            const UsdPrim mover = E._stage->GetPrimAtPath(r.mover);
            if (!mover) {
                return fail("property mover " + r.mover.GetString() +
                            " is missing");
            }
            const SdfPathVector weightObjects =
                _Targets(mover, _tokens->rigExecWeightObject);
            if (!weightObjects.empty()) {
                r.weightObject = weightObjects[0];
            }
            r.enabled = mover.GetAttribute(enabledName);
            r.defaultWeight = mover.GetAttribute(defaultWeightName);
            r.value = mover.GetAttribute(valueName);
            r.minimum = mover.GetAttribute(minName);
            r.maximum = mover.GetAttribute(maxName);
            TfToken operation;
            if (const UsdAttribute a = mover.GetAttribute(operationName)) {
                a.Get(&operation);
            }
            r.opValid = RigExecParsePropertyOp(operation, &r.op);
            if (floatArm && r.opValid && r.op == RigExecPropertyOp::Curve) {
                const UsdAttribute keys = mover.GetAttribute(keysName);
                const UsdAttribute tangents = mover.GetAttribute(tangentsName);
                for (const UsdAttribute &a : {keys, tangents}) {
                    if (RigExecBakedAnimatedOrConnected(a)) {
                        return fail("property mover " + r.mover.GetString() +
                                    " reads " + a.GetPath().GetString() +
                                    ", which is animated or connected; a "
                                    "baked curve holds its keys at one time "
                                    "only");
                    }
                }
                VtArray<GfVec2f> values;
                if (keys && keys.Get(&values)) {
                    r.keys.assign(values.begin(), values.end());
                }
                r.hasTangentsAttr = bool(tangents);
                values.clear();
                if (tangents && tangents.Get(&values)) {
                    r.tangents.assign(values.begin(), values.end());
                }
            }
            if (floatArm && r.opValid && r.op == RigExecPropertyOp::Path) {
                RigExecPathBinding binding;
                RigExecBindPathOperation(mover, &binding);
                r.path = binding.settings;
                r.pathPoints = binding.points;
            }
            chain.revisions.push_back(std::move(r));
        }
        for (const RigExecPhasedConnection &connection : E._phasedConnections) {
            if (connection.target != chain.target) {
                continue;
            }
            Desc::Phased phased;
            phased.consumer = connection.consumer;
            const Desc::ValueType consumer = valueType(connection.consumerType);
            phased.consumerType = consumer == Desc::ValueType::Float ||
                                          consumer == Desc::ValueType::Double
                                      ? consumer
                                      : chain.valueType;
            phased.applied = connection.applied;
            phased.hops = connection.hops;
            chain.phased.push_back(std::move(phased));
        }
        chains->push_back(std::move(chain));
    }
    return true;
}

} // namespace rigExec
