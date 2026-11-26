#include "rigExecOps/pose/poseOps.h"

#include "rigExecOps/constraint/spaceSwitchFields.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/poseMath.h"
#include "rigExecRuntime/rbfSolver.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(AvarInputs)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsValues)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsOut)

RIGEXEC_DEFINE_OP_HANDLE(SpaceCheckpoint)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointAvars)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointRotOrders)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointRotationSigns)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentExpression)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentExpressionAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedAuthoredMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointAnchor)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointOut)

RIGEXEC_DEFINE_OP_HANDLE(PoseInterpolator)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorNumeric)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorMask)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorDriverFinal)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorDriverRest)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorParentFinal)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorParentRest)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorExtraFinals)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorExtraRests)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorExtraParentFinals)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorExtraParentRests)
RIGEXEC_DEFINE_PORT_HANDLE(PoseInterpolatorOut)

RIGEXEC_DEFINE_OP_HANDLE(RestCompose)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeRestAvars)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeRestSpace)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeIntervening)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeParentRest)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeRestMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeRest)
RIGEXEC_DEFINE_PORT_HANDLE(RestComposeRoundTrip)

RIGEXEC_DEFINE_OP_HANDLE(LadderCompose)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeSpaces)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentSpaceLeaf)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeRotationSign)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeDefaultAvars)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeOwnRest)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentRest)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentDefault)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeSelfDefault)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeDefaultRoundTrip)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposePosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentSpace)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposePosedAuthoredMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposePosedAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeParentSpaceAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(LadderComposeOutRotationSign)

RIGEXEC_DEFINE_OP_HANDLE(ComposeSlot)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotAvars)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotRotOrder)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotRotationSign)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotPosedAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotPosedAuthoredMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotParentSpace)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotParentSpaceAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotParentPosed)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotBase)
RIGEXEC_DEFINE_PORT_HANDLE(ComposeSlotPosedMatrix)

RIGEXEC_DEFINE_OP_HANDLE(SwitchedComposeSlot)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotAvars)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotRotOrder)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotRotationSign)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotPosedAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotPosedAuthoredMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotParentPosed)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotParentDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotActive)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotSpaceDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotSpacePosed)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotSourceDefaults)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotSourcePosed)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotBase)
RIGEXEC_DEFINE_PORT_HANDLE(SwitchedComposeSlotPosedMatrix)

RIGEXEC_DEFINE_OP_HANDLE(ShiftedComposeSlot)
RIGEXEC_DEFINE_PORT_HANDLE(ShiftedComposeSlotBase)
RIGEXEC_DEFINE_PORT_HANDLE(ShiftedComposeSlotDelta)
RIGEXEC_DEFINE_PORT_HANDLE(ShiftedComposeSlotShifted)
RIGEXEC_DEFINE_PORT_HANDLE(ShiftedComposeSlotOut)
RIGEXEC_DEFINE_PORT_HANDLE(ShiftedComposeSlotPosedMatrix)

bool
PoseInterpolatorPayload::operator==(const PoseInterpolatorPayload &o) const
{
    return solve == o.solve && poseCount == o.poseCount &&
           enableTranslation == o.enableTranslation &&
           allowNegativeWeights == o.allowNegativeWeights &&
           extraParents == o.extraParents && rbfPoses == o.rbfPoses &&
           rbfTranslations == o.rbfTranslations &&
           rbfPoseTypes == o.rbfPoseTypes && rbfTwistAxis == o.rbfTwistAxis &&
           rbfKernel == o.rbfKernel && rbfRadius == o.rbfRadius &&
           rbfTranslationRadius == o.rbfTranslationRadius &&
           rbfRadii == o.rbfRadii &&
           rbfTranslationRadii == o.rbfTranslationRadii &&
           rbfNormalize == o.rbfNormalize &&
           rbfEnableRotation == o.rbfEnableRotation &&
           rbfEnableTranslation == o.rbfEnableTranslation &&
           rbfWeightRows == o.rbfWeightRows && rbfWeights == o.rbfWeights &&
           methodSolverType == o.methodSolverType &&
           methodDistance == o.methodDistance &&
           methodWeightFunction == o.methodWeightFunction &&
           methodNormalize == o.methodNormalize &&
           methodTwistAxis == o.methodTwistAxis &&
           methodRawCount == o.methodRawCount &&
           methodTargetCount == o.methodTargetCount &&
           methodRadius == o.methodRadius &&
           methodWeightThreshold == o.methodWeightThreshold &&
           methodTargets == o.methodTargets &&
           methodDistanceTargets == o.methodDistanceTargets &&
           methodTargetScales == o.methodTargetScales &&
           methodCoefficients == o.methodCoefficients &&
           networkInputCount == o.networkInputCount &&
           networkLayerSizes == o.networkLayerSizes &&
           networkActivations == o.networkActivations &&
           networkAlphas == o.networkAlphas &&
           networkWeights == o.networkWeights &&
           networkBiases == o.networkBiases &&
           networkInputMean == o.networkInputMean &&
           networkInputScale == o.networkInputScale &&
           networkDefaults == o.networkDefaults &&
           mapsInputCount == o.mapsInputCount &&
           mapRowInputs == o.mapRowInputs && mapRowFrom == o.mapRowFrom &&
           mapRowTo == o.mapRowTo && mapRowSlope == o.mapRowSlope &&
           mapRowCut == o.mapRowCut && mapBegin == o.mapBegin &&
           mapUnclamped == o.mapUnclamped;
}

uint32_t
SpaceCheckpointRotOrderCode(const char *text, size_t size)
{
    if (size != 3)
        return 0;
    return uint32_t(uint8_t(text[0])) | uint32_t(uint8_t(text[1])) << 8 |
           uint32_t(uint8_t(text[2])) << 16;
}

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using namespace opIo;
using namespace fieldIo;

// ---- AvarInputs ----

enum AvarInputsField : uint32_t { kAvarCount, kTargetCount, kTargets };

bool
CheckAvarInputs(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const uint64_t avars = ValueAt<uint64_t>(check.batch, kAvarCount, check.k);
    const auto targets = ListAt<uint32_t>(check.batch, kTargets, check.k);
    std::vector<uint8_t> written(size_t(avars), 0);
    for (size_t i = 0; i < targets.Size(); ++i) {
        if (targets.data[i] >= avars)
            return Fail(error, "AvarInputs",
                        "binding " + std::to_string(i) +
                            " names an avar past avarCount");
        written[targets.data[i]] = 1;
    }
    for (size_t a = 0; a < written.size(); ++a)
        if (!written[a])
            return Fail(error, "AvarInputs",
                        "avar " + std::to_string(a) + " has no binding");
    if (!SameCount(CheckElements(check, AvarInputsValues()), targets.Size()))
        return Fail(error, "AvarInputs", "needs one value per binding");
    return true;
}

bool
AvarInputsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto targets = ListAt<uint32_t>(batch, kTargets, k);
    const auto &values = *In<RigExecArray<double>>(io, AvarInputsValues());
    if (values.Size() != targets.Size())
        return Fail(error, "AvarInputs", "needs one value per binding");
    auto &out = Out<RigExecArray<double>>(io, AvarInputsOut());
    out.Resize(size_t(ValueAt<uint64_t>(batch, kAvarCount, k)));
    rigExec::RigExecWriteAvarBindings(
        targets.Size(), [&](size_t i) { return targets.data[i]; },
        [&](size_t i) { return values[i]; },
        [&](uint32_t avar, double value) { out[avar] = value; });
    return true;
}

bool
RunAvarInputs(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return AvarInputsOne(batch, k, error);
    });
}

// ---- SpaceCheckpoint ----

enum SpaceCheckpointField : uint32_t { kNoScaleCount, kNoScale };

using CheckpointMath = rigExec::RrPoseMathT<rigExecRig::PointFrame>;

void
AssignRotOrder(uint32_t code, std::string *text)
{
    if (!code)
        return;
    const char letters[3] = {char(code & 0xff), char((code >> 8) & 0xff),
                             char((code >> 16) & 0xff)};
    text->assign(letters, 3);
}

const rigExec::RigExecPortHandle *const kCheckpointSlotPorts[] = {
    &SpaceCheckpointRotOrders(),
    &SpaceCheckpointRotationSigns(),
    &SpaceCheckpointPosedDefault(),
    &SpaceCheckpointParentDefaultInverse(),
    &SpaceCheckpointParentExpression(),
    &SpaceCheckpointParentExpressionAuthored(),
    &SpaceCheckpointPosedAuthored(),
    &SpaceCheckpointPosedAuthoredMatrix(),
};

bool
CheckSpaceCheckpoint(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t slots = ListAt<uint8_t>(check.batch, kNoScale, check.k).Size();
    bool fits =
        SameCount(CheckElements(check, SpaceCheckpointAvars()), slots * 11);
    for (const rigExec::RigExecPortHandle *port : kCheckpointSlotPorts)
        fits = fits && SameCount(CheckElements(check, *port), slots);
    if (!fits)
        return Fail(error, "SpaceCheckpoint",
                    "needs 11 avars and one of each order, sign, matrix and "
                    "flag per slot");
    return true;
}

/// Op state: one checkpoint input per slot.
using CheckpointInputs =
    std::vector<rigExec::RigExecSpaceCheckpointInputT<Mat4d>>;

bool
SpaceCheckpointOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto noScale = ListAt<uint8_t>(batch, kNoScale, k);
    const size_t slots = noScale.Size();
    const auto &avars = *In<RigExecArray<double>>(io, SpaceCheckpointAvars());
    const auto &orders =
        *In<RigExecArray<uint32_t>>(io, SpaceCheckpointRotOrders());
    const auto &signs =
        *In<RigExecArray<uint32_t>>(io, SpaceCheckpointRotationSigns());
    const auto &posedD =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointPosedDefault());
    const auto &parentDinv =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointParentDefaultInverse());
    const auto &parentM =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointParentExpression());
    const auto &parentAuthored = *In<RigExecArray<uint8_t>>(
        io, SpaceCheckpointParentExpressionAuthored());
    const auto &posedAuthored =
        *In<RigExecArray<uint8_t>>(io, SpaceCheckpointPosedAuthored());
    const auto &posedAuthoredM =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointPosedAuthoredMatrix());
    if (avars.Size() != slots * 11 || orders.Size() != slots ||
        signs.Size() != slots || posedD.Size() != slots ||
        parentDinv.Size() != slots || parentM.Size() != slots ||
        parentAuthored.Size() != slots || posedAuthored.Size() != slots ||
        posedAuthoredM.Size() != slots)
        return Fail(error, "SpaceCheckpoint",
                    "needs 11 avars and one of each order, sign, matrix and "
                    "flag per slot");
    auto &inputs =
        *static_cast<CheckpointInputs *>(rigExec::RigExecBatchState(batch, k));
    inputs.resize(slots);
    for (size_t s = 0; s < slots; ++s) {
        rigExec::eval::PoseSlotComposeT<CheckpointMath> view;
        view.avars11 = &avars[s * 11];
        AssignRotOrder(orders[s], &view.rotOrder);
        view.rotationSign = signs[s];
        view.noScaleAvars = noScale.data[s] != 0;
        view.posedAuthored = posedAuthored[s] != 0;
        view.posedAuthoredM = &posedAuthoredM[s];
        view.posedD = &posedD[s];
        view.parentDinv = &parentDinv[s];
        view.parentSpaceM = &parentM[s];
        view.parentSpaceAuthored = parentAuthored[s] != 0;
        inputs[s] = rigExec::eval::CheckpointInputOf<CheckpointMath>(view);
    }
    const Mat4d *anchor = In<Mat4d>(io, SpaceCheckpointAnchor());
    Out<Mat4d>(io, SpaceCheckpointOut()) =
        rigExec::eval::SpaceCheckpointMatrix<CheckpointMath>(
            inputs.data(), slots,
            anchor ? *anchor : CheckpointMath::Identity4());
    return true;
}

bool
RunSpaceCheckpoint(const RigExecOpBatch &batch, size_t *failed,
                   std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SpaceCheckpointOne(batch, k, error);
    });
}

// ---- PoseInterpolator ----

enum PoseInterpolatorField : uint32_t {
    kSolve,
    kPoseCount,
    kEnableTranslation,
    kAllowNegative,
    kExtraParentCount,
    kExtraParents,
    kRbfPoseCount,
    kRbfPoses,
    kRbfTranslationCount,
    kRbfTranslations,
    kRbfPoseTypeCount,
    kRbfPoseTypes,
    kRbfTwistAxis,
    kRbfKernel,
    kRbfRadius,
    kRbfTranslationRadius,
    kRbfRadiusCount,
    kRbfRadii,
    kRbfTranslationRadiusCount,
    kRbfTranslationRadii,
    kRbfNormalize,
    kRbfEnableRotation,
    kRbfEnableTranslation,
    kRbfWeightRows,
    kRbfWeightCount,
    kRbfWeights,
    kMethodSolverType,
    kMethodDistance,
    kMethodWeightFunction,
    kMethodNormalize,
    kMethodTwistAxis,
    kMethodRawCount,
    kMethodTargetCount,
    kMethodRadius,
    kMethodWeightThreshold,
    kMethodTargetsCount,
    kMethodTargets,
    kMethodDistanceTargetsCount,
    kMethodDistanceTargets,
    kMethodScaleCount,
    kMethodScales,
    kMethodCoefficientCount,
    kMethodCoefficients,
    kNetworkInputCount,
    kNetworkLayerCount,
    kNetworkLayerSizes,
    kNetworkActivationCount,
    kNetworkActivations,
    kNetworkAlphaCount,
    kNetworkAlphas,
    kNetworkWeightCount,
    kNetworkWeights,
    kNetworkBiasCount,
    kNetworkBiases,
    kNetworkMeanCount,
    kNetworkMean,
    kNetworkScaleCount,
    kNetworkScale,
    kNetworkDefaultCount,
    kNetworkDefaults,
    kMapsInputCount,
    kMapRowCount,
    kMapRowInputs,
    kMapFromCount,
    kMapFrom,
    kMapToCount,
    kMapTo,
    kMapSlopeCount,
    kMapSlope,
    kMapCutCount,
    kMapCut,
    kMapBeginCount,
    kMapBegin,
    kMapUnclampedCount,
    kMapUnclamped
};

using rigExecRig::PointFrame;
using InterpolatorMath = rigExec::RrPoseInterpolatorMathT<PointFrame>;

/// The tables a node's solve reads, built from its payload once per bound
/// program.
struct PoseInterpolatorState {
    bool built = false;
    rigExec::RrRbfSolver solver;
    rigExec::RigExecRbfMethodTable method;
    rigExec::RigExecMlNetwork network;
    rigExec::RigExecAnimatedMapsTable maps;
    std::vector<double> weights;
};

template <class T>
std::vector<T>
ListOf(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    const auto view = ListAt<T>(batch, field, k);
    return std::vector<T>(view.data, view.data + view.Size());
}

bool
BuildRbf(const RigExecOpBatch &batch, size_t k, PoseInterpolatorState &s,
         std::string *error)
{
    const auto poses = ListOf<rigExecStd::Vec3d>(batch, kRbfPoses, k);
    const auto weights = ListAt<double>(batch, kRbfWeights, k);
    const uint64_t rows = ValueAt<uint64_t>(batch, kRbfWeightRows, k);
    const auto types = ListAt<uint32_t>(batch, kRbfPoseTypes, k);
    if (rows != ValueAt<uint64_t>(batch, kPoseCount, k) ||
        weights.Size() != rows * poses.size())
        return Fail(error, "PoseInterpolator",
                    "needs one weight row per pose weight and one column per "
                    "pose");
    std::vector<rigExec::RrRbfPoseType> poseTypes;
    for (size_t i = 0; i < types.Size(); ++i) {
        if (types.data[i] > uint32_t(rigExec::RrRbfPoseType::Twist))
            return Fail(error, "PoseInterpolator",
                        "names an unknown RBF pose type");
        poseTypes.push_back(rigExec::RrRbfPoseType(types.data[i]));
    }
    s.solver = rigExec::RrRbfSolver(
        poses, ListOf<rigExecStd::Vec3d>(batch, kRbfTranslations, k), poseTypes,
        ValueAt<rigExecStd::Vec3d>(batch, kRbfTwistAxis, k),
        EnumAt<rigExec::RrRbfKernel>(batch, kRbfKernel, k),
        ValueAt<double>(batch, kRbfRadius, k),
        ValueAt<double>(batch, kRbfTranslationRadius, k),
        FlagAt(batch, kRbfNormalize, k), FlagAt(batch, kRbfEnableRotation, k),
        FlagAt(batch, kRbfEnableTranslation, k));
    std::vector<std::vector<double>> table{size_t(rows)};
    for (size_t r = 0; r < table.size(); ++r)
        table[r].assign(weights.data + r * poses.size(),
                        weights.data + (r + 1) * poses.size());
    s.solver.SetSolvedTable(ListOf<double>(batch, kRbfRadii, k),
                            ListOf<double>(batch, kRbfTranslationRadii, k),
                            table);
    return true;
}

bool
BuildMethod(const RigExecOpBatch &batch, size_t k, PoseInterpolatorState &s,
            std::string *error)
{
    rigExec::RigExecRbfMethodTable &table = s.method;
    table.solverType =
        EnumAt<rigExec::RigExecRbfSolverType>(batch, kMethodSolverType, k);
    table.distanceMethod =
        EnumAt<rigExec::RigExecRbfDistanceMethod>(batch, kMethodDistance, k);
    table.weightFunction = EnumAt<rigExec::RigExecRbfWeightFunction>(
        batch, kMethodWeightFunction, k);
    table.normalizeMethod =
        EnumAt<rigExec::RigExecRbfNormalizeMethod>(batch, kMethodNormalize, k);
    table.twistAxis = int(ValueAt<uint32_t>(batch, kMethodTwistAxis, k));
    table.rawCount = size_t(ValueAt<uint64_t>(batch, kMethodRawCount, k));
    table.targetCount = size_t(ValueAt<uint64_t>(batch, kMethodTargetCount, k));
    table.radius = ValueAt<double>(batch, kMethodRadius, k);
    table.weightThreshold = ValueAt<double>(batch, kMethodWeightThreshold, k);
    table.targets = ListOf<double>(batch, kMethodTargets, k);
    table.distanceTargets = ListOf<double>(batch, kMethodDistanceTargets, k);
    table.targetScales = ListOf<double>(batch, kMethodScales, k);
    table.coefficients = ListOf<double>(batch, kMethodCoefficients, k);
    const size_t targets = table.targetCount;
    if (targets != ValueAt<uint64_t>(batch, kPoseCount, k) ||
        table.targets.size() != targets * table.rawCount ||
        table.targetScales.size() != targets ||
        (table.distanceMethod == rigExec::RigExecRbfDistanceMethod::TwistAngle
             ? table.distanceTargets.size() != targets
             : !table.distanceTargets.empty()) ||
        (table.solverType == rigExec::RigExecRbfSolverType::Interpolative
             ? table.coefficients.size() != targets * targets
             : !table.coefficients.empty()))
        return Fail(error, "PoseInterpolator",
                    "has an RBF method table that does not fit its targets");
    return true;
}

bool
BuildNetwork(const RigExecOpBatch &batch, size_t k, PoseInterpolatorState &s,
             std::string *error)
{
    const auto sizes = ListOf<int32_t>(batch, kNetworkLayerSizes, k);
    const auto codes = ListAt<uint32_t>(batch, kNetworkActivations, k);
    std::vector<int> layerSizes(sizes.begin(), sizes.end());
    std::vector<int> activations(codes.data, codes.data + codes.Size());
    std::string why;
    if (!rigExec::RigExecMlBuildNetwork(
            size_t(ValueAt<uint64_t>(batch, kNetworkInputCount, k)), layerSizes,
            activations, ListOf<float>(batch, kNetworkAlphas, k),
            ListOf<float>(batch, kNetworkWeights, k),
            ListOf<float>(batch, kNetworkBiases, k), &s.network, &why))
        return Fail(error, "PoseInterpolator", why);
    s.network.inputMean = ListOf<float>(batch, kNetworkMean, k);
    s.network.inputScale = ListOf<float>(batch, kNetworkScale, k);
    s.network.defaults = ListOf<float>(batch, kNetworkDefaults, k);
    const size_t inputs = s.network.inputCount;
    const size_t outputs = s.network.OutputCount();
    if (outputs != ValueAt<uint64_t>(batch, kPoseCount, k) ||
        (!s.network.inputMean.empty() &&
         s.network.inputMean.size() != inputs) ||
        (!s.network.inputScale.empty() &&
         s.network.inputScale.size() != inputs) ||
        (!s.network.defaults.empty() && s.network.defaults.size() != outputs))
        return Fail(error, "PoseInterpolator",
                    "has a network whose tables do not fit its inputs and "
                    "outputs");
    return true;
}

bool
BuildMaps(const RigExecOpBatch &batch, size_t k, PoseInterpolatorState &s,
          std::string *error)
{
    const auto inputs = ListAt<int32_t>(batch, kMapRowInputs, k);
    const auto from = ListAt<double>(batch, kMapFrom, k);
    const auto to = ListAt<double>(batch, kMapTo, k);
    const auto slope = ListAt<double>(batch, kMapSlope, k);
    const auto cut = ListAt<double>(batch, kMapCut, k);
    const auto begin = ListAt<uint64_t>(batch, kMapBegin, k);
    const auto unclamped = ListAt<uint8_t>(batch, kMapUnclamped, k);
    const size_t rows = inputs.Size();
    const size_t maps = begin.Size() ? begin.Size() - 1 : 0;
    bool fits = from.Size() == rows && to.Size() == rows &&
                slope.Size() == rows && cut.Size() == rows && begin.Size() &&
                begin.data[0] == 0 && begin.data[maps] == rows &&
                maps == ValueAt<uint64_t>(batch, kPoseCount, k) &&
                (unclamped.Size() == 0 || unclamped.Size() == maps);
    for (size_t m = 0; fits && m < maps; ++m)
        fits = begin.data[m] <= begin.data[m + 1];
    if (!fits)
        return Fail(error, "PoseInterpolator",
                    "has map rows that do not fit its maps");
    rigExec::RigExecAnimatedMapsTable &table = s.maps;
    table.inputCount = size_t(ValueAt<uint64_t>(batch, kMapsInputCount, k));
    for (size_t m = 0; m < maps; ++m) {
        std::vector<rigExec::RigExecConditionalRow> mapRows;
        for (size_t r = size_t(begin.data[m]); r < size_t(begin.data[m + 1]);
             ++r)
            mapRows.push_back({inputs.data[r], from.data[r], to.data[r],
                               slope.data[r], cut.data[r]});
        if (!table.AddMap(std::move(mapRows),
                          !(m < unclamped.Size() && unclamped.data[m] != 0)))
            return Fail(error, "PoseInterpolator",
                        "has a map row naming an input past inputCount");
    }
    return true;
}

bool
BuildPoseInterpolator(const RigExecOpBatch &batch, size_t k,
                      PoseInterpolatorState &s, std::string *error)
{
    switch (EnumAt<PoseSolve>(batch, kSolve, k)) {
    case PoseSolve::Rbf:
        return BuildRbf(batch, k, s, error);
    case PoseSolve::RbfMethod:
        return BuildMethod(batch, k, s, error);
    case PoseSolve::Network:
        return BuildNetwork(batch, k, s, error);
    case PoseSolve::Maps:
        return BuildMaps(batch, k, s, error);
    }
    return Fail(error, "PoseInterpolator", "names an unknown solve");
}

size_t
ExtraParentCount(const RigExecOpBatch &batch, size_t k)
{
    const auto parents = ListAt<uint8_t>(batch, kExtraParents, k);
    size_t count = 0;
    for (size_t e = 0; e < parents.Size(); ++e)
        count += parents.data[e] != 0;
    return count;
}

bool
CheckPoseInterpolator(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const RigExecOpBatch &batch = check.batch;
    const size_t k = check.k;
    PoseInterpolatorState scratch;
    if (!BuildPoseInterpolator(batch, k, scratch, error))
        return false;
    const PoseSolve solve = EnumAt<PoseSolve>(batch, kSolve, k);
    const bool driver = Fed(check, PoseInterpolatorDriverFinal());
    const bool numeric = Fed(check, PoseInterpolatorNumeric());
    if (driver != Fed(check, PoseInterpolatorDriverRest()) ||
        Fed(check, PoseInterpolatorParentFinal()) !=
            Fed(check, PoseInterpolatorParentRest()) ||
        (Fed(check, PoseInterpolatorParentFinal()) && !driver))
        return Fail(error, "PoseInterpolator",
                    "needs its driver and parent frames fed in final and "
                    "rest pairs");
    const size_t extras = ListAt<uint8_t>(batch, kExtraParents, k).Size();
    const size_t extraParents = ExtraParentCount(batch, k);
    const bool extraFeeds =
        solve == PoseSolve::RbfMethod || solve == PoseSolve::Network;
    if ((extras && !extraFeeds) ||
        !SameCount(CheckElements(check, PoseInterpolatorExtraFinals()),
                   extras) ||
        !SameCount(CheckElements(check, PoseInterpolatorExtraRests()),
                   extras) ||
        !SameCount(CheckElements(check, PoseInterpolatorExtraParentFinals()),
                   extraParents) ||
        !SameCount(CheckElements(check, PoseInterpolatorExtraParentRests()),
                   extraParents) ||
        (extras && !driver))
        return Fail(error, "PoseInterpolator",
                    "needs one extra driver frame per extra driver and one "
                    "extra parent frame per extra driver with a parent");
    const size_t values = CheckElements(check, PoseInterpolatorNumeric());
    const size_t known = numeric ? values : 0;
    switch (solve) {
    case PoseSolve::Rbf:
    case PoseSolve::RbfMethod:
        if (!numeric && !driver)
            return Fail(error, "PoseInterpolator",
                        "needs a driver frame or numeric values");
        if (solve == PoseSolve::RbfMethod && !numeric &&
            scratch.method.rawCount != 4 * (1 + extras))
            return Fail(error, "PoseInterpolator",
                        "measures four values per driver frame, not its "
                        "method's raw count");
        if (solve == PoseSolve::RbfMethod && numeric &&
            !SameCount(values, scratch.method.rawCount))
            return Fail(error, "PoseInterpolator",
                        "needs one numeric value per method raw value");
        break;
    case PoseSolve::Network:
        if (!SameCount(known, rigExec::kRigExecPortUnfed) &&
            (driver ? 4 * (1 + extras) : 0) + known !=
                scratch.network.inputCount)
            return Fail(error, "PoseInterpolator",
                        "needs a row of the network's input count");
        break;
    case PoseSolve::Maps:
        if (!numeric || !SameCount(values, scratch.maps.inputCount))
            return Fail(error, "PoseInterpolator",
                        "needs one numeric value per map input");
        break;
    }
    return true;
}

bool
PoseInterpolatorOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    auto &state = *static_cast<PoseInterpolatorState *>(
        rigExec::RigExecBatchState(batch, k));
    if (!state.built) {
        if (!BuildPoseInterpolator(batch, k, state, error))
            return false;
        state.built = true;
    }
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const size_t poseCount = size_t(ValueAt<uint64_t>(batch, kPoseCount, k));
    auto &out = Out<RigExecArray<float>>(io, PoseInterpolatorOut());
    out.Resize(poseCount);
    for (size_t i = 0; i < poseCount; ++i)
        out[i] = 0.0f;
    rigExec::RigExecPoseInterpolatorInputsT<PointFrame> input;
    input.enabled = InOr(io, PoseInterpolatorEnabled(), uint8_t(1)) != 0;
    if (const auto *numeric =
            In<RigExecArray<double>>(io, PoseInterpolatorNumeric()))
        input.numeric.assign(numeric->begin(), numeric->end());
    input.mask = InOr(io, PoseInterpolatorMask(), 1.0);
    input.driverFinal = In<PointFrame>(io, PoseInterpolatorDriverFinal());
    input.driverRest = In<PointFrame>(io, PoseInterpolatorDriverRest());
    input.parentFinal = In<PointFrame>(io, PoseInterpolatorParentFinal());
    input.parentRest = In<PointFrame>(io, PoseInterpolatorParentRest());
    const auto parents = ListAt<uint8_t>(batch, kExtraParents, k);
    if (parents.Size()) {
        const auto &finals =
            *In<RigExecArray<PointFrame>>(io, PoseInterpolatorExtraFinals());
        const auto &rests =
            *In<RigExecArray<PointFrame>>(io, PoseInterpolatorExtraRests());
        const auto *parentFinals = In<RigExecArray<PointFrame>>(
            io, PoseInterpolatorExtraParentFinals());
        const auto *parentRests = In<RigExecArray<PointFrame>>(
            io, PoseInterpolatorExtraParentRests());
        const size_t withParent = ExtraParentCount(batch, k);
        if (finals.Size() != parents.Size() || rests.Size() != parents.Size() ||
            (withParent && (!parentFinals || !parentRests ||
                            parentFinals->Size() != withParent ||
                            parentRests->Size() != withParent)))
            return Fail(error, "PoseInterpolator",
                        "needs one extra driver frame per extra driver");
        input.extraDrivers.resize(parents.Size());
        for (size_t e = 0, p = 0; e < parents.Size(); ++e) {
            auto &extra = input.extraDrivers[e];
            extra.final = &finals[e];
            extra.rest = &rests[e];
            if (parents.data[e]) {
                extra.parentFinal = &(*parentFinals)[p];
                extra.parentRest = &(*parentRests)[p];
                ++p;
            }
        }
    }
    const PoseSolve solve = EnumAt<PoseSolve>(batch, kSolve, k);
    rigExec::RigExecPoseInterpolatorRecordT<
        rigExec::RrRbfSolver &, const rigExec::RigExecRbfMethodTable &,
        const rigExec::RigExecMlNetwork *,
        const rigExec::RigExecAnimatedMapsTable *>
        record{state.solver,
               FlagAt(batch, kEnableTranslation, k),
               FlagAt(batch, kAllowNegative, k),
               poseCount,
               solve == PoseSolve::RbfMethod,
               state.method,
               solve == PoseSolve::Network,
               &state.network,
               solve == PoseSolve::Maps,
               &state.maps};
    using Status = rigExec::RigExecPoseInterpolatorStatus;
    switch (rigExec::RigExecRunPoseInterpolatorArithmetic<InterpolatorMath>(
        record, input, &state.weights)) {
    case Status::Disabled:
        return true;
    case Status::UnusableRotation:
        return Fail(error, "PoseInterpolator",
                    "has no usable frame for its driver");
    case Status::UnusableTranslation:
        return Fail(error, "PoseInterpolator",
                    "could not measure its driver's translation");
    case Status::CountMismatch:
        return Fail(error, "PoseInterpolator",
                    "solved " + std::to_string(state.weights.size()) +
                        " weights for " + std::to_string(poseCount) + " poses");
    case Status::Success:
        break;
    }
    for (size_t i = 0; i < poseCount; ++i)
        out[i] = static_cast<float>(state.weights[i]);
    return true;
}

bool
RunPoseInterpolator(const RigExecOpBatch &batch, size_t *failed,
                    std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return PoseInterpolatorOne(batch, k, error);
    });
}

// ---- RestCompose / LadderCompose / ComposeSlot ----

using SlotMath = CheckpointMath;
using PointFrame = rigExecRig::PointFrame;

enum SlotActivityField : uint32_t { kActive };
enum ComposeSlotField : uint32_t { kSlotActive, kSlotNoScale };

PointFrame
FrameOf(const std::array<rigExecStd::Vec3d, 4> &points, unsigned flags)
{
    PointFrame frame;
    frame.points = points;
    frame.flags = flags;
    return frame;
}

PointFrame
UnavailableFrame()
{
    return FrameOf(rigExec::eval::pose_ops::IdentityLandmarks<SlotMath>(), 0);
}

bool
CheckRestCompose(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, RestComposeRestAvars()), 6))
        return Fail(error, "RestCompose", "needs six rest avars");
    return true;
}

bool
RestComposeOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    Mat4d &restM = Out<Mat4d>(io, RestComposeRestMatrix());
    PointFrame &rest = Out<PointFrame>(io, RestComposeRest());
    Mat4d &roundTrip = Out<Mat4d>(io, RestComposeRoundTrip());
    if (!FlagAt(batch, kActive, k)) {
        restM = rigExec::eval::UnavailableSpace<SlotMath>();
        rest = UnavailableFrame();
        roundTrip = rigExec::eval::UnavailableSpace<SlotMath>();
        return true;
    }
    const auto &avars = *In<RigExecArray<double>>(io, RestComposeRestAvars());
    if (avars.Size() != 6)
        return Fail(error, "RestCompose", "needs six rest avars");
    const Mat4d identity = SlotMath::Identity4();
    const auto composed = rigExec::eval::RestComposeSlot<SlotMath>(
        &avars[0], *In<Mat4d>(io, RestComposeRestSpace()),
        InOr(io, RestComposeIntervening(), identity),
        InOr(io, RestComposeParentRest(), identity));
    restM = composed.restM;
    rest = FrameOf(composed.points, composed.flags);
    roundTrip = composed.roundTrip;
    return true;
}

bool
RunRestCompose(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return RestComposeOne(batch, k, error);
    });
}

bool
CheckLadderCompose(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, LadderComposeSpaces()), 6) ||
        !SameCount(CheckElements(check, LadderComposeAuthored()), 6) ||
        !SameCount(CheckElements(check, LadderComposeDefaultAvars()), 6))
        return Fail(error, "LadderCompose",
                    "needs six channel spaces, authored flags and default "
                    "avars");
    if (Fed(check, LadderComposeParentRest()) !=
        Fed(check, LadderComposeParentDefault()))
        return Fail(error, "LadderCompose",
                    "needs its parent's rest and default round trips fed "
                    "together");
    return true;
}

bool
LadderComposeOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    Mat4d &selfD = Out<Mat4d>(io, LadderComposeSelfDefault());
    Mat4d &parentDinv = Out<Mat4d>(io, LadderComposeParentDefaultInverse());
    Mat4d &defaultRoundTrip = Out<Mat4d>(io, LadderComposeDefaultRoundTrip());
    Mat4d &posedD = Out<Mat4d>(io, LadderComposePosedDefault());
    Mat4d &parentSpaceM = Out<Mat4d>(io, LadderComposeParentSpace());
    Mat4d &posedM = Out<Mat4d>(io, LadderComposePosedAuthoredMatrix());
    uint8_t &posedAuthored = Out<uint8_t>(io, LadderComposePosedAuthored());
    uint8_t &parentAuthored =
        Out<uint8_t>(io, LadderComposeParentSpaceAuthored());
    uint32_t &sign = Out<uint32_t>(io, LadderComposeOutRotationSign());
    if (!FlagAt(batch, kActive, k)) {
        const Mat4d unavailable = rigExec::eval::UnavailableSpace<SlotMath>();
        selfD = parentDinv = defaultRoundTrip = posedD = parentSpaceM = posedM =
            unavailable;
        posedAuthored = parentAuthored = 0;
        sign = 0;
        return true;
    }
    const auto &spaces = *In<RigExecArray<Mat4d>>(io, LadderComposeSpaces());
    const auto &authored =
        *In<RigExecArray<uint8_t>>(io, LadderComposeAuthored());
    const auto &avars =
        *In<RigExecArray<double>>(io, LadderComposeDefaultAvars());
    if (spaces.Size() != 6 || authored.Size() != 6 || avars.Size() != 6)
        return Fail(error, "LadderCompose",
                    "needs six channel spaces, authored flags and default "
                    "avars");
    rigExec::eval::PoseLadderInputT<SlotMath> in;
    in.spaces[rigExec::eval::kPoseLadderRestSpace] = SlotMath::Identity4();
    for (size_t c = 0; c < 6; ++c) {
        in.spaces[c + 1] = spaces[c];
        in.authored[c + 1] = authored[c] != 0;
        in.defaultAvars[c] = avars[c];
    }
    in.parentSpaceLeaf = *In<Mat4d>(io, LadderComposeParentSpaceLeaf());
    in.rotationSign = *In<rigExecStd::Vec3d>(io, LadderComposeRotationSign());
    in.ownRest = *In<Mat4d>(io, LadderComposeOwnRest());
    in.parentRest = In<Mat4d>(io, LadderComposeParentRest());
    in.parentDefault = In<Mat4d>(io, LadderComposeParentDefault());
    const auto ladder = rigExec::eval::LadderComposeSlot<SlotMath>(in);
    selfD = ladder.selfD;
    parentDinv = ladder.parentDinv;
    defaultRoundTrip = ladder.defaultRoundTrip;
    posedD = ladder.posedD;
    parentSpaceM = ladder.parentSpaceM;
    posedM = ladder.posed;
    posedAuthored = ladder.posedAuthored ? 1 : 0;
    parentAuthored = ladder.parentSpaceAuthored ? 1 : 0;
    sign = ladder.rotationSign;
    return true;
}

bool
RunLadderCompose(const RigExecOpBatch &batch, size_t *failed,
                 std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return LadderComposeOne(batch, k, error);
    });
}

bool
CheckComposeSlot(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, ComposeSlotAvars()), 11))
        return Fail(error, "ComposeSlot", "needs 11 avars");
    return true;
}

bool
ComposeSlotOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    PointFrame &base = Out<PointFrame>(io, ComposeSlotBase());
    Mat4d &posedM = Out<Mat4d>(io, ComposeSlotPosedMatrix());
    if (!FlagAt(batch, kSlotActive, k)) {
        base = UnavailableFrame();
        posedM = rigExec::eval::UnavailableSpace<SlotMath>();
        return true;
    }
    const auto &avars = *In<RigExecArray<double>>(io, ComposeSlotAvars());
    if (avars.Size() != 11)
        return Fail(error, "ComposeSlot", "needs 11 avars");
    rigExec::eval::PoseSlotComposeT<SlotMath> view;
    view.avars11 = &avars[0];
    AssignRotOrder(InOr(io, ComposeSlotRotOrder(), uint32_t(0)),
                   &view.rotOrder);
    view.rotationSign = *In<uint32_t>(io, ComposeSlotRotationSign());
    view.noScaleAvars = FlagAt(batch, kSlotNoScale, k);
    view.posedAuthored = *In<uint8_t>(io, ComposeSlotPosedAuthored()) != 0;
    view.posedAuthoredM = In<Mat4d>(io, ComposeSlotPosedAuthoredMatrix());
    view.posedD = In<Mat4d>(io, ComposeSlotPosedDefault());
    view.parentDinv = In<Mat4d>(io, ComposeSlotParentDefaultInverse());
    view.parentSpaceM = In<Mat4d>(io, ComposeSlotParentSpace());
    view.parentSpaceAuthored =
        *In<uint8_t>(io, ComposeSlotParentSpaceAuthored()) != 0;
    std::array<rigExecStd::Vec3d, 4> points = {
        SlotMath::Vec3Of(0, 0, 0), SlotMath::Vec3Of(0, 0, 0),
        SlotMath::Vec3Of(0, 0, 0), SlotMath::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    rigExec::eval::pose_ops::FrameFromMatrix<SlotMath>(
        rigExec::eval::ComposeUnswitchedMatrix<SlotMath>(
            view, InOr(io, ComposeSlotParentPosed(), SlotMath::Identity4())),
        &points, &flags);
    base = FrameOf(points, flags);
    posedM = rigExec::eval::pose_ops::SpaceOfFrame<SlotMath>(points, flags);
    return true;
}

bool
RunComposeSlot(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ComposeSlotOne(batch, k, error);
    });
}

// ---- SwitchedComposeSlot / ShiftedComposeSlot ----

enum SwitchedComposeSlotField : uint32_t {
    kSwitchedActive = spaceSwitchFields::kSpaceSwitchFieldCount,
    kSwitchedNoScale,
};

struct SwitchedSlotPorts {
    const rigExec::RigExecPortHandle &posedDefault =
        SwitchedComposeSlotPosedDefault();
    const rigExec::RigExecPortHandle &parentDefaultInverse =
        SwitchedComposeSlotParentDefaultInverse();
    const rigExec::RigExecPortHandle &parentPosed =
        SwitchedComposeSlotParentPosed();
    const rigExec::RigExecPortHandle &parentDefault =
        SwitchedComposeSlotParentDefault();
    const rigExec::RigExecPortHandle &spaceDefault =
        SwitchedComposeSlotSpaceDefault();
    const rigExec::RigExecPortHandle &spacePosed =
        SwitchedComposeSlotSpacePosed();
    const rigExec::RigExecPortHandle &sourceDefaults =
        SwitchedComposeSlotSourceDefaults();
    const rigExec::RigExecPortHandle &sourcePosed =
        SwitchedComposeSlotSourcePosed();
};

bool
CheckSwitchedComposeSlot(const rigExec::RigExecOpCheck &check,
                         std::string *error)
{
    const auto setup = spaceSwitchFields::SwitchAt(check.batch, check.k);
    const size_t count = setup.world.Size();
    if (!count)
        return Fail(error, "SwitchedComposeSlot", "needs at least one space");
    if (setup.filters.Size() && setup.filters.Size() != count)
        return Fail(error, "SwitchedComposeSlot",
                    "needs one filter per space or none");
    if (!SameCount(CheckElements(check, SwitchedComposeSlotAvars()), 11))
        return Fail(error, "SwitchedComposeSlot", "needs 11 avars");
    if (Fed(check, SwitchedComposeSlotSpaceDefault()) != setup.hasCarry ||
        Fed(check, SwitchedComposeSlotSpacePosed()) != setup.hasCarry)
        return Fail(error, "SwitchedComposeSlot",
                    "needs its carry space fed exactly when it has one");
    if (!SameCount(CheckElements(check, SwitchedComposeSlotSourceDefaults()),
                   count) ||
        !SameCount(CheckElements(check, SwitchedComposeSlotSourcePosed()),
                   count))
        return Fail(error, "SwitchedComposeSlot",
                    "needs one source default and posed space per space");
    return true;
}

void
PublishSlotFrame(const Mat4d &matrix, PointFrame &base, Mat4d &posedM)
{
    std::array<rigExecStd::Vec3d, 4> points = {
        SlotMath::Vec3Of(0, 0, 0), SlotMath::Vec3Of(0, 0, 0),
        SlotMath::Vec3Of(0, 0, 0), SlotMath::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    rigExec::eval::pose_ops::FrameFromMatrix<SlotMath>(matrix, &points, &flags);
    base = FrameOf(points, flags);
    posedM = rigExec::eval::pose_ops::SpaceOfFrame<SlotMath>(points, flags);
}

bool
SwitchedComposeSlotOne(const RigExecOpBatch &batch, size_t k,
                       std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    PointFrame &base = Out<PointFrame>(io, SwitchedComposeSlotBase());
    Mat4d &posedM = Out<Mat4d>(io, SwitchedComposeSlotPosedMatrix());
    if (!FlagAt(batch, kSwitchedActive, k)) {
        base = UnavailableFrame();
        posedM = rigExec::eval::UnavailableSpace<SlotMath>();
        return true;
    }
    if (*In<uint8_t>(io, SwitchedComposeSlotPosedAuthored()) != 0) {
        PublishSlotFrame(
            *In<Mat4d>(io, SwitchedComposeSlotPosedAuthoredMatrix()), base,
            posedM);
        return true;
    }
    const auto &avars =
        *In<RigExecArray<double>>(io, SwitchedComposeSlotAvars());
    if (avars.Size() != 11)
        return Fail(error, "SwitchedComposeSlot", "needs 11 avars");
    rigExec::eval::PoseSlotComposeT<SlotMath> view;
    view.avars11 = &avars[0];
    AssignRotOrder(InOr(io, SwitchedComposeSlotRotOrder(), uint32_t(0)),
                   &view.rotOrder);
    view.rotationSign = *In<uint32_t>(io, SwitchedComposeSlotRotationSign());
    view.noScaleAvars = FlagAt(batch, kSwitchedNoScale, k);
    const auto setup = spaceSwitchFields::SwitchAt(batch, k);
    spaceSwitchFields::SwitchInputs input;
    if (!spaceSwitchFields::SwitchInputsOf(io, setup, SwitchedSlotPorts(),
                                           &input))
        return Fail(error, "SwitchedComposeSlot",
                    "needs one source default and posed space per space");
    input.avars = rigExec::eval::ComposeSlotAvars<SlotMath>(view);
    input.active = *In<double>(io, SwitchedComposeSlotActive());
    Mat4d matrix = SlotMath::Identity4();
    if (!rigExec::RigExecRunSpaceSwitchArithmetic<
            spaceSwitchFields::SwitchMath>(spaceSwitchFields::RecordOf(setup),
                                           input, &matrix))
        return Fail(error, "SwitchedComposeSlot", "has no space to switch to");
    PublishSlotFrame(matrix, base, posedM);
    return true;
}

bool
RunSwitchedComposeSlot(const RigExecOpBatch &batch, size_t *failed,
                       std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SwitchedComposeSlotOne(batch, k, error);
    });
}

bool
ShiftedComposeSlotOne(const RigExecOpBatch &batch, size_t k)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    PointFrame &out = Out<PointFrame>(io, ShiftedComposeSlotOut());
    const auto &delta = *In<rigExecStd::Vec3d>(io, ShiftedComposeSlotDelta());
    if (delta[0] == 0 && delta[1] == 0 && delta[2] == 0) {
        out = *In<PointFrame>(io, ShiftedComposeSlotBase());
        Out<Mat4d>(io, ShiftedComposeSlotPosedMatrix()) =
            rigExec::eval::pose_ops::SpaceOfFrame<SlotMath>(out.points,
                                                            out.flags);
        return true;
    }
    PublishSlotFrame(*In<Mat4d>(io, ShiftedComposeSlotShifted()), out,
                     Out<Mat4d>(io, ShiftedComposeSlotPosedMatrix()));
    return true;
}

bool
RunShiftedComposeSlot(const RigExecOpBatch &batch, size_t *failed,
                      std::string *)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ShiftedComposeSlotOne(batch, k);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kAvarInputsPorts[] = {
    {&AvarInputsValues(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "values"},
    {&AvarInputsOut(), &rigExecStd::DoubleArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Field, nullptr, nullptr, kAvarCount},
};

void
AvarInputsValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const AvarInputsPayload *>(payload);
    out.Set(kAvarCount, p.avarCount);
    SetList(out, kTargetCount, p.targets);
}

const rigExec::RigExecFieldDesc kAvarInputsFields[] = {
    Value("avarCount", rigExecStd::UInt64Type()),
    Count("targetCount"),
    Items("targets", rigExecStd::UIntType(), kTargetCount),
};

const rigExec::RigExecOpDesc kAvarInputsDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "AvarInputs";
    d.ports = kAvarInputsPorts;
    d.portCount = std::size(kAvarInputsPorts);
    d.runBatch = &RunAvarInputs;
    d.validate = &CheckAvarInputs;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kAvarInputsFields;
    d.payloadFieldCount = std::size(kAvarInputsFields);
    d.payloadVersion = 1;
    d.payloadValues = &AvarInputsValuesOf;
    return d;
}();

const rigExec::RigExecPortDesc kSpaceCheckpointPorts[] = {
    {&SpaceCheckpointAvars(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "avars"},
    {&SpaceCheckpointRotOrders(), &rigExecStd::UIntArray(), kRigExecPortInput,
     "rotOrders"},
    {&SpaceCheckpointRotationSigns(), &rigExecStd::UIntArray(),
     kRigExecPortInput, "rotationSigns"},
    {&SpaceCheckpointPosedDefault(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "posedDefault"},
    {&SpaceCheckpointParentDefaultInverse(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "parentDefaultInverse"},
    {&SpaceCheckpointParentExpression(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "parentExpression"},
    {&SpaceCheckpointParentExpressionAuthored(), &rigExecStd::BoolArray(),
     kRigExecPortInput, "parentExpressionAuthored"},
    {&SpaceCheckpointPosedAuthored(), &rigExecStd::BoolArray(),
     kRigExecPortInput, "posedAuthored"},
    {&SpaceCheckpointPosedAuthoredMatrix(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "posedAuthoredMatrix"},
    {&SpaceCheckpointAnchor(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "anchor"},
    {&SpaceCheckpointOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "out"},
};

void
SpaceCheckpointValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SpaceCheckpointPayload *>(payload);
    SetList(out, kNoScaleCount, p.noScale);
}

const rigExec::RigExecFieldDesc kSpaceCheckpointFields[] = {
    Count("noScaleCount"),
    Items("noScale", rigExecStd::BoolType(), kNoScaleCount),
};

const rigExec::RigExecOpDesc kSpaceCheckpointDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SpaceCheckpoint";
    d.ports = kSpaceCheckpointPorts;
    d.portCount = std::size(kSpaceCheckpointPorts);
    d.runBatch = &RunSpaceCheckpoint;
    d.validate = &CheckSpaceCheckpoint;
    d.state = opIo::StateOf<CheckpointInputs>();
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kSpaceCheckpointFields;
    d.payloadFieldCount = std::size(kSpaceCheckpointFields);
    d.payloadVersion = 1;
    d.payloadValues = &SpaceCheckpointValuesOf;
    return d;
}();

constexpr uint32_t kOptionalInput = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPoseInterpolatorPorts[] = {
    {&PoseInterpolatorEnabled(), &rigExecStd::BoolType(), kOptionalInput,
     "enabled"},
    {&PoseInterpolatorNumeric(), &rigExecStd::DoubleArray(), kOptionalInput,
     "numeric"},
    {&PoseInterpolatorMask(), &rigExecStd::DoubleType(), kOptionalInput,
     "mask"},
    {&PoseInterpolatorDriverFinal(), &rigExecRig::PointFrameType(),
     kOptionalInput, "driverFinal"},
    {&PoseInterpolatorDriverRest(), &rigExecRig::PointFrameType(),
     kOptionalInput, "driverRest"},
    {&PoseInterpolatorParentFinal(), &rigExecRig::PointFrameType(),
     kOptionalInput, "parentFinal"},
    {&PoseInterpolatorParentRest(), &rigExecRig::PointFrameType(),
     kOptionalInput, "parentRest"},
    {&PoseInterpolatorExtraFinals(), &rigExecRig::PointFrameList(),
     kOptionalInput, "extraFinals"},
    {&PoseInterpolatorExtraRests(), &rigExecRig::PointFrameList(),
     kOptionalInput, "extraRests"},
    {&PoseInterpolatorExtraParentFinals(), &rigExecRig::PointFrameList(),
     kOptionalInput, "extraParentFinals"},
    {&PoseInterpolatorExtraParentRests(), &rigExecRig::PointFrameList(),
     kOptionalInput, "extraParentRests"},
    {&PoseInterpolatorOut(), &rigExecStd::FloatArray(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Field, nullptr, nullptr, kPoseCount},
};

void
PoseInterpolatorValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const PoseInterpolatorPayload *>(payload);
    SetEnum(out, kSolve, p.solve);
    out.Set(kPoseCount, p.poseCount);
    SetFlag(out, kEnableTranslation, p.enableTranslation);
    SetFlag(out, kAllowNegative, p.allowNegativeWeights);
    SetList(out, kExtraParentCount, p.extraParents);
    SetList(out, kRbfPoseCount, p.rbfPoses);
    SetList(out, kRbfTranslationCount, p.rbfTranslations);
    SetList(out, kRbfPoseTypeCount, p.rbfPoseTypes);
    out.Set(kRbfTwistAxis, p.rbfTwistAxis);
    out.Set(kRbfKernel, p.rbfKernel);
    out.Set(kRbfRadius, p.rbfRadius);
    out.Set(kRbfTranslationRadius, p.rbfTranslationRadius);
    SetList(out, kRbfRadiusCount, p.rbfRadii);
    SetList(out, kRbfTranslationRadiusCount, p.rbfTranslationRadii);
    SetFlag(out, kRbfNormalize, p.rbfNormalize);
    SetFlag(out, kRbfEnableRotation, p.rbfEnableRotation);
    SetFlag(out, kRbfEnableTranslation, p.rbfEnableTranslation);
    out.Set(kRbfWeightRows, p.rbfWeightRows);
    SetList(out, kRbfWeightCount, p.rbfWeights);
    out.Set(kMethodSolverType, p.methodSolverType);
    out.Set(kMethodDistance, p.methodDistance);
    out.Set(kMethodWeightFunction, p.methodWeightFunction);
    out.Set(kMethodNormalize, p.methodNormalize);
    out.Set(kMethodTwistAxis, p.methodTwistAxis);
    out.Set(kMethodRawCount, p.methodRawCount);
    out.Set(kMethodTargetCount, p.methodTargetCount);
    out.Set(kMethodRadius, p.methodRadius);
    out.Set(kMethodWeightThreshold, p.methodWeightThreshold);
    SetList(out, kMethodTargetsCount, p.methodTargets);
    SetList(out, kMethodDistanceTargetsCount, p.methodDistanceTargets);
    SetList(out, kMethodScaleCount, p.methodTargetScales);
    SetList(out, kMethodCoefficientCount, p.methodCoefficients);
    out.Set(kNetworkInputCount, p.networkInputCount);
    SetList(out, kNetworkLayerCount, p.networkLayerSizes);
    SetList(out, kNetworkActivationCount, p.networkActivations);
    SetList(out, kNetworkAlphaCount, p.networkAlphas);
    SetList(out, kNetworkWeightCount, p.networkWeights);
    SetList(out, kNetworkBiasCount, p.networkBiases);
    SetList(out, kNetworkMeanCount, p.networkInputMean);
    SetList(out, kNetworkScaleCount, p.networkInputScale);
    SetList(out, kNetworkDefaultCount, p.networkDefaults);
    out.Set(kMapsInputCount, p.mapsInputCount);
    SetList(out, kMapRowCount, p.mapRowInputs);
    SetList(out, kMapFromCount, p.mapRowFrom);
    SetList(out, kMapToCount, p.mapRowTo);
    SetList(out, kMapSlopeCount, p.mapRowSlope);
    SetList(out, kMapCutCount, p.mapRowCut);
    SetList(out, kMapBeginCount, p.mapBegin);
    SetList(out, kMapUnclampedCount, p.mapUnclamped);
}

const rigExec::RigExecFieldDesc kPoseInterpolatorFields[] = {
    Enum("solve", PoseSolve::Maps),
    Value("poseCount", rigExecStd::UInt64Type()),
    Flag("enableTranslation"),
    Flag("allowNegativeWeights"),
    Count("extraParentCount"),
    Items("extraParents", rigExecStd::BoolType(), kExtraParentCount),
    Count("rbfPoseCount"),
    Items("rbfPoses", rigExecStd::Vec3dType(), kRbfPoseCount),
    Count("rbfTranslationCount"),
    Items("rbfTranslations", rigExecStd::Vec3dType(), kRbfTranslationCount),
    Count("rbfPoseTypeCount"),
    Items("rbfPoseTypes", rigExecStd::UIntType(), kRbfPoseTypeCount),
    Value("rbfTwistAxis", rigExecStd::Vec3dType()),
    Enum("rbfKernel", rigExec::RrRbfKernel::Quintic),
    Value("rbfRadius", rigExecStd::DoubleType()),
    Value("rbfTranslationRadius", rigExecStd::DoubleType()),
    Count("rbfRadiusCount"),
    Items("rbfRadii", rigExecStd::DoubleType(), kRbfRadiusCount),
    Count("rbfTranslationRadiusCount"),
    Items("rbfTranslationRadii", rigExecStd::DoubleType(),
          kRbfTranslationRadiusCount),
    Flag("rbfNormalize"),
    Flag("rbfEnableRotation"),
    Flag("rbfEnableTranslation"),
    Value("rbfWeightRows", rigExecStd::UInt64Type()),
    Count("rbfWeightCount"),
    Items("rbfWeights", rigExecStd::DoubleType(), kRbfWeightCount),
    Enum("methodSolverType", rigExec::RigExecRbfSolverType::Interpolative),
    Enum("methodDistance", rigExec::RigExecRbfDistanceMethod::TwistAngle),
    Enum("methodWeightFunction", rigExec::RigExecRbfWeightFunction::Quintic),
    Enum("methodNormalize", rigExec::RigExecRbfNormalizeMethod::Always),
    Enum("methodTwistAxis", 2u),
    Value("methodRawCount", rigExecStd::UInt64Type()),
    Value("methodTargetCount", rigExecStd::UInt64Type()),
    Value("methodRadius", rigExecStd::DoubleType()),
    Value("methodWeightThreshold", rigExecStd::DoubleType()),
    Count("methodTargetsCount"),
    Items("methodTargets", rigExecStd::DoubleType(), kMethodTargetsCount),
    Count("methodDistanceTargetsCount"),
    Items("methodDistanceTargets", rigExecStd::DoubleType(),
          kMethodDistanceTargetsCount),
    Count("methodScaleCount"),
    Items("methodTargetScales", rigExecStd::DoubleType(), kMethodScaleCount),
    Count("methodCoefficientCount"),
    Items("methodCoefficients", rigExecStd::DoubleType(),
          kMethodCoefficientCount),
    Value("networkInputCount", rigExecStd::UInt64Type()),
    Count("networkLayerCount"),
    Items("networkLayerSizes", rigExecStd::IntType(), kNetworkLayerCount),
    Count("networkActivationCount"),
    Items("networkActivations", rigExecStd::UIntType(),
          kNetworkActivationCount),
    Count("networkAlphaCount"),
    Items("networkAlphas", rigExecStd::FloatType(), kNetworkAlphaCount),
    Count("networkWeightCount"),
    Items("networkWeights", rigExecStd::FloatType(), kNetworkWeightCount),
    Count("networkBiasCount"),
    Items("networkBiases", rigExecStd::FloatType(), kNetworkBiasCount),
    Count("networkMeanCount"),
    Items("networkInputMean", rigExecStd::FloatType(), kNetworkMeanCount),
    Count("networkScaleCount"),
    Items("networkInputScale", rigExecStd::FloatType(), kNetworkScaleCount),
    Count("networkDefaultCount"),
    Items("networkDefaults", rigExecStd::FloatType(), kNetworkDefaultCount),
    Value("mapsInputCount", rigExecStd::UInt64Type()),
    Count("mapRowCount"),
    Items("mapRowInputs", rigExecStd::IntType(), kMapRowCount),
    Count("mapFromCount"),
    Items("mapRowFrom", rigExecStd::DoubleType(), kMapFromCount),
    Count("mapToCount"),
    Items("mapRowTo", rigExecStd::DoubleType(), kMapToCount),
    Count("mapSlopeCount"),
    Items("mapRowSlope", rigExecStd::DoubleType(), kMapSlopeCount),
    Count("mapCutCount"),
    Items("mapRowCut", rigExecStd::DoubleType(), kMapCutCount),
    Count("mapBeginCount"),
    Items("mapBegin", rigExecStd::UInt64Type(), kMapBeginCount),
    Count("mapUnclampedCount"),
    Items("mapUnclamped", rigExecStd::BoolType(), kMapUnclampedCount),
};

const rigExec::RigExecOpDesc kPoseInterpolatorDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "PoseInterpolator";
    d.ports = kPoseInterpolatorPorts;
    d.portCount = std::size(kPoseInterpolatorPorts);
    d.runBatch = &RunPoseInterpolator;
    d.validate = &CheckPoseInterpolator;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kPoseInterpolatorFields;
    d.payloadFieldCount = std::size(kPoseInterpolatorFields);
    d.payloadVersion = 1;
    d.payloadValues = &PoseInterpolatorValuesOf;
    d.state = opIo::StateOf<PoseInterpolatorState>();
    return d;
}();

const rigExec::RigExecPortDesc kRestComposePorts[] = {
    {&RestComposeRestAvars(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "restAvars"},
    {&RestComposeRestSpace(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "restSpace"},
    {&RestComposeIntervening(), &rigExecStd::Mat4dType(), kOptionalInput,
     "intervening"},
    {&RestComposeParentRest(), &rigExecStd::Mat4dType(), kOptionalInput,
     "parentRest"},
    {&RestComposeRestMatrix(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "restMatrix"},
    {&RestComposeRest(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "rest"},
    {&RestComposeRoundTrip(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "roundTrip"},
};

void
SlotActivityValuesOf(const void *payload, RigExecFieldValues &out)
{
    SetFlag(out, kActive,
            static_cast<const SlotActivityPayload *>(payload)->active);
}

const rigExec::RigExecFieldDesc kSlotActivityFields[] = {Flag("active")};

rigExec::RigExecOpDesc
SlotOpDesc(const char *name, const rigExec::RigExecPortDesc *ports,
           size_t portCount, rigExec::RigExecOpBatchFn run,
           bool (*validate)(const rigExec::RigExecOpCheck &, std::string *),
           const rigExec::RigExecFieldDesc *fields, size_t fieldCount,
           void (*values)(const void *, RigExecFieldValues &))
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = run;
    d.validate = validate;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = fields;
    d.payloadFieldCount = fieldCount;
    d.payloadVersion = 1;
    d.payloadValues = values;
    return d;
}

const rigExec::RigExecOpDesc kRestComposeDesc =
    SlotOpDesc("RestCompose", kRestComposePorts, std::size(kRestComposePorts),
               &RunRestCompose, &CheckRestCompose, kSlotActivityFields,
               std::size(kSlotActivityFields), &SlotActivityValuesOf);

const rigExec::RigExecPortDesc kLadderComposePorts[] = {
    {&LadderComposeSpaces(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "spaces"},
    {&LadderComposeAuthored(), &rigExecStd::BoolArray(), kRigExecPortInput,
     "authored"},
    {&LadderComposeParentSpaceLeaf(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "parentSpaceLeaf"},
    {&LadderComposeRotationSign(), &rigExecStd::Vec3dType(), kRigExecPortInput,
     "rotationSign"},
    {&LadderComposeDefaultAvars(), &rigExecStd::DoubleArray(),
     kRigExecPortInput, "defaultAvars"},
    {&LadderComposeOwnRest(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "ownRest"},
    {&LadderComposeParentRest(), &rigExecStd::Mat4dType(), kOptionalInput,
     "parentRest"},
    {&LadderComposeParentDefault(), &rigExecStd::Mat4dType(), kOptionalInput,
     "parentDefault"},
    {&LadderComposeSelfDefault(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "selfDefault"},
    {&LadderComposeParentDefaultInverse(), &rigExecStd::Mat4dType(),
     kRigExecPortOutput, "parentDefaultInverse"},
    {&LadderComposeDefaultRoundTrip(), &rigExecStd::Mat4dType(),
     kRigExecPortOutput, "defaultRoundTrip"},
    {&LadderComposePosedDefault(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "posedDefault"},
    {&LadderComposeParentSpace(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "parentSpace"},
    {&LadderComposePosedAuthoredMatrix(), &rigExecStd::Mat4dType(),
     kRigExecPortOutput, "posedAuthoredMatrix"},
    {&LadderComposePosedAuthored(), &rigExecStd::BoolType(), kRigExecPortOutput,
     "posedAuthored"},
    {&LadderComposeParentSpaceAuthored(), &rigExecStd::BoolType(),
     kRigExecPortOutput, "parentSpaceAuthored"},
    {&LadderComposeOutRotationSign(), &rigExecStd::UIntType(),
     kRigExecPortOutput, "rotationSignMask"},
};

const rigExec::RigExecOpDesc kLadderComposeDesc = SlotOpDesc(
    "LadderCompose", kLadderComposePorts, std::size(kLadderComposePorts),
    &RunLadderCompose, &CheckLadderCompose, kSlotActivityFields,
    std::size(kSlotActivityFields), &SlotActivityValuesOf);

const rigExec::RigExecPortDesc kComposeSlotPorts[] = {
    {&ComposeSlotAvars(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "avars"},
    {&ComposeSlotRotOrder(), &rigExecStd::UIntType(), kOptionalInput,
     "rotOrder"},
    {&ComposeSlotRotationSign(), &rigExecStd::UIntType(), kRigExecPortInput,
     "rotationSign"},
    {&ComposeSlotPosedAuthored(), &rigExecStd::BoolType(), kRigExecPortInput,
     "posedAuthored"},
    {&ComposeSlotPosedAuthoredMatrix(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "posedAuthoredMatrix"},
    {&ComposeSlotPosedDefault(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "posedDefault"},
    {&ComposeSlotParentDefaultInverse(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "parentDefaultInverse"},
    {&ComposeSlotParentSpace(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "parentSpace"},
    {&ComposeSlotParentSpaceAuthored(), &rigExecStd::BoolType(),
     kRigExecPortInput, "parentSpaceAuthored"},
    {&ComposeSlotParentPosed(), &rigExecStd::Mat4dType(), kOptionalInput,
     "parentPosed"},
    {&ComposeSlotBase(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "base"},
    {&ComposeSlotPosedMatrix(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "posedMatrix"},
};

void
ComposeSlotValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const ComposeSlotPayload *>(payload);
    SetFlag(out, kSlotActive, p.active);
    SetFlag(out, kSlotNoScale, p.noScale);
}

const rigExec::RigExecFieldDesc kComposeSlotFields[] = {Flag("active"),
                                                        Flag("noScale")};

const rigExec::RigExecOpDesc kComposeSlotDesc =
    SlotOpDesc("ComposeSlot", kComposeSlotPorts, std::size(kComposeSlotPorts),
               &RunComposeSlot, &CheckComposeSlot, kComposeSlotFields,
               std::size(kComposeSlotFields), &ComposeSlotValuesOf);

const rigExec::RigExecPortDesc kSwitchedComposeSlotPorts[] = {
    {&SwitchedComposeSlotAvars(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "avars"},
    {&SwitchedComposeSlotRotOrder(), &rigExecStd::UIntType(), kOptionalInput,
     "rotOrder"},
    {&SwitchedComposeSlotRotationSign(), &rigExecStd::UIntType(),
     kRigExecPortInput, "rotationSign"},
    {&SwitchedComposeSlotPosedAuthored(), &rigExecStd::BoolType(),
     kRigExecPortInput, "posedAuthored"},
    {&SwitchedComposeSlotPosedAuthoredMatrix(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "posedAuthoredMatrix"},
    {&SwitchedComposeSlotPosedDefault(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "posedDefault"},
    {&SwitchedComposeSlotParentDefaultInverse(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "parentDefaultInverse"},
    {&SwitchedComposeSlotParentPosed(), &rigExecStd::Mat4dType(),
     kOptionalInput, "parentPosed"},
    {&SwitchedComposeSlotParentDefault(), &rigExecStd::Mat4dType(),
     kOptionalInput, "parentDefault"},
    {&SwitchedComposeSlotActive(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "active"},
    {&SwitchedComposeSlotSpaceDefault(), &rigExecStd::Mat4dType(),
     kOptionalInput, "spaceDefault"},
    {&SwitchedComposeSlotSpacePosed(), &rigExecStd::Mat4dType(), kOptionalInput,
     "spacePosed"},
    {&SwitchedComposeSlotSourceDefaults(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "sourceDefaults"},
    {&SwitchedComposeSlotSourcePosed(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "sourcePosed"},
    {&SwitchedComposeSlotBase(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "base"},
    {&SwitchedComposeSlotPosedMatrix(), &rigExecStd::Mat4dType(),
     kRigExecPortOutput, "posedMatrix"},
};

void
SwitchedComposeSlotValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SwitchedComposeSlotPayload *>(payload);
    spaceSwitchFields::SetSwitch(out, p.space);
    SetFlag(out, kSwitchedActive, p.active);
    SetFlag(out, kSwitchedNoScale, p.noScale);
}

const rigExec::RigExecFieldDesc kSwitchedComposeSlotFields[] = {
    Count("filterCount"),
    Items("filters", rigExecStd::IntType(), spaceSwitchFields::kFilterCount),
    Value("twistAxis", rigExecStd::Vec3dType()),
    Fixed(Flag("affectTranslation"), 3),
    Fixed(Flag("affectRotation"), 3),
    Fixed(Flag("affectScale"), 3),
    Flag("hasCarry"),
    Count("worldCount"),
    Items("world", rigExecStd::BoolType(), spaceSwitchFields::kWorldCount),
    Flag("active"),
    Flag("noScale"),
};

const rigExec::RigExecOpDesc kSwitchedComposeSlotDesc = SlotOpDesc(
    "SwitchedComposeSlot", kSwitchedComposeSlotPorts,
    std::size(kSwitchedComposeSlotPorts), &RunSwitchedComposeSlot,
    &CheckSwitchedComposeSlot, kSwitchedComposeSlotFields,
    std::size(kSwitchedComposeSlotFields), &SwitchedComposeSlotValuesOf);

const rigExec::RigExecPortDesc kShiftedComposeSlotPorts[] = {
    {&ShiftedComposeSlotBase(), &rigExecRig::PointFrameType(),
     kRigExecPortInput, "base"},
    {&ShiftedComposeSlotDelta(), &rigExecStd::Vec3dType(), kRigExecPortInput,
     "delta"},
    {&ShiftedComposeSlotShifted(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "shifted"},
    {&ShiftedComposeSlotOut(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "out"},
    {&ShiftedComposeSlotPosedMatrix(), &rigExecStd::Mat4dType(),
     kRigExecPortOutput, "posedMatrix"},
};

const rigExec::RigExecOpDesc kShiftedComposeSlotDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "ShiftedComposeSlot";
    d.ports = kShiftedComposeSlotPorts;
    d.portCount = std::size(kShiftedComposeSlotPorts);
    d.runBatch = &RunShiftedComposeSlot;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    return d;
}();

} // namespace

void
RegisterPoseOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(AvarInputs(), kAvarInputsDesc);
    registrar.Add(SpaceCheckpoint(), kSpaceCheckpointDesc);
    registrar.Add(PoseInterpolator(), kPoseInterpolatorDesc);
    registrar.Add(RestCompose(), kRestComposeDesc);
    registrar.Add(LadderCompose(), kLadderComposeDesc);
    registrar.Add(ComposeSlot(), kComposeSlotDesc);
    registrar.Add(SwitchedComposeSlot(), kSwitchedComposeSlotDesc);
    registrar.Add(ShiftedComposeSlot(), kShiftedComposeSlotDesc);
}

} // namespace rigExecOps
