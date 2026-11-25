// Every op payload saves and reads back equal, and bytes that cannot be a
// saved payload are refused without reading past them.
#include "rigExec/api/opFields.h"
#include "rigExec/api/registry.h"
#include "rigExecOps/constraint/autoClavicleOps.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/constraint/harmonicsOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/deform/blendShapeOp.h"
#include "rigExecOps/deform/clothOp.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/derivedOps.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/layeredSkinOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/patchOp.h"
#include "rigExecOps/deform/projectorOps.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceBindingOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wireOp.h"
#include "rigExecOps/deform/wrinkleOp.h"
#include "rigExecOps/property/mathOps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/poseCommitOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/solver/skeleton2dOps.h"
#include "rigExecOps/solver/splineIkOps.h"
#include "rigExecOps/weight/weightOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecOps;
using rigExecStd::Mat4d;
using rigExecStd::Vec2f;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

/// One constructed payload of an op, destroyed with it.
class Box {
public:
    explicit Box(const RigExecOpPayloadHandling &h)
        : _h(h), _storage(std::make_unique<std::max_align_t[]>(
                     h.size / sizeof(std::max_align_t) + 1))
    {
        _h.construct(Data());
    }
    Box(const RigExecOpPayloadHandling &h, const void *src)
        : _h(h), _storage(std::make_unique<std::max_align_t[]>(
                     h.size / sizeof(std::max_align_t) + 1))
    {
        _h.copy(Data(), src);
    }
    ~Box()
    {
        _h.destroy(Data());
    }
    Box(const Box &) = delete;
    Box &operator=(const Box &) = delete;
    void *Data()
    {
        return _storage.get();
    }

private:
    const RigExecOpPayloadHandling &_h;
    std::unique_ptr<std::max_align_t[]> _storage;
};

bool
Decodes(const RigExecOpPayloadHandling &h, const std::vector<uint8_t> &bytes,
        std::string *error = nullptr)
{
    Box box(h);
    return h.decode(bytes.data(), bytes.size(), box.Data(), error);
}

bool
FieldsDecode(const RigExecOpDesc &op, const std::vector<uint8_t> &bytes,
             std::string *error = nullptr)
{
    RigExecFieldValues values;
    return RigExecDecodePayloadFields(op, bytes.data(), bytes.size(), &values,
                                      error);
}

/// The declared-field form of CheckRoundTrip, on \p values.
void
CheckFieldsRoundTrip(const RigExecOpDesc &op, const RigExecFieldValues &values)
{
    const std::string name = op.name;
    std::string error;
    CHECK(RigExecCheckPayloadFields(op, values, &error));
    std::vector<uint8_t> bytes;
    RigExecEncodePayloadFields(op, values, bytes);
    CHECK(bytes.size() >= 4);
    RigExecFieldValues decoded;
    const bool ok = RigExecDecodePayloadFields(op, bytes.data(), bytes.size(),
                                               &decoded, &error);
    if (!ok)
        std::printf("%s: %s\n", name.c_str(), error.c_str());
    CHECK(ok && decoded == values);
    std::vector<uint8_t> again;
    RigExecEncodePayloadFields(op, decoded, again);
    CHECK(again == bytes);
    for (size_t n = 0; n < bytes.size(); ++n) {
        const std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + n);
        if (FieldsDecode(op, cut)) {
            std::printf("%s: %zu of %zu bytes decoded\n", name.c_str(), n,
                        bytes.size());
            CHECK(false);
        }
    }
    std::vector<uint8_t> longer = bytes;
    longer.push_back(0);
    CHECK(!FieldsDecode(op, longer, &error));
    CHECK(error.find("left after") != std::string::npos);
    std::vector<uint8_t> newer = bytes;
    newer[0] = uint8_t(newer[0] + 1);
    CHECK(!FieldsDecode(op, newer, &error));
    CHECK(error.find("version") != std::string::npos);
    RigExecFieldValues none;
    CHECK(!RigExecDecodePayloadFields(op, nullptr, 0, &none, &error));
}

/// Zero for every one-value and fixed field, and empty lists.
RigExecFieldValues
DefaultFields(const RigExecOpDesc &op)
{
    RigExecFieldValues values(op.payloadFieldCount);
    for (size_t f = 0; f < op.payloadFieldCount; ++f) {
        const RigExecFieldDesc &field = op.payloadFields[f];
        const size_t size = RigExecRegistry::Get().Type(*field.type).desc->size;
        size_t count = 0;
        if (field.extent == RigExecFieldExtent::Scalar)
            count = 1;
        else if (field.extent == RigExecFieldExtent::Fixed)
            count = field.count;
        const std::vector<uint8_t> zero(count * size, 0);
        values.SetBytes(f, zero.data(), count, size);
    }
    return values;
}

/// Saves \p payload, reads it back, and checks the result is equal and
/// saves to the same bytes; then checks damaged copies are refused.
void
CheckRoundTrip(const RigExecRegisteredOp &op, const void *payload)
{
    if (op.desc->payloadFieldCount) {
        std::vector<uint8_t> bytes;
        CHECK(RigExecEncodeOpPayload(*op.desc, payload, bytes));
        RigExecFieldValues values(op.desc->payloadFieldCount);
        op.desc->payloadValues(payload, values);
        CheckFieldsRoundTrip(*op.desc, values);
        std::vector<uint8_t> direct;
        RigExecEncodePayloadFields(*op.desc, values, direct);
        CHECK(direct == bytes);
        return;
    }
    const RigExecOpPayloadHandling &h = op.desc->payload;
    const std::string name = op.desc->name;
    std::vector<uint8_t> bytes;
    h.encode(payload, bytes);
    CHECK(bytes.size() >= 4);

    Box decoded(h);
    std::string error;
    const bool ok =
        h.decode(bytes.data(), bytes.size(), decoded.Data(), &error);
    if (!ok)
        std::printf("%s: %s\n", name.c_str(), error.c_str());
    CHECK(ok);
    CHECK(h.equal && h.equal(payload, decoded.Data()));
    std::vector<uint8_t> again;
    h.encode(decoded.Data(), again);
    CHECK(again == bytes);

    // A copy made through the handling saves the same bytes.
    Box copied(h, payload);
    std::vector<uint8_t> copiedBytes;
    h.encode(copied.Data(), copiedBytes);
    CHECK(copiedBytes == bytes);

    for (size_t n = 0; n < bytes.size(); ++n) {
        const std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + n);
        if (Decodes(h, cut)) {
            std::printf("%s: %zu of %zu bytes decoded\n", name.c_str(), n,
                        bytes.size());
            CHECK(false);
        }
    }
    std::vector<uint8_t> longer = bytes;
    longer.push_back(0);
    CHECK(!Decodes(h, longer, &error));
    CHECK(error.find("left after") != std::string::npos);

    std::vector<uint8_t> newer = bytes;
    newer[0] = uint8_t(newer[0] + 1);
    CHECK(!Decodes(h, newer, &error));
    CHECK(error.find("version") != std::string::npos);

    CHECK(!h.decode(nullptr, 0, Box(h).Data(), &error));
}

const RigExecRegisteredOp &
Op(const RigExecOpHandle &handle)
{
    return RigExecRegistry::Get().Op(handle);
}

void
TestEveryOp()
{
    const RigExecRegistry &registry = RigExecRegistry::Get();
    size_t withPayload = 0;
    for (size_t i = 0; i < registry.OpCount(); ++i) {
        const RigExecRegisteredOp &op = registry.OpAt(uint32_t(i));
        const RigExecOpPayloadHandling &h = op.desc->payload;
        if (op.desc->payloadFieldCount) {
            ++withPayload;
            CheckFieldsRoundTrip(*op.desc, DefaultFields(*op.desc));
            continue;
        }
        if (!h.size)
            continue;
        ++withPayload;
        CHECK(h.encode && h.decode && h.equal);
        if (!h.encode || !h.decode || !h.equal)
            continue;
        Box defaults(h);
        CheckRoundTrip(op, defaults.Data());
    }
    CHECK(withPayload >= 25);
}

std::shared_ptr<const BlendShapeLayout>
SparseLayout()
{
    auto layout = std::make_shared<BlendShapeLayout>();
    layout->offsets = {Vec3f(0.5f, -0.0f, 1.0f), Vec3f(2.0f, 3.0f, -4.0f)};
    layout->indices = {3, 7};
    layout->pointCount = 9;
    layout->valid = true;
    return layout;
}

void
TestFilledPayloads()
{
    {
        AutoClavicleOpPayload p;
        p.constants.basis[4] = -0.0;
        p.constants.kernel = 1;
        p.constants.normalize = false;
        p.constants.swings = {1.0, 0.0, 0.25, -0.5};
        p.constants.widths = {0.75};
        p.constants.gains = {2.0};
        p.constants.weights = {1.5};
        p.hasLimb = true;
        CheckRoundTrip(Op(AutoClavicle()), &p);
    }
    {
        ConstraintPayload p;
        p.offset = Vec3d(1.0, -2.0, 3.5);
        p.affect = {true, false, true};
        p.scaleAxes = {true, true, true};
        p.translationOffsets = {Vec3d(1.0, 2.0, 3.0), Vec3d(-1.0, 0.0, 0.5)};
        p.rotationOffsetsDegrees = {Vec3d(90.0, 0.0, -45.0)};
        p.rotationOrder = ConstraintEulerOrder::ZYX;
        p.blendShear = true;
        p.localAimVector = Vec3d(0.0, 0.0, 1.0);
        p.preserveInputUp = false;
        CheckRoundTrip(Op(ParentConstraint()), &p);
        CheckRoundTrip(Op(AimConstraint()), &p);
    }
    {
        HarmonicsPayload p;
        p.settings.waves = 3.0;
        p.settings.frequency = 1.25;
        p.settings.ignoreFirstFrame = false;
        p.hasSpace = true;
        p.frames = 17;
        CheckRoundTrip(Op(Harmonics()), &p);
    }
    {
        IkFkMatchPayload p;
        p.poleSwitch.filters = {0, 3, 1};
        p.poleSwitch.twistAxis = Vec3d(0.0, 1.0, 0.0);
        p.poleSwitch.affectRotation = {{true, false, true}};
        p.poleSwitch.hasCarry = true;
        p.poleSwitch.world = {1, 0, 1};
        p.poleSourceRides = {0, 1, 1};
        p.ikValue = 0.0;
        p.upperLengthOffset = 0.125;
        p.stretchLimb = false;
        CheckRoundTrip(Op(IkFkMatch()), &p);
        SpaceMatchPayload s;
        s.space = p.poleSwitch;
        CheckRoundTrip(Op(SpaceMatch()), &s);
    }
    {
        BlendShapePayload p;
        BlendShapeChannel dense;
        BlendShapeSample half;
        half.activation = 0.5f;
        half.points = {Vec3f(1.0f, 2.0f, 3.0f)};
        BlendShapeSample full;
        full.layout = SparseLayout();
        dense.samples = {half, full};
        p.channels = {dense, BlendShapeChannel()};
        p.surfaceFrame = true;
        CheckRoundTrip(Op(BlendShape()), &p);
    }
    {
        DeltaMushPayload p;
        p.iterations = 4;
        p.step = 0.25;
        p.pinBorders = false;
        p.settings.smoothing = RigExecDeltaMushSmoothing::LengthWeighted;
        p.settings.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        p.settings.smoothWeights = {0.0f, 1.0f, 0.5f};
        p.settings.edges = {0, 1, 1, 2};
        p.settings.onlySmooth = true;
        CheckRoundTrip(Op(DeltaMush()), &p);
    }
    {
        NormalsPayload p;
        p.perCorner = true;
        CheckRoundTrip(Op(RecomputeNormals()), &p);
    }
    {
        LatticePayload p;
        p.divisions = {3, 4, 5};
        p.settings.regularGrid = true;
        p.settings.interpolation[1] = RigExecLatticeInterpolation::CatmullRom;
        p.settings.origin = {1.0f, 2.0f, 3.0f};
        p.settings.strength = 0.5f;
        p.settings.mask = {1.0f, 0.0f};
        CheckRoundTrip(Op(Lattice()), &p);
    }
    {
        MatrixDeformPayload p;
        p.radial = true;
        CheckRoundTrip(Op(MatrixDeform()), &p);
    }
    {
        SurfaceProjectorPayload p;
        p.rayOrigin = Vec3d(1.0, 2.0, 3.0);
        p.shaderOffset[3][0] = 4.0;
        p.reproject = true;
        p.spaceNamed = true;
        CheckRoundTrip(Op(SurfaceProjector()), &p);
    }
    {
        SkinPayload p;
        p.method = SkinMethod::DualQuaternion;
        p.elementSize = 4;
        CheckRoundTrip(Op(Skin()), &p);
    }
    {
        SurfaceProjectPayload p;
        p.settings.mode = RigExecSurfaceSnapMode::OutsideSurface;
        p.settings.offset = 0.01f;
        p.settings.mask = {0.5f};
        p.settings.triangles = {0, 1, 2};
        CheckRoundTrip(Op(SurfaceProject()), &p);
    }
    {
        WirePayload p;
        p.order = 3;
        p.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
        p.dropoffDistance = 2.5;
        CheckRoundTrip(Op(Wire()), &p);
    }
    {
        WrinklePayload p;
        p.settings.iterations = 12;
        p.settings.topology = RigExecWrinkleTopology::SurfaceStruts;
        p.settings.bendStiffness = 0.3f;
        p.settings.tangentPlaneCollisions = false;
        p.settings.pinPoints = {4, 9};
        CheckRoundTrip(Op(Wrinkle()), &p);
    }
    {
        MathPayload p;
        p.op = RigExecPropertyOp::Path;
        p.path.valid = true;
        p.path.basis = RigExecPathBasis::CatmullRom;
        p.path.component = RigExecPathComponent::Heading;
        p.path.headingOffset = 90.0f;
        CheckRoundTrip(Op(FloatMath()), &p);
    }
    {
        FkChainPayload fk;
        fk.parentIndices = {-1, 0, 1};
        fk.scaleSegments = true;
        CheckRoundTrip(Op(FkChain()), &fk);
        TwoBoneIkPayload ik;
        ik.softDistancePolicy = true;
        ik.scaleCalibration = -0.25;
        CheckRoundTrip(Op(TwoBoneIk()), &ik);
        FkJointGroupsPayload groups;
        groups.jointBegin = 1;
        groups.parents = {-1, 0, 0};
        groups.groups = {{0, 2, true}, {2, 1, false}};
        groups.members = {1, 2, 0};
        CheckRoundTrip(Op(FkJointGroups()), &groups);
        PoseCommitPayload commit;
        commit.slotCount = 2;
        commit.closest = {0, -1, 1};
        commit.blocked = {0, 1, 0};
        commit.solverOutput = true;
        CheckRoundTrip(Op(PoseCommit()), &commit);
    }
    {
        BlendFramesPayload p;
        p.logScale = true;
        CheckRoundTrip(Op(BlendFrames()), &p);
        SampleRibbonFramesPayload r;
        r.sampleCount = 9;
        CheckRoundTrip(Op(SampleRibbonFrames()), &r);
        SingleChainIkPayload s;
        s.preserveJointOrientation = true;
        CheckRoundTrip(Op(SingleChainIk()), &s);
        SplineIkPayload sp;
        sp.restLengthChain = true;
        sp.minLengthRatio = 0.75;
        CheckRoundTrip(Op(SplineIk()), &sp);
    }
    {
        Skeleton2dPayload p;
        p.order = {2, 0, 1};
        RigExecBone2dSetup bone;
        bone.parent = 1;
        bone.flags = 9;
        bone.scaling = RigExecBone2dScaling::Uniform;
        bone.tipX = 2.0;
        bone.spaceCount = 2;
        bone.angleController = 0;
        bone.scaleFactor = 0.5;
        p.setup = {RigExecBone2dSetup(), bone};
        p.spaceSources = {-1, 3};
        p.targetCandidates = {1};
        RigExecJointGroup group;
        group.memberCount = 2;
        group.enabled = false;
        p.groups = {group};
        p.groupMembers = {0, 2};
        CheckRoundTrip(Op(Skeleton2d()), &p);
    }
    {
        WeightFieldPayload p;
        p.falloffCurve = {1.0f, 0.5f, 0.0f};
        p.clamp = true;
        p.planeAxis = WeightFieldAxis::X;
        p.planeBounded = true;
        CheckRoundTrip(Op(PlaneWeight()), &p);
        CombineWeightsPayload c;
        c.mode = WeightCombineMode::Overlay;
        c.inputs = WeightCombineInputs::Fields;
        c.clamp = true;
        CheckRoundTrip(Op(CombineWeights()), &c);
    }
    {
        PatchPayload p;
        p.uVertexCount = 7;
        p.vVertexCount = -2;
        p.uOrder = 4;
        p.vOrder = 16;
        p.uPeriodic = true;
        CheckRoundTrip(Op(Patch()), &p);
    }
    {
        SurfaceBindingPayload p;
        p.limit = true;
        p.framed = true;
        CheckRoundTrip(Op(SurfaceBinding()), &p);
    }
    {
        LayeredSkinPayload p;
        p.elementSize = 4;
        p.dualQuaternion = true;
        CheckRoundTrip(Op(LayeredSkin()), &p);
    }
    {
        ClothPayload p;
        rigExec::RigExecClothCompiledData &c = p.compiledRig;
        c.pointCount = 3;
        c.counts = {3};
        c.indices = {0, 1, 2};
        c.leftHanded = true;
        c.rest = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        c.pins = {1, 0, 0.5f};
        c.settings.substeps = 7;
        c.frameSeconds = 1.0 / 30.0;
        rigExec::RigExecClothColliderTopology collider;
        collider.path = "/Box.points";
        collider.pointCount = 3;
        collider.counts = {3};
        collider.indices = {2, 1, 0};
        c.colliders.push_back(collider);
        CheckRoundTrip(Op(Cloth()), &p);
    }
}

std::vector<uint8_t>
Hex(const char *text)
{
    const auto nibble = [](char h) {
        return uint8_t(h <= '9' ? h - '0' : h - 'a' + 10);
    };
    std::vector<uint8_t> bytes;
    std::string digits;
    for (const char *c = text; *c; ++c)
        if (*c != ' ')
            digits += *c;
    for (size_t i = 0; i + 1 < digits.size(); i += 2)
        bytes.push_back(
            uint8_t(nibble(digits[i]) << 4 | nibble(digits[i + 1])));
    return bytes;
}

/// The bytes the math ops' declared fields save: the operation, then the
/// path settings.
void
TestMathBytes()
{
    const RigExecOpDesc &op = *Op(FloatMath()).desc;
    MathPayload p;
    std::vector<uint8_t> bytes;
    CHECK(RigExecEncodeOpPayload(op, &p, bytes));
    CHECK(bytes == Hex("02000000 00000000 "
                       "00 00000000 00 00 00000000 00000000 00000000 "
                       "00000000"));
    p.op = RigExecPropertyOp::Path;
    p.path.valid = true;
    p.path.basis = RigExecPathBasis::CatmullRom;
    p.path.component = RigExecPathComponent::Heading;
    p.path.headingOffset = 90.0f;
    CHECK(RigExecEncodeOpPayload(op, &p, bytes));
    CHECK(bytes == Hex("02000000 07000000 "
                       "01 03000000 00 00 03000000 00000000 00000000 "
                       "0000b442"));
    for (const RigExecOpHandle *math :
         {&Vec3fMath(), &MatrixMath(), &DoubleMath()}) {
        std::vector<uint8_t> same;
        CHECK(RigExecEncodeOpPayload(*Op(*math).desc, &p, same));
        CHECK(same == bytes);
    }

    // An operation past the last one is refused.
    std::vector<uint8_t> bad = bytes;
    bad[4] = 8;
    std::string error;
    CHECK(!FieldsDecode(op, bad, &error));
    CHECK(error.find("field op: a value is out of range") != std::string::npos);
    bad = bytes;
    bad[8] = 2;
    CHECK(!FieldsDecode(op, bad, &error));
    CHECK(error.find("pathValid") != std::string::npos);
    // Bytes a version 1 payload saved, keys and tangents among its settings,
    // are refused.
    bad = bytes;
    bad[0] = 1;
    CHECK(!FieldsDecode(op, bad, &error));
}

std::string
HexOf(const std::vector<uint8_t> &bytes)
{
    static const char digits[] = "0123456789abcdef";
    std::string text;
    for (const uint8_t b : bytes) {
        text += digits[b >> 4];
        text += digits[b & 15];
    }
    return text;
}

/// \p payload of \p op saves exactly \p hex, the bytes its hand-written
/// save wrote before the op declared its fields.
void
CheckSaved(const RigExecOpHandle &op, const void *payload, const char *hex)
{
    std::vector<uint8_t> bytes;
    CHECK(RigExecEncodeOpPayload(*Op(op).desc, payload, bytes));
    const bool same = bytes == Hex(hex);
    if (!same)
        std::printf("%s saves %s\n", Op(op).desc->name, HexOf(bytes).c_str());
    CHECK(same);
}

/// The solver, constraint and weight payloads save the bytes they saved
/// before their fields were declared, at defaults and filled.
void
TestSavedBytes()
{
    {
        FkChainPayload fk;
        CheckSaved(FkChain(), &fk, "01000000000000000000000000");
        fk.parentIndices = {-1, 0, 1};
        fk.scaleSegments = true;
        CheckSaved(FkChain(), &fk,
                   "010000000300000000000000ffffffff000000000100000001");
        TwoBoneIkPayload ik;
        CheckSaved(TwoBoneIk(), &ik, "0100000000000000000000000000");
        ik.softDistancePolicy = true;
        ik.scaleCalibration = -0.25;
        CheckSaved(TwoBoneIk(), &ik, "010000000100000000000000d0bf");
    }
    {
        BlendFramesPayload p;
        CheckSaved(BlendFrames(), &p, "0100000000");
        p.logScale = true;
        CheckSaved(BlendFrames(), &p, "0100000001");
        SampleRibbonFramesPayload r;
        CheckSaved(SampleRibbonFrames(), &r, "0100000002000000");
        r.sampleCount = -9;
        CheckSaved(SampleRibbonFrames(), &r, "01000000f7ffffff");
        SingleChainIkPayload s;
        CheckSaved(SingleChainIk(), &s, "010000000000");
        s.preserveJointOrientation = true;
        CheckSaved(SingleChainIk(), &s, "010000000001");
        SplineIkPayload sp;
        CheckSaved(SplineIk(), &sp, "0100000000000000000000000000");
        sp.restLengthChain = true;
        sp.minLengthRatio = 0.75;
        sp.aimRootTangent = true;
        CheckSaved(SplineIk(), &sp, "0100000001000000000000e83f01");
    }
    {
        ConstraintPayload p;
        CheckSaved(PositionConstraint(), &p,
                   "01000000000000000000000000000000000000000000000000000000"
                   "01010101010101010100000000000000000000000000000000000000"
                   "0000000000000000000000f03f000000000000000000000000000000"
                   "000000000000000000000000000000f03f000000000000000001");
        p.offset = Vec3d(1.0, -2.0, 3.5);
        p.affect = {true, false, true};
        p.scaleAxes = {true, true, true};
        p.translationOffsets = {Vec3d(1.0, 2.0, 3.0), Vec3d(-1.0, 0.0, 0.5)};
        p.rotationOffsetsDegrees = {Vec3d(90.0, 0.0, -45.0)};
        p.rotationOrder = ConstraintEulerOrder::ZYX;
        p.blendShear = true;
        p.localAimVector = Vec3d(0.0, 0.0, 1.0);
        p.preserveInputUp = false;
        for (const RigExecOpHandle *op :
             {&PositionConstraint(), &RotationConstraint(), &ScaleConstraint(),
              &ParentConstraint(), &AimConstraint()})
            CheckSaved(
                *op, &p,
                "01000000000000000000f03f00000000000000c00000000000000c40"
                "0100010101010101010101010200000000000000000000000000f03f"
                "00000000000000400000000000000840000000000000f0bf00000000"
                "00000000000000000000e03f01000000000000000000000000805640"
                "000000000000000000000000008046c0050000000100000000000000"
                "000000000000000000000000000000f03f0000000000000000000000"
                "000000f03f000000000000000000");
    }
    {
        HarmonicsPayload p;
        CheckSaved(Harmonics(), &p,
                   "0100000000000000000014400000000000001440000000000000f03f"
                   "01010000000000");
        p.settings.waves = 3.0;
        p.settings.waveLength = -0.5;
        p.settings.frequency = 1.25;
        p.settings.ignoreFirstFrame = false;
        p.matchVelocity = false;
        p.hasSpace = true;
        p.frames = 17;
        CheckSaved(Harmonics(), &p,
                   "010000000000000000000840000000000000e0bf000000000000f43f"
                   "00000111000000");
    }
    {
        IkFkMatchPayload p;
        CheckSaved(IkFkMatch(), &p,
                   "010000000000000000000000000000000000f03f0000000000000000"
                   "00000000000000000101010101010101010000000000000000000000"
                   "000000000000000000000000f03f0000000000000000000000000000"
                   "0000000000000000000001");
        SpaceMatchPayload s;
        CheckSaved(SpaceMatch(), &s,
                   "010000000000000000000000000000000000f03f0000000000000000"
                   "0000000000000000010101010101010101000000000000000000");
        p.poleSwitch.filters = {0, 3, 1};
        p.poleSwitch.twistAxis = Vec3d(0.0, 1.0, 0.0);
        p.poleSwitch.affectRotation = {{true, false, true}};
        p.poleSwitch.hasCarry = true;
        p.poleSwitch.world = {1, 0, 1};
        p.poleSourceRides = {0, 1, 1};
        p.ikValue = 0.0;
        p.scaleCalibration = 2.0;
        p.upperLengthOffset = 0.125;
        p.lowerLengthOffset = -3.0;
        p.stretchLimb = false;
        CheckSaved(IkFkMatch(), &p,
                   "01000000030000000000000000000000030000000100000000000000"
                   "00000000000000000000f03f00000000000000000101010100010101"
                   "01010300000000000000010001030000000000000000010100000000"
                   "000000000000000000000040000000000000c03f00000000000008c0"
                   "00");
        s.space = p.poleSwitch;
        CheckSaved(SpaceMatch(), &s,
                   "01000000030000000000000000000000030000000100000000000000"
                   "00000000000000000000f03f00000000000000000101010100010101"
                   "01010300000000000000010001");
    }
    {
        AutoClavicleOpPayload p;
        CheckSaved(AutoClavicle(), &p,
                   "01000000000000000000f03f00000000000000000000000000000000"
                   "0000000000000000000000000000f03f000000000000000000000000"
                   "000000000000000000000000000000000000f03f000000000000f03f"
                   "9a9999999999d93f0000000001000000000000000000000000000000"
                   "000000000000000000000000000000000000");
        p.constants.basis[4] = -0.0;
        p.constants.kernel = 1;
        p.constants.normalize = false;
        p.constants.swings = {1.0, 0.0, 0.25, -0.5};
        p.constants.widths = {0.75};
        p.constants.gains = {2.0};
        p.constants.weights = {1.5};
        p.hasLimb = true;
        CheckSaved(AutoClavicle(), &p,
                   "01000000000000000000f03f00000000000000000000000000000000"
                   "00000000000000000000000000000080000000000000000000000000"
                   "000000000000000000000000000000000000f03f000000000000f03f"
                   "9a9999999999d93f01000000000400000000000000000000000000f0"
                   "3f0000000000000000000000000000d03f000000000000e0bf010000"
                   "0000000000000000000000e83f010000000000000000000000000000"
                   "400100000000000000000000000000f83f01");
    }
    {
        WeightFieldPayload p;
        CheckSaved(PlaneWeight(), &p, "010000000000000000000000000200000000");
        p.falloffCurve = {1.0f, 0.5f, 0.0f};
        p.clamp = true;
        p.planeAxis = WeightFieldAxis::X;
        p.planeBounded = true;
        for (const RigExecOpHandle *op :
             {&SphereWeight(), &PlaneWeight(), &CurveWeight()})
            CheckSaved(
                *op, &p,
                "0100000003000000000000000000803f0000003f0000000001000000"
                "0001");
        CombineWeightsPayload c;
        CheckSaved(CombineWeights(), &c, "01000000000000000000000000");
        c.mode = WeightCombineMode::Overlay;
        c.inputs = WeightCombineInputs::Fields;
        c.clamp = true;
        CheckSaved(CombineWeights(), &c, "01000000060000000100000001");
    }
}

/// The deform payloads save the bytes they saved before their fields were
/// declared, at defaults and filled.
void
TestDeformSavedBytes()
{
    {
        MatrixDeformPayload p;
        CheckSaved(MatrixDeform(), &p, "01000000 00");
        p.radial = true;
        CheckSaved(MatrixDeform(), &p, "01000000 01");
    }
    {
        SkinPayload p;
        CheckSaved(Skin(), &p, "01000000 00000000 01000000");
        p.method = SkinMethod::DualQuaternion;
        p.elementSize = 4;
        CheckSaved(Skin(), &p, "01000000 01000000 04000000");
    }
    {
        LayeredSkinPayload p;
        CheckSaved(LayeredSkin(), &p, "01000000 01000000 00");
        p.elementSize = 4;
        p.dualQuaternion = true;
        CheckSaved(LayeredSkin(), &p, "01000000 04000000 01");
    }
    {
        DeltaMushPayload p;
        CheckSaved(DeltaMush(), &p,
                   "01000000 0a000000 000000000000e03f 01 0000000000000000 "
                   "000000000000f03f 00000000 00000000 0000000000000000 "
                   "0000000000000000 00");
        p.iterations = 4;
        p.step = 0.25;
        p.pinBorders = false;
        p.settings.smoothing = RigExecDeltaMushSmoothing::LengthWeighted;
        p.settings.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        p.settings.smoothWeights = {0.0f, 1.0f, 0.5f};
        p.settings.edges = {0, 1, 1, 2};
        p.settings.onlySmooth = true;
        CheckSaved(
            DeltaMush(), &p,
            "01000000 04000000 000000000000d03f 00 0000000000000000 "
            "000000000000f03f 02000000 01000000 0300000000000000 00000000 "
            "0000803f 0000003f 0400000000000000 00000000 01000000 01000000 "
            "02000000 01");
    }
    {
        LatticePayload p;
        CheckSaved(
            Lattice(), &p,
            "01000000 02000000 02000000 02000000 00 02000000 02000000 "
            "02000000 000000bf 000000bf 000000bf 0000803f 0000803f 0000803f "
            "0000803f 0000000000000000");
        p.divisions = {3, 4, 5};
        p.settings.regularGrid = true;
        p.settings.interpolation[1] = RigExecLatticeInterpolation::CatmullRom;
        p.settings.origin = {1.0f, 2.0f, 3.0f};
        p.settings.strength = 0.5f;
        p.settings.mask = {1.0f, 0.0f};
        CheckSaved(
            Lattice(), &p,
            "01000000 03000000 04000000 05000000 01 02000000 03000000 "
            "02000000 0000803f 00000040 00004040 0000803f 0000803f 0000803f "
            "0000003f 0200000000000000 0000803f 00000000");
    }
    {
        WrinklePayload p;
        CheckSaved(
            Wrinkle(), &p,
            "01000000 50000000 00000000 02000000 0000803f 0000803f 0000803f "
            "cdcccc3d cdcc4c3e 01 01 00000000 0000803f 00000000 "
            "0000000000000000");
        p.settings.iterations = 12;
        p.settings.topology = RigExecWrinkleTopology::SurfaceStruts;
        p.settings.bendStiffness = 0.3f;
        p.settings.tangentPlaneCollisions = false;
        p.settings.pinPoints = {4, 9};
        CheckSaved(
            Wrinkle(), &p,
            "01000000 0c000000 01000000 02000000 0000803f 0000803f 0000803f "
            "9a99993e cdcc4c3e 01 00 00000000 0000803f 00000000 "
            "0200000000000000 04000000 09000000");
    }
    {
        SurfaceProjectPayload p;
        CheckSaved(
            SurfaceProject(), &p,
            "01000000 00000000 00000000 0000000000000000 0000000000000000");
        p.settings.mode = RigExecSurfaceSnapMode::OutsideSurface;
        p.settings.offset = 0.01f;
        p.settings.mask = {0.5f};
        p.settings.triangles = {0, 1, 2};
        CheckSaved(SurfaceProject(), &p,
                   "01000000 03000000 0ad7233c 0100000000000000 0000003f "
                   "0300000000000000 00000000 01000000 02000000");
    }
    {
        WirePayload p;
        CheckSaved(Wire(), &p,
                   "01000000 04000000 0000000000000000 0000000000000000");
        p.order = 3;
        p.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
        p.dropoffDistance = 2.5;
        CheckSaved(Wire(), &p,
                   "01000000 03000000 0600000000000000 0000000000000000 "
                   "0000000000000000 0000000000000000 000000000000f03f "
                   "000000000000f03f 000000000000f03f 0000000000000440");
    }
    {
        NormalsPayload p;
        CheckSaved(RecomputeNormals(), &p, "01000000 00 00");
        p.perCorner = true;
        p.leftHanded = true;
        CheckSaved(RecomputeNormals(), &p, "01000000 01 01");
    }
    {
        PatchPayload p;
        CheckSaved(Patch(), &p,
                   "01000000 00000000 00000000 00000000 00000000 00 00");
        p.uVertexCount = 7;
        p.vVertexCount = -2;
        p.uOrder = 4;
        p.vOrder = 16;
        p.uPeriodic = true;
        CheckSaved(Patch(), &p,
                   "01000000 07000000 feffffff 04000000 10000000 01 00");
    }
    {
        SurfaceBindingPayload p;
        CheckSaved(SurfaceBinding(), &p, "01000000 000000");
        p.limit = true;
        p.framed = true;
        CheckSaved(SurfaceBinding(), &p, "01000000 010001");
    }
    {
        SurfaceProjectorPayload p;
        CheckSaved(
            SurfaceProjector(), &p,
            "01000000 000000000000000000000000000000000000000000000000 "
            "00000000000000000000000000000000000000000000f03f "
            "0000000000000000000000000000f03f0000000000000000 "
            "000000000000f03f000000000000000000000000000000000000000000000000 "
            "0000000000000000000000000000f03f00000000000000000000000000000000 "
            "00000000000000000000000000000000000000000000f03f0000000000000000 "
            "000000000000000000000000000000000000000000000000000000000000f03f "
            "000000");
        p.rayOrigin = Vec3d(1.0, 2.0, 3.0);
        p.shaderOffset[3][0] = 4.0;
        p.reproject = true;
        p.spaceNamed = true;
        CheckSaved(
            SurfaceProjector(), &p,
            "01000000 000000000000f03f00000000000000400000000000000840 "
            "00000000000000000000000000000000000000000000f03f "
            "0000000000000000000000000000f03f0000000000000000 "
            "000000000000f03f000000000000000000000000000000000000000000000000 "
            "0000000000000000000000000000f03f00000000000000000000000000000000 "
            "00000000000000000000000000000000000000000000f03f0000000000000000 "
            "000000000000104000000000000000000000000000000000000000000000f03f "
            "010001");
    }
}

/// Values a writer cannot produce are refused.
void
TestCorruptValues()
{
    // A declared enum past its last value is refused.
    const RigExecOpDesc &skin = *Op(Skin()).desc;
    std::vector<uint8_t> bytes;
    SkinPayload p;
    CHECK(RigExecEncodeOpPayload(skin, &p, bytes));
    std::vector<uint8_t> badEnum = bytes;
    badEnum[4] = 7;
    std::string error;
    CHECK(!FieldsDecode(skin, badEnum, &error));
    CHECK(error.find("method: a value is out of range") != std::string::npos);

    const RigExecOpPayloadHandling &skeleton = Op(Skeleton2d()).desc->payload;
    Skeleton2dPayload grouped;
    grouped.groups = {RigExecJointGroup()};
    skeleton.encode(&grouped, bytes);
    // The group's enabled flag, ahead of the empty member list.
    bytes[bytes.size() - 9] = 2;
    CHECK(!Decodes(skeleton, bytes, &error));
    CHECK(error.find("flag") != std::string::npos);

    // A declared flag holding 2 is refused the same way.
    BlendFramesPayload b;
    CHECK(RigExecEncodeOpPayload(*Op(BlendFrames()).desc, &b, bytes));
    bytes.back() = 2;
    CHECK(!FieldsDecode(*Op(BlendFrames()).desc, bytes, &error));
    CHECK(error.find("logScale: a value is out of range") != std::string::npos);

    // A list length larger than the bytes left is refused before any item
    // is read or allocated.
    const RigExecOpPayloadHandling &blend = Op(BlendShape()).desc->payload;
    BlendShapePayload s;
    s.channels.resize(2);
    blend.encode(&s, bytes);
    for (size_t i = 4; i < 12; ++i)
        bytes[i] = 0xff;
    CHECK(!Decodes(blend, bytes, &error));
    CHECK(error.find("longer than") != std::string::npos);

    // A declared list's count past the bytes left is refused the same way.
    const RigExecOpDesc &wire = *Op(Wire()).desc;
    WirePayload w;
    w.knots = {1.0, 2.0};
    CHECK(RigExecEncodeOpPayload(wire, &w, bytes));
    for (size_t i = 8; i < 16; ++i)
        bytes[i] = 0xff;
    CHECK(!FieldsDecode(wire, bytes, &error));
    CHECK(error.find("longer than") != std::string::npos);
}

} // namespace

/// Whether \p type's columns are \p kinds, each in run \p runs[i].
bool
HasColumns(const RigExecTypeHandle &type,
           const std::vector<RigExecColumnKind> &kinds,
           const std::vector<uint32_t> &runs)
{
    const auto &columns = RigExecRegistry::Get().Type(type).columns;
    if (columns.size() != kinds.size())
        return false;
    for (size_t i = 0; i < kinds.size(); ++i)
        if (columns[i].kind != kinds[i] || columns[i].run != runs[i])
            return false;
    return true;
}

/// Standard values decompose into their scalars; arrays and records into
/// their elements' and members' columns, each array in a run of its own.
void
TestColumns()
{
    using K = RigExecColumnKind;
    const std::vector<K> vec3f(3, K::Float32);
    CHECK(HasColumns(rigExecStd::Vec3fType(), vec3f, {0, 0, 0}));
    CHECK(HasColumns(rigExecStd::Vec3fArray(), vec3f, {1, 1, 1}));
    CHECK(HasColumns(rigExecStd::Mat4dType(), std::vector<K>(16, K::Float64),
                     std::vector<uint32_t>(16, 0)));
    CHECK(HasColumns(rigExecStd::BoolType(), {K::UInt8}, {0}));
    std::vector<K> frame(12, K::Float64);
    frame.push_back(K::UInt32);
    CHECK(HasColumns(rigExecRig::PointFrameType(), frame,
                     std::vector<uint32_t>(13, 0)));
    std::vector<K> frameArray = frame;
    frameArray.insert(frameArray.end(), 12, K::Float64);
    std::vector<uint32_t> frameArrayRuns(13, 1);
    frameArrayRuns.insert(frameArrayRuns.end(), 12, 2);
    CHECK(HasColumns(rigExecRig::PointFrameArrayType(), frameArray,
                     frameArrayRuns));
    CHECK(RigExecRegistry::Get()
              .Type(rigExecRig::PointFrameArrayType())
              .runParents ==
          std::vector<uint32_t>({kRigExecInvalidIndex, 0, 0}));
    CHECK(HasColumns(
        rigExecRig::PointWeightsType(),
        {K::Float32, K::Int32, K::Float32, K::UInt32, K::UInt32, K::UInt8},
        {1, 2, 0, 0, 0, 0}));
}

int
main()
{
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors)
            std::printf("registry: %s\n", e.c_str());
        return 1;
    }
    TestEveryOp();
    TestFilledPayloads();
    TestCorruptValues();
    TestMathBytes();
    TestSavedBytes();
    TestDeformSavedBytes();
    TestColumns();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecOpPayloads passed\n");
    return 0;
}
