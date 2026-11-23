// Every op payload saves and reads back equal, and bytes that cannot be a
// saved payload are refused without reading past them.
#include "rigExec/api/registry.h"
#include "rigExecOps/constraint/autoClavicleOps.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/constraint/harmonicsOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/deform/blendShapeOp.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/derivedOps.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/projectorOps.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wireOp.h"
#include "rigExecOps/deform/wrinkleOp.h"
#include "rigExecOps/property/mathOps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/solver/skeleton2dOps.h"
#include "rigExecOps/solver/splineIkOps.h"
#include "rigExecOps/weight/weightOps.h"

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

/// Saves \p payload, reads it back, and checks the result is equal and
/// saves to the same bytes; then checks damaged copies are refused.
void
CheckRoundTrip(const RigExecRegisteredOp &op, const void *payload)
{
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
        p.keys = {Vec2f(0.0f, 1.0f), Vec2f(1.0f, 0.0f)};
        p.tangents = {Vec2f(0.5f, 0.5f), Vec2f(-0.5f, 2.0f)};
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
}

/// Values a writer cannot produce are refused.
void
TestCorruptValues()
{
    const RigExecOpPayloadHandling &skin = Op(Skin()).desc->payload;
    std::vector<uint8_t> bytes;
    SkinPayload p;
    skin.encode(&p, bytes);
    std::vector<uint8_t> badEnum = bytes;
    badEnum[4] = 7;
    std::string error;
    CHECK(!Decodes(skin, badEnum, &error));
    CHECK(error.find("enum") != std::string::npos);

    const RigExecOpPayloadHandling &blend = Op(BlendFrames()).desc->payload;
    BlendFramesPayload b;
    blend.encode(&b, bytes);
    bytes.back() = 2;
    CHECK(!Decodes(blend, bytes, &error));
    CHECK(error.find("flag") != std::string::npos);

    // A list length larger than the bytes left is refused before any item
    // is read or allocated.
    const RigExecOpPayloadHandling &wire = Op(Wire()).desc->payload;
    WirePayload w;
    w.knots = {1.0, 2.0};
    wire.encode(&w, bytes);
    for (size_t i = 8; i < 16; ++i)
        bytes[i] = 0xff;
    CHECK(!Decodes(wire, bytes, &error));
    CHECK(error.find("longer than") != std::string::npos);
}

} // namespace

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
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecOpPayloads passed\n");
    return 0;
}
