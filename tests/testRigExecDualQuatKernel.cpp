// The contract of the shared scaled dual-quaternion blend in
// rigExecMath/dualQuatKernel.h, checked over both host traits: the Gf core
// (rigExecMath/dualQuat.h) and the USD-free playback runtime
// (rigExecRuntime/affineMath.h). A null record is a missing influence
// (false, identity) unless its weight is exactly zero; the projection to the
// rigid half never dereferences one; and the two traits agree bit for bit.
// No stage, no plugin path, no GPU.
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/dualQuatKernel.h"
#include "rigExecRuntime/affineMath.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <type_traits>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

namespace {

// Exact bit equality, the comparison testRigExecRuntimeMath uses: the traits
// have to run the same operations in the same order.
bool
SameBits(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

template <class QuatA, class QuatB>
bool
SameQuatBits(const QuatA &a, const QuatB &b)
{
    if (!SameBits(a.GetReal(), b.GetReal())) {
        return false;
    }
    const auto &ia = a.GetImaginary();
    const auto &ib = b.GetImaginary();
    for (int i = 0; i < 3; ++i) {
        if (!SameBits(ia[i], ib[i])) {
            return false;
        }
    }
    return true;
}

template <class Mat3A, class Mat3B>
bool
SameMat3Bits(const Mat3A &a, const Mat3B &b)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!SameBits(a[i][j], b[i][j])) {
                return false;
            }
        }
    }
    return true;
}

template <class RecordA, class RecordB>
bool
SameRecordBits(const RecordA &a, const RecordB &b)
{
    return a.isRigid == b.isRigid && SameQuatBits(a.rigid.real, b.rigid.real) &&
           SameQuatBits(a.rigid.dual, b.rigid.dual) &&
           SameMat3Bits(a.stretch, b.stretch);
}

template <class Record>
bool
IsIdentityRecord(const Record &r)
{
    using Quat = typename Record::DualQuat::Quat;
    const Quat identityReal(1.0);
    const Quat zero(0.0);
    const typename Record::Mat3 identityStretch(1.0);
    return r.isRigid && SameQuatBits(r.rigid.real, identityReal) &&
           SameQuatBits(r.rigid.dual, zero) &&
           SameMat3Bits(r.stretch, identityStretch);
}

// One row-vector 4x4 spelled as exact dyadic doubles, so both traits receive
// bit-identical input matrices. GfMatrix4d and RrMat4d take the same
// row-major 16-double argument list.
struct Mat16 {
    double m[16];
};

template <class Mat4>
Mat4
ToMatrix(const Mat16 &m)
{
    return Mat4(m.m[0], m.m[1], m.m[2], m.m[3], m.m[4], m.m[5], m.m[6], m.m[7],
                m.m[8], m.m[9], m.m[10], m.m[11], m.m[12], m.m[13], m.m[14],
                m.m[15]);
}

// Two rigid joints and two non-rigid ones (non-uniform scale, shear), so the
// stretch half of the blend runs next to the rigid half.
const Mat16 kPalette[] = {
    // rotZ90, translation (1, 2, 3): rigid.
    {{0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0,
      3.0, 1.0}},
    // pre-stretch (2, 1, 0.5) then rotZ90, translation (0, -1, 4): non-rigid.
    {{0.0, 2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -1.0,
      4.0, 1.0}},
    // rotX90, translation (-2, 0.5, 4): rigid.
    {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -2.0, 0.5,
      4.0, 1.0}},
    // shear in x, no translation: non-rigid.
    {{2.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0}},
};
constexpr size_t kPaletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

const double kBlendWeights[] = {0.25, 0.75, 0.25, 0.75};

// A caller's own record type for the accessor form. Its rigid member
// deliberately does not sit at offset 0: an unguarded projection of a null
// record reads through address offsetof(rigid) instead of rejecting, so this
// layout is what separates a checked lookup from an accidentally null
// pointer. The accessor reads these members and the Mat3 alias only.
template <class Point, class Double> struct PaddedRecordT {
    using DualQuat = RigExecDualQuatKernelT<Point, Double>;
    using Mat3 = typename DualQuat::Mat3;

    double pad[13];
    DualQuat rigid;
    Mat3 stretch{1.0};
    bool isRigid = true;
};

using PaddedRecordGf = PaddedRecordT<GfVec3f, GfVec3d>;
using PaddedRecordRr = PaddedRecordT<RrVec3f, RrVec3d>;

static_assert(std::is_standard_layout<PaddedRecordGf>::value,
              "offsetof needs a standard-layout record");
static_assert(std::is_standard_layout<PaddedRecordRr>::value,
              "offsetof needs a standard-layout record");
static_assert(offsetof(PaddedRecordGf, rigid) > 0,
              "the rigid member must not sit at offset 0");
static_assert(offsetof(PaddedRecordRr, rigid) > 0,
              "the rigid member must not sit at offset 0");

template <class Point, class Double>
void
FillPadded(PaddedRecordT<Point, Double> *dst,
           const RigExecScaledDualQuatKernelT<Point, Double> &src)
{
    for (double &v : dst->pad) {
        v = 0.0;
    }
    dst->rigid = src.rigid;
    dst->stretch = src.stretch;
    dst->isRigid = src.isRigid;
}

template <class Point, class Double>
using PlainRecord = RigExecScaledDualQuatKernelT<Point, Double>;

template <class Point, class Double>
std::array<PlainRecord<Point, Double>, kPaletteSize>
DecomposeKernelPalette()
{
    std::array<PlainRecord<Point, Double>, kPaletteSize> palette;
    for (size_t i = 0; i < kPaletteSize; ++i) {
        palette[i] = RigExecScaledDualQuatFromMatrixKernel<Point, Double>(
            ToMatrix<typename RigExecDualQuatMath<Point, Double>::Mat4>(
                kPalette[i]));
    }
    return palette;
}

// A null record at a non-zero weight is a missing influence: false, and the
// identity record. The same table with every record present blends bitwise
// like the kernel's own records, so the accessor reads only rigid, stretch,
// isRigid and the Mat3 alias.
template <class Point, class Double>
void
CheckMissingRecordAtNonZeroOffset(const char *traitName)
{
    using Padded = PaddedRecordT<Point, Double>;
    std::printf("missing record (%s): offsetof(rigid) = %zu, sizeof = %zu\n",
                traitName, offsetof(Padded, rigid), sizeof(Padded));

    const auto plain = DecomposeKernelPalette<Point, Double>();
    std::array<Padded, kPaletteSize> table;
    for (size_t i = 0; i < kPaletteSize; ++i) {
        FillPadded(&table[i], plain[i]);
    }
    CHECK(!IsIdentityRecord(table[0]));
    CHECK(table[0].isRigid && !table[1].isRigid);

    auto getWeight = [](size_t i) { return kBlendWeights[i]; };
    for (size_t slot = 0; slot < kPaletteSize; ++slot) {
        auto getSdq = [&table, slot](size_t i) -> const Padded * {
            return i == slot ? nullptr : &table[i];
        };
        Padded result = table[2];
        CHECK(!RigExecBlendScaledDualQuatsKernel(kPaletteSize, getSdq,
                                                 getWeight, &result));
        CHECK(IsIdentityRecord(result));
    }

    auto getAll = [&table](size_t i) -> const Padded * { return &table[i]; };
    Padded fromPadded;
    CHECK(RigExecBlendScaledDualQuatsKernel(kPaletteSize, getAll, getWeight,
                                            &fromPadded));
    PlainRecord<Point, Double> fromPlain;
    CHECK(RigExecBlendScaledDualQuatsKernel(plain.data(), kBlendWeights,
                                            kPaletteSize, &fromPlain));
    CHECK(!fromPadded.isRigid);
    CHECK(SameRecordBits(fromPadded, fromPlain));
}

// A null or out-of-range record whose weight is exactly 0.0 is padding: it is
// ignored in the first, middle and last slot, and the result is bitwise the
// blend of the surviving records.
template <class Point, class Double>
void
CheckZeroWeightPaddingRule(const char *traitName)
{
    using ScaledDualQuat = PlainRecord<Point, Double>;
    std::printf("zero-weight padding (%s)\n", traitName);

    const auto palette = DecomposeKernelPalette<Point, Double>();
    const size_t paletteSize = 2;
    std::array<ScaledDualQuat, 2> survivors = {palette[0], palette[1]};
    const double survivorWeights[] = {0.25, 0.75};
    ScaledDualQuat reference;
    CHECK(RigExecBlendScaledDualQuatsKernel(survivors.data(), survivorWeights,
                                            paletteSize, &reference));
    CHECK(!reference.isRigid);

    // Accessor form: the lookup itself returns null in the padded slot.
    for (size_t slot = 0; slot < 3; ++slot) {
        double weights[3];
        for (size_t i = 0; i < 3; ++i) {
            weights[i] =
                i == slot ? 0.0 : survivorWeights[i < slot ? i : i - 1];
        }
        auto getSdq = [&survivors, paletteSize,
                       slot](size_t i) -> const ScaledDualQuat * {
            if (i == slot) {
                return nullptr;
            }
            const size_t survivor = i < slot ? i : i - 1;
            return survivor < paletteSize ? &survivors[survivor] : nullptr;
        };
        auto getWeight = [&weights](size_t i) { return weights[i]; };
        ScaledDualQuat result = palette[2];
        CHECK(RigExecBlendScaledDualQuatsKernel(3, getSdq, getWeight, &result));
        CHECK(SameRecordBits(result, reference));
    }

    // Indexed form: an index outside [0, paletteSize), and the same index at
    // a non-zero weight.
    const int outOfRange[] = {static_cast<int>(paletteSize), -1, -1000000};
    const double zeroWeights[][3] = {
        {0.0, 0.25, 0.75}, {0.25, 0.0, 0.75}, {0.25, 0.75, 0.0}};
    const double liveWeights[][3] = {
        {0.5, 0.25, 0.75}, {0.25, 0.5, 0.75}, {0.25, 0.75, 0.5}};
    for (const int invalid : outOfRange) {
        const int slots[][3] = {
            {invalid, 0, 1}, {0, invalid, 1}, {0, 1, invalid}};
        for (size_t slot = 0; slot < 3; ++slot) {
            ScaledDualQuat result = palette[2];
            CHECK(RigExecBlendScaledDualQuatsKernel(
                palette.data(), paletteSize, slots[slot], zeroWeights[slot], 3,
                &result));
            CHECK(SameRecordBits(result, reference));

            result = palette[2];
            CHECK(!RigExecBlendScaledDualQuatsKernel(
                palette.data(), paletteSize, slots[slot], liveWeights[slot], 3,
                &result));
            CHECK(IsIdentityRecord(result));
        }
    }
}

// The two host traits decompose the same matrices to the same records and
// blend them to the same bits.
void
CheckTraitsBlendBitForBit()
{
    std::printf("trait parity: GfVec3f/GfVec3d versus RrVec3f/RrVec3d\n");
    std::array<RigExecScaledDualQuat, kPaletteSize> gfPalette;
    for (size_t i = 0; i < kPaletteSize; ++i) {
        gfPalette[i] =
            RigExecScaledDualQuatFromMatrix(ToMatrix<GfMatrix4d>(kPalette[i]));
    }
    const auto rrPalette = DecomposeKernelPalette<RrVec3f, RrVec3d>();
    for (size_t i = 0; i < kPaletteSize; ++i) {
        CHECK(gfPalette[i].isRigid == rrPalette[i].isRigid);
        CHECK(SameRecordBits(gfPalette[i], rrPalette[i]));
    }

    const int indexSets[][2] = {{0, 1}, {1, 3}, {0, 2}, {3, 1}};
    for (const auto &indices : indexSets) {
        RigExecScaledDualQuat gfBlend;
        CHECK(RigExecBlendScaledDualQuatsKernel(gfPalette.data(), kPaletteSize,
                                                indices, kBlendWeights, 2,
                                                &gfBlend));
        PlainRecord<RrVec3f, RrVec3d> rrBlend;
        CHECK(RigExecBlendScaledDualQuatsKernel(rrPalette.data(), kPaletteSize,
                                                indices, kBlendWeights, 2,
                                                &rrBlend));
        CHECK(SameRecordBits(gfBlend, rrBlend));
    }
}

// The stretch mean runs for non-rigid entries, and a rigid-only blend keeps
// the exact identity stretch.
template <class Point, class Double>
void
CheckStretchMeanRuns(const char *traitName)
{
    using ScaledDualQuat = PlainRecord<Point, Double>;
    std::printf("stretch mean (%s)\n", traitName);
    const auto palette = DecomposeKernelPalette<Point, Double>();
    CHECK(palette[0].isRigid && !palette[1].isRigid);
    CHECK(palette[2].isRigid && !palette[3].isRigid);

    const typename ScaledDualQuat::Mat3 identityStretch(1.0);
    const int rigidOnly[] = {0, 2};
    ScaledDualQuat rigidBlend = palette[1];
    CHECK(RigExecBlendScaledDualQuatsKernel(palette.data(), kPaletteSize,
                                            rigidOnly, kBlendWeights, 2,
                                            &rigidBlend));
    CHECK(rigidBlend.isRigid);
    CHECK(SameMat3Bits(rigidBlend.stretch, identityStretch));

    const int scaledOnly[] = {1, 3};
    ScaledDualQuat scaledBlend;
    CHECK(RigExecBlendScaledDualQuatsKernel(palette.data(), kPaletteSize,
                                            scaledOnly, kBlendWeights, 2,
                                            &scaledBlend));
    CHECK(!scaledBlend.isRigid);
    CHECK(!SameMat3Bits(scaledBlend.stretch, identityStretch));
    CHECK(!SameRecordBits(scaledBlend, rigidBlend));

    // One non-rigid influence at weight 1.0: the mean is that record's
    // stretch exactly.
    const int one[] = {1};
    const double oneWeight[] = {1.0};
    ScaledDualQuat single;
    CHECK(RigExecBlendScaledDualQuatsKernel(palette.data(), kPaletteSize, one,
                                            oneWeight, 1, &single));
    CHECK(!single.isRigid);
    CHECK(SameMat3Bits(single.stretch, palette[1].stretch));
}

} // namespace

static void
TestStretchMeanRuns()
{
    CheckStretchMeanRuns<GfVec3f, GfVec3d>("GfVec3f/GfVec3d");
    CheckStretchMeanRuns<RrVec3f, RrVec3d>("RrVec3f/RrVec3d");
}

static void
TestZeroWeightPaddingRule()
{
    CheckZeroWeightPaddingRule<GfVec3f, GfVec3d>("GfVec3f/GfVec3d");
    CheckZeroWeightPaddingRule<RrVec3f, RrVec3d>("RrVec3f/RrVec3d");
}

static void
TestTraitsBlendBitForBit()
{
    CheckTraitsBlendBitForBit();
}

static void
TestMissingRecordAtNonZeroOffset()
{
    CheckMissingRecordAtNonZeroOffset<GfVec3f, GfVec3d>("GfVec3f/GfVec3d");
    CheckMissingRecordAtNonZeroOffset<RrVec3f, RrVec3d>("RrVec3f/RrVec3d");
}

int
main()
{
    TestStretchMeanRuns();
    TestZeroWeightPaddingRule();
    TestTraitsBlendBitForBit();
    TestMissingRecordAtNonZeroOffset();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecDualQuatKernel: all tests passed\n");
    return 0;
}
