// Bind decoded weight inputs and publish to the existing producer slots.
#include "stageArrayInputs.h"
#include "rigExecRuntime/labels.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecRuntime/store.h"
#include "rigExecRuntime/spaces.h"
#include "rigExecMath/volumeFieldCheck.h"
#include "rigExecMath/weightFieldKernel.h"
#include "rigExecEvalCore/weights.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <array>
#include <deque>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

// One weight object's token fields as the weight field branches on
// them, classified once at Open so no call compares text.
struct RrWeightFieldKind {
    enum class Type : uint8_t {
        Static,
        Dynamic,
        Combine,
        Sphere,
        Plane,
        Curve,
        Unknown
    };
    enum class Representation : uint8_t { Constant, Dense, Sparse, Other };
    Type type = Type::Unknown;
    Representation representation = Representation::Other;
    bool strict = false;    ///< rigExec:rangePolicy is `strict`
    bool clamp = false;     ///< rigExec:rangePolicy is `clamp`
    int combineMode = -1;   ///< _RrCombineMode; -1 for an unknown mode
    int planeAxis = -1;     ///< 0 x, 1 y, 2 z; -1 for an unknown axis
    bool bounded = false;   ///< rigExec:planeBounds is `bounded`
    bool unbounded = false; ///< rigExec:planeBounds is `unbounded`
};

// One point attribute of a volume gather, resolved at Open: its live
// path-read row, else the chain whose target it is (the chain-base
// fallback), else neither, when it contributes nothing.
struct RrWeightPointSource {
    int32_t row = -1;
    int32_t chain = -1;
};

struct RrWeightSupportCache {
    std::vector<int32_t> source, normalized;
    bool initialized = false;
};
struct RrWeightPacketScratch {
    std::vector<std::pair<int32_t, float>> pairs;
    std::vector<int32_t> support;
    std::vector<size_t> supportOrder;
    bool uniqueSupport = false;
    std::vector<float> combineField;
    std::array<std::vector<RrVec3f>, 3> points;
    std::vector<RrVec3f> localCurve;
};
struct RrWeightFieldScratch {
    std::deque<std::vector<float>> children;
    std::vector<uint32_t> seen;
    uint32_t seenGeneration = 0;
    std::vector<RrVec3f> localCurve;
    std::vector<std::array<RrWeightSupportCache, 2>> supports;
    std::vector<float> result;
};

struct RrWeightScratch {
    // Per weight object, the sources of its gathers, entry for entry with
    // its combine_target_points, target_points, sample_points and
    // curve_points.
    struct Gathers {
        std::vector<RrWeightPointSource> combineTargets, targets, samples,
            curves;
    };
    std::vector<Gathers> gathers;
    std::unordered_map<uint32_t, size_t> falloffRows;
    std::vector<RrWeightPacketScratch> packets;
    std::vector<RrWeightFieldScratch> fields;
    // Per weight object, its classified tokens.
    std::vector<RrWeightFieldKind> fieldKinds;
    std::vector<uint8_t> fieldBlocked;
    // Per weight object, the volume slot whose placement the weight field reads:
    // the no_scale_avars slot whose path is the object's own, else -1.
    std::vector<int32_t> volumeSlot;
    // Raw weightTarget slots used only after a failed primary Get.
    std::vector<int32_t> fieldFallbackSlots;
};

bool
RrWeightSizeScratch(RrProgram *program, std::string *error)
{
    if (!program || !program->geometry) {
        if (error) {
            *error = "weight scratch needs the geometry domain";
        }
        return false;
    }
    auto scratch = std::make_shared<RrWeightScratch>();
    program->weights = scratch;
    scratch->packets.resize(program->geometry->weightObjects.size());
    scratch->fields.resize(program->geometry->weightFields.size());
    for (auto &field : scratch->fields) {
        field.children.resize(program->geometry->weightObjects.size() + 1);
        field.supports.resize(program->geometry->weightObjects.size());
    }
    scratch->fieldFallbackSlots =
        RigExecStageWeightFieldFallbackSlots(*program->inputState.file);
    const auto &file = *program->inputState.file;
    scratch->fieldBlocked.assign(program->geometry->weightObjects.size(), 0);
    if (file.commonGraph)
        for (uint64_t id : file.commonGraph->excludedValues) {
            const auto &value = file.commonGraph->valueSpecs[size_t(id)];
            if (value.domain == uint32_t(RigExecWireSlotDomain::WeightPacket))
                scratch->fieldBlocked[value.slot] = 1;
        }
    auto &store = program->store;
    store.weightFieldResults.assign(program->geometry->weightFields.size(), {});
    store.weightFieldContexts.resize(program->geometry->weightFields.size());
    for (size_t f = 0; f < program->geometry->weightFields.size(); ++f) {
        const auto &field = program->geometry->weightFields[f];
        auto &context = store.weightFieldContexts[f];
        for (size_t r = 0; r < field.scalarReads.size(); ++r) {
            context.scalarReadIndices.emplace(
                std::make_pair(field.scalarObjects[r],
                               int32_t(field.scalarMembers[r])),
                r);
        }
        for (size_t r = 0; r < field.pointReads.size(); ++r)
            context.pointReadIndices.emplace(
                std::make_pair(field.pointReads[r].object,
                               field.pointReads[r].leaf),
                r);
        // Preserve native publication order, including collisions between
        // a chain target and another chain's phased consumer. Validity is
        // checked when reading; this inventory only freezes the context.
        for (int32_t c : field.availableChains) {
            const auto &chain = file.propertyChains[size_t(c)];
            context.versionsBySlot[chain.target].push_back(
                int32_t(chain.versionBase + chain.revisions.size()));
            for (const auto &record : file.phasedConsumers) {
                if (record.chain == uint32_t(c)) {
                    context.versionsBySlot[record.consumer].push_back(
                        int32_t(record.version));
                }
            }
        }
    }
    using Kind = RrWeightFieldKind;
    const std::vector<RigExecWireWeightObject> &objects =
        program->geometry->weightObjects;
    // Every gather's point sources: the live path-read row of the
    // attribute, else the chain whose target is exactly that attribute
    // (chain targets are attribute paths, so this matches by id).
    {
        std::unordered_map<uint32_t, size_t> chainByTarget;
        const std::vector<RigExecWireChain> &chains = program->geometry->chains;
        for (size_t c = 0; c < chains.size(); ++c) {
            chainByTarget.emplace(chains[c].target, c);
        }
        const auto sources = [&](const std::vector<uint32_t> &paths) {
            std::vector<RrWeightPointSource> out(paths.size());
            for (size_t i = 0; i < paths.size(); ++i) {
                out[i].row =
                    RrFindPathReadRow(program->pathReads, paths[i], false);
                const auto chain = chainByTarget.find(paths[i]);
                if (out[i].row < 0 && chain != chainByTarget.end()) {
                    out[i].chain = int32_t(chain->second);
                }
            }
            return out;
        };
        scratch->gathers.reserve(objects.size());
        for (const RigExecWireWeightObject &object : objects) {
            RrWeightScratch::Gathers gathers;
            gathers.combineTargets = sources(object.combineTargetPoints);
            gathers.targets = sources(object.targetPoints);
            gathers.samples = sources(object.samplePoints);
            gathers.curves = sources(object.curvePoints);
            scratch->gathers.push_back(std::move(gathers));
        }
    }
    if (file.formatVersion >= 27) {
        std::unordered_map<std::string, size_t> rows;
        for (size_t i = 0; i < program->pathReads.size(); ++i) {
            const auto &row = program->pathReads[i];
            if (!row.rest && row.read && row.read->read &&
                row.read->read->tag == RigExecWireInputTag::FloatArray)
                rows.emplace(program->TextOrEmpty(row.path), i);
        }
        for (const auto &object : objects) {
            // Row keys are spelled "<path>.<attribute>".
            const auto found =
                rows.find(program->TextOrEmpty(object.path) + "." +
                          SchemaAttrText::inputsFalloffSamples);
            if (found != rows.end())
                scratch->falloffRows.emplace(object.path, found->second);
        }
    }
    scratch->fieldKinds.reserve(objects.size());
    for (const RigExecWireWeightObject &object : objects) {
        const std::string type = program->TextOrEmpty(object.type);
        const std::string representation =
            program->TextOrEmpty(object.representation);
        const std::string rangePolicy =
            program->TextOrEmpty(object.rangePolicy);
        const std::string combineMode =
            program->TextOrEmpty(object.combineMode);
        // The plane tokens as the weight field reads them, not the composition's.
        const std::string planeAxis =
            program->TextOrEmpty(object.fieldPlaneAxis);
        const std::string planeBounds =
            program->TextOrEmpty(object.fieldPlaneBounds);
        Kind kind;
        kind.type =
            type == SchemaTypeName::RigExecStaticWeight    ? Kind::Type::Static
            : type == SchemaTypeName::RigExecDynamicWeight ? Kind::Type::Dynamic
            : type == SchemaTypeName::RigExecCombineWeight ? Kind::Type::Combine
            : type == SchemaTypeName::RigExecSphereWeight  ? Kind::Type::Sphere
            : type == SchemaTypeName::RigExecPlaneWeight   ? Kind::Type::Plane
            : type == SchemaTypeName::RigExecCurveWeight   ? Kind::Type::Curve
                                                         : Kind::Type::Unknown;
        kind.representation = representation == SchemaValueText::constant
                                  ? Kind::Representation::Constant
                              : representation == SchemaValueText::dense
                                  ? Kind::Representation::Dense
                              : representation == SchemaValueText::sparse
                                  ? Kind::Representation::Sparse
                                  : Kind::Representation::Other;
        kind.strict = rangePolicy == SchemaValueText::strict;
        kind.clamp = rangePolicy == SchemaValueText::clamp;
        // RigExecWeightCombine order, which _RrCombineMode mirrors.
        const char *const modes[] = {
            SchemaValueText::multiply, SchemaValueText::add,
            SchemaValueText::subtract, SchemaValueText::max,
            SchemaValueText::min,      SchemaValueText::average,
            SchemaValueText::overlay};
        for (int m = 0; m < 7; ++m) {
            if (combineMode == modes[m]) {
                kind.combineMode = m;
            }
        }
        kind.planeAxis = planeAxis == SchemaValueText::x   ? 0
                         : planeAxis == SchemaValueText::y ? 1
                         : planeAxis == SchemaValueText::z ? 2
                                                           : -1;
        kind.bounded = planeBounds == SchemaValueText::bounded;
        kind.unbounded = planeBounds == SchemaValueText::unbounded;
        scratch->fieldKinds.push_back(kind);
    }
    // A path names at most one slot, so an object has at most one volume.
    // A hand-assembled program may carry no slot tables: no object then
    // has a placement.
    std::unordered_map<uint32_t, int32_t> volumeByPath;
    if (program->slotMeta && program->constants) {
        const std::vector<uint32_t> &paths = program->slotMeta->paths;
        const std::vector<uint8_t> &noScale = program->constants->noScaleAvars;
        for (size_t i = 0; i < paths.size() && i < noScale.size(); ++i) {
            if (noScale[i]) {
                volumeByPath.emplace(paths[i], int32_t(i));
            }
        }
    }
    scratch->volumeSlot.reserve(objects.size());
    for (const RigExecWireWeightObject &object : objects) {
        const auto volume = volumeByPath.find(object.path);
        scratch->volumeSlot.push_back(
            volume == volumeByPath.end() ? -1 : volume->second);
    }
    return true;
}

namespace {

// The step's label for fatal errors. Weight steps cannot fail in the
// baked program (their indices close by construction); every failure
// below names a wire violation the runtime refuses to crash on.
std::string
_RrStepLabel(const RrProgram *program, size_t step)
{
    return RrStepLabel(*program, step);
}

RrWeightScratch *
_RrScratch(const RrProgram *program)
{
    return static_cast<RrWeightScratch *>(program->weights.get());
}

// A bound weight input as the float the builders consume. The read
// carries the input's own tag; the double arm mirrors the
// float-from-double coercion GetAttribute applies to a scalar float read.
float
_RrReadWeightFloat(const RrProgram *program, size_t object, int field)
{
    const RrInputValue value = program->ReadWeight(object, field);
    if (value.tag == RrInputTag::Float) {
        return value.f32;
    }
    if (value.tag == RrInputTag::Double) {
        return static_cast<float>(value.f64);
    }
    return 0.0f;
}

// The points one gather attribute reads this run when they are held in the
// runtime's type: its live path-read row (a static array, or the elements
// its input slots answer), else the base slot of the chain whose target it
// is. Null otherwise; *\p wireBase then names that chain's base as the
// bake captured it, or null for a failed read -- no row, a known-absent
// mark, a mistyped holding, or a chain with no base.
const std::vector<RrVec3f> *
_RrGatherSource(const RrProgram *program, const RrWeightPointSource &source,
                const std::vector<RigExecWireVec3f> **wireBase)
{
    *wireBase = nullptr;
    if (source.row >= 0) {
        return RrPathArray<RrVec3f>(program,
                                    program->pathReads[size_t(source.row)]);
    }
    if (source.chain < 0) {
        return nullptr;
    }
    const int32_t slot =
        program->geometry->chains[size_t(source.chain)].baseSlot;
    if (slot >= 0) {
        return RrInputArray<RrVec3f>(program, uint32_t(slot));
    }
    *wireBase = program->statics.ChainBase(size_t(source.chain));
    return nullptr;
}

// Appends one attribute's points to a volume gather. A failed read
// contributes nothing, exactly as a false GetAttribute does in the baked
// gather.
void
_RrGatherPoints(const RrProgram *program, const RrWeightPointSource &source,
                std::vector<RrVec3f> *out)
{
    const std::vector<RigExecWireVec3f> *wireBase = nullptr;
    if (const std::vector<RrVec3f> *points =
            _RrGatherSource(program, source, &wireBase)) {
        out->insert(out->end(), points->begin(), points->end());
        return;
    }
    if (!wireBase) {
        return;
    }
    for (const RigExecWireVec3f &p : *wireBase) {
        out->push_back(RrVec3f(p[0], p[1], p[2]));
    }
}

// The element count one attribute contributes to a combine's target
// fallback: the same read as above, measured rather than copied.
size_t
_RrGatherPointCount(const RrProgram *program, const RrWeightPointSource &source)
{
    const std::vector<RigExecWireVec3f> *wireBase = nullptr;
    if (const std::vector<RrVec3f> *points =
            _RrGatherSource(program, source, &wireBase)) {
        return points->size();
    }
    return wireBase ? wireBase->size() : 0;
}

void
_RrGatherPointList(const RrProgram *program,
                   const std::vector<RrWeightPointSource> &sources,
                   const std::vector<uint8_t> &valid, std::vector<RrVec3f> *out)
{
    for (size_t i = 0; i < sources.size(); ++i) {
        if (i < valid.size() && !valid[i]) {
            continue;
        }
        _RrGatherPoints(program, sources[i], out);
    }
}

// Packet envelope (RigExecWeightPacket::Resolve, types.cpp).

enum _RrRepresentation {
    _RrRepConstant,
    _RrRepDense,
    _RrRepSparse,
    _RrRepOther,
};

_RrRepresentation
_RrClassifyRepresentation(const RrProgram *program, uint32_t id)
{
    if (program->TokenEquals(id, SchemaValueText::dense)) {
        return _RrRepDense;
    }
    if (program->TokenEquals(id, SchemaValueText::sparse)) {
        return _RrRepSparse;
    }
    if (program->TokenEquals(id, SchemaValueText::constant)) {
        return _RrRepConstant;
    }
    return _RrRepOther;
}

// Rigid placement (RigExecRigidWorldToLocal, weightPackets.cpp, over
// GfMatrix4d::RemoveScaleShear from pxr/base/gf/matrix4d.cpp).

// Jacobi eigen decomposition of the symmetric 3x3 in _Jacobi3's upper
// triangle, in place. A line port of GfMatrix4d::_Jacobi3.
void
_RrJacobi3(RrMat4d *self, RrVec3d *eigenvalues, RrVec3d eigenvectors[3])
{
    (*eigenvalues)[0] = (*self)[0][0];
    (*eigenvalues)[1] = (*self)[1][1];
    (*eigenvalues)[2] = (*self)[2][2];
    eigenvectors[0] = RrVec3d::XAxis();
    eigenvectors[1] = RrVec3d::YAxis();
    eigenvectors[2] = RrVec3d::ZAxis();

    RrMat4d a = *self;
    RrVec3d b = *eigenvalues;
    RrVec3d z(0.0);

    for (int i = 0; i < 50; i++) {
        double sm = 0.0;
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                sm += std::abs(a[p][q]);
            }
        }

        if (sm == 0.0) {
            return;
        }

        const double thresh = (i < 3 ? (.2 * sm / (3 * 3)) : 0.0);

        for (int p = 0; p < 3; p++) {
            for (int q = p + 1; q < 3; q++) {
                double g = 100.0 * std::abs(a[p][q]);

                if (i > 3 &&
                    (std::abs((*eigenvalues)[p]) + g ==
                     std::abs((*eigenvalues)[p])) &&
                    (std::abs((*eigenvalues)[q]) + g ==
                     std::abs((*eigenvalues)[q]))) {
                    a[p][q] = 0.0;
                } else if (std::abs(a[p][q]) > thresh) {
                    double h = (*eigenvalues)[q] - (*eigenvalues)[p];
                    double t;

                    if (std::abs(h) + g == std::abs(h)) {
                        t = a[p][q] / h;
                    } else {
                        const double theta = 0.5 * h / a[p][q];
                        t = 1.0 /
                            (std::abs(theta) + std::sqrt(1.0 + theta * theta));
                        if (theta < 0.0) {
                            t = -t;
                        }
                    }

                    const double c = 1.0 / std::sqrt(1.0 + t * t);
                    const double s = t * c;
                    const double tau = s / (1.0 + c);
                    h = t * a[p][q];
                    z[p] -= h;
                    z[q] += h;
                    (*eigenvalues)[p] -= h;
                    (*eigenvalues)[q] += h;
                    a[p][q] = 0.0;

                    for (int j = 0; j < p; j++) {
                        g = a[j][p];
                        h = a[j][q];
                        a[j][p] = g - s * (h + g * tau);
                        a[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = p + 1; j < q; j++) {
                        g = a[p][j];
                        h = a[j][q];
                        a[p][j] = g - s * (h + g * tau);
                        a[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = q + 1; j < 3; j++) {
                        g = a[p][j];
                        h = a[q][j];
                        a[p][j] = g - s * (h + g * tau);
                        a[q][j] = h + s * (g - h * tau);
                    }

                    for (int j = 0; j < 3; j++) {
                        g = eigenvectors[j][p];
                        h = eigenvectors[j][q];
                        eigenvectors[j][p] = g - s * (h + g * tau);
                        eigenvectors[j][q] = h + s * (g - h * tau);
                    }
                }
            }
        }
        for (int p = 0; p < 3; p++) {
            (*eigenvalues)[p] = b[p] += z[p];
            z[p] = 0;
        }
    }
}

// Factors the matrix into rotation, scale, rotation, translation and
// perspective (M = r * s * -r * u * t). A line port of
// GfMatrix4d::Factor with its default eps.
bool
_RrFactorMatrix(const RrMat4d &self, RrMat4d *r, RrVec3d *s, RrMat4d *u,
                RrVec3d *t, RrMat4d *p)
{
    const double eps = 1e-10;

    p->SetDiagonal(1);

    RrMat4d a;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] = self[i][j];
        }
        a[3][i] = a[i][3] = 0.0;
        (*t)[i] = self[3][i];
    }
    a[3][3] = 1.0;

    const double det = a.GetDeterminant3();
    const double detSign = (det < 0.0 ? -1.0 : 1.0);
    const bool isSingular = det * detSign < eps;

    const RrMat4d b = a * a.GetTranspose();
    RrVec3d eigenvalues;
    RrVec3d eigenvectors[3];
    RrMat4d mutableB = b;
    _RrJacobi3(&mutableB, &eigenvalues, eigenvectors);
    r->Set(eigenvectors[0][0], eigenvectors[0][1], eigenvectors[0][2], 0.0,
           eigenvectors[1][0], eigenvectors[1][1], eigenvectors[1][2], 0.0,
           eigenvectors[2][0], eigenvectors[2][1], eigenvectors[2][2], 0.0, 0.0,
           0.0, 0.0, 1.0);

    RrMat4d sInv;
    sInv.SetIdentity();
    for (int i = 0; i < 3; i++) {
        if (eigenvalues[i] < eps) {
            (*s)[i] = detSign * eps;
        } else {
            (*s)[i] = detSign * std::sqrt(eigenvalues[i]);
        }
        sInv[i][i] = 1.0 / (*s)[i];
    }

    *u = *r * sInv * r->GetTranspose() * a;

    return !isSingular;
}

RrMat4d
_RrRemoveScaleShear(const RrMat4d &self)
{
    RrMat4d scaleOrientMat, factoredRotMat, perspMat;
    RrVec3d scale, translation;
    if (!_RrFactorMatrix(self, &scaleOrientMat, &scale, &factoredRotMat,
                         &translation, &perspMat)) {
        return self;
    }

    factoredRotMat.Orthonormalize();
    RrMat4d translate(1.0);
    translate.SetTranslateOnly(translation);
    return factoredRotMat * translate;
}

const std::vector<float> &
_RrFalloffSamples(const RrProgram *program, const RigExecWireWeightObject &wire)
{
    const auto *scratch = _RrScratch(program);
    const auto found = scratch->falloffRows.find(wire.path);
    if (found != scratch->falloffRows.end()) {
        const auto *samples =
            RrPathArray<float>(program, program->pathReads[found->second]);
        if (samples && !samples->empty())
            return *samples;
    }
    return wire.falloffCurve;
}

// A painted array (rigExec:values or rigExec:indices) as a read at the
// given time takes it, from the object's input slot: at Default
// (\p atDefault) its authored value, else its Default-time default; at the
// evaluation time any value it holds. \p none, an empty array, where the
// file lists no input: the object paints no such array.
template <class T>
const std::vector<T> &
_RrPainted(const RrProgram *program, int32_t slot, const std::vector<T> &none,
           bool atDefault)
{
    if (slot < 0) {
        return none;
    }
    const uint32_t s = uint32_t(slot);
    const std::vector<T> *read = atDefault && !RrInputArrayAuthored(program, s)
                                     ? RrInputArrayDefaultOf<T>(program, s)
                                     : RrInputArray<T>(program, s);
    return read ? *read : none;
}

struct _RrWeightFieldContext {
    const RigExecWireWeightField *field;
    const RrWeightScalarReadContext *reads;
    const std::vector<RrMat4d> *placements;
    const std::vector<char> *placed;
};

const std::vector<RrVec3f> *
_RrFieldPoints(const RrProgram *program, size_t object, int leaf,
               const _RrWeightFieldContext *context, bool *declared)
{
    *declared = false;
    if (!context)
        return nullptr;
    const auto found =
        context->reads->pointReadIndices.find({int32_t(object), leaf});
    if (found == context->reads->pointReadIndices.end())
        return nullptr;
    *declared = true;
    return RrResolveDeclaredPoints(
        program, *context->field->pointReads[found->second].binding);
}

// The points the weight field reads raw at the evaluation time (its samples, or
// a curve's points): input slot \p slot's elements when the file lists
// one and it holds a value; null when it reads none.
const std::vector<RrVec3f> *
_RrWeightFieldPoints(const RrProgram *program, int32_t slot)
{
    return slot >= 0 && RrInputHasValue(program, uint32_t(slot))
               ? RrInputArray<RrVec3f>(program, uint32_t(slot))
               : nullptr;
}

struct RrWeightMath {
    using Vec3f = RrVec3f;
    using Vec3d = RrVec3d;
    using Mat4 = RrMat4d;
    static Vec3d ToDouble(const Vec3f &p)
    {
        return {p[0], p[1], p[2]};
    }
    static Vec3f ToFloat(const Vec3d &p)
    {
        return {float(p[0]), float(p[1]), float(p[2])};
    }
    static float Dot(const Vec3f &a, const Vec3f &b)
    {
        return RrDot(a, b);
    }
    static Mat4 RemoveScaleShear(const Mat4 &m)
    {
        return _RrRemoveScaleShear(m);
    }
    static constexpr auto PointsToMatrix = static_cast<bool (*)(
        const std::array<Vec3d, 4> &, const std::array<Vec3d, 4> &, Mat4 *)>(
        RrPointsToMatrix);
};

struct RrWeightInputs {
    const RrProgram *program;
    const _RrWeightFieldContext *context = nullptr;
    const RrPointFrame *frame = nullptr;
    RrWeightFieldScratch *work = nullptr;
    const RigExecWireWeightObject &wire(size_t i) const
    {
        return program->geometry->weightObjects[i];
    }
    eval::WeightObjectFacts weightFacts(size_t i) const
    {
        const auto &w = wire(i);
        const auto &s = *_RrScratch(program);
        const auto &k = s.fieldKinds[i];
        eval::WeightObjectFacts f;
        f.kind = k.type == RrWeightFieldKind::Type::Unknown ? -1 : int(k.type);
        f.representation =
            k.representation == RrWeightFieldKind::Representation::Other
                ? -1
                : int(k.representation);
        f.rangePolicy = k.clamp    ? eval::kWeightPolicyClamp
                        : k.strict ? eval::kWeightPolicyStrict
                                   : eval::kWeightPolicyUnknown;
        f.strict = k.strict;
        f.clamp = k.clamp;
        f.combineMode = k.combineMode;
        f.base = w.base;
        f.inputCount = w.inputs.size();
        f.inputIndices = w.inputs.data();
        f.samplesInFlight = w.samplesInFlight;
        f.blocked = s.fieldBlocked[i];
        f.checkDynamicSupport = w.indicesSlot >= 0;
        return f;
    }
    std::string weightName(size_t i) const
    {
        return program->TextOrEmpty(wire(i).path);
    }
    std::string weightTypeText(size_t i) const
    {
        return program->TextOrEmpty(wire(i).type);
    }
    std::string weightCombineText(size_t i) const
    {
        return program->TextOrEmpty(wire(i).combineMode);
    }
    const std::string &weightStaticError(size_t i) const
    {
        return wire(i).fieldStaticError;
    }
    const std::string &weightPhaseError(size_t i) const
    {
        return wire(i).fieldPhaseError;
    }

    bool dynamicSupportMatches(size_t i, size_t base) const
    {
        const std::vector<int32_t> none;
        auto normalized =
            [&](size_t id,
                RrWeightSupportCache &cache) -> const std::vector<int32_t> & {
            const auto &source =
                _RrPainted(program, wire(id).indicesSlot, none, false);
            if (!cache.initialized || cache.source != source) {
                cache.source = source;
                cache.normalized = source;
                std::sort(cache.normalized.begin(), cache.normalized.end());
                cache.normalized.erase(std::unique(cache.normalized.begin(),
                                                   cache.normalized.end()),
                                       cache.normalized.end());
                cache.initialized = true;
            }
            return cache.normalized;
        };
        if (!fieldIndices(i).second)
            return true;
        auto &support = work->supports[i];
        return normalized(i, support[0]) == normalized(base, support[1]);
    }

    float fieldScalar(size_t i, int m, float fallback) const
    {
        if (context) {
            const auto &reads = *context->reads;
            const auto found = reads.scalarReadIndices.find({int32_t(i), m});
            return found == reads.scalarReadIndices.end()
                       ? fallback
                       : RrReadWeightScalar(
                             program,
                             context->field->scalarReads[found->second], reads,
                             fallback);
        }
        const auto &w = wire(i);
        const std::unique_ptr<RigExecWireInput> *members[] = {
            &w.defaultWeight, &w.driver,    &w.scale,      &w.bias,
            &w.strength,      &w.invert,    &w.falloffMin, &w.falloffMax,
            &w.scaleX,        &w.scaleY,    &w.scaleZ,     &w.scaleXPos,
            &w.scaleYPos,     &w.scaleZPos, &w.scaleXNeg,  &w.scaleYNeg,
            &w.scaleZNeg,     &w.extentU,   &w.extentV};
        const auto &input = *members[m];
        return input ? RrReadResolvedFloat(program, *input, fallback)
                     : fallback;
    }
    auto fieldValues(size_t i) const
    {
        const std::vector<float> none;
        const auto &v = _RrPainted(program, wire(i).valuesSlot, none, false);
        return std::make_pair(v.empty() ? nullptr : v.data(), v.size());
    }
    std::pair<const int32_t *, size_t> fieldIndices(size_t i) const
    {
        const std::vector<int32_t> none;
        const auto &v = _RrPainted(program, wire(i).indicesSlot, none, false);
        return std::make_pair(v.empty() ? nullptr : v.data(), v.size());
    }
    const RrMat4d *fieldPlacement(size_t i) const
    {
        const auto &store = program->store;
        const auto &placements =
            context ? *context->placements : store.volumePlacement;
        const auto &placed = context ? *context->placed : store.volumePlaced;
        const size_t slot = size_t(_RrScratch(program)->volumeSlot[i]);
        return slot < store.volumePlaced.size() && slot < placements.size() &&
                       placed[slot]
                   ? &placements[slot]
                   : nullptr;
    }
    eval::WeightFieldPoint<RrWeightMath> fieldPoints(size_t i, int leaf,
                                                     bool phased) const
    {
        bool declared = false;
        const std::vector<RrVec3f> *points = nullptr;
        if (phased) {
            points = _RrFieldPoints(program, i, leaf, context, &declared);
        } else {
            const auto &fallback = _RrScratch(program)->fieldFallbackSlots;
            const auto &w = wire(i);
            const int32_t slot = leaf == 0             ? w.fieldSamplesSlot
                                 : leaf == 2           ? w.fieldCurveSlot
                                 : i < fallback.size() ? fallback[i]
                                                       : -1;
            points = _RrWeightFieldPoints(program, slot);
        }
        return {declared, points != nullptr, points ? points->data() : nullptr,
                points ? points->size() : 0};
    }
    auto fieldFalloff(size_t i) const
    {
        const auto &v = _RrFalloffSamples(program, wire(i));
        return std::make_pair(v.data(), v.size());
    }
    std::string fieldPlaneToken(size_t i, bool bounds) const
    {
        const auto &w = wire(i);
        const int32_t slot =
            bounds ? w.fieldPlaneBoundsSlot : w.fieldPlaneAxisSlot;
        if (!context || slot < 0)
            return program->TextOrEmpty(bounds ? w.fieldPlaneBounds
                                               : w.fieldPlaneAxis);
        const auto &state = program->inputState;
        return state.slotHasValue[size_t(slot)]
                   ? program->TextOrEmpty(
                         uint32_t(state.slotCurrent[size_t(slot)].bits))
                   : std::string();
    }

    eval::WeightPacketFacts<RrWeightMath> packetFacts(size_t i) const
    {
        const auto &w = wire(i);
        eval::WeightPacketFacts<RrWeightMath> f;
        static_cast<eval::WeightObjectFacts &>(f) = weightFacts(i);
        const auto read = [&](size_t member, int field) {
            f.scalars[member] = _RrReadWeightFloat(program, i, field);
        };
        if (f.kind == eval::kWeightKindStatic) {
            read(0, RrWeightDefaultWeight);
            const std::vector<float> noValues;
            const std::vector<int32_t> noIndices;
            const auto &values =
                _RrPainted(program, w.valuesSlot, noValues, true);
            const auto &indices =
                _RrPainted(program, w.indicesSlot, noIndices, true);
            f.paintedValues = values.empty() ? nullptr : values.data();
            f.paintedValueCount = values.size();
            f.paintedIndices = indices.empty() ? nullptr : indices.data();
            f.paintedIndexCount = indices.size();
        } else if (f.kind == eval::kWeightKindDynamic) {
            read(1, RrWeightDriver);
            read(2, RrWeightScale);
            read(3, RrWeightBias);
        } else if (f.kind == eval::kWeightKindCombine) {
            const auto &targets =
                _RrScratch(program)->gathers[i].combineTargets;
            for (size_t k = 0; k < targets.size(); ++k)
                if (k >= w.combineTargetValid.size() || w.combineTargetValid[k])
                    f.combineTargetCount +=
                        _RrGatherPointCount(program, targets[k]);
            read(4, RrWeightStrength);
            read(5, RrWeightInvert);
        } else if (f.kind >= eval::kWeightKindSphere) {
            if (frame) {
                f.framePoints = &frame->points;
                f.frameValid = frame->IsValid();
                f.frameDegenerate = frame->IsDegenerate();
            }
            read(4, RrWeightStrength);
            read(5, RrWeightInvert);
            read(6, RrWeightFalloffMin);
            read(7, RrWeightFalloffMax);
            if (f.kind == eval::kWeightKindPlane) {
                f.planeAxis =
                    eval::WeightAxis(program->TextOrEmpty(w.planeAxis));
                f.planeBounds =
                    eval::WeightBounds(program->TextOrEmpty(w.planeBounds));
                if (f.planeBounds == eval::kWeightBoundsBounded) {
                    read(17, RrWeightExtentU);
                    read(18, RrWeightExtentV);
                }
            } else {
                for (int k = 0; k < 3; ++k)
                    read(8 + k, RrWeightScaleX + k);
                if (f.kind == eval::kWeightKindSphere)
                    for (int k = 0; k < 6; ++k)
                        read(11 + k, RrWeightScaleXPos + k);
            }
            const auto &falloff = _RrFalloffSamples(program, w);
            f.falloffCurve = falloff.data();
            f.falloffCurveCount = falloff.size();
        }
        return f;
    }
    eval::WeightPointViewT<RrWeightMath> volumePoints(size_t i,
                                                      size_t leaf) const
    {
        const auto &w = wire(i);
        const auto &g = _RrScratch(program)->gathers[i];
        auto &out = _RrScratch(program)->packets[i].points[leaf];
        out.clear();
        if (leaf == 0)
            _RrGatherPointList(program, g.samples, w.sampleValid, &out);
        else if (leaf == 1)
            _RrGatherPointList(program, g.targets, w.targetValid, &out);
        else
            _RrGatherPointList(program, g.curves, w.curveValid, &out);
        return {out.data(), out.size()};
    }
    eval::WeightFrameView<RrWeightMath> volumePlacementFacts(size_t, bool) const
    {
        return {&frame->points, frame->IsValid(), frame->IsDegenerate()};
    }
};

struct RrWeightState {
    const RrProgram *program;
    RrStore *store;
    RrWeightFieldScratch *work;
    size_t packetCount() const
    {
        return store->weightPackets.size();
    }
    size_t weightObjectCount() const
    {
        return program->geometry->weightObjects.size();
    }
    auto &fieldWorkspace() const
    {
        return *work;
    }
    std::vector<RrVec3f> *fieldCurveScratch() const
    {
        return &work->localCurve;
    }
    auto *packetScratch(size_t i) const
    {
        return &_RrScratch(program)->packets[i];
    }
    std::vector<RrVec3f> *packetCurveScratch(size_t i) const
    {
        return &packetScratch(i)->localCurve;
    }
    eval::WeightPacketView packetView(size_t i) const
    {
        const auto &p = store->weightPackets[i];
        return {p.valid,
                _RrClassifyRepresentation(program, p.representation),
                p.defaultWeight,
                p.values.data(),
                p.values.size(),
                p.indices.data(),
                p.indices.size()};
    }
    void storePacket(size_t i, eval::WeightPacketResult p) const
    {
        RrWeightPacket out;
        if (!p.bare) {
            const auto &w = program->geometry->weightObjects[i];
            out.representation = w.representation;
            out.rangePolicy = w.rangePolicy;
            out.defaultWeight = p.defaultWeight;
            out.valid = p.valid;
            out.values = std::move(p.values);
            out.indices = std::move(p.indices);
        }
        store->weightPackets[i] = std::move(out);
    }
    void setVolumePlacement(size_t slot, bool base,
                            const RrMat4d &placement) const
    {
        (base ? store->volumePlacementBase : store->volumePlacement)[slot] =
            placement;
        (base ? store->volumePlacedBase : store->volumePlaced)[slot] = 1;
    }
};

struct RrWeightHost {
    using Math = RrWeightMath;
    using Inputs = RrWeightInputs;
    using State = RrWeightState;
    using EvalValueFamily = eval::WeightValueFamily;
    Inputs in;
    State out;
    const Inputs &inputs() const
    {
        return in;
    }
    State &state()
    {
        return out;
    }
    const RigExecRangeRunner *ranges() const
    {
        return nullptr;
    }
};

bool
_RrResolveField(const RrProgram *program, size_t object, size_t count,
                const std::vector<RrVec3f> *current, std::vector<float> *values,
                std::string *error, const _RrWeightFieldContext *context,
                RrWeightFieldScratch &work)
{
    RrWeightHost host{{program, context, nullptr, &work},
                      {program, nullptr, &work}};
    const eval::WeightPointViewT<RrWeightMath> view =
        current ? eval::WeightPointViewT<RrWeightMath>{current->data(),
                                                       current->size()}
                : eval::WeightPointViewT<RrWeightMath>{};
    return eval::RunWeightField(host, object, count, current ? &view : nullptr,
                                values, error);
}

bool
_RrRunWeightPacket(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wireStep = (*program->steps)[step];
    const int32_t object = wireStep.object;
    if (object < 0 || size_t(object) >= program->stepWeightObjects ||
        size_t(object) >= store.weightPackets.size()) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " names no weight object";
        }
        return false;
    }
    const size_t index = size_t(object);
    const RigExecWireWeightObject &wire =
        program->geometry->weightObjects[index];
    RrWeightScratch *scratch = _RrScratch(program);

    const auto type = scratch->fieldKinds[index].type;
    using Type = RrWeightFieldKind::Type;
    const RrPointFrame *frame = nullptr;
    if (type != Type::Static && type != Type::Dynamic &&
        index >= scratch->gathers.size()) {
        if (error)
            *error = "weight step " + _RrStepLabel(program, step) +
                     " names a weight object Open did not resolve";
        return false;
    }
    if ((type == Type::Sphere || type == Type::Plane || type == Type::Curve) &&
        wire.providerSlot >= 0) {
        const size_t slot = size_t(wire.providerSlot);
        if (slot >= store.baseLast.size() ||
            size_t(store.baseLast[slot]) >= store.base.size()) {
            if (error)
                *error = "weight step " + _RrStepLabel(program, step) +
                         " is placed by no provider slot";
            return false;
        }
        frame = &store.base[size_t(store.baseLast[slot])];
    }
    RrWeightHost host{{program, nullptr, frame}, {program, &store, nullptr}};
    return eval::RunValueStep(host, {uint32_t(wireStep.kind), wireStep.object,
                                     wireStep.part, index})
               .outcome == eval::Outcome::Applied;
}

bool
_RrRunVolumePlacements(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wire = (*program->steps)[step];
    const std::vector<uint8_t> &noScale = program->constants->noScaleAvars;
    const size_t slot = size_t(wire.object);
    if ((wire.part != 1 && wire.part != 2) || wire.object < 0 ||
        slot >= program->slotMeta->paths.size() || slot >= noScale.size() ||
        !noScale[slot]) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " places no volume slot";
        }
        return false;
    }
    const bool base = wire.part == 2;
    const auto &last = base ? store.baseLast : store.finLast;
    const auto &frames = base ? store.base : store.fin;
    if (slot >= last.size() || frames.empty() ||
        slot >= store.volumePlacement.size() ||
        slot >= store.volumePlaced.size()) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " runs with no pose frames";
        }
        return false;
    }
    if (size_t(last[slot]) >= frames.size()) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " reads a pose frame beyond the pools";
        }
        return false;
    }
    const RrPointFrame &frame = frames[size_t(last[slot])];
    RrWeightHost host{{program, nullptr, &frame}, {program, &store, nullptr}};
    return eval::RunValueStep(
               host, {uint32_t(wire.kind), wire.object, wire.part, slot})
               .outcome == eval::Outcome::Applied;
}

} // namespace

bool
RrResolveWeightField(const RrProgram *program, size_t object, size_t count,
                     const std::vector<RrVec3f> *current,
                     std::vector<float> *weights, std::string *error)
{
    std::string unused;
    std::string *why = error ? error : &unused;
    if (!weights) {
        *why = "no weight field to resolve into";
        return false;
    }
    const RrWeightScratch *scratch = program ? _RrScratch(program) : nullptr;
    // A wire violation, not a weight field answer: Open checked every index the
    // pose and geometry tables hand in, and the file keeps each entry's
    // composition below the entry itself.
    if (!scratch || object >= program->geometry->weightObjects.size() ||
        object >= scratch->fieldKinds.size()) {
        *why = "the file holds no weight object " + std::to_string(object);
        return false;
    }
    RrWeightFieldScratch work;
    work.children.resize(scratch->fieldKinds.size() + 1);
    work.supports.resize(scratch->fieldKinds.size());
    const bool ok = _RrResolveField(program, object, count, current, weights,
                                    why, nullptr, work);
    if (!ok)
        weights->clear();
    return ok;
}

bool
RrResolveDeclaredWeightField(RrProgram *program, size_t fieldIndex,
                             size_t count, const std::vector<RrVec3f> *current)
{
    const auto &field = program->geometry->weightFields[fieldIndex];
    const bool base =
        field.placementPhase == fb::WeightFieldPlacementPhase::Base;
    const _RrWeightFieldContext context{
        &field, &program->store.weightFieldContexts[fieldIndex],
        base ? &program->store.volumePlacementBase
             : &program->store.volumePlacement,
        base ? &program->store.volumePlacedBase : &program->store.volumePlaced};
    auto &result = program->store.weightFieldResults[fieldIndex];
    auto &work = _RrScratch(program)->fields[fieldIndex];
    auto &values = work.result;
    std::string error;
    const bool ok = _RrResolveField(program, size_t(field.object), count,
                                    current, &values, &error, &context, work);
    // A failed result never exposes the preceding successful value.
    result.count = count;
    result.ok = ok;
    result.error = std::move(error);
    if (ok)
        result.values.swap(values);
    else
        result.values.clear();
    return ok;
}

bool
RrWeightFieldEffectiveInputMemo(const RrProgram *program, uint32_t fieldIndex,
                                std::string *key,
                                std::vector<uint32_t> *covered)
{
    using Kind = RrWeightFieldKind;
    const auto &field = program->geometry->weightFields[fieldIndex];
    const auto &reads = program->store.weightFieldContexts[fieldIndex];
    const auto append = [&](const auto &value) {
        key->append(reinterpret_cast<const char *>(&value), sizeof(value));
    };
    const RrVec3f *current = nullptr;
    size_t count = 1;
    if (field.form == fb::WeightFieldForm::Revision) {
        const bool present =
            RrRevisionWeightFieldInput(program, fieldIndex, &current, &count);
        append(present);
        append(uint64_t(count));
        if (!present)
            return true;
    } else
        append(uint64_t(count));
    const bool base =
        field.placementPhase == fb::WeightFieldPlacementPhase::Base;
    const auto &placements = base ? program->store.volumePlacementBase
                                  : program->store.volumePlacement;
    const auto &placed =
        base ? program->store.volumePlacedBase : program->store.volumePlaced;
    for (int32_t slot : field.volumes) {
        append(bool(placed[size_t(slot)]));
        if (placed[size_t(slot)])
            key->append(
                reinterpret_cast<const char *>(placements[size_t(slot)]._mtx),
                sizeof(double) * 16);
    }
    static constexpr float defaults[19] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 1,
                                           1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (size_t i = 0; i < field.scalarReads.size(); ++i) {
        const auto &input = field.scalarReads[i];
        const size_t member = size_t(field.scalarMembers[i]);
        append(RrReadWeightScalar(program, input, reads, defaults[member]));
        for (const auto *segment :
             {&input.propertyCandidates, &input.doubleCandidates})
            for (const auto &hop : *segment) {
                if (hop.version >= 0)
                    covered->push_back(uint32_t(hop.version));
                const auto found = reads.versionsBySlot.find(hop.slot);
                if (found != reads.versionsBySlot.end())
                    for (int32_t version : found->second)
                        if (version >= 0)
                            covered->push_back(uint32_t(version));
            }
    }
    const auto points = [&](const std::vector<RrVec3f> *value) {
        append(bool(value));
        if (!value)
            return;
        append(uint64_t(value->size()));
        if (!value->empty())
            key->append(reinterpret_cast<const char *>(value->data()),
                        value->size() * sizeof(RrVec3f));
    };
    const auto *scratch = _RrScratch(program);
    std::vector<int32_t> objects = field.scalarObjects;
    std::sort(objects.begin(), objects.end());
    objects.erase(std::unique(objects.begin(), objects.end()), objects.end());
    for (int32_t object : objects) {
        const auto &wire = program->geometry->weightObjects[size_t(object)];
        const auto falloff = scratch->falloffRows.find(wire.path);
        if (falloff != scratch->falloffRows.end())
            RrSourceReadMemo(
                program, *program->pathReads[falloff->second].read->read, key);
        for (int32_t slot : {wire.valuesSlot, wire.indicesSlot})
            if (slot >= 0) {
                RigExecWireInput raw;
                raw.tag = program->file->inputs[size_t(slot)].type();
                raw.mode = RigExecWireReadMode::Raw;
                raw.walk = {uint32_t(slot)};
                raw.constant = program->file->inputs[size_t(slot)].value();
                RrSourceReadMemo(program, raw, key);
            }
        // Field evaluation reads the compiled rig cycle gate, independently of
        // ordinary packet publication and its geometry consumer.
        append(bool(scratch->fieldBlocked[size_t(object)]));
        const auto &kind = scratch->fieldKinds[size_t(object)];
        if (kind.type != Kind::Type::Plane && kind.type != Kind::Type::Sphere &&
            kind.type != Kind::Type::Curve)
            continue;
        bool declared = false;
        const _RrWeightFieldContext context{&field, &reads, nullptr, nullptr};
        const auto *selected =
            _RrFieldPoints(program, size_t(object), 1, &context, &declared);
        if (!declared && !wire.samplesInFlight) {
            selected =
                _RrFieldPoints(program, size_t(object), 0, &context, &declared);
            if (!declared)
                selected = _RrWeightFieldPoints(program, wire.fieldSamplesSlot);
            if (!selected &&
                size_t(object) < scratch->fieldFallbackSlots.size())
                selected = _RrWeightFieldPoints(
                    program, scratch->fieldFallbackSlots[size_t(object)]);
        }
        if (wire.samplesInFlight && !declared) {
            if (count && !current)
                return false;
            append(bool(current || count == 0));
            append(uint64_t(count));
            if (count)
                key->append(reinterpret_cast<const char *>(current),
                            count * sizeof(RrVec3f));
        } else
            points(selected);
        bool curveDeclared = false;
        const auto *curve = _RrFieldPoints(program, size_t(object), 2, &context,
                                           &curveDeclared);
        if (!curveDeclared)
            curve = _RrWeightFieldPoints(program, wire.fieldCurveSlot);
        if (kind.type == Kind::Type::Curve)
            points(curve);
        if (kind.type == Kind::Type::Plane)
            for (int32_t slot :
                 {wire.fieldPlaneAxisSlot, wire.fieldPlaneBoundsSlot})
                if (slot >= 0) {
                    RigExecWireInput raw;
                    raw.tag = RigExecWireInputTag::Token;
                    raw.mode = RigExecWireReadMode::Raw;
                    raw.walk = {uint32_t(slot)};
                    raw.constant = program->file->inputs[size_t(slot)].value();
                    RrSourceReadMemo(program, raw, key);
                }
    }
    return true;
}

bool
RrRunWeightStep(RrProgram *program, size_t step, std::string *error)
{
    if (!program || !program->steps || !program->geometry ||
        !program->slotMeta || !program->constants || !program->weights ||
        step >= program->steps->size()) {
        if (error) {
            *error = "weight step names no step";
        }
        return false;
    }
    const RigExecWireStepKind kind = (*program->steps)[step].kind;
    if (kind == RigExecWireStepKind::WeightField) {
        const size_t fieldIndex = size_t((*program->steps)[step].object);
        const auto &field = program->geometry->weightFields[fieldIndex];
        if (field.form == fb::WeightFieldForm::Revision) {
            RrRunRevisionWeightField(program, fieldIndex);
        } else {
            RrResolveDeclaredWeightField(program, fieldIndex, 1, nullptr);
        }
        // Field failure is a published field outcome consumed by the
        // original consumer's failure policy, not an executor failure.
        return true;
    }
    if (kind == RigExecWireStepKind::VolumePlacements) {
        return _RrRunVolumePlacements(program, step, error);
    }
    if (kind != RigExecWireStepKind::WeightPacket) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " is not a weight step";
        }
        return false;
    }
    return _RrRunWeightPacket(program, step, error);
}

} // namespace rigExec
