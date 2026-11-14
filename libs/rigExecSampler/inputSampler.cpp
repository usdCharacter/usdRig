// rigExecSampler: see inputSampler.h for the contract.
#include "rigExecSampler/inputSampler.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include <algorithm>

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/resolveInfo.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

// An input the file does not mark Animated, on an attribute with at most one
// time sample and no spline or varying clip: every numeric time reads the
// same value (Default may read another, which a refresh covers).
bool
_StaticInput(bool animated, const UsdAttribute &attribute)
{
    return !animated && attribute && !attribute.ValueMightBeTimeVarying();
}

// Exact held-type test without a TfType lookup (no registry lock).
bool
_Holds(const VtValue &value, RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return value.IsHolding<double>();
    case RrInputTag::Float:
        return value.IsHolding<float>();
    case RrInputTag::Bool:
        return value.IsHolding<bool>();
    case RrInputTag::Int:
        return value.IsHolding<int>();
    case RrInputTag::Matrix4d:
        return value.IsHolding<GfMatrix4d>();
    case RrInputTag::Token:
        return value.IsHolding<TfToken>();
    case RrInputTag::Vec3d:
        return value.IsHolding<GfVec3d>();
    case RrInputTag::Vec3f:
        return value.IsHolding<GfVec3f>();
    case RrInputTag::Vec3i:
        return value.IsHolding<GfVec3i>();
    case RrInputTag::IntArray:
        return value.IsHolding<VtIntArray>();
    case RrInputTag::FloatArray:
        return value.IsHolding<VtFloatArray>();
    case RrInputTag::DoubleArray:
        return value.IsHolding<VtDoubleArray>();
    case RrInputTag::Vec2fArray:
        return value.IsHolding<VtVec2fArray>();
    case RrInputTag::Vec3fArray:
        return value.IsHolding<VtVec3fArray>();
    case RrInputTag::Vec3dArray:
        return value.IsHolding<VtVec3dArray>();
    case RrInputTag::Matrix4dArray:
        return value.IsHolding<VtMatrix4dArray>();
    case RrInputTag::TokenArray:
        return value.IsHolding<VtTokenArray>();
    case RrInputTag::BoolArray:
        return value.IsHolding<VtBoolArray>();
    default:
        break;
    }
    return false;
}

} // namespace

TfType
RigExecInputTagType(RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return TfType::Find<double>();
    case RrInputTag::Float:
        return TfType::Find<float>();
    case RrInputTag::Bool:
        return TfType::Find<bool>();
    case RrInputTag::Int:
        return TfType::Find<int>();
    case RrInputTag::Matrix4d:
        return TfType::Find<GfMatrix4d>();
    case RrInputTag::Token:
        return TfType::Find<TfToken>();
    case RrInputTag::Vec3d:
        return TfType::Find<GfVec3d>();
    case RrInputTag::Vec3f:
        return TfType::Find<GfVec3f>();
    case RrInputTag::Vec3i:
        return TfType::Find<GfVec3i>();
    case RrInputTag::IntArray:
        return TfType::Find<VtIntArray>();
    case RrInputTag::FloatArray:
        return TfType::Find<VtFloatArray>();
    case RrInputTag::DoubleArray:
        return TfType::Find<VtDoubleArray>();
    case RrInputTag::Vec2fArray:
        return TfType::Find<VtVec2fArray>();
    case RrInputTag::Vec3fArray:
        return TfType::Find<VtVec3fArray>();
    case RrInputTag::Vec3dArray:
        return TfType::Find<VtVec3dArray>();
    case RrInputTag::Matrix4dArray:
        return TfType::Find<VtMatrix4dArray>();
    case RrInputTag::TokenArray:
        return TfType::Find<VtTokenArray>();
    case RrInputTag::BoolArray:
        return TfType::Find<VtBoolArray>();
    default:
        break;
    }
    return TfType();
}

const char *
RigExecInputTagName(RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return "double";
    case RrInputTag::Float:
        return "float";
    case RrInputTag::Bool:
        return "bool";
    case RrInputTag::Int:
        return "int";
    case RrInputTag::Matrix4d:
        return "matrix4d";
    case RrInputTag::Token:
        return "token";
    case RrInputTag::Vec3d:
        return "vec3d";
    case RrInputTag::Vec3f:
        return "vec3f";
    case RrInputTag::Vec3i:
        return "int3";
    case RrInputTag::IntArray:
        return "int[]";
    case RrInputTag::FloatArray:
        return "float[]";
    case RrInputTag::DoubleArray:
        return "double[]";
    case RrInputTag::Vec2fArray:
        return "float2[]";
    case RrInputTag::Vec3fArray:
        return "float3[]";
    case RrInputTag::Vec3dArray:
        return "double3[]";
    case RrInputTag::Matrix4dArray:
        return "matrix4d[]";
    case RrInputTag::TokenArray:
        return "token[]";
    case RrInputTag::BoolArray:
        return "bool[]";
    default:
        break;
    }
    return "unknown";
}

bool
RigExecInputValueFrom(const VtValue &value, RrInputTag tag, RrInputValue *out)
{
    if (!out || tag == RrInputTag::Token || RrInputTagIsArray(tag) ||
        !_Holds(value, tag)) {
        return false;
    }
    RrInputValue v;
    v.tag = tag;
    switch (tag) {
    case RrInputTag::Double:
        v.f64 = value.UncheckedGet<double>();
        break;
    case RrInputTag::Float:
        v.f32 = value.UncheckedGet<float>();
        break;
    case RrInputTag::Bool:
        v.boolean = value.UncheckedGet<bool>();
        break;
    case RrInputTag::Int:
        v.i32 = int32_t(value.UncheckedGet<int>());
        break;
    case RrInputTag::Matrix4d: {
        const GfMatrix4d &m = value.UncheckedGet<GfMatrix4d>();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                v.matrix[r][c] = m[r][c];
            }
        }
        break;
    }
    case RrInputTag::Vec3d: {
        const GfVec3d &d = value.UncheckedGet<GfVec3d>();
        v.vec = RrVec3d(d[0], d[1], d[2]);
        break;
    }
    case RrInputTag::Vec3f: {
        const GfVec3f &f = value.UncheckedGet<GfVec3f>();
        v.vec3f = RrVec3f(f[0], f[1], f[2]);
        break;
    }
    case RrInputTag::Vec3i: {
        const auto &i = value.UncheckedGet<GfVec3i>();
        v.vec3i = {int32_t(i[0]), int32_t(i[1]), int32_t(i[2])};
        break;
    }
    case RrInputTag::Token:
        return false;
    }
    *out = v;
    return true;
}

bool
RigExecInputArrayFrom(const VtValue &value, RrInputTag tag,
                      RigExecRuntimeArray *out)
{
    if (!out || !RrInputTagIsArray(tag) || !_Holds(value, tag)) {
        return false;
    }
    out->tag = tag;
    out->count = value.GetArraySize();
    switch (tag) {
    case RrInputTag::IntArray:
        out->data = value.UncheckedGet<VtIntArray>().cdata();
        return true;
    case RrInputTag::FloatArray:
        out->data = value.UncheckedGet<VtFloatArray>().cdata();
        return true;
    case RrInputTag::DoubleArray:
        out->data = value.UncheckedGet<VtDoubleArray>().cdata();
        return true;
    case RrInputTag::Vec2fArray:
        out->data = value.UncheckedGet<VtVec2fArray>().cdata();
        return true;
    case RrInputTag::Vec3fArray:
        out->data = value.UncheckedGet<VtVec3fArray>().cdata();
        return true;
    case RrInputTag::Vec3dArray:
        out->data = value.UncheckedGet<VtVec3dArray>().cdata();
        return true;
    case RrInputTag::Matrix4dArray:
        out->data = value.UncheckedGet<VtMatrix4dArray>().cdata();
        return true;
    default:
        return false;
    }
}

namespace {

// Reads through \p query when given, else through \p a at the live stage.
bool _SampleInputAt(const UsdAttribute &a, const UsdAttributeQuery *query,
                    bool queryFallback, size_t index, const std::string &name,
                    RrInputTag tag, UsdTimeCode time,
                    RigExecRuntimeReader *reader, std::string *error);

} // namespace

bool
RigExecSampleInputAt(const UsdAttribute &a, size_t index,
                     const std::string &name, RrInputTag tag, UsdTimeCode time,
                     RigExecRuntimeReader *reader, std::string *error)
{
    return _SampleInputAt(a, nullptr, false, index, name, tag, time, reader,
                          error);
}

bool
RigExecSampleInputAt(const UsdAttributeQuery &q, size_t index,
                     const std::string &name, RrInputTag tag, UsdTimeCode time,
                     RigExecRuntimeReader *reader, std::string *error)
{
    return _SampleInputAt(q.GetAttribute(), &q, true, index, name, tag, time,
                          reader, error);
}

namespace {

bool
_SampleInputAt(const UsdAttribute &a, const UsdAttributeQuery *query,
               bool queryFallback, size_t index, const std::string &name,
               RrInputTag tag, UsdTimeCode time, RigExecRuntimeReader *reader,
               std::string *error)
{
    // A query returns the schema fallback for a blocked attribute where
    // UsdAttribute::Get returns nothing; a block must read as no value.
    const auto get = [&](auto *out) {
        if (!query) {
            return a.Get(out, time);
        }
        return query->Get(out, time) &&
               !(queryFallback && a.GetResolveInfo(time).ValueIsBlocked());
    };
    // A successful Get holds a value, so it is never blocked; only a failed
    // read asks the resolve whether a block caused it.
    const auto publishBlocked = [&](bool result, bool read) {
        if (!result || !RigExecRuntimeStageArrayInputs::CanSampleProviderValue(
                           *reader, index))
            return result;
        const bool blocked = !read && a.GetResolveInfo(time).ValueIsBlocked();
        return RigExecRuntimeStageArrayInputs::SetSampleBlocked(*reader, index,
                                                                blocked, error);
    };
    if (tag == RrInputTag::Token &&
        RigExecRuntimeStageArrayInputs::CanSampleToken(*reader, index)) {
        TfToken value;
        const bool read = get(&value);
        return publishBlocked(
            read ? RigExecRuntimeStageArrayInputs::SetTokenSample(
                       *reader, index, value.GetString(), error)
                 : RigExecRuntimeStageArrayInputs::ClearTokenSample(
                       *reader, index, error),
            read);
    }
    if (RrInputTagIsArray(tag)) {
        const bool bound =
            RigExecRuntimeStageArrayInputs::CanSample(*reader, index);
        if (!bound) {
            return reader->ResetInput(name, error);
        }
        VtValue value;
        if (!get(&value)) {
            return publishBlocked(RigExecRuntimeStageArrayInputs::ClearSample(
                                      *reader, index, error),
                                  false);
        }
        if (!_Holds(value, tag)) {
            if (error)
                *error = name + ": the stage sample is not a " +
                         RigExecInputTagName(tag);
            return false;
        }
        if (tag == RrInputTag::TokenArray) {
            const auto &tokens = value.UncheckedGet<VtTokenArray>();
            std::vector<std::string> text;
            text.reserve(tokens.size());
            for (const auto &token : tokens)
                text.push_back(token.GetString());
            return publishBlocked(
                RigExecRuntimeStageArrayInputs::SetTokenArraySample(
                    *reader, index, text, error),
                true);
        }
        if (tag == RrInputTag::BoolArray) {
            const auto &bits = value.UncheckedGet<VtBoolArray>();
            std::vector<uint8_t> bytes;
            bytes.reserve(bits.size());
            for (bool bit : bits)
                bytes.push_back(bit ? uint8_t(1) : uint8_t(0));
            const RigExecRuntimeArray array{tag, bytes.data(), bytes.size()};
            return publishBlocked(RigExecRuntimeStageArrayInputs::SetSample(
                                      *reader, index, array, error),
                                  true);
        }
        RigExecRuntimeArray array;
        if (!RigExecInputArrayFrom(value, tag, &array)) {
            if (error)
                *error = name + ": the stage sample is not a " +
                         RigExecInputTagName(tag);
            return false;
        }
        return publishBlocked(RigExecRuntimeStageArrayInputs::SetSample(
                                  *reader, index, array, error),
                              true);
    }
    RrInputValue value;
    value.tag = tag;
    bool read = false;
    switch (tag) {
    case RrInputTag::Double:
        read = get(&value.f64);
        break;
    case RrInputTag::Float:
        read = get(&value.f32);
        break;
    case RrInputTag::Bool:
        read = get(&value.boolean);
        break;
    case RrInputTag::Int: {
        int v = 0;
        read = get(&v);
        value.i32 = int32_t(v);
        break;
    }
    case RrInputTag::Matrix4d: {
        GfMatrix4d v(1.0);
        if ((read = get(&v))) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    value.matrix[r][c] = v[r][c];
                }
            }
        }
        break;
    }
    case RrInputTag::Token: {
        TfToken v;
        if (get(&v)) {
            return publishBlocked(
                reader->SetInputToken(name, v.GetString(), error), true);
        }
        break;
    }
    case RrInputTag::Vec3d: {
        GfVec3d v(0.0);
        if ((read = get(&v))) {
            value.vec = RrVec3d(v[0], v[1], v[2]);
        }
        break;
    }
    case RrInputTag::Vec3f: {
        GfVec3f v(0.0f);
        if ((read = get(&v))) {
            value.vec3f = RrVec3f(v[0], v[1], v[2]);
        }
        break;
    }
    case RrInputTag::Vec3i: {
        GfVec3i v(0);
        if ((read = get(&v)))
            value.vec3i = {int32_t(v[0]), int32_t(v[1]), int32_t(v[2])};
        break;
    }
    }
    // A failed read leaves the input with no value, as the stage has none
    // there; a value is taken as the stage holds it, finite or not.
    if (RigExecRuntimeStageArrayInputs::CanSampleProviderValue(*reader, index))
        return publishBlocked(
            read ? RigExecRuntimeStageArrayInputs::SetScalarSample(
                       *reader, index, value, error)
                 : RigExecRuntimeStageArrayInputs::ClearScalarSample(
                       *reader, index, error),
            read);
    return publishBlocked(read ? reader->SetSampledInputAt(index, value, error)
                               : reader->ClearInputAt(index, error),
                          read);
}

} // namespace

bool
RigExecInputSampler::Bind(const UsdStagePtr &stage,
                          const RigExecRuntimeReader &reader,
                          std::string *error)
{
    _animated.clear();
    _inputStates.clear();
    _animatedCount = 0;
    _warnings.clear();
    _sampled = false;
    _lastApplySampled = false;
    _staticStale = true;
    _lastReads = 0;
    if (!stage) {
        if (error) {
            *error = "the input sampler has no stage to read";
        }
        return false;
    }
    const size_t count = reader.GetInputCount();
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = reader.GetInputInfo(i);
        // Array bindings include private slots through the stage transport below.
        if (RrInputTagIsArray(info.type)) {
            continue;
        }
        const SdfPath path = SdfPath::IsValidPathString(info.name)
                                 ? SdfPath(info.name)
                                 : SdfPath();
        const UsdAttribute attribute = path.IsPropertyPath()
                                           ? stage->GetAttributeAtPath(path)
                                           : UsdAttribute();
        if (!attribute) {
            _warnings.push_back(info.name +
                                ": the stage has no such attribute");
            continue;
        }
        const SdfValueTypeName typeName = attribute.GetTypeName();
        if (!typeName || typeName.IsArray() ||
            typeName.GetType() != RigExecInputTagType(info.type)) {
            _warnings.push_back(info.name + ": the stage's attribute is a " +
                                typeName.GetAsToken().GetString() +
                                ", the input a " +
                                RigExecInputTagName(info.type));
            continue;
        }
        if (info.animated) {
            _Bound bound;
            bound.index = i;
            bound.name = info.name;
            bound.type = info.type;
            bound.attribute = attribute;
            _animatedCount += bound.animated ? 1 : 0;
            _animated.push_back(std::move(bound));
        }
    }
    for (const auto &info : RigExecRuntimeStageArrayInputs::Enumerate(reader)) {
        const SdfPath path(info.name);
        const auto attribute = stage->GetAttributeAtPath(path);
        if (!attribute || attribute.GetTypeName().GetType() !=
                              RigExecInputTagType(info.tag)) {
            _warnings.push_back(info.name + ": no matching stage array");
            continue;
        }
        _Bound bound;
        bound.index = info.slot;
        bound.name = info.name;
        bound.type = info.tag;
        bound.attribute = attribute;
        _animatedCount += bound.animated ? 1 : 0;
        _animated.push_back(std::move(bound));
    }
    for (const auto &info :
         RigExecRuntimeStageArrayInputs::EnumerateTokens(reader)) {
        const auto attribute = stage->GetAttributeAtPath(SdfPath(info.name));
        if (!attribute || attribute.GetTypeName().GetType() !=
                              RigExecInputTagType(info.tag)) {
            _warnings.push_back(info.name + ": no matching stage token");
            continue;
        }
        _Bound bound;
        bound.index = info.slot;
        bound.name = info.name;
        bound.type = info.tag;
        bound.attribute = attribute;
        _animatedCount += bound.animated ? 1 : 0;
        _animated.push_back(std::move(bound));
    }
    for (const auto &info :
         RigExecRuntimeStageArrayInputs::EnumerateProviderValues(reader)) {
        if (std::any_of(
                _animated.begin(), _animated.end(),
                [&](const _Bound &bound) { return bound.index == info.slot; }))
            continue;
        const auto attribute = stage->GetAttributeAtPath(SdfPath(info.name));
        if (!attribute || attribute.GetTypeName().GetType() !=
                              RigExecInputTagType(info.tag)) {
            _warnings.push_back(info.name +
                                ": no matching stage provider input");
            continue;
        }
        _Bound bound;
        bound.index = info.slot;
        bound.name = info.name;
        bound.type = info.tag;
        bound.attribute = attribute;
        bound.animated = info.animated;
        _animatedCount += bound.animated ? 1 : 0;
        _animated.push_back(std::move(bound));
    }
    for (_Bound &bound : _animated) {
        bound.isStatic = _StaticInput(bound.animated, bound.attribute);
        if (_inputStates.size() <= bound.index) {
            _inputStates.resize(bound.index + 1);
        }
        _inputStates[bound.index].bound = true;
    }
    _last = UsdTimeCode(reader.GetBakeTime());
    _sampled = true;
    return true;
}

bool
RigExecInputSampler::Apply(UsdTimeCode time, RigExecRuntimeReader *reader,
                           std::string *error, bool *sampled)
{
    _lastApplySampled = false;
    if (sampled) {
        *sampled = false;
    }
    if (_sampled && time == _last) {
        return true;
    }
    for (const _Bound &bound : _animated) {
        _inputStates[bound.index].sampled = false;
    }
    // A refresh reads every input and re-decides which are static: keys
    // authored since the last one make an input varying, removed keys make
    // it static again. Default and a numeric time can read different values
    // of an attribute keyed once, so a change of Default-ness refreshes too.
    const bool refresh = _staticStale || time.IsDefault() != _last.IsDefault();
    if (refresh) {
        for (_Bound &bound : _animated) {
            bound.query = _staticSkip ? UsdAttributeQuery(bound.attribute)
                                      : UsdAttributeQuery();
            VtValue fallback;
            bound.hasFallback =
                _staticSkip && bound.attribute.GetPrim()
                                   .GetPrimDefinition()
                                   .GetAttributeFallbackValue(
                                       bound.attribute.GetName(), &fallback);
            bound.isStatic = _StaticInput(bound.animated, bound.attribute);
        }
    }
    const bool skipStatic = _staticSkip && !refresh;
    _lastReads = 0;
    for (const _Bound &bound : _animated) {
        // Static sources retain their last sample until a refresh. Callers
        // layering values over them use DidSampleInput to detect replacement.
        if (skipStatic && bound.isStatic) {
            continue;
        }
        ++_lastReads;
        if (!_SampleInputAt(bound.attribute,
                            bound.query.IsValid() ? &bound.query : nullptr,
                            bound.hasFallback, bound.index, bound.name,
                            bound.type, time, reader, error)) {
            // Earlier slots may already have replaced their caller's values.
            // A retry must resample all slots, even at the last good time.
            Invalidate();
            return false;
        }
        _inputStates[bound.index].sampled = true;
        _lastApplySampled = true;
    }
    reader->TouchAnimatedInputs();
    _last = time;
    _sampled = true;
    _staticStale = false;
    if (sampled) {
        *sampled = true;
    }
    return true;
}

} // namespace rigExec
