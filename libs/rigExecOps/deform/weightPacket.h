#ifndef RIGEXEC_OPS_DEFORM_WEIGHT_PACKET_H
#define RIGEXEC_OPS_DEFORM_WEIGHT_PACKET_H

/// A weight object's packet (PointWeights) as the point ops read it.

#include "rigExecEvalCore/envelope.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExecOps::weightPacket {

using rigExecRig::PointWeights;
using rigExecRig::WeightRepresentation;

/// A weight packet as the envelope predicates read it.
inline rigExec::eval::EnvelopePacketView
PacketView(const PointWeights &packet)
{
    rigExec::eval::EnvelopePacketView view;
    const auto representation = WeightRepresentation(packet.representation);
    view.valid = packet.valid != 0;
    view.representation = representation == WeightRepresentation::Constant
                              ? rigExec::eval::kWeightConstant
                          : representation == WeightRepresentation::Dense
                              ? rigExec::eval::kWeightDense
                              : rigExec::eval::kWeightSparse;
    view.rangePolicy = rigExecRig::WeightRange(packet.rangePolicy) ==
                               rigExecRig::WeightRange::Clamp
                           ? rigExec::eval::kWeightPolicyClamp
                           : rigExec::eval::kWeightPolicyStrict;
    view.defaultWeight = packet.defaultWeight;
    view.values = packet.values.Data();
    view.valueCount = packet.values.Size();
    view.indices = packet.indices.Data();
    view.indexCount = packet.indices.Size();
    return view;
}

/// \p packet's weight at each of \p count points into \p out, validated as
/// a weight packet's ResolveAll validates it; false, \p out untouched, when
/// it does not resolve.
inline bool
ResolvePacket(const PointWeights &packet, size_t count, std::vector<float> *out)
{
    const auto usable = [](float v) {
        return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
    };
    const auto representation = WeightRepresentation(packet.representation);
    const float *values = packet.values.Data();
    const int32_t *indices = packet.indices.Data();
    const size_t valueCount = packet.values.Size();
    const size_t indexCount = packet.indices.Size();
    if (!packet.valid)
        return false;
    if (representation == WeightRepresentation::Constant) {
        if (valueCount || indexCount || !usable(packet.defaultWeight))
            return false;
        out->assign(count, packet.defaultWeight);
        return true;
    }
    if (representation == WeightRepresentation::Dense) {
        if (indexCount || valueCount != count)
            return false;
        for (size_t i = 0; i < count; ++i)
            if (!usable(values[i]))
                return false;
        out->assign(values, values + count);
        return true;
    }
    if (indexCount != valueCount ||
        (indexCount < count && !usable(packet.defaultWeight)))
        return false;
    for (size_t i = 0; i < indexCount; ++i)
        if (indices[i] < 0 || size_t(indices[i]) >= count ||
            (i > 0 && indices[i] <= indices[i - 1]) || !usable(values[i]))
            return false;
    out->assign(count, packet.defaultWeight);
    for (size_t i = 0; i < indexCount; ++i)
        (*out)[size_t(indices[i])] = values[i];
    return true;
}

} // namespace rigExecOps::weightPacket

#endif
