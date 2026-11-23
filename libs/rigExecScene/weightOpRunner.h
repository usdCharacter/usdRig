#ifndef RIGEXEC_SCENE_WEIGHT_OP_RUNNER_H
#define RIGEXEC_SCENE_WEIGHT_OP_RUNNER_H

/// A weight binding's combine records run as CombineWeights op nodes, one
/// per record, built on first use and rebuilt when the payload or the input
/// count baked into it moved. Owner-thread state of the scene runtime.

#include "rigExec/api/compile.h"
#include "rigExecScene/weightProgram.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecCombineOpNode;

class RigExecWeightCombineOps final : public RigExecWeightCombineRunner {
public:
    RigExecWeightCombineOps();
    ~RigExecWeightCombineOps() override;
    RigExecWeightCombineOps(RigExecWeightCombineOps &&) noexcept;
    RigExecWeightCombineOps &operator=(RigExecWeightCombineOps &&) noexcept;

    bool Combine(size_t record, const RigExecWeightRecord &, size_t count,
                 float strength, float invert, Resolve resolve,
                 const void *context, std::vector<float> *out,
                 std::string *error) override;

    /// The packet combine of \p record. A refusing op gives the bare packet
    /// a result outside the range policy gives.
    RigExecWeightPacket
    CombinePackets(size_t record, const RigExecWeightRecord &,
                   const std::vector<const RigExecWeightPacket *> &inputs,
                   size_t targetCount, float strength, float invert);

    RigExecCompileOptions compile;

private:
    std::vector<std::unique_ptr<RigExecCombineOpNode>> _nodes;
};

} // namespace rigExec

#endif
