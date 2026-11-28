#ifndef RIGEXEC_TESTS_WEIGHT_OP_RUNNER_H
#define RIGEXEC_TESTS_WEIGHT_OP_RUNNER_H

/// Weight combine records run as CombineWeights op nodes, one per record,
/// for tests that compare the op with the native combine. Built on first use
/// and rebuilt when the payload or the input count baked into it moved.

#include "rigExec/api/evaluator.h"
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

    /// The packet combine of \p record into \p packet. False with the
    /// compile or evaluation error where the op fails.
    bool CombinePackets(size_t record, const RigExecWeightRecord &,
                        const std::vector<const RigExecWeightPacket *> &inputs,
                        size_t targetCount, float strength, float invert,
                        RigExecWeightPacket *packet, std::string *error);

    RigExecCompileOptions compile;
    RigExecEvaluatorOptions evaluate;

private:
    std::vector<std::unique_ptr<RigExecCombineOpNode>> _nodes;
};

} // namespace rigExec

#endif
