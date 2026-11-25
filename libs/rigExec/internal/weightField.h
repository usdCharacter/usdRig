#ifndef RIGEXEC_WEIGHT_FIELD_H
#define RIGEXEC_WEIGHT_FIELD_H
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace rigExec {
class RigExecRigEvaluator;
struct RigExecBakedProgramImpl;
struct RigExecBakedStep;
bool RigExecBakedBuildWeightFields(const RigExecRigEvaluator &,
                                   RigExecBakedProgramImpl *,
                                   std::string *error);
/// The weight field whose op node \p step runs, or -1.
int RigExecBakedWeightFieldOf(const RigExecBakedProgramImpl &,
                              const RigExecBakedStep &step);
/// Adds an OpNode step per weight object of field \p field, each after the
/// objects it composes; \p head marks envelope fields' steps head steps.
void RigExecBakedAddWeightFieldSteps(RigExecBakedProgramImpl *, int field,
                                     bool head);
/// What a weight field op step reads, and the field's root node writes.
void RigExecBakedDeclareWeightField(RigExecBakedProgramImpl *,
                                    RigExecBakedStep *);
void RigExecBakedWeightFieldCoveredVersions(const RigExecBakedProgramImpl &,
                                            int field,
                                            std::vector<uint32_t> *versions);
/// Materialize only the canonical compiler's SCC exclusion outcome.
void RigExecBakedBindWeightCycleState(RigExecBakedProgramImpl *);
} // namespace rigExec
#endif
