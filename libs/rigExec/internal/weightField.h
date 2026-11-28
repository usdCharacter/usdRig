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
/// Per weight object, whether node \p part of field \p field reads its
/// inputs: the objects of the node's subtree, or every object of the field
/// for its root.
std::vector<char>
RigExecBakedWeightFieldNodeObjects(const RigExecBakedProgramImpl &, int field,
                                   int part);
/// Whether a node reading \p objects of field \p field reads its
/// revision's entering points, and so waits on the chain's earlier
/// revisions.
bool RigExecBakedWeightFieldNodeReadsChain(const RigExecBakedProgramImpl &,
                                           int field,
                                           const std::vector<char> &objects);
/// What a weight field op step reads (its node's objects' inputs), and the
/// field's root node writes.
void RigExecBakedDeclareWeightField(RigExecBakedProgramImpl *,
                                    RigExecBakedStep *);
void RigExecBakedWeightFieldCoveredVersions(const RigExecBakedProgramImpl &,
                                            int field,
                                            std::vector<uint32_t> *versions);
/// Materialize only the canonical compiler's SCC exclusion outcome.
void RigExecBakedBindWeightCycleState(RigExecBakedProgramImpl *);
} // namespace rigExec
#endif
