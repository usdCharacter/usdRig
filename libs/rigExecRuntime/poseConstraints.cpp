// Runtime constraints and their op nodes' lookups and stores.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

bool
RrLookUpConstraint(RrProgram *program, eval::ConstraintStepCache &cache,
                   const eval::OpLookup &lookup, const RigExecTypeDesc &type,
                   void *value, std::string *error)
{
    RrPoseHost host(program);
    return eval::LookUpConstraint(host, cache, lookup, type, value, error);
}

bool
RrStoreConstraint(RrProgram *program, size_t step,
                  eval::ConstraintStepCache &cache,
                  const eval::OpHostOutput &output, const void *value,
                  std::string *error)
{
    RrPoseHost host(program, step);
    return eval::StoreConstraint(host, cache, output, value, error);
}

std::string
RrConstraintFailure(const RrProgram *program, uint32_t commit,
                    const std::string &error)
{
    const RrProgram &p = *program;
    if (commit >= p.poses->commits.size())
        return error;
    const std::string mover = p.TextOrEmpty(p.poses->commits[commit].moverPath);
    return mover.empty() ? error : mover + ": " + error;
}

} // namespace rigExec
