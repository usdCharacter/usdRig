#ifndef RIGEXEC_API_COMPILE_H
#define RIGEXEC_API_COMPILE_H

/// Compiles a rig description into an immutable program. USD-free.

#include "rigExec/api/rigDesc.h"

#include <cstddef>
#include <memory>
#include <string>

namespace rigExec {

class RigExecProgram;

struct RigExecCompileOptions {
    /// Most nodes in one batch; 0 puts every node of one op at one level in
    /// a single batch. Changes dispatch only, never results.
    size_t maxBatchNodes = 0;
};

/// Null with \p error when an input is unfed or fed twice, a connection
/// joins different types, or the order has a cycle.
std::shared_ptr<const RigExecProgram>
RigExecCompile(const RigExecRigDesc &desc,
               const RigExecCompileOptions &options = {},
               std::string *error = nullptr);

} // namespace rigExec

#endif
