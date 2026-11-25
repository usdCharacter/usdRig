#ifndef RIGEXEC_BAKE_OP_GRAPH_H
#define RIGEXEC_BAKE_OP_GRAPH_H

#include "rigExecBinary/format.h"

#include <string>

namespace rigExec {

struct RigExecBakedProgramImpl;

/// Writes \p program's op nodes into \p file's op_graph and their ops and
/// output types into its manifest, which must exist. False with the reason
/// when a node's settings cannot be saved or an input is fed by something
/// the file cannot hold.
bool RigExecBakeOpGraph(const RigExecBakedProgramImpl &program,
                        fb::RigExecWireFile *file, std::string *error);

} // namespace rigExec

#endif
