#ifndef RIGEXEC_REGISTER_ORACLE_H
#define RIGEXEC_REGISTER_ORACLE_H

namespace rigExec {

/// Enables CPU reference checks and golden capture. CMake consumers linking
/// rigExec::rigExecOracle call this automatically; manual link users call it
/// before using these features. Repeated calls are safe. Once registered,
/// rigExecOracle must remain loaded for the process lifetime.
void RigExecRegisterOracle();

}
#endif
