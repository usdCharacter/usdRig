#ifndef RIGEXEC_REGISTER_SCENE_H
#define RIGEXEC_REGISTER_SCENE_H

namespace rigExec {

/// Enables scene-backed evaluator compilation. CMake consumers linking
/// rigExec::rigExecScene call this automatically; manual link users call it
/// before Compile or Evaluate. Repeated calls are safe. Once registered,
/// rigExecScene must remain loaded for the process lifetime.
void RigExecRegisterScene();

}
#endif
