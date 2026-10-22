#ifndef RIGEXEC_GRAPH_SCENE_PROGRAM_TRANSLATION_H
#define RIGEXEC_GRAPH_SCENE_PROGRAM_TRANSLATION_H
#include "sceneProgram.h"
namespace rigExec {
bool RigExecTranslateSceneProgram(const RigExecSceneDescriptors &,RigExecSceneProgram *,
    std::string *error=nullptr);
}
#endif
