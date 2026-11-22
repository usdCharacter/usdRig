#include "rigExec/api/programView.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/bakedProgramImpl.h"

namespace rigExec {

size_t
RigExecProgramView::ClusterCount() const
{
    return _program ? _program->GetStepGraph().clustering.clusters.size() : 0;
}

UsdTimeCode
RigExecProgramView::LastTime() const
{
    return _program ? _program->GetStepGraph().lastTime
                    : UsdTimeCode::Default();
}

} // namespace rigExec
