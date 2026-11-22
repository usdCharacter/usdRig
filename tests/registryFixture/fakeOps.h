#ifndef RIGEXEC_TESTS_REGISTRY_FIXTURE_FAKE_OPS_H
#define RIGEXEC_TESTS_REGISTRY_FIXTURE_FAKE_OPS_H

#include "rigExec/api/registry.h"

namespace fakeOps {

/// out = in * payload.scale
const rigExec::RigExecOpHandle &Scale();
const rigExec::RigExecPortHandle &ScaleIn();
const rigExec::RigExecPortHandle &ScaleOut();

/// out = in + 1.
const rigExec::RigExecOpHandle &Offset();
const rigExec::RigExecPortHandle &OffsetIn();
const rigExec::RigExecPortHandle &OffsetOut();

struct ScalePayload {
    float scale = 1.0f;
};

} // namespace fakeOps

#endif
