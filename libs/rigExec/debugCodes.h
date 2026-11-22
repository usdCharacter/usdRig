// RigExec debug codes.
// Off by default and free at runtime (one array lookup). Enable with
// TF_DEBUG=RIGEXEC_TAP_TIMING, or TF_DEBUG=RIGEXEC_* for all of them.
// Outside rigExec: TF_DEBUG_CODES specializes a pxr template (C2888).
#ifndef RIGEXEC_DEBUG_CODES_H
#define RIGEXEC_DEBUG_CODES_H

#include "pxr/pxr.h"
#include "pxr/base/tf/debug.h"

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEBUG_CODES(
    /// Wall time inside the ExecUsdSystem calls made by RigExecTapSet:
    /// BuildRequest, PrepareRequest (compilation) and Compute (execution),
    /// plus the tap count each request carries.
    RIGEXEC_TAP_TIMING
);

#endif  // RIGEXEC_DEBUG_CODES_H
