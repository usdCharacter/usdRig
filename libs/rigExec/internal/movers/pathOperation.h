// The float math mover's `path` operation as compile reads it: the curve
// rigExec:path names and the operation's uniform settings. Owning thread;
// the per-frame read is the curve's points, which the caller walks like any
// other input.
#ifndef RIGEXEC_MOVERS_PATH_OPERATION_H
#define RIGEXEC_MOVERS_PATH_OPERATION_H

#include "rigExecMath/pathCurveKernel.h"

#include "pxr/pxr.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <string>
#include <vector>

namespace rigExec {

struct RigExecPathBinding {
    /// settings.valid is false when the mover does not name exactly one
    /// supported curve.
    RigExecPathSettings settings;
    /// The UsdGeomBasisCurves prim and its points attribute.
    PXR_NS::SdfPath curve;
    PXR_NS::UsdAttribute points;
    /// Every authored property the settings were read from: the mover's
    /// relationship and uniforms and the curve's type, basis, wrap and
    /// curveVertexCounts. An edit to any of them changes the binding.
    std::vector<PXR_NS::SdfPath> structural;
};

/// Binds \p mover's path operation. False with \p error (when given) where
/// the binding is not valid; \p out is filled either way, with
/// settings.valid false on failure.
bool RigExecBindPathOperation(const PXR_NS::UsdPrim &mover,
                              RigExecPathBinding *out,
                              std::string *error = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_MOVERS_PATH_OPERATION_H
