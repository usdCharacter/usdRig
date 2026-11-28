#ifndef RIGEXEC_OPS_DEFORM_SURFACE_PROJECT_OP_H
#define RIGEXEC_OPS_DEFORM_SURFACE_PROJECT_OP_H

/// SurfaceProject: points snapped onto, inside or outside a surface mesh.
/// Default settings with identity spaces are the legacy projection
/// (surfaceProjectKernel.h), weighted by Strength; anything else is the snap
/// kernel (surfaceSnapKernel.h).

#include "rigExecMath/surfaceSnapKernel.h"
#include "rigExecOps/library.h"

namespace rigExecOps {

/// What a surface projection is configured with at compile.
struct SurfaceProjectPayload {
    rigExec::RigExecSurfaceSnapSettings settings;

    bool operator==(const SurfaceProjectPayload &o) const
    {
        return settings == o.settings;
    }
};

const rigExec::RigExecOpHandle &SurfaceProject();

// Points and Surface (Vec3fArray); Counts and Indices (IntArray, the
// surface's faces); Strength (optional float, 1 when unfed; the legacy
// projection only); TargetToSurface, SurfaceToTarget and SurfaceToMetric
// (optional Mat4d, identity when unfed); Envelope (optional FloatArray);
// Out. A snap the kernel refuses is an error.
const rigExec::RigExecPortHandle &SurfaceProjectPoints();
const rigExec::RigExecPortHandle &SurfaceProjectSurface();
const rigExec::RigExecPortHandle &SurfaceProjectCounts();
const rigExec::RigExecPortHandle &SurfaceProjectIndices();
const rigExec::RigExecPortHandle &SurfaceProjectStrength();
const rigExec::RigExecPortHandle &SurfaceProjectTargetToSurface();
const rigExec::RigExecPortHandle &SurfaceProjectSurfaceToTarget();
const rigExec::RigExecPortHandle &SurfaceProjectSurfaceToMetric();
const rigExec::RigExecPortHandle &SurfaceProjectEnvelope();
const rigExec::RigExecPortHandle &SurfaceProjectOut();

} // namespace rigExecOps

#endif
