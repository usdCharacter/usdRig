#ifndef RIGEXEC_OPS_DEFORM_PROJECTOR_OPS_H
#define RIGEXEC_OPS_DEFORM_PROJECTOR_OPS_H

/// Matrix outputs measured from a deforming surface. The kernel is
/// rigExecMath/surfaceProjectorKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>
#include "rigExecStdTypes/stdTypes.h"

namespace rigExecOps {

/// What a surface projector is configured with at compile.
struct SurfaceProjectorPayload {
    rigExecStd::Vec3d rayOrigin{0.0, 0.0, 0.0};
    rigExecStd::Vec3d rayDirection{0.0, 0.0, 1.0};
    rigExecStd::Vec3d rayUp{0.0, 1.0, 0.0};
    rigExecStd::Mat4d shaderOffset{1.0};
    bool reproject = false;
    /// A source space or rig space was named; unfed, the kernel keeps the
    /// authored ray and reports the missing space.
    bool sourceSpaceNamed = false;
    bool spaceNamed = false;

    bool operator==(const SurfaceProjectorPayload &o) const
    {
        return rayOrigin == o.rayOrigin && rayDirection == o.rayDirection &&
               rayUp == o.rayUp && shaderOffset == o.shaderOffset &&
               reproject == o.reproject &&
               sourceSpaceNamed == o.sourceSpaceNamed &&
               spaceNamed == o.spaceNamed;
    }
};

/// The projector's target-local shader matrix: shaderOffset carried by its
/// source and by how the material point under its ray moved from
/// BasePoints to Points.
const rigExec::RigExecOpHandle &SurfaceProjector();
/// Up to sixteen dials packed row-major into one matrix; a non-finite dial
/// is zero.
const rigExec::RigExecOpHandle &ShaderDials();

/// SurfaceProjectorReason when the projector measured a frame, and when it
/// measured none and the kernel noted nothing; otherwise the reason is the
/// kernel's first note's kind (RigExecSurfaceProjectorNote::Kind).
inline constexpr uint32_t kSurfaceProjectorSolved = 0;
inline constexpr uint32_t kSurfaceProjectorNoFrameNoted = 0xffffffffu;

/// SurfaceProjectorProbe bits: the first note's cast, re-cast, rest frame and
/// posed frame.
enum SurfaceProjectorProbeBits : uint32_t {
    kSurfaceProjectorCastHit = 1u << 0,
    kSurfaceProjectorRecastHit = 1u << 1,
    kSurfaceProjectorRestFrame = 1u << 2,
    kSurfaceProjectorPosedFrame = 1u << 3,
};

// SurfaceProjector: BasePoints and Points (Vec3fArray), Counts and Indices
// (IntArray); optional SourceBase and SourceFinal (Mat4d, together),
// SourceSpaceBase and SourceSpaceFinal (together), SpaceFinal and
// WorldToMesh (identity when unfed); Out (Mat4d, identity unless Reason is
// kSurfaceProjectorSolved), Reason (UInt), and the first note's Ray
// (DoubleArray: origin, direction, up) and Probe (UInt, probe bits).
const rigExec::RigExecPortHandle &SurfaceProjectorBasePoints();
const rigExec::RigExecPortHandle &SurfaceProjectorPoints();
const rigExec::RigExecPortHandle &SurfaceProjectorCounts();
const rigExec::RigExecPortHandle &SurfaceProjectorIndices();
const rigExec::RigExecPortHandle &SurfaceProjectorSourceBase();
const rigExec::RigExecPortHandle &SurfaceProjectorSourceFinal();
const rigExec::RigExecPortHandle &SurfaceProjectorSourceSpaceBase();
const rigExec::RigExecPortHandle &SurfaceProjectorSourceSpaceFinal();
const rigExec::RigExecPortHandle &SurfaceProjectorSpaceFinal();
const rigExec::RigExecPortHandle &SurfaceProjectorWorldToMesh();
const rigExec::RigExecPortHandle &SurfaceProjectorOut();
const rigExec::RigExecPortHandle &SurfaceProjectorReason();
const rigExec::RigExecPortHandle &SurfaceProjectorRay();
const rigExec::RigExecPortHandle &SurfaceProjectorProbe();

// ShaderDials: Dials (DoubleArray), Out (Mat4d).
const rigExec::RigExecPortHandle &ShaderDialsDials();
const rigExec::RigExecPortHandle &ShaderDialsOut();

} // namespace rigExecOps

#endif
