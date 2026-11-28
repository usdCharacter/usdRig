#ifndef RIGEXEC_OPS_DEFORM_RIBBON_OP_H
#define RIGEXEC_OPS_DEFORM_RIBBON_OP_H

/// Point ops driven by a solver's frames. Ribbon's kernel is
/// rigExecMath/ribbonKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

/// Each point moves by the rigid map from its rest frame sample to its
/// posed frame sample, lerped between the two samples around its bind u.
const rigExec::RigExecOpHandle &Ribbon();
/// Point i becomes frame i's origin: a solver's guides as points.
const rigExec::RigExecOpHandle &GuidePoints();

// Points (Vec3fArray), BindCoords (Vec2fArray, one per point; u in [0, 1]),
// Frames (rigExecRig PointFrameArray, at least two samples, rests parallel
// to frames), optional Envelope (FloatArray, as the point ops), Out. An
// unsolvable sample map is identity; anything else that does not fit is
// an error.
const rigExec::RigExecPortHandle &RibbonPoints();
const rigExec::RigExecPortHandle &RibbonBindCoords();
const rigExec::RigExecPortHandle &RibbonFrames();
const rigExec::RigExecPortHandle &RibbonEnvelope();
const rigExec::RigExecPortHandle &RibbonOut();

// GuidePoints: Points (one per frame), Frames (rests parallel to frames),
// optional Envelope, Out; Enabled, DefaultWeight, Packet and Written as
// VolumeCorrect's. Runs once over every point.
const rigExec::RigExecPortHandle &GuidePointsPoints();
const rigExec::RigExecPortHandle &GuidePointsFrames();
const rigExec::RigExecPortHandle &GuidePointsEnvelope();
const rigExec::RigExecPortHandle &GuidePointsOut();
const rigExec::RigExecPortHandle &GuidePointsEnabled();
const rigExec::RigExecPortHandle &GuidePointsDefaultWeight();
const rigExec::RigExecPortHandle &GuidePointsPacket();
const rigExec::RigExecPortHandle &GuidePointsWritten();

} // namespace rigExecOps

#endif
