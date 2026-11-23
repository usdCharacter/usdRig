#ifndef RIGEXEC_OPS_CONSTRAINT_MATCH_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_MATCH_OPS_H

/// IK/FK match and space match: the channel values that keep a control where it
/// is once a switch moves, as ordinary outputs. The kernels are
/// rigExecMath/ikFkMatchKernel.h and spaceMatchKernel.h, and the space switch
/// rigExecMath/poseArithmeticKernel.h.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExecOps {

/// Doubles per control on a Channels port: t3, s3, r3, spin, sign3, units,
/// then the rotation order as three character codes of 'X', 'Y' and 'Z'.
inline constexpr size_t kMatchChannelStride = 18;

/// A space switch's compiled setup (RigExecSpaceSwitchRecordT and the
/// static parts of its inputs).
struct SpaceSwitchSetup {
    /// Per source: 0 all, 1 twist, 2 swing, 3 orient; empty for all.
    std::vector<int> filters;
    rigExecStd::Vec3d twistAxis{1.0, 0.0, 0.0};
    std::array<bool, 3> affectTranslation{{true, true, true}};
    std::array<bool, 3> affectRotation{{true, true, true}};
    std::array<bool, 3> affectScale{{true, true, true}};
    /// The SpaceDefault and SpacePosed ports are fed.
    bool hasCarry = false;
    /// Per source: a world source, whose space ports entries are unread.
    std::vector<uint8_t> world;

    bool operator==(const SpaceSwitchSetup &o) const
    {
        return filters == o.filters && twistAxis == o.twistAxis &&
               affectTranslation == o.affectTranslation &&
               affectRotation == o.affectRotation &&
               affectScale == o.affectScale && hasCarry == o.hasCarry &&
               world == o.world;
    }
};

struct IkFkMatchPayload {
    SpaceSwitchSetup poleSwitch;
    /// Per pole switch source: it rides the IK control.
    std::vector<uint8_t> poleSourceRides;
    double ikValue = 1.0;
    double scaleCalibration = 0.0;
    double upperLengthOffset = 0.0;
    double lowerLengthOffset = 0.0;
    bool stretchLimb = true;

    bool operator==(const IkFkMatchPayload &o) const
    {
        return poleSwitch == o.poleSwitch &&
               poleSourceRides == o.poleSourceRides && ikValue == o.ikValue &&
               scaleCalibration == o.scaleCalibration &&
               upperLengthOffset == o.upperLengthOffset &&
               lowerLengthOffset == o.lowerLengthOffset &&
               stretchLimb == o.stretchLimb;
    }
};

/// How the pole's parent follows the IK control the match moves.
enum class IkFkPoleMode : uint32_t { Keep, Switch, Ride };

const rigExec::RigExecOpHandle &IkFkMatch();

// Frames are Mat4d, row vectors. Enabled (Bool); Weight and the limb's
// Switch, Stretch, SoftDistance, UpperScale, LowerScale, Pin and Twist
// (degrees) (Double); Sources (Mat4dArray: joints 0-2, IK root 3, effector
// 4, IK control 5, pole 6, FK controls 7-9); Rests (Mat4dArray: the joints'
// at 0-2, the five controls' at 3-7); Channels (DoubleArray, the five
// controls' at kMatchChannelStride each); HasSpace (Bool) with SpaceRest and
// SpacePosed (Mat4d), the IK solve's space. PoleMode (UInt, an
// IkFkPoleMode) and the pole switch's ports, read in Switch mode.
// IkValues (DoubleArray, 11) and FkValues (DoubleArray, 20), Valid (Bool),
// Shown (Mat4dArray, 5: the IK control, pole and FK controls as presented)
// and ShownMask (UInt, which of Shown are presented).
const rigExec::RigExecPortHandle &IkFkMatchEnabled();
const rigExec::RigExecPortHandle &IkFkMatchWeight();
const rigExec::RigExecPortHandle &IkFkMatchSwitch();
const rigExec::RigExecPortHandle &IkFkMatchStretch();
const rigExec::RigExecPortHandle &IkFkMatchSoftDistance();
const rigExec::RigExecPortHandle &IkFkMatchUpperScale();
const rigExec::RigExecPortHandle &IkFkMatchLowerScale();
const rigExec::RigExecPortHandle &IkFkMatchPin();
const rigExec::RigExecPortHandle &IkFkMatchTwist();
const rigExec::RigExecPortHandle &IkFkMatchSources();
const rigExec::RigExecPortHandle &IkFkMatchRests();
const rigExec::RigExecPortHandle &IkFkMatchChannels();
const rigExec::RigExecPortHandle &IkFkMatchHasSpace();
const rigExec::RigExecPortHandle &IkFkMatchSpaceRest();
const rigExec::RigExecPortHandle &IkFkMatchSpacePosed();
const rigExec::RigExecPortHandle &IkFkMatchPoleMode();
const rigExec::RigExecPortHandle &IkFkMatchPosedDefault();
const rigExec::RigExecPortHandle &IkFkMatchParentDefaultInverse();
const rigExec::RigExecPortHandle &IkFkMatchParentPosed();
const rigExec::RigExecPortHandle &IkFkMatchParentDefault();
const rigExec::RigExecPortHandle &IkFkMatchSwitchSpaceDefault();
const rigExec::RigExecPortHandle &IkFkMatchSwitchSpacePosed();
const rigExec::RigExecPortHandle &IkFkMatchSourceDefaults();
const rigExec::RigExecPortHandle &IkFkMatchSourcePosed();
const rigExec::RigExecPortHandle &IkFkMatchActive();
const rigExec::RigExecPortHandle &IkFkMatchIkValues();
const rigExec::RigExecPortHandle &IkFkMatchFkValues();
const rigExec::RigExecPortHandle &IkFkMatchValid();
const rigExec::RigExecPortHandle &IkFkMatchShown();
const rigExec::RigExecPortHandle &IkFkMatchShownMask();

struct SpaceMatchPayload {
    SpaceSwitchSetup space;

    bool operator==(const SpaceMatchPayload &o) const
    {
        return space == o.space;
    }
};

const rigExec::RigExecOpHandle &SpaceMatch();

// Target (Mat4d, the control's composed frame), Channels (DoubleArray, one
// control at kMatchChannelStride), and the switch's PosedDefault,
// ParentDefaultInverse, ParentPosed, ParentDefault, optional SpaceDefault
// and SpacePosed (Mat4d), SourceDefaults and SourcePosed (Mat4dArray, one
// per source). Values (DoubleArray, RigExecSpaceMatchStride per source) and
// Valid (Bool).
const rigExec::RigExecPortHandle &SpaceMatchTarget();
const rigExec::RigExecPortHandle &SpaceMatchChannels();
const rigExec::RigExecPortHandle &SpaceMatchPosedDefault();
const rigExec::RigExecPortHandle &SpaceMatchParentDefaultInverse();
const rigExec::RigExecPortHandle &SpaceMatchParentPosed();
const rigExec::RigExecPortHandle &SpaceMatchParentDefault();
const rigExec::RigExecPortHandle &SpaceMatchSpaceDefault();
const rigExec::RigExecPortHandle &SpaceMatchSpacePosed();
const rigExec::RigExecPortHandle &SpaceMatchSourceDefaults();
const rigExec::RigExecPortHandle &SpaceMatchSourcePosed();
const rigExec::RigExecPortHandle &SpaceMatchValues();
const rigExec::RigExecPortHandle &SpaceMatchValid();

} // namespace rigExecOps

#endif
