// Driven action: dial angle -> whole action frame -> channel values. The one
// numerical body behind RigExecDrivenAction in native, frozen and .rigexec
// evaluation.
//
// USD-free and header-only: the runtime instantiates it. No locks or
// statics.
//
// The side, wrap, tie and channel rules have no public source; they come
// from an unpublished internal specification (see docs/references.md,
// "2D skeleton rules without a public source").
//
// Units: driver samples and the angle are values of the driver property
// itself, normally a dial's avars:rz (degrees from its setup angle), so
// samples[0] is 0 for a dial whose action frame 0 is its setup.
#ifndef RIGEXEC_MATH_DRIVEN_ACTION_KERNEL_H
#define RIGEXEC_MATH_DRIVEN_ACTION_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

/// Samples within this many degrees of one revolution wrap.
inline constexpr double kRigExecDrivenActionFullTurnTolerance = 1.0;
inline constexpr double kRigExecDrivenActionFullTurn = 360.0;

/// Outputs before the channels: frame, weight, active.
inline constexpr size_t kRigExecDrivenActionFixedOutputs = 3;

/// Immutable compiled table of one driven action. Every field is structural:
/// the digest hashes the attributes it comes from, so an edit recompiles.
struct RigExecDrivenActionTable {
    std::vector<double> driver;                 ///< samples 0..N, driver units
    int side = 0;                               ///< authored rigExec:side, else RigExecDrivenActionSide
    bool fullTurn = false;
    bool tiesLater = false;                     ///< rigExec:frameTies == later
    /// One sample row per RigExecActionChannel child, in publication order.
    std::vector<std::vector<double>> channelSamples;
    std::vector<char> channelOffset;            ///< 1 offset, 0 value
    size_t OutputCount() const {
        return kRigExecDrivenActionFixedOutputs + channelSamples.size();
    }
    bool operator==(const RigExecDrivenActionTable &o) const {
        return driver == o.driver && side == o.side && fullTurn == o.fullTurn &&
               tiesLater == o.tiesLater && channelSamples == o.channelSamples &&
               channelOffset == o.channelOffset;
    }
};

/// The side of sample 0 an action serves when its rigExec:side is
/// fromSamples, judged once at compile from the samples alone: +1, -1, or
/// 0 when the driver never leaves sample 0 at frame 1 or at its end. When
/// the sample at frame 1 and the last sample lie on opposite sides, the
/// last sample decides and \p disagree is set (a compile note). An authored
/// rigExec:side replaces this.
inline int RigExecDrivenActionSide(const double *driver, size_t count,
    bool *disagree)
{
    if (disagree) *disagree = false;
    if (!driver || count < 2) return 0;
    const auto sign = [](double d) { return d > 0.0 ? 1 : d < 0.0 ? -1 : 0; };
    const int first = sign(driver[1] - driver[0]);
    const int last = sign(driver[count - 1] - driver[0]);
    if (first != 0 && last != 0 && first != last && disagree) *disagree = true;
    return last != 0 ? last : first;
}

/// Whether the driver turns one full revolution (wrap before searching):
/// |samples[N] - samples[0]| within the tolerance of 360.
inline bool RigExecDrivenActionFullTurn(const double *driver, size_t count)
{
    if (!driver || count < 2) return false;
    const double travel = std::abs(driver[count - 1] - driver[0]);
    return std::abs(travel - kRigExecDrivenActionFullTurn) <=
           kRigExecDrivenActionFullTurnTolerance;
}

/// A full-turn driver's angle, wrapped relative to sample 0 on the action's
/// side: an angle inside [min(samples), max(samples)] is returned
/// unchanged; outside it, side +1 maps it into [s0, s0 + 360) and side -1
/// into (s0 - 360, s0]. A 0 -> -360 dial therefore keeps 0 at rest and
/// maps -10 to -10, never 0 to -360. Side 0 leaves the angle as it is.
inline double RigExecDrivenActionWrapAngle(const double *driver, size_t count,
    int side, double angle)
{
    if (!driver || count == 0 || side == 0 || !std::isfinite(angle)) return angle;
    const auto range = std::minmax_element(driver, driver + count);
    if (angle >= *range.first && angle <= *range.second) return angle;
    const double s0 = driver[0];
    const double turn = kRigExecDrivenActionFullTurn;
    if (side > 0) {
        double d = std::fmod(angle - s0, turn);       // (-360, 360)
        if (d < 0.0) d += turn;
        if (d >= turn) d -= turn;                     // rounding of d + 360
        return s0 + d;
    }
    double d = std::fmod(s0 - angle, turn);
    if (d < 0.0) d += turn;
    if (d >= turn) d -= turn;
    return s0 - d;
}

/// The whole action frame for driver value \p angle (wrapped first when
/// \p fullTurn): 0 when the angle is not strictly on \p side of sample 0,
/// else the frame in [0, count - 1] whose sample is closest to the angle.
/// Ties go to the lowest frame, or to the highest when \p tiesLater (the range
/// rule, whose held tail repeats the last value). \p count >= 2; fewer
/// samples, a side of 0 or a non-finite angle give frame 0.
inline int32_t RigExecDrivenActionFrame(const double *driver, size_t count,
    int side, bool fullTurn, bool tiesLater, double angle)
{
    if (!driver || count < 2 || side == 0 || !std::isfinite(angle)) return 0;
    if (fullTurn) angle = RigExecDrivenActionWrapAngle(driver, count, side, angle);
    const double s0 = driver[0];
    if (!(side > 0 ? angle > s0 : angle < s0)) return 0;
    size_t best = 0;
    double bestDistance = std::abs(driver[0] - angle);
    for (size_t k = 1; k < count; ++k) {
        const double distance = std::abs(driver[k] - angle);
        if (distance < bestDistance || (tiesLater && distance == bestDistance)) {
            best = k;
            bestDistance = distance;
        }
    }
    return int32_t(best);
}

/// outputs:weight for \p frame of a \p count-sample driver, computed in
/// float so blend samples authored at the same k / N match bit for bit.
inline float RigExecDrivenActionWeight(int32_t frame, size_t count)
{
    return count > 1 ? float(frame) / float(count - 1) : 0.0f;
}

/// A channel's published value at \p frame: samples past the end hold the
/// last; offset mode subtracts sample 0. No samples publish 0.
inline double RigExecActionChannelValue(const double *samples, size_t count,
    int32_t frame, bool offset)
{
    if (!samples || count == 0) return 0.0;
    const size_t k = frame <= 0 ? 0 : std::min(size_t(frame), count - 1);
    return offset ? samples[k] - samples[0] : samples[k];
}

/// Every output of \p table for driver value \p angle into \p out
/// (table.OutputCount() values): frame, weight, active (1 or 0), then each
/// channel's value. A disabled action is not evaluated here: the
/// pose-interpolator family publishes zeros for a disabled node.
inline void RigExecDrivenActionEvaluate(const RigExecDrivenActionTable &table,
    double angle, double *out)
{
    const size_t count = table.driver.size();
    const int32_t frame = RigExecDrivenActionFrame(table.driver.data(), count,
        table.side, table.fullTurn, table.tiesLater, angle);
    out[0] = double(frame);
    out[1] = double(RigExecDrivenActionWeight(frame, count));
    out[2] = frame > 0 ? 1.0 : 0.0;
    for (size_t c = 0; c < table.channelSamples.size(); ++c) {
        const std::vector<double> &samples = table.channelSamples[c];
        const bool offset = c < table.channelOffset.size() && table.channelOffset[c];
        out[kRigExecDrivenActionFixedOutputs + c] =
            RigExecActionChannelValue(samples.data(), samples.size(), frame, offset);
    }
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_DRIVEN_ACTION_KERNEL_H
