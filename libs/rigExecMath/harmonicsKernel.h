// Method reference: the harmonic secondary-motion solver of
// tbttfox/BlurHarmonics (https://github.com/tbttfox/BlurHarmonics, master at
// the time of writing; src/lib/harmonicSolver.cpp). The repository publishes
// no license, so this is an independent implementation of its documented
// method, not a copy of its code. See docs/references.md and
// docs/concepts/secondary-motion.md.
//
// Each earlier frame's acceleration (the second difference of a driver's
// position) is an impulse that rings as a damped sine; the offset at the
// current frame is the sum of the rings still sounding. The sum is a finite
// impulse response over a fixed window of earlier positions, so the result
// is a pure function of those positions: no state survives between
// evaluations. Header-only and USD-free, so every evaluator, the zero-USD
// runtime included, sums in the same order.
//
// Conventions:
// - Positions are 3 doubles, oldest first; the last one is the current
//   frame. Samples are one frame apart.
// - The acceleration of sample i is p[i-1] - 2 p[i] + p[i+1]. The current
//   frame has no successor, so the newest acceleration is the previous
//   frame's, k = 1 frames back; the k-th one back rings for k * frequency
//   phase frames.
// - A history shorter than the window starts from rest: the oldest sample's
//   predecessor is taken equal to it, and ignoreFirstFrame drops that
//   oldest acceleration (the jolt of starting to move from rest).
#ifndef RIGEXEC_MATH_HARMONICS_KERNEL_H
#define RIGEXEC_MATH_HARMONICS_KERNEL_H

#include <cmath>
#include <cstddef>

namespace rigExec {

/// The longest window a harmonics step may ask for, in frames.
constexpr size_t RigExecHarmonicsMaxWindowFrames = 4096;

/// Compiled rig settings: they fix the window, so an edit recompiles.
struct RigExecHarmonicsSettings {
    /// How many oscillations one impulse rings for.
    double waves = 5.0;
    /// Frames per oscillation.
    double waveLength = 5.0;
    /// Phase frames one evaluated frame advances; above 1 rings faster and
    /// shortens the window.
    double frequency = 1.0;
    /// Drop the oldest acceleration of a history that starts from rest.
    bool ignoreFirstFrame = true;

    bool operator==(const RigExecHarmonicsSettings &o) const
    {
        return waves == o.waves && waveLength == o.waveLength &&
               frequency == o.frequency &&
               ignoreFirstFrame == o.ignoreFirstFrame;
    }
    bool operator!=(const RigExecHarmonicsSettings &o) const
    {
        return !(*this == o);
    }
};

/// Per-frame values, read at the current frame only.
struct RigExecHarmonicsParameters {
    double amplitude = 1.0;
    /// Per-axis multipliers in the measuring space.
    double axisAmplitude[3] = {1.0, 1.0, 1.0};
    /// How fast the ring dies; 0 holds the full amplitude until it ends.
    double decay = 3.0;
    /// The fraction of the amplitude left when the ring ends.
    double termination = 0.0;
    /// Scales the amplitude by waveLength / (2 pi), so an impulse leaves
    /// with the velocity change it arrived with.
    bool matchVelocity = true;
};

/// Whether \p s describes a usable window: finite, waves and waveLength
/// above zero, frequency above zero, and at most the frame cap.
inline bool
RigExecHarmonicsSettingsValid(const RigExecHarmonicsSettings &s)
{
    if (!std::isfinite(s.waves) || !std::isfinite(s.waveLength) ||
        !std::isfinite(s.frequency)) {
        return false;
    }
    if (!(s.waves > 0.0) || !(s.waveLength > 0.0) || !(s.frequency > 0.0)) {
        return false;
    }
    return s.waves * s.waveLength / s.frequency <=
           double(RigExecHarmonicsMaxWindowFrames - 2);
}

/// How many accelerations ring at the current frame: the largest k with
/// k * frequency below waves * waveLength. Zero for invalid settings.
inline size_t
RigExecHarmonicsRingingCount(const RigExecHarmonicsSettings &s)
{
    if (!RigExecHarmonicsSettingsValid(s)) return 0;
    const double length = s.waves * s.waveLength;
    double k = std::floor(length / s.frequency);
    while (k > 0.0 && k * s.frequency >= length) k -= 1.0;
    return size_t(k);
}

/// How many frames before the current one the solve reads: the ringing
/// accelerations plus the predecessor of the oldest. Zero for invalid
/// settings.
inline size_t
RigExecHarmonicsWindowFrames(const RigExecHarmonicsSettings &s)
{
    const size_t ringing = RigExecHarmonicsRingingCount(s);
    return ringing ? ringing + 1 : 0;
}

/// The ring's envelope at \p phase of a ring \p length phase frames long:
/// 1 at the impulse, falling to \p termination at the end.
inline double
RigExecHarmonicsEnvelope(double phase, double length, double decay,
                         double termination)
{
    const double fall =
        std::exp(-decay * phase / length) - phase * std::exp(-decay) / length;
    return (1.0 - termination) * fall + termination;
}

/// The offset at the current frame, in the space the positions are in.
/// \p positions holds \p count samples, oldest first, the last the current
/// frame. Writes zero and returns false for invalid settings or fewer than
/// two samples. Only the newest RigExecHarmonicsWindowFrames + 1 samples are
/// read; a shorter history starts from rest (see the header).
inline bool
RigExecHarmonicsOffset(const RigExecHarmonicsSettings &settings,
                       const RigExecHarmonicsParameters &parameters,
                       const double (*positions)[3], size_t count,
                       double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    const size_t ringing = RigExecHarmonicsRingingCount(settings);
    if (!ringing || !positions || count < 2) return false;
    const double length = settings.waves * settings.waveLength;
    const double twoPi = 6.283185307179586476925286766559;
    const double toRadians = twoPi / settings.waveLength;
    const double scale = parameters.matchVelocity
                             ? parameters.amplitude * settings.waveLength / twoPi
                             : parameters.amplitude;
    const size_t current = count - 1;
    double sum[3] = {0.0, 0.0, 0.0};
    for (size_t k = 1; k <= ringing && k <= current; ++k) {
        const size_t i = current - k;
        // The oldest sample of a short history has no predecessor: it
        // starts from rest.
        const bool fromRest = i == 0;
        if (fromRest && settings.ignoreFirstFrame) break;
        const double *prev = positions[fromRest ? i : i - 1];
        const double *here = positions[i];
        const double *next = positions[i + 1];
        const double phase = double(k) * settings.frequency;
        const double ring =
            std::sin(phase * toRadians) *
            RigExecHarmonicsEnvelope(phase, length, parameters.decay,
                                     parameters.termination);
        for (int a = 0; a < 3; ++a) {
            const double accel = prev[a] - 2.0 * here[a] + next[a];
            sum[a] -= accel * ring;
        }
    }
    for (int a = 0; a < 3; ++a) {
        out[a] = sum[a] * scale * parameters.axisAmplitude[a];
    }
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_HARMONICS_KERNEL_H
