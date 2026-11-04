// Joint groups (RigExecJointGroupAPI): alternative chains among a solver's
// joints, shared by RigExecFkChain and RigExecSkeleton2d in native, frozen
// and .rigexec evaluation.
//
// USD-free and header-only: the runtime instantiates it. No locks, statics
// or diagnostics. The rule has no public source (docs/references.md, "2D
// skeleton rules without a public source").
#ifndef RIGEXEC_MATH_JOINT_GROUP_KERNEL_H
#define RIGEXEC_MATH_JOINT_GROUP_KERNEL_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rigExec {

/// One group: members [memberBegin, memberBegin + memberCount) of the
/// solver's concatenated members, each the first joint of an alternative
/// chain. Structural; the active member arrives per evaluation.
struct RigExecJointGroup {
    uint32_t memberBegin = 0, memberCount = 0;
    bool enabled = true;
};

/// The active member of \p group for selector \p v: rounded half away from
/// zero, clamped to the members (NaN selects the first); -1 for a disabled
/// or empty group.
inline int32_t RigExecJointGroupSlot(const RigExecJointGroup &group, double v)
{
    if (!group.enabled || group.memberCount == 0) {
        return -1;
    }
    const int64_t count = int64_t(group.memberCount);
    int64_t slot = (v >= -0.5 && v < double(count) - 0.5) ? int64_t(std::llround(v)) : -1;
    if (slot < 0) {
        slot = !(v >= 0.0) ? 0 : count - 1;
    }
    return int32_t(slot);
}

/// Marks active[i] for joints [0, count): a joint at or under an inactive
/// member of an enabled group is inactive (0) unless it is also at or under
/// an active member; every other joint is active (1). parentOf(i) is joint
/// i's parent among them, or -1; the walk stops after count steps, so a
/// cycle in malformed data ends. \p slots receives each group's
/// RigExecJointGroupSlot. A member outside [0, count) matches no joint.
template <class ParentOf>
inline void RigExecResolveJointGroups(size_t count, ParentOf &&parentOf,
    const RigExecJointGroup *groups, size_t groupCount, const int32_t *members,
    const double *selectors, int32_t *slots, uint8_t *active)
{
    for (size_t i = 0; i < count; ++i) {
        active[i] = 1;
    }
    for (size_t g = 0; g < groupCount; ++g) {
        slots[g] = RigExecJointGroupSlot(groups[g], selectors[g]);
    }
    for (size_t i = 0; i < count; ++i) {
        bool underActive = false, underInactive = false;
        int32_t a = int32_t(i);
        for (size_t step = 0; a >= 0 && step < count && !(underActive && underInactive);
             ++step, a = parentOf(size_t(a))) {
            for (size_t g = 0; g < groupCount; ++g) {
                if (slots[g] < 0) {
                    continue;
                }
                const RigExecJointGroup &group = groups[g];
                for (uint32_t m = 0; m < group.memberCount; ++m) {
                    if (members[group.memberBegin + m] == a) {
                        if (int32_t(m) == slots[g]) {
                            underActive = true;
                        } else {
                            underInactive = true;
                        }
                    }
                }
            }
        }
        active[i] = (underActive || !underInactive) ? uint8_t(1) : uint8_t(0);
    }
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_JOINT_GROUP_KERNEL_H
