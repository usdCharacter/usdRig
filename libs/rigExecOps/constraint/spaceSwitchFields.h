#ifndef RIGEXEC_OPS_CONSTRAINT_SPACE_SWITCH_FIELDS_H
#define RIGEXEC_OPS_CONSTRAINT_SPACE_SWITCH_FIELDS_H

/// A space switch's declared payload fields, shared by the ops that run one
/// (the match ops and SwitchedComposeSlot): the field order, the reader of
/// one instance's setup, its setter, and the kernel's record. An op lays
/// these fields first. Private to rigExecOps.

#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecRuntime/spaceSwitchMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rigExecOps {
namespace spaceSwitchFields {

enum SpaceSwitchField : uint32_t {
    kFilterCount,
    kFilters,
    kTwistAxis,
    kAffectTranslation,
    kAffectRotation,
    kAffectScale,
    kHasCarry,
    kWorldCount,
    kWorld,
    kSpaceSwitchFieldCount,
};

/// One instance's SpaceSwitchSetup, its lists read in place.
struct SwitchSettings {
    rigExec::RigExecFieldView<const int32_t> filters;
    rigExecStd::Vec3d twistAxis;
    std::array<bool, 3> affectTranslation, affectRotation, affectScale;
    bool hasCarry = false;
    rigExec::RigExecFieldView<const uint8_t> world;
};

using SwitchRecord = rigExec::RigExecSpaceSwitchRecordT<rigExecStd::Vec3d>;
using SwitchInputs = rigExec::RigExecSpaceSwitchInputsT<rigExecStd::Mat4d>;
using SwitchMath = rigExec::RrSpaceSwitchMath;

inline std::array<bool, 3>
FlagsAt(const rigExec::RigExecOpBatch &batch, uint32_t field, size_t k)
{
    const auto flags = rigExec::RigExecPayloadField<uint8_t>(batch, field, k);
    return {{flags[0] != 0, flags[1] != 0, flags[2] != 0}};
}

inline SwitchSettings
SwitchAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    SwitchSettings s;
    s.filters = fieldIo::ListAt<int32_t>(batch, kFilters, k);
    s.twistAxis = fieldIo::ValueAt<rigExecStd::Vec3d>(batch, kTwistAxis, k);
    s.affectTranslation = FlagsAt(batch, kAffectTranslation, k);
    s.affectRotation = FlagsAt(batch, kAffectRotation, k);
    s.affectScale = FlagsAt(batch, kAffectScale, k);
    s.hasCarry = fieldIo::FlagAt(batch, kHasCarry, k);
    s.world = fieldIo::ListAt<uint8_t>(batch, kWorld, k);
    return s;
}

inline SwitchRecord
RecordOf(const SwitchSettings &setup)
{
    SwitchRecord record;
    record.filters.assign(setup.filters.begin(), setup.filters.end());
    record.twistAxis = setup.twistAxis;
    record.affectTranslation = setup.affectTranslation;
    record.affectRotation = setup.affectRotation;
    record.affectScale = setup.affectScale;
    return record;
}

inline void
SetFlags(rigExec::RigExecFieldValues &values, uint32_t field,
         const std::array<bool, 3> &flags)
{
    const uint8_t bytes[3] = {uint8_t(flags[0]), uint8_t(flags[1]),
                              uint8_t(flags[2])};
    values.Set(field, bytes, 3);
}

inline void
SetSwitch(rigExec::RigExecFieldValues &out, const SpaceSwitchSetup &s)
{
    fieldIo::SetList(out, kFilterCount, s.filters);
    out.Set(kTwistAxis, s.twistAxis);
    SetFlags(out, kAffectTranslation, s.affectTranslation);
    SetFlags(out, kAffectRotation, s.affectRotation);
    SetFlags(out, kAffectScale, s.affectScale);
    fieldIo::SetFlag(out, kHasCarry, s.hasCarry);
    fieldIo::SetList(out, kWorldCount, s.world);
}

/// The switch's inputs from \p port's ports (ParentPosed and ParentDefault
/// identity when unfed); false when the sources and the setup disagree.
template <class Port>
bool
SwitchInputsOf(const rigExec::RigExecOpIo &io, const SwitchSettings &setup,
               const Port &port, SwitchInputs *in)
{
    using rigExecStd::Mat4d;
    in->posedDefault = *opIo::In<Mat4d>(io, port.posedDefault);
    in->parentDefaultInverse = *opIo::In<Mat4d>(io, port.parentDefaultInverse);
    in->parentPosed = opIo::InOr(io, port.parentPosed, Mat4d(1.0));
    in->parentDefault = opIo::InOr(io, port.parentDefault, Mat4d(1.0));
    in->hasCarry = setup.hasCarry;
    if (setup.hasCarry) {
        const Mat4d *spaceDefault = opIo::In<Mat4d>(io, port.spaceDefault);
        const Mat4d *spacePosed = opIo::In<Mat4d>(io, port.spacePosed);
        if (!spaceDefault || !spacePosed)
            return false;
        in->spaceDefault = *spaceDefault;
        in->spacePosed = *spacePosed;
    }
    const auto &defaults =
        *opIo::In<rigExecStd::RigExecArray<Mat4d>>(io, port.sourceDefaults);
    const auto &posed =
        *opIo::In<rigExecStd::RigExecArray<Mat4d>>(io, port.sourcePosed);
    const size_t count = setup.world.Size();
    if (defaults.Size() != count || posed.Size() != count)
        return false;
    in->sources.resize(count);
    for (size_t k = 0; k < count; ++k) {
        in->sources[k].world = setup.world[k] != 0;
        in->sources[k].defaultSpace = defaults[k];
        in->sources[k].posedSpace = posed[k];
    }
    return true;
}

} // namespace spaceSwitchFields
} // namespace rigExecOps

#endif
