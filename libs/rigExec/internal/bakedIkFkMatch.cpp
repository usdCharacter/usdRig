// RigExecIkFkMatchConstraint in the native program: the step that computes a
// limb's match values and presented frames, and the publication that applies
// the presentation. The arithmetic is ikFkMatchKernel.h, shared with the
// binary runtime (rigExecRuntime/poseConstraints.cpp); this file only reads
// the program's tables into it and writes its answers back.

#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecScene/poseProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/types.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((outputsIkValues, SchemaAttrText::outputsIkValues))
    ((outputsFkValues, SchemaAttrText::outputsFkValues))
);

} // namespace

// One control's channels as the compose reads them (_ComposeAvarsOf).
void
RigExecBakedMatchChannelsOf(const RigExecBakedProgramImpl &B, int slot,
                            RigExecIkFkMatchChannels *out)
{
    RigExecIkFkMatchChannels &channels = *out;
    channels = RigExecIkFkMatchChannels();
    const double *a = &B.avars[size_t(slot) * 11];
    const bool noScale = B.noScaleAvars[size_t(slot)] != 0;
    const unsigned sign = size_t(slot) < B.rotationSign.size()
                              ? B.rotationSign[size_t(slot)]
                              : 0u;
    for (int k = 0; k < 3; ++k) {
        channels.t[k] = a[k];
        channels.s[k] = noScale ? 1.0 : a[3 + k];
        channels.r[k] = a[6 + k];
        channels.sign[k] = RigExecRotationSignFromMask(sign, k);
    }
    channels.spin = a[9];
    channels.units = a[10];
    const TfToken &order = B.rotOrder[size_t(slot)];
    if (order.size() == 3) {
        std::copy(order.GetText(), order.GetText() + 3, channels.order);
    }
}

void
RigExecBakedPresentIkFkMatches(const RigExecBakedProgramImpl &B,
                               RigExecRigPose *pose)
{
    const TfToken &ikValues = _tokens->outputsIkValues;
    const TfToken &fkValues = _tokens->outputsFkValues;
    for (const auto &c : B.constraints) {
        if (!c.ikFkMatch || c.sources.size() != 10)
            continue;
        if (c.matchValuesValid) {
            pose->movedProperties[c.path.AppendProperty(ikValues)] = VtValue(
                VtDoubleArray(c.matchIkValues.begin(), c.matchIkValues.end()));
            pose->movedProperties[c.path.AppendProperty(fkValues)] = VtValue(
                VtDoubleArray(c.matchFkValues.begin(), c.matchFkValues.end()));
        }
        if (!c.matchShownMask)
            continue;
        // Each shown control, and every control under it carried by its
        // revision.
        GfMatrix4d delta[5];
        bool deltaOk[5] = {false, false, false, false, false};
        for (size_t k = 0; k < 5; ++k) {
            if (!(c.matchShownMask & (1u << k)))
                continue;
            const auto found =
                pose->controlFrames.find(B.paths[size_t(c.sources[5 + k])]);
            if (found == pose->controlFrames.end())
                continue;
            GfMatrix4d before, after;
            deltaOk[k] = RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                               found->second.points, &before) &&
                         RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                               c.matchShown[k].points, &after);
            if (deltaOk[k])
                delta[k] = before.GetInverse() * after;
            found->second = c.matchShown[k];
        }
        for (const auto &[slot, rides] : c.matchRiders) {
            if (!deltaOk[size_t(rides)])
                continue;
            const auto found = pose->controlFrames.find(B.paths[size_t(slot)]);
            if (found == pose->controlFrames.end())
                continue;
            GfMatrix4d frame;
            if (RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      found->second.points, &frame)) {
                found->second =
                    RigExecFrameFromMatrix(frame * delta[size_t(rides)]);
            }
        }
    }
}

} // namespace rigExec
