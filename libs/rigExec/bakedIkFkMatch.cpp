// RigExecIkFkMatchConstraint in the native program: the step that computes a
// limb's match values and presented frames, and the publication that applies
// the presentation. The arithmetic is ikFkMatchKernel.h, shared with the
// binary runtime (rigExecRuntime/poseConstraints.cpp); this file only reads
// the program's tables into it and writes its answers back.

#include "bakedProgramImpl.h"
#include "frameExtraction.h"
#include "rigEvaluator.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecScene/poseProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/vt/types.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

bool
_MatrixOf(const RigExecPointFrame &frame, GfMatrix4d *out)
{
    return RigExecBakedUsable(frame) &&
           RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points, out);
}

RigExecIkFkMatchChannels
_ChannelsOf(const RigExecBakedProgramImpl &B, int slot)
{
    RigExecIkFkMatchChannels channels;
    RigExecBakedMatchChannelsOf(B, slot, &channels);
    return channels;
}

}  // namespace

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
RigExecBakedRunIkFkMatch(RigExecBakedProgramImpl &B, RigExecBakedStep *step,
                         RigExecBakedProgramImpl::Constraint &c,
                         const RigExecBakedCommit &commit, double weight,
                         const double scalars[7])
{
    c.matchValuesValid = false;
    c.matchShownMask = 0;
    // Sources: joints 0-2, IK root 3, effector 4, IK control 5, pole 6, FK
    // controls 7-9. Rests: joints 0-2, then the controls' at 3-7.
    GfMatrix4d input[10], rest[8];
    bool usable = c.sources.size() == 10 && commit.sourceReads.size() >= 10;
    for (size_t i = 0; usable && i < 10; ++i) {
        usable = c.sources[i] >= 0 &&
                 _MatrixOf(B.fin[size_t(commit.sourceReads[i])], &input[i]);
    }
    for (size_t i = 0; usable && i < 3; ++i) {
        usable = _MatrixOf(B.restFrames[size_t(c.sources[i])], &rest[i]);
    }
    for (size_t i = 0; usable && i < 5; ++i) {
        usable = _MatrixOf(B.restFrames[size_t(c.sources[5 + i])], &rest[3 + i]);
    }
    if (!usable) {
        step->diagnostics.push_back(
            c.pathText + " could not read its limb; constraint passed through");
        return;
    }
    const GfMatrix4d *control = input + 5;
    RigExecIkFkMatchChannels channels[5];
    for (size_t i = 0; i < 5; ++i) {
        channels[i] = _ChannelsOf(B, c.sources[5 + i]);
    }
    RigExecLimbStretch limb;
    limb.stretch = scalars[1];
    limb.softDistance = scalars[2];
    limb.upperScale = scalars[3];
    limb.lowerScale = scalars[4];
    limb.pin = scalars[5];
    limb.scaleCalibration = c.matchScaleCalibration;

    // The IK solve's space carries its rest lengths and soft distance.
    double space[16];
    const double *spaceMatrix = nullptr;
    if (c.spaceSlot >= 0) {
        GfMatrix4d spaceRest(1.0);
        const GfMatrix4d spacePosed = B.posedM[size_t(c.spaceSlot)];
        if (_MatrixOf(B.restFrames[size_t(c.spaceSlot)], &spaceRest) &&
            std::isfinite(spacePosed[3][0])) {
            const GfMatrix4d carry = spaceRest.GetInverse() * spacePosed;
            std::copy(carry.data(), carry.data() + 16, space);
            spaceMatrix = space;
        }
    }
    double restOrigins[3][3], restUpper = 0, restLower = 0, spaceFactor = 1;
    for (size_t i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) restOrigins[i][k] = rest[i][3][k];
    }
    RigExecIkFkMatchRestLengths(restOrigins, spaceMatrix, c.matchUpperOffset,
                                c.matchLowerOffset, &restUpper, &restLower,
                                &spaceFactor);
    limb.softDistance *= spaceFactor;

    RigExecIkFkMatchIkInputs ikIn;
    for (size_t i = 0; i < 3; ++i) ikIn.joints[i] = input[i].data();
    ikIn.root = input[3].data();
    ikIn.effectorPosed = input[4].data();
    ikIn.ikPosed = control[0].data();
    ikIn.ik = channels[0];
    ikIn.twistRadians = scalars[6] * std::acos(-1.0) / 180.0;
    ikIn.poleDistance =
        (rest[4].ExtractTranslation() - rest[1].ExtractTranslation())
            .GetLength() * spaceFactor;
    ikIn.polePosed = control[1].data();
    ikIn.hasStretch = c.matchStretchLimb;
    ikIn.limb = limb;
    ikIn.restUpper = restUpper;
    ikIn.restLower = restLower;
    RigExecIkFkMatchIkResult ik;
    RigExecIkFkMatchIk(ikIn, &ik);

    // The pole's parent once the IK control has moved: its own space switch
    // re-run with every source that rides the IK control carried by the
    // move, or its namespace parent carried the same way.
    RigExecIkFkMatchChannels pole = channels[1];
    double poleFrame[16];
    bool poleOk = false;
    if (ik.ok) {
        GfMatrix4d ikNew;
        std::copy(ik.ikFrame, ik.ikFrame + 16, ikNew.data());
        const GfMatrix4d moved = control[0].GetInverse() * ikNew;
        const auto composed = ikFkMatchKernel::Compose(channels[1]);
        GfMatrix4d avars;
        std::copy(composed.begin(), composed.end(), avars.data());
        GfMatrix4d parent = avars.GetInverse() * control[1];
        const int poleSlot = c.sources[6];
        if (c.matchPoleSwitch >= 0 && !B.posedAuthored[size_t(poleSlot)]) {
            const auto &sw = B.spaceSwitches[size_t(c.matchPoleSwitch)];
            auto &inputs = c.matchPoleInputs;
            inputs = sw.kernelInputs;
            inputs.avars = GfMatrix4d(1.0);
            for (size_t k = 0; k < inputs.sources.size() &&
                               k < c.matchPoleSourceRides.size(); ++k) {
                if (c.matchPoleSourceRides[k] && !inputs.sources[k].world) {
                    inputs.sources[k].posedSpace =
                        inputs.sources[k].posedSpace * moved;
                }
            }
            RigExecPointFrame frame;
            GfMatrix4d switched(1.0);
            if (RigExecRunSpaceSwitch(sw.kernelRecord, inputs, &frame) &&
                _MatrixOf(frame, &switched)) {
                parent = switched;
            }
        } else if (c.matchPoleParentRides) {
            parent = parent * moved;
        }
        poleOk = RigExecIkFkMatchPole(ik.poleTarget, parent.data(), &pole,
                                      poleFrame);
    }

    RigExecIkFkMatchFkInputs fkIn;
    GfMatrix4d offsets[3];
    for (size_t i = 0; i < 3; ++i) {
        fkIn.joints[i] = input[i].data();
        fkIn.fkPosed[i] = control[2 + i].data();
        fkIn.fk[i] = channels[2 + i];
        offsets[i] = rest[5 + i] * rest[i].GetInverse();
        fkIn.offsets[i] = offsets[i].data();
    }
    fkIn.hasStretch = c.matchStretchLimb;
    fkIn.upperScale = limb.upperScale;
    fkIn.lowerScale = limb.lowerScale;
    fkIn.restUpper = restUpper;
    fkIn.restLower = restLower;
    RigExecIkFkMatchFkResult fk;
    RigExecIkFkMatchFk(fkIn, &fk);

    if (ik.ok && poleOk && fk.ok) {
        RigExecIkFkMatchPack(ik, pole, fk, c.matchIkValues.data(),
                             c.matchFkValues.data());
        c.matchValuesValid = true;
    } else {
        step->diagnostics.push_back(
            c.pathText + " could not match its limb's other half");
    }

    const auto show = [&](size_t k, const double *frame) {
        GfMatrix4d matrix;
        std::copy(frame, frame + 16, matrix.data());
        c.matchShown[k] = RigExecFrameFromMatrix(matrix);
        c.matchShownMask |= 1u << k;
    };
    switch (RigExecIkFkMatchShownSide(scalars[0], c.matchIkValue, weight)) {
    case RigExecIkFkMatchShown::IkSide:
        if (ik.ok && poleOk) {
            show(0, ik.ikFrame);
            show(1, poleFrame);
        }
        break;
    case RigExecIkFkMatchShown::FkSide:
        if (fk.ok) {
            for (size_t i = 0; i < 3; ++i) show(2 + i, fk.frames[i]);
        }
        break;
    case RigExecIkFkMatchShown::None:
        break;
    }
}

void
RigExecBakedPresentIkFkMatches(const RigExecBakedProgramImpl &B,
                               RigExecRigPose *pose)
{
    static const TfToken ikValues("outputs:ikValues");
    static const TfToken fkValues("outputs:fkValues");
    for (const auto &c : B.constraints) {
        if (!c.ikFkMatch || c.sources.size() != 10) continue;
        if (c.matchValuesValid) {
            pose->movedProperties[c.path.AppendProperty(ikValues)] = VtValue(
                VtDoubleArray(c.matchIkValues.begin(), c.matchIkValues.end()));
            pose->movedProperties[c.path.AppendProperty(fkValues)] = VtValue(
                VtDoubleArray(c.matchFkValues.begin(), c.matchFkValues.end()));
        }
        if (!c.matchShownMask) continue;
        // Each shown control, and every control under it carried by its
        // revision.
        GfMatrix4d delta[5];
        bool deltaOk[5] = {false, false, false, false, false};
        for (size_t k = 0; k < 5; ++k) {
            if (!(c.matchShownMask & (1u << k))) continue;
            const auto found =
                pose->controlFrames.find(B.paths[size_t(c.sources[5 + k])]);
            if (found == pose->controlFrames.end()) continue;
            GfMatrix4d before, after;
            deltaOk[k] =
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      found->second.points, &before) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      c.matchShown[k].points, &after);
            if (deltaOk[k]) delta[k] = before.GetInverse() * after;
            found->second = c.matchShown[k];
        }
        for (const auto &[slot, rides] : c.matchRiders) {
            if (!deltaOk[size_t(rides)]) continue;
            const auto found = pose->controlFrames.find(B.paths[size_t(slot)]);
            if (found == pose->controlFrames.end()) continue;
            GfMatrix4d frame;
            if (RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      found->second.points, &frame)) {
                found->second =
                    RigExecFrameFromMatrix(frame * delta[size_t(rides)]);
            }
        }
    }
}

}  // namespace rigExec
