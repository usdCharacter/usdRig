// The rig the cloth suites share: a curtain pinned along its top edge
// follows the Root control (a matrix mover) and drapes onto a box that moves
// with it; RigExecClothMover runs last on the curtain. The cloth can be
// disabled, which leaves the points entering it.
#ifndef RIGEXEC_TEST_CLOTH_STAGE_H
#define RIGEXEC_TEST_CLOTH_STAGE_H

#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace rigExecClothStage {

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

inline const SdfPath kRig("/Asset/Rig");
inline const SdfPath kRoot("/Asset/Rig/Controls/Root");
inline const SdfPath kCloth("/Asset/Rig/Movers/CurtainCloth");
inline const SdfPath kCurtain("/Asset/Geom/Curtain.points");
inline const SdfPath kBox("/Asset/Geom/Box.points");

inline constexpr int kColumns = 7, kRows = 7;
inline constexpr float kSpacing = 4.0f;

// Root's avars:tx keys: a swing out from frame 1 to 9, a hold, a swing back.
inline constexpr double kKeys[][2] = {{1, 0}, {9, 24}, {18, 24}, {26, -10}, {40, -10}};

inline std::string
Join(const std::vector<std::string> &items)
{
    std::string out;
    for (size_t k = 0; k < items.size(); ++k) out += (k ? ", " : "") + items[k];
    return out;
}

inline std::string
Float(float v)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%.9g", double(v));
    return text;
}

// The curtain: kColumns x kRows points, row 0 at the top (y = 40), in the
// z = 0 plane, the top row pinned.
inline std::string
CurtainText()
{
    std::vector<std::string> points, counts, indices, pins;
    for (int j = 0; j < kRows; ++j) {
        for (int i = 0; i < kColumns; ++i) {
            const float x = -12.0f + kSpacing * float(i);
            const float y = 40.0f - kSpacing * float(j);
            points.push_back("(" + Float(x) + ", " + Float(y) + ", 0)");
            pins.push_back(j == 0 ? "1" : "0");
        }
    }
    for (int j = 0; j + 1 < kRows; ++j) {
        for (int i = 0; i + 1 < kColumns; ++i) {
            const int a = j * kColumns + i;
            counts.push_back("4");
            for (const int k : {a, a + 1, a + 1 + kColumns, a + kColumns})
                indices.push_back(std::to_string(k));
        }
    }
    return "            int[] faceVertexCounts = [" + Join(counts) + "]\n" +
           "            int[] faceVertexIndices = [" + Join(indices) + "]\n" +
           "            point3f[] points = [" + Join(points) + "]\n" +
           "            uniform token subdivisionScheme = \"none\"\n|" +
           Join(pins);
}

inline std::string
StageText(bool clothEnabled, const std::string &clothBody)
{
    std::string samples;
    for (const auto &key : kKeys)
        samples += std::to_string(int(key[0])) + ": " + std::to_string(key[1]) + ", ";
    const std::string curtain = CurtainText();
    const size_t bar = curtain.find('|');
    const std::string mesh = curtain.substr(0, bar);
    const std::string pins = curtain.substr(bar + 1);
    return R"(#usda 1.0
(
    endTimeCode = 60
    metersPerUnit = 0.01
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "Asset"
        def Scope "Controls"
        {
            def RigExecControl "Root" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:tx = 0
                double avars:tx.timeSamples = { )" + samples + R"( }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
        def Scope "Weights"
        {
            def RigExecStaticWeight "CurtainW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </Asset/Geom/Curtain.points>
            }
            def RigExecStaticWeight "BoxW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </Asset/Geom/Box.points>
            }
        }
        # Bottom sibling first: the follow movers run, then the cloth.
        def Scope "Movers"
        {
            def RigExecClothMover "CurtainCloth" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Curtain.points>
                bool inputs:enabled = )" + std::string(clothEnabled ? "true" : "false") + R"(
                rel rigExec:colliders = </Asset/Geom/Box> (
                    rigExecReadPhase = "final"
                )
                uniform float[] rigExec:pinWeights = [)" + pins + R"(]
)" + clothBody + R"(
            }
            def RigExecMatrixMover "CurtainFollow" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Curtain.points>
                rel rigExec:transform = </Asset/Rig/Controls/Root> (
                    rigExecReadPhase = "final"
                )
                rel rigExec:weightObject = </Asset/Rig/Weights/CurtainW>
            }
            def RigExecMatrixMover "BoxFollow" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Box.points>
                rel rigExec:transform = </Asset/Rig/Controls/Root> (
                    rigExecReadPhase = "final"
                )
                rel rigExec:weightObject = </Asset/Rig/Weights/BoxW>
            }
        }
    }
    def Scope "Geom"
    {
        def Mesh "Curtain"
        {
)" + mesh + R"(        }
        def Mesh "Box"
        {
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 3, 2, 6, 7, 0, 4, 5, 1, 0, 3, 7, 4, 1, 5, 6, 2]
            point3f[] points = [(-8, 0, -8), (8, 0, -8), (8, 0, 8), (-8, 0, 8), (-8, 15.5, -8), (8, 15.5, -8), (8, 15.5, 8), (-8, 15.5, 8)]
            uniform token subdivisionScheme = "none"
        }
    }
}
)";
}

inline UsdStageRefPtr
MakeStage(bool clothEnabled = true, const std::string &clothBody = std::string())
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    if (!stage->GetRootLayer()->ImportFromString(StageText(clothEnabled, clothBody))) {
        std::printf("the cloth stage did not parse\n");
        return nullptr;
    }
    return stage;
}

using Points = std::vector<GfVec3f>;

inline Points
PointsOf(const RigExecRigPose &pose, const SdfPath &path)
{
    const auto found = pose.movedProperties.find(path);
    if (!pose.valid || found == pose.movedProperties.end() ||
        !found->second.IsHolding<VtVec3fArray>()) {
        return {};
    }
    const VtVec3fArray &value = found->second.UncheckedGet<VtVec3fArray>();
    return Points(value.begin(), value.end());
}

inline bool
SameBits(const Points &a, const Points &b)
{
    return a.size() == b.size() &&
           (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(GfVec3f)) == 0);
}

inline float
Furthest(const Points &a, const Points &b)
{
    float most = a.size() == b.size() ? 0.0f : INFINITY;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i)
        most = std::max(most, (a[i] - b[i]).GetLength());
    return most;
}

}  // namespace rigExecClothStage

#endif  // RIGEXEC_TEST_CLOTH_STAGE_H
