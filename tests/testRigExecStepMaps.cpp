// Smart-bone step tables in a rig, through every evaluator. An action is one
// RigExecAnimatedMaps on a dial angle whose RigExecAnimatedMap children share
// slope-0 rows (one cell per sample value, outer cells to +-FLT_MAX, a gate
// row for the side not served): `weight` (k / N), `active` (0 at frame 0,
// else 1), `frame` (k) and one unclamped map per channel. Checked: whole
// frames against literal frame tables (ties by row order, the side gate, the
// hold past the ends, non-finite angles), a full turn's angle folded by
// float math movers with its seam on a second, raw input point row; additive
// offsets from several tables on one channel, an offset on top of mainline
// keys, correctives before deformation (the namespace order gate, three
// ways), stepped switch channels published as float primvars through Hydra
// and .rigexec playback (two blends on one switch: the top sibling wins),
// chained dials settling in one evaluation, a two-way cycle refused, edit
// tiers, and no stage authoring. Live evaluation is checked against the
// tables' expectations, the frame-cache worker and the .rigexec runtime
// against live, bit for bit.
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/playback.h"
#include "rigExecImaging/sceneIndices.h"
#include "rigExecImaging/snapshotStore.h"
#include "rigExecRuntime/runtime.h"

#include "pxr/base/plug/registry.h"
#include "pxr/imaging/hd/flattenedDataSourceProviders.h"
#include "pxr/imaging/hd/flatteningSceneIndex.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usdImaging/usdImaging/stageSceneIndex.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

namespace {

const SdfPath kRig("/Char/Rig");
const SdfPath kHead("/Char/Art/Head");
const SdfPath kStroke("/Char/Art/Stroke");
const SdfPath kKnob("/Char/Art/Knob");
constexpr int kFrames = 18;
const double kMax = double(std::numeric_limits<float>::max());

std::string
Control(const char *name)
{
    return "/Char/Rig/Controls/" + std::string(name);
}

SdfPath
Avar(const char *control, const char *avar)
{
    return SdfPath(Control(control) + ".avars:" + avar);
}

std::string
ActionPath(const char *action)
{
    return "/Char/Rig/Actions/" + std::string(action);
}

// An action map's published value.
SdfPath
Output(const char *action, const char *map)
{
    return SdfPath(ActionPath(action) + "/" + map + ".outputs:value");
}

std::string
Number(double v)
{
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.17g", v);
    return buffer;
}

template <class Values>
std::string
Array(const Values &values)
{
    std::string text = "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) text += ", ";
        text += Number(double(values[i]));
    }
    return text + "]";
}

// samples[k] = f(k) for k = 0..n.
std::vector<double>
Samples(int n, const std::function<double(int)> &f)
{
    std::vector<double> samples;
    for (int k = 0; k <= n; ++k) samples.push_back(f(k));
    return samples;
}

std::vector<double>
Ramp(double end, int n)
{
    return Samples(n, [=](int k) { return end * double(k) / double(n); });
}

// ---- Step tables ----

/// One row of a step table: slope 0 over [from, to] on `input`, adding the
/// value at `frame` (less the value at `minus` for a seam row).
struct Row {
    int input = 0;
    double from = 0.0, to = 0.0;
    int frame = 0;
    int minus = -1;
};

// The test's samples have float midpoints, so every bound is exact.
double
Exact(double v)
{
    if (double(float(v)) != v) {
        ++failures;
        std::printf("FAIL bound %.17g is no float\n", v);
    }
    return v;
}

bool
FullTurn(const std::vector<double> &s)
{
    return s.size() >= 2 && std::abs(std::abs(s.back() - s.front()) - 360.0) <= 1.0;
}

// The nearest-sample cells of the unique sample values, outer cells to
// +-FLT_MAX (the end frames hold). A cell's frame is the lowest k whose
// sample is its value (the highest with tiesLater). Cells sharing a bound
// both hold it, and only the first matching row of a same-input run counts,
// so the cells are listed by frame ascending (descending with tiesLater):
// the tie's winner owns the bound.
std::vector<Row>
Cells(const std::vector<double> &s, bool tiesLater)
{
    std::map<double, int> frame;
    for (size_t k = 0; k < s.size(); ++k) {
        if (tiesLater || !frame.count(s[k])) frame[s[k]] = int(k);
    }
    const std::vector<std::pair<double, int>> values(frame.begin(), frame.end());
    std::vector<Row> cells;
    for (size_t j = 0; j < values.size(); ++j) {
        Row row;
        row.from = j == 0 ? -kMax : Exact(0.5 * (values[j - 1].first + values[j].first));
        row.to = j + 1 == values.size() ? kMax
                                        : Exact(0.5 * (values[j].first + values[j + 1].first));
        row.frame = values[j].second;
        cells.push_back(row);
    }
    std::stable_sort(cells.begin(), cells.end(), [&](const Row &a, const Row &b) {
        return tiesLater ? a.frame > b.frame : a.frame < b.frame;
    });
    return cells;
}

// The rows of an action with driver samples s serving `side` (+1, -1): the
// gate first (s_0 and the other side read frame 0), then the cells. A full
// turn's input 0 is the folded angle (WrapMovers) and input 1 the raw dial,
// whose one point row at the seam s_0 +- 360 adds the last frame's value
// less frame 0's: the folded seam reads frame 0, the rule gives frame N.
std::vector<Row>
StepRows(const std::vector<double> &s, int side, bool tiesLater = false)
{
    std::vector<Row> rows;
    Row gate;
    gate.from = side > 0 ? -kMax : Exact(s[0]);
    gate.to = side > 0 ? Exact(s[0]) : kMax;
    rows.push_back(gate);
    for (const Row &cell : Cells(s, tiesLater)) rows.push_back(cell);
    if (FullTurn(s)) {
        Row seam;
        seam.input = 1;
        seam.from = seam.to = Exact(s[0] + 360.0 * double(side));
        seam.frame = int(s.size()) - 1;
        seam.minus = 0;
        rows.push_back(seam);
    }
    return rows;
}

struct ChannelSpec {
    std::string name;
    std::vector<double> samples;
    /// Value mode (the samples themselves) rather than offsets from sample 0.
    bool value = false;
};

struct MapArrays {
    VtIntArray inputs;
    VtFloatArray from, to, slope, cut;
};

MapArrays
Arrays(const std::vector<Row> &rows, const std::function<double(int)> &cutOf)
{
    MapArrays a;
    for (const Row &r : rows) {
        a.inputs.push_back(r.input);
        a.from.push_back(float(r.from));
        a.to.push_back(float(r.to));
        a.slope.push_back(0.0f);
        a.cut.push_back(float(r.minus < 0 ? cutOf(r.frame)
                                          : cutOf(r.frame) - cutOf(r.minus)));
    }
    return a;
}

struct MapSpec {
    std::string name;
    MapArrays arrays;
    bool clamp = true;
};

// weight k / N (float division, the corrective activations' values),
// active, frame k, then one unclamped map per channel: offset c[k] - c[0]
// or value c[k], the last sample held.
std::vector<MapSpec>
ActionMaps(const std::vector<double> &samples, int side, bool tiesLater,
           const std::vector<ChannelSpec> &channels)
{
    const std::vector<Row> rows = StepRows(samples, side, tiesLater);
    const int n = int(samples.size()) - 1;
    std::vector<MapSpec> maps;
    maps.push_back({"weight", Arrays(rows, [n](int k) {
                        return double(float(k) / float(n)); }), true});
    maps.push_back({"active", Arrays(rows, [](int k) { return k > 0 ? 1.0 : 0.0; }), true});
    maps.push_back({"frame", Arrays(rows, [](int k) { return double(k); }), false});
    for (const ChannelSpec &c : channels) {
        const std::vector<double> v = c.samples;
        const size_t last = v.size() - 1;
        const bool value = c.value;
        maps.push_back({c.name, Arrays(rows, [v, last, value](int k) {
                            const double at = v[std::min(size_t(k), last)];
                            return value ? at : at - v[0];
                        }), false});
    }
    return maps;
}

std::string
MapText(const MapSpec &map)
{
    const std::string i = "                ";
    return i + "def RigExecAnimatedMap \"" + map.name + "\"\n" + i + "{\n" +
           i + "    uniform int[] rigExec:rowInputs = " + Array(map.arrays.inputs) + "\n" +
           i + "    uniform float[] rigExec:rowFrom = " + Array(map.arrays.from) + "\n" +
           i + "    uniform float[] rigExec:rowTo = " + Array(map.arrays.to) + "\n" +
           i + "    uniform float[] rigExec:rowSlope = " + Array(map.arrays.slope) + "\n" +
           i + "    uniform float[] rigExec:rowCut = " + Array(map.arrays.cut) + "\n" +
           (map.clamp ? std::string() : i + "    uniform bool rigExec:clampOutput = false\n") +
           i + "}\n";
}

// The action's table node, no meshes; drivers that math movers revise are
// read at `final`.
std::string
Action(const char *name, const std::vector<std::string> &drivers,
       const std::vector<MapSpec> &maps, const std::string &extra = std::string(),
       bool revised = false)
{
    std::string targets;
    for (const std::string &d : drivers) {
        if (!targets.empty()) targets += ", ";
        targets += "<" + d + ">";
    }
    std::string text = std::string("            def RigExecAnimatedMaps \"") + name +
                       "\"\n            {\n"
                       "                rel rigExec:driverAttributes = [" + targets + "]" +
                       (revised ? " (rigExecReadPhase = \"final\")" : "") + "\n" + extra;
    for (const MapSpec &map : maps) text += MapText(map);
    return text + "            }\n";
}

// Re-records an action: its maps' rows and cuts, in one change.
void
AuthorMaps(const UsdStageRefPtr &stage, const char *action,
           const std::vector<MapSpec> &maps)
{
    SdfChangeBlock block;
    for (const MapSpec &map : maps) {
        const UsdPrim prim = stage->GetPrimAtPath(SdfPath(ActionPath(action) + "/" + map.name));
        CHECK(prim);
        if (!prim) continue;
        prim.GetAttribute(TfToken("rigExec:rowInputs")).Set(map.arrays.inputs);
        prim.GetAttribute(TfToken("rigExec:rowFrom")).Set(map.arrays.from);
        prim.GetAttribute(TfToken("rigExec:rowTo")).Set(map.arrays.to);
        prim.GetAttribute(TfToken("rigExec:rowSlope")).Set(map.arrays.slope);
        prim.GetAttribute(TfToken("rigExec:rowCut")).Set(map.arrays.cut);
    }
}

std::string
MathMover(const char *name, const std::string &target, const char *operation,
          const std::string &value, const std::string &weight = std::string(),
          const std::string &minimum = std::string(), const char *indent = "            ")
{
    const std::string i = indent;
    std::string text = i + "def RigExecFloatMathMover \"" + name +
        "\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n" + i + "{\n" +
        i + "    rel rigExec:moves = <" + target + ">\n" +
        i + "    uniform token rigExec:operation = \"" + operation + "\"\n";
    // A map output is read at `final` (its published value); an undeclared
    // connection would read the output's authored base.
    const std::string final = " (\n" + i + "        rigExecReadPhase = \"final\"\n" + i +
                              "    )\n";
    if (value[0] == '/') {
        text += i + "    float inputs:value" + final;
        text += i + "    float inputs:value.connect = <" + value + ">\n";
    } else if (value[0] == '{') {
        text += i + "    float inputs:value.timeSamples = " + value + "\n";
    } else {
        text += i + "    float inputs:value = " + value + "\n";
    }
    if (!weight.empty()) {
        text += i + "    float inputs:defaultWeight" + final;
        text += i + "    float inputs:defaultWeight.connect = <" + weight + ">\n";
    }
    if (!minimum.empty()) {
        text += i + "    float inputs:min = " + minimum + "\n";
    }
    return text + i + "}\n";
}

// A full turn's fold: float math movers on the action's custom floats
// turn:shift and turn:angle, so turn:angle holds the dial folded into the
// side's window, s_0 + posmod(x - s_0, 360) for side +1 and
// s_0 - posmod(s_0 - x, 360) for side -1 (s_0 = 0 here): shift = the dial,
// quantized down to a turn and negated (side -1: negated, then quantized),
// and angle = the dial plus the shift. Same-target movers run last sibling
// first, so the reorder lists them in reverse execution order.
std::string
WrapMovers(const char *action, const std::string &dial, int side)
{
    const std::string shift = ActionPath(action) + ".turn:shift";
    const std::string angle = ActionPath(action) + ".turn:angle";
    const char *in = "                ";
    std::vector<std::pair<const char *, std::string>> ops;
    ops.push_back({"shiftCopy", MathMover("shiftCopy", shift, "blend", dial, "", "", in)});
    if (side > 0) {
        ops.push_back({"shiftFloor", MathMover("shiftFloor", shift, "quantize", "360", "", "0", in)});
        ops.push_back({"shiftNegate", MathMover("shiftNegate", shift, "multiply", "-1", "", "", in)});
    } else {
        ops.push_back({"shiftNegate", MathMover("shiftNegate", shift, "multiply", "-1", "", "", in)});
        ops.push_back({"shiftFloor", MathMover("shiftFloor", shift, "quantize", "360", "", "0", in)});
    }
    ops.push_back({"angleCopy", MathMover("angleCopy", angle, "blend", dial, "", "", in)});
    ops.push_back({"angleShift", MathMover("angleShift", angle, "add", shift, "", "", in)});
    std::string order, movers;
    for (auto it = ops.rbegin(); it != ops.rend(); ++it) {
        if (!order.empty()) order += ", ";
        order += "\"" + std::string(it->first) + "\"";
        movers += it->second;
    }
    return std::string("            def Scope \"") + action + "_wrap\"\n            {\n"
           "                reorder nameChildren = [" + order + "]\n" + movers +
           "            }\n";
}

// The dials' mainline keys, one value per test frame 1..18 (every frame
// keyed, so a whole frame reads its key exactly).
std::map<std::string, std::vector<double>>
Mainline()
{
    std::map<std::string, std::vector<double>> keys;
    const auto key = [&](const char *name, std::map<int, double> values) {
        std::vector<double> &row = keys[name];
        row.assign(kFrames, 0.0);
        for (const auto &[frame, value] : values) row[size_t(frame - 1)] = value;
    };
    // Whole steps and the hold (1-6), the negative side (7-8); additive
    // offsets (9), a rotated frame (10), the order gate (11); stepped values
    // (12-15 and 17 with D11), the mainline switch (16), a chained dial (18).
    key("D1", {{2, 10.0}, {3, 25.3}, {4, 25.6}, {5, 99.4}, {6, 130.0},
               {7, -20.0}, {8, -80.0}, {9, 50.0}, {10, 50.0}, {11, 100.0}});
    key("D2", {{9, 50.0}});
    key("D7", {{10, 100.0}});
    key("D8", {{12, 20.0}, {13, 40.0}, {14, 70.0}, {15, 90.0}, {17, 70.0}});
    key("D11", {{17, 100.0}});
    key("D9", {{18, 100.0}});
    return keys;
}

std::string
TimeSamples(const std::vector<double> &row)
{
    std::string text = "{";
    for (size_t f = 0; f < row.size(); ++f) {
        if (f) text += ", ";
        text += std::to_string(f + 1) + ": " + Number(row[f]);
    }
    return text + "}";
}

enum class Correctives { Nested, SiblingBelow, SiblingAbove };

struct Options {
    Correctives correctives = Correctives::Nested;
    /// The top of the two switch blends (applied last).
    bool d8OnTop = true;
    /// Adds a k = 0 corrective sample (activation 0), which is refused.
    bool zeroSample = false;
    /// D10's table also drives D9: a two-way cycle.
    bool cycle = false;
    /// D10's dial is revised by a keyed add mover (+45 at frame 18) instead
    /// of D9's table: the table reads its dial's chain result.
    bool nudgeD10 = false;
};

std::string
ControlPrim(const char *name, const std::string &body)
{
    return std::string("            def RigExecControl \"") + name +
           "\" (prepend apiSchemas = [\"RigExecControlAPI\"])\n            {\n"
           "                matrix4d rest:space = ((1, 0, 0, 0), (0, 1, 0, 0), "
           "(0, 0, 1, 0), (0, 0, 0, 1))\n" + body + "            }\n";
}

std::string
CorrectiveMover(const std::string &scope, const char *indent, bool zeroSample)
{
    const std::string i = indent;
    const std::string input = scope + "/Correctives/D1";
    std::string samples = "[";
    if (zeroSample) samples += "<" + input + "/k0>, ";
    samples += "<" + input + "/k50>, <" + input + "/k100>]";
    // A blend input reads the computed weight without a declaration.
    std::string text =
        i + "def RigExecBlendShapeMover \"Correctives\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n" +
        i + "{\n" +
        i + "    rel rigExec:moves = </Char/Art/Stroke.points>\n" +
        i + "    rel rigExec:blendInputs = [<" + input + ">]\n" +
        i + "    def RigExecBlendInput \"D1\"\n" + i + "    {\n" +
        i + "        float inputs:weight.connect = <" + Output("D1", "weight").GetString() + ">\n" +
        i + "        rel rigExec:samples = " + samples + "\n";
    const auto sample = [&](const char *name, int k, const char *shape) {
        text += i + "        def RigExecBlendSample \"" + name + "\"\n" + i + "        {\n" +
                i + "            float rigExec:activation = " +
                Number(double(float(k) / float(100))) + "\n" +
                i + "            rel rigExec:blendShape = </Char/Art/Shapes/" + shape + ">\n" +
                i + "        }\n";
    };
    if (zeroSample) sample("k0", 0, "Q1_k0");
    sample("k50", 50, "Q1_k50");
    sample("k100", 100, "Q1_k100");
    return text + i + "    }\n" + i + "}\n";
}

// The dials' channels, by action.
std::vector<ChannelSpec>
D1Channels()
{
    return {{"Q_tx", Samples(100, [](int k) { return double(k) / 100.0; })}};
}

std::vector<ChannelSpec>
D1_2Channels()
{
    return {{"Q_ty", Samples(50, [](int k) { return -0.5 * double(k) / 50.0; })}};
}

std::vector<ChannelSpec>
D8Channels()
{
    return {{"Switch", Samples(90, [](int k) { return k < 30 ? 0.0 : k < 60 ? 1.0 : 2.0; }),
             true},
            // The channel's own values (1 at rest), published as an offset.
            {"Opacity", Samples(90, [](int k) { return 1.0 - 0.5 * double(k) / 90.0; })}};
}

std::string
Driver(const char *dial)
{
    return Control(dial) + ".avars:rz";
}

// Dials D1, D1 2 (its negative side), D2, D7, D8, D11, D9 and D10, each
// with its step table and channels.
UsdStageRefPtr
MakeStage(const Options &options = Options())
{
    const auto keys = Mainline();
    std::string controls;
    for (const char *dial : {"D1", "D2", "D7", "D8", "D11", "D9"}) {
        controls += ControlPrim(dial, "                double avars:rz.timeSamples = " +
                                          TimeSamples(keys.at(dial)) + "\n");
    }
    controls += ControlPrim("D10", "                double avars:rz = 0\n");
    std::vector<double> qy(kFrames, 0.0), crz(kFrames, 0.0), brz(kFrames, 0.0);
    qy[9] = 0.2;
    crz[9] = 30.0;
    brz[10] = 90.0;
    controls += ControlPrim("Q", "                double avars:tx = 0\n"
                                 "                double avars:ty.timeSamples = " +
                                     TimeSamples(qy) + "\n");
    controls += ControlPrim("Q2", "                double avars:ty = 0\n");
    controls += ControlPrim("Q6", "                double avars:tx = 0\n");
    controls += ControlPrim("C", "                double avars:rz.timeSamples = " +
                                     TimeSamples(crz) + "\n");

    std::string actions;
    actions += Action("D1", {Driver("D1")}, ActionMaps(Ramp(100.0, 100), +1, false, D1Channels()));
    actions += Action("D1_2", {Driver("D1")},
                      ActionMaps(Ramp(-50.0, 50), -1, false, D1_2Channels()));
    actions += Action("D2", {Driver("D2")}, ActionMaps(Ramp(100.0, 100), +1, false,
        {{"Q_tx", Samples(100, [](int k) { return 0.5 * double(k) / 100.0; })},
         {"Q2_ty", Samples(100, [](int k) { return 0.3 * double(k) / 100.0; })}}));
    actions += Action("D7", {Driver("D7")}, ActionMaps(Ramp(100.0, 100), +1, false,
        {{"C_rz", Samples(100, [](int k) { return 20.0 * double(k) / 100.0; })}}));
    actions += Action("D8", {Driver("D8")}, ActionMaps(Ramp(90.0, 90), +1, false, D8Channels()));
    actions += Action("D11", {Driver("D11")}, ActionMaps(Ramp(100.0, 100), +1, false,
        {{"Switch", Samples(100, [](int k) { return k == 100 ? 1.0 : 0.0; }), true}}));
    actions += Action("D9", {Driver("D9")}, ActionMaps(Ramp(100.0, 100), +1, false,
        {{"D10_rz", Samples(100, [](int k) { return 45.0 * double(k) / 100.0; })}}),
        std::string(), options.cycle);
    std::vector<ChannelSpec> d10 = {
        {"Q6_tx", Samples(90, [](int k) { return double(k) / 100.0; })}};
    if (options.cycle) d10.push_back({"D9_rz", Samples(90, [](int k) { return double(k); })});
    actions += Action("D10", {Driver("D10")}, ActionMaps(Ramp(90.0, 90), +1, false, d10),
                      std::string(), true);

    std::string movers;
    movers += MathMover("D1_Q_tx", Control("Q") + ".avars:tx", "add",
                        Output("D1", "Q_tx").GetString());
    movers += MathMover("D2_Q_tx", Control("Q") + ".avars:tx", "add",
                        Output("D2", "Q_tx").GetString());
    movers += MathMover("D1_2_Q_ty", Control("Q") + ".avars:ty", "add",
                        Output("D1_2", "Q_ty").GetString());
    movers += MathMover("D2_Q2_ty", Control("Q2") + ".avars:ty", "add",
                        Output("D2", "Q2_ty").GetString());
    movers += MathMover("D7_C_rz", Control("C") + ".avars:rz", "add",
                        Output("D7", "C_rz").GetString());
    if (options.nudgeD10) {
        std::vector<double> nudge(kFrames, 0.0);
        nudge[17] = 45.0;
        movers += MathMover("Nudge_D10", Control("D10") + ".avars:rz", "add",
                            TimeSamples(nudge));
    } else {
        movers += MathMover("D9_D10_rz", Control("D10") + ".avars:rz", "add",
                            Output("D9", "D10_rz").GetString());
    }
    movers += MathMover("D10_Q6_tx", Control("Q6") + ".avars:tx", "add",
                        Output("D10", "Q6_tx").GetString());
    if (options.cycle) {
        movers += MathMover("D10_D9_rz", Control("D9") + ".avars:rz", "add",
                            Output("D10", "D9_rz").GetString());
    }
    movers += MathMover("D8_Opacity", kHead.GetString() + ".primvars:opacity", "add",
                        Output("D8", "Opacity").GetString());
    movers += MathMover("Switch_D8", kHead.GetString() + ".primvars:switchIndex", "blend",
                        Output("D8", "Switch").GetString(),
                        Output("D8", "active").GetString());
    movers += MathMover("Switch_D11", kHead.GetString() + ".primvars:switchIndex", "blend",
                        Output("D11", "Switch").GetString(),
                        Output("D11", "active").GetString());
    movers += MathMover("KnobDial", kKnob.GetString() + ".userDial", "add", "1");
    // A primvar chain no table drives: the scalar publication alone.
    movers += MathMover("Glow", kHead.GetString() + ".primvars:glow", "add",
                        TimeSamples(Samples(kFrames - 1, [](int k) {
                            return 0.125 * double(k + 1); })));
    std::string skin =
        "            def RigExecSkinMover \"StrokeSkin\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n"
        "            {\n"
        "                rel rigExec:moves = </Char/Art/Stroke.points>\n"
        "                rel rigExec:influences = [</Char/Rig/Joints/B>]\n"
        "                int[] rigExec:jointIndices = [0, 0, 0]\n"
        "                float[] rigExec:jointWeights = [1, 1, 1]\n"
        "                uniform int rigExec:elementSize = 1\n";
    if (options.correctives == Correctives::Nested) {
        skin += CorrectiveMover("/Char/Rig/Movers/StrokeSkin", "                ",
                                options.zeroSample);
    }
    skin += "            }\n";
    movers += skin;
    if (options.correctives != Correctives::Nested) {
        movers += CorrectiveMover("/Char/Rig/Movers", "            ", options.zeroSample);
    }
    // Bottom sibling fires first: the top switch blend is applied last.
    std::string order = options.d8OnTop ? "\"Switch_D8\", \"Switch_D11\""
                                        : "\"Switch_D11\", \"Switch_D8\"";
    if (options.correctives == Correctives::SiblingBelow) {
        order += ", \"StrokeSkin\", \"Correctives\"";
    } else if (options.correctives == Correctives::SiblingAbove) {
        order += ", \"Correctives\", \"StrokeSkin\"";
    }

    std::string text =
        "#usda 1.0\n(\n    defaultPrim = \"Char\"\n    startTimeCode = 1\n"
        "    endTimeCode = 18\n    upAxis = \"Y\"\n)\n"
        "def Xform \"Char\"\n{\n"
        "    def RigExecRoot \"Rig\"\n    {\n"
        "        def Scope \"Controls\"\n        {\n" + controls + "        }\n"
        "        def Scope \"Joints\"\n        {\n"
        "            def RigExecJoint \"B\"\n            {\n"
        "                matrix4d rest:space = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (-0.5, -0.5, 0, 1))\n"
        "                double avars:rz.timeSamples = " + TimeSamples(brz) + "\n"
        "            }\n        }\n"
        "        def Scope \"Actions\"\n        {\n" + actions + "        }\n"
        "        def Scope \"Movers\"\n        {\n"
        "            reorder nameChildren = [" + order + "]\n" +
        movers + "        }\n"
        "    }\n"
        "    def Scope \"Art\"\n    {\n"
        "        def Xform \"Head\"\n        {\n"
        "            float primvars:switchIndex = 7\n"
        "            float primvars:opacity = 1\n"
        "            float primvars:glow = 0.5\n"
        "        }\n"
        "        def Xform \"Knob\"\n        {\n            float userDial = 0\n        }\n"
        "        def Mesh \"Stroke\"\n        {\n"
        "            int[] faceVertexCounts = [3]\n"
        "            int[] faceVertexIndices = [0, 1, 2]\n"
        "            point3f[] points = [(0, 0.3, 0), (0.5, 0.3, 0), (0, 0.8, 0)]\n"
        "        }\n"
        "        def Scope \"Shapes\"\n        {\n"
        "            def BlendShape \"Q1_k0\"\n            {\n"
        "                vector3f[] offsets = [(0, 0, 0)]\n                int[] pointIndices = [0]\n"
        "            }\n"
        "            def BlendShape \"Q1_k50\"\n            {\n"
        "                vector3f[] offsets = [(0.5, 0, 0)]\n                int[] pointIndices = [0]\n"
        "            }\n"
        "            def BlendShape \"Q1_k100\"\n            {\n"
        "                vector3f[] offsets = [(1, 0, 0)]\n                int[] pointIndices = [0]\n"
        "            }\n"
        "        }\n"
        "    }\n"
        "}\n";
    SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer->ImportFromString(text)) {
        std::printf("FAIL: the dials stage does not parse\n");
        ++failures;
        return UsdStageRefPtr();
    }
    return UsdStage::Open(layer);
}

double
Published(const RigExecRigPose &pose, const SdfPath &path)
{
    const auto found = pose.movedProperties.find(path);
    if (found == pose.movedProperties.end()) return std::nan("");
    const VtValue &v = found->second;
    if (v.IsHolding<float>()) return double(v.UncheckedGet<float>());
    if (v.IsHolding<double>()) return v.UncheckedGet<double>();
    return std::nan("");
}

GfVec3f
StrokePoint(const RigExecRigPose &pose, size_t index)
{
    const auto found = pose.movedProperties.find(kStroke.AppendProperty(TfToken("points")));
    if (found == pose.movedProperties.end() || !found->second.IsHolding<VtVec3fArray>()) {
        return GfVec3f(std::nanf(""));
    }
    const VtVec3fArray &points = found->second.UncheckedGet<VtVec3fArray>();
    return index < points.size() ? points[index] : GfVec3f(std::nanf(""));
}

bool
Near(double got, double expected, double tolerance = 1e-6)
{
    return std::abs(got - expected) <= tolerance;
}

void
Expect(const char *what, int frame, double got, double expected,
       double tolerance = 1e-6)
{
    if (!Near(got, expected, tolerance)) {
        ++failures;
        std::printf("FAIL %s at frame %d: %.9g, expected %.9g\n", what, frame, got,
                    expected);
    }
}

double
Key(const char *dial, int frame)
{
    return Mainline().at(dial)[size_t(frame - 1)];
}

// The closest whole frame of a linear 0 -> end ramp over n frames (the
// keyed angles are never ties).
int32_t
RampFrame(double angle, double end, int n)
{
    if (!(end > 0 ? angle > 0 : angle < 0)) return 0;
    return int32_t(std::min(double(n), std::floor(angle / end * double(n) + 0.5)));
}

// Every test frame against live, frozen (frame-cache worker) and the
// .rigexec runtime, in \p order with a repeat; the runtime's scalar
// primvars are the evaluator's list with live's values.
void
CheckEvaluators(RigExecRigEvaluator &evaluator, const UsdStageRefPtr &stage,
                const std::string &label, int frames = kFrames,
                std::vector<int> order = {3, 11, 1, 18, 14, 9, 17, 7, 7, 12, 16, 10})
{
    for (int f = 1; f <= frames; ++f) {
        std::string why;
        if (!RigExecFrozenMatchesLive(&evaluator, kRig, UsdTimeCode(double(f)), &why)) {
            ++failures;
            std::printf("FAIL %s frozen frame %d: %s\n", label.c_str(), f, why.c_str());
        }
    }
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &error)) {
        ++failures;
        std::printf("FAIL %s bake: %s\n", label.c_str(), error.c_str());
        return;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        ++failures;
        std::printf("FAIL %s open: %s\n", label.c_str(), error.c_str());
        return;
    }
    const std::vector<SdfPath> scalars = evaluator.GetScalarPrimvarTargets();
    for (int f : order) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f)));
        if (!player.Play(double(f), &error)) {
            ++failures;
            std::printf("FAIL %s play frame %d: %s\n", label.c_str(), f, error.c_str());
            return;
        }
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeRun(pose, player.Reader(), &diffs)) {
            ++failures;
            std::printf("FAIL %s runtime frame %d:\n", label.c_str(), f);
            for (const auto &d : diffs) std::printf("    %s\n", d.c_str());
        }
        const auto runtimeScalars = player.Reader().GetScalarPrimvars();
        CHECK(runtimeScalars.size() == scalars.size());
        for (size_t k = 0; k < runtimeScalars.size() && k < scalars.size(); ++k) {
            CHECK(runtimeScalars[k].path == scalars[k].GetString());
            if (runtimeScalars[k].value != float(Published(pose, scalars[k]))) {
                ++failures;
                std::printf("FAIL %s runtime scalar %s frame %d: %g, live %g\n",
                            label.c_str(), runtimeScalars[k].path.c_str(), f,
                            runtimeScalars[k].value, Published(pose, scalars[k]));
            }
        }
    }
}

std::string
Export(const SdfLayerHandle &layer)
{
    std::string text;
    layer->ExportToString(&text);
    return text;
}

bool
Compiled(RigExecRigEvaluator &evaluator, const char *label)
{
    std::vector<std::string> notes;
    if (!evaluator.Compile(&notes)) {
        ++failures;
        for (const auto &n : notes) std::printf("FAIL %s compile: %s\n", label, n.c_str());
        return false;
    }
    for (const auto &[path, why] : evaluator.GetSkippedOperations()) {
        ++failures;
        std::printf("FAIL %s skipped %s: %s\n", label, path.GetText(), why.c_str());
    }
    return true;
}

// ---- Literal frame tables ----

const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// Dial T's angle per frame (dial U's is its negation), and the frame each
// table gives it. Samples [0, 10, 20, 30]: 5, 15 and 25 are ties; past 30
// the last frame holds (out to FLT_MAX: a finite angle beyond it matches no
// row); s_0, the other side and a non-finite angle read 0.
const std::vector<double> kAngles = {-5, 0, 4, 5, 6, 10, 15, 15.001, 25, 29, 31,
                                     100, 370, 1e30, kNan, kInf, -kInf, 1e39};
// Ta [0, 10, 20, 30] earlier ties; Tn the negative side on dial U.
const std::vector<int> kTa = {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 3, 3, 0, 0, 0, 0};
// Tl [0, 10, 20, 30] later ties.
const std::vector<int> kTl = {0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0};
// Ts [0, 10, 10, 20] (a stepped repeat) earlier ties, Tsl later ties.
const std::vector<int> kTs = {0, 0, 0, 0, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0};
const std::vector<int> kTsl = {0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0};

// Dial W's angle per frame (dial V's is its negation) and the frame of a
// full turn 0 -> 360 by 5 degrees (N = 72) on side +1 (Wn: 0 -> -360 on
// side -1 over V). Inside [0, 360] an angle is left unwrapped: 360 itself
// is the last frame (the seam row); outside, the angle folds into the
// window first: 361 -> 1 -> frame 0, 370 -> 10 -> frame 2, -10 -> 350,
// 1e5 -> 280, 1e6 + 10 -> 290 (the fold has no turn bound).
const std::vector<double> kTurnAngles = {0, 2.5, 5, 180, 357.5, 359.9, 360, 361, 370,
                                         720, 725, -10, -350, 1e5, 1e6 + 10, 0, 0, 0};
const std::vector<int> kTurn = {0, 0, 1, 36, 71, 72, 72, 0, 2, 0, 1, 70, 2, 56, 58,
                                0, 0, 0};

// The full turn's channel: two degrees per frame, so the seam publishes 144.
std::vector<ChannelSpec>
SpinChannels()
{
    return {{"Spin", Samples(72, [](int k) { return 2.0 * double(k); })}};
}

// Dials T and U are plain double properties (a table reads any float or
// double property; a non-finite control angle would make its control frame
// unusable); W and V are control dials.
const SdfPath kDialT("/Char/Rig/Dials.angleT");
const SdfPath kDialU("/Char/Rig/Dials.angleU");

UsdStageRefPtr
MakeTableStage()
{
    std::string controls;
    for (const char *dial : {"W", "V"}) {
        controls += ControlPrim(dial, "                double avars:rz = 0\n");
    }
    const std::vector<double> plain = {0, 10, 20, 30}, stepped = {0, 10, 10, 20};
    const std::vector<double> negative = {0, -10, -20, -30};
    const std::string turnFloats = "                float turn:shift = 0\n"
                                   "                float turn:angle = 0\n";
    std::string actions;
    const std::string t = kDialT.GetString(), u = kDialU.GetString();
    actions += Action("Ta", {t}, ActionMaps(plain, +1, false, {}));
    actions += Action("Tl", {t}, ActionMaps(plain, +1, true, {}));
    actions += Action("Tn", {u}, ActionMaps(negative, -1, false, {}));
    actions += Action("Ts", {t}, ActionMaps(stepped, +1, false, {}));
    actions += Action("Tsl", {t}, ActionMaps(stepped, +1, true, {}));
    actions += Action("Wp", {ActionPath("Wp") + ".turn:angle", Driver("W")},
                      ActionMaps(Ramp(360.0, 72), +1, false, SpinChannels()), turnFloats, true);
    actions += Action("Wn", {ActionPath("Wn") + ".turn:angle", Driver("V")},
                      ActionMaps(Ramp(-360.0, 72), -1, false, SpinChannels()), turnFloats, true);
    const std::string movers = WrapMovers("Wp", Driver("W"), +1) +
                               WrapMovers("Wn", Driver("V"), -1);
    const std::string text =
        "#usda 1.0\n(\n    defaultPrim = \"Char\"\n    startTimeCode = 1\n"
        "    endTimeCode = " + std::to_string(kAngles.size()) + "\n    upAxis = \"Y\"\n)\n"
        "def Xform \"Char\"\n{\n"
        "    def RigExecRoot \"Rig\"\n    {\n"
        "        def Scope \"Controls\"\n        {\n" + controls + "        }\n"
        "        def Scope \"Dials\"\n        {\n"
        "            double angleT = 0\n            double angleU = 0\n        }\n"
        "        def Scope \"Actions\"\n        {\n" + actions + "        }\n"
        "        def Scope \"Movers\"\n        {\n" + movers + "        }\n"
        "    }\n"
        "}\n";
    SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer->ImportFromString(text)) {
        std::printf("FAIL: the frame-table stage does not parse\n");
        ++failures;
        return UsdStageRefPtr();
    }
    UsdStageRefPtr stage = UsdStage::Open(layer);
    // One angle per frame (non-finite ones too, which USDA text cannot say).
    for (size_t f = 0; f < kAngles.size(); ++f) {
        const UsdTimeCode time(double(f + 1));
        stage->GetAttributeAtPath(kDialT).Set(kAngles[f], time);
        stage->GetAttributeAtPath(kDialU).Set(-kAngles[f], time);
        stage->GetAttributeAtPath(Avar("W", "rz")).Set(kTurnAngles[f], time);
        stage->GetAttributeAtPath(Avar("V", "rz")).Set(-kTurnAngles[f], time);
    }
    return stage;
}

// The literal tables: frame, weight, active and the full turn's channel per
// frame in every evaluator.
void
TestFrameTables()
{
    CHECK(kTa.size() == kAngles.size() && kTl.size() == kAngles.size() &&
          kTs.size() == kAngles.size() && kTsl.size() == kAngles.size() &&
          kTurnAngles.size() == kAngles.size() && kTurn.size() == kAngles.size());
    UsdStageRefPtr stage = MakeTableStage();
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!Compiled(evaluator, "frame tables")) return;
    const struct {
        const char *action;
        const std::vector<int> *frames;
        int n;
    } tables[] = {{"Ta", &kTa, 3}, {"Tl", &kTl, 3}, {"Tn", &kTa, 3}, {"Ts", &kTs, 3},
                  {"Tsl", &kTsl, 3}, {"Wp", &kTurn, 72}, {"Wn", &kTurn, 72}};
    for (int f = 1; f <= int(kAngles.size()); ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f)));
        CHECK(pose.valid);
        for (const auto &t : tables) {
            const int k = (*t.frames)[size_t(f - 1)];
            // Tn and Wn read the negated angles (dials U and V).
            const double angle = t.frames == &kTurn ? kTurnAngles[size_t(f - 1)]
                                                    : kAngles[size_t(f - 1)];
            const bool negated = t.action[1] == 'n';
            const std::string what = std::string(t.action) + " at " +
                                     Number(negated ? -angle : angle);
            Expect((what + ": frame").c_str(), f, Published(pose, Output(t.action, "frame")),
                   double(k), 0.0);
            Expect((what + ": weight").c_str(), f, Published(pose, Output(t.action, "weight")),
                   double(float(k) / float(t.n)), 0.0);
            Expect((what + ": active").c_str(), f, Published(pose, Output(t.action, "active")),
                   k > 0 ? 1.0 : 0.0, 0.0);
        }
        for (const char *turn : {"Wp", "Wn"}) {
            Expect((std::string(turn) + " spin").c_str(), f, Published(pose, Output(turn, "Spin")),
                   2.0 * double(kTurn[size_t(f - 1)]), 0.0);
        }
    }
    std::vector<int> order;
    for (int f = int(kAngles.size()); f >= 1; --f) order.push_back(f);
    order.push_back(7);
    CheckEvaluators(evaluator, stage, "frame tables", int(kAngles.size()), order);
}

// ---- The dials rig ----

// Every table's published values at every frame (whole steps, the hold,
// sides, ranges, additive offsets, stepped switch values, the chained
// dial's offset); the compiled scalar list; parity in every evaluator;
// edit tiers; and no stage authoring.
void
TestOutputs()
{
    Options options;
    options.nudgeD10 = true;
    UsdStageRefPtr stage = MakeStage(options);
    if (!stage) return;
    const std::string rootBefore = Export(stage->GetRootLayer());
    const std::string sessionBefore = Export(stage->GetSessionLayer());

    RigExecRigEvaluator evaluator(stage, kRig);
    if (!Compiled(evaluator, "dials")) return;
    std::vector<std::string> reasons;
    CHECK(evaluator.IsBakeable(&reasons));
    for (const auto &r : reasons) std::printf("  not bakeable: %s\n", r.c_str());

    // The compiled scalar list: float primvars only, never the knob's dial;
    // the tables publish none (no rigExec:meshes).
    const SdfPath glow = kHead.AppendProperty(TfToken("primvars:glow"));
    const SdfPath opacity = kHead.AppendProperty(TfToken("primvars:opacity"));
    const SdfPath sw = kHead.AppendProperty(TfToken("primvars:switchIndex"));
    const std::vector<SdfPath> scalars = evaluator.GetScalarPrimvarTargets();
    CHECK((scalars == std::vector<SdfPath>{glow, opacity, sw}));

    for (int f = 1; f <= kFrames; ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f)));
        CHECK(pose.valid);
        // D1: whole steps and the hold; the negative side is D1 2's.
        const double q1x[] = {0, 0.10, 0.25, 0.26, 0.99, 1.00, 0, 0, 0.5, 0.5, 1.0,
                              0, 0, 0, 0, 0, 0, 0};
        Expect("D1 q1 x offset", f, Published(pose, Output("D1", "Q_tx")), q1x[f - 1]);
        const int32_t d1 = RampFrame(Key("D1", f), 100.0, 100);
        Expect("D1 frame", f, Published(pose, Output("D1", "frame")), d1, 0.0);
        Expect("D1 weight", f, Published(pose, Output("D1", "weight")),
               double(float(d1) / float(100)), 0.0);
        Expect("D1 active", f, Published(pose, Output("D1", "active")),
               d1 > 0 ? 1.0 : 0.0, 0.0);
        // Negative side: D1 2 holds at its end.
        Expect("D1 2 q1 y offset", f, Published(pose, Output("D1_2", "Q_ty")),
               f == 7 ? -0.20 : f == 8 ? -0.50 : 0.0);
        // Additive offsets: two tables at +50 each publish their own offset;
        // the add movers on q1 sum them (0.5 + 0.25), q2 takes 0.15.
        Expect("D2 q1 x offset", f, Published(pose, Output("D2", "Q_tx")), f == 9 ? 0.25 : 0.0);
        Expect("D2 q2 y offset", f, Published(pose, Output("D2", "Q2_ty")), f == 9 ? 0.15 : 0.0);
        if (f == 9) {
            Expect("additive q1 x sum", f,
                   double(float(Published(pose, Output("D1", "Q_tx")))) +
                       double(float(Published(pose, Output("D2", "Q_tx")))),
                   0.75);
        }
        // Rotated frame: +20 on C at D7 = +100, unclamped.
        Expect("D7 C rz offset", f, Published(pose, Output("D7", "C_rz")), f == 10 ? 20.0 : 0.0);
        // Stepped values: S0 S1 S2 S2 with D8 active; D11's S1 at its last
        // frame.
        const int32_t d8 = RampFrame(Key("D8", f), 90.0, 90);
        Expect("D8 active", f, Published(pose, Output("D8", "active")),
               d8 > 0 ? 1.0 : 0.0, 0.0);
        Expect("D8 switch value", f, Published(pose, Output("D8", "Switch")),
               d8 < 30 ? 0.0 : d8 < 60 ? 1.0 : 2.0, 0.0);
        Expect("D8 opacity offset", f, Published(pose, Output("D8", "Opacity")),
               double(float((1.0 - 0.5 * double(d8) / 90.0) - 1.0)));
        Expect("D11 switch value", f, Published(pose, Output("D11", "Switch")),
               f == 17 ? 1.0 : 0.0, 0.0);
        Expect("D11 active", f, Published(pose, Output("D11", "active")),
               f == 17 ? 1.0 : 0.0, 0.0);
        // Chained dial: D9 at +100 publishes D10's +45.
        Expect("D9 D10 rz offset", f, Published(pose, Output("D9", "D10_rz")),
               f == 18 ? 45.0 : 0.0);
        // A dial a property chain revises drives its table with the chain's
        // result (+45 at frame 18), in the same evaluation.
        Expect("D10 rz chain", f, Published(pose, Avar("D10", "rz")), f == 18 ? 45.0 : 0.0);
        Expect("D10 frame", f, Published(pose, Output("D10", "frame")),
               f == 18 ? 45.0 : 0.0, 0.0);
        Expect("D10 q6 x offset", f, Published(pose, Output("D10", "Q6_tx")),
               f == 18 ? 0.45 : 0.0);
        // The primvar chain no table drives publishes its own value.
        Expect("glow", f, Published(pose, glow), 0.5 + 0.125 * double(f), 0.0);
    }

    CheckEvaluators(evaluator, stage, "dials");
    // Compile, evaluation, freezing, baking and playback author nothing.
    CHECK(Export(stage->GetRootLayer()) == rootBefore);
    CHECK(Export(stage->GetSessionLayer()) == sessionBefore);

    // Edit tiers. inputs:enabled is a value: the signature holds and a
    // disabled table publishes zeros on every map, value channels included.
    const size_t signature = evaluator.GetCompiledRigSignature();
    UsdPrim d8 = stage->GetPrimAtPath(SdfPath(ActionPath("D8")));
    d8.CreateAttribute(TfToken("inputs:enabled"), SdfValueTypeNames->Bool).Set(false);
    {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(14.0));
        CHECK(evaluator.GetCompiledRigSignature() == signature);
        for (const char *map : {"frame", "weight", "active", "Switch", "Opacity"}) {
            const SdfPath output = Output("D8", map);
            Expect(output.GetText(), 14, Published(pose, output), 0.0, 0.0);
        }
        // The blend passes the mainline switch through; the offset adds 0.
        Expect("disabled D8 switch", 14, Published(pose, sw), 7.0, 0.0);
        Expect("disabled D8 opacity", 14, Published(pose, opacity), 1.0, 0.0);
    }
    d8.GetAttribute(TfToken("inputs:enabled")).Set(true);
    CHECK(Published(evaluator.Evaluate(UsdTimeCode(14.0)), Output("D8", "Switch")) == 2.0);
    CHECK(evaluator.GetCompiledRigSignature() == signature);
    // A dial's own key is a value too.
    {
        UsdAttribute dial = stage->GetAttributeAtPath(Avar("D1", "rz"));
        dial.Set(30.0, UsdTimeCode(3.0));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(3.0));
        CHECK(evaluator.GetCompiledRigSignature() == signature);
        Expect("re-keyed D1", 3, Published(pose, Output("D1", "Q_tx")), 0.30);
        dial.Set(25.3, UsdTimeCode(3.0));
        evaluator.Evaluate(UsdTimeCode(3.0));
    }
    // Rows, cuts and the clamp are structural: each edit recompiles (a new
    // signature) and changes the output; reverting it returns the original
    // signature.
    const auto structural = [&](const char *what, const std::function<void()> &edit,
                                const std::function<void()> &revert, int frame,
                                const SdfPath &read, double expected) {
        const size_t was = evaluator.GetCompiledRigSignature();
        edit();
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(frame)));
        if (evaluator.GetCompiledRigSignature() == was) {
            ++failures;
            std::printf("FAIL %s edit left the compiled rig signature as it was\n", what);
        }
        Expect(what, frame, Published(pose, read), expected);
        revert();
        evaluator.Evaluate(UsdTimeCode(double(frame)));
        CHECK(evaluator.GetCompiledRigSignature() == was);
    };
    const auto original = [&](const char *action, std::vector<MapSpec> maps) {
        return [&stage, action, maps]() { AuthorMaps(stage, action, maps); };
    };
    // Re-recorded samples (every 2 degrees): 25.3 is closest to 26, frame 13.
    structural("re-recorded rows",
               original("D1", ActionMaps(Ramp(200.0, 100), +1, false, D1Channels())),
               original("D1", ActionMaps(Ramp(100.0, 100), +1, false, D1Channels())),
               3, Output("D1", "Q_tx"), 0.13);
    // The other side: D1 2's rows re-made for side +1 read 0 at -20.
    structural("side",
               original("D1_2", ActionMaps(Ramp(-50.0, 50), +1, false, D1_2Channels())),
               original("D1_2", ActionMaps(Ramp(-50.0, 50), -1, false, D1_2Channels())),
               7, Output("D1_2", "Q_ty"), 0.0);
    {
        // A held tail (samples min(k, 50)): its cell is the earliest tied
        // frame, 50, with earlier ties and the tail's end, 100, with later.
        const std::vector<double> held = Samples(100, [](int k) { return double(std::min(k, 50)); });
        AuthorMaps(stage, "D1", ActionMaps(held, +1, false, D1Channels()));
        Expect("held tail, earlier", 9,
               Published(evaluator.Evaluate(UsdTimeCode(9.0)), Output("D1", "Q_tx")), 0.5);
        structural("later ties", original("D1", ActionMaps(held, +1, true, D1Channels())),
                   original("D1", ActionMaps(held, +1, false, D1Channels())),
                   9, Output("D1", "Q_tx"), 1.0);
        AuthorMaps(stage, "D1", ActionMaps(Ramp(100.0, 100), +1, false, D1Channels()));
        evaluator.Evaluate(UsdTimeCode(9.0));
    }
    {
        // Value cuts publish the channel's own value at frame 90 (0.5) where
        // offset cuts publish its change from rest (-0.5).
        std::vector<ChannelSpec> channels = D8Channels();
        channels[1].value = true;
        structural("value cuts", original("D8", ActionMaps(Ramp(90.0, 90), +1, false, channels)),
                   original("D8", ActionMaps(Ramp(90.0, 90), +1, false, D8Channels())),
                   15, Output("D8", "Opacity"), 0.5);
        channels = D8Channels();
        channels[0].samples = Samples(90, [](int k) { return k < 45 ? 0.0 : 3.0; });
        structural("channel values",
                   original("D8", ActionMaps(Ramp(90.0, 90), +1, false, channels)),
                   original("D8", ActionMaps(Ramp(90.0, 90), +1, false, D8Channels())),
                   13, Output("D8", "Switch"), 0.0);
    }
    {
        // Clamped, the 20-degree channel publishes 1.
        const UsdAttribute clamp = stage->GetAttributeAtPath(
            SdfPath(ActionPath("D7") + "/C_rz.rigExec:clampOutput"));
        CHECK(clamp);
        structural("clampOutput", [&]() { clamp.Set(true); }, [&]() { clamp.Set(false); },
                   10, Output("D7", "C_rz"), 1.0);
    }
    CHECK(evaluator.GetCompiledRigSignature() == signature);

    // A driver that is no property sets the table aside alone.
    stage->GetPrimAtPath(SdfPath(ActionPath("D11")))
        .GetRelationship(TfToken("rigExec:driverAttributes"))
        .SetTargets({SdfPath(Control("D11") + ".avars:missing")});
    const RigExecRigPose refused = evaluator.Evaluate(UsdTimeCode(17.0));
    CHECK(evaluator.GetSkippedOperations().count(SdfPath(ActionPath("D11"))) == 1);
    CHECK(evaluator.GetSkippedOperations().size() == 1);
    for (const auto &[path, why] : evaluator.GetSkippedOperations()) {
        std::printf("  set aside %s: %s\n", path.GetText(), why.c_str());
    }
    Expect("D8 beside a refused table", 17, Published(refused, Output("D8", "active")),
           1.0, 0.0);
}

// The order gate: the corrective nested under the skin and the same mover
// as a sibling below it both act before the bones; above it, after. In
// every evaluator.
void
TestCorrectiveOrder()
{
    const struct {
        Correctives layout;
        const char *label;
        double x, y;
    } cases[] = {
        {Correctives::Nested, "nested", -1.3, 1.0},
        {Correctives::SiblingBelow, "sibling below", -1.3, 1.0},
        {Correctives::SiblingAbove, "sibling above", -0.3, 0.0},
    };
    for (const auto &c : cases) {
        Options options;
        options.correctives = c.layout;
        UsdStageRefPtr stage = MakeStage(options);
        if (!stage) continue;
        RigExecRigEvaluator evaluator(stage, kRig);
        if (!Compiled(evaluator, c.label)) continue;
        const GfVec3f rest = StrokePoint(evaluator.Evaluate(UsdTimeCode(1.0)), 0);
        CHECK(Near(rest[0], 0.0) && Near(rest[1], 0.3));
        const GfVec3f p = StrokePoint(evaluator.Evaluate(UsdTimeCode(11.0)), 0);
        if (!Near(p[0], c.x, 1e-5) || !Near(p[1], c.y, 1e-5)) {
            ++failures;
            std::printf("FAIL corrective %s: q1 (%g, %g), expected (%g, %g)\n", c.label,
                        p[0], p[1], c.x, c.y);
        }
        CheckEvaluators(evaluator, stage, c.label);
    }
    // A k = 0 sample (activation 0) is refused: the blend fails atomically
    // and the corrective leaves the points unmoved, so B turns the setup
    // point.
    Options zero;
    zero.zeroSample = true;
    UsdStageRefPtr stage = MakeStage(zero);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> notes;
    if (evaluator.Compile(&notes) && evaluator.GetSkippedOperations().empty()) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(11.0));
        const GfVec3f p = StrokePoint(pose, 0);
        if (!Near(p[0], -1.3, 1e-5) || !Near(p[1], 0.0, 1e-5)) {
            ++failures;
            std::printf("FAIL k = 0 corrective applied: q1 (%g, %g)\n", p[0], p[1]);
        }
        CHECK(!pose.diagnostics.empty());
    }
}

// Scalar publication: the compiled primvar chains reach Hydra as constant
// float primvars on the group prim, live (bridge + results scene index over
// the stage scene index) and from .rigexec playback, identically; the
// knob's float dial mints no published prim. With \p switches, the switch
// values the tables drive are checked as well (needs the channel movers).
void
TestHydraPublish(bool switches)
{
    UsdStageRefPtr stage = MakeStage();
    if (!stage) return;
    auto liveStore = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge live(stage, kRig, liveStore);
    std::vector<std::string> errors;
    CHECK(live.Compile(&errors));
    for (const auto &e : errors) std::printf("  bridge compile: %s\n", e.c_str());

    auto stageIndex = UsdImagingStageSceneIndex::New();
    stageIndex->SetStage(stage);
    stageIndex->ApplyPendingUpdates();
    auto flattened = HdFlatteningSceneIndex::New(stageIndex, HdFlattenedDataSourceProviders());
    auto results = RigExecResultsSceneIndex::New(flattened, liveStore);

    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> notes;
    CHECK(evaluator.Compile(&notes));
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    const std::filesystem::path scratch = std::filesystem::temp_directory_path() /
        ("testRigExecStepMaps-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::error_code made;
    CHECK(std::filesystem::create_directory(scratch, made) && !made);
    if (made || bytes.empty()) return;
    const std::string binary = (scratch / "tds.rigexec").string();
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(bytes.data()),
                     std::streamsize(bytes.size()));
    }
    auto playStore = std::make_shared<RigExecSnapshotStore>();
    RigExecBakedPlayback play(stage, kRig, playStore);
    CHECK(play.Open(binary, &error));

    const TfToken switchIndex("switchIndex"), glow("glow");
    const float expected[] = {0, 1, 2, 2, 7, 2};
    const auto drawn = [&](const TfToken &name, float *out) {
        const HdPrimvarsSchema primvars =
            HdPrimvarsSchema::GetFromParent(results->GetPrim(kHead).dataSource);
        const HdPrimvarSchema primvar = primvars.GetPrimvar(name);
        const HdSampledDataSourceHandle value = primvar.GetPrimvarValue();
        const HdTokenDataSourceHandle interpolation = primvar.GetInterpolation();
        if (!value || !interpolation ||
            interpolation->GetTypedValue(0) != HdPrimvarSchemaTokens->constant) {
            return false;
        }
        const VtValue v = value->GetValue(0);
        if (!v.IsHolding<float>()) return false;
        *out = v.UncheckedGet<float>();
        return true;
    };
    for (int f = 12; f <= 17; ++f) {
        const UsdTimeCode time{double(f)};
        const auto liveResult = live.EvaluateAndPublishResult(time);
        CHECK(liveResult.ok);
        results->NotifyGenerationPublished(liveResult.dirtied);
        const auto playResult = play.EvaluateAndPublishResult(time);
        CHECK(playResult.ok);
        const RigExecImagingSnapshotConstPtr a = liveStore->Get();
        const RigExecImagingSnapshotConstPtr b = playStore->Get();
        CHECK(a && b);
        if (!a || !b) continue;
        CHECK(a->prims.count(kKnob) == 0 && b->prims.count(kKnob) == 0);
        const auto liveHead = a->prims.find(kHead);
        const auto playHead = b->prims.find(kHead);
        CHECK(liveHead != a->prims.end() && playHead != b->prims.end());
        if (liveHead == a->prims.end() || playHead == b->prims.end()) continue;
        CHECK(liveHead->second.shaderFloats == playHead->second.shaderFloats);
        CHECK(liveHead->second.shaderFloats.size() == 3);
        const float glowValue = float(0.5 + 0.125 * double(f));
        CHECK(liveHead->second.shaderFloats.count(glow) &&
              liveHead->second.shaderFloats.at(glow) == glowValue);
        float got = -1.0f;
        CHECK(drawn(glow, &got) && got == glowValue);
        if (switches) {
            CHECK(liveHead->second.shaderFloats.count(switchIndex) &&
                  liveHead->second.shaderFloats.at(switchIndex) == expected[f - 12]);
            got = -1.0f;
            if (!drawn(switchIndex, &got) || got != expected[f - 12]) {
                ++failures;
                std::printf("FAIL Hydra switchIndex at frame %d: %g, expected %g\n", f,
                            got, expected[f - 12]);
            }
        }
    }
    std::error_code ignored;
    std::filesystem::remove_all(scratch, ignored);
}

// ---- Channels: property movers reading table outputs ----

// The dials through their add and blend movers: q1/q2 offsets, C's mainline
// plus the table, the switch and opacity primvars, chained dials in one
// evaluation, in every evaluator.
void
TestChannels()
{
    UsdStageRefPtr stage = MakeStage();
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!Compiled(evaluator, "channels")) return;
    const SdfPath qx = Avar("Q", "tx"), qy = Avar("Q", "ty"), q2y = Avar("Q2", "ty");
    const SdfPath crz = Avar("C", "rz"), d10 = Avar("D10", "rz"), q6x = Avar("Q6", "tx");
    const SdfPath sw = kHead.AppendProperty(TfToken("primvars:switchIndex"));
    const SdfPath opacity = kHead.AppendProperty(TfToken("primvars:opacity"));
    for (int f = 1; f <= kFrames; ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f)));
        const double q1x[] = {0, 0.10, 0.25, 0.26, 0.99, 1.00, 0, 0, 0.75, 0.5, 1.0,
                              0, 0, 0, 0, 0, 0, 0};
        Expect("q1 x", f, Published(pose, qx), q1x[f - 1]);
        Expect("q1 y", f, Published(pose, qy), f == 7 ? -0.2 : f == 8 ? -0.5 : f == 10 ? 0.2 : 0.0);
        Expect("q2 y", f, Published(pose, q2y), f == 9 ? 0.15 : 0.0);
        Expect("C rz", f, Published(pose, crz), f == 10 ? 50.0 : 0.0);
        Expect("D10 rz", f, Published(pose, d10), f == 18 ? 45.0 : 0.0);
        Expect("q6 x", f, Published(pose, q6x), f == 18 ? 0.45 : 0.0);
        const double sws[] = {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 1, 2, 2, 7, 2, 7};
        Expect("switch", f, Published(pose, sw), sws[f - 1], 0.0);
        const int32_t d8 = RampFrame(Key("D8", f), 90.0, 90);
        Expect("opacity", f, Published(pose, opacity),
               double(1.0f + float((1.0 - 0.5 * double(d8) / 90.0) - 1.0)));
    }
    // Rotated frame: C's frame turns to 50 degrees.
    {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(10.0));
        const auto found = pose.controlFrames.find(SdfPath(Control("C")));
        CHECK(found != pose.controlFrames.end());
        if (found != pose.controlFrames.end()) {
            const GfVec3d x = found->second.X() - found->second.Origin();
            const double angle = std::atan2(x[1], x[0]) * 180.0 / 3.14159265358979323846;
            Expect("C world angle", 10, angle, 50.0, 1e-4);
        }
    }
    // A chained dial settles in one evaluation: frame 18 alone gives the
    // sequence's answer.
    {
        RigExecRigEvaluator alone(stage, kRig);
        std::vector<std::string> notes;
        CHECK(alone.Compile(&notes));
        Expect("q6 x alone", 18, Published(alone.Evaluate(UsdTimeCode(18.0)), q6x), 0.45);
    }
    CheckEvaluators(evaluator, stage, "channels");

    // Two stepped blends on one switch: the top sibling is applied last and
    // wins while both are active; reordering swaps the winner.
    Options options;
    options.d8OnTop = false;
    UsdStageRefPtr swapped = MakeStage(options);
    if (swapped) {
        RigExecRigEvaluator e(swapped, kRig);
        if (Compiled(e, "D11 on top")) {
            Expect("D11 on top", 17, Published(e.Evaluate(UsdTimeCode(17.0)), sw), 1.0, 0.0);
            swapped->GetPrimAtPath(SdfPath("/Char/Rig/Movers"))
                .SetChildrenReorder({TfToken("Switch_D8"), TfToken("Switch_D11")});
            Expect("D8 on top", 17, Published(e.Evaluate(UsdTimeCode(17.0)), sw), 2.0, 0.0);
        }
    }

    // Two tables driving each other's dials form an operation cycle,
    // refused.
    Options cyclic;
    cyclic.cycle = true;
    UsdStageRefPtr loop = MakeStage(cyclic);
    if (loop) {
        RigExecRigEvaluator e(loop, kRig);
        std::vector<std::string> notes;
        const bool compiled = e.Compile(&notes);
        bool refused = !compiled || !e.GetSkippedOperations().empty();
        for (const auto &note : notes)
            refused = refused || note.find("cycl") != std::string::npos;
        if (compiled) {
            for (const auto &d : e.Evaluate(UsdTimeCode(18.0)).diagnostics)
                refused = refused || d.find("cycl") != std::string::npos;
        }
        if (!refused) {
            ++failures;
            std::printf("FAIL the D9/D10 cycle was not refused\n");
        }
    }
    TestHydraPublish(true);
}

}  // namespace

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const bool channels = argc > 1 && std::string(argv[1]) == "channels";
    if (channels) {
        TestChannels();
    } else {
        TestFrameTables();
        TestOutputs();
        TestCorrectiveOrder();
        TestHydraPublish(false);
    }
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecStepMaps%s: all tests passed\n", channels ? " channels" : "");
    return 0;
}
