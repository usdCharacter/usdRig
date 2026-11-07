// Input capture over layers whose state delegate is not Sdf's plain one
// (History's recorder, ovlive's BoundLayer): the capture installs its own
// delegate, forwards each change hook to the delegate it replaced, and puts
// that delegate back at teardown. A process holds one capture, so the groups
// run in order on it; uncaptured runs of the same script are the reference.
#include "rigExec/inputReplay.h"
#include "rigExec/inputReplayValues.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/abstractData.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/layerStateDelegate.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/relationshipSpec.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace fs = std::filesystem;

namespace {
int failures = 0, groups = 0;
#define CHECK(x)                                                               \
    do {                                                                       \
        if (!(x)) {                                                            \
            ++failures;                                                        \
            std::printf("FAIL %d: %s\n", __LINE__, #x);                        \
        }                                                                      \
    } while (0)
using Strings = std::vector<std::string>;

bool
SetCaptureEnvironment(const char *name, const std::string &value)
{
#ifdef _WIN32
    // USD changes the Windows environment; the recorder reads the CRT table.
    if (_putenv_s(name, value.c_str()) != 0)
        return false;
#endif
    return TfSetenv(name, value);
}
bool
UnsetCaptureEnvironment(const char *name)
{
#ifdef _WIN32
    if (_putenv_s(name, "") != 0)
        return false;
#endif
    return TfUnsetenv(name);
}

std::string
Bytes(const VtValue &value)
{
    std::string bytes, error;
    return RigExecEncodeInputValue(value, &bytes, &error)
               ? bytes
               : "unencodable " + value.GetTypeName();
}
std::string
Bytes(const SdfAbstractDataConstValue &value)
{
    VtValue copy;
    return value.GetValue(&copy) ? Bytes(copy) : "uncopyable";
}
std::string
Text(const SdfLayerHandle &layer)
{
    std::string text;
    layer->ExportToString(&text);
    return text;
}

// A recorder stand-in that forwards nothing. Like History's, it reads the
// state before each change through its own layer handle.
class HookLog final : public SdfSimpleLayerStateDelegate {
public:
    static TfWeakPtr<HookLog> InstallOn(const SdfLayerHandle &layer)
    {
        TfRefPtr<HookLog> log =
            TfCreateRefPtr(std::make_unique<HookLog>(layer).release());
        layer->SetStateDelegate(log);
        return log;
    }
    explicit HookLog(const SdfLayerHandle &layer) : _layer(layer)
    {
    }
    Strings hooks, attaches;
    size_t installedHooks = 0;

private:
    SdfLayerHandle _layer;

    void Log(const std::string &hook, const SdfPath &path,
             const std::string &detail)
    {
        if (get_pointer(_layer->GetStateDelegate()) == this)
            ++installedHooks;
        hooks.push_back(hook + " " + path.GetString() + " " + detail);
    }
    std::string Old(const SdfPath &path, const TfToken &field)
    {
        return Bytes(_layer->GetField(path, field));
    }
    std::string OldKey(const SdfPath &path, const TfToken &field,
                       const TfToken &key)
    {
        return Bytes(_layer->GetFieldDictValueByKey(path, field, key));
    }
    std::string OldSample(const SdfPath &path, double time)
    {
        VtValue value;
        return _layer->QueryTimeSample(path, time, &value) ? Bytes(value)
                                                           : "none";
    }
    std::string Spec(const SdfPath &path)
    {
        std::string text = std::to_string(int(_layer->GetSpecType(path)));
        for (const auto &field : _layer->ListFields(path))
            text += " " + field.GetString();
        return text;
    }

    void _OnSetLayer(const SdfLayerHandle &layer) override
    {
        attaches.push_back(layer ? layer->GetIdentifier() : "null");
    }
    void _OnSetField(const SdfPath &p, const TfToken &f,
                     const VtValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetField(p, f, v);
        Log("setField", p, f.GetString() + " " + Old(p, f) + " > " + Bytes(v));
    }
    void _OnSetField(const SdfPath &p, const TfToken &f,
                     const SdfAbstractDataConstValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetField(p, f, v);
        Log("setField*", p, f.GetString() + " " + Old(p, f) + " > " + Bytes(v));
    }
    void _OnSetFieldDictValueByKey(const SdfPath &p, const TfToken &f,
                                   const TfToken &k, const VtValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetFieldDictValueByKey(p, f, k, v);
        Log("setDictKey", p,
            f.GetString() + ":" + k.GetString() + " " + OldKey(p, f, k) +
                " > " + Bytes(v));
    }
    void _OnSetFieldDictValueByKey(const SdfPath &p, const TfToken &f,
                                   const TfToken &k,
                                   const SdfAbstractDataConstValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetFieldDictValueByKey(p, f, k, v);
        Log("setDictKey*", p,
            f.GetString() + ":" + k.GetString() + " " + OldKey(p, f, k) +
                " > " + Bytes(v));
    }
    void _OnSetTimeSample(const SdfPath &p, double t, const VtValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetTimeSample(p, t, v);
        Log("setTimeSample", p,
            std::to_string(t) + " " + OldSample(p, t) + " > " + Bytes(v));
    }
    void _OnSetTimeSample(const SdfPath &p, double t,
                          const SdfAbstractDataConstValue &v) override
    {
        SdfSimpleLayerStateDelegate::_OnSetTimeSample(p, t, v);
        Log("setTimeSample*", p,
            std::to_string(t) + " " + OldSample(p, t) + " > " + Bytes(v));
    }
    void _OnCreateSpec(const SdfPath &p, SdfSpecType t, bool inert) override
    {
        SdfSimpleLayerStateDelegate::_OnCreateSpec(p, t, inert);
        Log("createSpec", p,
            std::to_string(int(t)) + (inert ? " inert" : "") + " was " +
                Spec(p));
    }
    void _OnDeleteSpec(const SdfPath &p, bool inert) override
    {
        SdfSimpleLayerStateDelegate::_OnDeleteSpec(p, inert);
        Log("deleteSpec", p,
            std::string(inert ? "inert " : "") + "was " + Spec(p));
    }
    void _OnMoveSpec(const SdfPath &from, const SdfPath &to) override
    {
        SdfSimpleLayerStateDelegate::_OnMoveSpec(from, to);
        Log("moveSpec", from,
            to.GetString() + " was " + Spec(from) + " / " + Spec(to));
    }
    void _OnPushChild(const SdfPath &p, const TfToken &f,
                      const TfToken &v) override
    {
        SdfSimpleLayerStateDelegate::_OnPushChild(p, f, v);
        Log("pushChild(token)", p,
            f.GetString() + " " + Old(p, f) + " + " + v.GetString());
    }
    void _OnPushChild(const SdfPath &p, const TfToken &f,
                      const SdfPath &v) override
    {
        SdfSimpleLayerStateDelegate::_OnPushChild(p, f, v);
        Log("pushChild(path)", p,
            f.GetString() + " " + Old(p, f) + " + " + v.GetString());
    }
    void _OnPopChild(const SdfPath &p, const TfToken &f,
                     const TfToken &v) override
    {
        SdfSimpleLayerStateDelegate::_OnPopChild(p, f, v);
        Log("popChild(token)", p,
            f.GetString() + " " + Old(p, f) + " - " + v.GetString());
    }
    void _OnPopChild(const SdfPath &p, const TfToken &f,
                     const SdfPath &v) override
    {
        SdfSimpleLayerStateDelegate::_OnPopChild(p, f, v);
        Log("popChild(path)", p,
            f.GetString() + " " + Old(p, f) + " - " + v.GetString());
    }
};

fs::path dir, rootPath, subPath, actions;
const SdfPath rigPath("/Rig"), tx("/Rig/J.avars:tx"), fromSub("/FromSub");

void
Prepare()
{
    dir = fs::temp_directory_path() /
          ("rigexec-input-hosting-" +
           std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(fs::create_directory(dir));
    rootPath = dir / "root.usda";
    subPath = dir / "sub.usda";
    actions = dir / "hosting.actions";
    const auto provenance = dir / "provenance.txt";
    {
        std::ofstream out(provenance);
        out << "test-owned hosting inputs\n";
        CHECK(bool(out));
    }
    CHECK(SetCaptureEnvironment("RIGEXEC_INPUT_REPLAY_PROVENANCE",
                                provenance.string()));
    auto sub = SdfLayer::CreateNew(subPath.string());
    SdfPrimSpec::New(sub->GetPseudoRoot(), "FromSub", SdfSpecifierDef);
    CHECK(sub->Save());
    auto stage = UsdStage::CreateNew(rootPath.string());
    stage->GetRootLayer()->SetSubLayerPaths({sub->GetIdentifier()});
    auto joint = stage->DefinePrim(SdfPath("/Rig/J"), TfToken("RigExecJoint"));
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    joint.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1));
    joint.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(2.0);
    CHECK(stage->GetRootLayer()->Save());
}

struct Run {
    std::string text;
    Strings rootHooks, subHooks, subAttaches;
    size_t rootInstalled = 0, subInstalled = 0;
    bool subWentWithItsLayer = false;
};

// Every change hook is reached on the root: Sdf's own edits, the
// type-erased entry points, and an undo's shape through the installed
// delegate (a pop sends no notice alone, so each goes with a delete).
void
Script(const UsdStageRefPtr &stage, RigExecRigEvaluator &rig,
       const SdfLayerHandle &root, SdfLayerRefPtr &sub,
       const TfWeakPtr<HookLog> &rootLog, const TfWeakPtr<HookLog> &subLog,
       Run &run)
{
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (const auto &error : errors)
        std::printf("compile: %s\n", error.c_str());
    CHECK(rig.Evaluate(UsdTimeCode(1)).valid);
    CHECK(stage->GetAttributeAtPath(tx).Set(3.0));
    const double four = 4;
    const SdfAbstractDataConstTypedValue<double> typedFour(&four);
    root->SetField(tx, SdfFieldKeys->Default,
                   static_cast<const SdfAbstractDataConstValue &>(typedFour));
    CHECK(rig.Evaluate(UsdTimeCode(1)).valid);
    {
        auto edits =
            SdfPrimSpec::New(root->GetPseudoRoot(), "Edits", SdfSpecifierDef);
        auto a = SdfPrimSpec::New(edits, "A", SdfSpecifierDef);
        auto b = SdfPrimSpec::New(edits, "B", SdfSpecifierDef);
        root->SetFieldDictValueByKey(edits->GetPath(), SdfFieldKeys->CustomData,
                                     TfToken("note"),
                                     VtValue(std::string("hosted")));
        const int count = 2;
        const SdfAbstractDataConstTypedValue<int> typedCount(&count);
        root->SetFieldDictValueByKey(
            edits->GetPath(), SdfFieldKeys->CustomData, TfToken("count"),
            static_cast<const SdfAbstractDataConstValue &>(typedCount));
        auto samples =
            SdfAttributeSpec::New(edits, "samples", SdfValueTypeNames->Double);
        root->SetTimeSample(samples->GetPath(), 1.0, VtValue(1.5));
        root->SetTimeSample(samples->GetPath(), 2.0, 2.5);
        CHECK(rig.Evaluate(UsdTimeCode(2)).valid);
        auto rel = SdfRelationshipSpec::New(edits, "rel");
        rel->GetTargetPathList().Append(a->GetPath());
        const VtValue targets =
            root->GetField(rel->GetPath(), SdfFieldKeys->TargetPaths);
        rel->GetTargetPathList().Append(b->GetPath());
        SdfPrimSpec::New(edits, "D", SdfSpecifierDef)->SetName("E");
        CHECK(rig.Evaluate(UsdTimeCode(2)).valid);
        {
            SdfChangeBlock block;
            const auto delegate = root->GetStateDelegate();
            delegate->PopChild(edits->GetPath(), SdfChildrenKeys->PrimChildren,
                               TfToken("E"));
            delegate->DeleteSpec(edits->GetPath().AppendChild(TfToken("E")),
                                 false);
            delegate->SetField(rel->GetPath(), SdfFieldKeys->TargetPaths,
                               targets);
            delegate->PopChild(rel->GetPath(),
                               SdfChildrenKeys->RelationshipTargetChildren,
                               b->GetPath());
            delegate->DeleteSpec(rel->GetPath().AppendTarget(b->GetPath()),
                                 false);
        }
    }
    CHECK(rig.Evaluate(UsdTimeCode(3)).valid);
    sub->SetFieldDictValueByKey(fromSub, SdfFieldKeys->CustomData,
                                TfToken("seen"), VtValue(1));
    CHECK(rig.Evaluate(UsdTimeCode(3)).valid);
    if (subLog) {
        run.subHooks = subLog->hooks;
        run.subAttaches = subLog->attaches;
        run.subInstalled = subLog->installedHooks;
    }
    const SdfLayerHandle subHandle(sub);
    root->SetSubLayerPaths({});
    sub.Reset();
    run.subWentWithItsLayer = !subHandle && !subLog;
    CHECK(rig.Evaluate(UsdTimeCode(3)).valid);
    // A bulk replace is recorded as one action, but each change it applies
    // still reaches the hosted delegate. Its diff order is unspecified.
    std::string text = Text(root);
    const auto note = text.find("\"hosted\"");
    CHECK(note != std::string::npos);
    if (note != std::string::npos)
        text.replace(note, 8, "\"imported\"");
    text += "def \"Imported\"\n{\n}\n";
    const size_t mark = rootLog ? rootLog->hooks.size() : 0;
    CHECK(RigExecInputReplayImportFromString(root, text));
    if (rootLog) {
        CHECK(rootLog->hooks.size() > mark);
        std::sort(rootLog->hooks.begin() + std::ptrdiff_t(mark),
                  rootLog->hooks.end());
    }
    CHECK(rig.Evaluate(UsdTimeCode(4)).valid);
    run.text = Text(root);
    if (rootLog) {
        run.rootHooks = rootLog->hooks;
        run.rootInstalled = rootLog->installedHooks;
    }
}

Run
Uncaptured(bool logs)
{
    Run run;
    auto stage = UsdStage::Open(rootPath.string());
    SdfLayerRefPtr root = stage->GetRootLayer(),
                   sub = SdfLayer::Find(subPath.string());
    TfWeakPtr<HookLog> rootLog, subLog;
    if (logs) {
        rootLog = HookLog::InstallOn(root);
        subLog = HookLog::InstallOn(sub);
    }
    {
        RigExecRigEvaluator rig(stage, rigPath);
        Script(stage, rig, root, sub, rootLog, subLog, run);
    }
    const SdfLayerHandle weak(root);
    stage.Reset();
    root.Reset();
    CHECK(!weak); // the next run reads the files afresh
    return run;
}

Run direct, plain, captured;
struct {
    UsdStageRefPtr stage;
    SdfLayerRefPtr root;
    std::unique_ptr<RigExecRigEvaluator> rig;
    TfWeakPtr<HookLog> rootLog, subLog;
    Strings rootAttachesAtStart, subAttachesAtStart;
    bool started = false;
} hosted;

// History watches first, as the launcher orders it; the capture then starts
// over its delegates when the first evaluator is created.
void
CaptureHostsTheDelegateItReplaces()
{
    ++groups;
    CHECK(SetCaptureEnvironment("RIGEXEC_INPUT_REPLAY",
                                "capture:" + actions.string()));
    hosted.stage = UsdStage::Open(rootPath.string());
    hosted.root = hosted.stage->GetRootLayer();
    SdfLayerRefPtr sub = SdfLayer::Find(subPath.string());
    hosted.rootLog = HookLog::InstallOn(hosted.root);
    hosted.subLog = HookLog::InstallOn(sub);
    hosted.rig = std::make_unique<RigExecRigEvaluator>(hosted.stage, rigPath);
    hosted.started = fs::exists(actions);
    CHECK(hosted.started);
    CHECK(hosted.rootLog && hosted.subLog);
    CHECK(get_pointer(hosted.root->GetStateDelegate()) !=
          get_pointer(hosted.rootLog));
    CHECK(get_pointer(sub->GetStateDelegate()) != get_pointer(hosted.subLog));
    hosted.rootAttachesAtStart = hosted.rootLog->attaches;
    hosted.subAttachesAtStart = hosted.subLog->attaches;
    Script(hosted.stage, *hosted.rig, hosted.root, sub, hosted.rootLog,
           hosted.subLog, captured);

    CHECK(!direct.rootHooks.empty() && !direct.subHooks.empty());
    CHECK(captured.rootHooks == direct.rootHooks);
    CHECK(captured.subHooks == direct.subHooks);
    if (captured.rootHooks != direct.rootHooks)
        for (size_t i = 0;
             i < std::max(captured.rootHooks.size(), direct.rootHooks.size());
             ++i)
            std::printf(
                "  %s | %s\n",
                i < direct.rootHooks.size() ? direct.rootHooks[i].c_str() : "-",
                i < captured.rootHooks.size() ? captured.rootHooks[i].c_str()
                                              : "-");
    std::set<std::string> kinds;
    for (const auto &hook : direct.rootHooks)
        kinds.insert(hook.substr(0, hook.find(' ')));
    CHECK(kinds.size() == 13);
    CHECK(direct.rootInstalled == direct.rootHooks.size() &&
          direct.subInstalled == direct.subHooks.size());
    CHECK(captured.rootInstalled == 0 && captured.subInstalled == 0);
}

// Sdf's SetStateDelegate tells the delegate it replaces that its layer is
// gone; after that, while the capture runs, nothing may attach or detach it.
void
HostedDelegateIsNeverDetached()
{
    ++groups;
    CHECK(hosted.rootAttachesAtStart ==
          (Strings{hosted.root->GetIdentifier(), "null"}));
    CHECK(hosted.subAttachesAtStart.size() == 2 &&
          hosted.subAttachesAtStart.back() == "null");
    CHECK(hosted.rootLog &&
          hosted.rootLog->attaches == hosted.rootAttachesAtStart);
    CHECK(captured.subAttaches == hosted.subAttachesAtStart);
}

// The capture holds the only strong reference to what it hosts.
void
HostedDelegateGoesWithItsLayer()
{
    ++groups;
    CHECK(direct.subWentWithItsLayer);
    CHECK(captured.subWentWithItsLayer);
}

void
TeardownReinstallsTheReplacedDelegate()
{
    ++groups;
    const bool dirty = hosted.root->IsDirty();
    CHECK(dirty);
    hosted.rig.reset();
    RigExecFinalizeInputReplay();
    CHECK(hosted.rootLog && get_pointer(hosted.root->GetStateDelegate()) ==
                                get_pointer(hosted.rootLog));
    if (!hosted.rootLog)
        return;
    CHECK(hosted.rootLog->attaches ==
          (Strings{hosted.root->GetIdentifier(), "null",
                   hosted.root->GetIdentifier()}));
    CHECK(hosted.root->IsDirty() == dirty);
    const auto size = fs::file_size(actions);
    const size_t hooks = hosted.rootLog->hooks.size(),
                 installed = hosted.rootLog->installedHooks;
    hosted.root->SetField(SdfPath("/Edits"), SdfFieldKeys->Documentation,
                          VtValue(std::string("after")));
    CHECK(hosted.rootLog->hooks.size() == hooks + 1 &&
          hosted.rootLog->installedHooks == installed + 1);
    CHECK(fs::file_size(actions) == size);
}

void
CaptureOfAHostedLayerReplaysTheSameLayer()
{
    ++groups;
    CHECK(!plain.text.empty());
    CHECK(direct.text == plain.text);
    CHECK(captured.text == plain.text);
    CHECK(UnsetCaptureEnvironment("RIGEXEC_INPUT_REPLAY"));
    const SdfLayerHandle weak(hosted.root);
    hosted.stage.Reset();
    hosted.root.Reset();
    CHECK(!weak);
    CHECK(!hosted.rootLog);
    // Held, so the replay rebuilds this layer and its result can be read.
    SdfLayerRefPtr replayed = SdfLayer::FindOrOpen(rootPath.string());
    std::string error;
    CHECK(RigExecReplayInputActions(actions.string(), &error));
    if (!error.empty())
        std::printf("replay: %s\n", error.c_str());
    CHECK(Text(replayed) == plain.text);
}
} // namespace

int
main()
{
    const char *existing = std::getenv("RIGEXEC_INPUT_REPLAY");
    if (existing && *existing) {
        std::printf("an external input capture is active\n");
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
    RigExecLoadComputations();
    Prepare();
    direct = Uncaptured(true);
    plain = Uncaptured(false);
    CaptureHostsTheDelegateItReplaces();
    if (hosted.started) {
        HostedDelegateIsNeverDetached();
        HostedDelegateGoesWithItsLayer();
        TeardownReinstallsTheReplacedDelegate();
        CaptureOfAHostedLayerReplaysTheSameLayer();
    }
    fs::remove_all(dir);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecInputReplayHosting_OK (%d groups)\n", groups);
    return 0;
}
