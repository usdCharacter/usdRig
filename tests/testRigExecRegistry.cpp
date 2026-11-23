// Registry freeze, handles and errors with fake type and op libraries.
#include "registryFixture/fakeOps.h"
#include "registryFixture/fakeTypes.h"
#include "rigExec/api/registry.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

bool
Mentions(const std::vector<std::string> &errors, const char *text)
{
    for (const std::string &e : errors) {
        if (e.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// An op whose port names a type no library registers.
RIGEXEC_DEFINE_OP_HANDLE(BadOp)
RIGEXEC_DEFINE_PORT_HANDLE(BadPort)

bool
NoRun(const void *, void *, const RigExecOpIo &, std::string *)
{
    return true;
}

void
RegisterBad(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&BadPort(), &fakeTypes::Unregistered(), kRigExecPortInput, "x"}};
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Bad";
        d.ports = ports;
        d.portCount = 1;
        d.run = &NoRun;
        return d;
    }();
    registrar.Add(BadOp(), desc);
}

// An op whose output claims to be an array.
RIGEXEC_DEFINE_OP_HANDLE(ArrayOutOp)
RIGEXEC_DEFINE_PORT_HANDLE(ArrayOutPort)

void
RegisterArrayOut(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&ArrayOutPort(), &fakeTypes::Float(),
         kRigExecPortOutput | kRigExecPortArray, "list"}};
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "ArrayOut";
        d.ports = ports;
        d.portCount = 1;
        d.run = &NoRun;
        return d;
    }();
    registrar.Add(ArrayOutOp(), desc);
}

// Windowed threading without windowFrames, and windowFrames without it.
RIGEXEC_DEFINE_OP_HANDLE(UnframedOp)
RIGEXEC_DEFINE_PORT_HANDLE(UnframedPort)
RIGEXEC_DEFINE_OP_HANDLE(FramedOp)
RIGEXEC_DEFINE_PORT_HANDLE(FramedPort)

uint32_t
TwoFrames(const void *)
{
    return 2;
}

void
RegisterWindowMismatch(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc unframedPorts[] = {
        {&UnframedPort(), &fakeTypes::Float(), kRigExecPortOutput, "out"}};
    static const RigExecPortDesc framedPorts[] = {
        {&FramedPort(), &fakeTypes::Float(), kRigExecPortOutput, "out"}};
    static const RigExecOpDesc unframed = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Unframed";
        d.ports = unframedPorts;
        d.portCount = 1;
        d.run = &NoRun;
        d.threading = RigExecOpThreading::Windowed;
        return d;
    }();
    static const RigExecOpDesc framed = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Framed";
        d.ports = framedPorts;
        d.portCount = 1;
        d.run = &NoRun;
        d.windowFrames = &TwoFrames;
        return d;
    }();
    registrar.Add(UnframedOp(), unframed);
    registrar.Add(FramedOp(), framed);
}

// A Bakeable op whose payload cannot be saved.
RIGEXEC_DEFINE_OP_HANDLE(UnsavedOp)
RIGEXEC_DEFINE_PORT_HANDLE(UnsavedPort)

struct UnsavedPayload {
    float scale = 1.0f;
};

void
RegisterUnsaved(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&UnsavedPort(), &fakeTypes::Float(), kRigExecPortOutput, "out"}};
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Unsaved";
        d.ports = ports;
        d.portCount = 1;
        d.run = &NoRun;
        d.capabilities = kRigExecOpBakeable;
        d.payload = RigExecPayloadHandlingFor<UnsavedPayload>();
        return d;
    }();
    registrar.Add(UnsavedOp(), desc);
}

// An op with neither a run nor a batch function, and one with only a
// batch function.
RIGEXEC_DEFINE_OP_HANDLE(IdleOp)
RIGEXEC_DEFINE_PORT_HANDLE(IdlePort)
RIGEXEC_DEFINE_OP_HANDLE(BatchOnlyOp)
RIGEXEC_DEFINE_PORT_HANDLE(BatchOnlyPort)

bool
NoBatch(const RigExecOpBatch &, size_t *, std::string *)
{
    return true;
}

RigExecOpDesc
FloatOutDesc(const char *name, const RigExecPortDesc *ports)
{
    RigExecOpDesc d;
    d.nameSpace = "test";
    d.name = name;
    d.ports = ports;
    d.portCount = 1;
    return d;
}

void
RegisterIdle(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&IdlePort(), &fakeTypes::Float(), kRigExecPortOutput, "out"}};
    static const RigExecOpDesc desc = FloatOutDesc("Idle", ports);
    registrar.Add(IdleOp(), desc);
}

void
RegisterBatchOnly(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&BatchOnlyPort(), &fakeTypes::Float(), kRigExecPortOutput, "out"}};
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d = FloatOutDesc("BatchOnly", ports);
        d.runBatch = &NoBatch;
        return d;
    }();
    registrar.Add(BatchOnlyOp(), desc);
}

void
TestValidateErrors(const RigExecRegistryInit &good)
{
    {
        RigExecOpLibraryDesc bad;
        bad.name = "idle";
        bad.registerOps = &RegisterIdle;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&bad);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "op test:Idle: no run or batch function"));
    }
    {
        RigExecOpLibraryDesc batchOnly;
        batchOnly.name = "batchOnly";
        batchOnly.registerOps = &RegisterBatchOnly;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&batchOnly);
        std::vector<std::string> errors;
        CHECK(RigExecRegistry::Validate(init, &errors));
        CHECK(errors.empty());
    }
    {
        RigExecOpLibraryDesc bad;
        bad.name = "unsaved";
        bad.registerOps = &RegisterUnsaved;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&bad);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "payload needs encode and decode"));
    }
    {
        RigExecOpLibraryDesc bad;
        bad.name = "windowMismatch";
        bad.registerOps = &RegisterWindowMismatch;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&bad);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "op test:Unframed: Windowed threading"));
        CHECK(Mentions(errors, "op test:Framed: Windowed threading"));
    }
    {
        RigExecOpLibraryDesc bad;
        bad.name = "arrayOut";
        bad.registerOps = &RegisterArrayOut;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&bad);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "port list is an array output"));
    }
    {
        RigExecOpLibraryDesc bad;
        bad.name = "bad";
        bad.registerOps = &RegisterBad;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&bad);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "port x uses a type that is not registered"));
    }
    {
        RigExecRegistryInit init = good;
        init.typeLibraries.push_back(good.typeLibraries.front());
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "listed twice"));
    }
    {
        RigExecTypeLibraryDesc old = *good.typeLibraries.front();
        old.abiVersion = kRigExecRegistryAbiVersion + 1;
        RigExecRegistryInit init;
        init.typeLibraries.push_back(&old);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "ABI version"));
    }
    {
        // Ops without their type library.
        RigExecRegistryInit init;
        init.opLibraries = good.opLibraries;
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
    }
    // Validate never writes handles.
    CHECK(!fakeTypes::Float().IsValid());
    CHECK(!RigExecRegistry::IsInitialized());
}

void
TestInitialize(const RigExecRegistryInit &init)
{
    std::vector<std::string> errors;
    CHECK(RigExecRegistry::Initialize(init, &errors));
    for (const std::string &e : errors) {
        std::printf("  %s\n", e.c_str());
    }
    CHECK(RigExecRegistry::IsInitialized());
    const RigExecRegistry &r = RigExecRegistry::Get();
    CHECK(r.TypeCount() == 2);
    CHECK(r.OpCount() == 2);
    CHECK(fakeTypes::Float().Index() == 0);
    CHECK(fakeTypes::Vec3Type().Index() == 1);
    CHECK(!fakeTypes::Unregistered().IsValid());
    CHECK(r.Type(fakeTypes::Vec3Type()).desc->size == sizeof(fakeTypes::Vec3));
    CHECK(fakeOps::ScaleIn().Index() == 0);
    CHECK(fakeOps::ScaleOut().Index() == 1);
    const RigExecRegisteredOp &offset = r.Op(fakeOps::Offset());
    CHECK(offset.portTypes.size() == 2 &&
          offset.portTypes[0] == fakeTypes::Vec3Type().Index());
    CHECK(r.Fingerprint() != 0);

    // Run the two ops through their registered descriptors.
    const RigExecRegisteredOp &scale = r.Op(fakeOps::Scale());
    fakeOps::ScalePayload payload;
    payload.scale = 2.0f;
    const fakeTypes::Vec3 in{1.0f, 2.0f, 3.0f};
    fakeTypes::Vec3 mid{}, out{};
    const void *scaleIn[2] = {&in, nullptr};
    void *scaleOut[2] = {nullptr, &mid};
    CHECK(scale.desc->run(&payload, nullptr, {scaleIn, scaleOut, 2}, nullptr));
    const void *offsetIn[2] = {&mid, nullptr};
    void *offsetOut[2] = {nullptr, &out};
    CHECK(
        offset.desc->run(nullptr, nullptr, {offsetIn, offsetOut, 2}, nullptr));
    CHECK(out.x == 3.0f && out.y == 5.0f && out.z == 7.0f);

    // An op with only run is driven per instance by the library's batch
    // loop, which skips inactive instances.
    fakeTypes::Vec3 first{}, second{};
    const void *batchIn[4] = {&in, nullptr, &mid, nullptr};
    void *batchOut[4] = {nullptr, &first, nullptr, &second};
    const char active[2] = {1, 0};
    RigExecOpBatch batch;
    batch.op = offset.desc;
    batch.count = 2;
    batch.active = active;
    batch.portCount = 2;
    batch.inputs = batchIn;
    batch.outputs = batchOut;
    size_t failed = 0;
    CHECK(offset.runBatch && offset.runBatch(batch, &failed, nullptr));
    CHECK(first.x == 2.0f && first.y == 3.0f && first.z == 4.0f);
    CHECK(second.x == 0.0f && second.y == 0.0f && second.z == 0.0f);

    std::vector<std::string> again;
    CHECK(!RigExecRegistry::Initialize(init, &again));
    CHECK(Mentions(again, "called twice"));
}

} // namespace

int
main()
{
    const RigExecRegistryInit init = RigExecLinkedRegistrations();
    CHECK(init.typeLibraries.size() == 1);
    CHECK(init.opLibraries.size() == 1);
    TestValidateErrors(init);
    TestInitialize(init);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecRegistry: ok\n");
    return 0;
}
