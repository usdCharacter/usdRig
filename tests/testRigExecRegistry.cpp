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

// Two ops ordered after each other.
RIGEXEC_DEFINE_OP_HANDLE(CycleA)
RIGEXEC_DEFINE_OP_HANDLE(CycleB)

void
RegisterCycle(RigExecOpRegistrar &registrar)
{
    static const RigExecOpHandle *const afterB[] = {&CycleB()};
    static const RigExecOpHandle *const afterA[] = {&CycleA()};
    static const RigExecOpDesc a = [] {
        RigExecOpDesc d;
        d.name = "CycleA";
        d.after = afterB;
        d.afterCount = 1;
        d.run = &NoRun;
        return d;
    }();
    static const RigExecOpDesc b = [] {
        RigExecOpDesc d;
        d.name = "CycleB";
        d.after = afterA;
        d.afterCount = 1;
        d.run = &NoRun;
        return d;
    }();
    registrar.Add(CycleA(), a);
    registrar.Add(CycleB(), b);
}

void
TestValidateErrors(const RigExecRegistryInit &good)
{
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
        RigExecOpLibraryDesc cycle;
        cycle.name = "cycle";
        cycle.registerOps = &RegisterCycle;
        RigExecRegistryInit init = good;
        init.opLibraries.push_back(&cycle);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        CHECK(Mentions(errors, "cycle"));
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
    CHECK(offset.after.size() == 1 &&
          offset.after[0] == fakeOps::Scale().Index());
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
