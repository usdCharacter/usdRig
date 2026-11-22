// Reassociation probe with its own main, compiled only by the computed
// per-source-option witnesses: a rejection must name a consuming target,
// so the poisoned source cannot belong to an earlier target.
#include <cstdio>
#include <cstdlib>
#include <cstring>

static unsigned
rigexecBits(float v)
{
    unsigned u = 0;
    std::memcpy(&u, &v, sizeof(u));
    return u;
}

int
main(int argc, char **argv)
{
    const float x = 1e8f;
    const float y = argc > 1 ? static_cast<float>(std::atof(argv[1])) : 1e-8f;
    std::printf("probe reassoc %08x\n", rigexecBits(x + y - x));
    return 0;
}
