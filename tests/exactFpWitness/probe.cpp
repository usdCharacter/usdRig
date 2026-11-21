// Effective-mode probe for the exact-FP witness. Prints the bit patterns of
// a reassociation expression and a multiply-add contraction expression. An
// exact-mode compiler yields the two-step patterns; a reassociating or
// contracting compiler yields different last bits, so the printed patterns
// witness the mode the compiler actually used. Operands stay runtime or
// volatile values so no mode constant-folds them.
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern void rigexecPrintGeneratedProbe();

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

    const float a = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 1.5f;
    const float b =
        argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.14999999f;
    const float c =
        argc > 4 ? static_cast<float>(std::atof(argv[4])) : 0.14999999f;
    std::printf("probe contract %08x\n", rigexecBits(a * b + c));

    // The grouped-option witness defines KEEP1 and KEEP2 in one SHELL:
    // entry; each guard checked separately proves both definitions
    // survived the conflict scrub as independent options.
#if defined(KEEP1)
    std::printf("probe keep1 1\n");
#endif
#if defined(KEEP2)
    std::printf("probe keep2 2\n");
#endif

    rigexecPrintGeneratedProbe();
    return 0;
}
