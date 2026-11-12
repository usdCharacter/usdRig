// A standalone host that reads the installed rigExecBinary headers without
// OpenUSD must still be able to take the cloth mover type name's extent:
// sizeof, array reference deduction and std::size all report it.
#include "rigExecBinary/clothPlayback.h"

#include <cstdio>
#include <cstring>
#include <iterator>
#include <string_view>

using namespace rigExec;

template <std::size_t N>
constexpr std::size_t
ArrayExtent(const char (&name)[N])
{
    return std::size(name);
}

static_assert(sizeof(RigExecClothMoverType) ==
                  sizeof(SchemaTypeName::RigExecClothMover),
              "RigExecClothMoverType must keep the schema spelling's extent");
static_assert(ArrayExtent(RigExecClothMoverType) ==
                  sizeof(SchemaTypeName::RigExecClothMover),
              "RigExecClothMoverType must be an array of the schema extent");
static_assert(std::string_view(RigExecClothMoverType) ==
                  SchemaTypeName::RigExecClothMover,
              "RigExecClothMoverType must spell the schema cloth mover type");

int
main()
{
    char spelling[sizeof(RigExecClothMoverType)];
    std::memcpy(spelling, RigExecClothMoverType, sizeof(spelling));
    if (spelling[sizeof(spelling) - 1] != '\0' ||
        std::strcmp(spelling, RigExecClothMoverType) != 0) {
        std::fprintf(stderr,
                     "RigExecClothMoverType is not a nul-terminated array\n");
        return 1;
    }
    std::printf("RigExecClothMoverType: %s (%zu bytes)\n", spelling,
                sizeof(spelling));
    return 0;
}
