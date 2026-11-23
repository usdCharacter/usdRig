// The patch mover's declared inputs, in declaration order: the native
// assembly and the scene's op binding read them by position.
#ifndef RIGEXEC_SCENE_PATCH_INPUTS_H
#define RIGEXEC_SCENE_PATCH_INPUTS_H

#include "rigExecScene/geometryProgram.h"

#include <cstddef>
#include <vector>

namespace rigExec {
namespace patchInputs {

// The patch's ten, the neutral UV points, one points array per
// rigExec:uvShapes target, then the shape weights -- one per shape, read
// from the sources inputs:uvWeights connects to, in order, or, with no
// connection, the authored array itself -- and last the two contact index
// lists, rigExec:uvContactLower and rigExec:uvContactUpper.
enum Input : size_t {
    PatchPoints = 0,
    PatchPointWeights,
    PatchUVertexCount,
    PatchVVertexCount,
    PatchUOrder,
    PatchVOrder,
    PatchUKnots,
    PatchVKnots,
    PatchUForm,
    PatchVForm,
    UvPoints,
    UvShapes
};

// How many shapes the declared inputs hold, and whether their weights are
// connected sources (one Dial each) or the authored array.
struct Layout {
    size_t shapes = 0;
    bool connected = false;
};

inline size_t
WeightsBegin(const Layout &layout)
{
    return UvShapes + layout.shapes;
}

inline size_t
ContactsBegin(const Layout &layout)
{
    return WeightsBegin(layout) + (layout.connected ? layout.shapes : 1);
}

inline size_t
InputCount(const Layout &layout)
{
    return ContactsBegin(layout) + 2;
}

inline Layout
LayoutOf(const std::vector<RigExecRevisionLeafKey> &keys)
{
    Layout layout;
    size_t k = UvShapes;
    while (k < keys.size() &&
           keys[k].type == RigExecRevisionLeafType::Vec3fArray) {
        ++layout.shapes;
        ++k;
    }
    layout.connected =
        k < keys.size() && keys[k].type == RigExecRevisionLeafType::Dial;
    return layout;
}

} // namespace patchInputs
} // namespace rigExec

#endif
