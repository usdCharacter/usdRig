// Recomputed-normals layout: the authored normals' interpolation and the
// mesh orientation applied to per-point normals. Header-only and USD-free;
// the binary runtime instantiates it over its own vector type.
#ifndef RIGEXEC_MATH_NORMALS_LAYOUT_KERNEL_H
#define RIGEXEC_MATH_NORMALS_LAYOUT_KERNEL_H

#include <cstddef>
#include <vector>

namespace rigExec {

/// Lays \p normals (one per point) out as the authored normals store them:
/// one per face corner through \p cornerPoints (faceVertexIndices) when
/// \p perCorner, and negated on a leftHanded mesh. False, leaving
/// \p normals unchanged, when a corner names no point.
template <class Vec3>
bool
RigExecLayoutNormals(std::vector<Vec3> *normals, const int *cornerPoints,
                     size_t cornerCount, bool perCorner, bool leftHanded)
{
    if (perCorner) {
        if (cornerCount > 0 && !cornerPoints)
            return false;
        std::vector<Vec3> corners(cornerCount);
        for (size_t c = 0; c < cornerCount; ++c) {
            const int v = cornerPoints[c];
            if (v < 0 || static_cast<size_t>(v) >= normals->size())
                return false;
            corners[c] = (*normals)[static_cast<size_t>(v)];
        }
        normals->swap(corners);
    }
    if (leftHanded)
        for (Vec3 &n : *normals)
            n = -n;
    return true;
}

} // namespace rigExec

#endif
