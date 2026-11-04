"""What rigExecImaging's displayed generation publishes, for tools and tests
without a Hydra chain.

Other frames come through Hydra (docs/specs/frames-on-demand.md): a host
samples a rig output at a time offset and the rig answers from its frame
table, never on the reader's thread. Without a chain, set the context's time
first (RigExecImaging_SetTimeForStage) and read what it published here. Qt-
free ctypes over the C surface (libs/rigExecImaging/registry.h); nothing
here evaluates, authors or publishes.

The library is the one RIGEXEC_IMAGING_DLL names, else the one beside this
package (installed: <prefix>/lib; build tree: <build>), else the source
tree's build/ directory. It must be the copy the host activated the rig
through, or every read answers NO_CONTEXT.
"""

import ctypes
import os
import sys

OK = 0
NOT_DRIVEN = -1
NO_CONTEXT = -2

_libraries = {}


def _library_file_name():
    if os.name == "nt":
        return "rigExecImaging.dll"
    if sys.platform == "darwin":
        return "librigExecImaging.dylib"
    return "librigExecImaging.so"


def imaging_library_path():
    """Where rigExecImaging is: RIGEXEC_IMAGING_DLL, else beside this
    package's parent (installed <prefix>/lib/python/rigexec -> <prefix>/lib,
    build tree <build>/python/rigexec -> <build>), else <repo>/build."""
    explicit = os.environ.get("RIGEXEC_IMAGING_DLL")
    if explicit:
        return explicit
    package_dir = os.path.dirname(os.path.abspath(__file__))
    name = _library_file_name()
    candidates = [
        os.path.normpath(os.path.join(package_dir, "..", "..", name)),
        os.path.normpath(os.path.join(package_dir, "..", "..", "build", name)),
    ]
    return next((path for path in candidates if os.path.isfile(path)),
                candidates[0])


def _load(library):
    """`library` is a path, an already loaded ctypes.CDLL, or None for
    imaging_library_path()."""
    if library is not None and not isinstance(library, (str, bytes, os.PathLike)):
        lib = library
    else:
        path = os.fspath(library) if library is not None else imaging_library_path()
        lib = _libraries.get(path)
        if lib is None:
            lib = ctypes.CDLL(path)
            _libraries[path] = lib
    points = lib.RigExecImaging_GetPublishedPointsForStage
    if points.argtypes is None:
        points.argtypes = [ctypes.c_longlong, ctypes.c_char_p,
                           ctypes.POINTER(ctypes.c_float), ctypes.c_int]
        points.restype = ctypes.c_int
        scalar = lib.RigExecImaging_GetPublishedScalarPrimvarForStage
        scalar.argtypes = [ctypes.c_longlong, ctypes.c_char_p, ctypes.c_char_p,
                           ctypes.POINTER(ctypes.c_float)]
        scalar.restype = ctypes.c_int
        revision = lib.RigExecImaging_GetPublishedXformRevisionForStage
        revision.argtypes = [ctypes.c_longlong, ctypes.c_char_p,
                             ctypes.POINTER(ctypes.c_double)]
        revision.restype = ctypes.c_int
    return lib


def stage_cache_id(stage, insert=True):
    """The stage's UsdUtils.StageCache id as an int, or None when the cache
    does not hold it and insert is False. With insert (the default), an
    uncached stage is inserted: the caller who inserts owns the later Erase.
    The reads below pass insert=False and never insert."""
    from pxr import UsdUtils
    cache = UsdUtils.StageCache.Get()
    stage_id = cache.GetId(stage)
    if not stage_id.IsValid():
        if not insert:
            return None
        stage_id = cache.Insert(stage)
    return stage_id.ToLongInt()


def published_points(stage, prim_path, library=None):
    """(OK, Vt.Vec3fArray) of the displayed generation, (NOT_DRIVEN, None)
    when the prim publishes no points, (NO_CONTEXT, None) without a context."""
    from pxr import Vt
    sid = stage_cache_id(stage, insert=False)
    if sid is None:
        return NO_CONTEXT, None
    lib = _load(library)
    path = str(prim_path).encode("utf-8")
    count = lib.RigExecImaging_GetPublishedPointsForStage(sid, path, None, 0)
    if count < 0:
        return (NO_CONTEXT if count == NO_CONTEXT else NOT_DRIVEN), None
    buffer = (ctypes.c_float * (3 * max(count, 1)))()
    again = lib.RigExecImaging_GetPublishedPointsForStage(sid, path, buffer, count)
    if again != count:
        return NOT_DRIVEN, None
    return OK, Vt.Vec3fArray([(buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2])
                              for i in range(count)])


def published_scalar_primvar(stage, prim_path, name, library=None):
    """(OK, float) for a compiled scalar primvar ("opacity" or
    "primvars:opacity"), (NOT_DRIVEN, None) when not published,
    (NO_CONTEXT, None) without a context."""
    sid = stage_cache_id(stage, insert=False)
    if sid is None:
        return NO_CONTEXT, None
    if name.startswith("primvars:"):
        name = name[len("primvars:"):]
    value = ctypes.c_float()
    rc = _load(library).RigExecImaging_GetPublishedScalarPrimvarForStage(
        sid, str(prim_path).encode("utf-8"), name.encode("utf-8"),
        ctypes.byref(value))
    if rc == 1:
        return OK, value.value
    return (NO_CONTEXT if rc == NO_CONTEXT else NOT_DRIVEN), None


def published_xform_revision(stage, prim_path, library=None):
    """(OK, Gf.Matrix4d) revised * base^-1, the asset-space map the rig
    applies to the prim's world; (NOT_DRIVEN, None) when it drives no
    transform there; (NO_CONTEXT, None) without a context."""
    from pxr import Gf
    sid = stage_cache_id(stage, insert=False)
    if sid is None:
        return NO_CONTEXT, None
    values = (ctypes.c_double * 16)()
    rc = _load(library).RigExecImaging_GetPublishedXformRevisionForStage(
        sid, str(prim_path).encode("utf-8"), values)
    if rc == 1:
        return OK, Gf.Matrix4d(*[tuple(values[r * 4:(r + 1) * 4]) for r in range(4)])
    return (NO_CONTEXT if rc == NO_CONTEXT else NOT_DRIVEN), None
