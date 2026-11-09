#
# RigExec usdview plugin: one stage's handle on rigExecImaging.
#
# rigExecImaging keeps one imaging CONTEXT per UsdStage (docs/
# multistage-imaging.md): its own snapshot store, chains, previews, warming
# and published generation. Its C surface is stage-scoped throughout: every
# per-stage entry point names its stage --
#
#   RigExecImaging_<Name>ForStage(stageCacheId, args...)  THAT stage's context
#
# -- and there is no stage-less spelling to fall back to. A usdview session
# must only ever touch its own stage, and several sessions share one
# process in a host, so everything the plugins do goes through an
# ImagingHandle bound to the session's stage.
#
# The handle ALSO answers to the plain C names, as HANDLE ADAPTERS:
#
#   handle.SetTime(frame), handle.OnIdle(), handle.GetGeneration(), ...
#   getattr(handle, "RigExecImaging_SetTime")  -- what cacheStripModel and
#       the strip panel look entry points up by.
#
# Those names are NOT library symbols; each one resolves ONLY to
#
#   <Name>ForStage(handle.CacheId(), ...)
#
# with the stage's cache id prepended exactly once. A name the library does
# not export raises AttributeError from both the method and getattr,
# exactly like a CDLL does, so `getattr(handle, name, None)` and the
# existing `except AttributeError` fallbacks say "absent" for it.
#
# Activation, the control-frame read, the activity probe and the context
# count take the cache id in their own names (RigExecImaging_ActivateFor-
# Stage, GetControlFrameAssetSpace, IsActiveForStage, ContextCount) and
# pass through the handle with their library signatures. Deactivation is
# scoped too: a handle only ever deactivates its own stage
# (DeactivateForStage); a session holding no cached stage deactivates
# nothing.
#
# A library that does not export the required scoped entry points
# (_REQUIRED below) cannot keep this session's calls on this session's
# stage. BindLibrary -- and every ImagingHandle built over it -- fails
# with an AttributeError naming what is missing, so the host reports the
# unsupported library instead of silently acting on another session's
# context.
#
import ctypes

from pxr import UsdUtils


_PREFIX = "RigExecImaging_"
_SUFFIX = "ForStage"

_ll = ctypes.c_longlong
_i = ctypes.c_int
_d = ctypes.c_double
_s = ctypes.c_char_p
_dp = ctypes.POINTER(ctypes.c_double)
_ip = ctypes.POINTER(ctypes.c_int)
_fp = ctypes.POINTER(ctypes.c_float)

# name -> (restype, argtypes) of RigExecImaging_<Name>ForStage AFTER its
# leading stage cache id (long long). Only per-stage names appear here;
# the id-taking symbols bound in _STAGE_ID below carry the id in their
# own signatures.
_PER_STAGE = {
    "SetTime": (_i, [_d]),
    "OnEditCommitted": (_i, []),
    "OnIdle": (_i, []),
    "ServiceFrameDemand": (_i, []),
    "HasFrameDemand": (_i, []),
    "WriteProfileSummary": (_i, [_s]),
    "Deactivate": (None, []),
    "GetGeneration": (_ll, []),
    "GetMovedFloats": (_i, [_s, _fp, _i]),
    "SetWeightOverlay": (_i, [_s]),
    "GetFrameStates": (_i, [_s, _dp, _ip, _i]),
    "ClearFrameCache": (_i, [_s]),
    "WarmRange": (_i, [_s, _dp, _i]),
    "GetWarmingCompletedCount": (_ll, []),
    "GetWarmingProgressCount": (_ll, []),
    "BeginPreview": (_i, [_s]),
    "UpdatePreview": (_i, [_dp, _i]),
    "EndPreview": (_i, []),
    "EndPreviewWithoutPublish": (_i, []),
    "GetGuideBoundsAssetSpace": (_i, [_s, _dp]),
    "GetAllGuideBoundsAssetSpace": (_i, [_dp]),
}

# Entry points that take the stage cache id in their own names.
_STAGE_ID = {
    "ActivateForStage": (_i, [_ll, _s, _d]),
    "GetControlFrameAssetSpace": (_i, [_ll, _s, _d, _i, _dp]),
    "IsActiveForStage": (_i, [_ll]),
    "ContextCount": (_i, []),
}

# The scoped entry points without which no call could be kept on THIS
# stage: activation, the frame change, and shutdown. A library missing
# one is unsupported: BindLibrary raises rather than handing out a
# handle that could reach whichever stage another session named last.
_REQUIRED = ("ActivateForStage", "SetTimeForStage", "DeactivateForStage")

_BOUND_MARK = "_rigExecImagingHandleBound"


def _Bind(function, restype, argtypes):
    try:
        function.restype = restype
        function.argtypes = argtypes
    except (AttributeError, TypeError):
        pass    # a test double: plain Python callables take no signature


def BindLibrary(lib):
    """
    Give every scoped symbol `lib` exports its ctypes signature, once.

    Idempotent and tolerant of OPTIONAL features: a scoped symbol the
    library does not export is skipped, and a double that is not a CDLL
    is left alone. `lib` must export the required scoped entry points
    (_REQUIRED); one that does not raises AttributeError naming what is
    missing, so an unsupported library is reported at load rather than
    silently routed to another stage's context.
    """
    if lib is None:
        return lib
    missing = [name for name in _REQUIRED
               if getattr(lib, _PREFIX + name, None) is None]
    if missing:
        raise AttributeError(
            "rigExecImaging lacks the required stage-scoped entry "
            "points: " + ", ".join(_PREFIX + name for name in missing))
    if getattr(lib, _BOUND_MARK, False):
        return lib
    for name, (restype, argtypes) in _PER_STAGE.items():
        scoped = getattr(lib, _PREFIX + name + _SUFFIX, None)
        if scoped is not None:
            _Bind(scoped, restype, [_ll] + list(argtypes))
    for name, (restype, argtypes) in _STAGE_ID.items():
        function = getattr(lib, _PREFIX + name, None)
        if function is not None:
            _Bind(function, restype, argtypes)
    try:
        setattr(lib, _BOUND_MARK, True)
    except Exception:
        pass
    return lib


def StageCacheId(stage, insert=False):
    """
    `stage`'s UsdUtils.StageCache id as an int, or -1.

    With `insert`, a stage not in the cache is inserted -- which transfers
    ownership to the cache in this binding, so only the container that
    also erases it (rigExecUsdview._ReleaseCachedStage) passes True.
    """
    if not stage:
        return -1
    try:
        cache = UsdUtils.StageCache.Get()
        stageId = cache.GetId(stage)
        if not stageId.IsValid():
            if not insert:
                return -1
            stageId = cache.Insert(stage)
        return stageId.ToLongInt() if stageId.IsValid() else -1
    except Exception:
        return -1


class _Scoped(object):
    """A scoped entry point called with this handle's cache id in front."""

    __slots__ = ("_handle", "_function")

    def __init__(self, handle, function):
        self._handle = handle
        self._function = function

    def __call__(self, *args):
        return self._function(self._handle.CacheId(), *args)


class ImagingHandle(object):
    """
    One stage's view of rigExecImaging.

    `lib` is the loaded library (or a test double carrying the same
    scoped symbols); `stage` the session's UsdStage. With `insert`, the
    stage is put in UsdUtils.StageCache if it is not there yet (see
    StageCacheId); without, a stage that is not cached has id -1 until
    someone caches it, and the id is looked up again on the next call.
    """

    def __init__(self, lib, stage, insert=False):
        self._lib = BindLibrary(lib)
        self._stage = stage
        self._insert = bool(insert)
        self._cacheId = StageCacheId(stage, self._insert)

    def __repr__(self):
        return "<ImagingHandle stage=%s id=%d>" % (
            getattr(self._stage, "GetRootLayer", lambda: None)()
            if self._stage else None,
            self._cacheId)

    # -- identity --------------------------------------------------------

    @property
    def lib(self):
        return self._lib

    @property
    def stage(self):
        return self._stage

    def CacheId(self):
        """The stage's cache id, looked up again while it is still -1."""
        if self._cacheId < 0 and self._stage:
            self._cacheId = StageCacheId(self._stage, self._insert)
        return self._cacheId

    # -- dispatch --------------------------------------------------------

    def _Entry(self, name):
        """The callable for per-stage `name`, legacy call signature;
        raises AttributeError like a CDLL when the scoped spelling is not
        exported."""
        lib = self._lib
        if lib is None:
            raise AttributeError(_PREFIX + name + _SUFFIX)
        return _Scoped(self, getattr(lib, _PREFIX + name + _SUFFIX))

    def Has(self, name):
        """Whether `name` (legacy spelling, prefix optional) is callable."""
        if name.startswith(_PREFIX):
            name = name[len(_PREFIX):]
        try:
            if name in _PER_STAGE:
                self._Entry(name)
                return True
            return getattr(self._lib, _PREFIX + name, None) is not None
        except AttributeError:
            return False

    def __getattr__(self, name):
        # Only reached for names the class does not define: the plain C
        # spellings, which this class ADAPTS -- each resolves to the
        # scoped symbol with this handle's cache id in front, never to a
        # library symbol of that name. Entry points that already take the
        # cache id pass through with their own (identical) signature.
        if name.startswith(_PREFIX):
            short = name[len(_PREFIX):]
            if short in _PER_STAGE:
                return self._Entry(short)
            if short in _STAGE_ID and self._lib is not None:
                return getattr(self._lib, name)
        raise AttributeError(name)

    # -- activation --------------------------------------------------------

    def Activate(self, rigPath=b"", frame=0.0):
        """RigExecImaging_ActivateForStage on this stage. 0 on success."""
        if isinstance(rigPath, str):
            rigPath = rigPath.encode("utf-8")
        activate = getattr(self._lib, _PREFIX + "ActivateForStage")
        return activate(self.CacheId(), rigPath, frame)

    def Deactivate(self):
        """Stop evaluating THIS stage and no other."""
        return self._Entry("Deactivate")()

    def IsActive(self):
        """True/False from the scoped activity probe, None when the
        library cannot say."""
        probe = getattr(self._lib, _PREFIX + "IsActiveForStage", None)
        if probe is None:
            return None
        return bool(probe(self.CacheId()))

    # -- the per-stage surface, by its adapter names -------------------------

    def SetTime(self, frame):
        return self._Entry("SetTime")(frame)

    def OnEditCommitted(self):
        return self._Entry("OnEditCommitted")()

    def OnIdle(self):
        return self._Entry("OnIdle")()

    def ServiceFrameDemand(self):
        """Frames on demand: cached frames Hydra readers asked for become
        frame-table entries (docs/specs/frames-on-demand.md). Returns the
        count added, -1 when inactive."""
        return self._Entry("ServiceFrameDemand")()

    def HasFrameDemand(self):
        """1 while frames on demand wait, 0 when none, -1 when inactive."""
        return self._Entry("HasFrameDemand")()

    def WriteProfileSummary(self, path):
        if isinstance(path, str):
            path = path.encode("utf-8")
        return self._Entry("WriteProfileSummary")(path)

    def GetGeneration(self):
        return self._Entry("GetGeneration")()

    def GetMovedFloats(self, packedPaths, out, count):
        return self._Entry("GetMovedFloats")(packedPaths, out, count)

    def SetWeightOverlay(self, primPath):
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        return self._Entry("SetWeightOverlay")(primPath)

    def GetFrameStates(self, rigPath, frames, statesOut, count):
        return self._Entry("GetFrameStates")(rigPath, frames, statesOut,
                                             count)

    def ClearFrameCache(self, rigPath):
        return self._Entry("ClearFrameCache")(rigPath)

    def WarmRange(self, rigPath, frames, count):
        return self._Entry("WarmRange")(rigPath, frames, count)

    def GetWarmingCompletedCount(self):
        return self._Entry("GetWarmingCompletedCount")()

    def BeginPreview(self, packedPaths):
        if isinstance(packedPaths, str):
            packedPaths = packedPaths.encode("utf-8")
        return self._Entry("BeginPreview")(packedPaths)

    def UpdatePreview(self, values, count):
        return self._Entry("UpdatePreview")(values, count)

    def EndPreview(self):
        return self._Entry("EndPreview")()

    def GetGuideBoundsAssetSpace(self, primPath, outMinMax):
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        return self._Entry("GetGuideBoundsAssetSpace")(primPath, outMinMax)

    def GetAllGuideBoundsAssetSpace(self, outMinMax):
        return self._Entry("GetAllGuideBoundsAssetSpace")(outMinMax)

    def GetControlFrameAssetSpace(self, primPath, frame, isDefault,
                                  outMatrix):
        """Already stage-scoped in every library: it takes the cache id."""
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        read = getattr(self._lib, _PREFIX + "GetControlFrameAssetSpace")
        return read(self.CacheId(), primPath, frame, isDefault, outMatrix)


def ContextCount(lib):
    """RigExecImaging_ContextCount(), the directory's context count."""
    count = getattr(lib, _PREFIX + "ContextCount", None)
    return None if count is None else int(count())
