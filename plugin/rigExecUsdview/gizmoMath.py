#
# RigExec usdview gizmo math: the compatibility face of the three owner
# modules -- gizmoFrames (rig root, pose providers, rest/default/posed
# frames), gizmoRotation (Euler composition, decomposition and world-axis
# solving) and gizmoTargets (edit targets, group deltas and the writer).
# Every name this file used to define is still importable from here; the
# implementations live in the owners.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): the headless suite
# tests/python/test_gizmo_math.py imports this facade, so it exercises all
# three owners through it.
#
# The names below are aliases, not uses: this facade re-exports the whole
# top-level surface the single-file module had, including the imports it
# carried (math, OrderedDict, the pxr names, sessionRegistry), so every
# previously importable name still resolves here to the same object.
import math
from collections import OrderedDict

from pxr import Gf, Sdf, Tf, Usd, UsdGeom

try:
    import gizmoFrames
    import gizmoRotation
    import gizmoTargets
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoFrames
    import gizmoRotation
    import gizmoTargets
    import sessionRegistry

from gizmoFrames import (
    AVAR_T, AVAR_R, AVAR_S, AVAR_RSPIN, AVAR_ORDER, AVAR_ROTATION_SIGN,
    REST_T, REST_R, REST_SPACE, POSED_SPACE, DEFAULT_T, DEFAULT_R,
    DEFAULT_SPACE, AVAR_DEFAULT_SPACE, POSED_DEFAULT_SPACE, PARENT_SPACE,
    PARENT_DEFAULT_SPACE, AVAR_UNIT_SCALE, _COMPUTED_SPACES,
    VOLUME_WEIGHT_TYPE_NAMES, NormalizeAvarScale, ComposeAvarMatrix,
    RotationSign, IsRigXformable, ReadsScaleAvars, _EvalContext, _openScopes,
    _AmbientScope, _Context, MemoScope, _MemoMatrix, FindRigRoot,
    _FindParentXformable, SOLVER_JOINTS_REL, MOVER_MOVES_REL,
    SPACE_TARGET_REL, RIG_WRITTEN_RELS, OVERWRITING_MOVER_TYPES,
    SPACE_SWITCH_TYPES, _PosedPaths, SolverPosedPaths, PresentedBy,
    SolverPosedCache, NoticeAffectsTarget, _previewValues, _PreviewBucket,
    _DropFromPreview, SetPreviewValues, SetPreviewValue, PreviewValues,
    _Previewed, _ScalarAvarValue, ScalarAvar, _MatrixAttr, RestLocal,
    InterveningXform, RestSpace, DefaultLocal, _PoseAuthorityError,
    _ComputedSpace, _ComputeSpace, AvarsMatrix, RigFrames, _RefuseBothModes,
    _publishedControlFrameReaders, SetPublishedControlFrameReader,
    _HasPublishedControlFrameReader, _ReadPublishedControlFrame,
    ComputeRigFrames, _PreferPublishedFrame, _ApplyChannelSpaces,
    _ComputeRigFrames, _PoseProvider, PoseProviderChain, PoseProviderPaths
)
from gizmoRotation import (
    ROTATION_ORDERS, _AXES, _CYCLIC, _IDENTITY, _NormalizeOrder,
    _AxisRotation, RotationFromEuler, _Unwrap, DecomposeEuler, _Linear,
    _RotationOnly, _WorldRotation, _RotationVector, _FromRotationVector,
    _ROTATION_SOLVE_STEPS, _ROTATION_SOLVE_TOLERANCE, _ROTATION_SOLVE_STEP,
    SolveWorldRotation, GimbalAxes
)
from gizmoTargets import (
    WRITE_ANIMATION, WRITE_DEFAULT, CHANNELS_POSE, CHANNELS_PIVOT,
    GROUP_PIVOT_CENTER, GROUP_PIVOT_LEAD, GROUP_PIVOT_INDIVIDUAL,
    GROUP_PIVOT_MODES, SetAnimated, AnimatedWrites, _AdoptionWrites, Writer,
    _FrameAt, _SnapValue, _SnapTranslation, _ScaleAxes, Target, _RigTarget,
    RigPoseTarget, RigPivotTarget, _XFORM_ORDER_NAMES, _ZERO3F,
    _XformCommonMatrix, _PreservedChild, _XformTarget, XformPoseTarget,
    XformPivotTarget, _ConnectedAvar, RIG_TARGET_TYPE_NAMES,
    _IsRigTargetType, MakeTarget, _GROUP_EPSILON_DEGREES,
    _GROUP_EPSILON_LENGTH, _IsIdentity, _Centroid, GroupTarget,
    MakeGroupTarget
)
# The owners rebind these at call time, so a one-time alias would freeze the
# import-time value; resolve them from the owner module instead.
_LATE_BOUND = {"_keylessControlFrameReader": gizmoFrames}


def __getattr__(name):
    if name in _LATE_BOUND:
        return getattr(_LATE_BOUND[name], name)
    raise AttributeError("module 'gizmoMath' has no attribute %r" % name)


def __dir__():
    return sorted(set(globals()) | set(_LATE_BOUND))
