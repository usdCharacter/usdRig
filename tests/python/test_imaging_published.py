"""rigexec.imaging through the C surface of rigExecImaging: what the
displayed generation publishes for a prim -- points, a scalar primvar, the
driven transform revision -- after SetTime at each frame; the reads leave the
generation and the layers alone; status codes for an undriven prim, an
unpublished primvar and a stage without a context. (Other frames come
through Hydra: tests/testRigExecFrameDemand.cpp.)

argv[1] is the generated rigExecSchema resources directory of the build
whose rigExecImaging the test loads (RIGEXEC_IMAGING_DLL, else the build
tree next to it).
"""

import ctypes
import os
import sys

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Gf, Sdf, Usd, UsdGeom, UsdUtils, Vt
import rigexec
from rigexec import imaging

RIG = "/Char/Rig"
B1 = "/Char/Rig/Bones/B1"
B2 = "/Char/Rig/Bones/B2"
STROKE = "/Char/Art/Stroke"
HEAD = "/Char/Art/Head"
STATIC = "/Char/Art/Static"
COUNT = 6


def _translate(x, y):
    return Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(x, y, 0.0))


def _mover(stage, path, type_name, moves):
    prim = stage.DefinePrim(path, type_name)
    prim.ApplyAPI("RigExecMoverAPI")
    prim.GetRelationship("rigExec:moves").SetTargets([moves])
    return prim


def make_rig():
    """Two bones in the XY plane, a stroke skinned to both, a group bound to
    the second bone (transform-domain matrix mover) whose opacity follows
    the first bone's angle (animated map)."""
    stage = Usd.Stage.CreateInMemory()
    character = UsdGeom.Xform.Define(stage, "/Char")
    character.AddTranslateOp().Set(Gf.Vec3d(5.0, 2.0, 0.0))
    stage.DefinePrim(RIG, "RigExecRoot")
    stage.DefinePrim("/Char/Rig/Bones", "Scope")
    b1 = stage.DefinePrim(B1, "RigExecControl")
    b2 = stage.DefinePrim(B2, "RigExecControl")
    b1.GetAttribute("rest:space").Set(_translate(-0.5, -0.2))
    b2.GetAttribute("rest:space").Set(_translate(0.5, 0.0))
    for t in range(1, 5):
        b1.GetAttribute("avars:rz").Set(30.0 * (t - 1), t)
        b2.GetAttribute("avars:rz").Set(-15.0 * (t - 1), t)
        b2.GetAttribute("avars:tx").Set(0.125 * (t - 1), t)

    UsdGeom.Xform.Define(stage, "/Char/Art")
    stroke = UsdGeom.BasisCurves.Define(stage, STROKE)
    stroke.CreatePointsAttr().Set(Vt.Vec3fArray(
        [(-0.5 + 0.25 * i, 0.0625 * i, 0.0) for i in range(COUNT)]))
    stroke.CreateCurveVertexCountsAttr().Set([COUNT])
    stroke.CreateTypeAttr().Set("linear")
    head = UsdGeom.Xform.Define(stage, HEAD)
    head.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.5, 0.0))
    still = UsdGeom.BasisCurves.Define(stage, STATIC)
    still.CreatePointsAttr().Set([(1, 1, 0), (2, 1, 0)])
    still.CreateCurveVertexCountsAttr().Set([2])

    stage.DefinePrim("/Char/Rig/Movers", "Scope")
    skin = _mover(stage, "/Char/Rig/Movers/StrokeSkin", "RigExecSkinMover",
                  STROKE + ".points")
    skin.CreateRelationship("rigExec:influences").SetTargets([B1, B2])
    skin.CreateAttribute("rigExec:elementSize", Sdf.ValueTypeNames.Int).Set(2)
    indices, weights = [], []
    for i in range(COUNT):
        w = i / float(COUNT - 1)
        indices += [0, 1]
        weights += [1.0 - w, w]
    skin.CreateAttribute("rigExec:jointIndices",
                         Sdf.ValueTypeNames.IntArray).Set(indices)
    skin.CreateAttribute("rigExec:jointWeights",
                         Sdf.ValueTypeNames.FloatArray).Set(weights)
    bind = _mover(stage, "/Char/Rig/Movers/HeadBind", "RigExecMatrixMover",
                  HEAD)
    bind.CreateRelationship("rigExec:transform").SetTargets([B2])

    stage.DefinePrim("/Char/Rig/AnimatedMaps", "Scope")
    maps = stage.DefinePrim("/Char/Rig/AnimatedMaps/Fade",
                            "RigExecAnimatedMaps")
    maps.CreateRelationship("rigExec:driverAttributes").SetTargets(
        [B1 + ".avars:rz"])
    maps.CreateRelationship("rigExec:meshes").SetTargets([HEAD])
    fade = stage.DefinePrim("/Char/Rig/AnimatedMaps/Fade/opacity",
                            "RigExecAnimatedMap")
    fade.GetAttribute("rigExec:rowInputs").Set([0])
    fade.GetAttribute("rigExec:rowFrom").Set([0.0])
    fade.GetAttribute("rigExec:rowTo").Set([90.0])
    fade.GetAttribute("rigExec:rowSlope").Set([1.0 / 90.0])
    fade.GetAttribute("rigExec:rowCut").Set([0.0])
    return stage


def _imaging():
    lib = ctypes.CDLL(imaging.imaging_library_path())
    lib.RigExecImaging_ActivateForStage.argtypes = [
        ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double]
    lib.RigExecImaging_ActivateForStage.restype = ctypes.c_int
    lib.RigExecImaging_SetTimeForStage.argtypes = [
        ctypes.c_longlong, ctypes.c_double]
    lib.RigExecImaging_SetTimeForStage.restype = ctypes.c_int
    lib.RigExecImaging_GetGenerationForStage.argtypes = [ctypes.c_longlong]
    lib.RigExecImaging_GetGenerationForStage.restype = ctypes.c_longlong
    lib.RigExecImaging_DeactivateForStage.argtypes = [ctypes.c_longlong]
    lib.RigExecImaging_DeactivateForStage.restype = None
    return lib


def _same_points(a, b):
    return len(a) == len(b) and all(
        tuple(p) == tuple(q) for p, q in zip(a, b))


def _layers(stage):
    return {layer.identifier: (layer.dirty, layer.ExportToString())
            for layer in stage.GetUsedLayers()}


def main():
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    lib = _imaging()
    stage = make_rig()
    cache = UsdUtils.StageCache.Get()
    assert not cache.GetId(stage).IsValid()
    assert imaging.published_points(stage, STROKE) == (imaging.NO_CONTEXT, None)
    assert imaging.published_xform_revision(stage, HEAD) == (imaging.NO_CONTEXT, None)
    assert imaging.published_scalar_primvar(stage, HEAD, "opacity") == (imaging.NO_CONTEXT, None)
    assert imaging.stage_cache_id(stage, insert=False) is None
    stage_id = imaging.stage_cache_id(stage)
    assert cache.GetId(stage).IsValid()
    assert lib.RigExecImaging_ActivateForStage(stage_id, RIG.encode(), 1.0) == 0

    shown = {}
    for t in range(1, 5):
        assert lib.RigExecImaging_SetTimeForStage(stage_id, float(t)) == 0
        generation = lib.RigExecImaging_GetGenerationForStage(stage_id)
        layers = _layers(stage)
        status, points = imaging.published_points(stage, STROKE)
        assert status == imaging.OK, (t, status)
        assert isinstance(points, Vt.Vec3fArray) and len(points) == COUNT
        xform_status, revision = imaging.published_xform_revision(stage, HEAD)
        assert xform_status == imaging.OK, (t, xform_status)
        scalar_status, opacity = imaging.published_scalar_primvar(
            stage, HEAD, "primvars:opacity")
        assert scalar_status == imaging.OK, (t, scalar_status)
        assert lib.RigExecImaging_GetGenerationForStage(stage_id) == generation
        assert _layers(stage) == layers
        shown[t] = (points, revision, opacity)
    assert not _same_points(shown[1][0], shown[3][0])
    assert shown[1][1] != shown[3][1]
    assert abs(shown[3][2] - 60.0 / 90.0) < 1e-6, shown[3][2]

    assert imaging.published_points(stage, STATIC) == (imaging.NOT_DRIVEN, None)
    assert imaging.published_points(stage, HEAD) == (imaging.NOT_DRIVEN, None)
    assert imaging.published_xform_revision(stage, STATIC) == (imaging.NOT_DRIVEN, None)
    assert imaging.published_scalar_primvar(stage, HEAD, "noSuchPrimvar") == (
        imaging.NOT_DRIVEN, None)
    other = make_rig()
    assert imaging.published_points(other, STROKE)[0] == imaging.NO_CONTEXT
    assert not cache.GetId(other).IsValid()

    lib.RigExecImaging_DeactivateForStage(stage_id)
    cache.Erase(stage)
    print("test_imaging_published: OK")


if __name__ == "__main__":
    main()
