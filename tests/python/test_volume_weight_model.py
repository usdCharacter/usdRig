#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/volumeWeightModel.py.

The volume weight panel (volumeWeightUI.py) is a thin Qt driver over
this module's authoring functions, so everything the panel does to the
stage is exercised here without a display: creating each weight type,
editing rigExec:inputWeights order, inserting/moving/deleting a falloff
knot, and authoring a value at a specific time -- each undone through
the shared rigExecUndo stack.

Usage: test_volume_weight_model.py [<generated schema resources dir>]
"""
import os
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Usd, UsdGeom  # noqa: E402

import rigExecUndo  # noqa: E402
import volumeWeightModel  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tolerance=1e-4):
    return abs(float(a) - float(b)) <= tolerance


def _ClosePoints(first, second):
    return (len(first) == len(second) and all(
        _Close(a, b) and _Close(c, d)
        for (a, c), (b, d) in zip(first, second)))


def _RegisterSchema():
    # bin/test/run_python_tests.sh hands the generated resources to the
    # schema tests as argv[1]; without it (or under ctest one day) fall
    # back to the selected build tree, then to PXR_PLUGINPATH_NAME.
    if Plug.Registry().GetPluginWithName("rigExecSchema") is None:
        here = os.path.dirname(os.path.abspath(__file__))
        repo = os.path.dirname(os.path.dirname(here))
        build = os.environ.get("RIGEXEC_BUILD_DIR") or os.path.join(
            repo, "build")
        candidates = []
        if len(sys.argv) > 1:
            candidates.append(sys.argv[1])
        candidates.append(os.path.join(
            build, "usd", "rigExecSchema", "resources"))
        for candidate in candidates:
            if os.path.isdir(candidate):
                Plug.Registry().RegisterPlugins(candidate)
                break
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecSphereWeight"),
           "RigExecSphereWeight schema is not registered; pass the "
           "generated resources dir (build/usd/rigExecSchema/resources)")


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")
    return stage


def _StageWithMesh():
    stage = _Stage()
    mesh = UsdGeom.Mesh.Define(stage, "/Rig/Geom/Body")
    mesh.CreatePointsAttr([
        Gf.Vec3f(-4.0, -2.0, -2.0), Gf.Vec3f(4.0, -2.0, -2.0),
        Gf.Vec3f(4.0, 2.0, 2.0), Gf.Vec3f(-4.0, 2.0, 2.0)])
    mesh.CreateExtentAttr([
        Gf.Vec3f(-4.0, -2.0, -2.0), Gf.Vec3f(4.0, 2.0, 2.0)])
    return stage, mesh.GetPrim()


def TestModelStaysQtFree():
    """The split's whole point: the model imports without Qt."""
    for name in sys.modules:
        _Check(not name.startswith("PySide")
               and not name.startswith("PyQt")
               and name != "pxr.Usdviewq"
               and not name.startswith("pxr.Usdviewq."),
               "the model pulled in Qt via %s" % name)


def TestCreateEachTypeIsOneUndo():
    for typeName in volumeWeightModel.VOLUME_WEIGHT_TYPE_NAMES:
        stage = _Stage()
        undo = rigExecUndo.UndoStack()
        volumeWeightModel.SetUndoStack(undo)
        try:
            parentPath, parentExisted = \
                volumeWeightModel.ChooseWeightParentPath(stage, [])
            name = volumeWeightModel.DefaultPrimNameForType(typeName)
            newPath = parentPath.AppendChild(name)
            paths = [newPath] + ([] if parentExisted else [parentPath])
            with volumeWeightModel.Recording(stage, paths, "Create"):
                prim = volumeWeightModel.CreateVolumeWeightPrim(
                    stage, typeName, [], Usd.TimeCode.Default())
            _Check(bool(prim) and prim.GetTypeName() == typeName,
                   "%s was not created" % typeName)
            _Check(prim.GetPath() == newPath,
                   "%s landed at %s, not %s"
                   % (typeName, prim.GetPath(), newPath))
            undo.Undo()
            _Check(not stage.GetPrimAtPath(newPath),
                   "undo left %s behind" % newPath)
            _Check(not stage.GetPrimAtPath(parentPath),
                   "undo left the Weights scope behind")
            undo.Redo()
            _Check(bool(stage.GetPrimAtPath(newPath)),
                   "redo did not bring %s back" % typeName)
        finally:
            volumeWeightModel.SetUndoStack(None)


def TestCreateBindsSelection():
    stage, meshPrim = _StageWithMesh()
    undo = rigExecUndo.UndoStack()
    volumeWeightModel.SetUndoStack(undo)
    try:
        with volumeWeightModel.Recording(
                stage, [Sdf.Path("/Rig/Weights")], "Create"):
            prim = volumeWeightModel.CreateVolumeWeightPrim(
                stage, "RigExecSphereWeight", [meshPrim],
                Usd.TimeCode.Default())
        targets = prim.GetRelationship(
            "rigExec:weightTarget").GetTargets()
        _Check(targets == [Sdf.Path("/Rig/Geom/Body.points")],
               "weightTarget not bound: %s" % targets)
        _Check(prim.GetAttribute("avars:tx").Get() is not None,
               "placement was not seeded from the mesh")
        undo.Undo()
        _Check(not stage.GetPrimAtPath(prim.GetPath()),
               "undo left the bound weight behind")
    finally:
        volumeWeightModel.SetUndoStack(None)


def TestInputWeightOrderUndo():
    stage = _Stage()
    sphere = volumeWeightModel.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())
    plane = volumeWeightModel.CreateVolumeWeightPrim(
        stage, "RigExecPlaneWeight", [], Usd.TimeCode.Default())
    combine = volumeWeightModel.CreateVolumeWeightPrim(
        stage, "RigExecCombineWeight", [], Usd.TimeCode.Default())
    undo = rigExecUndo.UndoStack()
    volumeWeightModel.SetUndoStack(undo)
    try:
        with volumeWeightModel.Recording(
                stage, [combine.GetPath()], "Add inputs"):
            added = volumeWeightModel.AddInputWeightPaths(
                combine, [sphere.GetPath(), plane.GetPath()])
        _Check(added == 2, "added %d inputs, expected 2" % added)
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            sphere.GetPath(), plane.GetPath()],
            "input order wrong: %s"
            % volumeWeightModel.GetInputWeightPaths(combine))
        undo.Undo()
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [],
               "undo did not clear the inputs")
        undo.Redo()
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            sphere.GetPath(), plane.GetPath()], "redo lost the inputs")

        with volumeWeightModel.Recording(
                stage, [combine.GetPath()], "Reorder inputs"):
            volumeWeightModel.SetInputWeightPaths(
                combine, [plane.GetPath(), sphere.GetPath()])
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            plane.GetPath(), sphere.GetPath()], "reorder did not land")
        undo.Undo()
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            sphere.GetPath(), plane.GetPath()],
            "undo did not restore the input order")

        with volumeWeightModel.Recording(
                stage, [combine.GetPath()], "Remove input"):
            removed = volumeWeightModel.RemoveInputWeightPaths(
                combine, [sphere.GetPath()])
        _Check(removed == 1, "removed %d inputs, expected 1" % removed)
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            plane.GetPath()], "removal did not land")
        undo.Undo()
        _Check(volumeWeightModel.GetInputWeightPaths(combine) == [
            sphere.GetPath(), plane.GetPath()],
            "undo did not restore the removed input")
    finally:
        volumeWeightModel.SetUndoStack(None)


def _FalloffAttr():
    stage = _Stage()
    prim = volumeWeightModel.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())
    attr = prim.GetAttribute("rigExec:falloffCurve")
    _Check(bool(attr), "no rigExec:falloffCurve attribute")
    volumeWeightModel.EnsureDefaultSpline(attr, "linear", force=True)
    _Check(volumeWeightModel.GetSplineKnotPoints(attr) == [
        (0.0, 0.0), (1.0, 1.0)], "linear did not start on the diagonal")
    return stage, prim, attr


def TestSplineKnotEditsUndo():
    stage, prim, attr = _FalloffAttr()
    undo = rigExecUndo.UndoStack()
    volumeWeightModel.SetUndoStack(undo)
    try:
        with volumeWeightModel.Recording(
                stage, [prim.GetPath()], "Insert knot"):
            x = volumeWeightModel.InsertSplineKnot(attr, 0.5, 0.9)
        _Check(_Close(x, 0.5), "insert landed at %s" % x)
        _Check(_ClosePoints(
            volumeWeightModel.GetSplineKnotPoints(attr),
            [(0.0, 0.0), (0.5, 0.9), (1.0, 1.0)]),
            "insert did not land: %s"
            % volumeWeightModel.GetSplineKnotPoints(attr))
        undo.Undo()
        _Check(volumeWeightModel.GetSplineKnotPoints(attr) == [
            (0.0, 0.0), (1.0, 1.0)],
            "undo did not remove the knot: %s"
            % volumeWeightModel.GetSplineKnotPoints(attr))
        undo.Redo()
        _Check(len(volumeWeightModel.GetSplineKnotPoints(attr)) == 3,
               "redo did not bring the knot back")

        with volumeWeightModel.Recording(
                stage, [prim.GetPath()], "Move knot"):
            moved = volumeWeightModel.MoveSplineKnot(
                attr, 0.5, 0.3, 0.2)
        _Check(moved is not None and _Close(moved, 0.3),
               "move returned %s" % (moved,))
        _Check(_ClosePoints(
            volumeWeightModel.GetSplineKnotPoints(attr),
            [(0.0, 0.0), (0.3, 0.2), (1.0, 1.0)]),
            "move did not land: %s"
            % volumeWeightModel.GetSplineKnotPoints(attr))
        undo.Undo()
        _Check(_ClosePoints(
            volumeWeightModel.GetSplineKnotPoints(attr),
            [(0.0, 0.0), (0.5, 0.9), (1.0, 1.0)]),
            "undo did not put the knot back: %s"
            % volumeWeightModel.GetSplineKnotPoints(attr))

        with volumeWeightModel.Recording(
                stage, [prim.GetPath()], "Delete knot"):
            deleted = volumeWeightModel.DeleteSplineKnot(attr, 0.5)
        _Check(deleted, "delete refused a 3-knot spline")
        _Check(volumeWeightModel.GetSplineKnotPoints(attr) == [
            (0.0, 0.0), (1.0, 1.0)],
            "delete did not land: %s"
            % volumeWeightModel.GetSplineKnotPoints(attr))
        undo.Undo()
        _Check(len(volumeWeightModel.GetSplineKnotPoints(attr)) == 3,
               "undo did not restore the deleted knot")
    finally:
        volumeWeightModel.SetUndoStack(None)


def TestTimeSpecificWriteUndo():
    stage = _Stage()
    prim = volumeWeightModel.CreateVolumeWeightPrim(
        stage, "RigExecSphereWeight", [], Usd.TimeCode.Default())
    attr = prim.GetAttribute("inputs:falloffMax")
    attr.Set(1.0)
    undo = rigExecUndo.UndoStack()
    volumeWeightModel.SetUndoStack(undo)
    try:
        volumeWeightModel.SetAtTime(attr, 7.0, Usd.TimeCode(5.0))
        _Check(undo.CanUndo(), "the timed write pushed no entry")
        _Check(attr.Get(Usd.TimeCode(5.0)) == 7.0,
               "the timed write did not land")
        _Check(attr.Get() == 1.0, "the default moved with it")
        undo.Undo()
        _Check(attr.Get(Usd.TimeCode(5.0)) == 1.0,
               "undo did not restore frame 5, got %s"
               % attr.Get(Usd.TimeCode(5.0)))
        _Check(attr.GetNumTimeSamples() == 0,
               "undo left time samples behind")
        undo.Redo()
        _Check(attr.Get(Usd.TimeCode(5.0)) == 7.0,
               "redo did not reapply the timed write")

        # Once the attribute carries animation, SetVisibleAtTime writes
        # at the frame rather than to the default.
        volumeWeightModel.SetVisibleAtTime(
            attr, 3.0, Usd.TimeCode(5.0))
        _Check(attr.Get(Usd.TimeCode(5.0)) == 3.0,
               "the visible write did not land")
        undo.Undo()
        _Check(attr.Get(Usd.TimeCode(5.0)) == 7.0,
               "undo did not restore the timed value, got %s"
               % attr.Get(Usd.TimeCode(5.0)))
    finally:
        volumeWeightModel.SetUndoStack(None)


def main():
    _RegisterSchema()
    groups = [
        ("the model imports without Qt", TestModelStaysQtFree),
        ("create each type is one undo", TestCreateEachTypeIsOneUndo),
        ("create binds the selection", TestCreateBindsSelection),
        ("input weight order undoes", TestInputWeightOrderUndo),
        ("spline knot edits undo", TestSplineKnotEditsUndo),
        ("a time-specific write undoes", TestTimeSpecificWriteUndo),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_VOLUME_WEIGHT_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
