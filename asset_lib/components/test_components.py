# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""Independent fit and evaluation checks; run with bin/_env configured."""
import json
import tempfile
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom
import rigexec
from compose import LIBRARY, bind_component, validate_component


def matrix(x=0, y=0, z=0):
    return Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(x, y, z))


def check_leg(folder, size):
    file = Path(folder) / ('leg_' + str(size) + '.usda')
    stage = Usd.Stage.CreateNew(str(file))
    stage.DefinePrim('/Example', 'Xform')
    stage.DefinePrim('/Example/Rig', 'RigExecRoot')
    stage.DefinePrim('/Example/Rig/Joints', 'Scope')
    world = stage.DefinePrim('/Example/Rig/World', 'RigExecControl')
    hips = stage.DefinePrim('/Example/Rig/World/Hips', 'RigExecControl')
    for prim in (world, hips):
        prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(['RigExecControlAPI']))
        prim.CreateAttribute('rest:space', Sdf.ValueTypeNames.Matrix4d).Set(matrix())
    paths = {
        'Control_HipRoot': '/Example/Rig/World/Hips/Root',
        'Control_FootIK': '/Example/Rig/World/Goal',
        'Control_KneePole': '/Example/Rig/World/Pole',
        'Joint_LegUpper': '/Example/Rig/Joints/Upper',
        'Joint_LegLower': '/Example/Rig/Joints/Upper/Lower',
        'Joint_Foot': '/Example/Rig/Joints/Upper/Lower/End',
        'Solver_LegIK': '/Example/Rig/Solve',
    }
    ports = {'Rig': '/Example/Rig', 'World': str(world.GetPath()), 'Hips': str(hips.GetPath())}
    points = UsdGeom.Points.Define(stage, '/Example/Geometry/Foot')
    points.CreatePointsAttr([(2 * size, 0, 0)])
    attachments = [{'template': 'Deform_Foot_Absolute', 'path': '/Example/Rig/Deform',
                    'relationships': {
                        'rigExec:moves': {'targets': ['/Example/Geometry/Foot.points']},
                        'rigExec:transform': {'targets': [paths['Joint_Foot']]},
                    }}]
    manifest = json.loads((LIBRARY / 'leg/component.json').read_text())
    bind_component(stage, 'leg', paths, ports, attachments)
    frames = {'Control_FootIK': matrix(2 * size), 'Control_KneePole': matrix(size, 0, size),
              'Joint_LegLower': matrix(size), 'Joint_Foot': matrix(size)}
    for role, definition in manifest['nodes'].items():
        prim = stage.GetPrimAtPath(paths[role])
        for field in definition['fit']:
            name = field['name']
            typ = Sdf.ValueTypeNames.Find(field['type'])
            value = frames.get(role, matrix()) if name == 'rest:space' else (
                Gf.Vec3f(.5) if name == 'guide:displayColor' else (
                    Gf.Vec3d(0) if name == 'guide:offset' else .02 * size))
            prim.CreateAttribute(name, typ, custom=False).Set(value)
    validate_component(stage, 'leg', paths, ports, attachments)
    rig = rigexec.Rig(stage, '/Example/Rig')
    rig.compile()
    rig.cpu_reference = True
    rest = rig.evaluate(1)
    assert rest.valid and rest.reference_mismatches == 0
    initial = rest.moved_property('/Example/Geometry/Foot.points')[0]
    stage.GetPrimAtPath(paths['Control_FootIK']).GetAttribute('avars:ty').Set(.3 * size)
    stage.GetPrimAtPath(paths['Control_FootIK']).GetAttribute('avars:tx').Set(-.2 * size)
    moved = rig.evaluate(1)
    assert moved.valid and moved.reference_mismatches == 0
    final = moved.moved_property('/Example/Geometry/Foot.points')[0]
    assert (Gf.Vec3d(*final) - Gf.Vec3d(*initial)).GetLength() > .1 * size
    missing = stage.GetPrimAtPath(paths['Control_FootIK'])
    missing.RemoveProperty('rest:space')
    try:
        validate_component(stage, 'leg', paths, ports, attachments)
    except ValueError as error:
        assert 'Missing or incompatible fit' in str(error)
    else:
        raise AssertionError('An unfitted control was accepted')
    print('PASS independent leg fit and IK:', size)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='rig-components-') as folder:
        for size in (0.5, 2.0):
            check_leg(folder, size)
