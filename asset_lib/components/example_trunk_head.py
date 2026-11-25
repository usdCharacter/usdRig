# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""Worked trunk and head binding; run as a script with bin/_env configured.

Binds torso, spine and head into /Demo with every fit value in the fit layer,
then checks the rejected bindings, a compiling assembly, weighted motion with no
skincluster in sight, and that the entry asset alone reopens clean with the
manifest control hierarchy.

The `compose` import below resolves from this file's folder, so run the script
rather than importing it. Its fit values are arbitrary test numbers and its
body is a `UsdGeom.Points` stand-in for a character mesh.
"""
import json
import math
import os.path
import tempfile
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom
import rigexec
from compose import LIBRARY, bind_component, compose_assembly, validate_component


ROOT = '/Demo'
RIG = ROOT + '/Rig'
# Stand-in for a character mesh: a real rig binds the same `.points` endpoint.
MESH = ROOT + '/Geo/body'
CHAIN = ('torso', 'spine', 'head')
DRIVING = {'torso': 'Control_Hips', 'spine': 'Control_Chest', 'head': 'Control_Neck'}
COLORS = {'torso': (.85, .45, .15), 'spine': (.25, .7, .3), 'head': (.25, .45, .9)}
SCALES = {'torso': .18, 'spine': .16, 'head': .1}
INFLUENCE = {'torso': ((1, 2, 3), (1., .5, .15)),
             'spine': ((3, 4, 5), (1., .6, .25)),
             'head': ((4, 5), (1., .5))}
EXCLUDED = 0
STEP = .25
POINTS = ((.12, 0, .15), (.1, .02, .45), (-.11, .03, .7),
          (0, .13, .95), (-.09, -.06, 1.2), (.05, 0, 1.45))
POSED = (('torso', 'Control_COG', 'tz', .06), ('spine', 'Control_Chest', 'rz', 14.),
         ('head', 'Control_Neck', 'rx', 9.))
# The worked-trunk-head walkthrough quotes this diagnostic verbatim.
PARENT_ERROR = ('Provider parent differs for /Demo/Rig/World/COG/Hips/Spine1; '
                'expected /Demo/Rig/World/COG')


def manifest(component):
    return json.loads((LIBRARY / component / 'component.json').read_text())


def destinations(component, start):
    """One destination per manifest role, nested under its declared parent."""
    paths, depths = {}, {}
    for role, definition in manifest(component)['nodes'].items():
        parent = definition['parent']
        # A port prim is a parent, so a port-parented role nests under it.
        if parent.startswith('port:'):
            paths[role], depths[role] = start + '/' + role[len('Control_'):], 0
        else:
            above = parent[len('node:'):]
            paths[role] = paths[above] + '/' + role[len('Control_'):]
            depths[role] = depths[above] + 1
    return paths, depths


def plan(spine_parent=None):
    """Nodes, ports and attachments for the trunk and head chain."""
    # Each port binds to a prim of its providing instance: torso's Rig to this
    # scaffold's root, spine's COG to torso's Control_COG, and head's Chest to
    # spine's Control_Chest, so the traced chain reads World, COG, Spine1,
    # Spine2, Chest, Neck, Head.
    nodes, ports, depths, attachments = {}, {}, {}, {}
    ports['torso'] = {'Rig': RIG}
    nodes['torso'], depths['torso'] = destinations('torso', RIG)
    ports['spine'] = {'COG': nodes['torso']['Control_COG']}
    nodes['spine'], depths['spine'] = destinations('spine', spine_parent or ports['spine']['COG'])
    ports['head'] = {'Chest': nodes['spine']['Control_Chest']}
    nodes['head'], depths['head'] = destinations('head', ports['head']['Chest'])
    for instance in CHAIN:
        template = 'Deform_' + DRIVING[instance][len('Control_'):] + '_Weighted'
        data = manifest(instance)
        assert template in data['templates'], (instance, template,
                                               sorted(data['templates']))
        weight = RIG + '/Weights/' + instance + '/Weight'
        attachments[instance] = [
            {'template': 'Weight', 'path': weight,
             'relationships': {'rigExec:weightTarget': {'targets': [MESH + '.points']}}},
            {'template': template, 'path': RIG + '/Movers/' + instance + '/' + template,
             'relationships': {
                 'rigExec:moves': {'targets': [MESH + '.points']},
                 'rigExec:transform': {'targets': [nodes[instance][DRIVING[instance]]]},
                 'rigExec:transformSpace': {'targets': [RIG + '/World']},
                 'rigExec:weightObject': {'targets': [weight]}}}]
    return nodes, ports, depths, attachments


def fit(field, instance, depth):
    name = field['name']
    if name == 'rest:space':
        return Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(0, 0, STEP * depth))
    if name == 'guide:displayColor':
        return Gf.Vec3f(*COLORS[instance])
    if name == 'guide:offset':
        return Gf.Vec3d(0, 0, 0)
    if name.startswith('guide:scale'):
        return SCALES[instance]
    if name == 'rigExec:indices':
        return list(INFLUENCE[instance][0])
    if name == 'rigExec:values':
        return list(INFLUENCE[instance][1])
    raise AssertionError('Unhandled fit field: ' + name)


def scaffold(stage, nodes, depths, attachments):
    """Author the fitted scaffold the composition layer references into."""
    # Undefined ancestors would hide the referenced descendants.
    stage.DefinePrim(ROOT, 'Xform')
    stage.DefinePrim(RIG, 'RigExecRoot')
    for scope in ('Movers', 'Weights'):
        stage.DefinePrim(RIG + '/' + scope, 'Scope')
        for instance in CHAIN:
            stage.DefinePrim(RIG + '/' + scope + '/' + instance, 'Scope')
    for kind in ('nodes', 'templates'):
        for instance in CHAIN:
            entries = (nodes[instance].items() if kind == 'nodes'
                       else ((a['template'], a['path']) for a in attachments[instance]))
            for role, path in entries:
                definition = manifest(instance)[kind][role]
                depth = depths[instance].get(role, 0) if kind == 'nodes' else 0
                prim = stage.DefinePrim(path, definition['type'])
                for field in definition['fit']:
                    attr = prim.CreateAttribute(field['name'],
                                                Sdf.ValueTypeNames.Find(field['type']),
                                                custom=False)
                    attr.Set(fit(field, instance, depth))


def write(folder, prefix, nodes, ports, depths, attachments):
    """Write the fit, bindings, composition, entry and model layers.

    Returns the written entry and model layer paths."""
    names = {kind: prefix + suffix for kind, suffix in
             (('fit', '_rig.fit.usda'), ('composition', '_rig.components.usda'),
              ('entry', '_rig.usda'), ('model', '_model.usda'))}
    layer = Sdf.Layer.CreateNew(str(folder / names['fit']))
    fit_stage = Usd.Stage.Open(layer)
    scaffold(fit_stage, nodes, depths, attachments)
    layer.Save()
    instances = {name: {'component': name, 'nodes': nodes[name], 'ports': ports[name],
                        'attachments': attachments[name]} for name in CHAIN}
    try:
        library = os.path.relpath(LIBRARY, folder).replace('\\', '/')
    except ValueError:
        # Windows cannot express a relative library path across drive letters.
        library = LIBRARY.resolve().as_posix()
    payload = {'version': 1, 'library': library, 'fit': names['fit'], 'instances': instances}
    (folder / (prefix + '_rig.bindings.json')).write_text(json.dumps(payload, indent=2))
    compose_assembly(folder / (prefix + '_rig.bindings.json'), folder / names['composition'])
    entry = Sdf.Layer.CreateNew(str(folder / names['entry']))
    entry.subLayerPaths = [names['fit'], names['composition']]
    entry_stage = Usd.Stage.Open(entry)
    entry_stage.SetDefaultPrim(entry_stage.GetPrimAtPath(RIG))
    entry.Save()
    model = Sdf.Layer.CreateNew(str(folder / names['model']))
    model.subLayerPaths = [names['entry']]
    model_stage = Usd.Stage.Open(model)
    UsdGeom.Points.Define(model_stage, MESH).CreatePointsAttr(list(POINTS))
    model.Save()
    return entry.realPath, model.realPath


def check_ports(stage, nodes, ports):
    for instance, role in (('spine', 'COG'), ('head', 'Chest')):
        try:
            bind_component(stage, instance, nodes[instance], {'Hips': ports[instance][role]})
        except ValueError as error:
            assert 'Ports must exactly match the ' + instance + ' manifest' in str(error), error
        else:
            raise AssertionError('Rebound ' + instance + ' through another component port')
    print('PASS spine and head ports come from their manifests')


def check_parents(folder, nodes):
    """A correct port name with a node under another provider is still rejected."""
    # spine hangs off Hips, whose provider is torso, not the COG provider; the
    # second write below emits that rig's layers under the 'wrong' prefix.
    spine_parent = nodes['torso']['Control_Hips']
    wrong_nodes, wrong_ports, wrong_depths, wrong_attachments = plan(spine_parent)
    _, model = write(folder, 'wrong', wrong_nodes, wrong_ports,
                     wrong_depths, wrong_attachments)
    stage = Usd.Stage.Open(model)
    try:
        validate_component(stage, 'spine', wrong_nodes['spine'], wrong_ports['spine'],
                           wrong_attachments['spine'])
    except ValueError as error:
        # The expected provider is the bound COG port, not the enclosing Hips.
        expected = ('Provider parent differs for '
                    + wrong_nodes['spine']['Control_Spine1']
                    + '; expected ' + wrong_ports['spine']['COG'])
        assert str(error) == expected, error
        assert expected == PARENT_ERROR, expected
    else:
        raise AssertionError('A spine chain under another provider was accepted')
    print('PASS provider parents enforced')


def evaluate(rig):
    pose = rig.evaluate(1)
    assert pose.valid and pose.reference_mismatches == 0, pose.diagnostics
    points = [tuple(point) for point in pose.moved_property(MESH + '.points')]
    assert all(math.isfinite(value) for point in points for value in point), points
    return points


def check_assembly(stage, nodes, ports, attachments):
    for instance in CHAIN:
        validate_component(stage, instance, nodes[instance], ports[instance],
                           attachments[instance])
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    rig.cpu_reference = True
    evaluate(rig)
    print('PASS trunk and head bind, validate and compile')
    return rig


def check_motion(stage, rig, nodes):
    rest = evaluate(rig)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    gaits = []
    for amplitude in (1., 2.):
        for instance, role, channel, amount in POSED:
            path = nodes[instance][role]
            stage.GetPrimAtPath(path).GetAttribute('avars:' + channel).Set(amount * amplitude)
        gaits.append(evaluate(rig))
    shifts = [[(Gf.Vec3d(*base) - Gf.Vec3d(*moved)).GetLength()
               for base, moved in zip(rest, gait)] for gait in gaits]
    for index in sorted({i for influence in INFLUENCE.values() for i in influence[0]}):
        assert shifts[0][index] > 1e-3, (index, shifts[0][index])
        assert shifts[1][index] > 1.5 * shifts[0][index], (index, shifts[0][index])
    for gait in gaits:
        assert rest[EXCLUDED] == gait[EXCLUDED], (rest[EXCLUDED], gait[EXCLUDED])
    body = stage.GetPrimAtPath(MESH)
    skel = [attr.GetName() for attr in body.GetAuthoredAttributes()
            if str(attr.GetName()).startswith('skel:')]
    assert 'SkelBindingAPI' not in body.GetAppliedSchemas(), body.GetAppliedSchemas()
    assert not skel, skel
    print('PASS weighted motion without a skincluster')


def check_reopen(path, nodes, ports):
    """Reopen the entry asset on its own; only the model layer adds the mesh."""
    stage = Usd.Stage.Open(path)
    assert not list(stage.GetCompositionErrors()), list(stage.GetCompositionErrors())
    # The mesh lives in the model layer, so this stage is the entry, not the model.
    assert not stage.GetPrimAtPath(MESH).IsValid(), MESH
    for instance in CHAIN:
        for role, definition in manifest(instance)['nodes'].items():
            prim = stage.GetPrimAtPath(nodes[instance][role])
            assert prim and prim.IsDefined(), (instance, role)
            assert prim.GetTypeName() == definition['type'], (instance, role)
            parent = definition['parent']
            above = (ports[instance][parent[len('port:'):]] if parent.startswith('port:')
                     else nodes[instance][parent[len('node:'):]])
            assert prim.GetParent().GetPath() == Sdf.Path(above), (nodes[instance][role], above)
    print('PASS entry reopens with no composition errors')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='rig-trunk-head-') as name:
        folder = Path(name)
        nodes, ports, depths, attachments = plan()
        entry, model = write(folder, 'demo', nodes, ports, depths, attachments)
        stage = Usd.Stage.Open(model)
        check_ports(stage, nodes, ports)
        check_parents(folder, nodes)
        rig = check_assembly(stage, nodes, ports, attachments)
        check_motion(stage, rig, nodes)
        check_reopen(entry, nodes, ports)
