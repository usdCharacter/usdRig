# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""Worked control handoff; run with bin/_env configured.

Composes a small /Demo stand-in character from the torso, spine and leg
components inside a temporary directory, derives demo_rig.controls.json from
the composed public controls, and asserts the seven handoff gates: coverage,
responses, pose, no-write, rebuild, missing-pole-control and
prebuilt-only-recipe.
Prints one PASS line per gate and exits non-zero on the first failure.
Rendering stays host-side (bin/usdview.sh demo_model.usda); the automated
gates here are numerical.

    . ./bin/_env.sh && "$PY" asset_lib/components/example_control_handoff.py
"""
import hashlib
import json
import math
import os
import shutil
import struct
import sys
import tempfile
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom
import rigexec
from compose import LIBRARY, compose_assembly, validate_component

# The PoseChannels cross-check lives in the usdview plugin; it is pure Python.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'plugin' / 'rigExecUsdview'))
import avarEditorModel

ROOT = '/Demo'
RIG = ROOT + '/Rig'
MESH = ROOT + '/Geo/body'
KNEE_POLE = RIG + '/World/KneePole'
CHAIN = ('torso', 'spine', 'leg')
MODEL = 'demo_model.usda'
ENTRY = 'demo_rig.usda'
FIT = 'demo_rig.fit.usda'
COMPOSITION = 'demo_rig.components.usda'
BINDINGS = 'demo_rig.bindings.json'
ANIMATION = 'demo_animation.usda'
TABLE = 'demo_rig.controls.json'
LAYERS = (MODEL, ENTRY, FIT, COMPOSITION, ANIMATION)

DIAG_TABLE_OMITS = 'coverage gate: demo_rig.controls.json omits /Demo/Rig/World/KneePole'
DIAG_TABLE_NAMES = ('coverage gate: demo_rig.controls.json names a control the '
                    'stage lacks: /Demo/Rig/World/KneePole')
DIAG_PREBUILT = 'rebuild gate: demo_rig.components.usda must be absent before the recipe runs'
DIAG_BINDING_REMOVAL = 'Node roles must exactly match the leg manifest'

COLORS = {'torso': (.85, .45, .15), 'spine': (.25, .7, .3), 'leg': (.25, .45, .9)}
SCALES = {'torso': .18, 'spine': .16, 'leg': .12}
RADIUS = .04
REST = {
    ('torso', 'Control_World'): (0, 0, 0),
    ('torso', 'Control_COG'): (0, .95, 0),
    ('torso', 'Control_Hips'): (0, .05, 0),
    ('spine', 'Control_Spine1'): (0, .10, 0),
    ('spine', 'Control_Spine2'): (0, .12, 0),
    ('spine', 'Control_Chest'): (0, .13, 0),
    ('leg', 'Control_HipRoot'): (0, 0, 0),
    ('leg', 'Control_FootIK'): (0, .15, 0),
    ('leg', 'Control_KneePole'): (.35, .55, 0),
    ('leg', 'Joint_LegUpper'): (0, 1.0, 0),
    ('leg', 'Joint_LegLower'): (0, -.50, 0),
    ('leg', 'Joint_Foot'): (0, -.45, 0),
}
INFLUENCE = {'torso': ((1, 2), (.4, 1.)), 'spine': ((0, 1), (1., .4)),
             'leg': ((3, 4), (.7, 1.))}
DOCUMENTED = sorted(i for indices, _ in INFLUENCE.values() for i in indices)
UNWEIGHTED = 6
POINTS = ((0, 1.30, 0), (0, 1.16, .08), (0, .99, .06),
          (.09, .74, .02), (.11, .44, 0), (0, .15, .05), (0, 1.45, -.05))
DRIVER = {'torso': ('Control_Hips', 'Deform_Hips_Weighted', 'Hips'),
          'spine': ('Control_Chest', 'Deform_Chest_Weighted', 'Chest'),
          'leg': ('Joint_LegLower', 'Deform_LegLower_Weighted', 'LegLower')}

WALK = (1, 5, 9, 13, 16)
RUN = (17, 21, 25, 29, 32)
RUN_FRAME = 25
ANIM_KEYS = (
    ('torso', 'Control_COG', 'avars:ty', {1: 0., 5: .04, 9: 0., 13: .04, 16: 0.,
                                          17: .06, 21: .02, 25: .06, 29: .02, 32: .06}),
    ('torso', 'Control_COG', 'avars:tz', {1: -.10, 9: .10, 16: -.10,
                                          17: -.16, 25: .16, 32: -.16}),
    ('leg', 'Control_FootIK', 'avars:tz', {1: -.25, 5: 0., 9: .25, 13: 0., 16: -.25,
                                           17: -.40, 21: 0., 25: .40, 29: 0., 32: -.40}),
)
POLE_CHANNEL = 'avars:tz'
POLE_CHANNELS = ('avars:tx', 'avars:ty', 'avars:tz')
POLE_POSE = .45
LEG_WEIGHTED = (3, 4)
MIN_POLE_SHIFT = 1e-2
MIN_FRAME_SHIFT = 1e-3

CONFIG_CHANNELS = frozenset(('avars:defaultSpace', 'avars:rotationOrder',
                             'avars:rotationSign', 'avars:unitScaleFactor'))
SCHEMA_AVARS = frozenset('avars:' + n for n in
                         ('tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'rspin', 'sx', 'sy', 'sz'))
ROW_UNITS = 'stage length unit (degrees for avars:r*, avars:rspin)'
TRANS_RANGE = (-0.45, 0.45)
POLE_RANGE = (-0.6, 0.6)
ROT_RANGE = (-45.0, 45.0)
TRANSFORM_CHANNELS = ('avars:tx', 'avars:ty', 'avars:tz')
ROTATION_CHANNELS = ('avars:rx', 'avars:ry', 'avars:rz')
# The stand-in points each property moves when posed alone at its suggested
# range end in this fit. gate_responses poses every row in isolation and
# requires exactly these indices to move; never write a table claim this gate
# has not measured.
CLAIMS = {
    'World': {c: () for c in TRANSFORM_CHANNELS + ROTATION_CHANNELS},
    'COG': {'avars:tx': (0, 1, 2, 3, 4), 'avars:ty': (0, 1, 2, 3, 4),
            'avars:tz': (0, 1, 2, 3, 4), 'avars:rx': (0, 1, 2, 3, 4),
            'avars:ry': (1, 2), 'avars:rz': (0, 1, 2, 3, 4)},
    'Hips': {'avars:tx': (1, 2, 3, 4), 'avars:ty': (1, 2, 3, 4),
             'avars:tz': (1, 2, 3, 4), 'avars:rx': (1, 2), 'avars:ry': (1, 2),
             'avars:rz': (1, 2)},
    'HipRoot': {},
    'Spine1': {'avars:tx': (0, 1), 'avars:ty': (0, 1), 'avars:tz': (0, 1),
               'avars:rx': (0, 1), 'avars:ry': (1,), 'avars:rz': (0, 1)},
    'Spine2': {'avars:tx': (0, 1), 'avars:ty': (0, 1), 'avars:tz': (0, 1),
               'avars:rx': (0, 1), 'avars:ry': (1,), 'avars:rz': (0, 1)},
    'Chest': {'avars:tx': (0, 1), 'avars:ty': (0, 1), 'avars:tz': (0, 1),
              'avars:rx': (1,), 'avars:ry': (1,), 'avars:rz': (1,)},
    'FootIK': {'avars:tx': (3, 4), 'avars:ty': (3, 4), 'avars:tz': (3, 4),
               'avars:rx': (), 'avars:ry': (), 'avars:rz': ()},
    'KneePole': {'avars:tx': (), 'avars:ty': (), 'avars:tz': (3, 4)},
}
NO_RESPONSE = 'no measured response on the stand-in points at the suggested range'
ROLE = {'World': 'rig-wide offset',
        'COG': 'offsets the hips and spine chains',
        'Hips': 'drives the torso deform and ports the leg',
        'HipRoot': 're-roots the leg IK chain; no authored channels on this control',
        'Spine1': 'offsets the chest chain',
        'Spine2': 'offsets the chest',
        'Chest': 'drives the spine deform',
        'FootIK': 'moves the foot goal',
        'KneePole': 'steers the knee bend'}
FOOTIK_OMISSION = "the foot goal's rotations reach no stand-in weight: no foot " \
                  'or toe weights are authored'
# Why a zero-response row moves nothing; the row's omitted_capability. The
# foot rotation disclosure also stands in the top-level OMITTED list.
OMIT_ROW = {
    'World': 'movers measure against World, so a World offset carries the '
             'whole rig and no evaluated point changes',
    'FootIK': FOOTIK_OMISSION,
    'KneePole': 'the pole reaches the stand-in through avars:tz: a tx or ty '
                'pose at the suggested range leaves every point at rest'}
OMITTED = ['no arm, hand or finger instances',
           'no eye, gaze or blink instance',
           FOOTIK_OMISSION]


def manifest(component):
    return json.loads((LIBRARY / component / 'component.json').read_text())


def tail(role):
    for prefix in ('Control_', 'Joint_', 'Solver_'):
        if role.startswith(prefix):
            return role[len(prefix):]
    return role


def plan():
    """Destination paths follow each manifest's provider-parent chain."""
    nodes, ports, attachments = {}, {}, {}
    ports['torso'] = {'Rig': RIG}
    ports['spine'] = {}
    for instance in CHAIN:
        def place(role, seen=()):
            if role in nodes.get(instance, {}):
                return nodes[instance][role]
            if role in seen:
                raise AssertionError('cycle in the provider-parent chain at ' + role)
            parent = manifest(instance)['nodes'][role].get('parent')
            if parent is None:
                path = RIG + '/Solve/' + instance
            elif parent.startswith('port:'):
                path = ports[instance][parent[5:]] + '/' + tail(role)
            else:
                path = place(parent[5:], seen + (role,)) + '/' + tail(role)
            nodes.setdefault(instance, {})[role] = path
            return path
        for role in manifest(instance)['nodes']:
            place(role)
        if instance == 'torso':
            ports['spine']['COG'] = nodes['torso']['Control_COG']
            ports['leg'] = {'Hips': nodes['torso']['Control_Hips'],
                            'World': nodes['torso']['Control_World'],
                            'Rig': RIG}
    for instance in CHAIN:
        role, template, name = DRIVER[instance]
        weight = RIG + '/Weights/' + instance + '/Weight'
        attachments[instance] = [
            {'template': 'Weight', 'path': weight,
             'relationships': {'rigExec:weightTarget': {'targets': [MESH + '.points']}}},
            {'template': template, 'path': RIG + '/Movers/' + instance + '/' + name,
             'relationships': {
                 'rigExec:moves': {'targets': [MESH + '.points']},
                 'rigExec:transform': {'targets': [nodes[instance][role]]},
                 'rigExec:transformSpace': {'targets': [RIG + '/World']},
                 'rigExec:weightObject': {'targets': [weight]}}}]
    return nodes, ports, attachments


def fit_value(field, instance, role):
    name = field['name']
    if name == 'rest:space':
        x, y, z = REST[(instance, role)]
        return Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(x, y, z))
    if name == 'guide:displayColor':
        return Gf.Vec3f(*COLORS[instance])
    if name == 'guide:offset':
        return Gf.Vec3d(0, 0, 0)
    if name.startswith('guide:scale'):
        return SCALES[instance]
    if name == 'guide:radius':
        return RADIUS
    if name == 'rigExec:indices':
        return list(INFLUENCE[instance][0])
    if name == 'rigExec:values':
        return list(INFLUENCE[instance][1])
    raise AssertionError('Unhandled fit field: ' + name)


def scaffold(stage, nodes, attachments, skip=None):
    stage.DefinePrim(ROOT, 'Xform')
    stage.DefinePrim(RIG, 'RigExecRoot')
    for scope in ('Movers', 'Weights', 'Solve'):
        stage.DefinePrim(RIG + '/' + scope, 'Scope')
        for instance in CHAIN:
            stage.DefinePrim(RIG + '/' + scope + '/' + instance, 'Scope')
    for kind in ('nodes', 'templates'):
        for instance in CHAIN:
            entries = (nodes[instance].items() if kind == 'nodes'
                       else ((a['template'], a['path']) for a in attachments[instance]))
            for role, path in entries:
                if path == skip:
                    continue
                definition = manifest(instance)[kind][role]
                prim = stage.DefinePrim(path, definition['type'])
                for field in definition['fit']:
                    prim.CreateAttribute(field['name'],
                                        Sdf.ValueTypeNames.Find(field['type']),
                                        custom=False).Set(fit_value(field, instance, role))


def library_field(folder):
    try:
        return os.path.relpath(LIBRARY, folder).replace('\\', '/')
    except ValueError:  # Windows: library on another drive has no relative path
        return Path(LIBRARY).resolve().as_posix()


def write_character(folder, nodes, ports, attachments):
    fit = Sdf.Layer.CreateNew(str(folder / FIT))
    scaffold(Usd.Stage.Open(fit), nodes, attachments)
    fit.Save()
    instances = {name: {'component': name, 'nodes': nodes[name], 'ports': ports[name],
                        'attachments': attachments[name]} for name in CHAIN}
    payload = {'version': 1, 'library': library_field(folder), 'fit': FIT,
               'instances': instances}
    (folder / BINDINGS).write_text(json.dumps(payload, indent=2))
    compose_assembly(folder / BINDINGS, folder / COMPOSITION)
    entry = Sdf.Layer.CreateNew(str(folder / ENTRY))
    entry.subLayerPaths = [FIT, COMPOSITION]
    entry_stage = Usd.Stage.Open(entry)
    entry_stage.SetDefaultPrim(entry_stage.GetPrimAtPath(ROOT))
    entry.Save()
    model = Sdf.Layer.CreateNew(str(folder / MODEL))
    model.subLayerPaths = [ENTRY]
    model_stage = Usd.Stage.Open(model)
    model_stage.SetDefaultPrim(model_stage.GetPrimAtPath(ROOT))
    UsdGeom.Points.Define(model_stage, MESH).CreatePointsAttr(list(POINTS))
    model.Save()
    keyframes(folder, nodes)


def keyframes(folder, nodes):
    layer = Sdf.Layer.CreateNew(str(folder / ANIMATION))
    layer.subLayerPaths = [MODEL]
    layer.Save()
    layer = Sdf.Layer.FindOrOpen(str(folder / ANIMATION))
    for instance, role, channel, samples in ANIM_KEYS:
        path = nodes[instance][role]
        # OverridePrim leaves no spec in this USD build; author through Sdf.
        spec = Sdf.AttributeSpec(Sdf.CreatePrimInLayer(layer, path), channel,
                                 Sdf.ValueTypeNames.Double)
        for frame, value in samples.items():
            layer.SetTimeSample(Sdf.Path(path + '.' + channel), frame, value)
    layer.Save()


def open_rig(folder, layer):
    stage = Usd.Stage.Open(str(folder / layer))
    errors = list(stage.GetCompositionErrors())
    assert not errors, 'composition errors in ' + layer + ': ' + str(errors)
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    rig.cpu_reference = True
    return stage, rig


def points_of(pose):
    return pose.moved_property(MESH + '.points')


def bits(points):
    return b''.join(struct.pack('<ddd', *[float(c) for c in p]) for p in points)


def displacement(before, after):
    return [(Gf.Vec3d(*a) - Gf.Vec3d(*b)).GetLength() for a, b in zip(before, after)]


def layer_digests(folder):
    return {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
            for name in LAYERS}


def defined_controls(stage):
    found = set()
    stack = [stage.GetPrimAtPath(RIG)]
    while stack:
        for child in stack.pop().GetChildren():
            if child.GetTypeName() == 'RigExecControl':
                found.add(str(child.GetPath()))
            stack.append(child)
    return found


def stage_channels(stage, path):
    prim = stage.GetPrimAtPath(path)
    authored = {str(a.GetName()) for a in prim.GetAuthoredAttributes()
                if str(a.GetName()).startswith('avars:')
                and str(a.GetName()) not in CONFIG_CHANNELS}
    pose = {str(a.GetName()) for a in avarEditorModel.PoseChannels(prim, stage)}
    # Schema defaults claim all ten avars; only authored and custom channels count.
    unreported = authored - pose
    if unreported:
        raise AssertionError('coverage gate: PoseChannels does not report ' + path + ' '
                             + ' '.join(sorted(unreported)))
    return tuple(sorted(authored | (pose - SCHEMA_AVARS)))


def stage_controls(stage, pose):
    published = set(pose.control_paths())
    defined = defined_controls(stage)
    if published != defined:
        raise AssertionError('coverage gate: published controls differ from the stage: '
                             + ', '.join(sorted(published ^ defined)))
    return {path: stage_channels(stage, path) for path in sorted(defined)}


def provider_of(stage, path):
    parent = stage.GetPrimAtPath(path).GetParent()
    while parent and parent.GetPath().pathString != RIG:
        if parent.GetTypeName() == 'RigExecControl':
            return str(parent.GetPath())
        parent = parent.GetParent()
    return 'rig'


def range_of(control, channel):
    if not channel:
        return []
    if channel[6] == 'r':
        return ROT_RANGE
    return POLE_RANGE if control == 'KneePole' else TRANS_RANGE


def index_text(indices):
    if len(indices) == 1:
        return 'point ' + str(indices[0])
    if indices == tuple(range(indices[0], indices[-1] + 1)):
        return 'points %d-%d' % (indices[0], indices[-1])
    return 'points ' + ', '.join(str(i) for i in indices)


def response_for(control, channel):
    if not channel:
        return ROLE[control]
    moves = CLAIMS[control][channel]
    if moves:
        return ROLE[control] + '; moves ' + index_text(moves) + ' at the suggested range'
    return ROLE[control] + '; ' + NO_RESPONSE


def omitted_for(control, channel):
    if channel and not CLAIMS[control][channel]:
        return OMIT_ROW[control]
    return ''


def build_table(stage, rig):
    controls = stage_controls(stage, rig.evaluate(-1))
    rows = []
    for path, channels in sorted(controls.items()):
        control = path.rsplit('/', 1)[1]
        names = channels or ('',)
        for channel in names:
            rows.append({'prim': path,
                         'property': channel,
                         'space': provider_of(stage, path),
                         'units': ROW_UNITS,
                         'suggested_range': list(range_of(control, channel)),
                         'enforced_limit': None,
                         'expected_mesh_response': response_for(control, channel),
                         'omitted_capability': omitted_for(control, channel)})
    return {'version': 1,
            'stage': MODEL,
            'rig': RIG,
            'units': {'metersPerUnit': float(UsdGeom.GetStageMetersPerUnit(stage)),
                      'upAxis': str(UsdGeom.GetStageUpAxis(stage))},
            'controls': rows,
            'omitted_capabilities': list(OMITTED)}


def diag_omits(path):
    return 'coverage gate: ' + TABLE + ' omits ' + path


def diag_names(path):
    return 'coverage gate: ' + TABLE + ' names a control the stage lacks: ' + path


def compare_coverage(controls, rows):
    table_prims = {row['prim'] for row in rows}
    for path in sorted(controls):
        if path not in table_prims:
            raise AssertionError(diag_omits(path))
    for path in sorted(table_prims):
        if path not in controls:
            raise AssertionError(diag_names(path))
    table_channels = {}
    for row in rows:
        if row['property']:
            table_channels.setdefault(row['prim'], set()).add(row['property'])
    for path, channels in sorted(controls.items()):
        listed = table_channels.get(path, set())
        for channel in sorted(set(channels) - listed):
            raise AssertionError('coverage gate: ' + TABLE + ' omits '
                                 + path + ' ' + channel)
        for channel in sorted(listed - set(channels)):
            raise AssertionError('coverage gate: ' + TABLE + ' names ' + path + ' '
                                 + channel + ' the stage lacks')


def gate_coverage(folder, stage, rig):
    table = json.loads((folder / TABLE).read_text())
    assert table == build_table(stage, rig), 'coverage gate: ' + TABLE + ' drifted'
    controls = stage_controls(stage, rig.evaluate(-1))
    compare_coverage(controls, table['controls'])
    pole = controls.get(KNEE_POLE)
    assert pole == POLE_CHANNELS, 'coverage gate: ' + KNEE_POLE + ' channels are ' + str(pole)
    print('PASS coverage: %d controls including %s with %s'
          % (len(controls), KNEE_POLE, ' '.join(POLE_CHANNELS)))


ROW_SHIFT = 1e-3  # every claimed mover measured >= 2.9e-3 at the range end
ROW_REST = 1e-5   # a point left at rest measured <= 3.4e-8 (float noise) here


def gate_responses(folder, stage, rig):
    rows = json.loads((folder / TABLE).read_text())['controls']
    base = points_of(rig.evaluate(-1))
    session = stage.GetSessionLayer()
    probes = 0
    for row in rows:
        path, channel = row['prim'], row['property']
        if not channel:
            continue
        control = path.rsplit('/', 1)[1]
        assert control in CLAIMS and channel in CLAIMS[control], \
            'response gate: no measured claim for ' + path + ' ' + channel
        claimed = CLAIMS[control][channel]
        # Isolate the row: one channel on the session layer, evaluated at rest,
        # then removed before the next row.
        anon = Sdf.Layer.CreateAnonymous('.usda')
        session.subLayerPaths = list(session.subLayerPaths) + [anon.identifier]
        stage.SetEditTarget(Usd.EditTarget(anon))
        stage.GetPrimAtPath(path).GetAttribute(channel).Set(row['suggested_range'][1])
        posed = rig.evaluate(-1)
        stage.SetEditTarget(Usd.EditTarget(session))
        session.subLayerPaths = [s for s in session.subLayerPaths
                                 if s != anon.identifier]
        assert posed.valid and posed.reference_mismatches == 0, \
            'response gate: posing ' + path + ' ' + channel + ' failed the rig'
        moved = displacement(base, points_of(posed))
        for index in claimed:
            assert moved[index] > ROW_SHIFT, \
                'response gate: ' + path + ' ' + channel + ' leaves claimed point ' \
                + str(index) + ' at ' + str(moved[index])
        for index, shift in enumerate(moved):
            if index not in claimed:
                assert shift < ROW_REST, \
                    'response gate: ' + path + ' ' + channel + ' moves point ' \
                    + str(index) + ' the table leaves at rest'
        probes += 1
    print('PASS responses: %d isolated property poses moved exactly the points '
          'their rows claim at the suggested range' % probes)


def gate_pose(stage, rig, keyed):
    assert (KNEE_POLE, POLE_CHANNEL) not in keyed, \
        'pose gate: the posed channel is keyed in ' + ANIMATION
    rest = rig.evaluate(-1)
    assert rest.valid and rest.reference_mismatches == 0, 'pose gate: rest failed'
    # A session-layer edit: the delivered layers on disk stay untouched.
    session = stage.GetSessionLayer()
    anon = Sdf.Layer.CreateAnonymous('.usda')
    session.subLayerPaths = list(session.subLayerPaths) + [anon.identifier]
    stage.SetEditTarget(Usd.EditTarget(anon))
    stage.GetPrimAtPath(KNEE_POLE).GetAttribute(POLE_CHANNEL).Set(POLE_POSE)
    posed = rig.evaluate(-1)
    assert posed.valid and posed.reference_mismatches == 0, 'pose gate: the pose failed'
    moved = displacement(points_of(rest), points_of(posed))
    for index in LEG_WEIGHTED:
        assert moved[index] > MIN_POLE_SHIFT, \
            'pose gate: ' + KNEE_POLE + ' ' + POLE_CHANNEL + ' did not move point ' \
            + str(index) + ' (' + str(moved[index]) + ')'
    assert bits(points_of(rest))[UNWEIGHTED * 24:(UNWEIGHTED + 1) * 24] \
        == bits(points_of(posed))[UNWEIGHTED * 24:(UNWEIGHTED + 1) * 24], \
        'pose gate: unweighted point ' + str(UNWEIGHTED) + ' moved'
    print('PASS pose: session-layer %s %s=%s moved indices %d-%d by %.3f/%.3f and '
          'left index %d bit-identical'
          % (KNEE_POLE, POLE_CHANNEL, POLE_POSE, LEG_WEIGHTED[0], LEG_WEIGHTED[-1],
             moved[LEG_WEIGHTED[0]], moved[LEG_WEIGHTED[-1]], UNWEIGHTED))


def gate_no_write(digests, folder):
    after = layer_digests(folder)
    for name in LAYERS:
        assert after[name] == digests[name], 'no-write gate: ' + name + ' changed on disk'
    print('PASS no-write: %d delivered .usda layers unchanged by the pose step'
          % len(LAYERS))


def rebuild(dest):
    """The recorded checkout-relative sequence: fit -> composition -> animation."""
    if (dest / COMPOSITION).exists():
        raise AssertionError(DIAG_PREBUILT)
    data = json.loads((dest / BINDINGS).read_text())
    data['library'] = library_field(dest)
    (dest / BINDINGS).write_text(json.dumps(data, indent=2))
    compose_assembly(dest / BINDINGS, dest / COMPOSITION)
    assert (dest / COMPOSITION).exists(), 'rebuild gate: the recipe did not create ' \
        + COMPOSITION
    return open_rig(dest, ANIMATION)


def gate_rebuild(root, folder):
    _, delivered = open_rig(folder, ANIMATION)
    dest = root / 'moved' / 'deeper checkout'
    shutil.copytree(str(folder), str(dest))
    (dest / COMPOSITION).unlink()
    _, rebuilt = rebuild(dest)
    rest = delivered.evaluate(-1)
    regenerated = rebuilt.evaluate(-1)
    assert regenerated.valid and regenerated.reference_mismatches == 0, \
        'rebuild gate: the regenerated rest pose failed'
    assert bits(points_of(regenerated)) == bits(points_of(rest)), \
        'rebuild gate: rest differs from the delivered build'
    keyed = rebuilt.evaluate(RUN_FRAME)
    assert keyed.valid and keyed.reference_mismatches == 0, 'rebuild gate: frame %d failed' \
        % RUN_FRAME
    assert bits(points_of(keyed)) == bits(points_of(delivered.evaluate(RUN_FRAME))), \
        'rebuild gate: frame %d differs from the delivered build' % RUN_FRAME
    best = {}
    for label, segment in (('walk', WALK), ('run', RUN)):
        worst = [0.] * len(POINTS)
        for frame in segment:
            pose = rebuilt.evaluate(frame)
            assert pose.valid, 'rebuild gate: frame %d is invalid' % frame
            moved = displacement(points_of(rest), points_of(pose))
            assert all(math.isfinite(c) for p in points_of(pose) for c in p), \
                'rebuild gate: frame %d is not finite' % frame
            assert all(moved[i] > MIN_FRAME_SHIFT for i in DOCUMENTED), \
                'rebuild gate: frame %d leaves a documented index still' % frame
            worst = [max(a, b) for a, b in zip(worst, moved)]
        best[label] = worst
    assert max(best['run'][i] for i in LEG_WEIGHTED) > \
        max(best['walk'][i] for i in LEG_WEIGHTED), \
        'rebuild gate: the run segment does not out-stride the walk segment'
    print('PASS rebuild: relocated copy regenerated %s; rest and frame %d are '
          'bit-identical, every keyed frame moves indices %d-%d'
          % (COMPOSITION, RUN_FRAME, DOCUMENTED[0], DOCUMENTED[-1]))


def gate_missing_pole(root, folder, stage, rig, nodes, attachments):
    rows = json.loads((folder / TABLE).read_text())['controls']
    controls = stage_controls(stage, rig.evaluate(-1))
    without_rows = [row for row in rows if row['prim'] != KNEE_POLE]
    try:
        compare_coverage(controls, without_rows)
    except AssertionError as error:
        assert str(error) == DIAG_TABLE_OMITS, str(error)
    else:
        raise AssertionError('missing-pole-control gate: an incomplete table was accepted')
    # The composition references recreate node prims, so a scaffold that drops
    # the pole prim is what a missing control looks like before composition.
    scratch = root / 'missing-pole'
    scratch.mkdir()
    layer = Sdf.Layer.CreateNew(str(scratch / FIT))
    scaffold(Usd.Stage.Open(layer), nodes, attachments, skip=KNEE_POLE)
    layer.Save()
    pruned = Usd.Stage.Open(str(scratch / FIT))
    pruned_controls = {path: stage_channels(pruned, path)
                       for path in sorted(defined_controls(pruned))}
    try:
        compare_coverage(pruned_controls, rows)
    except AssertionError as error:
        assert str(error) == DIAG_TABLE_NAMES, str(error)
    else:
        raise AssertionError('missing-pole-control gate: a scaffold without '
                             + KNEE_POLE + ' was accepted')
    # Pruning the pole prim from the delivered fit layer is not a control
    # removal: the composition references recreate node prims, so a rebuild
    # returns the control.
    pruned_fit = root / 'pruned-fit'
    shutil.copytree(str(folder), str(pruned_fit))
    (pruned_fit / COMPOSITION).unlink()
    fit_stage = Usd.Stage.Open(str(pruned_fit / FIT))
    assert fit_stage.RemovePrim(KNEE_POLE), \
        'missing-pole-control gate: no ' + KNEE_POLE + ' prim to prune in ' + FIT
    fit_stage.GetRootLayer().Save()
    pruned_stage, pruned_rig = rebuild(pruned_fit)
    assert KNEE_POLE in stage_controls(pruned_stage, pruned_rig.evaluate(-1)), \
        'missing-pole-control gate: the rebuild dropped the pruned-away control'
    # The removal path is the bindings entry: the composer refuses a partial
    # node set instead of building a rig without the control.
    removed = root / 'removed-pole'
    shutil.copytree(str(folder), str(removed))
    (removed / COMPOSITION).unlink()
    data = json.loads((removed / BINDINGS).read_text())
    data['instances']['leg']['nodes'].pop('Control_KneePole')
    (removed / BINDINGS).write_text(json.dumps(data, indent=2))
    try:
        rebuild(removed)
    except ValueError as error:
        assert str(error) == DIAG_BINDING_REMOVAL, str(error)
    else:
        raise AssertionError('missing-pole-control gate: a rebuild after deleting '
                             + KNEE_POLE + "'s bindings entry produced a rig")
    print('PASS missing-pole-control: dropped %s rows or scaffold prims fail with '
          'the control named; pruning only the fit prim returns it after a '
          'rebuild, and deleting its bindings entry fails with %s' %
          (KNEE_POLE, DIAG_BINDING_REMOVAL))


def gate_prebuilt(root, folder):
    dest = root / 'prebuilt-checkout'
    shutil.copytree(str(folder), str(dest))
    before = {name: (dest / name).read_bytes() for name in (BINDINGS, COMPOSITION)}
    try:
        rebuild(dest)
    except AssertionError as error:
        assert str(error) == DIAG_PREBUILT, str(error)
    else:
        raise AssertionError('prebuilt-only-recipe gate: opening the prebuilt stage '
                             'without regenerating the composition was accepted')
    for name in (BINDINGS, COMPOSITION):
        assert (dest / name).read_bytes() == before[name], \
            'prebuilt-only-recipe gate: the refusal changed ' + name
    print('PASS prebuilt-only-recipe: the recipe refuses a present %s and leaves '
          'the bindings and composition byte-identical' % COMPOSITION)


def main():
    with tempfile.TemporaryDirectory(prefix='demo-control-handoff-') as name:
        root = Path(name)
        folder = root / 'character'
        folder.mkdir()
        nodes, ports, attachments = plan()
        assert nodes['leg']['Control_KneePole'] == KNEE_POLE
        write_character(folder, nodes, ports, attachments)
        stage, rig = open_rig(folder, MODEL)
        for instance in CHAIN:
            validate_component(stage, instance, nodes[instance], ports[instance],
                               attachments[instance])
        (folder / TABLE).write_text(json.dumps(build_table(stage, rig), indent=2))
        digests = layer_digests(folder)
        keyed = {(nodes[i][role], channel) for i, role, channel, _ in ANIM_KEYS}
        gate_coverage(folder, stage, rig)
        gate_responses(folder, stage, rig)
        gate_pose(stage, rig, keyed)
        gate_no_write(digests, folder)
        gate_rebuild(root, folder)
        gate_missing_pole(root, folder, stage, rig, nodes, attachments)
        gate_prebuilt(root, folder)


if __name__ == '__main__':
    main()
