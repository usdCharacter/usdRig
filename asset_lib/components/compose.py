# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""Bind reusable rig components to a character's hierarchy and fit layer.

Run in the environment from bin/_env.bat or bin/_env.sh:
    python asset_lib/components/compose.py BINDINGS.json OUTPUT.components.usda

The bindings document supplies a fit layer, component node destinations,
external prim ports, and per-mesh attachment wiring. Relative paths resolve
against that document. Requires the project's OpenUSD and RigExec schemas.
"""
import argparse
import json
import os
from pathlib import Path

from pxr import Sdf, Usd

LIBRARY = Path(__file__).resolve().parent


def _manifest(component, library):
    if component not in ('leg', 'spine', 'torso', 'arm', 'head', 'eye', 'mouth'):
        raise ValueError('Unknown component: ' + component)
    folder = Path(library) / component
    data = json.loads((folder / 'component.json').read_text())
    if data['version'] != 1:
        raise ValueError('Unsupported component version')
    return folder, data


def _endpoint(symbol, nodes, ports):
    category, value = symbol.split(':', 1)
    role, dot, prop = value.partition('.')
    paths = nodes if category == 'node' else ports
    path = Sdf.Path(paths[role])
    if not path.IsAbsolutePath() or not path.IsPrimPath():
        raise ValueError('Expected absolute prim path for ' + symbol)
    return path.AppendProperty(prop) if dot else path


def _reference(stage, path, asset, template):
    target = stage.GetEditTarget().GetLayer()
    if not target.realPath:
        raise ValueError('Component assembly requires a file-backed edit layer')
    try:
        relative = os.path.relpath(asset, Path(target.realPath).parent).replace('\\', '/')
    except ValueError:
        # Windows cannot express a relative reference across drive letters.
        relative = Path(asset).resolve().as_posix()
    prim = stage.OverridePrim(path)
    prim.GetReferences().SetReferences([Sdf.Reference(relative, Sdf.Path(template))])
    return prim


def _relationships(prim, relationships, resolve):
    for name, data in relationships.items():
        rel = prim.CreateRelationship(name, custom=False)
        rel.SetTargets([resolve(t) for t in data['targets']])
        if 'readPhase' in data:
            rel.SetMetadata('rigExecReadPhase', data['readPhase'])


def bind_component(stage, component, nodes, ports, attachments=(), library=LIBRARY):
    """Reference a component into a fitted stage, preserving caller hierarchy.

    `nodes` maps every manifest node role to an absolute destination prim path.
    `ports` maps manifest external ports to absolute prim paths. `attachments`
    supplies repeatable deformation/weight templates with explicit relationship
    targets. Fit values must be authored in a stronger character layer. Call
    validate_component after binding all components in an assembly.
    """
    folder, manifest = _manifest(component, library)
    if set(nodes) != set(manifest['nodes']):
        raise ValueError('Node roles must exactly match the ' + component + ' manifest')
    if set(ports) != set(manifest['ports']):
        raise ValueError('Ports must exactly match the ' + component + ' manifest')
    destinations = list(nodes.values()) + [a['path'] for a in attachments]
    if len(destinations) != len(set(destinations)):
        raise ValueError('Component destinations must be unique')
    for path in destinations:
        sdfpath = Sdf.Path(path)
        if not sdfpath.IsAbsolutePath() or not sdfpath.IsPrimPath():
            raise ValueError('Expected absolute prim destination: ' + path)
    asset = folder / manifest['asset']
    resolve = lambda value: _endpoint(value, nodes, ports)
    for role, definition in manifest['nodes'].items():
        prim = _reference(stage, nodes[role], asset, definition['prim'])
        _relationships(prim, definition['relationships'], resolve)
        for name, values in definition['connections'].items():
            prim.GetAttribute(name).SetConnections([resolve(value) for value in values])
    for attachment in attachments:
        definition = manifest['templates'][attachment['template']]
        if set(attachment['relationships']) != set(definition['relationships']):
            raise ValueError('Attachment relationships do not match ' + attachment['template'])
        prim = _reference(stage, attachment['path'], asset, definition['prim'])
        _relationships(prim, attachment['relationships'], Sdf.Path)


def validate_component(stage, component, nodes, ports, attachments=(), library=LIBRARY):
    """Reject missing fit values, broken wiring, or an incompatible hierarchy."""
    _, manifest = _manifest(component, library)
    definitions = [(nodes[role], data) for role, data in manifest['nodes'].items()]
    definitions += [(a['path'], manifest['templates'][a['template']]) for a in attachments]
    for path, definition in definitions:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsDefined() or prim.GetTypeName() != definition['type']:
            raise ValueError('Missing or incompatible node: ' + path)
        for field in definition['fit']:
            attr = prim.GetAttribute(field['name'])
            if not attr or not attr.HasAuthoredValueOpinion() or str(attr.GetTypeName()) != field['type']:
                raise ValueError('Missing or incompatible fit: ' + path + '.' + field['name'])
        if 'parent' in definition:
            expected = _endpoint(definition['parent'], nodes, ports)
            parent = prim.GetParent()
            while parent and parent.GetTypeName() not in ('RigExecControl', 'RigExecJoint', 'RigExecRoot'):
                parent = parent.GetParent()
            if not parent or parent.GetPath() != expected:
                raise ValueError('Provider parent differs for ' + path + '; expected ' + str(expected))
        for prop in prim.GetAuthoredProperties():
            targets = prop.GetTargets() if isinstance(prop, Usd.Relationship) else prop.GetConnections()
            for target in targets:
                if not stage.GetObjectAtPath(target):
                    raise ValueError('Unresolved target: ' + str(target))


def compose_assembly(bindings_path, output):
    """Rebuild a reference/wiring layer from bindings and its existing fit."""
    bindings_path = Path(bindings_path).resolve()
    output = Path(output).resolve()
    data = json.loads(bindings_path.read_text())
    if data['version'] != 1:
        raise ValueError('Unsupported bindings version')
    library = (bindings_path.parent / data['library']).resolve()
    fit_path = (bindings_path.parent / data['fit']).resolve()
    if output == fit_path or output == bindings_path:
        raise ValueError('Output must differ from the fit and bindings inputs')
    fit = Sdf.Layer.FindOrOpen(str(fit_path))
    if not fit:
        raise ValueError('Cannot open fit layer: ' + str(fit_path))
    layer = Sdf.Layer.FindOrOpen(str(output)) if output.exists() else Sdf.Layer.CreateNew(str(output))
    layer.TransferContent(fit)
    # Retain the fitted hierarchy and child ordering; the fit owns values.
    def strip(path):
        spec = layer.GetPrimAtPath(path)
        if spec:
            for prop in list(spec.properties):
                spec.RemoveProperty(prop)
    layer.Traverse(Sdf.Path.absoluteRootPath, strip)
    layer.documentation = 'Rig component references and explicit port wiring; values live in the character fit.'
    assembly = Sdf.Layer.CreateAnonymous()
    assembly.subLayerPaths = [str(fit_path), str(output)]
    stage = Usd.Stage.Open(assembly)
    stage.SetEditTarget(layer)
    all_paths = []
    for instance in data['instances'].values():
        all_paths.extend(instance['nodes'].values())
        all_paths.extend(a['path'] for a in instance.get('attachments', []))
    if len(all_paths) != len(set(all_paths)):
        raise ValueError('Assembly components overlap')
    for instance in data['instances'].values():
        bind_component(stage, library=library, **instance)
    # Geometry ports are provided by the model that includes this rig layer;
    # validate_component is used on that complete stage by the caller.
    layer.Save()
    return layer


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bindings')
    parser.add_argument('output')
    args = parser.parse_args()
    compose_assembly(args.bindings, args.output)
