"""Changes the assistant makes to the stage, as a list of actions.

The same actions serve the change_stage tool and models without tool
calling, which end their answer with a fenced ```usdrig-actions block of
JSON that the panel applies. Every edit goes to the stage's current edit
target. Qt-free.
"""
import json
import logging
import re

from pxr import Gf, Sdf, Usd, Vt

from . import host

logger = logging.getLogger(host.LOGGER_NAME)

FENCE = 'usdrig-actions'
MAX_OUTLINE = 400

INSTRUCTIONS = '''
You can change the open stage. You have no files or tools: the only way to
change it is to end your answer with one fenced block like this, and
usdview applies it straight away. Never say you wrote or saved a file.

```usdrig-actions
[
  {"action": "set_attr", "path": "/Biped/Rig/Controls/L_Arm",
   "name": "avars:ikFk", "value": 1.0, "time": 12},
  {"action": "define_prim", "path": "/Biped/Rig/Movers/Extra",
   "type": "Xform"}
]
```

Actions (paths are absolute prim paths such as /Root/Child):
{reference}
Give only the block for the changes the user asked for, and a short
sentence saying what you changed.
'''

ACTION_REFERENCE = '''
- define_prim: path, and optionally type (a schema type name such as
  Xform, Scope or RigExecRoot); missing parents are made as plain prims
- override_prim: path (an over, to add opinions to a prim from elsewhere)
- remove_prim: path (removes its spec on the edit target only)
- rename_prim: path, name (the new short name)
- reparent_prim: path, parent (the new parent's path)
- set_active: path, value (true or false)
- reorder_children: path, names (child names, first first)
- set_attr: path, name, value, and optionally type (an Sdf value type
  such as double, float3, token, matrix4d, string[]; needed only when the
  attribute is new), time (a frame: writes a key there instead of the
  default), custom (default true for new attributes)
- clear_attr: path, name, and optionally time (clears one key)
- block_attr: path, name (blocks its value)
- set_connections: path, name, sources (property paths such as
  /Rig/Ctl.outputs:value; an empty list clears them)
- set_relationship: path, name, targets (prim or property paths)
- apply_schema: path, schema (an API schema name); remove_schema likewise
- set_variant: path, set, variant
- add_reference: path, asset (a file path), and optionally prim (a prim
  path in that file)
- add_sublayer: asset (a file path), and optionally position (0 is the
  strongest)
- set_edit_target: layer (an identifier or file name from the layer
  stack)
- set_metadata: path, key, value (and optionally name for an attribute's)
'''

_BLOCK = re.compile(r'```' + FENCE + r'[ \t]*\n(.*?)```', re.S)


def action_reference():
    return ACTION_REFERENCE.strip()


def instructions():
    return INSTRUCTIONS.strip().replace('{reference}', action_reference())


def find_actions(text):
    """The actions in a reply, and any problem reading them."""
    actions, problems = [], []
    for block in _BLOCK.findall(text or ''):
        try:
            found = json.loads(block)
        except ValueError as err:
            problems.append('Could not read the changes: {}'.format(err))
            continue
        if isinstance(found, dict):
            found = found.get('actions', [found])
        if not isinstance(found, list):
            problems.append('The changes were not a list.')
            continue
        for action in found:
            if isinstance(action, dict) and action.get('action'):
                actions.append(action)
            else:
                problems.append('Skipped a change with no action: '
                                '{}'.format(action))
    return actions, problems


def without_actions(text, summary=None):
    """text with its action blocks swapped for a short summary."""
    if not _BLOCK.search(text or ''):
        return text
    return _BLOCK.sub(lambda m: summary or '', text).rstrip()


def describe_stage(stage):
    """The layer stack and an outline of the prims, so a model without
    tools knows what is there before it changes anything."""
    if stage is None:
        return ''
    target = stage.GetEditTarget().GetLayer()
    lines = ['Layers, strongest first:']
    for layer in stage.GetLayerStack(includeSessionLayers=False):
        mark = ' (edit target)' if layer == target else ''
        lines.append('  {}{}'.format(layer.identifier, mark))
    prims = [p for p in stage.Traverse()]
    lines.append('Prims ({}):'.format(len(prims)))
    for prim in prims[:MAX_OUTLINE]:
        lines.append('  {} {}'.format(prim.GetPath(), prim.GetTypeName()))
    if len(prims) > MAX_OUTLINE:
        lines.append('  ... {} more'.format(len(prims) - MAX_OUTLINE))
    return '\n'.join(lines)


class ActionError(Exception):
    pass


def _matrix(value):
    rows = [list(r) for r in value]
    if len(rows) == 16 and not isinstance(rows[0], list):
        rows = [value[i:i + 4] for i in range(0, 16, 4)]
    return Gf.Matrix4d(*[float(v) for row in rows for v in row])


def to_value(value, type_name):
    """value from JSON, made into what an attribute of type_name holds."""
    if value is None:
        return None
    vtype = Sdf.ValueTypeNames.Find(str(type_name))
    if not vtype:
        raise ActionError('There is no value type {}'.format(type_name))
    python = vtype.type.pythonClass
    name = str(vtype)
    if name.startswith('matrix4d') and not name.endswith('[]'):
        return _matrix(value)
    if name == 'matrix4d[]':
        return Vt.Matrix4dArray([_matrix(v) for v in value])
    if name in ('asset',):
        return Sdf.AssetPath(str(value))
    if python is None:
        return value
    try:
        if isinstance(value, (list, tuple)) and not name.endswith('[]'):
            return python(*value)
        return python(value)
    except Exception as err:
        raise ActionError('{} is not a {}: {}'.format(
            json.dumps(value)[:80], name, err))


def to_json(value):
    """A USD value as plain JSON."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (Sdf.AssetPath,)):
        return value.path
    if isinstance(value, (Sdf.Path,)):
        return str(value)
    if isinstance(value, Gf.Matrix4d):
        return [list(value.GetRow(i)) for i in range(4)]
    try:
        items = list(value)
    except TypeError:
        return str(value)
    if len(items) > 200:
        return [to_json(v) for v in items[:200]] + [
            '... {} more'.format(len(items) - 200)]
    return [to_json(v) for v in items]


class Applier(object):
    """Applies actions to stage, each one to the current edit target.

    :param stage: the Usd.Stage
    :param select: called with a list of paths for the select action
    """

    def __init__(self, stage, select=None):
        self.stage = stage
        self.select = select

    def apply(self, actions):
        # No Sdf.ChangeBlock: each action reads the stage the one before it
        # made, and a stage does not recompose inside a block.
        done, failed = [], []
        for action in actions:
            name = str(action.get('action') or '')
            handler = getattr(self, 'do_' + name, None)
            if handler is None:
                failed.append('There is no action {}'.format(name))
                continue
            try:
                done.append(handler(action))
            except ActionError as err:
                failed.append('{}: {}'.format(name, err))
            except Exception as err:
                logger.exception('Action {} failed'.format(name))
                failed.append('{}: {}'.format(name, str(err).strip()))
        return done, failed

    # Helpers
    def path(self, action, key='path'):
        text = str(action.get(key) or '').strip()
        if not text:
            raise ActionError('"{}" is needed'.format(key))
        if not text.startswith('/'):
            text = '/' + text
        if not Sdf.Path.IsValidPathString(text):
            raise ActionError('{} is not a path'.format(text))
        return Sdf.Path(text)

    def prim(self, action, key='path'):
        path = self.path(action, key)
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            raise ActionError('There is no prim {}'.format(path))
        return prim

    def attr(self, action):
        prim = self.prim(action)
        name = str(action.get('name') or '')
        attr = prim.GetAttribute(name)
        if not attr:
            raise ActionError('{} has no attribute {}'.format(
                prim.GetPath(), name))
        return attr

    def time(self, action):
        if action.get('time') is None:
            return Usd.TimeCode.Default()
        return Usd.TimeCode(float(action['time']))

    @staticmethod
    def when(time):
        return '' if time.IsDefault() else ' at frame {:g}'.format(
            time.GetValue())

    # Prims
    def do_define_prim(self, action):
        path = self.path(action)
        type_name = str(action.get('type') or '')
        prim = self.stage.DefinePrim(path, type_name)
        if not prim:
            raise ActionError('Could not define {}'.format(path))
        return 'Defined {}{}'.format(path, ' as ' + type_name
                                     if type_name else '')

    def do_override_prim(self, action):
        path = self.path(action)
        if not self.stage.OverridePrim(path):
            raise ActionError('Could not override {}'.format(path))
        return 'Added an over for {}'.format(path)

    def do_remove_prim(self, action):
        path = self.path(action)
        if not self.stage.RemovePrim(path):
            raise ActionError('{} has no spec on the edit target to '
                              'remove; deactivate it instead'.format(path))
        return 'Removed {}'.format(path)

    def _namespace_edit(self, prim, new_path):
        editor_type = getattr(Usd, 'NamespaceEditor', None)
        if editor_type is None:
            raise ActionError('This USD has no namespace editor')
        editor = editor_type(self.stage)
        if not editor.MovePrimAtPath(prim.GetPath(), new_path):
            raise ActionError('Cannot move {} to {}'.format(
                prim.GetPath(), new_path))
        if not editor.ApplyEdits():
            raise ActionError('Moving {} to {} failed'.format(
                prim.GetPath(), new_path))

    def do_rename_prim(self, action):
        prim = self.prim(action)
        name = str(action.get('name') or '')
        if not Sdf.Path.IsValidIdentifier(name):
            raise ActionError('{} is not a valid name'.format(name))
        old = prim.GetPath()
        self._namespace_edit(prim, old.GetParentPath().AppendChild(name))
        return 'Renamed {} to {}'.format(old, name)

    def do_reparent_prim(self, action):
        prim = self.prim(action)
        parent = self.path(action, 'parent')
        old = prim.GetPath()
        self._namespace_edit(prim, parent.AppendChild(old.name))
        return 'Moved {} under {}'.format(old, parent)

    def do_set_active(self, action):
        path = self.path(action)
        prim = self.stage.GetPrimAtPath(path) or self.stage.OverridePrim(
            path)
        value = bool(action.get('value', True))
        prim.SetActive(value)
        return '{} {}'.format('Activated' if value else 'Deactivated', path)

    def do_reorder_children(self, action):
        prim = self.prim(action)
        names = [str(n) for n in action.get('names') or []]
        prim.SetChildrenReorder(names)
        return 'Reordered the children of {}'.format(prim.GetPath())

    # Properties
    def do_set_attr(self, action):
        prim = self.prim(action)
        name = str(action.get('name') or '')
        if not name:
            raise ActionError('"name" is needed')
        attr = prim.GetAttribute(name)
        if not attr:
            type_name = action.get('type')
            if not type_name:
                raise ActionError('{} has no attribute {}; give a "type" to '
                                  'make it'.format(prim.GetPath(), name))
            vtype = Sdf.ValueTypeNames.Find(str(type_name))
            if not vtype:
                raise ActionError('There is no value type {}'.format(
                    type_name))
            attr = prim.CreateAttribute(name, vtype,
                                        bool(action.get('custom', True)))
        value = to_value(action.get('value'), attr.GetTypeName())
        time = self.time(action)
        if not attr.Set(value, time):
            raise ActionError('Could not set {}'.format(attr.GetPath()))
        return 'Set {} to {}{}'.format(
            attr.GetPath(), json.dumps(to_json(value))[:80], self.when(time))

    def do_clear_attr(self, action):
        attr = self.attr(action)
        time = self.time(action)
        if time.IsDefault():
            attr.Clear()
        else:
            attr.ClearAtTime(time)
        return 'Cleared {}{}'.format(attr.GetPath(), self.when(time))

    def do_block_attr(self, action):
        attr = self.attr(action)
        attr.Block()
        return 'Blocked {}'.format(attr.GetPath())

    def do_set_connections(self, action):
        attr = self.attr(action)
        sources = [Sdf.Path(str(s)) for s in action.get('sources') or []]
        attr.SetConnections(sources)
        return 'Connected {} to {}'.format(
            attr.GetPath(), ', '.join(str(s) for s in sources) or 'nothing')

    def do_set_relationship(self, action):
        prim = self.prim(action)
        name = str(action.get('name') or '')
        rel = prim.GetRelationship(name) or prim.CreateRelationship(name)
        targets = [Sdf.Path(str(t)) for t in action.get('targets') or []]
        rel.SetTargets(targets)
        return 'Pointed {} at {}'.format(
            rel.GetPath(), ', '.join(str(t) for t in targets) or 'nothing')

    def do_apply_schema(self, action):
        prim = self.prim(action)
        schema = str(action.get('schema') or '')
        if not prim.AddAppliedSchema(schema):
            raise ActionError('Could not apply {}'.format(schema))
        return 'Applied {} to {}'.format(schema, prim.GetPath())

    def do_remove_schema(self, action):
        prim = self.prim(action)
        schema = str(action.get('schema') or '')
        if not prim.RemoveAppliedSchema(schema):
            raise ActionError('Could not remove {}'.format(schema))
        return 'Removed {} from {}'.format(schema, prim.GetPath())

    def do_set_variant(self, action):
        prim = self.prim(action)
        vset = prim.GetVariantSets().GetVariantSet(str(action.get('set')))
        if not vset or not vset.SetVariantSelection(
                str(action.get('variant'))):
            raise ActionError('Could not select {} in {}'.format(
                action.get('variant'), action.get('set')))
        return 'Selected {} {} on {}'.format(
            action.get('set'), action.get('variant'), prim.GetPath())

    def do_add_reference(self, action):
        path = self.path(action)
        prim = self.stage.GetPrimAtPath(path) or self.stage.OverridePrim(
            path)
        asset = str(action.get('asset') or '')
        if not asset:
            raise ActionError('"asset" is needed')
        target = action.get('prim')
        if target:
            prim.GetReferences().AddReference(asset, Sdf.Path(str(target)))
        else:
            prim.GetReferences().AddReference(asset)
        return 'Referenced {} on {}'.format(asset, path)

    def do_add_sublayer(self, action):
        asset = str(action.get('asset') or '')
        if not asset:
            raise ActionError('"asset" is needed')
        layer = self.stage.GetEditTarget().GetLayer()
        position = int(action.get('position', 0))
        layer.subLayerPaths.insert(position, asset)
        return 'Added the sublayer {} to {}'.format(asset, layer.identifier)

    def do_set_edit_target(self, action):
        wanted = str(action.get('layer') or '')
        for layer in self.stage.GetLayerStack():
            names = {layer.identifier, layer.realPath or '',
                     (layer.realPath or '').replace('\\', '/').split(
                         '/')[-1]}
            if wanted in names:
                self.stage.SetEditTarget(Usd.EditTarget(layer))
                return 'Edits now go to {}'.format(layer.identifier)
        raise ActionError('No layer {} in the layer stack'.format(wanted))

    def do_set_metadata(self, action):
        prim = self.prim(action)
        target = prim
        if action.get('name'):
            target = prim.GetProperty(str(action['name']))
            if not target:
                raise ActionError('{} has no property {}'.format(
                    prim.GetPath(), action['name']))
        key = str(action.get('key') or '')
        if not target.SetMetadata(key, action.get('value')):
            raise ActionError('Could not set {}'.format(key))
        return 'Set {} on {}'.format(key, target.GetPath())

    def do_select(self, action):
        paths = [str(self.path({'path': p}))
                 for p in action.get('paths') or []]
        if self.select is not None:
            self.select(paths)
        return 'Selected {}'.format(', '.join(paths) or 'nothing')
