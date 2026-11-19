"""What the assistant is told about usdRig and about the selected prims."""
import logging
import re

from . import host

logger = logging.getLogger(host.LOGGER_NAME)

MAX_PRIMS = 8
MAX_ATTRS = 60

SYSTEM = (
    'You are an assistant inside usdview with the usdRig tools loaded. '
    'usdRig evaluates character rigs in OpenUSD. A rig is the namespace '
    'under a RigExecRoot prim. Controls and solvers produce transforms, '
    'and movers (any prim with rigExec:moves) revise transforms, '
    'properties or geometry. Where a prim sits decides when it runs: '
    'sibling order and nesting set every stack, so the hierarchy reads '
    'like the evaluation order. Scope names such as Movers, Solvers, '
    'Controls, Joints and Weights are only conventions; a prim\'s role '
    'comes from its schema and wiring. Animators pose a rig through its '
    'avars (animation variables) and controls. Evaluation never writes to '
    'the stage: posed results go to Hydra. Only authoring edits the '
    'stage, on the current edit target layer.\n\n'
    'Paths are absolute prim paths such as /Biped/Rig/Controls/L_Arm. '
    'Times are frames on the stage\'s time line.'
)


def system_prompt():
    return SYSTEM


def describe_selection(session):
    """Text describing the selected prims, or '' when none."""
    if session is None:
        return ''
    try:
        prims = session.selected_prims()
    except Exception:
        logger.debug('Could not read the selection', exc_info=True)
        return ''
    if not prims:
        return ''
    time = session.time()
    parts = [describe_prim(p, time) for p in prims[:MAX_PRIMS]]
    if len(prims) > MAX_PRIMS:
        parts.append('({} more prims are selected.)'.format(
            len(prims) - MAX_PRIMS))
    return '\n\n'.join(parts)


def describe_prim(prim, time):
    lines = ['{} ({})'.format(prim.GetPath(), prim.GetTypeName() or
                              'no type')]
    schemas = prim.GetAppliedSchemas()
    if schemas:
        lines.append('  applied schemas: ' + ', '.join(schemas))
    attrs = prim.GetAuthoredAttributes()
    for attr in attrs[:MAX_ATTRS]:
        try:
            value = attr.Get(time)
        except Exception:
            value = '?'
        lines.append('  {} {} = {}'.format(
            attr.GetTypeName(), attr.GetName(), _short(value)))
    if len(attrs) > MAX_ATTRS:
        lines.append('  ... {} more attributes'.format(
            len(attrs) - MAX_ATTRS))
    for rel in prim.GetAuthoredRelationships():
        targets = [str(t) for t in rel.GetTargets()]
        lines.append('  rel {} = [{}]'.format(rel.GetName(),
                                              ', '.join(targets)))
    return '\n'.join(lines)


def _short(value, limit=160):
    text = str(value)
    return text if len(text) <= limit else text[:limit] + '...'


_CODE = re.compile(r'```([\w+-]*)[ \t]*\n(.*?)```', re.S)


def last_code_block(text):
    """The last Python block in text, or None."""
    found = None
    for language, code in _CODE.findall(text or ''):
        if language.lower() in ('', 'python', 'py'):
            found = code.rstrip('\n')
    return found
