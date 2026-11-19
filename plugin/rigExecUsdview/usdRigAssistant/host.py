"""What the assistant needs from the application it runs in: its name,
where it keeps its preferences and chats, and its look.

Preferences are one JSON file, written whole on every change so another
usdview reading it never sees half of it.
"""
import json
import logging
import os
import threading

LOGGER_NAME = 'usdRig.assistant'
APP_NAME = 'usdRig'
VERSION = '1'

USER_DIR = os.environ.get('USDRIG_USER_DIR') or os.path.join(
    os.path.expanduser('~'), '.usdRig')
PREF_DIR = os.path.join(USER_DIR, 'assistant')

logger = logging.getLogger(LOGGER_NAME)


class USER_PREF(object):
    AI_ENABLED = 'ai_assistant_enabled'
    AI_PROVIDER = 'ai_assistant_provider'
    AI_MODELS = 'ai_assistant_models'
    AI_AUTH = 'ai_assistant_auth'
    AI_INCLUDE_SELECTION = 'ai_assistant_include_selection'
    AI_ALLOW_EDITS = 'ai_assistant_allow_edits'
    AI_ALLOW_RUNS = 'ai_assistant_allow_runs'
    AI_MEMORY = 'ai_assistant_memory'
    AI_FOLDERS = 'ai_assistant_folders'
    AI_OPENAI_HOST_ID = 'ai_assistant_openai_host_id'
    AI_OPENAI_CLIENT_ID = 'ai_assistant_openai_client_id'
    AI_OPENAI_ACCOUNTS = 'ai_assistant_openai_accounts'
    AI_LOCAL_URL = 'ai_assistant_local_url'
    AI_CLAUDE_CODE_PATH = 'ai_assistant_claude_code_path'
    AI_CODEX_PATH = 'ai_assistant_codex_path'


def write_whole_file(path, mode, dump):
    """Write path through a temporary file next to it, so a reader only
    ever sees all of it. Gives back True when path was replaced."""
    folder = os.path.dirname(path)
    if folder:
        os.makedirs(folder, exist_ok=True)
    tmp = '{}.{}.tmp'.format(path, os.getpid())
    try:
        with open(tmp, mode) as fp:
            dump(fp)
        os.replace(tmp, path)
        return True
    except OSError as err:
        logger.warning('Could not write {}: {}'.format(path, err))
        try:
            os.remove(tmp)
        except OSError:
            pass
        return False


class Prefs(dict):
    """A dict saved to a JSON file whenever it changes."""

    def __init__(self, path=None):
        super(Prefs, self).__init__()
        self.path = path
        self._lock = threading.Lock()
        if path and os.path.isfile(path):
            try:
                with open(path, 'r', encoding='utf-8') as fp:
                    loaded = json.load(fp)
                if isinstance(loaded, dict):
                    super(Prefs, self).update(loaded)
            except (OSError, ValueError) as err:
                logger.warning('Could not read {}: {}'.format(path, err))

    def _save(self):
        if not self.path:
            return
        with self._lock:
            data = dict(self)
            write_whole_file(self.path, 'w', lambda fp: json.dump(
                data, fp, indent=1, sort_keys=True))

    def __setitem__(self, key, value):
        super(Prefs, self).__setitem__(key, value)
        self._save()

    def __delitem__(self, key):
        super(Prefs, self).__delitem__(key)
        self._save()

    def pop(self, key, *default):
        had = key in self
        value = super(Prefs, self).pop(key, *default)
        if had:
            self._save()
        return value

    def update(self, *args, **kwargs):
        super(Prefs, self).update(*args, **kwargs)
        self._save()


user_prefs = Prefs(os.path.join(PREF_DIR, 'prefs.json'))


# The look, in usdview's dark palette.
GRAPH = '#1d1d1d'
BASE = '#242424'
RAISED = '#2c2c2c'
HOVER = '#353535'
BORDER = '#333333'
BORDER_STRONG = '#474747'
TEXT = '#e6e6e6'
TEXT_DIM = '#a6a6a6'
TEXT_FAINT = '#6e6e6e'
ACCENT = '#909090'
ACCENT_STRONG = '#525252'
ACCENT_DIM = '#3c3c3c'
LINK = '#6fb3ff'
ERROR = '#e07070'

RADIUS = 6
TEXT_PX = 13
SMALL_PX = 11

_TOKENS = {name: value for name, value in globals().items()
           if name.isupper() and isinstance(value, str) and
           value.startswith('#')}


def fill(sheet):
    """A stylesheet with each {NAME} replaced by its color from here."""
    import re
    return re.sub(r'\{([A-Z_]+)\}',
                  lambda m: _TOKENS.get(m.group(1), m.group(0)), sheet)


def default_font(size=None):
    from pxr.Usdviewq.qt import QtGui
    font = QtGui.QFont()
    if size:
        font.setPointSizeF(size)
    return font


DEFAULT_FONT_SIZE = 10


def font(px=TEXT_PX, weight=None, italic=False, mono=False):
    """usdview's font at a pixel height."""
    from pxr.Usdviewq.qt import QtGui, QtWidgets
    app = QtWidgets.QApplication.instance()
    result = QtGui.QFont(app.font()) if app is not None else QtGui.QFont()
    if mono:
        result.setFamilies(['Consolas', 'DejaVu Sans Mono', 'Menlo',
                            'monospace'])
    result.setPixelSize(int(px))
    if weight is not None:
        result.setWeight(weight)
    result.setItalic(italic)
    return result


def color(value, alpha=None):
    """A QColor from a hex string or an (r, g, b, a) tuple."""
    from pxr.Usdviewq.qt import QtGui
    result = QtGui.QColor(*value) if isinstance(value, tuple) \
        else QtGui.QColor(value)
    if alpha is not None:
        result.setAlphaF(alpha)
    return result

# Code colors in the chat, by token kind: (color, 'bold'/'italic' or None).
CODE_STYLES = {
    'keyword': ('#619ea8', None),
    'operator': ('red', None),
    'brace': ('darkGray', None),
    'defclass': ('#d8d8d8', None),
    'string': ('#d162d1', None),
    'string2': ('#79aa63', 'italic'),
    'comment': ('#909090', 'italic'),
    'decorator': ('#6bd13d', None),
    'todo': ('#d9aa00', None),
    'self': ('#ff00e9', 'italic'),
    'numbers': ('brown', None),
}
