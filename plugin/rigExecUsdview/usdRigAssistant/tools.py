"""The tools the assistant uses on the stage and rigs open in usdview.

The same tools are given to every provider: through the provider's own
tool calling for API keys, ChatGPT, Grok and local models, and through the
MCP server (see mcp_server) for a Claude Code or Codex sign in. Each tool
runs on usdview's main thread, as the stage is not thread safe, and tells
the panel what it did so the chat can show it.

Looking needs nothing turned on. Changing the stage (posing included)
needs "Edit stage", and running Python needs "Run code", both toggles
under the message box. The changes one reply makes are recorded on the RigExec
undo stack in the order they were made, and share a step while that step is
still the newest thing there.

Files are read as in Claude Code: the open stage's folder is free to read,
and anywhere else the user is asked first (see memory).
"""
import base64
import collections
import contextlib
import ctypes
import enum
import fnmatch
import io
import json
import logging
import os
import sys
import threading
import time
import traceback

from pxr import Sdf, Usd, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtWidgets

from . import files, host, session as session_module, stage_actions
from .memory import Memory, inside, norm

logger = logging.getLogger(host.LOGGER_NAME)

READ, SHOW, EDIT, RUN, MEMORY = 'read', 'show', 'edit', 'run', 'memory'
# What the user answers when asked to let the assistant read a folder.
ONCE, ALWAYS, DENY = 'once', 'always', 'deny'
_UNGROUPED = object()
MAX_TEXT = 20000
MAX_LOG_LINES = 400
CALL_TIMEOUT = 600
_STOPPED = 'The user stopped the reply.'
_STALE = 'The reply belongs to a document that is no longer current.'
_PENDING_EXPIRY = ('The tool expired before it started; it did not run.', True)
_RUNNING_EXPIRY = ('The tool already started; its outcome is unknown. '
                   'Do not retry automatically.', True)
RIG_ROOT = 'RigExecRoot'
CAPTURE_DIR = os.path.join(host.PREF_DIR, 'captures')

INSTRUCTIONS = '''
You have tools for the stage and rigs open in usdview. Look before you
answer: read the prims the question is about rather than guessing, and
search when you do not know where something is. Paths are absolute prim
paths such as /Biped/Rig/Controls/L_Arm.

To pose or animate, use list_avars to see a control's channels, then
set_avars. Avars a mover recomputes are listed as hidden: setting them
changes nothing you can see, so pose the controls that drive them. Use
change_stage for other edits (prims, attributes, connections,
relationships, references, layers). Every edit goes to the stage's edit
target. When you have change_stage or set_avars, make the changes; never
only describe them. Check what a change did when it matters, with
get_prim, evaluate_rig or capture_viewport.

check_rig and evaluate_rig compile the rig on their own, apart from the
viewport, and author nothing. get_live_evaluator reads the viewport's own
evaluator.

When you have run_python, you can run Python in usdview, with
usdviewApi, stage, Usd, Sdf, Gf, UsdGeom and rigexec ready. Print what
you want to see. Prefer the other tools for edits: they are clearer in
the chat.

You can read files on this computer with list_files, read_file and
search_files. The open stage's folder is free to read; anywhere else the
user is asked first, so give a short reason. Use inspect_import to see
what a module offers, rather than guessing at its functions.

When the user tells you something to keep in mind for later chats, or
asks you to remember it, save it with remember as one short note.
Keep your answer short and say what you changed.
'''


def instructions():
    return INSTRUCTIONS.strip()


class Tool(object):
    def __init__(self, name, kind, description, schema, handler, label,
                 needs_stage=True):
        self.name = name
        self.kind = kind
        self.description = description
        self.schema = schema
        self.handler = handler
        # How the chat says what it did, from the arguments.
        self.label = label
        self.needs_stage = needs_stage


class Image(object):
    """A tool result with a picture: text for every provider, and the PNG
    for those that can look at it."""

    def __init__(self, text, png):
        self.text = text
        self.png = png


def _obj(properties=None, required=None):
    schema = {'type': 'object', 'properties': properties or {}}
    if required:
        schema['required'] = list(required)
    return schema


PATH = {'type': 'string', 'description': 'An absolute prim path, such as '
                                         '/Biped/Rig'}
PATHS = {'type': 'array', 'items': {'type': 'string'},
         'description': 'Absolute prim paths'}
NAMES = {'type': 'array', 'items': {'type': 'string'},
         'description': 'Prim paths, or names such as elbow_l_def'}
TIME = {'type': 'number', 'description': 'A frame; the current frame when '
                                         'left out'}
READ_REASON = {'type': 'string', 'description': 'Why you want to read '
                                                'there, in one short '
                                                'sentence, shown to the '
                                                'user if they are asked'}
READ_ABOUT = {'type': 'string', 'description': 'What the folder holds, in '
                                               'a few words, such as "the '
                                               'rig library". Remembered '
                                               'if the user always allows '
                                               'it'}


def _place(path):
    """A file or folder, short enough for the chat."""
    if not path:
        return ''
    path = str(path).rstrip('/\\')
    return os.path.basename(path) or path


class _Tee(object):
    """Passes writes on to a stream, keeping a copy."""

    def __init__(self, stream, kept):
        self.stream = stream
        self.kept = kept

    def write(self, text):
        self.kept.append(text)
        if self.stream is not None:
            return self.stream.write(text)
        return len(text)

    def __getattr__(self, name):
        return getattr(self.stream, name)


class RecentLog(logging.Handler):
    """Keeps the last lines Python logged in usdview, for get_output_log
    and to give back what a run logged."""

    def __init__(self, size=2000):
        super(RecentLog, self).__init__(level=logging.INFO)
        self.lines = collections.deque(maxlen=size)
        self.count = 0
        self.setFormatter(logging.Formatter(
            '%(levelname)s %(name)s: %(message)s'))

    def emit(self, record):
        try:
            text = self.format(record)
        except Exception:
            return
        self.count += 1
        self.lines.append((self.count, record.levelno, text))

    def since(self, count):
        return [line for line in self.lines if line[0] > count]


_log = None


def recent_log():
    """usdview's recent Python log, started on first use."""
    global _log
    if _log is None:
        _log = RecentLog()
        root = logging.getLogger()
        if _log not in root.handlers:
            root.addHandler(_log)
    return _log


def _clip(text, limit=MAX_TEXT):
    if len(text) <= limit:
        return text
    return text[:limit] + '\n... ({} more characters)'.format(
        len(text) - limit)


def _short(paths, most=3):
    paths = list(paths or [])
    if len(paths) <= most:
        return ', '.join(paths) or 'nothing'
    return '{} and {} more'.format(', '.join(paths[:most]),
                                   len(paths) - most)


def _value(value):
    return stage_actions.to_json(value)


class ToolError(Exception):
    pass


class _JobState(enum.Enum):
    PENDING = enum.auto()
    RUNNING = enum.auto()
    DONE = enum.auto()
    EXPIRED = enum.auto()
    CANCELLED = enum.auto()


class _Job:
    def __init__(self, name, args, lock):
        self.name = name
        self.args = args
        self.deadline = time.monotonic() + CALL_TIMEOUT
        self.state = _JobState.PENDING
        self.result = None
        self.ready = threading.Condition(lock)


class Toolbox(QtCore.QObject):
    """The tools for one reply.

    :param session: the session.Session of the open usdview, or None
    :param can_edit: True to offer change_stage, set_avars and saving
    :param can_run: True to offer run_python
    :param memory: the Memory to read and save notes and folders in
    :param chat_folders: a set of the folders allowed for this chat only,
        kept by the chat and added to as the user allows more
    :param ask_access: called with (folder, reason) when the assistant
        wants to read somewhere new; gives back ONCE, ALWAYS or DENY.
        Without it, nowhere new can be read.
    :param document_current: optional owner-thread generation check
    :param edit_allowed: optional owner-thread check that "Edit stage" is
        still on; it can only narrow the can_edit grant made at Send
    :param run_allowed: the same for "Run code" and can_run
    """
    # {'id', 'name', 'kind', 'label', 'state': 'running', 'done' or
    # 'failed', 'detail'}
    activity = QtCore.Signal(dict)
    # A call ended and none is left: the owner can settle the reply.
    settled = QtCore.Signal()
    _invoke = QtCore.Signal(object)

    def __init__(self, session, can_edit=False, can_run=False, parent=None,
                 memory=None, chat_folders=None, ask_access=None,
                 document_current=None, edit_allowed=None, run_allowed=None):
        super(Toolbox, self).__init__(parent)
        self.session = session
        self._origin_stage = session.stage if session is not None else None
        self._have_stage = self._origin_stage is not None
        self._document_current = document_current
        self.edit_allowed = edit_allowed
        self.run_allowed = run_allowed
        self.can_edit = can_edit
        self.can_run = can_run
        self.memory = memory if memory is not None else Memory()
        self.chat_folders = chat_folders if chat_folders is not None \
            else set()
        self.ask_access = ask_access
        # Folders the user said no to in this reply, so it is not asked
        # again and again.
        self.denied = set()
        self.cancelled = threading.Event()
        self._job_lock = threading.Lock()
        self._jobs = set()
        self.done = []
        self.failed = []
        # The reply's undo step on the editor's history: the first change
        # opens it and the ones after join it while it is still the newest
        # thing there. It reaches the stack only once a change is in it.
        self._history_step = None
        self._history_pushed = False
        self.calls = 0
        self._running = False
        self._namespace = None
        self._invoke.connect(self._run_job, QtCore.Qt.QueuedConnection)
        self.tools = collections.OrderedDict(
            (tool.name, tool) for tool in self._all_tools())
        recent_log()

    @property
    def stage(self):
        return self._origin_stage

    def _on_owner_thread(self):
        return QtCore.QThread.currentThread() is self.thread()

    def is_current(self):
        """Check document identity independently of cancellation."""
        if not self._on_owner_thread():
            return False
        if self._document_current is not None and not self._document_current():
            return False
        stage = self.session.stage if self.session is not None else None
        return stage is self._origin_stage

    def _require_current(self):
        if self.cancelled.is_set():
            raise ToolError(_STOPPED)
        if not self.is_current():
            raise ToolError(_STALE)

    def may_edit(self):
        """Edit consent: granted at Send and still on. Asked on the owner
        thread, so turning "Edit stage" off takes the edit tools away."""
        return self.can_edit and (self.edit_allowed is None or
                                  bool(self.edit_allowed()))

    def may_run(self):
        return self.can_run and (self.run_allowed is None or
                                 bool(self.run_allowed()))

    def outstanding_calls(self):
        """Calls this reply made that have not ended. The owner thread
        delivers the ones other threads asked for."""
        with self._job_lock:
            return len(self._jobs) + (1 if self._running else 0)

    def _settle_if_idle(self):
        with self._job_lock:
            idle = not self._jobs and not self._running
        if idle:
            self.settled.emit()

    # What is offered
    def available(self):
        allowed = {READ, SHOW, MEMORY}
        if self.can_edit:
            allowed.add(EDIT)
        if self.can_run:
            allowed.add(RUN)
        return [t for t in self.tools.values() if t.kind in allowed and
                (self._have_stage or not t.needs_stage)]

    def anthropic_tools(self):
        return [{'name': t.name, 'description': t.description,
                 'input_schema': t.schema} for t in self.available()]

    def responses_tools(self):
        return [{'type': 'function', 'name': t.name,
                 'description': t.description, 'parameters': t.schema}
                for t in self.available()]

    def chat_completions_tools(self):
        return [{'type': 'function',
                 'function': {'name': t.name, 'description': t.description,
                              'parameters': t.schema}}
                for t in self.available()]

    def mcp_tools(self):
        return [{'name': t.name, 'description': t.description,
                 'inputSchema': t.schema} for t in self.available()]

    # Calling
    def call(self, name, args):
        """Run a tool from any thread. Gives back (text, is_error)."""
        result, failed = self.call_result(name, args)
        if isinstance(result, Image):
            return result.text, failed
        return result, failed

    def call_content(self, name, args):
        """Run a tool from any thread. Gives back (MCP content, is_error)."""
        result, failed = self.call_result(name, args)
        if isinstance(result, Image):
            return [{'type': 'text', 'text': result.text},
                    {'type': 'image', 'mimeType': 'image/png',
                     'data': base64.b64encode(result.png).decode('ascii')}
                    ], failed
        return [{'type': 'text', 'text': result}], failed

    def call_result(self, name, args):
        if isinstance(args, str):
            try:
                args = json.loads(args or '{}')
            except ValueError as err:
                return 'The arguments were not JSON: {}'.format(err), True
        args = args if isinstance(args, dict) else {}
        if self._on_owner_thread():
            was_running = self._running
            self._running = True
            try:
                return self._call(name, args)
            finally:
                self._running = was_running
                if not was_running:
                    self._settle_if_idle()
        if not isinstance(QtWidgets.QApplication.instance(), QtWidgets.QApplication):
            return 'The toolbox cannot run off its owner thread without Qt.', True
        job = _Job(name, args, self._job_lock)
        with self._job_lock:
            if self.cancelled.is_set():
                return _STOPPED, True
            self._jobs.add(job)
        try:
            self._invoke.emit(job)
        except Exception as err:
            self._fail_pending_job(job, ('Could not queue the tool: {}'.format(err), True))
        with job.ready:
            while job.state in (_JobState.PENDING, _JobState.RUNNING):
                remaining = job.deadline - time.monotonic()
                if remaining <= 0:
                    if job.state is _JobState.RUNNING:
                        return _RUNNING_EXPIRY
                    self._complete_job_locked(job, _JobState.EXPIRED, _PENDING_EXPIRY)
                    break
                job.ready.wait(remaining)
            return job.result

    def _complete_job_locked(self, job, state, result):
        job.state, job.result = state, result
        self._jobs.remove(job)
        job.ready.notify_all()

    def _eligible_locked(self, job):
        if job.state is not _JobState.PENDING:
            return False
        if self.cancelled.is_set():
            self._complete_job_locked(job, _JobState.CANCELLED, (_STOPPED, True))
            return False
        if time.monotonic() >= job.deadline:
            self._complete_job_locked(job, _JobState.EXPIRED, _PENDING_EXPIRY)
            return False
        return True

    def _fail_pending_job(self, job, result):
        with self._job_lock:
            if self._eligible_locked(job):
                self._complete_job_locked(job, _JobState.DONE, result)

    @QtCore.Slot(object)
    def _run_job(self, job):
        with self._job_lock:
            if not self._eligible_locked(job):
                return
        try:
            if self._running:
                # A nested prompt can deliver another job on the owner thread.
                QtCore.QTimer.singleShot(30, lambda: self._run_job(job))
                return
            current = self.is_current()
        except Exception as err:
            self._fail_pending_job(job, self._job_error(job, err))
            return
        with self._job_lock:
            if not self._eligible_locked(job):
                return
            if not current:
                self._complete_job_locked(job, _JobState.DONE, (_STALE, True))
                return
            job.state = _JobState.RUNNING
        self._running = True
        result = ('The tool did not complete.', True)
        try:
            result = self._call(job.name, job.args)
        except Exception as err:
            result = self._job_error(job, err)
        finally:
            self._running = False
            with self._job_lock:
                self._complete_job_locked(job, _JobState.DONE, result)
            self._settle_if_idle()

    def _job_error(self, job, err):
        logger.exception('Assistant tool %s failed', job.name)
        return '{} failed: {}'.format(job.name, err), True

    def _call(self, name, args):
        tool = self.tools.get(name)
        offered = tool in self.available() if tool else False
        self.calls += 1
        call_id = self.calls
        label = name
        if tool:
            try:
                label = tool.label(args)
            except Exception:
                label = name
        try:
            self._require_current()
            if not offered or (tool.kind == EDIT and not self.may_edit()) \
                    or (tool.kind == RUN and not self.may_run()):
                if tool and tool.kind == EDIT:
                    text = ('Changing the stage is turned off. Ask the user to '
                            'turn on "Edit stage" under the message box.')
                elif tool and tool.kind == RUN:
                    text = ('Running code is turned off. Ask the user to turn '
                            'on "Run code" under the message box.')
                elif not self._have_stage and tool and tool.needs_stage:
                    text = 'No stage is open.'
                else:
                    text = 'There is no tool called {}.'.format(name)
                if offered:
                    # Offered at Send, revoked since: the reply records it.
                    self.failed.append(text)
                raise ToolError(text)
            self._tell(call_id, tool, label, 'running')
            self._require_current()
            result = tool.handler(args)
        except (ToolError, stage_actions.ActionError) as err:
            self._tell(call_id, tool, label, 'failed', str(err))
            return str(err), True
        except Exception as err:
            logger.exception('Assistant tool {} failed'.format(name))
            text = '{} failed: {}'.format(name, err)
            self._tell(call_id, tool, label, 'failed', text)
            return text, True
        self._tell(call_id, tool, label, 'done')
        if isinstance(result, Image):
            result.text = _clip(result.text)
            return result, False
        if not isinstance(result, str):
            result = json.dumps(result, indent=1, default=str)
        return _clip(result), False

    def _tell(self, call_id, tool, label, state, detail=''):
        self.activity.emit({'id': call_id,
                            'name': tool.name if tool else '',
                            'kind': tool.kind if tool else READ,
                            'label': label, 'state': state,
                            'detail': detail})

    def cancel(self):
        with self._job_lock:
            self.cancelled.set()
            for job in tuple(self._jobs):
                if job.state is _JobState.PENDING:
                    self._complete_job_locked(job, _JobState.CANCELLED, (_STOPPED, True))

    # Helpers
    def path(self, args, key='path', required=True):
        text = args.get(key)
        if text in (None, ''):
            if required:
                raise ToolError('"{}" is needed'.format(key))
            return None
        text = str(text).strip()
        if not text.startswith('/'):
            text = '/' + text
        text = text.rstrip('/') or '/'
        if not Sdf.Path.IsValidPathString(text):
            raise ToolError('{} is not a prim path'.format(text))
        return Sdf.Path(text)

    def prim(self, args, key='path', required=True):
        path = self.path(args, key, required)
        if path is None:
            return None
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            raise ToolError('There is no prim {}'.format(path))
        return prim

    def time(self, args, key='time'):
        if args.get(key) is None:
            return self.session.time()
        return Usd.TimeCode(float(args[key]))

    def stage_folder(self):
        return self.session.stage_folder() if self.session else None

    def may_read(self, path):
        folder = self.stage_folder()
        if folder and inside(path, folder):
            return True
        if any(inside(path, f) for f in self.chat_folders):
            return True
        return self.memory.allowed(path)

    def readable(self, args, key='path', reason=''):
        """The path asked for, once the user lets it be read."""
        path = str(args.get(key) or '').strip()
        if not path:
            folder = self.stage_folder()
            if not folder:
                raise ToolError('Give a "{}"; the stage is not saved, so '
                                'there is no folder of its own'.format(key))
            path = folder
        path = os.path.expanduser(path)
        if not os.path.isabs(path):
            base = self.stage_folder()
            if not base:
                raise ToolError('Give a full path; the stage is not saved, '
                                'so there is no folder to start from')
            path = os.path.join(base, path)
        path = os.path.normpath(path)
        if self.may_read(path):
            return path
        folder = path if os.path.isdir(path) else os.path.dirname(path)
        if not os.path.isdir(folder):
            raise ToolError('There is no folder {}'.format(folder))
        if norm(folder) in self.denied:
            raise ToolError('The user did not let you read {}.'.format(
                folder))
        answer = DENY
        if self.ask_access is not None:
            reason = str(args.get('reason') or reason or '').strip()
            self._require_current()
            answer = self.ask_access(folder, reason)
        self._require_current()
        if answer == ALWAYS:
            self.memory.allow(folder, str(args.get('about') or ''))
        elif answer == ONCE:
            self.chat_folders.add(folder)
        else:
            self.denied.add(norm(folder))
            raise ToolError('The user did not let you read {}. Do not ask '
                            'for it again in this reply.'.format(folder))
        return path

    def rig_roots(self):
        return [p for p in self.stage.Traverse()
                if p.GetTypeName() == RIG_ROOT]

    def rig_root_of(self, prim):
        while prim and not prim.IsPseudoRoot():
            if prim.GetTypeName() == RIG_ROOT:
                return prim
            prim = prim.GetParent()
        return None

    def _undo_stack(self):
        """The editor's history, or None when it cannot be reached."""
        if self.session is None:
            return None
        try:
            return self.session.undo_stack()
        except RuntimeError:
            logger.warning('Could not reach the undo stack', exc_info=True)
            return None

    def _groups_edits(self, stack):
        """Grouping needs both newest-step identity and redo visibility;
        the documented host protocol requires neither."""
        return (callable(getattr(stack, 'LastEdit', None)) and
                callable(getattr(stack, 'CanRedo', None)))

    def _step_is_open(self, stack, step):
        """Only the newest step on the current history can be extended."""
        return stack.LastEdit() is step and not stack.CanRedo()

    def _forget_step(self):
        self._history_step = None
        self._history_pushed = False

    def _push(self, stack, step):
        try:
            stack.Push(step)
        except RuntimeError:
            logger.warning('Could not add the assistant\'s changes to '
                           'the undo stack', exc_info=True)
            # The step is lost: later changes open their own rather than
            # piling into one that never reached the history.
            self._forget_step()

    def _open_step(self, stack, label):
        """Open the step one tool call's changes share, before it authors.
        Nothing goes on the stack yet: a call that turns out to change
        nothing records nothing, and leaves the history and its redo
        branch as they were. Unsupported hosts receive one push per edit."""
        if stack is None:
            return None
        if not self._groups_edits(stack):
            return _UNGROUPED
        if self._history_step is not None and not (
                self._history_pushed and
                self._step_is_open(stack, self._history_step)):
            # An undo took this step back, or another producer committed
            # after it: a change written now is not part of it, and its
            # own push drops the redo branch.
            self._forget_step()
        if self._history_step is None:
            self._history_step = session_module.GroupEdit(label, [])
        return self._history_step

    def _close_step(self, stack, step, edit):
        """Record the change made inside the open step, while it is still
        newest, so a commit arriving from a notice the write fires keeps
        the authored order. `edit` is None when the write touched nothing,
        and then nothing is recorded at all."""
        if stack is None:
            return
        if step is _UNGROUPED:
            if edit is not None:
                self._push(stack, edit)
            return
        if edit is None:
            if step is not None and not step.edits:
                # Nothing was written into the step: it never reaches the
                # stack, so the older steps and the redo branch are intact.
                self._forget_step()
            return
        if self._history_pushed and not self._step_is_open(stack, step):
            # Another producer committed while this call was open, so the
            # step is no longer newest and this change needs its own.
            self._forget_step()
            self._push(stack, edit)
            return
        step.edits.append(edit)
        if not self._history_pushed:
            self._history_pushed = True
            self._push(stack, step)

    @contextlib.contextmanager
    def _authored_edit(self, stack, paths, label, defer_notice=False):
        """One change: its history step is opened before the write and
        filled in when the change is made, so a commit arriving from a
        notice the write fires keeps the authored order. A change that
        turns out to touch nothing records no undo."""
        import rigExecUndo
        step = self._open_step(stack, label)
        scope = rigExecUndo.SpecScope(self.stage, paths, None, label)
        closed = False
        scope.__enter__()
        try:
            # Begin maps edit-target paths before the block; commit and
            # record before its notice can dispatch another editor write.
            block = Sdf.ChangeBlock() if defer_notice else contextlib.nullcontext()
            with block:
                try:
                    yield scope
                except BaseException:
                    scope.__exit__(*sys.exc_info())
                    raise
                else:
                    scope.__exit__(None, None, None)
                    self._close_step(stack, step, scope.edit)
                    closed = True
        finally:
            if not closed:
                self._close_step(stack, step, scope.edit)

    # The tools
    def _all_tools(self):
        return [
            Tool('get_stage_overview', READ,
                 'What is open: the file, the layer stack (strongest '
                 'first) and the edit target the changes go to, the time '
                 'range and current frame, the rigs (RigExecRoot prims), '
                 'the selection and whether RigExec is evaluating live.',
                 _obj(), self.get_stage_overview,
                 lambda a: 'Looked over the stage'),
            Tool('list_prims', READ,
                 'The prims under a path, as an indented outline with each '
                 'prim\'s type, a few levels deep.',
                 _obj({'path': dict(PATH, description='Where to start, / '
                                                      'by default'),
                       'depth': {'type': 'integer', 'default': 2},
                       'limit': {'type': 'integer', 'default': 300},
                       'type': {'type': 'string', 'description':
                                'Only list prims of this type name'}}),
                 self.list_prims,
                 lambda a: 'Listed {}'.format(a.get('path') or 'the stage')),
            Tool('get_prim', READ,
                 'Everything about one prim: its type and applied schemas, '
                 'its attributes with their values at a frame (and whether '
                 'they are keyed or connected), its relationships, '
                 'children, composition arcs and the layers with opinions '
                 'on it.',
                 _obj({'path': PATH, 'time': TIME,
                       'authored_only': {'type': 'boolean', 'default':
                                         False, 'description': 'Only the '
                                         'authored attributes'}},
                      ['path']),
                 self.get_prim,
                 lambda a: 'Read {}'.format(a.get('path'))),
            Tool('find_prims', READ,
                 'Find prims by name (text or a * pattern), type, applied '
                 'schema or attribute name.',
                 _obj({'text': {'type': 'string', 'description':
                                'In the name, or a pattern such as *_Arm'},
                       'type': {'type': 'string'},
                       'schema': {'type': 'string', 'description':
                                  'An applied API schema'},
                       'attribute': {'type': 'string', 'description':
                                     'Prims having an attribute whose '
                                     'name contains this'},
                       'under': PATH,
                       'limit': {'type': 'integer', 'default': 100}}),
                 self.find_prims,
                 lambda a: 'Searched for {}'.format(
                     a.get('text') or a.get('type') or a.get('schema') or
                     a.get('attribute') or 'prims')),
            Tool('get_attribute', READ,
                 'One attribute in full: its value at a frame, its keys '
                 '(time samples or spline knots), connections, metadata '
                 'and which layers hold opinions on it, strongest first.',
                 _obj({'path': PATH, 'name': {'type': 'string'},
                       'time': TIME,
                       'max_keys': {'type': 'integer', 'default': 50}},
                      ['path', 'name']),
                 self.get_attribute,
                 lambda a: 'Read {}.{}'.format(a.get('path'),
                                               a.get('name'))),
            Tool('list_rigs', READ,
                 'The rigs on the stage (RigExecRoot prims), with how many '
                 'controls, joints, movers and solvers each holds.',
                 _obj(), self.list_rigs, lambda a: 'Listed the rigs'),
            Tool('list_avars', READ,
                 'The avars (animation channels) a control or prim offers '
                 'for posing, with their values at a frame and whether '
                 'they are keyed. Avars a mover recomputes are named as '
                 'hidden: setting them does nothing visible.',
                 _obj({'path': PATH, 'time': TIME}, ['path']),
                 self.list_avars,
                 lambda a: 'Read the avars of {}'.format(a.get('path'))),
            Tool('check_rig', READ,
                 'Compile a rig on its own, apart from the viewport, and '
                 'report whether it compiles, the errors, the operations '
                 'it skips and the order its movers run in.',
                 _obj({'path': dict(PATH, description='The RigExecRoot; '
                                    'the only rig when left out')}),
                 self.check_rig,
                 lambda a: 'Checked {}'.format(a.get('path') or 'the rig')),
            Tool('evaluate_rig', READ,
                 'Evaluate a rig at a frame, apart from the viewport and '
                 'without authoring anything, and give back its '
                 'diagnostics and the world frames of the joints and '
                 'controls asked for (all of them, up to limit, when '
                 'none are named).',
                 _obj({'path': dict(PATH, description='The RigExecRoot; '
                                    'the only rig when left out'),
                       'time': TIME,
                       'joints': NAMES, 'controls': NAMES,
                       'limit': {'type': 'integer', 'default': 20}}),
                 self.evaluate_rig,
                 lambda a: 'Evaluated {}{}'.format(
                     a.get('path') or 'the rig',
                     ' at frame {}'.format(a['time'])
                     if a.get('time') is not None else '')),
            Tool('get_live_evaluator', READ,
                 'What the viewport\'s own RigExec evaluator last did: its '
                 'rigs, the frame it ran, its generation and how many '
                 'operations its graph has. It compiles and evaluates '
                 'nothing.',
                 _obj({'path': dict(PATH, description='A RigExecRoot; '
                                    'lists the rigs when left out')}),
                 self.get_live_evaluator,
                 lambda a: 'Read the live evaluator'),
            Tool('capture_viewport', READ,
                 'A picture of usdview\'s viewport as it is now, to look '
                 'at a pose or check a change.',
                 _obj(), self.capture_viewport,
                 lambda a: 'Looked at the viewport'),
            Tool('get_output_log', READ,
                 'The last lines Python logged in usdview, to look for '
                 'errors and warnings.',
                 _obj({'lines': {'type': 'integer', 'default': 80},
                       'errors_only': {'type': 'boolean',
                                       'default': False}}),
                 self.get_output_log, lambda a: 'Read the log',
                 needs_stage=False),
            Tool('select_prims', SHOW,
                 'Select prims in usdview to show them to the user, and '
                 'frame them in the viewport if asked. An empty list '
                 'clears the selection.',
                 _obj({'paths': PATHS,
                       'frame': {'type': 'boolean', 'default': False}},
                      ['paths']),
                 self.select_prims,
                 lambda a: 'Selected {}'.format(_short(a.get('paths')))),
            Tool('set_frame', SHOW,
                 'Go to a frame on usdview\'s time line.',
                 _obj({'frame': {'type': 'number'}}, ['frame']),
                 self.set_frame,
                 lambda a: 'Went to frame {}'.format(a.get('frame'))),
            Tool('set_avars', EDIT,
                 'Pose or animate: set avars on controls. With key true '
                 '(the default) each value is keyed at the frame; with key '
                 'false it becomes the default value, which a key '
                 'outranks. Values are numbers, or a token or bool for '
                 'those channels.',
                 _obj({'values': {
                     'type': 'array',
                     'items': _obj({'path': PATH,
                                    'name': {'type': 'string',
                                             'description': 'Such as '
                                             'avars:rx'},
                                    'value': {}},
                                   ['path', 'name', 'value'])},
                       'time': TIME,
                       'key': {'type': 'boolean', 'default': True}},
                      ['values']),
                 self.set_avars,
                 lambda a: 'Set {} avar{}'.format(
                     len(a.get('values') or []),
                     '' if len(a.get('values') or []) == 1 else 's')),
            Tool('reset_avars', EDIT,
                 'Put avars back to the rig\'s rest: every editable avar '
                 'of the prims given, or only the names given.',
                 _obj({'paths': PATHS,
                       'names': {'type': 'array', 'items':
                                 {'type': 'string'}},
                       'time': TIME,
                       'key': {'type': 'boolean', 'default': True}},
                      ['paths']),
                 self.reset_avars,
                 lambda a: 'Reset the avars of {}'.format(
                     _short(a.get('paths')))),
            Tool('change_stage', EDIT,
                 'Change the stage on its edit target. Give a list of '
                 'changes, each an object with an "action" and its '
                 'fields:\n' + stage_actions.action_reference(),
                 _obj({'changes': {'type': 'array',
                                   'items': {'type': 'object'}}},
                      ['changes']),
                 self.change_stage,
                 lambda a: 'Made {} change{}'.format(
                     len(a.get('changes') or []),
                     '' if len(a.get('changes') or []) == 1 else 's')),
            Tool('save_layers', EDIT,
                 'Save layers with unsaved changes to their files: the '
                 'edit target, or the layers named (identifiers or file '
                 'names), or every dirty layer with all true.',
                 _obj({'layers': {'type': 'array', 'items':
                                  {'type': 'string'}},
                       'all': {'type': 'boolean', 'default': False}}),
                 self.save_layers, lambda a: 'Saved layers'),
            Tool('run_python', RUN,
                 'Run Python in usdview to try something or do what the '
                 'other tools cannot. usdviewApi, stage, Usd, Sdf, Gf, '
                 'UsdGeom and rigexec are ready, and names you define are '
                 'kept for later runs in this reply. Print what you want '
                 'to see; the value of a last bare expression is given '
                 'back too. Changes to the edit target can be undone with '
                 'the rest of the reply; changes to other layers cannot.',
                 _obj({'code': {'type': 'string'}}, ['code']),
                 self.run_python, lambda a: 'Ran Python'),
            Tool('list_files', READ,
                 'List the files in a folder on this computer. The open '
                 'stage\'s folder is free to read; for anywhere else the '
                 'user is asked first, so say why in reason.',
                 _obj({'path': {'type': 'string', 'description':
                                'A folder, or a path relative to the '
                                'stage\'s folder'},
                       'pattern': {'type': 'string', 'default': '*'},
                       'recursive': {'type': 'boolean', 'default': False},
                       'reason': READ_REASON, 'about': READ_ABOUT}),
                 self.list_files,
                 lambda a: 'Listed {}'.format(_place(a.get('path'))),
                 needs_stage=False),
            Tool('read_file', READ,
                 'Read a text file, with line numbers. Long files are read '
                 'a part at a time with start_line and end_line.',
                 _obj({'path': {'type': 'string'},
                       'start_line': {'type': 'integer', 'default': 1},
                       'end_line': {'type': 'integer'},
                       'reason': READ_REASON, 'about': READ_ABOUT}, ['path']),
                 self.read_file,
                 lambda a: 'Read {}'.format(_place(a.get('path'))),
                 needs_stage=False),
            Tool('search_files', READ,
                 'Search the files under a folder for text (or a regular '
                 'expression with regex), giving each line that matches.',
                 _obj({'text': {'type': 'string'},
                       'path': {'type': 'string'},
                       'pattern': {'type': 'string', 'default': '*',
                                   'description': 'Only files matching '
                                                  'this, such as *.usda'},
                       'regex': {'type': 'boolean', 'default': False},
                       'reason': READ_REASON, 'about': READ_ABOUT}, ['text']),
                 self.search_files,
                 lambda a: 'Searched {} for "{}"'.format(
                     _place(a.get('path')) or 'the stage\'s folder',
                     a.get('text')),
                 needs_stage=False),
            Tool('inspect_import', READ,
                 'What a Python module offers: where it is, its '
                 'submodules, functions and classes with their arguments. '
                 'With member (such as "Rig" or "Builder.create") gives '
                 'that one in full, with its source. It is not run to find '
                 'out, unless it is already imported.',
                 _obj({'module': {'type': 'string', 'description':
                                  'A module, such as rigexec or '
                                  'pxr.UsdGeom'},
                       'member': {'type': 'string'}}, ['module']),
                 self.inspect_import,
                 lambda a: 'Inspected {}{}'.format(
                     a.get('module'),
                     '.' + a['member'] if a.get('member') else ''),
                 needs_stage=False),
            Tool('remember', MEMORY,
                 'Save a short note to remember in every later chat, such '
                 'as how the user likes things done or what a folder '
                 'holds. The user can see and delete it in the settings.',
                 _obj({'note': {'type': 'string'}}, ['note']),
                 self.remember,
                 lambda a: 'Remembered "{}"'.format(
                     _clip(str(a.get('note') or ''), 60)),
                 needs_stage=False),
            Tool('forget', MEMORY,
                 'Forget a note, by the id it is listed with, when it is '
                 'wrong or the user asks.',
                 _obj({'id': {'type': 'string'}}, ['id']),
                 self.forget, lambda a: 'Forgot a note', needs_stage=False),
        ]

    # Looking at the stage
    def get_stage_overview(self, args):
        stage = self.stage
        target = stage.GetEditTarget().GetLayer()
        layers = []
        for layer in stage.GetLayerStack(includeSessionLayers=True):
            info = {'layer': layer.identifier}
            if layer == target:
                info['edit_target'] = True
            if layer.dirty:
                info['unsaved_changes'] = True
            if stage.IsLayerMuted(layer.identifier):
                info['muted'] = True
            layers.append(info)
        rigs = [str(p.GetPath()) for p in self.rig_roots()]
        info = {'file': self.session.stage_file() or '(not saved)',
                'layers': layers,
                'start_frame': stage.GetStartTimeCode(),
                'end_frame': stage.GetEndTimeCode(),
                'frames_per_second': stage.GetFramesPerSecond(),
                'current_frame': _value(self.session.time().GetValue()),
                'up_axis': str(UsdGeom.GetStageUpAxis(stage)),
                'meters_per_unit': UsdGeom.GetStageMetersPerUnit(stage),
                'default_prim': str(stage.GetDefaultPrim().GetPath())
                if stage.GetDefaultPrim() else None,
                'rigs': rigs,
                'rig_evaluating_live': self.session.rig_active(),
                'selected': [str(p.GetPath())
                             for p in self.session.selected_prims()]}
        return info

    def list_prims(self, args):
        root = self.prim(args, required=False) or self.stage.GetPseudoRoot()
        depth = max(1, int(args.get('depth') or 2))
        limit = max(1, int(args.get('limit') or 300))
        wanted = str(args.get('type') or '')
        lines = []
        total = [0]

        def walk(prim, level):
            for child in prim.GetAllChildren():
                if wanted and child.GetTypeName() != wanted:
                    if level + 1 < depth:
                        walk(child, level + 1)
                    continue
                total[0] += 1
                if len(lines) < limit:
                    marks = []
                    if not child.IsActive():
                        marks.append('inactive')
                    if not child.IsDefined():
                        marks.append('over')
                    lines.append('{}{}  {}{}'.format(
                        '  ' * level, child.GetPath(),
                        child.GetTypeName() or '-',
                        '  [{}]'.format(', '.join(marks)) if marks else ''))
                if level + 1 < depth:
                    walk(child, level + 1)

        walk(root, 0)
        if not lines:
            return '{} has no children{}.'.format(
                root.GetPath(), ' of type ' + wanted if wanted else '')
        if total[0] > len(lines):
            lines.append('... and {} more'.format(total[0] - len(lines)))
        return '\n'.join(lines)

    def get_prim(self, args):
        prim = self.prim(args)
        time = self.time(args)
        attrs = {}
        source = (prim.GetAuthoredAttributes() if args.get('authored_only')
                  else prim.GetAttributes())
        for attr in source:
            entry = {'type': str(attr.GetTypeName())}
            try:
                entry['value'] = _value(attr.Get(time))
            except Exception as err:
                entry['value'] = '(could not read: {})'.format(err)
            if attr.GetNumTimeSamples() or getattr(
                    attr, 'HasSpline', lambda: False)():
                entry['keyed'] = True
            if attr.HasAuthoredConnections():
                entry['connected_to'] = [str(p)
                                         for p in attr.GetConnections()]
            if not attr.HasAuthoredValue() and \
                    not attr.HasAuthoredConnections():
                entry['authored'] = False
            attrs[attr.GetName()] = entry
        rels = {r.GetName(): [str(t) for t in r.GetTargets()]
                for r in prim.GetRelationships()}
        arcs = []
        query = Usd.PrimCompositionQuery(prim)
        for arc in query.GetCompositionArcs():
            kind = str(arc.GetArcType()).split('.')[-1]
            if kind in ('Root',):
                continue
            arcs.append({'arc': kind,
                         'layer': arc.GetTargetNode().layerStack.identifier
                         .rootLayer.identifier,
                         'path': str(arc.GetTargetNode().path)})
        rig = self.rig_root_of(prim)
        info = {'path': str(prim.GetPath()),
                'type': prim.GetTypeName() or None,
                'applied_schemas': list(prim.GetAppliedSchemas()),
                'active': prim.IsActive(),
                'specifier': str(prim.GetSpecifier()).split('.')[-1],
                'in_rig': str(rig.GetPath()) if rig else None,
                'time': _value(time.GetValue()) if not time.IsDefault()
                else 'default',
                'attributes': attrs,
                'relationships': rels,
                'children': [c.GetName() for c in prim.GetAllChildren()],
                'layers_with_opinions': [
                    s.layer.identifier for s in prim.GetPrimStack()]}
        if arcs:
            info['composition_arcs'] = arcs
        variants = prim.GetVariantSets()
        names = variants.GetNames()
        if names:
            info['variants'] = {n: variants.GetVariantSelection(n)
                                for n in names}
        return info

    def find_prims(self, args):
        text = str(args.get('text') or '').strip().lower()
        wanted_type = str(args.get('type') or '')
        schema = str(args.get('schema') or '')
        attribute = str(args.get('attribute') or '').lower()
        if not (text or wanted_type or schema or attribute):
            raise ToolError('Give text, type, schema or attribute')
        under = self.prim(args, 'under', required=False)
        limit = max(1, int(args.get('limit') or 100))
        pattern = text if any(c in text for c in '*?[') else None
        prims = (Usd.PrimRange(under) if under is not None
                 else self.stage.Traverse())
        found = []
        for prim in prims:
            name = prim.GetName().lower()
            if text and not (fnmatch.fnmatch(name, pattern) if pattern
                             else text in name):
                continue
            if wanted_type and prim.GetTypeName() != wanted_type:
                continue
            if schema and schema not in prim.GetAppliedSchemas():
                continue
            if attribute and not any(attribute in a.GetName().lower()
                                     for a in prim.GetAttributes()):
                continue
            found.append('{}  {}'.format(prim.GetPath(),
                                         prim.GetTypeName() or '-'))
            if len(found) >= limit:
                found.append('... stopped at {}'.format(limit))
                break
        return '\n'.join(found) if found else 'Nothing matched.'

    def get_attribute(self, args):
        prim = self.prim(args)
        name = str(args.get('name') or '')
        attr = prim.GetAttribute(name)
        if not attr:
            raise ToolError('{} has no attribute {}'.format(prim.GetPath(),
                                                           name))
        time = self.time(args)
        most = max(1, int(args.get('max_keys') or 50))
        info = {'path': str(attr.GetPath()),
                'type': str(attr.GetTypeName()),
                'value': _value(attr.Get(time)),
                'time': _value(time.GetValue()) if not time.IsDefault()
                else 'default',
                'custom': attr.IsCustom(),
                'variability': str(attr.GetVariability()).split('.')[-1]}
        if attr.HasAuthoredValue():
            default = attr.Get(Usd.TimeCode.Default())
            info['default'] = _value(default)
        samples = attr.GetTimeSamples()
        if samples:
            info['time_samples'] = {
                '{:g}'.format(t): _value(attr.Get(t))
                for t in samples[:most]}
            if len(samples) > most:
                info['more_time_samples'] = len(samples) - most
        spline = getattr(attr, 'GetSpline', None)
        if spline is not None and getattr(attr, 'HasSpline',
                                          lambda: False)():
            knots = spline().GetKnots()
            info['spline_knots'] = {
                '{:g}'.format(k.GetTime()): _value(k.GetValue())
                for k in list(knots)[:most]}
        if attr.HasAuthoredConnections():
            info['connected_to'] = [str(p) for p in attr.GetConnections()]
        tokens = attr.GetMetadata('allowedTokens')
        if tokens:
            info['allowed_tokens'] = [str(t) for t in tokens]
        doc = attr.GetDocumentation()
        if doc:
            info['documentation'] = doc
        info['layers_with_opinions'] = [
            s.layer.identifier for s in attr.GetPropertyStack(time)]
        return info

    def list_rigs(self, args):
        rigs = []
        for root in self.rig_roots():
            counts = collections.Counter()
            for prim in Usd.PrimRange(root):
                type_name = prim.GetTypeName()
                if type_name == 'RigExecControl':
                    counts['controls'] += 1
                elif type_name == 'RigExecJoint':
                    counts['joints'] += 1
                elif 'Solver' in type_name:
                    counts['solvers'] += 1
                if prim.HasAttribute('rigExec:moves') or \
                        prim.HasRelationship('rigExec:moves'):
                    counts['movers'] += 1
            rigs.append(dict({'path': str(root.GetPath())}, **counts))
        return rigs or 'There are no rigs (RigExecRoot prims) on the stage.'

    def _avar_model(self):
        import avarEditorModel
        return avarEditorModel

    def list_avars(self, args):
        model = self._avar_model()
        prim = self.prim(args)
        time = self.time(args)
        channels, hidden = model.DiscoverChannels(prim, self.stage)
        listed = []
        for channel in channels:
            entry = {'name': channel.name,
                     'value': _value(channel.attr.Get(time)),
                     'kind': channel.kind}
            if channel.unit:
                entry['unit'] = channel.unit
            if channel.allowedTokens:
                entry['allowed'] = channel.allowedTokens
            if model.IsAnimated(channel.attr):
                entry['keyed'] = True
            listed.append(entry)
        result = {'path': str(prim.GetPath()), 'avars': listed}
        if hidden:
            result['hidden'] = hidden
        if not listed and not hidden:
            return '{} has no avars.'.format(prim.GetPath())
        return result

    def _rig_path(self, args):
        path = self.path(args, required=False)
        if path is not None:
            prim = self.stage.GetPrimAtPath(path)
            if not prim:
                raise ToolError('There is no prim {}'.format(path))
            root = self.rig_root_of(prim)
            if root is None:
                raise ToolError('{} is not in a rig'.format(path))
            return root.GetPath()
        roots = self.rig_roots()
        if len(roots) == 1:
            return roots[0].GetPath()
        if not roots:
            raise ToolError('There are no rigs on the stage')
        raise ToolError('There are {} rigs; give the path of one: {}'.format(
            len(roots), _short([str(r.GetPath()) for r in roots], 10)))

    def _compiled_rig(self, rig_path):
        try:
            import rigexec
        except ImportError as err:
            raise ToolError('The rigexec module is not available: '
                            '{}'.format(err))
        rig = rigexec.Rig(self.stage, str(rig_path))
        try:
            rig.compile()
        except ValueError as err:
            return rig, str(err)
        return rig, None

    def check_rig(self, args):
        rig_path = self._rig_path(args)
        started = time.time()
        rig, errors = self._compiled_rig(rig_path)
        result = {'rig': str(rig_path),
                  'compiled': errors is None,
                  'seconds': round(time.time() - started, 2)}
        if errors:
            result['errors'] = errors
            return result
        for key, getter in (('mover_order', 'mover_order'),
                            ('skipped_operations', 'skipped_operations')):
            try:
                value = getattr(rig, getter)()
                result[key] = _value(value)
            except Exception as err:
                result[key] = '(not available: {})'.format(err)
        return result

    def evaluate_rig(self, args):
        rig_path = self._rig_path(args)
        rig, errors = self._compiled_rig(rig_path)
        if errors:
            return {'rig': str(rig_path), 'compiled': False,
                    'errors': errors}
        time_code = self.time(args)
        value = time_code.GetValue() if not time_code.IsDefault() else 0.0
        pose = rig.evaluate(value)
        limit = max(1, int(args.get('limit') or 40))
        result = {'rig': str(rig_path), 'time': value,
                  'valid': bool(pose.valid),
                  'diagnostics': _value(list(pose.diagnostics or []))}

        def matches(path, want):
            return path == want or path.endswith('/' + want.lstrip('/'))

        def frames(paths, wanted, read):
            paths = [str(p) for p in paths]
            wanted = [str(w) for w in (wanted or [])]
            chosen = [p for p in paths if any(matches(p, w)
                                              for w in wanted)] \
                if wanted else paths
            out = {}
            for path in chosen[:limit]:
                try:
                    out[path] = read(path)
                except Exception as err:
                    out[path] = '(could not read: {})'.format(err)
            if len(chosen) > limit:
                out['...'] = '{} more; name the ones you want'.format(
                    len(chosen) - limit)
            missing = [w for w in wanted
                       if not any(matches(p, w) for p in paths)]
            return out, missing

        joint_paths = list(pose.joint_paths())
        control_paths = list(pose.control_paths())
        joints, missing_joints = frames(
            joint_paths, args.get('joints'),
            lambda p: _frame(pose.joint_frame(str(p))))
        controls, missing_controls = frames(
            control_paths, args.get('controls'),
            lambda p: _frame(pose.control_frame(str(p))))
        result['joint_frames'] = joints
        result['control_frames'] = controls
        result['joint_count'] = len(joint_paths)
        result['control_count'] = len(control_paths)
        if missing_joints or missing_controls:
            result['not_in_pose'] = missing_joints + missing_controls
        return result

    def get_live_evaluator(self, args):
        handle = self.session.imaging()
        if handle is None or handle.lib is None:
            return ('RigExec is not evaluating in this usdview, so there is '
                    'no live evaluator. check_rig and evaluate_rig still '
                    'work.')
        rig_path = self.path(args, required=False)
        entry = getattr(handle.lib, 'RigExecImaging_LiveDebugJsonForStage',
                        None)
        if entry is None:
            raise ToolError('This RigExec library has no live inspection')
        entry.restype = ctypes.c_int
        entry.argtypes = [ctypes.c_longlong, ctypes.c_char_p,
                          ctypes.c_char_p, ctypes.c_char_p,
                          ctypes.c_char_p, ctypes.c_int]
        rig = str(rig_path or '').encode('utf-8')
        size = entry(handle.CacheId(), rig, b'', b'', None, 0)
        if size < 0:
            return 'RigExec has no live session for this stage.'
        buffer = ctypes.create_string_buffer(size + 1)
        entry(handle.CacheId(), rig, b'', b'', buffer, size + 1)
        try:
            data = json.loads(buffer.value.decode('utf-8'))
        except ValueError:
            return _clip(buffer.value.decode('utf-8', 'replace'))
        return _summarize_live(data)

    def capture_viewport(self, args):
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        path = os.path.join(CAPTURE_DIR, 'viewport.png')
        width, height = self.session.grab_viewport(path)
        self._require_current()
        with open(path, 'rb') as fp:
            png = fp.read()
        return Image('The viewport at frame {:g}, {} by {} pixels, saved '
                     'to {}.'.format(self.session.time().GetValue(), width,
                                     height, path), png)

    def get_output_log(self, args):
        lines = max(1, min(int(args.get('lines') or 80), MAX_LOG_LINES))
        entries = list(recent_log().lines)
        if args.get('errors_only'):
            entries = [e for e in entries if e[1] >= logging.WARNING]
        text = '\n'.join(e[2] for e in entries[-lines:])
        return text or 'The log is empty.'

    # Showing
    def select_prims(self, args):
        paths = args.get('paths') or []
        if isinstance(paths, str):
            paths = [paths]
        paths = [self.path({'path': p}) for p in paths]
        found = self._select(paths, frame=bool(args.get('frame')))
        missing = [str(p) for p in paths if str(p) not in found]
        text = 'Selected {}.'.format(_short(found, 10))
        if missing:
            text += ' Not found: {}.'.format(', '.join(missing))
        return text

    def _select(self, paths, frame=False):
        self._require_current()
        found = self.session.select(paths)
        if frame and found:
            # Selection change signals can reenter the viewer before framing.
            self._require_current()
            self.session.frame_selection()
        return found

    def set_frame(self, args):
        if args.get('frame') is None:
            raise ToolError('Give a "frame"')
        now = self.session.set_frame(float(args['frame']))
        return 'The current frame is {:g}.'.format(now.GetValue())

    # Changing the stage
    def _write_time(self, args):
        if not args.get('key', True):
            return Usd.TimeCode.Default()
        time_code = self.time(args)
        if time_code.IsDefault():
            raise ToolError('There is no current frame to key at; give a '
                            '"time" or set key false')
        return time_code

    def _channel(self, model, prim, name):
        channels, hidden = model.DiscoverChannels(prim, self.stage)
        for channel in channels:
            if channel.name == name or channel.shortName == name:
                return channel
        if name in hidden:
            raise ToolError('{}.{} is hidden: a mover recomputes it or it '
                            'is not a value the avar editor sets'.format(
                                prim.GetPath(), name))
        raise ToolError('{} has no avar {}'.format(prim.GetPath(), name))

    def set_avars(self, args):
        model = self._avar_model()
        values = args.get('values')
        if isinstance(values, dict):
            values = [values]
        if not isinstance(values, list) or not values:
            raise ToolError('"values" must be a list of {path, name, value}')
        key = bool(args.get('key', True))
        time_code = self._write_time(args)
        mode = model.WRITE_ANIMATION if key else model.WRITE_DEFAULT
        planned, problems = [], []
        for item in values:
            try:
                prim = self.prim(item)
                channel = self._channel(model, prim,
                                        str(item.get('name') or ''))
                planned.append((channel, item.get('value')))
            except ToolError as err:
                problems.append(str(err))
        if not planned:
            raise ToolError('; '.join(problems))
        channels = [c for c, _ in planned]
        stack = self._undo_stack()
        step = self._open_step(stack, 'AI Assistant pose')
        scope = model.EditScope(self.stage, channels, None,
                               'AI Assistant pose')
        # The plans read the composed stage and the channel's keys, so they
        # are made before the change block.
        plans = [model.PlanWrite(c, v, time_code, mode)
                 for c, v in planned]
        # The block holds the write's USD notice until the step is on the
        # history, so a commit the notice carries follows it.
        with Sdf.ChangeBlock():
            with scope:
                model.ApplyPlanWrites(plans)
            self._close_step(stack, step, scope.edit)
        # The warnings read the composed stage, which waits for the block.
        warnings = [w for w in (model.PlanWarning(plan) for plan in plans)
                    if w]
        when = '' if time_code.IsDefault() else ' at frame {:g}'.format(
            time_code.GetValue())
        for channel, value in planned:
            self.done.append('Set {}.{} to {}{}'.format(
                channel.prim.GetPath(), channel.name, value, when))
        self.failed += problems
        lines = ['Set {} avar{}{}{}.'.format(
            len(planned), '' if len(planned) == 1 else 's',
            ' as keys' if key else ' as defaults', when)]
        lines += ['Warning: ' + w for w in warnings]
        lines += ['Could not: ' + p for p in problems]
        return '\n'.join(lines)

    def reset_avars(self, args):
        model = self._avar_model()
        paths = args.get('paths') or []
        if isinstance(paths, str):
            paths = [paths]
        names = set(str(n) for n in args.get('names') or [])
        key = bool(args.get('key', True))
        time_code = self._write_time(args)
        mode = model.WRITE_ANIMATION if key else model.WRITE_DEFAULT
        channels = []
        for path in paths:
            prim = self.prim({'path': path})
            found, _hidden = model.DiscoverChannels(prim, self.stage)
            channels += [c for c in found if not names or c.name in names or
                         c.shortName in names]
        if not channels:
            raise ToolError('No avars to reset')
        stack = self._undo_stack()
        step = self._open_step(stack, 'AI Assistant reset')
        scope = model.EditScope(self.stage, channels, None,
                               'AI Assistant reset')
        plans = [model.PlanReset(c, time_code, mode) for c in channels]
        # As in set_avars: the step is on the history before the reset's
        # notice runs, so a commit it carries follows the reset.
        with Sdf.ChangeBlock():
            with scope:
                model.ApplyPlanWrites(plans)
            self._close_step(stack, step, scope.edit)
        self.done.append('Reset {} avars on {}'.format(len(channels),
                                                        _short(paths)))
        return 'Reset {} avars.'.format(len(channels))

    def change_stage(self, args):
        changes = args.get('changes')
        if isinstance(changes, dict):
            changes = [changes]
        if not isinstance(changes, list) or not changes:
            raise ToolError('"changes" must be a list of changes')
        stack = self._undo_stack()

        def guard(change):
            target = self.stage.GetEditTarget()
            paths = _spec_paths([change], target)
            return self._authored_edit(
                stack, paths, 'AI Assistant changes',
                defer_notice=change.get('action') in
                stage_actions.DEFER_NOTICE_ACTIONS)

        applier = stage_actions.Applier(self.stage, select=self._select,
                                        revalidate=self.is_current,
                                        bracket=guard)
        done, failed = applier.apply(changes)
        self.done += done
        self.failed += failed
        lines = ['Done: ' + d for d in done] + \
            ['Could not: ' + f for f in failed]
        return '\n'.join(lines) or 'Nothing changed.'

    def save_layers(self, args):
        stage = self.stage
        wanted = [str(w) for w in args.get('layers') or []]
        stack = stage.GetLayerStack(includeSessionLayers=False)
        if args.get('all'):
            chosen = [l for l in stack if l.dirty]
        elif wanted:
            chosen = []
            for layer in stack:
                real = (layer.realPath or '').replace('\\', '/')
                if layer.identifier in wanted or real in wanted or \
                        real.split('/')[-1] in wanted:
                    chosen.append(layer)
        else:
            chosen = [stage.GetEditTarget().GetLayer()]
        saved, problems = [], []
        for layer in chosen:
            if layer.anonymous:
                problems.append('{} is anonymous and has no file'.format(
                    layer.identifier))
            elif not layer.dirty:
                continue
            elif layer.Save():
                saved.append(layer.identifier)
            else:
                problems.append('Could not save {}'.format(layer.identifier))
        self.done += ['Saved ' + s for s in saved]
        self.failed += problems
        lines = ['Saved ' + s for s in saved] + problems
        return '\n'.join(lines) or 'Nothing needed saving.'

    # Running
    def run_python(self, args):
        source = str(args.get('code') or '')
        if not source.strip():
            raise ToolError('Give some code to run')
        if self._namespace is None:
            self._namespace = _namespace(self.session, self.stage)
        layer = self.stage.GetEditTarget().GetLayer() if self.stage \
            else None
        before = session_module.snapshot(layer) if layer is not None \
            else None
        log = recent_log()
        mark = log.count
        started = time.time()
        stack = self._undo_stack()
        step = (self._open_step(stack, 'AI Assistant Python')
                if before is not None else None)
        printed = io.StringIO()
        result = {}
        with contextlib.redirect_stdout(printed), \
                contextlib.redirect_stderr(printed):
            try:
                value = _exec(source, self._namespace)
                if value is not None:
                    result['value'] = _clip(repr(value), 4000)
            except Exception:
                result['error'] = traceback.format_exc(limit=8)
        if before is not None:
            after = session_module.snapshot(layer)
            edit = None
            if not session_module.same_content(before, after):
                edit = session_module.LayerSnapshotEdit(
                    'AI Assistant Python', layer, before, after)
                self.done.append('Changed {} with Python'.format(
                    layer.identifier))
            self._close_step(stack, step, edit)
        result['seconds'] = round(time.time() - started, 2)
        text = printed.getvalue().strip()
        if text:
            result['printed'] = _clip(text, 12000)
        errors = [e[2] for e in log.since(mark) if e[1] >= logging.WARNING]
        if errors:
            result['logged'] = errors[-20:]
        if self.is_current() and not self.cancelled.is_set():
            self.session.update_viewport()
        return result

    # Files and imports
    def list_files(self, args):
        path = self.readable(args)
        try:
            return files.list_files(path, args.get('pattern') or '*',
                                    bool(args.get('recursive')))
        except files.FileError as err:
            raise ToolError(str(err))

    def read_file(self, args):
        path = self.readable(args)
        try:
            return files.read_file(path, args.get('start_line') or 1,
                                   args.get('end_line'))
        except files.FileError as err:
            raise ToolError(str(err))

    def search_files(self, args):
        path = self.readable(args)
        try:
            return files.search_files(path, args.get('text'),
                                      args.get('pattern') or '*',
                                      regex=bool(args.get('regex')))
        except files.FileError as err:
            raise ToolError(str(err))

    def inspect_import(self, args):
        name = str(args.get('module') or '').strip()
        if not name:
            raise ToolError('Give a "module"')
        try:
            return files.inspect_module(name, args.get('member') or None)
        except files.FileError as err:
            raise ToolError(str(err))
        except SyntaxError as err:
            raise ToolError('{} does not parse: {}'.format(name, err))

    # Memory
    def remember(self, args):
        try:
            note = self.memory.add(args.get('note') or '')
        except ValueError as err:
            raise ToolError(str(err))
        return 'Remembered as note {}.'.format(note['id'])

    def forget(self, args):
        if not self.memory.remove(str(args.get('id') or '')):
            raise ToolError('There is no note {}'.format(args.get('id')))
        return 'Forgotten.'


_PROPERTY_ACTIONS = ('set_attr', 'clear_attr', 'block_attr',
                     'set_connections', 'set_relationship', 'set_metadata')


def _spec_paths(changes, target=None):
    """Paths to snapshot before a write; target maps missing ancestors
    to the layer where the write will create them."""
    paths = set()

    def add(text):
        text = str(text or '').strip()
        if text and not text.startswith('/'):
            text = '/' + text
        if text and Sdf.Path.IsValidPathString(text):
            paths.add(Sdf.Path(text))
            return Sdf.Path(text)
        return None

    for change in changes:
        if not isinstance(change, dict):
            continue
        action = change.get('action')
        name = str(change.get('name') or '')
        if action in _PROPERTY_ACTIONS and name:
            text = str(change.get('path') or '').strip()
            if text and not text.startswith('/'):
                text = '/' + text
            if Sdf.Path.IsValidPathString(text) and \
                    Sdf.Path.IsValidNamespacedIdentifier(name):
                paths.add(Sdf.Path(text).AppendProperty(name))
            continue
        prim = add(change.get('path'))
        if prim is not None and action == 'rename_prim' and change.get(
                'name') and Sdf.Path.IsValidIdentifier(str(change['name'])):
            paths.add(prim.GetParentPath().AppendChild(str(change['name'])))
            paths.add(prim.GetParentPath())
        if prim is not None and action == 'reparent_prim':
            parent = add(change.get('parent'))
            if parent is not None:
                paths.add(parent.AppendChild(prim.name))
            paths.add(prim.GetParentPath())
        if prim is not None and target is not None and action in (
                'define_prim', 'override_prim', 'set_active',
                'add_reference'):
            top = prim
            parent = prim.GetParentPath()
            while parent != Sdf.Path.absoluteRootPath:
                mapped = target.MapToSpecPath(parent)
                if not mapped or target.GetLayer().GetPrimAtPath(
                        mapped) is not None:
                    break
                top = parent
                parent = parent.GetParentPath()
            paths.add(top)
    # A prim's snapshot holds its subtree, so its descendants need none.
    kept = [p for p in paths
            if not any(p != q and p.HasPrefix(q) for q in paths)]
    return sorted(kept)


def _namespace(session, stage):
    from pxr import Gf, Sdf as _Sdf, Usd as _Usd, UsdGeom as _UsdGeom, Vt
    names = {'__name__': '__usdrig_assistant__',
             'usdviewApi': session.api if session else None,
             'stage': stage,
             'Gf': Gf, 'Sdf': _Sdf, 'Usd': _Usd, 'UsdGeom': _UsdGeom,
             'Vt': Vt}
    try:
        import rigexec
        names['rigexec'] = rigexec
    except ImportError:
        pass
    return names


def _exec(source, namespace):
    """Run source in namespace; gives back the value of a last bare
    expression, as the Python prompt does."""
    import ast
    tree = ast.parse(source, '<assistant>', 'exec')
    last = None
    if tree.body and isinstance(tree.body[-1], ast.Expr):
        last = ast.Expression(tree.body.pop().value)
    exec(compile(tree, '<assistant>', 'exec'), namespace)
    if last is not None:
        return eval(compile(last, '<assistant>', 'eval'), namespace)
    return None


def _frame(frame):
    return {'origin': list(frame.origin), 'x_axis': list(frame.x_axis),
            'y_axis': list(frame.y_axis), 'z_axis': list(frame.z_axis)}


def _summarize_live(data):
    """The live evaluator's JSON without its long graph and trace rows."""
    if not isinstance(data, dict):
        return data
    out = {}
    for key, value in data.items():
        if isinstance(value, list) and len(value) > 20:
            out[key] = '({} rows)'.format(len(value))
        elif isinstance(value, dict) and len(json.dumps(value)) > 4000:
            out[key] = {k: ('({} rows)'.format(len(v))
                            if isinstance(v, list) and len(v) > 20 else v)
                        for k, v in value.items()}
        else:
            out[key] = value
    return out
