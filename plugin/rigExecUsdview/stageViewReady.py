"""Session-owned StageView installation on the usdview UI thread."""

import logging
from typing import NamedTuple, Optional
import weakref

import sessionRegistry


DEFAULT_RETRIES = 40


class InstallResult(NamedTuple):
    status: str
    controller: object = None
    error: Optional[str] = None


_PENDING = InstallResult("pending")
_UNAVAILABLE = InstallResult("unavailable")


def StageView(usdviewApi):
    """Return the host's actual view, or None before usdview creates it."""
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _Warn(tool, diagnostic):
    # A warning handler must not interfere with publication or a Qt slot.
    try:
        logging.getLogger(__name__).warning("%s: %s", tool, diagnostic)
    except Exception:
        pass


def _Diagnostic(cause, error):
    try:
        text = str(error)
    except Exception:
        text = ""
    return "%s failed: %s" % (cause, text or type(error).__name__)


def _DisposeTimer(timer, slot):
    if timer is not None:
        try:
            timer.stop()
        except Exception:
            pass
        if slot is not None:
            try:
                timer.timeout.disconnect(slot)
            except Exception:
                pass
        try:
            timer.deleteLater()
        except Exception:
            pass


class _Record:
    # This object is one generation. Only a live generation can publish.
    def __init__(self):
        self.live = True
        self.busy = False
        self.unavailable = False
        self.results = {}
        self.pending = {}
        self.api = None
        self.remaining = 0
        self.timer = None
        self.slot = None

    def Dispose(self):
        self.live = False
        self._StopWaiting()
        self.pending.clear()
        self.results.clear()

    def _StopWaiting(self):
        timer, self.timer = self.timer, None
        slot, self.slot = self.slot, None
        self.api = None
        _DisposeTimer(timer, slot)

    def _Settle(self, result):
        self._StopWaiting()
        if not self.live:
            return
        self.unavailable = result.status == "unavailable"
        while self.live and self.pending:
            tools = list(self.pending)
            for tool in tools:
                self.results[tool] = result
            pending, self.pending = self.pending, {}
            pending.clear()
            if result.error is not None:
                for tool in tools:
                    if not self.live:
                        break
                    _Warn(tool, result.error)

    def _Drain(self):
        self._StopWaiting()
        # Reentrant registrations append behind all earlier requests.
        while self.live and self.pending:
            tool = next(iter(self.pending))
            self._Install(tool, *self.pending.pop(tool))

    def _Install(self, tool, create, onReady):
        try:
            controller = create()
            if controller is None:
                raise ValueError("returned no controller")
        except Exception as error:
            diagnostic = _Diagnostic("factory", error)
            if self.live:
                self.results[tool] = InstallResult("failed", error=diagnostic)
                _Warn(tool, diagnostic)
            return
        if not self.live:
            return
        self.results[tool] = InstallResult("ready", controller)
        if onReady is not None:
            try:
                onReady(controller)
            except Exception as error:
                diagnostic = _Diagnostic("notification", error)
                if self.live:
                    _Warn(tool, diagnostic)

    def Probe(self, api, retries=None):
        if not self.live or self.busy:
            return
        self.busy = True
        cause = "probe"
        try:
            ready = StageView(api) is not None
            if not self.live:
                return
            if ready:
                self._Drain()
                return
            host = getattr(api, "_UsdviewApi__appController", None)
            noRender = self.live and bool(getattr(host, "_noRender", False))
            if not self.live:
                return
            cause = "timer"
            self.remaining = (self.remaining - 1 if retries is None
                              else max(0, retries))
            if not self.live:
                return
            if noRender or self.remaining <= 0 or not self._Arm(api):
                self._Settle(_UNAVAILABLE)
        except Exception as error:
            diagnostic = _Diagnostic(cause, error)
            if self.live:
                self._Settle(InstallResult("failed", error=diagnostic))
        finally:
            self.busy = False

    def _Arm(self, api):
        if self.timer is None:
            try:
                from pxr.Usdviewq.qt import QtCore
            except ImportError:
                return False
            if not self.live:
                return False
            app = QtCore.QCoreApplication.instance()
            if not self.live or app is None or app.closingDown():
                return False
            thread = app.thread()
            if not self.live:
                return False
            if QtCore.QThread.currentThread() != thread:
                raise RuntimeError("StageView registration is not on the UI thread")
            dispatcher = QtCore.QAbstractEventDispatcher.instance(thread)
            if not self.live or dispatcher is None:
                return False
            timer = QtCore.QTimer()
            if not self.live:
                _DisposeTimer(timer, None)
                return False
            self.timer = timer
            timer.setSingleShot(True)
            if not self.live:
                return False
            recordRef = weakref.ref(self)

            def _Timeout():
                record = recordRef()
                if record is not None and record.timer is not None:
                    record.Probe(record.api)

            self.slot = _Timeout
            timer.timeout.connect(_Timeout)
            if not self.live:
                return False
            self.api = api
        timer = self.timer
        timer.start(0)
        if self.live and not timer.isActive():
            raise RuntimeError("could not start StageView retry timer")
        return self.live


_sessions = sessionRegistry.SessionRegistry("StageView readiness", _Record.Dispose)


def Register(usdviewApi, tool, create, retries=DEFAULT_RETRIES, onReady=None):
    """Reserve a session/tool once; re-query GetResult after a pending result."""
    key = sessionRegistry.SessionKey(usdviewApi)
    record = _sessions.Get(key)
    if record is not None and tool in record.results:
        return record.results[tool]
    if key is None or (record is not None and record.unavailable):
        if record is not None:
            record.results[tool] = _UNAVAILABLE
        return _UNAVAILABLE
    first = record is None
    if first:
        record = _Record()
    record.results[tool] = _PENDING
    record.pending[tool] = (create, onReady)
    if first:
        # The destroyed watch may call out while the entry is being filed.
        record.busy = True
        _sessions.Set(key, record)
        record.busy = False
    if record.live and not record.busy and record.timer is None:
        record.Probe(usdviewApi, retries)
    return record.results.get(tool, _UNAVAILABLE)


def GetResult(usdviewApi, tool):
    """Return the latest immutable snapshot, or None for an unregistered tool."""
    record = _sessions.Get(usdviewApi)
    return record.results.get(tool) if record is not None else None


def Retire(usdviewApi):
    """Cancel waiting work and forget this session's results."""
    _sessions.Pop(usdviewApi)


def _ReadyItems(tool):
    for key, record in _sessions.Items():
        result = record.results.get(tool)
        if result is not None and result.status == "ready":
            yield key, result.controller


def Controller(usdviewApi, tool):
    """Use the explicit session, or the active/sole ready session for tool."""
    if usdviewApi is not None:
        result = GetResult(usdviewApi, tool)
        return result.controller if result is not None else None
    windows = list(sessionRegistry._WindowChain(sessionRegistry.ActiveWindow()))
    ready = list(_ReadyItems(tool))
    for window in windows:
        for key, controller in ready:
            if window is key:
                return controller
    return ready[0][1] if len(ready) == 1 else None


def Controllers(tool):
    """Return all ready controllers for this tool."""
    return [controller for _key, controller in _ReadyItems(tool)]
