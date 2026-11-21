#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Stage lifecycle policy and reconciliation records for the noodle editor.

The service gates notices and reloads by lifetime and generation. Scheduling,
warning and synchronous reconciliation are supplied by the caller.
"""

from contextlib import contextmanager
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# The published trace of one lifecycle step.
#
# Each record names the epoch that produced it, so a consumer can tell the
# current generation's work from a stale one. They are records, not a queue:
# the reload and notice paths emit them as the work happens, and the model
# mutations they describe are performed by the adapter. Paths are strings, id
# collections are tuples, and endpoint / port metadata keeps the order the
# graph iterates in.
# ---------------------------------------------------------------------------

MODE_STAGE = "stage"
MODE_BLUEPRINT = "blueprint"
MODE_CONTAINER = "container"


@dataclass(frozen=True, slots=True)
class BeginReload:
    """Paint is off and the previous stage's transient state is dropped.

    *frame* is the time the reloaded graph reads at.
    """

    epoch: int
    frame: object


@dataclass(frozen=True, slots=True)
class Load:
    """Re-run the loader for *mode* (one of MODE_STAGE / MODE_BLUEPRINT /
    MODE_CONTAINER) on *stage*. The stage is live only inside the
    owner-thread dispatch that applies this record."""

    epoch: int
    stage: object
    mode: str
    prim_path: object


@dataclass(frozen=True, slots=True)
class ClearHistory:
    """Drop the undo history: its records name prims on a stage that is gone."""

    epoch: int


@dataclass(frozen=True, slots=True)
class Remove:
    """Node ids the graph lost -- a dropped prim, or a rebuilt graph."""

    epoch: int
    ids: tuple


@dataclass(frozen=True, slots=True)
class Adopt:
    """Scope paths the graph took on because a prim appeared in USD."""

    epoch: int
    paths: tuple


@dataclass(frozen=True, slots=True)
class Invalidate:
    """The rebuild / repaint decision a notice pass ended with."""

    epoch: int
    paths: tuple
    links: bool
    text: bool
    paint: bool


@dataclass(frozen=True, slots=True)
class FinishReload:
    """The deferred render-cache clear, links and text dirty, paint back on."""

    epoch: int


@dataclass(frozen=True, slots=True)
class StageContext:
    """IDs in the graph before reload, including synthetic nodes."""

    node_ids: tuple


@dataclass(frozen=True, slots=True)
class NoticeContext:
    """Selection mute flag, without a whole-graph read for each notice."""

    syncing_selection: bool


def _nodeIdPrimPath(nodeId):
    """The prim-path half of a node id.

    Most node ids ARE prim paths. Blueprint / Container I/O nodes are
    synthesized rather than prim-backed and carry their pin in a ``#`` suffix
    (``/Bp#input:foo:bar``); the part before the ``#`` is still the prim path
    they hang off, which is what makes them follow that prim when it moves.
    """
    return nodeId.split("#", 1)[0]


def _isPathRelated(pathA, pathB):
    """Whether one path is the other, or contains the other.

    "Contains" spans both separators USD uses: ``/`` for a child prim and ``.``
    for a property, so ``/World/Cube`` is related to ``/World/Cube/Leg`` and to
    ``/World/Cube.inputs:x`` alike -- a resync can name either, and both mean
    "the node at /World/Cube is stale".

    The boundary is the point. Plain ``startswith`` reads ``/World/Cube`` as an
    ancestor of ``/World/CubeTwo``, so editing one prim invalidates unrelated
    siblings whose names merely share a prefix.
    """
    if pathA == pathB:
        return True
    longer, shorter = (pathA, pathB) if len(pathA) > len(pathB) else (pathB, pathA)
    if not longer.startswith(shorter):
        return False
    # The pseudo-root is the one path that already ends in its own separator.
    if shorter.endswith("/"):
        return True
    return longer[len(shorter)] in "/."


class _ReconcileTrace:
    """Builds the reconcile records for one pass and hands them to the sink."""

    def __init__(self, sink, epoch):
        self._sink = sink
        self._epoch = epoch

    def _publish(self, record):
        if self._sink is not None:
            self._sink(record)

    def remove(self, ids):
        self._publish(Remove(self._epoch, ids))

    def adopt(self, paths):
        self._publish(Adopt(self._epoch, paths))

    def invalidate(self, paths, links, text, paint):
        self._publish(
            Invalidate(self._epoch, paths, links, text, paint)
        )


class _NoticeReceiver:
    """What the UsdNoticeHandler calls, stamped with a stage generation.

    ``UsdNoticeHandler`` keeps its protocol -- an object with
    ``_handleUsdChanges(resyncedPaths, infoChangedPaths, movedPaths=None)`` --
    but in production the object behind it is this receiver, not the widget.
    It carries the epoch of its binding, so a delivery that no longer matches
    the live generation never reaches the model.
    """

    def __init__(self, service):
        self._service = service
        self._epoch = service.epoch()
        self._revoked = False

    def rebind(self, epoch):
        self._epoch = epoch
        self._revoked = False

    def revoke(self):
        self._revoked = True

    def _handleSuppressedUsdChanges(self, resynced, info, moved):
        if not self._revoked:
            self._service._recordSuppressed(self._epoch, resynced, info, moved)

    def _handleUsdChanges(
        self, resyncedPaths, infoChangedPaths, movedPaths=None
    ):
        if self._revoked:
            return
        self._service.changed(
            self._epoch, resyncedPaths, infoChangedPaths, movedPaths
        )


class _SuppressionToken:
    def __init__(self, owner):
        self._owner = owner

    def release(self):
        """Release this acquisition on the editor's owner thread."""
        owner = self._owner
        if owner is not None:
            self._owner = None
            owner._releaseSuppression(self)


class StageSync:
    """One editor's stage lifecycle: replace, bind, notice, finish, close.

    Constructed per editor with the callables the view supplies -- ``read()``
    for one owner-thread read of the view for a reload, ``notice()`` for the
    narrower read a USD notice needs, ``apply(delta)`` to perform a mutation
    synchronously on the owner thread, ``post(callback)`` to schedule on the
    widget owner thread, ``listen(receiver)`` to build the editor's
    UsdNoticeHandler, and ``warn(message)`` for the diagnostics channel. None
    of them may import Qt or GL here, which is what lets the whole lifecycle
    run headless against a recording stub.

    Every replace moves one *epoch*. Deferred callbacks and notice deliveries
    carry the epoch they were produced under, and anything that no longer
    matches the live epoch is dropped: a stage replace in flight cannot be
    undone by a timer tick from the previous stage.
    """

    def __init__(
        self, read, notice, apply, post, listen, warn=None, sink=None, *,
        reconcile=None, prepare=None, reset=None, defer=None, graphReplaced=None,
    ):
        self._read = read
        self._notice = notice
        self._reconcile = reconcile
        self._prepare = prepare
        self._reset = reset
        self._defer = defer
        self._apply = apply
        self._post = post
        self._on_owner = on_owner
        self._post_notice = post_notice or post
        self._warn = warn
        self._sink = sink
        self._graphReplaced = graphReplaced
        self._pendingChanges = None
        self._epoch = 0
        self._finishPending = False
        self._closed = False
        self._receiver = _NoticeReceiver(self)
        self._handler = listen(self._receiver)
        self._suppressionTokens = set()
        self._enabledBeforeSuppression = None

    def epoch(self):
        return self._epoch

    def isClosed(self):
        return self._closed

    def acquireSuppression(self, *, reconcile=False):
        """Inhibit this editor's notices until its last live token is released."""
        if self._closed:
            return _SuppressionToken(None)
        token = _SuppressionToken(self)
        if not self._suppressionTokens:
            self._enabledBeforeSuppression = self._handler.isEnabled()
            self._handler.setEnabled(False)
        if reconcile and self._enabledBeforeSuppression and self._pendingChanges is None:
            self._pendingChanges = (self._epoch, set(), set(), {})
        self._suppressionTokens.add(token)
        return token

    def _releaseSuppression(self, token):
        if token not in self._suppressionTokens:
            return
        self._suppressionTokens.remove(token)
        if not self._suppressionTokens:
            self._handler.setEnabled(self._enabledBeforeSuppression)
            pending, self._pendingChanges = self._pendingChanges, None
            if pending is not None and (pending[1] or pending[2]):
                self.changed(*pending)

    def _recordSuppressed(self, epoch, resynced, info, moved):
        pending = self._pendingChanges
        if pending is None or pending[0] != epoch or epoch != self._epoch:
            return
        pending[1].update(resynced)
        pending[2].update(info)
        for source, destination in (moved or {}).items():
            for original, intermediate in tuple(pending[3].items()):
                if intermediate == source:
                    pending[3][original] = destination
                    break
            else:
                pending[3][source] = destination

    @contextmanager
    def suppressed(self):
        """Ignore USD notices for the duration of the block."""
        token = self.acquireSuppression()
        try:
            yield
        finally:
            token.release()

    # -- replace / reload -------------------------------------------------

    def replace(self, stage, mode, prim_path, frame):
        """Reload after usdview has replaced its stage, deferring GL work."""
        if self._closed:
            return
        ctx = self._read()
        reset = self._reset

        def begin(epoch):
            self._dispatch(BeginReload(epoch, frame))

        def populate(epoch):
            if stage is None:
                reset()
                self._dispatch(ClearHistory(epoch))
            else:
                self._publish(Remove(epoch, ctx.node_ids))
                self._dispatch(Load(epoch, stage, mode, prim_path))

        try:
            self._transition(stage, populate, reset, begin)
        except Exception as error:
            if self._warn is not None:
                self._warn("Failed to reload graph after stage replace: %s" % error)

    def loadGraph(self, stage, populate, reset):
        """Switch graphs synchronously; a successful sample can stay unbound."""
        if not self._closed:
            self._transition(stage, lambda epoch: populate(), reset)

    def _transition(self, stage, populate, reset, begin=None):
        self._epoch += 1
        epoch = self._epoch
        self._pendingChanges = None
        if begin is not None:
            self._finishPending = True
        defer = self._defer(True) if self._finishPending and self._defer else None
        try:
            self.bind(None)
            if begin is not None:
                begin(epoch)
            if self._closed or epoch != self._epoch:
                return
            if self._prepare:
                self._prepare()
            if self._closed or epoch != self._epoch:
                return
            populate(epoch)
            if not self._closed and epoch == self._epoch and stage is not None:
                self.bind(stage)
        except Exception:
            if not self._closed and epoch == self._epoch:
                reset()
            raise
        finally:
            if not self._closed and epoch == self._epoch:
                if defer is not None:
                    self._defer(False)
            if not self._closed and epoch == self._epoch and self._finishPending:
                self._scheduleFinish(epoch)
            if not self._closed and epoch == self._epoch and self._graphReplaced:
                self._graphReplaced()

    def boundStage(self):
        """The stage that actually owns the adapter's live notice key."""
        return self._handler.getStage()

    def bind(self, stage):
        """Change the sender within this epoch and stamp its receiver."""
        if self._closed:
            return None
        self._receiver.revoke()
        if stage is None:
            self._handler.unregister()
        else:
            self._handler.register(stage)
            self._receiver.rebind(self._epoch)
        return self.boundStage()

    def finish(self, epoch):
        """Bring one generation's rendering back up -- the latest live one only.

        A stale epoch does nothing at all: its render caches belong to a graph
        that no longer exists, and re-enabling paint would let the widget touch
        a GL surface the newer replace has already torn down.
        """
        if self._closed or epoch != self._epoch or not self._finishPending:
            return
        self._finishPending = False
        self._dispatch(FinishReload(epoch))

    def close(self):
        """Revoke the notice receiver and invalidate anything still queued."""
        if self._closed:
            return
        self._closed = True
        self._finishPending = False
        for token in self._suppressionTokens:
            token._owner = None
        self._suppressionTokens.clear()
        self._pendingChanges = None
        # Past every queued finish and every receiver stamp: the epoch moving
        # is what makes the deferred callbacks return without touching the
        # model.
        self._epoch += 1
        self._receiver.revoke()
        self._handler.unregister()

    # -- USD notices ------------------------------------------------------

    def changed(self, epoch, resyncedPaths, infoChangedPaths, movedPaths=None):
        """One USD notice, delivered under the epoch that bound its stage."""
        if self._closed or epoch != self._epoch or self.boundStage() is None:
            return
        if self._on_owner is not None and not self._on_owner():
            self._post_notice(
                lambda: self.changed(epoch, resyncedPaths, infoChangedPaths, movedPaths)
            )
            return
        ctx = self._notice()
        if ctx.syncing_selection:
            return
        self._reconcile(
            resyncedPaths,
            infoChangedPaths,
            _ReconcileTrace(self._sink, epoch) if self._sink is not None else None,
            movedPaths,
        )

    # -- internals --------------------------------------------------------

    def _publish(self, delta):
        if self._sink is not None:
            self._sink(delta)

    def _dispatch(self, delta):
        self._publish(delta)
        if not self._closed and delta.epoch == self._epoch:
            self._apply(delta)

    def _scheduleFinish(self, epoch):
        """Post a finish for the transition that now owns the pending reload.

        The callback carries its epoch and the binding it was queued under:
        a replace or a close since then makes it stale, and a stale finish
        must not re-enable paint on a graph that has moved on.
        """
        binding = self.boundStage()

        def _run():
            if (
                self._closed
                or epoch != self._epoch
                or binding is not self.boundStage()
            ):
                return
            self.finish(epoch)

        self._post(_run)
