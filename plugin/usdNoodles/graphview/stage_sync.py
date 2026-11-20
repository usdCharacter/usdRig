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
class Remap:
    """Nodes re-keyed across a namespace move, ``(old, new)`` pairs."""

    epoch: int
    pairs: tuple


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

    def remap(self, pairs):
        self._publish(Remap(self._epoch, pairs))

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

    def _handleUsdChanges(
        self, resyncedPaths, infoChangedPaths, movedPaths=None
    ):
        if self._revoked:
            return
        self._service.changed(
            self._epoch, resyncedPaths, infoChangedPaths, movedPaths
        )


class StageSync:
    """One editor's stage lifecycle: replace, bind, notice, finish, close.

    Constructed per editor with the callables the view supplies -- ``read()``
    for one owner-thread read of the view for a reload, ``notice()`` for the
    narrower read a USD notice needs, ``apply(delta)`` to perform a mutation
    synchronously on the owner thread, ``post(callback)`` to schedule on the
    widget owner thread, ``listen(receiver)`` to build the editor's
    UsdNoticeHandler, and ``warn(message)`` for diagnostics.
    ``reconcile(resyncedPaths, infoChangedPaths, movedPaths, trace)`` runs
    synchronously on the owner thread for an admitted notice.

    Every replace moves one *epoch*. Deferred callbacks and notice deliveries
    carry the epoch they were produced under, and anything that no longer
    matches the live epoch is dropped: a stage replace in flight cannot be
    undone by a timer tick from the previous stage.
    """

    def __init__(
        self, read, notice, apply, post, listen, warn=None, sink=None, *, reconcile
    ):
        self._read = read
        self._notice = notice
        self._reconcile = reconcile
        self._apply = apply
        self._post = post
        self._warn = warn
        self._sink = sink
        self._epoch = 0
        self._boundStage = None
        self._closed = False
        self._receiver = _NoticeReceiver(self)
        self._handler = listen(self._receiver)

    # -- the editor's notice handler -------------------------------------

    def handler(self):
        """The UsdNoticeHandler built by ``listen``.

        The view keeps it as ``view._noticeHandler``: the authoring sites
        suppress through that object, and the loaders register the stage on it
        (``register`` unregisters first), so this service never registers a
        second listener.
        """
        return self._handler

    def epoch(self):
        return self._epoch

    def isClosed(self):
        return self._closed

    @contextmanager
    def suppressed(self):
        """Ignore USD notices for the duration of the block.

        The state lives on the editor's UsdNoticeHandler, and the value in
        front when the block opened is restored on the normal path and on an
        exception, so nesting cannot leave the editor deaf.
        """
        handler = self._handler
        previous = handler.isEnabled()
        handler.setEnabled(False)
        try:
            yield
        finally:
            handler.setEnabled(previous)

    # -- replace / reload -------------------------------------------------

    def replace(self, stage, mode, prim_path, frame):
        """usdview replaced the stage: tear the graph down, rebuild it.

        The epoch moves first, so every callback an earlier replace queued --
        and the notice receiver stamped with the old stage -- is stale before
        any teardown work starts.

        A closed editor ignores the delivery entirely: the widget can outlive
        its close event, and a replacement that got through would re-arm the
        notice receiver and reload a graph nobody is displaying.
        """
        if self._closed:
            return
        node_ids = self._read().node_ids
        self._epoch += 1
        epoch = self._epoch

        begin = BeginReload(epoch, frame)
        self._publish(begin)
        self._apply(begin)
        self.bind(stage)

        if stage is None:
            clear = ClearHistory(epoch)
            self._publish(clear)
            self._apply(clear)
            self._scheduleFinish(epoch)
            return

        # The loaders build a whole new nodeGraph, so every id in the graph
        # read above is gone; recorded rather than applied.
        self._publish(Remove(epoch, node_ids))
        load = Load(epoch, stage, mode, prim_path)
        self._publish(load)

        try:
            self._apply(load)
        except Exception as error:
            if self._warn is not None:
                self._warn(
                    "Failed to reload graph after stage replace: %s" % error
                )
        self._scheduleFinish(epoch)

    def bind(self, stage):
        """Record the stage this generation listens to, and stamp the receiver.

        The loaders call ``register`` on the UsdNoticeHandler themselves, so
        nothing here registers -- that would double-register. Re-binding
        replaces whatever epoch the receiver carried, which is what revokes
        the previous binding: a notice from a replaced stage cannot reach the
        model.
        """
        self._boundStage = stage
        self._receiver.rebind(self._epoch)
        return stage

    def finish(self, epoch):
        """Bring one generation's rendering back up -- the latest live one only.

        A stale epoch does nothing at all: its render caches belong to a graph
        that no longer exists, and re-enabling paint would let the widget touch
        a GL surface the newer replace has already torn down.
        """
        if self._closed or epoch != self._epoch:
            return
        delta = FinishReload(epoch)
        self._publish(delta)
        self._apply(delta)

    def close(self):
        """Revoke the notice receiver and invalidate anything still queued."""
        self._closed = True
        # Past every queued finish and every receiver stamp: the epoch moving
        # is what makes the deferred callbacks return without touching the
        # model.
        self._epoch += 1
        self._boundStage = None
        self._receiver.revoke()
        if self._handler is not None:
            self._handler.unregister()

    # -- USD notices ------------------------------------------------------

    def changed(self, epoch, resyncedPaths, infoChangedPaths, movedPaths=None):
        """One USD notice, delivered under the epoch that bound its stage."""
        if self._closed or epoch != self._epoch:
            return
        ctx = self._notice()
        if ctx.syncing_selection:
            return
        self._reconcile(
            resyncedPaths,
            infoChangedPaths,
            movedPaths,
            (
                _ReconcileTrace(self._sink, epoch)
                if self._sink is not None
                else None
            ),
        )

    # -- internals --------------------------------------------------------

    def _publish(self, delta):
        if self._sink is not None:
            self._sink(delta)

    def _scheduleFinish(self, epoch):
        """One finish per replace, on the widget owner thread.

        The callback carries its epoch and the binding it was queued under:
        a replace or a close since then makes it stale, and a stale finish
        must not re-enable paint on a graph that has moved on.
        """
        binding = self._boundStage

        def _run():
            if (
                self._closed
                or epoch != self._epoch
                or binding is not self._boundStage
            ):
                return
            self.finish(epoch)

        self._post(_run)
