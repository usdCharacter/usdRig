#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""
USD Notice handler for the usdNoodles graph editor.
"""

from pxr import Tf, Usd


class UsdNoticeHandler:
    """
    Handles USD change notifications for the graph editor.

    The handler categorizes changes into:
    - Resynced paths: Structural changes (prims added/removed, connections changed)
    - Info-only changes: Property value changes (positions, metadata)
    """

    def __init__(self, graphView):
        self._graphView = graphView
        self._listener = None
        self._stage = None
        # Flag to temporarily disable processing (e.g., during our own edits)
        self._enabled = True

    def register(self, stage):
        self.unregister()

        if not stage:
            return

        listener = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._onObjectsChanged, stage
        )
        self._listener, self._stage = listener, stage

    def unregister(self):
        if self._listener:
            self._listener.Revoke()
            self._listener = None
        self._stage = None

    def setEnabled(self, enabled):
        """
        Use this to temporarily disable processing during batch edits
        from the graph editor itself to avoid recursive updates.
        """
        self._enabled = enabled

    def isEnabled(self):
        return self._enabled

    @staticmethod
    def _movedPaths(notice):
        """Return ``{oldPath: newPath}`` for prims this notice *moved*.

        A namespace move (rename, reparent, or both at once) reaches the
        listener as two ordinary resyncs -- one for a path that no longer
        resolves and one for a path that just appeared -- which is
        indistinguishable from a delete plus an unrelated create unless you ask
        USD. ``GetPrimResyncType`` is that question: it classifies each resynced
        path and hands back the path it is paired with, so the editor can follow
        the prim instead of throwing its node away and hoping something adds an
        equivalent one back.

        Only the *Source* half is reported; its paired path is the destination,
        so the pair is complete from one side. Older USD builds have no
        ``GetPrimResyncType`` -- there the result is empty and callers fall back
        to plain drop-and-adopt reconciliation.
        """
        if not hasattr(notice, "GetPrimResyncType"):
            return {}

        moved = {}
        for path in notice.GetResyncedPaths():
            try:
                resyncType, pairedPath = notice.GetPrimResyncType(path)
            except Exception:
                continue
            if not pairedPath:
                continue
            if resyncType.name not in (
                "RenameSource",
                "ReparentSource",
                "RenameAndReparentSource",
            ):
                continue
            moved[str(path)] = str(pairedPath)
        return moved

    def _onObjectsChanged(self, notice, sender):
        if self._stage is None or sender is not self._stage:
            return

        if not self._graphView:
            return

        select = getattr(self._graphView, "_selectUsdChanges", None)
        if select is not None:
            callback = select(sender)
        else:
            callback = (self._graphView._handleUsdChanges if self._enabled else
                        getattr(self._graphView, "_handleSuppressedUsdChanges", None))
        selectResyncs = getattr(self._graphView, "_selectUsdResyncs", None)
        observe = selectResyncs(sender) if selectResyncs is not None else None
        if callback is None and observe is None:
            return

        resyncedPaths = {str(path) for path in notice.GetResyncedPaths()}
        movedPaths = self._movedPaths(notice)
        if observe is not None and resyncedPaths:
            # Identity evidence cannot wait for deferred or suppressed UI delivery.
            observe(resyncedPaths, movedPaths)
        if callback is None:
            return
        # "/" stays in the resynced set: it is USD's only spelling for a
        # stage-wide resync (pseudo-root metadata, reconfigured composition)
        # and reconciliation treats it as a scope-membership recheck, never
        # as a node path (see adoptPrimsNewToScope).

        # Collect info-only changes (property value changes)
        # These allow targeted cache invalidation
        infoChangedPaths = set()
        for path in notice.GetChangedInfoOnlyPaths():
            pathStr = str(path)
            if pathStr == "/":
                continue
            infoChangedPaths.add(pathStr)

        if resyncedPaths or infoChangedPaths:
            callback(resyncedPaths, infoChangedPaths, movedPaths)

    def getStage(self):
        return self._stage
