#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Graph command payloads, dispatch, and legacy link operation helpers."""

from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Literal

from pxr import Sdf, Tf

from ..pinUtils import direction_hint_candidate_names
from .selection import LinkKey


# Payloads carry path strings and detached values. Handlers resolve live state.
@dataclass(frozen=True)
class Select:
    mode: Literal["replace", "union", "toggle"]
    node_ids: tuple[str, ...] = ()
    link_keys: tuple[LinkKey, ...] = ()


@dataclass(frozen=True)
class SelectAll:
    pass


@dataclass(frozen=True)
class ClearSelection:
    pass


@dataclass(frozen=True)
class Navigate:
    direction: Literal["backward", "forward"]


@dataclass(frozen=True)
class FinishNodeDrag:
    pass


@dataclass(frozen=True)
class AutoLayout:
    pass


@dataclass(frozen=True)
class OpenContainer:
    pass


@dataclass(frozen=True)
class SaveStage:
    pass


@dataclass(frozen=True)
class Undo:
    pass


@dataclass(frozen=True)
class Redo:
    pass


@dataclass(frozen=True)
class Create:
    parent: str
    type: str
    position: tuple[float, float]


@dataclass(frozen=True)
class Activate:
    paths: tuple[str, ...]
    state: bool


@dataclass(frozen=True)
class RemoveVisual:
    node_ids: tuple[str, ...]
    link_keys: tuple[LinkKey, ...] = ()


@dataclass(frozen=True)
class DeleteSelection:
    pass


@dataclass(frozen=True)
class Rename:
    parent: str
    old: str
    new: str


@dataclass(frozen=True)
class SetValue:
    property: str
    components: tuple[float | int | bool | str, ...]
    mode: Literal["auto", "default", "animation"]
    time: float | None = None
    value: float | int | bool | str | None = None


@dataclass(frozen=True)
class Schema:
    path: str
    api: str
    action: Literal["apply", "remove"]


@dataclass(frozen=True)
class LinkEndpoints:
    source_id: str
    source_port: str
    target_id: str
    target_port: str
    source_property_name: str = ""
    target_property_name: str = ""


@dataclass(frozen=True)
class Connect:
    endpoints: LinkEndpoints


@dataclass(frozen=True)
class Disconnect:
    endpoints: LinkEndpoints


@dataclass(frozen=True)
class Replace:
    old: LinkEndpoints
    new: LinkEndpoints


@dataclass(frozen=True)
class Reconnect:
    old: LinkEndpoints
    new: LinkEndpoints


_INTENT_TYPES = frozenset((
    Select, SelectAll, ClearSelection, Navigate, FinishNodeDrag, AutoLayout,
    OpenContainer, SaveStage, Undo, Redo, Create, Activate, RemoveVisual,
    DeleteSelection,
    Rename, SetValue, Schema, Connect, Disconnect, Replace, Reconnect,
))


class CommandRegistry:
    """Dispatch an exact intent type to one owner-thread handler."""

    def __init__(self):
        self._handlers = {}

    def register(self, intent_type, handler):
        if intent_type not in _INTENT_TYPES:
            raise TypeError(f"Unknown intent type: {intent_type!r}")
        if intent_type in self._handlers:
            raise ValueError(f"Handler already registered for {intent_type.__name__}")
        if not callable(handler):
            raise TypeError("Intent handler must be callable")
        self._handlers[intent_type] = handler

    def dispatch(self, intent):
        handler = self._handlers[type(intent)]
        return handler(intent)


def _collectSelectedLinkData(view):
    """Return serialized connection data for selected links."""
    result = []
    if view._selectedLinks:
        for linkIndex in sorted(view._selectedLinks, reverse=True):
            if 0 <= linkIndex < len(view.links):
                link = view.links[linkIndex]
                result.append(
                    view._makeSerializedLinkRecord(
                        link.sourceNodeId,
                        link.sourcePort,
                        link.targetNodeId,
                        link.targetPort,
                        sourcePropertyName=str(
                            getattr(link, "sourcePropertyName", "")
                        ),
                        targetPropertyName=str(
                            getattr(link, "targetPropertyName", "")
                        ),
                    )
                )
    return result


def _collectIncidentLinkKeys(view, selectedNodeIds, existingLinkRecords=None):
    """Collect deduplicated link-key tuples for links incident to selectedNodeIds.

    If existingLinkRecords is provided, those are deserialized and seeded
    into the seen set so they are not duplicated in the result.
    """
    seen = set()
    if existingLinkRecords:
        for linkRecord in existingLinkRecords:
            try:
                record = view._deserializeLinkRecord(linkRecord)
            except (KeyError, IndexError, TypeError):
                continue
            seen.add(
                (
                    record["sourceNodeId"],
                    record["sourcePort"],
                    record["targetNodeId"],
                    record["targetPort"],
                    record["sourcePropertyName"],
                    record["targetPropertyName"],
                )
            )
    result = []
    for link in view.links:
        if (
            link.sourceNodeId in selectedNodeIds
            or link.targetNodeId in selectedNodeIds
        ):
            key = (
                link.sourceNodeId,
                link.sourcePort,
                link.targetNodeId,
                link.targetPort,
                str(getattr(link, "sourcePropertyName", "")),
                str(getattr(link, "targetPropertyName", "")),
            )
            if key not in seen:
                seen.add(key)
                result.append(key)
    return result


def _makeSerializedLinkRecord(
    srcId,
    srcPort,
    tgtId,
    tgtPort,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    return {
        "sourceNodeId": srcId,
        "sourcePort": srcPort,
        "targetNodeId": tgtId,
        "targetPort": tgtPort,
        "sourcePropertyName": sourcePropertyName,
        "targetPropertyName": targetPropertyName,
    }


def _deserializeLinkRecord(linkRecord):
    if isinstance(linkRecord, dict):
        return {
            "sourceNodeId": str(linkRecord.get("sourceNodeId", "")),
            "sourcePort": str(linkRecord.get("sourcePort", "")),
            "targetNodeId": str(linkRecord.get("targetNodeId", "")),
            "targetPort": str(linkRecord.get("targetPort", "")),
            "sourcePropertyName": str(linkRecord.get("sourcePropertyName", "")),
            "targetPropertyName": str(linkRecord.get("targetPropertyName", "")),
        }

    srcId, srcPort, tgtId, tgtPort = linkRecord[:4]
    sourcePropertyName = linkRecord[4] if len(linkRecord) > 4 else ""
    targetPropertyName = linkRecord[5] if len(linkRecord) > 5 else ""
    return {
        "sourceNodeId": srcId,
        "sourcePort": srcPort,
        "targetNodeId": tgtId,
        "targetPort": tgtPort,
        "sourcePropertyName": sourcePropertyName,
        "targetPropertyName": targetPropertyName,
    }


def _linkMatchesRecord(view, link, linkRecord):
    record = view._deserializeLinkRecord(linkRecord)
    return (
        link.sourceNodeId == record["sourceNodeId"]
        and link.sourcePort == record["sourcePort"]
        and link.targetNodeId == record["targetNodeId"]
        and link.targetPort == record["targetPort"]
        and str(getattr(link, "sourcePropertyName", ""))
        == record["sourcePropertyName"]
        and str(getattr(link, "targetPropertyName", ""))
        == record["targetPropertyName"]
    )


def _expandSdfListOpItems(listOp):
    """Flatten an Sdf list-op into a single list across all four slots."""
    if not listOp:
        return []
    return (
        list(listOp.explicitItems)
        + list(listOp.prependedItems)
        + list(listOp.appendedItems)
        + list(listOp.addedItems)
    )


def _buildAttrSearchPaths(inputPath, outputPath, targetPort, srcPort):
    """Build (propPath, otherPrimPath, portNameSet) tuples for attr scan.

    Enumerates every direction-hint spelling and the bare/literal name on
    both sides, so connections authored on ``in:``/``input:``/``inputs:``
    (or namespaced attrs such as ``rig1:space``) are all discoverable.
    """
    srcPortNames = set(direction_hint_candidate_names(srcPort, is_input=False))
    tgtPortNames = set(direction_hint_candidate_names(targetPort, is_input=True))
    paths = []
    if targetPort:
        for name in direction_hint_candidate_names(targetPort, is_input=True):
            paths.append((inputPath.AppendProperty(name), outputPath, srcPortNames))
    if srcPort:
        for name in direction_hint_candidate_names(srcPort, is_input=False):
            paths.append((outputPath.AppendProperty(name), inputPath, tgtPortNames))
    return paths


def _buildRelSearchEntries(inputPath, outputPath, targetPort, srcPort):
    """Build (relName, ownerPath, targetPrimPath) tuples for relationship scan."""
    entries = []
    if targetPort:
        entries.append((targetPort, inputPath, outputPath))
    if srcPort:
        entries.append((srcPort, outputPath, inputPath))
    return entries


def _layerAuthorsAttrConnection(view, layer, attrPaths):
    """Return True if any attr path in this layer has a matching connection."""
    for propPath, otherPrimPath, portNames in attrPaths:
        attrSpec = layer.GetAttributeAtPath(propPath)
        if not attrSpec:
            continue
        for conn_path in view._expandSdfListOpItems(attrSpec.connectionPathList):
            if (
                conn_path.GetPrimPath() == otherPrimPath
                and conn_path.name in portNames
            ):
                return True
    return False


def _layerAuthorsRelTarget(view, layer, relEntries):
    """Return True if any relationship in this layer targets the other prim."""
    for relName, ownerPath, targetPrimPath in relEntries:
        propSpec = layer.GetPropertyAtPath(ownerPath.AppendProperty(relName))
        if not propSpec:
            continue
        targetList = getattr(propSpec, "targetPathList", None)
        for target in view._expandSdfListOpItems(targetList):
            if target == targetPrimPath or target.GetPrimPath() == targetPrimPath:
                return True
    return False


def _findConnectionAuthoringLayer(view, inputPrim, targetPort, outputPrim, srcPort):
    """Find which layer in the stack authors the connection.

    Checks both attribute connections and relationship targets.
    Returns the layer identifier (string) or None if not found.
    """
    stage = view.nodeGraph.getStage()
    if not stage:
        return None
    if not inputPrim or not inputPrim.IsValid():
        return None
    if not outputPrim or not outputPrim.IsValid():
        return None

    inputPath = inputPrim.GetPath()
    outputPath = outputPrim.GetPath()
    attrPaths = view._buildAttrSearchPaths(
        inputPath, outputPath, targetPort, srcPort
    )
    relEntries = view._buildRelSearchEntries(
        inputPath, outputPath, targetPort, srcPort
    )

    for layer in stage.GetLayerStack():
        if view._layerAuthorsAttrConnection(layer, attrPaths):
            return layer.identifier
        if view._layerAuthorsRelTarget(layer, relEntries):
            return layer.identifier
    return None


@contextmanager
def _editOnLayer(view, stage, layer_id, fallback_layer):
    """Set the stage edit target to the named layer, restore on exit.

    Falls back to ``fallback_layer`` when ``layer_id`` is empty or no
    longer loadable. Warns when a non-empty layer identifier can no
    longer be resolved so silent fallback is observable.
    """
    layer = Sdf.Layer.Find(layer_id) if layer_id else fallback_layer
    if layer is None:
        if layer_id:
            Tf.Warn(
                f"Layer {layer_id!r} no longer loadable; falling back to "
                f"{fallback_layer.identifier}"
            )
        layer = fallback_layer
    prevTarget = stage.GetEditTarget()
    try:
        stage.SetEditTarget(layer)
        yield layer
    finally:
        stage.SetEditTarget(prevTarget)


def _buildConnectionKwargs(record):
    """Build create/delete-connection kwargs from a deserialized record."""
    if record["sourcePropertyName"] or record["targetPropertyName"]:
        return {
            "source_property_name": record["sourcePropertyName"],
            "target_property_name": record["targetPropertyName"],
        }
    return {}


def _groupLinksByLayer(link_data):
    by_layer = defaultdict(list)
    for link_record in link_data:
        by_layer[link_record.get("_layer_id")].append(link_record)
    return by_layer


def _runLinkOperationOnLayer(
    view, stage, layer_id, link_records, op_name, failure_verb
):
    """Call create_connection or delete_connection per record on a layer.

    ``op_name`` must be ``"create_connection"`` or ``"delete_connection"``.
    ``failure_verb`` appears in the ``Tf.Warn`` emitted on op failure.
    Fallback (when ``layer_id`` is empty or no longer loadable) is the
    stage's current edit target.
    """
    fallback = stage.GetEditTarget().GetLayer()
    with view._editOnLayer(stage, layer_id, fallback):
        for linkRecord in link_records:
            record = view._deserializeLinkRecord(linkRecord)
            srcId = record["sourceNodeId"]
            srcPort = record["sourcePort"]
            tgtId = record["targetNodeId"]
            tgtPort = record["targetPort"]
            outputPrim = stage.GetPrimAtPath(srcId)
            inputPrim = stage.GetPrimAtPath(tgtId)
            if not (inputPrim and outputPrim):
                continue
            library = view._findLibraryForPrim(inputPrim)
            if not library:
                continue
            op = getattr(library, op_name)
            success = op(
                stage,
                outputPrim,
                srcPort,
                inputPrim,
                tgtPort,
                **view._buildConnectionKwargs(record),
            )
            if not success:
                Tf.Warn(
                    f"Failed to {failure_verb} connection "
                    f"{srcId}.{srcPort} -> {tgtId}.{tgtPort}"
                )


def _findLibraryForPrim(view, prim):
    """Find which node library can handle this prim type."""
    if __debug__:
        matches = [
            lib
            for lib in view._nodeLibraries
            if lib.enabled and lib.can_handle_prim(prim)
        ]
        if len(matches) > 1:
            Tf.Warn(
                f"Multiple libraries claim prim {prim.GetPath()}: "
                f"{[m.get_name() for m in matches]}"
            )
    for library in view._nodeLibraries:
        if library.enabled and library.can_handle_prim(prim):
            return library
    return None
