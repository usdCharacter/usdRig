#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Authored prim operations and their stage-scoped history bridge."""

import os
import sys
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import partial
from threading import get_ident
from weakref import ref

from pxr import Gf, Sdf, Tf, Usd

from .. import noodlesValues, primRename
from ..nodeActivation import (
    capture_activation, capture_activation_at, prune_activation_ancestors,
    replay_activation,
)
from ..primAuthoring import PrimAuthor
from ..undoHistory import history_for_stage
from . import intents
from . import selection as _selection
from .legacy import legacy_symbol
from .selection import LinkKey
from . import model as _model
from .stage_adapter import savedRenamePlacement
from .stage_sync import _nodeIdPrimPath


class HistoryBridge:
    """One adapter to the existing per-stage history, not another stack."""

    def __init__(self, stage):
        self.history = history_for_stage(stage)

    def push(self, description, do_func, undo_func, *, view):
        self.history.push(description, do_func, undo_func, view=view)

    def undo(self, view):
        self.history.undo(view)

    def redo(self, view):
        self.history.redo(view)


@dataclass(frozen=True)
class EditSite:
    """Use the recorded mapping, never the invoking view's current target."""

    stage_ref: object
    layer_id: str
    target: object
    paths: tuple

    @classmethod
    def capture(cls, stage, target, paths):
        layer = target.GetLayer()
        if not layer:
            raise RuntimeError("No authoring layer")
        pairs = tuple((Sdf.Path(p), target.MapToSpecPath(Sdf.Path(p))) for p in paths)
        site = cls(ref(stage), layer.identifier, target, pairs)
        site.resolve(stage)
        return site

    def resolve(self, stage):
        if not stage or self.stage_ref() != stage:
            raise RuntimeError("Edit belongs to a different stage")
        layer = _find_layer(stage, self.layer_id)
        if layer != self.target.GetLayer():
            raise RuntimeError(f"Authoring layer was replaced: {self.layer_id}")
        for path, mapped in self.paths:
            if not mapped or self.target.MapToSpecPath(path) != mapped:
                raise RuntimeError(f"Cannot map {path} into {self.layer_id}")
        return self.target


def _find_layer(stage, identifier):
    for layer in (*stage.GetLayerStack(), *stage.GetUsedLayers()):
        if layer.identifier == identifier and not stage.IsLayerMuted(identifier):
            if not layer.permissionToEdit:
                raise RuntimeError(f"Authoring layer is not editable: {identifier}")
            return layer
    raise RuntimeError(f"Authoring layer is unavailable: {identifier}")


class Journal:
    """Register an inverse before every mutation, including the current step."""

    def __init__(self):
        self._restore = []
        self._deferred = []

    def before(self, restore):
        self._restore.append(restore)

    def defer(self, call):
        """Visual-only work for after the commit, when new prims compose."""
        self._deferred.append(call)

    def run_deferred(self):
        pending, self._deferred = self._deferred, []
        for call in pending:
            call()

    def rollback(self):
        errors = []
        pending, self._restore = self._restore, []
        self._deferred = []
        with Sdf.ChangeBlock():
            for restore in reversed(pending):
                try:
                    restore()
                except Exception as error:
                    errors.append(error)
        if errors:
            raise ExceptionGroup("Could not restore edit journal", errors)


def _notice_enabled(notice):
    enabled = notice.isEnabled() if hasattr(notice, 'isEnabled') else True
    return enabled if isinstance(enabled, bool) else True


@contextmanager
def _edit_transaction(stage, target, view, *, batch=False):
    notice = getattr(view, '_noticeHandler', None)
    enabled = _notice_enabled(notice)
    journal = Journal()
    if notice is not None:
        notice.setEnabled(False)
    try:
        if batch:
            # Only prepared Sdf edits and model work independent of composition.
            with _batched(journal), Usd.EditContext(stage, target):
                yield journal
        else:
            with Usd.EditContext(stage, target):
                try:
                    yield journal
                except BaseException:
                    journal.rollback()
                    raise
        # Notices are released and the target restored; deferred visual work
        # runs with the stage composed. SDF and history are already consistent.
        journal.run_deferred()
    finally:
        if notice is not None:
            notice.setEnabled(enabled)


@dataclass(frozen=True)
class _Replay:
    operation: object
    forward: bool
    manages_edit_target = True

    def __call__(self, view):
        return self.operation(self.forward, view)


def _record(history, view, description, replay):
    # No callbacks invoke this view: history supplies the replaying view.
    do, undo = _Replay(replay, True), _Replay(replay, False)
    if 'UsdNoodles.graphView' in sys.modules:
        push = legacy_symbol('_push_undo_command')
        if push is not _push_undo_command:
            push(description, do, undo, view=view)
            return
    history.push(description, do, undo, view=view)


def _activation_step(edit, value):
    replay_activation((replace(edit, before=value),), restore=True)


def _activate_replay(site, records, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    with _edit_transaction(stage, target, view, batch=True) as journal:
        for path, edit, after in records if forward else reversed(records):
            current = capture_activation_at(target, path)
            journal.before(partial(_activation_step, current, current.before))
            _activation_step(edit, after if forward else edit.before)


def _backup_layer(layer):
    copy = Sdf.Layer.CreateAnonymous()
    copy.TransferContent(layer)
    return copy


def _journal_layer(journal, layer):
    backup = _backup_layer(layer)
    journal.before(partial(layer.TransferContent, backup))
    return backup


def _spec_fields(layer):
    return _subtree_fields(layer, Sdf.Path.absoluteRootPath)


_ABSENT = object()
_LIST_FIELDS = {'targetPaths': 'targetPathList', 'connectionPaths': 'connectionPathList'}


def set_spec_field(spec, key, value):
    """Restore a single authored field, including a full Sdf path list op."""
    if isinstance(key, tuple) and key[0] == "timeSamples":
        time = key[1]
        if value is _ABSENT:
            spec.layer.EraseTimeSample(spec.path, time)
        else:
            spec.layer.SetTimeSample(spec.path, time, value)
    elif key in _LIST_FIELDS:
        proxy = getattr(spec, _LIST_FIELDS[key])
        proxy.ClearEdits()
        if value is _ABSENT:
            return
        if value.isExplicit:
            proxy.ClearEditsAndMakeExplicit()
            proxy.explicitItems = value.explicitItems
        else:
            for bucket in ('addedItems', 'prependedItems', 'appendedItems',
                           'deletedItems', 'orderedItems'):
                setattr(proxy, bucket, getattr(value, bucket))
    elif value is _ABSENT:
        spec.ClearInfo(key)
    else:
        spec.SetInfo(key, value)


def _rename_delta(layer, backup, old_path, new_path):
    before, after = _spec_fields(backup), _spec_fields(layer)
    moved = old_path in before and new_path in after and old_path not in after
    changes = []
    matched = set()
    removed = set()
    for path, fields in before.items():
        new = path.ReplacePrefix(old_path, new_path) if moved else path
        if new not in after:
            removed.add(path)
            continue
        matched.add(new)
        for key in sorted(fields.keys() | after[new].keys()):
            old_value = fields.get(key, _ABSENT)
            new_value = after[new].get(key, _ABSENT)
            if key == 'timeSamples':
                old_samples = {} if old_value is _ABSENT else old_value
                new_samples = {} if new_value is _ABSENT else new_value
                for time in sorted(old_samples.keys() | new_samples.keys()):
                    previous = old_samples.get(time, _ABSENT)
                    current = new_samples.get(time, _ABSENT)
                    if previous != current:
                        changes.append((path, new, (key, time), previous, current))
            elif old_value != new_value:
                changes.append((path, new, key, old_value, new_value))
    created = after.keys() - matched
    existence = []
    for paths, source, added in ((removed, backup, False), (created, layer, True)):
        snapshot = Sdf.Layer.CreateAnonymous() if paths else None
        for path in sorted(paths):
            if any(prefix in paths for prefix in path.GetPrefixes()[:-1]):
                continue
            _copy_spec(source, snapshot, path)
            existence.append((path, snapshot, added))
    return layer.identifier, moved, old_path, new_path, tuple(changes), tuple(existence)


def _copy_spec(source, target, path):
    parent = path.GetParentPath()
    if parent != Sdf.Path.absoluteRootPath and not target.GetPrimAtPath(parent):
        Sdf.CreatePrimInLayer(target, parent)
    if not Sdf.CopySpec(source, path, target, path):
        raise RuntimeError(f"Cannot copy spec {path}")


def _remove_spec(layer, path):
    if not layer.GetObjectAtPath(path):
        return
    batch = Sdf.BatchNamespaceEdit()
    batch.Add(path, Sdf.Path.emptyPath)
    if not layer.Apply(batch):
        raise RuntimeError(f"Cannot remove spec {path}")


def _subtree_fields(layer, root):
    fields = {}

    def visit(path):
        if path.IsTargetPath():
            return  # List proxies own these child specs.
        spec = layer.GetObjectAtPath(path)
        if spec:
            fields[path] = {key: spec.GetInfo(key) for key in spec.ListInfoKeys()}

    layer.Traverse(root, visit)
    return fields


@contextmanager
def _batched(journal):
    with Sdf.ChangeBlock():
        try:
            yield
        except BaseException:
            journal.rollback()
            raise


def _resolve_patches(stage, records, forward):
    steps = []
    for identifier, moved, old, new, fields, existence in records if forward else reversed(records):
        layer = _find_layer(stage, identifier)
        batch = None
        if moved:
            batch = Sdf.BatchNamespaceEdit()
            batch.Add(old if forward else new, new if forward else old)
            if not layer.CanApply(batch):
                raise RuntimeError(f"Cannot replay namespace edit on {identifier}")
        for before_path, after_path, _, _, _ in fields:
            path = before_path if forward else after_path
            if not layer.GetObjectAtPath(path):
                raise RuntimeError(f"Cannot replay fixup for {path} on {identifier}")
        steps.append((layer, batch, fields, existence))
    return steps


def _apply_patches(steps, forward, journal):
    for layer, batch, fields, existence in steps:
        if batch is not None:
            _journal_layer(journal, layer)
            if not layer.Apply(batch):
                raise RuntimeError(f"Could not replay namespace edit on {layer.identifier}")
        for old, new, key, before, after in fields:
            path = new if forward else old
            spec = layer.GetObjectAtPath(path)
            current = (spec.GetInfo('timeSamples').get(key[1], _ABSENT)
                       if isinstance(key, tuple) else
                       spec.GetInfo(key) if spec.HasInfo(key) else _ABSENT)
            journal.before(partial(_restore_field, layer, path, key, current))
            set_spec_field(spec, key, after if forward else before)
        for path, snapshot, added in existence:
            spec = layer.GetObjectAtPath(path)
            if forward == added:
                if spec:
                    raise RuntimeError(f"Cannot replace existing spec {path} on {identifier}")
            elif not spec or _subtree_fields(layer, path) != _subtree_fields(snapshot, path):
                raise RuntimeError(f"Cannot remove modified or missing spec {path} on {identifier}")
            if forward == added:
                journal.before(partial(_remove_spec, layer, path))
                _copy_spec(snapshot, layer, path)
            else:
                journal.before(partial(_copy_spec, snapshot, layer, path))
                _remove_spec(layer, path)


def _restore_field(layer, path, key, value):
    set_spec_field(layer.GetObjectAtPath(path), key, value)


def _journal_remap(view, remap, journal):
    reverse = {new: old for old, new in remap.items()}
    selections = None
    if hasattr(view, 'nodes'):
        selections = (frozenset(getattr(view, '_selectedNodes', ())),
                      frozenset(getattr(getattr(view, '_selection', None), 'link_keys', ())))

    def restore():
        view._remapMovedNodes(reverse)
        if selections is not None:
            _restore_selection(view, *selections)

    journal.before(restore)


def _refresh_prim_handles(view, stage, new_ids):
    """Repoint remapped nodes at their composed prims after the commit."""
    nodes = getattr(view, 'nodes', None)
    if not nodes:
        return
    for new_id in new_ids:
        node = nodes.get(new_id)
        if node is not None and getattr(node, '_prim', None) is not None:
            node._prim = stage.GetPrimAtPath(new_id)


def _remap_and_refresh(view, stage, journal, remap):
    applied = view._remapMovedNodes(dict(remap))
    if applied:
        journal.defer(partial(_refresh_prim_handles, view, stage, tuple(applied.values())))
    return applied


def _finish_rename(view, old, new, placement):
    with _model.transaction_for(view):
        node_id = view._adoptRenamedPrim(old, new, *placement)
        view.clearSelection()
        if node_id:
            _selection.select_node(view, node_id)


def _rename_replay(site, records, path_map, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    steps = _resolve_patches(stage, records, forward)
    remap = dict(path_map) if forward else {new: old for old, new in path_map}
    adopts = hasattr(view, '_adoptRenamedPrim')
    placements = [(old, new, savedRenamePlacement(view.nodes, old))
                  for old, new in remap.items()] if adopts else ()
    with _edit_transaction(stage, target, view, batch=True) as journal:
        if not adopts:
            _journal_remap(view, remap, journal)
        _apply_patches(steps, forward, journal)
        if adopts:
            for old, new, placement in placements:
                journal.defer(partial(_finish_rename, view, old, new, placement))
        else:
            _remap_and_refresh(view, stage, journal, remap)
        if hasattr(view, '_afterUsdviewRefresh'):
            view._afterUsdviewRefresh(view._syncSelectionToPrimtree)
    return True


class Editor:
    """Owner-thread dispatch; callbacks are used only during the initiating edit."""

    def __init__(self, *, stage, view, edit_target=None, author=None, model=None,
                 history=None, registry=None):
        self.stage = stage
        self.view = view
        self.edit_target = edit_target or (lambda: self.stage().GetEditTarget())
        self.author = author or PrimAuthor.create_node_prim
        self.model = model
        self.history = history
        self.registry = registry if registry is not None else intents.CommandRegistry()
        self._thread = get_ident()
        for kind, handler in ((intents.Activate, self._activate),
                              (intents.Create, self._create),
                              (intents.Rename, self._rename),
                              (intents.RemoveVisual, self._remove_visual),
                              (intents.SetValue, lambda i: _set_value(self.view(), i)),
                              (intents.Schema, lambda i: _schema_command(self.view(), i)),
                              *((kind, lambda i: _edit_connections(self.view(), i))
                                for kind in (intents.Connect, intents.Disconnect,
                                             intents.Replace, intents.Reconnect))):
            self.registry.register(kind, handler)
        if registry is None:
            self.registry.register(intents.DeleteSelection, self._delete_selected)

    def execute(self, intent):
        if get_ident() != self._thread:
            raise RuntimeError("Editor is confined to its owner thread")
        return self.registry.dispatch(intent)

    @contextmanager
    def creation_author(self, author):
        previous, self.author = self.author, author
        try:
            yield
        finally:
            self.author = previous

    def _context(self, paths):
        stage = self.stage()
        if not stage:
            raise RuntimeError("No stage to edit")
        site = EditSite.capture(stage, self.edit_target(), paths)
        return stage, site, self.view(), self.history or HistoryBridge(stage)

    def _activate(self, intent):
        paths = tuple(dict.fromkeys(intent.paths))
        stage, site, view, history = self._context(paths)
        target = site.resolve(stage)
        records = tuple((path, capture_activation_at(target, path), intent.state)
                        for path in paths)
        if not records:
            return
        if any(not prim or prim.IsPseudoRoot() or prim.IsInstanceProxy()
               for prim in (stage.GetPrimAtPath(path) for path in paths)):
            raise RuntimeError("Cannot author activation for this prim")
        with _edit_transaction(stage, target, view, batch=True) as journal:
            for _, edit, value in records:
                journal.before(partial(_activation_step, edit, edit.before))
                _activation_step(edit, value)
            if self.model:
                self.model(view, intent, paths, journal)
            _record(history, view, 'Activate Nodes' if intent.state else 'Delete Selected',
                    partial(_activate_replay, site, records))

    def _create(self, intent):
        stage, site, view, history = self._context((intent.parent,))
        target = site.resolve(stage)
        with _edit_transaction(stage, target, view, batch=True) as journal:
            backup = _journal_layer(journal, target.GetLayer())
            path = self.author(stage, Sdf.Path(intent.parent), intent.type,
                               Gf.Vec2d(*intent.position))
            if not path:
                raise RuntimeError(f"Failed to create {intent.type}")
            path = Sdf.Path(path)
            site = EditSite.capture(stage, target, (path,))
            spec_path = target.MapToSpecPath(path)
            if not spec_path or not target.GetLayer().GetPrimAtPath(spec_path):
                raise RuntimeError("Failed to get created prim")
            edit = capture_activation_at(target, path)
            old = backup.GetPrimAtPath(edit.path)
            prior = old.GetInfo("active") if old and old.HasInfo("active") else None
            record = replace(edit, before=prior)
            if self.model:
                self.model(view, intent, path, journal)
            _record(history, view, f'Create Node: {intent.type}', partial(
                _create_replay, site, path, record))
        return path

    def _rename(self, intent):
        old = Sdf.Path(intent.parent).AppendChild(intent.old)
        stage, site, view, history = self._context((old,))
        prim = stage.GetPrimAtPath(old)
        status, message = primRename.validate_new_name(prim, intent.new)
        if status != primRename.RENAMED:
            return status, None, message
        new = old.ReplaceName(intent.new)
        target = site.resolve(stage)
        site = EditSite.capture(stage, target, (old, new))
        editor, message = primRename.prepare_rename(stage, prim, intent.new)
        if editor is None:
            return primRename.REFUSED, None, message
        layers = tuple(editor.GetLayersToEdit())
        for layer in layers:
            _find_layer(stage, layer.identifier)
        remap = ((str(old), str(new)),)
        adopts = hasattr(view, '_adoptRenamedPrim')
        placement = savedRenamePlacement(view.nodes, str(old)) if adopts else None
        with _edit_transaction(stage, target, view, batch=True) as journal:
            if not adopts:
                _journal_remap(view, dict(remap), journal)
            before = tuple(_journal_layer(journal, layer) for layer in layers)
            if not editor.ApplyEdits():
                raise RuntimeError("the rename could not be applied")
            mapped_old, mapped_new = (spec_path for _, spec_path in site.paths)
            records = tuple(_rename_delta(layer, backup,
                                          mapped_old if backup.GetPrimAtPath(mapped_old) else old,
                                          mapped_new if backup.GetPrimAtPath(mapped_old) else new)
                            for layer, backup in zip(layers, before))
            if adopts:
                journal.defer(partial(_finish_rename, view, str(old), str(new), placement))
                view._afterUsdviewRefresh(view._syncSelectionToPrimtree)
            elif self.model:
                self.model(view, intent, new, journal)
            else:
                _remap_and_refresh(view, stage, journal, dict(remap))
            _record(history, view, f'Rename: {intent.old} -> {intent.new}',
                    partial(_rename_replay, site, records, remap))
        return primRename.RENAMED, new, ''

    def _delete_selected(self, _intent):
        return _delete_selection(self.view())

    def _remove_visual(self, intent):
        stage, site, view, history = self._context(())
        with _edit_transaction(stage, site.resolve(stage), view) as journal:
            before = VisualState.capture(view)
            journal.before(partial(before.restore, view))
            _remove_visual(view, intent)
            after = VisualState.capture(view)
            if before.nodes != after.nodes or before.links != after.links:
                _record(history, view, 'Remove Nodes', partial(
                    _visual_replay, site, before, after))
            return len(before.nodes) - len(after.nodes)


@dataclass(frozen=True)
class VisualState:
    nodes: tuple
    links: tuple
    selected_nodes: frozenset
    selected_links: frozenset
    owned_links: tuple
    sync: bool
    z_order: int

    @classmethod
    def capture(cls, view):
        return cls(tuple(view.nodes.items()), tuple(view.links),
                   frozenset(getattr(view, '_selectedNodes', ())),
                   frozenset(LinkKey.from_link(view.links[i])
                             for i in getattr(view, '_selectedLinks', ())
                             if 0 <= i < len(view.links)),
                   tuple((key, tuple(getattr(node, 'inputLinks', ())),
                          tuple(getattr(node, 'outputLinks', ())))
                         for key, node in view.nodes.items()),
                   bool(getattr(view.nodeGraph, 'syncSelectionToPrimTree', False)),
                   getattr(view, '_nextZOrder', 0))

    def restore(self, view, *, repaint=False):
        view.nodes.clear()
        view.nodes.update(self.nodes)
        view.links[:] = self.links
        for key, inputs, outputs in self.owned_links:
            node = view.nodes[key]
            for field, links in (('inputLinks', inputs), ('outputLinks', outputs)):
                if hasattr(node, field):
                    getattr(node, field)[:] = links
        view.nodeGraph.syncSelectionToPrimTree = self.sync
        if hasattr(view, '_nextZOrder'):
            view._nextZOrder = self.z_order
        _restore_selection(view, self.selected_nodes, self.selected_links)
        _refresh_visual_state(view, repaint=repaint)


def _restore_selection(view, nodes, links):
    if hasattr(view, '_selection'):
        view._selection.update('replace', nodes, links)
    elif hasattr(view, '_selectedNodes'):
        view._selectedNodes.clear()
        view._selectedNodes.update(nodes)
    for key, node in view.nodes.items():
        node.selected = key in nodes
    for link in getattr(view, "links", ()):
        link.selected = LinkKey.from_link(link) in links
    if hasattr(view, "_linksNeedSync"):
        view._linksNeedSync = True
        view._portHighlightsDirty = True


def _refresh_visual_state(view, *, repaint=True):
    from .model import state_for_view

    state = state_for_view(view)
    if state is not None:
        state._refresh_derived()
    if hasattr(view, 'linksChanged'):
        view.linksChanged = True
        view.textChanged = True
    if repaint and hasattr(view, 'update'):
        view.update()


def _remove_visual(view, intent, *, rebuild=True):
    nodes = set(intent.node_ids) & view.nodes.keys()
    keys = set(intent.link_keys)
    for node_id in nodes:
        del view.nodes[node_id]
        if hasattr(view, '_spatialIndex'):
            view._spatialIndex.removeNode(node_id)
    rebuilt = rebuild and nodes and hasattr(view, '_rebuildLinks')
    if rebuilt:
        view._rebuildLinks()
    removed = [link for link in view.links if LinkKey.from_link(link) in keys or (
        not rebuilt and (link.sourceNodeId in nodes or link.targetNodeId in nodes))]
    if removed:
        removed_ids = {id(link) for link in removed}
        view.links[:] = [link for link in view.links if id(link) not in removed_ids]
        for link in removed:
            for node_id, field in ((link.sourceNodeId, 'outputLinks'),
                                   (link.targetNodeId, 'inputLinks')):
                owned = getattr(view.nodes.get(node_id), field, None)
                if owned is not None:
                    owned[:] = [item for item in owned if item is not link]
    _restore_selection(view, set(getattr(view, '_selectedNodes', ())) - nodes,
                       set(getattr(getattr(view, '_selection', None), 'link_keys', ())) - keys)
    _refresh_visual_state(view)


def _visual_replay(site, before, after, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    source, result = (before, after) if forward else (after, before)
    with _edit_transaction(stage, target, view) as journal:
        _restore_visual_delta(view, source, result, journal)
    return True


def _restore_visual_delta(view, source, result, journal):
    current = VisualState.capture(view)
    journal.before(partial(current.restore, view))
    nodes = _merge_visual(current.nodes, source.nodes, result.nodes, lambda pair: pair[0])
    links = _merge_visual(current.links, source.links, result.links, LinkKey.from_link)
    source_owned = {key: (inputs, outputs) for key, inputs, outputs in source.owned_links}
    result_owned = {key: (inputs, outputs) for key, inputs, outputs in result.owned_links}
    current_owned = {key: (inputs, outputs) for key, inputs, outputs in current.owned_links}
    empty = ((), ())
    links_by_key = {LinkKey.from_link(link): link for link in links}
    owned = tuple((key, *(
        tuple(links_by_key.get(LinkKey.from_link(link), link) for link in
              _merge_visual(live, old, new, LinkKey.from_link))
        for live, old, new in zip(current_owned.get(key, empty),
                                  source_owned.get(key, empty), result_owned.get(key, empty))
    )) for key, _ in nodes)
    result = replace(current, nodes=nodes, links=links, owned_links=owned,
                     selected_nodes=current.selected_nodes & dict(nodes).keys(),
                     selected_links=current.selected_links &
                     {LinkKey.from_link(link) for link in links})
    result.restore(view, repaint=True)


def _merge_visual(current, source, result, key):
    old = {key(item) for item in source}
    new = {key(item) for item in result}
    removed = old - new
    merged = [item for item in current if key(item) not in removed]
    present = old | {key(item) for item in merged}
    for index, item in enumerate(result):
        identity = key(item)
        if identity not in present:
            merged.insert(min(index, len(merged)), item)
            present.add(identity)
    return tuple(merged)


def _create_replay(site, path, edit, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    visual = hasattr(view, '_addNewNodeFast') and hasattr(view, 'links')
    prim = stage.GetPrimAtPath(path) if visual and forward else None
    with _edit_transaction(stage, target, view, batch=True) as journal:
        current = capture_activation_at(target, path)
        if not target.GetLayer().GetPrimAtPath(edit.path):
            raise RuntimeError(f"Created spec is missing: {edit.path}")
        journal.before(partial(_activation_step, current, current.before))
        _activation_step(edit, forward)
        if visual:
            before = VisualState.capture(view)
            journal.before(partial(before.restore, view))
            if forward:
                if str(path) not in view.nodes and not _select_created(
                        view, prim, stage, path.name):
                    raise RuntimeError(f'Failed to add {path.name} to graph')
            else:
                _remove_visual(view, intents.RemoveVisual((str(path),)))
            return True


def _finish_create_visual(view, stage, result, identifier):
    if not _select_created(view, stage.GetPrimAtPath(result), stage, identifier):
        raise RuntimeError(f'Failed to add {identifier} to graph')
    view.update()


def _apply_view_model(view, intent, result, journal):
    stage = view.nodeGraph.getStage()
    if isinstance(intent, intents.Create):
        if not (hasattr(view, '_addNewNodeFast') and hasattr(view, 'links')):
            return
        before = VisualState.capture(view)
        journal.before(partial(before.restore, view))
        if str(result) in view.nodes:
            raise RuntimeError(f'Failed to add {intent.type} to graph')
        journal.defer(partial(_finish_create_visual, view, stage, result, intent.type))
    elif isinstance(intent, intents.Rename):
        old = str(Sdf.Path(intent.parent).AppendChild(intent.old))
        new = str(result)
        _remap_and_refresh(view, stage, journal, {old: new})
        view.clearSelection()
        if new in view.nodes:
            _selection.select_node(view, new)
            view._afterUsdviewRefresh(view._syncSelectionToPrimtree)


def editor_for_view(view, registry):
    return Editor(stage=lambda: view.nodeGraph.getStage(), view=lambda: view,
                  model=_apply_view_model, registry=registry)


def _view_editor(view):
    editor = getattr(view, '_editor', None)
    if editor is None:
        return editor_for_view(view, intents.CommandRegistry())
    return editor


def _initUndoTracking(view, stage):
    """Bind this view to the stage's history without resetting other views."""
    view._undoHistory = HistoryBridge(stage).history


_POSITION = 'ui:nodegraph:node:pos'


@dataclass(frozen=True)
class PositionOpinion:
    prim: object
    path: object
    existed: bool
    fields: object

    @classmethod
    def capture(cls, target, scene_path):
        prim = capture_activation_at(target, scene_path)
        path = target.MapToSpecPath(Sdf.Path(scene_path).AppendProperty(_POSITION))
        if not path or path.GetPrimPath() != prim.path:
            raise RuntimeError(f"Cannot map position for {scene_path}")
        spec = prim.layer.GetAttributeAtPath(path)
        fields = tuple((key, spec.GetInfo(key)) for key in spec.ListInfoKeys()) if spec else ()
        return cls(prim, path, bool(spec), fields)

    def restore(self):
        layer = self.prim.layer
        spec = layer.GetAttributeAtPath(self.path)
        if self.existed:
            fields = dict(self.fields)
            if spec:
                spec.owner.RemoveProperty(spec)
            owner = layer.GetPrimAtPath(self.prim.path)
            if not owner:
                owner = Sdf.CreatePrimInLayer(layer, self.prim.path)
            spec = Sdf.AttributeSpec(
                owner, _POSITION,
                Sdf.ValueTypeNames.Find(fields.get('typeName', 'float2')))
            for key, field_value in self.fields:
                if key != 'typeName':
                    set_spec_field(spec, key, field_value)
        elif spec:
            spec.ClearInfo('default')
            if set(spec.ListInfoKeys()) <= {"custom", "typeName", "variability"}:
                spec.owner.RemoveProperty(spec)
            prune_activation_ancestors(self.prim)


def _position_replay(site, records, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    with _edit_transaction(stage, target, view, batch=True) as journal:
        for node_id, old, new, _, _ in records if forward else reversed(records):
            if old is not None:
                current = PositionOpinion.capture(target, _nodeIdPrimPath(node_id))
                journal.before(current.restore)
                (new if forward else old).restore()
        for node_id, _, _, before, after in records:
            node = view.nodes.get(node_id)
            if node is not None:
                journal.before(partial(node.setDisplayPosition, Gf.Vec2d(node.position)))
                node.setDisplayPosition(Gf.Vec2d(*(after if forward else before)))
        view._resetRenderCachesAfterBulkMove(rebuild_links=False)
        view.update()
    return True


def edit_node_positions(view, positions, description, apply, *, old_display=None):
    stage = view.nodeGraph.getStage()
    if not stage:
        return apply()
    target = stage.GetEditTarget()
    entries = []
    for key, pos in positions.items():
        node = view.nodes.get(key)
        if node is None:
            continue
        prim = stage.GetPrimAtPath(_nodeIdPrimPath(key))
        path = prim.GetPath() if prim and not prim.IsPseudoRoot() and not prim.IsInstanceProxy() else None
        entries.append((key, path, tuple((old_display or {}).get(key, node.position)), tuple(pos)))
    site = EditSite.capture(stage, target, tuple(
        path.AppendProperty(_POSITION) for _, path, _, _ in entries if path))
    old = tuple(PositionOpinion.capture(target, path) if path else None
                for _, path, _, _ in entries)
    with _edit_transaction(stage, target, view, batch=True) as journal:
        for (key, _, pos, _), opinion in zip(entries, old):
            node = view.nodes[key]
            journal.before(partial(_restore_drag_state, node, Gf.Vec2d(*pos),
                                   node._dragging, node._dragPosition))
            if opinion is not None:
                journal.before(opinion.restore)
        apply()
        records = tuple((key, before, PositionOpinion.capture(target, path) if path else None,
                         start, end) for (key, path, start, end), before in zip(entries, old))
        if records and description:
            _record(HistoryBridge(stage), view, description, partial(_position_replay, site, records))


def _restore_drag_state(node, position, dragging, drag_position):
    node.setDisplayPosition(position)
    node._dragging, node._dragPosition = dragging, drag_position


def _applyNodePositions(view, positions):
    """Author ``{nodeId: (x, y)}`` to USD in one batched change and re-render.

    Writes every position inside a single ``Sdf.ChangeBlock`` with the USD
    notice handler disabled (mirrors ``_finalizeNodeDrag`` /
    ``_persistNodePositions``) so the move emits one notice instead of N,
    then re-bakes the node/text/icon caches and refreshes only the moved
    links' endpoints (connections are unchanged, so it skips the full
    connection re-read). Used by auto-layout and its undo/redo.
    """
    enabled = _notice_enabled(view._noticeHandler)
    view._noticeHandler.setEnabled(False)
    try:
        with Sdf.ChangeBlock():
            for node_id, pos in positions.items():
                view._authorNodePosition(node_id, pos)
    finally:
        view._noticeHandler.setEnabled(enabled)
    # Only positions changed (not connections): re-bake the node/text/icon
    # drawables, then refresh the moved links' endpoints incrementally
    # (paintLinks re-syncs to the C++ renderer each frame) -- far cheaper at
    # scale than a full _rebuildLinks that re-reads every prim's connections.
    # Authoring above bumped each node's position version, so
    # _reconcileMovedNodes refreshes the spatial index, link endpoints, and
    # port highlights before the next paint and hit-test. This method only
    # re-bakes the drawables (which bake in the position) and requests a
    # repaint.
    view._resetRenderCachesAfterBulkMove(rebuild_links=False)
    view.update()


def _restoreNodePositions(view, positions):
    """Replay positions even for nodes absent from the invoking canvas."""
    from ..models import write_node_position

    stage = view.nodeGraph.getStage()
    with Sdf.ChangeBlock():
        for node_id, pos in positions.items():
            write_node_position(stage.GetPrimAtPath(_nodeIdPrimPath(node_id)), pos)
            node = view.nodes.get(node_id)
            if node is not None:
                node.setDisplayPosition(Gf.Vec2d(*pos))


def _authorNodePosition(view, node_id, pos):
    """Write one node's position to USD and update its render caches."""
    node = view.nodes.get(node_id)
    if node is None:
        return
    vec = Gf.Vec2d(pos[0], pos[1])
    node._writePositionToUsdRaw(vec)
    node.setDisplayPosition(vec)


def _performUndo(view):
    """Execute undo in this view's stage history."""
    mgr = view._undoHistory
    if mgr is None:
        return
    if not mgr.canUndo():
        Tf.Status("Undo: nothing to undo")
        return

    Tf.Status(f"Undo: executing '{mgr.undoDescription()}'")
    # A bridged backend reloads every view after its replay; reload
    # here only on failure.
    bridged = getattr(mgr, "bridged", lambda: False)()
    failed = False
    reconciled = False
    try:
        reconciled = mgr.undo(view)
    except Exception as e:
        failed = True
        Tf.Warn(f"Undo callback failed: {e}")
    # Legacy callbacks rely on reload; journaled callbacks reconcile in place.
    if failed or (not bridged and not reconciled):
        view._reloadCurrentGraph()


def _performRedo(view):
    """Execute redo in this view's stage history."""
    mgr = view._undoHistory
    if mgr is None:
        return
    if not mgr.canRedo():
        return

    bridged = getattr(mgr, "bridged", lambda: False)()
    failed = False
    reconciled = False
    try:
        reconciled = mgr.redo(view)
    except Exception as e:
        failed = True
        Tf.Warn(f"Redo callback failed: {e}")
    # Always reload so the visual state matches the USD layer state,
    # even when the redo callback raised an exception.
    if failed or (not bridged and not reconciled):
        view._reloadCurrentGraph()


def _persistNodePositions(view, stage, layer):
    """Write all current node positions to the given layer.

    Positions live in the session layer during interactive edits.
    This flushes them to a persistent layer before saving so they
    survive reload.
    """
    previous = stage.GetEditTarget()
    with Sdf.ChangeBlock():
        stage.SetEditTarget(layer)
        journal = Journal()
        if isinstance(layer, Sdf.Layer):
            _journal_layer(journal, layer)
        try:
            for node in view.nodes.values():
                node._writePositionToUsdRaw(node.position)
        except BaseException:
            journal.rollback()
            raise
        finally:
            stage.SetEditTarget(previous)


def _saveLayerAsSpelled(layer):
    """Save `layer`, keeping each path spelled as it was typed.

    A plain ``layer.Save()`` writes every relationship target and
    connection absolute -- USD anchors relative paths the moment they
    are authored -- so a relative path typed in the RigExec Layer
    Opinions panel, or read from the file, would be rewritten by this
    save.  The RigExec plugin's pathSpelling module keeps the spelling;
    it is on the path whenever that plugin is loaded, and without it
    there is no spelling to keep.
    """
    try:
        import pathSpelling
    except ImportError:
        layer.Save()
        return
    report = pathSpelling.SaveLayer(layer)
    if report.note:
        Tf.Warn("Save Stage: %s" % report)


def _saveStage(view):
    """Save the current edit target layer to disk.

    Saves whichever layer is currently selected as the Edit Target.
    Triggered by Ctrl+S (Cmd+S on Mac).
    """
    stage = view.nodeGraph.getStage() if view.nodeGraph else None
    if not stage:
        view._showPopupMessage("No stage to save")
        return

    try:
        layer = stage.GetEditTarget().GetLayer()

        if not layer:
            view._showPopupMessage("No edit target layer to save")
            return

        if layer.anonymous:
            view._showPopupMessage(
                f"Layer '{layer.identifier}' is anonymous — use File > Save As"
            )
            return

        view._persistNodePositions(stage, layer)
        view._saveLayerAsSpelled(layer)

        # Warn if other layers have unsaved edits that this save didn't cover.
        unsaved = []
        try:
            for otherLayer in stage.GetLayerStack(includeSessionLayers=False):
                if otherLayer and otherLayer != layer and otherLayer.dirty:
                    unsaved.append(
                        os.path.basename(otherLayer.identifier)
                        or otherLayer.identifier
                    )
        except Exception:
            pass
        if unsaved:
            names = ", ".join(unsaved[:3])
            suffix = f" (+{len(unsaved) - 3} more)" if len(unsaved) > 3 else ""
            view._showPopupMessage(
                f"Saved layer: {layer.identifier}\n"
                f"⚠ Unsaved changes on: {names}{suffix}"
            )
        else:
            view._showPopupMessage(f"Saved layer: {layer.identifier}")

    except Exception as e:
        view._showPopupMessage(f"Save failed: {e}")
        import traceback

        traceback.print_exc()


def _deleteAllSelected(view):
    return _delete_selection(view)


def _selected_node_pairs(view):
    return [(key, node) for key, node in view.nodes.items() if node.selected]


def _prepare_link_removals(view, stage, links):
    prepared = []
    matched = set()
    for data in links:
        record = view._deserializeLinkRecord(data)
        link = next((link for link in view.links if id(link) not in matched and
                     all(getattr(link, field, '') == value
                         for field, value in record.items())), None)
        if link is not None:
            matched.add(id(link))
            ends = _ends_from_link(link)
        else:
            ends = intents.LinkEndpoints(*(record[field] for field in (
                'sourceNodeId', 'sourcePort', 'targetNodeId', 'targetPort',
                'sourcePropertyName', 'targetPropertyName')))
        # Legacy records lack owner metadata; retain their layer routing.
        link_target = None
        if not ends.owner_property:
            source = stage.GetPrimAtPath(_nodeIdPrimPath(ends.source_id))
            dest = stage.GetPrimAtPath(_nodeIdPrimPath(ends.target_id))
            if source and dest:
                layer_id = view._findConnectionAuthoringLayer(
                    dest, ends.target_port, source, ends.source_port)
                if layer_id:
                    target = stage.GetEditTarget()
                    layer = _find_layer(stage, layer_id)
                    link_target = target if layer == target.GetLayer() else stage.GetEditTargetForLocalLayer(layer)
        prepared.append((record, _prepare_connection(view, stage, ends, False,
                                                       removal_target=link_target)))
    return prepared


def _apply_link_removals(view, removals, journal):
    layers = {}
    for record, writes in removals:
        for write in writes:
            layer = write[0].target.GetLayer()
            if layer.identifier not in layers:
                layers[layer.identifier] = (layer, _journal_layer(journal, layer))
            _write_connection(write)
        view._removeLinkFromGraph(
            record['sourceNodeId'], record['sourcePort'],
            record['targetNodeId'], record['targetPort'],
            sourcePropertyName=record['sourcePropertyName'],
            targetPropertyName=record['targetPropertyName'])
    empty = Sdf.Path.emptyPath
    return tuple(_rename_delta(layer, backup, empty, empty) for layer, backup in layers.values())


def _delete_selection(view, *, links_only_when_incident=False):
    selected = _selected_node_pairs(view)
    links = [] if links_only_when_incident else view._collectSelectedLinkData()
    if selected:
        links.extend(view._collectIncidentLinkKeys({key for key, _ in selected}, links))
    if not (selected or links):
        return 0, 0
    stage = view.nodeGraph.getStage()
    if not stage:
        return 0, 0
    target = stage.GetEditTarget()
    paths = tuple(str(prim.GetPath()) for _, node in selected
                  if (prim := node.getUsdPrim()) and prim.IsValid())
    site = EditSite.capture(stage, target, paths)
    if any(prim.IsPseudoRoot() or prim.IsInstanceProxy()
           for prim in (stage.GetPrimAtPath(path) for path in paths)):
        raise RuntimeError("Cannot author activation for this prim")
    removals = _prepare_link_removals(view, stage, links)
    sites = tuple(write[0] for _, writes in removals for write in writes)
    link_count = len(removals)
    with _edit_transaction(stage, target, view, batch=True) as journal:
        visual = VisualState.capture(view)
        journal.before(partial(visual.restore, view))
        patches = _apply_link_removals(view, removals, journal)
        edits = tuple((path, capture_activation_at(target, path), False) for path in paths)
        for _, edit, value in edits:
            journal.before(partial(_activation_step, edit, edit.before))
            _activation_step(edit, value)
        if edits:
            _remove_visual(view, intents.RemoveVisual(paths), rebuild=False)
        view.clearLinkSelection()
        if edits or link_count:
            view.linksChanged = True
            view.textChanged = True
            view._clearRenderCache()
            view.update()
            final = VisualState.capture(view)
            _record(HistoryBridge(stage), view, 'Delete Selected', partial(
                _delete_replay, site, sites, edits, patches, visual, final))
    if not links_only_when_incident and (edits or link_count):
        parts = []
        if edits:
            parts.append(f"{len(edits)} node(s)")
        if link_count:
            parts.append(f"{link_count} connection(s)")
        msg = "Deleted " + " and ".join(parts)
        view._showPopupMessage(msg)
        Tf.Status(f"[DELETE] {msg}")
    return len(edits), link_count


def _delete_replay(site, link_sites, edits, patches, before, after, forward, view):
    stage = view.nodeGraph.getStage()
    target = site.resolve(stage)
    for link_site in link_sites:
        link_site.resolve(stage)
    steps = _resolve_patches(stage, patches, forward)
    with _edit_transaction(stage, target, view, batch=True) as journal:
        if forward:
            _apply_patches(steps, forward, journal)
        for path, edit, value in edits if forward else reversed(edits):
            current = capture_activation_at(target, path)
            journal.before(partial(_activation_step, current, current.before))
            _activation_step(edit, value if forward else edit.before)
        if not forward:
            _apply_patches(steps, forward, journal)
        _restore_visual_delta(view, before if forward else after,
                              after if forward else before, journal)
    return True


def _deactivateNodesInUsd(view, selectedNodes):
    """Deactivate selected nodes in USD and remove from graph state.

    Returns (activation_edits, count), captured before each authored edit.
    """
    activation_edits = []
    count = 0
    for nodeId, node in selectedNodes:
        prim = node.getUsdPrim()
        if not prim or not prim.IsValid():
            continue
        try:
            edit = capture_activation(prim)
            if not prim.SetActive(False):
                continue
            activation_edits.append(edit)
            del view.nodes[nodeId]
            view._selectedNodes.discard(nodeId)
            view._spatialIndex.removeNode(nodeId)
            count += 1
        except Exception as e:
            Tf.Warn(f"Failed to delete node {nodeId}: {e}")
            import traceback

            traceback.print_exc()
    return activation_edits, count


def _pushDeleteSelectedUndo(view, deleted_links, activation_edits):
    """Push an undo command that restores deleted links and nodes.

    Links and nodes are stored with their layer info so undo/redo
    can operate on the correct layers regardless of the current edit target.
    """
    stage = view.nodeGraph.getStage()
    if not stage:
        Tf.Warn(
            "Cannot register undo for delete: no stage available. "
            f"{len(deleted_links)} link(s) and {len(activation_edits)} node(s) "
            "have already been deleted and will not be undoable."
        )
        return

    # Links already have layer info from _deleteLinksInUsd
    link_data_with_layers = deleted_links

    def _undo(gv, link_data=link_data_with_layers, node_data=tuple(activation_edits)):
        gv._applyRestoreDeleted(link_data, node_data)
        gv.linksChanged = True
        gv._rebuildLinks()
        gv.update()

    def _redo(gv, link_data=link_data_with_layers, node_data=tuple(activation_edits)):
        gv._applyDeleteSelected(link_data, node_data)
        gv.linksChanged = True
        gv._rebuildLinks()
        gv.update()

    legacy_symbol("_push_undo_command")("Delete Selected", _redo, _undo, view=view)


def _applyRestoreDeleted(view, link_data, node_data):
    """Restore recorded activation opinions before reconnecting prims."""
    stage = view.nodeGraph.getStage()
    if not stage:
        return
    replay_activation(node_data, restore=True)
    # Stage recomposition must finish before looking up reactivated prims.
    for layer_id, link_records in view._groupLinksByLayer(link_data).items():
        view._runLinkOperationOnLayer(
            stage, layer_id, link_records, "create_connection", "restore"
        )


def _applyDeleteSelected(view, link_data, node_data):
    """Remove connections, then reapply deletion at the captured spec paths."""
    stage = view.nodeGraph.getStage()
    if not stage:
        return
    for layer_id, link_records in view._groupLinksByLayer(link_data).items():
        view._runLinkOperationOnLayer(
            stage, layer_id, link_records, "delete_connection", "delete"
        )
    replay_activation(node_data, restore=False)


def deleteSelectedNodes(view):
    """Delete all selected nodes (and their incident links) from USD.

    Nodes are deactivated via ``prim.SetActive(False)`` on the stage's
    current edit target. Incident links are removed from whichever layer
    authored the connection (determined via
    ``_findConnectionAuthoringLayer``) so they do not persist as dangling
    links after deactivation.
    """
    if not view.nodes:
        return

    selectedNodes = [
        (nodeId, node) for nodeId, node in view.nodes.items() if node.selected
    ]
    if not selectedNodes:
        Tf.Status("[DELETE] No nodes selected to delete")
        return

    Tf.Status(f"[DELETE] Deleting {len(selectedNodes)} selected node(s)")

    stage = view.nodeGraph.getStage()
    if not stage:
        Tf.Warn("Cannot delete nodes: no stage available")
        return

    legacy_symbol("warn_if_non_persistent_edit_target")(stage)
    deletedCount, _ = _delete_selection(view, links_only_when_incident=True)
    if deletedCount > 0:
        view._showPopupMessage(f"Deleted {deletedCount} node(s)")
        Tf.Status(f"[DELETE] Deleted {deletedCount} node(s)")
    else:
        view._showPopupMessage("No nodes deleted")
        Tf.Status("[DELETE] No nodes were deleted")


def _createNodeFromHotbox(view, node_data, screen_pos):
    """Create a new node using the appropriate library.

    API schema entries are not node types: choosing one applies the schema
    to the selected prims instead of creating a prim.

    Args:
        node_data: Dict with 'identifier', 'library' and 'kind' keys from
            the hotbox
        screen_pos: Screen position tuple (x, y) where hotbox was opened
    """
    from ..apiSchemas import API_SCHEMA_KIND

    if node_data.get("kind") == API_SCHEMA_KIND:
        view._applyApiSchemaFromHotbox(node_data["identifier"])
        return

    if not view._usdviewApi:
        view._showPopupMessage("Cannot create node: no USD stage")
        return

    from pxr import Gf

    stage = view._usdviewApi.stage
    if not stage:
        view._showPopupMessage("Cannot create node: no USD stage")
        return

    library = node_data["library"]
    identifier = node_data["identifier"]

    world_pos = (
        legacy_symbol("QtCore").QPointF(screen_pos[0], screen_pos[1]) / view.zoom
    ) + legacy_symbol("QtCore").QPointF(view.panX, view.panY)

    parent_path = view._resolveParentPath(stage)
    if not parent_path:
        view._showPopupMessage("No Blueprint/Container found to add node to")
        return

    editor = _view_editor(view)
    try:
        # The library can add type-specific attributes beyond PrimAuthor's
        # common prim/position fields. Keep its strategy out of the payload.
        with editor.creation_author(library.create_node):
            prim_path = editor.execute(intents.Create(
                str(parent_path), identifier, (world_pos.x(), world_pos.y())))
    except RuntimeError as error:
        view._showPopupMessage(str(error))
        return

    if prim_path:
        prim_name = str(prim_path).split("/")[-1]
        view._showPopupMessage(f"Created {prim_name}")


def _resolveParentPath(view, stage):
    """Resolve the parent path for node creation.

    The usdview selection wins: new prims are created under the prim
    focused in the prim tree. With nothing selected, fall back to the
    container the graph is currently displaying, then to the first
    Blueprint on the stage, then to the root.
    """
    from ..primAuthoring import PrimAuthor

    selectedParent = view._resolveSelectedParentPrim()
    if selectedParent is not None:
        return selectedParent.GetPath()

    parent_path = view._currentPrimPath if view._currentPrimPath else None
    if not parent_path:
        parent_path = PrimAuthor.get_current_container_path(
            stage, view._usdviewApi.dataModel if view._usdviewApi else None
        )
    return parent_path


def _beginRename(view, nodeId):
    """Open the inline editor on *nodeId*, seeded with its current name."""
    # Blueprint I/O nodes are synthesized rather than backed by a prim, so
    # there is no name to change.
    if "#" in nodeId:
        view._showPopupMessage("This node is not backed by a prim")
        return

    stage = view._usdviewApi.stage if view._usdviewApi else None
    prim = stage.GetPrimAtPath(nodeId) if stage else None
    if not prim or not prim.IsValid():
        view._showWarningPopup("Cannot rename: the prim is gone")
        return

    view._renamingNodeId = nodeId
    view._renameInput.text = prim.GetName()
    view._renameInput.cursor_position = len(view._renameInput.text)
    view.update()


def _cancelRename(view):
    """Abandon the inline editor without authoring anything."""
    view._renamingNodeId = ""
    view._renameInput.clear()
    view.update()


def _commitRename(view):
    """Apply the typed name to the prim, with everything a rename implies.

    A recoverable objection -- an invalid identifier, a name a sibling
    already has -- leaves the editor open so the typo can be fixed. A
    refusal from USD closes it, because no amount of retyping changes a
    composition arc.
    """
    from ..primRename import (
        INVALID_NAME,
        NAME_TAKEN,
        RENAMED,
        UNCHANGED,
    )

    nodeId = view._renamingNodeId
    newName = view._renameInput.text.strip()
    if not nodeId:
        return

    stage = view._usdviewApi.stage if view._usdviewApi else None
    prim = stage.GetPrimAtPath(nodeId) if stage else None
    if not prim or not prim.IsValid():
        view._cancelRename()
        view._showWarningPopup("Cannot rename: the prim is gone")
        return

    oldName = prim.GetName()
    status, newPath, message = _view_editor(view).execute(intents.Rename(
        str(prim.GetPath().GetParentPath()), oldName, newName))

    if status == UNCHANGED:
        view._cancelRename()
        return

    if status in (INVALID_NAME, NAME_TAKEN):
        view._showWarningPopup(message)
        return  # stay open; the name is fixable

    if status != RENAMED:
        view._cancelRename()
        view._showWarningPopup(f"Cannot rename {oldName}: {message}")
        return

    view._renamingNodeId = ""
    view._renameInput.clear()

    view._showPopupMessage(f"Renamed {oldName} to {newName}")
    view.update()


def _select_created(view, prim, stage, identifier):
    newNodeId = view._addNewNodeFast(prim, stage)
    if not newNodeId:
        view._showPopupMessage(f"Failed to add {identifier} to graph")
        return False

    view.textChanged = True
    view._nodeRenderManager.invalidateAll()
    view.nodeGraph.syncSelectionToPrimTree = True

    view.clearSelection()
    if newNodeId in view.nodes:
        _selection.select_node(view, newNodeId)
        # ...and carry that into usdview, which is the other half of
        # "the new prim is selected". Marking node.selected only lights
        # it up on this canvas; the prim tree, the property editor and
        # the viewport all read dataModel.selection, and nothing else on
        # the creation path writes it.
        #
        # Every other selecting path already syncs -- click, marquee,
        # Select All, navigate-to-connected. Creation was the one that
        # did not, so a freshly made prim was the only way to end up
        # with the two halves of the app disagreeing about what is
        # selected.
        view._syncSelectionToPrimtree()

    return True


def _onNodeCreated(view, prim, prim_path, stage, identifier):
    """Handle post-creation setup: add to graph, select, push undo."""
    if not _select_created(view, prim, stage, identifier):
        return

    created_path = str(prim_path)
    legacy_symbol("_push_undo_command")(
        f"Create Node: {identifier}",
        _make_prim_activation_undo([created_path], activate=True),
        _make_prim_activation_undo([created_path], activate=False),
        view=view,
    )

    view.update()


def _push_undo_command(description, do_func, undo_func, *, view):
    """Record an already applied edit in the view's stage history.

    Callbacks accept the view invoking undo/redo and capture edit data only.
    """
    if view._undoHistory is not None:
        HistoryBridge(view.nodeGraph.getStage()).push(description, do_func, undo_func, view=view)


def _make_prim_activation_undo(paths, activate):
    """Return a callable that activates or deactivates prims at *paths*.

    The activation opinion is authored to the stage's current edit target.

    Args:
        paths: List of prim path strings.
        activate: True to set prims active, False to deactivate.
    """

    def _fn(gv, _paths=paths, _activate=activate):
        stage = gv.nodeGraph.getStage()
        if not stage:
            return
        for path_str in _paths:
            prim = stage.GetPrimAtPath(path_str)
            if prim:
                prim.SetActive(_activate)

    return _fn


def _make_rename_undo(parent_path, from_name, to_name):
    """Return a callable that renames a sibling of *parent_path*.

    Named by (parent, name) rather than by a captured prim, because the prim
    the caller held is invalidated by the rename that this reverses -- the
    path it was fetched from no longer resolves.
    """

    def _fn(gv, _parent=parent_path, _from=from_name, _to=to_name):
        from ..primRename import RENAMED, rename_prim

        stage = gv.nodeGraph.getStage()
        if not stage:
            return
        oldPath = f"{_parent}/{_from}".replace("//", "/")
        prim = stage.GetPrimAtPath(oldPath)
        if not prim or not prim.IsValid():
            Tf.Warn("Rename undo: no prim named %s under %s" % (_from, _parent))
            return
        placement = savedRenamePlacement(gv.nodes, oldPath)
        status, newPath, message = rename_prim(stage, prim, _to)
        if status != RENAMED:
            Tf.Warn("Rename undo failed (%s): %s" % (status, message))
            return
        # Follow the node across, exactly as the forward rename does. Reloading
        # instead would undo the name and lose the graph in the same gesture.
        gv._adoptRenamedPrim(oldPath, str(newPath), *placement)

    return _fn


def _make_connection_edit(
    src_id,
    src_port,
    tgt_id,
    tgt_port,
    create,
    *,
    source_property_name="",
    target_property_name="",
):
    """Return a callable that creates or deletes a single USD connection.

    The connection is authored on the stage's current edit target. If the
    user has selected a sublayer, the connection lands there.

    Args:
        create: True to create the connection, False to delete it.
    """

    def _fn(gv):
        stage = gv.nodeGraph.getStage()
        if not stage:
            return
        out_p = stage.GetPrimAtPath(src_id)
        in_p = stage.GetPrimAtPath(tgt_id)
        if not (out_p and in_p):
            return
        lib = gv._findLibraryForPrim(in_p)
        if not lib:
            return
        connection_kwargs = {}
        if source_property_name or target_property_name:
            connection_kwargs = {
                "source_property_name": source_property_name,
                "target_property_name": target_property_name,
            }
        if create:
            lib.create_connection(
                stage, out_p, src_port, in_p, tgt_port, **connection_kwargs
            )
        else:
            lib.delete_connection(
                stage, out_p, src_port, in_p, tgt_port, **connection_kwargs
            )

    return _fn


def _make_reconnect_edits(
    old_src,
    old_src_port,
    old_tgt,
    old_tgt_port,
    new_src,
    new_src_port,
    new_tgt,
    new_tgt_port,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    """Return (redo, undo) callables for a reconnection."""
    delete_old = _make_connection_edit(
        old_src, old_src_port, old_tgt, old_tgt_port, create=False
    )
    create_old = _make_connection_edit(
        old_src, old_src_port, old_tgt, old_tgt_port, create=True
    )
    delete_new = _make_connection_edit(
        new_src,
        new_src_port,
        new_tgt,
        new_tgt_port,
        create=False,
        source_property_name=sourcePropertyName,
        target_property_name=targetPropertyName,
    )
    create_new = _make_connection_edit(
        new_src,
        new_src_port,
        new_tgt,
        new_tgt_port,
        create=True,
        source_property_name=sourcePropertyName,
        target_property_name=targetPropertyName,
    )

    def redo(gv):
        delete_old(gv)
        create_new(gv)
        gv.linksChanged = True
        gv.update()

    def undo(gv):
        delete_new(gv)
        create_old(gv)
        gv.linksChanged = True
        gv.update()

    return redo, undo


def _finishConnectionReplacement(
    view,
    oldLink,
    outputNodeId,
    outputPortName,
    inputNodeId,
    inputPortName,
    reconnect_state=None,
    outputPropertyName="",
    inputPropertyName="",
):
    """
    Finish replacing an old connection after highlighting it.

    Args:
        oldLink: The old link to remove
        outputNodeId: ID of the new output node
        outputPortName: Name of the new output port
        inputNodeId: ID of the new input node
        inputPortName: Name of the new input port
        reconnect_state: Captured reconnect endpoints, if this was a
            reconnect drag.  Passed through to _createConnection so
            the deferred call is immune to instance-state overwrites.
    """
    oldLink.highlighted = False
    oldLink.highlightColor = None
    view._linksNeedSync = True

    if isinstance(view.nodeGraph.getStage(), Usd.Stage):
        new = intents.LinkEndpoints(outputNodeId, outputPortName, inputNodeId,
                                    inputPortName, outputPropertyName, inputPropertyName)
        old = _ends_from_link(oldLink)
        command = (intents.Reconnect(*_reconnect_command_args(reconnect_state, new, (old,)))
                   if reconnect_state is not None else intents.Replace(old, new))
        return _dispatch_connection(view, command, reconnect_state)

    view._createConnection(
        outputNodeId,
        outputPortName,
        inputNodeId,
        inputPortName,
        reconnect_state,
        sourcePropertyName=outputPropertyName,
        targetPropertyName=inputPropertyName,
    )

    # Rebuild links from USD so the UI reflects whether the old
    # connection was replaced or both coexist (multi-connection pins).
    view.linksChanged = True
    view._rebuildLinks()


def _deleteLinksInUsd(view, stage, linksToDelete):
    """Delete connections in USD and remove from local link list.

    Returns (actually_deleted_links, count). Each deleted link record
    includes '_layer_id' indicating which layer it was deleted from.
    Deletes are issued via the authoring layer when one is identified;
    a missing-but-named layer triggers a Tf.Warn via _editOnLayer.
    """
    actually_deleted = []
    count = 0
    for linkRecord in linksToDelete:
        record = view._deserializeLinkRecord(linkRecord)
        srcId = record["sourceNodeId"]
        srcPort = record["sourcePort"]
        tgtId = record["targetNodeId"]
        tgtPort = record["targetPort"]
        inputNode = view.nodes.get(tgtId)
        outputNode = view.nodes.get(srcId)
        if not inputNode or not outputNode:
            continue
        inputPrim = inputNode.getUsdPrim()
        outputPrim = outputNode.getUsdPrim()
        if not inputPrim or not outputPrim:
            continue
        library = view._findLibraryForPrim(inputPrim)
        if not library:
            continue

        layer_id = view._findConnectionAuthoringLayer(
            inputPrim, tgtPort, outputPrim, srcPort
        )
        connection_kwargs = view._buildConnectionKwargs(record)

        fallback = stage.GetEditTarget().GetLayer()
        with view._editOnLayer(stage, layer_id, fallback):
            success = library.delete_connection(
                stage,
                outputPrim,
                srcPort,
                inputPrim,
                tgtPort,
                **connection_kwargs,
            )

        if success:
            record_with_layer = dict(record)
            record_with_layer["_layer_id"] = layer_id
            actually_deleted.append(record_with_layer)
            view._removeLinkFromGraph(
                srcId,
                srcPort,
                tgtId,
                tgtPort,
                sourcePropertyName=record["sourcePropertyName"],
                targetPropertyName=record["targetPropertyName"],
            )
            count += 1
        elif layer_id is None:
            Tf.Warn(
                f"Failed to delete connection {srcId}.{srcPort} -> "
                f"{tgtId}.{tgtPort}: authoring layer not found and "
                f"fallback deletion returned False"
            )
    return actually_deleted, count


def _abortConnection(view, message, reconnect_state=None):
    Tf.Warn(message)
    if view._reconnectingLink:
        view._cancelLinkReconnect()
    elif reconnect_state is not None:
        oldSrcId, oldSrcPort, oldTgtId, oldTgtPort = reconnect_state[:4]
        srcNode = view.nodes.get(oldSrcId)
        tgtNode = view.nodes.get(oldTgtId)
        if srcNode and tgtNode:
            view._addLinkToGraph(
                oldSrcId, oldSrcPort, oldTgtId, oldTgtPort, srcNode, tgtNode
            )


def _resolveConnectionEndpoints(view, outputNodeId, inputNodeId):
    outputNode = view.nodes.get(outputNodeId)
    inputNode = view.nodes.get(inputNodeId)
    if not outputNode or not inputNode:
        view._abortConnection("Cannot create connection: invalid node IDs")
        return None

    outputPrim = outputNode.getUsdPrim()
    inputPrim = inputNode.getUsdPrim()
    if not outputPrim or not inputPrim:
        view._abortConnection("Cannot create connection: invalid USD prims")
        return None

    library = view._findLibraryForPrim(inputPrim)
    if not library:
        view._abortConnection(
            f"Cannot create connection: no library handles prim type '{inputPrim.GetTypeName()}'"
        )
        return None

    return outputNode, inputNode, outputPrim, inputPrim, library


def _deleteOldConnection(view, stage, oldSrcId, oldSrcPort, oldTgtId, oldTgtPort):
    oldInputNode = view.nodes.get(oldTgtId)
    oldOutputNode = view.nodes.get(oldSrcId)
    if not oldInputNode or not oldOutputNode:
        return False
    oldInputPrim = oldInputNode.getUsdPrim()
    oldOutputPrim = oldOutputNode.getUsdPrim()
    if not oldInputPrim or not oldOutputPrim:
        return False
    oldLib = view._findLibraryForPrim(oldInputPrim)
    if not oldLib:
        return False
    return oldLib.delete_connection(
        stage, oldOutputPrim, oldSrcPort, oldInputPrim, oldTgtPort
    )


def _createConnection(
    view,
    outputNodeId,
    outputPort,
    inputNodeId,
    inputPort,
    reconnect_state=None,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    """Create a connection in USD via the appropriate node library."""
    if isinstance(view.nodeGraph.getStage(), Usd.Stage):
        new = intents.LinkEndpoints(outputNodeId, outputPort, inputNodeId, inputPort,
                                    sourcePropertyName, targetPropertyName)
        if reconnect_state is None and view._reconnectingLink:
            reconnect_state = _reconnect_endpoints(view)
        command = (intents.Reconnect(*_reconnect_command_args(reconnect_state, new))
                   if reconnect_state is not None else intents.Connect(new))
        return _dispatch_connection(view, command, reconnect_state)

    resolved = view._resolveConnectionEndpoints(outputNodeId, inputNodeId)
    if not resolved:
        view._abortConnection(
            "Failed to resolve connection endpoints", reconnect_state
        )
        return

    outputNode, inputNode, outputPrim, inputPrim, library = resolved

    stage = view.nodeGraph.getStage()
    if not stage:
        Tf.Warn("Cannot create connection: no stage")
        return
    legacy_symbol("warn_if_non_persistent_edit_target")(stage)

    view._noticeHandler.setEnabled(False)
    try:
        connection_kwargs = {}
        if sourcePropertyName or targetPropertyName:
            connection_kwargs = {
                "source_property_name": sourcePropertyName,
                "target_property_name": targetPropertyName,
            }
        success = library.create_connection(
            stage,
            outputPrim,
            outputPort,
            inputPrim,
            inputPort,
            **connection_kwargs,
        )

        if not success:
            view._abortConnection(
                "Failed to create connection in USD", reconnect_state
            )
            return

        if reconnect_state is not None:
            isReconnect = True
            oldSrcId, oldSrcPort, oldTgtId, oldTgtPort = reconnect_state[:4]
        else:
            isReconnect = view._reconnectingLink
            oldSrcId = view._reconnectOldSourceNodeId
            oldSrcPort = view._reconnectOldSourcePort
            oldTgtId = view._reconnectOldTargetNodeId
            oldTgtPort = view._reconnectOldTargetPort

        if isReconnect:
            view._clearReconnectState()

            if not view._deleteOldConnection(
                stage, oldSrcId, oldSrcPort, oldTgtId, oldTgtPort
            ):
                # Old connection may have been implicitly replaced by
                # create_connection (e.g. SetConnections on the same
                # input attribute). Continue with the new connection.
                Tf.Status(
                    "Reconnect: old connection already removed "
                    f"({oldSrcId}.{oldSrcPort} -> {oldTgtId}.{oldTgtPort})"
                )

        view._addLinkToGraph(
            outputNodeId,
            outputPort,
            inputNodeId,
            inputPort,
            outputNode,
            inputNode,
            sourcePropertyName=sourcePropertyName,
            targetPropertyName=targetPropertyName,
        )

        if isReconnect:
            redo_fn, undo_fn = _make_reconnect_edits(
                oldSrcId,
                oldSrcPort,
                oldTgtId,
                oldTgtPort,
                outputNodeId,
                outputPort,
                inputNodeId,
                inputPort,
                sourcePropertyName=sourcePropertyName,
                targetPropertyName=targetPropertyName,
            )
            legacy_symbol("_push_undo_command")("Reconnect", redo_fn, undo_fn, view=view)
        else:
            legacy_symbol("_push_undo_command")(
                "Create Connection",
                _make_connection_edit(
                    outputNodeId,
                    outputPort,
                    inputNodeId,
                    inputPort,
                    create=True,
                    source_property_name=sourcePropertyName,
                    target_property_name=targetPropertyName,
                ),
                _make_connection_edit(
                    outputNodeId,
                    outputPort,
                    inputNodeId,
                    inputPort,
                    create=False,
                    source_property_name=sourcePropertyName,
                    target_property_name=targetPropertyName,
                ),
                view=view,
            )

    finally:
        # The notice handler is suppressed across the authoring window so
        # _handleUsdChanges cannot drop the cached per-prim link scan that
        # back-reference resolution reads from.  Drop it explicitly here so
        # a later re-add of an absent prim does not resurrect a connection
        # we just authored away.
        view._invalidateBackReferenceLinkDataCache()
        view._noticeHandler.setEnabled(True)
        view.update()


def _deleteConnection(
    view,
    outputNodeId,
    outputPort,
    inputNodeId,
    inputPort,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    """Delete a connection in USD via the appropriate node library."""
    if isinstance(view.nodeGraph.getStage(), Usd.Stage):
        return _dispatch_connection(view, intents.Disconnect(intents.LinkEndpoints(
            outputNodeId, outputPort, inputNodeId, inputPort,
            sourcePropertyName, targetPropertyName)))

    inputNode = view.nodes.get(inputNodeId)
    outputNode = view.nodes.get(outputNodeId)
    if not inputNode or not outputNode:
        return

    inputPrim = inputNode.getUsdPrim()
    outputPrim = outputNode.getUsdPrim()
    if not inputPrim or not outputPrim:
        return

    library = view._findLibraryForPrim(inputPrim)
    if not library:
        Tf.Warn(
            f"Cannot delete connection: no library handles prim type '{inputPrim.GetTypeName()}'"
        )
        return

    view._noticeHandler.setEnabled(False)
    try:
        connection_kwargs = {}
        if sourcePropertyName or targetPropertyName:
            connection_kwargs = {
                "source_property_name": sourcePropertyName,
                "target_property_name": targetPropertyName,
            }
        success = library.delete_connection(
            view.nodeGraph.getStage(),
            outputPrim,
            outputPort,
            inputPrim,
            inputPort,
            **connection_kwargs,
        )

        if success:
            view._removeLinkFromGraph(
                outputNodeId,
                outputPort,
                inputNodeId,
                inputPort,
                sourcePropertyName=sourcePropertyName,
                targetPropertyName=targetPropertyName,
            )

            legacy_symbol("_push_undo_command")(
                "Delete Connection",
                _make_connection_edit(
                    outputNodeId,
                    outputPort,
                    inputNodeId,
                    inputPort,
                    create=False,
                    source_property_name=sourcePropertyName,
                    target_property_name=targetPropertyName,
                ),
                _make_connection_edit(
                    outputNodeId,
                    outputPort,
                    inputNodeId,
                    inputPort,
                    create=True,
                    source_property_name=sourcePropertyName,
                    target_property_name=targetPropertyName,
                ),
                view=view,
            )
    finally:
        # Mirror _createConnection: drop the back-reference cache while the
        # notice handler is still suppressed, so a later re-add of an
        # absent prim does not resurrect the connection we just deleted.
        view._invalidateBackReferenceLinkDataCache()
        view._noticeHandler.setEnabled(True)


def _applyApiSchemaFromHotbox(view, identifier):
    """Apply an API schema chosen in the hotbox to the selected prims.

    Prims the schema forbids (its canOnlyApplyTo types) are skipped rather
    than authored, and reported in the summary popup. Multiple-apply
    schemas get a generated instance name per prim.

    Args:
        identifier: USD schema identifier, e.g. "PhysicsRigidBodyAPI"
    """
    selectedPrims = view._resolveSelectedPrims()
    if not selectedPrims:
        view._showWarningPopup(f"Select a prim to apply {identifier}")
        return

    applied, already_present, skipped = _view_editor(view).execute(intents.Schema(
        tuple(str(prim.GetPath()) for prim in selectedPrims), identifier, 'apply'))
    view._showApiSchemaResult(identifier, applied, already_present, skipped)


def _showApiSchemaResult(view, identifier, applied, alreadyPresent, skipped):
    """Report what an API schema application did.

    Args:
        identifier: Schema identifier that was applied
        applied: List of (primPath, instanceName) pairs newly authored
        alreadyPresent: Count of prims that already carried the schema
        skipped: List of (primName, reason) pairs that were skipped
    """
    if not applied and not alreadyPresent:
        reason = skipped[0][1] if skipped else "no prims selected"
        view._showWarningPopup(f"Could not apply {identifier}: {reason}")
        return

    if applied:
        noun = "prim" if len(applied) == 1 else "prims"
        summary = f"Applied {identifier} to {len(applied)} {noun}"
    else:
        noun = "prim" if alreadyPresent == 1 else "prims"
        summary = f"{identifier} already on {alreadyPresent} {noun}"

    notes = []
    if applied and alreadyPresent:
        notes.append(f"{alreadyPresent} already had it")
    if skipped:
        names = ", ".join(name for name, _ in skipped[:3])
        if len(skipped) > 3:
            names += f", +{len(skipped) - 3} more"
        notes.append(f"skipped {len(skipped)}: {names}")

    if not notes:
        view._showPopupMessage(summary)
        return

    message = f"{summary} ({'; '.join(notes)})"
    if skipped:
        view._showWarningPopup(message)
    else:
        view._showPopupMessage(message)


def _writeValueComponents(view, state, components, warn=True):
    """Author *components* and refresh just that node's cells."""
    intent = intents.SetValue(
        str(Sdf.Path(state['primPath']).AppendProperty(state['prop'])),
        tuple(components), view._cachedValueWriteMode, state.get('time'))
    if warn:
        return _view_editor(view).execute(intent)
    return _set_value(view, intent, warn=False, preview=state.setdefault('preview', {}))


def _make_value_edit(prim_path, property_name, components, time_code, mode):
    """Return a callable that authors *components* onto one attribute.

    Named by path rather than by a captured ``Usd.Prim``: ``_performUndo``
    calls ``_reloadCurrentGraph`` afterwards, which rebuilds every node from
    the stage, so a prim handle captured here would be talking about an
    object nobody else is looking at any more.
    """

    def _fn(
        gv,
        _path=str(prim_path),
        _prop=property_name,
        _components=tuple(components),
        _time=time_code,
        _mode=mode,
    ):
        stage = gv.nodeGraph.getStage()
        if not stage:
            return
        prim = stage.GetPrimAtPath(_path)
        if not prim or not prim.IsValid():
            Tf.Warn("Value undo: no prim at %s" % _path)
            return
        ok, reason = noodlesValues.write_value(
            stage, prim, _prop, _components, _time, mode=_mode
        )
        if not ok:
            Tf.Warn("Value undo failed on %s.%s: %s" % (_path, _prop, reason))
            return
        node = gv.nodes.get(_path)
        if node is not None and hasattr(node, "invalidate_value_row"):
            node.invalidate_value_row(_prop)

    return _fn


def _pushValueUndo(view, primPath, propertyName, oldComponents, newComponents,
                   timeCode):
    """Record the local opinion delta accumulated by a completed drag."""
    preview = getattr(view, '_finishedValuePreview', None)
    if preview:
        site = preview['site']
        stage = view.nodeGraph.getStage()
        site.resolve(stage)
        path = Sdf.Path(primPath).AppendProperty(propertyName)
        _record_opinions(view, stage, f'Set {path}', (site,), preview['patches'],
                         partial(_refresh_value, path))
    else:
        # Direct callers supply a committed edit without a drag journal.
        mode = view._cachedValueWriteMode
        legacy_symbol('_push_undo_command')(
            f'Set {primPath}.{propertyName}',
            _make_value_edit(primPath, propertyName, newComponents, timeCode, mode),
            _make_value_edit(primPath, propertyName, oldComponents, timeCode, mode),
            view=view)


def _updateMung(view, event):
    """Drag a number left/right. Live preview, nothing on the undo stack."""
    state = view._mungState
    if state is None or not state["mungable"]:
        return
    dx = float(event.position().x()) - state["baseScreenX"]
    if not view._mungActive:
        if abs(dx) < float(view._cachedMungDragThresholdPx):
            return
        view._mungActive = True
        state['noticeEnabled'] = _notice_enabled(view._noticeHandler)
        # Our own writes must not come back to us as notices -- the same
        # bracket _finalizeNodeDrag uses, for the same reason. USD still
        # notifies Hydra, so usdview's viewport updates live.
        view._noticeHandler.setEnabled(False)
    QtCore = legacy_symbol("QtCore")
    mods = event.modifiers()
    mult = noodlesValues.mung_multiplier(
        bool(mods & QtCore.Qt.ShiftModifier),
        bool(mods & QtCore.Qt.ControlModifier),
    )
    # Absolute, from the press position and the ORIGINAL value, never
    # incremental: dragging back to where the press landed then restores
    # exactly the value it started at, instead of accumulating drift.
    value = state["base"] + dx * state["step"] * mult
    components = noodlesValues.apply_delta(
        state["original"], state["comp"], value, state["typeName"]
    )
    if components != state["current"]:
        if view._writeValueComponents(state, components, warn=False)[0]:
            state["current"] = components


def _finishMung(view):
    """Release: commit the drag as ONE undo entry, or open the editor."""
    state = view._mungState
    view._mungState = None
    if state is None:
        return
    if not view._mungActive:
        # Never crossed the threshold, so nothing was authored and nothing
        # is on the undo stack: this was a click, and a click types.
        view._beginValueEdit(state["nodeId"], state["prop"], state["comp"])
        return

    view._mungActive = False
    final = tuple(state["current"])
    original = tuple(state["original"])
    preview = state.get('preview')
    if preview:
        if final == original:
            _cancel_value_preview(view, state)
            return
        try:
            stage = view.nodeGraph.getStage()
            preview['site'].resolve(stage)
            noodlesValues._warn_edit_target(stage)
            view._finishedValuePreview = preview
            view._pushValueUndo(state['primPath'], state['prop'], original, final, state['time'])
        finally:
            view._finishedValuePreview = None
            view._noticeHandler.setEnabled(state.get('noticeEnabled', True))
        view._startValueFlash(state['nodeId'], state['prop'], 'commit')
    else:
        view._noticeHandler.setEnabled(state.get('noticeEnabled', True))


def _cancel_value_preview(view, state):
    try:
        preview = state.get('preview')
        if preview:
            path = preview['site'].paths[0][0]
            _replay_opinions((preview['site'],), preview['patches'],
                             partial(_refresh_value, path), False, view)
    finally:
        view._noticeHandler.setEnabled(state.get('noticeEnabled', True))


def _cancelMung(view):
    """Esc mid-drag: restore the local opinion without recording history."""
    state = view._mungState
    view._mungState = None
    if state is not None:
        if view._mungActive:
            view._mungActive = False
            _cancel_value_preview(view, state)
        view.update()


def _toggleBoolValue(view, hit):
    """A checkbox: one click, one write, one undo entry."""
    old = tuple(hit.row.components)
    new = (not bool(old[0]),)
    state = {
        "nodeId": hit.nodeId,
        "primPath": hit.primPath,
        "prop": hit.propertyName,
        "time": view._currentTimeCode,
    }
    ok, reason = view._writeValueComponents(state, new, warn=True)
    if not ok:
        view._showWarningPopup("%s: %s" % (hit.propertyName, reason))
        return
    view._startValueFlash(hit.nodeId, hit.propertyName, "commit")


def _beginValueEdit(view, nodeId, propertyName, comp):
    """Open the keyboard editor on one component, seeded unelided."""
    row = view._findValueRow(nodeId, propertyName)
    if row is None or not row.editable:
        return
    if row.kind == noodlesValues.KIND_BOOL:
        return
    comp = max(0, min(int(comp), row.count - 1))
    text = noodlesValues.format_component(row.components[comp], row.type_name)
    view._valueEditTarget = (nodeId, propertyName, comp)
    view._valueEditInput.text = text
    view._valueEditInput.cursor_position = len(text)
    view._valueEditFresh = True
    view._valueEditError = False
    view.update()


def _cancelValueEdit(view):
    """Close the editor without authoring anything."""
    view._valueEditTarget = None
    view._valueEditInput.clear()
    view._valueEditFresh = True
    view._valueEditError = False
    view.update()


def _commitValueEdit(view):
    """Parse and author the typed value. False means "still open".

    A recoverable objection -- text that is not a number, a token that is
    not in ``allowedTokens`` -- leaves the editor open so the typo can be
    fixed, exactly as ``_commitRename`` does for a fixable name.
    """
    target = view._valueEditTarget
    if target is None:
        return True
    nodeId, propertyName, comp = target
    row = view._findValueRow(nodeId, propertyName)
    if row is None:
        view._cancelValueEdit()
        return True

    text = view._valueEditInput.text
    ok, value = noodlesValues.parse_component(text, row.type_name)
    if not ok:
        # The editor stays open with the error tint on its outline: the
        # typo is fixable, and closing it would throw away what was typed.
        view._valueEditError = True
        view._showWarningPopup(
            "%r is not a %s" % (text.strip(), row.type_name)
        )
        return False
    if row.kind == noodlesValues.KIND_TOKEN_ENUM and row.tokens:
        if value not in row.tokens:
            view._valueEditError = True
            view._showWarningPopup(
                "%s is not one of: %s" % (value, ", ".join(row.tokens))
            )
            return False

    old = tuple(row.components)
    new = noodlesValues.apply_delta(old, comp, value, row.type_name)
    view._valueEditTarget = None
    view._valueEditInput.clear()
    view._valueEditFresh = True
    view._valueEditError = False
    if new == old:
        view.update()
        return True

    state = {
        "nodeId": nodeId,
        "primPath": _nodeIdPrimPath(nodeId),
        "prop": propertyName,
        "time": view._currentTimeCode,
    }
    written, reason = view._writeValueComponents(state, new, warn=True)
    if not written:
        view._showWarningPopup("%s: %s" % (propertyName, reason))
        view._startValueFlash(nodeId, propertyName, "error")
        return True
    view._startValueFlash(nodeId, propertyName, "commit")
    view.update()
    return True


def _advanceValueEdit(view, backwards=False):
    """Tab: commit, then open the next editable cell, wrapping."""
    target = view._valueEditTarget
    if target is None:
        return
    nodeId, propertyName, comp = target
    if not view._commitValueEdit():
        return  # the value did not parse; stay where the typo is
    cells = view._editableValueCells(nodeId)
    if not cells:
        return
    try:
        index = cells.index((propertyName, comp))
    except ValueError:
        index = -1 if backwards else 0
    else:
        index = (index + (-1 if backwards else 1)) % len(cells)
    nextProp, nextComp = cells[index]
    view._beginValueEdit(nodeId, nextProp, nextComp)


def _commitTokenPopup(view):
    popup = view._tokenPopup
    view._tokenPopup = None
    if popup is None:
        return
    item = view._tokenPopupList.get_selected_item()
    if item is None:
        view.update()
        return
    new = (str(item.name),)
    old = tuple(popup["original"])
    if new == old:
        view.update()
        return
    ok, reason = view._writeValueComponents(popup, new, warn=True)
    if not ok:
        view._showWarningPopup("%s: %s" % (popup["prop"], reason))
        view.update()
        return
    view._startValueFlash(popup["nodeId"], popup["prop"], "commit")
    view.update()

def _replay_opinions(sites, patches, refresh, forward, view):
    stage = view.nodeGraph.getStage()
    targets = [site.resolve(stage) for site in sites]
    steps = _resolve_patches(stage, patches, forward)
    with _edit_transaction(stage, targets[0], view, batch=True) as journal:
        _apply_patches(steps, forward, journal)
        journal.defer(partial(refresh, view))
    return True


def _opinion_patches(backups):
    empty = Sdf.Path.emptyPath
    return tuple(_rename_delta(layer, backup, empty, empty)
                 for layer, backup in backups.items())


def _record_opinions(view, stage, description, sites, patches, refresh):
    if any(record[-2] or record[-1] for record in patches):
        _record(HistoryBridge(stage), view, description,
                partial(_replay_opinions, sites, patches, refresh))


def _refresh_value(path, view):
    node = view.nodes.get(str(path.GetPrimPath()))
    if node is not None and hasattr(node, 'invalidate_value_row'):
        node.invalidate_value_row(path.name)


def _set_value(view, intent, *, warn=True, preview=None):
    stage = view.nodeGraph.getStage()
    if not stage:
        return False, 'no stage'
    path = Sdf.Path(intent.property)
    target = stage.GetEditTarget()
    try:
        site = EditSite.capture(stage, target, (path,))
        if preview and (preview['site'].layer_id != site.layer_id or
                        preview['site'].paths != site.paths or
                        preview['site'].target != site.target):
            return False, 'edit target changed during drag'
        layer = target.GetLayer()
        mode, time = intent.mode, intent.time
        if time is not None and not isinstance(time, Usd.TimeCode):
            time = Usd.TimeCode(time)
        with _edit_transaction(stage, target, view) as journal:
            backup = _journal_layer(journal, layer)
            ok, reason = noodlesValues.write_value(
                stage, stage.GetPrimAtPath(path.GetPrimPath()), path.name,
                intent.components, time, mode=mode, warn=warn)
            if not ok:
                journal.rollback()
                return False, reason
            refresh = partial(_refresh_value, path)
            if preview is not None:
                if not preview:
                    preview.update(site=site, backup=backup)
                preview['patches'] = _opinion_patches({layer: preview['backup']})
            else:
                _record_opinions(view, stage, f'Set {path}', (site,),
                                 _opinion_patches({layer: backup}), refresh)
            journal.defer(partial(refresh, view))
    except (RuntimeError, Tf.ErrorException) as error:
        return False, str(error)
    return True, ''


def _refresh_connections(view, order=()):
    view._invalidateBackReferenceLinkDataCache()
    view.linksChanged = True
    view._rebuildLinks()
    if order:
        positions = {key: index for index, key in enumerate(order)}
        ordered = iter(sorted((link for link in view.links if LinkKey.from_link(link) in positions),
                              key=lambda link: positions[LinkKey.from_link(link)]))
        links = [next(ordered) if LinkKey.from_link(link) in positions else link
                 for link in view.links]
        if any(a is not b for a, b in zip(links, view.links)):
            view.links[:] = links
            _selection.reconcile(view)
            _refresh_visual_state(view, repaint=False)
    view.update()


def _replay_connections(sites, patches, orders, forward, view):
    def refresh(invoking_view):
        _refresh_connections(invoking_view, orders[int(forward)])

    return _replay_opinions(sites, patches, refresh, forward, view)


def _ends_from_link(link):
    key = LinkKey.from_link(link)
    owner = getattr(link, 'propertyOwnerNodeId', '')
    return intents.LinkEndpoints(
        key.source_id, link.sourcePort, key.target_id, link.targetPort,
        getattr(link, 'sourcePropertyName', ''), getattr(link, 'targetPropertyName', ''),
        key.owner_property if owner else '', key.kind == 'relationship')


def _prepare_connection(view, stage, ends, create, *, removal_target=None):
    source = stage.GetPrimAtPath(_nodeIdPrimPath(ends.source_id))
    dest = stage.GetPrimAtPath(_nodeIdPrimPath(ends.target_id))
    if ((create and (not source or not dest)) or
            any(prim and prim.IsInstanceProxy() for prim in (source, dest))):
        raise RuntimeError('Invalid USD connection endpoints')
    target = stage.GetEditTarget()
    source_path = Sdf.Path(_nodeIdPrimPath(ends.source_id))
    dest_path = Sdf.Path(_nodeIdPrimPath(ends.target_id))
    sp, tp = ends.source_port, ends.target_port
    sn, tn = ends.source_property_name, ends.target_property_name
    if not create and ends.owner_property:
        owner = Sdf.Path(ends.owner_property)
        prop = (stage.GetRelationshipAtPath(owner) if ends.relationship else
                stage.GetAttributeAtPath(owner))
        if not prop:
            raise RuntimeError(f'Connection owner not found: {owner}')
        other, name = ((source_path, sn) if owner.GetPrimPath() == dest_path else
                       (dest_path, tn))
        endpoint = other.AppendProperty(name) if name else other
        paths = prop.GetTargets() if ends.relationship else prop.GetConnections()
        changes = ((prop, (endpoint,)),) if endpoint in paths else ()
    else:
        library = view._findLibraryForPrim(dest if dest else source)
        if not library:
            raise RuntimeError('No library handles the connection endpoints')
        if create:
            for prim, name, other, port, prop, is_input in (
                    (source, sn or sp, dest, tp, tn, True),
                    (dest, tn or tp, source, sp, sn, False)):
                rel = prim.GetRelationship(name) if name else None
                if rel:
                    path = library._resolve_endpoint_path(other, port, prop, is_input=is_input)
                    changes = ((rel, (path,)),)
                    break
            else:
                a = source.GetAttribute(sn) if sn else library._resolve_attr(source, sp, False)
                b = dest.GetAttribute(tn) if tn else library._resolve_attr(dest, tp, True)
                if not a or not b:
                    raise RuntimeError('Failed to resolve connection attributes')
                changes = ((b, (a.GetPath(),)),)
        elif source and dest:
            changes = library.prepare_connection_removal(
                source, sp, dest, tp, source_property_name=sn, target_property_name=tn)
        else:
            changes = ()
    if not changes:
        raise RuntimeError('Connection not found in USD')
    writes = []
    for prop, paths in changes:
        relationship = isinstance(prop, Usd.Relationship)
        field = 'targetPaths' if relationship else 'connectionPaths'
        for endpoint in paths:
            if not relationship and not create:
                owner_is_dest = prop.GetPrim() == dest
                owner_name, other_name = (tn, sn) if owner_is_dest else (sn, tn)
                if (owner_name and prop.GetName() != owner_name or
                        other_name and endpoint.name != other_name):
                    continue
            path = prop.GetPath()
            edit_target = removal_target or target
            if not create and removal_target is None:
                for spec in prop.GetPropertyStack():
                    if not spec.HasInfo(field):
                        continue
                    candidate = (target if target.GetLayer() == spec.layer else
                                 stage.GetEditTargetForLocalLayer(spec.layer)
                                 if stage.HasLocalLayer(spec.layer) else None)
                    if candidate is None or candidate.MapToSpecPath(path) != spec.path:
                        continue
                    mapped = candidate.MapToSpecPath(endpoint).StripAllVariantSelections()
                    op = spec.GetInfo(field)
                    if mapped in (*op.explicitItems, *op.addedItems,
                                  *op.prependedItems, *op.appendedItems):
                        edit_target = candidate
                        break
            site = EditSite.capture(stage, edit_target, (path, endpoint))
            mapped_path, mapped_endpoint = (pair[1] for pair in site.paths)
            layer = edit_target.GetLayer()
            factory = (partial(Sdf.CreateRelationshipInLayer, layer, mapped_path,
                               Sdf.VariabilityUniform, prop.IsCustom()) if relationship else
                       partial(Sdf.CreatePrimAttributeInLayer, layer, mapped_path,
                               prop.GetTypeName(), prop.GetVariability(), prop.IsCustom()))
            writes.append((site, mapped_path, mapped_endpoint.StripAllVariantSelections(),
                           field, factory, create))
    if not writes:
        raise RuntimeError('Connection not found at the specified properties')
    return writes


def _write_connection(write):
    site, path, endpoint, field, factory, create = write
    layer = site.target.GetLayer()
    spec = layer.GetObjectAtPath(path) or factory()
    if not spec:
        raise RuntimeError(f'Cannot edit {path}')
    proxy = getattr(spec, _LIST_FIELDS[field])
    if not create:
        proxy.Remove(endpoint)
    elif proxy.isExplicit:
        if endpoint not in proxy.explicitItems:
            proxy.explicitItems = [*proxy.explicitItems, endpoint]
    else:
        if endpoint in proxy.deletedItems:
            proxy.deletedItems = [p for p in proxy.deletedItems if p != endpoint]
        if all(endpoint not in bucket for bucket in
               (proxy.addedItems, proxy.prependedItems, proxy.appendedItems)):
            proxy.prependedItems = [*proxy.prependedItems, endpoint]


def _edit_connections(view, intent, *, description=None):
    stage = view.nodeGraph.getStage()
    if not stage:
        raise RuntimeError('No stage to edit')
    target = stage.GetEditTarget()
    if isinstance(intent, intents.Connect):
        old, new, label = (), intent.endpoints, 'Create Connection'
    elif isinstance(intent, intents.Disconnect):
        old, new, label = (intent.endpoints,), None, intent.description
    else:
        old, new = (intent.old, *getattr(intent, 'additional_old', ())), intent.new
        label = 'Reconnect' if isinstance(intent, intents.Reconnect) else 'Create Connection'
    legacy_symbol('warn_if_non_persistent_edit_target')(stage)
    writes = [write for ends in dict.fromkeys(old) if ends != new
              for write in _prepare_connection(view, stage, ends, False)]
    if new is not None:
        writes.extend(_prepare_connection(view, stage, new, True))
    sites = tuple(write[0] for write in writes)
    orders = [getattr(intent, "prior_order", ()) or tuple(map(LinkKey.from_link, view.links)), ()]

    def refresh(view):
        _refresh_connections(view)
        orders[1] = tuple(map(LinkKey.from_link, view.links))

    with _edit_transaction(stage, target, view, batch=True) as journal:
        backups = {}
        for site in sites:
            layer = site.target.GetLayer()
            if layer not in backups:
                backups[layer] = _journal_layer(journal, layer)
        for write in writes:
            _write_connection(write)
        patches = _opinion_patches(backups)
        if any(record[-2] or record[-1] for record in patches):
            _record(HistoryBridge(stage), view, description or label,
                    partial(_replay_connections, sites, patches, orders))
        journal.defer(partial(refresh, view))
    return True


def _schema_command(view, intent):
    from ..apiSchemas import ALREADY_PRESENT, APPLIED, apply_to_prim, remove_from_prim

    stage = view.nodeGraph.getStage()
    paths = (intent.path,) if isinstance(intent.path, str) else intent.path
    target = stage.GetEditTarget()
    site = EditSite.capture(stage, target, paths)
    applied, skipped, already = [], [], 0
    with _edit_transaction(stage, target, view) as journal:
        layer = target.GetLayer()
        backup = _journal_layer(journal, layer)
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or prim.IsInstanceProxy():
                skipped.append((path, 'prim is not editable'))
                continue
            if intent.action == 'remove':
                api, _, instance = intent.api.partition(':')
                if remove_from_prim(prim, api, instance):
                    applied.append((path, instance))
                continue
            status, instance, reason = apply_to_prim(prim, intent.api)
            if status == APPLIED:
                applied.append((path, instance))
            elif status == ALREADY_PRESENT:
                already += 1
            else:
                skipped.append((prim.GetName(), reason))
        if applied:
            refresh = _reload_schema_graph
            _record_opinions(view, stage, f'{intent.action.title()} {intent.api}', (site,),
                             _opinion_patches({layer: backup}), refresh)
            journal.defer(partial(refresh, view))
    return applied, already, skipped


def _reload_schema_graph(view):
    view._reloadCurrentGraph()


@dataclass(frozen=True)
class ReconnectCapture:
    endpoints: intents.LinkEndpoints
    order: tuple[LinkKey, ...]


def _capture_reconnect(view, link):
    graph = getattr(view, 'nodeGraph', None)
    if graph is not None and isinstance(graph.getStage(), Usd.Stage):
        return ReconnectCapture(_ends_from_link(link), tuple(map(LinkKey.from_link, view.links)))
    return None


def _reconnect_command_args(capture, new, additional_old=()):
    if not isinstance(capture, ReconnectCapture):
        capture = ReconnectCapture(intents.LinkEndpoints(*capture), ())
    return capture.endpoints, new, additional_old, capture.order


def _reconnect_endpoints(view):
    capture = getattr(view, '_reconnectCapture', None)
    if isinstance(capture, ReconnectCapture):
        return capture
    return ReconnectCapture(intents.LinkEndpoints(
        view._reconnectOldSourceNodeId, view._reconnectOldSourcePort,
        view._reconnectOldTargetNodeId, view._reconnectOldTargetPort,
        view._reconnectOldSourcePropertyName, view._reconnectOldTargetPropertyName), ())


def _dispatch_connection(view, intent, reconnect_state=None):
    try:
        result = _view_editor(view).execute(intent)
    except (RuntimeError, Tf.ErrorException) as error:
        Tf.Warn(str(error))
        _refresh_connections(view, getattr(intent, "prior_order", ()))
        result = False
    if reconnect_state is not None and view._reconnectingLink:
        view._clearReconnectState()
    return result
