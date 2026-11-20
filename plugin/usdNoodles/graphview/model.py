#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Graph state, deltas and change subscription: the model owns the graph.

``GraphModelState`` wraps one live ``NodeGraph`` and its ``SpatialIndex``
and is the only writer later layers use: every mutation enters through
``apply(delta)``, which validates references and the base revision before
touching anything, journals the touched entries, refreshes the native
mirrors, the connection cache and the spatial index together at the outer
commit, and publishes exactly one immutable ``ChangeSet`` to ``subscribe``
callbacks. A failed apply rolls the collections back, leaves both
revisions untouched, and publishes nothing; an empty delta is a no-op
with no publication and no revision bump. The state keeps no second
writable nodes/links lookup: it mutates the wrapped collections in place
and the journal exists only for the open transaction.

Revisions: ``revision`` advances on every committed non-empty delta;
``topology_revision`` advances only for membership, replacement, or link
endpoint-identity changes (insert / remove / move / replace, never a
plain update). Applies nest: inside ``transaction()`` (or an outer
``apply``) inner deltas accumulate into the outer commit, which publishes
once. An error anywhere in a transaction aborts the whole transaction.
``Replace`` retires the state -- old subscribers get the final replace
record, then are disconnected -- and the replacement graph gets a fresh
state; a subscription installed on the replaced graph sees the new graph
only. The state is confined to the thread that created it.

Link identity is ``LinkKey``: canonical endpoint ids/pins plus the
property identity (``propertyName``, "" for plain attribute links) and,
for unresolved ends, the explicit dangling id. Later layers import this
definition rather than redefining it.

``MoveNodes`` re-keys simultaneously (namespace-move chains stay correct)
and carries incident link endpoints with the nodes; ``RemoveNode``
cascades to incident links. Both differ from the legacy deferred paths,
which left links stale until the next USD re-derive, only in the
transient between the mutation and that re-derive: the committed state
agrees everywhere (collections, per-node link lists, native adjacency,
spatial index) immediately.

The remainder of this module is the graph/topology and geometry subset of
``GraphView``, moved verbatim as module-level functions whose first
parameter is ``view``. Names the suite patches on the legacy module
resolve at call time through ``legacy_symbol``; everything else is
imported directly.
"""

import threading
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass

from pxr import Gf, Sdf

from UsdNoodles.core import LinkData

from .. import noodlesValues
from .._schema_pin_names import (
    get_display_pin_name_for_property_name,
    resolve_property_name_for_display_pin,
)
from ..linkRenderer import computeLinkCurveBounds
from ..nodeLayout import place_node
from ..pinUtils import (
    get_relationship_link_metadata,
    is_multi_target_relationship_input_pin,
    is_relationship_input_pin,
    is_relationship_pin,
    split_direction_hint,
)
from ..spatial import SpatialIndex
from UsdNoodles.graphview import selection as _selection
from .legacy import legacy_symbol


_MISSING = object()

KIND_ATTRIBUTE = "attribute"
KIND_RELATIONSHIP = "relationship"


class GraphDeltaError(ValueError):
    """A delta was rejected and nothing was mutated.

    Covers unknown op types, base-revision mismatches, dangling
    references (unknown node, occupied target, unmatched link key),
    malformed replace requests, and use of a retired state or an
    aborted transaction.
    """


@dataclass(frozen=True, slots=True)
class LinkKey:
    """Canonical identity of one link, without row/index or address.

    ``kind`` is ``KIND_ATTRIBUTE`` or ``KIND_RELATIONSHIP``;
    ``owner_property`` is the owning ``propertyName`` ("" for plain
    attribute links); ``dangling_id`` is the unresolved end's node id
    for dangling links and "" for complete ones.
    """

    kind: str
    owner_property: str
    source_id: str
    source_pin: str
    target_id: str
    target_pin: str
    dangling_id: str = ""


def link_key_for(link):
    """Derive the canonical :class:`LinkKey` for a link object."""
    if getattr(link, "is_relationship_link", False):
        kind = KIND_RELATIONSHIP
    else:
        kind = KIND_ATTRIBUTE
    dangling_id = ""
    if getattr(link, "isDangling", False):
        if getattr(link, "danglingDirection", "") == "output":
            dangling_id = str(link.targetNodeId)
        else:
            dangling_id = str(link.sourceNodeId)
    return LinkKey(
        kind=kind,
        owner_property=str(getattr(link, "propertyName", "") or ""),
        source_id=str(link.sourceNodeId),
        source_pin=str(link.sourcePort),
        target_id=str(link.targetNodeId),
        target_pin=str(link.targetPort),
        dangling_id=dangling_id,
    )


@dataclass(frozen=True, slots=True)
class InsertNode:
    """Add ``node`` under ``node_id``. Topology."""

    node_id: str
    node: object


@dataclass(frozen=True, slots=True)
class RemoveNode:
    """Drop ``node_id`` and its incident links. Topology."""

    node_id: str


@dataclass(frozen=True, slots=True)
class MoveNodes:
    """Re-key ``pairs`` (``(old_id, new_id)``) simultaneously. Topology.

    Simultaneous, so namespace-move chains (a->b while b->c) and the
    draw order both survive; incident link endpoints move with the
    nodes. A bulk op because sequential single re-keys cannot express
    either property.
    """

    pairs: tuple

    def __post_init__(self):
        object.__setattr__(self, "pairs", tuple(self.pairs))


@dataclass(frozen=True, slots=True)
class InsertLink:
    """Append ``link``; both endpoint nodes must exist. Topology."""

    link: object


@dataclass(frozen=True, slots=True)
class RemoveLink:
    """Drop the first link matching ``link_key``. Topology.

    ``source_property`` / ``target_property`` refine the match when
    several links share the key's endpoints; empty matches anything.
    """

    link_key: LinkKey
    source_property: str = ""
    target_property: str = ""


@dataclass(frozen=True, slots=True)
class UpdateNode:
    """The caller mutated ``node_id`` in place; refresh and report it."""

    node_id: str


@dataclass(frozen=True, slots=True)
class UpdateLink:
    """The caller mutated the matching link in place; refresh it."""

    link_key: LinkKey
    source_property: str = ""
    target_property: str = ""


@dataclass(frozen=True, slots=True)
class Replace:
    """Swap the wrapped graph; retires the state. Topology.

    Must be the only op of its delta and cannot join an open
    transaction. ``spatial_index=None`` keeps the current index, which
    is what a stage reload does (the view owns one index for life).
    """

    node_graph: object
    spatial_index: object = None


_DELTA_OPS = (
    InsertNode,
    RemoveNode,
    MoveNodes,
    InsertLink,
    RemoveLink,
    UpdateNode,
    UpdateLink,
    Replace,
)


@dataclass(frozen=True, slots=True)
class GraphDelta:
    """One ordered batch of ops, optionally guarded by a base revision."""

    ops: tuple = ()
    base_revision: object = None

    def __post_init__(self):
        object.__setattr__(self, "ops", tuple(self.ops))


@dataclass(frozen=True, slots=True)
class ChangeSet:
    """The immutable publication of one outer commit."""

    revision: int
    topology_revision: int
    changed_nodes: tuple = ()
    removed_nodes: tuple = ()
    changed_links: tuple = ()
    removed_links: tuple = ()
    remaps: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "changed_nodes", tuple(self.changed_nodes))
        object.__setattr__(self, "removed_nodes", tuple(self.removed_nodes))
        object.__setattr__(self, "changed_links", tuple(self.changed_links))
        object.__setattr__(self, "removed_links", tuple(self.removed_links))
        object.__setattr__(self, "remaps", tuple(self.remaps))


def _resolves_graph_pair(node_graph, spatial_index):
    """Whether the pair is a real graph worth wrapping in a state.

    The spatial index must be the real native object, not merely
    method-shaped: stub views pair real graphs with mock indexes and
    keep the direct mutation path.
    """
    try:
        nodes = getattr(node_graph, "nodes", None)
        links = getattr(node_graph, "links", None)
        if not isinstance(nodes, dict) or not isinstance(links, list):
            return False
        return isinstance(spatial_index, SpatialIndex)
    except Exception:
        return False


def _node_bounds(node):
    return Gf.Range2d(
        Gf.Vec2d(node.position),
        Gf.Vec2d(node.position + node.size),
    )


def _link_bounds(link):
    return computeLinkCurveBounds(link, margin=20.0)


class GraphModelState:
    """The transaction boundary over one node graph and spatial index.

    Wraps the existing ``NodeGraph`` collections (``nodes`` / ``links``),
    the native adjacency, and the ``SpatialIndex``; every mutation flows
    through :meth:`apply` so the four agree at each outer commit. There
    is no second writable lookup: ops mutate the wrapped collections in
    place and the journal lives only for the open transaction.
    """

    def __init__(self, node_graph, spatial_index):
        if not _resolves_graph_pair(node_graph, spatial_index):
            raise TypeError(
                "GraphModelState needs a graph with dict nodes / list "
                "links and a spatial index; got %r / %r"
                % (type(node_graph).__name__, type(spatial_index).__name__)
            )
        self._graph = node_graph
        self._spatial = spatial_index
        self._owner_thread = threading.get_ident()
        self._revision = 0
        self._topology_revision = 0
        self._subscribers = []
        self._depth = 0
        self._journal = []
        self._aborted = False
        self._acc_changed_nodes = []
        self._acc_removed_nodes = []
        self._acc_changed_links = []
        self._acc_removed_links = []
        self._acc_remaps = []
        self._acc_topology = False
        self._retired = False

    @property
    def graph(self):
        return self._graph

    @property
    def spatial_index(self):
        return self._spatial

    @property
    def revision(self):
        return self._revision

    @property
    def topology_revision(self):
        return self._topology_revision

    @property
    def retired(self):
        return self._retired

    def _check_thread(self):
        if threading.get_ident() != self._owner_thread:
            raise RuntimeError(
                "GraphModelState is confined to its owner thread"
            )

    def subscribe(self, callback):
        """Run ``callback(change_set)`` after each outer commit.

        Returns an idempotent ``unsubscribe`` callable. Delivery order
        is subscription order; every subscriber runs even when an
        earlier one raises, and the first exception then propagates
        (the commit itself is not rolled back).
        """
        self._check_thread()
        if self._retired:
            raise GraphDeltaError("subscribe on a retired GraphModelState")
        if not callable(callback):
            raise TypeError("subscribe needs a callable")
        self._subscribers.append(callback)

        def unsubscribe():
            try:
                self._subscribers.remove(callback)
            except ValueError:
                pass

        return unsubscribe

    @contextmanager
    def transaction(self):
        """Batch applies into one outer commit with one publication.

        Inner ``apply`` calls accumulate; an error anywhere aborts the
        whole transaction (collections roll back, revisions untouched,
        nothing published), even when the body catches the error.
        """
        self._check_thread()
        if self._retired:
            raise GraphDeltaError("transaction on a retired GraphModelState")
        if self._depth == 0:
            self._begin_txn()
        self._depth += 1
        try:
            yield self
        except Exception:
            self._rollback()
            self._reset_txn()
            raise
        if self._aborted:
            self._rollback()
            self._reset_txn()
            return
        self._depth -= 1
        if self._depth == 0:
            self._commit()

    def apply(self, delta):
        """Validate and run ``delta``; the outer commit publishes once.

        Returns the :class:`ChangeSet` at the outer commit, ``None``
        for a nested call (it joins the outer commit), and an empty
        un-published record for an empty delta. Raises
        :class:`GraphDeltaError` (after rolling back) for a bad base
        revision, a dangling reference, or any other rejection.
        """
        self._check_thread()
        if self._retired:
            raise GraphDeltaError("apply on a retired GraphModelState")
        if self._aborted:
            raise GraphDeltaError(
                "the open transaction already failed; it rolls back on exit"
            )
        if not isinstance(delta, GraphDelta):
            raise TypeError(
                "apply needs a GraphDelta, got %r"
                % type(delta).__name__
            )
        if (
            delta.base_revision is not None
            and delta.base_revision != self._revision
        ):
            raise GraphDeltaError(
                "delta base revision %r does not match %r"
                % (delta.base_revision, self._revision)
            )
        for op in delta.ops:
            if not isinstance(op, _DELTA_OPS):
                raise GraphDeltaError(
                    "unknown delta op %r" % type(op).__name__
                )
        replaces = [op for op in delta.ops if isinstance(op, Replace)]
        if replaces:
            if len(delta.ops) != 1:
                raise GraphDeltaError(
                    "Replace must be the only op of its delta"
                )
            if self._depth != 0:
                raise GraphDeltaError(
                    "Replace cannot join an open transaction"
                )
            return self.replace(
                replaces[0].node_graph, replaces[0].spatial_index
            )
        if not delta.ops:
            return ChangeSet(self._revision, self._topology_revision)
        implicit = self._depth == 0
        if implicit:
            self._begin_txn()
        self._depth += 1
        try:
            for op in delta.ops:
                self._apply_op(op)
        except Exception:
            if implicit:
                self._rollback()
                self._reset_txn()
            else:
                self._aborted = True
            raise
        self._depth -= 1
        if implicit:
            return self._commit()
        return None

    def replace(self, node_graph, spatial_index=None):
        """Publish the swap to ``node_graph`` and retire this state.

        Old subscribers receive the one replace record (old ids
        removed, new ids changed), then are disconnected; the new
        graph gets a fresh state with its own subscriptions. Returns
        the replace :class:`ChangeSet`. A replace onto the identical
        pair is a no-op: no publication, no retirement.
        """
        self._check_thread()
        if self._retired:
            raise GraphDeltaError("replace on a retired GraphModelState")
        if self._depth != 0:
            raise GraphDeltaError(
                "replace cannot join an open transaction"
            )
        spatial = self._spatial if spatial_index is None else spatial_index
        if node_graph is self._graph and spatial is self._spatial:
            return ChangeSet(self._revision, self._topology_revision)
        if not _resolves_graph_pair(node_graph, spatial):
            raise GraphDeltaError(
                "the replacement graph does not resolve to wrappable "
                "collections"
            )
        removed_nodes = tuple(self._graph.nodes.keys())
        removed_links = tuple(
            link_key_for(link) for link in self._graph.links
        )
        changed_nodes = tuple(node_graph.nodes.keys())
        changed_links = tuple(
            link_key_for(link) for link in node_graph.links
        )
        self._revision += 1
        self._topology_revision += 1
        changeset = ChangeSet(
            revision=self._revision,
            topology_revision=self._topology_revision,
            changed_nodes=changed_nodes,
            removed_nodes=removed_nodes,
            changed_links=changed_links,
            removed_links=removed_links,
        )
        snapshot = list(self._subscribers)
        self._retire()
        self._publish_to(changeset, snapshot)
        return changeset

    def _retire(self):
        self._subscribers = []
        self._retired = True

    def _publish_to(self, changeset, subscribers):
        failure = None
        for callback in subscribers:
            try:
                callback(changeset)
            except Exception as error:
                if failure is None:
                    failure = error
        if failure is not None:
            raise failure

    def _begin_txn(self):
        self._journal = []
        self._aborted = False
        self._acc_changed_nodes = []
        self._acc_removed_nodes = []
        self._acc_changed_links = []
        self._acc_removed_links = []
        self._acc_remaps = []
        self._acc_topology = False

    def _reset_txn(self):
        self._depth = 0
        self._begin_txn()

    def _commit(self):
        if (
            not self._journal
            and not self._acc_changed_nodes
            and not self._acc_removed_nodes
            and not self._acc_changed_links
            and not self._acc_removed_links
            and not self._acc_remaps
        ):
            self._reset_txn()
            return ChangeSet(self._revision, self._topology_revision)
        try:
            self._refresh_derived()
        except Exception:
            self._rollback()
            self._reset_txn()
            raise
        self._revision += 1
        if self._acc_topology:
            self._topology_revision += 1
        changeset = ChangeSet(
            revision=self._revision,
            topology_revision=self._topology_revision,
            changed_nodes=tuple(self._acc_changed_nodes),
            removed_nodes=tuple(self._acc_removed_nodes),
            changed_links=tuple(self._acc_changed_links),
            removed_links=tuple(self._acc_removed_links),
            remaps=tuple(self._acc_remaps),
        )
        subscribers = list(self._subscribers)
        self._reset_txn()
        self._publish_to(changeset, subscribers)
        return changeset

    def _refresh_derived(self):
        graph = self._graph
        spatial = self._spatial
        spatial.clear()
        for node_id, node in graph.nodes.items():
            spatial.insertNode(node_id, _node_bounds(node))
        for index, link in enumerate(graph.links):
            spatial.insertLink(index, _link_bounds(link))
        sync_nodes = getattr(graph, "syncNodesFromModels", None)
        if callable(sync_nodes):
            sync_nodes(graph.nodes)
        sync_links = getattr(graph, "syncLinksFromModels", None)
        if callable(sync_links):
            sync_links(graph.links)
        build_cache = getattr(graph, "buildConnectionCache", None)
        if callable(build_cache):
            build_cache()

    def _apply_op(self, op):
        if isinstance(op, InsertNode):
            self._op_insert_node(op)
        elif isinstance(op, RemoveNode):
            self._op_remove_node(op)
        elif isinstance(op, MoveNodes):
            self._op_move_nodes(op)
        elif isinstance(op, InsertLink):
            self._op_insert_link(op)
        elif isinstance(op, RemoveLink):
            self._op_remove_link(op)
        elif isinstance(op, UpdateNode):
            self._op_update_node(op)
        elif isinstance(op, UpdateLink):
            self._op_update_link(op)
        else:
            raise GraphDeltaError(
                "unknown delta op %r" % type(op).__name__
            )

    def _op_insert_node(self, op):
        nodes = self._graph.nodes
        if op.node_id in nodes:
            raise GraphDeltaError(
                "insert: node %r is already in the graph" % op.node_id
            )
        if op.node is None:
            raise GraphDeltaError("insert: node is None")
        nodes[op.node_id] = op.node
        self._journal.append(("remove_node", op.node_id))
        self._acc_changed_nodes.append(op.node_id)
        self._acc_topology = True

    def _op_remove_node(self, op):
        nodes = self._graph.nodes
        links = self._graph.links
        node_id = op.node_id
        if node_id not in nodes:
            raise GraphDeltaError(
                "remove: unknown node %r" % node_id
            )
        order = list(nodes.keys()).index(node_id)
        node = nodes.pop(node_id)
        incident = [
            (index, link)
            for index, link in enumerate(links)
            if link.sourceNodeId == node_id
            or link.targetNodeId == node_id
        ]
        detached = []
        for index, link in reversed(incident):
            links.pop(index)
            for end_id in (link.sourceNodeId, link.targetNodeId):
                endpoint = nodes.get(end_id)
                if endpoint is None:
                    continue
                for side in ("inputLinks", "outputLinks"):
                    owned = getattr(endpoint, side, None)
                    if owned is None:
                        continue
                    for owned_index, owned_link in enumerate(owned):
                        if owned_link is link:
                            owned.pop(owned_index)
                            detached.append(
                                (end_id, side, owned_index, link)
                            )
                            break
        self._journal.append(
            ("restore_node", node_id, order, node, incident, detached)
        )
        self._acc_removed_nodes.append(node_id)
        self._acc_removed_links.extend(
            link_key_for(link) for _, link in incident
        )
        self._acc_topology = True

    def _op_move_nodes(self, op):
        pairs = tuple(op.pairs)
        if not pairs:
            raise GraphDeltaError("move: no pairs")
        nodes = self._graph.nodes
        olds = [old for old, _ in pairs]
        news = [new for _, new in pairs]
        for old, new in pairs:
            if old not in nodes:
                raise GraphDeltaError("move: unknown node %r" % old)
            if old == new:
                raise GraphDeltaError("move: identity pair %r" % old)
        if len(set(news)) != len(news):
            raise GraphDeltaError("move: duplicate target id")
        clobbered = sorted(set(news) - set(olds) & set(nodes))
        if clobbered:
            raise GraphDeltaError(
                "move: target already in use: %s" % ", ".join(clobbered)
            )
        forward = dict(pairs)
        get_stage = getattr(self._graph, "getStage", None)
        stage = get_stage() if callable(get_stage) else None
        node_olds = []
        for old, new in pairs:
            node = nodes[old]
            prim_backed = "#" not in new
            if prim_backed:
                node_olds.append(
                    (
                        node,
                        old,
                        True,
                        node._primPath,
                        node._prim,
                        node.name,
                    )
                )
                node.id = new
                node._primPath = Sdf.Path(new)
                if stage is not None:
                    node._prim = stage.GetPrimAtPath(new)
                node.name = Sdf.Path(new).name
            else:
                node_olds.append((node, old, False, None, None, None))
                node.id = new
        link_olds = []
        for link in self._graph.links:
            old_src = link.sourceNodeId
            old_tgt = link.targetNodeId
            new_src = forward.get(old_src, old_src)
            new_tgt = forward.get(old_tgt, old_tgt)
            if new_src == old_src and new_tgt == old_tgt:
                continue
            old_data_src = getattr(link, "data_sourceNodeId", _MISSING)
            old_data_tgt = getattr(link, "data_targetNodeId", _MISSING)
            old_owner = getattr(link, "propertyOwnerNodeId", _MISSING)
            link_olds.append(
                (link, old_src, old_tgt, old_data_src, old_data_tgt,
                 old_owner)
            )
            if (
                old_data_src is not _MISSING
                and old_data_src == old_src
                and new_src != old_src
            ):
                link.data_sourceNodeId = new_src
            if (
                old_data_tgt is not _MISSING
                and old_data_tgt == old_tgt
                and new_tgt != old_tgt
            ):
                link.data_targetNodeId = new_tgt
            if old_owner is not _MISSING and old_owner in forward:
                link.propertyOwnerNodeId = forward[old_owner]
            link.sourceNodeId = new_src
            link.targetNodeId = new_tgt
        old_dict = nodes
        self._graph.nodes = {
            forward.get(node_id, node_id): node
            for node_id, node in nodes.items()
        }
        self._journal.append(
            ("unmove_nodes", pairs, node_olds, link_olds, old_dict)
        )
        self._acc_remaps.extend(pairs)
        self._acc_changed_nodes.extend(news)
        self._acc_topology = True

    def _op_insert_link(self, op):
        link = op.link
        if link is None:
            raise GraphDeltaError("insert_link: link is None")
        for attr in (
            "sourceNodeId",
            "sourcePort",
            "targetNodeId",
            "targetPort",
        ):
            if not hasattr(link, attr):
                raise GraphDeltaError(
                    "insert_link: link lacks %r" % attr
                )
        nodes = self._graph.nodes
        if link.sourceNodeId not in nodes:
            raise GraphDeltaError(
                "insert_link: unknown source %r" % link.sourceNodeId
            )
        if link.targetNodeId not in nodes:
            raise GraphDeltaError(
                "insert_link: unknown target %r" % link.targetNodeId
            )
        self._graph.links.append(link)
        nodes[link.sourceNodeId].outputLinks.append(link)
        nodes[link.targetNodeId].inputLinks.append(link)
        self._journal.append(
            (
                "remove_inserted_link",
                link,
                link.sourceNodeId,
                link.targetNodeId,
            )
        )
        self._acc_changed_links.append(link_key_for(link))
        self._acc_topology = True

    def _find_link(self, key, source_property, target_property):
        for index, link in enumerate(self._graph.links):
            if link_key_for(link) != key:
                continue
            if source_property and str(
                getattr(link, "sourcePropertyName", "") or ""
            ) != source_property:
                continue
            if target_property and str(
                getattr(link, "targetPropertyName", "") or ""
            ) != target_property:
                continue
            return index, link
        return None, None

    def _op_remove_link(self, op):
        index, link = self._find_link(
            op.link_key, op.source_property, op.target_property
        )
        if link is None:
            raise GraphDeltaError(
                "remove_link: no link matches %r" % (op.link_key,)
            )
        nodes = self._graph.nodes
        self._graph.links.pop(index)
        detached = []
        for end_id, side in (
            (link.sourceNodeId, "outputLinks"),
            (link.targetNodeId, "inputLinks"),
        ):
            endpoint = nodes.get(end_id)
            if endpoint is None:
                continue
            owned = getattr(endpoint, side, None)
            if owned is None:
                continue
            for owned_index, owned_link in enumerate(owned):
                if owned_link is link:
                    owned.pop(owned_index)
                    detached.append((end_id, side, owned_index, link))
                    break
        self._journal.append(("restore_link", index, link, detached))
        self._acc_removed_links.append(op.link_key)
        self._acc_topology = True

    def _op_update_node(self, op):
        if op.node_id not in self._graph.nodes:
            raise GraphDeltaError(
                "update: unknown node %r" % op.node_id
            )
        self._acc_changed_nodes.append(op.node_id)

    def _op_update_link(self, op):
        _, link = self._find_link(
            op.link_key, op.source_property, op.target_property
        )
        if link is None:
            raise GraphDeltaError(
                "update_link: no link matches %r" % (op.link_key,)
            )
        self._acc_changed_links.append(op.link_key)

    def _rollback(self):
        for entry in reversed(self._journal):
            tag = entry[0]
            if tag == "remove_node":
                self._graph.nodes.pop(entry[1], None)
            elif tag == "restore_node":
                self._rollback_restore_node(entry)
            elif tag == "unmove_nodes":
                self._rollback_unmove_nodes(entry)
            elif tag == "remove_inserted_link":
                self._rollback_remove_inserted_link(entry)
            elif tag == "restore_link":
                self._rollback_restore_link(entry)

    def _rollback_restore_node(self, entry):
        _, node_id, order, node, incident, detached = entry
        nodes = self._graph.nodes
        items = list(nodes.items())
        items.insert(min(order, len(items)), (node_id, node))
        nodes.clear()
        nodes.update(items)
        links = self._graph.links
        for index, link in incident:
            links.insert(min(index, len(links)), link)
        for end_id, side, owned_index, link in reversed(detached):
            owned = getattr(nodes[end_id], side)
            owned.insert(min(owned_index, len(owned)), link)

    def _rollback_unmove_nodes(self, entry):
        _, _, node_olds, link_olds, old_dict = entry
        self._graph.nodes = old_dict
        for node, old_id, prim_backed, old_path, old_prim, old_name in (
            node_olds
        ):
            node.id = old_id
            if prim_backed:
                node._primPath = old_path
                node._prim = old_prim
                node.name = old_name
        for record in link_olds:
            (
                link,
                old_src,
                old_tgt,
                old_data_src,
                old_data_tgt,
                old_owner,
            ) = record
            link.sourceNodeId = old_src
            link.targetNodeId = old_tgt
            if old_data_src is not _MISSING:
                link.data_sourceNodeId = old_data_src
            if old_data_tgt is not _MISSING:
                link.data_targetNodeId = old_data_tgt
            if old_owner is not _MISSING:
                link.propertyOwnerNodeId = old_owner

    def _rollback_remove_inserted_link(self, entry):
        _, link, src_id, tgt_id = entry
        links = self._graph.links
        for index, candidate in enumerate(links):
            if candidate is link:
                links.pop(index)
                break
        nodes = self._graph.nodes
        for end_id, side in (
            (src_id, "outputLinks"),
            (tgt_id, "inputLinks"),
        ):
            owned = getattr(nodes[end_id], side)
            for index, candidate in enumerate(owned):
                if candidate is link:
                    owned.pop(index)
                    break

    def _rollback_restore_link(self, entry):
        _, index, link, detached = entry
        links = self._graph.links
        links.insert(min(index, len(links)), link)
        nodes = self._graph.nodes
        for end_id, side, owned_index, owned_link in reversed(detached):
            owned = getattr(nodes[end_id], side)
            owned.insert(min(owned_index, len(owned)), owned_link)


def state_for_view(view):
    """The view's live ``GraphModelState``, attached lazily.

    Returns the attached state when it still wraps the view's current
    graph pair, retires a superseded one and installs a fresh state
    otherwise, and returns ``None`` for stub views whose graph does
    not resolve to wrappable collections (those keep the direct
    mutation path). Attaching never fails the caller.
    """
    try:
        graph = view.nodeGraph
        spatial = view._spatialIndex
    except Exception:
        return None
    if not _resolves_graph_pair(graph, spatial):
        return None
    try:
        attached = getattr(view, "_graphModelState", None)
    except Exception:
        attached = None
    if isinstance(attached, GraphModelState) and not attached.retired:
        if attached.graph is graph and attached.spatial_index is spatial:
            return attached
        attached._retire()
    state = GraphModelState(graph, spatial)
    try:
        view._graphModelState = state
    except Exception:
        pass
    return state


def attached_state(view):
    """The live state attached to ``view``, or ``None`` (never creates)."""
    try:
        attached = getattr(view, "_graphModelState", None)
        graph = view.nodeGraph
    except Exception:
        return None
    if (
        isinstance(attached, GraphModelState)
        and not attached.retired
        and attached.graph is graph
    ):
        return attached
    return None


def transaction_for(view):
    """A coalescing transaction for ``view``, or a null context.

    The stage reconciliation pass runs inside this so a synchronous
    USD notice joins the initiating transaction and publishes once;
    stub views without a state get a no-op context instead.
    """
    state = state_for_view(view)
    if state is None:
        return nullcontext()
    return state.transaction()


def note_graph_replaced(view, old_state, old_graph):
    """Publish the reload replace and install the new graph's state.

    ``old_state`` / ``old_graph`` are the pre-load pair (see
    :func:`attached_state`): when the load swapped the graph, the old
    state publishes its final replace record and retires -- the old
    subscriptions are disconnected before the fresh state is
    installed. Returns the view's current state, if it resolves.
    """
    try:
        graph = view.nodeGraph
        spatial = view._spatialIndex
    except Exception:
        return None
    if not _resolves_graph_pair(graph, spatial):
        if old_state is not None and not old_state.retired:
            old_state._retire()
        return None
    if (
        old_state is not None
        and not old_state.retired
        and old_graph is not None
        and graph is not old_graph
    ):
        old_state.replace(graph, spatial)
    return state_for_view(view)


def construct_node_for_insert(view, prim, stage):
    """Build (but do not insert) the node adoption would add for ``prim``.

    The construct half of :func:`add_new_node_fast` -- factory,
    sizing, z-order -- without the insert, so the stage-sync adopt
    path can validate and insert through :meth:`GraphModelState.apply`
    instead of mutating the collections directly. Returns ``None``
    when no library handles the prim type.
    """
    node = view._nodeFactory.create_node_from_prim(prim, stage)
    if not node:
        return None
    if view.fontAtlas:
        view.nodeGraph._calculateNodeSize(
            node, view.textRenderer.calculateTextWidth, view.fontAtlas
        )
    view._nextZOrder += 1
    node.zOrder = view._nextZOrder
    return node


# ---------------------------------------------------------------------------
# The graph/topology subset of GraphView, moved verbatim: module-level
# functions whose first parameter is the duck-typed view. Names the suite
# patches on the legacy module resolve through legacy_symbol at call time.
# ---------------------------------------------------------------------------


def add_back_reference_dangling_links(view):
    """Add dangling links for connections authored on prims NOT in the graph
    that reference prims IN the graph.

    USD attribute connections are usually authored on the input side
    (``downstream.inputs:x.connect = </Upstream.outputs:result>``). When the
    downstream prim is in the graph but the upstream prim is not, the input
    side is handled by ``_processNodeLinks`` via the downstream node's
    cached link list. The reverse — upstream in graph, downstream absent —
    is NOT visible from the upstream prim's own attributes, because the
    connection is authored on the downstream prim's input.

    This scan walks the stage once and caches the per-prim USD link data
    keyed by prim path. The cache is invalidated by
    ``_invalidateBackReferenceLinkDataCache`` when stage composition
    changes (see ``_handleUsdChanges`` and ``_onStageReplaced``); between
    invalidations the cached data is reused, so subsequent
    ``_rebuildLinks`` calls do not re-traverse the stage. The graph-side
    ``self.nodes`` membership filter is applied at use time, not cache
    time, because graph membership changes (add/remove node from the
    editor) do not affect what USD authored.
    """
    nodeGraph = getattr(view, "nodeGraph", None)
    stage = nodeGraph.getStage() if nodeGraph is not None else None
    if not stage:
        return

    # Build / reuse the per-prim USD link cache.
    cache = getattr(view, "_backReferenceLinkDataCache", None)
    if cache is None:
        cache = {}
        for prim in stage.Traverse():
            try:
                link_dicts = legacy_symbol("collect_links_for_prim")(prim)
            except (RuntimeError, TypeError, AttributeError):
                continue
            if not link_dicts:
                continue
            cache[str(prim.GetPath())] = link_dicts
        view._backReferenceLinkDataCache = cache

    # Build a key set of links already added so we don't duplicate the
    # forward direction the per-node pass already produced.
    existing_keys = set()
    for existing in view.links:
        existing_keys.add(
            (
                existing.sourceNodeId,
                existing.sourcePort,
                existing.targetNodeId,
                existing.targetPort,
                str(getattr(existing, "sourcePropertyName", "")),
                str(getattr(existing, "targetPropertyName", "")),
            )
        )

    for prim_path_str, link_dicts in cache.items():
        if prim_path_str in view.nodes:
            # Forward direction already handled by _processNodeLinks.
            continue

        for link_data in link_dicts:
            src_id = str(link_data.get("sourceNodeId", ""))
            tgt_id = str(link_data.get("targetNodeId", ""))

            present_is_source = src_id in view.nodes
            present_is_target = tgt_id in view.nodes
            if not present_is_source and not present_is_target:
                continue

            key = (
                src_id,
                str(link_data.get("sourcePinName", "")),
                tgt_id,
                str(link_data.get("targetPinName", "")),
                str(link_data.get("sourcePropertyName", "")),
                str(link_data.get("targetPropertyName", "")),
            )
            if key in existing_keys:
                continue
            existing_keys.add(key)

            link = view._makeLinkFromUsdData(link_data)

            if present_is_source:
                present_node = view.nodes[src_id]
                view._setupDanglingLinkEndpoint(
                    link, present_node, link.sourcePort, isOutput=True
                )
                link.data_sourceNodeId = src_id
                link.data_targetNodeId = tgt_id
                link.is_input_link = False
            else:
                present_node = view.nodes[tgt_id]
                view._setupDanglingLinkEndpoint(
                    link, present_node, link.targetPort, isOutput=False
                )
                link.data_sourceNodeId = src_id
                link.data_targetNodeId = tgt_id
                link.is_input_link = True

            view.links.append(link)


def rebuild_links(view):
    """Rebuild all links by reading USD connections directly.

    This method reads connection data directly from USD prims,
    ensuring links always reflect the current USD state. This is
    important for responding to external USD edits.
    """
    view.links.clear()




    for node in view.nodes.values():
        # Virtual nodes (Blueprint I/O) have no USD prim; keep their cached
        # inputLinks/outputLinks as-is.
        if "#" in node.id:
            continue
        # For USD-backed nodes, read connections fresh from the prim
        # (clears the cached links and rebuilds from USD).
        if node._prim and node._prim.IsValid():
            view._populateNodeLinksFromPrim(node, node._prim)





    view._surfaceConnectionEndpointPins()


    for node in view.nodes.values():
        view._processNodeLinks(node)

    # Scan the stage for back-references: connections authored on absent
    # prims that target present prims. Without this, an authored connection
    # like B.inputs:x.connect = A.outputs:result is invisible from A when B
    # is not in the graph, leaving A's output pin appearing disconnected.
    view._addBackReferenceDanglingLinks()

    # Deduplicate links (bare/dual-pin attributes can be discovered
    # from both endpoints, producing duplicate link entries).
    seen_links = set()
    deduped = []
    for link in view.links:
        key = (
            link.sourceNodeId,
            link.sourcePort,
            link.targetNodeId,
            link.targetPort,
            str(getattr(link, "sourcePropertyName", "")),
            str(getattr(link, "targetPropertyName", "")),
        )
        if key not in seen_links:
            seen_links.add(key)
            deduped.append(link)
    if len(deduped) != len(view.links):
        view.links.clear()
        view.links.extend(deduped)

    _selection.reconcile(view)

    # Sync the freshly-rebuilt links into the C++ View Model BEFORE rebuilding
    # the connection cache, so the cache (isPortConnected / folded-header /
    # synthetic-relationship endpoints, consumed by the port-highlight pass)
    # reflects the current links rather than the previous frame's snapshot.
    # paintLinks re-syncs the same list later for per-frame hover flags; this
    # closes the update-path gap on the link-change frame itself.
    view.nodeGraph.syncLinksFromModels(view.links)
    # The snapshot was just refreshed from the rebuilt links; clear the
    # incremental sync flag and force hover to re-derive against the new link
    # indices on the next paint.
    view._linksNeedSync = False
    view._lastSyncedLinkHover = -1
    # Connections were rebuilt, so per-port connected/disconnected colors may
    # have changed; force the port-highlight geometry to rebuild next paint.
    view._portHighlightsDirty = True
    view._rebuildConnectionCache()

    # Rebuild spatial index with updated nodes and links
    with legacy_symbol("_ProfileSection")(view._profiler, "Rebuild Spatial Index"):
        view._spatialIndex.clear()
        for nodeId, node in view.nodes.items():
            bounds = Gf.Range2d(
                Gf.Vec2d(node.position),
                Gf.Vec2d(node.position + node.size),
            )
            view._spatialIndex.insertNode(nodeId, bounds)
        for linkIndex, link in enumerate(view.links):
            bounds = view._computeLinkBounds(link)
            view._spatialIndex.insertLink(linkIndex, bounds)

    view.linksChanged = False


def process_node_links(view, node):
    """Process a node's inputLinks and outputLinks into renderable links.

    Args:
        node: NodeModel whose links should be processed
    """
    # Process input links (connections on input attributes)
    # For input links: sourceNodeId = connected node (provides output),
    #                   targetNodeId = this node (receives input)
    for link in node.inputLinks:
        if link.sourceNodeId not in view.nodes:
            # Source node is not in the graph - create dangling link.
            # Applies to both attribute and relationship links: the port
            # stays filled and double-click adds the missing prim back.
            view._setupDanglingLinkEndpoint(
                link, node, link.targetPort, isOutput=False
            )
            link.data_sourceNodeId = (
                link.sourceNodeId
            )  # The missing node would be source
            link.data_targetNodeId = node.id  # This node receives data
            link.is_input_link = True
            view.links.append(link)
            continue

        connectedNode = view.nodes[link.sourceNodeId]

        # Setup link endpoints using renderer port positions
        # startNode = connectedNode (output side), endNode = this node (input side)
        view._setupLinkEndpoints(
            link, connectedNode, node, link.sourcePort, link.targetPort
        )

        # Track actual data flow direction:
        # For inputLinks: data flows FROM connectedNode TO current node
        link.data_sourceNodeId = (
            connectedNode.id
        )  # Node providing data (has output)
        link.data_targetNodeId = node.id  # Node receiving data (has input)

        link.is_input_link = True
        link.isDangling = False

        view.links.append(link)

    # Process output links (connections on output attributes)
    for link in node.outputLinks:
        if link.targetNodeId not in view.nodes:
            # Target node is not in the graph - create dangling link.
            # Applies to both attribute and relationship links: the port
            # stays filled and double-click adds the missing prim back.
            view._setupDanglingLinkEndpoint(
                link, node, link.sourcePort, isOutput=True
            )
            link.data_sourceNodeId = node.id  # This node provides data
            link.data_targetNodeId = (
                link.targetNodeId
            )  # The missing node would receive
            link.is_input_link = False
            view.links.append(link)
            continue

        connectedNode = view.nodes[link.targetNodeId]

        # Setup link endpoints using renderer port positions
        view._setupLinkEndpoints(
            link, node, connectedNode, link.sourcePort, link.targetPort
        )

        # Track actual data flow direction:
        # For outputLinks: data flows FROM current node TO connectedNode
        link.data_sourceNodeId = node.id  # Node providing data (has output)
        link.data_targetNodeId = connectedNode.id  # Node receiving data (has input)

        link.is_input_link = False
        link.isDangling = False

        view.links.append(link)


def add_node_from_dangling_link(view, link):
    """Add the missing node referenced by a dangling link.

    Args:
        link: The dangling LinkData whose missing node should be added

    Returns:
        True if the node was successfully added, False otherwise
    """
    if not view._usdviewApi:
        view._showPopupMessage("Cannot add node: no USD stage")
        return False

    stage = view._usdviewApi.stage
    if not stage:
        view._showPopupMessage("Cannot add node: no USD stage")
        return False

    # Determine which node is missing
    missingNodeId = None
    if link.isDangling:
        # Check which end of the link is missing
        if link.data_sourceNodeId and link.data_sourceNodeId not in view.nodes:
            missingNodeId = link.data_sourceNodeId
        elif link.data_targetNodeId and link.data_targetNodeId not in view.nodes:
            missingNodeId = link.data_targetNodeId

    if not missingNodeId:
        view._showPopupMessage("No missing node to add")
        return False

    # Get the prim from USD
    prim = stage.GetPrimAtPath(missingNodeId)
    if not prim or not prim.IsValid():
        view._showPopupMessage(f"Prim not found: {missingNodeId}")
        return False

    # Use NodeFactory to create the node
    node = view._nodeFactory.create_node_from_prim(prim, stage)
    if not node:
        view._showPopupMessage(
            f"Cannot create node for prim type: {prim.GetTypeName()}"
        )
        return False

    view._populateNodeLinksFromPrim(node, prim)

    # Calculate node size
    if view.fontAtlas:
        view.nodeGraph._calculateNodeSize(
            node, view.textRenderer.calculateTextWidth, view.fontAtlas
        )

    # Register first, then place by connection exactly like the add-node
    # path: the expanded node lands one column beside the node it wires
    # to (inputs left, dependents right), staggered clear of siblings,
    # so double-clicking a relationship/property reads like the L
    # layout instead of dropping the node at the viewport center. The
    # placement doubles as the position setup the renderer needs to
    # compute pin positions before _rebuildLinks runs on the next paint.
    view.nodes[node.id] = node
    view._nextZOrder += 1
    node.zOrder = view._nextZOrder
    view._placeAddedNodes([node], fallback=view._viewportAnchor())

    # Enable prim tree syncing since we now have nodes with valid USD paths
    view.nodeGraph.syncSelectionToPrimTree = True

    view.linksChanged = True
    view.textChanged = True
    view._clearRenderCache()
    view.update()

    view._showPopupMessage(f"Added: {prim.GetName()}")
    return True


def add_link_to_graph(
    view,
    outputNodeId,
    outputPort,
    inputNodeId,
    inputPort,
    outputNode,
    inputNode,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    """Add a new link to the local graph state (no USD mutation)."""
    link = LinkData()
    link.sourceNodeId = outputNodeId
    link.sourcePort = outputPort
    link.targetNodeId = inputNodeId
    link.targetPort = inputPort
    link.sourcePropertyName = sourcePropertyName
    link.targetPropertyName = targetPropertyName
    relationship_metadata = get_relationship_link_metadata(
        outputNodeId,
        outputPort,
        inputNodeId,
        inputPort,
    )
    link.propertyOwnerNodeId = str(
        relationship_metadata.get("propertyOwnerNodeId", "")
    )
    link.propertyName = str(relationship_metadata.get("propertyName", ""))
    link.is_relationship_link = bool(
        relationship_metadata.get("is_relationship_link", False)
    )

    # USD fallback: the name-based metadata only knows about the
    # hardcoded sources/affects pins.  An unknown-name relationship like
    # material:binding would otherwise drop in as a regular attribute
    # link, so the persistent prim-target arrowhead at the top of the
    # target prim would not draw until the next USD-driven rebuild.
    if not link.is_relationship_link:
        view._classifyRelationshipLinkFromUsd(
            link,
            outputNode,
            inputNode,
            outputPort,
            inputPort,
            sourcePropertyName,
            targetPropertyName,
        )

    view._setupLinkEndpoints(link, outputNode, inputNode, outputPort, inputPort)

    link.data_sourceNodeId = outputNodeId
    link.data_targetNodeId = inputNodeId
    link.is_input_link = False
    link.isDangling = False

    view.links.append(link)

    linkIndex = len(view.links) - 1
    view._spatialIndex.insertLink(linkIndex, view._computeLinkBounds(link))

    view._rebuildConnectionCache()

    outputNode.outputLinks.append(link)
    inputNode.inputLinks.append(link)



    view._linksNeedSync = True
    view._portHighlightsDirty = True


def remove_link_from_graph(
    view,
    outputNodeId,
    outputPort,
    inputNodeId,
    inputPort,
    *,
    sourcePropertyName="",
    targetPropertyName="",
):
    """Remove a link from the local graph state (no USD mutation)."""
    linkRecord = view._makeSerializedLinkRecord(
        outputNodeId,
        outputPort,
        inputNodeId,
        inputPort,
        sourcePropertyName=sourcePropertyName,
        targetPropertyName=targetPropertyName,
    )
    linkToRemove = None
    for link in view.links:
        if view._linkMatchesRecord(link, linkRecord):
            linkToRemove = link
            break

    if not linkToRemove:
        return

    linkIndex = view.links.index(linkToRemove)

    outputNode = view.nodes.get(outputNodeId)
    inputNode = view.nodes.get(inputNodeId)
    if outputNode and linkToRemove in outputNode.outputLinks:
        outputNode.outputLinks.remove(linkToRemove)
    if inputNode and linkToRemove in inputNode.inputLinks:
        inputNode.inputLinks.remove(linkToRemove)

    view._spatialIndex.removeLink(linkIndex)
    view.links.remove(linkToRemove)

    # Re-index remaining links in spatial index (indices shifted)
    for i in range(linkIndex, len(view.links)):
        view._spatialIndex.removeLink(i + 1)
        view._spatialIndex.insertLink(i, view._computeLinkBounds(view.links[i]))

    view._rebuildConnectionCache()
    # A link was removed in place (no full rebuild); re-mirror to C++ and
    # rebuild port highlights (the removed connection changes port colors).
    view._linksNeedSync = True
    view._portHighlightsDirty = True
    view.update()


def surface_connection_endpoint_pins(view):
    """Ensure every connection endpoint renders as a pin/row.

    A connection can reference a property that is not an authored or
    schema-declared attribute on its prim -- e.g. a typed prim whose schema
    is not registered, so its outputs exist only as the *targets* of other
    prims' connections. Such endpoints have no pin row, so the noodle
    attaches to the bare node edge with no pin. This surfaces every
    non-relationship endpoint as a dual pin row on its node, uniformly,
    regardless of schema availability. Relationship endpoints are left to
    the synthetic-port path. Idempotent: endpoints that already have a
    visible port are skipped.
    """
    endpoint_inputs: dict[str, list[str]] = {}
    endpoint_outputs: dict[str, list[str]] = {}

    def _note(mapping, node_id, pin_name):
        if not node_id or not pin_name or node_id not in view.nodes:
            return
        mapping.setdefault(node_id, []).append(pin_name)

    for node in view.nodes.values():
        for link in list(node.inputLinks) + list(node.outputLinks):
            if getattr(link, "is_relationship_link", False):
                continue
            _note(endpoint_outputs, link.sourceNodeId, link.sourcePort)
            _note(endpoint_inputs, link.targetNodeId, link.targetPort)

    for node_id in set(endpoint_inputs) | set(endpoint_outputs):
        node = view.nodes.get(node_id)
        if node is None or not hasattr(node, "add_connection_endpoint_pins"):
            continue
        added = node.add_connection_endpoint_pins(
            endpoint_inputs.get(node_id, ()),
            endpoint_outputs.get(node_id, ()),
        )
        # Added rows change the node's row count, so recompute its size for
        # correct hit-testing and spatial bounds.
        if added and view.fontAtlas:
            view.nodeGraph._calculateNodeSize(
                node,
                view.textRenderer.calculateTextWidth,
                view.fontAtlas,
            )


def add_new_node_fast(view, prim, stage):
    """Add a newly created node to the graph with minimal overhead.

    This is optimized for newly created nodes that have NO connections yet.
    It skips the full link rebuild and only does the minimal work needed.

    Args:
        prim: The USD prim for the new node
        stage: The USD stage

    Returns:
        str: The node ID if successful, None otherwise
    """
    primPath = prim.GetPath()
    primPathStr = str(primPath)

    # Skip if already in graph
    if primPathStr in view.nodes:
        return None

    # Use NodeFactory to create the node
    node = view._nodeFactory.create_node_from_prim(prim, stage)
    if not node:
        return None

    # New nodes have no connections, so inputLinks and outputLinks are empty
    # No need to process USD attributes for connections

    # Calculate node size
    if view.fontAtlas:
        view.nodeGraph._calculateNodeSize(
            node, view.textRenderer.calculateTextWidth, view.fontAtlas
        )

    # Add to nodes dictionary
    view.nodes[node.id] = node
    view._nextZOrder += 1
    node.zOrder = view._nextZOrder

    # Index the new node for hit-testing (no links / highlights yet).
    view._updateNodeSpatialBounds({node.id})

    return node.id


def classify_relationship_link_from_usd(
    link,
    outputNode,
    inputNode,
    outputPort,
    inputPort,
    sourcePropertyName,
    targetPropertyName,
):
    """Mark ``link`` as a relationship link when the USD prim says so.

    Mutates ``link.is_relationship_link``, ``link.propertyOwnerNodeId``
    and ``link.propertyName`` in place when the source-side port name or
    target-side port name resolves to an authored USD relationship on the
    corresponding prim.  Returns silently if nothing matches.
    """
    for prim_node, property_candidates, owner_id in (
        (
            outputNode,
            (sourcePropertyName, outputPort),
            getattr(outputNode, "id", "") if outputNode is not None else "",
        ),
        (
            inputNode,
            (targetPropertyName, inputPort),
            getattr(inputNode, "id", "") if inputNode is not None else "",
        ),
    ):
        if prim_node is None:
            continue
        prim_getter = getattr(prim_node, "getUsdPrim", None)
        prim = prim_getter() if callable(prim_getter) else None
        if prim is None or not prim.IsValid():
            continue
        for property_name in property_candidates:
            if not property_name:
                continue
            rel = prim.GetRelationship(property_name)
            if rel and rel.IsValid():
                link.is_relationship_link = True
                link.propertyOwnerNodeId = owner_id
                link.propertyName = property_name
                return


def reconcile_moved_nodes(view):
    """Refresh position-derived state for every node whose ``position``
    version advanced since the last call (see ``VersionedVec2d``). Movers only
    set the position; this pass updates everything that depends on it, so none
    of them can leave it half-updated.

    A process-global move generation keeps the common case (nothing moved,
    e.g. a pan) at O(1). A drag refreshes its own derived state and is skipped
    here; the release frame reconciles the final positions.
    """
    if view.draggingNodes:
        return
    gen = legacy_symbol("positionWriteGeneration")()
    if gen == view._reconcilePositionGen:
        return
    view._reconcilePositionGen = gen
    seen = view._reconciledPositionVersions
    moved = set()
    for node_id, node in view.nodes.items():
        version = node.positionVersion
        if seen.get(node_id) != version:
            seen[node_id] = version
            moved.add(node_id)
    view._applyMovedNodeDerivedState(moved)


def update_links_for_moved_nodes(view, movedNodeIds):
    """Update all links that connect to any of the moved nodes.

    This finds links where EITHER end connects to a moved node and updates
    both endpoints. Much more efficient than rebuilding all links.

    Args:
        movedNodeIds: Set of node IDs that moved
    """
    for linkIdx, link in enumerate(view.links):
        # Check if either end of the link connects to a moved node
        sourceNodeId = getattr(link, "data_sourceNodeId", None)
        targetNodeId = getattr(link, "data_targetNodeId", None)

        if sourceNodeId in movedNodeIds or targetNodeId in movedNodeIds:
            # At least one end moved - need to update both endpoints
            if sourceNodeId not in view.nodes or targetNodeId not in view.nodes:
                continue

            sourceNode = view.nodes[sourceNodeId]
            targetNode = view.nodes[targetNodeId]

            # Both input and output links: sourceNode provides output, targetNode receives input
            view._setupLinkEndpoints(
                link,
                sourceNode,
                targetNode,
                link.sourcePort,
                link.targetPort,
            )

            # The endpoints just moved, so this link's spatial-index bounds
            # are stale. Re-insert (insertLink is an upsert) so the link
            # stays pickable during the drag instead of only after the next
            # full rebuild on linksChanged.
            view._spatialIndex.insertLink(linkIdx, view._computeLinkBounds(link))

    # Endpoints moved on the link snapshot; re-mirror to C++ next paint.
    view._linksNeedSync = True


def collect_sibling_dangling_links(view, seed_link):
    """Return every dangling link that shares ``seed_link``'s present-side port.

    A USD relationship with N targets produces N separate dangling
    ``LinkData`` objects that all share the source prim/port and stack
    their indicators on top of each other.  When the user expands the
    indicator we want to bring in ALL of those missing targets, not just
    the first one a port-hit search happens to return.
    """
    if not getattr(seed_link, "isDangling", False):
        return [seed_link]

    if seed_link.sourceNodeId in view.nodes:
        present_id = seed_link.sourceNodeId
        present_port = seed_link.sourcePort
        present_property = str(getattr(seed_link, "sourcePropertyName", ""))
        present_attr = "sourceNodeId"
        present_port_attr = "sourcePort"
        present_property_attr = "sourcePropertyName"
    elif seed_link.targetNodeId in view.nodes:
        present_id = seed_link.targetNodeId
        present_port = seed_link.targetPort
        present_property = str(getattr(seed_link, "targetPropertyName", ""))
        present_attr = "targetNodeId"
        present_port_attr = "targetPort"
        present_property_attr = "targetPropertyName"
    else:
        return [seed_link]

    siblings = []
    for link in view.links:
        if not getattr(link, "isDangling", False):
            continue
        if getattr(link, present_attr, "") != present_id:
            continue
        if getattr(link, present_port_attr, "") != present_port:
            continue
        if str(getattr(link, present_property_attr, "")) != present_property:
            continue
        siblings.append(link)
    return siblings or [seed_link]


def add_prim_as_node(view, prim, stage, addedNodeIds):
    """Helper to add a single prim as a node to the graph.

    Args:
        prim: USD prim to add
        stage: USD stage
        addedNodeIds: List to append added node IDs to

    Returns:
        True if node was added, False otherwise
    """
    primPath = prim.GetPath()
    primPathStr = str(primPath)

    # Skip if already in graph
    if primPathStr in view.nodes:
        return False

    # Use NodeFactory to create the node
    node = view._nodeFactory.create_node_from_prim(prim, stage)
    if not node:
        # No library can handle this prim type
        return False

    view._populateNodeLinksFromPrim(node, prim)

    # Calculate node size
    if view.fontAtlas:
        view.nodeGraph._calculateNodeSize(
            node, view.textRenderer.calculateTextWidth, view.fontAtlas
        )

    # Placement is deferred to the caller, which grid-places the whole batch
    # at once (sized to the batch and anchored below existing content).
    view.nodes[node.id] = node
    view._nextZOrder += 1
    node.zOrder = view._nextZOrder
    addedNodeIds.append(node.id)
    return True


def setup_dangling_link_endpoint(view, link, node, pinName, isOutput):
    """Set up a dangling link endpoint as a short straight line from the port.

    Args:
        link: LinkData to configure as dangling
        node: The node that owns the port
        pinName: Name of the port
        isOutput: True if this is an output port, False for input
    """
    # Get the port position (handles dual pins via mirroring)
    property_name = (
        str(getattr(link, "sourcePropertyName", ""))
        if isOutput
        else str(getattr(link, "targetPropertyName", ""))
    )
    portPos = view._getLinkEndpointPosition(
        node,
        pinName,
        isOutput,
        property_name,
        preferPropertySide=bool(getattr(link, "is_relationship_link", False)),
    )

    # Set the start and end points for the dangling link
    link.start = portPos

    # Calculate end point as a short line in the appropriate direction
    linkLength = LinkData.DANGLING_LINK_LENGTH
    if isOutput:
        # Output ports point to the right
        link.end = Gf.Vec2d(portPos[0] + linkLength, portPos[1])
    else:
        # Input ports point to the left
        link.end = Gf.Vec2d(portPos[0] - linkLength, portPos[1])

    link.isDangling = True
    link.danglingDirection = "output" if isOutput else "input"


def get_links_for_node_direction(view, node, direction):
    pinsToLinks = {}

    for i, link in enumerate(view.links):
        if direction == "output":
            # Links where this node is the data source (outputs)
            dataSource = getattr(link, "data_sourceNodeId", None)
            if dataSource == node.id:
                # Find pin index for sorting
                pinName = link.sourcePort
                pinIdx = view._findPinIndex(node.outputPins, pinName)
                if pinIdx == -1:
                    pinIdx = 999  # Put unknown pins at end
                if pinIdx not in pinsToLinks:
                    pinsToLinks[pinIdx] = []
                pinsToLinks[pinIdx].append((i, link))
        else:  # direction == "input"
            # Links where this node is the data target (inputs)
            dataTarget = getattr(link, "data_targetNodeId", None)
            if dataTarget == node.id:
                # For input links, the current node receives data
                if link.sourceNodeId == node.id:
                    pinName = link.sourcePort
                else:
                    pinName = link.targetPort
                pinIdx = view._findPinIndex(node.inputPins, pinName)
                if pinIdx == -1:
                    pinIdx = 999
                if pinIdx not in pinsToLinks:
                    pinsToLinks[pinIdx] = []
                pinsToLinks[pinIdx].append((i, link))

    # Sort links within each pin by link index for stable ordering
    for pinIdx in pinsToLinks:
        pinsToLinks[pinIdx].sort(key=lambda x: x[0])

    return pinsToLinks


def get_link_endpoint_position(
    view,
    node,
    pinName,
    isOutput,
    propertyName="",
    preferPropertySide=False,
):
    if not pinName and not propertyName:
        return view._getNodeTopCenterPosition(node)

    resolved_pin = view._resolveLinkDisplayPinName(
        node,
        pinName,
        propertyName,
        isOutput,
    )
    if propertyName and is_relationship_pin(propertyName):
        return view._getRelationshipRowEndpointPosition(
            node,
            resolved_pin,
            isOutput,
        )

    property_side = view._getPropertyOutputSide(propertyName)

    if property_side is None:
        return view._getPortPosition(node, resolved_pin, isOutput)
    if preferPropertySide:
        return view._getPortPosition(node, resolved_pin, property_side)
    if property_side == isOutput:
        return view._getPortPosition(node, resolved_pin, isOutput)

    opposite_pos = view._getPortPosition(node, resolved_pin, property_side)
    if opposite_pos is None:
        return view._getPortPosition(node, resolved_pin, isOutput)
    return view._mirrorPortPosition(node, opposite_pos)


def update_node_link_endpoints(view, node):
    """Update link endpoints for all links connected to a specific node.

    This is much faster than rebuilding all links when only one node has moved.

    Args:
        node: NodeModel whose connected links need updating
    """
    # Update input links (where this node receives data)
    # For input links: sourceNodeId = connected node (output), targetNodeId = this node
    for link in node.inputLinks:
        if link.sourceNodeId not in view.nodes:
            continue

        connectedNode = view.nodes[link.sourceNodeId]

        # Update endpoints
        view._setupLinkEndpoints(
            link, connectedNode, node, link.sourcePort, link.targetPort
        )

    # Update output links (where this node provides data)
    for link in node.outputLinks:
        if link.targetNodeId not in view.nodes:
            continue

        connectedNode = view.nodes[link.targetNodeId]

        # Update endpoints
        view._setupLinkEndpoints(
            link, node, connectedNode, link.sourcePort, link.targetPort
        )


def sync_links_if_needed(view, graph):
    """Mirror self.links into the held C++ GraphModel snapshot, but only when
    that snapshot is stale.

    syncLinksFromModels copies every LinkData across the C++ boundary, so
    doing it unconditionally every frame is O(links) of pure marshalling on
    the redraw hot path. The snapshot only goes stale when the hovered link
    changes, or when some other path mutates a rendered link field
    (selection, highlight, endpoints, membership) and flags
    self._linksNeedSync. A plain pan/zoom redraw changes neither, so it skips
    the marshal entirely.

    Hover is derived from self._linkUnderCursor and re-applied here (rather
    than every frame) whenever the hovered index changes; it persists on the
    snapshot between frames, so there is no per-frame set/clear sweep.
    """
    if (
        view._linkUnderCursor == view._lastSyncedLinkHover
        and not view._linksNeedSync
    ):
        return
    for i, link in enumerate(view.links):
        link.hovered = i == view._linkUnderCursor
    view._lastSyncedLinkHover = view._linkUnderCursor
    graph.syncLinksFromModels(view.links)
    view._linksNeedSync = False


def setup_link_endpoints(view, link, startNode, endNode, startPinName, endPinName):
    """Set up link endpoints using renderer-provided port positions.

    Args:
        link: LinkData to configure
        startNode: Node where the link starts (has the output port)
        endNode: Node where the link ends (has the input port)
        startPinName: Name of the output port on startNode
        endPinName: Name of the input port on endNode
    """
    # Get port positions from renderers (isOutput=True for start, False for end)
    link.start = view._getLinkEndpointPosition(
        startNode,
        startPinName,
        True,
        str(getattr(link, "sourcePropertyName", "")),
        preferPropertySide=bool(getattr(link, "is_relationship_link", False)),
    )
    link.end = view._getLinkEndpointPosition(
        endNode,
        endPinName,
        False,
        str(getattr(link, "targetPropertyName", "")),
        preferPropertySide=bool(getattr(link, "is_relationship_link", False)),
    )


def has_exact_link(
    view,
    outputNodeId,
    outputPortName,
    inputNodeId,
    inputPortName,
    outputPropertyName="",
    inputPropertyName="",
):
    """Return True if an identical link already exists."""
    out_prop = outputPropertyName or ""
    in_prop = inputPropertyName or ""
    for existing in view.links:
        if (
            existing.sourceNodeId == outputNodeId
            and existing.sourcePort == outputPortName
            and existing.targetNodeId == inputNodeId
            and existing.targetPort == inputPortName
            and (getattr(existing, "sourcePropertyName", "") or "") == out_prop
            and (getattr(existing, "targetPropertyName", "") or "") == in_prop
        ):
            return True
    return False


def resolve_link_display_pin_name(view, node, pinName, propertyName, isOutput):
    if propertyName and hasattr(node, "_prim") and node._prim:
        display_pin_name = get_display_pin_name_for_property_name(
            node._prim,
            propertyName,
        )
        if display_pin_name is not None:
            pinName = display_pin_name
    elif propertyName:
        # No prim available: strip any direction hint and keep namespaced
        # names intact (they double as their own display pin name).
        pinName = split_direction_hint(propertyName)[1]

    if hasattr(node, "resolve_pin_name"):
        resolution_side = view._getPropertyOutputSide(propertyName)
        if resolution_side is None:
            resolution_side = isOutput
        try:
            return node.resolve_pin_name(pinName, is_output=resolution_side)
        except TypeError:
            return node.resolve_pin_name(pinName)
    return pinName


def find_link_to_input(view, nodeId, portName):
    """
    Find link connected to specified input port.

    Args:
        nodeId: ID of the node
        portName: Name of the input port

    Returns:
        LinkData if found, None otherwise
    """
    if is_multi_target_relationship_input_pin(portName):
        return None
    for link in view.links:
        if link.targetNodeId == nodeId and link.targetPort == portName:
            return link
    return None


def make_link_from_usd_data(link_data):
    link = LinkData()
    link.sourceNodeId = link_data["sourceNodeId"]
    link.sourcePort = link_data["sourcePinName"]
    link.targetNodeId = link_data["targetNodeId"]
    link.targetPort = link_data["targetPinName"]
    link.is_input_link = bool(link_data["is_input_link"])
    link.propertyOwnerNodeId = str(link_data.get("propertyOwnerNodeId", ""))
    link.propertyName = str(link_data.get("propertyName", ""))
    link.is_relationship_link = bool(link_data.get("is_relationship_link", False))
    link.sourcePropertyName = str(link_data.get("sourcePropertyName", ""))
    link.targetPropertyName = str(link_data.get("targetPropertyName", ""))
    return link


def add_nodes_from_dangling_link(view, seed_link):
    """Add every missing target for the dangling-link expansion ``seed_link``.

    Returns True when at least one node was added.  Wraps
    :meth:`_addNodeFromDanglingLink` so a USD relationship with multiple
    targets is fully expanded in a single user gesture.
    """
    added_any = False
    for link in view._collectSiblingDanglingLinks(seed_link):
        if view._addNodeFromDanglingLink(link):
            added_any = True
    return added_any


def populate_node_links_from_prim(view, node, prim):
    node.inputLinks = []
    node.outputLinks = []
    for link_data in legacy_symbol("collect_links_for_prim")(prim):
        link = view._makeLinkFromUsdData(link_data)
        if link.is_input_link:
            node.inputLinks.append(link)
        else:
            node.outputLinks.append(link)


def invalidate_back_reference_link_data_cache(view):
    """Drop the cached per-prim USD link scan used for back-references.

    Called when stage composition changes (resync notice, stage replace)
    so the next ``_addBackReferenceDanglingLinks`` call re-traverses the
    stage. In steady state (no USD edits between paints) the cache is
    reused, avoiding the O(prim-count) traversal on every paint.
    """
    view._backReferenceLinkDataCache = None


def get_links_for_node_flat(view, node, direction):
    pinsToLinks = view._getLinksForNodeDirection(node, direction)

    # Flatten to list sorted by pin index
    result = []
    for pinIdx in sorted(pinsToLinks.keys()):
        result.extend(pinsToLinks[pinIdx])

    return result


def resolve_port_property_name(view, node, pinName, isOutput):
    if not hasattr(node, "_prim") or not node._prim:
        return pinName
    side = "output" if isOutput else "input"
    property_name = resolve_property_name_for_display_pin(node._prim, pinName, side)
    if property_name is not None:
        return property_name
    return pinName


def find_pin_index(view, pins, pinName):
    for i, pin in enumerate(pins):
        if pin == pinName:
            return i
    return -1


def compute_link_bounds(view, link):
    """Curve-inclusive Gf.Range2d for the spatial index.

    Uses the shaped-curve bounds (covering the backward S-bow) rather than
    the straight start/end box, so broad-phase queries do not mis-cull
    backward links whose curve bulges outside their endpoint box. The bounds
    are zoom-independent; the 20px margin covers link line width and the
    prim-target arrow inset.
    """
    return computeLinkCurveBounds(link, margin=20.0)


def rebuild_connection_cache(view):
    view.nodeGraph.buildConnectionCache()


# Placement and port geometry
def node_rect(node):
    """``(x, y, w, h)`` floats: a node as layout input."""
    return (
        float(node.position[0]),
        float(node.position[1]),
        float(node.size[0]),
        float(node.size[1]),
    )


def batch_extents(view, placeable):
    """The (width, height) the grid below will actually occupy.

    Mirrors the layout loop exactly -- uniform columns sized to the widest
    node, rows only as tall as their own tallest node. Guessing high here
    is not "safe": an over-tall box collides with things the batch would
    never have reached, and each collision pushes the anchor further down,
    which is how a batch ends up nowhere near where it was asked to go.
    """
    cellW = max(n.size[0] for n in placeable) + view._AUTO_GRID_GAP
    cols = view._AUTO_GRID_COLUMNS
    width = cellW * min(len(placeable), cols)
    height = 0.0
    for rowStart in range(0, len(placeable), cols):
        rowNodes = placeable[rowStart : rowStart + cols]
        height += max(n.size[1] for n in rowNodes) + view._AUTO_GRID_GAP
    return width, height


def push_below_occupied(view, anchorX, anchorY, placeable, placed):
    """Slide ``anchorY`` down until the batch clears the nodes already placed.

    A caller-chosen anchor says *where the user is looking*, not *where
    there is room*; dropping a batch straight onto existing nodes would hide
    both. Each pass moves the anchor just under whatever it collides with
    and re-tests, bounded by the number of placed nodes because every pass
    clears at least one of them.
    """
    if not placed:
        return anchorY

    batchWidth, batchHeight = view._batchExtents(placeable)

    for _ in range(len(placed)):
        collided = False
        for node in placed:
            nx, ny = node.position[0], node.position[1]
            nw, nh = node.size[0], node.size[1]
            if (
                anchorX < nx + nw
                and anchorX + batchWidth > nx
                and anchorY < ny + nh
                and anchorY + batchHeight > ny
            ):
                anchorY = ny + nh + view._AUTO_GRID_GAP
                collided = True
                break
        if not collided:
            break
    return anchorY


def grid_place_nodes(view, nodes, anchor=None):
    """Lay out the (0, 0) nodes among ``nodes`` in a grid and return them.

    Only nodes still at the USD default (0, 0) are placed; nodes with their
    own position (an authored ``ui:nodegraph:node:pos`` or one the loader set
    for virtual input/output nodes) are left untouched. The batch is anchored
    at world origin when nothing else is placed (a fresh load) and otherwise
    just below the bounding box of the already-placed nodes, so adding into an
    arranged graph never lands on top of it. Columns are a uniform width and
    each row is only as tall as the tallest node in that row, so one very tall
    node doesn't inflate every row's spacing. Placement is display-only (no USD
    authoring), so it can't fail on a prim that won't take the attribute and
    opening a stage never dirties it.

    Args:
        nodes: Candidate nodes; only those still at (0, 0) are placed.
        anchor: Optional ``(x, y)`` in world space to start the batch at,
            for callers that need the result somewhere specific -- adding
            into the viewport the user is looking at, rather than below
            content that may be far off screen. The batch is still pushed
            clear of anything already placed that it would land on.
    """
    placeable = [n for n in nodes if n.position[0] == 0.0 and n.position[1] == 0.0]
    if not placeable:
        return []
    placeableIds = {n.id for n in placeable}

    # Anchor below everything already placed (origin if there is nothing).
    anchorX, anchorY = 0.0, 0.0
    placed = [
        n
        for n in view.nodes.values()
        if n.id not in placeableIds
        and (n.position[0] != 0.0 or n.position[1] != 0.0)
    ]
    if anchor is not None:
        anchorX, anchorY = float(anchor[0]), float(anchor[1])
        anchorY = view._pushBelowOccupied(anchorX, anchorY, placeable, placed)
    elif placed:
        anchorX = min(n.position[0] for n in placed)
        anchorY = (
            max(n.position[1] + n.size[1] for n in placed) + view._AUTO_GRID_GAP
        )

    # Uniform column width keeps columns aligned; each row's height is sized
    # to the tallest node IN THAT ROW (not the whole batch) so a single very
    # tall node doesn't inflate every row's vertical gap.
    cellW = max(n.size[0] for n in placeable) + view._AUTO_GRID_GAP
    cols = view._AUTO_GRID_COLUMNS
    y = anchorY
    for rowStart in range(0, len(placeable), cols):
        rowNodes = placeable[rowStart : rowStart + cols]
        for col, node in enumerate(rowNodes):
            # Display-only (no USD authoring): never fails on a prim that can't
            # take ui:nodegraph:node:pos, and opening a stage doesn't dirty it.
            node.setDisplayPosition(Gf.Vec2d(anchorX + col * cellW, y))
        y += max(n.size[1] for n in rowNodes) + view._AUTO_GRID_GAP
    return placeable


def place_added_nodes(view, nodes, fallback=None):
    """Position freshly added nodes by connection; return the placed ones.

    Only nodes still at the USD default (0, 0) are placed; nodes with
    their own position (authored ``ui:nodegraph:node:pos`` or loader-set)
    are left untouched. Each placed node lands beside the neighbors it
    wires to -- one column past its inputs, else one column before its
    outputs -- and slides to a slot that overlaps nothing; unconnected
    nodes take *fallback* (world origin into an empty canvas, else the
    caller's viewport anchor). Only the new nodes move. Placement is
    display-only (no USD authoring), like the grid this replaces on the
    add path.
    """
    placeable = [n for n in nodes if n.position[0] == 0.0 and n.position[1] == 0.0]
    if not placeable:
        return []
    fx, fy = (
        (0.0, 0.0)
        if fallback is None
        else (float(fallback[0]), float(fallback[1]))
    )
    # Edges come from the node link lists, not view.links: the graph
    # link list rebuilds on the next paint, but the added nodes' lists
    # were populated when they were added and everyone else's are
    # current. Attribute and relationship links place alike: any noodle
    # touching the new node tells it where to stand.
    preds_of = {}
    succs_of = {}
    for node in view.nodes.values():
        preds_of[node.id] = [
            link.sourceNodeId for link in getattr(node, "inputLinks", [])
        ]
        succs_of[node.id] = [
            link.targetNodeId for link in getattr(node, "outputLinks", [])
        ]
    batch_ids = {n.id for n in placeable}
    positioned = {
        nid: n for nid, n in view.nodes.items() if nid not in batch_ids
    }
    placed = []
    for node in placeable:
        preds = [
            node_rect(positioned[pid])
            for pid in preds_of.get(node.id, [])
            if pid in positioned
        ]
        succs = [
            node_rect(positioned[sid])
            for sid in succs_of.get(node.id, [])
            if sid in positioned
        ]
        occupants = [node_rect(n) for n in positioned.values()]
        x, y = place_node(
            (float(node.size[0]), float(node.size[1])),
            preds,
            succs,
            occupants,
            (fx, fy),
        )
        node.setDisplayPosition(Gf.Vec2d(x, y))
        positioned[node.id] = node
        placed.append(node)
    return placed


def auto_layout_nodes(view):
    """Arrange every node left-to-right by connection (L / context menu).

    Layered layout over every rendered link (see :mod:`nodeLayout`):
    sources left, sinks right, cycles sharing a column, over-full
    layers wrapped into sub-columns, components tiled into rows and
    unwired nodes parked in a grid below. Attribute and relationship
    links rank alike -- any noodle on the canvas is a connection the
    user reads, so all of them lay out. Only dangling links (an
    endpoint off the canvas) are skipped. The new positions are
    authored to USD as a single undo step.

    Each link also carries the y offset of its own two endpoints,
    taken from the endpoints the renderer already resolved: on a node
    two thousand pixels tall, "line the boxes up" still leaves the
    noodle sweeping the whole height, so the layout lines up the
    PINS. A link whose endpoints are not resolved yet falls back to
    the plain pair, which aims at the node centres instead.
    """
    if not view.nodes:
        return

    sizes = {
        node_id: (float(node.size[0]), float(node.size[1]))
        for node_id, node in view.nodes.items()
    }
    edges = []
    for link in view.links:
        src = link.sourceNodeId
        dst = link.targetNodeId
        if src not in sizes or dst not in sizes:
            continue
        start = getattr(link, "start", None)
        end = getattr(link, "end", None)
        try:
            start_y = float(start[1])
            end_y = float(end[1])
        except (TypeError, IndexError, ValueError):
            edges.append((src, dst))
            continue
        source_node = view.nodes[src]
        target_node = view.nodes[dst]
        edges.append(
            (
                src,
                dst,
                start_y - float(source_node.position[1]),
                end_y - float(target_node.position[1]),
            )
        )
    positions = legacy_symbol("layout_graph")(sizes, edges)
    if not positions:
        return

    # Capture old and new positions so the whole arrange is one undo step
    # (mirrors _finalizeNodeDrag).
    old_positions = {}
    new_positions = {}
    for node_id, node in view.nodes.items():
        new_pos = positions.get(node_id)
        if new_pos is None:
            continue
        old = node.position
        old_positions[node_id] = (old[0], old[1])
        new_positions[node_id] = (new_pos[0], new_pos[1])

    if not new_positions:
        return

    view._applyNodePositions(new_positions)

    legacy_symbol("_push_undo_command")(
        "Auto Layout",
        lambda gv, p=new_positions: gv._restoreNodePositions(p),
        lambda gv, p=old_positions: gv._restoreNodePositions(p),
        view=view,
    )

    view.frameAll()


def update_node_spatial_bounds(view, node_ids):
    """Refresh the spatial-index bounds for the given moved nodes.

    Hit-testing (click, marquee, hover) resolves nodes through the spatial
    index, so any path that changes a node's position must keep the index
    consistent with the drawn position. This is the shared helper that does
    it; callers that move nodes route through here instead of recomputing
    bounds inline.
    """
    for node_id in node_ids:
        node = view.nodes.get(node_id)
        if node is None:
            continue
        bounds = Gf.Range2d(
            Gf.Vec2d(node.position),
            Gf.Vec2d(node.position + node.size),
        )
        view._spatialIndex.updateNode(node_id, bounds)


def apply_moved_node_derived_state(view, moved):
    """Refresh the state derived from a node's position for ``moved``:
    spatial-index bounds (hit-testing), link endpoints, and port highlights.

    Both callers that move nodes (the per-frame reconcile and the live drag)
    go through here, so the set of position-derived state is defined in one
    place. Render re-baking is not included; it differs by caller (bulk moves
    call _resetRenderCachesAfterBulkMove, a drag rides the transform frame).
    """
    if not moved:
        return
    view._updateNodeSpatialBounds(moved)
    view._updateLinksForMovedNodes(moved)
    view._portHighlightsDirty = True


def node_has_visible_port(view, node, pinName, isOutput):
    if hasattr(node, "has_visible_port"):
        return node.has_visible_port(pinName, isOutput)
    pins = node.outputPins if isOutput else node.inputPins
    if pinName in pins:
        return True
    if isOutput:
        return pinName in getattr(node, "_dual_pin_names", set())
    return pinName in getattr(node, "_output_dual_pin_names", set())


def get_node_top_center_position(node):
    return Gf.Vec2d(
        node.position[0] + node.size[0] * 0.5,
        node.position[1],
    )


def get_property_output_side(propertyName):
    side, _stripped = split_direction_hint(propertyName)
    if side == "output" or is_relationship_pin(propertyName):
        return True
    if side == "input" or is_relationship_input_pin(propertyName):
        return False
    # Bare attrs (dual pins) and other namespaced attrs have no inherent
    # side here; the caller falls back to the link's own direction.
    return None


def mirror_port_position(node, portPos):
    mirrored_x = node.position[0] + node.size[0] - (portPos[0] - node.position[0])
    return type(portPos)(mirrored_x, portPos[1])


def get_visible_port_position(view, node, pinName, isOutput):
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    if isOutput and pinName in getattr(node, "_dual_pin_names", set()):
        inPos = renderer.getPortPosition(node, pinName, False, view.fontAtlas)
        if inPos:
            return view._mirrorPortPosition(node, inPos)
        return None
    if not isOutput and pinName in getattr(node, "_output_dual_pin_names", set()):
        outPos = renderer.getPortPosition(node, pinName, True, view.fontAtlas)
        if outPos:
            return view._mirrorPortPosition(node, outPos)
        return None
    if not view._nodeHasVisiblePort(node, pinName, isOutput):
        return None
    return renderer.getPortPosition(node, pinName, isOutput, view.fontAtlas)


def get_relationship_row_endpoint_position(view, node, pinName, isOutput):
    visiblePos = view._getVisiblePortPosition(node, pinName, isOutput)
    if visiblePos is not None:
        return visiblePos

    oppositePos = view._getVisiblePortPosition(node, pinName, not isOutput)
    if oppositePos is not None:
        return view._mirrorPortPosition(node, oppositePos)

    fallbackPos = view._getPortPosition(node, pinName, isOutput)
    if fallbackPos is not None:
        return fallbackPos
    return view._getNodeTopCenterPosition(node)


def get_port_position(view, node, pinName, isOutput):
    """Resolve a pin's port position via the C++ View Model.

    Thin shim over ``GraphNodeRenderer.resolvePortPosition``, the single
    node-local resolver that handles synthetic relationship ports, dual /
    output-dual mirrors, and normal pins. Python no longer computes port
    geometry — it just reads the View Model.
    """
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return None
    return renderer.resolvePortPosition(node, pinName, isOutput)


def get_connected_nodes(view, nodeId):
    return view.nodeGraph.getConnectedNodeIds(nodeId)


def append_triangle_vertices(view, vertices, points, depth, color):
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    width = max(max_x - min_x, 1.0)
    height = max(max_y - min_y, 1.0)
    r = int(color[0] * 255)
    g = int(color[1] * 255)
    b = int(color[2] * 255)
    a = int(color[3] * 255)
    for point_x, point_y in points:
        vertices.append(
            legacy_symbol("NodeVertex")(
                float(point_x),
                float(point_y),
                depth,
                float(point_x - min_x),
                float(point_y - min_y),
                float(width),
                float(height),
                r,
                g,
                b,
                a,
                0.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )


def node_has_authored_position(view, node):
    """Return True if the node's prim already has an authored position.

    ``ui:nodegraph:node:pos`` is the editor's per-node placement metadata.
    When it is authored, the position was set deliberately (a previous
    editor session, an upstream tool, or hand-authoring) and must be
    respected when the node is added to the graph instead of being replaced
    with an auto-layout position.
    """
    prim = node.getUsdPrim()
    if not prim or not prim.IsValid():
        return False
    posAttr = prim.GetAttribute("ui:nodegraph:node:pos")
    return bool(posAttr and posAttr.HasValue())


def position_node_in_viewport(view, node):
    """Position a node in the center of the current viewport, avoiding overlap.

    A node whose prim already has an authored ``ui:nodegraph:node:pos`` is
    left where it is — we respect the authored position instead of
    overwriting it with an auto-computed placement. Only nodes that have no
    position of their own are auto-placed (which also avoids leaving them
    stacked at the USD default of (0, 0)).
    """
    if view._nodeHasAuthoredPosition(node):
        # Respect the authored position, but read it once so the cached and
        # C++-side positions sync from the authored USD value without
        # re-authoring it. The noodle renderer reads the C++
        # NodeData.position (via getPortPosition) to place link endpoints;
        # the old code reached that sync through the position setter, so
        # skipping the setter here must not skip the sync, or connected
        # noodles draw to a stale (0, 0) endpoint.
        _ = node.position
        return
    scaledW = view.width() / view.zoom
    scaledH = view.height() / view.zoom
    centerX = view.panX + scaledW * 0.5
    centerY = view.panY + scaledH * 0.5

    nodeW = node.size[0] if node.size[0] > 0 else 200
    nodeH = node.size[1] if node.size[1] > 0 else 100

    # Start at viewport center
    x = centerX - nodeW * 0.5
    y = centerY - nodeH * 0.5

    # Nudge to avoid overlap with existing nodes. The cap scales with the node
    # count so the search can always clear a full row instead of giving up
    # after a fixed number of steps and dropping the node onto others.
    padding = 30.0
    for _ in range(len(view.nodes) + 1):
        overlap = False
        for existingNode in view.nodes.values():
            ex, ey = existingNode.position[0], existingNode.position[1]
            ew = existingNode.size[0] if existingNode.size[0] > 0 else 200
            eh = existingNode.size[1] if existingNode.size[1] > 0 else 100
            if (
                x < ex + ew + padding
                and x + nodeW + padding > ex
                and y < ey + eh + padding
                and y + nodeH + padding > ey
            ):
                overlap = True
                # Place to the right of the overlapping node
                x = ex + ew + padding
                break
        if not overlap:
            break

    node.position = Gf.Vec2d(x, y)


# Inline value geometry and metrics
def value_row_layout(view, node):
    """``(input_pins, in_slots, in_kinds, occupied)`` for the value rows.

    The C++ slot/kind vectors are copied out of the bindings ONCE per
    layout epoch instead of once per node per frame: they only change in
    the C++ layout passes (which bump ``node._row_layout_version``) or
    are cleared by ``invalidateCache`` (which drops this cache), so an
    equal version means reusable geometry.
    """
    version = getattr(node, "_row_layout_version", None)
    cached = getattr(node, "_value_row_layout", None)
    if version is not None and cached is not None and cached[0] == version:
        return cached[1], cached[2], cached[3], cached[4]
    input_pins = list(getattr(node, "inputPins", []))
    in_slots = list(node.inputRowSlots)
    in_kinds = list(node.inputRowKinds)
    # Every slot an output-side row occupies. A real output row's label is
    # right-aligned to the node's right edge -- into exactly the space the
    # cell wants -- so where the two share a slot, the cell gives way.
    out_pins = getattr(node, "outputPins", [])
    out_slots = list(node.outputRowSlots)
    occupied = set(
        out_slots[i] if i < len(out_slots) else i for i in range(len(out_pins))
    )
    if version is not None:
        try:
            node._value_row_layout = (
                version,
                input_pins,
                in_slots,
                in_kinds,
                occupied,
            )
        except AttributeError:
            pass
    return input_pins, in_slots, in_kinds, occupied


def value_row_geometry(view, node):
    """Yield ``(row, cellRect, subRects, H)`` for each value-bearing row.

    Row geometry comes from :meth:`_getRowHitMetrics` -- the same function
    the connection hit-tests use -- so the cells cannot drift away from
    the rows they belong to (which is the failure
    ``testUsdNoodlesRowDrift.py`` exists to prevent).

    The four row-level suppressions live here rather than in the value
    model, because each of them is a fact about the LAYOUT:
      * a collapsed title has no rows at all;
      * a group HEADER is not a property row. The C++ row kinds are
        ``0 = normal, 1 = folded header, 2 = unfolded header, 3 = child``
        (noodles core/NodeData.h:48), so 1 and 2 are dropped and 3 -- a
        real property inside an open group, which is where
        ``rigExec:controlSpace`` lives -- keeps its cell;
      * a slot shared with an output row has that row's label
        right-aligned into exactly the space the cell wants;
      * a vector too narrow to read drops back to today's look.
    """
    if not view._cachedShowAttributeValues:
        return
    if getattr(node, "_title_collapsed", False):
        return
    rows_fn = getattr(node, "value_rows", None)
    if rows_fn is None:
        return
    renderer = node.renderer if node.renderer else view.defaultRenderer
    if not renderer:
        return
    rows = rows_fn(view._currentTimeCode)
    if not rows:
        return
    cell_w = float(getattr(node, "_value_cell_width", 0.0) or 0.0)
    if cell_w <= 0.0:
        return

    # Import on demand: the widgets package initializer loads the UI.
    from ..widgets import valueCell

    nx = float(node.position[0])
    nw = float(node.size[0])
    if not valueCell.node_is_wide_enough(nw, renderer.getPortWidth()):
        return
    port_start_y, h = view._getRowHitMetrics(node, renderer)
    band = view._valueTextBand(h)

    input_pins, in_slots, in_kinds, occupied = view._valueRowLayout(node)

    for i, pin_name in enumerate(input_pins):
        row = rows.get((pin_name, False))
        if row is None:
            continue
        kind = in_kinds[i] if i < len(in_kinds) else 0
        if kind in (1, 2):
            continue
        slot = in_slots[i] if i < len(in_slots) else i
        if slot in occupied:
            continue
        row_top = port_start_y + slot * h
        rect = valueCell.cell_rect(
            nx,
            nw,
            row_top,
            h,
            cell_w,
            top=row_top + band[0],
            height=band[1],
        )
        if rect is None:
            continue
        inset = valueCell.swatch_inset(h) if row.has_swatch else 0.0
        subs = valueCell.sub_rects_fit(rect, row.count, h, inset)
        if not subs:
            continue
        yield row, rect, subs, h


def value_cell_rect_for(view, node, propertyName, comp=0):
    """World rect of one component's cell, or None when it has none.

    The paint pass, the hit test and the tests all ask through here, so
    there is one answer to "where is that cell".
    """
    for row, rect, subs, _h in view._valueRowGeometry(node):
        if row.property_name != propertyName:
            continue
        if 0 <= comp < len(subs):
            return subs[comp]
        return rect
    return None


def as_world_xy(point):
    if hasattr(point, "x"):
        return float(point.x()), float(point.y())
    return float(point[0]), float(point[1])


def value_font_size(view):
    """Value text size: a little smaller than the row label, never its own.

    Derived from nodePinFontSize so a user who scales the node fonts
    scales the values with them.
    """
    return float(view._cachedNodePinFontSize) * float(
        view._cachedValueFontScale
    )


def value_digit_advance(view, size):
    """The widest digit at *size*: the cell every digit is set in."""
    cached = view._valueDigitAdvanceCache.get(size)
    if cached is None:
        cached = max(
            view.textRenderer.calculateTextWidth(d, size)
            for d in "0123456789"
        )
        view._valueDigitAdvanceCache[size] = cached
    return cached


def value_text_width(view, text, size, tabular):
    if not tabular:
        return view.textRenderer.calculateTextWidth(text, size)
    advance = view._valueDigitAdvance(size)
    total = 0.0
    for ch in text:
        if ch.isdigit():
            total += advance
        else:
            total += view._valueGlyph(ch, size)[1]
    return total


def value_state_colors(view, row, theme, hovered, active):
    """(fill, stroke, textColor, italicShear) for one row's pill.

    Fill is None where the state asks for no pill at all: a schema
    fallback is information the user did not author, so it is shown as
    dim text on the bare row rather than as a filled field inviting a
    click.
    """
    fill = theme.fillAuthored
    stroke = theme.hairline
    text = theme.textNormal
    italic = 0.0
    if row.state == noodlesValues.STATE_CONNECTED or not row.editable:
        fill = None
        stroke = None
        text = theme.textConnected if row.state == (
            noodlesValues.STATE_CONNECTED
        ) else theme.textDim
        italic = 0.20
    elif row.state == noodlesValues.STATE_FALLBACK:
        fill = None
        stroke = None
        text = theme.textDim
    elif row.state == noodlesValues.STATE_ANIMATED:
        text = theme.textAnimated
    if hovered and fill is not None:
        fill = theme.fillHover
    if active:
        fill = theme.fillAccent
        stroke = theme.accent
    return fill, stroke, text, italic


def find_value_row(view, nodeId, propertyName):
    node = view.nodes.get(nodeId)
    if node is None:
        return None
    lookup = getattr(node, "value_row_for_property", None)
    if lookup is not None:
        return lookup(propertyName, view._currentTimeCode)
    rows_fn = getattr(node, "value_rows", None)
    if rows_fn is None:
        return None
    for row in rows_fn(view._currentTimeCode).values():
        if row.property_name == propertyName:
            return row
    return None


def editable_value_cells(view, nodeId):
    """Every ``(propertyName, component)`` Tab can reach, in row order."""
    cells = []
    node = view.nodes.get(nodeId)
    if node is None:
        return cells
    for row, _rect, subs, _h in view._valueRowGeometry(node):
        if not row.editable or row.kind == noodlesValues.KIND_BOOL:
            continue
        for index in range(len(subs)):
            cells.append((row.property_name, index))
    return cells


def value_axis_width(view, label, size):
    """Width of one axis hint (``x``/``y``/``z``/``r``/...) at *size*.

    A handful of constant single characters measured once each instead
    of once per vector row per frame.
    """
    key = (label, size)
    width = view._valueAxisWidthCache.get(key)
    if width is None:
        width = view.textRenderer.calculateTextWidth(label, size)
        view._valueAxisWidthCache[key] = width
    return width


def value_fitted_texts(view, nodeId, row, slotWidth, fontSize, rowMeasure):
    """Display texts for one row's components, fitted once per value.

    The fit is pure arithmetic over (components, width, font): refitting
    it every frame re-measures every candidate format through C++, so
    the answer is cached until one of those changes.
    """
    key = (
        row.components,
        round(float(slotWidth), 2),
        row.type_name,
        fontSize,
    )
    entry = view._valueFitCache.get((nodeId, row.property_name))
    if entry is not None and entry[0] == key:
        return entry[1]
    texts = noodlesValues.fit_components(
        row.components, row.type_name, max(slotWidth, 1.0), rowMeasure
    )
    view._valueFitCache[(nodeId, row.property_name)] = (key, texts)
    return texts
