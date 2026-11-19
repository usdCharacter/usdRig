#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Prim-path identity for graph nodes: the base layer of the graph view.

Path relations decide both which nodes follow a namespace edit and which
nodes a resync invalidates. Everything here works on the strings the
producers and consumers already agree on, so it imports nothing: no pxr, no
Qt/GL, no ``UsdNoodles`` module, no stage lookup, and no path parsing.
"""


def node_id_prim_path(node_id: str) -> str:
    """The prim-path half of a node id.

    Most node ids ARE prim paths. Blueprint / Container I/O nodes are
    synthesized rather than prim-backed and carry their pin in a ``#`` suffix
    (``/Bp#input:foo:bar``); the part before the ``#`` is still the prim path
    they hang off, which is what makes them follow that prim when it moves.
    """
    return node_id.split("#", 1)[0]


def paths_related(path_a: str, path_b: str) -> bool:
    """Whether one path is the other, or contains the other.

    "Contains" spans both separators USD uses: ``/`` for a child prim and ``.``
    for a property, so ``/World/Cube`` is related to ``/World/Cube/Leg`` and to
    ``/World/Cube.inputs:x`` alike -- a resync can name either, and both mean
    "the node at /World/Cube is stale".

    The boundary is the point. Plain ``startswith`` reads ``/World/Cube`` as an
    ancestor of ``/World/CubeTwo``, so editing one prim invalidates unrelated
    siblings whose names merely share a prefix.
    """
    if path_a == path_b:
        return True
    longer, shorter = (path_a, path_b) if len(path_a) > len(path_b) else (path_b, path_a)
    if not longer.startswith(shorter):
        return False
    # The pseudo-root is the one path that already ends in its own separator.
    if shorter.endswith("/"):
        return True
    return longer[len(shorter)] in "/."
