#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Synthetic I/O endpoints for blueprint and container scopes.

One derivation with three callers: the blueprint loader, the container
loader, and moved-scope reconciliation in ``graphview/stage_adapter``.
A scope move must emit exactly what a fresh load at the destination
emits -- the same attribute selection, connection walk, fieldname
metadata and fieldname-mode rule for inputs and outputs -- or the
reconciled pins disagree with the stage they reflect.
"""

from .pinUtils import infer_bare_link_direction, split_direction_hint


def accept_blueprint_child(childPrim):
    """Whether the blueprint loader processes *childPrim*'s connections."""
    return not (
        childPrim.GetTypeName() in ("", None)
        and not childPrim.GetAttributes()
    )


def accept_container_child(childPrim):
    """Whether the container loader processes *childPrim*'s connections."""
    return childPrim.GetTypeName() in ("ExecNode", "Container", "Shader")


def attribute_fieldname(attr):
    """The fieldname in *attr*'s assetInfo metadata, or None.

    Custom data first, then the metadata dictionary fallback.
    """
    assetInfo = attr.GetCustomDataByKey("assetInfo")
    if assetInfo and isinstance(assetInfo, dict):
        return assetInfo.get("fieldname")
    metadata = attr.GetAllMetadata()
    if metadata and "assetInfo" in metadata:
        assetInfo = metadata["assetInfo"]
        if isinstance(assetInfo, dict):
            return assetInfo.get("fieldname")
    return None


def scope_pin_names(scopePrim):
    """The scope prim's own pins as ``(inputs, outputs)`` pin-name lists.

    Direction-hint attributes land on their side; bare attributes appear
    on both. Every attribute the stage composes counts, authored or not.
    """
    inputs = []
    outputs = []
    for attr in scopePrim.GetAttributes():
        attrName = attr.GetName()
        side, pinName = split_direction_hint(attrName)
        if side == "input":
            inputs.append(pinName)
        elif side == "output":
            outputs.append(pinName)
        elif ":" not in attrName:
            if attrName not in inputs:
                inputs.append(attrName)
            if attrName not in outputs:
                outputs.append(attrName)
    return (inputs, outputs)


def scope_child_connections(childPrim, useFieldnames):
    """One record per USD connection on *childPrim*'s pin attributes.

    Each record holds ``pin_name``, ``is_input``, ``target_prim_path``,
    ``target_pin_name``, ``target_is_input`` and ``fieldname`` (None
    unless *useFieldnames*). Bare attributes infer their side per
    connection from the target; other namespaced attributes are skipped.
    """
    for attr in childPrim.GetAttributes():
        attrName = attr.GetName()
        isBare = False
        side, pinName = split_direction_hint(attrName)
        if side == "input":
            isInput = True
        elif side == "output":
            isInput = False
        elif ":" not in attrName:
            isBare = True
            isInput = True
        else:
            continue
        connections = attr.GetConnections()
        if not connections:
            continue
        fieldname = (
            attribute_fieldname(attr) if useFieldnames else None
        )
        for connPath in connections:
            connPathStr = str(connPath)
            if "." not in connPathStr:
                continue
            targetPrimPath, targetPropName = connPathStr.rsplit(".", 1)
            side, targetPinName = split_direction_hint(targetPropName)
            if side == "input":
                targetIsInput = True
            elif side == "output":
                targetIsInput = False
            else:
                targetIsInput = True
            if isBare:
                isInput = infer_bare_link_direction(targetPropName)
            yield {
                "pin_name": pinName,
                "is_input": isInput,
                "target_prim_path": targetPrimPath,
                "target_pin_name": targetPinName,
                "target_is_input": targetIsInput,
                "fieldname": fieldname,
            }


def scope_fieldnames(scopePrim, scopePathStr, useFieldnames, acceptChild):
    """The ``(inputs, outputs)`` fieldname sets for connections to the scope.

    Each entry is a ``(scopePinName, fieldname)`` pair from a child's
    connection metadata. Only children *acceptChild* takes part, the
    same selection the loader that owns this scope uses.
    """
    fieldnameInputs = set()
    fieldnameOutputs = set()
    for child in scopePrim.GetChildren():
        if not acceptChild(child):
            continue
        for conn in scope_child_connections(child, useFieldnames):
            if conn["target_prim_path"] != scopePathStr:
                continue
            fieldname = conn["fieldname"]
            if useFieldnames and fieldname:
                if conn["target_is_input"]:
                    fieldnameInputs.add((conn["target_pin_name"], fieldname))
                else:
                    fieldnameOutputs.add((conn["target_pin_name"], fieldname))
    return (fieldnameInputs, fieldnameOutputs)


def endpoint_specs(inputs, outputs, fieldnameInputs, fieldnameOutputs,
                   useFieldnames):
    """Ordered ``(inputs, outputs)`` synthetic endpoint lists.

    Each entry is ``(suffix, displayName)``; the node id is the scope
    path plus the suffix. A side with fieldnames emits one endpoint per
    ``(scopePin, fieldname)`` pair instead of its plain pins.
    """
    if useFieldnames and fieldnameInputs:
        inputSpecs = [
            ("#input:%s:%s" % (pinName, fieldname), fieldname)
            for pinName, fieldname in sorted(
                fieldnameInputs, key=lambda x: (x[0], x[1])
            )
        ]
    else:
        inputSpecs = [("#input:" + pinName, pinName) for pinName in inputs]
    if useFieldnames and fieldnameOutputs:
        outputSpecs = [
            ("#output:%s:%s" % (pinName, fieldname), fieldname)
            for pinName, fieldname in sorted(
                fieldnameOutputs, key=lambda x: (x[0], x[1])
            )
        ]
    else:
        outputSpecs = [
            ("#output:" + pinName, pinName) for pinName in outputs
        ]
    return (inputSpecs, outputSpecs)


def scope_endpoints(scopePrim, scopePathStr, useFieldnames, acceptChild):
    """The full ``(inputs, outputs)`` endpoint derivation for one scope."""
    inputs, outputs = scope_pin_names(scopePrim)
    fieldnameInputs, fieldnameOutputs = scope_fieldnames(
        scopePrim, scopePathStr, useFieldnames, acceptChild
    )
    return endpoint_specs(
        inputs, outputs, fieldnameInputs, fieldnameOutputs, useFieldnames
    )
