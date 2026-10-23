#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Capture and replay node activation opinions at their authored sites."""

from dataclasses import dataclass

from pxr import Sdf


@dataclass(frozen=True)
class ActivationEdit:
    layer: object
    path: object
    before: object
    existing_paths: tuple


def capture_activation(prim):
    target = prim.GetStage().GetEditTarget()
    layer = target.GetLayer()
    path = target.MapToSpecPath(prim.GetPath())
    if not layer or path == Sdf.Path.emptyPath:
        raise RuntimeError("Node cannot be mapped into the current edit target")
    spec = layer.GetPrimAtPath(path)
    before = spec.GetInfo("active") if spec and spec.HasInfo("active") else None
    existing = tuple(prefix for prefix in path.GetPrefixes()
                     if layer.GetPrimAtPath(prefix))
    return ActivationEdit(layer, path, before, existing)


def replay_activation(edits, restore):
    """Restore prior opinions or author deletion, without changing edit targets."""
    with Sdf.ChangeBlock():
        for edit in edits:
            value = edit.before if restore else False
            spec = edit.layer.GetPrimAtPath(edit.path)
            if value is not None:
                if not spec:
                    spec = Sdf.CreatePrimInLayer(edit.layer, edit.path)
                spec.SetInfo("active", value)
            elif spec:
                spec.ClearInfo("active")
                for path in reversed(edit.path.GetPrefixes()):
                    # Preserve pre-existing specs and the variant's container.
                    if path in edit.existing_paths or path.IsPrimVariantSelectionPath():
                        break
                    ancestor = edit.layer.GetPrimAtPath(path)
                    if not ancestor or not ancestor.IsInert():
                        break
                    parent = ancestor.nameParent
                    if parent is None:
                        del edit.layer.rootPrims[ancestor.name]
                    else:
                        del parent.nameChildren[ancestor.name]
