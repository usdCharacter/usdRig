# ![Bone 2D Group](../../icons/concept.png) Bone 2D Group

*API schema that makes a prim (conventionally a Scope) an alternative-chain bone group: exactly one member chain is active at a time.*

| | |
|---|---|
| **Node type** | `RigExecBone2dGroupAPI` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Apply `RigExecBone2dGroupAPI` to a prim, conventionally a `Scope`, and list
it on a [Skeleton 2D](skeleton_2d.md)'s `rigExec:boneGroups`. The group names the
first bone of each alternative chain in that skeleton. Exactly one member is
active; the bones under the inactive members publish their rest frames and
neither solve nor join lock or target chains. Animating `rigExec:activeBone`
swaps the visible chain.

Membership is the schema's built-in `UsdCollectionAPI` instance `members`: the
bones of `collection:members:includes` in list order, less any bone in
`collection:members:excludes`.

One alternative-chain bone group, applied to a prim (by
convention a Scope) that a RigExecSkeleton2d lists in
rigExec:boneGroups. The group holds alternative chains in one skeleton;
its members are the first bone of each chain. Exactly one member is
active; bones under inactive members publish their rest frames and
neither solve nor join chains. A disabled group keeps every member
active.

Membership is the built-in collection "members": the bones of
collection:members:includes in list order, less any bone in
collection:members:excludes. The collection is explicit-only; a group
whose expansion rule is not explicitOnly, or that authors a membership
expression, is refused at compile.

## How it works

The group's membership is structural and compiled once. The collection is
explicit-only: its `expansionRule` defaults to `explicitOnly`, and a group that
changes it or authors a `membershipExpression` is refused when the rig compiles,
the same way in every evaluator. The active index arrives per evaluation, held
between keys so the active member steps at its keys; an out-of-range value
clamps to the nearest member. A group with `rigExec:enabled` false keeps every
member active.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `collection:members:includes` | Ordered first bones of the alternative chains. | yes |
| `collection:members:excludes` | Bones dropped from the includes. | no |

## Parameters

### Node parameters

#### `collection:members:expansionRule`

*Type:* `uniform token`. *Default:* `"explicitOnly"`.

Valid values: `explicitOnly`, `expandPrims`, `expandPrimsAndProperties`.

Members are listed explicitly; descendants of an included
bone are never members themselves.

#### `rigExec:enabled`

*Type:* `uniform bool`. *Default:* `true`.

A disabled group keeps every member active.

#### `rigExec:activeBone`

*Type:* `double`. *Default:* `0`.

Index into the members (rounded; out of range clamps to the
nearest member). Authored with held knots, so the active member
steps at its keys.

## Example

Two hand drawings, an open hand and a fist, hang as two chains from the
wrist. A group lists both chain roots, and `rigExec:activeBone` is keyed 0 then
1: the open hand's bones solve until the key and the fist's bones afterward,
while the other chain sits at its rest.

## Tips

- List the group on the skeleton's `rigExec:boneGroups`; a group the skeleton does not list has no effect.
- Bind each chain's geometry to its own bones so the inactive drawing is not deformed.
- A member's chain is the bones under it: nest each alternative drawing's bones under its member.

## See also

- [Skeleton 2D](skeleton_2d.md)
- [Bone 2D](bone_2d.md)

---

[UsdRig](../index.md)
