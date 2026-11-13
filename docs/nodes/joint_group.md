# ![Joint Group](../../icons/concept.png) Joint Group

*API schema that makes a prim (conventionally a Scope) a group of alternative chains among a solver's joints: exactly one member chain is active at a time.*

| | |
|---|---|
| **Node type** | `RigExecJointGroupAPI` |
| **Example** | [joint_group.usda](../examples/joint_group.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Apply `RigExecJointGroupAPI` to a prim, conventionally a `Scope`, and list
it on the `rigExec:jointGroups` of an [FK Chain](fk_chain.md) or a
[Skeleton 2D](skeleton_2d.md). The group names the first joint of each
alternative chain among that solver's joints. Exactly one member is active;
the joints under the inactive members (the member and its descendants that
the solver lists) publish their rest frames. Animating `rigExec:activeMember`
swaps the visible chain.

Membership is the schema's built-in `UsdCollectionAPI` instance `members`: the
joints of `collection:members:includes` in list order, less any joint in
`collection:members:excludes`.

Alternative chains among a solver's joints, applied to a prim
(by convention a Scope) that a RigExecFkChain or RigExecSkeleton2d lists
in rigExec:jointGroups. Each member is the first joint of one
alternative chain. Exactly one member is active; the joints under an
inactive member (the member and its descendants that the solver lists)
publish their rest frames, composed through the solver's start frame;
a 2D skeleton's inactive bones neither solve nor join lock or target
chains. A joint under an active member stays active where groups nest.
A disabled group keeps every member active.

Membership is the built-in collection "members": the joints of
collection:members:includes in list order, less any joint in
collection:members:excludes. The collection is explicit-only; a group
whose expansion rule is not explicitOnly, or that authors a membership
expression, is refused at compile, as is a member that is not a joint
of the solver or a joint that is a member of two groups.

## How it works

The group's membership is structural and compiled once. The collection is
explicit-only: its `expansionRule` defaults to `explicitOnly`, and a group that
changes it or authors a `membershipExpression` is refused when the rig compiles,
the same way in every evaluator, as is a member that is not a joint of the
solver and a joint that is a member of two groups. The active index arrives per
evaluation, held between keys so the active member steps at its keys; it rounds
to the nearest member and an out-of-range value clamps. A group with
`rigExec:enabled` false keeps every member active. Where groups nest, a joint
under any active member stays active.

An FK chain publishes an inactive joint's output rest carried by the start
frame's rest-to-pose delta, the frame the chain gives it when no control moves;
the chain still composes later elements from every control. A 2D skeleton
publishes an inactive bone's rest frame, and the bone neither solves nor joins
lock or target chains. The same operation graph runs in native, frozen and
`.rigexec` evaluation.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `collection:members:includes` | Ordered first joints of the alternative chains. | yes |
| `collection:members:excludes` | Joints dropped from the includes. | no |

## Parameters

### Node parameters

#### `collection:members:expansionRule`

*Type:* `uniform token`. *Default:* `"explicitOnly"`.

Valid values: `explicitOnly`, `expandPrims`, `expandPrimsAndProperties`.

Members are listed explicitly; descendants of an included
joint are never members themselves.

#### `rigExec:enabled`

*Type:* `uniform bool`. *Default:* `true`.

A disabled group keeps every member active.

#### `rigExec:activeMember`

*Type:* `double`. *Default:* `0`.

Index into the members (rounded; out of range clamps to the
nearest member). Authored with held knots, so the active member
steps at its keys.

## Example

A wrist carries two alternative fingers, an open one and a curled one with
a tip, posed by one FK chain hanging from the arm. A group lists both fingers,
and `rigExec:activeMember` is keyed 0 then 1 with held knots: the open finger
poses until the key and the curled finger afterward, while the other finger
sits at its rest and moves with the arm. On a 2D skeleton, two hand drawings
switch the same way.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\joint_group.usda
```

## Tips

- List the group on the solver's `rigExec:jointGroups`; a group no solver lists has no effect.
- Bind each chain's geometry to its own joints so the inactive drawing is not deformed.
- A member's chain is the joints under it: nest each alternative's joints under its member.

## See also

- [FK Chain](fk_chain.md)
- [Skeleton 2D](skeleton_2d.md)
- [Bone 2D](bone_2d.md)

---

[UsdRig](../index.md)
