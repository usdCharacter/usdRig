# Method references

These public sources describe dependencies and numerical techniques used in
RigExec. They are references for implementation and review, not claims of
affiliation, endorsement, or ownership of the underlying techniques.

| Area | Public source | RigExec code |
|---|---|---|
| OpenExec computation registration and reference requests | [OpenExec introduction](https://openusd.org/release/intro_to_openexec.html) and [system design](https://openusd.org/dev/api/page__execution__system__design.html) | `libs/rigExec/computations.cpp`, `libs/rigExec/moverKernels.cpp`, `libs/rigExec/types.cpp`, `libs/rigExec/tapSet.cpp`; used only by the [reference check](concepts/baked-vs-dynamic.md#openexec-reference-checks) and tests. Production evaluation uses the shared operation graph in `libs/rigExecGraph`. |
| Scene-index publication | [Hydra getting started](https://openusd.org/dev/api/_page__hydra__getting__started__guide.html) | `libs/rigExecImaging` |
| Dual-quaternion skinning | Kavan et al., *Geometric Skinning with Approximate Dual Quaternion Blending* (2008), [author project page](https://users.cs.utah.edu/~ladislav/dq/index.html) | `libs/rigExecMath/dualQuat.cpp` |
| Delta Mush | Mancewicz, Derksen, and Wilson, *Delta Mush: Smoothing Deformations While Preserving Detail* (2014), [publisher entry](https://dl.acm.org/doi/10.1145/2614106.2614144) | `libs/rigExecMath/deltaMushKernel.h` |
| Position projection concepts | Müller, Heidelberger, Hennix, and Ratcliff, *Position Based Dynamics* (2007), [author-hosted paper](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf), [DOI](https://doi.org/10.1016/j.jvcir.2007.01.005) | `libs/rigExecMath/wrinkleKernel.h` |
| Wrinkle reference lengths and bounded offsets | Müller and Chentanez, *Wrinkle Meshes* (2010), [author-hosted paper](https://matthias-research.github.io/pages/publications/wrinkleMeshes.pdf), [DOI](https://doi.org/10.2312/SCA/SCA10/085-091) | `libs/rigExecMath/wrinkleKernel.h`; [method and limitations](concepts/wrinkle-deformation.md) |
| Lattice (free-form) deformation | Sederberg and Parry, *Free-Form Deformation of Solid Geometric Models*, ACM SIGGRAPH Computer Graphics 20(4):151–160 (1986), [DOI](https://doi.org/10.1145/15886.15903) | `libs/rigExecMath/latticeKernel.h` |
| Ray-triangle intersection | Möller and Trumbore, *Fast, Minimum Storage Ray-Triangle Intersection*, Journal of Graphics Tools 2(1):21–28 (1997), [publication entry](https://en-cg-web.coecis.cornell.edu/pubs/1997/MT97.html) | `libs/rigExecMath/surfaceProjectorKernel.h` (surface projector, `RigExecRaycastSurface`) |
| Path curve bases | [UsdGeomBasisCurves](https://openusd.org/release/api/class_usd_geom_basis_curves.html) (linear; cubic bezier, bspline, catmullRom; nonperiodic and pinned wrap); Catmull and Rom, *A Class of Local Interpolating Splines*, in Barnhill and Riesenfeld (eds.), *Computer Aided Geometric Design* (1974), [DOI](https://doi.org/10.1016/B978-0-12-079050-0.50020-5) | `libs/rigExecMath/pathCurveKernel.h` (float math `path` operation) |
| Arc length by Gauss–Legendre quadrature | NIST *Digital Library of Mathematical Functions*, [§3.5(v) Gauss quadrature](https://dlmf.nist.gov/3.5#v) | `libs/rigExecMath/pathCurveKernel.h` |
| NURBS patch evaluation | Piegl and Tiller, *The NURBS Book* (1997), [DOI](https://doi.org/10.1007/978-3-642-59223-2) | `libs/rigExecMath/geometryKernels.h` (`RigExecNurbsPatch`), `libs/rigExecMath/geometryKernels.cpp` (`RigExecNurbsPatch::EvaluateNormalized`, `RigExecApplyPatchMover`); [Patch Mover](nodes/patch_mover.md) |
| RigLogic radial-basis solvers, machine-learned behavior and animated maps | Epic Games, [OpenRigLogic](https://github.com/EpicGames/OpenRigLogic) (MIT), RigLogic 13.5.0 at commit [`9ca65ac`](https://github.com/EpicGames/OpenRigLogic/commit/9ca65ac08b64a1ea5f0d79466e64706b3ac524ba): `src/riglogic/rbf/cpu`, `src/riglogic/ml/cpu`, `src/riglogic/conditionaltable` and `src/riglogic/animatedmaps`; [RigLogic whitepaper](https://cdn2.unrealengine.com/rig-logic-whitepaper-v2-5c9f23f7e210.pdf). What was ported and what was added is listed in [RigLogic behaviours](concepts/riglogic.md#provenance) | `libs/rigExecMath/rbfMethodKernel.h` (RigExecPoseInterpolator `rigExec:solver` additive and interpolative), `libs/rigExecMath/mlpKernel.h` (RigExecMlMover), `libs/rigExecMath/conditionalTableKernel.h` (RigExecAnimatedMaps); reference values from RigLogic itself in `tests/fixtures/rigLogicReference.inc` |
| Swing-twist decomposition | Dobrowolski, *Swing-twist decomposition in Clifford algebra* (2015), [arXiv:1506.05481](https://arxiv.org/abs/1506.05481) | `libs/rigExec/solverKernels.cpp` (space switch rotation filters), `libs/rigExecMath/rbf.cpp` |

Implementation details and limitations are documented with each operator.
In particular, referencing Delta Mush does not claim implementation of the
distinct Direct Delta Mush method. Third-party source and asset provenance
is tracked separately in [third-party notices](../THIRD_PARTY_NOTICES.md).
The wrinkle mover independently adapts position projection, reference lengths,
and bounded offsets using a deterministic fold guide and affine edge targets.
Its guide and target equations are documented in the
[method guide](concepts/wrinkle-deformation.md). It does not reproduce the
papers' nonlinear distance projection, the separate refined mesh, or the
persistent offset state of *Wrinkle Meshes*. Its public behavior references
do not imply use of proprietary implementation code.

## Persistent surface bindings

`RigExecSurfaceBindingMover` evaluates generalized barycentric positions and
offsets in an orthonormal tangent/bitangent/normal frame. Polygon normals use
Newell's area vector; optional smooth normals interpolate normalized,
area-weighted vertex normals. The Blender
[Surface Deform evaluator](https://github.com/blender/blender/blob/main/source/blender/modifiers/intern/MOD_surfacedeform.cc)
is the reference for weighted polygon bindings and normal-offset import.
Limit position and tangent derivatives are supplied as factored control-point
stencils, as described by the
[OpenSubdiv limit stencil factory](https://graphics.pixar.com/opensubdiv/docs/far_overview.html).
The runtime consumes these immutable attributes without tessellation or rebinding.

## Extended deformation settings

The independent Blender behavior fixtures cover
[Corrective Smooth](https://github.com/blender/blender/blob/main/source/blender/modifiers/intern/MOD_correctivesmooth.cc),
[nearest-surface shrinkwrap](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/shrinkwrap.cc),
and [lattice deformation](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/lattice_deform.cc).
The shared, stage-free kernels implement the selected mathematical behavior;
Blender-generated fixture coordinates test the results independently.

Newly authored surface, Delta Mush, or lattice settings require updated runtime
consumers. Binary exports mark these semantics using wire revision aliases
19 (SurfaceProject), 20 (DeltaMush), and 21 (Lattice). Current readers normalize
these to the existing operations; older readers reject the unsupported opcode.
Exports using only legacy settings retain their original operation codes.
USD assets must likewise be opened with the updated schema and runtime.


## Affine frame computation contracts

The stage-free `libs/rigExecMath/affineFrameKernels.cpp` implements joint
inheritance, affine frame copying/mapping, weighted rest-to-pose parenting,
constraint space conversion, tracking, stretch and location remapping.
Production evaluation runs these kernels as provider-program operations
(`libs/rigExecGraph/providerProgram.cpp`). The OpenExec expressions in
`libs/rigExec/affineFrameComputations.cpp` read declared inputs and call the
same kernels, but nothing requests them. Source behavior references
include Blender's [constraint evaluator](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/constraint.cc),
[joint inheritance](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/armature.cc),
and [rotation channel extraction](https://github.com/blender/blender/blob/main/source/blender/blenlib/intern/math_rotation_c.cc).
The shared schemas define the runtime contracts independently of the file
format that authored them. Independent source fixtures cover local-space
location remapping, inherited nonuniform transforms and sequential skinning.
