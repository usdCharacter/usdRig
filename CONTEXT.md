# usdRig domain glossary

This glossary defines the canonical terminology for usdRig. The project and
its evaluation engine have distinct names.

## Language

**usdRig**:
The OpenUSD-based character-rigging project encompassing the RigExec engine,
rig authoring tools, viewer tools, examples, and documentation.
_Avoid_: RigExec when referring to the project as a whole.

**RigExec**:
The character-rig evaluation engine within usdRig, which computes rig results
from controls, solvers, constraints, and ordered deformation operations.
_Avoid_: usdRig when specifically distinguishing the engine from other project tools.

**Rig**:
A character's connected controls, solvers, constraints, and deformation
operations that define how its outputs respond to its inputs.

**Control**:
An authored rig element exposing parameters used to pose or animate a rig.

**Mover**:
An ordered rig operation that revises transforms, properties, or geometry.
_Avoid_: Deformer as a synonym for every mover; movers can revise non-geometric values.

**Rig component**:
A reusable rig definition with connection requirements and character-specific
fitting data supplied by the assembly that uses it.
