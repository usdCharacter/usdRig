"""Skin binding authoring: skin-mover rows generated from volume weights."""

from . import _native


def generate_skin_weights(stage, skin_mover, point_indices=None):
    """Author a skin mover's rows from the volume weights of its influences.

    ``skin_mover`` is the path (string, ``Sdf.Path`` or handle) of a
    ``RigExecSkinMover``. Its ``rigExec:moves`` target's points are read at
    Default and carried into the rig's asset space by their setup transform.
    Every direct ``RigExecVolumeWeight`` child (``RigExecSphereWeight``,
    ``RigExecPlaneWeight``, ``RigExecCurveWeight``) of an influence in
    ``rigExec:influences`` is measured at its pose-free
    rest placement through the evaluator's packet builder: falloff band,
    range policy, ``inputs:scaleX/Y/Z`` and the shape's own settings (a
    sphere's Pos/Neg scales, a plane's axis, bounds and extents, a curve's
    one ``rigExec:curve`` points source read at Default and carried into
    asset space like the skin's points) as the evaluator reads them. Several volumes of one influence sum, clamped to
    1, and an influence without one contributes 0. Each row is then
    normalised over the influences, and ``rigExec:jointIndices`` index that
    list. A row whose fields sum to zero stays all zero, so the point holds
    its rest under ``classicLinear``'s weight shortfall. A row lists its
    non-zero influences in influence order, padded with index 0 and weight 0
    to ``rigExec:elementSize``, the widest row (at least 1).

    With ``point_indices`` (an iterable of point numbers) only those rows are
    replaced: the other rows keep their authored non-zero slots, an
    unauthored layout reading as zero rows, and ``elementSize`` is recomputed
    over the merged rows. Tools author rigid rows (one weight of 1) and
    released rows (all zero) themselves and regenerate the rest this way.

    Authors only ``rigExec:jointIndices``, ``rigExec:jointWeights`` and
    ``rigExec:elementSize``, at the stage's edit target; never evaluates a
    pose. Returns a dict: ``rows_written`` (rows replaced),
    ``element_size`` and ``influences_without_volumes`` (paths, in list
    order). Raises ``ValueError`` for a malformed mover, influence, volume
    weight or point index and ``RuntimeError`` when a placement or field cannot be
    resolved; nothing is authored then.
    """
    return _native.generate_skin_weights(stage, skin_mover, point_indices)


__all__ = ["generate_skin_weights"]
