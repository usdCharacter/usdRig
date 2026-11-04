#!/bin/bash
# bin/test/run_testusdview_rig_parity.sh -- 2D rig evaluation parity end to end
# (tests/testUsdviewRigParity.py).
#
# Needs a logged-in GUI session (testusdview, native Cocoa). Screenshots go
# to $RIGEXEC_RIGPARITY_SHOTS (default: $TMPDIR/rigparity-shots,
# rigparity-*.png); the JSON report to $RIGEXEC_RIGPARITY_REPORT (default:
# $TMPDIR/rigparity.json). Prints RIGEXEC_RIGPARITY_OK.
#
# The build tree is the _env.sh default ($_RIGEXEC_BUILD, override with
# RIGEXEC_BUILD_DIR); rigexec_build refreshes it first.
#
# Exits non-zero on a failed check, a Tf error or Python traceback in the
# session's output, or a screenshot the session did not write.
#
# Usage: bin/test/run_testusdview_rig_parity.sh [rendererDisplayName]
set -euo pipefail

. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

WORK="$(mktemp -d "${TMPDIR:-/tmp}/rigexec-rigparity.XXXXXX")"
STAGE="$WORK/rigparity_session.usda"
LOG="$WORK/testusdview.log"
export RIGEXEC_RIGPARITY_REPORT="${RIGEXEC_RIGPARITY_REPORT:-${TMPDIR:-/tmp}/rigparity.json}"
export RIGEXEC_RIGPARITY_SHOTS="${RIGEXEC_RIGPARITY_SHOTS:-${TMPDIR:-/tmp}/rigparity-shots}"
mkdir -p "$RIGEXEC_RIGPARITY_SHOTS"

# The session stage: frame 0 (the setup frame) is reachable; the bone layer
# is a plain Xform until the script makes it one; the art is a linear curve
# set in the layer's own space (identity transforms): the two
# sides of a leg from the hip down and around the foot.
cat > "$STAGE" <<'USDA'
#usda 1.0
(
    defaultPrim = "Doc"
    endTimeCode = 24
    framesPerSecond = 24
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "Doc"
{
    def Xform "BoneLayer"
    {
        def BasisCurves "Art"
        {
            uniform token type = "linear"
            uniform token wrap = "nonperiodic"
            int[] curveVertexCounts = [21, 20, 9]
            point3f[] points = [
                (-0.06, 0.7, 0), (-0.06, 0.65, 0), (-0.06, 0.6, 0), (-0.06, 0.55, 0),
                (-0.06, 0.5, 0), (-0.06, 0.45, 0), (-0.06, 0.4, 0), (-0.06, 0.35, 0),
                (-0.06, 0.3, 0), (-0.06, 0.25, 0), (-0.06, 0.2, 0), (-0.06, 0.15, 0),
                (-0.06, 0.1, 0), (-0.06, 0.05, 0), (-0.06, 0, 0), (-0.06, -0.05, 0),
                (-0.06, -0.1, 0), (-0.06, -0.15, 0), (-0.06, -0.2, 0), (-0.06, -0.25, 0),
                (-0.06, -0.36, 0),
                (0.06, 0.7, 0), (0.06, 0.65, 0), (0.06, 0.6, 0), (0.06, 0.55, 0),
                (0.06, 0.5, 0), (0.06, 0.45, 0), (0.06, 0.4, 0), (0.06, 0.35, 0),
                (0.06, 0.3, 0), (0.06, 0.25, 0), (0.06, 0.2, 0), (0.06, 0.15, 0),
                (0.06, 0.1, 0), (0.06, 0.05, 0), (0.06, 0, 0), (0.06, -0.05, 0),
                (0.06, -0.1, 0), (0.06, -0.15, 0), (0.06, -0.2, 0), (0.06, -0.24, 0),
                (0.06, -0.24, 0), (0.1, -0.24, 0), (0.15, -0.24, 0), (0.22, -0.24, 0),
                (0.24, -0.3, 0), (0.22, -0.36, 0), (0.15, -0.36, 0), (0.05, -0.36, 0),
                (-0.06, -0.36, 0)]
            float[] widths = [0.025] (
                interpolation = "constant"
            )
            color3f[] primvars:displayColor = [(0.05, 0.2, 0.75)]
        }
    }
}
USDA

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

STARTED="$WORK/started"
: > "$STARTED"
STATUS=0
"$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewRigParity.py" "$@" \
    "$STAGE" > "$LOG" 2>&1 || STATUS=$?
cat "$LOG"

if [ "$STATUS" -ne 0 ]; then
    echo "FAILED: testusdview exited $STATUS (report: $RIGEXEC_RIGPARITY_REPORT)"
fi
if grep -q "Traceback" "$LOG"; then
    echo "FAILED: a Python traceback in the session output"
    STATUS=1
fi
if grep -Eq "(Coding|Runtime|Fatal) Error|Failed verification" "$LOG"; then
    echo "FAILED: a Tf error in the session output:"
    grep -E "(Coding|Runtime|Fatal) Error|Failed verification" "$LOG" | head -5
    STATUS=1
fi
if ! grep -q "RIGEXEC_RIGPARITY_OK" "$LOG"; then
    echo "FAILED: the session did not print RIGEXEC_RIGPARITY_OK"
    STATUS=1
fi
for SHOT in offset target reparent groups control; do
    FILE="$RIGEXEC_RIGPARITY_SHOTS/rigparity-$SHOT.png"
    if [ ! -s "$FILE" ] || [ ! "$FILE" -nt "$STARTED" ]; then
        echo "FAILED: screenshot $FILE was not written by this run"
        STATUS=1
    fi
done
rm -rf "$WORK"
exit $STATUS
