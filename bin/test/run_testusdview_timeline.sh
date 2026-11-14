#!/bin/bash
# bin/test/run_testusdview_timeline.sh -- headless end-to-end test of the
# timeline's frame range fields (tests/testUsdviewTimeline.py) on
# examples/ArmShotAnim.usda.
#
# Types into the start and end fields and checks usdview's slider, range
# fields and current frame through undo and outside edits. Prints
# RIGEXEC_TIMELINE_OK.
#
# Usage: bin/test/run_testusdview_timeline.sh [rendererDisplayName]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/ArmShotAnim.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewTimeline.py" \
     "$@" "$STAGE"
