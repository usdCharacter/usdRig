#!/bin/bash
# Exercise Execution Stack on a cache-owned viewer stage: compiled order,
# compiler diagnostics, registration and filters.
# Usage: bin/test/run_testusdview_execstack.sh [rendererDisplayName]
# Set RIGEXEC_EXECSTACK_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/biped_rigged.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewExecStack.py" \
     "$@" "$STAGE"
