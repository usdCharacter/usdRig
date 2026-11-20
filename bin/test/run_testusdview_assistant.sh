#!/bin/bash
# bin/test/run_testusdview_assistant.sh -- headless end-to-end test of the
# AI Assistant (tests/testUsdviewAssistant.py) on
# examples/biped/Biped_anim_ikfk.usda.
#
# Asserts the RigExec menu item opens the settings until the assistant is
# turned on and then docks the panel, and that its tools work on the live
# biped: listing rigs and avars, a keyed pose that moves the evaluated
# elbow and comes back off the shared undo stack, the live evaluator, an
# independent compile, Python in usdview, and a viewport picture reaching
# an MCP client as image content. No provider is called. Prints
# RIGEXEC_ASSISTANT_OK.
#
# Usage: bin/test/run_testusdview_assistant.sh [rendererDisplayName]
# Set RIGEXEC_ASSISTANT_SHOT=/path/prefix to keep the settings and window
# grabs.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/biped/Biped_anim_ikfk.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewAssistant.py" \
     "$@" "$STAGE"
