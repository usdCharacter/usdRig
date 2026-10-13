#!/bin/bash
# bin/test/run_testusdview_marking_menu.sh -- headless end-to-end test of
# the right-click marking menu (tests/testUsdviewMarkingMenu.py) on
# examples/biped/Biped_stack.usda.
#
# Sends real mouse events to the viewport and asserts a right-click on or
# with selected controls opens the marking menu, that flicks and clicks run
# their commands, and that every other right-click stays usdview's. Prints
# MARKING_MENU_OK.
#
# Usage: bin/test/run_testusdview_marking_menu.sh [rendererDisplayName]
# Set RIGEXEC_MARKING_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/biped/Biped_stack.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewMarkingMenu.py" \
     "$@" "$STAGE"
