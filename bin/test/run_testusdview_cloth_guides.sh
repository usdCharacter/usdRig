#!/bin/bash
# bin/test/run_testusdview_cloth_guides.sh -- verify attachment guide rods on
# the cloth materials example (tests/testUsdviewClothGuides.py).
# The POSIX twin of run_testusdview_cloth_guides.bat.
# Requires a current rigExecImaging build.
#
# Usage: bin/test/run_testusdview_cloth_guides.sh [rendererDisplayName]   (e.g. Embree)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

STAGE="$RIG/docs/examples/cloth_materials.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the .bat
# helper. Prepending to the positional parameters rather than collecting an
# array keeps this working under bash 3.2, where expanding an empty array with
# set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewClothGuides.py" \
     "$@" "$STAGE"
