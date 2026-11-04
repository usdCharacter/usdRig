#!/bin/bash
# Print, and with --run execute, the CTest targets a source change affects.
#
# The direct set is the inner loop. The transitive set is what to run
# before pushing. --run configures nothing: it builds the direct
# executables in the tree bin/_env.sh selects (RIGEXEC_BUILD_DIR, default
# ./build) and runs the selector's ctest line there.
#
# Usage:
#   bin/test_changed.sh libs/rigExecMath/solvers.cpp
#   bin/test_changed.sh --diff origin/rigexec-format-v4
#   bin/test_changed.sh --run libs/rigExecBinary/format.cpp
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN=0
ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--run" ]; then
        RUN=1
    else
        ARGS+=("$arg")
    fi
done

if [ "${#ARGS[@]}" -eq 0 ]; then
    echo "usage: bin/test_changed.sh [--run] [--diff REV] [paths...]" >&2
    exit 2
fi

python3 "$ROOT/tools/select_affected_tests.py" "${ARGS[@]}"

if [ "$RUN" -eq 0 ]; then
    exit 0
fi

mapfile -t lines < <(python3 "$ROOT/tools/select_affected_tests.py" "${ARGS[@]}")
build_line=""
ctest_line=""
for line in "${lines[@]}"; do
    case "$line" in
        "  cmake --build "*) build_line="${line#  }" ;;
        "  ctest "*) ctest_line="${line#  }" ;;
    esac
done

if [ -z "$build_line" ] || [ -z "$ctest_line" ]; then
    echo "test_changed: selector produced no build/ctest command" >&2
    exit 1
fi

# The tree and environment bin/_env.sh selects (RIGEXEC_BUILD_DIR, else build/).
. "$ROOT/bin/_env.sh"
cd "$ROOT"
targets="${build_line#cmake --build build --target }"
ctest_args="${ctest_line#ctest --test-dir build }"
# shellcheck disable=SC2086
eval "cmake --build \"\$_RIGEXEC_BUILD\" --target $targets"
# shellcheck disable=SC2086
eval "ctest --test-dir \"\$_RIGEXEC_BUILD\" $ctest_args"
