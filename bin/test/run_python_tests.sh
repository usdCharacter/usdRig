#!/bin/bash
# bin/test/run_python_tests.sh -- headless Qt-free tests for the usdview plugin
# modules (undo stack, gizmo math, gizmo screen helpers, the layer-opinion
# and composition-arc authoring models).
#
# Usage: bin/test/run_python_tests.sh [test_name ...]   (default: all)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"
rigexec_require_python

# The native evaluator binding is optional (built in build-python, not
# build); test_gizmo_math compares against it when importable.
# Prefer the selected tree's binding: _env.sh pairs its Python modules
# with its native libraries. Keep the legacy build-python fallback only
# for the default tree, never for an explicitly selected build.
if [ -z "${RIGEXEC_BUILD_DIR:-}" ] &&
   ! ls "$_RIGEXEC_BUILD"/python/_rigexec*.so >/dev/null 2>&1; then
    export PYTHONPATH="$RIG/build-python/python:$PYTHONPATH"
fi
SCHEMA="$_RIGEXEC_BUILD/usd/rigExecSchema/resources"

TESTS=("$@")
if [ ${#TESTS[@]} -eq 0 ]; then
    TESTS=(test_rigexec_undo test_gizmo_math test_gizmo_screen
           test_gizmo_settings test_gizmo_drag test_gizmo_snap
           test_viewcube_math test_rigexec_stage_edits test_graph_model
           test_graph_screen test_layer_opinions_model
           test_path_spelling test_composition_arcs_model
           test_gizmo_preview
           test_touchpose_model test_gizmo_marquee
           test_picker_scene test_picker_hover_model test_marking_menu_model test_marking_menu_ui test_pip_picker test_picker_layout
           test_imaging_handle test_session_state test_env_build_dir
           test_editor_extension test_timeline_model)
fi

# Only the tests listed here read argv[1], to Plug-register the generated
# schema. The rest never touch sys.argv -- they need neither a schema nor
# a build -- so handing them $SCHEMA would tell a reader otherwise.
SCHEMA_TESTS=" test_rigexec_undo test_gizmo_math test_rigexec_stage_edits test_graph_model test_picker_scene test_pip_picker test_picker_layout "
for t in "${TESTS[@]}"; do
    echo "== $t"
    case "$SCHEMA_TESTS" in
        *" $t "*) "$PY" "$RIG/tests/python/$t.py" "$SCHEMA" ;;
        *)        "$PY" "$RIG/tests/python/$t.py" ;;
    esac
done
