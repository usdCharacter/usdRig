# runVerifyBinary.cmake: one fixture's bake-then-verify gate.
#
# Bakes STAGE at TIME with the rigExecBake CLI, then checks an independent
# fresh bake for byte determinism and complete native owner/default/source
# conformance before gating the zero-USD
# runtime against the baked path with rigExecPose --verify-binary: the
# binary's defaults at TIME, every frame of FRAMES (TIME alone when CLASS is
# static) with its Animated inputs sampled from the stage, and two drags of
# each input DRAGS names against the session-layer edit. The bake's static
# report must agree with CLASS: a static stage holds at least one animated
# source in static data, an inputs stage none. Run as a ctest entry (see
# rigexec_add_verify_binary_test in CMakeLists.txt); fails loudly on either
# half.
# Required -D arguments: BAKE, CONFORMANCE, POSE, STAGE, FRAMES, OUT. Optional: TIME (the
# first of FRAMES by default), CLASS (inputs or static; inputs by default),
# DRAGS (comma-separated <prim>.<attr> list).
if (NOT DEFINED BAKE OR NOT DEFINED CONFORMANCE OR NOT DEFINED POSE OR NOT DEFINED STAGE
        OR NOT DEFINED FRAMES OR NOT DEFINED OUT)
    message(FATAL_ERROR "runVerifyBinary.cmake: BAKE, CONFORMANCE, POSE, STAGE, FRAMES "
                        "and OUT are all required")
endif()
string(REPLACE "," ";" _frame_list "${FRAMES}")
if (NOT DEFINED TIME OR TIME STREQUAL "")
    list(GET _frame_list 0 TIME)
endif()
if (NOT DEFINED CLASS OR CLASS STREQUAL "")
    set(CLASS inputs)
endif()
if (CLASS STREQUAL "static")
    set(_verify_frames "${TIME}")
elseif (CLASS STREQUAL "inputs")
    set(_verify_frames "${FRAMES}")
else()
    message(FATAL_ERROR "runVerifyBinary.cmake: CLASS is '${CLASS}', "
                        "expected inputs or static")
endif()
set(_drag_args)
set(_drag_count 0)
if (DEFINED DRAGS AND NOT DRAGS STREQUAL "")
    string(REPLACE "," ";" _drag_list "${DRAGS}")
    foreach(_drag IN LISTS _drag_list)
        list(APPEND _drag_args --drag-input "${_drag}")
        math(EXPR _drag_count "${_drag_count} + 1")
    endforeach()
endif()
# Capture directly to per-fixture files so CI retains every subprocess ledger.
execute_process(
    COMMAND "${BAKE}" "${STAGE}" --time "${TIME}" --report-static
            -o "${OUT}"
    RESULT_VARIABLE _bake_rc
    OUTPUT_FILE "${OUT}.bake.stdout.log"
    ERROR_FILE "${OUT}.bake.stderr.log")
file(READ "${OUT}.bake.stdout.log" _bake_out)
file(READ "${OUT}.bake.stderr.log" _bake_err)
if (NOT _bake_rc EQUAL 0)
    message(FATAL_ERROR
        "bake failed (${_bake_rc}):\n${_bake_out}\n${_bake_err}")
endif()
# The class guard: a class cannot outlive its reason.
if (NOT _bake_out MATCHES "static report: ([0-9]+) animated source")
    message(FATAL_ERROR "the bake printed no static report (exit ${_bake_rc}):"
                        "\nstdout: ${_bake_out}\nstderr: ${_bake_err}")
endif()
set(_static_sources "${CMAKE_MATCH_1}")
if ((CLASS STREQUAL "static" AND _static_sources EQUAL 0) OR
        (CLASS STREQUAL "inputs" AND NOT _static_sources EQUAL 0))
    message(FATAL_ERROR "CLASS is ${CLASS}, but the static report lists "
                        "${_static_sources} animated source(s):\n${_bake_out}")
endif()
message(STATUS "class ${CLASS}: ${_static_sources} animated static "
               "source(s)")
execute_process(
    COMMAND "${CONFORMANCE}" --conformance "${STAGE}" "${TIME}" "${CLASS}" "${OUT}"
    RESULT_VARIABLE _conformance_rc
    OUTPUT_FILE "${OUT}.conformance.stdout.log"
    ERROR_FILE "${OUT}.conformance.stderr.log")
file(READ "${OUT}.conformance.stdout.log" _conformance_out)
file(READ "${OUT}.conformance.stderr.log" _conformance_err)
message(STATUS "${_conformance_out}")
if (NOT _conformance_rc EQUAL 0)
    message(FATAL_ERROR "binary conformance failed (${_conformance_rc}):\n${_conformance_out}\n${_conformance_err}")
endif()
if (NOT _conformance_out MATCHES "binary conformance: .* failures=0")
    message(FATAL_ERROR "binary conformance printed no successful ledger "
                        "(exit ${_conformance_rc}):"
                        "\nstdout: ${_conformance_out}"
                        "\nstderr: ${_conformance_err}")
endif()
set(_pose_command "${POSE}" "${STAGE}" --verify-binary "${OUT}"
                  --frames "${_verify_frames}" ${_drag_args})
execute_process(
    COMMAND ${_pose_command}
    RESULT_VARIABLE _pose_rc
    OUTPUT_FILE "${OUT}.pose.stdout.log"
    ERROR_FILE "${OUT}.pose.stderr.log")
file(READ "${OUT}.pose.stdout.log" _pose_out)
file(READ "${OUT}.pose.stderr.log" _pose_err)
# Always show the ledger: a passing gate with no ledger is a gate that
# cannot be audited.
message(STATUS "${_pose_out}")
if (NOT _pose_rc EQUAL 0)
    message(FATAL_ERROR
        "verify-binary failed (${_pose_rc}):\n${_pose_err}")
endif()
# A run that exits 0 must also have printed both ledgers, each counting
# what it was asked to verify.
string(REPLACE "," ";" _verify_list "${_verify_frames}")
list(LENGTH _verify_list _frame_count)
math(EXPR _drag_values "${_drag_count} * 2")
if (NOT _pose_out MATCHES
        "verify-binary: ${_frame_count} of ${_frame_count} frame\\(s\\) match")
    message(FATAL_ERROR "verify-binary printed no ledger matching "
                        "${_frame_count} frame(s) (exit ${_pose_rc}):"
                        "\ncommand: ${_pose_command}"
                        "\nstderr: ${_pose_err}")
endif()
if (NOT _pose_out MATCHES
        "verify-binary: ${_drag_values} of ${_drag_values} drag\\(s\\) match")
    message(FATAL_ERROR "verify-binary printed no ledger matching "
                        "${_drag_values} drag(s) (exit ${_pose_rc}):"
                        "\ncommand: ${_pose_command}"
                        "\nstderr: ${_pose_err}")
endif()
