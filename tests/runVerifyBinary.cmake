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
# half. An exit-0 invocation that printed no ledger bytes still fails the
# gate; every failure report names the exit code, both ledger byte counts, and
# whether the binary the bake was asked to write is on disk. Required -D
# arguments: BAKE, CONFORMANCE, POSE, STAGE, FRAMES, OUT. Optional: TIME (the
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
# Describe one finished invocation for a failure report: the bytes each of its
# ledgers holds, and whether the binary it was asked to write is on disk.
function(_ledger_evidence _out_file _err_file _product)
    if (EXISTS "${_out_file}")
        file(SIZE "${_out_file}" _out_bytes)
        set(_out_text "${_out_bytes} byte(s)")
    else()
        set(_out_text "no file")
    endif()
    if (EXISTS "${_err_file}")
        file(SIZE "${_err_file}" _err_bytes)
        set(_err_text "${_err_bytes} byte(s)")
    else()
        set(_err_text "no file")
    endif()
    if (EXISTS "${_product}")
        file(SIZE "${_product}" _product_bytes)
        set(_product_text "${_product} is ${_product_bytes} byte(s)")
    else()
        set(_product_text "no binary was written")
    endif()
    set(_ledger_out_text "${_out_text}" PARENT_SCOPE)
    set(_ledger_err_text "${_err_text}" PARENT_SCOPE)
    set(_ledger_product "${_product_text}" PARENT_SCOPE)
endfunction()
# Drop any earlier bake of this fixture so the evidence below describes this run.
file(REMOVE "${OUT}")
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
    _ledger_evidence("${OUT}.bake.stdout.log" "${OUT}.bake.stderr.log"
                     "${OUT}")
    message(FATAL_ERROR
        "the bake printed no static report (exit ${_bake_rc}; stdout "
        "${_ledger_out_text}, stderr ${_ledger_err_text}; ${_ledger_product}):"
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
    _ledger_evidence("${OUT}.conformance.stdout.log"
                     "${OUT}.conformance.stderr.log" "${OUT}")
    message(FATAL_ERROR
        "binary conformance printed no successful ledger "
        "(exit ${_conformance_rc}; stdout ${_ledger_out_text}, "
        "stderr ${_ledger_err_text}; ${_ledger_product}):"
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
    _ledger_evidence("${OUT}.pose.stdout.log" "${OUT}.pose.stderr.log"
                     "${OUT}")
    message(FATAL_ERROR "verify-binary printed no ledger matching "
                        "${_frame_count} frame(s) (exit ${_pose_rc}; stdout "
                        "${_ledger_out_text}, stderr ${_ledger_err_text}; "
                        "${_ledger_product}):"
                        "\ncommand: ${_pose_command}"
                        "\nstdout: ${_pose_out}"
                        "\nstderr: ${_pose_err}")
endif()
if (NOT _pose_out MATCHES
        "verify-binary: ${_drag_values} of ${_drag_values} drag\\(s\\) match")
    _ledger_evidence("${OUT}.pose.stdout.log" "${OUT}.pose.stderr.log"
                     "${OUT}")
    message(FATAL_ERROR "verify-binary printed no ledger matching "
                        "${_drag_values} drag(s) (exit ${_pose_rc}; stdout "
                        "${_ledger_out_text}, stderr ${_ledger_err_text}; "
                        "${_ledger_product}):"
                        "\ncommand: ${_pose_command}"
                        "\nstdout: ${_pose_out}"
                        "\nstderr: ${_pose_err}")
endif()
