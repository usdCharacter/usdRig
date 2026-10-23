# An enabled suite must report missing oracle support without terminating its host.
# The core held-program observer also sees this mode and writes an empty index.
if(NOT DEFINED DRIVER OR NOT DEFINED BINARY_DIR)
    message(FATAL_ERROR "DRIVER and BINARY_DIR are required")
endif()
get_filename_component(_root "${BINARY_DIR}" ABSOLUTE)
string(RANDOM LENGTH 16 ALPHABET 0123456789abcdef _suffix)
set(_directory "${_root}/oracle-missing-${_suffix}")
cmake_path(IS_PREFIX _root "${_directory}" NORMALIZE _inside)
if(NOT _inside OR EXISTS "${_directory}")
    message(FATAL_ERROR "Cannot reserve isolated oracle test output")
endif()
file(MAKE_DIRECTORY "${_directory}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_GOLDEN_SUITE=capture:${_directory}"
    "RIGEXEC_GOLDEN_SUITE_NAME=missing-oracle"
    "RIGEXEC_GOLDEN_SUITE_RAW=0"
    "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_COMPILE_GOLDEN="
    "${DRIVER}" golden-suite
    RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 10)
file(REMOVE_RECURSE "${_directory}")
if(NOT _result STREQUAL "0" OR
   NOT _err MATCHES "observer completion: RigExec oracle library is unavailable")
    message(FATAL_ERROR "Missing oracle suite did not return a recoverable error: ${_result}\n${_out}\n${_err}")
endif()
