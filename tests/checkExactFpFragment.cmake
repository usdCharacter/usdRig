# Guards cmake/rigexec_flags.cmake as the single authority for the exact-FP
# settings and for the USD-free file lists: the flag list the build writes
# must be the fragment's, and the manifests the SDK ships must be the same
# paths, relocated against a checkout root.

if (NOT SOURCE_DIR OR NOT BINARY_DIR OR NOT STAGED_DIR)
    message(FATAL_ERROR
        "checkExactFpFragment.cmake needs SOURCE_DIR, BINARY_DIR and STAGED_DIR")
endif ()

file(GLOB _compiler_info "${BINARY_DIR}/CMakeFiles/*/CMakeCXXCompiler.cmake")
if (_compiler_info)
    list(SORT _compiler_info ORDER DESCENDING)
    list(GET _compiler_info 0 _compiler_info)
    include("${_compiler_info}")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC"
        OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
        set(MSVC TRUE)
    endif ()
else ()
    file(STRINGS "${BINARY_DIR}/CMakeCache.txt" _compiler_id
        REGEX "^CMAKE_CXX_COMPILER_ID:")
    if (_compiler_id MATCHES "MSVC")
        set(MSVC TRUE)
    endif ()
endif ()
include("${SOURCE_DIR}/cmake/rigexec_flags.cmake")
if (NOT COMMAND rigexec_apply_exact_fp_flags
      OR NOT COMMAND rigexec_get_exact_fp_flags)
    message(FATAL_ERROR "cmake/rigexec_flags.cmake defines neither flag function")
endif ()

rigexec_get_exact_fp_flags(_expected)
file(STRINGS "${BINARY_DIR}/exactFpFlags.txt" _generated)
list(JOIN _expected "|" _expected_text)
list(JOIN _generated "|" _generated_text)
if (NOT _expected_text STREQUAL _generated_text)
    message(FATAL_ERROR "build/exactFpFlags.txt lists ${_generated_text}, the fragment says ${_expected_text}")
endif ()

foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    if (NOT EXISTS "${STAGED_DIR}/${_manifest}.txt")
        message(FATAL_ERROR
            "no staged ${STAGED_DIR}/${_manifest}.txt: build the rigexecSdkManifests target")
    endif ()
endforeach ()

foreach (_manifest runtimeSources evalCoreSources)
    file(STRINGS "${BINARY_DIR}/${_manifest}.txt" _built)
    file(STRINGS "${STAGED_DIR}/${_manifest}.txt" _shipped)
    list(LENGTH _built _built_count)
    list(LENGTH _shipped _shipped_count)
    if (NOT _built_count EQUAL _shipped_count OR NOT _built_count)
        message(FATAL_ERROR "${_manifest}.txt: ${_built_count} built, ${_shipped_count} shipped")
    endif ()
    foreach (_entry IN LISTS _shipped)
        if (IS_ABSOLUTE "${_entry}")
            message(FATAL_ERROR "${_manifest}.txt ships an absolute path: ${_entry}")
        endif ()
        if (NOT EXISTS "${SOURCE_DIR}/${_entry}")
            message(FATAL_ERROR "${_manifest}.txt path does not resolve from the checkout root: ${_entry}")
        endif ()
    endforeach ()
endforeach ()

foreach (_manifest runtimeIncludes evalCoreIncludes)
    file(STRINGS "${STAGED_DIR}/${_manifest}.txt" _shipped)
    list(LENGTH _shipped _shipped_count)
    if (NOT _shipped_count)
        message(FATAL_ERROR "${_manifest}.txt ships no include directory")
    endif ()
    foreach (_entry IN LISTS _shipped)
        if (IS_ABSOLUTE "${_entry}")
            message(FATAL_ERROR "${_manifest}.txt ships an absolute path: ${_entry}")
        endif ()
        if (NOT IS_DIRECTORY "${SOURCE_DIR}/${_entry}")
            message(FATAL_ERROR "${_manifest}.txt include does not resolve from the checkout root: ${_entry}")
        endif ()
    endforeach ()
endforeach ()

message(STATUS "exact-FP fragment and manifests agree (${_expected_text})")
