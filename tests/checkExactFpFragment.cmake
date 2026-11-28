# Guards cmake/rigexec_flags.cmake as the single authority for the exact-FP
# settings and for the USD-free file lists: the flag list the build writes
# must be the fragment's, and the manifests the SDK ships must be the same
# entries, relocated into the installed-manifest grammar of
# cmake/rigExecConfig.cmake.in and resolvable under the root their reserved
# first component selects.

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


foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes
         opCoreSources)
    if (NOT EXISTS "${STAGED_DIR}/${_manifest}.txt")
        message(FATAL_ERROR
            "no staged ${STAGED_DIR}/${_manifest}.txt: build the rigexecSdkManifests target")
    endif ()
endforeach ()

# The staged copies must be the relocation of the current build lists, in
# order and byte for byte: an equal count in a different order, or a list
# from an earlier generation, is a stale export.
include("${SOURCE_DIR}/cmake/manifestRelocate.cmake")
foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes
         opCoreSources)
    if (NOT EXISTS "${BINARY_DIR}/${_manifest}.txt")
        message(FATAL_ERROR
            "generated ${BINARY_DIR}/${_manifest}.txt is missing; the staged copy is stale")
    endif ()
    file(STRINGS "${BINARY_DIR}/${_manifest}.txt" _built)
    if (_manifest MATCHES "Includes$")
        set(_is_directory TRUE)
    else ()
        set(_is_directory FALSE)
    endif ()
    rigexec_relocate_manifest_text("${SOURCE_DIR}" "${BINARY_DIR}" _built
        ${_is_directory} _expected _expected_entries)
    file(READ "${STAGED_DIR}/${_manifest}.txt" _shipped_text)
    if (NOT _shipped_text STREQUAL _expected)
        message(FATAL_ERROR
            "${_manifest}.txt is not the ordered relocation of the current ${BINARY_DIR}/${_manifest}.txt (stale export)")
    endif ()
endforeach ()

# The manifests list translation units a separate consumer root builds;
# CMake's precompiled-header support injects a synthetic cmake_pch.cxx
# into a target's sources at generate time and it must stay out.
foreach (_manifest runtimeSources evalCoreSources opCoreSources)
    file(STRINGS "${BINARY_DIR}/${_manifest}.txt" _built)
    foreach (_entry IN LISTS _built)
        if (_entry MATCHES "(^|/|\\\\)CMakeFiles/"
            OR _entry MATCHES "(^|/|\\\\)cmake_pch\\.(cxx|cpp|c)$")
            message(FATAL_ERROR
                "${_manifest}.txt lists compiler-generated unit ${_entry}")
        endif ()
    endforeach ()
endforeach ()

# A staged install ships the generated payload beside the manifests, so the
# scratch tree mirrors that install(FILES) payload from the build tree before
# resolving runtime-generated/ entries under the manifest dir.
set(_manifest_dir "${BINARY_DIR}/exactFpFragmentCheck")
file(REMOVE_RECURSE "${_manifest_dir}")
if (NOT IS_DIRECTORY "${BINARY_DIR}/runtime-generated")
    message(FATAL_ERROR
        "${BINARY_DIR}/runtime-generated is missing: configure and build rigExecRuntime first")
endif ()
file(COPY "${BINARY_DIR}/runtime-generated" DESTINATION "${_manifest_dir}")

# One shipped entry against the installed grammar: never absolute, never a
# '..' segment; the reserved runtime-generated/ component resolves with its
# kind under the manifest directory, every other entry under the checkout.
function (check_installed_entry entry manifest kind root manifest_dir)
    cmake_path(SET _path "${entry}")
    cmake_path(IS_ABSOLUTE _path _absolute)
    if (_absolute)
        message(FATAL_ERROR "${manifest}.txt ships an absolute path: ${entry}")
    endif ()
    if (entry MATCHES "(^|/)\\.\\.(/|$)")
        message(FATAL_ERROR "${manifest}.txt ships a '..' segment: ${entry}")
    endif ()
    if (entry MATCHES "^runtime-generated(/|$)")
        set(_resolved "${manifest_dir}/${entry}")
    else ()
        set(_resolved "${root}/${entry}")
    endif ()
    if (kind STREQUAL "file")
        if (NOT EXISTS "${_resolved}" OR IS_DIRECTORY "${_resolved}")
            message(FATAL_ERROR
                "${manifest}.txt source does not resolve to a file: ${entry} -> ${_resolved}")
        endif ()
    else ()
        if (NOT IS_DIRECTORY "${_resolved}")
            message(FATAL_ERROR
                "${manifest}.txt include does not resolve to a directory: ${entry} -> ${_resolved}")
        endif ()
    endif ()
endfunction ()

foreach (_manifest runtimeSources evalCoreSources opCoreSources)
    file(STRINGS "${BINARY_DIR}/${_manifest}.txt" _built)
    file(STRINGS "${STAGED_DIR}/${_manifest}.txt" _shipped)
    list(LENGTH _built _built_count)
    list(LENGTH _shipped _shipped_count)
    if (NOT _built_count EQUAL _shipped_count OR NOT _built_count)
        message(FATAL_ERROR "${_manifest}.txt: ${_built_count} built, ${_shipped_count} shipped")
    endif ()
    set(_generated_seen FALSE)
    foreach (_entry IN LISTS _shipped)
        check_installed_entry("${_entry}" "${_manifest}" file "${SOURCE_DIR}" "${_manifest_dir}")
        if (_entry MATCHES "^runtime-generated(/|$)")
            set(_generated_seen TRUE)
        endif ()
    endforeach ()
    if (_manifest STREQUAL runtimeSources AND NOT _generated_seen)
        message(FATAL_ERROR "${_manifest}.txt ships no runtime-generated/ entry")
    endif ()
endforeach ()

foreach (_manifest runtimeIncludes evalCoreIncludes)
    file(STRINGS "${STAGED_DIR}/${_manifest}.txt" _shipped)
    list(LENGTH _shipped _shipped_count)
    if (NOT _shipped_count)
        message(FATAL_ERROR "${_manifest}.txt ships no include directory")
    endif ()
    foreach (_entry IN LISTS _shipped)
        check_installed_entry("${_entry}" "${_manifest}" directory "${SOURCE_DIR}" "${_manifest_dir}")
    endforeach ()
endforeach ()

message(STATUS "exact-FP fragment and manifests agree (${_expected_text})")
