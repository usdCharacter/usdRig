
# Install-time step: rewrite the build's USD-free source and include
# manifests into the installed-manifest grammar (cmake/rigExecConfig.cmake.in)
# and write them under the staged destination. Entries under this build's
# runtime-generated/ tree keep that reserved first component and resolve
# under the installed manifest directory; every other entry is rewritten
# checkout-relative and resolves under the matching consumer checkout.
# Relative entries resolve against the declaring checkout root before
# classification, and a build nested under the checkout classifies the same
# as a separate sibling build. Invoke with SOURCE_ROOT, BUILD_DIR, PREFIX
# and DATA_DIR, where PREFIX carries any DESTDIR staging root.
# Every source entry is a translation unit a consumer must compile, so an
# entry with an unsupported extension is a producer bug and fails here rather
# than being dropped downstream.
foreach (_required SOURCE_ROOT BUILD_DIR PREFIX DATA_DIR)    if (NOT ${_required})
        message(FATAL_ERROR "exportManifests.cmake needs -D${_required}=...")
    endif ()
endforeach ()


foreach (_root SOURCE_ROOT BUILD_DIR)
    file(TO_CMAKE_PATH "${${_root}}" _root_path)
    cmake_path(SET _root_path NORMALIZE "${_root_path}")
    if (EXISTS "${_root_path}")
        file(REAL_PATH "${_root_path}" _root_path)
    endif ()
    cmake_path(SET ${_root} NORMALIZE "${_root_path}")
endforeach ()
set(_generated_root "${BUILD_DIR}/runtime-generated")
set(_unit_extensions ".c" ".cc" ".cxx" ".cpp" ".c++")

function (export_manifest _manifest)
    get_filename_component(_name "${_manifest}" NAME_WE)
    if (NOT EXISTS "${_manifest}")
        message(FATAL_ERROR
            "${_manifest} is missing: configure and build rigExecRuntime first")
    endif ()
    if (_name MATCHES "Includes$")
        set(_directory TRUE)
    endif ()
    file(STRINGS "${_manifest}" _entries)
    set(_relocated "")
    foreach (_entry IN LISTS _entries)
        file(TO_CMAKE_PATH "${_entry}" _path)
        if (_path STREQUAL "")
            continue ()
        endif ()
        cmake_path(IS_ABSOLUTE _path _absolute)
        if (_absolute)
            set(_candidate "${_path}")
        else ()
            set(_candidate "${SOURCE_ROOT}/${_path}")
        endif ()
        cmake_path(SET _candidate NORMALIZE "${_candidate}")
        if (EXISTS "${_candidate}")
            file(REAL_PATH "${_candidate}" _resolved)
            cmake_path(SET _resolved NORMALIZE "${_resolved}")
        else ()
            set(_resolved "${_candidate}")
        endif ()
        cmake_path(IS_PREFIX _generated_root "${_resolved}" NORMALIZE _in_generated)
        if (_in_generated)
            cmake_path(RELATIVE_PATH _resolved BASE_DIRECTORY "${BUILD_DIR}"
                OUTPUT_VARIABLE _out)
            if (NOT _out MATCHES "^runtime-generated(/|$)")
                message(FATAL_ERROR
                    "${_manifest}: ${_entry} left the generated payload root: ${_out}")
            endif ()
        else ()
            cmake_path(IS_PREFIX BUILD_DIR "${_resolved}" NORMALIZE _in_build)
            if (_in_build)
                message(FATAL_ERROR
                    "${_manifest}: ${_entry} is in this build tree but outside the generated payload at ${_generated_root}")
            endif ()
            cmake_path(IS_PREFIX SOURCE_ROOT "${_resolved}" NORMALIZE _in_checkout)
            if (NOT _in_checkout)
                message(FATAL_ERROR
                    "${_manifest}: ${_entry} resolves outside this checkout and this build tree: ${_resolved}")
            endif ()
            cmake_path(RELATIVE_PATH _resolved BASE_DIRECTORY "${SOURCE_ROOT}"
                OUTPUT_VARIABLE _out)
        endif ()
        if (_out STREQUAL "")
            message(FATAL_ERROR "${_manifest}: ${_entry} names a root, not an entry")
        endif ()
        if (NOT EXISTS "${_resolved}")
            message(FATAL_ERROR
                "${_manifest}: ${_entry} does not exist (${_resolved}): configure and build rigExecRuntime first")
        endif ()
        if (_directory AND NOT IS_DIRECTORY "${_resolved}")
            message(FATAL_ERROR "${_manifest}: include ${_entry} is not a directory")
        endif ()
        if (NOT _directory AND IS_DIRECTORY "${_resolved}")
            message(FATAL_ERROR "${_manifest}: source ${_entry} is a directory, not a file")
        endif ()
        if (NOT _directory)
            get_filename_component(_extension "${_resolved}" EXT)
            string(TOLOWER "${_extension}" _extension)
            list(FIND _unit_extensions "${_extension}" _unit_index)
            if (_unit_index EQUAL -1)
                message(FATAL_ERROR
                    "${_manifest}: source ${_entry} is not a translation unit "
                    "(supported extensions: ${_unit_extensions})")
            endif ()
        endif ()
        list(APPEND _relocated "${_out}")
    endforeach ()
    if (NOT _relocated)
        message(FATAL_ERROR "${_manifest} lists no entries")
    endif ()
    file(MAKE_DIRECTORY "${PREFIX}/${DATA_DIR}")
    list(JOIN _relocated "\n" _text)
    file(WRITE "${PREFIX}/${DATA_DIR}/${_name}.txt" "${_text}\n")
endfunction ()
foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    set(_path "${BUILD_DIR}/${_manifest}.txt")
    if (NOT EXISTS "${_path}")
        message(FATAL_ERROR
            "${_path} is missing: configure and build rigExecRuntime first")
    endif ()
    file(STRINGS "${_path}" _entries)
    rigexec_relocate_manifest_text("${SOURCE_ROOT}" _text _entries)
    file(WRITE "${OUT_DIR}/${_manifest}.txt" "${_text}")
endforeach ()
