# Relocation transform shared by the build-time export (exportManifests.cmake)
# and the install-time freshness gate (verifyManifests.cmake). A shipped entry
# is either under the build's reserved runtime-generated/ payload root or
# relative to the checkout root, so a consumer root joins the relative entries
# with its own checkout instead of copying producer build paths; a compiled
# unit anywhere else in the build tree names a file no consumer has.
function(rigexec_relocate_manifest_text source_root build_dir entries_var
                                       is_directory out_text_var out_entries_var)
    set(_generated_root "${build_dir}/runtime-generated")
    # One translation-unit policy for every consumer (the standalone compile
    # check and the Godot SConstruct): the FINAL suffix names the language,
    # case-sensitively, so mesh.scalar.cxx is a C++ unit.
    set(_unit_extensions ".c" ".cc" ".cxx" ".cpp" ".c++")
    set(_relocated "")
    foreach (_entry IN LISTS ${entries_var})
        string(STRIP "${_entry}" _entry)
        if (_entry STREQUAL "")
            continue ()
        endif ()
        file(TO_CMAKE_PATH "${_entry}" _path)
        cmake_path(IS_ABSOLUTE _path _absolute)
        if (_absolute)
            set(_candidate "${_path}")
        else ()
            set(_candidate "${source_root}/${_path}")
        endif ()
        cmake_path(SET _candidate NORMALIZE "${_candidate}")
        cmake_path(IS_PREFIX _generated_root "${_candidate}" NORMALIZE
            _underGenerated)
        cmake_path(IS_PREFIX build_dir "${_candidate}" NORMALIZE _underBuild)
        cmake_path(IS_PREFIX source_root "${_candidate}" NORMALIZE
            _underSource)
        if (_underGenerated)
            cmake_path(RELATIVE_PATH _candidate BASE_DIRECTORY "${build_dir}"
                OUTPUT_VARIABLE _out)
        elseif (_underBuild)
            message(FATAL_ERROR
                "manifest entry ${_entry} is in the build tree ${build_dir} "
                "outside the generated payload at ${_generated_root}")
        elseif (_underSource)
            cmake_path(RELATIVE_PATH _candidate BASE_DIRECTORY "${source_root}"
                OUTPUT_VARIABLE _out)
        else ()
            message(FATAL_ERROR
                "manifest entry ${_entry} resolves outside the checkout "
                "${source_root} and its generated payload: ${_candidate}")
        endif ()
        if (_out STREQUAL "")
            message(FATAL_ERROR "manifest entry ${_entry} names a root, not an entry")
        endif ()
        # A shipped generated unit has to be in the payload the install stages.
        if (_underGenerated AND NOT EXISTS "${build_dir}/${_out}")
            message(FATAL_ERROR
                "manifest entry ${_entry} names generated unit ${_out}, which "
                "${build_dir}/runtime-generated does not hold")
        endif ()
        if (NOT is_directory)
            get_filename_component(_extension "${_candidate}" LAST_EXT)
            list(FIND _unit_extensions "${_extension}" _unitIndex)
            if (_unitIndex EQUAL -1)
                message(FATAL_ERROR
                    "manifest entry ${_entry} is not a translation unit "
                    "(supported final suffixes: ${_unit_extensions})")
            endif ()
        endif ()
        list(APPEND _relocated "${_out}")
    endforeach ()
    if (NOT _relocated)
        message(FATAL_ERROR "manifest lists no entries")
    endif ()
    list(JOIN _relocated "\n" _text)
    set(${out_text_var} "${_text}\n" PARENT_SCOPE)
    set(${out_entries_var} "${_relocated}" PARENT_SCOPE)
endfunction ()
