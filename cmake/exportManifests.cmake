# Build-time step: rewrite the build's USD-free source and include manifests
# into the installed-manifest grammar with cmake/manifestRelocate.cmake and
# write the staged copies to OUT_DIR. See cmake/rigExecConfig.cmake.in for the
# grammar the shipped entries follow.
include("${CMAKE_CURRENT_LIST_DIR}/manifestRelocate.cmake")

foreach (_required SOURCE_ROOT BUILD_DIR OUT_DIR)
    if (NOT ${_required})
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

foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    set(_input "${BUILD_DIR}/${_manifest}.txt")
    if (NOT EXISTS "${_input}")
        message(FATAL_ERROR
            "${_input} is missing: configure and build rigExecRuntime first")
    endif ()
    file(STRINGS "${_input}" _entries)
    if (_manifest MATCHES "Includes$")
        set(_is_directory TRUE)
    else ()
        set(_is_directory FALSE)
    endif ()
    rigexec_relocate_manifest_text("${SOURCE_ROOT}" "${BUILD_DIR}" _entries
        ${_is_directory} _text _relocated)
    file(MAKE_DIRECTORY "${OUT_DIR}")
    file(WRITE "${OUT_DIR}/${_manifest}.txt" "${_text}")
endforeach ()
