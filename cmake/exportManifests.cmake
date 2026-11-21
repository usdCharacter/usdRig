# Build-time step: rewrite the build's USD-free source and include manifests
# with every path made relative to the checkout that holds it, so a host
# building those sources outside CMake joins the list with its own root instead
# of copying it. Invoke with SOURCE_ROOT, BUILD_DIR and OUT_DIR.
foreach (_required SOURCE_ROOT BUILD_DIR OUT_DIR)
    if (NOT ${_required})
        message(FATAL_ERROR "exportManifests.cmake needs -D${_required}=...")
    endif ()
endforeach ()

include("${CMAKE_CURRENT_LIST_DIR}/manifestRelocate.cmake")

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

foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    if (NOT EXISTS "${OUT_DIR}/${_manifest}.txt")
        message(FATAL_ERROR
            "exportManifests.cmake wrote no ${OUT_DIR}/${_manifest}.txt")
    endif ()
endforeach ()
