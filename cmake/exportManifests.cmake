# Build-time step: rewrite the build's USD-free source and include manifests
# with every path made relative to the checkout that holds it, so a host
# building those sources outside CMake joins the list with its own root instead
# of copying it. Invoke with SOURCE_ROOT, BUILD_DIR and OUT_DIR.
foreach (_required SOURCE_ROOT BUILD_DIR OUT_DIR)
    if (NOT ${_required})
        message(FATAL_ERROR "exportManifests.cmake needs -D${_required}=...")
    endif ()
endforeach ()

foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    set(_path "${BUILD_DIR}/${_manifest}.txt")
    if (NOT EXISTS "${_path}")
        message(FATAL_ERROR
            "${_path} is missing: configure and build rigExecRuntime first")
    endif ()
    file(STRINGS "${_path}" _entries)
    set(_relocated "")
    foreach (_entry IN LISTS _entries)
        if (IS_ABSOLUTE "${_entry}")
            file(RELATIVE_PATH _entry "${SOURCE_ROOT}" "${_entry}")
        endif ()
        list(APPEND _relocated "${_entry}")
    endforeach ()
    list(JOIN _relocated "\n" _text)
    file(WRITE "${OUT_DIR}/${_manifest}.txt" "${_text}\n")
endforeach ()

foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    if (NOT EXISTS "${OUT_DIR}/${_manifest}.txt")
        message(FATAL_ERROR
            "exportManifests.cmake wrote no ${OUT_DIR}/${_manifest}.txt")
    endif ()
endforeach ()
