# The SDK manifests ship as ordinary staged install inputs: the build writes
# the relocated copies into a build-tree directory, and install(FILES) places
# them like any other file, so DESTDIR and CMAKE_STAGING_PREFIX apply.

function(rigexec_add_sdk_manifest_export)
    cmake_parse_arguments(RIGEXEC_MANIFEST_EXPORT ""
        "SOURCE_ROOT;BUILD_DIR;OUT_DIR" "" ${ARGN})
    foreach (_required SOURCE_ROOT BUILD_DIR OUT_DIR)
        if (NOT RIGEXEC_MANIFEST_EXPORT_${_required})
            message(FATAL_ERROR
                "rigexec_add_sdk_manifest_export needs ${_required}=...")
        endif ()
    endforeach ()

    set(_outputs "")
    foreach (_manifest runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
        list(APPEND _outputs "${RIGEXEC_MANIFEST_EXPORT_OUT_DIR}/${_manifest}.txt")
    endforeach ()

    add_custom_command(OUTPUT ${_outputs}
        COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_ROOT=${RIGEXEC_MANIFEST_EXPORT_SOURCE_ROOT}"
            "-DBUILD_DIR=${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}"
            "-DOUT_DIR=${RIGEXEC_MANIFEST_EXPORT_OUT_DIR}"
            -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/exportManifests.cmake"
        DEPENDS
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/exportManifests.cmake"
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/manifestRelocate.cmake"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/runtimeSources.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/runtimeIncludes.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/evalCoreSources.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/evalCoreIncludes.txt"
        VERBATIM)
    add_custom_target(rigexecSdkManifests ALL DEPENDS ${_outputs})
endfunction()

# No configure-time existence check: the copies appear during the build, and a
# missing or stale one has to fail the install step. INPUT_DIR and SOURCE_ROOT
# name the generated lists and root the staged copies were relocated against;
# the install re-derives them and refuses a staged copy that no longer
# matches, so a reconfigure that renames or reorders an export can never
# install the previous generation's bytes. install(FILES) skips a copy whose
# timestamps say "up to date" even when the bytes differ, so a follow-up
# install code compares installed bytes with the staged ones and rewrites a
# skipped copy instead of shipping it.
function(rigexec_install_sdk_manifests)
    cmake_parse_arguments(RIGEXEC_MANIFEST_INSTALL ""
        "BUILD_DIR;INPUT_DIR;SOURCE_ROOT;DESTINATION" "" ${ARGN})
    foreach (_required BUILD_DIR INPUT_DIR SOURCE_ROOT DESTINATION)
        if (NOT RIGEXEC_MANIFEST_INSTALL_${_required})
            message(FATAL_ERROR
                "rigexec_install_sdk_manifests needs ${_required}=...")
        endif ()
    endforeach ()
    set(_staged "${RIGEXEC_MANIFEST_INSTALL_BUILD_DIR}")
    install(CODE "
set(RIGEXEC_VERIFY_SOURCE_ROOT \"${RIGEXEC_MANIFEST_INSTALL_SOURCE_ROOT}\")
set(RIGEXEC_VERIFY_INPUT_DIR \"${RIGEXEC_MANIFEST_INSTALL_INPUT_DIR}\")
set(RIGEXEC_VERIFY_STAGED_DIR \"${_staged}\")
include(\"${CMAKE_CURRENT_FUNCTION_LIST_DIR}/verifyManifests.cmake\")")
    install(FILES
        "${_staged}/runtimeSources.txt"
        "${_staged}/runtimeIncludes.txt"
        "${_staged}/evalCoreSources.txt"
        "${_staged}/evalCoreIncludes.txt"
        DESTINATION "${RIGEXEC_MANIFEST_INSTALL_DESTINATION}")
    install(CODE "
set(_staged \"${_staged}\")
set(_dest \"${RIGEXEC_MANIFEST_INSTALL_DESTINATION}\")
# file(INSTALL) prepends DESTDIR itself; the generated script only knows the
# logical prefix. The effective destination is exactly one layout: with
# DESTDIR set it is the staged tree only, never the logical prefix, so a
# manifest already installed on the host is left untouched and the packaged
# copy is the one verified and refreshed.
if (DEFINED ENV{DESTDIR} AND NOT \"\$ENV{DESTDIR}\" STREQUAL \"\")
    string(REGEX REPLACE \"^[A-Za-z]:\" \"\" _stripped \"\${CMAKE_INSTALL_PREFIX}\")
    set(_candidates
        \"\$ENV{DESTDIR}/\${_stripped}/\${_dest}\"
        \"\$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/\${_dest}\")
else ()
    set(_candidates \"\${CMAKE_INSTALL_PREFIX}/\${_dest}\")
endif ()
foreach (_manifest IN ITEMS runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    set(_installed \"\")
    foreach (_candidate IN LISTS _candidates)
        if (EXISTS \"\${_candidate}/\${_manifest}.txt\")
            set(_installed \"\${_candidate}/\${_manifest}.txt\")
            break ()
        endif ()
    endforeach ()
    if (NOT _installed)
        message(FATAL_ERROR
            \"SDK manifest \${_manifest}: install placed no \"
            \"\${_manifest}.txt for prefix \${CMAKE_INSTALL_PREFIX} \"
            \"(DESTDIR: \$ENV{DESTDIR}).\")
    endif ()
    file(READ \"\${_staged}/\${_manifest}.txt\" _wanted)
    file(READ \"\${_installed}\" _present)
    if (NOT _present STREQUAL _wanted)
        file(WRITE \"\${_installed}\" \"\${_wanted}\")
        file(READ \"\${_installed}\" _present)
        if (NOT _present STREQUAL _wanted)
            message(FATAL_ERROR
                \"SDK manifest \${_manifest}: could not update \${_installed} to the staged bytes.\")
        endif ()
        message(STATUS \"SDK manifest \${_manifest}: overwrote a same-timestamp stale copy at \${_installed}.\")
    endif ()
endforeach ()")
endfunction()
