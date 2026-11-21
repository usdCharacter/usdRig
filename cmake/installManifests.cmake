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
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/runtimeSources.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/runtimeIncludes.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/evalCoreSources.txt"
            "${RIGEXEC_MANIFEST_EXPORT_BUILD_DIR}/evalCoreIncludes.txt"
        VERBATIM)
    add_custom_target(rigexecSdkManifests ALL DEPENDS ${_outputs})
endfunction()

# No configure-time existence check: the copies appear during the build, and a
# missing one has to fail the install step.
function(rigexec_install_sdk_manifests)
    cmake_parse_arguments(RIGEXEC_MANIFEST_INSTALL ""
        "BUILD_DIR;DESTINATION" "" ${ARGN})
    if (NOT RIGEXEC_MANIFEST_INSTALL_BUILD_DIR
        OR NOT RIGEXEC_MANIFEST_INSTALL_DESTINATION)
        message(FATAL_ERROR
            "rigexec_install_sdk_manifests needs BUILD_DIR=... DESTINATION=...")
    endif ()
    install(FILES
        "${RIGEXEC_MANIFEST_INSTALL_BUILD_DIR}/runtimeSources.txt"
        "${RIGEXEC_MANIFEST_INSTALL_BUILD_DIR}/runtimeIncludes.txt"
        "${RIGEXEC_MANIFEST_INSTALL_BUILD_DIR}/evalCoreSources.txt"
        "${RIGEXEC_MANIFEST_INSTALL_BUILD_DIR}/evalCoreIncludes.txt"
        DESTINATION "${RIGEXEC_MANIFEST_INSTALL_DESTINATION}")
endfunction()
