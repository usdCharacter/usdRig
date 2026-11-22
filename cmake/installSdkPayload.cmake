# Install rules for the SDK a non-CMake consumer builds from: the generated
# runtime payload, the relocated manifests, and the exact-FP settings those
# sources were compiled with. The top-level build and the spaces regression
# fixture (tests/installSpaces, driven by tests/checkSdkInstallSpaces.cmake)
# share this module, so the fixture exercises the shipped install rather than a
# copy of it. Payload, manifests and flags all land in DATA_DIR of the same
# effective staging root install(FILES) uses, so DESTDIR stages the SDK whole.
# Every path interpolated into install(CODE) stays quoted, so the generated
# script keeps each payload path and each exporter -D argument whole when the
# producer build or the install prefix contains spaces. The prefix itself
# defers to install time, so cmake --install --prefix applies.
include(CMakeParseArguments)
function(rigexec_install_sdk_payload)
    cmake_parse_arguments(_sdk ""
        "SOURCE_ROOT;BUILD_DIR;EXPORT_SCRIPT;DATA_DIR;FLAGS_FILE"
        "PAYLOAD" ${ARGN})
    foreach (_required SOURCE_ROOT BUILD_DIR EXPORT_SCRIPT DATA_DIR)
        if (NOT _sdk_${_required})
            message(FATAL_ERROR
                "rigexec_install_sdk_payload needs ${_required}=...")
        endif ()
    endforeach ()
    if (_sdk_FLAGS_FILE)
        install(FILES "${_sdk_FLAGS_FILE}" DESTINATION "${_sdk_DATA_DIR}")
    endif ()
    if (_sdk_PAYLOAD)
        install(CODE "
        set(_rigexec_install_payloads \"${_sdk_PAYLOAD}\")
        foreach (_payload IN LISTS _rigexec_install_payloads)
            if (NOT EXISTS \"\${_payload}\")
                message(FATAL_ERROR
                    \"\${_payload} is missing: configure and build rigExecRuntime first\")
            endif ()
        endforeach ()
    ")
        foreach (_file IN LISTS _sdk_PAYLOAD)
            file(RELATIVE_PATH _rel "${_sdk_BUILD_DIR}" "${_file}")
            if (_rel MATCHES "(^|/)\.\.(/|$)")
                message(FATAL_ERROR
                    "rigexec_install_sdk_payload: ${_file} is outside ${_sdk_BUILD_DIR}")
            endif ()
            get_filename_component(_dir "${_rel}" DIRECTORY)
            install(FILES "${_file}"
                DESTINATION "${_sdk_DATA_DIR}/${_dir}")
        endforeach ()
    endif ()
    # install(FILES) applies DESTDIR to DESTINATION; a script rule has to
    # apply it itself, or the staged tree gets the payload and not the
    # manifests beside it.
    install(CODE "
set(_rigexec_stage_prefix \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\")
execute_process(COMMAND \"${CMAKE_COMMAND}\"
    \"-DSOURCE_ROOT=${_sdk_SOURCE_ROOT}\"
    \"-DBUILD_DIR=${_sdk_BUILD_DIR}\"
    \"-DPREFIX=\${_rigexec_stage_prefix}\"
    \"-DDATA_DIR=${_sdk_DATA_DIR}\"
    -P \"${_sdk_EXPORT_SCRIPT}\"
    RESULT_VARIABLE _rigexec_export_status)
if (NOT _rigexec_export_status EQUAL 0)
    message(FATAL_ERROR
        \"manifest export failed (status \${_rigexec_export_status})\")
endif ()")
endfunction()
