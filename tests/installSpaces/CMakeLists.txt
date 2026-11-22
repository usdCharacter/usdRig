cmake_minimum_required(VERSION 3.26)
project(RigExecInstallSpaces NONE)

# Fixture for tests/checkSdkInstallSpaces.cmake: it reuses the production
# export and install rules instead of restating them, so the spaces check
# cannot drift from the product. Never add_subdirectory'd.
if (NOT RIGEXEC_SDK_INSTALL_MODULE OR NOT RIGEXEC_SDK_SOURCE_ROOT
    OR NOT RIGEXEC_SDK_BUILD_DIR OR NOT RIGEXEC_SDK_STAGED_DIR
    OR NOT RIGEXEC_SDK_FLAGS_FILE OR NOT RIGEXEC_INSTALL_DATADIR)
    message(FATAL_ERROR
        "RigExecInstallSpaces needs RIGEXEC_SDK_INSTALL_MODULE, "
        "RIGEXEC_SDK_SOURCE_ROOT, RIGEXEC_SDK_BUILD_DIR, "
        "RIGEXEC_SDK_STAGED_DIR, RIGEXEC_SDK_FLAGS_FILE and "
        "RIGEXEC_INSTALL_DATADIR")
endif ()

include("${RIGEXEC_SDK_INSTALL_MODULE}")

rigexec_add_sdk_manifest_export(
    SOURCE_ROOT "${RIGEXEC_SDK_SOURCE_ROOT}"
    BUILD_DIR "${RIGEXEC_SDK_BUILD_DIR}"
    OUT_DIR "${RIGEXEC_SDK_STAGED_DIR}")

rigexec_install_sdk_manifests(
    BUILD_DIR "${RIGEXEC_SDK_STAGED_DIR}"
    INPUT_DIR "${RIGEXEC_SDK_BUILD_DIR}"
    SOURCE_ROOT "${RIGEXEC_SDK_SOURCE_ROOT}"
    DESTINATION "${RIGEXEC_INSTALL_DATADIR}"
    FLAGS_FILE "${RIGEXEC_SDK_FLAGS_FILE}")
