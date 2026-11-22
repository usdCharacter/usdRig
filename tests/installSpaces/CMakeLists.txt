cmake_minimum_required(VERSION 3.26)
project(RigExecInstallSpaces NONE)

# Fixture for tests/checkSdkInstallSpaces.cmake: it reuses the production
# install rule instead of restating it, so the spaces check cannot drift from
# the product. Never add_subdirectory'd.
if (NOT RIGEXEC_SDK_INSTALL_MODULE OR NOT RIGEXEC_SDK_SOURCE_ROOT
    OR NOT RIGEXEC_SDK_BUILD_DIR OR NOT RIGEXEC_SDK_EXPORT_SCRIPT
    OR NOT RIGEXEC_SDK_FLAGS_FILE OR NOT RIGEXEC_INSTALL_DATADIR)
    message(FATAL_ERROR
        "RigExecInstallSpaces needs RIGEXEC_SDK_INSTALL_MODULE, RIGEXEC_SDK_SOURCE_ROOT, RIGEXEC_SDK_BUILD_DIR, RIGEXEC_SDK_EXPORT_SCRIPT, RIGEXEC_SDK_FLAGS_FILE and RIGEXEC_INSTALL_DATADIR")
endif()

include("${RIGEXEC_SDK_INSTALL_MODULE}")

set(_payload
    "${RIGEXEC_SDK_BUILD_DIR}/runtime-generated/moverCatalog.cpp"
    "${RIGEXEC_SDK_BUILD_DIR}/runtime-generated/TestMover/TestMoverKernel.cpp"
    "${RIGEXEC_SDK_BUILD_DIR}/runtime-generated/TestMover/TestMoverSchema.h"
    "${RIGEXEC_SDK_BUILD_DIR}/runtime-generated/TestMover/TestMoverRegistration.h")

rigexec_install_sdk_payload(
    SOURCE_ROOT "${RIGEXEC_SDK_SOURCE_ROOT}"
    BUILD_DIR "${RIGEXEC_SDK_BUILD_DIR}"
    EXPORT_SCRIPT "${RIGEXEC_SDK_EXPORT_SCRIPT}"
    DATA_DIR "${RIGEXEC_INSTALL_DATADIR}"
    FLAGS_FILE "${RIGEXEC_SDK_FLAGS_FILE}"
    PAYLOAD ${_payload})
