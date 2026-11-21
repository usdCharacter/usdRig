cmake_minimum_required(VERSION 3.26)
project(RigExecInstallStaging NONE)

# Fixture for tests/checkInstallStaging.cmake: it reuses the production install
# rule instead of restating it, so the staging check cannot drift from the
# product. Never add_subdirectory'd.
if (NOT RIGEXEC_MANIFEST_MODULE OR NOT RIGEXEC_STAGED_MANIFESTS
    OR NOT RIGEXEC_MANIFEST_INPUT_DIR OR NOT RIGEXEC_MANIFEST_SOURCE_ROOT
    OR NOT RIGEXEC_INSTALL_DATADIR)
    message(FATAL_ERROR
        "RigExecInstallStaging needs RIGEXEC_MANIFEST_MODULE, RIGEXEC_STAGED_MANIFESTS, RIGEXEC_MANIFEST_INPUT_DIR, RIGEXEC_MANIFEST_SOURCE_ROOT and RIGEXEC_INSTALL_DATADIR")
endif()

include("${RIGEXEC_MANIFEST_MODULE}")
rigexec_install_sdk_manifests(BUILD_DIR "${RIGEXEC_STAGED_MANIFESTS}"
    INPUT_DIR "${RIGEXEC_MANIFEST_INPUT_DIR}"
    SOURCE_ROOT "${RIGEXEC_MANIFEST_SOURCE_ROOT}"
    DESTINATION "${RIGEXEC_INSTALL_DATADIR}")
