# Install-time freshness gate for the SDK manifests, included from the
# install(CODE) that rigexec_install_sdk_manifests registers. Re-derives each
# relocated list from the current build-tree inputs and compares bytes with
# the staged copy: a staged list that no longer matches its inputs is stale,
# and the install names it instead of shipping it. No build runs here.
# The installer sets RIGEXEC_VERIFY_SOURCE_ROOT, RIGEXEC_VERIFY_INPUT_DIR and
# RIGEXEC_VERIFY_STAGED_DIR before include().
include("${CMAKE_CURRENT_LIST_DIR}/manifestRelocate.cmake")

foreach (_manifest IN ITEMS runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
    set(_input "${RIGEXEC_VERIFY_INPUT_DIR}/${_manifest}.txt")
    set(_staged "${RIGEXEC_VERIFY_STAGED_DIR}/${_manifest}.txt")
    if (NOT EXISTS "${_input}")
        message(FATAL_ERROR
            "SDK manifest ${_manifest}: generated input ${_input} is "
            "missing, so the staged ${_manifest}.txt is stale. Reconfigure "
            "and rebuild rigexecSdkManifests before installing.")
    endif ()
    if (NOT EXISTS "${_staged}")
        message(FATAL_ERROR
            "SDK manifest ${_manifest}: staged ${_staged} is missing. "
            "Build the rigexecSdkManifests target before installing.")
    endif ()
    file(STRINGS "${_input}" _entries)
    rigexec_relocate_manifest_text("${RIGEXEC_VERIFY_SOURCE_ROOT}" _expected _entries)
    file(READ "${_staged}" _actual)
    if (NOT _actual STREQUAL _expected)
        message(FATAL_ERROR
            "SDK manifest ${_manifest}: staged ${_staged} does not match "
            "the current ${_input} (stale export). Rebuild the "
            "rigexecSdkManifests target before installing.")
    endif ()
endforeach ()
