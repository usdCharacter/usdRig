# Runs real `cmake --install` over the SDK manifest rule: a plain prefix, a
# DESTDIR-staged install, a CMAKE_STAGING_PREFIX-staged install, and an
# incomplete staged export. The fixture project carries the production install
# rule, so the check exercises the shipped install rather than a copy of it.

if (NOT SOURCE_DIR OR NOT BINARY_DIR OR NOT SCRATCH_DIR)
    message(FATAL_ERROR
        "checkInstallStaging.cmake needs SOURCE_DIR, BINARY_DIR and SCRATCH_DIR")
endif()
if (NOT INSTALL_DATADIR)
    message(FATAL_ERROR
        "checkInstallStaging.cmake needs INSTALL_DATADIR=... (where the manifests ship)")
endif()
if (NOT GENERATOR)
    # cmake -P has no generator of its own; reuse the tree under check.
    file(STRINGS "${BINARY_DIR}/CMakeCache.txt" _generator
        REGEX "^CMAKE_GENERATOR:")
    string(REPLACE "CMAKE_GENERATOR:INTERNAL=" "" GENERATOR "${_generator}")
    if (NOT GENERATOR)
        set(GENERATOR "Unix Makefiles")
    endif()
endif()

set(_manifests runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
set(_staged "${BINARY_DIR}/sdkManifests")
set(_module "${SOURCE_DIR}/cmake/installManifests.cmake")
set(_fixture "${SOURCE_DIR}/tests/installStaging")

file(REMOVE_RECURSE "${SCRATCH_DIR}")

function(_clean_and_fail text)
    file(REMOVE_RECURSE "${SCRATCH_DIR}")
    message(FATAL_ERROR "${text}")
endfunction()

# _rc and _out are the caller's: macros keep them in the scenario's scope.
macro(_configure_fixture build_dir staged_dir configure_args)
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${_fixture}" -B "${build_dir}" -G "${GENERATOR}"
        "-DRIGEXEC_MANIFEST_MODULE=${_module}"
        "-DRIGEXEC_STAGED_MANIFESTS=${staged_dir}"
        "-DRIGEXEC_INSTALL_DATADIR=${INSTALL_DATADIR}"
        ${configure_args}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("fixture configure failed (${GENERATOR}): ${_out}")
    endif()
endmacro()

# Staging is requested through the environment, which execute_process inherits.
macro(_install build_dir install_args)
    execute_process(COMMAND "${CMAKE_COMMAND}" --install "${build_dir}"
        ${install_args}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
endmacro()

function(_assert_identical expected actual label)
    if (NOT EXISTS "${actual}")
        _clean_and_fail("${label}: ${actual} is missing after install")
    endif()
    file(READ "${expected}" _expected_hex HEX)
    file(READ "${actual}" _actual_hex HEX)
    if (NOT _expected_hex STREQUAL _actual_hex)
        _clean_and_fail("${label}: ${actual} is not byte-identical to ${expected}")
    endif()
endfunction()

function(_check_manifests root label)
    foreach (_manifest IN LISTS _manifests)
        _assert_identical("${_staged}/${_manifest}.txt"
            "${root}/${INSTALL_DATADIR}/${_manifest}.txt" "${label}")
    endforeach()
endfunction()

foreach (_manifest IN LISTS _manifests)
    if (NOT EXISTS "${_staged}/${_manifest}.txt")
        _clean_and_fail("no staged ${_staged}/${_manifest}.txt: build the rigexecSdkManifests target first")
    endif()
endforeach()

# Probe what this CMake and generator really do before asserting anything.
macro(_probe probe_dir build_dir configure_args install_args destdir)
    file(WRITE "${probe_dir}/probe.txt" "probe\n")
    file(WRITE "${probe_dir}/CMakeLists.txt"
        "cmake_minimum_required(VERSION 3.26)\n"
        "project(InstallStagingProbe NONE)\n"
        "install(FILES \"${probe_dir}/probe.txt\" DESTINATION \"share/probe\")\n")
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${probe_dir}" -B "${build_dir}" -G "${GENERATOR}"
        ${configure_args}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("staging probe configure failed (${GENERATOR}): ${_out}")
    endif()
    set(ENV{DESTDIR} "${destdir}")
    _install("${build_dir}" "${install_args}")
    set(ENV{DESTDIR} "")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("staging probe install failed: ${_out}")
    endif()
endmacro()

set(_destdir_root "")
set(_staging_root "")
set(_staging_keeps_prefix FALSE)

set(_probe_dir "${SCRATCH_DIR}/probeDestdir")
_probe("${_probe_dir}" "${_probe_dir}/build" ""
    "--prefix;${_probe_dir}/logical" "${_probe_dir}/stage")
if (EXISTS "${_probe_dir}/stage${_probe_dir}/logical/share/probe/probe.txt")
    set(_destdir_root "${_probe_dir}/stage${_probe_dir}/logical")
    message(STATUS "probe: DESTDIR supported (${GENERATOR}, CMake ${CMAKE_VERSION})")
else()
    message(STATUS "probe: DESTDIR ignored (${GENERATOR}, CMake ${CMAKE_VERSION})")
endif()

set(_probe_dir "${SCRATCH_DIR}/probeStaging")
_probe("${_probe_dir}" "${_probe_dir}/build"
    "-DCMAKE_INSTALL_PREFIX=${_probe_dir}/logical;-DCMAKE_STAGING_PREFIX=${_probe_dir}/stage"
    "" "")
if (EXISTS "${_probe_dir}/stage/share/probe/probe.txt")
    set(_staging_root "${_probe_dir}/stage")
    message(STATUS "probe: CMAKE_STAGING_PREFIX supported (${GENERATOR}, CMake ${CMAKE_VERSION})")
elseif (EXISTS "${_probe_dir}/stage${_probe_dir}/logical/share/probe/probe.txt")
    set(_staging_root "${_probe_dir}/stage${_probe_dir}/logical")
    set(_staging_keeps_prefix TRUE)
    message(STATUS "probe: CMAKE_STAGING_PREFIX supported (${GENERATOR}, CMake ${CMAKE_VERSION})")
else()
    message(STATUS "probe: CMAKE_STAGING_PREFIX ignored (${GENERATOR}, CMake ${CMAKE_VERSION})")
endif()

if (NOT _destdir_root AND NOT _staging_root)
    _clean_and_fail("no staging mechanism is supported by ${GENERATOR} (CMake ${CMAKE_VERSION})")
endif()

# (a) plain prefix install.
_configure_fixture("${SCRATCH_DIR}/prefix" "${_staged}" "")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/prefix" "--prefix;${SCRATCH_DIR}/prefixInstall")
if (NOT _rc EQUAL 0)
    _clean_and_fail("prefix install: cmake --install exited ${_rc}: ${_out}")
endif()
_check_manifests("${SCRATCH_DIR}/prefixInstall" "prefix install")
message(STATUS "prefix install: pass (four manifests in ${SCRATCH_DIR}/prefixInstall/${INSTALL_DATADIR})")

# (b) DESTDIR staging: everything lands under <stage><prefix>, nothing in the prefix.
if (_destdir_root)
    set(_logical "${SCRATCH_DIR}/destdirLogical")
    set(_stage "${SCRATCH_DIR}/destdirStage")
    _configure_fixture("${SCRATCH_DIR}/destdir" "${_staged}" "")
    set(ENV{DESTDIR} "${_stage}")
    _install("${SCRATCH_DIR}/destdir" "--prefix;${_logical}")
    set(ENV{DESTDIR} "")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("staged install (DESTDIR): cmake --install exited ${_rc}: ${_out}")
    endif()
    if (EXISTS "${_logical}")
        _clean_and_fail("staged install (DESTDIR): wrote files to the unstaged prefix ${_logical}")
    endif()
    _check_manifests("${_stage}${_logical}" "staged install (DESTDIR)")
    message(STATUS "staged install (DESTDIR): pass (${_stage}${_logical}/${INSTALL_DATADIR}, nothing in ${_logical})")
else()
    message(STATUS "staged install (DESTDIR): unsupported (${GENERATOR}, CMake ${CMAKE_VERSION})")
endif()

# (c) CMAKE_STAGING_PREFIX staging: same shape, the logical prefix comes from configure.
if (_staging_root)
    set(_logical "${SCRATCH_DIR}/stagingLogical")
    set(_stage "${SCRATCH_DIR}/stagingStage")
    _configure_fixture("${SCRATCH_DIR}/staging" "${_staged}"
        "-DCMAKE_INSTALL_PREFIX=${_logical};-DCMAKE_STAGING_PREFIX=${_stage}")
    set(ENV{DESTDIR} "")
    _install("${SCRATCH_DIR}/staging" "")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("staged install (CMAKE_STAGING_PREFIX): cmake --install exited ${_rc}: ${_out}")
    endif()
    if (EXISTS "${_logical}")
        _clean_and_fail("staged install (CMAKE_STAGING_PREFIX): wrote files to the unstaged prefix ${_logical}")
    endif()
    if (_staging_keeps_prefix)
        _check_manifests("${_stage}${_logical}" "staged install (CMAKE_STAGING_PREFIX)")
        message(STATUS "staged install (CMAKE_STAGING_PREFIX): pass (${_stage}${_logical}/${INSTALL_DATADIR}, nothing in ${_logical})")
    else()
        _check_manifests("${_stage}" "staged install (CMAKE_STAGING_PREFIX)")
        message(STATUS "staged install (CMAKE_STAGING_PREFIX): pass (${_stage}/${INSTALL_DATADIR}, nothing in ${_logical})")
    endif()
else()
    message(STATUS "staged install (CMAKE_STAGING_PREFIX): unsupported (${GENERATOR}, CMake ${CMAKE_VERSION})")
endif()

# (d) incomplete export: the install step must fail and name the missing file.
set(_missing "evalCoreIncludes")
set(_partial "${SCRATCH_DIR}/incomplete")
foreach (_manifest IN LISTS _manifests)
    file(COPY "${_staged}/${_manifest}.txt" DESTINATION "${_partial}")
endforeach()
file(REMOVE "${_partial}/${_missing}.txt")
_configure_fixture("${SCRATCH_DIR}/incompleteBuild" "${_partial}" "")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/incompleteBuild"
    "--prefix;${SCRATCH_DIR}/incompleteInstall")
if (_rc EQUAL 0)
    _clean_and_fail("incomplete export: cmake --install exited 0 with ${_missing}.txt absent from ${_partial}")
endif()
if (NOT "${_out}" MATCHES "${_missing}")
    _clean_and_fail("incomplete export: install failed (${_rc}) without naming ${_missing}: ${_out}")
endif()
message(STATUS "incomplete export: pass (cmake --install exited ${_rc} naming ${_missing}.txt)")

file(REMOVE_RECURSE "${SCRATCH_DIR}")
message(STATUS "install staging check: pass (${GENERATOR}, CMake ${CMAKE_VERSION})")
