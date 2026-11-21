# Runs real `cmake --install` over the SDK payload rule from a producer
# build directory containing spaces into an override prefix containing
# spaces, then requires a missing payload to fail naming that file. The
# fixture project carries the production install rule, so the check
# exercises the shipped install rather than a copy of it.
if (NOT SOURCE_DIR OR NOT BINARY_DIR OR NOT SCRATCH_DIR)
    message(FATAL_ERROR
        "checkSdkInstallSpaces.cmake needs SOURCE_DIR, BINARY_DIR and SCRATCH_DIR")
endif()
if (NOT INSTALL_DATADIR)
    message(FATAL_ERROR
        "checkSdkInstallSpaces.cmake needs INSTALL_DATADIR=... (where the manifests ship)")
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

set(_module "${SOURCE_DIR}/cmake/installSdkPayload.cmake")
set(_export "${SOURCE_DIR}/cmake/exportManifests.cmake")
set(_fixture "${SOURCE_DIR}/tests/installSpaces")
set(_source_root "${SCRATCH_DIR}/fixture src")
set(_build_dir "${SCRATCH_DIR}/build root")
set(_fixture_build "${_build_dir}/fixture-build")
set(_install_prefix "${SCRATCH_DIR}/prefix root")
set(_configure_prefix "${SCRATCH_DIR}/configure prefix")

file(REMOVE_RECURSE "${SCRATCH_DIR}")

function(_fail text)
    message(FATAL_ERROR "${text} (scratch kept at ${SCRATCH_DIR})")
endfunction()

file(MAKE_DIRECTORY "${_source_root}/fixture include"
    "${_build_dir}/runtime-generated/TestMover")
file(WRITE "${_source_root}/fixture.cpp" "// fixture checkout source\n")
file(WRITE "${_source_root}/fixture include/fixture.h" "// fixture checkout header\n")
foreach (_payload moverCatalog.cpp TestMover/TestMoverKernel.cpp
    TestMover/TestMoverSchema.h TestMover/TestMoverRegistration.h)
    file(WRITE "${_build_dir}/runtime-generated/${_payload}"
        "// fixture payload ${_payload}\n")
endforeach()
file(WRITE "${_build_dir}/runtimeSources.txt"
    "${_build_dir}/runtime-generated/moverCatalog.cpp\n${_source_root}/fixture.cpp\n")
file(WRITE "${_build_dir}/runtimeIncludes.txt"
    "${_source_root}/fixture include\n")
file(WRITE "${_build_dir}/evalCoreSources.txt" "${_source_root}/fixture.cpp\n")
file(WRITE "${_build_dir}/evalCoreIncludes.txt"
    "${_source_root}/fixture include\n")

execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${_fixture}" -B "${_fixture_build}"
        -G "${GENERATOR}"
        "-DRIGEXEC_SDK_INSTALL_MODULE=${_module}"
        "-DRIGEXEC_SDK_SOURCE_ROOT=${_source_root}"
        "-DRIGEXEC_SDK_BUILD_DIR=${_build_dir}"
        "-DRIGEXEC_SDK_EXPORT_SCRIPT=${_export}"
        "-DRIGEXEC_INSTALL_DATADIR=${INSTALL_DATADIR}"
        "-DCMAKE_INSTALL_PREFIX=${_configure_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (NOT _rc EQUAL 0)
    _fail("fixture configure failed (${GENERATOR}): ${_out}")
endif()

set(ENV{DESTDIR} "")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${_fixture_build}"
        --prefix "${_install_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (NOT _rc EQUAL 0)
    _fail("spaced install: cmake --install exited ${_rc}: ${_out}")
endif()
if (EXISTS "${_configure_prefix}")
    _fail("spaced install: wrote to the configure prefix ${_configure_prefix} instead of the --prefix override")
endif()

function(_assert_manifest name expected)
    set(_path "${_install_prefix}/${INSTALL_DATADIR}/${name}.txt")
    if (NOT EXISTS "${_path}")
        _fail("spaced install: ${_path} is missing after install")
    endif()
    file(READ "${_path}" _actual)
    if (NOT _actual STREQUAL expected)
        _fail("spaced install: ${_path} holds [${_actual}], expected [${expected}]")
    endif()
endfunction()

_assert_manifest(runtimeSources
    "runtime-generated/moverCatalog.cpp\nfixture.cpp\n")
_assert_manifest(runtimeIncludes "fixture include\n")
_assert_manifest(evalCoreSources "fixture.cpp\n")
_assert_manifest(evalCoreIncludes "fixture include\n")
foreach (_payload moverCatalog.cpp TestMover/TestMoverKernel.cpp
    TestMover/TestMoverSchema.h TestMover/TestMoverRegistration.h)
    if (NOT EXISTS
        "${_install_prefix}/${INSTALL_DATADIR}/runtime-generated/${_payload}")
        _fail("spaced install: payload ${_payload} is missing under ${_install_prefix}/${INSTALL_DATADIR}/runtime-generated")
    endif()
endforeach()
message(STATUS "spaced install: pass (manifests and payload under ${_install_prefix}/${INSTALL_DATADIR})")

file(REMOVE "${_build_dir}/runtime-generated/moverCatalog.cpp")
set(_second_prefix "${SCRATCH_DIR}/prefix root 2")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${_fixture_build}"
        --prefix "${_second_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (_rc EQUAL 0)
    _fail("missing payload: cmake --install exited 0 with moverCatalog.cpp removed")
endif()
if (NOT _out MATCHES "moverCatalog\\.cpp")
    _fail("missing payload: install failed (${_rc}) without naming moverCatalog.cpp: ${_out}")
endif()
message(STATUS "missing payload: pass (cmake --install exited ${_rc} naming moverCatalog.cpp)")

file(REMOVE_RECURSE "${SCRATCH_DIR}")
message(STATUS "SDK install spaces check: pass (${GENERATOR}, CMake ${CMAKE_VERSION})")
