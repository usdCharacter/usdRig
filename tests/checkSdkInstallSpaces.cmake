# Runs real `cmake --install` over the shipped SDK manifest rule from a
# producer build directory containing spaces into an override prefix
# containing spaces, then requires a missing payload to fail naming that
# file. The fixture project carries the production export and install rules,
# so the check exercises the shipped install rather than a copy of it.
if (NOT SOURCE_DIR OR NOT BINARY_DIR OR NOT SCRATCH_DIR)
    message(FATAL_ERROR
        "checkSdkInstallSpaces.cmake needs SOURCE_DIR, BINARY_DIR and SCRATCH_DIR")
endif ()
if (NOT INSTALL_DATADIR)
    message(FATAL_ERROR
        "checkSdkInstallSpaces.cmake needs INSTALL_DATADIR=... (where the manifests ship)")
endif ()
if (NOT GENERATOR)
    # cmake -P has no generator of its own; reuse the tree under check.
    file(STRINGS "${BINARY_DIR}/CMakeCache.txt" _generator
        REGEX "^CMAKE_GENERATOR:")
    string(REPLACE "CMAKE_GENERATOR:INTERNAL=" "" GENERATOR "${_generator}")
    if (NOT GENERATOR)
        set(GENERATOR "Unix Makefiles")
    endif ()
endif ()

set(_module "${SOURCE_DIR}/cmake/installManifests.cmake")
set(_export "${SOURCE_DIR}/cmake/exportManifests.cmake")
set(_fixture "${SOURCE_DIR}/tests/installSpaces")
set(_source_root "${SCRATCH_DIR}/fixture src")
set(_build_dir "${SCRATCH_DIR}/build root")
set(_staged_dir "${_build_dir}/sdkManifests")
set(_fixture_build "${_build_dir}/fixture-build")
set(_install_prefix "${SCRATCH_DIR}/prefix root")
set(_configure_prefix "${SCRATCH_DIR}/configure prefix")
set(_stage_root "${SCRATCH_DIR}/stage root")

file(REMOVE_RECURSE "${SCRATCH_DIR}")

function(_fail text)
    message(FATAL_ERROR "${text} (scratch kept at ${SCRATCH_DIR})")
endfunction ()

file(MAKE_DIRECTORY "${_source_root}/fixture include"
    "${_build_dir}/runtime-generated/TestMover")
file(WRITE "${_source_root}/fixture.cpp" "// fixture checkout source\n")
file(WRITE "${_source_root}/fixture.cxx" "// fixture payload unit\n")
file(WRITE "${_source_root}/mesh.scalar.cxx" "// fixture multi-dot unit\n")
file(WRITE "${_source_root}/fixture include/fixture.h" "// fixture checkout header\n")
foreach (_payload moverCatalog.cpp TestMover/TestMoverKernel.cpp
    TestMover/TestMoverSchema.h TestMover/TestMoverRegistration.h)
    file(WRITE "${_build_dir}/runtime-generated/${_payload}"
        "// fixture payload ${_payload}\n")
endforeach ()
file(WRITE "${_build_dir}/exactFpFlags.txt" "-fno-fast-math\n-ffp-contract=off\n")
# A build's manifests may name several units in one directory: here
# fixture.cpp and fixture.cxx share a stem, and mesh.scalar.cxx is a multi-dot
# unit beside them. The exported manifest keeps all three spellings distinct
# (asserted below), which is what lets a consumer key an object target by the
# full source filename.
file(WRITE "${_build_dir}/runtimeSources.txt"
    "${_build_dir}/runtime-generated/moverCatalog.cpp\n${_source_root}/fixture.cpp\n${_source_root}/fixture.cxx\n${_source_root}/mesh.scalar.cxx\n")
file(WRITE "${_build_dir}/runtimeIncludes.txt"
    "${_source_root}/fixture include\n")
file(WRITE "${_build_dir}/evalCoreSources.txt" "${_source_root}/fixture.cpp\n")
file(WRITE "${_build_dir}/evalCoreIncludes.txt"
    "${_source_root}/fixture include\n")

function(_configure_fixture build_dir data_dir)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -S "${_fixture}" -B "${build_dir}"
            -G "${GENERATOR}"
            "-DRIGEXEC_SDK_INSTALL_MODULE=${_module}"
            "-DRIGEXEC_SDK_SOURCE_ROOT=${_source_root}"
            "-DRIGEXEC_SDK_BUILD_DIR=${_build_dir}"
            "-DRIGEXEC_SDK_STAGED_DIR=${_staged_dir}"
            "-DRIGEXEC_SDK_FLAGS_FILE=${_build_dir}/exactFpFlags.txt"
            "-DRIGEXEC_INSTALL_DATADIR=${data_dir}"
            "-DCMAKE_INSTALL_PREFIX=${_configure_prefix}"
            ${ARGN}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
    if (NOT _rc EQUAL 0)
        _fail("fixture configure failed (${GENERATOR}): ${_out}")
    endif ()
endfunction ()

function(_build_fixture build_dir)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" --build "${build_dir}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
    if (NOT _rc EQUAL 0)
        _fail("fixture build failed (${GENERATOR}): ${_out}")
    endif ()
endfunction ()

# Every source-manifest entry is a translation unit the consumer compiles, so
# an unsupported final suffix has to be rejected at export, not dropped. The
# suffix list is case-sensitive, and only the FINAL suffix decides the
# language: mesh.scalar.cxx is a C++ unit next to a mesh.cpp.
file(WRITE "${_source_root}/notes.txt" "// not a translation unit\n")
file(WRITE "${_source_root}/Notes.CPP" "// uppercase suffix is not a unit\n")
set(_bad_build "${SCRATCH_DIR}/bad build")
file(MAKE_DIRECTORY "${_bad_build}")
foreach (_bad notes.txt Notes.CPP)
    file(WRITE "${_bad_build}/runtimeSources.txt" "${_bad}\n")
    execute_process(
        COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_ROOT=${_source_root}" "-DBUILD_DIR=${_bad_build}"
            "-DOUT_DIR=${SCRATCH_DIR}/bad sdk"
            -P "${_export}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    string(CONCAT _out "${_out}" "${_err}")
    if (_rc EQUAL 0)
        _fail("unsupported entry: the exporter exported ${_bad} instead of rejecting it")
    endif ()
    if (NOT _out MATCHES "translation unit")
        _fail("unsupported entry: rejection of ${_bad} did not name the contract: ${_out}")
    endif ()
    message(STATUS "unsupported entry: pass (export rejected ${_bad})")
endforeach ()

# A compiled unit in the build tree outside the reserved payload root names a
# file no consumer checkout has, so the exporter must refuse it rather than
# ship an unresolvable entry.
file(MAKE_DIRECTORY "${_bad_build}/other-generated")
file(WRITE "${_bad_build}/runtimeSources.txt"
    "${_bad_build}/other-generated/unit.cpp\n")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_ROOT=${_source_root}" "-DBUILD_DIR=${_bad_build}"
        "-DOUT_DIR=${SCRATCH_DIR}/bad sdk 2"
        -P "${_export}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (_rc EQUAL 0)
    _fail("build-tree entry: the exporter shipped ${_bad_build}/other-generated/unit.cpp")
endif ()
if (NOT _out MATCHES "runtime-generated")
    _fail("build-tree entry: rejection did not name the payload root: ${_out}")
endif ()
message(STATUS "build-tree entry: pass (export refused a unit outside runtime-generated)")

_configure_fixture("${_fixture_build}" "${INSTALL_DATADIR}")
_build_fixture("${_fixture_build}")

set(ENV{DESTDIR} "")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${_fixture_build}"
        --prefix "${_install_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (NOT _rc EQUAL 0)
    _fail("spaced install: cmake --install exited ${_rc}: ${_out}")
endif ()
if (EXISTS "${_configure_prefix}")
    _fail("spaced install: wrote to the configure prefix ${_configure_prefix} instead of the --prefix override")
endif ()

function(_assert_manifest root name expected)
    set(_path "${root}/${INSTALL_DATADIR}/${name}.txt")
    if (NOT EXISTS "${_path}")
        _fail("install: ${_path} is missing")
    endif ()
    file(READ "${_path}" _actual)
    if (NOT _actual STREQUAL expected)
        _fail("install: ${_path} holds [${_actual}], expected [${expected}]")
    endif ()
endfunction ()

function(_assert_sdk root data_dir)
    set(_sdk "${root}/${data_dir}")
    foreach (_name runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
        if (NOT EXISTS "${_sdk}/${_name}.txt")
            _fail("install: ${_sdk}/${_name}.txt is missing")
        endif ()
    endforeach ()
    if (NOT EXISTS "${_sdk}/exactFpFlags.txt")
        _fail("install: ${_sdk}/exactFpFlags.txt is missing")
    endif ()
    foreach (_payload moverCatalog.cpp TestMover/TestMoverKernel.cpp
        TestMover/TestMoverSchema.h TestMover/TestMoverRegistration.h)
        if (NOT EXISTS "${_sdk}/runtime-generated/${_payload}")
            _fail("install: payload ${_payload} is missing under ${_sdk}/runtime-generated")
        endif ()
    endforeach ()
endfunction ()

_assert_sdk("${_install_prefix}" "${INSTALL_DATADIR}")
_assert_manifest("${_install_prefix}" runtimeSources
    "runtime-generated/moverCatalog.cpp\nfixture.cpp\nfixture.cxx\nmesh.scalar.cxx\n")
_assert_manifest("${_install_prefix}" runtimeIncludes "fixture include\n")
_assert_manifest("${_install_prefix}" evalCoreSources "fixture.cpp\n")
_assert_manifest("${_install_prefix}" evalCoreIncludes "fixture include\n")
file(READ "${_install_prefix}/${INSTALL_DATADIR}/exactFpFlags.txt" _flags)
if (NOT _flags MATCHES "fno-fast-math")
    _fail("install: exactFpFlags.txt holds [${_flags}]")
endif ()
message(STATUS "spaced install: pass (manifests, flags and payload under ${_install_prefix}/${INSTALL_DATADIR})")

# DESTDIR staging is the supported POSIX packaging flow: every install rule
# honours it, so the staged tree must hold the payload and the manifests
# beside it while the unstaged prefix stays empty. A fresh prefix keeps the
# first install's files out of the comparison. DESTDIR plus an absolute
# --prefix concatenates two absolute paths, which is not a supported layout on
# Windows, so the ordinary spaced-prefix install above is what runs there.
if (NOT WIN32)
    set(_staged_prefix "${SCRATCH_DIR}/staged prefix")
    set(ENV{DESTDIR} "${_stage_root}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" --install "${_fixture_build}"
            --prefix "${_staged_prefix}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    set(ENV{DESTDIR} "")
    string(CONCAT _out "${_out}" "${_err}")
    if (NOT _rc EQUAL 0)
        _fail("staged install: cmake --install exited ${_rc}: ${_out}")
    endif ()
    _assert_sdk("${_stage_root}${_staged_prefix}" "${INSTALL_DATADIR}")
    # Every install destination honours DESTDIR, so the fresh logical prefix
    # itself must still be absent: one stray manifest, flags file or generated
    # source there means a rule wrote beside the stage root instead of under it.
    if (EXISTS "${_staged_prefix}")
        file(GLOB_RECURSE _stray RELATIVE "${_staged_prefix}" "${_staged_prefix}/*")
        list(JOIN _stray ", " _stray)
        string(CONCAT _why "staged install: DESTDIR staging populated the unstaged "
            "prefix ${_staged_prefix}: ${_stray}")
        _fail("${_why}")
    endif ()
    message(STATUS "staged install: pass (everything under ${_stage_root}${_staged_prefix}/${INSTALL_DATADIR})")
else ()
    message(STATUS "staged install: skipped (DESTDIR staging is POSIX-only)")
endif ()

# CMAKE_STAGING_PREFIX stages by rewriting the install prefix, so the whole set
# must land under the staging directory and nothing under the configured prefix.
set(_staging_prefix "${SCRATCH_DIR}/staging prefix")
_configure_fixture("${SCRATCH_DIR}/staging build" "${INSTALL_DATADIR}"
    "-DCMAKE_STAGING_PREFIX=${_staging_prefix}")
_build_fixture("${SCRATCH_DIR}/staging build")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${SCRATCH_DIR}/staging build"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (NOT _rc EQUAL 0)
    _fail("staging prefix: cmake --install exited ${_rc}: ${_out}")
endif ()
_assert_sdk("${_staging_prefix}" "${INSTALL_DATADIR}")
if (EXISTS "${_configure_prefix}")
    file(GLOB_RECURSE _stray RELATIVE "${_configure_prefix}" "${_configure_prefix}/*")
    list(JOIN _stray ", " _stray)
    _fail("staging prefix: wrote to the configured prefix ${_configure_prefix}: ${_stray}")
endif ()
message(STATUS "staging prefix: pass (everything under ${_staging_prefix}/${INSTALL_DATADIR})")

# The same payload under a non-default, spaced data directory: the consumer
# reads one directory, so flags and payload must travel with the manifests.
set(_alt_data_dir "sdk data")
set(_alt_prefix "${SCRATCH_DIR}/alt prefix")
_configure_fixture("${SCRATCH_DIR}/alt build" "${_alt_data_dir}")
_build_fixture("${SCRATCH_DIR}/alt build")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${SCRATCH_DIR}/alt build"
        --prefix "${_alt_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (NOT _rc EQUAL 0)
    _fail("non-default data directory: cmake --install exited ${_rc}: ${_out}")
endif ()
_assert_sdk("${_alt_prefix}" "${_alt_data_dir}")
if (EXISTS "${_alt_prefix}/${INSTALL_DATADIR}/runtimeSources.txt")
    _fail("non-default data directory: the SDK also landed in ${INSTALL_DATADIR}")
endif ()
message(STATUS "non-default data directory: pass (${_alt_data_dir})")

file(REMOVE "${_build_dir}/runtime-generated/moverCatalog.cpp")
set(_second_prefix "${SCRATCH_DIR}/prefix root 2")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${_fixture_build}"
        --prefix "${_second_prefix}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
string(CONCAT _out "${_out}" "${_err}")
if (_rc EQUAL 0)
    _fail("missing payload: cmake --install exited 0 with moverCatalog.cpp removed")
endif ()
if (NOT _out MATCHES "moverCatalog\\.cpp")
    _fail("missing payload: install failed (${_rc}) without naming moverCatalog.cpp: ${_out}")
endif ()
message(STATUS "missing payload: pass (cmake --install exited ${_rc} naming moverCatalog.cpp)")

file(REMOVE_RECURSE "${SCRATCH_DIR}")
message(STATUS "SDK install spaces check: pass (${GENERATOR}, CMake ${CMAKE_VERSION})")
