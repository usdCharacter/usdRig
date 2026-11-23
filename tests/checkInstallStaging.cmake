# Runs real `cmake --install` over the shipped manifest rule. The fixture
# includes the production install rule, so the check cannot drift from it.

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

# Child configures must reach the build tool through forwarding, not PATH
# lookup: Ninja's CMAKE_MAKE_PROGRAM and the VS platform/toolset are not
# re-derivable from the generator name alone.
function(_parent_cache_var name out_var)
    file(STRINGS "${BINARY_DIR}/CMakeCache.txt" _line REGEX "^${name}:")
    string(REGEX REPLACE "^[^=]*=" "" _value "${_line}")
    set(${out_var} "${_value}" PARENT_SCOPE)
endfunction()
_parent_cache_var(CMAKE_MAKE_PROGRAM _make_program)
_parent_cache_var(CMAKE_GENERATOR_PLATFORM _generator_platform)
_parent_cache_var(CMAKE_GENERATOR_TOOLSET _generator_toolset)

set(_forward "")
if (_make_program)
    list(APPEND _forward "-DCMAKE_MAKE_PROGRAM=${_make_program}")
endif()
if (_generator_platform AND GENERATOR MATCHES "Visual Studio|Xcode")
    list(APPEND _forward "-A;${_generator_platform}")
endif()
if (_generator_toolset AND GENERATOR MATCHES "Visual Studio|Xcode")
    list(APPEND _forward "-T;${_generator_toolset}")
endif()

# Resolve the timestamp setter to an absolute path while PATH is still
# whole: the filter below can remove the directory that holds it.
if (WIN32)
    find_program(_mtime_tool powershell.exe)
else()
    find_program(_mtime_tool touch)
endif()
if (NOT _mtime_tool)
    message(FATAL_ERROR
        "checkInstallStaging.cmake needs a timestamp setter (touch or powershell)")
endif()

# Hide every PATH directory holding the make program, so a child configure
# can succeed only through the forwarded CMAKE_MAKE_PROGRAM.
set(_saved_path "$ENV{PATH}")
set(_path_sep ":")
if (WIN32)
    set(_path_sep ";")
endif()
set(_restricted_path "")
if (_make_program)
    get_filename_component(_make_name "${_make_program}" NAME)
    string(REPLACE "${_path_sep}" ";" _path_parts "$ENV{PATH}")
    set(_path_kept "")
    foreach (_part IN LISTS _path_parts)
        if (NOT _part STREQUAL "")
            file(TO_CMAKE_PATH "${_part}" _part_cmake)
            if (NOT EXISTS "${_part_cmake}/${_make_name}"
                AND NOT EXISTS "${_part_cmake}/${_make_name}.exe")
                list(APPEND _path_kept "${_part}")
            endif()
        endif()
    endforeach()
    string(REPLACE ";" "${_path_sep}" _restricted_path "${_path_kept}")
endif()
set(_hide_make FALSE)
if (GENERATOR MATCHES "Ninja" AND _restricted_path)
    set(ENV{PATH} "${_restricted_path}")
    set(_hide_make TRUE)
    message(STATUS "${_make_name} copies removed from PATH for child configures")
endif()

set(_manifests runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
set(_staged "${BINARY_DIR}/sdkManifests")
set(_module "${SOURCE_DIR}/cmake/installManifests.cmake")
set(_export "${SOURCE_DIR}/cmake/exportManifests.cmake")
set(_fixture "${SOURCE_DIR}/tests/installStaging")

file(REMOVE_RECURSE "${SCRATCH_DIR}")

function(_clean_and_fail text)
    file(REMOVE_RECURSE "${SCRATCH_DIR}")
    set(ENV{PATH} "${_saved_path}")
    message(FATAL_ERROR "${text}")
endfunction()

# _rc and _out are the caller's: macros keep them in the scenario's scope.
macro(_configure_fixture build_dir staged_dir configure_args)
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${_fixture}" -B "${build_dir}" -G "${GENERATOR}"
        "-DRIGEXEC_MANIFEST_MODULE=${_module}"
        "-DRIGEXEC_STAGED_MANIFESTS=${staged_dir}"
        "-DRIGEXEC_MANIFEST_INPUT_DIR=${BINARY_DIR}"
        "-DRIGEXEC_MANIFEST_SOURCE_ROOT=${SOURCE_DIR}"
        "-DRIGEXEC_INSTALL_DATADIR=${INSTALL_DATADIR}"
        ${_forward}
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

# cmake -E has no reference-mode touch: passing -r there creates a file
# literally named "-r". The copy is verified, and the tool runs by absolute
# path because the filtered PATH may have removed its directory.
function(_copy_mtime reference target)
    if (WIN32)
        execute_process(COMMAND "${_mtime_tool}" -NoProfile -Command
            "(Get-Item -LiteralPath '${target}').LastWriteTime = (Get-Item -LiteralPath '${reference}').LastWriteTime"
            RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    else ()
        execute_process(COMMAND "${_mtime_tool}" -r "${reference}" "${target}"
            RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    endif ()
    if (NOT _rc EQUAL 0)
        _clean_and_fail("timestamp copy failed for ${target}: ${_out}${_err}")
    endif ()
    file(TIMESTAMP "${reference}" _ref_time "%s" UTC)
    file(TIMESTAMP "${target}" _tgt_time "%s" UTC)
    if (NOT _ref_time STREQUAL _tgt_time)
        _clean_and_fail("timestamp copy: ${target} (${_tgt_time}) still "
            "differs from ${reference} (${_ref_time})")
    endif ()
endfunction()

function(_assert_different_bytes first second label)
    file(READ "${first}" _first_hex HEX)
    file(READ "${second}" _second_hex HEX)
    if (_first_hex STREQUAL _second_hex)
        _clean_and_fail("${label}: ${first} and ${second} already hold the same bytes")
    endif ()
endfunction()

foreach (_manifest IN LISTS _manifests)
    if (NOT EXISTS "${_staged}/${_manifest}.txt")
        _clean_and_fail("no staged ${_staged}/${_manifest}.txt: build the rigexecSdkManifests target first")
    endif()
endforeach()

# DESTDIR joins the staging root with the prefix, dropping a Windows drive
# letter; one expression covers both platforms.
function(_destdir_logical_root stage_root logical_prefix out_var)
    string(REGEX REPLACE "^[A-Za-z]:" "" _stripped "${logical_prefix}")
    set(${out_var} "${stage_root}${_stripped}" PARENT_SCOPE)
endfunction()

macro(_probe probe_dir build_dir configure_args install_args destdir)
    file(WRITE "${probe_dir}/probe.txt" "probe\n")
    file(WRITE "${probe_dir}/CMakeLists.txt"
        "cmake_minimum_required(VERSION 3.26)\n"
        "project(InstallStagingProbe NONE)\n"
        "install(FILES \"${probe_dir}/probe.txt\" DESTINATION \"share/probe\")\n")
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${probe_dir}" -B "${build_dir}" -G "${GENERATOR}"
        ${_forward}
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
_destdir_logical_root("${_probe_dir}/stage" "${_probe_dir}/logical" _probe_root)
if (EXISTS "${_probe_root}/share/probe/probe.txt")
    set(_destdir_root "${_probe_root}")
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

# Unix must honor DESTDIR: a packager's staged tree is the documented route.
# Windows may stage through CMAKE_STAGING_PREFIX instead.
if (UNIX AND NOT _destdir_root)
    _clean_and_fail("DESTDIR staging is not honored by ${GENERATOR} (CMake ${CMAKE_VERSION}) on this Unix host")
endif()

# With the make program unreachable through PATH, a configure without the
# forwarded CMAKE_MAKE_PROGRAM must fail.
if (_hide_make)
    set(_probe_dir "${SCRATCH_DIR}/probeNoMake")
    file(MAKE_DIRECTORY "${_probe_dir}")
    file(WRITE "${_probe_dir}/CMakeLists.txt"
        "cmake_minimum_required(VERSION 3.26)\n"
        "project(NoMakeProbe NONE)\n")
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${_probe_dir}" -B "${_probe_dir}/build" -G "${GENERATOR}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if (_rc EQUAL 0)
        _clean_and_fail("make-program forwarding: a ${GENERATOR} configure succeeded "
        "through PATH although every directory holding ${_make_name} was "
        "removed from it")
    endif()
    if (NOT "${_out}${_err}" MATCHES "MAKE_PROGRAM")
        _clean_and_fail("make-program forwarding: the no-forward configure failed for another reason: ${_out}${_err}")
    endif()
    message(STATUS "make-program forwarding: pass (${_make_name} unreachable through the filtered PATH)")
endif()

_configure_fixture("${SCRATCH_DIR}/prefix" "${_staged}" "")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/prefix" "--prefix;${SCRATCH_DIR}/prefixInstall")
if (NOT _rc EQUAL 0)
    _clean_and_fail("prefix install: cmake --install exited ${_rc}: ${_out}")
endif()
_check_manifests("${SCRATCH_DIR}/prefixInstall" "prefix install")
message(STATUS "prefix install: pass (four manifests in ${SCRATCH_DIR}/prefixInstall/${INSTALL_DATADIR})")

# (b) DESTDIR staging: nothing may reach the logical prefix.
set(_broot "")
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
    _destdir_logical_root("${_stage}" "${_logical}" _broot)
    _check_manifests("${_broot}" "staged install (DESTDIR)")
    message(STATUS "staged install (DESTDIR): pass (${_broot}/${INSTALL_DATADIR}, nothing in ${_logical})")
else()
    if (NOT WIN32)
        _clean_and_fail("staged install (DESTDIR): ${GENERATOR} on Unix did not "
            "honor DESTDIR (probe found no file under ${_probe_root}); the "
            "documented support contract requires it")
    endif()
    message(STATUS "staged install (DESTDIR): unsupported (${GENERATOR}, CMake "
    "${CMAKE_VERSION}); the Windows staging route below covers this platform")
endif()

# (c) CMAKE_STAGING_PREFIX staging: the logical prefix comes from configure,
# not --prefix.
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
        message(STATUS "staged install (CMAKE_STAGING_PREFIX): pass "
            "(${_stage}${_logical}/${INSTALL_DATADIR}, nothing in ${_logical})")
    else()
        _check_manifests("${_stage}" "staged install (CMAKE_STAGING_PREFIX)")
        message(STATUS "staged install (CMAKE_STAGING_PREFIX): pass "
            "(${_stage}/${INSTALL_DATADIR}, nothing in ${_logical})")
    endif()
else()
    _clean_and_fail("staged install (CMAKE_STAGING_PREFIX): unsupported by ${GENERATOR} (CMake ${CMAKE_VERSION})")
endif()

# (d) incomplete export must fail by name, not install the rest.
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

# (e) staged bytes that no longer match their inputs must fail by name.
set(_staleDir "${SCRATCH_DIR}/stale")
foreach (_manifest IN LISTS _manifests)
    file(COPY "${_staged}/${_manifest}.txt" DESTINATION "${_staleDir}")
endforeach()
file(APPEND "${_staleDir}/runtimeSources.txt" "libs/rigExecMath/staleEntry.cpp\n")
_configure_fixture("${SCRATCH_DIR}/staleBuild" "${_staleDir}" "")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/staleBuild" "--prefix;${SCRATCH_DIR}/staleInstall")
if (_rc EQUAL 0)
    _clean_and_fail("stale export: cmake --install exited 0 with a staged "
        "runtimeSources.txt that no longer matches its input")
endif()
if (NOT "${_out}" MATCHES "runtimeSources")
    _clean_and_fail("stale export: install failed (${_rc}) without naming runtimeSources: ${_out}")
endif()
if (EXISTS "${SCRATCH_DIR}/staleInstall/${INSTALL_DATADIR}/runtimeSources.txt")
    _clean_and_fail("stale export: the stale runtimeSources.txt was installed anyway")
endif()
message(STATUS "stale export: pass (cmake --install exited ${_rc} naming runtimeSources.txt)")

# (f) a generated input that disappeared must fail by name even when the
# staged copy survives.
set(_vanished "evalCoreIncludes")
set(_lostInputs "${SCRATCH_DIR}/lostInputs")
set(_keptStaged "${SCRATCH_DIR}/keptStaged")
foreach (_manifest IN LISTS _manifests)
    file(COPY "${_staged}/${_manifest}.txt" DESTINATION "${_keptStaged}")
    # The copied inputs are a snapshot build tree: their producer paths move
    # with it, payload included, so the only inconsistency is the deleted file.
    file(READ "${BINARY_DIR}/${_manifest}.txt" _input_text)
    string(REPLACE "${BINARY_DIR}" "${_lostInputs}" _input_text "${_input_text}")
    file(WRITE "${_lostInputs}/${_manifest}.txt" "${_input_text}")
endforeach()
if (EXISTS "${BINARY_DIR}/runtime-generated")
    file(COPY "${BINARY_DIR}/runtime-generated" DESTINATION "${_lostInputs}")
endif()
file(REMOVE "${_lostInputs}/${_vanished}.txt")
_configure_fixture("${SCRATCH_DIR}/lostBuild" "${_keptStaged}"
    "-DRIGEXEC_MANIFEST_INPUT_DIR=${_lostInputs}")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/lostBuild" "--prefix;${SCRATCH_DIR}/lostInstall")
if (_rc EQUAL 0)
    _clean_and_fail("vanished input: cmake --install exited 0 with "
        "${_lostInputs}/${_vanished}.txt deleted and a stale staged copy "
        "present")
endif()
if (NOT "${_out}" MATCHES "${_vanished}")
    _clean_and_fail("vanished input: install failed (${_rc}) without naming ${_vanished}: ${_out}")
endif()
message(STATUS "vanished input: pass (cmake --install exited ${_rc} naming ${_vanished}.txt)")

# (g) install(FILES) skips a copy whose timestamps say "up to date", so a
# staged change with an equal timestamp must still reach the destination.
set(_gInputs "${SCRATCH_DIR}/refreshInputs")
set(_gStage "${SCRATCH_DIR}/refreshStaged")
function(_produce_manifests inputs_dir stage_dir tag)
    # The fixture snapshot is a build tree, so it carries the reserved payload
    # root the install rule stages, including a nested mover directory.
    file(MAKE_DIRECTORY "${inputs_dir}/runtime-generated/${tag}")
    file(WRITE "${inputs_dir}/runtime-generated/${tag}Catalog.cpp"
        "// staged payload for ${tag}\n")
    file(WRITE "${inputs_dir}/runtime-generated/${tag}/moverSchema.h"
        "// staged payload header for ${tag}\n")
    file(WRITE "${inputs_dir}/runtimeSources.txt"
        "libs/rigExecMath/${tag}.cpp\n${inputs_dir}/runtime-generated/${tag}Catalog.cpp\n")
    file(WRITE "${inputs_dir}/runtimeIncludes.txt" "include/${tag}/runtime\n")
    file(WRITE "${inputs_dir}/evalCoreSources.txt" "libs/rigExecMath/${tag}Core.cpp\n")
    file(WRITE "${inputs_dir}/evalCoreIncludes.txt" "include/${tag}/evalCore\n")
    execute_process(COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_ROOT=${SOURCE_DIR}" "-DBUILD_DIR=${inputs_dir}" "-DOUT_DIR=${stage_dir}"
        -P "${_export}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if (NOT _rc EQUAL 0)
        _clean_and_fail("refresh: producer run (${tag}) failed: ${_out}${_err}")
    endif()
endfunction()

_produce_manifests("${_gInputs}" "${_gStage}" first)
_configure_fixture("${SCRATCH_DIR}/refresh" "${_gStage}"
    "-DRIGEXEC_MANIFEST_INPUT_DIR=${_gInputs}")
set(_gPrefix "${SCRATCH_DIR}/refreshInstall")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/refresh" "--prefix;${_gPrefix}")
if (NOT _rc EQUAL 0)
    _clean_and_fail("refresh: first prefix install exited ${_rc}: ${_out}")
endif()
foreach (_manifest IN LISTS _manifests)
    _assert_identical("${_gStage}/${_manifest}.txt"
        "${_gPrefix}/${INSTALL_DATADIR}/${_manifest}.txt" "refresh install")
endforeach()

_produce_manifests("${_gInputs}" "${_gStage}" second)
# Equal mtime with different bytes is exactly the state install(FILES)
# calls "Up-to-date"; prove it before reinstalling, or the scenario
# proves nothing.
foreach (_manifest IN LISTS _manifests)
    _copy_mtime("${_gPrefix}/${INSTALL_DATADIR}/${_manifest}.txt"
        "${_gStage}/${_manifest}.txt")
    _assert_different_bytes("${_gStage}/${_manifest}.txt"
        "${_gPrefix}/${INSTALL_DATADIR}/${_manifest}.txt" "refresh precondition")
endforeach()
message(STATUS "refresh: staged mtimes match the installed copies and their bytes differ")
_install("${SCRATCH_DIR}/refresh" "--prefix;${_gPrefix}")
if (NOT _rc EQUAL 0)
    _clean_and_fail("refresh: reinstall over the same prefix exited ${_rc}: ${_out}")
endif()
foreach (_manifest IN LISTS _manifests)
    _assert_identical("${_gStage}/${_manifest}.txt"
        "${_gPrefix}/${INSTALL_DATADIR}/${_manifest}.txt" "reinstall over the same prefix")
endforeach()
if (_broot)
    # Scenario (c) reassigns _stage/_logical; recompute (b)'s roots here.
    set(_gdStage "${SCRATCH_DIR}/destdirStage")
    set(_gdLogical "${SCRATCH_DIR}/destdirLogical")
    _destdir_logical_root("${_gdStage}" "${_gdLogical}" _gdRoot)
    set(_gStage2 "${SCRATCH_DIR}/refreshStaged2")
    set(_gInputs2 "${SCRATCH_DIR}/refreshInputs2")
    foreach (_pair runtimeSources runtimeIncludes evalCoreSources evalCoreIncludes)
        file(COPY "${_gStage}/${_pair}.txt" DESTINATION "${_gStage2}")
        # The snapshot build tree moves, so its input paths move with it.
        file(READ "${_gInputs}/${_pair}.txt" _moved_input)
        string(REPLACE "${_gInputs}" "${_gInputs2}" _moved_input "${_moved_input}")
        file(WRITE "${_gInputs2}/${_pair}.txt" "${_moved_input}")
    endforeach()
    file(COPY "${_gInputs}/runtime-generated" DESTINATION "${_gInputs2}")
    _configure_fixture("${SCRATCH_DIR}/refreshDest" "${_gStage2}"
        "-DRIGEXEC_MANIFEST_INPUT_DIR=${_gInputs2}")
    set(ENV{DESTDIR} "${_gdStage}")
    _install("${SCRATCH_DIR}/refreshDest" "--prefix;${_gdLogical}")
    set(ENV{DESTDIR} "")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("refresh (DESTDIR): first staged install exited ${_rc}: ${_out}")
    endif()
    file(WRITE "${_gInputs2}/runtimeSources.txt" "libs/rigExecMath/third.cpp\n")
    execute_process(COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_ROOT=${SOURCE_DIR}" "-DBUILD_DIR=${_gInputs2}" "-DOUT_DIR=${_gStage2}"
        -P "${_export}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if (NOT _rc EQUAL 0)
        _clean_and_fail("refresh (DESTDIR): producer rerun failed: ${_out}${_err}")
    endif()
    # A staged install must neither read from nor write to the logical
    # prefix; a sentinel there is checked byte for byte after reinstall.
    file(WRITE "${_gdLogical}/${INSTALL_DATADIR}/runtimeSources.txt" "hostInstalledSentinel\n")
    file(WRITE "${SCRATCH_DIR}/sentinelExpect.txt" "hostInstalledSentinel\n")
    _copy_mtime("${_gdRoot}/${INSTALL_DATADIR}/runtimeSources.txt"
        "${_gStage2}/runtimeSources.txt")
    _assert_different_bytes("${_gStage2}/runtimeSources.txt"
        "${_gdRoot}/${INSTALL_DATADIR}/runtimeSources.txt" "refresh (DESTDIR) precondition")
    set(ENV{DESTDIR} "${_gdStage}")
    _install("${SCRATCH_DIR}/refreshDest" "--prefix;${_gdLogical}")
    set(ENV{DESTDIR} "")
    if (NOT _rc EQUAL 0)
        _clean_and_fail("refresh (DESTDIR): reinstall over the same staging root exited ${_rc}: ${_out}")
    endif()
    _assert_identical("${_gStage2}/runtimeSources.txt"
        "${_gdRoot}/${INSTALL_DATADIR}/runtimeSources.txt" "reinstall over the same staging root")
    _assert_identical("${SCRATCH_DIR}/sentinelExpect.txt"
        "${_gdLogical}/${INSTALL_DATADIR}/runtimeSources.txt" "logical prefix sentinel")
    file(GLOB_RECURSE _logical_files LIST_DIRECTORIES FALSE "${_gdLogical}/*")
    if (NOT "${_logical_files}" STREQUAL "${_gdLogical}/${INSTALL_DATADIR}/runtimeSources.txt")
        _clean_and_fail("refresh (DESTDIR): the reinstall wrote more than "
            "the sentinel into the unstaged prefix ${_gdLogical}: ${_logical_files}")
    endif()
    message(STATUS "refresh: pass (plain prefix and ${_broot} both updated "
        "to the changed staged bytes; the logical prefix kept only its sentinel)")
else()
    message(STATUS "refresh: plain prefix only (no supported DESTDIR staging root)")
endif()

# (h) a reconfigure that changes an input list without rebuilding the
# staged manifest must fail the install, not reinstall old bytes.
set(_hInputs "${SCRATCH_DIR}/reconfInputs")
set(_hStage "${SCRATCH_DIR}/reconfStaged")
_produce_manifests("${_hInputs}" "${_hStage}" first)
_configure_fixture("${SCRATCH_DIR}/reconf" "${_hStage}"
    "-DRIGEXEC_MANIFEST_INPUT_DIR=${_hInputs}")
set(ENV{DESTDIR} "")
_install("${SCRATCH_DIR}/reconf" "--prefix;${SCRATCH_DIR}/reconfInstall")
if (NOT _rc EQUAL 0)
    _clean_and_fail("reconfigure regression: first install exited ${_rc}: ${_out}")
endif()
file(WRITE "${_hInputs}/runtimeSources.txt" "libs/rigExecMath/regenerated.cpp\n")
_configure_fixture("${SCRATCH_DIR}/reconf" "${_hStage}"
    "-DRIGEXEC_MANIFEST_INPUT_DIR=${_hInputs}")
_install("${SCRATCH_DIR}/reconf" "--prefix;${SCRATCH_DIR}/reconfInstall")
if (_rc EQUAL 0)
    _clean_and_fail("reconfigure regression: cmake --install exited 0 after a "
        "reconfigure changed runtimeSources input without a rebuild of the "
        "staged manifest")
endif()
if (NOT "${_out}" MATCHES "runtimeSources")
    _clean_and_fail("reconfigure regression: install failed (${_rc}) without naming runtimeSources: ${_out}")
endif()
message(STATUS "reconfigure regression: pass (cmake --install exited ${_rc} naming runtimeSources.txt)")

set(ENV{PATH} "${_saved_path}")
file(REMOVE_RECURSE "${SCRATCH_DIR}")
message(STATUS "install staging check: pass (${GENERATOR}, CMake ${CMAKE_VERSION})")
