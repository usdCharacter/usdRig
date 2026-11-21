# Driver for ctest test testRigExecExactFpCompiler: an effective-mode
# witness for cmake/rigexec_flags.cmake. It configures and builds a
# throwaway project (tests/exactFpWitness) inside a build directory whose
# path contains "third_party", then audits the compiler commands recorded
# in compile_commands.json and runs numeric probes that only print the
# exact bit patterns when the compiler neither reassociates nor contracts.
# Required: WITNESS_DIR, WITNESS_MODULE, BUILD_ROOT, GENERATOR,
# CXX_COMPILER, CXX_COMPILER_ID.
cmake_minimum_required(VERSION 3.26)
foreach (_req WITNESS_DIR WITNESS_MODULE BUILD_ROOT GENERATOR
         CXX_COMPILER CXX_COMPILER_ID)
    if (NOT DEFINED ${_req})
        message(FATAL_ERROR "exactFpCompilerWitness: ${_req} is required")
    endif ()
endforeach ()

set(_msvc FALSE)
if (CXX_COMPILER_ID STREQUAL "MSVC")
    set(_msvc TRUE)
endif ()
# Generators that build one configuration per invocation place binaries in a
# per-config directory; Visual Studio and Xcode also emit no
# compile_commands.json, leaving only the configure/build legs and the
# numeric probes there.
set(_multi FALSE)
set(_cfgdir "")
if (GENERATOR MATCHES "Visual Studio|Xcode|Ninja Multi-Config")
    set(_multi TRUE)
    set(_cfgdir "/Release")
endif ()
if (_msvc)
    set(_fast_flags "/fp:fast")
    set(_fast_head "/fp:fast")
    set(_fast_tail "/fp:fast")
    set(_conflict_flag "/fp:fast")
    set(_canonical "/fp:precise")
    set(_exe ".exe")
else ()
    set(_fast_flags "-ffp-contract=fast;-ffast-math")
    set(_fast_head "-ffp-contract=fast")
    set(_fast_tail "-ffast-math")
    set(_conflict_flag "-ffast-math")
    set(_canonical "-fno-fast-math;-ffp-contract=off")
    set(_exe "")
endif ()

function(_witness_fail)
    set(_msg "")
    foreach (_part IN LISTS ARGN)
        string(APPEND _msg " ${_part}")
    endforeach ()
    message(FATAL_ERROR "exactFpCompilerWitness:${_msg}")
endfunction()

# Counts non-overlapping literal occurrences.
function(_witness_count text needle out)
    set(_n 0)
    set(_rest "${text}")
    while (1)
        string(FIND "${_rest}" "${needle}" _pos)
        if (_pos EQUAL -1)
            break ()
        endif ()
        math(EXPR _n "${_n} + 1")
        string(LENGTH "${needle}" _len)
        math(EXPR _off "${_pos} + ${_len}")
        string(SUBSTRING "${_rest}" ${_off} -1 _rest)
    endwhile ()
    set(${out} ${_n} PARENT_SCOPE)
endfunction()

function(_witness_configure work)
    set(_extra "")
    if (_multi)
        list(APPEND _extra
            "-DCMAKE_CONFIGURATION_TYPES=Release"
            "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=${work}/Release")
    endif ()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E rm -rf "${work}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -G "${GENERATOR}"
            "-DCMAKE_BUILD_TYPE=Release"
            ${_extra}
            "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
            "-DWITNESS_MODULE=${WITNESS_MODULE}"
            "-DWITNESS_FAST_FLAGS=${_fast_flags}"
            "-DWITNESS_FAST_HEAD=${_fast_head}"
            "-DWITNESS_FAST_TAIL=${_fast_tail}"
            "-DWITNESS_CONFLICT_FLAG=${_conflict_flag}"
            ${ARGN}
            -S "${WITNESS_DIR}" -B "${work}"
        RESULT_VARIABLE _rc
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err)
    set(WITNESS_RC "${_rc}" PARENT_SCOPE)
    set(WITNESS_LOG "${_out}${_err}" PARENT_SCOPE)
endfunction()

function(_witness_audit work)
    if (NOT EXISTS "${work}/compile_commands.json")
        message(STATUS "exactFpCompilerWitness: ${work} emits no "
            "compile_commands.json; command audit unavailable, numeric "
            "probes still run")
        return ()
    endif ()
    file(READ "${work}/compile_commands.json" _cc)
    string(JSON _len LENGTH "${_cc}")
    math(EXPR _last "${_len} - 1")
    set(_owned 0)
    set(_vendored 0)
    set(_generated 0)
    foreach (_i RANGE 0 ${_last})
        string(JSON _cmd GET "${_cc}" ${_i} command)
        string(JSON _file GET "${_cc}" ${_i} file)
        if (_cmd MATCHES "witnessVendored")
            math(EXPR _vendored "${_vendored} + 1")
            # The vendored target keeps its fast family; the canonical
            # flags are not appended, so any directory copy that precedes
            # must stay strictly before the fast entries (last occurrence
            # of the family wins, which is the mode this target keeps).
            foreach (_flag IN LISTS _fast_flags)
                _witness_count("${_cmd}" "${_flag}" _n)
                if (_n LESS 1)
                    _witness_fail(
                        "vendored command lost fast flag ${_flag}: ${_cmd}")
                endif ()
            endforeach ()
            if (NOT _msvc)
                _witness_count("${_cmd}" "-fno-fast-math" _n)
                if (_n GREATER 0)
                    _witness_fail(
                        "vendored command gained -fno-fast-math: ${_cmd}")
                endif ()
                _witness_count("${_cmd}" "-ffp-contract=off" _offCount)
                if (_offCount GREATER 0)
                    string(FIND "${_cmd}" "-ffp-contract=fast" _fpos)
                    string(FIND "${_cmd}" "-ffp-contract=off" _opos)
                    if (_fpos LESS_EQUAL _opos)
                        _witness_fail("vendored contract family order wrong: "
                            "${_cmd}")
                    endif ()
                endif ()
            endif ()
            continue ()
        endif ()
        math(EXPR _owned "${_owned} + 1")
        # Every owned translation unit carries each canonical flag exactly
        # once and no other floating-point-mode option at all, so nothing
        # can follow the canonical settings.
        foreach (_flag IN LISTS _canonical)
            _witness_count("${_cmd}" "${_flag}" _n)
            if (NOT _n EQUAL 1)
                _witness_fail("owned command has ${_n} copies of "
                    "${_flag}: ${_cmd}")
            endif ()
        endforeach ()
        if (_msvc)
            _witness_count("${_cmd}" "/fp:" _n)
            if (NOT _n EQUAL 1)
                _witness_fail("owned command has stray /fp: options: ${_cmd}")
            endif ()
        else ()
            _witness_count("${_cmd}" "-ffp-contract=" _n)
            if (NOT _n EQUAL 1)
                _witness_fail("owned command has stray contract options: "
                    "${_cmd}")
            endif ()
            foreach (_bad "-ffast-math" "-Ofast" "-ffp-contract=fast")
                _witness_count("${_cmd}" "${_bad}" _n)
                if (_n GREATER 0)
                    _witness_fail("owned command kept conflict flag "
                        "${_bad}: ${_cmd}")
                endif ()
            endforeach ()
        endif ()
        # PCH classification by exact target directory and input
        # translation unit: the creation TU compiles the generated PCH
        # source, and every ordinary source of a PCH-using target consumes
        # it. Target-name patterns anchor on ".dir/" so witnessPch never
        # matches witnessPchReuse commands.
        if (NOT _msvc)
            if (_cmd MATCHES "CMakeFiles/(witnessReuse|witnessPchReuse)\\.dir/"
                AND NOT _cmd MATCHES "cmake_pch|autogen")
                _witness_fail("reuse command lacks the PCH include: ${_cmd}")
            endif ()
        else ()
            if (_file MATCHES "cmake_pch\\.(cxx|hxx)$")
                if (NOT _cmd MATCHES "/Yc")
                    _witness_fail("PCH creation TU lacks /Yc: ${_cmd}")
                endif ()
            elseif (_cmd MATCHES "CMakeFiles/(witnessExact|witnessReuse|witnessPch|witnessPchReuse)\\.dir/")
                if (NOT _cmd MATCHES "/Yu")
                    _witness_fail("PCH-consuming TU lacks /Yu: ${_cmd}")
                endif ()
            endif ()
        endif ()
        # Grouped SHELL: options: the conflicting flag was scrubbed from
        # inside the grouping while both definitions survived as separate
        # options of the same command.
        if (_cmd MATCHES "CMakeFiles/witnessGrouped\\.dir/")
            foreach (_keep "-DKEEP1" "-DKEEP2")
                _witness_count("${_cmd}" "${_keep}" _n)
                if (NOT _n EQUAL 1)
                    _witness_fail("grouped command lost ${_keep} "
                        "(${_n} copies): ${_cmd}")
                endif ()
            endforeach ()
        endif ()
        if (_file MATCHES "third_party[\\\\/]+generated[\\\\/]+generatedProbe\\.cpp")
            math(EXPR _generated "${_generated} + 1")
        endif ()
    endforeach ()
    if (_owned LESS 5)
        _witness_fail("only ${_owned} owned entries audited")
    endif ()
    if (NOT _vendored EQUAL 2)
        _witness_fail("expected 2 vendored entries, got ${_vendored}")
    endif ()
    if (_generated LESS 2)
        _witness_fail("generated third_party-named source not audited: "
            "${_generated} entries")
    endif ()
    # The whole build tree lives under a path containing "third_party":
    # relocated builds keep the exact mode for owned targets.
    if (NOT work MATCHES "third_party")
        _witness_fail("audit expected a third_party-named build path")
    endif ()
endfunction()

function(_witness_run_and_check_exact exe)
    if (NOT EXISTS "${exe}${_exe}")
        _witness_fail("missing executable ${exe}${_exe}")
    endif ()
    execute_process(COMMAND "${exe}${_exe}"
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if (NOT _rc EQUAL 0)
        _witness_fail("${exe} exited ${_rc}: ${_err}")
    endif ()
    # Exact mode: no reassociation (x + y - x is x + y, then minus x) and
    # no multiply-add contraction (two rounded steps).
    if (NOT _out MATCHES "probe reassoc 00000000")
        _witness_fail("${exe} reassociated: ${_out}")
    endif ()
    if (NOT _out MATCHES "probe contract 3ec00000")
        _witness_fail("${exe} contracted: ${_out}")
    endif ()
    if (NOT _out MATCHES "generated contract 3ec00000")
        _witness_fail("${exe} contracted the generated TU: ${_out}")
    endif ()
endfunction()

# Leg 1: enforcement through generator-expression target options, PCH
# creation and reuse (two producer/consumer pairs, one behind a conditional
# interface link), a third_party-named generated source, and a build
# directory named like a vendored tree.
set(_build_config "")
if (_multi)
    set(_build_config "--config" "Release")
endif ()
set(_work1 "${BUILD_ROOT}/third_party-build")
_witness_configure("${_work1}")
if (NOT WITNESS_RC EQUAL 0)
    _witness_fail("enforcement leg failed to configure: ${WITNESS_LOG}")
endif ()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${_work1}"
    ${_build_config}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if (NOT _rc EQUAL 0)
    _witness_fail("enforcement leg failed to build: ${_out}${_err}")
endif ()
_witness_audit("${_work1}")
foreach (_exact_exe witnessExact witnessReuse witnessPch witnessPchReuse
                   witnessGrouped)
    _witness_run_and_check_exact("${_work1}${_cfgdir}/${_exact_exe}")
endforeach ()

# The grouped witness must carry BOTH definitions independently: a scrub
# that re-joins tokens without the SHELL: prefix loses them as options and
# these lines disappear.
execute_process(COMMAND "${_work1}${_cfgdir}/witnessGrouped${_exe}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if (NOT _rc EQUAL 0)
    _witness_fail("witnessGrouped exited ${_rc}: ${_err}")
endif ()
if (NOT _out MATCHES "probe keep1 1")
    _witness_fail("witnessGrouped lost KEEP1: ${_out}")
endif ()
if (NOT _out MATCHES "probe keep2 2")
    _witness_fail("witnessGrouped lost KEEP2: ${_out}")
endif ()

# The vendored leg must discriminate: a fast-mode build of the same probes
# prints a different reassociation pattern, so the exact patterns above are
# not trivially achievable by any compiler mode. Reassociation (not FMA
# contraction) is the discriminator: it needs no FMA-capable target and
# applies to both toolchains.
execute_process(COMMAND "${_work1}${_cfgdir}/witnessVendored${_exe}"
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
if (NOT _rc EQUAL 0)
    _witness_fail("witnessVendored exited ${_rc}: ${_err}")
endif ()
if (_out MATCHES "probe reassoc 00000000")
    _witness_fail("witnessVendored did not reassociate: ${_out}")
endif ()

# Leg 2: a conflict owned by a conditionally linked namespaced interface
# target ($<$<CONFIG:Release>:Pkg::Fast>, no unqualified counterpart)
# cannot be scrubbed or ordered away, so configuration must abort with the
# actionable diagnostic naming the flag.
set(_work2 "${BUILD_ROOT}/interface-conflict")
_witness_configure("${_work2}" -DWITNESS_CONFLICT=interface)
if (WITNESS_RC EQUAL 0)
    _witness_fail("interface conflict configured without a diagnostic")
endif ()
if (NOT WITNESS_LOG MATCHES "rigexec exact-FP")
    _witness_fail("interface conflict diagnostic missing:\n${WITNESS_LOG}")
endif ()
if (NOT WITNESS_LOG MATCHES "${_conflict_flag}")
    _witness_fail("interface conflict diagnostic does not name the flag:\n"
        "${WITNESS_LOG}")
endif ()

# Leg 3: a per-source floating-point conflict sits after the target's own
# options on the real command and its property is shared by every target of
# the directory, so the helper must reject it, naming the source, rather
# than rewrite the shared property.
set(_work3 "${BUILD_ROOT}/source-conflict")
_witness_configure("${_work3}" -DWITNESS_SOURCE_CONFLICT=ON)
if (WITNESS_RC EQUAL 0)
    _witness_fail("source conflict configured without a diagnostic")
endif ()
if (NOT WITNESS_LOG MATCHES "source .*probe\\.cpp: ${_fast_head}")
    _witness_fail("source conflict diagnostic does not name the source and "
        "flag:\n${WITNESS_LOG}")
endif ()

message(STATUS "exactFpCompilerWitness: command audit and numeric probes "
    "agree on the exact mode")
