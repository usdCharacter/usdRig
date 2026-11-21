# Throwaway project driven by tests/exactFpCompilerWitness.cmake: it
# applies the root project's exact-FP helper to adversarial target shapes;
# the driver audits the real compiler commands and runs the numeric
# probes. WITNESS_MODULE, WITNESS_FAST_FLAGS, WITNESS_FAST_HEAD,
# WITNESS_FAST_TAIL and WITNESS_CONFLICT_FLAG come from the driver, and
# each WITNESS_*_LINK, WITNESS_*_OPTION and WITNESS_MIXED/ADJACENT/
# DEEP/ONE_ARG mode adds a target whose only route to a fast option is
# that one link- or compile-option shape.
cmake_minimum_required(VERSION 3.26)
project(rigexec_exact_fp_witness LANGUAGES CXX)

if (NOT DEFINED WITNESS_MODULE)
    message(FATAL_ERROR "exactFpWitness: WITNESS_MODULE is required")
endif ()
include("${WITNESS_MODULE}")

# Same contraction-family directory policy the root project installs at top
# scope.
rigexec_get_exact_fp_directory_flags(_fp_directory_flags)
add_compile_options(${_fp_directory_flags})

# A generated source under a build-directory component named like a vendored
# tree: only an ownership marker may exempt a target from the exact-FP walk.
set(_gen_dir "${CMAKE_BINARY_DIR}/third_party/generated")
add_custom_command(OUTPUT "${_gen_dir}/generatedProbe.cpp"
    COMMAND "${CMAKE_COMMAND}" "-DOUT=${_gen_dir}/generatedProbe.cpp"
            -P "${CMAKE_CURRENT_LIST_DIR}/genSource.cmake")

add_library(fpIface INTERFACE)
# A namespaced name with no unqualified counterpart: an enforcement pass
# that splits a generator-expression link entry at its last colon sees only
# "Fast" and misses this dependency entirely.
add_library(Pkg::Fast ALIAS fpIface)
add_library(fpQuiet INTERFACE)
add_library(Pkg::Quiet ALIAS fpQuiet)
if (WITNESS_CONFLICT STREQUAL "interface")
    # A conflict owned by a linked interface target cannot be scrubbed or
    # reordered, so the helper must abort configuration. The link itself is
    # conditional: a generator-expression link entry must not smuggle the
    # interface past the check in configurations where it applies.
    target_compile_options(fpIface INTERFACE
        "$<$<CONFIG:Release>:${WITNESS_CONFLICT_FLAG}>")
endif ()

add_executable(witnessExact
    probe.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_precompile_headers(witnessExact PRIVATE probePch.h)
if (NOT WITNESS_IF_LINK AND NOT WITNESS_DEEP_LINK AND NOT WITNESS_PROP_LINK
    AND NOT WITNESS_BRANCH_LINK AND NOT WITNESS_UNSUPPORTED_LINK
    AND NOT WITNESS_MIXED_DIRECT AND NOT WITNESS_MIXED_IF
    AND NOT WITNESS_ONE_ARG_PROP AND NOT WITNESS_ADJACENT_DIRECT
    AND NOT WITNESS_ADJACENT_IF AND NOT WITNESS_PROP_IFACE_OPTION
    AND NOT WITNESS_PROP_SOURCE_OPTION)
    target_link_libraries(witnessExact PRIVATE $<$<CONFIG:Release>:Pkg::Fast>)
endif ()

if (WITNESS_IF_LINK)
    # An $<IF:cond,true,false> link entry: both result branches must be
    # inspected, and this entry is the only route to Pkg::Fast.
    add_executable(witnessIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessIf PRIVATE
        $<IF:$<CONFIG:Release>,Pkg::Fast,Pkg::Quiet>)
endif ()

if (WITNESS_PROP_LINK)
    # The Release branch is computed through a target-property lookup:
    # the selected dependency is the property's value, not the operand
    # names, and it is Pkg::Fast.
    add_executable(witnessIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(witnessIf PROPERTIES FP_DEP "Pkg::Fast")
    target_link_libraries(witnessIf PRIVATE
        $<IF:$<CONFIG:Release>,$<TARGET_PROPERTY:witnessIf,FP_DEP>,Pkg::Quiet>)
endif ()

if (WITNESS_BRANCH_LINK)
    # One IF entry with seven expression nodes but no branch deeper than
    # four, all quiet: the nesting cutoff bounds one branch, not the
    # total node count, so this legal rig must configure.
    add_executable(witnessBranching
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessBranching PRIVATE
        $<IF:$<CONFIG:Release>,$<1:$<1:$<1:Pkg::Quiet>>>,$<1:$<1:$<1:Pkg::Quiet>>>>)
endif ()

if (WITNESS_MIXED_DIRECT)
    # A link entry mixing literal text and an expression:
    # Pkg::$<1:Fast> expands to Pkg::Fast, but the unexpanded entry
    # names no target.
    add_executable(witnessMixed
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessMixed PRIVATE Pkg::$<1:Fast>)
endif ()

if (WITNESS_MIXED_IF)
    # The same expansion hidden in an IF result branch.
    add_executable(witnessMixedIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessMixedIf PRIVATE
        $<IF:$<CONFIG:Release>,Pkg::$<1:Fast>,Pkg::Quiet>)
endif ()

if (WITNESS_ONE_ARG_PROP)
    # The one-operand TARGET_PROPERTY form reads the property from the
    # consuming target: it must be resolved there, not fail on a missing
    # second operand.
    add_executable(witnessOneArg
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(witnessOneArg PROPERTIES FP_DEP "Pkg::Fast")
    target_link_libraries(witnessOneArg PRIVATE $<TARGET_PROPERTY:FP_DEP>)
endif ()

if (WITNESS_ADJACENT_DIRECT)
    # A concatenation whose leading expression closes before the entry
    # ends: "$<1:Pkg::>Fast" expands to Pkg::Fast.
    add_executable(witnessAdjacent
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessAdjacent PRIVATE $<1:Pkg::>Fast)
endif ()

if (WITNESS_ADJACENT_IF)
    # The same concatenation as an IF result branch: the entry itself is
    # complete, so the boundary check must cover nested results too.
    add_executable(witnessAdjacentIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessAdjacentIf PRIVATE
        $<IF:$<CONFIG:Release>,$<1:Pkg::>Fast,Pkg::Quiet>)
endif ()

if (WITNESS_PROP_IFACE_OPTION)
    # The interface target exports a property lookup instead of a literal
    # flag: the raw option entry names no floating-point option, but the
    # lookup expands to FP_MODE's fast value after the consumer's exact
    # options.
    add_executable(witnessPropIface
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(fpIface PROPERTIES FP_MODE "${WITNESS_CONFLICT_FLAG}")
    target_compile_options(fpIface INTERFACE
        "$<TARGET_PROPERTY:fpIface,FP_MODE>")
    target_link_libraries(witnessPropIface PRIVATE fpIface)
endif ()

if (WITNESS_PROP_SOURCE_OPTION)
    # The same lookup supplied through per-source COMPILE_OPTIONS: the
    # shared source property must be judged by its effective value, not
    # its raw text.
    set_target_properties(fpIface PROPERTIES FP_MODE "${WITNESS_CONFLICT_FLAG}")
    set_source_files_properties(probe.cpp PROPERTIES
        COMPILE_OPTIONS "$<TARGET_PROPERTY:fpIface,FP_MODE>")
    add_executable(witnessPropSource
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
endif ()

if (WITNESS_UNSUPPORTED_LINK)
    # The property name is itself a generator expression: the selected
    # dependency cannot be resolved statically, so the configure must
    # abort rather than accept the un-inspected result.
    add_executable(witnessBad
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(witnessBad PROPERTIES Release_DEP "Pkg::Fast")
    target_link_libraries(witnessBad PRIVATE
        $<TARGET_PROPERTY:witnessBad,$<CONFIG:Release>_DEP>)
endif ()

if (WITNESS_DEEP_LINK)
    # A chain nested past the inspection cutoff: it must be reported,
    # never accepted at the limit.
    add_executable(witnessDeep
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessDeep PRIVATE
        $<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:Pkg::Fast>>>>>>)
endif ()

# Own options grouped through SHELL:, so the conflicting flag and the two
# independent definitions share one property entry.
add_executable(witnessGrouped
    probe.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_compile_options(witnessGrouped PRIVATE
    "SHELL:${WITNESS_CONFLICT_FLAG} -DKEEP1 -DKEEP2")

if (WITNESS_SOURCE_CONFLICT)
    # A per-source conflict cannot be ordered away, and its property is
    # shared by every target of the directory: reject, never rewrite.
    set_source_files_properties(probe.cpp PROPERTIES
        COMPILE_OPTIONS "${WITNESS_FAST_HEAD}")
endif ()

add_executable(witnessReuse
    probe.cpp
    reuse.cpp
    "${_gen_dir}/generatedProbe.cpp")
# Own generator-expression floating-point options plus PCH reuse.
target_compile_options(witnessReuse PRIVATE
    "$<$<CONFIG:Release>:${WITNESS_FAST_TAIL}>")
target_precompile_headers(witnessReuse REUSE_FROM witnessExact)

# A second producer/consumer PCH pair behind a conditional interface link
# that carries no options: creation and reuse must both keep the exact
# mode after the walk rewrites the consumer's option list.
add_executable(witnessPch
    probe.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_precompile_headers(witnessPch PRIVATE probePch.h)
target_link_libraries(witnessPch PRIVATE $<BUILD_INTERFACE:fpQuiet>)
add_executable(witnessPchReuse
    probe.cpp
    reuse.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_precompile_headers(witnessPchReuse REUSE_FROM witnessPch)

add_executable(witnessVendored
    probe.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_compile_options(witnessVendored PRIVATE ${WITNESS_FAST_FLAGS})
set_target_properties(witnessVendored PROPERTIES RIGEXEC_VENDORED TRUE)

# The root project's placement invariant: this is the last CMake code in
# this scope that touches a target.
rigexec_apply_exact_fp_flags_to_directory_targets()
