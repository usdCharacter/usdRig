# Throwaway project driven by tests/exactFpCompilerWitness.cmake: it applies
# the root project's exact-FP helper to adversarial target shapes and the
# witness driver then audits the real compiler commands and runs the
# numeric probes. WITNESS_MODULE, WITNESS_FAST_FLAGS, WITNESS_FAST_HEAD,
# WITNESS_FAST_TAIL and WITNESS_CONFLICT_FLAG come from the driver.
# WITNESS_IF_LINK, WITNESS_PROP_LINK, WITNESS_BRANCH_LINK,
# WITNESS_UNSUPPORTED_LINK and WITNESS_DEEP_LINK each add a target whose
# only route to Pkg::Fast or Pkg::Quiet is a single link expression: a
# literal IF, a property-computed IF branch, a many-node shallow branch
# tree, an unresolvable computed result, and a chain past the inspection
# cutoff.
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
    AND NOT WITNESS_BRANCH_LINK AND NOT WITNESS_UNSUPPORTED_LINK)
    target_link_libraries(witnessExact PRIVATE $<$<CONFIG:Release>:Pkg::Fast>)
endif ()

if (WITNESS_IF_LINK)
    # An $<IF:cond,true,false> link entry is comma-separated, so an
    # inspection pass that treats the expression body as one value queues
    # neither branch and the fast branch's interface reaches the command
    # line unchallenged in Release. This is the only path to Pkg::Fast and
    # Pkg::Quiet, so the short-conditional witness above cannot mask it.
    add_executable(witnessIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessIf PRIVATE
        $<IF:$<CONFIG:Release>,Pkg::Fast,Pkg::Quiet>)
endif ()

if (WITNESS_PROP_LINK)
    # The Release branch is computed: $<TARGET_PROPERTY:witnessIf,FP_DEP>
    # resolves to Pkg::Fast only through the target property, so an
    # inspection pass that treats the operator's operands as dependencies
    # queues witnessIf and the literal property name and never sees the
    # selected fast interface.
    add_executable(witnessIf
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(witnessIf PROPERTIES FP_DEP "Pkg::Fast")
    target_link_libraries(witnessIf PRIVATE
        $<IF:$<CONFIG:Release>,$<TARGET_PROPERTY:witnessIf,FP_DEP>,Pkg::Quiet>)
endif ()

if (WITNESS_BRANCH_LINK)
    # One IF entry with six expression nodes but no branch nested deeper
    # than four: every branch must be inspected, and a cutoff that counts
    # popped nodes across sibling branches would reject this legal quiet
    # rig as too deeply nested.
    add_executable(witnessBranching
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessBranching PRIVATE
        $<IF:$<CONFIG:Release>,$<1:$<1:$<1:$<1:Pkg::Quiet>>>>,Pkg::Quiet>)
endif ()

if (WITNESS_UNSUPPORTED_LINK)
    # The property name is itself a generator expression, so the selected
    # dependency cannot be resolved statically: the configure must abort
    # naming the consumer and the full original entry, never accept the
    # un-inspected result.
    add_executable(witnessBad
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    set_target_properties(witnessBad PROPERTIES Release_DEP "Pkg::Fast")
    target_link_libraries(witnessBad PRIVATE
        $<TARGET_PROPERTY:witnessBad,$<CONFIG:Release>_DEP>)
endif ()

if (WITNESS_DEEP_LINK)
    # Nested one level past the inspection cutoff: the helper must not
    # silently give up and accept the interface behind the nesting.
    add_executable(witnessDeep
        probe.cpp
        "${_gen_dir}/generatedProbe.cpp")
    target_link_libraries(witnessDeep PRIVATE
        $<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:$<$<CONFIG:Release>:Pkg::Fast>>>>>>)
endif ()

# Own options grouped through SHELL:, so the conflicting flag and the two
# independent definitions share one property entry. The scrub must drop the
# flag while both -D definitions survive as separate options.
add_executable(witnessGrouped
    probe.cpp
    "${_gen_dir}/generatedProbe.cpp")
target_compile_options(witnessGrouped PRIVATE
    "SHELL:${WITNESS_CONFLICT_FLAG} -DKEEP1 -DKEEP2")

if (WITNESS_SOURCE_CONFLICT)
    # Source-level conflicts sit after every target option on the real
    # command line, so they cannot be ordered away and the shared source
    # property must not be rewritten for other consumers: the helper must
    # reject them with a specific diagnostic at configuration time.
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
