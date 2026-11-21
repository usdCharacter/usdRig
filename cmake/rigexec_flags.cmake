include_guard(GLOBAL)

# Single authority for exact floating-point compilation. Floating-point
# speed options silently rewrite results, so every owned compile target must
# see the same mode. A compiler resolves a duplicated family by its last
# occurrence, so a conflicting option anywhere after the settings flips the
# mode; a scoped or shared option must not be applied by text. These
# functions return the flag list once per compiler family and enforce the
# mode on a target. Each directory scope that declares compiled targets
# calls the walk for its own targets; CMakeLists.txt states the placement
# rule beside the root walk.
#
# See docs/concepts/baked-vs-dynamic.md.

# Return the exact-FP settings for the active compiler family in out_var.
function(rigexec_get_exact_fp_flags out_var)
    if (MSVC)
        # /fp:precise keeps IEEE semantics; it is already MSVC's default, so
        # the value is a reset against an ambient /fp:fast, not a new mode.
        set(${out_var} "/fp:precise" PARENT_SCOPE)
    else ()
        # -fno-fast-math restores IEEE semantics against an ambient
        # -ffast-math, which would rewrite results.
        set(${out_var} "-fno-fast-math" "-ffp-contract=off" PARENT_SCOPE)
    endif ()
endfunction()

# Directory-scope policy: the contraction family only, derived from the
# canonical list so it cannot drift from it. A scope installs this with
# add_compile_options to hold OpenUSD-style contraction off for every target
# it declares, while a vendored target compiled in that scope can still
# re-declare the fast family as its last own option and keep its codegen.
# MSVC installs no directory option: its default is already /fp:precise, and
# a directory /fp: would outrank an ambient /fp:fast that vendored scopes
# expect to keep.
function(rigexec_get_exact_fp_directory_flags out_var)
    rigexec_get_exact_fp_flags(_rigexec_fp_all)
    set(_rigexec_fp_dir "")
    if (NOT MSVC)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_all)
            if (_rigexec_fp_flag MATCHES "fp-contract")
                list(APPEND _rigexec_fp_dir "${_rigexec_fp_flag}")
            endif ()
        endforeach ()
    endif ()
    set(${out_var} ${_rigexec_fp_dir} PARENT_SCOPE)
endfunction()

# PRIVATE: regex matching any option that changes the floating-point
# reassociation, contraction, or precision mode. The canonical exact-FP flags
# themselves match, so a caller that compares must first exclude entries
# equal to a canonical flag. Entry detection and token filtering both use
# this substring form, so a grouped entry like "-O2 -ffast-math" is caught.
function(rigexec_get_exact_fp_conflict_regex out_var)
    set(${out_var} [[-ffast-math|-Ofast|-ffp-contract=|[/-]fp:]] PARENT_SCOPE)
endfunction()

# PRIVATE: the content of the outermost generator expression, split at the
# first colon outside nested generator expressions: the content of
# $<$<CONFIG:Release>:Pkg::Fast> is Pkg::Fast, not Fast. Returns empty for
# expressions without a content colon.
function(rigexec_fp_genex_content entry out_var)
    string(LENGTH "${entry}" _rigexec_gc_len)
    set(_rigexec_gc_i 2)
    set(_rigexec_gc_depth 0)
    set(_rigexec_gc_content "")
    while (_rigexec_gc_i LESS _rigexec_gc_len)
        string(SUBSTRING "${entry}" ${_rigexec_gc_i} 1 _rigexec_gc_ch)
        if (_rigexec_gc_ch STREQUAL ":" AND _rigexec_gc_depth EQUAL 0)
            math(EXPR _rigexec_gc_start "${_rigexec_gc_i} + 1")
            math(EXPR _rigexec_gc_stop "${_rigexec_gc_len} - ${_rigexec_gc_start} - 1")
            if (_rigexec_gc_stop GREATER 0)
                string(SUBSTRING "${entry}" ${_rigexec_gc_start}
                    ${_rigexec_gc_stop} _rigexec_gc_content)
            endif ()
            break ()
        elseif (_rigexec_gc_ch STREQUAL "$")
            string(SUBSTRING "${entry}" ${_rigexec_gc_i} 2 _rigexec_gc_pair)
            if (_rigexec_gc_pair STREQUAL "$<")
                math(EXPR _rigexec_gc_depth "${_rigexec_gc_depth} + 1")
                math(EXPR _rigexec_gc_i "${_rigexec_gc_i} + 1")
            endif ()
        elseif (_rigexec_gc_ch STREQUAL ">" AND _rigexec_gc_depth GREATER 0)
            math(EXPR _rigexec_gc_depth "${_rigexec_gc_depth} - 1")
        endif ()
        math(EXPR _rigexec_gc_i "${_rigexec_gc_i} + 1")
    endwhile ()
    set(${out_var} "${_rigexec_gc_content}" PARENT_SCOPE)
endfunction()

# PRIVATE: the settings belong to the translation units that instantiate the
# kernels, not to their consumers. A compiler resolves a duplicated family by
# its last occurrence, so the canonical flags must finish the target's own
# option list: every literal or generator-expression copy of the family in
# the target's options is removed token by token, so unrelated options
# grouped in the same entry (including a SHELL: grouping prefix) keep their
# place and order. Options the ordering
# cannot outrank abort configuration instead: per-source COMPILE_OPTIONS,
# which CMake places after the target's own options (and whose properties
# are shared by every target of the directory that sets them, so they must
# not be rewritten for other consumers), and the interface compile options of
# the link closure.
function(rigexec_apply_exact_fp_flags target)
    if (NOT TARGET "${target}")
        message(FATAL_ERROR "rigexec_apply_exact_fp_flags: no target ${target}")
    endif ()
    rigexec_get_exact_fp_flags(_rigexec_fp_flags)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    rigexec_fp_reject_unrankable_conflicts("${target}")
    get_target_property(_rigexec_fp_own "${target}" COMPILE_OPTIONS)
    if (NOT _rigexec_fp_own)
        set(_rigexec_fp_own "")
    endif ()
    set(_rigexec_fp_kept "")
    foreach (_rigexec_fp_entry IN LISTS _rigexec_fp_own)
        if (NOT _rigexec_fp_entry MATCHES "${_rigexec_fp_conflict}")
            list(APPEND _rigexec_fp_kept "${_rigexec_fp_entry}")
            continue ()
        endif ()
        # Keep the non-conflicting tokens of a grouped entry, in order, and
        # keep the SHELL: prefix so the survivors stay separate options.
        set(_rigexec_fp_prefix "")
        set(_rigexec_fp_body "${_rigexec_fp_entry}")
        if (_rigexec_fp_body MATCHES "^SHELL:")
            set(_rigexec_fp_prefix "SHELL:")
            string(SUBSTRING "${_rigexec_fp_body}" 6 -1 _rigexec_fp_body)
        endif ()
        string(REGEX REPLACE "[ \t]+" ";" _rigexec_fp_tokens
            "${_rigexec_fp_body}")
        set(_rigexec_fp_part "")
        foreach (_rigexec_fp_token IN LISTS _rigexec_fp_tokens)
            if (_rigexec_fp_token STREQUAL "")
                continue ()
            endif ()
            if (NOT _rigexec_fp_token MATCHES "${_rigexec_fp_conflict}")
                list(APPEND _rigexec_fp_part "${_rigexec_fp_token}")
            endif ()
        endforeach ()
        if (_rigexec_fp_part)
            list(JOIN _rigexec_fp_part " " _rigexec_fp_join)
            list(APPEND _rigexec_fp_kept "${_rigexec_fp_prefix}${_rigexec_fp_join}")
        endif ()
    endforeach ()
    if (_rigexec_fp_kept)
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS
            ${_rigexec_fp_kept})
    else ()
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS "")
    endif ()
    target_compile_options("${target}" PRIVATE ${_rigexec_fp_flags})
endfunction()

# PRIVATE: the top-level segments of a generator-expression body, split at
# commas and semicolons outside nested expressions: the body
# $<CONFIG:Release>,Pkg::Fast,Pkg::Quiet of an $<IF:...> yields three
# segments so both result branches are inspectable, while a colon inside
# $<CONFIG:Release> is not a boundary.
function(rigexec_fp_genex_segments body out_var)
    string(LENGTH "${body}" _rigexec_gs_len)
    set(_rigexec_gs_i 0)
    set(_rigexec_gs_depth 0)
    set(_rigexec_gs_cur "")
    set(_rigexec_gs_segs "")
    while (_rigexec_gs_i LESS _rigexec_gs_len)
        string(SUBSTRING "${body}" ${_rigexec_gs_i} 1 _rigexec_gs_ch)
        if (_rigexec_gs_ch STREQUAL "$")
            string(SUBSTRING "${body}" ${_rigexec_gs_i} 2 _rigexec_gs_pair)
            if (_rigexec_gs_pair STREQUAL "$<")
                math(EXPR _rigexec_gs_depth "${_rigexec_gs_depth} + 1")
            endif ()
            string(APPEND _rigexec_gs_cur "$")
        elseif ((_rigexec_gs_ch STREQUAL "," OR _rigexec_gs_ch STREQUAL ";")
                AND _rigexec_gs_depth EQUAL 0)
            list(APPEND _rigexec_gs_segs "${_rigexec_gs_cur}")
            set(_rigexec_gs_cur "")
        elseif (_rigexec_gs_ch STREQUAL ">" AND _rigexec_gs_depth GREATER 0)
            math(EXPR _rigexec_gs_depth "${_rigexec_gs_depth} - 1")
            string(APPEND _rigexec_gs_cur ">")
        else ()
            string(APPEND _rigexec_gs_cur "${_rigexec_gs_ch}")
        endif ()
        math(EXPR _rigexec_gs_i "${_rigexec_gs_i} + 1")
    endwhile ()
    list(APPEND _rigexec_gs_segs "${_rigexec_gs_cur}")
    set(${out_var} "${_rigexec_gs_segs}" PARENT_SCOPE)
endfunction()

# PRIVATE: the operator of a generator expression: the text between the
# opening "$<" and the first colon outside nested expressions, so
# "$<IF:...>" yields "IF", "$<TARGET_PROPERTY:tgt,P>" yields
# "TARGET_PROPERTY", and the conditional form "$<$<CONFIG:Release>:lib>"
# yields the predicate "$<CONFIG:Release>" (never a body segment). Empty
# when the expression has no top-level colon.
function(rigexec_fp_genex_operator entry out_var)
    string(LENGTH "${entry}" _rigexec_go_len)
    set(_rigexec_go_i 2)
    set(_rigexec_go_depth 0)
    set(_rigexec_go_op "")
    while (_rigexec_go_i LESS _rigexec_go_len)
        string(SUBSTRING "${entry}" ${_rigexec_go_i} 1 _rigexec_go_ch)
        if (_rigexec_go_ch STREQUAL ":" AND _rigexec_go_depth EQUAL 0)
            break ()
        endif ()
        if (_rigexec_go_ch STREQUAL "$")
            string(SUBSTRING "${entry}" ${_rigexec_go_i} 2 _rigexec_go_pair)
            if (_rigexec_go_pair STREQUAL "$<")
                math(EXPR _rigexec_go_depth "${_rigexec_go_depth} + 1")
            endif ()
        elseif (_rigexec_go_ch STREQUAL ">" AND _rigexec_go_depth GREATER 0)
            math(EXPR _rigexec_go_depth "${_rigexec_go_depth} - 1")
        endif ()
        string(APPEND _rigexec_go_op "${_rigexec_go_ch}")
        math(EXPR _rigexec_go_i "${_rigexec_go_i} + 1")
    endwhile ()
    set(${out_var} "${_rigexec_go_op}" PARENT_SCOPE)
endfunction()

# PRIVATE: floating-point options that no ordering can outrank abort
# configuration with the owning target, flag and source named: per-source
# COMPILE_OPTIONS entries (last on the real command line, and shared by
# every target of the directory that sets them) and the INTERFACE_COMPILE_OPTIONS
# of the direct requirements and their transitive link closure. A conditional
# link entry ($<$<CONFIG:Release>:Pkg::Fast>, $<BUILD_INTERFACE:lib>) still
# names a target whose interface applies in some configuration, so generator
# expressions are unwrapped to their real dependencies: $<IF:...> queues
# both result branches but never its predicate, $<TARGET_PROPERTY:tgt,PROP>
# is resolved to the property's value instead of queueing the lookup's own
# operands, and a computed result that cannot be resolved statically is
# rejected with the consuming target and the full entry named. Nesting is
# measured per branch: many shallow branches all get inspected, while a
# chain deeper than the cutoff is reported with the consuming target and
# the expression named, never accepted. Link-relevant transport operators
# (LINK_ONLY, BUILD_INTERFACE, INSTALL_INTERFACE, $<0:...>, $<1:...>) and
# the conditional form $<cond:...> unwrap; any other operator produces
# text rather than a dependency name and is rejected the same way.
function(rigexec_fp_reject_unrankable_conflicts target)
    rigexec_get_exact_fp_flags(_rigexec_fp_canonical)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    set(_rigexec_fp_found "")
    get_target_property(_rigexec_fp_srcs "${target}" SOURCES)
    foreach (_rigexec_fp_src IN LISTS _rigexec_fp_srcs)
        get_source_file_property(_rigexec_fp_sopts "${_rigexec_fp_src}"
            COMPILE_OPTIONS)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_sopts)
            if (_rigexec_fp_flag MATCHES "${_rigexec_fp_conflict}"
                AND NOT _rigexec_fp_flag IN_LIST _rigexec_fp_canonical)
                list(APPEND _rigexec_fp_found
                    "source ${_rigexec_fp_src}: ${_rigexec_fp_flag}")
            endif ()
        endforeach ()
    endforeach ()
    get_target_property(_rigexec_fp_queue "${target}" LINK_LIBRARIES)
    get_target_property(_rigexec_fp_ifaces "${target}" INTERFACE_LINK_LIBRARIES)
    if (_rigexec_fp_ifaces)
        list(APPEND _rigexec_fp_queue ${_rigexec_fp_ifaces})
    endif ()
    if (NOT _rigexec_fp_queue)
        set(_rigexec_fp_queue "")
    endif ()
    set(_rigexec_fp_seen "")
    set(_rigexec_fp_unwrap_operators
        "LINK_ONLY;BUILD_INTERFACE;INSTALL_INTERFACE;1")
    while (_rigexec_fp_queue)
        list(POP_FRONT _rigexec_fp_queue _rigexec_fp_lib)
        if (_rigexec_fp_lib MATCHES "^\\$<")
            # Unwrap generator expressions to their real dependencies. The
            # local queue carries (depth, expression) pairs so the cutoff
            # bounds one branch's nesting, not the total node count across
            # sibling branches.
            set(_rigexec_fp_local "1;${_rigexec_fp_lib}")
            while (_rigexec_fp_local)
                list(POP_FRONT _rigexec_fp_local _rigexec_fp_depth
                    _rigexec_fp_probe)
                if (NOT _rigexec_fp_probe MATCHES "^\\$<")
                    if (TARGET "${_rigexec_fp_probe}")
                        list(APPEND _rigexec_fp_queue "${_rigexec_fp_probe}")
                    endif ()
                    continue ()
                endif ()
                if (_rigexec_fp_depth GREATER 5)
                    list(APPEND _rigexec_fp_found
                        "${target}: un-inspectable link expression ${_rigexec_fp_lib}")
                    continue ()
                endif ()
                rigexec_fp_genex_content("${_rigexec_fp_probe}"
                    _rigexec_fp_body)
                rigexec_fp_genex_operator("${_rigexec_fp_probe}"
                    _rigexec_fp_op)
                if (_rigexec_fp_body)
                    rigexec_fp_genex_segments("${_rigexec_fp_body}"
                        _rigexec_fp_segs)
                else ()
                    set(_rigexec_fp_segs "")
                endif ()
                set(_rigexec_fp_skip_first 0)
                if (_rigexec_fp_op STREQUAL "IF")
                    # The first segment is the predicate: it selects a
                    # branch, it is never itself a dependency.
                    set(_rigexec_fp_skip_first 1)
                elseif (_rigexec_fp_op STREQUAL "0")
                    # $<0:...> always expands to nothing.
                    set(_rigexec_fp_segs "")
                elseif (_rigexec_fp_op STREQUAL "TARGET_PROPERTY")
                    # The operands name a lookup, not a dependency: the
                    # dependency is the property's value, itself possibly
                    # a generator-expression string. A property name that
                    # is not a literal cannot be resolved statically, so
                    # that form is rejected with the consumer and the
                    # full entry named.
                    list(LENGTH _rigexec_fp_segs _rigexec_fp_nsegs)
                    list(GET _rigexec_fp_segs 0 _rigexec_fp_owner)
                    list(GET _rigexec_fp_segs 1 _rigexec_fp_propname)
                    if (_rigexec_fp_nsegs LESS 2
                        OR "${_rigexec_fp_owner}" MATCHES "[$<>]"
                        OR "${_rigexec_fp_propname}" MATCHES "[$<>]"
                        OR _rigexec_fp_nsegs GREATER 2)
                        # A computed owner or property name, or the
                        # alternate-value form whose literal alternates
                        # can themselves name targets, is not statically
                        # resolvable.
                        list(APPEND _rigexec_fp_found
                            "${target}: unresolvable link expression ${_rigexec_fp_lib}")
                        continue ()
                    endif ()
                    if (TARGET "${_rigexec_fp_owner}")
                        get_target_property(_rigexec_fp_pval
                            "${_rigexec_fp_owner}" "${_rigexec_fp_propname}")
                        if (NOT _rigexec_fp_pval
                            OR _rigexec_fp_pval MATCHES "NOTFOUND$")
                            set(_rigexec_fp_pval "")
                        endif ()
                        math(EXPR _rigexec_fp_child
                            "${_rigexec_fp_depth} + 1")
                        foreach (_rigexec_fp_part IN LISTS _rigexec_fp_pval)
                            list(APPEND _rigexec_fp_local
                                "${_rigexec_fp_child};${_rigexec_fp_part}")
                        endforeach ()
                    endif ()
                    continue ()
                elseif (NOT _rigexec_fp_op MATCHES "^[$]<"
                    AND NOT _rigexec_fp_op IN_LIST
                        _rigexec_fp_unwrap_operators)
                    # The operator produces text (a configuration name, a
                    # file path, a comparison result), not a dependency
                    # name. Accepting it un-inspected would let a
                    # dependency hide behind the text, so reject it with
                    # the consuming target and the full entry named.
                    list(APPEND _rigexec_fp_found
                        "${target}: link expression result not known to name a target: ${_rigexec_fp_lib}")
                    continue ()
                endif ()
                math(EXPR _rigexec_fp_child "${_rigexec_fp_depth} + 1")
                set(_rigexec_fp_idx 0)
                foreach (_rigexec_fp_part IN LISTS _rigexec_fp_segs)
                    math(EXPR _rigexec_fp_idx "${_rigexec_fp_idx} + 1")
                    if (_rigexec_fp_skip_first AND _rigexec_fp_idx EQUAL 1)
                        continue ()
                    endif ()
                    list(APPEND _rigexec_fp_local
                        "${_rigexec_fp_child};${_rigexec_fp_part}")
                endforeach ()
            endwhile ()
            continue ()
        endif ()
        if (_rigexec_fp_lib IN_LIST _rigexec_fp_seen)
            continue ()
        endif ()
        list(APPEND _rigexec_fp_seen "${_rigexec_fp_lib}")
        if (NOT TARGET "${_rigexec_fp_lib}")
            continue ()
        endif ()
        get_target_property(_rigexec_fp_iopts
            "${_rigexec_fp_lib}" INTERFACE_COMPILE_OPTIONS)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_iopts)
            if (_rigexec_fp_flag MATCHES "${_rigexec_fp_conflict}"
                AND NOT _rigexec_fp_flag IN_LIST _rigexec_fp_canonical)
                list(APPEND _rigexec_fp_found
                    "${_rigexec_fp_lib}: ${_rigexec_fp_flag}")
            endif ()
        endforeach ()
        get_target_property(_rigexec_fp_next
            "${_rigexec_fp_lib}" INTERFACE_LINK_LIBRARIES)
        if (_rigexec_fp_next)
            list(APPEND _rigexec_fp_queue ${_rigexec_fp_next})
        endif ()
    endwhile ()
    if (_rigexec_fp_found)
        list(JOIN _rigexec_fp_found "\n  " _rigexec_fp_text)
        message(FATAL_ERROR
            "rigexec exact-FP: target ${target} carries floating-point mode "
            "options or link expressions that CMake can place after the "
            "exact-FP settings:\n  "
            "${_rigexec_fp_text}\nCMake orders per-source COMPILE_OPTIONS "
            "after the target's own options and shared interface options "
            "after both. Remove them, or mark the owning target "
            "RIGEXEC_VENDORED if it must keep its own floating-point "
            "codegen.")
    endif ()
endfunction()

# Apply the exact-FP settings to every compiled target of the calling
# directory scope. The directory option policy is inherited by
# subdirectories, but this walk is not, so each scope calls it for its own
# targets. A vendored target opts out with the RIGEXEC_VENDORED target
# property: exemption is an explicit
# ownership marker, never source-path text, so a generated source under a
# build directory that happens to be named like a vendored tree still gets
# the exact mode.
function(rigexec_apply_exact_fp_flags_to_directory_targets)
    get_property(_rigexec_fp_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    set(_rigexec_fp_done 0)
    set(_rigexec_fp_skipped 0)
    foreach (_rigexec_fp_target IN LISTS _rigexec_fp_targets)
        get_target_property(_rigexec_fp_type "${_rigexec_fp_target}" TYPE)
        if (_rigexec_fp_type STREQUAL "INTERFACE_LIBRARY"
            OR _rigexec_fp_type STREQUAL "UTILITY")
            continue ()
        endif ()
        get_target_property(_rigexec_fp_srcs "${_rigexec_fp_target}" SOURCES)
        if (NOT _rigexec_fp_srcs)
            continue ()
        endif ()
        get_target_property(_rigexec_fp_vendored "${_rigexec_fp_target}"
            RIGEXEC_VENDORED)
        if (_rigexec_fp_vendored)
            math(EXPR _rigexec_fp_skipped "${_rigexec_fp_skipped} + 1")
            continue ()
        endif ()
        rigexec_apply_exact_fp_flags("${_rigexec_fp_target}")
        math(EXPR _rigexec_fp_done "${_rigexec_fp_done} + 1")
    endforeach ()
    message(STATUS "rigexec_apply_exact_fp_flags_to_directory_targets: "
                   "applied to ${_rigexec_fp_done} targets, "
                   "skipped ${_rigexec_fp_skipped}")
endfunction()
