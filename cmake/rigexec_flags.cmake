include_guard(GLOBAL)

# Single authority for exact floating-point compilation. A compiler
# resolves a duplicated flag family by its last occurrence, so every
# owned compile target must end its own option list with the canonical
# flags, and no conflicting option may follow them. These functions
# return the flag list once per compiler family and enforce the mode on
# a target; each directory scope calls the walk for its own targets.
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
# canonical list. MSVC installs no directory option: its default is
# already /fp:precise, and a directory /fp: would outrank an ambient
# /fp:fast that vendored scopes expect to keep.
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

# PRIVATE: index of the ">" closing the "$<" that opens the entry, or -1
# when unbalanced. The entry's final character being ">" is not
# sufficient: "$<1:a>$<2:b>" ends with one while its first close sits
# mid-entry.
function(rigexec_fp_genex_close_index entry out_var)
    string(LENGTH "${entry}" _rigexec_gx_len)
    set(_rigexec_gx_i 0)
    set(_rigexec_gx_depth 0)
    set(_rigexec_gx_close -1)
    while (_rigexec_gx_i LESS _rigexec_gx_len)
        string(SUBSTRING "${entry}" ${_rigexec_gx_i} 1 _rigexec_gx_ch)
        if (_rigexec_gx_ch STREQUAL "$")
            string(SUBSTRING "${entry}" ${_rigexec_gx_i} 2 _rigexec_gx_pair)
            if (_rigexec_gx_pair STREQUAL "$<")
                math(EXPR _rigexec_gx_depth "${_rigexec_gx_depth} + 1")
                math(EXPR _rigexec_gx_i "${_rigexec_gx_i} + 1")
            endif ()
        elseif (_rigexec_gx_ch STREQUAL ">" AND _rigexec_gx_depth GREATER 0)
            math(EXPR _rigexec_gx_depth "${_rigexec_gx_depth} - 1")
            if (_rigexec_gx_depth EQUAL 0)
                set(_rigexec_gx_close ${_rigexec_gx_i})
                break ()
            endif ()
        endif ()
        math(EXPR _rigexec_gx_i "${_rigexec_gx_i} + 1")
    endwhile ()
    set(${out_var} ${_rigexec_gx_close} PARENT_SCOPE)
endfunction()

# PRIVATE: TRUE when the entry is one expression: its opening "$<"
# closes at the entry's last character. "$<1:Pkg::>Fast" and
# "$<1:a>$<2:b>" are concatenations whose meaning only expansion reveals.
function(rigexec_fp_genex_complete entry out_var)
    rigexec_fp_genex_close_index("${entry}" _rigexec_gc_idx)
    string(LENGTH "${entry}" _rigexec_gc_len)
    math(EXPR _rigexec_gc_last "${_rigexec_gc_len} - 1")
    if (_rigexec_gc_idx EQUAL _rigexec_gc_last)
        set(${out_var} TRUE PARENT_SCOPE)
    else ()
        set(${out_var} "" PARENT_SCOPE)
    endif ()
endfunction()

# PRIVATE: the canonical flags must finish the target's own option list:
# every literal or generator-expression copy of the family in the
# target's options is removed token by token (a SHELL: grouping prefix
# survives). Options the ordering cannot outrank abort configuration
# instead: per-source COMPILE_OPTIONS, which CMake places last and whose
# properties are shared across targets, and the interface compile options
# of the link closure.
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

# PRIVATE: reason text when a compile-option entry could contribute a
# conflicting floating-point option after expansion, empty when it is
# provably safe. Literal and conditional text is judged as written; a
# $<TARGET_PROPERTY:...> lookup contributes its property's value, judged
# recursively; unresolvable lookups and values past the nesting cutoff
# are rejected.
function(rigexec_fp_option_conflict_reason entry ctx_target depth out_var)
    rigexec_get_exact_fp_flags(_rigexec_fp_canonical)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    set(${out_var} "" PARENT_SCOPE)
    if (${depth} GREATER 5)
        set(${out_var} "option value nested past the inspection cutoff"
            PARENT_SCOPE)
        return ()
    endif ()
    if (entry MATCHES "[$]<(GENEX_EVAL|EVAL_EXPR):")
        set(${out_var} "unevaluable option expression" PARENT_SCOPE)
        return ()
    endif ()
    if (entry MATCHES "${_rigexec_fp_conflict}"
        AND NOT entry IN_LIST _rigexec_fp_canonical)
        set(${out_var} "conflicting option" PARENT_SCOPE)
        return ()
    endif ()
    set(_rigexec_or_tail "${entry}")
    set(_rigexec_or_offset 0)
    while (1)
        string(FIND "${_rigexec_or_tail}" "$<TARGET_PROPERTY:"
            _rigexec_or_pos)
        if (_rigexec_or_pos EQUAL -1)
            break ()
        endif ()
        math(EXPR _rigexec_or_start "${_rigexec_or_offset} + ${_rigexec_or_pos}")
        string(SUBSTRING "${entry}" ${_rigexec_or_start} -1
            _rigexec_or_from)
        rigexec_fp_genex_close_index("${_rigexec_or_from}" _rigexec_or_close)
        if (_rigexec_or_close LESS 0)
            set(${out_var} "unbalanced option expression" PARENT_SCOPE)
            return ()
        endif ()
        math(EXPR _rigexec_or_len "${_rigexec_or_close} + 1")
        string(SUBSTRING "${entry}" ${_rigexec_or_start} ${_rigexec_or_len}
            _rigexec_or_expr)
        rigexec_fp_genex_content("${_rigexec_or_expr}" _rigexec_or_body)
        rigexec_fp_genex_segments("${_rigexec_or_body}" _rigexec_or_args)
        list(LENGTH _rigexec_or_args _rigexec_or_n)
        if (_rigexec_or_n EQUAL 1)
            set(_rigexec_or_owner "${ctx_target}")
            list(GET _rigexec_or_args 0 _rigexec_or_prop)
        elseif (_rigexec_or_n EQUAL 2)
            list(GET _rigexec_or_args 0 _rigexec_or_owner)
            list(GET _rigexec_or_args 1 _rigexec_or_prop)
        else ()
            set(${out_var} "unresolvable option lookup" PARENT_SCOPE)
            return ()
        endif ()
        if (_rigexec_or_prop STREQUAL ""
            OR _rigexec_or_owner MATCHES "[$<>]"
            OR _rigexec_or_prop MATCHES "[$<>]"
            OR NOT TARGET "${_rigexec_or_owner}")
            set(${out_var} "unresolvable option lookup" PARENT_SCOPE)
            return ()
        endif ()
        get_target_property(_rigexec_or_value "${_rigexec_or_owner}"
            "${_rigexec_or_prop}")
        if (_rigexec_or_value
            AND NOT _rigexec_or_value MATCHES "NOTFOUND$")
            math(EXPR _rigexec_or_child "${depth} + 1")
            foreach (_rigexec_or_part IN LISTS _rigexec_or_value)
                rigexec_fp_option_conflict_reason("${_rigexec_or_part}"
                    "${ctx_target}" ${_rigexec_or_child} _rigexec_or_sub)
                if (_rigexec_or_sub)
                    set(${out_var}
                        "via ${_rigexec_or_prop} of ${_rigexec_or_owner}: ${_rigexec_or_sub} (${_rigexec_or_part})"
                        PARENT_SCOPE)
                    return ()
                endif ()
            endforeach ()
        endif ()
        math(EXPR _rigexec_or_offset "${_rigexec_or_start} + ${_rigexec_or_len}")
        string(SUBSTRING "${entry}" ${_rigexec_or_offset} -1
            _rigexec_or_tail)
    endwhile ()
endfunction()

# PRIVATE: options no ordering can outrank abort configuration naming
# the consuming target, option source and entry: per-source
# COMPILE_OPTIONS and the INTERFACE_COMPILE_OPTIONS of the transitive
# link closure, each judged by its effective value. Link entries are
# unwrapped to their real dependencies; entries not known to name a
# target and chains past the per-branch nesting cutoff are rejected
# naming the full entry.
function(rigexec_fp_reject_unrankable_conflicts target)
    rigexec_get_exact_fp_flags(_rigexec_fp_canonical)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    set(_rigexec_fp_found "")
    get_target_property(_rigexec_fp_srcs "${target}" SOURCES)
    foreach (_rigexec_fp_src IN LISTS _rigexec_fp_srcs)
        get_source_file_property(_rigexec_fp_sopts "${_rigexec_fp_src}"
            COMPILE_OPTIONS)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_sopts)
            rigexec_fp_option_conflict_reason("${_rigexec_fp_flag}"
                "${target}" 1 _rigexec_fp_reason)
            if (_rigexec_fp_reason)
                list(APPEND _rigexec_fp_found
                    "source ${_rigexec_fp_src}: ${_rigexec_fp_flag} (${_rigexec_fp_reason})")
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
        if (NOT _rigexec_fp_lib MATCHES "^[$]<"
            AND _rigexec_fp_lib MATCHES "[$]<")
            # "Pkg::$<1:Fast>" expands to a dependency the raw entry
            # never spells.
            list(APPEND _rigexec_fp_found
                "${target}: mixed literal and expression link entry ${_rigexec_fp_lib}")
            continue ()
        endif ()
        if (_rigexec_fp_lib MATCHES "^[$]<")
            # The local queue carries (depth, expression) pairs so the
            # cutoff bounds one branch's nesting, not the total node
            # count across sibling branches.
            set(_rigexec_fp_local "1;${_rigexec_fp_lib}")
            while (_rigexec_fp_local)
                list(POP_FRONT _rigexec_fp_local _rigexec_fp_depth
                    _rigexec_fp_probe)
                if (NOT _rigexec_fp_probe MATCHES "^[$]<")
                    if (_rigexec_fp_probe MATCHES "[$]<")
                        # A mixed branch ("Pkg::$<1:Fast>") selects a
                        # dependency no listed name reveals.
                        list(APPEND _rigexec_fp_found
                            "${target}: mixed literal and expression link result ${_rigexec_fp_probe} in ${_rigexec_fp_lib}")
                    elseif (TARGET "${_rigexec_fp_probe}")
                        list(APPEND _rigexec_fp_queue "${_rigexec_fp_probe}")
                    endif ()
                    continue ()
                endif ()
                if (_rigexec_fp_depth GREATER 5)
                    list(APPEND _rigexec_fp_found
                        "${target}: un-inspectable link expression ${_rigexec_fp_lib}")
                    continue ()
                endif ()
                rigexec_fp_genex_complete("${_rigexec_fp_probe}"
                    _rigexec_fp_whole)
                if (NOT _rigexec_fp_whole)
                    # "$<1:Pkg::>Fast" names a dependency only after
                    # expansion; its leading expression alone names none.
                    if (_rigexec_fp_probe STREQUAL _rigexec_fp_lib)
                        list(APPEND _rigexec_fp_found
                            "${target}: concatenated link expression ${_rigexec_fp_probe}")
                    else ()
                        list(APPEND _rigexec_fp_found
                            "${target}: concatenated link expression ${_rigexec_fp_probe} in ${_rigexec_fp_lib}")
                    endif ()
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
                    # dependency is the property's value. The one-operand
                    # form reads it from the consuming target; a
                    # non-literal owner or name is unresolvable.
                    list(LENGTH _rigexec_fp_segs _rigexec_fp_nsegs)
                    if (_rigexec_fp_nsegs EQUAL 1)
                        set(_rigexec_fp_owner "${target}")
                        list(GET _rigexec_fp_segs 0 _rigexec_fp_propname)
                    elseif (_rigexec_fp_nsegs EQUAL 2)
                        list(GET _rigexec_fp_segs 0 _rigexec_fp_owner)
                        list(GET _rigexec_fp_segs 1 _rigexec_fp_propname)
                    else ()
                        list(APPEND _rigexec_fp_found
                            "${target}: unresolvable link expression ${_rigexec_fp_lib}")
                        continue ()
                    endif ()
                    if ("${_rigexec_fp_owner}" MATCHES "[$<>]"
                        OR "${_rigexec_fp_propname}" MATCHES "[$<>]")
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
                    # The operator produces text, not a dependency name.
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
            rigexec_fp_option_conflict_reason("${_rigexec_fp_flag}"
                "${target}" 1 _rigexec_fp_reason)
            if (_rigexec_fp_reason)
                list(APPEND _rigexec_fp_found
                    "${_rigexec_fp_lib}: ${_rigexec_fp_flag} (${_rigexec_fp_reason})")
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
# directory scope; the directory policy is inherited by subdirectories,
# this walk is not. A vendored target opts out with the RIGEXEC_VENDORED
# target property: an explicit ownership marker, never source-path text.
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
