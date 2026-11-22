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

# PRIVATE: regex matching a complete compiler argument that sets the
# floating-point reassociation, contraction, or precision mode. Every
# alternative is anchored, so a match is always one whole argument: an
# argument that merely carries a mode spelling in its value
# (-DFP_LABEL="-ffast-math") is a preprocessor definition, not a mode
# switch, and never matches. Callers classify the arguments an entry spells
# with rigexec_fp_strip_mode_arguments, which also covers a grouped entry
# like "-O2 -ffast-math". Each result-altering sub-option of
# -ffast-math is listed on its own: one spelled after the canonical flags
# restores the loosened mode past them. The canonical exact-FP flags
# themselves match, so a caller that rejects must first exclude an argument
# equal to a canonical flag. The IEEE-safe negations
# (-fno-unsafe-math-optimizations, -fsigned-zeros and friends) match no
# alternative, so a target asking for the safe form is left alone.
function(rigexec_get_exact_fp_mode_regex out_var)
    set(_rigexec_fp_modes
        "-ffast-math"
        "-fno-fast-math"
        "-Ofast"
        "-funsafe-math-optimizations"
        "-fassociative-math"
        "-ffinite-math-only"
        "-freciprocal-math"
        "-fno-signed-zeros"
        "-fno-trapping-math"
        "-ffp-contract=[^ ]*"
        "-ffp-model=[^ ]*"
        "[-/]fp:[^ ]*")
    set(_rigexec_fp_alternatives "")
    foreach (_rigexec_fp_mode IN LISTS _rigexec_fp_modes)
        list(APPEND _rigexec_fp_alternatives "^${_rigexec_fp_mode}$")
    endforeach ()
    list(JOIN _rigexec_fp_alternatives "|" _rigexec_fp_regex)
    set(${out_var} "${_rigexec_fp_regex}" PARENT_SCOPE)
endfunction()

# PRIVATE: rewrite one option entry with each mode-switch argument it spells
# removed, and report the removed spellings. The entry is walked once: a
# leading SHELL: prefix, double-quoted values, and generator-expression
# punctuation and condition text all stay put, so the entry keeps a shape
# CMake accepts ($<cond:> is legal), while only complete switch arguments
# are cut. A quoted value, or a mode spelling glued behind an = inside a
# -D definition, is therefore never a candidate, and each candidate is a
# whole switch a compiler could receive. Callers that reject read the
# removal list and leave the entry alone; callers that rewrite use the
# entry. An entry with no mode switch comes back unchanged.
function(rigexec_fp_strip_mode_arguments entry entry_out found_out)
    rigexec_get_exact_fp_mode_regex(_rigexec_sm_mode)
    set(_rigexec_sm_prefix "")
    set(_rigexec_sm_body "${entry} ")
    if (_rigexec_sm_body MATCHES "^SHELL:")
        set(_rigexec_sm_prefix "SHELL:")
        string(SUBSTRING "${_rigexec_sm_body}" 6 -1 _rigexec_sm_body)
    endif ()
    string(LENGTH "${_rigexec_sm_body}" _rigexec_sm_len)
    set(_rigexec_sm_i 0)
    set(_rigexec_sm_out "")
    set(_rigexec_sm_gap "")
    set(_rigexec_sm_run "")
    set(_rigexec_sm_quote 0)
    set(_rigexec_sm_found "")
    while (_rigexec_sm_i LESS _rigexec_sm_len)
        string(SUBSTRING "${_rigexec_sm_body}" ${_rigexec_sm_i} 1
            _rigexec_sm_ch)
        if (_rigexec_sm_quote)
            string(APPEND _rigexec_sm_out "${_rigexec_sm_ch}")
            if (_rigexec_sm_ch STREQUAL "\"")
                set(_rigexec_sm_quote 0)
            endif ()
            math(EXPR _rigexec_sm_i "${_rigexec_sm_i} + 1")
            continue ()
        endif ()
        if (_rigexec_sm_ch STREQUAL " " OR _rigexec_sm_ch STREQUAL "\t"
            OR _rigexec_sm_ch STREQUAL "<" OR _rigexec_sm_ch STREQUAL ">"
            OR _rigexec_sm_ch STREQUAL "\"")
            if (NOT _rigexec_sm_run STREQUAL "")
                set(_rigexec_sm_head "")
                set(_rigexec_sm_test "${_rigexec_sm_run}")
                if (_rigexec_sm_test MATCHES "^(0:|1:|:)(.+)$")
                    set(_rigexec_sm_head "${CMAKE_MATCH_1}")
                    set(_rigexec_sm_test "${CMAKE_MATCH_2}")
                endif ()
                string(LENGTH "${_rigexec_sm_test}" _rigexec_sm_full)
                string(REGEX REPLACE "[>$]+$" "" _rigexec_sm_arg
                    "${_rigexec_sm_test}")
                string(LENGTH "${_rigexec_sm_arg}" _rigexec_sm_kept)
                math(EXPR _rigexec_sm_glue
                    "${_rigexec_sm_full} - ${_rigexec_sm_kept}")
                if (_rigexec_sm_glue GREATER 0)
                    string(SUBSTRING "${_rigexec_sm_test}"
                        ${_rigexec_sm_kept} -1 _rigexec_sm_tail)
                else ()
                    set(_rigexec_sm_tail "")
                endif ()
                if (_rigexec_sm_arg MATCHES "${_rigexec_sm_mode}")
                    list(APPEND _rigexec_sm_found "${_rigexec_sm_arg}")
                    string(APPEND _rigexec_sm_out
                        "${_rigexec_sm_head}${_rigexec_sm_tail}")
                elseif (_rigexec_sm_out STREQUAL "")
                    string(APPEND _rigexec_sm_out "${_rigexec_sm_run}")
                else ()
                    string(APPEND _rigexec_sm_out
                        "${_rigexec_sm_gap}${_rigexec_sm_run}")
                endif ()
                set(_rigexec_sm_gap "")
                set(_rigexec_sm_run "")
            endif ()
            if (_rigexec_sm_ch STREQUAL " " OR _rigexec_sm_ch STREQUAL "\t")
                string(APPEND _rigexec_sm_gap "${_rigexec_sm_ch}")
            else ()
                string(APPEND _rigexec_sm_out
                    "${_rigexec_sm_gap}${_rigexec_sm_ch}")
                set(_rigexec_sm_gap "")
                if (_rigexec_sm_ch STREQUAL "\"")
                    set(_rigexec_sm_quote 1)
                endif ()
            endif ()
        else ()
            string(APPEND _rigexec_sm_run "${_rigexec_sm_ch}")
        endif ()
        math(EXPR _rigexec_sm_i "${_rigexec_sm_i} + 1")
    endwhile ()
    set(${entry_out} "${_rigexec_sm_prefix}${_rigexec_sm_out}" PARENT_SCOPE)
    set(${found_out} "${_rigexec_sm_found}" PARENT_SCOPE)
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
# target's options is removed argument by argument (a SHELL: grouping
# prefix survives, and a definition that only carries a mode spelling in
# its value stays; a copy of a canonical flag moves without a note).
# Options the ordering cannot outrank abort configuration
# instead: per-source COMPILE_OPTIONS, which CMake places last and whose
# properties are shared across targets, and the interface compile options
# of the link closure.
function(rigexec_apply_exact_fp_flags target)
    if (NOT TARGET "${target}")
        message(FATAL_ERROR "rigexec_apply_exact_fp_flags: no target ${target}")
    endif ()
    rigexec_get_exact_fp_flags(_rigexec_fp_flags)
    rigexec_fp_reject_unrankable_conflicts("${target}")
    get_target_property(_rigexec_fp_own "${target}" COMPILE_OPTIONS)
    if (NOT _rigexec_fp_own)
        set(_rigexec_fp_own "")
    endif ()
    set(_rigexec_fp_kept "")
    foreach (_rigexec_fp_entry IN LISTS _rigexec_fp_own)
        rigexec_fp_strip_mode_arguments("${_rigexec_fp_entry}"
            _rigexec_fp_stripped _rigexec_fp_found)
        set(_rigexec_fp_notable "")
        foreach (_rigexec_fp_mode IN LISTS _rigexec_fp_found)
            if (NOT _rigexec_fp_mode IN_LIST _rigexec_fp_flags)
                list(APPEND _rigexec_fp_notable "${_rigexec_fp_mode}")
            endif ()
        endforeach ()
        if (_rigexec_fp_notable)
            message(STATUS
                "rigexec exact-FP: ${target}: removing floating-point mode "
                "option ${_rigexec_fp_notable} from its own compile options")
        endif ()
        if (NOT _rigexec_fp_stripped STREQUAL "")
            list(APPEND _rigexec_fp_kept "${_rigexec_fp_stripped}")
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

# PRIVATE: the complete compiler arguments an entry that leads with a
# generator expression can produce. A conditional body may contribute
# its payload or nothing; both cases include any trailing text. Unsupported
# operators leave the set unestablished.
function(rigexec_fp_option_arguments entry depth out_var)
    if (NOT entry MATCHES "[$]<")
        set(${out_var} "${entry}" PARENT_SCOPE)
        return ()
    endif ()
    if (${depth} GREATER 5 OR NOT entry MATCHES "^[$]<")
        set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
        return ()
    endif ()
    rigexec_fp_genex_close_index("${entry}" _rigexec_og_close)
    if (_rigexec_og_close LESS 0)
        set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
        return ()
    endif ()
    math(EXPR _rigexec_og_len "${_rigexec_og_close} + 1")
    string(SUBSTRING "${entry}" 0 ${_rigexec_og_len} _rigexec_og_expr)
    string(SUBSTRING "${entry}" ${_rigexec_og_len} -1 _rigexec_og_tail)
    rigexec_fp_genex_operator("${_rigexec_og_expr}" _rigexec_og_op)
    rigexec_fp_genex_content("${_rigexec_og_expr}" _rigexec_og_body)
    math(EXPR _rigexec_og_child "${depth} + 1")
    if (_rigexec_og_op STREQUAL "0")
        rigexec_fp_option_arguments("${_rigexec_og_tail}"
            ${_rigexec_og_child} _rigexec_og_out)
        set(${out_var} "${_rigexec_og_out}" PARENT_SCOPE)
        return ()
    endif ()
    set(_rigexec_og_branches "")
    set(_rigexec_og_nb 0)
    if (_rigexec_og_op STREQUAL "IF")
        rigexec_fp_genex_segments("${_rigexec_og_body}" _rigexec_og_segs)
        list(LENGTH _rigexec_og_segs _rigexec_og_nb)
        if (NOT _rigexec_og_nb EQUAL 3)
            set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
            return ()
        endif ()
    elseif (_rigexec_og_op STREQUAL "1")
        set(_rigexec_og_branches "${_rigexec_og_body}")
        set(_rigexec_og_nb 1)
    elseif (_rigexec_og_op MATCHES "^[$]<")
        rigexec_fp_genex_complete("${_rigexec_og_op}" _rigexec_og_predicate)
        if (NOT _rigexec_og_predicate)
            set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
            return ()
        endif ()
        set(_rigexec_og_branches "${_rigexec_og_body}")
        set(_rigexec_og_nb 1)
    else ()
        set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
        return ()
    endif ()
    if (_rigexec_og_nb LESS 1)
        # A branch list with nothing to contribute still leaves the
        # trailing text as the false-branch argument.
        rigexec_fp_option_arguments("${_rigexec_og_tail}"
            ${_rigexec_og_child} _rigexec_og_out)
        set(${out_var} "${_rigexec_og_out}" PARENT_SCOPE)
        return ()
    endif ()
    set(_rigexec_og_out "")
    set(_rigexec_og_seen 0)
    if (_rigexec_og_op STREQUAL "IF")
        # Empty result branches still yield the trailing text; iterate
        # by index so foreach does not drop them from the list.
        math(EXPR _rigexec_og_last "${_rigexec_og_nb} - 1")
        set(_rigexec_og_i 1)
        while (_rigexec_og_i LESS_EQUAL _rigexec_og_last)
            list(GET _rigexec_og_segs ${_rigexec_og_i} _rigexec_og_branch)
            rigexec_fp_option_arguments("${_rigexec_og_branch}${_rigexec_og_tail}"
                ${_rigexec_og_child} _rigexec_og_sub)
            if (_rigexec_og_sub STREQUAL "RIGEXEC_UNESTABLISHED")
                set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
                return ()
            endif ()
            list(APPEND _rigexec_og_out "${_rigexec_og_sub}")
            math(EXPR _rigexec_og_seen "${_rigexec_og_seen} + 1")
            math(EXPR _rigexec_og_i "${_rigexec_og_i} + 1")
        endwhile ()
    else ()
        rigexec_fp_option_arguments("${_rigexec_og_branches}${_rigexec_og_tail}"
            ${_rigexec_og_child} _rigexec_og_sub)
        if (_rigexec_og_sub STREQUAL "RIGEXEC_UNESTABLISHED")
            set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
            return ()
        endif ()
        list(APPEND _rigexec_og_out "${_rigexec_og_sub}")
        math(EXPR _rigexec_og_seen "${_rigexec_og_seen} + 1")
    endif ()
    if (_rigexec_og_op MATCHES "^[$]<")
        rigexec_fp_option_arguments("${_rigexec_og_tail}"
            ${_rigexec_og_child} _rigexec_og_false)
        if (_rigexec_og_false STREQUAL "RIGEXEC_UNESTABLISHED")
            set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
            return ()
        endif ()
        list(APPEND _rigexec_og_out "${_rigexec_og_false}")
    endif ()
    list(LENGTH _rigexec_og_out _rigexec_og_count)
    if (_rigexec_og_count GREATER 16)
        set(${out_var} "RIGEXEC_UNESTABLISHED" PARENT_SCOPE)
        return ()
    endif ()
    set(${out_var} "${_rigexec_og_out}" PARENT_SCOPE)
endfunction()

# PRIVATE: reason text when a compile-option entry could contribute a
# conflicting floating-point option, empty when it is provably safe. Only
# complete switch arguments count, so a definition that carries a mode
# spelling as its value is never a conflict. A bare $<TARGET_PROPERTY:...>
# entry is judged per value item; a lookup concatenated into text is
# substituted into the entry and the complete compiler argument is judged,
# so a split spelling cannot re-form a conflicting option. An entry leading
# with an expression is accepted only when every complete argument it can
# expand to is judged safe; unresolvable lookups, multi-value
# concatenations, expressions with no established argument set and values
# past the nesting cutoff are rejected naming the full entry.
function(rigexec_fp_option_conflict_reason entry ctx_target depth out_var)
    rigexec_get_exact_fp_flags(_rigexec_fp_canonical)
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
    rigexec_fp_strip_mode_arguments("${entry}" _rigexec_fp_entry_ignored
        _rigexec_fp_entry_args)
    foreach (_rigexec_fp_arg IN LISTS _rigexec_fp_entry_args)
        if (NOT _rigexec_fp_arg IN_LIST _rigexec_fp_canonical)
            set(${out_var} "conflicting option (${_rigexec_fp_arg})"
                PARENT_SCOPE)
            return ()
        endif ()
    endforeach ()
    string(FIND "${entry}" "$<TARGET_PROPERTY:" _rigexec_or_pos)
    if (_rigexec_or_pos EQUAL -1)
        if (NOT entry MATCHES "[$]<")
            return ()
        endif ()
        if (NOT entry MATCHES "^[$]<")
            # Literal text around an expression: the joined argument is
            # not spelled by any part, and no lookup resolves it.
            set(${out_var} "concatenated option expression (${entry})"
                PARENT_SCOPE)
            return ()
        endif ()
        # The entry leads with an expression: judge every complete
        # argument it can produce, and reject the entry itself when an
        # argument cannot be established.
        math(EXPR _rigexec_or_child "${depth} + 1")
        rigexec_fp_option_arguments("${entry}" ${_rigexec_or_child}
            _rigexec_or_args)
        if (_rigexec_or_args STREQUAL "RIGEXEC_UNESTABLISHED")
            set(${out_var} "unestablished option expression (${entry})"
                PARENT_SCOPE)
            return ()
        endif ()
        foreach (_rigexec_or_arg IN LISTS _rigexec_or_args)
            rigexec_fp_option_conflict_reason("${_rigexec_or_arg}"
                "${ctx_target}" ${_rigexec_or_child} _rigexec_or_sub)
            if (_rigexec_or_sub)
                set(${out_var} "${_rigexec_or_sub}" PARENT_SCOPE)
                return ()
            endif ()
        endforeach ()
        return ()
    endif ()
    string(SUBSTRING "${entry}" ${_rigexec_or_pos} -1 _rigexec_or_from)
    rigexec_fp_genex_close_index("${_rigexec_or_from}" _rigexec_or_close)
    if (_rigexec_or_close LESS 0)
        set(${out_var} "unbalanced option expression" PARENT_SCOPE)
        return ()
    endif ()
    math(EXPR _rigexec_or_len "${_rigexec_or_close} + 1")
    string(SUBSTRING "${entry}" ${_rigexec_or_pos} ${_rigexec_or_len}
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
        set(${out_var} "unresolvable option lookup (${_rigexec_or_expr})"
            PARENT_SCOPE)
        return ()
    endif ()
    if (_rigexec_or_prop STREQUAL ""
        OR _rigexec_or_owner MATCHES "[$<>]"
        OR _rigexec_or_prop MATCHES "[$<>]"
        OR _rigexec_or_prop STREQUAL "COMPILE_OPTIONS"
        OR NOT TARGET "${_rigexec_or_owner}")
        set(${out_var} "unresolvable option lookup (${_rigexec_or_expr})"
            PARENT_SCOPE)
        return ()
    endif ()
    get_target_property(_rigexec_or_value "${_rigexec_or_owner}"
        "${_rigexec_or_prop}")
    if (_rigexec_or_value AND _rigexec_or_value MATCHES "NOTFOUND$")
        set(_rigexec_or_value "")
    endif ()
    math(EXPR _rigexec_or_child "${depth} + 1")
    if (entry STREQUAL _rigexec_or_expr)
        foreach (_rigexec_or_part IN LISTS _rigexec_or_value)
            rigexec_fp_option_conflict_reason("${_rigexec_or_part}"
                "${ctx_target}" ${_rigexec_or_child} _rigexec_or_sub)
            if (_rigexec_or_sub)
                set(${out_var}
                    "via ${_rigexec_or_prop} of ${_rigexec_or_owner}: ${_rigexec_or_sub}"
                    PARENT_SCOPE)
                return ()
            endif ()
        endforeach ()
        set(${out_var} "" PARENT_SCOPE)
        return ()
    endif ()
    list(LENGTH _rigexec_or_value _rigexec_or_vn)
    if (_rigexec_or_vn GREATER 1)
        set(${out_var}
            "computed option value cannot form one argument (${_rigexec_or_expr})"
            PARENT_SCOPE)
        return ()
    endif ()
    string(SUBSTRING "${entry}" 0 ${_rigexec_or_pos} _rigexec_or_prefix)
    math(EXPR _rigexec_or_suffix_at "${_rigexec_or_pos} + ${_rigexec_or_len}")
    string(SUBSTRING "${entry}" ${_rigexec_or_suffix_at} -1
        _rigexec_or_suffix)
    set(_rigexec_or_expanded
        "${_rigexec_or_prefix}${_rigexec_or_value}${_rigexec_or_suffix}")
    rigexec_fp_option_conflict_reason("${_rigexec_or_expanded}"
        "${ctx_target}" ${_rigexec_or_child} _rigexec_or_sub)
    if (_rigexec_or_sub)
        set(${out_var}
            "via ${_rigexec_or_prop} of ${_rigexec_or_owner}: ${_rigexec_or_sub}"
            PARENT_SCOPE)
        return ()
    endif ()
    set(${out_var} "" PARENT_SCOPE)
endfunction()

# PRIVATE: CMake stores semicolons inside an interface generator expression
# as list separators. Reassemble only fragments whose opening expression
# has not closed before inspecting its complete option entry.
function(rigexec_fp_option_entries values out_var)
    set(_rigexec_oe_entries "")
    set(_rigexec_oe_pending "")
    foreach (_rigexec_oe_part IN LISTS values)
        if (_rigexec_oe_pending STREQUAL "")
            set(_rigexec_oe_pending "${_rigexec_oe_part}")
        else ()
            string(APPEND _rigexec_oe_pending ";${_rigexec_oe_part}")
        endif ()
        if (_rigexec_oe_pending MATCHES "^[$]<")
            rigexec_fp_genex_close_index("${_rigexec_oe_pending}"
                _rigexec_oe_close)
            if (_rigexec_oe_close LESS 0)
                continue ()
            endif ()
        endif ()
        string(REPLACE ";" "\\;" _rigexec_oe_escaped "${_rigexec_oe_pending}")
        list(APPEND _rigexec_oe_entries "${_rigexec_oe_escaped}")
        set(_rigexec_oe_pending "")
    endforeach ()
    if (NOT _rigexec_oe_pending STREQUAL "")
        string(REPLACE ";" "\\;" _rigexec_oe_escaped "${_rigexec_oe_pending}")
        list(APPEND _rigexec_oe_entries "${_rigexec_oe_escaped}")
    endif ()
    set(${out_var} "${_rigexec_oe_entries}" PARENT_SCOPE)
endfunction()

# PRIVATE: options no ordering can outrank abort configuration naming
# the consuming target, option source and entry: per-source
# COMPILE_OPTIONS and the INTERFACE_COMPILE_OPTIONS of the transitive
# link closure, each judged by its effective value. Link entries are
# unwrapped to their real dependencies; entries not known to name a
# target and chains past the per-branch nesting cutoff are rejected
# naming the full entry.
function(rigexec_fp_reject_unrankable_conflicts target)
    set(_rigexec_fp_found "")
    get_target_property(_rigexec_fp_srcs "${target}" SOURCES)
    foreach (_rigexec_fp_src IN LISTS _rigexec_fp_srcs)
        get_source_file_property(_rigexec_fp_sopts "${_rigexec_fp_src}"
            COMPILE_OPTIONS)
        rigexec_fp_option_entries("${_rigexec_fp_sopts}" _rigexec_fp_entries)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_entries)
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
        rigexec_fp_option_entries("${_rigexec_fp_iopts}" _rigexec_fp_entries)
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_entries)
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
