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

# PRIVATE: floating-point options that no ordering can outrank abort
# configuration with the owning target, flag and source named: per-source
# COMPILE_OPTIONS entries (last on the real command line, and shared by
# every target of the directory that sets them) and the INTERFACE_COMPILE_OPTIONS
# of the direct requirements and their transitive link closure. A conditional
# link entry ($<$<CONFIG:Release>:Pkg::Fast>, $<BUILD_INTERFACE:lib>) still
# names a target whose interface applies in some configuration, so generator
# expressions are unwrapped to their full content, namespaced names included.
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
    while (_rigexec_fp_queue)
        list(POP_FRONT _rigexec_fp_queue _rigexec_fp_lib)
        if (_rigexec_fp_lib MATCHES "^\\$<")
            # Unwrap generator expressions to their real content: a
            # conditional link still smuggles its target's interface into
            # the configurations where the condition holds, and the target
            # name may be namespaced or nested in another expression.
            set(_rigexec_fp_probe "${_rigexec_fp_lib}")
            set(_rigexec_fp_depth 0)
            while (_rigexec_fp_probe MATCHES "^\\$<"
                   AND _rigexec_fp_depth LESS 5)
                math(EXPR _rigexec_fp_depth "${_rigexec_fp_depth} + 1")
                rigexec_fp_genex_content("${_rigexec_fp_probe}"
                    _rigexec_fp_body)
                if (NOT _rigexec_fp_body)
                    break ()
                endif ()
                set(_rigexec_fp_probe "")
                foreach (_rigexec_fp_part IN LISTS _rigexec_fp_body)
                    if (TARGET "${_rigexec_fp_part}")
                        list(APPEND _rigexec_fp_queue "${_rigexec_fp_part}")
                    elseif (_rigexec_fp_part MATCHES "^\\$<")
                        set(_rigexec_fp_probe "${_rigexec_fp_part}")
                    endif ()
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
            "options that CMake can place after the exact-FP settings:\n  "
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
