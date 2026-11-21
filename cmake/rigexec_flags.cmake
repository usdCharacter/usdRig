include_guard(GLOBAL)

# The single authority for the exact floating-point settings: the baked,
# frozen and .rigexec paths agree bitwise only when the compiler neither
# reassociates nor contracts multiply-adds. See
# docs/concepts/baked-vs-dynamic.md.
function(rigexec_get_exact_fp_flags out_var)
    if (MSVC)
        set(${out_var} /fp:precise PARENT_SCOPE)
    else ()
        set(${out_var} -fno-fast-math -ffp-contract=off PARENT_SCOPE)
    endif ()
endfunction()

# Directory-scope policy: the contraction family only, derived from the
# canonical list so it cannot drift from it. A scope installs this with
# add_compile_options to hold OpenUSD-style contraction off for every target
# it declares, while a vendored target compiled in that scope can still
# re-declare the fast family as its last own option and keep its codegen.
function(rigexec_get_exact_fp_directory_flags out_var)
    rigexec_get_exact_fp_flags(_rigexec_fp_all)
    set(_rigexec_fp_dir "")
    foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_all)
        if (_rigexec_fp_flag MATCHES "fp-contract|/fp:")
            list(APPEND _rigexec_fp_dir "${_rigexec_fp_flag}")
        endif ()
    endforeach ()
    set(${out_var} ${_rigexec_fp_dir} PARENT_SCOPE)
endfunction()

# Any option that changes the floating-point reassociation or contraction
# mode. The canonical exact-FP flags themselves match this family, so a
# caller that compares must first exclude entries equal to a canonical flag.
function(rigexec_get_exact_fp_conflict_regex out_var)
    set(${out_var} [[-ffast-math|-Ofast|-ffp-contract=|[/-]fp:]] PARENT_SCOPE)
endfunction()

# PRIVATE: the settings belong to the translation units that instantiate the
# kernels, not to their consumers. A compiler resolves a duplicated family by
# its last occurrence, so the canonical flags must finish the target's own
# option list: every literal or generator-expression copy of the family in
# the target's or its sources' options is removed rather than trusted to
# precede, and every other entry keeps its place and order.
function(rigexec_apply_exact_fp_flags target)
    if (NOT TARGET "${target}")
        message(FATAL_ERROR "rigexec_apply_exact_fp_flags: no target ${target}")
    endif ()
    rigexec_get_exact_fp_flags(_rigexec_fp_flags)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    get_target_property(_rigexec_fp_own "${target}" COMPILE_OPTIONS)
    if (NOT _rigexec_fp_own)
        set(_rigexec_fp_own "")
    endif ()
    set(_rigexec_fp_kept "")
    foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_own)
        if (NOT _rigexec_fp_flag MATCHES "${_rigexec_fp_conflict}")
            list(APPEND _rigexec_fp_kept "${_rigexec_fp_flag}")
        endif ()
    endforeach ()
    if (_rigexec_fp_kept)
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS ${_rigexec_fp_kept})
    else ()
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS "")
    endif ()
    get_target_property(_rigexec_fp_srcs "${target}" SOURCES)
    foreach (_rigexec_fp_src IN LISTS _rigexec_fp_srcs)
        get_source_file_property(_rigexec_fp_sopts "${_rigexec_fp_src}" COMPILE_OPTIONS)
        if (NOT _rigexec_fp_sopts)
            continue ()
        endif ()
        set(_rigexec_fp_skept "")
        foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_sopts)
            if (NOT _rigexec_fp_flag MATCHES "${_rigexec_fp_conflict}")
                list(APPEND _rigexec_fp_skept "${_rigexec_fp_flag}")
            endif ()
        endforeach ()
        set_source_files_properties("${_rigexec_fp_src}" PROPERTIES
            COMPILE_OPTIONS "${_rigexec_fp_skept}")
    endforeach ()
    rigexec_fp_reject_interface_conflicts("${target}")
    target_compile_options("${target}" PRIVATE ${_rigexec_fp_flags})
endfunction()

# PRIVATE: reject floating-point options owned outside the target, which
# CMake may place after the appended canonical flags and cannot be scrubbed:
# the INTERFACE_COMPILE_OPTIONS of the target's direct requirements and
# their transitive interface closure.
function(rigexec_fp_reject_interface_conflicts target)
    rigexec_get_exact_fp_flags(_rigexec_fp_canonical)
    rigexec_get_exact_fp_conflict_regex(_rigexec_fp_conflict)
    get_target_property(_rigexec_fp_queue "${target}" LINK_LIBRARIES)
    get_target_property(_rigexec_fp_ifaces "${target}" INTERFACE_LINK_LIBRARIES)
    if (_rigexec_fp_ifaces)
        list(APPEND _rigexec_fp_queue ${_rigexec_fp_ifaces})
    endif ()
    if (NOT _rigexec_fp_queue)
        return ()
    endif ()
    set(_rigexec_fp_seen "")
    set(_rigexec_fp_found "")
    while (_rigexec_fp_queue)
        list(POP_FRONT _rigexec_fp_queue _rigexec_fp_lib)
        if (_rigexec_fp_lib MATCHES "^\\$<LINK_ONLY:(.+)>$")
            set(_rigexec_fp_lib "${CMAKE_MATCH_1}")
        elseif (_rigexec_fp_lib MATCHES "^\\$")
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
            "rigexec exact-FP: target ${target} links libraries whose "
            "interface compile options conflict with the exact floating-point "
            "mode:\n  ${_rigexec_fp_text}\nCMake can place those options after "
            "the canonical exact-FP flags. Remove them, or mark the owning "
            "target RIGEXEC_VENDORED if it must keep its own floating-point "
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
# the exact mode. Interface/utility targets have no translation units.
function(rigexec_apply_exact_fp_flags_to_directory_targets)
    get_property(_rigexec_fp_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    set(_rigexec_fp_applied 0)
    set(_rigexec_fp_skipped 0)
    foreach (_rigexec_fp_target IN LISTS _rigexec_fp_targets)
        get_target_property(_rigexec_fp_type "${_rigexec_fp_target}" TYPE)
        if (_rigexec_fp_type STREQUAL "INTERFACE_LIBRARY"
            OR _rigexec_fp_type STREQUAL "UTILITY")
            math(EXPR _rigexec_fp_skipped "${_rigexec_fp_skipped} + 1")
            continue ()
        endif ()
        get_target_property(_rigexec_fp_vendored
            "${_rigexec_fp_target}" RIGEXEC_VENDORED)
        if (_rigexec_fp_vendored)
            math(EXPR _rigexec_fp_skipped "${_rigexec_fp_skipped} + 1")
            continue ()
        endif ()
        rigexec_apply_exact_fp_flags("${_rigexec_fp_target}")
        math(EXPR _rigexec_fp_applied "${_rigexec_fp_applied} + 1")
    endforeach ()
    message(STATUS "rigexec_apply_exact_fp_flags_to_directory_targets: "
        "applied to ${_rigexec_fp_applied} targets, skipped ${_rigexec_fp_skipped}")
endfunction()
