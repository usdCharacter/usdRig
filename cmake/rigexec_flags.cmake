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

# PRIVATE: the settings belong to the translation units that instantiate the
# kernels, not to their consumers. A compiler resolves a duplicated family by
# its last occurrence, so the canonical flags must finish the target's own
# option list: any earlier literal copy is removed, every other entry keeps
# its place and order.
function(rigexec_apply_exact_fp_flags target)
    if (NOT TARGET "${target}")
        message(FATAL_ERROR "rigexec_apply_exact_fp_flags: no target ${target}")
    endif ()
    rigexec_get_exact_fp_flags(_rigexec_fp_flags)
    get_target_property(_rigexec_fp_own "${target}" COMPILE_OPTIONS)
    if (NOT _rigexec_fp_own)
        set(_rigexec_fp_own "")
    endif ()
    foreach (_rigexec_fp_flag IN LISTS _rigexec_fp_flags)
        list(REMOVE_ITEM _rigexec_fp_own "${_rigexec_fp_flag}")
    endforeach ()
    if (_rigexec_fp_own)
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS ${_rigexec_fp_own})
    else ()
        set_property(TARGET "${target}" PROPERTY COMPILE_OPTIONS "")
    endif ()
    target_compile_options("${target}" PRIVATE ${_rigexec_fp_flags})
endfunction()

# Apply the exact-FP settings to every compiled target of the calling
# directory scope. Subdirectories are not covered by the directory property,
# so each scope calls this for its own targets. Vendored sources keep their
# own codegen, and interface/utility targets have no translation units.
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
        get_target_property(_rigexec_fp_sources "${_rigexec_fp_target}" SOURCES)
        if (_rigexec_fp_sources MATCHES "thirdparty/|third_party/")
            math(EXPR _rigexec_fp_skipped "${_rigexec_fp_skipped} + 1")
            continue ()
        endif ()
        rigexec_apply_exact_fp_flags("${_rigexec_fp_target}")
        math(EXPR _rigexec_fp_applied "${_rigexec_fp_applied} + 1")
    endforeach ()
    message(STATUS "rigexec_apply_exact_fp_flags_to_directory_targets: "
        "applied to ${_rigexec_fp_applied} targets, skipped ${_rigexec_fp_skipped}")
endfunction()
