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
# kernels, not to their consumers. A flag the target already carries through
# the build's directory options is not repeated.
function(rigexec_apply_exact_fp_flags target)
    if (NOT TARGET "${target}")
        message(FATAL_ERROR "rigexec_apply_exact_fp_flags: no target ${target}")
    endif ()
    rigexec_get_exact_fp_flags(_rigexec_fp_flags)
    get_target_property(_rigexec_fp_existing "${target}" COMPILE_OPTIONS)
    if (NOT _rigexec_fp_existing)
        set(_rigexec_fp_existing "")
    endif ()
    set(_rigexec_fp_add "")
    foreach (_flag IN LISTS _rigexec_fp_flags)
        if (NOT _flag IN_LIST _rigexec_fp_existing)
            list(APPEND _rigexec_fp_add "${_flag}")
        endif ()
    endforeach ()
    if (_rigexec_fp_add)
        target_compile_options("${target}" PRIVATE ${_rigexec_fp_add})
    endif ()
endfunction()
