# Relocation transform shared by the build-time export (exportManifests.cmake)
# and the install-time freshness gate (verifyManifests.cmake): entries under
# the rig source root become root-relative, so a host building the staged
# lists joins them with its own root instead of copying build paths.
function(rigexec_relocate_manifest_text source_root out_var entries_var)
    set(_relocated "")
    foreach (_entry IN LISTS ${entries_var})
        if (IS_ABSOLUTE "${_entry}")
            file(RELATIVE_PATH _entry "${source_root}" "${_entry}")
        endif ()
        list(APPEND _relocated "${_entry}")
    endforeach ()
    list(JOIN _relocated "\n" _text)
    set(${out_var} "${_text}\n" PARENT_SCOPE)
endfunction ()
