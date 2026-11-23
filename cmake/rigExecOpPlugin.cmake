include_guard(GLOBAL)
include(CMakeParseArguments)
include("${CMAKE_CURRENT_LIST_DIR}/rigexec_flags.cmake")

# The registry ABI the plugin's plugInfo names, read from the
# rigExec/api/registry.h it compiles against; discovery refuses a plugin
# whose plugInfo names another version.
function(_rigexec_op_registry_abi_version out_var)
    if (TARGET rigExecOpCore)
        set(_header "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../libs/rigExec/api/registry.h")
    else()
        get_target_property(_includes rigExec::rigExecOpCore
            INTERFACE_INCLUDE_DIRECTORIES)
        list(GET _includes 0 _include)
        set(_header "${_include}/rigExec/api/registry.h")
    endif()
    file(STRINGS "${_header}" _line
        REGEX "kRigExecRegistryAbiVersion = [0-9]+")
    if (NOT _line MATCHES "kRigExecRegistryAbiVersion = ([0-9]+)")
        message(FATAL_ERROR "No kRigExecRegistryAbiVersion in ${_header}")
    endif()
    set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
endfunction()

# An op library shipped as an OpenUSD plugin, available inside the usdRig
# build and from find_package(rigExec):
#
#   rigexec_add_op_plugin(name SOURCES ...
#       [LIBRARIES ...] [TYPE_LIBRARIES entry ...] [BINDERS]
#       [SCHEMA schema.usda SCHEMA_TYPE type] [PLUGIN_ROOT dir] [NO_INSTALL])
#
# The sources define RIGEXEC_OP_LIBRARY_ENTRY(name), a
# RIGEXEC_TYPE_LIBRARY_ENTRY(entry) per TYPE_LIBRARIES entry, and with
# BINDERS RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY(name). The plugInfo names them;
# rigExecScene discovers the plugin before the op registry initializes. A
# SCHEMA adds the codeless schema the plugin's op is authored with.
function(rigexec_add_op_plugin name)
    cmake_parse_arguments(P "NO_INSTALL;BINDERS"
        "PLUGIN_ROOT;SCHEMA;SCHEMA_TYPE" "SOURCES;LIBRARIES;TYPE_LIBRARIES"
        ${ARGN})
    if (P_UNPARSED_ARGUMENTS OR P_KEYWORDS_MISSING_VALUES OR NOT P_SOURCES)
        message(FATAL_ERROR
            "rigexec_add_op_plugin(${name}) requires SOURCES; optional LIBRARIES, TYPE_LIBRARIES, BINDERS, SCHEMA with SCHEMA_TYPE, PLUGIN_ROOT and NO_INSTALL")
    endif()
    if (NOT name MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
        message(FATAL_ERROR "Op plugin names must be C/C++ identifiers: ${name}")
    endif()
    if ((P_SCHEMA AND NOT P_SCHEMA_TYPE) OR (P_SCHEMA_TYPE AND NOT P_SCHEMA))
        message(FATAL_ERROR "rigexec_add_op_plugin(${name}): SCHEMA and SCHEMA_TYPE go together")
    endif()
    foreach(_directory LIBDIR PLUGINDIR)
        if (NOT DEFINED RIGEXEC_INSTALL_${_directory})
            if (_directory STREQUAL "PLUGINDIR")
                set(RIGEXEC_INSTALL_${_directory} "lib/usd")
            else()
                set(RIGEXEC_INSTALL_${_directory} "lib")
            endif()
        endif()
    endforeach()
    add_library(${name} SHARED ${P_SOURCES})
    target_compile_features(${name} PRIVATE cxx_std_17)
    if (TARGET rigExec::rigExecBuiltins)
        set(_builtins rigExec::rigExecBuiltins)
    else()
        set(_builtins rigExecBuiltins)
    endif()
    target_link_libraries(${name} PRIVATE rigExec::rigExecOpCore ${_builtins}
        ${P_LIBRARIES})
    if (P_BINDERS)
        target_link_libraries(${name} PRIVATE rigExec::rigExec
            rigExec::rigExecScene)
    endif()
    # Discovery finds the entries by name.
    set_target_properties(${name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    if (MSVC)
        target_compile_options(${name} PRIVATE /permissive- /Zc:__cplusplus /bigobj)
        target_compile_definitions(${name} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    rigexec_apply_exact_fp_flags(${name})

    if (P_PLUGIN_ROOT)
        get_filename_component(_plugin_root "${P_PLUGIN_ROOT}" ABSOLUTE
            BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    else()
        set(_plugin_root "${CMAKE_BINARY_DIR}/usd/rigExecOpPlugins")
    endif()
    set(_resources "${_plugin_root}/${name}/$<CONFIG>/resources")
    set_property(TARGET ${name} PROPERTY RIGEXEC_PLUGIN_RESOURCE_DIR "${_resources}")
    file(WRITE "${_plugin_root}/plugInfo.json"
        "{\"Includes\": [\"*/resources/\", \"*/*/resources/\"]}\n")
    set(_types "")
    foreach(_entry IN LISTS P_TYPE_LIBRARIES)
        if (_types)
            string(APPEND _types ", ")
        endif()
        string(APPEND _types "\"${_entry}_RigExecTypeLibrary\"")
    endforeach()
    _rigexec_op_registry_abi_version(_abi)
    set(_info "\"abiVersion\": ${_abi}, \"entry\": \"${name}_RigExecOpLibrary\"")
    if (_types)
        string(APPEND _info ", \"typeEntries\": [${_types}]")
    endif()
    if (P_BINDERS)
        string(APPEND _info ", \"binders\": \"${name}_RigExecExternalOpBinders\"")
    endif()
    set(_includes "")
    if (P_SCHEMA)
        set(_includes "\"Includes\": [\"schema/\"], ")
    endif()
    set(_metadata "{${_includes}\"Plugins\": [{\"Type\": \"library\", \"Name\": \"${name}\", \"Root\": \".\", \"ResourcePath\": \".\", \"LibraryPath\": \"@LIBRARY@\", \"Info\": {\"RigExecOpLibrary\": {${_info}}}}]}\n")
    string(REPLACE "@LIBRARY@" "$<TARGET_FILE:${name}>" _build_metadata "${_metadata}")
    file(GENERATE OUTPUT "${_resources}/plugInfo.json" CONTENT "${_build_metadata}")

    if (P_SCHEMA)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        if (DEFINED USD_INSTALL_DIR)
            set(_usd "${USD_INSTALL_DIR}")
        else()
            set(_usd "${rigExec_USD_INSTALL_DIR}")
        endif()
        set(_generator "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generateMoverSchema.py")
        get_filename_component(_schema "${P_SCHEMA}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        set(_generated "${CMAKE_CURRENT_BINARY_DIR}/${name}-generated")
        file(MAKE_DIRECTORY "${_generated}")
        set(_schema_header "${_generated}/${name}Schema.h")
        set(_schema_outputs "${_schema_header}"
            "${_resources}/schema/generatedSchema.usda"
            "${_resources}/schema/plugInfo.json")
        add_custom_command(OUTPUT ${_schema_outputs}
            COMMAND "${Python3_EXECUTABLE}" -S "${_generator}"
                --usd "${_usd}" --schema "${_schema}" --type "${P_SCHEMA_TYPE}"
                --namespace "${name}Schema" --header "${_schema_header}"
                --resources "${_resources}/schema"
            DEPENDS "${_schema}" "${_generator}"
            VERBATIM)
        add_custom_target(${name}Schema DEPENDS ${_schema_outputs})
        add_dependencies(${name} ${name}Schema)
        target_include_directories(${name} PRIVATE "${_generated}")
    endif()

    if (NOT P_NO_INSTALL)
        set(_install_resources "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecOpPlugins/${name}/resources")
        file(RELATIVE_PATH _relative_lib
            "${CMAKE_INSTALL_PREFIX}/${_install_resources}"
            "${CMAKE_INSTALL_PREFIX}/${RIGEXEC_INSTALL_LIBDIR}")
        string(REPLACE "@LIBRARY@" "${_relative_lib}/$<TARGET_FILE_NAME:${name}>"
            _install_metadata "${_metadata}")
        file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${name}-install/$<CONFIG>/plugInfo.json"
            CONTENT "${_install_metadata}")
        if (APPLE)
            set_target_properties(${name} PROPERTIES INSTALL_RPATH "@loader_path;${rigExec_LIBRARY_DIR}")
        elseif (UNIX)
            set_target_properties(${name} PROPERTIES INSTALL_RPATH "$ORIGIN;${rigExec_LIBRARY_DIR}")
        endif()
        install(TARGETS ${name}
            RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
            LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
            ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}-install/$<CONFIG>/plugInfo.json"
            DESTINATION "${_install_resources}")
        install(FILES "${_plugin_root}/plugInfo.json"
            DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecOpPlugins")
        if (P_SCHEMA)
            install(FILES "${_resources}/schema/generatedSchema.usda"
                "${_resources}/schema/plugInfo.json"
                DESTINATION "${_install_resources}/schema")
        endif()
    endif()
endfunction()
