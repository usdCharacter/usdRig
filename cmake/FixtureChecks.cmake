# Fixed floating-point captures belong to one compiler/libm configuration.
# Every host also checks semantics and exact backend parity in both CPU modes.
set(_rigexec_platform_goldens OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$" AND
   CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
   CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 13 AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
    set(_rigexec_platform_goldens ON)
endif()

function(rigexec_add_fixture_checks name directory fixture rig)
    set(_source "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/${directory}/${fixture}.usda")
    set(_program "${CMAKE_CURRENT_BINARY_DIR}/${directory}_${fixture}.rigexec")
    set(_bake "${name}Bake_${fixture}")
    add_test(NAME ${_bake} COMMAND rigExecBakeTool
        "${_source}" --rig "${rig}" --time 1 -o "${_program}")
    set_tests_properties(${_bake} PROPERTIES FIXTURES_SETUP ${_bake}
        ENVIRONMENT "RIGEXEC_ENABLE_SIMD=0")

    foreach(_simd 0 1)
        set(_capture "${name}Reference_${fixture}_simd${_simd}")
        set(_reference "${CMAKE_CURRENT_BINARY_DIR}/${directory}_${fixture}_simd${_simd}.reference")
        add_test(NAME ${_capture} COMMAND rigExecPose "${_source}"
            --rig "${rig}" --frames 0,1,2,3,4 --cpu-reference
            --golden-out "${_reference}")
        set_tests_properties(${_capture} PROPERTIES FIXTURES_SETUP ${_capture}
            LABELS "fixture-parity" ENVIRONMENT "RIGEXEC_ENABLE_SIMD=${_simd}")
        foreach(_backend native frozen runtime)
            set(_test "${name}Parity_${fixture}_${_backend}_simd${_simd}")
            add_test(NAME ${_test} COMMAND rigExecPose "${_source}"
                --rig "${rig}" --frames 0,1,2,3,4 --golden-backend ${_backend}
                --golden-program "${_program}" --golden-check "${_reference}")
            set_tests_properties(${_test} PROPERTIES
                FIXTURES_REQUIRED "${_capture};${_bake}" LABELS "fixture-parity"
                ENVIRONMENT "RIGEXEC_ENABLE_SIMD=${_simd}")
        endforeach()
    endforeach()

    if(_rigexec_platform_goldens)
        foreach(_backend native frozen runtime)
            set(_test "${name}Golden_${fixture}_${_backend}")
            add_test(NAME ${_test} COMMAND rigExecPose "${_source}"
                --rig "${rig}" --frames 0,1,2,3,4 --golden-backend ${_backend}
                --golden-program "${_program}"
                --golden-check "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/${directory}/${fixture}.golden")
            set_tests_properties(${_test} PROPERTIES FIXTURES_REQUIRED ${_bake}
                LABELS "golden" ENVIRONMENT "RIGEXEC_ENABLE_SIMD=0")
        endforeach()
    endif()
endfunction()
