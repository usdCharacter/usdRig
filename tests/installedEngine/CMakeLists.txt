cmake_minimum_required(VERSION 3.26)
project(RigExecInstalledEngine LANGUAGES CXX)

# A consumer of a package built with RIGEXEC_WITH_USD=OFF: it finds rigExec
# with no OpenUSD on the machine and builds against the engine targets only.
find_package(rigExec CONFIG REQUIRED)
if (rigExec_WITH_USD OR TARGET rigExec::rigExec)
    message(FATAL_ERROR "expected an engine-only rigExec package")
endif()
enable_testing()

add_executable(testRigExecInstalledBinaryHeaders
    ../testRigExecInstalledBinaryHeaders.cpp)
target_compile_features(testRigExecInstalledBinaryHeaders PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledBinaryHeaders PRIVATE
    rigExec::rigExecBinary)
add_test(NAME testRigExecInstalledBinaryHeaders
    COMMAND testRigExecInstalledBinaryHeaders)

set(_rigexec_op_core_tus)
foreach(_header registry opPorts rigDesc compile taps taskSystem evaluator
                view inputWriter runState)
    set(_tu "${CMAKE_CURRENT_BINARY_DIR}/opCoreHeaders/${_header}.cpp")
    file(CONFIGURE OUTPUT "${_tu}"
        CONTENT "#include \"rigExec/api/${_header}.h\"\n")
    list(APPEND _rigexec_op_core_tus "${_tu}")
endforeach()
add_library(testRigExecInstalledOpCoreHeaders OBJECT ${_rigexec_op_core_tus})
target_compile_features(testRigExecInstalledOpCoreHeaders PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledOpCoreHeaders PRIVATE
    rigExec::rigExecOpCore)

add_executable(testRigExecInstalledRuntime ../testRigExecRuntimeFixture.cpp)
target_compile_features(testRigExecInstalledRuntime PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledRuntime PRIVATE
    rigExec::rigExecRuntime)
add_test(NAME testRigExecInstalledRuntime COMMAND testRigExecInstalledRuntime
    "${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/minimal.rigexec")
set_tests_properties(testRigExecInstalledRuntime PROPERTIES
    ENVIRONMENT_MODIFICATION
        "LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_LIBRARY_DIR}")
