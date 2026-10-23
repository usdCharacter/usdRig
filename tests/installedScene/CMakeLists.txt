cmake_minimum_required(VERSION 3.26)
project(RigExecInstalledDependencies LANGUAGES CXX)

find_package(OpenGL REQUIRED)
find_package(rigExec CONFIG REQUIRED)
enable_testing()

add_executable(testRigExecInstalledScene ../testRigExecSceneDependency.cpp)
target_compile_features(testRigExecInstalledScene PRIVATE cxx_std_17)
target_compile_definitions(testRigExecInstalledScene PRIVATE RIGEXEC_EXPECT_SCENE)
target_link_libraries(testRigExecInstalledScene PRIVATE rigExec::rigExecScene)

# A manual link has no INTERFACE_SOURCES. Its explicit API call must retain
# the DLL/import library (and a DSO linked with --as-needed).
add_executable(testRigExecExplicitScene ../testRigExecSceneDependency.cpp)
target_compile_features(testRigExecExplicitScene PRIVATE cxx_std_17)
target_compile_definitions(testRigExecExplicitScene PRIVATE
    RIGEXEC_EXPECT_SCENE RIGEXEC_EXPLICIT_SCENE_REGISTRATION)
target_link_libraries(testRigExecExplicitScene PRIVATE
    rigExec::rigExec "$<TARGET_LINKER_FILE:rigExec::rigExecScene>")

add_executable(testRigExecInstalledSceneMissing ../testRigExecSceneDependency.cpp)
target_compile_features(testRigExecInstalledSceneMissing PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledSceneMissing PRIVATE rigExec::rigExec)

add_executable(testRigExecInstalledOracle ../testRigExecOracleDependency.cpp)
target_compile_features(testRigExecInstalledOracle PRIVATE cxx_std_17)
target_compile_definitions(testRigExecInstalledOracle PRIVATE
    RIGEXEC_EXPECT_ORACLE RIGEXEC_ORACLE_GOLDEN_ONLY)
target_link_libraries(testRigExecInstalledOracle PRIVATE rigExec::rigExecOracle)

add_executable(testRigExecExplicitOracle ../testRigExecOracleDependency.cpp)
target_compile_features(testRigExecExplicitOracle PRIVATE cxx_std_17)
target_compile_definitions(testRigExecExplicitOracle PRIVATE
    RIGEXEC_EXPECT_ORACLE RIGEXEC_ORACLE_GOLDEN_ONLY RIGEXEC_EXPLICIT_ORACLE_REGISTRATION)
target_link_libraries(testRigExecExplicitOracle PRIVATE
    rigExec::rigExec "$<TARGET_LINKER_FILE:rigExec::rigExecOracle>")

add_executable(testRigExecInstalledOracleMissing ../testRigExecOracleDependency.cpp)
target_compile_features(testRigExecInstalledOracleMissing PRIVATE cxx_std_17)
target_compile_definitions(testRigExecInstalledOracleMissing PRIVATE RIGEXEC_ORACLE_GOLDEN_ONLY)
target_link_libraries(testRigExecInstalledOracleMissing PRIVATE rigExec::rigExec)

foreach(_test testRigExecInstalledScene testRigExecExplicitScene testRigExecInstalledSceneMissing)
    add_test(NAME ${_test} COMMAND ${_test}
        "${rigExec_PLUGIN_DIR}/rigExecSchema/resources")
    set_tests_properties(${_test} PROPERTIES
        ENVIRONMENT "PXR_PLUGINPATH_NAME=${rigExec_PLUGIN_DIR}/rigExecSchema/resources"
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")
endforeach()

foreach(_test testRigExecInstalledOracle testRigExecExplicitOracle testRigExecInstalledOracleMissing)
    add_test(NAME ${_test} COMMAND ${_test})
    set_tests_properties(${_test} PROPERTIES
        ENVIRONMENT "RIGEXEC_GOLDEN_SUITE=;RIGEXEC_INPUT_REPLAY="
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")
endforeach()
