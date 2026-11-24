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

# A USD-free consumer of the installed binary headers: the cloth mover type
# name keeps the array contract the runtime installer and its hosts expect.
add_executable(testRigExecInstalledBinaryHeaders ../testRigExecInstalledBinaryHeaders.cpp)
target_compile_features(testRigExecInstalledBinaryHeaders PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledBinaryHeaders PRIVATE rigExec::rigExecBinary)
add_test(NAME testRigExecInstalledBinaryHeaders
    COMMAND testRigExecInstalledBinaryHeaders)

# The installed SDK surface: RigExecProgramView answers its queries, the
# evaluator's execution records are not nameable, and include/rigExec holds
# api/ plus exactly the internal headers tools/rigexec_installed_internal.txt
# lists.
get_target_property(_rigexec_include_dirs rigExec::rigExecGraph
                    INTERFACE_INCLUDE_DIRECTORIES)
list(GET _rigexec_include_dirs 0 _rigexec_include_dir)
add_executable(testRigExecInstalledSeal ../testRigExecInstalledSeal.cpp)
target_compile_features(testRigExecInstalledSeal PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledSeal PRIVATE
    rigExec::rigExec rigExec::rigExecScene plug usd usdGeom)
add_test(NAME testRigExecInstalledSeal COMMAND testRigExecInstalledSeal
    "${_rigexec_include_dir}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/rigexec_installed_internal.txt")
set_tests_properties(testRigExecInstalledSeal PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=${rigExec_PLUGIN_DIR}/rigExecSchema/resources"
    ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")

# The background scheduler links from rigExecPlayback, not from the engine.
add_executable(testRigExecInstalledPlayback ../testRigExecInstalledPlayback.cpp)
target_compile_features(testRigExecInstalledPlayback PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledPlayback PRIVATE
    rigExec::rigExecPlayback)
add_test(NAME testRigExecInstalledPlayback COMMAND testRigExecInstalledPlayback)
set_tests_properties(testRigExecInstalledPlayback PROPERTIES
    ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")

# Each op-core API header compiles on its own against the installed op core,
# which carries no OpenUSD include path.
set(_rigexec_op_core_tus)
foreach(_header registry opPorts rigDesc compile taps taskSystem evaluator
                view inputWriter)
    set(_tu "${CMAKE_CURRENT_BINARY_DIR}/opCoreHeaders/${_header}.cpp")
    file(CONFIGURE OUTPUT "${_tu}"
        CONTENT "#include \"rigExec/api/${_header}.h\"\n")
    list(APPEND _rigexec_op_core_tus "${_tu}")
endforeach()
add_library(testRigExecInstalledOpCoreHeaders OBJECT ${_rigexec_op_core_tus})
target_compile_features(testRigExecInstalledOpCoreHeaders PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledOpCoreHeaders PRIVATE
    rigExec::rigExecOpCore)

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

# A mover plugin generated by the installed cmake modules against the installed
# headers, then loaded by the installed registry: the API version the module
# stamps must be the one the loader accepts.
rigexec_add_mover(testExternalMover NO_INSTALL
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-plugins"
    DEFINITION ../externalMovers/kernel.h
    TYPE rigExec::TestQuadraticDefinition
    KERNEL ../externalMovers/kernel.cpp
    USD_ADAPTER ../externalMovers/mover.cpp
    FACTORY RigExecQuadraticUsdAdapter
    SCHEMA ../externalMovers/schema.usda
    SCHEMA_TYPE ExternalQuadraticMover)
get_target_property(installedExternalMoverResources testExternalMover
                    RIGEXEC_PLUGIN_RESOURCE_DIR)

add_executable(testRigExecInstalledMover ../testRigExecInstalledMover.cpp)
target_compile_features(testRigExecInstalledMover PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledMover PRIVATE
    rigExec::rigExec rigExec::rigExecScene rigExec::rigExecAuthoring
    plug usd usdGeom)
add_dependencies(testRigExecInstalledMover testExternalMover)
add_test(NAME testRigExecInstalledMover
    COMMAND testRigExecInstalledMover "${installedExternalMoverResources}")
set_tests_properties(testRigExecInstalledMover PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=${rigExec_PLUGIN_DIR}/rigExecSchema/resources;${installedExternalMoverResources}"
    ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")

# An op plugin built by the installed rigexec_add_op_plugin against the
# installed headers, then discovered by the installed rigExecScene.
rigexec_add_op_plugin(testOffsetOps NO_INSTALL BINDERS
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-op-plugins"
    SOURCES ../opPlugins/offsetOps.cpp
    SCHEMA ../opPlugins/schema.usda
    SCHEMA_TYPE TestOffsetMover)
get_target_property(installedOffsetOpsResources testOffsetOps
                    RIGEXEC_PLUGIN_RESOURCE_DIR)
add_executable(testRigExecInstalledOpPlugins ../testRigExecOpPlugins.cpp)
target_compile_features(testRigExecInstalledOpPlugins PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledOpPlugins PRIVATE
    rigExec::rigExec rigExec::rigExecScene rigExec::rigExecBuiltins plug usd)
add_dependencies(testRigExecInstalledOpPlugins testOffsetOps)
add_test(NAME testRigExecInstalledOpPlugins
    COMMAND testRigExecInstalledOpPlugins good "${installedOffsetOpsResources}"
        "$<TARGET_FILE:testOffsetOps>"
        "${CMAKE_CURRENT_BINARY_DIR}/testRigExecInstalledOpPlugins-scratch")
set_tests_properties(testRigExecInstalledOpPlugins PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=${rigExec_PLUGIN_DIR}/rigExecSchema/resources"
    ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")

# The fixture mover's op plugin, built against the installed SDK beside the
# mover: the installed scene program runs the mover on its op.
rigexec_add_op_plugin(testQuadraticOps NO_INSTALL BINDERS
    BINDS ExternalQuadraticMover
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-op-plugins"
    SOURCES ../externalMovers/quadraticOps.cpp
    LIBRARIES testExternalMoverKernel)
target_include_directories(testQuadraticOps PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../externalMovers")
get_target_property(installedQuadraticOpsResources testQuadraticOps
                    RIGEXEC_PLUGIN_RESOURCE_DIR)
add_executable(testRigExecInstalledOpPluginScene ../testRigExecOpPluginScene.cpp)
target_compile_features(testRigExecInstalledOpPluginScene PRIVATE cxx_std_17)
target_link_libraries(testRigExecInstalledOpPluginScene PRIVATE
    rigExec::rigExec rigExec::rigExecScene plug usd)
add_dependencies(testRigExecInstalledOpPluginScene testExternalMover
    testQuadraticOps)
add_test(NAME testRigExecInstalledOpPluginScene
    COMMAND testRigExecInstalledOpPluginScene
        "${installedExternalMoverResources}" "${installedQuadraticOpsResources}")
set_tests_properties(testRigExecInstalledOpPluginScene PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=${rigExec_PLUGIN_DIR}/rigExecSchema/resources"
    ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${rigExec_LIBRARY_DIR};PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/bin;PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;LD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib;DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_LIBRARY_DIR};DYLD_LIBRARY_PATH=path_list_prepend:${rigExec_USD_INSTALL_DIR}/lib")

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
