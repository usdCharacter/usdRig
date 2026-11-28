cmake_minimum_required(VERSION 3.26)
project(RigExecOpPluginFixture LANGUAGES CXX)

# Also a standalone SDK consumer for installed-build tests.
if(NOT TARGET rigExec::rigExec)
    find_package(rigExec CONFIG REQUIRED)
endif()

rigexec_add_op_plugin(testOffsetOps NO_INSTALL BINDERS
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-op-plugins"
    SOURCES offsetOps.cpp
    SCHEMA schema.usda
    SCHEMA_TYPE TestOffsetMover)
