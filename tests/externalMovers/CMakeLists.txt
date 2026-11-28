cmake_minimum_required(VERSION 3.26)
project(RigExecExternalMoverFixture LANGUAGES CXX)

# This directory is also a standalone SDK consumer for installed-build tests.
if(NOT TARGET rigExec::rigExec)
    find_package(rigExec CONFIG REQUIRED)
endif()

rigexec_add_mover(testExternalMover NO_INSTALL
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-plugins"
    DEFINITION kernel.h
    TYPE rigExec::TestQuadraticDefinition
    KERNEL kernel.cpp
    USD_ADAPTER mover.cpp
    FACTORY RigExecQuadraticUsdAdapter
    SCHEMA schema.usda
    SCHEMA_TYPE ExternalQuadraticMover)

# The same mover as an op plugin: its op runs the mover's kernel, and its
# binder turns the mover's declared inputs into the op's.
rigexec_add_op_plugin(testQuadraticOps NO_INSTALL BINDERS
    BINDS ExternalQuadraticMover
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-op-plugins"
    SOURCES quadraticOps.cpp
    LIBRARIES testExternalMoverKernel)
target_include_directories(testQuadraticOps PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
