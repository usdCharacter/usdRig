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
