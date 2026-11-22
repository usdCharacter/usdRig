# rigExecRegistry: the type and op registry. USD-free; composed into
# librigExec like the other layers and linked as objects by USD-free tests.
add_library(rigExecRegistry OBJECT
    libs/rigExec/internal/registry/registry.cpp)
target_compile_features(rigExecRegistry PUBLIC cxx_std_17)
target_include_directories(rigExecRegistry PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
set_target_properties(rigExecRegistry PROPERTIES POSITION_INDEPENDENT_CODE ON)
