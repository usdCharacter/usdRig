# rigExecCore: the USD-free rig description, compile and evaluator. Composed
# into librigExec like the other layers and linked as objects by USD-free
# tests. Depends on rigExecRegistry only.
add_library(rigExecCore OBJECT
    libs/rigExec/internal/compile/rigDesc.cpp
    libs/rigExec/internal/compile/compile.cpp
    libs/rigExec/internal/exec/evaluator.cpp
    libs/rigExec/internal/exec/threadTaskSystem.cpp)
target_compile_features(rigExecCore PUBLIC cxx_std_17)
target_include_directories(rigExecCore PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
set_target_properties(rigExecCore PROPERTIES POSITION_INDEPENDENT_CODE ON)
