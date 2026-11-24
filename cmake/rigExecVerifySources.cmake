# rigExecVerify: the checking layer. Baked verification, oracle dispatch,
# scalar reference adaptation, input replay and the computation tap set. A
# build boundary, not a shipped library: the objects are composed into
# librigExec by the root list, and this target is never installed, exported
# or linked. It becomes an opt-in shared library later.
add_library(rigExecVerify OBJECT
    libs/rigExec/internal/bakedVerify.cpp
    libs/rigExec/internal/oracleDispatch.cpp
    libs/rigExec/internal/scalarReferenceAdapter.cpp
    libs/rigExec/internal/inputReplay.cpp
    libs/rigExec/internal/verifyObservers.cpp
    libs/rigExec/internal/tapSet.cpp
    libs/rigExec/internal/computations.cpp)
target_compile_features(rigExecVerify PRIVATE cxx_std_17)
target_include_directories(rigExecVerify PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecVerify PRIVATE
    rigExecGraph rigExecMath
    tf gf vt sdf ts usd usdGeom plug work exec execUsd ef vdf)
set_target_properties(rigExecVerify PROPERTIES POSITION_INDEPENDENT_CODE ON)
