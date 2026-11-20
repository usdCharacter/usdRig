# rigExecSchedule: the detached-work layer. Frame and task caches, the
# invalidation index, frozen snapshots, sampling and workers. A build
# boundary, not a shipped library: the objects are composed into librigExec
# by the root list, and this target is never installed, exported or linked.
add_library(rigExecSchedule OBJECT
    libs/rigExec/backgroundScheduler.cpp
    libs/rigExec/frameCache.cpp
    libs/rigExec/frameCacheSparsity.cpp
    libs/rigExec/taskListCache.cpp
    libs/rigExec/outputAffectedIndex.cpp
    libs/rigExec/frozenContext.cpp
    libs/rigExec/frozenPropertySampling.cpp
    libs/rigExec/frozenSampling.cpp
    libs/rigExec/frozenSnapshot.cpp
    libs/rigExec/frozenWorker.cpp)
target_compile_features(rigExecSchedule PRIVATE cxx_std_17)
target_include_directories(rigExecSchedule PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecSchedule PRIVATE
    rigExecGraph rigExecMath
    tf gf vt sdf ts usd usdGeom usdSkel plug work)
set_target_properties(rigExecSchedule PROPERTIES POSITION_INDEPENDENT_CODE ON)
