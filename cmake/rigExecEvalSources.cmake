# rigExecEval: the evaluation layer: domain bodies, mover and solver
# execution, weight packets, upstream and windowed reads, and the GPU
# translation jobs. A build boundary, not a shipped library:
# the objects are composed into librigExec by the root list, and this target is
# never installed, exported or linked.
add_library(rigExecEval OBJECT
    libs/rigExec/internal/bakedGeometry.cpp
    libs/rigExec/internal/bakedHarmonics.cpp
    libs/rigExec/internal/bakedIkFkMatch.cpp
    libs/rigExec/internal/bakedPose.cpp
    libs/rigExec/internal/bakedProperties.cpp
    libs/rigExec/internal/bakedSpaces.cpp
    libs/rigExec/internal/bakedWeights.cpp
    libs/rigExec/internal/bakedUpstream.cpp
    libs/rigExec/internal/bakedSchedule.cpp
    libs/rigExec/internal/bakedSlotDomains.cpp
    libs/rigExec/internal/bakedOpNodes.cpp
    libs/rigExec/internal/bakedTrace.cpp
    libs/rigExec/internal/moverGraph.cpp
    libs/rigExec/internal/solverKernels.cpp
    libs/rigExec/internal/weightField.cpp
    libs/rigExec/internal/weightPackets.cpp
    libs/rigExec/internal/movers/moverRegistry.cpp
    libs/rigExec/internal/movers/matrixMover.cpp
    libs/rigExec/internal/movers/skinMover.cpp
    libs/rigExec/internal/movers/armatureMover.cpp
    libs/rigExec/internal/movers/blendShapeMover.cpp
    libs/rigExec/internal/movers/smoothMover.cpp
    libs/rigExec/internal/movers/deltaMushMover.cpp
    libs/rigExec/internal/movers/wrinkleMover.cpp
    libs/rigExec/internal/movers/volumeCorrectMover.cpp
    libs/rigExec/internal/movers/latticeMover.cpp
    libs/rigExec/internal/movers/surfaceBindingMover.cpp
    libs/rigExec/internal/movers/patchMover.cpp
    libs/rigExec/internal/movers/surfaceMover.cpp
    libs/rigExec/internal/movers/curveMover.cpp
    libs/rigExec/internal/movers/mathMovers.cpp
    libs/rigExec/internal/movers/pathOperation.cpp
    libs/rigExec/internal/rigEvaluatorGeometry.cpp
    libs/rigExec/internal/rigEvaluatorInputs.cpp
    libs/rigExec/internal/rigEvaluatorPose.cpp
    libs/rigExec/internal/rigEvaluatorProperties.cpp
    libs/rigExec/internal/rigEvaluatorWindows.cpp
    libs/rigExec/internal/gpuDeformTranslation.cpp
    libs/rigExec/internal/gpuCloth.cpp
    # Scene records' typed operation bodies.
    libs/rigExecScene/solverProgram.cpp
    libs/rigExecScene/typedValues.cpp)
target_compile_features(rigExecEval PRIVATE cxx_std_17)
target_include_directories(rigExecEval PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecEval PRIVATE
    rigExecGraph rigExecMath rigExecEvalCore
    tf gf vt sdf ts usd usdGeom plug work)
set_target_properties(rigExecEval PROPERTIES POSITION_INDEPENDENT_CODE ON)
