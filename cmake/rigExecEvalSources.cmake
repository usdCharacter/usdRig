# rigExecEval: the evaluation layer. Domain bodies, mover and solver
# execution, weight packets, upstream and windowed reads, and the GPU
# translation jobs until they move to rigExecGpu. A build boundary, not a
# shipped library: the objects are composed into librigExec by the root
# list, and this target is never installed, exported or linked.
add_library(rigExecEval OBJECT
    libs/rigExec/bakedGeometry.cpp
    libs/rigExec/bakedHarmonics.cpp
    libs/rigExec/bakedIkFkMatch.cpp
    libs/rigExec/bakedPose.cpp
    libs/rigExec/bakedProperties.cpp
    libs/rigExec/bakedSpaces.cpp
    libs/rigExec/bakedWeights.cpp
    libs/rigExec/bakedUpstream.cpp
    libs/rigExec/bakedSchedule.cpp
    libs/rigExec/bakedTrace.cpp
    libs/rigExec/moverGraph.cpp
    libs/rigExec/moverKernels.cpp
    libs/rigExec/solverKernels.cpp
    libs/rigExec/weightField.cpp
    libs/rigExec/weightPackets.cpp
    libs/rigExec/affineFrameComputations.cpp
    libs/rigExec/movers/moverRegistry.cpp
    libs/rigExec/movers/matrixMover.cpp
    libs/rigExec/movers/skinMover.cpp
    libs/rigExec/movers/armatureMover.cpp
    libs/rigExec/movers/blendShapeMover.cpp
    libs/rigExec/movers/smoothMover.cpp
    libs/rigExec/movers/deltaMushMover.cpp
    libs/rigExec/movers/wrinkleMover.cpp
    libs/rigExec/movers/volumeCorrectMover.cpp
    libs/rigExec/movers/latticeMover.cpp
    libs/rigExec/movers/surfaceBindingMover.cpp
    libs/rigExec/movers/patchMover.cpp
    libs/rigExec/movers/surfaceMover.cpp
    libs/rigExec/movers/curveMover.cpp
    libs/rigExec/movers/mathMovers.cpp
    libs/rigExec/movers/pathOperation.cpp
    libs/rigExec/rigEvaluatorGeometry.cpp
    libs/rigExec/rigEvaluatorInputs.cpp
    libs/rigExec/rigEvaluatorPose.cpp
    libs/rigExec/rigEvaluatorProperties.cpp
    libs/rigExec/rigEvaluatorWindows.cpp
    libs/rigExec/gpuDeformTranslation.cpp
    libs/rigExec/gpuCloth.cpp
    # Scene programs that run typed operation bodies.
    libs/rigExecScene/solverProgram.cpp
    libs/rigExecScene/typedValues.cpp)
target_compile_features(rigExecEval PRIVATE cxx_std_17)
target_include_directories(rigExecEval PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecEval PRIVATE
    rigExecGraph rigExecMath rigExecEvalCore
    tf gf vt sdf ts usd usdGeom usdSkel plug work exec execUsd ef vdf)
set_target_properties(rigExecEval PROPERTIES POSITION_INDEPENDENT_CODE ON)
