# rigExecCompile: the compile layer of the evaluator. Stage discovery,
# scene dispatch, dependency and value descriptors, digests, validation and
# the baked-program build. A build boundary, not a shipped library: the
# objects are composed into librigExec by the root list, and this target is
# never installed, exported or linked.
add_library(rigExecCompile OBJECT
    libs/rigExec/internal/types.cpp
    libs/rigExec/internal/sceneDispatch.cpp
    libs/rigExec/internal/bakedOpGraph.cpp
    libs/rigExec/internal/bakedOpValues.cpp
    libs/rigExec/internal/crossDomainInputs.cpp
    libs/rigExec/internal/parallel.cpp
    libs/rigExec/internal/frozenDigest.cpp
    libs/rigExec/internal/rigEvaluatorChainPlan.cpp
    libs/rigExec/internal/rigEvaluatorCompile.cpp
    libs/rigExec/internal/rigEvaluatorCompileChains.cpp
    libs/rigExec/internal/rigEvaluatorCompileCommit.cpp
    libs/rigExec/internal/rigEvaluatorCompileConstraints.cpp
    libs/rigExec/internal/rigEvaluatorCompilePoseGraph.cpp
    libs/rigExec/internal/rigEvaluatorCompileSolvers.cpp
    libs/rigExec/internal/rigEvaluatorDependencies.cpp
    libs/rigExec/internal/rigEvaluatorDigest.cpp
    libs/rigExec/internal/rigEvaluatorLiveGraph.cpp
    libs/rigExec/internal/rigEvaluatorValidation.cpp
    libs/rigExec/internal/bakedProgram.cpp
    libs/rigExec/internal/bakedProgramAdopt.cpp
    libs/rigExec/internal/bakedProgramBakeable.cpp
    libs/rigExec/internal/bakedProgramBuild.cpp
    libs/rigExec/internal/bakedProgramBuildGraph.cpp
    libs/rigExec/internal/bakedProgramBuildPose.cpp
    libs/rigExec/internal/bakedProgramDescribe.cpp
    libs/rigExec/internal/bakedProgramEdits.cpp
    libs/rigExec/internal/programView.cpp
    libs/rigExec/internal/compiledProgram.cpp
    # Scene records the compile composes from captured descriptors.
    libs/rigExecScene/constraintProgram.cpp
    libs/rigExecScene/geometryProgram.cpp
    libs/rigExecScene/poseProgram.cpp
    libs/rigExecScene/propertyProgram.cpp
    libs/rigExecScene/providerContextBinding.cpp
    libs/rigExecScene/providerProgram.cpp
    libs/rigExecScene/sceneCompileInputs.cpp
    libs/rigExecScene/weightProgram.cpp)
target_compile_features(rigExecCompile PRIVATE cxx_std_17)
target_include_directories(rigExecCompile PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecCompile PRIVATE
    rigExecGraph rigExecMathGf rigExecInputValues
    tf gf vt sdf ts usd usdGeom plug work)
set_target_properties(rigExecCompile PROPERTIES POSITION_INDEPENDENT_CODE ON)
