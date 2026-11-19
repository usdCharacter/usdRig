# rigExecCompile: the compile layer of the evaluator. Stage discovery,
# scene dispatch, dependency and value descriptors, digests, validation and
# the baked-program build. A build boundary, not a shipped library: the
# objects are composed into librigExec by the root list, and this target is
# never installed, exported or linked.
add_library(rigExecCompile OBJECT
    libs/rigExec/types.cpp
    libs/rigExec/sceneDispatch.cpp
    libs/rigExec/bakedOpGraph.cpp
    libs/rigExec/bakedOpValues.cpp
    libs/rigExec/crossDomainInputs.cpp
    libs/rigExec/parallel.cpp
    libs/rigExec/frozenDigest.cpp
    libs/rigExec/rigEvaluatorChainPlan.cpp
    libs/rigExec/rigEvaluatorCompile.cpp
    libs/rigExec/rigEvaluatorDependencies.cpp
    libs/rigExec/rigEvaluatorDigest.cpp
    libs/rigExec/rigEvaluatorLiveGraph.cpp
    libs/rigExec/rigEvaluatorValidation.cpp
    libs/rigExec/bakedProgram.cpp
    # Scene programs that compose compiled descriptors.
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
    rigExecGraph rigExecMath rigExecInputValues
    tf gf vt sdf ts usd usdGeom usdSkel plug work)
set_target_properties(rigExecCompile PROPERTIES POSITION_INDEPENDENT_CODE ON)
