cmake_minimum_required(VERSION 3.26)
project(rigExec VERSION 0.2.0 LANGUAGES C CXX)
include(cmake/TestLabels.cmake)
option(RIGEXEC_WITH_USD
       "Build the OpenUSD adapter, imaging, authoring, bake and USD tests" ON)

# RigExec builds against an unchanged installed OpenUSD v26.08
set(USD_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../usd-install"
    CACHE PATH "OpenUSD installation prefix")

if (RIGEXEC_WITH_USD)
# garch's exported link interface names OpenGL::GL, so that imported
# target must exist before pxrTargets is included. A cross toolchain
# keeps today's behavior: its USD prefix names its own graphics API.
if (NOT CMAKE_CROSSCOMPILING)
    find_package(OpenGL REQUIRED)
endif()
find_package(pxr REQUIRED CONFIG PATHS "${USD_INSTALL_DIR}" NO_DEFAULT_PATH)
endif()  # RIGEXEC_WITH_USD

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if (MSVC)
    add_compile_options(/permissive- /Zc:__cplusplus /bigobj /W3
        /wd4244 /wd4305 /wd4267)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES
        WIN32_LEAN_AND_MEAN)
endif()

# Bitwise parity with Gf and the frozen oracles needs OpenUSD's own
# floating-point contraction: USD builds with -ffp-contract=off on Apple
# (cmake/defaults/clangdefaults.cmake) so a*b+c is never fused into FMA.
# Clang on ARM64 contracts by default, which drifts every memcmp/bitwise
# test (RuntimeMath, RuntimePose, RuntimeGeometry, ImagingPlayback,
# verify_binary, the Rbf oracle) by an ulp. -fno-fast-math does not cover
# this; only -ffp-contract=off does. MSVC needs nothing: /fp:precise is
# its default and it never auto-contracts. cmake/rigexec_flags.cmake owns
# the flag spelling; this scope installs only its contraction-family
# directory policy, so a vendored target compiled in a child scope can
# still re-declare the fast family and keep its codegen while the exact-FP
# finalization walk enforces the effective mode on every owned target.
# Docs: docs/concepts/baked-vs-dynamic.md.
include(cmake/rigexec_flags.cmake)
rigexec_get_exact_fp_directory_flags(_rigexec_fp_directory_flags)
add_compile_options(${_rigexec_fp_directory_flags})
include(cmake/installManifests.cmake)
set(RIGEXEC_INSTALL_LIBDIR     "lib"
    CACHE STRING "Install directory for libraries, relative to the prefix")
set(RIGEXEC_INSTALL_INCLUDEDIR "include"
    CACHE STRING "Install directory for headers, relative to the prefix")
set(RIGEXEC_INSTALL_PLUGINDIR  "lib/usd"
    CACHE STRING "Install directory for USD plugins, relative to the prefix")
set(RIGEXEC_INSTALL_PYTHONDIR  "lib/python"
    CACHE STRING "Install directory for Python modules, relative to the prefix")
set(RIGEXEC_INSTALL_CMAKEDIR   "lib/cmake/rigExec"
    CACHE STRING "Install directory for the CMake package, relative to the prefix")
set(RIGEXEC_INSTALL_DATADIR    "share/rigExec"
    CACHE STRING "Install directory for the exported build manifests, relative to the prefix")

if (APPLE)
    set(CMAKE_INSTALL_NAME_DIR "@rpath")
    set(CMAKE_INSTALL_RPATH "@loader_path")
elseif (UNIX)
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# USD-free op scheduler. libs/rigExecGraph is only this archive.
# USD scene translation is rigExecScene, linked into the shared library below.
add_library(rigExecGraph STATIC libs/rigExecGraph/opGraph.cpp)
target_compile_features(rigExecGraph PUBLIC cxx_std_17)
target_compile_definitions(rigExecGraph PUBLIC
    $<$<PLATFORM_ID:Windows>:NOMINMAX>)
target_include_directories(rigExecGraph PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
set_target_properties(rigExecGraph PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecGraph ALIAS rigExecGraph)

# EvalCore: the step contract the baked program and playback share. The step
# bodies are templates each front end instantiates, so only the non-templated
# ordinal table lands in this archive. It links nothing but the scheduler and
# uses only rigExecMath's USD-free headers. The wire enum it reuses is
# generated against the vendored FlatBuffers headers.
add_library(rigExecEvalCore STATIC libs/rigExecEvalCore/kinds.cpp)
target_compile_features(rigExecEvalCore PUBLIC cxx_std_17)
target_include_directories(rigExecEvalCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/flatbuffers/include>)
target_link_libraries(rigExecEvalCore PUBLIC rigExecGraph)
set_target_properties(rigExecEvalCore PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecEvalCore ALIAS rigExecEvalCore)

# rigExecMath: the USD-free kernel headers.
add_library(rigExecMath INTERFACE)
target_include_directories(rigExecMath INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)

if (RIGEXEC_WITH_USD)
# rigExecMathGf: the point-frame and solver kernels written against Gf.
add_library(rigExecMathGf STATIC
    libs/rigExecMath/affineFrameKernels.cpp
    libs/rigExecMath/pointFrame.cpp
    libs/rigExecMath/solvers.cpp
    libs/rigExecMath/singleChainIk.cpp
    libs/rigExecMath/propertyMath.cpp
    libs/rigExecMath/geometryKernels.cpp
    libs/rigExecMath/weightFields.cpp
    libs/rigExecMath/simdKernels.cpp
    libs/rigExecMath/dualQuat.cpp
    libs/rigExecMath/splineIk.cpp
    libs/rigExecMath/rbf.cpp
)
target_link_libraries(rigExecMathGf PUBLIC rigExecMath arch tf gf vt)
set_target_properties(rigExecMathGf PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Keep value-codec template instantiations private to its archive. Only the
# two declared codec functions become Windows DLL exports.
add_library(rigExecInputValues STATIC libs/rigExec/internal/inputReplayValues.cpp)
target_include_directories(rigExecInputValues PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecInputValues PRIVATE vt sdf ts)
target_compile_definitions(rigExecInputValues PRIVATE RIGEXEC_INPUT_VALUES_EXPORTS)
set_target_properties(rigExecInputValues PROPERTIES POSITION_INDEPENDENT_CODE ON)

# USD scene translation. A shared library: librigExec does not absorb it.
# Frame-path program runners are sources of rigExec, below.
add_library(rigExecScene SHARED
    libs/rigExecScene/authoredRestSpace.cpp
    libs/rigExecScene/autoClavicleGraph.cpp
    libs/rigExecScene/constraintSceneTranslation.cpp
    libs/rigExecScene/externalOps.cpp
    libs/rigExecScene/geometryOps.cpp
    libs/rigExecScene/opPlugins.cpp
    libs/rigExecScene/geometrySceneTranslation.cpp
    libs/rigExecScene/jointGroupTranslation.cpp
    libs/rigExecScene/poseOps.cpp
    libs/rigExecScene/poseSceneTranslation.cpp
    libs/rigExecScene/propertySceneTranslation.cpp
    libs/rigExecScene/providerRecordExport.cpp
    libs/rigExecScene/registerScene.cpp
    libs/rigExecScene/sceneLinkAnchor.cpp
    libs/rigExecScene/sceneDescriptors.cpp
    libs/rigExecScene/sceneGraphBinding.cpp
    libs/rigExecScene/sceneGraphTypedRead.cpp
    libs/rigExecScene/sceneTypedReads.cpp
    libs/rigExecScene/skeleton2dTranslation.cpp
    libs/rigExecScene/solverSceneTranslation.cpp
    libs/rigExecScene/usdSceneAccess.cpp
    libs/rigExecScene/weightSceneTranslation.cpp
    libs/rigExecScene/workTaskSystem.cpp
)
target_compile_features(rigExecScene PUBLIC cxx_std_17)
target_include_directories(rigExecScene PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecScene PUBLIC
    tf gf vt sdf ts usd usdGeom)
target_link_libraries(rigExecScene PRIVATE rigExecGraph rigExecMathGf
    rigExecBuiltins plug js work)
set_target_properties(rigExecScene PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecScene ALIAS rigExecScene)
target_sources(rigExecScene INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/rigExecScene/sceneLinkAnchor.cpp>"
    "$<INSTALL_INTERFACE:${RIGEXEC_INSTALL_CMAKEDIR}/sceneLinkAnchor.cpp>")
endif()  # RIGEXEC_WITH_USD

# The op core: type and op registry, rig description, compile and
# evaluator. It links no OpenUSD library. A shared library, so that rigExec,
# rigExecBuiltins, rigExecRuntime and every executable share one registry
# and one set of handle indices per process; a static archive would give
# each Windows DLL that links it its own copy.
find_package(Threads REQUIRED)
add_library(rigExecOpCore SHARED
    libs/rigExec/internal/registry/registry.cpp
    libs/rigExec/internal/registry/opFields.cpp
    libs/rigExec/internal/compile/rigDesc.cpp
    libs/rigExec/internal/compile/compile.cpp
    libs/rigExec/internal/exec/addressWait.cpp
    libs/rigExec/internal/exec/cpuReferenceGpuDevice.cpp
    libs/rigExec/internal/exec/evaluator.cpp
    libs/rigExec/internal/exec/frameView.cpp
    libs/rigExec/internal/exec/inputChannel.cpp
    libs/rigExec/internal/exec/run.cpp
    libs/rigExec/internal/exec/runState.cpp
    libs/rigExec/internal/exec/threadTaskSystem.cpp
    libs/rigExec/internal/exec/view.cpp)
target_compile_features(rigExecOpCore PUBLIC cxx_std_17)
target_include_directories(rigExecOpCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecOpCore PUBLIC rigExecGraph
    PRIVATE Threads::Threads)
set_target_properties(rigExecOpCore PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecOpCore ALIAS rigExecOpCore)

if (RIGEXEC_WITH_USD)
# The evaluator's layers, as object libraries: build boundaries, not
# shipped libraries. Each file declares its own target, sources, include
# directories and links, and they compose into librigExec below. None
# of them is installed, exported, aliased or linked.
include(cmake/rigExecCompileSources.cmake)
include(cmake/rigExecEvalSources.cmake)
include(cmake/rigExecScheduleSources.cmake)
include(cmake/rigExecVerifySources.cmake)

# rigExec: schema bindings and native evaluation. It does not link the
# scene or oracle libraries. Callers that compile a stage link
# rigExecScene. Cpu-reference and golden runs link rigExecOracle. The
# layer objects compose in DAG order; only the facade's lifecycle and
# source-notice sources compile here.
add_library(rigExec SHARED
    $<TARGET_OBJECTS:rigExecCompile>
    $<TARGET_OBJECTS:rigExecEval>
    $<TARGET_OBJECTS:rigExecSchedule>
    $<TARGET_OBJECTS:rigExecVerify>
    libs/rigExec/internal/rigEvaluator.cpp
    libs/rigExec/internal/rigEvaluatorNotices.cpp
)
target_include_directories(rigExec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExec PUBLIC
    rigExecOpCore
    plug sdf usd usdGeom work
)

target_link_libraries(rigExec PRIVATE
    rigExecGraph rigExecMathGf rigExecEvalCore rigExecInputValues
    rigExecBuiltins ts)
# On Windows the consumers of rigExec need those two archives as well.

# Scene translation calls back into the evaluator. The evaluator does not
# link the scene library; callers that compile a stage do.
target_link_libraries(rigExecScene PUBLIC rigExec)

# Test oracles and goldens. A shared library so librigExec does not absorb
# it and a static math edge does not land on every consumer. Not linked
# into imaging, playback, or runtime tests. The Python module links it,
# so the install ships the library beside librigExec. A cpu-reference or
# golden run reports an error when the library is missing.
add_library(rigExecOracle SHARED
    libs/rigExecOracle/goldenPose.cpp
    libs/rigExecOracle/goldenSuite.cpp
    libs/rigExecOracle/bakedExecCrossCheck.cpp
    libs/rigExecOracle/bakedExecCrossCheckRows.cpp
    libs/rigExecOracle/weightReference.cpp
    libs/rigExecOracle/scalarReference.cpp
    libs/rigExecOracle/scalarReferenceRun.cpp
    libs/rigExecOracle/oracleInputs.cpp
    libs/rigExecOracle/registerOracle.cpp
    libs/rigExecOracle/oracleLinkAnchor.cpp
)
target_compile_features(rigExecOracle PUBLIC cxx_std_17)
target_include_directories(rigExecOracle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecOracle PRIVATE rigExecMathGf)
target_link_libraries(rigExecOracle PUBLIC
    rigExec tf gf vt sdf ts usd usdGeom)
set_target_properties(rigExecOracle PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecOracle ALIAS rigExecOracle)
target_sources(rigExecOracle INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/rigExecOracle/oracleLinkAnchor.cpp>"
    "$<INSTALL_INTERFACE:${RIGEXEC_INSTALL_CMAKEDIR}/oracleLinkAnchor.cpp>")

target_sources(rigExec INTERFACE
    "$<BUILD_INTERFACE:$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${CMAKE_CURRENT_SOURCE_DIR}/libs/rigExec/internal/observerHostExit.cpp>>")

add_library(rigExec::rigExec ALIAS rigExec)
endif()  # RIGEXEC_WITH_USD
include(cmake/rigExecMoverPlugin.cmake)
include(cmake/rigExecMover.cmake)
include(cmake/rigExecRegistration.cmake)
include(cmake/rigExecOpPlugin.cmake)

# The op core's built-in type and op libraries. rigExecScene runs geometry
# through them, so they build with or without tests.
rigexec_add_type_library(rigExecStdTypes
    SOURCES libs/rigExecStdTypes/stdTypes.cpp)
rigexec_add_type_library(rigExecRigTypes
    SOURCES libs/rigExecRigTypes/rigTypes.cpp
    REQUIRES rigExecStdTypes)
rigexec_add_op_library(rigExecOps
    SOURCES libs/rigExecOps/library.cpp
        libs/rigExecOps/property/mathOps.cpp
        libs/rigExecOps/deform/pointOps.cpp
        libs/rigExecOps/deform/deltaMushOp.cpp
        libs/rigExecOps/deform/latticeOp.cpp
        libs/rigExecOps/deform/wrinkleOp.cpp
        libs/rigExecOps/deform/surfaceProjectOp.cpp
        libs/rigExecOps/deform/matrixDeformOp.cpp
        libs/rigExecOps/deform/skinOp.cpp
        libs/rigExecOps/deform/blendShapeOp.cpp
        libs/rigExecOps/deform/derivedOps.cpp
        libs/rigExecOps/deform/wireOp.cpp
        libs/rigExecOps/deform/ribbonOp.cpp
        libs/rigExecOps/deform/projectorOps.cpp
        libs/rigExecOps/deform/patchOp.cpp
        libs/rigExecOps/deform/surfaceBindingOp.cpp
        libs/rigExecOps/deform/layeredSkinOp.cpp
        libs/rigExecOps/deform/clothOp.cpp
        libs/rigExecOps/constraint/autoClavicleOps.cpp
        libs/rigExecOps/constraint/constraintOps.cpp
        libs/rigExecOps/constraint/harmonicsOps.cpp
        libs/rigExecOps/constraint/matchOps.cpp
        libs/rigExecOps/constraint/matrixMoverOps.cpp
        libs/rigExecOps/weight/weightOps.cpp
        libs/rigExecOps/solver/chainSolverOps.cpp
        libs/rigExecOps/solver/frameBlendOps.cpp
        libs/rigExecOps/solver/ribbonFramesOps.cpp
        libs/rigExecOps/solver/singleChainIkOps.cpp
        libs/rigExecOps/solver/skeleton2dOps.cpp
        libs/rigExecOps/solver/splineIkOps.cpp
        libs/rigExecOps/space/spaceOps.cpp
        libs/rigExecOps/space/providerOps.cpp
        libs/rigExecOps/solver/poseCommitOps.cpp
        libs/rigExecOps/pose/poseOps.cpp
    USES_TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes)
target_link_libraries(rigExecOps PRIVATE rigExecEvalCore)

# The built-in type and op libraries in one shared module, so every library
# and executable sees one set of handles. Their sources compile here as its
# own objects, which is what the Windows export-all exports.
get_target_property(_rigexec_builtin_ops rigExecOps SOURCES)
add_library(rigExecBuiltins SHARED libs/rigExecBuiltins/builtins.cpp
    libs/rigExecStdTypes/stdTypes.cpp libs/rigExecRigTypes/rigTypes.cpp
    ${_rigexec_builtin_ops})
target_compile_features(rigExecBuiltins PUBLIC cxx_std_17)
target_include_directories(rigExecBuiltins PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>"
    "$<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>")
target_link_libraries(rigExecBuiltins PUBLIC rigExecOpCore PRIVATE rigExecEvalCore)

if (RIGEXEC_WITH_USD)
add_library(rigExecAuthoring SHARED
    libs/rigExecAuthoring/generateSkinWeights.cpp
    libs/rigExecAuthoring/rigBuilder.cpp
    libs/rigExecAuthoring/schemaAuthoring.cpp)
target_include_directories(rigExecAuthoring PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecAuthoring PUBLIC
    sdf tf gf vt usd)
# The skin-weight generator composes rests and measures volume weights
# through rigExecScene, which brings rigExec.
target_link_libraries(rigExecAuthoring PRIVATE rigExecMathGf rigExecScene)

# rigExecGpu: the per-vertex deform program on the GPU as an OpenGL 4.5
# compute shader, on a worker thread that owns its own hidden context, and
# the GLSL body the Storm ExtComputation kernel shares.
add_library(rigExecGpu SHARED
    libs/rigExecGpu/glContext.cpp
    libs/rigExecGpu/gpuDeformExecutor.cpp
    libs/rigExecGpu/gpuDeformShader.cpp
    libs/rigExecGpu/clothShader.cpp)
target_include_directories(rigExecGpu PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecGpu PUBLIC rigExec tf gf)
target_link_libraries(rigExecGpu PRIVATE garch)
endif()  # RIGEXEC_WITH_USD
include(cmake/WindowsArchiveLinks.cmake)

if (RIGEXEC_WITH_USD)
# rigExecPlayback: the background scheduler that warms the frame cache on
# its own worker threads. The engine does not link it: frozen evaluation
# polls the cancellation token the scheduler hands out. Imaging links it
# publicly because its registry header names the scheduler.
add_library(rigExecPlayback SHARED libs/rigExecPlayback/backgroundScheduler.cpp)
target_compile_features(rigExecPlayback PUBLIC cxx_std_17)
target_include_directories(rigExecPlayback PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecPlayback PUBLIC rigExec tf sdf usd)
set_target_properties(rigExecPlayback PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecPlayback ALIAS rigExecPlayback)

# rigExecImaging: Hydra 2.0 scene-index publication (spec §10).
# The registry coordination domains compile into private object libraries
# below and compose into the one shared library. The domain translation
# units share a single RigExecImagingRegistry context, so these are build
# boundaries, not shipped libraries: none links rigExecImaging or another
# object library, and none is installed or exported. Each object is
# composed into the shared target exactly once.
add_library(rigExecImagingContext OBJECT
    # The imaging directory and chain binding: the process-level stage ->
    # context map, the chain registrations and the legacy current slot.
    libs/rigExecImaging/registryDirectory.cpp
    # Session lifecycle: explicit and automatic activation with the
    # transactional commit, the authored-guide display on initial failure,
    # rig-root noting and the teardown that stops the warming pool.
    libs/rigExecImaging/registryLifecycle.cpp
    # Publication: session evaluation into one combined generation, the
    # store publish with its generation and frame table, and the broadcast
    # to the bound chains.
    libs/rigExecImaging/registryPublication.cpp
    # Edit response: the guide-only and rig stage-notice handlers, the
    # read-root collection and refresh, and the generation cancel plus the
    # per-time retirement lanes.
    libs/rigExecImaging/registryInvalidation.cpp
    # Upstream inputs: chain-pulled tables, each session's slice, the
    # serial retirement and the live hand-off at a time.
    libs/rigExecImaging/registryUpstream.cpp
    # Diagnostics facade: read-only queries over the scheduler, bridge and
    # warm-index owners; no counters of its own.
    libs/rigExecImaging/registryDiagnostics.cpp)
add_library(rigExecImagingFrames OBJECT
    # Warm work: the frozen-work job factory, the lazy frozen refresh, the
    # standing burst sample cache and the per-lane workspace pool.
    libs/rigExecImaging/registryWarmWork.cpp
    # Warming triggers: the commit and idle enqueues with the sampling
    # budgets, the cursor sweep, the rewarm queue and the fallback fill.
    libs/rigExecImaging/registryWarming.cpp
    # Frame-cache control: settings refresh, the scheduler-fenced clear, a
    # rig's warm range and the warm-index frame states.
    libs/rigExecImaging/registryCache.cpp
    # Frames on demand: the frame-table stamp, the cached-frame read and the
    # demand service call (docs/specs/frames-on-demand.md).
    libs/rigExecImaging/registryFrameDemand.cpp)
add_library(rigExecImagingInteraction OBJECT
    # Weight overlay and manipulation preview: declared slots sampled as
    # evaluator overrides and chain xform deltas.
    libs/rigExecImaging/registryPreview.cpp)
add_library(rigExecImagingQueries OBJECT
    # Guide geometry queries: published/subtree bounds and the
    # UsdGeomBoundable extent callback.
    libs/rigExecImaging/guideBounds.cpp
    libs/rigExecImaging/guideExtent.cpp)
add_library(rigExecImagingHost OBJECT
    # Host-facing C surface: the extern "C" activation and query exports.
    libs/rigExecImaging/registryCapi.cpp)

# The object libraries compile against the usage requirements the shared
# target compiles its own sources against; a shared target's requirements
# do not propagate backwards into linked object libraries. PIC because
# the objects become part of a shared library; the export define because
# the C API headers must compile dllexport, not dllimport, on Windows.
# rigExecScene enters as link-only: its INTERFACE link anchor is a source,
# and every target that links the scene library compiles its own copy.
# The shared target links rigExecScene plainly and carries the one anchor
# into the DSO.
foreach(_rigexec_imaging_obj IN ITEMS
        rigExecImagingContext rigExecImagingFrames
        rigExecImagingInteraction rigExecImagingQueries
        rigExecImagingHost)
    target_include_directories(${_rigexec_imaging_obj} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>)
    target_link_libraries(${_rigexec_imaging_obj} PUBLIC
        rigExec
        rigExecPlayback
        hd
        usdImaging
        usdUtils
        # TouchPose: the wrapped terminal shader is generated through Sdr from
        hio
        sdr
        work
    )
    target_link_libraries(${_rigexec_imaging_obj} PRIVATE
        rigExecRuntime rigExecSampler $<LINK_ONLY:rigExecScene> rigExecGpu)
    target_compile_definitions(${_rigexec_imaging_obj}
        PRIVATE RIGEXEC_IMAGING_EXPORTS)
    set_target_properties(${_rigexec_imaging_obj} PROPERTIES
        POSITION_INDEPENDENT_CODE ON)
endforeach()

add_library(rigExecImaging SHARED
    libs/rigExecImaging/sceneIndices.cpp
    libs/rigExecImaging/bridge.cpp
    libs/rigExecImaging/bridgeMotion.cpp
    libs/rigExecImaging/bridgeWarmPublication.cpp
    libs/rigExecImaging/bridgeFreshness.cpp
    libs/rigExecImaging/bridgeGuideGeometry.cpp
    libs/rigExecImaging/bridgeGuides.cpp
    libs/rigExecImaging/bridgeVolumeGuides.cpp
    libs/rigExecImaging/bridgeSnapshot.cpp
    libs/rigExecImaging/bridgeSampling.cpp
    libs/rigExecImaging/bridgeFrameCache.cpp
    libs/rigExecImaging/bridgeCacheFreshness.cpp
    libs/rigExecImaging/playback.cpp
    libs/rigExecImaging/sceneIndexPlugin.cpp
    libs/rigExecImaging/rigAdapter.cpp
    libs/rigExecImaging/touchOverlayAdapter.cpp
    # TouchPose: posed-mesh picking and the Storm shader highlight.
    libs/rigExecImaging/touchPoseMesh.cpp
    libs/rigExecImaging/touchPoseHighlight.cpp
    libs/rigExecImaging/touchPoseApi.cpp
    # The 1.4 warm-frame index: shared-owned per-frame completions.
    libs/rigExecImaging/warmIndex.cpp
    libs/rigExecImaging/profilerApi.cpp
    libs/rigExecImaging/liveDebug.cpp
    # Pulled upstream scene-index inputs (rigExecInputs).
    libs/rigExecImaging/upstreamTable.cpp
    libs/rigExecImaging/frameTable.cpp
)
target_include_directories(rigExecImaging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
# Compose the domain object libraries into the shared library first: CMake
# adds their objects to this target only, never to its consumers, and
# composing first keeps their link order over the scene library ahead of
# the dependency archives.
target_link_libraries(rigExecImaging PRIVATE
    rigExecImagingContext rigExecImagingFrames rigExecImagingInteraction
    rigExecImagingQueries rigExecImagingHost)
target_link_libraries(rigExecImaging PUBLIC
    rigExec
    rigExecPlayback
    hd
    usdImaging
    usdUtils
    # TouchPose: the wrapped terminal shader is generated through Sdr from
    hio
    sdr
    work
)
target_link_libraries(rigExecImaging PRIVATE rigExecRuntime rigExecSampler rigExecScene rigExecGpu)
target_compile_definitions(rigExecImaging PRIVATE RIGEXEC_IMAGING_EXPORTS)
endif()  # RIGEXEC_WITH_USD

# The vendored FlatBuffers C++ runtime headers (Apache-2.0, unmodified; see
# thirdparty/flatbuffers/README.md). The generated headers under
# libs/rigExecBinary/generated static_assert this exact version.
add_library(rigExecFlatBuffers INTERFACE)
target_include_directories(rigExecFlatBuffers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/flatbuffers/include>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
add_library(rigExec::rigExecFlatBuffers ALIAS rigExecFlatBuffers)

add_library(rigExecLzma STATIC
    thirdparty/lzma/C/LzmaEnc.c
    thirdparty/lzma/C/LzmaDec.c
    thirdparty/lzma/C/LzFind.c
    thirdparty/lzma/C/CpuArch.c)
target_include_directories(rigExecLzma PRIVATE thirdparty/lzma/C)
target_compile_definitions(rigExecLzma PRIVATE Z7_ST RIGEXEC_LZMA_PORTABLE_SCALAR)
# Vendored code keeps its own floating-point codegen: the exact-FP walk
# exempts targets by this marker, never by source-path text.
set_target_properties(rigExecLzma PROPERTIES POSITION_INDEPENDENT_CODE ON
    RIGEXEC_VENDORED TRUE)
add_library(rigExec::rigExecLzma ALIAS rigExecLzma)

add_library(rigExecBinary STATIC libs/rigExecBinary/format.cpp
    libs/rigExecBinary/manifest.cpp
    libs/rigExecBinary/transport.cpp)
add_library(rigExec::rigExecBinary ALIAS rigExecBinary)
target_include_directories(rigExecBinary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_include_directories(rigExecBinary PRIVATE thirdparty/lzma/C)
target_link_libraries(rigExecBinary PUBLIC rigExecFlatBuffers rigExecOpCore
    PRIVATE rigExecLzma)
set_target_properties(rigExecBinary PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(rigExecRuntime STATIC
    libs/rigExecRuntime/open.cpp
    libs/rigExecRuntime/exec.cpp
    libs/rigExecRuntime/closure.cpp
    libs/rigExecRuntime/publish.cpp
    libs/rigExecRuntime/kernels.cpp
    libs/rigExecRuntime/pose.cpp
    libs/rigExecRuntime/poseConstraints.cpp
    libs/rigExecRuntime/poseMath.cpp
    libs/rigExecRuntime/poseSolvers.cpp
    libs/rigExecRuntime/poseSteps.cpp
    libs/rigExecRuntime/geometry.cpp
    libs/rigExecRuntime/weights.cpp
    libs/rigExecRuntime/inputs.cpp
    libs/rigExecRuntime/labels.cpp
    libs/rigExecRuntime/opNodes.cpp
    libs/rigExecRuntime/properties.cpp
    libs/rigExecRuntime/spaces.cpp)
target_include_directories(rigExecRuntime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecRuntime PUBLIC rigExecBinary rigExecGraph rigExecEvalCore
    rigExecOpCore)
set_target_properties(rigExecRuntime PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Installed names, so a mover plugin's tests can export and replay its rigs
# the same way inside this build and against an installed SDK.
add_library(rigExec::rigExecRuntime ALIAS rigExecRuntime)
rigexec_add_mover(ClothMover BUILTIN
    DEFINITION libs/rigExec/internal/movers/clothDefinition.h
    TYPE rigExec::RigExecClothDefinition
    USD_ADAPTER libs/rigExec/internal/movers/clothMover.cpp
    FACTORY RigExecClothUsdAdapter
    SCHEMA libs/rigExecSchema/schema.usda
    SCHEMA_TYPE RigExecClothMover)
# The standalone compiler check consumes the same generated module sources.
# Snapshot SOURCES at configure time: the generator expression is evaluated
# at generate time, when MSVC precompiled-header support has injected the
# synthetic cmake_pch.cxx, and that compiler-generated unit must not reach
# a manifest a separate consumer root builds. Generated mover units are
# present here because rigexec_add_mover adds them with target_sources.
get_property(_rigexec_runtime_manifest_sources TARGET rigExecRuntime PROPERTY SOURCES)
list(FILTER _rigexec_runtime_manifest_sources EXCLUDE REGEX "(^|/|\\\\)(CMakeFiles/|cmake_pch\\.(cxx|cpp|c)$)")
list(JOIN _rigexec_runtime_manifest_sources "\n" _rigexec_runtime_manifest_text)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/runtimeSources.txt"
    CONTENT "${_rigexec_runtime_manifest_text}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/runtimeIncludes.txt"
    CONTENT "$<JOIN:$<TARGET_PROPERTY:rigExecRuntime,INCLUDE_DIRECTORIES>,\n>\n")
get_property(_rigexec_evalcore_manifest_sources TARGET rigExecEvalCore PROPERTY SOURCES)
list(FILTER _rigexec_evalcore_manifest_sources EXCLUDE REGEX "(^|/|\\\\)(CMakeFiles/|cmake_pch\\.(cxx|cpp|c)$)")
list(JOIN _rigexec_evalcore_manifest_sources "\n" _rigexec_evalcore_manifest_text)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/evalCoreSources.txt"
    CONTENT "${_rigexec_evalcore_manifest_text}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/evalCoreIncludes.txt"
    CONTENT "$<JOIN:$<TARGET_PROPERTY:rigExecEvalCore,INCLUDE_DIRECTORIES>,\n>\n")
# The op core and the built-in type and op libraries: a host that builds the
# runtime from these manifests compiles them too, and passes the libraries'
# entries (RIGEXEC_TYPE_LIBRARY_ENTRY, RIGEXEC_OP_LIBRARY_ENTRY) to
# RigExecRuntimeReader::Open for files with op nodes.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/opCoreSources.txt"
    CONTENT "$<JOIN:$<TARGET_PROPERTY:rigExecOpCore,SOURCES>;$<TARGET_PROPERTY:rigExecStdTypes,SOURCES>;$<TARGET_PROPERTY:rigExecRigTypes,SOURCES>;$<TARGET_PROPERTY:rigExecOps,SOURCES>,\n>\n")

set(RIGEXEC_SDK_MANIFEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdkManifests")
rigexec_add_sdk_manifest_export(
    SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}"
    BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}"
    OUT_DIR "${RIGEXEC_SDK_MANIFEST_DIR}")

# What a host that builds these USD-free sources outside CMake needs: the
# exact-FP flag list and the same source and include manifests this build
# checks. The installed copies come from the same generated files under the
# installed-manifest grammar stated in cmake/rigExecConfig.cmake.in: entries
# for generated translation units keep the reserved runtime-generated/
# component and join the installed data directory, the rest join the
# consumer's matching checkout.
rigexec_get_exact_fp_flags(_rigexec_exact_fp_flags)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/cmake/rigexec_flags.cmake")
list(JOIN _rigexec_exact_fp_flags "\n" _rigexec_exact_fp_text)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/exactFpFlags.txt"
    CONTENT "${_rigexec_exact_fp_text}\n")
rigexec_add_test(NAME testRigExecExactFpFragment
    COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
        "-DSTAGED_DIR=${RIGEXEC_SDK_MANIFEST_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkExactFpFragment.cmake")
rigexec_add_test(NAME testRigExecInstallStaging
    COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
        "-DSCRATCH_DIR=${CMAKE_CURRENT_BINARY_DIR}/installStagingCheck"
        "-DINSTALL_DATADIR=${RIGEXEC_INSTALL_DATADIR}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkInstallStaging.cmake")
set_tests_properties(testRigExecInstallStaging PROPERTIES LABELS "fast"
    TIMEOUT 120)
# Effective-mode witness: configures and builds a throwaway project through
# the same helper, then checks the real compiler commands and runs numeric
# probes proving the exact mode survives generator expressions, per-source
# options, PCH creation and reuse, a third_party-named generated path, and a
# relocated build directory, and that unrankable link expressions abort the
# configuration by name. Needs only a C++ compiler.
rigexec_add_test(NAME testRigExecExactFpCompiler
    COMMAND "${CMAKE_COMMAND}"
        "-DWITNESS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/tests/exactFpWitness"
        "-DWITNESS_MODULE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigexec_flags.cmake"
        "-DBUILD_ROOT=${CMAKE_CURRENT_BINARY_DIR}/exactFpWitness"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DCXX_COMPILER=${CMAKE_CXX_COMPILER}"
        "-DCXX_COMPILER_ID=${CMAKE_CXX_COMPILER_ID}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/exactFpCompilerWitness.cmake")
set_tests_properties(testRigExecExactFpCompiler PROPERTIES TIMEOUT 300
    ENVIRONMENT "CMAKE_BUILD_PARALLEL_LEVEL=2")
rigexec_add_test(NAME testRigExecSdkInstallSpaces
    COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
        "-DSCRATCH_DIR=${CMAKE_CURRENT_BINARY_DIR}/sdkInstallSpacesCheck"
        "-DINSTALL_DATADIR=${RIGEXEC_INSTALL_DATADIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkSdkInstallSpaces.cmake")

if (RIGEXEC_WITH_USD)
# rigExecSampler: hands a .rigexec's Animated inputs the values a stage holds
# at each new time (RigExecInputSampler), for every consumer that plays the
# runtime over a stage. The runtime itself stays USD-free.
add_library(rigExecSampler STATIC
    libs/rigExecSampler/inputSampler.cpp
    libs/rigExecSampler/runtimePoseProjection.cpp)
target_include_directories(rigExecSampler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecSampler PUBLIC rigExecRuntime usd sdf tf gf)
set_target_properties(rigExecSampler PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecSampler ALIAS rigExecSampler)

# rigExecBake: .rigexec baking from a compiled evaluator (M1).
add_library(rigExecBake SHARED
    libs/rigExecBake/arrayReads.cpp
    libs/rigExecBake/bake.cpp
    libs/rigExecBake/serialize.cpp
    libs/rigExecBake/opGraph.cpp
    libs/rigExecBake/computedCapture.cpp
    libs/rigExecBake/pathTable.cpp
    libs/rigExecBake/revisionReads.cpp
    libs/rigExecBake/staticCapture.cpp
    libs/rigExecBake/staticReport.cpp)
target_include_directories(rigExecBake PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecBake PUBLIC
    rigExec
    rigExecBinary)
target_link_libraries(rigExecBake PRIVATE rigExecScene)
add_library(rigExec::rigExecBake ALIAS rigExecBake)

add_executable(rigExecBakeTool tools/rigExecBake.cpp)
set_target_properties(rigExecBakeTool PROPERTIES OUTPUT_NAME rigExecBake)
target_link_libraries(rigExecBakeTool PRIVATE rigExecBake)
target_compile_definitions(rigExecBakeTool PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

add_executable(rigExecReplayInputs tools/rigExecReplayInputs.cpp)
target_link_libraries(rigExecReplayInputs PRIVATE rigExec)

add_executable(rigExecPose tools/rigExecPose.cpp)
target_link_libraries(rigExecPose PRIVATE rigExec rigExecRuntime rigExecSampler)
target_compile_definitions(rigExecPose PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
endif()  # RIGEXEC_WITH_USD

# Bundle launcher for release downloads (packaging/README.md). No USD or
# RigExec dependency: it locates the bundled Python/usdview relative to
# itself, so it also builds in a tree configured without Python.
option(RIGEXEC_BUILD_VIEWER_LAUNCHER
       "Build the usdRigView launcher for release bundles" OFF)
if (RIGEXEC_BUILD_VIEWER_LAUNCHER)
    add_executable(usdRigView packaging/launcher/usdRigView.cpp)
    target_compile_features(usdRigView PRIVATE cxx_std_17)
    set_target_properties(usdRigView PROPERTIES OUTPUT_NAME usdRigView)
endif()

if (CMAKE_CROSSCOMPILING)
    set(_rigexec_tests_default OFF)
else()
    set(_rigexec_tests_default ON)
endif()
option(RIGEXEC_BUILD_TESTS "Build the RigExec test and probe executables"
       ${_rigexec_tests_default})

# The USD install the tests RUN against: its bin/lib go on the test
# PATH, and RIGEXEC_USD_INSTALL points the Python helpers at its
# site-packages. This is USD_INSTALL_DIR unless that is a CMake shim
# rather than a real install (as with the Houdini build, where the
# shim lives in cmake/houdini but the binaries come from HOUDINI_ROOT).
set(RIGEXEC_TEST_USD_DIR "${USD_INSTALL_DIR}"
    CACHE PATH "USD install whose binaries the tests run against")


if (RIGEXEC_BUILD_TESTS)
enable_testing()
find_package(Threads REQUIRED)

if (RIGEXEC_WITH_USD)
set(_rigexec_schema_resource_define
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
# The generated imaging plugInfo, for the test that drives the real
set(_rigexec_imaging_resource_define
    RIGEXEC_IMAGING_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources")

add_executable(testRigExecSceneDependency tests/testRigExecSceneDependency.cpp)
target_compile_definitions(testRigExecSceneDependency PRIVATE RIGEXEC_EXPECT_SCENE)
target_link_libraries(testRigExecSceneDependency PRIVATE rigExecScene)
rigexec_add_test(NAME testRigExecSceneDependency COMMAND testRigExecSceneDependency
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

# Deliberately core-only: missing scene support is an error, not process exit.
add_executable(testRigExecSceneMissing tests/testRigExecSceneDependency.cpp)
target_link_libraries(testRigExecSceneMissing PRIVATE rigExec)
rigexec_add_test(NAME testRigExecSceneMissing COMMAND testRigExecSceneMissing
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

add_executable(testRigExecSceneRecovery tests/testRigExecSceneDependency.cpp)
target_compile_definitions(testRigExecSceneRecovery PRIVATE RIGEXEC_TEST_SCENE_RECOVERY)
target_link_libraries(testRigExecSceneRecovery PRIVATE rigExecScene)
rigexec_add_test(NAME testRigExecSceneRecovery COMMAND testRigExecSceneRecovery
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

foreach(_case Dependency Missing Recovery)
    add_executable(testRigExecOracle${_case} tests/testRigExecOracleDependency.cpp)
    target_link_libraries(testRigExecOracle${_case} PRIVATE rigExecScene)
    rigexec_add_test(NAME testRigExecOracle${_case} COMMAND testRigExecOracle${_case}
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_tests_properties(testRigExecOracle${_case} PROPERTIES
        ENVIRONMENT "RIGEXEC_GOLDEN_SUITE=;RIGEXEC_INPUT_REPLAY=")
endforeach()
target_compile_definitions(testRigExecOracleDependency PRIVATE RIGEXEC_EXPECT_ORACLE)
target_compile_definitions(testRigExecOracleRecovery PRIVATE RIGEXEC_TEST_ORACLE_RECOVERY)
target_link_libraries(testRigExecOracleDependency PRIVATE rigExecOracle)
target_link_libraries(testRigExecOracleRecovery PRIVATE rigExecOracle)
rigexec_add_test(NAME testRigExecOracleGoldenSuiteMissing COMMAND "${CMAKE_COMMAND}"
    "-DDRIVER=$<TARGET_FILE:testRigExecOracleMissing>"
    "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkOracleGoldenSuiteMissing.cmake")

add_subdirectory(tests/externalMovers)
add_subdirectory(tests/opPlugins)
add_executable(testRigExecOpPlugins tests/testRigExecOpPlugins.cpp)
target_link_libraries(testRigExecOpPlugins PRIVATE rigExec rigExecScene
    rigExecBuiltins plug usd)
add_dependencies(testRigExecOpPlugins testOffsetOps)
foreach(_mode good abi symbol late)
    rigexec_add_test(NAME testRigExecOpPlugins_${_mode} COMMAND testRigExecOpPlugins
        ${_mode}
        "$<TARGET_GENEX_EVAL:testOffsetOps,$<TARGET_PROPERTY:testOffsetOps,RIGEXEC_PLUGIN_RESOURCE_DIR>>"
        "$<TARGET_FILE:testOffsetOps>"
        "${CMAKE_CURRENT_BINARY_DIR}/testRigExecOpPlugins-scratch/${_mode}")
endforeach()
add_dependencies(testRigExecOpPlugins testExternalMover testQuadraticOps)
foreach(_mode scopeUsed scopeOther)
    rigexec_add_test(NAME testRigExecOpPlugins_${_mode} COMMAND testRigExecOpPlugins
        ${_mode}
        "$<TARGET_GENEX_EVAL:testOffsetOps,$<TARGET_PROPERTY:testOffsetOps,RIGEXEC_PLUGIN_RESOURCE_DIR>>"
        "$<TARGET_FILE:testOffsetOps>"
        "${CMAKE_CURRENT_BINARY_DIR}/testRigExecOpPlugins-scratch/${_mode}"
        "$<TARGET_GENEX_EVAL:testExternalMover,$<TARGET_PROPERTY:testExternalMover,RIGEXEC_PLUGIN_RESOURCE_DIR>>"
        "$<TARGET_GENEX_EVAL:testQuadraticOps,$<TARGET_PROPERTY:testQuadraticOps,RIGEXEC_PLUGIN_RESOURCE_DIR>>")
endforeach()
add_executable(testRigExecOpPluginScene tests/testRigExecOpPluginScene.cpp)
target_link_libraries(testRigExecOpPluginScene PRIVATE rigExec rigExecScene
    rigExecBuiltins plug usd)
add_dependencies(testRigExecOpPluginScene testExternalMover testQuadraticOps)
rigexec_add_test(NAME testRigExecOpPluginScene COMMAND testRigExecOpPluginScene
    "$<TARGET_GENEX_EVAL:testExternalMover,$<TARGET_PROPERTY:testExternalMover,RIGEXEC_PLUGIN_RESOURCE_DIR>>"
    "$<TARGET_GENEX_EVAL:testQuadraticOps,$<TARGET_PROPERTY:testQuadraticOps,RIGEXEC_PLUGIN_RESOURCE_DIR>>")
add_executable(testRigExecExternalMovers tests/testRigExecExternalMovers.cpp)
target_link_libraries(testRigExecExternalMovers PRIVATE
    rigExec rigExecAuthoring rigExecBake rigExecRuntime usd usdGeom plug)
add_dependencies(testRigExecExternalMovers testExternalMover)
rigexec_add_test(NAME testRigExecExternalMovers COMMAND testRigExecExternalMovers
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    "$<TARGET_GENEX_EVAL:testExternalMover,$<TARGET_PROPERTY:testExternalMover,RIGEXEC_PLUGIN_RESOURCE_DIR>>"
    "${CMAKE_CURRENT_BINARY_DIR}/registeredMover.rigexec")
add_executable(testRigExecRegisteredMover tests/testRigExecRegisteredMover.cpp)
target_link_libraries(testRigExecRegisteredMover PRIVATE rigExecRuntime rigExecBuiltins
    testExternalMoverKernel)
rigexec_add_test(NAME testRigExecRegisteredMover COMMAND testRigExecRegisteredMover
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/registeredMover.rigexec")
# TestRegistration asserts on the global loader, so an ambient plugin path
# naming a stale third-party mover must not leak in. The test registers
# both directories explicitly, so discovery adds nothing.
set_tests_properties(testRigExecExternalMovers PROPERTIES ENVIRONMENT
    "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

endif()  # RIGEXEC_WITH_USD
# The mover schema generator must work without USD Python (CI's OpenUSD
# has none): the text reader is golden-tested on its own, and checked
# against the pxr/usdGenSchema path wherever pxr imports. The interpreter
# probe inside rigexec_add_mover stays function-local, so probe again here.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
rigexec_add_test(NAME testRigExecMoverSchemaGen COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testRigExecMoverSchemaGen.py"
    "${RIGEXEC_TEST_USD_DIR}")

if (TARGET usdRigView)
    rigexec_add_test(NAME testRigExecBundlePackaging COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/packaging/test_assemble_bundle.py"
        "$<TARGET_FILE:usdRigView>")
    set_tests_properties(testRigExecBundlePackaging PROPERTIES
        LABELS "fast" TIMEOUT 120)
endif()

rigexec_add_test(NAME testRigExecMacSigning COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/packaging/test_sign_macos.py")
set_tests_properties(testRigExecMacSigning PROPERTIES
    LABELS "fast" TIMEOUT 120)

# The test selector must attribute an OBJECT library's sources to the
# targets that compose it, or an imaging domain edit would select no
# test at all. The self-check runs the selector's parse()/select()
# against synthetic CMake text and this repository's imaging inventory.
rigexec_add_test(NAME testRigExecTestSelectorObjectSources
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/select_affected_tests.py" --self-test)

# The allocation checker is textual, so its extractor is golden-tested on
# synthetic sources of every banned form before the repository scan runs.
rigexec_add_test(NAME testRigExecAllocationChecker
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testRigExecAllocationChecker.py")
set_tests_properties(testRigExecAllocationChecker PROPERTIES LABELS "fast")

# No target links, includes or imports an OpenUSD library usdRig dropped
# (tools/check_pxr_libraries.py, FORBIDDEN).
rigexec_add_test(NAME testRigExecPxrLibraryChecker
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testRigExecPxrLibraryChecker.py")
set_tests_properties(testRigExecPxrLibraryChecker PROPERTIES LABELS "fast")

# The legal packaging layer is stdlib-only. The manifest must partition the
# tree exactly once, name every shipped legal text by its canonical file name,
# and agree with the install prefix and the shipped archives member by member.
rigexec_add_test(NAME testRigExecLegalPackaging
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testRigExecLegalPackaging.py")

rigexec_add_test(NAME testRigExecLicenseManifest
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_licenses.py")
set_tests_properties(testRigExecLicenseManifest PROPERTIES LABELS "fast")

# The checker's own rules are proven on synthetic trees and archives, so a
# broken rule cannot pass against a manifest that agrees with itself.
rigexec_add_test(NAME testRigExecLicenseChecker
    COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_licenses.py" --self-test)
set_tests_properties(testRigExecLicenseChecker PROPERTIES LABELS "fast")

if (RIGEXEC_WITH_USD)
add_executable(testRigExecMath tests/testRigExecMath.cpp)
add_executable(testRigExecWrinkleMath tests/testRigExecWrinkleMath.cpp)
target_link_libraries(testRigExecWrinkleMath PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecWrinkleMath COMMAND testRigExecWrinkleMath)
add_executable(testRigExecWrinkle tests/testRigExecWrinkle.cpp)
target_link_libraries(testRigExecWrinkle PRIVATE rigExec rigExecAuthoring rigExecBake rigExecRuntime rigExecSampler)
rigexec_add_test(NAME testRigExecWrinkle COMMAND testRigExecWrinkle
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    "${CMAKE_CURRENT_BINARY_DIR}/wrinkle_test.rigexec")
set_tests_properties(testRigExecWrinkle PROPERTIES FIXTURES_SETUP WrinkleBinary)
add_executable(testRigExecWrinkleRuntime tests/testRigExecWrinkleRuntime.cpp)
target_link_libraries(testRigExecWrinkleRuntime PRIVATE rigExecRuntime rigExecBuiltins)
rigexec_add_test(NAME testRigExecWrinkleRuntime COMMAND testRigExecWrinkleRuntime
    "${CMAKE_CURRENT_BINARY_DIR}/wrinkle_test.rigexec")
set_tests_properties(testRigExecWrinkleRuntime PROPERTIES FIXTURES_REQUIRED WrinkleBinary)
endif()  # RIGEXEC_WITH_USD
add_executable(testRigExecSkinSplit tests/testRigExecSkinSplit.cpp)
target_link_libraries(testRigExecSkinSplit PRIVATE rigExecRuntime)
rigexec_add_test(NAME testRigExecSkinSplit COMMAND testRigExecSkinSplit)
add_executable(testRigExecRuntimeFixture tests/testRigExecRuntimeFixture.cpp)
target_link_libraries(testRigExecRuntimeFixture PRIVATE rigExecRuntime)
rigexec_add_test(NAME testRigExecRuntimeFixture COMMAND testRigExecRuntimeFixture
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/minimal.rigexec")
add_executable(testRigExecRuntimeOpCore tests/testRigExecRuntimeOpCore.cpp)
target_include_directories(testRigExecRuntimeOpCore PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tests")
target_link_libraries(testRigExecRuntimeOpCore PRIVATE rigExecRuntime rigExecOpCore)
rigexec_add_test(NAME testRigExecRuntimeOpCore COMMAND testRigExecRuntimeOpCore
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/minimal.rigexec")
add_executable(testRigExecManifest tests/testRigExecManifest.cpp)
target_link_libraries(testRigExecManifest PRIVATE rigExecBinary)
rigexec_add_test(NAME testRigExecManifest COMMAND testRigExecManifest
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/minimal.rigexec")
add_executable(testRigExecSharedBuffers tests/testRigExecSharedBuffers.cpp)
target_link_libraries(testRigExecSharedBuffers PRIVATE rigExecRuntime)
rigexec_add_test(NAME testRigExecSharedBuffers COMMAND testRigExecSharedBuffers)
if (RIGEXEC_WITH_USD)
add_executable(testRigExecDeltaMush tests/testRigExecDeltaMush.cpp)
target_link_libraries(testRigExecDeltaMush PRIVATE rigExec rigExecAuthoring rigExecBake rigExecRuntime rigExecSampler)
rigexec_add_test(NAME testRigExecDeltaMush COMMAND testRigExecDeltaMush "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
add_executable(testRigExecDeltaMushRuntime tests/testRigExecDeltaMushRuntime.cpp)
target_link_libraries(testRigExecDeltaMushRuntime PRIVATE rigExecRuntime)
rigexec_add_test(NAME testRigExecDeltaMushBakeFile COMMAND rigExecBakeTool
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/delta_mush_mover.usda"
    --rig /Rig --time 1
    -o "${CMAKE_CURRENT_BINARY_DIR}/delta_mush_test.rigexec")
set_tests_properties(testRigExecDeltaMushBakeFile PROPERTIES FIXTURES_SETUP DeltaMushBinary)
rigexec_add_test(NAME testRigExecDeltaMushRuntime COMMAND testRigExecDeltaMushRuntime
    "${CMAKE_CURRENT_BINARY_DIR}/delta_mush_test.rigexec")
set_tests_properties(testRigExecDeltaMushRuntime PROPERTIES FIXTURES_REQUIRED DeltaMushBinary)
target_link_libraries(testRigExecMath PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecMath COMMAND testRigExecMath)

add_executable(testRigExecSingleChainIk tests/testRigExecSingleChainIk.cpp)
target_link_libraries(testRigExecSingleChainIk PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecSingleChainIk COMMAND testRigExecSingleChainIk)

add_executable(testRigExecGpuDeform tests/testRigExecGpuDeform.cpp)
target_link_libraries(testRigExecGpuDeform PRIVATE rigExec rigExecGpu)
rigexec_add_test(NAME testRigExecGpuDeform COMMAND testRigExecGpuDeform)
# The same program with the executor's start forced to throw: every
# concurrent Get() must still settle on the terminal failed state.
rigexec_add_test(NAME testRigExecGpuDeformStartupThrow COMMAND testRigExecGpuDeform)
set_tests_properties(testRigExecGpuDeformStartupThrow PROPERTIES
    ENVIRONMENT "RIGEXEC_GPU_TEST_THROW_STARTUP=1")
# The cloth's device solve against the CPU kernel, and a live rig on it.
add_executable(testRigExecGpuCloth tests/testRigExecGpuCloth.cpp)
target_link_libraries(testRigExecGpuCloth PRIVATE rigExec rigExecScene rigExecGpu)
target_compile_definitions(testRigExecGpuCloth PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecGpuCloth COMMAND testRigExecGpuCloth)
add_executable(testRigExecDualQuat tests/testRigExecDualQuat.cpp)
target_link_libraries(testRigExecDualQuat PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecDualQuat COMMAND testRigExecDualQuat)

# The shared kernel's contract over both host traits; no stage, no plugin.
add_executable(testRigExecDualQuatKernel tests/testRigExecDualQuatKernel.cpp)
target_link_libraries(testRigExecDualQuatKernel PRIVATE rigExecMathGf rigExecRuntime)
rigexec_add_test(NAME testRigExecDualQuatKernel COMMAND testRigExecDualQuatKernel)
set_tests_properties(testRigExecDualQuatKernel PROPERTIES LABELS "fast")

add_executable(testRigExecSplineIk tests/testRigExecSplineIk.cpp)
target_link_libraries(testRigExecSplineIk PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecSplineIk COMMAND testRigExecSplineIk)

add_executable(testRigExecRbf tests/testRigExecRbf.cpp)
target_link_libraries(testRigExecRbf PRIVATE rigExecMathGf)
target_compile_definitions(testRigExecRbf PRIVATE
    RIGEXEC_RBF_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/psd_parity.json")
rigexec_add_test(NAME testRigExecRbf COMMAND testRigExecRbf)
endif()  # RIGEXEC_WITH_USD

# The RigLogic kernels are header-only and USD-free; rigExecGraph supplies
# only the include root.
add_executable(testRigExecRigLogicKernels tests/testRigExecRigLogicKernels.cpp)
target_link_libraries(testRigExecRigLogicKernels PRIVATE rigExecGraph)
rigexec_add_test(NAME testRigExecRigLogicKernels COMMAND testRigExecRigLogicKernels)

# The harmonics and cloth kernels are header-only and USD-free.
add_executable(testRigExecHarmonicsKernel tests/testRigExecHarmonicsKernel.cpp)
target_link_libraries(testRigExecHarmonicsKernel PRIVATE rigExecGraph)
rigexec_add_test(NAME testRigExecHarmonicsKernel COMMAND testRigExecHarmonicsKernel)
add_executable(testRigExecClothKernel tests/testRigExecClothKernel.cpp)
target_link_libraries(testRigExecClothKernel PRIVATE rigExecGraph)
rigexec_add_test(NAME testRigExecClothKernel COMMAND testRigExecClothKernel)

if (RIGEXEC_WITH_USD)
# RigLogic's behaviours in a rig: live, frame cache and .rigexec parity.
add_executable(testRigExecRigLogic tests/testRigExecRigLogic.cpp)
target_link_libraries(testRigExecRigLogic PRIVATE rigExec rigExecAuthoring
    rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecRigLogic PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecRigLogic COMMAND testRigExecRigLogic)

# Secondary motion in a rig: native evaluation against the kernel, random
# access, drags, the measuring space, the envelope and the history memo.
add_executable(testRigExecHarmonics tests/testRigExecHarmonics.cpp)
target_link_libraries(testRigExecHarmonics PRIVATE rigExec rigExecScene rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecHarmonics PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecHarmonics COMMAND testRigExecHarmonics)

# Cloth in a rig: native evaluation against the kernel, random access,
# drags, a background job and .rigexec parity with the runtime's own kernel.
add_executable(testRigExecCloth tests/testRigExecCloth.cpp)
target_link_libraries(testRigExecCloth PRIVATE rigExec rigExecScene rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecCloth PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_CLOTH_EXAMPLE="${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/cloth_mover.usda")
rigexec_add_test(NAME testRigExecCloth COMMAND testRigExecCloth)

add_executable(testRigExecConstraints tests/testRigExecConstraints.cpp)
target_link_libraries(testRigExecConstraints PRIVATE rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecConstraints PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecConstraints COMMAND testRigExecConstraints)

add_executable(testRigExecInteropNotices tests/testRigExecInteropNotices.cpp)
target_link_libraries(testRigExecInteropNotices PRIVATE rigExec)
rigexec_add_test(NAME testRigExecInteropNotices COMMAND testRigExecInteropNotices)

set(RIGEXEC_GOLDEN_SUITE_DIR "" CACHE PATH "Independent original-source suite goldens")
function(rigexec_add_graph_test name)
    cmake_parse_arguments(A "CONE_VERIFY;SEPARATE_PARALLEL" "" "" ${ARGN})
    set(_args "${A_UNPARSED_ARGUMENTS}")
    if (RIGEXEC_GOLDEN_SUITE_DIR)
        rigexec_add_test(NAME ${name}Golden COMMAND ${name} ${_args})
        set_tests_properties(${name}Golden PROPERTIES LABELS "golden"
            ENVIRONMENT "RIGEXEC_GOLDEN_SUITE=check:${RIGEXEC_GOLDEN_SUITE_DIR};RIGEXEC_GOLDEN_SUITE_NAME=${name};RIGEXEC_ENABLE_PARALLEL_EVAL=0;RIGEXEC_BAKED_SCHEDULE=serial")
    endif()
    foreach(_schedule serial parallel)
        # The base entry covers parallel execution unless its arguments or
        # environment select a different case. Pin it against caller overrides.
        if (_schedule STREQUAL "parallel" AND NOT A_SEPARATE_PARALLEL)
            set_property(TEST ${name} APPEND PROPERTY ENVIRONMENT
                "RIGEXEC_BAKED_SCHEDULE=parallel" "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
        else()
            rigexec_add_test(NAME ${name}Graph_${_schedule} COMMAND ${name} ${_args})
            set_tests_properties(${name}Graph_${_schedule} PROPERTIES
                ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule};RIGEXEC_ENABLE_PARALLEL_EVAL=1")
        endif()
        if (A_CONE_VERIFY)
            foreach(_grain default 0)
                set(_cone_test "${name}Cones_${_schedule}")
                if (_grain STREQUAL "0")
                    set(_cone_test "${name}Cones_grain0_${_schedule}")
                endif()
                rigexec_add_test(NAME ${_cone_test} COMMAND ${name} ${_args})
                set_tests_properties(${_cone_test} PROPERTIES
                    ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1;RIGEXEC_BAKED_SCHEDULE=${_schedule};RIGEXEC_ENABLE_PARALLEL_EVAL=1")
                if (_grain STREQUAL "0")
                    set_property(TEST ${_cone_test} APPEND
                        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")
                endif()
            endforeach()
        endif()
    endforeach()
endfunction()
rigexec_add_graph_test(testRigExecConstraints CONE_VERIFY)
endif()  # RIGEXEC_WITH_USD

add_executable(testRigExecSkeleton2dKernel tests/testRigExecSkeleton2dKernel.cpp)
target_link_libraries(testRigExecSkeleton2dKernel PRIVATE rigExecMath)
rigexec_add_test(NAME testRigExecSkeleton2dKernel COMMAND testRigExecSkeleton2dKernel)

if (RIGEXEC_WITH_USD)
add_executable(testRigExecSkeleton2d tests/testRigExecSkeleton2d.cpp)
target_link_libraries(testRigExecSkeleton2d PRIVATE rigExec rigExecScene
    rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecSkeleton2d PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSkeleton2d COMMAND testRigExecSkeleton2d)

add_executable(testRigExecJointGroups tests/testRigExecJointGroups.cpp)
target_link_libraries(testRigExecJointGroups PRIVATE rigExec rigExecScene
    rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecJointGroups PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecJointGroups COMMAND testRigExecJointGroups)
include(cmake/FixtureChecks.cmake)
rigexec_add_fixture_checks(testRigExecSkeleton2d skeleton2d nested_chain /Char/Rig)

add_executable(testRigExecLayerBind tests/testRigExecLayerBind.cpp)
target_link_libraries(testRigExecLayerBind PRIVATE rigExec rigExecScene
    rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecLayerBind PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_LAYER_BIND_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/layerBind")
rigexec_add_test(NAME testRigExecLayerBind COMMAND testRigExecLayerBind)
rigexec_add_fixture_checks(testRigExecLayerBind layerBind layer_bind /Char/Rig)
rigexec_add_fixture_checks(testRigExecVolumeSkin volumeSkin chain_2d /Char/Rig)
rigexec_add_fixture_checks(testRigExecVolumeSkin volumeSkin arm_3d /Arm/Rig)
rigexec_add_fixture_checks(testRigExecVolumeSkin volumeSkin arm_3d_regenerated /Arm/Rig)

add_executable(testRigExecStepMaps tests/testRigExecStepMaps.cpp)
target_link_libraries(testRigExecStepMaps PRIVATE rigExec rigExecScene
    rigExecBake rigExecRuntime rigExecSampler rigExecImaging)
target_compile_definitions(testRigExecStepMaps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecStepMaps COMMAND testRigExecStepMaps)
rigexec_add_test(NAME testRigExecStepMapsChannels COMMAND testRigExecStepMaps channels)
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecStepMapsChannels_${_schedule}
        COMMAND testRigExecStepMaps channels)
    set_tests_properties(testRigExecStepMapsChannels_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule};RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

# Affine frame providers and the layered skin revision: numerical results,
# then a control edit that must re-run both through their declared inputs;
# the runtime's expressions against Gf's, and .rigexec playback.
add_executable(testRigExecAffineFrames tests/testRigExecAffineFrames.cpp)
target_link_libraries(testRigExecAffineFrames PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
rigexec_add_test(NAME testRigExecAffineFrames COMMAND testRigExecAffineFrames
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
set_tests_properties(testRigExecAffineFrames PROPERTIES ENVIRONMENT
    "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
rigexec_add_graph_test(testRigExecAffineFrames CONE_VERIFY
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
foreach(_entry testRigExecAffineFramesGolden
               testRigExecAffineFramesGraph_serial
               testRigExecAffineFramesCones_serial
               testRigExecAffineFramesCones_parallel
               testRigExecAffineFramesCones_grain0_serial
               testRigExecAffineFramesCones_grain0_parallel)
    if (TEST ${_entry})
        set_property(TEST ${_entry} APPEND PROPERTY ENVIRONMENT
            "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    endif()
endforeach()

# The exec-row goldens of one rig: testRigExecExecGoldens_<name> compares the
# production evaluator's rows with the checked-in set, and with
# RIGEXEC_RECORD_EXEC_ROWS=1 records them at the first, middle and last of
# FRAMES (tests/fixtures/README.md).
function(rigexec_add_exec_rows_test)
    cmake_parse_arguments(A "" "NAME;STAGE;FRAMES" "" ${ARGN})
    string(REPLACE "," ";" _all "${A_FRAMES}")
    list(LENGTH _all _count)
    math(EXPR _middle "${_count} / 2")
    math(EXPR _last "${_count} - 1")
    list(GET _all 0 ${_middle} ${_last} _frames)
    list(REMOVE_DUPLICATES _frames)
    rigexec_add_test(NAME testRigExecExecGoldens_${A_NAME} COMMAND testRigExecExecGoldens
        "${A_STAGE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/execRows"
        ${A_NAME} ${_frames})
    set_tests_properties(testRigExecExecGoldens_${A_NAME} PROPERTIES
        LABELS "golden" ENVIRONMENT "RIGEXEC_ENABLE_SIMD=0")
endfunction()

function(rigexec_add_example_graph_test)
    cmake_parse_arguments(A "GUIDES" "NAME;RIG;STAGE;FRAMES" "" ${ARGN})
    # DEFINED, not truthiness: a frame list that is the single frame 0 is
    # false to if().
    if (NOT DEFINED A_NAME OR NOT DEFINED A_FRAMES OR
            (NOT DEFINED A_RIG AND NOT DEFINED A_STAGE) OR
            (DEFINED A_RIG AND DEFINED A_STAGE))
        message(FATAL_ERROR "rigexec_add_example_graph_test: "
                            "NAME, FRAMES and one of RIG or STAGE are required")
    endif()
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_example_graph_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    set(_guides "")
    if (A_GUIDES)
        set(_guides "--guides")
    endif()
    if (DEFINED A_RIG)
        set(A_STAGE "${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}")
    endif()
    rigexec_add_test(NAME example_graph_${A_NAME} COMMAND rigExecPose
        "${A_STAGE}"
        --frames "${A_FRAMES}" --cpu-reference ${_guides})
    set_tests_properties(example_graph_${A_NAME} PROPERTIES
        ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1"
        FAIL_REGULAR_EXPRESSION "FAIL:")
    set(_schema_resources
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_property(TEST example_graph_${A_NAME} APPEND PROPERTY ENVIRONMENT
        "PXR_PLUGINPATH_NAME=${_schema_resources}")
    rigexec_add_exec_rows_test(NAME ${A_NAME} STAGE "${A_STAGE}"
        FRAMES "${A_FRAMES}")
endfunction()

include("${CMAKE_CURRENT_SOURCE_DIR}/tests/exampleFixtures.cmake")
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _fixture_fields "${_fixture}")
    list(GET _fixture_fields 0 _fixture_stage)
    list(GET _fixture_fields 1 _fixture_frames)
    # The entry name is the stage path with the separators CMake test names
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    rigexec_add_example_graph_test(NAME ${_fixture_name}
        RIG ${_fixture_stage} FRAMES ${_fixture_frames} GUIDES)
endforeach()

rigexec_add_example_graph_test(NAME biped_Biped_stack_anim
    RIG biped/Biped_stack_anim.usda FRAMES 1,50,100,150,200 GUIDES)

# Surface projectors whose rigExec:space, rigExec:sources and
# rigExec:sourceSpace are constrained controls that nothing else reads: the
# projector frames are built from base and final of all three.
rigexec_add_example_graph_test(NAME projector_spaces
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/projector_spaces.usda"
    FRAMES 1,4,7,10 GUIDES)

# Two volumes with one placement step each, one of them placed differently
# at base (a mover's packet) and at final (pose.weightFrames, a
# current-phase field) because a constraint revises its joint after the
# solver.
rigexec_add_example_graph_test(NAME volume_placements
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/volume_placements.usda"
    FRAMES 1,5,10 GUIDES)

# AtPrim read phases on rigExec:transform, read through frame records. The
# .rigexec runtime's FrameMatrix steps are held to the baked program by
# verify_binary_frame_record_fallbacks.
rigexec_add_example_graph_test(NAME frame_record_fallbacks
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/frame_record_fallbacks.usda"
    FRAMES 1,4,8 GUIDES)

# AtPrim read phases that name solver checkpoints, one of them a record the
# solver never publishes. rigExecPose's parity mode is the dynamic/baked
# comparison and never runs the CPU oracle, which has no AtPrim branch. The
# .rigexec runtime is held to the baked program by
# verify_binary_solver_checkpoint.
rigexec_add_example_graph_test(NAME solver_checkpoint
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/solver_checkpoint.usda"
    FRAMES 1,5,9 GUIDES)

# A lattice whose cage is its own target, read at `preceding`: it sees the
# points the movers before it left. This checks dynamic/baked parity only;
# testRigExecPhaseBindings pins the values, and
# verify_binary_preceding_own_chain holds the .rigexec runtime to them.
rigexec_add_example_graph_test(NAME preceding_own_chain
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/preceding_own_chain.usda"
    FRAMES 1,5,9 GUIDES)

# The RigLogic docs examples: an ML mover over a skinned arm, animated maps
# driving morphs and mesh primvars, and the pose interpolator's additive,
# interpolative and numeric solves. Frames sit between the authored keys;
# verify_binary_docs_* below holds the .rigexec runtime to the same stages.
set(RIGEXEC_RIGLOGIC_DOCS_EXAMPLES
    "ml_mover|1001,1008,1020,1031,1041"
    "animated_maps|1001,1011,1023,1035,1040"
    "pose_interpolator_riglogic|1001,1010,1013,1024,1037,1044")
foreach(_entry IN LISTS RIGEXEC_RIGLOGIC_DOCS_EXAMPLES)
    string(REPLACE "|" ";" _fields "${_entry}")
    list(GET _fields 0 _stage)
    list(GET _fields 1 _frames)
    rigexec_add_example_graph_test(NAME docs_${_stage}
        STAGE "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/${_stage}.usda"
        FRAMES ${_frames} GUIDES)
endforeach()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/tests/exampleFixtures.h.in"
               "${CMAKE_CURRENT_BINARY_DIR}/tests/rigExecExampleFixtures.h"
               @ONLY)
add_executable(testRigExecExampleParity tests/testRigExecExampleParity.cpp)
target_link_libraries(testRigExecExampleParity PRIVATE rigExec)
target_include_directories(testRigExecExampleParity PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecExampleParity PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecExampleParity COMMAND testRigExecExampleParity
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecExampleParity PROPERTIES
    FAIL_REGULAR_EXPRESSION "comparison mismatch")
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecExampleParityCones_${_schedule}
        COMMAND testRigExecExampleParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecExampleParityCones_${_schedule} PROPERTIES
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(TEST testRigExecExampleParityCones_${_schedule} APPEND
        PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    rigexec_add_test(NAME testRigExecExampleParityCones_grain0_${_schedule}
        COMMAND testRigExecExampleParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecExampleParityCones_grain0_${_schedule}
        PROPERTIES
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(TEST testRigExecExampleParityCones_grain0_${_schedule} APPEND
        PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

# Extended nearest-surface snapping, persistent surface bindings and
# regular-grid lattices against reference values; each also replays its
# .rigexec export.
add_executable(testRigExecSurfaceSnap tests/testRigExecSurfaceSnap.cpp)
target_link_libraries(testRigExecSurfaceSnap PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecSurfaceSnap PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSurfaceSnap COMMAND testRigExecSurfaceSnap)
rigexec_add_graph_test(testRigExecSurfaceSnap CONE_VERIFY)

add_executable(testRigExecSurfaceBinding tests/testRigExecSurfaceBinding.cpp)
target_link_libraries(testRigExecSurfaceBinding PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecSurfaceBinding PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSurfaceBinding COMMAND testRigExecSurfaceBinding)
rigexec_add_graph_test(testRigExecSurfaceBinding CONE_VERIFY)

add_executable(testRigExecPatchMover tests/testRigExecPatchMover.cpp)
target_link_libraries(testRigExecPatchMover PRIVATE
    rigExec rigExecAuthoring rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecPatchMover PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecPatchMover COMMAND testRigExecPatchMover)
rigexec_add_graph_test(testRigExecPatchMover CONE_VERIFY)

add_executable(testRigExecLattice tests/testRigExecLattice.cpp)
target_link_libraries(testRigExecLattice PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecLattice PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecLattice COMMAND testRigExecLattice)
rigexec_add_graph_test(testRigExecLattice CONE_VERIFY)

add_executable(testRigExecSurfaceProjector
    tests/testRigExecSurfaceProjector.cpp)
target_link_libraries(testRigExecSurfaceProjector PRIVATE rigExec)
target_compile_definitions(testRigExecSurfaceProjector PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSurfaceProjector
    COMMAND testRigExecSurfaceProjector "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Rigs under translated, rotated, non-uniformly scaled, animated ancestors
# above the asset root, near and far from the origin: native, frozen and
# .rigexec playback.
add_executable(testRigExecOutsideTransforms
    tests/testRigExecOutsideTransforms.cpp)
target_link_libraries(testRigExecOutsideTransforms PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler rigExecEvalCore)
target_compile_definitions(testRigExecOutsideTransforms PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecOutsideTransforms
    COMMAND testRigExecOutsideTransforms
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
foreach(_case IN ITEMS outside-target outside-target-far
                       projector-animated-ancestor)
    rigexec_add_test(NAME testRigExecOutsideTransforms_${_case}
        COMMAND testRigExecOutsideTransforms
            "${CMAKE_CURRENT_SOURCE_DIR}/examples"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures" ${_case})
endforeach()

add_executable(testRigExecGeometryOpsBakedParity
    tests/testRigExecGeometryOpsBakedParity.cpp)
target_link_libraries(testRigExecGeometryOpsBakedParity PRIVATE rigExec)
target_compile_definitions(testRigExecGeometryOpsBakedParity PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecGeometryOpsBakedParity
    COMMAND testRigExecGeometryOpsBakedParity
    "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
set_tests_properties(testRigExecGeometryOpsBakedParity PROPERTIES
    FAIL_REGULAR_EXPRESSION "comparison mismatch")
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecGeometryOpsBakedParityCones_${_schedule}
        COMMAND testRigExecGeometryOpsBakedParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
    set_tests_properties(testRigExecGeometryOpsBakedParityCones_${_schedule}
        PROPERTIES
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(TEST testRigExecGeometryOpsBakedParityCones_${_schedule}
        APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    rigexec_add_test(NAME testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        COMMAND testRigExecGeometryOpsBakedParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
    set_tests_properties(
        testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        PROPERTIES
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(
        TEST testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

endif()  # RIGEXEC_WITH_USD
rigexec_add_type_library(rigExecTestFakeTypes
    SOURCES tests/registryFixture/fakeTypes.cpp)
rigexec_add_op_library(rigExecTestFakeOps
    SOURCES tests/registryFixture/fakeOps.cpp
    USES_TYPE_LIBRARIES rigExecTestFakeTypes)
add_executable(testRigExecRegistry tests/testRigExecRegistry.cpp)
target_link_libraries(testRigExecRegistry PRIVATE rigExecOpCore)
target_include_directories(testRigExecRegistry PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs" "${CMAKE_CURRENT_SOURCE_DIR}/tests")
rigexec_link_registrations(testRigExecRegistry
    TYPE_LIBRARIES rigExecTestFakeTypes OP_LIBRARIES rigExecTestFakeOps)
rigexec_link_registrations(testRigExecRuntimeOpCore
    TYPE_LIBRARIES rigExecTestFakeTypes OP_LIBRARIES rigExecTestFakeOps)
rigexec_add_test(NAME testRigExecRegistry COMMAND testRigExecRegistry)

add_executable(testRigExecCore tests/testRigExecCore.cpp)
target_link_libraries(testRigExecCore PRIVATE rigExecOpCore)
target_include_directories(testRigExecCore PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs" "${CMAKE_CURRENT_SOURCE_DIR}/tests")
target_link_libraries(testRigExecCore PRIVATE Threads::Threads)
rigexec_link_registrations(testRigExecCore
    TYPE_LIBRARIES rigExecTestFakeTypes OP_LIBRARIES rigExecTestFakeOps)
rigexec_add_test(NAME testRigExecCore COMMAND testRigExecCore)
if (RIGEXEC_WITH_USD)
# The same body on the task system over OpenUSD's work library.
add_executable(testRigExecCoreWork tests/testRigExecCore.cpp)
target_compile_definitions(testRigExecCoreWork PRIVATE
    RIGEXEC_TEST_WORK_TASK_SYSTEM)
target_link_libraries(testRigExecCoreWork PRIVATE rigExecOpCore rigExecScene
    Threads::Threads)
target_include_directories(testRigExecCoreWork PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs" "${CMAKE_CURRENT_SOURCE_DIR}/tests")
rigexec_link_registrations(testRigExecCoreWork
    TYPE_LIBRARIES rigExecTestFakeTypes OP_LIBRARIES rigExecTestFakeOps)
rigexec_add_test(NAME testRigExecCoreWork COMMAND testRigExecCoreWork)
endif()  # RIGEXEC_WITH_USD

add_executable(testRigExecStdTypes tests/testRigExecStdTypes.cpp)
target_link_libraries(testRigExecStdTypes PRIVATE rigExecOpCore)
target_include_directories(testRigExecStdTypes PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecStdTypes PRIVATE Threads::Threads)
rigexec_link_registrations(testRigExecStdTypes
    TYPE_LIBRARIES rigExecStdTypes)
rigexec_add_test(NAME testRigExecStdTypes COMMAND testRigExecStdTypes)

add_executable(testRigExecRigTypes tests/testRigExecRigTypes.cpp)
target_link_libraries(testRigExecRigTypes PRIVATE rigExecOpCore)
target_include_directories(testRigExecRigTypes PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecRigTypes PRIVATE Threads::Threads)
rigexec_link_registrations(testRigExecRigTypes
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes)
rigexec_add_test(NAME testRigExecRigTypes COMMAND testRigExecRigTypes)

if (RIGEXEC_WITH_USD)
add_executable(testRigExecOpsMath tests/testRigExecOpsMath.cpp)
target_link_libraries(testRigExecOpsMath PRIVATE rigExecOpCore)
target_include_directories(testRigExecOpsMath PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecOpsMath PRIVATE rigExecMathGf Threads::Threads)
rigexec_link_registrations(testRigExecOpsMath
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecOpsMath COMMAND testRigExecOpsMath)
set_tests_properties(testRigExecOpsMath PROPERTIES LABELS "fast")
endif()  # RIGEXEC_WITH_USD
add_executable(testRigExecOpsSpaces tests/testRigExecOpsSpaces.cpp)
target_link_libraries(testRigExecOpsSpaces PRIVATE rigExecOpCore)
target_include_directories(testRigExecOpsSpaces PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecOpsSpaces PRIVATE Threads::Threads)
rigexec_link_registrations(testRigExecOpsSpaces
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecOpsSpaces COMMAND testRigExecOpsSpaces)
set_tests_properties(testRigExecOpsSpaces PROPERTIES LABELS "fast")
add_executable(testRigExecOpsPose tests/testRigExecOpsPose.cpp)
target_link_libraries(testRigExecOpsPose PRIVATE rigExecOpCore)
target_include_directories(testRigExecOpsPose PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecOpsPose PRIVATE rigExecEvalCore
    Threads::Threads)
rigexec_link_registrations(testRigExecOpsPose
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecOpsPose COMMAND testRigExecOpsPose)
set_tests_properties(testRigExecOpsPose PROPERTIES LABELS "fast")
add_executable(testRigExecGpuDevice tests/testRigExecGpuDevice.cpp)
target_link_libraries(testRigExecGpuDevice PRIVATE rigExecOpCore
    rigExecEvalCore Threads::Threads)
rigexec_add_test(NAME testRigExecGpuDevice COMMAND testRigExecGpuDevice)
add_executable(rigExecGenSchema tools/rigExecGenSchema.cpp)
target_link_libraries(rigExecGenSchema PRIVATE rigExecOpCore)
target_include_directories(rigExecGenSchema PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(rigExecGenSchema PRIVATE Threads::Threads)
rigexec_link_registrations(rigExecGenSchema
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecSchemaReport COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/schema_report.py"
    --tool $<TARGET_FILE:rigExecGenSchema>
    --schema "${CMAKE_CURRENT_SOURCE_DIR}/libs/rigExecSchema/schema.usda")
set_tests_properties(testRigExecSchemaReport PROPERTIES LABELS "fast")
if (RIGEXEC_WITH_USD)
add_executable(testRigExecOpsPoints tests/testRigExecOpsPoints.cpp)
target_link_libraries(testRigExecOpsPoints PRIVATE rigExecOpCore)
target_include_directories(testRigExecOpsPoints PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecOpsPoints PRIVATE rigExecMathGf Threads::Threads)
rigexec_link_registrations(testRigExecOpsPoints TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecOpsPoints COMMAND testRigExecOpsPoints)
set_tests_properties(testRigExecOpsPoints PROPERTIES LABELS "fast")
endif()  # RIGEXEC_WITH_USD
add_executable(testRigExecOpPayloads tests/testRigExecOpPayloads.cpp)
target_include_directories(testRigExecOpPayloads PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libs")
target_link_libraries(testRigExecOpPayloads PRIVATE rigExecOpCore rigExecMath)
rigexec_link_registrations(testRigExecOpPayloads
    TYPE_LIBRARIES rigExecStdTypes rigExecRigTypes OP_LIBRARIES rigExecOps)
rigexec_add_test(NAME testRigExecOpPayloads COMMAND testRigExecOpPayloads)
set_tests_properties(testRigExecOpPayloads PROPERTIES LABELS "fast")

if (RIGEXEC_WITH_USD)
# Host-side bridge from translated USD properties to op nodes; not installed.
add_library(rigExecSceneOps STATIC libs/rigExecScene/propertyOps.cpp)
target_link_libraries(rigExecSceneOps PUBLIC rigExec rigExecScene
    rigExecBuiltins PRIVATE rigExecMath)
set_target_properties(rigExecSceneOps PROPERTIES POSITION_INDEPENDENT_CODE ON)
# One op node per call, for the tests that compare an op with its kernel.
add_library(rigExecTestOpRunners STATIC tests/opRunners/geometryOpRunner.cpp
    tests/opRunners/poseOpRunner.cpp tests/opRunners/weightOpRunner.cpp)
target_link_libraries(rigExecTestOpRunners PUBLIC rigExecSceneOps
    PRIVATE rigExecMath)
add_executable(testRigExecScenePropertyOps tests/testRigExecScenePropertyOps.cpp)
target_link_libraries(testRigExecScenePropertyOps PRIVATE rigExecSceneOps)
target_compile_definitions(testRigExecScenePropertyOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecScenePropertyOps COMMAND testRigExecScenePropertyOps)
add_executable(testRigExecSceneGeometryOps tests/testRigExecSceneGeometryOps.cpp)
target_link_libraries(testRigExecSceneGeometryOps PRIVATE rigExecTestOpRunners)
rigexec_add_test(NAME testRigExecSceneGeometryOps COMMAND testRigExecSceneGeometryOps)
set_tests_properties(testRigExecSceneGeometryOps PROPERTIES LABELS "fast")
add_executable(testRigExecSceneExternalOps tests/testRigExecSceneExternalOps.cpp)
target_link_libraries(testRigExecSceneExternalOps PRIVATE rigExecTestOpRunners)
target_compile_definitions(testRigExecSceneExternalOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSceneExternalOps COMMAND testRigExecSceneExternalOps)
set_tests_properties(testRigExecSceneExternalOps PROPERTIES LABELS "fast")
add_executable(testRigExecScenePoseOps tests/testRigExecScenePoseOps.cpp)
target_link_libraries(testRigExecScenePoseOps PRIVATE rigExecTestOpRunners)
target_compile_definitions(testRigExecScenePoseOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecScenePoseOps COMMAND testRigExecScenePoseOps)

add_executable(testRigExecBakedOpParity tests/testRigExecBakedOpParity.cpp)
target_link_libraries(testRigExecBakedOpParity PRIVATE rigExecTestOpRunners
    rigExecEvalCore)
target_compile_definitions(testRigExecBakedOpParity PRIVATE
    ${_rigexec_schema_resource_define})
set(_rigexec_op_parity_rigs
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/01_FkChainTail.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/02_TwoBoneIkLeg.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/03_IkFkBlendClamp.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/04_BlendShapeFace.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/05_TwistRibbonSpine.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/06_LatticeBulge.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/07_SurfaceDrape.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/08_AimEyes.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/09_PropertyMathMovers.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/10_AimXformTurret.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/11_VolumeWeights.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/12_PatchCurve.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/13_ReadPhases.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/14_VolumeConstrainedSweep.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/15_TransformMatrixMover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/16_ConnectionReadPhases.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/20_MatrixSphere.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/ArmRig.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/aimtest_points.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/aim_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/blend_point_frames.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/blendshape_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/combine_weight.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/curve_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/delta_mush_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/fk_chain.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/float_math_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/lattice_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/matrix_math_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/matrix_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/parent_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/position_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/ribbon.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/rotation_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/scale_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/single_chain_ik_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/skeleton_2d.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/skin_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/smooth_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/spline_ik.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/surface_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/twist_distribution.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/two_bone_ik.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/vec3f_math_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/volume_correct_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/wrinkle_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/space_switch.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/joint_group.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/cloth_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/projector_spaces.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_path_reads.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_weights.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_chains.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/volume_placements.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/external_op_parity.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/frame_record_fallbacks.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/solver_checkpoint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/space_switch_carry.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/space_switch_dial.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/space_switch_nested.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/space_switch_same_round.usda"
    --dense
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/19_HarmonicsChain.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/harmonics_constraint.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/pose_interpolator.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/pose_interpolator_riglogic.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/ml_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/animated_maps.usda"
    --painted
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/matrix_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/skin_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/smooth_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/delta_mush_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/lattice_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/curve_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/wrinkle_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/surface_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/volume_correct_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/ribbon.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/05_TwistRibbonSpine.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/06_LatticeBulge.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/ArmRig.usda"
    --chunked
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/skin_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/oneloop_two_limbs.usda")
rigexec_add_test(NAME testRigExecBakedOpParity COMMAND testRigExecBakedOpParity
    ${_rigexec_op_parity_rigs})
set_tests_properties(testRigExecBakedOpParity PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_SIMD=1")
rigexec_add_test(NAME testRigExecBakedOpParityScalar
    COMMAND testRigExecBakedOpParity ${_rigexec_op_parity_rigs})
set_tests_properties(testRigExecBakedOpParityScalar PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_SIMD=0")

add_executable(testRigExecOpGraph tests/testRigExecOpGraph.cpp)
target_link_libraries(testRigExecOpGraph PRIVATE rigExecGraph work Threads::Threads)
rigexec_add_test(NAME testRigExecOpGraph COMMAND testRigExecOpGraph)

add_executable(testRigExecExecCrossCheck tests/testRigExecExecCrossCheck.cpp)
target_link_libraries(testRigExecExecCrossCheck PRIVATE rigExec)
target_compile_definitions(testRigExecExecCrossCheck PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecExecCrossCheck COMMAND testRigExecExecCrossCheck
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/independentConstraints")

add_executable(testRigExecOpValues tests/testRigExecOpValues.cpp)
target_link_libraries(testRigExecOpValues PRIVATE rigExec)
rigexec_add_test(NAME testRigExecOpValues COMMAND testRigExecOpValues)

add_executable(testRigExecCompiledProgram tests/testRigExecCompiledProgram.cpp)
target_link_libraries(testRigExecCompiledProgram PRIVATE rigExec)
target_compile_definitions(testRigExecCompiledProgram PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecCompiledProgram COMMAND testRigExecCompiledProgram)

add_executable(testRigExecEvalCoreKinds tests/testRigExecEvalCoreKinds.cpp)
target_link_libraries(testRigExecEvalCoreKinds PRIVATE rigExec rigExecEvalCore)
rigexec_add_test(NAME testRigExecEvalCoreKinds COMMAND testRigExecEvalCoreKinds)

add_executable(testRigExecEvalCoreProperties
    tests/testRigExecEvalCoreProperties.cpp)
target_link_libraries(testRigExecEvalCoreProperties PRIVATE
    rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecEvalCoreProperties PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecEvalCoreProperties PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecEvalCoreProperties COMMAND testRigExecEvalCoreProperties
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecSurfaceKernelCache tests/testRigExecSurfaceKernelCache.cpp)
target_link_libraries(testRigExecSurfaceKernelCache PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecSurfaceKernelCache COMMAND testRigExecSurfaceKernelCache)

add_executable(testRigExecExecGoldens tests/testRigExecExecGoldens.cpp)
target_link_libraries(testRigExecExecGoldens PRIVATE rigExec)
target_compile_definitions(testRigExecExecGoldens PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_exec_rows_test(NAME computed_ik_space
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_ik_space.usda"
    FRAMES 1,5,10)
foreach(_constraint aim rotation parent)
    foreach(_target xform points)
        rigexec_add_exec_rows_test(NAME constraint_${_constraint}_${_target}
            STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/independentConstraints/${_constraint}_${_target}.usda"
            FRAMES 0)
    endforeach()
endforeach()

add_executable(testRigExecInputActions tests/testRigExecInputActions.cpp)
target_link_libraries(testRigExecInputActions PRIVATE rigExec)
rigexec_add_test(NAME testRigExecInputActions COMMAND "${CMAKE_COMMAND}"
    "-DDRIVER=$<TARGET_FILE:testRigExecInputActions>"
    "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
    "-DSCHEMA=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkInputActions.cmake")
set_tests_properties(testRigExecInputActions PROPERTIES LABELS "golden;input-protocol")

add_executable(testRigExecGoldenSuiteObserver tests/testRigExecGoldenSuiteObserver.cpp)
target_link_libraries(testRigExecGoldenSuiteObserver PRIVATE rigExec)
rigexec_add_test(NAME testRigExecGoldenSuiteObserver COMMAND "${CMAKE_COMMAND}"
    "-DDRIVER=$<TARGET_FILE:testRigExecGoldenSuiteObserver>"
    "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkGoldenSuiteObserver.cmake")
set_tests_properties(testRigExecGoldenSuiteObserver PROPERTIES LABELS "golden")

foreach(_domain Geometry Weight)
    add_executable(testRigExec${_domain}Program tests/testRigExec${_domain}Program.cpp)
    target_link_libraries(testRigExec${_domain}Program PRIVATE rigExec)
    rigexec_add_test(NAME testRigExec${_domain}Program COMMAND testRigExec${_domain}Program)
endforeach()

add_executable(testRigExecEnvelopeVolumes tests/testRigExecEnvelopeVolumes.cpp)
target_link_libraries(testRigExecEnvelopeVolumes PRIVATE rigExec)
target_compile_definitions(testRigExecEnvelopeVolumes PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecEnvelopeVolumes_${_schedule} COMMAND testRigExecEnvelopeVolumes
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    set_tests_properties(testRigExecEnvelopeVolumes_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule};RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

add_executable(testRigExecGraphBindings tests/testRigExecGraphBindings.cpp)
target_link_libraries(testRigExecGraphBindings PRIVATE rigExec)
rigexec_add_test(NAME testRigExecGraphBindings COMMAND testRigExecGraphBindings)

add_executable(testRigExecInputReplayValues tests/testRigExecInputReplayValues.cpp)
target_link_libraries(testRigExecInputReplayValues PRIVATE rigExec)
rigexec_add_test(NAME testRigExecInputReplayValues COMMAND testRigExecInputReplayValues)

add_executable(testRigExecInputReplayHosting tests/testRigExecInputReplayHosting.cpp)
target_link_libraries(testRigExecInputReplayHosting PRIVATE rigExec)
target_compile_definitions(testRigExecInputReplayHosting PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecInputReplayHosting COMMAND testRigExecInputReplayHosting)
set_tests_properties(testRigExecInputReplayHosting PROPERTIES LABELS "input-protocol")

add_executable(testRigExecTypedValues tests/testRigExecTypedValues.cpp)
target_link_libraries(testRigExecTypedValues PRIVATE rigExec)
rigexec_add_test(NAME testRigExecTypedValues COMMAND testRigExecTypedValues)

add_executable(testRigExecGoldenPose tests/testRigExecGoldenPose.cpp)
target_link_libraries(testRigExecGoldenPose PRIVATE rigExec)
rigexec_add_test(NAME testRigExecGoldenPose COMMAND testRigExecGoldenPose)

add_executable(testRigExecOracleInputs tests/testRigExecOracleInputs.cpp)
target_link_libraries(testRigExecOracleInputs PRIVATE rigExec)
rigexec_add_test(NAME testRigExecOracleInputs COMMAND testRigExecOracleInputs)

add_executable(testRigExecRuntimeMath tests/testRigExecRuntimeMath.cpp)
target_link_libraries(testRigExecRuntimeMath PRIVATE rigExecRuntime rigExecMathGf gf)
rigexec_add_test(NAME testRigExecRuntimeMath COMMAND testRigExecRuntimeMath)

add_executable(testRigExecRuntimeLoader tests/testRigExecRuntimeLoader.cpp)
target_link_libraries(testRigExecRuntimeLoader PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimeLoader PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeLoader PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples"
    RIGEXEC_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
rigexec_add_test(NAME testRigExecRuntimeLoader COMMAND testRigExecRuntimeLoader
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecRuntimePose tests/testRigExecRuntimePose.cpp)
target_link_libraries(testRigExecRuntimePose PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimePose PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimePose PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecRuntimePose COMMAND testRigExecRuntimePose
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecRuntimeAutoClavicle COMMAND testRigExecRuntimePose
    --auto-clavicle "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The same suite baked with every step in its own cluster, so a drag at a
# fixed frame reaches each step only through the dirtying owed to it (a
# moved avar, an override flag, a moved chain result) and never through a
# cluster it shares with another step.
rigexec_add_test(NAME testRigExecRuntimePoseFineClusters COMMAND testRigExecRuntimePose
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecRuntimePoseFineClusters PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecRuntimeWeights tests/testRigExecRuntimeWeights.cpp)
target_link_libraries(testRigExecRuntimeWeights PRIVATE rigExecRuntime rigExecBake rigExecSampler
    rigExecBuiltins)
target_include_directories(testRigExecRuntimeWeights PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeWeights PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples"
    RIGEXEC_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
rigexec_add_test(NAME testRigExecRuntimeWeights COMMAND testRigExecRuntimeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The same suite with every step in its own cluster, so a drag of one
# volume's control reruns that volume's VolumePlacements step alone and the
# other volume's placement is the one its skipped step kept.
rigexec_add_test(NAME testRigExecRuntimeWeightsFineClusters
    COMMAND testRigExecRuntimeWeights "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecRuntimeWeightsFineClusters PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecRuntimeGeometry tests/testRigExecRuntimeGeometry.cpp)
target_link_libraries(testRigExecRuntimeGeometry PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimeGeometry PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeGeometry PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecRuntimeGeometry COMMAND testRigExecRuntimeGeometry
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The same suite baked with every step in its own cluster, so a skin chunk
# runs only when a joint its key declares moves.
rigexec_add_test(NAME testRigExecRuntimeGeometryFineClusters
    COMMAND testRigExecRuntimeGeometry "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecRuntimeGeometryFineClusters PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecRuntimeInputs tests/testRigExecRuntimeInputs.cpp)
target_link_libraries(testRigExecRuntimeInputs PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecRuntimeInputs PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecRuntimeInputs COMMAND testRigExecRuntimeInputs
    "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")

# The float math mover's path and quantize operations: the curve kernel,
# the evaluator, a baked binary along the timeline and with its points set,
# interactive overrides and stage edits of the points, compile refusals.
add_executable(testRigExecPathOps tests/testRigExecPathOps.cpp)
target_link_libraries(testRigExecPathOps PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecPathOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecPathOps COMMAND testRigExecPathOps)
rigexec_add_graph_test(testRigExecPathOps CONE_VERIFY)

endif()  # RIGEXEC_WITH_USD
# The single-FlatBuffer format on hand-built files: bit exactness, the
# refusals before and in verification, determinism, validation rules.
add_executable(testRigExecFormat tests/testRigExecFormat.cpp)
target_link_libraries(testRigExecFormat PRIVATE rigExecBinary)
rigexec_add_test(NAME testRigExecFormat COMMAND testRigExecFormat)

if (RIGEXEC_WITH_USD)
add_executable(testRigExecBinary tests/testRigExecBinary.cpp)
target_link_libraries(testRigExecBinary PRIVATE rigExecBake rigExecAuthoring)
target_include_directories(testRigExecBinary PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecBinary PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecBinary COMMAND testRigExecBinary
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# RIG names a stage under examples/, STAGE any other stage by full path.
# The test bakes the stage once at TIME (default: the first of FRAMES) and
# checks its publications at TIME, at every frame of FRAMES with the binary's
# Animated inputs sampled from the stage, and for two drags of each input
# DRAGS names (a comma-separated list of <prim>.<attr>) against the same value
# authored in the session layer. CLASS is `inputs` (every animated source is an
# input, so the binary follows the stage at every frame) or `static` (an
# animated source is static data the binary holds at TIME, so it verifies at
# TIME alone, plus the drags); see tests/exampleFixtures.cmake.
function(rigexec_add_verify_binary_test)
    cmake_parse_arguments(A ""
        "NAME;RIG;STAGE;FRAMES;TIME;CLASS;DRAGS"
        "" ${ARGN})
    if (NOT DEFINED A_NAME OR NOT DEFINED A_FRAMES OR
            (NOT DEFINED A_RIG AND NOT DEFINED A_STAGE) OR
            (DEFINED A_RIG AND DEFINED A_STAGE))
        message(FATAL_ERROR "rigexec_add_verify_binary_test: NAME, FRAMES "
                            "and one of RIG or STAGE are required")
    endif()
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_verify_binary_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    if (DEFINED A_CLASS AND NOT A_CLASS STREQUAL "inputs" AND
            NOT A_CLASS STREQUAL "static")
        message(FATAL_ERROR "rigexec_add_verify_binary_test: CLASS is "
                            "'${A_CLASS}', expected inputs or static")
    endif()
    if (DEFINED A_RIG)
        set(A_STAGE "${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}")
    endif()
    set(_options)
    foreach(_option IN ITEMS TIME CLASS DRAGS)
        if (DEFINED A_${_option})
            list(APPEND _options "-D${_option}=${A_${_option}}")
        endif()
    endforeach()
    set(_out "${CMAKE_CURRENT_BINARY_DIR}/verify-binary/${A_NAME}.rigexec")
    rigexec_add_test(NAME verify_binary_${A_NAME} COMMAND "${CMAKE_COMMAND}"
        "-DBAKE=$<TARGET_FILE:rigExecBakeTool>" "-DPOSE=$<TARGET_FILE:rigExecPose>"
        "-DCONFORMANCE=$<TARGET_FILE:testRigExecBinary>"
        "-DSTAGE=${A_STAGE}"
        "-DFRAMES=${A_FRAMES}" "-DOUT=${_out}" ${_options}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/runVerifyBinary.cmake")
    set(_schema_resources
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_property(TEST verify_binary_${A_NAME} APPEND PROPERTY ENVIRONMENT
        "PXR_PLUGINPATH_NAME=${_schema_resources}")
endfunction()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/verify-binary")
# Every baking example, in its fixture class, dragging the table's control
# avar and operator input (fields 2-5) where the row names them.
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _fixture_fields "${_fixture}")
    list(GET _fixture_fields 0 _fixture_stage)
    list(GET _fixture_fields 1 _fixture_frames)
    list(GET _fixture_fields 2 _fixture_control_prim)
    list(GET _fixture_fields 3 _fixture_control_avar)
    list(GET _fixture_fields 4 _fixture_operator_prim)
    list(GET _fixture_fields 5 _fixture_operator_input)
    list(GET _fixture_fields 6 _fixture_bakes)
    list(GET _fixture_fields 8 _fixture_animation)
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    set(_fixture_drags)
    if (NOT _fixture_control_prim STREQUAL "-")
        list(APPEND _fixture_drags
            "${_fixture_control_prim}.${_fixture_control_avar}")
    endif()
    if (NOT _fixture_operator_prim STREQUAL "-")
        list(APPEND _fixture_drags
            "${_fixture_operator_prim}.${_fixture_operator_input}")
    endif()
    set(_fixture_drag_args)
    if (_fixture_drags)
        string(REPLACE ";" "," _fixture_drag_list "${_fixture_drags}")
        set(_fixture_drag_args DRAGS "${_fixture_drag_list}")
    endif()
    if (_fixture_bakes STREQUAL "YES")
        rigexec_add_verify_binary_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames}
            CLASS ${_fixture_animation} ${_fixture_drag_args})
    endif()
endforeach()

# The stack's binary round trip; see the parity entry above for why the
rigexec_add_verify_binary_test(NAME biped_Biped_stack_anim
    RIG biped/Biped_stack_anim.usda FRAMES 1,50,100,150,200 CLASS inputs)

# The results the runtime computes rather than replays (constraint
# envelopes, current-phase weight fields), which no example exercises:
# the binary, baked at the first frame, must compute them at every frame.
rigexec_add_verify_binary_test(NAME computed_weights
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_weights.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# Property chains the runtime computes in the cases no example covers
# together: envelopes on property movers, a double chain, vec3f and matrix
# chains, the chains' diagnostics, phased consumers, and a constraint
# weight read through a chain at a declared `final`.
rigexec_add_verify_binary_test(NAME computed_chains
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_chains.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# Two-bone IK measured in an animated rigExec:space and a keyed
# rigExec:spaceMatrix, one with a keyed spaceMatrix alone, and a spline IK
# whose rest is rebuilt in the moving space: the solvers' spaceMatrix is a
# registered read the runtime both consumes and computes.
rigexec_add_verify_binary_test(NAME computed_ik_space
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_ik_space.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# The assemblers' connection-following scalar reads of every type (int,
# bool, float, a float widened to double, double3, matrix4d), including a
# float whose connection ends on a double with no value, which reads the
# head's own value. The mesh's keyed points are a chain base the binary
# holds at the bake time, so the class is static; the drags show the reads
# are computed: the keyed avar a mover input and a shader dial read through
# their connections, a keyed float dial, and the two heads whose
# connections end on the valueless double. Baked at frame 10, where the
# points have left the mush's rest points, so a drag of the mush's inputs
# moves its output (a mush of its own rest returns it). The same stage with
# the points held by a stronger default is of the inputs class: its binary
# follows the keyed double3, matrix4d and float reads at every frame.
rigexec_add_verify_binary_test(NAME computed_path_reads
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_path_reads.usda"
    FRAMES 1,4,7,10 TIME 10 CLASS inputs
    DRAGS "/PathReadAsset/Rig/Controls/Dial.avars:amount,/PathReadAsset/Rig/Dials.gain,/PathReadAsset/Rig/Dials.held,/PathReadAsset/Rig/Movers/Mush.inputs:step")
rigexec_add_verify_binary_test(NAME computed_path_reads_held_points
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecPathReadsHeldPoints.usda"
    FRAMES 1,4,7,10 CLASS inputs)

# Space switches nested under switched controls, whose compose reads an
# ancestor recomposed before that ancestor's own switch: the nested pair,
# a pair resolved in one round, the nested pair reading keyed dials that
# hold no default, and a switch whose parent and rigExec:space are
# recomposed on a control above them. The nested pair is baked at frame 2,
# where its controls have left their rests (at rest a switch holds its
# target whatever its index), and drags each switch's inputs:activeSpace
# there.
set(_switch_drags_space_switch_nested
    TIME 2
    DRAGS "/Rig/Movers/sSpaces.inputs:activeSpace,/Rig/Movers/pSpaces.inputs:activeSpace")
foreach(_switch_fixture IN ITEMS space_switch_nested space_switch_same_round
        space_switch_dial space_switch_carry)
    rigexec_add_verify_binary_test(NAME ${_switch_fixture}
        STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/${_switch_fixture}.usda"
        FRAMES 0,1,2,3 CLASS inputs ${_switch_drags_${_switch_fixture}})
endforeach()

# Projector providers that only a projector reads: guards the export and
# its ProviderMatrix producers, not the projector values (matrix primvars
# are outside the runtime output contract verify-binary compares). The
# mesh's keyed points are the mush's chain base, so the class is static;
# the drag moves the constraints' driver, which separates each provider's
# final from its base.
rigexec_add_verify_binary_test(NAME projector_spaces
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/projector_spaces.usda"
    FRAMES 1,4,7,10 TIME 7 CLASS inputs
    DRAGS "/ProjectorAsset/Rig/Controls/Driver.avars:tx")

# One VolumePlacements step per volume (part 1), each writing its own
# WeightFrames slot, and a current-phase field reading both placements.
# Each drag moves one volume's control, so one placement moves and the
# other must hold.
rigexec_add_verify_binary_test(NAME volume_placements
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/volume_placements.usda"
    FRAMES 1,5,10 CLASS inputs
    DRAGS "/PlacementAsset/Rig/Controls/ACtl.avars:rz,/PlacementAsset/Rig/Controls/BCtl.avars:rz")

# AtPrim read phases on rigExec:transform, read in the fold through the
# FrameMatrix steps' records: a constraint's record, a disabled exit that
# still records, and lists that fall through to an earlier record. A drag on
# Dial moves one reader's weight; a drag on Late re-runs C3's record alone.
rigexec_add_verify_binary_test(NAME frame_record_fallbacks
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/frame_record_fallbacks.usda"
    FRAMES 1,4,8 CLASS inputs
    DRAGS "/RecordAsset/Rig/Controls/Dial.avars:amount,/RecordAsset/Rig/Controls/Late.avars:tz")

# AtPrim read phases that name solver checkpoints, gated by the solver
# commit's present bytes: LegBlend never publishes the knee, so its record
# is never valid and P3 falls through to LegFK's record. A drag on ZKnee
# moves LegZ's record alone; one on KneeTarget moves KneeMove, which runs
# after LegFK, so P3's fold re-runs over LegFK's unchanged record.
rigexec_add_verify_binary_test(NAME solver_checkpoint
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/solver_checkpoint.usda"
    FRAMES 1,5,9 CLASS inputs
    DRAGS "/CheckpointAsset/Rig/Controls/ZKnee.avars:rx,/CheckpointAsset/Rig/Controls/KneeTarget.avars:tx")

# Dense blend samples read at `final` and `preceding`, each bound to a
# version of a chain two animated lifts move. Baked at frame 1, where the
# lifts are identity: playback computes each sample from its bound version
# on every run, so frames 3 and 7 and the drag of CtlB (which moves the
# `final` sample alone) follow the lifts rather than the bake's points.
rigexec_add_verify_binary_test(NAME phased_blend_samples
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/phased_blend_samples.usda"
    FRAMES 1,3,7 CLASS inputs
    DRAGS "/Asset/Rig/Controls/CtlB.avars:tx")

# A lattice whose cage is its own target, read at `preceding`: the runtime
# reads the points entering it through the binding's one version.
rigexec_add_verify_binary_test(NAME preceding_own_chain
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/preceding_own_chain.usda"
    FRAMES 1,5,9 CLASS inputs
    DRAGS "/PrecedingAsset/Rig/Controls/LiftCtl.avars:ty")

# Three skin layouts the file stores raw because the sparse form cannot
# hold them, OddSkin, WideOddSkin and WideEmptySkin, and two it stores
# sparse, ShortSkin and RowSkin. OddSkin, WideOddSkin, WideEmptySkin and
# ShortSkin fail and RowSkin skins; playback gives the evaluators'
# MoverFailed lines and counts the same work. The four failing skins'
# meshes have keyed points, chain bases held at the bake time, so the
# class is static. The same stage, and a two-limb rig, cut into chunks of two
# vertices whatever their ready levels: the only bakes that store chunked
# revisions, so Open's chunk rules meet real cuts and playback runs the
# chunk, staleness and re-cut paths.
rigexec_add_verify_binary_test(NAME raw_skin_layouts
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/raw_skin_layouts.usda"
    FRAMES 1,4,7,10 TIME 1 CLASS inputs
    DRAGS "/RawSkinAsset/Rig/Controls/C0.avars:rz,/RawSkinAsset/Rig/Controls/C0/C1.avars:rz")
rigexec_add_verify_binary_test(NAME raw_skin_layouts_chunked
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/raw_skin_layouts.usda"
    FRAMES 1,4,7,10 TIME 1 CLASS inputs
    DRAGS "/RawSkinAsset/Rig/Controls/C0.avars:rz,/RawSkinAsset/Rig/Controls/C0/C1.avars:rz")
rigexec_add_verify_binary_test(NAME oneloop_two_limbs_chunked
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/oneloop_two_limbs.usda"
    FRAMES 1,4,8,12 CLASS inputs
    DRAGS "/LimbsAsset/Rig/Controls/A0.avars:rz,/LimbsAsset/Rig/Controls/BEffector.avars:tx")
foreach(_chunked IN ITEMS raw_skin_layouts_chunked oneloop_two_limbs_chunked)
    set_property(TEST verify_binary_${_chunked} APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_CHUNK_VERTS=2" "RIGEXEC_BAKED_CHUNK_ALWAYS=1")
endforeach()

# The RigLogic docs examples, baked at their first frame. The ML mover's
# drags move its frame driver and its mask; the maps' a map input and the
# morph envelope; the interpolators' both drivers, which every solve reads.
set(_riglogic_drags_ml_mover
    "/MlAsset/Rig/Controls/Elbow.avars:rz,/MlAsset/Rig/Movers/Geometry/Muscles.inputs:mask")
set(_riglogic_drags_animated_maps
    "/MapsAsset/Rig/Controls/BrowL.avars:ty,/MapsAsset/Rig/Movers/Geometry/Brows.inputs:defaultWeight")
set(_riglogic_drags_pose_interpolator_riglogic
    "/RbfAsset/Rig/Controls/Shoulder.avars:rz,/RbfAsset/Rig/Controls/Elbow.avars:rz")
foreach(_entry IN LISTS RIGEXEC_RIGLOGIC_DOCS_EXAMPLES)
    string(REPLACE "|" ";" _fields "${_entry}")
    list(GET _fields 0 _stage)
    list(GET _fields 1 _frames)
    rigexec_add_verify_binary_test(NAME docs_${_stage}
        STAGE "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/${_stage}.usda"
        FRAMES ${_frames} CLASS inputs
        DRAGS "${_riglogic_drags_${_stage}}")
endforeach()

# The secondary-motion and cloth docs examples: windowed operations, whose
# .rigexec reader replays the frames before each one into its history (the
# harness does, as a host scrubbing to that frame would). The drags move the
# control that sets the motion off.
rigexec_add_verify_binary_test(NAME docs_harmonics_constraint
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/harmonics_constraint.usda"
    FRAMES 1,10,26,33,50 CLASS inputs
    DRAGS "/BobAsset/Rig/Controls/Root.avars:tx")
rigexec_add_verify_binary_test(NAME docs_cloth_mover
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/cloth_mover.usda"
    FRAMES 1,12,30,41,60 CLASS inputs
    DRAGS "/ClothAsset/Rig/Controls/Bar.avars:tx")
# A joint group on an FK chain: the drags switch the active finger and move
# the start frame the inactive finger's rest rides.
rigexec_add_verify_binary_test(NAME docs_joint_group
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/joint_group.usda"
    FRAMES 1001,1011,1012,1024 CLASS inputs
    DRAGS "/JointGroupAsset/Rig/Groups/Fingers.rigExec:activeMember,/JointGroupAsset/Rig/Controls/Arm.avars:tx")

add_executable(testRigExecWeightFields tests/testRigExecWeightFields.cpp)
target_link_libraries(testRigExecWeightFields PRIVATE rigExecMathGf)
rigexec_add_test(NAME testRigExecWeightFields COMMAND testRigExecWeightFields)

add_executable(testRigExecBakedMode tests/testRigExecBakedMode.cpp)
target_link_libraries(testRigExecBakedMode PRIVATE rigExec)
target_compile_definitions(testRigExecBakedMode PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecBakedMode COMMAND testRigExecBakedMode
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBakedAttribute tests/testRigExecBakedAttribute.cpp)
target_link_libraries(testRigExecBakedAttribute PRIVATE rigExec)
target_compile_definitions(testRigExecBakedAttribute
    PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecBakedAttribute COMMAND testRigExecBakedAttribute
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecBakedAttributeEnvironment
    COMMAND testRigExecBakedAttribute
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecBakedAttributeEnvironment PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")

# The step graph behind the baked program: the edges, not the values.
add_executable(testRigExecBakedSchedule tests/testRigExecBakedSchedule.cpp)
target_link_libraries(testRigExecBakedSchedule PRIVATE rigExec)
target_compile_definitions(testRigExecBakedSchedule PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecBakedSchedule COMMAND testRigExecBakedSchedule
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecBakedScheduleCones_${_schedule}
        COMMAND testRigExecBakedSchedule
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecBakedScheduleCones_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1"
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(TEST testRigExecBakedScheduleCones_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    rigexec_add_test(NAME testRigExecBakedScheduleCones_grain0_${_schedule}
        COMMAND testRigExecBakedSchedule
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecBakedScheduleCones_grain0_${_schedule}
        PROPERTIES ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1"
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    set_property(TEST testRigExecBakedScheduleCones_grain0_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

# The baked op trace: completion order against the step graph, under both
# executors.
add_executable(testRigExecOpTrace tests/testRigExecOpTrace.cpp)
add_executable(testRigExecLiveDebug tests/testRigExecLiveDebug.cpp)
target_link_libraries(testRigExecLiveDebug PRIVATE rigExecImaging)
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecLiveDebug_${_schedule} COMMAND testRigExecLiveDebug
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_tests_properties(testRigExecLiveDebug_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule};RIGEXEC_ENABLE_PARALLEL_EVAL=1;RIGEXEC_FRAME_CACHE=off")
endforeach()
target_link_libraries(testRigExecOpTrace PRIVATE rigExec)
target_compile_definitions(testRigExecOpTrace PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecOpTrace_${_schedule}
        COMMAND testRigExecOpTrace "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecOpTrace_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    set_property(TEST testRigExecOpTrace_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()
# Calibration alone measures: op accumulators fill without STEP_TIMING.
rigexec_add_test(NAME testRigExecOpTrace_calibrate
    COMMAND testRigExecOpTrace "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecOpTrace_calibrate PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1;RIGEXEC_ENABLE_PARALLEL_EVAL=1")

# The one-loop acceptance criteria on their fixtures, under both executors.
add_executable(testRigExecOneLoopAcceptance tests/testRigExecOneLoopAcceptance.cpp)
target_link_libraries(testRigExecOneLoopAcceptance PRIVATE rigExec rigExecBake rigExecRuntime)
target_compile_definitions(testRigExecOneLoopAcceptance PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecOneLoopAcceptance_${_schedule}
        COMMAND testRigExecOneLoopAcceptance "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    set_tests_properties(testRigExecOneLoopAcceptance_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    set_property(TEST testRigExecOneLoopAcceptance_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

# A vertex group forwards points its mover does not weight without
# recomputing its support: a gated chain through the native program, a
# frame-cache worker and the runtime, and an arm edit on the biped.
add_executable(testRigExecGroupForwarding tests/testRigExecGroupForwarding.cpp)
target_link_libraries(testRigExecGroupForwarding PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecGroupForwarding PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecGroupForwarding_${_schedule}
        COMMAND testRigExecGroupForwarding "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecGroupForwarding_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
endforeach()

add_executable(testRigExecOpSteps tests/testRigExecOpSteps.cpp)
target_link_libraries(testRigExecOpSteps PRIVATE
    rigExec rigExecBuiltins rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecOpSteps PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecOpSteps_${_schedule}
        COMMAND testRigExecOpSteps --write
            "${CMAKE_CURRENT_BINARY_DIR}/opSteps_${_schedule}.rigexec")
    set_tests_properties(testRigExecOpSteps_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
endforeach()
set_tests_properties(testRigExecOpSteps_serial PROPERTIES
    FIXTURES_SETUP rigExecOpStepsFile)
rigexec_add_test(NAME testRigExecOpStepsScalar COMMAND testRigExecOpSteps)
set_tests_properties(testRigExecOpStepsScalar PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=parallel;RIGEXEC_ENABLE_SIMD=0")
rigexec_add_test(NAME testRigExecOpSteps_unregistered
    COMMAND testRigExecOpSteps --unregistered
        "${CMAKE_CURRENT_BINARY_DIR}/opSteps_serial.rigexec")
set_tests_properties(testRigExecOpSteps_unregistered PROPERTIES
    FIXTURES_REQUIRED rigExecOpStepsFile)

add_executable(testRigExecCrossDomainInputs tests/testRigExecCrossDomainInputs.cpp)
target_link_libraries(testRigExecCrossDomainInputs PRIVATE rigExec)
target_compile_definitions(testRigExecCrossDomainInputs PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    rigexec_add_test(NAME testRigExecCrossDomainInputs_${_schedule}
        COMMAND testRigExecCrossDomainInputs "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    set_tests_properties(testRigExecCrossDomainInputs_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    set_property(TEST testRigExecCrossDomainInputs_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

# Exercise the default scheduler and production grain, with no schedule override.
rigexec_add_test(NAME testRigExecOneLoopAcceptance_production
    COMMAND testRigExecOneLoopAcceptance "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
set_tests_properties(testRigExecOneLoopAcceptance_production PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1"
    ENVIRONMENT_MODIFICATION "RIGEXEC_BAKED_SCHEDULE=unset:;RIGEXEC_BAKED_GRAIN_US=unset:"
    LABELS "one-loop")
rigexec_add_test(NAME testRigExecOneLoopRetired
    COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DMANIFEST=${CMAKE_CURRENT_SOURCE_DIR}/tests/oneLoopRetiredSymbols.cmake"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/checkOneLoopRetired.cmake")
set_tests_properties(testRigExecOneLoopRetired PROPERTIES LABELS "one-loop")

add_executable(testRigExecSolverBake tests/testRigExecSolverBake.cpp)
target_link_libraries(testRigExecSolverBake PRIVATE rigExec)
target_compile_definitions(testRigExecSolverBake
    PRIVATE ${_rigexec_schema_resource_define})
target_include_directories(testRigExecSolverBake PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
rigexec_add_test(NAME testRigExecSolverBake COMMAND testRigExecSolverBake
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# Lists the solvers whose space rest can move after Build, and those whose
# space slot sits under one of their own joints (the refresh's candidates).
rigexec_add_test(NAME testRigExecSolverBakeSpaceRestCandidates
    COMMAND testRigExecSolverBake "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    --space-rest-candidates)
rigexec_add_graph_test(testRigExecSolverBake CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecSolverStacking tests/testRigExecSolverStacking.cpp)
target_link_libraries(testRigExecSolverStacking PRIVATE rigExec)
target_compile_definitions(testRigExecSolverStacking
    PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSolverStacking COMMAND testRigExecSolverStacking
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecSolverStacking CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecArm tests/testRigExecArm.cpp)
target_link_libraries(testRigExecArm PRIVATE rigExec)
target_compile_definitions(testRigExecArm PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecArm COMMAND testRigExecArm
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecArm CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImaging tests/testRigExecImaging.cpp)
target_link_libraries(testRigExecImaging PRIVATE
    rigExecImaging rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecImaging PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
rigexec_add_test(NAME testRigExecImaging COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecImagingFineClusters COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecImagingFineClusters PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")
rigexec_add_test(NAME testRigExecImagingFineClustersReference COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecImagingFineClustersReference PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0;RIGEXEC_ENABLE_PARALLEL_EVAL=0")

add_executable(testRigExecImagingRender tests/testRigExecImagingRender.cpp)
target_link_libraries(testRigExecImagingRender PRIVATE
    rigExecImaging usdImagingGL hio)
target_compile_definitions(testRigExecImagingRender PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
rigexec_add_test(NAME testRigExecImagingRender COMMAND testRigExecImagingRender
    "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/render")
set_tests_properties(testRigExecImagingRender PROPERTIES
    SKIP_RETURN_CODE 77 TIMEOUT 60)

add_executable(testRigExecProfilerApi tests/testRigExecProfilerApi.cpp)
target_link_libraries(testRigExecProfilerApi PRIVATE rigExecImaging js)
target_compile_definitions(testRigExecProfilerApi PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecProfilerApi COMMAND testRigExecProfilerApi
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingMultiStage
    tests/testRigExecImagingMultiStage.cpp)
target_link_libraries(testRigExecImagingMultiStage PRIVATE rigExecImaging)
target_compile_definitions(testRigExecImagingMultiStage PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
rigexec_add_test(NAME testRigExecImagingMultiStage COMMAND testRigExecImagingMultiStage
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingPlayback tests/testRigExecImagingPlayback.cpp)
target_link_libraries(testRigExecImagingPlayback PRIVATE rigExecImaging rigExecBake)
target_include_directories(testRigExecImagingPlayback PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecImagingPlayback PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
rigexec_add_test(NAME testRigExecImagingPlayback COMMAND testRigExecImagingPlayback
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# Frames on demand: rig outputs sampled at a time offset through Hydra.
add_executable(testRigExecFrameDemand tests/testRigExecFrameDemand.cpp)
target_link_libraries(testRigExecFrameDemand PRIVATE rigExecImaging rigExecBake rigExec)
target_compile_definitions(testRigExecFrameDemand PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecFrameDemand COMMAND testRigExecFrameDemand)

add_executable(testRigExecTouchPose tests/testRigExecTouchPose.cpp)
target_link_libraries(testRigExecTouchPose PRIVATE rigExecImaging)
rigexec_add_test(NAME testRigExecTouchPose COMMAND testRigExecTouchPose)

add_executable(testRigExecVolumeWeights tests/testRigExecVolumeWeights.cpp)
# The sphere-under-bone case compares native, frozen and .rigexec evaluation.
target_link_libraries(testRigExecVolumeWeights PRIVATE rigExec rigExecBake
    rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecVolumeWeights PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecVolumeWeights COMMAND testRigExecVolumeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecVolumeWeights
    CONE_VERIFY SEPARATE_PARALLEL "${CMAKE_CURRENT_SOURCE_DIR}/examples")
foreach(_entry testRigExecVolumeWeightsGraph_serial
               testRigExecVolumeWeightsGraph_parallel
               testRigExecVolumeWeightsCones_serial
               testRigExecVolumeWeightsCones_parallel
               testRigExecVolumeWeightsCones_grain0_serial
               testRigExecVolumeWeightsCones_grain0_parallel)
    set_property(TEST ${_entry} APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_TEST_BAKED_PATH=1")
endforeach()

# Phased point/frame reads use exact bound versions. These cases retain
# original-source query evidence and run the graph's cone verifier in both
# schedules, including singleton clusters.
add_executable(testRigExecPhaseBindings tests/testRigExecPhaseBindings.cpp)
target_link_libraries(testRigExecPhaseBindings PRIVATE rigExec)
target_compile_definitions(testRigExecPhaseBindings PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecPhaseBindings COMMAND testRigExecPhaseBindings
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecPhaseBindings
    CONE_VERIFY "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecWeightOverlay tests/testRigExecWeightOverlay.cpp)
target_link_libraries(testRigExecWeightOverlay PRIVATE rigExecImaging)
target_compile_definitions(testRigExecWeightOverlay PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecWeightOverlay COMMAND testRigExecWeightOverlay
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecMoverGraph tests/testRigExecMoverGraph.cpp)
target_link_libraries(testRigExecMoverGraph PRIVATE rigExec)
target_compile_definitions(testRigExecMoverGraph PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecMoverGraph COMMAND testRigExecMoverGraph)

# The compiled rig skin layout: what has to invalidate it, and what Compile
add_executable(testRigExecSkinTopology tests/testRigExecSkinTopology.cpp)
target_link_libraries(testRigExecSkinTopology PRIVATE rigExec)
target_compile_definitions(testRigExecSkinTopology PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSkinTopology COMMAND testRigExecSkinTopology)
set_tests_properties(testRigExecSkinTopology PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
# The baked program's own layout handles: authored edits, drags, time
# samples, a bailed generation, and the exported layout fields.
rigexec_add_graph_test(testRigExecSkinTopology CONE_VERIFY SEPARATE_PARALLEL
    --baked "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecStaticInputCache
    tests/testRigExecStaticInputCache.cpp)
target_link_libraries(testRigExecStaticInputCache PRIVATE rigExec)
rigexec_add_test(NAME testRigExecStaticInputCache COMMAND testRigExecStaticInputCache)

add_executable(testRigExecPoseInterpolator
    tests/testRigExecPoseInterpolator.cpp)
target_link_libraries(testRigExecPoseInterpolator PRIVATE rigExec rigExecMathGf)
target_compile_definitions(testRigExecPoseInterpolator PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecPoseInterpolator COMMAND testRigExecPoseInterpolator)

add_executable(testRigExecCompiledRigRests tests/testRigExecCompiledRigRests.cpp)
target_link_libraries(testRigExecCompiledRigRests PRIVATE rigExec)
target_compile_definitions(testRigExecCompiledRigRests PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecCompiledRigRests COMMAND testRigExecCompiledRigRests
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecCompiledRigRests PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
rigexec_add_graph_test(testRigExecCompiledRigRests CONE_VERIFY SEPARATE_PARALLEL
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Bit-identical results from the parallel per-point kernels, both ways round:
add_executable(testRigExecParallelKernels tests/testRigExecParallelKernels.cpp)
target_link_libraries(testRigExecParallelKernels PRIVATE rigExec work)
rigexec_add_test(NAME testRigExecParallelKernels COMMAND testRigExecParallelKernels)
rigexec_add_test(NAME testRigExecParallelKernelsSerial
    COMMAND testRigExecParallelKernels)
set_tests_properties(testRigExecParallelKernelsSerial PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")

add_executable(testRigExecChainLevels tests/testRigExecChainLevels.cpp)
target_link_libraries(testRigExecChainLevels PRIVATE rigExec work)
target_compile_definitions(testRigExecChainLevels PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecChainLevels COMMAND testRigExecChainLevels)
set_tests_properties(testRigExecChainLevels PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
rigexec_add_test(NAME testRigExecChainLevelsRepeat
    COMMAND testRigExecChainLevels 20)
set_tests_properties(testRigExecChainLevelsRepeat PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")

add_executable(testRigExecProfiler tests/testRigExecProfiler.cpp)
target_link_libraries(testRigExecProfiler PRIVATE rigExec)
rigexec_add_test(NAME testRigExecProfiler COMMAND testRigExecProfiler)

# Measures what the compile's path bookkeeping costs; asserts nothing, so it
add_executable(benchPathLookup tests/benchPathLookup.cpp)
target_link_libraries(benchPathLookup PRIVATE rigExec)

# Scratch: which USD reads stop scaling across threads (see the file header).
add_executable(benchStageReads tests/benchStageReads.cpp)
target_link_libraries(benchStageReads PRIVATE rigExec)

add_executable(testRigExecFrameCache tests/testRigExecFrameCache.cpp)
target_link_libraries(testRigExecFrameCache PRIVATE rigExec)
rigexec_add_test(NAME testRigExecFrameCache COMMAND testRigExecFrameCache)

add_executable(testRigExecFrozenContext tests/testRigExecFrozenContext.cpp)
target_link_libraries(testRigExecFrozenContext PRIVATE rigExec rigExecAuthoring rigExecBake rigExecSampler)
rigexec_add_test(NAME testRigExecFrozenContext COMMAND testRigExecFrozenContext "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBackgroundScheduler tests/testRigExecBackgroundScheduler.cpp)
target_link_libraries(testRigExecBackgroundScheduler PRIVATE rigExecPlayback)
rigexec_add_test(NAME testRigExecBackgroundScheduler COMMAND testRigExecBackgroundScheduler)

add_executable(testRigExecFrameCacheSparsity tests/testRigExecFrameCacheSparsity.cpp)
target_link_libraries(testRigExecFrameCacheSparsity PRIVATE rigExec)
target_compile_definitions(testRigExecFrameCacheSparsity PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecFrameCacheSparsity COMMAND testRigExecFrameCacheSparsity)
# One step per cluster, so that a switched group and the provider it
# recomposes land in different clusters (TestRecomposedVersionSeedsSparseReuse).
set_tests_properties(testRigExecFrameCacheSparsity PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecFrameCacheCones_Verify tests/testRigExecFrameCacheCones_Verify.cpp)
target_link_libraries(testRigExecFrameCacheCones_Verify PRIVATE rigExec)
rigexec_add_test(NAME testRigExecFrameCacheCones_Verify COMMAND testRigExecFrameCacheCones_Verify "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingFrameCache tests/testRigExecImagingFrameCache.cpp)
target_link_libraries(testRigExecImagingFrameCache PRIVATE rigExecImaging rigExecBake)
target_include_directories(testRigExecImagingFrameCache PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecImagingFrameCache PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
rigexec_add_test(NAME testRigExecImagingFrameCache COMMAND testRigExecImagingFrameCache
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_test(NAME testRigExecImagingFrameCacheDefault COMMAND "${CMAKE_COMMAND}" -E env
    --unset=RIGEXEC_BAKED_SCHEDULE
    RIGEXEC_FRAME_CACHE=on RIGEXEC_ENABLE_PARALLEL_EVAL=1
    "$<TARGET_FILE:testRigExecImagingFrameCache>"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples" --default-mode)

foreach(_rigexec_plugin_test testRigExecFrameCacheCones_Verify
        testRigExecFrozenContext
        testRigExecImagingFrameCache
        testRigExecImagingFrameCacheDefault)
    set_property(TEST ${_rigexec_plugin_test} APPEND PROPERTY
        ENVIRONMENT_MODIFICATION
        "PXR_PLUGINPATH_NAME=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "PXR_PLUGINPATH_NAME=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources")
endforeach()

add_executable(benchFrameCache tests/benchFrameCache.cpp)
target_link_libraries(benchFrameCache PRIVATE rigExec)
target_compile_definitions(benchFrameCache PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchFrameCacheWarm tests/benchFrameCacheWarm.cpp)
target_link_libraries(benchFrameCacheWarm PRIVATE rigExec rigExecPlayback)
target_compile_definitions(benchFrameCacheWarm PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchCommitLag tests/benchCommitLag.cpp)
target_link_libraries(benchCommitLag PRIVATE rigExecImaging rigExecBake)
target_include_directories(benchCommitLag PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(benchCommitLag PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})

add_executable(benchEditLatency tests/benchEditLatency.cpp)
target_link_libraries(benchEditLatency PRIVATE rigExec)
target_compile_definitions(benchEditLatency PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(testRigExecDeterminism tests/testRigExecDeterminism.cpp)
target_link_libraries(testRigExecDeterminism PRIVATE rigExec)
target_compile_definitions(testRigExecDeterminism PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecDeterminism COMMAND testRigExecDeterminism
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/tutorial_rolling_ball.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/biped/Biped_anim.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/delta_mush_mover.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/skin_mover.usda")

add_executable(benchRigSpeed tests/benchRigSpeed.cpp)
target_link_libraries(benchRigSpeed PRIVATE rigExec)
target_compile_definitions(benchRigSpeed PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchPlayback tests/benchPlayback.cpp)
target_link_libraries(benchPlayback PRIVATE rigExecImaging rigExecBake)
target_include_directories(benchPlayback PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(benchPlayback PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})

add_executable(testRigExecInteractive tests/testRigExecInteractive.cpp)
target_link_libraries(testRigExecInteractive PRIVATE rigExec)
target_compile_definitions(testRigExecInteractive PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecInteractive COMMAND testRigExecInteractive)
rigexec_add_graph_test(testRigExecInteractive CONE_VERIFY)

add_executable(testRigExecValueEdits tests/testRigExecValueEdits.cpp)
target_link_libraries(testRigExecValueEdits PRIVATE rigExec)
target_compile_definitions(testRigExecValueEdits PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecValueEdits COMMAND testRigExecValueEdits
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecValueEdits CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Sampled leaves: every patchable binding and every packet assembly read
# before the region, equal to the read it replaced, re-read exactly when
# that read can have moved; and no step body reads the stage.
add_executable(testRigExecLeaves tests/testRigExecLeaves.cpp)
target_link_libraries(testRigExecLeaves PRIVATE rigExecBake rigExecAuthoring)
target_include_directories(testRigExecLeaves PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecLeaves PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecLeaves COMMAND testRigExecLeaves
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The base entry covers parallel body purity without the cone verifier.
rigexec_add_graph_test(testRigExecLeaves CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Property revisions as head-tier ops: the ops equal the evaluator's
# chains bit for bit and line for line, run only when what they read moved,
# and still publish every run.
add_executable(testRigExecPropertyOps tests/testRigExecPropertyOps.cpp)
target_link_libraries(testRigExecPropertyOps PRIVATE rigExecBake)
target_compile_definitions(testRigExecPropertyOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecPropertyOps COMMAND testRigExecPropertyOps
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecPropertyOps CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# The rest chain and the default-space ladder as head-tier ops: the ten
# exported tables equal the single loop the ops were split from, every rest
# reader declares what it indexes, a rest that moves alone re-runs its
# readers and not its compose, and a rest edit seeds through the tier.
add_executable(testRigExecRestOps tests/testRigExecRestOps.cpp)
target_link_libraries(testRigExecRestOps PRIVATE rigExec)
target_compile_definitions(testRigExecRestOps PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecRestOps COMMAND testRigExecRestOps
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecRestOps CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecUpstreamInputs tests/testRigExecUpstreamInputs.cpp)
target_link_libraries(testRigExecUpstreamInputs PRIVATE
    rigExecBake rigExecImaging rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecUpstreamInputs PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecUpstreamInputs COMMAND testRigExecUpstreamInputs
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_graph_test(testRigExecUpstreamInputs
    CONE_VERIFY "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecScopedClears tests/testRigExecScopedClears.cpp)
target_link_libraries(testRigExecScopedClears PRIVATE rigExec)
target_compile_definitions(testRigExecScopedClears PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecScopedClears COMMAND testRigExecScopedClears)
rigexec_add_graph_test(testRigExecScopedClears CONE_VERIFY)

add_executable(testRigExecDefaultSpaces tests/testRigExecDefaultSpaces.cpp)
target_link_libraries(testRigExecDefaultSpaces PRIVATE rigExec)
target_compile_definitions(testRigExecDefaultSpaces PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecDefaultSpaces COMMAND testRigExecDefaultSpaces)
set_tests_properties(testRigExecDefaultSpaces PROPERTIES
    ENVIRONMENT "RIGEXEC_VERIFY_POSEINFO=1")

add_executable(testRigExecSchemaAuthoring
    tests/testRigExecSchemaAuthoring.cpp)
target_link_libraries(testRigExecSchemaAuthoring PRIVATE rigExecAuthoring)
target_compile_definitions(testRigExecSchemaAuthoring PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecSchemaAuthoring COMMAND testRigExecSchemaAuthoring)
rigexec_add_test(NAME testRigExecSchemaAuthoringMissing
    COMMAND testRigExecSchemaAuthoring --missing-schema)
set_tests_properties(testRigExecSchemaAuthoringMissing PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=")

add_executable(testRigExecRigBuilder tests/testRigExecRigBuilder.cpp)
target_link_libraries(testRigExecRigBuilder PRIVATE rigExecAuthoring)
target_compile_definitions(testRigExecRigBuilder PRIVATE
    ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecRigBuilder COMMAND testRigExecRigBuilder)

# The engine authors nothing: asserted by whole-scene equality rather than by
add_executable(testRigExecNoAuthoring tests/testRigExecNoAuthoring.cpp)
target_link_libraries(testRigExecNoAuthoring PRIVATE rigExecImaging)
target_compile_definitions(testRigExecNoAuthoring PRIVATE ${_rigexec_schema_resource_define})
rigexec_add_test(NAME testRigExecNoAuthoring COMMAND testRigExecNoAuthoring
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBounds tests/testRigExecBounds.cpp)
target_link_libraries(testRigExecBounds PRIVATE plug tf gf usd usdGeom)
target_compile_definitions(testRigExecBounds PRIVATE ${_rigexec_schema_resource_define})
add_dependencies(testRigExecBounds rigExecImaging)
rigexec_add_test(NAME testRigExecBounds COMMAND testRigExecBounds
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(probeCodingError tests/probeCodingError.cpp)
target_link_libraries(probeCodingError PRIVATE rigExec)
target_compile_definitions(probeCodingError PRIVATE ${_rigexec_schema_resource_define})

add_executable(probeImagingPipeline tests/probeImagingPipeline.cpp)
target_link_libraries(probeImagingPipeline PRIVATE rigExecImaging usdImaging usdGeom)
target_compile_definitions(probeImagingPipeline PRIVATE ${_rigexec_schema_resource_define})

function(rigexec_add_serial_test name)
    rigexec_add_test(NAME ${name}Serial COMMAND ${name} ${ARGN})
    set(_env)
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    list(FILTER _env EXCLUDE REGEX "^RIGEXEC_ENABLE_PARALLEL_EVAL=")
    list(APPEND _env "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
    set_tests_properties(${name}Serial PROPERTIES ENVIRONMENT "${_env}")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}Serial PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
    get_test_property(${name} FAIL_REGULAR_EXPRESSION _plain_fail)
    if (_plain_fail)
        set_tests_properties(${name}Serial PROPERTIES
            FAIL_REGULAR_EXPRESSION "${_plain_fail}")
    endif()
endfunction()
foreach(_suite testRigExecBackgroundScheduler testRigExecConstraints
               testRigExecFrameCacheSparsity
               testRigExecInteractive testRigExecPoseInterpolator)
    rigexec_add_serial_test(${_suite})
endforeach()

function(rigexec_add_digest_gate_test name)
    rigexec_add_test(NAME ${name}DigestGate COMMAND ${ARGN})
    set(_env "RIGEXEC_VERIFY_DIGEST_GATE=1")
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    set_tests_properties(${name}DigestGate PROPERTIES ENVIRONMENT "${_env}")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}DigestGate PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
endfunction()
foreach(_suite testRigExecConstraints testRigExecInteractive)
    rigexec_add_digest_gate_test(${_suite} ${_suite})
endforeach()

function(rigexec_add_scoped_clear_test name)
    rigexec_add_test(NAME ${name}ScopedClears COMMAND ${ARGN})
    set(_env "RIGEXEC_VERIFY_SCOPED_CLEARS=1")
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    set_tests_properties(${name}ScopedClears PROPERTIES
        ENVIRONMENT "${_env}"
        FAIL_REGULAR_EXPRESSION "comparison mismatch")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}ScopedClears PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
endfunction()
foreach(_suite testRigExecInteractive testRigExecScopedClears
               testRigExecSkinTopology)
    rigexec_add_scoped_clear_test(${_suite} ${_suite})
endforeach()
foreach(_suite testRigExecBakedMode testRigExecValueEdits)
    rigexec_add_scoped_clear_test(${_suite} ${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()
foreach(_suite testRigExecCompiledRigRests testRigExecNoAuthoring)
    rigexec_add_digest_gate_test(${_suite} ${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()
foreach(_suite testRigExecArm testRigExecBakedMode testRigExecBakedSchedule
               testRigExecBinary
               testRigExecFrameCacheCones_Verify testRigExecFrozenContext
               testRigExecImaging testRigExecImagingPlayback
               testRigExecNoAuthoring testRigExecRuntimeGeometry
               testRigExecRuntimeLoader testRigExecRuntimePose
               testRigExecRuntimeWeights testRigExecSolverBake
               testRigExecSolverStacking testRigExecVolumeWeights
               testRigExecWeightOverlay)
    rigexec_add_serial_test(${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()
endif()  # RIGEXEC_WITH_USD

endif()  # RIGEXEC_BUILD_TESTS

if (RIGEXEC_WITH_USD)
if (CMAKE_CROSSCOMPILING)
    set(_rigexec_python_default OFF)
else()
    # Probe quietly: the interpreter must be found with dev headers, and the
    find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
    if (Python3_FOUND AND Python3_Development.Module_FOUND)
        execute_process(
            COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
            OUTPUT_VARIABLE _rigexec_pybind11_probe
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _rigexec_pybind11_probe_result)
        if (_rigexec_pybind11_probe_result EQUAL 0 AND EXISTS "${_rigexec_pybind11_probe}")
            set(_rigexec_python_default ON)
        else()
            set(_rigexec_python_default OFF)
        endif()
    else()
        set(_rigexec_python_default OFF)
    endif()
endif()
option(RIGEXEC_BUILD_PYTHON "Build the RigExec Python bindings (_rigexec + rigexec)"
       ${_rigexec_python_default})

if (RIGEXEC_BUILD_PYTHON)
    add_subdirectory(python)
endif()

if (NOT CMAKE_CROSSCOMPILING AND TARGET python AND TARGET garch AND TARGET glf)
    set(_rigexec_usdnoodles_default ON)
else()
    set(_rigexec_usdnoodles_default OFF)
endif()
option(RIGEXEC_BUILD_USDNOODLES
       "Build the usdNoodles node-graph editor plugin for usdview"
       ${_rigexec_usdnoodles_default})

if (RIGEXEC_BUILD_USDNOODLES)
    find_package(OpenGL REQUIRED)
    add_subdirectory(plugin/usdNoodles)
endif()

if (RIGEXEC_BUILD_PYTHON AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)
    # rigexec.imaging through ctypes against this build's rigExecImaging.
    rigexec_add_test(NAME testRigExecImagingPublishedPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_imaging_published.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_python.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPythonSchemaAuthoring
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_schema_authoring.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # rigexec.binding.generate_skin_weights against hand-computed rows, and
    # the volumeSkin fixtures' rows against the generator.
    rigexec_add_test(NAME testRigExecGenerateSkinWeights
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_generate_skin_weights.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecSplineIkPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_spline_ik.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecFkControlSpacePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_fk_control_space.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecFkStartFramePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_fk_start_frame.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecBipedHandPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_biped_hand.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecTrsMastersPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_trs_masters.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecWireInvariantPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_wire_invariant.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The deformation stack must keep running in its authored order: the head
    # wires are the first four deformers on the face and everything after
    # them is shaped by what they leave behind.
    rigexec_add_test(NAME testRigExecDeformerOrderPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_deformer_order.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A pose sample sitting on top of its neutral makes the RBF degenerate
    # and the weights run away.
    rigexec_add_test(NAME testRigExecPoseRotationsPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_pose_rotations.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The projector's ray is the SOURCE FRAME's Z axis, and the two eye
    # bind joints are 180 degrees apart rather than mirrored.
    rigexec_add_test(NAME testRigExecEyeSymmetryPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_eye_symmetry.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A drag is a SWEEP: a deformer correct at both ends of one can
    # still jump in the middle, and every other check here sets one
    # value and looks once.
    rigexec_add_test(NAME testRigExecFaceSmoothPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_face_smooth.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The spine's volume preservation must stay in the spine.
    rigexec_add_test(NAME testRigExecRigidHeadPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_rigid_head.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A radial weight turns a fraction of the rotation about the pivot the
    # transform turns about; a linear one follows the chord, which falls
    # inside the arc. On a blink that walks the lid into the eyeball.
    rigexec_add_test(NAME testRigExecBlinkClearancePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_blink_clearance.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
            "$<TARGET_FILE:rigExecBakeTool>" "$<TARGET_FILE:rigExecPose>")
    # Two lips on one patch touch but never cross, at any mix of the 27
    # ARKit avars of examples/18_PatchMouth.usda.
    rigexec_add_test(NAME testRigExecPatchMouthPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_patch_mouth.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecFaceDrivesPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_face_drives.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # inputs:enabled must actually switch a deformer off: the Deformer
    # Toggles panel is only a bisecting tool if the switch it writes is
    # one the evaluator obeys.
    rigexec_add_test(NAME testRigExecDeformerEnabledPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_deformer_enabled.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPoseCyclePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_pose_cycle.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPsdNumeric COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_psd_numeric.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # AnimatedMaps step tables through rigexec.Rig: weight, active, offset
    # and unclamped value maps, the disable switch, and an undeclared read.
    rigexec_add_test(NAME testRigExecStepMapsPython COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_step_maps.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecSpaceSwitch COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecLiveOperationGraph COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_live_operation_graph.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # Switches nested under switched controls: bakeable, and the program,
    # the walk and the drag path agree bit for bit; a switch reading under
    # its own target is refused.
    rigexec_add_test(NAME testRigExecSpaceSwitchVersions COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch_versions.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    # Again with every frame's cone checked against the whole program, one
    # step per cluster: a recomposed version reads avars a cone must reach.
    rigexec_add_test(NAME testRigExecSpaceSwitchVersionsCones
        COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch_versions.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    rigexec_add_test(NAME testRigExecBake COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_bake.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecInverse COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_inverse.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecSkinLayoutOverrides COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_skin_layout_overrides.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecCurveKeyOverrides COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_curve_key_overrides.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPathPointOverrides COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_path_point_overrides.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecBakedCone COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_cone.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecOpTracePython COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_op_trace.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # Under the serial executor the file also checks that no two ops of a
    # thread overlap from memo to publication.
    rigexec_add_test(NAME testRigExecOpTracePython_serial COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_op_trace.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecGil COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_gil.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecBakedPsd COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_psd.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRestLocal COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_local.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRestMigration COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_migration.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testInterveningXform COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_intervening_xform.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecCompiledRigConstants COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_compiled_rig_constants.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecWireMover COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_wire_mover.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecFloatCurveOp COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_float_curve_op.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecMatrixSpace COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_matrix_space.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecMoverDiscovery COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_mover_discovery.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecSkippedOperations COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_skipped_operations.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecBakedAvarPatch COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_baked_avar_patch.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecPxrHygiene COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_pxr_import_hygiene.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecBakedAttributePython
        COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_attribute.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testRigExecSdkSmoke
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_sdk_binding_smoke.py"
            "$<TARGET_FILE_DIR:_rigexec>")

    rigexec_add_test(NAME testRigExecUndo
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_undo.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testEditorPublications
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_editor_publications.py")
    rigexec_add_test(NAME testGizmoMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_math.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGizmoFrames
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_frames.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGizmoRotation
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_rotation.py")
    rigexec_add_test(NAME testGizmoTargets
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_targets.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGizmoModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_model.py")
    rigexec_add_test(NAME testPoseReaderViz
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_pose_reader_viz.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testLayerOpinionsModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_layer_opinions_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testAvarEditorModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_avar_editor_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testCompositionArcsModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_composition_arcs_model.py")
    rigexec_add_test(NAME testPathSpelling
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_path_spelling.py")
    rigexec_add_test(NAME testMoverChainOrder
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_mover_chain_order.py")
    rigexec_add_test(NAME testIkFkMatch
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_ikfk_match.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testSpaceMatch
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_space_match.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testAutoClavicle
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_auto_clavicle.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGizmoPreview
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_preview.py")
    rigexec_add_test(NAME testGizmoDeferredRelease
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_deferred_release.py")
    set_tests_properties(testGizmoDeferredRelease PROPERTIES SKIP_RETURN_CODE 77)
    rigexec_add_test(NAME testGizmoOwnCommit
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_own_commit.py")
    set_tests_properties(testGizmoOwnCommit PROPERTIES SKIP_RETURN_CODE 77)
    rigexec_add_test(NAME testGizmoDocumentEdit
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_document_edit.py")
    set_tests_properties(testGizmoDocumentEdit PROPERTIES SKIP_RETURN_CODE 77)
    rigexec_add_test(NAME testGizmoScreen
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_screen.py")
    rigexec_add_test(NAME testGizmoSettings
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_settings.py")
    rigexec_add_test(NAME testGraphModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_graph_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGraphEditorModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_graph_editor_model.py")
    rigexec_add_test(NAME testVolumeWeightModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_volume_weight_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testPathCurveModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_path_curve_model.py")
    rigexec_add_test(NAME testGraphScreen
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_graph_screen.py")
    rigexec_add_test(NAME testRigExecStageEdits
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_test(NAME testGizmoDrag
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_drag.py")
    rigexec_add_test(NAME testGizmoSnap
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_snap.py")
    rigexec_add_test(NAME testViewCubeMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_viewcube_math.py")
    rigexec_add_test(NAME testGizmoMarquee
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_marquee.py")
    rigexec_add_test(NAME testCacheStripModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_cache_strip_model.py")
    rigexec_add_test(NAME testTimelineModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_timeline_model.py")
    rigexec_add_test(NAME testPlayblastModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_playblast_model.py")
    rigexec_add_test(NAME testCacheStripPanel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_cache_strip_panel.py")
    rigexec_add_test(NAME testUsdviewFrameCacheDriver
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_usdview_frame_cache.py")
    rigexec_add_test(NAME testImagingHandle
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_imaging_handle.py")
    rigexec_add_test(NAME testSessionState
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_session_state.py")

    rigexec_add_test(NAME testRigExecProfilerModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_profiler_model.py")

    foreach(_python_test testRigExecPython testRigExecPythonSchemaAuthoring
                         testRigExecGenerateSkinWeights
                         testRigExecSplineIkPython testRigExecPatchMouthPython
                         testRigExecFkControlSpacePython
                         testRigExecBake testRigExecInverse
                         testRigExecPsdNumeric testRigExecStepMapsPython
                         testRigExecSpaceSwitch
                         testRigExecLiveOperationGraph
                         testRigExecSpaceSwitchVersions
                         testRigExecSpaceSwitchVersionsCones
                         testRestLocal testRestMigration
                         testInterveningXform testRigExecCompiledRigConstants
                         testRigExecBakedAvarPatch testRigExecWireMover
                         testRigExecPxrHygiene
                         testRigExecMatrixSpace
                         testRigExecMoverDiscovery
                         testRigExecSkippedOperations
                         testRigExecFloatCurveOp
                         testRigExecOpTracePython testRigExecOpTracePython_serial
                         testRigExecUndo testEditorPublications testGizmoMath testGizmoScreen
                         testGizmoFrames testGizmoRotation testGizmoTargets testGizmoModel
                         testPoseReaderViz
                         testLayerOpinionsModel testCompositionArcsModel
                         testPathSpelling
                         testMoverChainOrder testIkFkMatch testSpaceMatch
                         testAutoClavicle
                         testAvarEditorModel
                         testGizmoPreview testGizmoDeferredRelease testGizmoOwnCommit
                         testGizmoDocumentEdit testGizmoSettings testGizmoDrag testGizmoSnap
                         testViewCubeMath testRigExecStageEdits
                         testGraphModel testGraphEditorModel testVolumeWeightModel
                         testPathCurveModel testGraphScreen
                         testCacheStripModel testTimelineModel
                         testPlayblastModel
                         testCacheStripPanel testUsdviewFrameCacheDriver
                         testImagingHandle
                         testSessionState testRigExecProfilerModel
                         testGizmoMarquee testRigExecImagingPublishedPython
                         testRigExecGil testRigExecBakedPsd testRigExecBakedCone
                         testRigExecBakedAttributePython
                         testRigExecSdkSmoke
                         testRigExecBipedHandPython testRigExecBlinkClearancePython
                         testRigExecDeformerEnabledPython
                         testRigExecDeformerOrderPython testRigExecEyeSymmetryPython
                         testRigExecFaceDrivesPython testRigExecFaceSmoothPython
                         testRigExecFkStartFramePython testRigExecRigidHeadPython
                         testRigExecPoseCyclePython testRigExecPoseRotationsPython
                         testRigExecTrsMastersPython testRigExecWireInvariantPython
                         testRigExecCurveKeyOverrides testRigExecPathPointOverrides
                         testRigExecSkinLayoutOverrides)
        set_tests_properties(${_python_test} PROPERTIES ENVIRONMENT
            "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/python")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "RIGEXEC_USD_INSTALL=${RIGEXEC_TEST_USD_DIR}")
    endforeach()
    set_property(TEST testRigExecOpTracePython_serial APPEND PROPERTY
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=serial")
    # APPEND, after the foreach above: its set_tests_properties owns the
    # base ENVIRONMENT value for every listed test.
    set_property(TEST testRigExecImagingPublishedPython APPEND PROPERTY
        ENVIRONMENT "RIGEXEC_IMAGING_DLL=$<TARGET_FILE:rigExecImaging>")
    set_property(TEST testRigExecSpaceSwitchVersionsCones APPEND PROPERTY
        ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1" "RIGEXEC_BAKED_GRAIN_US=0")
    rigexec_add_digest_gate_test(testRigExecStageEdits
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_scoped_clear_test(testRigExecStageEdits
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
endif()

if (RIGEXEC_BUILD_USDNOODLES AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)
    rigexec_add_test(NAME testUsdNoodles
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -s . -p "test*.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/python/UsdNoodles/testenv")
    set_tests_properties(testUsdNoodles PROPERTIES ENVIRONMENT
        "RIGEXEC_USD_INSTALL=${RIGEXEC_TEST_USD_DIR}")
    file(GLOB _rigexec_usd_site_dirs LIST_DIRECTORIES true
        "${RIGEXEC_TEST_USD_DIR}/Lib/site-packages"
        "${RIGEXEC_TEST_USD_DIR}/lib/python*/site-packages"
        "${RIGEXEC_TEST_USD_DIR}/lib/python")
    foreach(_rigexec_usd_site_dir IN LISTS _rigexec_usd_site_dirs)
        if (IS_DIRECTORY "${_rigexec_usd_site_dir}/pxr")
            set_property(TEST testUsdNoodles APPEND PROPERTY
                ENVIRONMENT_MODIFICATION
                "PYTHONPATH=path_list_prepend:${_rigexec_usd_site_dir}")
        endif()
    endforeach()
    set_property(TEST testUsdNoodles APPEND PROPERTY ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/python")
    set_property(TEST testUsdNoodles APPEND PROPERTY ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/python/UsdNoodles/testenv")
endif()

# External repositories supply their own CMakeLists and register plugin targets.
set(RIGEXEC_MOVER_PLUGIN_DIRS "" CACHE STRING
    "Semicolon-separated paths to external mover plugin source directories")
foreach(_mover_dir IN LISTS RIGEXEC_MOVER_PLUGIN_DIRS)
    get_filename_component(_mover_dir "${_mover_dir}" ABSOLUTE
        BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if (NOT EXISTS "${_mover_dir}/CMakeLists.txt")
        message(FATAL_ERROR "Mover plugin directory has no CMakeLists.txt: ${_mover_dir}")
    endif()
    string(SHA256 _mover_id "${_mover_dir}")
    add_subdirectory("${_mover_dir}" "${CMAKE_CURRENT_BINARY_DIR}/mover-plugins/${_mover_id}")
endforeach()
endif()  # RIGEXEC_WITH_USD

rigexec_label_tests()

get_property(_rigexec_tests DIRECTORY PROPERTY TESTS)
foreach(_rigexec_test IN LISTS _rigexec_tests)
    get_property(_rigexec_test_timeout TEST ${_rigexec_test} PROPERTY TIMEOUT)
    if (NOT _rigexec_test_timeout)
        set_tests_properties(${_rigexec_test} PROPERTIES TIMEOUT 300)
    endif ()
    if (WIN32)
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/bin"
            "PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    elseif (APPLE)
        # Mach-O reads DYLD_LIBRARY_PATH and ELF reads LD_LIBRARY_PATH;
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "DYLD_LIBRARY_PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "DYLD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    else()
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "LD_LIBRARY_PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "LD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endforeach()
# Every test gets 300s. Several suites sit past two minutes on a 4-core
# runner, and a 120s cap kills them before the assertions finish. The
# frame-cache suites also wait on short rendezvous, so each takes the
# whole ctest slot instead of sharing it.
if(TEST testRigExecImagingFrameCache)
    set_tests_properties(testRigExecImagingFrameCache
        testRigExecImagingFrameCacheDefault PROPERTIES PROCESSORS 4)
endif()

# Precompiled headers: the stable OpenUSD and standard headers nearly every
# translation unit parses, compiled once. Never a project header, so editing
# project code never rebuilds one. MSVC only. A consumer may define more
# macros than its PCH was built with but none fewer, so the shared owner
# links only usd, whose interface definitions every USD-facing target has.
# The runtime stays USD-free with a standard-library PCH of its own.
# -DCMAKE_DISABLE_PRECOMPILE_HEADERS=ON builds without them, which checks
# that every source includes what it uses.
if (MSVC AND NOT CMAKE_DISABLE_PRECOMPILE_HEADERS)
    set(_rigexec_pch_std
        <algorithm> <array> <atomic> <cmath> <cstddef> <cstdint> <cstdio>
        <cstring> <functional> <limits> <map> <memory> <optional> <set>
        <string> <tuple> <type_traits> <unordered_map> <unordered_set>
        <utility> <variant> <vector>)
if (RIGEXEC_WITH_USD)
    set(_rigexec_pch_usd
        <pxr/pxr.h>
        <pxr/base/gf/matrix3d.h> <pxr/base/gf/matrix4d.h>
        <pxr/base/gf/quatd.h> <pxr/base/gf/rotation.h>
        <pxr/base/gf/vec2f.h> <pxr/base/gf/vec3d.h> <pxr/base/gf/vec3f.h>
        <pxr/base/gf/vec3i.h> <pxr/base/gf/vec4d.h>
        <pxr/base/plug/registry.h>
        <pxr/base/tf/diagnostic.h> <pxr/base/tf/functionRef.h>
        <pxr/base/tf/hash.h> <pxr/base/tf/notice.h> <pxr/base/tf/span.h>
        <pxr/base/tf/token.h> <pxr/base/tf/type.h> <pxr/base/tf/weakBase.h>
        <pxr/base/ts/spline.h>
        <pxr/base/vt/array.h> <pxr/base/vt/dictionary.h>
        <pxr/base/vt/value.h>
        <pxr/usd/sdf/layer.h> <pxr/usd/sdf/path.h> <pxr/usd/sdf/types.h>
        <pxr/usd/sdf/valueTypeName.h>
        <pxr/usd/usd/attribute.h> <pxr/usd/usd/attributeQuery.h>
        <pxr/usd/usd/common.h> <pxr/usd/usd/notice.h>
        <pxr/usd/usd/object.h> <pxr/usd/usd/prim.h>
        <pxr/usd/usd/relationship.h> <pxr/usd/usd/stage.h>
        <pxr/usd/usd/timeCode.h> <pxr/usd/usdGeom/xformCache.h>)

    file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/rigExecUsdPch.cpp"
        CONTENT "// Owns the shared USD precompiled header.\n")
    add_library(rigExecUsdPch OBJECT
        "${CMAKE_CURRENT_BINARY_DIR}/rigExecUsdPch.cpp")
    target_link_libraries(rigExecUsdPch PRIVATE usd)
    target_precompile_headers(rigExecUsdPch PRIVATE
        ${_rigexec_pch_std} ${_rigexec_pch_usd})
endif()  # RIGEXEC_WITH_USD
    target_precompile_headers(rigExecRuntime PRIVATE ${_rigexec_pch_std})

if (RIGEXEC_WITH_USD)
    # The libraries, and every test, bench or probe executable linking one
    # of the USD-facing libraries. The imaging object libraries carry the
    # same compile options and definitions as rigExecImaging, so they
    # reuse the same PCH.
    set(_rigexec_pch_users rigExec rigExecScene rigExecOracle rigExecPlayback rigExecImaging
        rigExecImagingContext rigExecImagingFrames
        rigExecImagingInteraction rigExecImagingQueries rigExecImagingHost
        rigExecCompile rigExecEval rigExecSchedule rigExecVerify
        rigExecBake)
    get_property(_rigexec_pch_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_target IN LISTS _rigexec_pch_targets)
        get_target_property(_type ${_target} TYPE)
        get_target_property(_links ${_target} LINK_LIBRARIES)
        if (_type STREQUAL "EXECUTABLE" AND _links AND
            _target MATCHES "^(test|bench|probe)")
            foreach(_lib rigExec rigExecBake rigExecImaging rigExecAuthoring
                    rigExecSampler rigExecPlayback)
                if (_lib IN_LIST _links)
                    list(APPEND _rigexec_pch_users ${_target})
                    break()
                endif()
            endforeach()
        endif()
    endforeach()
    foreach(_target IN LISTS _rigexec_pch_users)
        target_precompile_headers(${_target} REUSE_FROM rigExecUsdPch)
    endforeach()
endif()  # RIGEXEC_WITH_USD
endif()

# Executables that compile a stage through rigExec alone name the scene
# library, so the hooks are loaded. Imaging and bake link it privately:
# playback, runtime, and imaging tests load the library and are not scene
# tests.
foreach(_rigexec_scene_exe IN ITEMS
    benchEditLatency benchFrameCache benchFrameCacheWarm benchRigSpeed
    testRigExecDeterminism
    probeCodingError
    rigExecPose rigExecBakeTool
    testRigExecAffineFrames testRigExecArm testRigExecBakedAttribute
    testRigExecBakedMode testRigExecBakedSchedule testRigExecBinary
    testRigExecChainLevels testRigExecConstraints testRigExecCrossDomainInputs
    testRigExecDefaultSpaces testRigExecDeltaMush testRigExecEnvelopeVolumes
    testRigExecCompiledProgram testRigExecCompiledRigRests testRigExecExampleParity testRigExecExecCrossCheck
    testRigExecExecGoldens testRigExecExternalMovers
    testRigExecFrameCacheCones_Verify testRigExecFrameCacheSparsity
    testRigExecFrozenContext testRigExecGeometryOpsBakedParity
    testRigExecGeometryProgram testRigExecGraphBindings
    testRigExecGroupForwarding testRigExecInputActions testRigExecInputReplayHosting
    testRigExecOpSteps
    testRigExecInteractive testRigExecInteropNotices testRigExecLattice testRigExecLeaves
    testRigExecOneLoopAcceptance testRigExecOpTrace testRigExecOutsideTransforms
    testRigExecPatchMover testRigExecPhaseBindings testRigExecPoseInterpolator
    testRigExecPropertyOps testRigExecRestOps testRigExecRuntimeInputs
    testRigExecScenePoseOps testRigExecScopedClears testRigExecSkinTopology
    testRigExecSolverBake
    testRigExecSolverStacking testRigExecSurfaceBinding testRigExecSurfaceProjector
    testRigExecSurfaceSnap testRigExecValueEdits testRigExecVolumeWeights
    testRigExecWeightProgram testRigExecWrinkle
    testRigExecRigLogic)
    if (TARGET ${_rigexec_scene_exe})
        target_link_libraries(${_rigexec_scene_exe} PRIVATE rigExecScene)
    endif()
endforeach()

# Oracles are linked only where a test runs cpu-reference, golden capture,
# or an exec-row capture. Imaging, playback, and runtime tests do not.
foreach(_rigexec_oracle_exe IN ITEMS
    rigExecPose benchEditLatency
    testRigExecArm testRigExecBakedAttribute testRigExecBakedMode
    testRigExecBakedSchedule testRigExecConstraints testRigExecDeltaMush
    testRigExecCompiledRigRests testRigExecExampleParity
    testRigExecExecCrossCheck testRigExecExecGoldens
    testRigExecExternalMovers testRigExecFrozenContext
    testRigExecBakedOpParity
    testRigExecGeometryOpsBakedParity testRigExecGoldenPose
    testRigExecGoldenSuiteObserver testRigExecInputActions testRigExecInteractive
    testRigExecOpTrace testRigExecOracleInputs testRigExecPatchMover testRigExecPhaseBindings
    testRigExecPropertyOps testRigExecRestOps testRigExecSkinTopology
    testRigExecSolverStacking testRigExecVolumeWeights testRigExecWrinkle)
    if (TARGET ${_rigexec_oracle_exe})
        target_link_libraries(${_rigexec_oracle_exe} PRIVATE rigExecOracle)
    endif()
endforeach()

# Placement invariant for the exact-FP finalize walk: it must stay the last
# code in this scope that mutates target options. It runs after every
# target_precompile_headers producer and REUSE_FROM consumer above and after
# every compile-affecting target_link_libraries call in this scope, the
# scene and oracle links just above included, because interface and
# per-source options reach the command line after the appended exact-FP
# flags. Only install() rules follow it here.
rigexec_apply_exact_fp_flags_to_directory_targets()

# Install
include(CMakePackageConfigHelpers)

set(RIGEXEC_INSTALL_TARGETS
    rigExecLzma rigExecGraph rigExecEvalCore rigExecMath rigExecMathGf rigExec
    rigExecScene
    rigExecOracle rigExecAuthoring rigExecImaging rigExecGpu
    rigExecFlatBuffers rigExecBinary rigExecBake rigExecRuntime rigExecSampler
    rigExecBuiltins rigExecOpCore rigExecPlayback)
if (NOT RIGEXEC_WITH_USD)
    set(_rigexec_declared_install_targets ${RIGEXEC_INSTALL_TARGETS})
    set(RIGEXEC_INSTALL_TARGETS "")
    foreach(_target IN LISTS _rigexec_declared_install_targets)
        if (TARGET ${_target})
            list(APPEND RIGEXEC_INSTALL_TARGETS ${_target})
        endif()
    endforeach()
endif()
install(TARGETS ${RIGEXEC_INSTALL_TARGETS}
    EXPORT rigExecTargets
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")

# A named-target build never reaches the ALL hook, and the install rules
# consume the staged manifests.
foreach (_target ${RIGEXEC_INSTALL_TARGETS} _rigexec rigExecPose rigExecBakeTool
                 rigExecReplayInputs usdRigView)
    if (TARGET ${_target})
        add_dependencies(${_target} rigexecSdkManifests)
    endif ()
endforeach ()

if (RIGEXEC_WITH_USD)
install(FILES libs/rigExecScene/sceneLinkAnchor.cpp libs/rigExecOracle/oracleLinkAnchor.cpp
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")
install(FILES libs/rigExecOracle/registerOracle.h
    DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/rigExecOracle")
endif()  # RIGEXEC_WITH_USD

# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
set(_rigexec_usd_header_dirs rigExecScene rigExecAuthoring rigExecImaging
    rigExecGpu rigExecBake rigExecSampler rigExecPlayback)
foreach(_lib rigExecGraph rigExecEvalCore rigExecScene rigExecMath rigExec rigExecSchema rigExecAuthoring rigExecImaging rigExecGpu rigExecBinary rigExecBake rigExecRuntime rigExecSampler rigExecStdTypes rigExecRigTypes rigExecBuiltins rigExecPlayback)
    if (NOT RIGEXEC_WITH_USD AND _lib IN_LIST _rigexec_usd_header_dirs)
        continue()
    endif()
    install(DIRECTORY "libs/${_lib}/"
        DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/${_lib}"
        FILES_MATCHING
            PATTERN ".*" EXCLUDE
            PATTERN "__pycache__" EXCLUDE
            PATTERN "guideBoundsInternal.h" EXCLUDE
            PATTERN "registryInternal.h" EXCLUDE
            PATTERN "evaluatorTesting.h" EXCLUDE
            PATTERN "internal" EXCLUDE
            PATTERN "*.h")
endforeach()
# rigExec installs api/ only, plus the internal headers that installed
# headers still include; tools/check_internal_includes.py keeps the list
# equal to that closure.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/rigexec_installed_internal.txt")
file(STRINGS tools/rigexec_installed_internal.txt _rigexec_installed_internal
    REGEX "^[^#]")
foreach(_header IN LISTS _rigexec_installed_internal)
    get_filename_component(_dir "${_header}" DIRECTORY)
    install(FILES "libs/rigExec/internal/${_header}"
        DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/rigExec/internal/${_dir}")
endforeach()
# The FlatBuffers runtime headers the generated rigExecBinary headers
# include; their license installs with the others below.
install(DIRECTORY thirdparty/flatbuffers/include/flatbuffers
    DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}")

if (RIGEXEC_WITH_USD)
set(_rigexec_schema_resources
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
set(_rigexec_schema_pluginfo "${_rigexec_schema_resources}/plugInfo.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/generatedSchema.usda")
file(READ
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    _rigexec_schema_pluginfo_content)
string(REPLACE "\"Type\": \"resource\"" "\"Type\": \"library\""
    _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")

foreach(_rigexec_boundable
        RigExecXformable RigExecControl RigExecJoint RigExecFkChain
        RigExecTwoBoneIk RigExecBlendPointFrames RigExecTwistDistribution
        RigExecRibbon RigExecSplineIk RigExecVolumeWeight RigExecSphereWeight
        RigExecPlaneWeight RigExecCurveWeight RigExecClothConstraintSet)
    string(REPLACE
        "\"schemaIdentifier\": \"${_rigexec_boundable}\""
        "\"implementsComputeExtent\": true, \"schemaIdentifier\": \"${_rigexec_boundable}\""
        _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")
endforeach()
string(REPLACE "\"Name\": \"rigExecSchema\""
    "\"LibraryPath\": \"../../../$<TARGET_FILE_NAME:rigExecImaging>\", \"Name\": \"rigExecSchema\""
    _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")
file(GENERATE OUTPUT "${_rigexec_schema_pluginfo}"
    CONTENT "${_rigexec_schema_pluginfo_content}")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/generatedSchema.usda"
    "${_rigexec_schema_resources}/generatedSchema.usda" COPYONLY)

# The node-graph icons the plugInfo type entries name (a class's
# extraPlugInfo nodeGraphIcon, relative to these resources): exactly those,
# so the plugin ships no art it does not declare.
file(STRINGS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    _rigexec_icon_lines REGEX "\"nodeGraphIcon\"")
set(_rigexec_schema_icons "")
foreach(_rigexec_icon_line IN LISTS _rigexec_icon_lines)
    string(REGEX REPLACE ".*\"nodeGraphIcon\": \"icons/([^\"]+)\".*" "\\1"
        _rigexec_icon "${_rigexec_icon_line}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/icons/${_rigexec_icon}"
        "${_rigexec_schema_resources}/icons/${_rigexec_icon}" COPYONLY)
    list(APPEND _rigexec_schema_icons
        "${CMAKE_CURRENT_SOURCE_DIR}/icons/${_rigexec_icon}")
endforeach()

install(FILES
        "${_rigexec_schema_pluginfo}"
        "${_rigexec_schema_resources}/generatedSchema.usda"
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources")
install(FILES ${_rigexec_schema_icons}
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources/icons")

install(FILES plugin/usdRigViewShared/usdRigViewMenu.py
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}")

install(DIRECTORY plugin/rigExecUsdview/
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}/rigExecUsdview"
    FILES_MATCHING
        PATTERN ".*" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.py"
        PATTERN "*.json"
        PATTERN "*.png")


set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecImaging/resources/plugInfo.json.in")
set(_rigexec_pluginfo
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources/plugInfo.json")
file(READ
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecImaging/resources/plugInfo.json.in"
    _rigexec_pluginfo_content)
string(REPLACE "@RIGEXEC_IMAGING_LIBRARY_FILENAME@"
    "$<TARGET_FILE_NAME:rigExecImaging>"
    _rigexec_pluginfo_content "${_rigexec_pluginfo_content}")
file(GENERATE OUTPUT "${_rigexec_pluginfo}"
    CONTENT "${_rigexec_pluginfo_content}")

install(FILES "${_rigexec_pluginfo}"
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecImaging/resources")
endif()  # RIGEXEC_WITH_USD

# CMake package, so consumers can find_package(rigExec) and link
install(EXPORT rigExecTargets
    FILE rigExecTargets.cmake
    NAMESPACE rigExec::
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")

configure_package_config_file(
    cmake/rigExecConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfig.cmake"
    INSTALL_DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}"
    PATH_VARS RIGEXEC_INSTALL_PLUGINDIR RIGEXEC_INSTALL_PYTHONDIR
              RIGEXEC_INSTALL_LIBDIR RIGEXEC_INSTALL_DATADIR)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfigVersion.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigExecMoverPlugin.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigExecMover.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigExecOpPlugin.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigexec_flags.cmake"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generateMoverSchema.py"
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")

# What a host that compiles the USD-free sources without CMake (the Godot and
# Unreal extensions) needs: the relocated manifests, the generated runtime
# translation units they name, and the exact-FP flags this tree was built with.
# rigexec_install_sdk_manifests stages all three under one destination so
# DESTDIR keeps them together.
rigexec_install_sdk_manifests(BUILD_DIR "${RIGEXEC_SDK_MANIFEST_DIR}"
    INPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
    SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}"
    DESTINATION "${RIGEXEC_INSTALL_DATADIR}"
    FLAGS_FILE "${CMAKE_CURRENT_BINARY_DIR}/exactFpFlags.txt")
install(FILES LICENSE NOTICE THIRD_PARTY_NOTICES.md
    DESTINATION "share/rigExec")
install(FILES plugin/usdNoodles/LICENSE.txt
    DESTINATION "share/rigExec/licenses" RENAME OpenUSD-LICENSE.txt)
install(FILES thirdparty/flatbuffers/LICENSE
    DESTINATION "share/rigExec/licenses" RENAME FlatBuffers-LICENSE.txt)

install(FILES thirdparty/lzma/NOTICE.md
    DESTINATION "share/rigExec/licenses" RENAME LZMA-SDK-NOTICE.md)
install(FILES thirdparty/lzma/DOC/lzma-sdk.txt
    DESTINATION "share/rigExec/licenses" RENAME LZMA-SDK-LICENSE.txt)

# Source-relative copies of every legal text THIRD_PARTY_NOTICES.md links, so
# those links resolve inside the install prefix. The usdNoodles texts ship
# whether or not RIGEXEC_BUILD_USDNOODLES built the editor; they are checked in.
install(FILES thirdparty/lzma/NOTICE.md
    DESTINATION "share/rigExec/thirdparty/lzma")
install(FILES thirdparty/lzma/DOC/lzma-sdk.txt
    DESTINATION "share/rigExec/thirdparty/lzma/DOC")
install(FILES thirdparty/flatbuffers/LICENSE
    DESTINATION "share/rigExec/thirdparty/flatbuffers")
install(FILES plugin/usdNoodles/LICENSE.txt
         plugin/usdNoodles/NOODLES_LICENSE.txt
         plugin/usdNoodles/THIRD_PARTY_LICENSES.txt
    DESTINATION "share/rigExec/plugin/usdNoodles")

# The manifest ships with the prefix so a consumer can trace every installed
# legal text back to its source, and the checker that keeps that record honest
# ships beside it with the member reader it imports, so an installed consumer
# runs it without a checkout. docs/references.md and docs/concepts/riglogic.md
# are documentation prose, not legal texts, and are deliberately not installed.
install(FILES tools/license_manifest.json
         tools/check_licenses.py
         docs/legal_members.py
    DESTINATION "share/rigExec/tools")

# These call rigExecMathGf archive functions directly; the rigExec shared
# library exports only the ones it references itself.
foreach(_rigexec_math_user
        benchFrameCache
        benchFrameCacheWarm
        testRigExecAffineFrames
        testRigExecArm
        testRigExecBakedSchedule
        testRigExecBinary
        testRigExecConstraints
        testRigExecDeltaMush
        testRigExecLattice
        testRigExecMoverGraph
        testRigExecParallelKernels
        testRigExecPatchMover
        testRigExecPropertyOps
        testRigExecRuntimePose
        testRigExecRuntimeWeights
        testRigExecSceneGeometryOps
        testRigExecScenePoseOps
        testRigExecSurfaceSnap
        testRigExecVolumeWeights
        testRigExecWrinkle
    )
    if(TARGET ${_rigexec_math_user})
        target_link_libraries(${_rigexec_math_user} PRIVATE rigExecMathGf)
    endif()
endforeach()
