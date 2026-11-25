#include "rigExecOps/library.h"

namespace rigExecOps {
namespace {

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    RegisterPropertyMathOps(registrar);
    RegisterPointOps(registrar);
    RegisterDeltaMushOps(registrar);
    RegisterLatticeOps(registrar);
    RegisterWrinkleOps(registrar);
    RegisterSurfaceProjectOps(registrar);
    RegisterMatrixDeformOps(registrar);
    RegisterSkinOps(registrar);
    RegisterBlendShapeOps(registrar);
    RegisterDerivedOps(registrar);
    RegisterWireOps(registrar);
    RegisterRibbonOps(registrar);
    RegisterProjectorOps(registrar);
    RegisterConstraintOps(registrar);
    RegisterWeightOps(registrar);
    RegisterChainSolverOps(registrar);
    RegisterFrameBlendOps(registrar);
    RegisterRibbonFramesOps(registrar);
    RegisterSplineIkOps(registrar);
    RegisterSingleChainIkOps(registrar);
    RegisterSkeleton2dOps(registrar);
    RegisterAutoClavicleOps(registrar);
    RegisterHarmonicsOps(registrar);
    RegisterMatchOps(registrar);
    RegisterPatchOps(registrar);
    RegisterSurfaceBindingOps(registrar);
    RegisterLayeredSkinOps(registrar);
    RegisterClothOps(registrar);
    RegisterSpaceOps(registrar);
    RegisterPoseCommitOps(registrar);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "rigExecOps";
    d.registerOps = &Register;
    return d;
}();

} // namespace
} // namespace rigExecOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecOps)
{
    return &rigExecOps::kLibrary;
}
