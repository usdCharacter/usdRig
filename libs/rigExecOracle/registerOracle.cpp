#include "registerOracle.h"
#include "rigExec/oracleDispatch.h"
#include "rigExec/goldenSuite.h"
#include "rigExec/goldenPose.h"
#include "rigExec/oracleInputs.h"
#include "rigExec/scalarReferenceAdapter.h"
#include "rigExec/weightReference.h"

#include <memory>

namespace rigExec {
void RigExecOracleBeginBody(RigExecBakedProgramImpl *, uint64_t, UsdTimeCode);
bool RigExecOracleRunBody(RigExecBakedProgramImpl *, UsdTimeCode, RigExecRigPose *);
void RigExecOracleFinalizeGolden();
std::unique_ptr<RigExecGoldenSuiteObserver> RigExecOracleCreateGolden(const SdfPath &);
bool RigExecOracleEncodeGoldenValue(const VtValue &, std::string *, std::string *);
bool RigExecOracleEncodeGoldenPose(const RigExecRigPose &, std::vector<RigExecGoldenValue> *,
    std::string *, bool);
std::string RigExecOracleGoldenDouble(double);
std::string RigExecOracleGoldenEscape(const std::string &);
uint64_t RigExecOracleGoldenDigest(const std::string &);
std::string RigExecOracleGoldenHex(uint64_t);
std::string RigExecOracleGoldenVisit(const std::string &, size_t, const RigExecRigPose &,
    const std::vector<RigExecGoldenValue> &, bool);
bool RigExecOracleCompareGolden(const std::string &, const std::string &, std::string *);

namespace {
const RigExecOracleHooks hooks{
    &RigExecOracleBeginBody,
    &RigExecOracleRunBody,
    &RigExecCaptureOracleInputs,
    &RigExecCaptureWeightReference,
    &RigExecOracleCreateGolden,
    &RigExecOracleFinalizeGolden,
    &RigExecOracleEncodeGoldenValue,
    &RigExecOracleEncodeGoldenPose,
    &RigExecOracleGoldenDouble,
    &RigExecOracleGoldenEscape,
    &RigExecOracleGoldenDigest,
    &RigExecOracleGoldenHex,
    &RigExecOracleGoldenVisit,
    &RigExecOracleCompareGolden
};
}

void RigExecRegisterOracle() { RigExecInstallOracleHooks(&hooks); }
}
