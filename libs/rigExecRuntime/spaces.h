#ifndef RIGEXEC_RUNTIME_SPACES_H
#define RIGEXEC_RUNTIME_SPACES_H
#include "store.h"
namespace rigExec {
bool RrReadCrossDomain(const RrProgram *, int32_t, RrWireValue *);
const std::vector<RrVec3f> *
RrResolveDeclaredPoints(const RrProgram *, const RigExecWirePointsBinding &);
const std::vector<RrVec3f> *
RrReadCrossDomainPoints(const RrProgram *,
                        const fb::RigExecWireCrossDomainRead &);
bool RrReadCrossDomainPoint(const RrProgram *,
                            const fb::RigExecWireCrossDomainRead &, RrVec3f *);
bool RrOpenProviderProgram(RrProgram *, std::string *);
/// The effective-input key of the op node that answers row \p object of
/// provider value producer \p part (eval::spaces::ProviderProducer), or
/// that runs provider op \p object (part 0).
bool RrProviderEffectiveInputMemo(const RrProgram *, int part, int64_t object,
                                  std::string *, std::vector<uint32_t> *);
RrWireValue RrReadProviderRefreshBlocker(const RrProgram *, int);
} // namespace rigExec
#endif
