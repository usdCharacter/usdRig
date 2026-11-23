// A consumer of the installed rigExecPlayback library: the background
// scheduler links from its own library, and the cancellation token it hands
// out follows the rig's generation.
#include "rigExecPlayback/backgroundScheduler.h"

#include <cstdio>

using namespace rigExec;

int
main()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    const SdfPath rig("/Asset/Rig");
    const RigExecCancellationToken token = scheduler.CancellationToken(rig, 0);
    if (token.IsCanceled()) {
        std::printf("FAIL a token for the current generation is canceled\n");
        return 1;
    }
    scheduler.CancelGeneration(rig);
    if (!token.IsCanceled() || scheduler.CurrentGeneration(rig) != 1) {
        std::printf("FAIL canceling the generation left the token current\n");
        return 1;
    }
    std::printf("testRigExecInstalledPlayback: all tests passed\n");
    return 0;
}
