#include<CpuThread.h>

extern CpuStateContext* global_context;

double CpuThread::tick() {
    assert(global_context != nullptr);
    // some checks
    return loop();
}
