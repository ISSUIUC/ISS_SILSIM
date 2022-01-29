#include<CpuThread.h>

extern CpuStateContext* global_context;

double CpuThread::tick() {
    assert(global_context != nullptr);
    // convert sleep time to seconds
    return loop() / 1000.0;
}
