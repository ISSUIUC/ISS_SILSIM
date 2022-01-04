#include"ChRt.h"
#include"../../../../src/CpuStateContext.h"

CpuStateContext context;
//context.system_time


void chMtxLock(mutex_t * mtx){
    mtx->lock();
}

void chMtxUnlock(mutex_t * mtx){
    mtx->unlock();
}

uint32_t chVTGetSystemTime(){
    return context.system_time / 1000;
}
