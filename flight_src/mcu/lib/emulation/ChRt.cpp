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


msg_t chSemWait(semaphore_t *sp){

    while((*sp) == 0){

    }

    (*sp)--;

    return MSG_OK;
}


void chSemSignal(semaphore_t *sp){

    (*sp)++;

}

/*

signal
signal

wait
wait
wait
------

*/
