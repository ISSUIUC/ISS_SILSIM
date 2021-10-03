//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_CHRT_H
#define SILSIM_CHRT_H

#include <atomic>
#include <mutex>
#include <cstring>

#define SEMAPHORE_DECL(name, n) std::atomic<size_t> name{n};
#define MUTEX_DECL(name) std::mutex name;


typedef std::atomic<size_t> semaphore_t;
typedef std::mutex mutex_t;
typedef uint32_t sysinterval_t;
typedef uint32_t msg_t;
typedef uint32_t cnt_t;

#define __itoa _itoa
#define TIME_IMMEDIATE (sysinterval_t)0
#define MSG_OK (msg_t)0

struct {
    static void println(const char *){

    }
} Serial;

uint32_t chVTGetSystemTime(){
    return 0;
}

void chMtxLock(mutex_t * mtx){
    mtx->lock();
}

void chMtxUnlock(mutex_t * mtx){
    mtx->unlock();
}

void chSysLock();

void chSysUnlock();

void chSemObjectInit(semaphore_t *sp, cnt_t n);
void chSemReset(semaphore_t *sp, cnt_t n);
void chSemResetI(semaphore_t *sp, cnt_t n);
msg_t chSemWait(semaphore_t *sp);
msg_t chSemWaitS(semaphore_t *sp);
msg_t chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout);
msg_t chSemWaitTimeoutS(semaphore_t *sp, sysinterval_t timeout);
void chSemSignal(semaphore_t *sp);
void chSemSignalI(semaphore_t *sp);
void chSemAddCounterI(semaphore_t *sp, cnt_t n);
msg_t chSemSignalWait(semaphore_t *sps, semaphore_t *spw);

#endif  // SILSIM_CHRT_H
