//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_CHRT_H
#define SILSIM_CHRT_H

#include <atomic>
#include <mutex>
#include <cstring>
#include <cmath>

#define SEMAPHORE_DECL(name, n) std::atomic<size_t> name{n};
#define MUTEX_DECL(name) std::mutex name;


typedef std::atomic<size_t> semaphore_t;
typedef std::mutex mutex_t;
typedef uint32_t sysinterval_t;
typedef uint32_t msg_t;
typedef uint32_t cnt_t;
typedef uint32_t systime_t;

#define CH_CFG_ST_FREQUENCY 100000
#define TIME_I2MS(x) (((systime_t)(x) * (systime_t)(1000) + CH_CFG_ST_FREQUENCY - 1) / CH_CFG_ST_FREQUENCY)

#define __itoa _itoa
#define TIME_IMMEDIATE (sysinterval_t)0
#define MSG_OK (msg_t)0

struct SerialClass{
    static void println(const char *);
};

extern SerialClass Serial;

uint32_t chVTGetSystemTime();

void chMtxLock(mutex_t * mtx);

void chMtxUnlock(mutex_t * mtx);

void chSysLock();

void chSysUnlock();

#endif  // SILSIM_CHRT_H
