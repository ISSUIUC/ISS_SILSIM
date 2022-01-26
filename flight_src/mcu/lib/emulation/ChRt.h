//
// Created by 16182 on 10/3/2021.
//

#ifndef SILSIM_CHRT_H
#define SILSIM_CHRT_H

#include <atomic>
#include <mutex>
#include <cstring>
#include <cmath>
#include <charconv>
#include <memory>

#define SEMAPHORE_DECL(name, n) std::atomic<size_t> name{n};
#define MUTEX_DECL(name) std::mutex name
#define THD_WORKING_AREA(name, size) char name[size]

typedef std::mutex mutex_t;
typedef uint32_t sysinterval_t;
typedef uint32_t msg_t;
typedef uint32_t cnt_t;
typedef uint32_t systime_t;
using std::min;
using std::max;

#define CH_CFG_ST_FREQUENCY 100000
#define TIME_I2MS(x) (((systime_t)(x) * (systime_t)(1000) + CH_CFG_ST_FREQUENCY - 1) / CH_CFG_ST_FREQUENCY)
#define chThdCreateStatic(working_area, sizeof_working_area, priority, thread_name, sensor_pointers)

inline char * itoa(int x, char * buffer, int radix){
    std::to_chars(buffer, buffer + 16, x, radix);
}

#define __itoa itoa
#define TIME_IMMEDIATE (sysinterval_t)0
#define MSG_OK (msg_t)0
#define NORMALPRIO ((unsigned char)128)

struct SerialClass{
    static void println(const char *);
    static void begin(uint32_t frequency);
    static void write(void * data, uint32_t size);
};

extern SerialClass Serial;
extern SerialClass Serial1;

uint32_t chVTGetSystemTime();

void chMtxLock(mutex_t * mtx);

void chMtxUnlock(mutex_t * mtx);

void chSysLock();

void chSysUnlock();

void chThdSleepMilliseconds(uint32_t ms);

void chBegin(void (*mainThread)());

#endif  // SILSIM_CHRT_H
