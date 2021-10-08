//
// Created by 16182 on 9/30/2021.
//

#include "CpuThread.h"

CpuThread::CpuThread(void (*FSW_function)()) {
    FSW_function_ = FSW_function;
}

double CpuThread::tick(CpuStateContext const& context) {
    return real_tick_(context);
}

double CpuThread::real_tick_(CpuStateContext const& context) {
    (*FSW_function_)();

    return context.timestamp + 0.5f;
}
