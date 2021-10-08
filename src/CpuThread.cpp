//
// Created by 16182 on 9/30/2021.
//

#include "CpuThread.h"

CpuThread::CpuThread((*FSW_function)()) {
    FSW_function_ = FSW_function;
}
CpuThread::tick(CpuStateContext const& context) {
    return real_tick(context);
}
double CpuThread::real_tick_(CpuStateContext const& context) {
    (*FSW_function_)();
    return 0;
}
