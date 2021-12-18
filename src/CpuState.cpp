/**
 * @file        CpuState.cpp
 * @authors     Ayberk Yaraneri, Brooke Novosad, Nicholas Phillips
 *
 * @brief       Member function implementations of CpuState threads
 * classes
 *
 * The CpuState holds the tick function for the threads to run on. It also
 * contains the vector of different threads and its runtimes in absolute time
 * so we know when to run each thread and they are independent of each other.
 *
 */
#include "CpuState.h"

CpuState::CpuState() {}

/**
 * @brief Decides if any threads should run and updates the times of the threads
 *
 * @param timestamp Current time in absolute time
 */

void CpuState::tick(double timestamp) {
    CpuStateContext context{};
    for (auto& i : threads_) {
        CpuThread* thread = i.first;
        double time = i.second;
        // if not sleep time
        if (time <= timestamp) {
            context.timestamp = timestamp;
            i.second += thread->tick(context);
        }
    }
}

/**
 * @brief Adds a thread to the vector of threads, sets its runtime to 0
 *
 * @param thread The thread to be added
 */

void CpuState::add_thread(CpuThread* thread) {
    threads_.emplace_back(thread, 0);
}
