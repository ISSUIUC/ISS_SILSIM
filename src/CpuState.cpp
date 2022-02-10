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
#include <algorithm>

/**
 * @brief Decides if any threads should run and updates the times of the threads
 *
 * @param timestamp Current time in absolute time
 */

/*
 * start function for the emulated flight code
 * it calls add_thread to add emulated threads to the emulated cpu
 */
void emu_setup();

extern CpuStateContext* global_context;

void CpuState::tick(double timestamp) {
    global_context = &context;
    if(!has_started){
        has_started = true;
        emu_setup();
    }
    if(threads_.empty()) return;
    context.system_time = timestamp;
    while(true){
        //get the first scheduled thread to run
        auto min_thread = std::min_element(threads_.begin(), threads_.end(),
                                           [](const auto & a, const auto & b){
                                               return a.second < b.second;
                                           }
                                           );

        if(min_thread->second > timestamp) break;
        min_thread->second += min_thread->first->tick();
    }
}

/**
 * @brief Adds a thread to the vector of threads, sets its runtime to 0
 *
 * @param thread The thread to be added
 */

void CpuState::add_thread(std::unique_ptr<CpuThread> thread) {
    threads_.emplace_back(std::move(thread), 0);
}
