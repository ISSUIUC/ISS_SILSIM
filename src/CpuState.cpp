//
// Created by 16182 on 9/30/2021.
//

#include "CpuState.h"

CpuState::CpuState() {

}

void CpuState::tick(double timestamp) {
    CpuStateContext context;
    for(unsigned i = 0; i < threads_.size(); i++){
        CpuThread thread = threads_.at(i).first; // why is this not liking it?
        double time = threads_.at(i).second;
        //if not sleep time
        if (time == timestamp) {
            context.timestamp = timestamp;
            thread.tick(context);
        }
            //run the thread
            //thread.run(); ??
    }
}
void CpuState::add_thread(std::unique_ptr<CpuThread> thread) {
    threads_.emplace_back(std::move(thread), 0);
}

