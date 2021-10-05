//
// Created by 16182 on 10/5/2021.
//

#include "CpuState.h"

int main(){
    auto fsm_thread = std::make_unique<Rocket_FSM>();
    CpuState cpu;
    cpu.add_thread(std::move(fsm_thread));
    cpu.tick(100);
}