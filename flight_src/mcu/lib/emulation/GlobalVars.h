//
// Created by 16182 on 1/28/2022.
//

#ifndef SILSIM_GLOBALVARS_H
#define SILSIM_GLOBALVARS_H

#include<CpuStateContext.h>

extern CpuStateContext* global_context;

#define Serial (global_context->Serial)
#define Serial1 (global_context->Serial1)
#define mpu_data (global_context->mpu_data)
#define sensor_pointers (global_context->sensor_pointers)
#define servo_cw (global_context->servo_cw)
#define servo_ccw (global_context->servo_ccw)
#define lowGimu (global_context->lowGimu)
#define highGimu (global_context->highGimu)
#define gps (global_context->gps)
#define sensorData (global_context->sensorData)
#define barometer (global_context->barometer)

#endif  // SILSIM_GLOBALVARS_H
