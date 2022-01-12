/**
 * @file        PhysicsEngine.h
 * @authors     Ayberk Yaraneri
 *
 * @brief       PhysicsEngine class definition
 *
 * The PhysicsEngine class encapsulates a particular algorithm that is used
 * to advance the state of the simulation in time. These algorithms are
 * commonly responsible for calculating time varying quantities that govern
 * the trajectory of the rocket, such as angular and axial acceleration.
 *
 */

#ifndef _PHYSICS_ENGINE_H_
#define _PHYSICS_ENGINE_H_

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <string>
#include <vector>

#include "Propulsion.h"
#include "Rocket.h"
#include "quaternion.h"

class PhysicsEngine {
   public:
    PhysicsEngine(Rocket& rocket, SolidMotor& motor)
        : rocket_(rocket), motor_(motor){};

    virtual void march_step(double tStamp, double tStep) = 0;

   protected:
    Rocket& rocket_;
    SolidMotor& motor_;
};

class ForwardEuler : public PhysicsEngine {
   public:
    ForwardEuler(Rocket& rocket, SolidMotor& motor);

    void march_step(double tStamp, double tStep) override;

   private:
    Quaternion<double> update_quaternion(Quaternion<double> q_ornt,
                                         Vector3 omega_if, double tStep);

    std::shared_ptr<spdlog::logger> euler_logger;
};

#endif
