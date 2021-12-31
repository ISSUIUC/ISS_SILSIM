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
    ForwardEuler(Rocket& rocket, SolidMotor& motor)
        : PhysicsEngine(rocket, motor){};

    void march_step(double tStamp, double tStep) override;
};

class RungeKutta : public PhysicsEngine {
    public:
     RungeKutta(Rocket& rocket, SolidMotor& motor)
        : PhysicsEngine(rocket, motor){};

    void march_step(double tStamp, double tStep) override;
    Vector3 calc_accel(double tStamp, Vector3 vel_if);
    Vector3 calc_ang_accel(double tStamp, Vector3 vel_if, Vector3 ang_vel_if);
};

#endif
