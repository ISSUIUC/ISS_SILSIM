/**
 * @file        PhysicsEngine.h
 * @authors     Ayberk Yaraneri
 *              Jacob Gugala
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

class PhysicsEngine {
   public:
    PhysicsEngine(Rocket& rocket, SolidMotor& motor)
        : rocket_(rocket), motor_(motor){};

    virtual void march_step(double tStamp, double tStep) = 0;

   protected:
    Quaterniond update_quaternion(Quaterniond q_ornt, Vector3d omega_if,
                                  double tStep) const;
    Rocket& rocket_;
    SolidMotor& motor_;
};

class ForwardEuler : public PhysicsEngine {
   public:
    ForwardEuler(Rocket& rocket, SolidMotor& motor);

    void march_step(double tStamp, double tStep) override;

   private:
    std::shared_ptr<spdlog::logger> euler_logger;
};

class RungeKutta : public PhysicsEngine {
   public:
    RungeKutta(Rocket& rocket, SolidMotor& motor)
        : PhysicsEngine(rocket, motor){};

    void march_step(double tStamp, double tStep) override;

   private:
    struct RungeKuttaState {
        Vector3d pos;
        Vector3d vel;
        Vector3d accel;
        Vector3d ang_vel;
        Vector3d ang_accel;
    };

    Vector3d calc_net_force(double tStamp, const Rocket& rocket);
    Vector3d calc_net_torque(double tStamp, const Rocket& rocket);
    RungeKuttaState calc_state(double tStamp, double tStep, RungeKuttaState k);
};

#endif
