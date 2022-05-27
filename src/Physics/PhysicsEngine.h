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
 * the trajectory of the rocket, such as angular and axial acceleration,
 * velocity and position.
 *
 */

#ifndef _PHYSICS_ENGINE_H_
#define _PHYSICS_ENGINE_H_
#define _USE_MATH_DEFINES

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <string>
#include <vector>

#include "Propulsion.h"
#include "Rocket.h"

// Shortening the typename for   a e s t h e t i c s
typedef std::shared_ptr<spdlog::sinks::basic_file_sink_mt>
    spdlog_basic_sink_ptr;

/*****************************************************************************/
/* PhysicsEngine Base Class and Derivatives */
/*****************************************************************************/

/** PhysicsEngine Base Class
 * This class is a pure virtual class that defines an interface that physics
 * engine implementations need to abide by. i.e. all implementations that derive
 * from this class will need to have a "march_step()" function that marches the
 * simulation physics forward by one step. The algorithm used to do so depends
 * on the particular implementation
 *
 * This class is also the home for some commonly used utility funcitons. These
 * utility functions are in the protected segment of the class, which means
 * their implementations are shared with child classes that inherit from this
 * base class.
 */
class PhysicsEngine {
   public:
    PhysicsEngine(Rocket& rocket, RocketMotor& motor)
        : rocket_(rocket), motor_(motor){};

    virtual void march_step(double tStamp, double tStep) = 0;

   protected:
    std::pair<Vector3d, Vector3d> calc_forces_and_moments(double tStamp,
                                                          Vector3d pos_enu,
                                                          Vector3d vel_enu);

    Quaterniond update_quaternion(Quaterniond q_ornt, Vector3d omega_enu,
                                  double tStep) const;
    Rocket& rocket_;
    RocketMotor& motor_;

    std::shared_ptr<spdlog::logger> engine_logger_;
};

/** ForwardEuler Derived Class
 * This physics engine implents probably the simplest physics simulation
 * algorithm that exists. To move the simulation forward, it adds the product of
 * a values derivative and the simulation timestep to the value itself to move
 * it forward.
 *
 * Pretty straightforward.
 */
class ForwardEuler : public PhysicsEngine {
   public:
    ForwardEuler(Rocket& rocket, RocketMotor& motor,
                 spdlog_basic_sink_ptr silsim_sink)
        : PhysicsEngine(rocket, motor) {
        if (silsim_sink) {
            engine_logger_ =
                std::make_shared<spdlog::logger>("ForwardEuler", silsim_sink);
            engine_logger_->set_level(spdlog::level::debug);
        }
    };

    void march_step(double tStamp, double tStep) override;
};

/** ForwardEuler Derived Class
 * This physics engine implents the fourth order Runge Kutta algorithm.
 *
 * This means it calculates the rocket's state in *four* different locations in
 * space and time, and performs a specific average among these points to result
 * in a more accurate final rocket state.
 *
 * RK4 is certainly more computationally expensive in comparison to
 * ForwardEuler, but it does provide greater accuracy for the same simulation
 * timestep.
 *
 */
class RungeKutta : public PhysicsEngine {
   public:
    RungeKutta(Rocket& rocket, RocketMotor& motor,
               spdlog_basic_sink_ptr silsim_sink)
        : PhysicsEngine(rocket, motor) {
        if (silsim_sink) {
            engine_logger_ =
                std::make_shared<spdlog::logger>("RungeKutta", silsim_sink);
            engine_logger_->set_level(spdlog::level::debug);
        }
    };

    void march_step(double tStamp, double tStep) override;

   private:
    struct RungeKuttaState {
        Vector3d pos;
        Vector3d vel;
        Vector3d accel;
        Vector3d ang_vel;
        Vector3d ang_accel;
    };

    RungeKuttaState calc_state(double tStamp, double tStep, RungeKuttaState k);
};

#endif