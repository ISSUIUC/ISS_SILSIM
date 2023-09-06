/**
 * @file        Flaps.h
 * @authors     Nicholas Phillips
 *
 * @brief       Class definition for Flap control surfaces
 *
 *
 */

#ifndef SILSIM_FLAPS_H
#define SILSIM_FLAPS_H

class Flaps {
   public:
    Flaps() {}

    void write_extension(double extension);
    void update(double dt);
    void set_movement_rate(double rate) { max_movement_rate_ = rate; };
    double extension() const { return real_extension_; }

   private:
    double target_extension_ = 0;
    double real_extension_ = 0;

    // Duration for 0% to 100% extension in seconds
    double max_movement_rate_ = 0.5;
};

#endif  // SILSIM_FLAPS_H
