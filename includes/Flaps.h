//
// Created by 16182 on 5/2/2022.
//

#ifndef SILSIM_FLAPS_H
#define SILSIM_FLAPS_H

#include<algorithm>

class Flaps {
   public:
    void write_extension(double extension){
        target_extension = std::clamp(extension, 0.0, 1.0);
    }

    void update(double dt){
        if(target_extension > real_extension){
            real_extension = std::min(target_extension, real_extension + max_movement_rate * dt);
        } else {
            real_extension = std::max(target_extension, real_extension - max_movement_rate * dt);
        }
    }

    double extension() const {
        return real_extension;
    }

   private:
    double target_extension = 0;
    double real_extension = 0;

    // 0% to 100% extension in 0.5 seconds
    static constexpr double max_movement_rate = 0.5;
};

#endif  // SILSIM_FLAPS_H
