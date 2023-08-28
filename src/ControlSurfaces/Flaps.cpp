/**
 * @file        Flaps.cpp
 * @authors     Nicholas Phillips
 *
 * @brief       Member function implementations for Flap control surfaces
 *
 *
 */

#include "Flaps.h"

#include <algorithm>
#include <sstream>

void Flaps::write_extension(double extension) {
    if (extension > 1.0) {
        target_extension_ = 1.0;
    } else if (extension < 0.0) {
        target_extension_ = 0.0;
    } else {
        target_extension_ = extension;
    }
}

void Flaps::update(double dt) {
    if (target_extension_ > real_extension_) {
        real_extension_ = std::min(target_extension_,
                                   real_extension_ + max_movement_rate_ * dt);
    } else {
        real_extension_ = std::max(target_extension_,
                                   real_extension_ - max_movement_rate_ * dt);
    }
}
