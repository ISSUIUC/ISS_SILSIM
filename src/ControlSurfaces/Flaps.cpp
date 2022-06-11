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
    target_extension_ = std::clamp(extension, 0.0, 1.0);
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

void Flaps::log_flap_state(double tStamp) {
    if (flaps_logger_) {
        // clang-format off
        std::stringstream datalog_ss;

        datalog_ss << "DATA,"
                   << tStamp << ","
                   << target_extension_ << ","
                   << real_extension_; 

        flaps_logger_->info(datalog_ss.str());
        // clang-format on
    }
}
