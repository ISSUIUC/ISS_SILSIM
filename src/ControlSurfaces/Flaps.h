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

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

// Shortening the typename for   a e s t h e t i c s
typedef std::shared_ptr<spdlog::sinks::basic_file_sink_mt>
    spdlog_basic_sink_ptr;

class Flaps {
   public:
    Flaps(spdlog_basic_sink_ptr silsim_sink) {
        if (silsim_sink) {
            flaps_logger_ =
                std::make_shared<spdlog::logger>("Flaps", silsim_sink);
            flaps_logger_->info("DATALOG_FORMAT," + datalog_format_string);
        }
    }

    void write_extension(double extension);
    void update(double dt);
    void set_movement_rate(double rate) { max_movement_rate_ = rate; };
    double extension() const { return real_extension_; }

    void log_flap_state(double tStamp);

   private:
    double target_extension_ = 0;
    double real_extension_ = 0;

    // Duration for 0% to 100% extension in seconds
    double max_movement_rate_ = 0.5;

    std::shared_ptr<spdlog::logger> flaps_logger_;
    std::string datalog_format_string =
        "timestmap,target_extension,real_extension";
};

#endif  // SILSIM_FLAPS_H
