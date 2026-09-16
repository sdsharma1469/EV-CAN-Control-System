#pragma once

#include <cstdint>
#include <optional>

#include "can_messages.hpp"

namespace evcan {

struct ControllerConfig {
    double max_torque_nm{320.0};
    double accel_brake_fault_threshold_pct{20.0};
    double slip_derate_start{0.08};
    double slip_cutoff{0.25};
    std::uint64_t pedal_timeout_ms{100};
    std::uint64_t wheel_speed_timeout_ms{100};
};

class VehicleController {
public:
    explicit VehicleController(ControllerConfig config = {});

    void on_frame(const CanFrame& frame);
    TorqueCommand update(std::uint64_t now_ms);
    CanFrame make_output_frame(std::uint64_t now_ms) const;

    void reset_faults();

    bool pedal_counter_fault() const { return pedal_counter_fault_; }
    bool pedal_plausibility_fault() const { return pedal_plausibility_fault_; }
    bool input_timeout_fault() const { return input_timeout_fault_; }
    const TorqueCommand& last_command() const { return last_command_; }

private:
    ControllerConfig config_;
    std::optional<PedalStatus> pedal_;
    std::optional<WheelSpeeds> wheels_;
    std::optional<std::uint64_t> last_pedal_ms_;
    std::optional<std::uint64_t> last_wheels_ms_;
    std::optional<std::uint8_t> last_pedal_counter_;

    bool pedal_counter_fault_{false};
    bool pedal_plausibility_fault_{false};
    bool input_timeout_fault_{false};
    TorqueCommand last_command_{};

    double calculate_slip_ratio() const;
};

}  // namespace evcan
