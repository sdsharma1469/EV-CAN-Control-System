#include "vehicle_controller.hpp"

#include <algorithm>
#include <cmath>

namespace evcan {

VehicleController::VehicleController(ControllerConfig config) : config_(config) {}

void VehicleController::on_frame(const CanFrame& frame) {
    if (frame.id == PEDAL_STATUS_ID) {
        const auto decoded = decode_pedal_status(frame);

        if (last_pedal_counter_.has_value()) {
            const auto expected = static_cast<std::uint8_t>((*last_pedal_counter_ + 1U) & 0x0FU);
            if (decoded.counter != expected) {
                pedal_counter_fault_ = true;
            }
        }

        last_pedal_counter_ = decoded.counter;
        pedal_ = decoded;
        last_pedal_ms_ = frame.timestamp_ms;

        if (decoded.accelerator_pct >= config_.accel_brake_fault_threshold_pct &&
            decoded.brake_pct >= config_.accel_brake_fault_threshold_pct) {
            pedal_plausibility_fault_ = true;
        }
        return;
    }

    if (frame.id == WHEEL_SPEED_ID) {
        wheels_ = decode_wheel_speeds(frame);
        last_wheels_ms_ = frame.timestamp_ms;
    }
}

double VehicleController::calculate_slip_ratio() const {
    if (!wheels_.has_value()) {
        return 0.0;
    }

    const double reference_kph =
        (wheels_->front_left_kph + wheels_->front_right_kph) * 0.5;
    const double driven_kph =
        (wheels_->rear_left_kph + wheels_->rear_right_kph) * 0.5;

    const double denominator = std::max(reference_kph, 5.0);
    return (driven_kph - reference_kph) / denominator;
}

TorqueCommand VehicleController::update(std::uint64_t now_ms) {
    const bool pedal_stale = !last_pedal_ms_.has_value() ||
        now_ms > *last_pedal_ms_ + config_.pedal_timeout_ms;
    const bool wheels_stale = !last_wheels_ms_.has_value() ||
        now_ms > *last_wheels_ms_ + config_.wheel_speed_timeout_ms;
    input_timeout_fault_ = pedal_stale || wheels_stale;

    const double slip = calculate_slip_ratio();

    if (pedal_counter_fault_ || pedal_plausibility_fault_ || input_timeout_fault_ ||
        !pedal_.has_value() || !wheels_.has_value()) {
        last_command_ = TorqueCommand{0.0, ControllerState::Fault, slip};
        return last_command_;
    }

    // A brake request overrides positive drive torque even below the pedal-conflict
    // threshold. The higher threshold above is reserved for a plausibility fault.
    if (pedal_->brake_pct > 2.0) {
        last_command_ = TorqueCommand{0.0, ControllerState::Ready, slip};
        return last_command_;
    }

    double torque = (pedal_->accelerator_pct / 100.0) * config_.max_torque_nm;
    ControllerState state = ControllerState::Ready;

    if (slip > config_.slip_derate_start) {
        const double span = config_.slip_cutoff - config_.slip_derate_start;
        const double reduction = span > 0.0
            ? std::clamp((slip - config_.slip_derate_start) / span, 0.0, 1.0)
            : 1.0;
        torque *= (1.0 - reduction);
        state = ControllerState::Derate;
    }

    last_command_ = TorqueCommand{std::max(0.0, torque), state, slip};
    return last_command_;
}

CanFrame VehicleController::make_output_frame(std::uint64_t now_ms) const {
    return encode_torque_command(last_command_, now_ms);
}

void VehicleController::reset_faults() {
    pedal_counter_fault_ = false;
    pedal_plausibility_fault_ = false;
    input_timeout_fault_ = false;
    last_pedal_counter_.reset();
}

}  // namespace evcan
