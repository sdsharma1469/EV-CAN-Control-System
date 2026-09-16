#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "can_messages.hpp"
#include "vehicle_controller.hpp"

namespace {

using evcan::ControllerState;
using evcan::PedalStatus;
using evcan::VehicleController;
using evcan::WheelSpeeds;

void send_nominal_inputs(VehicleController& controller, std::uint64_t t,
                         std::uint8_t counter, double accel = 50.0,
                         double front_kph = 30.0, double rear_kph = 30.2) {
    controller.on_frame(evcan::encode_pedal_status(PedalStatus{accel, 0.0, counter}, t));
    controller.on_frame(evcan::encode_wheel_speeds(
        WheelSpeeds{front_kph, front_kph, rear_kph, rear_kph}, t));
}

void test_nominal_torque() {
    VehicleController controller;
    send_nominal_inputs(controller, 0, 0, 50.0);
    const auto cmd = controller.update(0);

    assert(cmd.state == ControllerState::Ready);
    assert(std::abs(cmd.torque_nm - 160.0) < 1e-6);
}

void test_slip_derates_torque() {
    VehicleController controller;
    send_nominal_inputs(controller, 0, 0, 60.0, 30.0, 35.4);
    const auto cmd = controller.update(0);

    assert(cmd.state == ControllerState::Derate);
    assert(cmd.torque_nm > 0.0);
    assert(cmd.torque_nm < 192.0);
    assert(cmd.slip_ratio > 0.08);
}

void test_pedal_conflict_fault() {
    VehicleController controller;
    controller.on_frame(evcan::encode_pedal_status(PedalStatus{45.0, 35.0, 0}, 0));
    controller.on_frame(evcan::encode_wheel_speeds(WheelSpeeds{20.0, 20.0, 20.0, 20.0}, 0));
    const auto cmd = controller.update(0);

    assert(cmd.state == ControllerState::Fault);
    assert(cmd.torque_nm == 0.0);
    assert(controller.pedal_plausibility_fault());
}

void test_timeout_fault_and_recovery() {
    VehicleController controller;
    send_nominal_inputs(controller, 0, 0);
    assert(controller.update(0).state == ControllerState::Ready);

    assert(controller.update(101).state == ControllerState::Fault);
    assert(controller.input_timeout_fault());

    send_nominal_inputs(controller, 110, 1);
    assert(controller.update(110).state == ControllerState::Ready);
}

void test_counter_fault_latches() {
    VehicleController controller;
    send_nominal_inputs(controller, 0, 0);
    assert(controller.update(0).state == ControllerState::Ready);

    send_nominal_inputs(controller, 10, 0);
    const auto cmd = controller.update(10);

    assert(cmd.state == ControllerState::Fault);
    assert(controller.pedal_counter_fault());
}

void test_serialization_round_trip() {
    const PedalStatus pedal{62.3, 4.7, 12};
    const auto pedal_frame = evcan::encode_pedal_status(pedal, 123);
    const auto decoded_pedal = evcan::decode_pedal_status(pedal_frame);
    assert(std::abs(decoded_pedal.accelerator_pct - 62.3) < 1e-6);
    assert(std::abs(decoded_pedal.brake_pct - 4.7) < 1e-6);
    assert(decoded_pedal.counter == 12);

    const WheelSpeeds wheels{10.1, 10.2, 10.3, 10.4};
    const auto wheel_frame = evcan::encode_wheel_speeds(wheels, 456);
    const auto decoded_wheels = evcan::decode_wheel_speeds(wheel_frame);
    assert(std::abs(decoded_wheels.rear_right_kph - 10.4) < 1e-6);
}

}  // namespace

int main() {
    test_nominal_torque();
    test_slip_derates_torque();
    test_pedal_conflict_fault();
    test_timeout_fault_and_recovery();
    test_counter_fault_latches();
    test_serialization_round_trip();

    std::cout << "All EV CAN controller tests passed.\n";
    return 0;
}
