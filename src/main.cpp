#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "can_messages.hpp"
#include "vehicle_controller.hpp"

namespace {

using evcan::CanFrame;
using evcan::PedalStatus;
using evcan::VehicleController;
using evcan::WheelSpeeds;

class SimulatedCanBus {
public:
    explicit SimulatedCanBus(VehicleController& controller) : controller_(controller) {}

    void publish(const CanFrame& frame) {
        controller_.on_frame(frame);
    }

private:
    VehicleController& controller_;
};

struct ScenarioInputs {
    double accelerator_pct{0.0};
    double brake_pct{0.0};
    double front_kph{0.0};
    double rear_kph{0.0};
    bool publish_pedal{true};
    bool freeze_counter{false};
};

ScenarioInputs make_inputs(const std::string& scenario, std::uint64_t time_ms) {
    const double t = static_cast<double>(time_ms) / 1000.0;
    const double base_speed = 12.0 + std::min(t * 8.0, 28.0);
    const double accel = time_ms < 800 ? 20.0 : (time_ms < 2200 ? 55.0 : 35.0);

    ScenarioInputs inputs;
    inputs.accelerator_pct = accel;
    inputs.front_kph = base_speed;
    inputs.rear_kph = base_speed + 0.2;

    if (scenario == "slip" && time_ms >= 1200 && time_ms < 1800) {
        inputs.rear_kph = base_speed * 1.18;
    } else if (scenario == "pedal-conflict" && time_ms >= 1800) {
        inputs.accelerator_pct = 45.0;
        inputs.brake_pct = 35.0;
    } else if (scenario == "pedal-timeout" && time_ms >= 1200 && time_ms < 1400) {
        inputs.publish_pedal = false;
    } else if (scenario == "counter-freeze" && time_ms >= 1500 && time_ms < 1600) {
        inputs.freeze_counter = true;
    }

    return inputs;
}

bool valid_scenario(const std::string& scenario) {
    return scenario == "nominal" || scenario == "slip" ||
           scenario == "pedal-conflict" || scenario == "pedal-timeout" ||
           scenario == "counter-freeze";
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenario = "nominal";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: ev_can_sim [--scenario nominal|slip|pedal-conflict|pedal-timeout|counter-freeze]\n";
            return EXIT_SUCCESS;
        }
    }

    if (!valid_scenario(scenario)) {
        std::cerr << "Unknown scenario: " << scenario << '\n';
        return EXIT_FAILURE;
    }

    VehicleController controller;
    SimulatedCanBus bus(controller);
    std::uint8_t pedal_counter = 0;
    std::uint8_t last_transmitted_counter = 0;

    std::cout << "time_ms,accel_pct,brake_pct,vehicle_kph,driven_kph,torque_nm,state,slip_ratio\n";
    std::cout << std::fixed << std::setprecision(3);

    for (std::uint64_t time_ms = 0; time_ms <= 3000; time_ms += 10) {
        const auto inputs = make_inputs(scenario, time_ms);

        if (inputs.publish_pedal) {
            std::uint8_t tx_counter = pedal_counter;
            if (inputs.freeze_counter) {
                tx_counter = last_transmitted_counter;
            } else {
                last_transmitted_counter = tx_counter;
                pedal_counter = static_cast<std::uint8_t>((pedal_counter + 1U) & 0x0FU);
            }

            bus.publish(evcan::encode_pedal_status(
                PedalStatus{inputs.accelerator_pct, inputs.brake_pct, tx_counter},
                time_ms));
        }

        bus.publish(evcan::encode_wheel_speeds(
            WheelSpeeds{inputs.front_kph, inputs.front_kph,
                        inputs.rear_kph, inputs.rear_kph},
            time_ms));

        const auto command = controller.update(time_ms);
        const auto output_frame = controller.make_output_frame(time_ms);
        const auto decoded_output = evcan::decode_torque_command(output_frame);

        if (time_ms % 100 == 0) {
            std::cout << time_ms << ','
                      << inputs.accelerator_pct << ','
                      << inputs.brake_pct << ','
                      << inputs.front_kph << ','
                      << inputs.rear_kph << ','
                      << decoded_output.torque_nm << ','
                      << evcan::state_name(decoded_output.state) << ','
                      << decoded_output.slip_ratio << '\n';
        }
    }

    return EXIT_SUCCESS;
}
