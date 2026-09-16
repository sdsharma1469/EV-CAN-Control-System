#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

namespace evcan {

constexpr std::uint32_t PEDAL_STATUS_ID = 0x100;
constexpr std::uint32_t WHEEL_SPEED_ID = 0x110;
constexpr std::uint32_t TORQUE_COMMAND_ID = 0x300;

struct CanFrame {
    std::uint32_t id{0};
    std::array<std::uint8_t, 8> data{};
    std::uint8_t dlc{8};
    std::uint64_t timestamp_ms{0};
};

enum class ControllerState : std::uint8_t {
    Ready = 0,
    Derate = 1,
    Fault = 2,
};

struct PedalStatus {
    double accelerator_pct{0.0};
    double brake_pct{0.0};
    std::uint8_t counter{0};
};

struct WheelSpeeds {
    double front_left_kph{0.0};
    double front_right_kph{0.0};
    double rear_left_kph{0.0};
    double rear_right_kph{0.0};
};

struct TorqueCommand {
    double torque_nm{0.0};
    ControllerState state{ControllerState::Fault};
    double slip_ratio{0.0};
};

inline void write_u16(std::array<std::uint8_t, 8>& data, std::size_t offset,
                      std::uint16_t value) {
    data.at(offset) = static_cast<std::uint8_t>(value & 0xFFU);
    data.at(offset + 1) = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

inline std::uint16_t read_u16(const std::array<std::uint8_t, 8>& data,
                              std::size_t offset) {
    return static_cast<std::uint16_t>(data.at(offset)) |
           (static_cast<std::uint16_t>(data.at(offset + 1)) << 8U);
}

inline void write_i16(std::array<std::uint8_t, 8>& data, std::size_t offset,
                      std::int16_t value) {
    write_u16(data, offset, static_cast<std::uint16_t>(value));
}

inline std::int16_t read_i16(const std::array<std::uint8_t, 8>& data,
                             std::size_t offset) {
    return static_cast<std::int16_t>(read_u16(data, offset));
}

inline CanFrame encode_pedal_status(const PedalStatus& signal,
                                    std::uint64_t timestamp_ms) {
    CanFrame frame;
    frame.id = PEDAL_STATUS_ID;
    frame.timestamp_ms = timestamp_ms;
    frame.dlc = 5;

    const auto accel = static_cast<std::uint16_t>(std::clamp(signal.accelerator_pct, 0.0, 100.0) * 10.0 + 0.5);
    const auto brake = static_cast<std::uint16_t>(std::clamp(signal.brake_pct, 0.0, 100.0) * 10.0 + 0.5);
    write_u16(frame.data, 0, accel);
    write_u16(frame.data, 2, brake);
    frame.data[4] = static_cast<std::uint8_t>(signal.counter & 0x0FU);
    return frame;
}

inline PedalStatus decode_pedal_status(const CanFrame& frame) {
    if (frame.id != PEDAL_STATUS_ID || frame.dlc < 5) {
        throw std::invalid_argument("invalid pedal status CAN frame");
    }
    return PedalStatus{
        read_u16(frame.data, 0) / 10.0,
        read_u16(frame.data, 2) / 10.0,
        static_cast<std::uint8_t>(frame.data[4] & 0x0FU),
    };
}

inline CanFrame encode_wheel_speeds(const WheelSpeeds& signal,
                                    std::uint64_t timestamp_ms) {
    CanFrame frame;
    frame.id = WHEEL_SPEED_ID;
    frame.timestamp_ms = timestamp_ms;
    frame.dlc = 8;

    const auto scaled = [](double kph) {
        return static_cast<std::uint16_t>(std::clamp(kph, 0.0, 6553.5) * 10.0 + 0.5);
    };

    write_u16(frame.data, 0, scaled(signal.front_left_kph));
    write_u16(frame.data, 2, scaled(signal.front_right_kph));
    write_u16(frame.data, 4, scaled(signal.rear_left_kph));
    write_u16(frame.data, 6, scaled(signal.rear_right_kph));
    return frame;
}

inline WheelSpeeds decode_wheel_speeds(const CanFrame& frame) {
    if (frame.id != WHEEL_SPEED_ID || frame.dlc < 8) {
        throw std::invalid_argument("invalid wheel-speed CAN frame");
    }
    return WheelSpeeds{
        read_u16(frame.data, 0) / 10.0,
        read_u16(frame.data, 2) / 10.0,
        read_u16(frame.data, 4) / 10.0,
        read_u16(frame.data, 6) / 10.0,
    };
}

inline CanFrame encode_torque_command(const TorqueCommand& command,
                                      std::uint64_t timestamp_ms) {
    CanFrame frame;
    frame.id = TORQUE_COMMAND_ID;
    frame.timestamp_ms = timestamp_ms;
    frame.dlc = 5;

    const auto torque = static_cast<std::int16_t>(std::clamp(command.torque_nm, -3276.8, 3276.7) * 10.0);
    const auto slip = static_cast<std::int16_t>(std::clamp(command.slip_ratio, -32.768, 32.767) * 1000.0);
    write_i16(frame.data, 0, torque);
    frame.data[2] = static_cast<std::uint8_t>(command.state);
    write_i16(frame.data, 3, slip);
    return frame;
}

inline TorqueCommand decode_torque_command(const CanFrame& frame) {
    if (frame.id != TORQUE_COMMAND_ID || frame.dlc < 5) {
        throw std::invalid_argument("invalid torque-command CAN frame");
    }
    return TorqueCommand{
        read_i16(frame.data, 0) / 10.0,
        static_cast<ControllerState>(frame.data[2]),
        read_i16(frame.data, 3) / 1000.0,
    };
}

inline const char* state_name(ControllerState state) {
    switch (state) {
        case ControllerState::Ready: return "READY";
        case ControllerState::Derate: return "DERATE";
        case ControllerState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

}  // namespace evcan
