# EV CAN Control System

A small software-defined-vehicle project that models a CAN-based electric-vehicle control network in C++ and provides Python tooling for fault-injection experiments.

The repository contains three simulated ECUs:

- **Pedal ECU** publishes accelerator/brake position and a rolling counter.
- **Dynamics ECU** publishes four wheel speeds.
- **Vehicle Control Unit (VCU)** validates incoming messages, checks pedal plausibility, estimates wheel slip, applies traction limiting, and publishes a torque command plus controller state.

The project is intentionally hardware-independent: the default executable uses an in-process simulated CAN bus so the control logic can be compiled and tested on any machine. The message definitions and controller interfaces are kept separate so the same logic can later be connected to SocketCAN or embedded CAN drivers.

## Highlights

- C++17 controller and simulated ECU implementation
- Explicit CAN identifiers and byte-level serialization
- Accelerator/brake plausibility checking
- Message-counter and timeout monitoring
- Simple traction-control torque limiting from wheel-speed slip
- Controller states: `READY`, `DERATE`, and `FAULT`
- Python fault-injection runner for automated scenario testing
- Unit tests for nominal operation, plausibility faults, stale messages, and traction limiting
- DBC file documenting the project CAN signals

## Architecture

```text
 +------------+     CAN 0x100      +---------------------+
 | Pedal ECU  | -----------------> |                     |
 +------------+                    |                     |
                                   |        VCU          | ---- CAN 0x300 ----> Torque / state
 +------------+     CAN 0x110      |                     |
 | Dynamics   | -----------------> |                     |
 |    ECU     |                    +---------------------+
 +------------+
```

The default simulator advances in 10 ms steps. Driver inputs and wheel speeds are encoded into CAN frames, passed through the simulated bus, decoded by the VCU, and converted into a torque request. Fault scenarios can drop frames, freeze counters, or inject implausible pedal values.

## Build

Requirements:

- CMake 3.16+
- A C++17 compiler
- Python 3.9+ for the optional test runner

```bash
git clone https://github.com/sdsharma1469/EV-CAN-Control-System.git
cd EV-CAN-Control-System
cmake -S . -B build
cmake --build build
```

Run the simulator:

```bash
./build/ev_can_sim
```

Run C++ tests:

```bash
ctest --test-dir build --output-on-failure
```

Run automated fault scenarios:

```bash
python3 tools/fault_injection.py --binary ./build/ev_can_sim
```

## Example output

```text
time_ms,accel_pct,brake_pct,vehicle_kph,driven_kph,torque_nm,state
0,20.0,0.0,18.0,18.1,80.0,READY
...
1200,65.0,0.0,28.0,34.0,105.6,DERATE
...
2500,45.0,35.0,31.0,31.2,0.0,FAULT
```

`DERATE` indicates that the controller is still operational but torque has been reduced, for example because driven-wheel slip exceeded the configured threshold. `FAULT` commands zero torque when an input safety check fails.

## Controller behavior

The VCU performs four checks before issuing torque:

1. **Freshness** — pedal and wheel-speed messages must arrive within their watchdog windows.
2. **Counter continuity** — repeated or invalid rolling counters are rejected.
3. **Pedal plausibility** — simultaneous high accelerator and brake input forces the controller to `FAULT`.
4. **Traction limiting** — excessive driven-wheel speed relative to vehicle speed progressively reduces requested torque and reports `DERATE`.

This is an educational controller, not production automotive software. It does not implement ISO 26262 processes, a production CAN stack, or real vehicle calibration.

## Repository structure

```text
EV-CAN-Control-System/
├── CMakeLists.txt
├── include/
│   ├── can_messages.hpp
│   └── vehicle_controller.hpp
├── src/
│   ├── main.cpp
│   └── vehicle_controller.cpp
├── tests/
│   └── test_controller.cpp
├── tools/
│   └── fault_injection.py
├── dbc/
│   └── ev_control.dbc
└── docs/
    └── architecture.md
```

## Why this project exists

The goal is to practice the kinds of interfaces that appear in embedded and vehicle-controls software: deterministic message layouts, stateful validation, watchdogs, controller derating, fault handling, and automated validation. The project keeps the control algorithm independent of the transport layer so the same VCU logic can be reused with a real CAN backend later.

## Limitations / next steps

- Replace the in-process bus with Linux SocketCAN.
- Add a motor/vehicle plant instead of scripted wheel speeds.
- Add an RTOS-targeted hardware abstraction layer.
- Add signal-range and CRC checks in addition to rolling counters.
- Add SIL regression traces and hardware bench validation.

## License

MIT
