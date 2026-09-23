# LaunchFSW-SIL

## Overview

LaunchFSW-SIL is a small C++20 launch-vehicle flight-sequence controller with a deterministic software-in-the-loop simulation.
It demonstrates state-machine design, caller-controlled time, sensor and command interfaces, fault handling, failure injection, automated testing, and Windows continuous integration.
It is an educational portfolio project rather than a complete or flight-ready rocket computer.

## Key Features

- Deterministic flight-state controller
- Fixed-tick SIL simulation
- Delay-based vehicle model
- Scheduled commands and injected failures
- Timeout and unexpected-state fault handling
- Recorded simulation timeline
- Selectable nominal and failure demonstrations
- Catch2 tests and Windows/MSVC CI

## Architecture

- `FlightCore`
	- Owns controller state, transitions, operator-command processing, timeouts, and faults.
	- Knows nothing about the simulator.
- `SIL`
	- Owns simulated time, scheduling, the vehicle model, execution ordering, recording, and termination.
- `App`
	- Defines demonstration scenarios, runs the production controller and simulator, and renders results.

### Controller Interface

`FlightComputer` consumes:

- An **operator command**: `ARM`, `LAUNCH`, or `NONE`
- **Sensor observations**: engine running, liftoff detected, and cutoff condition met
- A caller-supplied monotonic timestamp

It produces:

- The current flight state
- An **ignition output**, represented in code by `m_bIgnitionCommand`
- The operator-command processing result
- A latched fault reason

## Deterministic Timing Model

- `FlightComputer` never reads wall-clock time.
- Every update receives a caller-supplied monotonic timestamp.
- The `SIL` runner owns simulated time and advances by a fixed tick.
- At time `T`, the controller consumes sensor observations produced during the preceding interval.
- The operator command is processed at `T`. The injected vehicle event and resulting ignition output then affect the vehicle during `T` to `T + tick`.
- Those resulting observations are consumed by the controller at `T + tick`.
- Inputs, schedules, configuration, and tick duration are explicit, so identical runs produce identical records.

```text
Previous vehicle interval
        |
        v
Sensors observed at T -> FlightComputer update at T
                |
                v
        Ignition output and vehicle event
                |
                v
        Vehicle advances T to T + tick
                |
                v
        Sensors observed at T + tick
```

See the [simulation design](docs/simulation-design.md#update-sequence) for the complete execution order.

## Flight Sequence

```text
SAFE --ARM--> ARMED
ARMED --LAUNCH--> IGNITION
IGNITION --engine running--> THRUST_BUILDUP
THRUST_BUILDUP --liftoff detected--> POWERED_ASCENT
POWERED_ASCENT --cutoff condition--> ENGINE_CUTOFF
ENGINE_CUTOFF --engine stopped--> COAST
```

- `ARM` and `LAUNCH` are operator commands.
- Later transitions are driven by sensor observations.
- `ABORT` is a latched terminal state entered when a monitored fault occurs.
- The controller sets the ignition output to false when entering `ENGINE_CUTOFF` or `ABORT`.

See the [flight computer design](docs/flight-computer-design.md#transition-state) for the complete transition table.

## Demo Application

### Available Scenarios

| Key | Description | Expected sequence outcome |
| --- | --- | --- |
| `nominal` | Nominal launch sequence through coast | `COMPLETED` |
| `ignition-timeout` | Inhibits engine start until the controller aborts | `ABORTED` |

### Command-Line Usage

```text
LaunchFSWApp.exe
LaunchFSWApp.exe nominal
LaunchFSWApp.exe ignition-timeout
LaunchFSWApp.exe all
LaunchFSWApp.exe help
```

- No argument defaults to `nominal`.
- `all` runs every registered scenario.
- `help` displays available options. Also accepts:
  - `-h`
  - `-help`
  - `--help`
- Unknown options return a nonzero exit code.
- A deliberately aborted simulation still returns zero because it is a successfully executed demonstration, not an application failure.

### Example Output

Abbreviated nominal output:

```text
LaunchFSW-SIL

----------------------------

Scenario: Nominal launch

|--------|--------|-------|-----|------|-----|----------------|-----|----------|-------|
| t (ms) | cmd    | event | eng | lift | cut | state          | ign | result   | fault |
|--------|--------|-------|-----|------|-----|----------------|-----|----------|-------|
| 0      | NONE   | NONE  | N   | N    | N   | SAFE           | N   | NONE     | NONE  |
| 1000   | ARM    | NONE  | N   | N    | N   | ARMED          | N   | ACCEPTED | NONE  |
| 2000   | LAUNCH | NONE  | N   | N    | N   | IGNITION       | Y   | ACCEPTED | NONE  |
| 3000   | NONE   | NONE  | N   | N    | N   | IGNITION       | Y   | NONE     | NONE  |
| 4000   | NONE   | NONE  | Y   | N    | N   | THRUST_BUILDUP | Y   | NONE     | NONE  |
 ... intermediate updates omitted ...
| 7000   | NONE   | NONE  | Y   | Y    | N   | POWERED_ASCENT | Y   | NONE     | NONE  |
 ... intermediate updates omitted ...
| 12000  | NONE   | NONE  | Y   | Y    | Y   | ENGINE_CUTOFF  | N   | NONE     | NONE  |
 ... intermediate updates omitted ...
| 16000  | NONE   | NONE  | N   | Y    | Y   | COAST          | N   | NONE     | NONE  |
|--------|--------|-------|-----|------|-----|----------------|-----|----------|-------|

Termination: COAST_REACHED
Final State: COAST
Final fault: NONE
Sequence outcome: COMPLETED

```

## Building

### Prerequisites

- Windows 10 or 11
- Visual Studio 2022 with the Desktop development with C++ workload
- `CMake` 3.24 or newer
- `vcpkg` - manifest mode installs Catch2 automatically during configuration
- `Git`

### Configure and Build

```bat
set VCPKG_ROOT=C:\path\to\vcpkg
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build --config Release --parallel
```

`VCPKG_ROOT` must point to the user’s vcpkg installation.

### Run App

```bat
build\App\Release\LaunchFSWApp.exe help
build\App\Release\LaunchFSWApp.exe nominal
```

### Run Tests

```bat
ctest --test-dir build -C Release --output-on-failure
```

Tests cover controller transitions, command handling, timing boundaries, fault latching, vehicle-model behavior, failure injection, runner validation, and closed-loop scenarios.

## Safety and Fault Handling

| Fault | Trigger |
| --- | --- |
| `NON_MONOTONIC_TIME` | Supplied timestamp is earlier than the previous timestamp |
| `TIME_NOT_ADVANCED` | Supplied timestamp equals the previous timestamp |
| `IGNITION_TIMEOUT` | Engine-running confirmation is not received by the ignition deadline |
| `LIFTOFF_TIMEOUT` | Liftoff confirmation is not received by the liftoff deadline |
| `UNEXPECTED_ENGINE_SHUTDOWN` | Engine stops during thrust buildup or powered ascent |
| `ENGINE_CUTOFF_TIMEOUT` | Engine shutdown is not confirmed by the cutoff deadline |
| `UNEXPECTED_ENGINE_START` | Engine running is observed during coast |

- A valid sensor confirmation received exactly at its timeout deadline takes priority and succeeds.
- Invalid operator commands are rejected without skipping sensor-driven safety processing.
- Entering `ABORT` sets the ignition output to false.
- `ABORT` and its original fault reason remain latched for subsequent updates.

## Design Documentation

- [Flight computer design](docs/flight-computer-design.md)
- [Simulation design](docs/simulation-design.md)

## Scope and Limitations

- Is educational and demonstrative, not flight-ready software.
- Uses Boolean sensor observations and configurable delays instead of physical vehicle dynamics.
- Models one flight computer with no redundancy or voting.
- Uses a single-threaded fixed-tick simulation, not an RTOS or real-time scheduler.
- Does not model GNC, aerodynamics, 6-DOF motion, orbital mechanics, avionics buses, HIL, landing, or reusable-vehicle behavior.
- Does not establish hardware timing guarantees, worst-case execution time, certification evidence, or operational safety.
- Is currently built and tested through Windows/MSVC CI.

## Future Work

- Add a small number of additional injected-fault scenarios.
- Exercise the standard C++ implementation with another compiler/platform in CI.
- Support loading deterministic scenario definitions from a simple data file.
- Replace the delay-based vehicle model through `VehicleModelInterface` with a higher-fidelity model without changing `FlightComputer`.
- Explore GNC, RTOS, avionics-bus, or HIL integration only as separate future projects.
