# Simulation

## Purpose

The simulation provides a deterministic software-in-the-loop environment for exercising the flight computer with scheduled commands and simulated vehicle feedback. 
It records each update so nominal behavior, timing boundaries, and injected failures can be inspected and tested.

The initial vehicle model intentionally uses Boolean observations and configurable delays. 
The simulation architecture allows it to be replaced later by a higher-fidelity physics-based vehicle model without changing the flight computer.

## Update Sequence

| Step | Owner | Action | 
| ---- | ----- | ------ |
| 1 | SimulationRunner | Retrieve command & vehicle event scheduled for the current simulation time | 
| 2 | FlightComputer | Process the command, current sensor snapshot, and current simulation time to produce the controller output |
| 3 | SimulationRunner | Record the timestamp, command, event, sensor snapshot, and controller output | 
| 4 | SimulationRunner | Check controller-state and maximum duration termination conditions | 
| 5 | VehicleModel | Apply vehicle event, then consume the ignition command and advance the vehicle simulation by one tick, finally produce the next observations |
| 6 | SimulationRunner | Store the vehicle observations for the next flight computer update |
| 7 | SimulationRunner | Advance the simulation clock by one tick and begin the next cycle | 

Notes 
- First flight-computer update occurs at simulation time zero, and the initial sensor snapshot contains all false observations.
- Commands are scheduled using elapsed simulation time.
- The sensor snapshot used at time T represents vehicle behaviour completed before time T.
- The ignition command produced at time T affects the vehicle from T to T + tick.
	- Observations produced during that interval are consumed at T + tick.
- After the first update, every flight-computer update receives a timestamp greater than the previous timestamp.
- An update at the exact maximum duration is processed. 
	- If no higher-priority termination occurs, the scenario then terminates because its maximum duration was reached.
	- If a controller terminal condition and maximum duration occur together, the controller result takes priority.

## Simulation Behaviour

### Failure injection

The initial simulation supports scheduled injection of the following vehicle failures:
- No engine start
- No liftoff detected
- Unexpected engine shutdown during powered flight 
- Engine fails to stop after cutoff
- Unexpected engine restart after cutoff

Failure injection changes simulated vehicle behavior. It does not directly change the flight-computer state.

### Termination

Every scenario has a maximum duration to prevent an infinite simulation

A scenario may terminate when:
- The flight computer reaches COAST 
- The flight computer reaches ABORT
- The configured maximum duration is reached

A scenario testing an unexpected restart remains active after entering COAST and terminates when the expected abort occurs or maximum duration is reached.

### Recorded Data

Each simulation cycle records:
- Current simulation time
- Issued command
- Sensor snapshot
- Flight-computer state
- Ignition command
- Command result
- Fault reason
- Vehicle event

The application presents these records as a readable transition timeline.


## Vehicle Model Interface

Boundary between SimulationRunner and any vehicle model.

| Direction | Data | Purpose | 
| --------- | ---- | ------- |
| Input | ignition_command | Desired engine command produced by the flight computer | 
| Input | tick_duration | Duration of simulated time the vehicle must advance | 
| Input | vehicle_event | Event to be applied to the vehicle model | 
| Output | SensorSnapshot | Vehicle observations produced for the next FlightComputer update | 

Notes:
- The vehicle model is updated once during every non-terminated simulation cycle.
- The returned SensorSnapshot is stored by SimulationRunner and consumed by FlightComputer during the next cycle.
- The vehicle model owns its internal vehicle state and event timers.
- The vehicle model does not inspect the FlightComputer state, command schedule, termination policy, or fault reason.
- A new vehicle-model instance is created for each scenario.
- Given the same configuration and sequence of inputs, the vehicle model produces the same observations.
- The vehicle model receives its model-specific configuration when it is created. Configuration is not supplied during each update.

## Basic Vehicle Model

### Vehicle Behaviour

| Condition | Simulated Behaviour | 
| --------- | ------------------- |
| ignition_command changes from false to true | Start the engine-start timer |
| Engine-start delay expires | Set engine_running = true |
| engine_running becomes true | Start the liftoff timer|
| Liftoff timer expires | Set liftoff_detected = true |
| liftoff_detected becomes true | Start the powered-flight timer |
| Powered-flight timer expires | Set cutoff_condition_met = true | 
| ignition_command changes from true to false while the engine is running | Start the engine-stop timer |
| Engine-stop timer expires | Set engine_running = false |

Once liftoff_detected or cutoff_condition_met is true, it remains true for the rest of the simulation.

### Timer rules 

- Each timer starts only once for its triggering event.
- Active timers accumulate the supplied tick-duration.
- A timer expires on the first tick where its elapsed duration is greater than or equal to its configured delay.
- If a delay is not an exact multiple of the tick, its event occurs on the next tick boundary.
- An event occurring at the end of a tick may start another timer, but the new timer begins accumulating on the next vehicle update.
- A vehicle event recorded at time T is applied during the interval from T to T + tick, and its resulting observations are consumed at T + tick.

### Events 

- INHIBIT_*:
	- Latch for the remainder of the scenario 
	- Applied to an event that already occurred does not reverse it
- FORCE_ENGINE_STOP:
	- Immediately sets engine_running = false
	- Cancels any active engine-stop timer
	- Cancels any active engine-start timer
- FORCE_ENGINE_RESTART:
	- Immediately sets engine_running = true
	- Cancels any active engine-stop timer
- Any resulting observation change is reflected in the next SensorSnapshot produced by the vehicle model.

## Definitions

- **SimulationRunner**
	- One Scenario per simulation
	- Owns the simulation clock
	- Executes update cycles
	- Records results 
	- Determines when the scenario ends 
	- Receives a fresh FlightComputer instance for each simulation
	- Receives a fresh VehicleModel instance for each simulation
	- A SimulationRunner instance executes exactly one simulation and may only be run once.
- **VehicleModelInterface**
	- Boundary between SimulationRunner and any vehicle model
- **BasicVehicleModel**
	- Consumes the ignition command
	- Provides vehicles observations
- **ScenarioConfig**
	- Defines timing, injected failures, scenario termination policy
- **FlightComputer**
	- Consumes commands and sensor observations
	- Produces controller output
	

## Types

- **SimulationRunner**: An object that executes the simulation update sequence, records results, and determines when the scenario ends.
	- m_SimulationConfig 
	- m_ScenarioConfig
	- m_VehicleModel
	- m_CurrentTime
	- m_FlightComputer
- **VehicleModelInterface**: Boundary to the vehicle model
	- ApplyEvent(vehicle_event)
	- Step(ignition_command, delta_time) -> SensorSnapshot
- **BasicVehicleModel**: An object that simulates the engine and vehicle behaviour using configurable delays and Boolean observations.
- **Config**
	- **SimulationConfig**
		- Simulation tick duration
		- Maximum scenario duration
	- **ScenarioConfig**
		- list of SimulationScheduleEntry
		- Termination policy
	- **BasicVehicleModelConfig**
		- Engine-start delay
		- Liftoff delay
		- Powered-ascent duration
		- Engine-stop delay
- **SimulationRecord**: A structure representing the recorded data for a single simulation cycle.
	- timestamp
	- command
	- sensor_snapshot
	- flight_state
	- ignition_command
	- command_result
	- fault_reason
	- event
- **VehicleEvent**: An enumeration representing the different events that can occur in the vehicle model.
	- **NONE**
	- **INHIBIT_ENGINE_START**
	- **INHIBIT_LIFTOFF**
	- **INHIBIT_ENGINE_STOP**
	- **FORCE_ENGINE_STOP**
	- **FORCE_ENGINE_RESTART**
- **SimulationResult**: A structure representing the overall result of a simulation scenario.
	- records: List of SimulationRecord
	- termination_reason: TerminationReason
	- final_controller_output: ControllerOutput
- **SimulationScheduleEntry**: A structure representing scheduled simulation inputs, to be issued at a specific simulation time.
	- time: Duration
	- command: Command
	- event: VehicleEvent
- **TerminationReason**: An enumeration representing the reason for scenario termination.
	- **MAX_DURATION_REACHED**
	- **ABORT**
	- **COAST_REACHED**

## Design Decisions

- The simulation clock is deterministic and does not use wall-clock time.
- The flight computer remains independent of the simulation.
- The vehicle model reacts only to controller outputs and scheduled vehicle events.
- The first implementation uses Boolean observations and configurable delays.
- The simulation does not model trajectory, forces, fuel consumption, guidance, navigation, landing, or recovery.

## Future Work

- A higher-fidelity vehicle model could replace BasicVehicleModel without changing SimulationRunner or FlightComputer.
- A physics-based model could calculate continuous vehicle truth and convert that truth into the same SensorSnapshot interface.
- A separate sensor model could later add measurement delay, noise, and invalid data without exposing simulation truth to FlightComputer.
