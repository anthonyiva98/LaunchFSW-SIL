

# Flight Computer

## Transition State 

| Current State | Input\Event | Next State | Output / Observable Result | 
| ------------- | ----------- | ---------- | -------------------------- |
| SAFE | ARM command | ARMED | State = ARMED, ignition_command = false, Fault = NONE | 
| ARMED | LAUNCH command | IGNITION | State = IGNITION, ignition_command = true, Fault = NONE  | 
| IGNITION | engine_running = true | THRUST_BUILDUP | State = THRUST_BUILDUP, ignition_command = true, Fault = NONE   |
| IGNITION | engine_running = false at timeout | ABORT | State = ABORT, ignition_command = false, Fault = IGNITION_TIMEOUT | 
| THRUST_BUILDUP | liftoff_detected = true | POWERED_ASCENT | State = POWERED_ASCENT, ignition_command = true, Fault = NONE  |
| THRUST_BUILDUP | liftoff_detected = false at timeout | ABORT | State = ABORT, ignition_command = false, Fault = LIFTOFF_TIMEOUT | 
| THRUST_BUILDUP | engine_running = false | ABORT | State = ABORT, ignition_command = false, Fault = UNEXPECTED_ENGINE_SHUTDOWN | 
| POWERED_ASCENT | engine_running = false | ABORT | State = ABORT, ignition_command = false, Fault = UNEXPECTED_ENGINE_SHUTDOWN | 
| POWERED_ASCENT | cutoff_condition_met = true | ENGINE_CUTOFF | State = ENGINE_CUTOFF, ignition_command = false, Fault = NONE | 
| POWERED_ASCENT | engine_running = false and cutoff_condition_met = true | ABORT | State = ABORT, ignition_command = false, Fault = UNEXPECTED_ENGINE_SHUTDOWN | 
| ENGINE_CUTOFF | engine_running = false | COAST | State = COAST, ignition_command = false, Fault = NONE | 
| ENGINE_CUTOFF | engine_running = true at timeout | ABORT | State = ABORT, ignition_command = false, Fault = ENGINE_CUTOFF_TIMEOUT | 
| COAST | engine_running = true | ABORT | State = ABORT, ignition_command = false, Fault = UNEXPECTED_ENGINE_START |

Notes: 
- The flight computer reports its desired ignition-command level on every update. 
	- It remains true through: IGNITION, THRUST_BUILDUP, and POWERED_ASCENT states. 
	- It becomes false in: SAFE, ARMED, ENGINE_CUTOFF, COAST or ABORT states.
- Each update, valid sensor observations are processed before timeout conditions. Therefore, confirmation received at the deadline is accepted.
- Once the controller enters ABORT, subsequent updates retain the ABORT state, keep ignition_command false, and preserve the original fault reason
- Priority
	1. Immediate safety faults (Ex. unexpected engine shutdown)
	1. Successful sensor-driven transitions (Ex. engine running, liftoff detected)
	1. Timeout failures
- Time
	- Ignition timeout begins upon entering IGNITION
	- Liftoff timeout begins upon entering THRUST_BUILDUP
	- Engine cutoff timeout begins upon entering ENGINE_CUTOFF
- Mission Complete:
    - COAST is reached when the engine is confirmed shut down after the cutoff condition is met.
    - The flight computer remains in COAST until the external scenario is stopped.

## Definitions

- **Flight State**: The current state of the flight computer. One of:
	- **SAFE**: The flight computer is in a safe state, with no ignition command issued.
	- **ARMED**: The flight computer has received an ARM command and is ready to initiate ignition.
	- **IGNITION**: The flight computer has received a LAUNCH command and is actively commanding ignition.
	- **THRUST_BUILDUP**: The flight computer has confirmed that the engine is running and is in the process of building thrust.
	- **POWERED_ASCENT**: The flight computer is in the powered ascent phase, with the engine running and providing thrust.
	- **ENGINE_CUTOFF**: The flight computer has commanded the engine to turn off and is awaiting confirmation that the engine stopped.
	- **COAST**: The successful final flight phase for this project. The controller continues monitoring for an unexpected engine restart while the SIL decides when to stop the scenario.
	- **ABORT**: The flight computer is in an abort state, with the ignition command disabled.
- **Input Commands**: Commands received by the flight computer to change its state. One of:
	- **ARM**: Command to arm the flight computer for ignition.
	- **LAUNCH**: Command to initiate ignition and start the launch sequence.
- **Sensor Observations**: Observations from sensors that inform the flight computer about the status of the engine and the vehicle. Contains:
	- **engine_running = true**: A confirmation from sensors that the engine has successfully started and is running.
	- **engine_running = false**: A confirmation from sensors that the engine is not running.
	- **liftoff_detected = true**: A confirmation from sensors that the vehicle has lifted off the ground.
	- **liftoff_detected = false**: A confirmation from sensors that the vehicle has not lifted off the ground.
	- **cutoff_condition_met = true**: A confirmation derived from sensors that the conditions for engine cutoff have been met.
	- **cutoff_condition_met = false**: A confirmation derived from sensors that the conditions for engine cutoff have not been met.
- **Controller Output**: The output from the flight computer that commands the ignition system. One of:
	- **ignition_command = true**: Engine ignition command is active, indicating that the engine should be running.
	- **ignition_command = false**: Engine ignition command is inactive, indicating that the engine should not be running.
- **Fault reasons**: The reasons for transitioning to the ABORT state. One of:
	- **NONE**: No faults detected.
	- **NON_MONOTONIC_TIME**: The flight computer received a non-monotonic timestamp from the caller, indicating a potential issue with timekeeping.
	- **TIME_NOT_ADVANCED**: The flight computer received a timestamp that is not greater than the previous timestamp, indicating a potential issue with timekeeping.
	- **IGNITION_TIMEOUT**: The flight computer failed to confirm engine running within the timeout deadline.
	- **LIFTOFF_TIMEOUT**: The flight computer failed to detect liftoff within the timeout deadline.
	- **ENGINE_CUTOFF_TIMEOUT**: The flight computer failed to confirm engine shutdown after the cutoff condition is met within the timeout deadline.
	- **UNEXPECTED_ENGINE_START**: The flight computer detected an unexpected engine start after engine shutdown was confirmed.
	- **UNEXPECTED_ENGINE_SHUTDOWN**: The flight computer detected an unexpected engine shutdown after engine operation was confirmed.
- **Time**:
	- Update will contain a current_time member: a monotonic timestamp supplied by the caller. 
		- Its epoch is unspecified; only differences between timestamps are meaningful. 
		- The SIL controls this timestamp deterministically, while a deployed integration would obtain it from a monotonic hardware or system clock.

## Types

- **FlightState**: An enumeration representing the different states of the flight computer.
	- **SAFE** 
	- **ARMED** 
	- **IGNITION**
	- **THRUST_BUILDUP** 
	- **POWERED_ASCENT** 
	- **ENGINE_CUTOFF**
	- **COAST**
	- **ABORT** 
- **Command**: An enumeration representing the different input commands that can be sent to the flight computer.
	- **NONE** : No command issued
	- **ARM** : Command to transition to ARMED state 
	- **LAUNCH** : Command to perform launch sequence
- **FaultReason**: An enumeration representing the different fault reasons that can cause the flight computer to transition to the ABORT state.
	- **NONE**
	- **NON_MONOTONIC_TIME**
	- **TIME_NOT_ADVANCED**
	- **IGNITION_TIMEOUT** 
	- **LIFTOFF_TIMEOUT** 
	- **ENGINE_CUTOFF_TIMEOUT**
	- **UNEXPECTED_ENGINE_SHUTDOWN** 
	- **UNEXPECTED_ENGINE_START**
- **SensorSnapshot**: A structure representing the current observations from the sensors.
	- engine_running: Boolean
	- liftoff_detected: Boolean
	- cutoff_condition_met: Boolean
- **CommandResult**: An enumeration representing the result of processing a command.
	- **NONE**: No command was processed
	- **ACCEPTED**: The command was successfully processed and caused a state transition
	- **REJECTED**: The command was rejected due to being invalid in the current state
- **ControllerOutput**: A structure representing the output from the flight computer
	- state: FlightState
	- ignition_command: Boolean
	- fault: FaultReason
	- command_result: CommandResult
- **TimeoutConfig**: A config structure representing the timeout durations
	- ignition_timeout: Duration - engine running confirmation.
	- liftoff_timeout: Duration - liftoff detection.
	- engine_cutoff_timeout: Duration - engine cutoff confirmation.
- **UpdateInput**: A structure representing the input to the flight computer for a single update cycle
	- command: Command
	- sensors: SensorSnapshot
	- current_time: Time

## Design Decisions

- **FlightComputer class**
	- On the first update, the controller records the supplied monotonic timestamp. 
		- On subsequent updates, a timestamp earlier than the previously accepted timestamp causes a latched abort with NON_MONOTONIC_TIME. 
		- Equal timestamps are not permitted.