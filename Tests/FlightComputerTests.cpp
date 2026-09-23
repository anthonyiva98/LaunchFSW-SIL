#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>

#include "FlightTypes.h"
#include "FlightComputer.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{
	struct FlightTestScenario
	{
		FlightCore::TimeoutConfig timeoutConfig;
		FlightCore::FlightComputer fsw{ timeoutConfig };
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::ControllerOutput output;

		FlightTestScenario() : output(fsw.Update({ FlightCore::Command::NONE, snapshot, currentTime }))
		{
			REQUIRE(output.m_state == FlightCore::FlightState::SAFE);
		}

		void Update(FlightCore::Command command = FlightCore::Command::NONE, bool in_bUpdateTime = true)
		{
			if (in_bUpdateTime)
			{
				currentTime += 1s;
			}
			output = fsw.Update({ command, snapshot, currentTime });
		}

		void DriveNominallyTo(FlightCore::FlightState targetState)
		{
			while (output.m_state != targetState)
			{
				const auto previousState = output.m_state;

				switch (output.m_state)
				{
				case FlightCore::FlightState::SAFE:
					Update(FlightCore::Command::ARM);
					break;

				case FlightCore::FlightState::ARMED:
					Update(FlightCore::Command::LAUNCH);
					break;

				case FlightCore::FlightState::IGNITION:
					snapshot.m_bEngineRunning = true;
					Update();
					break;

				case FlightCore::FlightState::THRUST_BUILDUP:
					snapshot.m_bLiftoffDetected = true;
					Update();
					break;

				case FlightCore::FlightState::POWERED_ASCENT:
					snapshot.m_bCutoffConditionMet = true;
					Update();
					break;

				case FlightCore::FlightState::ENGINE_CUTOFF:
					snapshot.m_bEngineRunning = false;
					Update();
					break;

				case FlightCore::FlightState::COAST:
				case FlightCore::FlightState::ABORT:
					FAIL("Target state cannot be reached through the nominal path");
				}

				REQUIRE(output.m_state != previousState);
			}
		}
	};
}

TEST_CASE("FlightComputer Valid Update", "[FlightComputer]")
{
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	SECTION("Initial state is SAFE")
	{
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::SAFE);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Transition from SAFE to ARMED on ARM command")
	{
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		auto output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ARMED);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::ACCEPTED);
	}

	SECTION("Successful LAUNCH Command sequence")
	{
		// ARM the flight computer first
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		auto output = fsw.Update(input);

		INFO("ARM accepted: expecting transition to ARMED");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::ARMED);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

		// Now send the LAUNCH command
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("LAUNCH accepted: expecting transition to IGNITION");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::IGNITION);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

		// Simulate waiting for engine start
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("No engine confirmation: expecting to remain in IGNITION.");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::IGNITION);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate engine running
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("ENGINE_RUNNING detected: expecting transition to THRUST_BUILDUP");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate waiting for liftoff
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("No liftoff detected: expecting to remain in THRUST_BUILDUP");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bLiftoffDetected);

		REQUIRE(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate liftoff detection
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bLiftoffDetected = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("LIFTOFF_DETECTED: expecting transition to POWERED_ASCENT");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bLiftoffDetected);

		REQUIRE(output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate time passing without any issues
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("Cutoff condition not met: expecting to remain in POWERED_ASCENT.");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bCutoffConditionMet);

		REQUIRE(output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		REQUIRE(output.m_bIgnitionCommand == true);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate engine cutoff conditions met
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bCutoffConditionMet = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("CUTOFF_CONDITION_MET detected: expecting transition to ENGINE_CUTOFF");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bCutoffConditionMet);

		REQUIRE(output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate time passing without any issues
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("Engine still running: expecting to remain in ENGINE_CUTOFF");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate engine cutoff 
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = false;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("ENGINE_RUNNING = false detected: expecting transition to COAST");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::COAST);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate time passing without any issues
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);

		INFO("COAST detected: expecting stayed in COAST");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count());
		CAPTURE(snapshot.m_bEngineRunning);

		REQUIRE(output.m_state == FlightCore::FlightState::COAST);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
	}
	
}


TEST_CASE("FlightComputer Update Command rejection", "[FlightComputer]")
{
	FlightTestScenario scenario;

	SECTION("Reject LAUNCH command in SAFE state")
	{
		scenario.Update(FlightCore::Command::LAUNCH);

		CHECK(scenario.output.m_state == FlightCore::FlightState::SAFE);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject ARM command in ARMED state")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::ARMED);
		scenario.Update(FlightCore::Command::ARM);

		CHECK(scenario.output.m_state == FlightCore::FlightState::ARMED);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject any command in IGNITION state, while engine confirmation stays valid")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::IGNITION);

		INFO("LAUNCH accepted: expecting transition to IGNITION");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.currentTime.time_since_epoch()).count());
		CAPTURE(scenario.snapshot.m_bEngineRunning);

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::IGNITION);
		REQUIRE(scenario.output.m_bIgnitionCommand == true);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

		// Try to send ARM command in IGNITION state, with no sensor state changes
		scenario.Update(FlightCore::Command::ARM);

		INFO("ARM command in IGNITION state: expecting rejection and remain in IGNITION");

		CHECK(scenario.output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send LAUNCH command in IGNITION state, with no sensor state changes
		scenario.Update(FlightCore::Command::LAUNCH);

		INFO("LAUNCH command in IGNITION state: expecting rejection and remain in IGNITION");

		CHECK(scenario.output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send any command in IGNITION state, with engine running confirmation
		scenario.snapshot.m_bEngineRunning = true;
		scenario.Update(FlightCore::Command::ARM);

		INFO("Any command in IGNITION state with engine running confirmation: expecting command rejection, transition to THRUST_BUILDUP");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.currentTime.time_since_epoch()).count());
		CAPTURE(scenario.snapshot.m_bEngineRunning);

		CHECK(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

	}

	SECTION("Reject any command in THRUST_BUILDUP state, while liftoff confirmation stays valid")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::THRUST_BUILDUP);

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		REQUIRE(scenario.output.m_bIgnitionCommand == true);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

		// Try to send ARM command in THRUST_BUILDUP state, with no sensor state changes
		scenario.Update(FlightCore::Command::ARM);

		INFO("ARM command in THRUST_BUILDUP state: expecting rejection and remain in THRUST_BUILDUP");

		CHECK(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send LAUNCH command in THRUST_BUILDUP state, with no sensor state changes
		scenario.Update(FlightCore::Command::LAUNCH);

		INFO("LAUNCH command in THRUST_BUILDUP state: expecting rejection and remain in THRUST_BUILDUP");

		CHECK(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send any command in THRUST_BUILDUP state with liftoff detected
		scenario.snapshot.m_bLiftoffDetected = true;
		scenario.Update(FlightCore::Command::ARM);

		INFO("Any command in THRUST_BUILDUP state with liftoff detected: expecting command rejection, transition to POWERED_ASCENT");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.currentTime.time_since_epoch()).count());
		CAPTURE(scenario.snapshot.m_bLiftoffDetected);

		CHECK(scenario.output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}
	
	SECTION("Reject any command in POWERED_ASCENT state, while engine cutoff confirmation stays valid")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::POWERED_ASCENT);

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		REQUIRE(scenario.output.m_bIgnitionCommand == true);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

		// Try to send ARM command in POWERED_ASCENT state, with no sensor state changes
		scenario.Update(FlightCore::Command::ARM);

		INFO("ARM command in POWERED_ASCENT state: expecting rejection and remain in POWERED_ASCENT");

		CHECK(scenario.output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send LAUNCH command in POWERED_ASCENT state, with no sensor state changes
		scenario.Update(FlightCore::Command::LAUNCH);

		INFO("LAUNCH command in POWERED_ASCENT state: expecting rejection and remain in POWERED_ASCENT");

		CHECK(scenario.output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send any command in POWERED_ASCENT state with cuttoff detected
		scenario.snapshot.m_bCutoffConditionMet = true;
		scenario.Update(FlightCore::Command::ARM);

		INFO("Any command in POWERED_ASCENT state with cutoff detected: expecting command rejection, transition to ENGINE_CUTOFF");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.currentTime.time_since_epoch()).count());
		CAPTURE(scenario.snapshot.m_bCutoffConditionMet);

		CHECK(scenario.output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject any command in ENGINE_CUTOFF state, while engine confirmation stays valid")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::ENGINE_CUTOFF);

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		REQUIRE(scenario.output.m_bIgnitionCommand == false);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

		// Try to send ARM command in ENGINE_CUTOFF state, with no sensor state changes
		scenario.Update(FlightCore::Command::ARM);

		INFO("ARM command in ENGINE_CUTOFF state: expecting rejection and remain in ENGINE_CUTOFF");

		CHECK(scenario.output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send LAUNCH command in ENGINE_CUTOFF state, with no sensor state changes
		scenario.Update(FlightCore::Command::LAUNCH);

		INFO("LAUNCH command in ENGINE_CUTOFF state: expecting rejection and remain in ENGINE_CUTOFF");

		CHECK(scenario.output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send any command in ENGINE_CUTOFF state with engine confirmation detected
		scenario.snapshot.m_bEngineRunning = false;
		scenario.Update(FlightCore::Command::ARM);

		INFO("Any command in ENGINE_CUTOFF state with engine confirmation detected: expecting command rejection, transition to COAST");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.currentTime.time_since_epoch()).count());
		CAPTURE(scenario.snapshot.m_bEngineRunning);

		CHECK(scenario.output.m_state == FlightCore::FlightState::COAST);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject any command in COAST state")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::COAST);

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::COAST);
		REQUIRE(scenario.output.m_bIgnitionCommand == false);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

		// Try to send ARM command in COAST state, with no sensor state changes
		scenario.Update(FlightCore::Command::ARM);

		INFO("ARM command in COAST state: expecting rejection and remain in COAST");

		CHECK(scenario.output.m_state == FlightCore::FlightState::COAST);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Try to send LAUNCH command in COAST state, with no sensor state changes
		scenario.Update(FlightCore::Command::LAUNCH);

		INFO("LAUNCH command in COAST state: expecting rejection and remain in COAST");

		CHECK(scenario.output.m_state == FlightCore::FlightState::COAST);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

}


TEST_CASE("FlightComputer Update IGNITION timeout deadline", "[FlightComputer]")
{
	FlightTestScenario scenario;
	scenario.DriveNominallyTo(FlightCore::FlightState::IGNITION);

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::IGNITION);
	REQUIRE(scenario.output.m_bIgnitionCommand == true);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

	SECTION("Engine confirmation exactly at the deadline succeeds")
	{
		scenario.currentTime += scenario.timeoutConfig.m_ignitionTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bEngineRunning = true;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected thrust_build at the deadline with engine on");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_ignitionTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("No confirmation at the deadline produces IGNITION_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_ignitionTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bEngineRunning = false;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected ignition timeout at the deadline with engine off");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_ignitionTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Confirmation after the deadline still produces IGNITION_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_ignitionTimeout + 1s; // Advance time passed the deadline
		scenario.snapshot.m_bEngineRunning = true;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected ignition timeout after the deadline regardless of engine state");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_ignitionTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

}


TEST_CASE("FlightComputer Update LIFTOFF timeout deadline", "[FlightComputer]")
{
	FlightTestScenario scenario;
	scenario.DriveNominallyTo(FlightCore::FlightState::THRUST_BUILDUP);

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
	REQUIRE(scenario.output.m_bIgnitionCommand == true);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Liftoff exactly at the deadline succeeds")
	{
		scenario.currentTime += scenario.timeoutConfig.m_liftoffTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bLiftoffDetected = true;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected POWERED_ASCENT at the deadline with liftoff detected");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_liftoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(scenario.output.m_bIgnitionCommand == true);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("No confirmation at the deadline produces LIFTOFF_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_liftoffTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bLiftoffDetected = false;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected liftoff timeout at the deadline with no liftoff detected");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_liftoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::LIFTOFF_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Confirmation after the deadline still produces LIFTOFF_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_liftoffTimeout + 1s; // Advance time passed the deadline
		scenario.snapshot.m_bLiftoffDetected = true;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected liftoff timeout after the deadline regardless of liftoff detection");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_liftoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::LIFTOFF_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

}


TEST_CASE("FlightComputer Update ENGINE_CUTOFF timeout deadline", "[FlightComputer]")
{
	FlightTestScenario scenario;
	scenario.DriveNominallyTo(FlightCore::FlightState::ENGINE_CUTOFF);

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::ENGINE_CUTOFF);
	REQUIRE(scenario.output.m_bIgnitionCommand == false);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Engine cutoff exactly at the deadline succeeds")
	{
		scenario.currentTime += scenario.timeoutConfig.m_engineCutoffTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bEngineRunning = false;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected COAST at the deadline with engine shutdown confirmed");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_engineCutoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::COAST);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("No confirmation at the deadline produces ENGINE_CUTOFF_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_engineCutoffTimeout; // Advance time to exactly the deadline
		scenario.snapshot.m_bEngineRunning = true;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected ENGINE_CUTOFF_TIMEOUT at the deadline while the engine remains running");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_engineCutoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::ENGINE_CUTOFF_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Confirmation after the deadline still produces ENGINE_CUTOFF_TIMEOUT")
	{
		scenario.currentTime += scenario.timeoutConfig.m_engineCutoffTimeout + 1s; // Advance time passed the deadline
		scenario.snapshot.m_bEngineRunning = false;
		scenario.Update(FlightCore::Command::NONE, false);

		INFO("Expected ENGINE_CUTOFF_TIMEOUT when engine shutdown is confirmed after the deadline");
		CAPTURE(std::chrono::duration_cast<std::chrono::milliseconds>(scenario.timeoutConfig.m_engineCutoffTimeout).count());

		CHECK(scenario.output.m_state == FlightCore::FlightState::ABORT);
		CHECK(scenario.output.m_bIgnitionCommand == false);
		CHECK(scenario.output.m_fault == FlightCore::FaultReason::ENGINE_CUTOFF_TIMEOUT);
		CHECK(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

}


TEST_CASE("FlightComputer Update Unexpected engine start in COAST produces ABORT", "[FlightComputer]")
{
	// Drive the controller to COAST
	FlightTestScenario scenario;
	scenario.DriveNominallyTo(FlightCore::FlightState::COAST);

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::COAST);
	REQUIRE(scenario.output.m_bIgnitionCommand == false);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

	// Simulate engine detected on
	scenario.snapshot.m_bEngineRunning = true;
	scenario.Update();

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::ABORT);
	REQUIRE(scenario.output.m_bIgnitionCommand == false);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_START);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

}


TEST_CASE("FlightComputer Update Unexpected Engine Shutdown", "[FlightComputer]")
{
	FlightTestScenario scenario;
	scenario.DriveNominallyTo(FlightCore::FlightState::THRUST_BUILDUP);

	REQUIRE(scenario.output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
	REQUIRE(scenario.output.m_bIgnitionCommand == true);
	REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Engine shutdown during THRUST_BUILDUP produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		scenario.snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		scenario.Update();

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(scenario.output.m_bIgnitionCommand == false);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Engine shutdown during POWERED_ASCENT produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		scenario.DriveNominallyTo(FlightCore::FlightState::POWERED_ASCENT);

		scenario.snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		scenario.Update();

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(scenario.output.m_bIgnitionCommand == false);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Engine shutdown during successful liftoff produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		scenario.snapshot.m_bLiftoffDetected = true; // Simulate liftoff
		scenario.snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		scenario.Update();

		REQUIRE(scenario.output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(scenario.output.m_bIgnitionCommand == false);
		REQUIRE(scenario.output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(scenario.output.m_commandResult == FlightCore::CommandResult::NONE);
	}


}


TEST_CASE("FlightComputer Update time updates", "[FlightComputer]")
{
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	SECTION("Unchanged time produces TIME_NOT_ADVANCED fault")
	{
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		fsw.Update(input);

		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		fsw.Update(input);

		// Do not advance time
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::TIME_NOT_ADVANCED);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Non-monotonic time produces NON_MONOTONIC_TIME fault")
	{
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		fsw.Update(input);

		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		fsw.Update(input);

		currentTime = currentTime - 2s; // Move time backwards by 2 seconds
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::NON_MONOTONIC_TIME);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
	}
}


TEST_CASE("FlightComputer Update Command rejection in ABORT state", "[FlightComputer]")
{
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	// Transition to ABORT state
	FlightCore::SensorSnapshot snapshot;
	FlightCore::TimePoint currentTime{};
	FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + timeoutConfig.m_ignitionTimeout + 1s; // Advance time passed the deadline
	snapshot.m_bEngineRunning = false; // Simulate engine not running
	input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
	auto output = fsw.Update(input);

	REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
	REQUIRE(output.m_bIgnitionCommand == false);
	REQUIRE(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
	REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Reject any command in ABORT state")
	{
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);
		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::ARM, snapshot, currentTime);
		output = fsw.Update(input);
		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::REJECTED);

		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		output = fsw.Update(input);
		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}
}

