#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>

#include "FlightTypes.h"
#include "FlightComputer.h"

#include <chrono>

using namespace std::chrono_literals;

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

		// Now send the LAUNCH command
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

		// Simulate waiting for engine start
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate engine running
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate waiting for liftoff
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate liftoff detection
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bLiftoffDetected = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);

		// Simulate time passing without any issues
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		output = fsw.Update(input);
		CHECK(output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}
	
}


TEST_CASE("FlightComputer Update Command rejection", "[FlightComputer]")
{
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	SECTION("Reject LAUNCH command in SAFE state")
	{
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::LAUNCH, snapshot, currentTime);
		auto output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::SAFE);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject ARM command in ARMED state")
	{
		// First, arm the flight computer
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		auto output = fsw.Update(input);

		// Now, try to arm it again
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::ARM, snapshot, currentTime);
		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ARMED);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::REJECTED);
	}

	SECTION("Reject any command in IGNITION state, while engine confirmation stays valid")
	{
		// First, arm the flight computer
		FlightCore::SensorSnapshot snapshot;
		FlightCore::TimePoint currentTime{};
		FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
		fsw.Update(input);

		// Now, send the LAUNCH command to transition to IGNITION
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		fsw.Update(input);

		// Now, try to send any command in IGNITION state
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::ARM, snapshot, currentTime);
		auto output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Now, try to send any command in IGNITION state
		currentTime = currentTime + 1s; // Advance time by 1 second
		input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::IGNITION);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::REJECTED);

		// Simulate engine running
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = true;
		input = FlightCore::UpdateInput(FlightCore::Command::ARM, snapshot, currentTime);
		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::REJECTED);

	}
}


TEST_CASE("FlightComputer Update IGNITION timeout deadline", "[FlightComputer]")
{
	// Starting from IGNITION
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	FlightCore::SensorSnapshot snapshot;
	FlightCore::TimePoint currentTime{};
	FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
	auto output = fsw.Update(input);

	REQUIRE(output.m_state == FlightCore::FlightState::IGNITION);
	REQUIRE(output.m_bIgnitionCommand == true);
	REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(output.m_commandResult == FlightCore::CommandResult::ACCEPTED);

	SECTION("Engine confirmation exactly at the deadline succeeds")
	{

		currentTime = currentTime + timeoutConfig.m_ignitionTimeout; // Advance time to exactly the deadline
		snapshot.m_bEngineRunning = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected thrust_build at the deadline with engine on");
		CAPTURE(timeoutConfig.m_ignitionTimeout.count());

		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("No confirmation at the deadline produces IGNITION_TIMEOUT")
	{
		currentTime = currentTime + timeoutConfig.m_ignitionTimeout; // Advance time to exactly the deadline
		snapshot.m_bEngineRunning = false;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected ignition timeout at the deadline with engine off");
		CAPTURE(timeoutConfig.m_ignitionTimeout.count());

		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ABORT);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Confirmation after the deadline still produces IGNITION_TIMEOUT")
	{

		currentTime = currentTime + timeoutConfig.m_ignitionTimeout + 1s; // Advance time passed the deadline
		snapshot.m_bEngineRunning = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected ignition timeout after the deadline regardless of engine state");
		CAPTURE(timeoutConfig.m_ignitionTimeout.count());

		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ABORT);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::IGNITION_TIMEOUT);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

}


TEST_CASE("FlightComputer Update LIFTOFF timeout deadline", "[FlightComputer]")
{
	// Starting from THRUST_BUILDUP
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);

	FlightCore::SensorSnapshot snapshot;
	FlightCore::TimePoint currentTime{};
	FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	snapshot.m_bEngineRunning = true;
	input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
	auto output = fsw.Update(input);

	REQUIRE(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
	REQUIRE(output.m_bIgnitionCommand == true);
	REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Liftoff exactly at the deadline succeeds")
	{

		currentTime = currentTime + timeoutConfig.m_liftoffTimeout; // Advance time to exactly the deadline
		snapshot.m_bLiftoffDetected = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected POWERED_ASCENT at the deadline with liftoff detected");
		CAPTURE(timeoutConfig.m_liftoffTimeout.count());

		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::POWERED_ASCENT);
		CHECK(output.m_bIgnitionCommand == true);
		CHECK(output.m_fault == FlightCore::FaultReason::NONE);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("No confirmation at the deadline produces LIFTOFF_TIMEOUT")
	{
		currentTime = currentTime + timeoutConfig.m_liftoffTimeout; // Advance time to exactly the deadline
		snapshot.m_bLiftoffDetected = false;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected liftoff timeout at the deadline with no liftoff detected");
		CAPTURE(timeoutConfig.m_liftoffTimeout.count());

		output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ABORT);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::LIFTOFF_TIMEOUT);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Confirmation after the deadline still produces LIFTOFF_TIMEOUT")
	{

		currentTime = currentTime + timeoutConfig.m_liftoffTimeout + 1s; // Advance time passed the deadline
		snapshot.m_bLiftoffDetected = true;
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);

		INFO("Expected liftoff timeout after the deadline regardless of liftoff detection");
		CAPTURE(timeoutConfig.m_liftoffTimeout.count());

		auto output = fsw.Update(input);

		CHECK(output.m_state == FlightCore::FlightState::ABORT);
		CHECK(output.m_bIgnitionCommand == false);
		CHECK(output.m_fault == FlightCore::FaultReason::LIFTOFF_TIMEOUT);
		CHECK(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

}


TEST_CASE("FlightComputer Update Unexpected Engine Shutdown", "[FlightComputer]")
{
	// Starting from THRUST_BUILDUP
	FlightCore::TimeoutConfig timeoutConfig;
	FlightCore::FlightComputer fsw(timeoutConfig);
	FlightCore::SensorSnapshot snapshot;
	FlightCore::TimePoint currentTime{};
	FlightCore::UpdateInput input(FlightCore::Command::ARM, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	input = FlightCore::UpdateInput(FlightCore::Command::LAUNCH, snapshot, currentTime);
	fsw.Update(input);

	currentTime = currentTime + 1s; // Advance time by 1 second
	snapshot.m_bEngineRunning = true;
	input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
	auto output = fsw.Update(input);

	REQUIRE(output.m_state == FlightCore::FlightState::THRUST_BUILDUP);
	REQUIRE(output.m_bIgnitionCommand == true);
	REQUIRE(output.m_fault == FlightCore::FaultReason::NONE);
	REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);

	SECTION("Engine shutdown during THRUST_BUILDUP produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Engine shutdown during POWERED_ASCENT produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bLiftoffDetected = true; // Simulate liftoff
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		fsw.Update(input);

		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
	}

	SECTION("Engine shutdown during successful liftoff produces UNEXPECTED_ENGINE_SHUTDOWN")
	{
		currentTime = currentTime + 1s; // Advance time by 1 second
		snapshot.m_bLiftoffDetected = true; // Simulate liftoff
		snapshot.m_bEngineRunning = false; // Simulate engine shutdown
		input = FlightCore::UpdateInput(FlightCore::Command::NONE, snapshot, currentTime);
		auto output = fsw.Update(input);

		REQUIRE(output.m_state == FlightCore::FlightState::ABORT);
		REQUIRE(output.m_bIgnitionCommand == false);
		REQUIRE(output.m_fault == FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
		REQUIRE(output.m_commandResult == FlightCore::CommandResult::NONE);
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

