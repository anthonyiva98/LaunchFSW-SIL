#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>

#include "TestHelpers.h"

#include "FlightTypes.h"
#include "sim/BasicVehicleModel.h"
#include "sim/SimulationRunner.h"

#include <chrono>
#include <cstddef>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace Simulation;
using namespace FlightCore;

namespace
{
	struct ExpectedSimulationRecord
	{
		Duration m_elapsed = Duration::zero();
		Command m_command = Command::NONE;
		VehicleEvent m_event = VehicleEvent::NONE;

		bool m_bEngineRunning = false;
		bool m_bLiftoffDetected = false;
		bool m_bCutoffConditionMet = false;

		FlightState m_state = FlightState::SAFE;
		bool m_bIgnitionCommand = false;
		FaultReason m_fault = FaultReason::NONE;
		CommandResult m_commandResult = CommandResult::NONE;
	};

	void CheckRecord( const SimulationRecord& actual, const ExpectedSimulationRecord& expected)
	{
		INFO(
			"Expected record at "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(expected.m_elapsed).count()
			<< " ms");

		CHECK(actual.m_time == TimePoint{} + expected.m_elapsed);
		CHECK(actual.m_command == expected.m_command);
		CHECK(actual.m_event == expected.m_event);

		CHECK(actual.m_snapshot.m_bEngineRunning == expected.m_bEngineRunning);
		CHECK(actual.m_snapshot.m_bLiftoffDetected == expected.m_bLiftoffDetected);
		CHECK(actual.m_snapshot.m_bCutoffConditionMet == expected.m_bCutoffConditionMet);

		CHECK(actual.m_controllerOutput.m_state == expected.m_state);
		CHECK(actual.m_controllerOutput.m_bIgnitionCommand == expected.m_bIgnitionCommand);
		CHECK(actual.m_controllerOutput.m_fault == expected.m_fault);
		CHECK(actual.m_controllerOutput.m_commandResult == expected.m_commandResult);
	}

	struct SimulationTestScenario
	{
		TimeoutConfig m_timeoutConfig{};
		BasicVehicleModelConfig m_vehicleConfig = TestHelpers::MakeBasicVehicleConfig();
		SimulationConfig m_simulationConfig{ 100ms, 300ms };
		ScenarioConfig m_scenarioConfig{};

		void Schedule( Duration time, Command command = Command::NONE, VehicleEvent event = VehicleEvent::NONE)
		{
			m_scenarioConfig.m_schedule.push_back( { time, command, event });
		}

		SimulationResult Run() const
		{
			FlightComputer fsw{ m_timeoutConfig };
			BasicVehicleModel vehicle{ m_vehicleConfig };

			SimulationRunner runner{
				m_simulationConfig,
				m_scenarioConfig,
				fsw,
				vehicle
			};

			return runner.Run();
		}
	};

}

TEST_CASE("SimulationRunner processes the maximum-duration update", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario{};
	const SimulationResult l_simResult = l_scenario.Run();

	REQUIRE(l_simResult.m_terminationReason == TerminationReason::MAX_DURATION_REACHED);
	REQUIRE(l_simResult.m_records.size() == 4);

	Duration l_elapsed = Duration::zero();

	for (const auto& l_record : l_simResult.m_records)
	{
		CheckRecord(l_record, {
			.m_elapsed = l_elapsed
			});

		l_elapsed += l_scenario.m_simulationConfig.m_tickDuration;
	}

	REQUIRE(l_simResult.m_finalControllerOutput.m_state == FlightState::SAFE);
	REQUIRE(l_simResult.m_finalControllerOutput.m_fault == FaultReason::NONE);
}


TEST_CASE("SimulationRunner delivers commands at their scheduled time", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario{};
	l_scenario.Schedule(100ms, Command::ARM);
	const SimulationResult l_simResult = l_scenario.Run();
	const auto& l_records = l_simResult.m_records;

	REQUIRE(l_simResult.m_terminationReason == TerminationReason::MAX_DURATION_REACHED);
	REQUIRE(l_records.size() == 4);

	CheckRecord(l_records[0], {});

	CheckRecord(l_records[1], {
		.m_elapsed = 100ms,
		.m_command = Command::ARM,
		.m_state = FlightState::ARMED,
		.m_commandResult = CommandResult::ACCEPTED
		});

	CheckRecord(l_records[2], {
		.m_elapsed = 200ms,
		.m_state = FlightState::ARMED
		});

	CheckRecord(l_records[3], {
		.m_elapsed = 300ms,
		.m_state = FlightState::ARMED
		});

	REQUIRE(l_simResult.m_finalControllerOutput.m_state == FlightState::ARMED);
	REQUIRE(l_simResult.m_finalControllerOutput.m_fault == FaultReason::NONE);

}


TEST_CASE("SimulationRunner applies vehicle events during the following interval", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario{};
	l_scenario.Schedule(100ms, Command::NONE, VehicleEvent::FORCE_ENGINE_RESTART);
	const SimulationResult l_simResult = l_scenario.Run();
	const auto& l_records = l_simResult.m_records;

	REQUIRE(l_simResult.m_terminationReason == TerminationReason::MAX_DURATION_REACHED);
	REQUIRE(l_records.size() == 4);

	CheckRecord(l_records[0], {});

	CheckRecord(l_records[1], {
		.m_elapsed = 100ms,
		.m_event = VehicleEvent::FORCE_ENGINE_RESTART
		});

	CheckRecord(l_records[2], {
		.m_elapsed = 200ms,
		.m_bEngineRunning = true
		});

	CheckRecord(l_records[3], {
		.m_elapsed = 300ms,
		.m_bEngineRunning = true
		});

	REQUIRE(l_simResult.m_finalControllerOutput.m_state == FlightState::SAFE);
	REQUIRE(l_simResult.m_finalControllerOutput.m_fault == FaultReason::NONE);

}


TEST_CASE("SimulationRunner completes the nominal launch sequence", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario{};
	// Nominal COAST occurs at 16s, intentionally matching maximum duration.
	l_scenario.m_simulationConfig = { 1s, 16s };

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule(2s, Command::LAUNCH);

	const SimulationResult l_result = l_scenario.Run();
	const auto& l_records = l_result.m_records;

	REQUIRE(l_result.m_terminationReason == TerminationReason::COAST_REACHED);
	REQUIRE(l_result.m_records.size() == 17);

	CheckRecord(l_records[0], {});

	CheckRecord(l_records[1], {
		.m_elapsed = 1s,
		.m_command = Command::ARM,
		.m_state = FlightState::ARMED,
		.m_commandResult = CommandResult::ACCEPTED
		});

	CheckRecord(l_records[2], {
		.m_elapsed = 2s,
		.m_command = Command::LAUNCH,
		.m_state = FlightState::IGNITION,
		.m_bIgnitionCommand = true,
		.m_commandResult = CommandResult::ACCEPTED
		});

	CheckRecord(l_records[4], {
		.m_elapsed = 4s,
		.m_bEngineRunning = true,
		.m_state = FlightState::THRUST_BUILDUP,
		.m_bIgnitionCommand = true
		});

	CheckRecord(l_records[7], {
		.m_elapsed = 7s,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_state = FlightState::POWERED_ASCENT,
		.m_bIgnitionCommand = true
		});

	CheckRecord(l_records[12], {
		.m_elapsed = 12s,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::ENGINE_CUTOFF,
		});

	CheckRecord(l_records[16], {
		.m_elapsed = 16s,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::COAST,
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::COAST);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::NONE);

}


TEST_CASE("SimulationRunner terminates on abort", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	// IGNITION_TIMEOUT occurs at 5s, intentionally matching maximum duration.
	l_scenario.m_simulationConfig = { 1s, 5s };

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule( 2s, Command::LAUNCH, VehicleEvent::INHIBIT_ENGINE_START);

	const SimulationResult l_result = l_scenario.Run();

	REQUIRE(l_result.m_terminationReason == TerminationReason::ABORT);
	REQUIRE(l_result.m_records.size() == 6);

	CheckRecord(l_result.m_records.back(), {
		.m_elapsed = 5s,
		.m_state = FlightState::ABORT,
		.m_fault = FaultReason::IGNITION_TIMEOUT
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::ABORT);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::IGNITION_TIMEOUT);
	CHECK_FALSE( l_result.m_finalControllerOutput.m_bIgnitionCommand);

}


TEST_CASE("SimulationRunner continues after COAST", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	l_scenario.m_simulationConfig = { 1s, 20s };
	l_scenario.m_scenarioConfig.m_coastTerminationPolicy = CoastTerminationPolicy::CONTINUE_AFTER_COAST;

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule(2s, Command::LAUNCH);
	l_scenario.Schedule( 17s, Command::NONE, VehicleEvent::FORCE_ENGINE_RESTART);

	const SimulationResult l_result = l_scenario.Run();
	const auto& l_records = l_result.m_records;

	REQUIRE(l_result.m_terminationReason == TerminationReason::ABORT);
	REQUIRE(l_result.m_records.size() == 19);

	CheckRecord(l_records[16], {
		.m_elapsed = 16s,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::COAST
		});

	CheckRecord(l_records[17], {
		.m_elapsed = 17s,
		.m_event = VehicleEvent::FORCE_ENGINE_RESTART,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::COAST
		});

	CheckRecord(l_records[18], {
		.m_elapsed = 18s,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::ABORT,
		.m_fault = FaultReason::UNEXPECTED_ENGINE_START
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::ABORT);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::UNEXPECTED_ENGINE_START);
}


TEST_CASE("SimulationRunner rejects invalid configuration", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;

	SECTION("Tick duration must be positive")
	{
		l_scenario.m_simulationConfig.m_tickDuration = Duration::zero();

		REQUIRE_THROWS_AS( l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Maximum duration cannot be negative")
	{
		l_scenario.m_simulationConfig.m_maxDuration = -100ms;

		REQUIRE_THROWS_AS( l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Maximum duration must align with tick duration")
	{
		l_scenario.m_simulationConfig = { 100ms, 250ms };

		REQUIRE_THROWS_AS( l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Schedule entries cannot be negative")
	{
		l_scenario.Schedule(-100ms);

		REQUIRE_THROWS_AS(l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Schedule entries cannot be after m_maxDuration")
	{
		l_scenario.Schedule(l_scenario.m_simulationConfig.m_maxDuration + 1s);

		REQUIRE_THROWS_AS(l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Schedule entries must align with tick duration")
	{
		l_scenario.m_simulationConfig = { 100ms, 300ms };
		l_scenario.Schedule(150ms);

		REQUIRE_THROWS_AS(l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Schedule entries cannot have duplicates")
	{
		l_scenario.Schedule(100ms);
		l_scenario.Schedule(100ms);

		REQUIRE_THROWS_AS(l_scenario.Run(), std::invalid_argument);
	}

	SECTION("Schedule entries must be strictly increasing")
	{
		l_scenario.Schedule(200ms);
		l_scenario.Schedule(100ms);

		REQUIRE_THROWS_AS( l_scenario.Run(), std::invalid_argument);
	}

}


TEST_CASE("SimulationRunner processes time zero when maximum duration is zero", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	l_scenario.m_simulationConfig = { 100ms, 0ms };

	const SimulationResult l_result = l_scenario.Run();

	REQUIRE(l_result.m_terminationReason == TerminationReason::MAX_DURATION_REACHED);
	REQUIRE(l_result.m_records.size() == 1);

	CheckRecord(l_result.m_records.front(), {});
	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::SAFE);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::NONE);
}


TEST_CASE("SimulationRunner processes inhibited liftoff", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	l_scenario.m_simulationConfig = { 1s, 20s };

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule(2s, Command::LAUNCH);
	l_scenario.Schedule(4s, Command::NONE, VehicleEvent::INHIBIT_LIFTOFF);

	const SimulationResult l_result = l_scenario.Run();

	REQUIRE(l_result.m_terminationReason == TerminationReason::ABORT);
	REQUIRE(l_result.m_records.size() == 12);

	CheckRecord(l_result.m_records.back(), {
		.m_elapsed = 11s,
		.m_bEngineRunning = true,
		.m_state = FlightState::ABORT,
		.m_fault = FaultReason::LIFTOFF_TIMEOUT
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::ABORT);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::LIFTOFF_TIMEOUT);

}


TEST_CASE("SimulationRunner detects unexpected engine shutdown", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	l_scenario.m_simulationConfig = { 1s, 20s };

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule(2s, Command::LAUNCH);
	l_scenario.Schedule(8s, Command::NONE, VehicleEvent::FORCE_ENGINE_STOP);

	const SimulationResult l_result = l_scenario.Run();
	const auto& l_records = l_result.m_records;

	REQUIRE(l_result.m_terminationReason == TerminationReason::ABORT);
	REQUIRE(l_records.size() == 10);

	CheckRecord(l_records[8], {
		.m_elapsed = 8s,
		.m_event = VehicleEvent::FORCE_ENGINE_STOP,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_state = FlightState::POWERED_ASCENT,
		.m_bIgnitionCommand = true
		});

	CheckRecord(l_records.back(), {
		.m_elapsed = 9s,
		.m_bLiftoffDetected = true,
		.m_state = FlightState::ABORT,
		.m_fault = FaultReason::UNEXPECTED_ENGINE_SHUTDOWN
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::ABORT);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::UNEXPECTED_ENGINE_SHUTDOWN);
}


TEST_CASE("SimulationRunner detects engine cutoff timeout", "[SimulationRunner]")
{
	SimulationTestScenario l_scenario;
	l_scenario.m_simulationConfig = { 1s, 30s };

	l_scenario.Schedule(1s, Command::ARM);
	l_scenario.Schedule(2s, Command::LAUNCH);
	l_scenario.Schedule(12s, Command::NONE, VehicleEvent::INHIBIT_ENGINE_STOP);

	const SimulationResult l_result = l_scenario.Run();
	const auto& l_records = l_result.m_records;

	REQUIRE(l_result.m_terminationReason == TerminationReason::ABORT);
	REQUIRE(l_records.size() == 23);

	CheckRecord(l_records[12], {
		.m_elapsed = 12s,
		.m_event = VehicleEvent::INHIBIT_ENGINE_STOP,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::ENGINE_CUTOFF
		});

	CheckRecord(l_records.back(), {
		.m_elapsed = 22s,
		.m_bEngineRunning = true,
		.m_bLiftoffDetected = true,
		.m_bCutoffConditionMet = true,
		.m_state = FlightState::ABORT,
		.m_fault = FaultReason::ENGINE_CUTOFF_TIMEOUT
		});

	CHECK(l_result.m_finalControllerOutput.m_state == FlightState::ABORT);
	CHECK(l_result.m_finalControllerOutput.m_fault == FaultReason::ENGINE_CUTOFF_TIMEOUT);
}


TEST_CASE("SimulationRunner can only run once", "[SimulationRunner]")
{
	SimulationConfig l_simulationConfig{ 100ms, 0ms };
	ScenarioConfig l_scenarioConfig{};
	TimeoutConfig l_timeoutConfig{};

	FlightComputer l_fsw{ l_timeoutConfig };
	BasicVehicleModel l_vehicle{ TestHelpers::MakeBasicVehicleConfig() };

	SimulationRunner l_runner{
		l_simulationConfig,
		l_scenarioConfig,
		l_fsw,
		l_vehicle
	};

	const SimulationResult l_result = l_runner.Run();

	REQUIRE(l_result.m_terminationReason == TerminationReason::MAX_DURATION_REACHED);
	REQUIRE_THROWS_AS(l_runner.Run(), std::logic_error);
}
