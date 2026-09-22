#pragma once

#include "FlightTypes.h"

#include <vector>

namespace Simulation
{
	enum class VehicleEvent
	{
		NONE,
		INHIBIT_ENGINE_START,
		INHIBIT_ENGINE_STOP,
		INHIBIT_LIFTOFF,
		FORCE_ENGINE_STOP,
		FORCE_ENGINE_RESTART
	};

	enum class TerminationReason
	{
		NONE,
		MAX_DURATION_REACHED,
		ABORT,
		COAST_REACHED
	};

	struct SimulationScheduleEntry
	{
		FlightCore::Duration m_time;
		FlightCore::Command m_command = FlightCore::Command::NONE;
		VehicleEvent m_event = VehicleEvent::NONE;
	};

	struct SimulationRecord
	{
		FlightCore::TimePoint m_time;
		FlightCore::Command m_command = FlightCore::Command::NONE;
		VehicleEvent m_event = VehicleEvent::NONE;
		FlightCore::SensorSnapshot m_snapshot;
		FlightCore::ControllerOutput m_controllerOutput;
	};

	struct SimulationResult
	{
		std::vector<SimulationRecord> m_records;
		TerminationReason m_terminationReason;
		FlightCore::ControllerOutput m_finalControllerOutput;

		SimulationResult(std::vector<SimulationRecord> in_records, TerminationReason in_terminationReason, FlightCore::ControllerOutput in_finalControllerOutput)
			: m_records(std::move(in_records)), m_terminationReason(in_terminationReason), m_finalControllerOutput(in_finalControllerOutput)
		{}
	};

	struct SimulationConfig
	{
		FlightCore::Duration m_tickDuration = FlightCore::Duration::zero();
		FlightCore::Duration m_maxDuration = FlightCore::Duration::zero();
	};

	struct ScenarioConfig
	{
		std::vector<SimulationScheduleEntry> m_schedule;
		TerminationReason m_expectedTerminationReason = TerminationReason::NONE;
		FlightCore::FlightState m_expectedFinalState = FlightCore::FlightState::SAFE;
		FlightCore::FaultReason m_expectedFinalFault = FlightCore::FaultReason::NONE;
	};

}