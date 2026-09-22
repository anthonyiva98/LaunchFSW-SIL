#pragma once

#include <chrono>

namespace FlightCore
{
	using TimePoint = std::chrono::steady_clock::time_point;
	using Duration = std::chrono::steady_clock::duration;

	enum class FlightState
	{
		SAFE,
		ARMED,
		IGNITION,
		THRUST_BUILDUP,
		POWERED_ASCENT,
		ENGINE_CUTOFF,
		COAST,
		ABORT
	};

	enum class Command
	{
		NONE,
		ARM,
		LAUNCH
	};

	enum class CommandResult
	{
		NONE,
		ACCEPTED,
		REJECTED
	};

	enum class FaultReason
	{
		NONE,
		NON_MONOTONIC_TIME,
		TIME_NOT_ADVANCED,
		IGNITION_TIMEOUT,
		LIFTOFF_TIMEOUT,
		UNEXPECTED_ENGINE_SHUTDOWN,
		ENGINE_CUTOFF_TIMEOUT,
		UNEXPECTED_ENGINE_START
	};

	struct SensorSnapshot
	{
		bool m_bEngineRunning = false;
		bool m_bLiftoffDetected = false;
		bool m_bCutoffConditionMet = false;
	};

	struct ControllerOutput
	{
		FlightState m_state = FlightState::SAFE;
		bool m_bIgnitionCommand = false;
		FaultReason m_fault = FaultReason::NONE;
		CommandResult m_commandResult = CommandResult::NONE;
	};

	struct TimeoutConfig
	{
		std::chrono::milliseconds m_ignitionTimeout = std::chrono::milliseconds(3000);
		std::chrono::milliseconds m_liftoffTimeout = std::chrono::milliseconds(7000);
		std::chrono::milliseconds m_engineCutoffTimeout = std::chrono::milliseconds(10000);
	};

	struct UpdateInput
	{
		Command m_command;
		SensorSnapshot m_snapshot;
		TimePoint m_currentTime;

		UpdateInput(Command command, SensorSnapshot snapshot, TimePoint currentTime)
			: m_command(command), m_snapshot(snapshot), m_currentTime(currentTime) {}
	};
}

