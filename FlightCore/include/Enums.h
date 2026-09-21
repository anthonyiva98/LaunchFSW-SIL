#pragma once

namespace FlightCore
{
	enum class FlightState
	{
		SAFE,
		ARMED,
		IGNITION,
		THRUST_BUILDUP,
		POWERED_ASCENT,
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
		UNEXPECTED_ENGINE_SHUTDOWN
	};
}