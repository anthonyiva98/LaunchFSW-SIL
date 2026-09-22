#pragma once

#include "FlightTypes.h"

#include <optional>

namespace FlightCore
{
	class FlightComputer
	{
	public:
		explicit FlightComputer(const TimeoutConfig& timeoutConfig): m_timeoutConfig(timeoutConfig) {}
		ControllerOutput Update(const UpdateInput& input);

	private:
		TimeoutConfig m_timeoutConfig;
		std::optional<TimePoint> m_lastUpdateTime{};
		std::optional<TimePoint> m_ignitionStartTime{};
		std::optional<TimePoint> m_thrustBuildupStartTime{};
		std::optional<TimePoint> m_engineCutoffStartTime{};
		FlightState m_state = FlightState::SAFE;
		FaultReason m_fault = FaultReason::NONE;
	};
}