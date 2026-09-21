
#include "FlightComputer.h"


namespace FlightCore
{
	ControllerOutput FlightComputer::Update(const UpdateInput& input)
	{
		if (m_state == FlightState::ABORT)
		{
			return ControllerOutput{ m_state, false, m_fault, input.m_command == Command::NONE ? CommandResult::NONE : CommandResult::REJECTED };
		}

		if (m_lastUpdateTime.has_value())
		{
			if (input.m_currentTime < m_lastUpdateTime.value())
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::NON_MONOTONIC_TIME;
				return ControllerOutput{ m_state, false, m_fault };
			}
			
			if (input.m_currentTime == m_lastUpdateTime.value())
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::TIME_NOT_ADVANCED;
				return ControllerOutput{ m_state, false, m_fault };
			}
		}

		m_lastUpdateTime = input.m_currentTime;
		CommandResult l_commandResult = CommandResult::NONE;

		switch (m_state)
		{
		case FlightState::SAFE:
			if (input.m_command == Command::ARM)
			{
				m_state = FlightState::ARMED;
				l_commandResult = CommandResult::ACCEPTED;
			}
			else if (input.m_command == Command::LAUNCH)
			{
				l_commandResult = CommandResult::REJECTED;
			}
			break;
		case FlightState::ARMED:
			if (input.m_command == Command::LAUNCH)
			{
				m_state = FlightState::IGNITION;
				m_ignitionStartTime = input.m_currentTime;
				l_commandResult = CommandResult::ACCEPTED;
			}
			else if (input.m_command == Command::ARM)
			{
				l_commandResult = CommandResult::REJECTED;
			}
			break;
		case FlightState::IGNITION:
		{
			if (input.m_command != Command::NONE)
			{
				l_commandResult = CommandResult::REJECTED;
			}

			Duration elapsed = input.m_currentTime - m_ignitionStartTime.value();

			if (input.m_snapshot.m_bEngineRunning && elapsed <= m_timeoutConfig.m_ignitionTimeout)
			{
				m_state = FlightState::THRUST_BUILDUP;
				m_thrustBuildupStartTime = input.m_currentTime;
			}
			else if (elapsed >= m_timeoutConfig.m_ignitionTimeout)
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::IGNITION_TIMEOUT;
			}
			break;
		}
		case FlightState::THRUST_BUILDUP:
		{
			if (input.m_command != Command::NONE)
			{
				l_commandResult = CommandResult::REJECTED;
			}

			if (!input.m_snapshot.m_bEngineRunning)
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::UNEXPECTED_ENGINE_SHUTDOWN;
				break;
			}

			Duration elapsed = input.m_currentTime - m_thrustBuildupStartTime.value();

			if (input.m_snapshot.m_bLiftoffDetected && elapsed <= m_timeoutConfig.m_liftoffTimeout)
			{
				m_state = FlightState::POWERED_ASCENT;
			}
			else if (elapsed >= m_timeoutConfig.m_liftoffTimeout)
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::LIFTOFF_TIMEOUT;
			}
			break;
		}
		case  FlightState::POWERED_ASCENT:
			if (input.m_command != Command::NONE)
			{
				l_commandResult = CommandResult::REJECTED;
			}

			if (!input.m_snapshot.m_bEngineRunning)
			{
				m_state = FlightState::ABORT;
				m_fault = FaultReason::UNEXPECTED_ENGINE_SHUTDOWN;
				break;
			}
			break;
		}

		bool l_bIgnitionCommand = (m_state == FlightState::IGNITION || m_state == FlightState::THRUST_BUILDUP || m_state == FlightState::POWERED_ASCENT);

		return ControllerOutput{ m_state, l_bIgnitionCommand, m_fault, l_commandResult };
	}
}
