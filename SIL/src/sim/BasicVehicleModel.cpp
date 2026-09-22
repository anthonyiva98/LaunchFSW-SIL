
#include <sim/BasicVehicleModel.h>

namespace Simulation
{
	void BasicVehicleModel::ApplyEvent(VehicleEvent event)
	{
		m_event = event;
	}

	void BasicVehicleModel::ProcessPendingEvent()
	{
		switch (m_event)
		{
		case VehicleEvent::INHIBIT_ENGINE_START:
			m_bInhibitEngineStart = true;
			break;
		case VehicleEvent::INHIBIT_ENGINE_STOP:
			m_bInhibitEngineStop = true;
			break;
		case VehicleEvent::INHIBIT_LIFTOFF:
			m_bInhibitLiftoff = true;
			break;
		case VehicleEvent::FORCE_ENGINE_STOP:
			m_bEngineRunning = false;
			m_engineStopElapsed.reset();
			m_engineStartElapsed.reset();
			break;
		case VehicleEvent::FORCE_ENGINE_RESTART:
			m_bEngineRunning = true;
			m_engineStopElapsed.reset();
			break;
		default:
			break;
		}

		m_event = VehicleEvent::NONE;
	}

	void BasicVehicleModel::AdvanceActiveTimers(FlightCore::Duration deltaTime)
	{
		if (m_engineStartElapsed.has_value())
		{
			m_engineStartElapsed = m_engineStartElapsed.value() + deltaTime;
		}

		if (m_liftoffElapsed.has_value())
		{
			m_liftoffElapsed = m_liftoffElapsed.value() + deltaTime;
		}

		if (m_poweredFlightElapsed.has_value())
		{
			m_poweredFlightElapsed = m_poweredFlightElapsed.value() + deltaTime;
		}

		if (m_engineStopElapsed.has_value())
		{
			m_engineStopElapsed = m_engineStopElapsed.value() + deltaTime;
		}
	}

	void BasicVehicleModel::ResolveExpiredTimers()
	{
		// Capture expiry state before applying transitions so a timer started by this update cannot also expire until the next vehicle update.
		const bool engineStartExpired = m_engineStartElapsed.has_value() && m_engineStartElapsed.value() >= m_config.m_engineStartDelay;
		const bool liftoffExpired = m_liftoffElapsed.has_value() && m_liftoffElapsed.value() >= m_config.m_liftoffDelay;
		const bool poweredAscentCompleted = m_poweredFlightElapsed.has_value() && m_poweredFlightElapsed.value() >= m_config.m_poweredAscentDuration;
		const bool engineStopExpired = m_engineStopElapsed.has_value() && m_engineStopElapsed.value() >= m_config.m_engineStopDelay;

		if (engineStartExpired)
		{
			m_engineStartElapsed.reset();

			if (!m_bInhibitEngineStart)
			{
				m_bEngineRunning = true;
				m_liftoffElapsed = FlightCore::Duration::zero();
			}
		}

		if (liftoffExpired)
		{
			m_liftoffElapsed.reset();

			if (!m_bInhibitLiftoff)
			{
				m_bLiftoffDetected = true;
				m_poweredFlightElapsed = FlightCore::Duration::zero();
			}

		}

		if (poweredAscentCompleted)
		{
			m_poweredFlightElapsed.reset();
			m_bCutoffConditionMet = true;
		}

		if (engineStopExpired)
		{
			m_engineStopElapsed.reset();

			if (!m_bInhibitEngineStop)
				m_bEngineRunning = false;
		}
	}

	void BasicVehicleModel::ProcessIgnitionCommandEdges(bool ignitionCommand)
	{
		const bool ignitionRising = !m_bPreviousIgnitionCommand && ignitionCommand;
		const bool ignitionFalling = m_bPreviousIgnitionCommand && !ignitionCommand;

		if (ignitionRising)
		{
			m_engineStartElapsed = FlightCore::Duration::zero();
			m_engineStopElapsed.reset();
		}

		if (ignitionFalling)
		{
			m_engineStartElapsed.reset();

			if (m_bEngineRunning)
			{
				m_engineStopElapsed = FlightCore::Duration::zero();
			}
		}
	}

	FlightCore::SensorSnapshot BasicVehicleModel::BuildSensorSnapshot() const
	{
		FlightCore::SensorSnapshot out_snapshot;
		out_snapshot.m_bEngineRunning = m_bEngineRunning;
		out_snapshot.m_bLiftoffDetected = m_bLiftoffDetected;
		out_snapshot.m_bCutoffConditionMet = m_bCutoffConditionMet;
		return out_snapshot;
	}

	FlightCore::SensorSnapshot BasicVehicleModel::Step(bool ignitionCommand, FlightCore::Duration deltaTime)
	{
		// Events and command edges apply at the start of the interval.
		ProcessPendingEvent();
		ProcessIgnitionCommandEdges(ignitionCommand);

		// Active timers then advance by deltaTime.
		AdvanceActiveTimers(deltaTime);
		ResolveExpiredTimers();
		m_bPreviousIgnitionCommand = ignitionCommand;

		// Returned snapshot represents the vehicle state at the end of that interval.
		return BuildSensorSnapshot();
	}

}