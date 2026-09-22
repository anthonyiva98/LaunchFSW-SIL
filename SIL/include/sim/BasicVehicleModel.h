#pragma once

#include "SimulationTypes.h"
#include "VehicleModelInterface.h"
#include "FlightTypes.h"

#include <optional>


namespace Simulation 
{

	struct BasicVehicleModelConfig
	{
		FlightCore::Duration m_engineStartDelay = FlightCore::Duration::zero();
		FlightCore::Duration m_liftoffDelay = FlightCore::Duration::zero();
		FlightCore::Duration m_poweredAscentDuration = FlightCore::Duration::zero();
		FlightCore::Duration m_engineStopDelay = FlightCore::Duration::zero();
	};

	class BasicVehicleModel final : public VehicleModelInterface
	{
	public:

		explicit BasicVehicleModel(const BasicVehicleModelConfig& config) : m_config(config) {}

		void ApplyEvent(VehicleEvent event) override;
		FlightCore::SensorSnapshot Step( bool ignitionCommand, FlightCore::Duration deltaTime) override;

	private:
		BasicVehicleModelConfig m_config;

		std::optional<FlightCore::Duration> m_engineStartElapsed;
		std::optional<FlightCore::Duration> m_liftoffElapsed;
		std::optional<FlightCore::Duration> m_poweredFlightElapsed;
		std::optional<FlightCore::Duration> m_engineStopElapsed;

		bool m_bPreviousIgnitionCommand = false;

		bool m_bInhibitEngineStart = false;
		bool m_bInhibitLiftoff = false;
		bool m_bInhibitEngineStop = false;

		bool m_bEngineRunning = false;
		bool m_bLiftoffDetected = false;
		bool m_bCutoffConditionMet = false;

		VehicleEvent m_event = VehicleEvent::NONE;

		void ProcessPendingEvent();
		void AdvanceActiveTimers(FlightCore::Duration deltaTime);
		void ResolveExpiredTimers();
		void ProcessIgnitionCommandEdges(bool ignitionCommand);
		const FlightCore::SensorSnapshot BuildSensorSnapshot();
	};
}