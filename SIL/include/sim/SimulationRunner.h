#pragma once

#include "SimulationTypes.h"
#include "VehicleModelInterface.h"

#include "FlightComputer.h"

namespace Simulation
{

	class SimulationRunner
	{

	public:
		explicit SimulationRunner(const SimulationConfig& config, const ScenarioConfig& scenarioConfig, FlightCore::FlightComputer& fsw, VehicleModelInterface& vehicleModel)
			: m_SimulationConfig(config), m_ScenarioConfig(scenarioConfig), m_FlightComputer(fsw), m_VehicleModel(vehicleModel)
		{
			ValidateConfig();
		}

		SimulationResult Run();

	private:
		SimulationConfig m_SimulationConfig;
		ScenarioConfig m_ScenarioConfig;
		FlightCore::FlightComputer& m_FlightComputer;
		VehicleModelInterface& m_VehicleModel;
		FlightCore::TimePoint m_currentTime;

		bool m_bHasRun = false;

		void ValidateConfig() const;
		SimulationRecord CreateRecord(const FlightCore::UpdateInput& input, const FlightCore::ControllerOutput& output, VehicleEvent event) const;
	};

}
