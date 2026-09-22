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
		{}

		SimulationResult Run();

	private:
		SimulationConfig m_SimulationConfig;
		ScenarioConfig m_ScenarioConfig;
		FlightCore::FlightComputer& m_FlightComputer;
		VehicleModelInterface& m_VehicleModel;
		FlightCore::TimePoint m_currentTime;

	};

}