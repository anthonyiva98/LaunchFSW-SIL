#pragma once

#include "FlightTypes.h"
#include "sim/SimulationTypes.h"
#include "sim/BasicVehicleModel.h"

#include <string>

struct DemoScenario
{
    std::string m_name;
    FlightCore::TimeoutConfig m_timeoutConfig;
    Simulation::SimulationConfig m_simulationConfig;
    Simulation::BasicVehicleModelConfig m_vehicleModelConfig;
    Simulation::ScenarioConfig m_scenarioConfig;
};

DemoScenario MakeNominalScenario();
DemoScenario MakeIgnitionTimeoutScenario();