

#include "DemoScenarios.h"

#include <chrono>

using namespace std::chrono_literals;

DemoScenario MakeNominalScenario()
{
	DemoScenario out_DemoScenario;

    out_DemoScenario.m_name = "Nominal launch";

    out_DemoScenario.m_simulationConfig = Simulation::SimulationConfig{ 1s, 20s };
    out_DemoScenario.m_timeoutConfig = FlightCore::TimeoutConfig{ 3s, 7s, 10s };
    out_DemoScenario.m_vehicleModelConfig = Simulation::BasicVehicleModelConfig{ 2s, 3s, 5s, 4s };
    out_DemoScenario.m_scenarioConfig = {
        {{1s, FlightCore::Command::ARM},
        {2s, FlightCore::Command::LAUNCH}},
        Simulation::CoastTerminationPolicy::TERMINATE_ON_COAST
    };

	return out_DemoScenario;
}

DemoScenario MakeIgnitionTimeoutScenario()
{
    DemoScenario out_DemoScenario = MakeNominalScenario();

    out_DemoScenario.m_name = "Ignition timeout";
    out_DemoScenario.m_scenarioConfig.m_schedule = {
        {1s, FlightCore::Command::ARM},
        {2s, FlightCore::Command::LAUNCH, Simulation::VehicleEvent::INHIBIT_ENGINE_START}
    };

    return out_DemoScenario;
}