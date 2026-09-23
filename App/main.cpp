
#include "DemoScenarios.h"
#include "ConsoleReporter.h"

#include "FlightComputer.h"
#include "sim/SimulationTypes.h"
#include "sim/SimulationRunner.h"
#include "sim/BasicVehicleModel.h"

#include <iostream>
#include <exception>
#include <array>
#include <string_view>

namespace
{
    using ScenarioFactory = DemoScenario(*)();

    struct ScenarioOption
    {
        std::string_view m_key;
        std::string_view m_description;
        ScenarioFactory m_factory;
    };

    const std::array<ScenarioOption, 2> g_scenarioOptions{ {
    {
        "nominal",
        "Nominal launch through coast",
        &MakeNominalScenario
    },
    {
        "ignition-timeout",
        "Engine start inhibited, causing an ignition timeout",
        &MakeIgnitionTimeoutScenario
    }
    } };

    const ScenarioOption* FindScenario(std::string_view key)
    {
        for (const ScenarioOption& option : g_scenarioOptions)
        {
            if (option.m_key == key)
            {
                return &option;
            }
        }

        return nullptr;
    }

    void PrintHelp(std::ostream& output, std::string_view executableName)
    {
        output << "Usage: " << executableName << " [scenario|all|help]\n\n";

        output << "Scenarios: key | description\n";

        for (const ScenarioOption& option : g_scenarioOptions)
        {
            output << "  " << option.m_key << " | " << option.m_description << '\n';
        }

        output << "\nOther options:\n"
            << "  all   Run all scenarios\n"
            << "  help  Show this help\n"
            << "  -     With no argument, the nominal scenario runs.\n";
    }

    void RunScenario(const DemoScenario& in_scenario)
    {
        FlightCore::FlightComputer l_fsw = FlightCore::FlightComputer(in_scenario.m_timeoutConfig);
        Simulation::BasicVehicleModel l_bvm = Simulation::BasicVehicleModel(in_scenario.m_vehicleModelConfig);
        Simulation::SimulationRunner l_simRunner = Simulation::SimulationRunner(in_scenario.m_simulationConfig, in_scenario.m_scenarioConfig, l_fsw, l_bvm);

        Simulation::SimulationResult l_result = l_simRunner.Run();

        PrintSimulationReport(std::cout, in_scenario.m_name, l_result);
    }
}

int main(int argc, char* argv[])
{

    try
    {
        std::cout << "LaunchFSW-SIL\n";

        if (argc > 2)
        {
            std::cerr << "Error: expected at most one argument.\n\n";
            PrintHelp(std::cerr, argv[0]);
            return 1;
        }

        std::string_view selection = argc == 1 ? "nominal" : argv[1];

        if (selection == "help" || selection == "--help" || selection == "-h")
        {
            PrintHelp(std::cout, argv[0]);
        }
        else if (selection == "all")
        {
            for (const ScenarioOption& option : g_scenarioOptions)
            {
                RunScenario(option.m_factory());
            }
        }
        else
        {
            const ScenarioOption* scenario = FindScenario(selection);
            if (scenario == nullptr)
            {
                std::cerr << "Error: unknown scenario '" << selection << "'.\n\n";
                PrintHelp(std::cerr, argv[0]);
                return 1;
            }

            RunScenario(scenario->m_factory());
        }

    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
