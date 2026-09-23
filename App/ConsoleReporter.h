#pragma once

#include "sim/SimulationTypes.h"

#include <iosfwd>
#include <string_view>


void PrintSimulationReport(
    std::ostream& output,
    std::string_view scenarioName,
    const Simulation::SimulationResult& result);