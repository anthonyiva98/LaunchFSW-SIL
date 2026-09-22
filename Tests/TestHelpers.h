#pragma once

#include "FlightTypes.h"
#include "sim/BasicVehicleModel.h"

#include <chrono>

namespace TestHelpers
{
    inline Simulation::BasicVehicleModelConfig MakeBasicVehicleConfig(
        FlightCore::Duration engineStartDelay = std::chrono::seconds{ 2 },
        FlightCore::Duration liftoffDelay = std::chrono::seconds{ 3 },
        FlightCore::Duration poweredAscentDuration = std::chrono::seconds{ 5 },
        FlightCore::Duration engineStopDelay = std::chrono::seconds{ 4 })
    {
        Simulation::BasicVehicleModelConfig config;
        config.m_engineStartDelay = engineStartDelay;
        config.m_liftoffDelay = liftoffDelay;
        config.m_poweredAscentDuration = poweredAscentDuration;
        config.m_engineStopDelay = engineStopDelay;
        return config;
    }
}