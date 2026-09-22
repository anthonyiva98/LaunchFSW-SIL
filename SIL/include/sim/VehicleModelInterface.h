#pragma once

#include "SimulationTypes.h"
#include "FlightTypes.h"


namespace Simulation
{

    class VehicleModelInterface
    {
    public:
        virtual ~VehicleModelInterface() = default;

        virtual void ApplyEvent(VehicleEvent event) = 0;
        virtual FlightCore::SensorSnapshot Step( bool ignitionCommand, FlightCore::Duration deltaTime) = 0;
    };

}