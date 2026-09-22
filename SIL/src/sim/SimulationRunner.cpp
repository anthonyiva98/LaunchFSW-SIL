
#include <sim/SimulationRunner.h>

namespace Simulation
{
	SimulationResult SimulationRunner::Run()
	{

		std::vector<SimulationRecord> l_records;
		TerminationReason l_terminationReason;
		FlightCore::ControllerOutput l_finalControllerOutput;


		return SimulationResult(l_records, l_terminationReason, l_finalControllerOutput);
	}
}