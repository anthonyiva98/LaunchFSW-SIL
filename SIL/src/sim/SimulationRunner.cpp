
#include <sim/SimulationRunner.h>

#include <optional>
#include <stdexcept>
#include <cstddef>

using namespace FlightCore;

namespace Simulation
{
	void SimulationRunner::ValidateConfig() const
	{
		const Duration zero = Duration::zero();

		if (m_SimulationConfig.m_tickDuration <= zero)
			throw std::invalid_argument("m_SimulationConfig.m_tickDuration must be greater than 0.");

		if (m_SimulationConfig.m_maxDuration < zero)
			throw std::invalid_argument("m_SimulationConfig.m_maxDuration must be greater than or equal to 0.");

		if (m_SimulationConfig.m_maxDuration % m_SimulationConfig.m_tickDuration != zero)
			throw std::invalid_argument("Maximum duration must be aligned with tick duration.");

		std::optional<Duration> prevTime;

		for (const SimulationScheduleEntry& entry : m_ScenarioConfig.m_schedule)
		{
			if (entry.m_time < zero)
				throw std::invalid_argument("Simulation schedule entry cannot have negative time.");

			if (entry.m_time > m_SimulationConfig.m_maxDuration)
				throw std::invalid_argument("Simulation schedule entry cannot be after m_SimulationConfig.m_maxDuration.");

			if (entry.m_time % m_SimulationConfig.m_tickDuration != zero)
				throw std::invalid_argument("Simulation schedule entry is not aligned with m_SimulationConfig.m_tickDuration.");

			if (prevTime.has_value())
			{
				if (entry.m_time == prevTime.value())
					throw std::invalid_argument("Simulation schedule entry has duplicate.");
				else if (entry.m_time < prevTime.value())
					throw std::invalid_argument("Simulation schedule entry not in order.");
			}

			prevTime = entry.m_time;
		}

	}

	SimulationRecord SimulationRunner::CreateRecord(const UpdateInput& input, const ControllerOutput& output, VehicleEvent event) const
	{
		SimulationRecord record;

		record.m_time = input.m_currentTime;
		record.m_command = input.m_command;
		record.m_snapshot = input.m_snapshot;
		record.m_controllerOutput = output;
		record.m_event = event;

		return record;
	}

	SimulationResult SimulationRunner::Run()
	{
		if (m_bHasRun)
			throw std::logic_error("SimulationRunner may only be run once.");

		m_bHasRun = true;

		std::vector<SimulationRecord> l_records;
		TerminationReason l_terminationReason = TerminationReason::NONE;
		ControllerOutput l_output;

		m_currentTime = TimePoint{};
		SensorSnapshot l_sensorSnapshot{};

		Duration l_elapsed = Duration::zero();
		std::size_t l_iNextScheduleIndex = 0;

		while (true)
		{
			Command l_command = Command::NONE;
			VehicleEvent l_event = VehicleEvent::NONE;

			if (l_iNextScheduleIndex < m_ScenarioConfig.m_schedule.size() && m_ScenarioConfig.m_schedule[l_iNextScheduleIndex].m_time == l_elapsed)
			{
				l_command = m_ScenarioConfig.m_schedule[l_iNextScheduleIndex].m_command;
				l_event = m_ScenarioConfig.m_schedule[l_iNextScheduleIndex].m_event;
				++l_iNextScheduleIndex;
			}

			// The snapshot consumed at T was produced by vehicle behavior completed during the preceding interval.
			UpdateInput l_input = UpdateInput(l_command, l_sensorSnapshot, m_currentTime);
			l_output = m_FlightComputer.Update(l_input);


			// The current update is always recorded before evaluating termination.
			l_records.push_back(CreateRecord(l_input, l_output, l_event));

			// Controller terminal states take priority when maximum duration occurs simultaneously.
			if (l_output.m_state == FlightState::ABORT)
			{
				l_terminationReason = TerminationReason::ABORT;
				break;
			}
			else if (l_output.m_state == FlightState::COAST && m_ScenarioConfig.m_coastTerminationPolicy == CoastTerminationPolicy::TERMINATE_ON_COAST)
			{
				l_terminationReason = TerminationReason::COAST_REACHED;
				break;
			}

			if (l_elapsed >= m_SimulationConfig.m_maxDuration)
			{
				l_terminationReason = TerminationReason::MAX_DURATION_REACHED;
				break;
			}

			m_VehicleModel.ApplyEvent(l_event);
			l_sensorSnapshot = m_VehicleModel.Step(l_output.m_bIgnitionCommand, m_SimulationConfig.m_tickDuration);

			m_currentTime += m_SimulationConfig.m_tickDuration;
			l_elapsed += m_SimulationConfig.m_tickDuration;
		}

		return SimulationResult(std::move(l_records), l_terminationReason, l_output);
	}
}
