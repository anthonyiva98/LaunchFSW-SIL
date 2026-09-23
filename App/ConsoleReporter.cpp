
#include "ConsoleReporter.h"

#include "FlightTypes.h"
#include "sim/SimulationTypes.h"

#include <vector>
#include <chrono>
#include <ostream>
#include <string_view>

#include <algorithm>
#include <iomanip>
#include <string>

namespace
{
    struct ReportRow
    {
        std::string m_time;
        std::string m_command;
        std::string m_event;
        std::string m_engine;
        std::string m_liftoff;
        std::string m_cutoff;
        std::string m_state;
        std::string m_ignition;
        std::string m_cmd_result;
        std::string m_fault;
    };

    std::string_view CommandToString(FlightCore::Command in_command)
    {
        switch (in_command)
        {
        case FlightCore::Command::ARM:
            return "ARM";
        case FlightCore::Command::LAUNCH:
            return "LAUNCH";
        case FlightCore::Command::NONE:
            return "NONE";
        default:
            break;
        }

        return "UNKNOWN";
    }

    std::string_view EventToString(Simulation::VehicleEvent in_event)
    {
        switch (in_event)
        {
        case Simulation::VehicleEvent::INHIBIT_ENGINE_START:
            return "INHIBIT_ENGINE_START";
        case Simulation::VehicleEvent::INHIBIT_ENGINE_STOP:
            return "INHIBIT_ENGINE_STOP";
        case Simulation::VehicleEvent::INHIBIT_LIFTOFF:
            return "INHIBIT_LIFTOFF";
        case Simulation::VehicleEvent::FORCE_ENGINE_STOP:
            return "FORCE_ENGINE_STOP";
        case Simulation::VehicleEvent::FORCE_ENGINE_RESTART:
            return "FORCE_ENGINE_RESTART";
        case Simulation::VehicleEvent::NONE:
            return "NONE";
        default:
            break;
        }

        return "UNKNOWN";
    }

    std::string_view FlightStateToString(FlightCore::FlightState in_state)
    {
        switch (in_state)
        {
        case FlightCore::FlightState::SAFE:
            return "SAFE";
        case FlightCore::FlightState::ARMED:
            return "ARMED";
        case FlightCore::FlightState::IGNITION:
            return "IGNITION";
        case FlightCore::FlightState::THRUST_BUILDUP:
            return "THRUST_BUILDUP";
        case FlightCore::FlightState::POWERED_ASCENT:
            return "POWERED_ASCENT";
        case FlightCore::FlightState::ENGINE_CUTOFF:
            return "ENGINE_CUTOFF";
        case FlightCore::FlightState::COAST:
            return "COAST";
        case FlightCore::FlightState::ABORT:
            return "ABORT";
        default:
            break;
        }

        return "UNKNOWN";
    }

    std::string_view CmdResultToString(FlightCore::CommandResult in_cmdResult)
    {
        switch (in_cmdResult)
        {
        case FlightCore::CommandResult::ACCEPTED:
            return "ACCEPTED";
        case FlightCore::CommandResult::REJECTED:
            return "REJECTED";
        case FlightCore::CommandResult::NONE:
            return "NONE";
        default:
            break;
        }

        return "UNKNOWN";
    }

    std::string_view FaultToString(FlightCore::FaultReason in_fault)
    {
        switch (in_fault)
        {
        case FlightCore::FaultReason::ENGINE_CUTOFF_TIMEOUT:
            return "ENGINE_CUTOFF_TIMEOUT";
        case FlightCore::FaultReason::IGNITION_TIMEOUT:
            return "IGNITION_TIMEOUT";
        case FlightCore::FaultReason::LIFTOFF_TIMEOUT:
            return "LIFTOFF_TIMEOUT";
        case FlightCore::FaultReason::NON_MONOTONIC_TIME:
            return "NON_MONOTONIC_TIME";
        case FlightCore::FaultReason::TIME_NOT_ADVANCED:
            return "TIME_NOT_ADVANCED";
        case FlightCore::FaultReason::UNEXPECTED_ENGINE_SHUTDOWN:
            return "UNEXPECTED_ENGINE_SHUTDOWN";
        case FlightCore::FaultReason::UNEXPECTED_ENGINE_START:
            return "UNEXPECTED_ENGINE_START";
        case FlightCore::FaultReason::NONE:
            return "NONE";
        default:
            break;
        }

        return "UNKNOWN";
    }

    std::string_view TerminationReasonToString(Simulation::TerminationReason in_terminationReason)
    {
        switch (in_terminationReason)
        {
        case Simulation::TerminationReason::ABORT:
            return "ABORT";
        case Simulation::TerminationReason::COAST_REACHED:
            return "COAST_REACHED";
        case Simulation::TerminationReason::MAX_DURATION_REACHED:
            return "MAX_DURATION_REACHED";
        case Simulation::TerminationReason::NONE:
            return "NONE";
        default:
            break;
        }

        return "UNKNOWN";
    }

    void PrintSeparator( std::ostream& output, const std::vector<std::size_t>& widths)
    {
        for (const std::size_t width : widths)
        {
            output << "|-" << std::string(width, '-') << '-';
        }

        output << "|\n";
    }

    void PrintRow(std::ostream& output, const std::vector<std::size_t>& widths, const ReportRow& row)
    {
        output << "| ";
        output << std::left << std::setw(static_cast<int>(widths[0])) << row.m_time;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[1])) << row.m_command;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[2])) << row.m_event;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[3])) << row.m_engine;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[4])) << row.m_liftoff;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[5])) << row.m_cutoff;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[6])) << row.m_state;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[7])) << row.m_ignition;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[8])) << row.m_cmd_result;
        output << " | ";
        output << std::left << std::setw(static_cast<int>(widths[9])) << row.m_fault;
        output << " |\n";
    }

    void PrintTable(std::ostream& output, const std::vector<ReportRow>& rows)
    {
        // Assumes row[0] is headers. Inserts formatting between each column entry, and after header row
        const auto originalFlags = output.flags();

        // Set column widths
        std::vector<std::size_t> widths(10, 0);
        for (const auto& row : rows)
        {
            widths[0] = std::max(widths[0], row.m_time.size());
            widths[1] = std::max(widths[1], row.m_command.size());
            widths[2] = std::max(widths[2], row.m_event.size());
            widths[3] = std::max(widths[3], row.m_engine.size());
            widths[4] = std::max(widths[4], row.m_liftoff.size());
            widths[5] = std::max(widths[5], row.m_cutoff.size());
            widths[6] = std::max(widths[6], row.m_state.size());
            widths[7] = std::max(widths[7], row.m_ignition.size());
            widths[8] = std::max(widths[8], row.m_cmd_result.size());
            widths[9] = std::max(widths[9], row.m_fault.size());
        }


        // Print table header
        PrintSeparator(output, widths);
        PrintRow(output, widths, rows.front());
        PrintSeparator(output, widths);

        // Print table rows
        for (std::size_t i = 1; i < rows.size(); ++i)
        {
            PrintRow(output, widths, rows[i]);
        }

        PrintSeparator(output, widths);
        output << "\n";
        output.flags(originalFlags);
    }

    void PrintRecords(std::ostream& output, const std::vector<Simulation::SimulationRecord>& in_records)
    {
        std::vector<ReportRow> rows;
        rows.reserve(in_records.size() + 1);

        ReportRow header;

        header.m_time = "t (ms)";
        header.m_command = "cmd";
        header.m_event = "event";
        header.m_engine = "eng";
        header.m_liftoff = "lift";
        header.m_cutoff = "cut";
        header.m_state = "state";
        header.m_ignition = "ign";
        header.m_cmd_result = "result";
        header.m_fault = "fault";

        rows.push_back(header);

        FlightCore::TimePoint l_firstRecordTime = in_records[0].m_time;

        for (const auto& l_record : in_records)
        {
            ReportRow row;

            row.m_time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(l_record.m_time - l_firstRecordTime).count());
            row.m_command = CommandToString(l_record.m_command);
            row.m_event = EventToString(l_record.m_event);
            row.m_engine = l_record.m_snapshot.m_bEngineRunning ? "Y" : "N";
            row.m_liftoff = l_record.m_snapshot.m_bLiftoffDetected ? "Y" : "N";
            row.m_cutoff = l_record.m_snapshot.m_bCutoffConditionMet ? "Y" : "N";
            row.m_state = FlightStateToString(l_record.m_controllerOutput.m_state);
            row.m_ignition = l_record.m_controllerOutput.m_bIgnitionCommand ? "Y" : "N";
            row.m_cmd_result = CmdResultToString(l_record.m_controllerOutput.m_commandResult);
            row.m_fault = FaultToString(l_record.m_controllerOutput.m_fault);

            rows.push_back(row);
        }

        PrintTable(output, rows);
    }
}


void PrintSimulationReport(
    std::ostream& output,
    std::string_view scenarioName,
    const Simulation::SimulationResult& result)
{
    output << "\n----------------------------\n\n";
    output << "Scenario: " << scenarioName << "\n\n";

    if (result.m_records.size() == 0)
    {
        output << "0 records...\n\n";
    }
    else
    {
        PrintRecords(output, result.m_records);
    }

    output << "Termination: " << TerminationReasonToString(result.m_terminationReason) << '\n';
    output << "Final State: " << FlightStateToString(result.m_finalControllerOutput.m_state) << '\n';
    output << "Final fault: " << FaultToString(result.m_finalControllerOutput.m_fault) << '\n';

    output << "Sequence outcome: ";
    if (result.m_terminationReason == Simulation::TerminationReason::ABORT)
        output << "ABORTED" << '\n';
    else if (result.m_terminationReason == Simulation::TerminationReason::COAST_REACHED)
        output << "COMPLETED" << '\n';
    else if (result.m_terminationReason == Simulation::TerminationReason::MAX_DURATION_REACHED)
        output << "INCOMPLETE" << '\n';
    else
        output << "UNKNOWN" << '\n';
}
