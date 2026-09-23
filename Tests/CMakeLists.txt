find_package(Catch2 3 REQUIRED)

add_executable(LaunchFSWTests
    FlightComputerTests.cpp
    BasicVehicleModelTests.cpp
    SimulationRunnerTests.cpp
)

target_link_libraries(
    LaunchFSWTests
    PRIVATE
        FlightCore
        SIL
        Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(LaunchFSWTests)

add_test(
    NAME LaunchFSWApp.Nominal
    COMMAND LaunchFSWApp
)

add_test(
    NAME LaunchFSWApp.IgnitionTimeout
    COMMAND LaunchFSWApp ignition-timeout
)

add_test(
    NAME LaunchFSWApp.Help
    COMMAND LaunchFSWApp help
)

add_test(
    NAME LaunchFSWApp.InvalidScenario
    COMMAND LaunchFSWApp invalid-scenario
)

set_tests_properties(
    LaunchFSWApp.Nominal
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Sequence outcome: COMPLETED"
)

set_tests_properties(
    LaunchFSWApp.IgnitionTimeout
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Sequence outcome: ABORTED"
)

set_tests_properties(
    LaunchFSWApp.Help
    PROPERTIES
        PASS_REGULAR_EXPRESSION "Usage:"
)

set_tests_properties(
    LaunchFSWApp.InvalidScenario
    PROPERTIES
        WILL_FAIL TRUE
)
