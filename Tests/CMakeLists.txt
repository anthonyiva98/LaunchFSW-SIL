find_package(Catch2 3 REQUIRED)

add_executable(LaunchFSWTests
    FlightComputerTests.cpp
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