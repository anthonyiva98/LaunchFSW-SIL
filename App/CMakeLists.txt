add_executable(LaunchFSWApp
    main.cpp
)

target_sources(
    LaunchFSWApp
    PRIVATE
        DemoScenarios.cpp
        ConsoleReporter.cpp
)

target_link_libraries(
    LaunchFSWApp
    PRIVATE
        FlightCore
        SIL
)