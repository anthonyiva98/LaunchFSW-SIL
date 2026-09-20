add_executable(LaunchFSWApp
    main.cpp
)

target_link_libraries(
    LaunchFSWApp
    PRIVATE
        FlightCore
        SIL
)