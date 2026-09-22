add_library(SIL STATIC)

target_sources(
    SIL
    PRIVATE
        src/SIL.cpp
        src/sim/SimulationRunner.cpp
        src/sim/BasicVehicleModel.cpp
)

target_include_directories(
    SIL
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    SIL
    PUBLIC
        FlightCore
)