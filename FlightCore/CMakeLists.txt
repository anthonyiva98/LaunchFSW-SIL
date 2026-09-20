add_library(FlightCore STATIC)

target_include_directories(
    FlightCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(
    FlightCore
    PRIVATE
        src/FlightComputer.cpp
)