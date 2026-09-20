add_library(SIL STATIC)

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