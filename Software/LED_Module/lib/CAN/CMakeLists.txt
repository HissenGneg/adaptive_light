cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 20)

add_library(lib-can INTERFACE)

target_sources(lib-can INTERFACE
    CAN.c
    mcp_interface.c
)


target_link_libraries(lib-can INTERFACE
    hardware_spi
    hardware_timer
)