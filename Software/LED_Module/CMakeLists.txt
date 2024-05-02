cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 20)

include(pico_sdk_import.cmake)

project(main)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-unused-variable
        -Wint-conversion
        )
#set(PICO_BOARD JBCock)
#set(CMAKE_CXX_FLAGS "-O1")
pico_sdk_init()
# rest of your project
add_executable(main
        main.c
        sensors.c
        )
# pull in common dependencies


target_link_libraries(main pico_stdlib hardware_adc pico_multicore hardware_spi hardware_pwm hardware_dma hardware_i2c hardware_uart lib-can)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/CAN)
# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
#pico_enable_stdio_uart(main 1)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)