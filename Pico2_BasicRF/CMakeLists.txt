cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico2_basic_rf C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_executable(pico2_basic_rf
    src/main.c
)

target_link_libraries(pico2_basic_rf
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(pico2_basic_rf 1)
pico_enable_stdio_uart(pico2_basic_rf 0)

pico_add_extra_outputs(pico2_basic_rf)
