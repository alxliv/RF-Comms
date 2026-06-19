cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico2_v1_rf C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_executable(pico2_v1_rf
    src/main.c
    src/nrf24.c
    src/protocol.c
)

target_include_directories(pico2_v1_rf PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

set_source_files_properties(
    src/main.c
    src/nrf24.c
    src/protocol.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic"
)

target_link_libraries(pico2_v1_rf
    pico_stdlib
    hardware_gpio
    hardware_spi
)

pico_enable_stdio_usb(pico2_v1_rf 1)
pico_enable_stdio_uart(pico2_v1_rf 0)

pico_add_extra_outputs(pico2_v1_rf)
