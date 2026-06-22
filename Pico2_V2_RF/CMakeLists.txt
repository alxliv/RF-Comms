cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico2_v2_rf C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

include(${CMAKE_CURRENT_LIST_DIR}/lib/RF24/CMakeLists.txt)

execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_HASH MATCHES "^[0-9a-fA-F]{8}$")
    set(GIT_HASH "00000000")
endif()

string(TIMESTAMP BUILD_DATE "%Y%m%d")

add_executable(pico2_v2_rf
    src/main.cpp
)

target_compile_definitions(pico2_v2_rf PRIVATE
    FW_GIT_HASH=0x${GIT_HASH}u
    FW_BUILD_DATE=${BUILD_DATE}u
)

set_source_files_properties(
    src/main.cpp
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Wno-comment"
)

target_link_libraries(pico2_v2_rf
    pico_stdlib
    hardware_gpio
    hardware_spi
    RF24
)

pico_enable_stdio_usb(pico2_v2_rf 1)
pico_enable_stdio_uart(pico2_v2_rf 0)

pico_add_extra_outputs(pico2_v2_rf)
