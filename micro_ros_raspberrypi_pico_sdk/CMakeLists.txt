cmake_minimum_required(VERSION 3.13)

# This source-only project is meant to be overlaid on the upstream micro-ROS
# Raspberry Pi Pico SDK (kilted) checkout, which provides libmicroros/.
if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Set PICO_SDK_PATH to your local Pico SDK checkout")
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/libmicroros.a")
    message(FATAL_ERROR "Missing libmicroros. Clone upstream micro_ros_raspberrypi_pico_sdk (kilted branch) and copy these source files into its root. See README.md.")
endif()

add_executable(main main.c pico_uart_transport.c)

target_include_directories(main PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/include"
)

target_link_libraries(main PRIVATE
    pico_stdlib
    hardware_pwm
    "${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/libmicroros.a"
)

target_compile_options(main PRIVATE -ffunction-sections -fdata-sections)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
pico_add_extra_outputs(main)
