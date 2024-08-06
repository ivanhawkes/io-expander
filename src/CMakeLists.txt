cmake_minimum_required(VERSION 3.14)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(~/projects/pico-sdk/pico_sdk_init.cmake)

project(io-expander)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(io-expander)

target_sources(io-expander PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Main.cpp
        )

# In addition to pico_stdlib required for common PicoSDK functionality, give access to double button reboot.
target_link_libraries(io-expander PUBLIC
        pico_stdlib
        hardware_i2c
        pico_bootsel_via_double_reset)

pico_add_extra_outputs(io-expander)

# Enable USB *or* UART output.
pico_enable_stdio_usb(io-expander 0)
pico_enable_stdio_uart(io-expander 1)
