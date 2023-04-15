cmake_minimum_required(VERSION 3.14)

add_executable(io-expander)

target_sources(io-expander PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Main.cpp
        )

# Set a path to our includes.
target_include_directories(io-expander PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

# In addition to pico_stdlib required for common PicoSDK functionality, give access to double button reboot.
target_link_libraries(io-expander PUBLIC pico_stdlib hardware_adc
        pico_bootsel_via_double_reset)

pico_add_extra_outputs(io-expander)

# Enable USB *or* UART output.
pico_enable_stdio_usb(io-expander 0)
pico_enable_stdio_uart(io-expander 1)
