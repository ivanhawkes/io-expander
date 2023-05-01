cmake_minimum_required(VERSION 3.14)

# Initialize the SDK directly.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(control_panels C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Build with lower optomisation level.
set(PICO_DEOPTIMIZED_DEBUG 1)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

set(MAIN_PROJECT_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_compile_options(
    -Wall
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized)

# Source folder.
add_subdirectory(src)

# Set a path to our includes.
target_include_directories(io-expander PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
