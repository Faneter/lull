cmake_minimum_required(VERSION 3.22)

# lull - Embedded HAL abstraction library
# Zero-overhead compile-time abstraction layer for MCU peripherals

add_library(lull INTERFACE)

# Include directories
target_include_directories(lull INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# Source files that need compilation (propagated to consumers via INTERFACE)
target_sources(lull INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Mahony/MahonyAHRS.cpp
)
