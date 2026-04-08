#[[
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#]]

cmake_minimum_required(VERSION 3.10)
project(chassis LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find pthreads
find_package(Threads REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core source files
set(CHASSIS_CORE_SOURCES
    src/chassis_core.c
)

# Driver source files
set(CHASSIS_DRIVER_SOURCES "")
set(_default_drivers "drv_uart_esp32")

# Use SROBOTIS_CONTROL_BASE_ENABLED_DRIVERS if defined, otherwise use defaults
if(DEFINED SROBOTIS_CONTROL_BASE_ENABLED_DRIVERS)
    set(_drivers "${SROBOTIS_CONTROL_BASE_ENABLED_DRIVERS}")
else()
    set(_drivers "${_default_drivers}")
endif()

foreach(_drv IN LISTS _drivers)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${_drv}.c")
        list(APPEND CHASSIS_DRIVER_SOURCES "src/drivers/${_drv}.c")
        message(STATUS "[chassis] Including driver: ${_drv}")
    else()
        message(FATAL_ERROR "[chassis] Driver '${_drv}' not found at src/drivers/${_drv}.c")
    endif()
endforeach()

# All source files
set(CHASSIS_SOURCES
    ${CHASSIS_CORE_SOURCES}
    ${CHASSIS_DRIVER_SOURCES}
)

# Create shared library
add_library(chassis SHARED ${CHASSIS_SOURCES})
target_link_libraries(chassis PRIVATE Threads::Threads m)

# Set compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(chassis PRIVATE
        -Wall
        -Wextra
    )
endif()

# Install rules
install(TARGETS chassis
    EXPORT ${SROBOTIS_EXPORT_SET}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES include/chassis.h
    DESTINATION include
)

# Optional: Build test executables
option(CHASSIS_BUILD_TESTS "Build chassis test programs" ON)

if(CHASSIS_BUILD_TESTS)
    add_executable(test_chassis_uart test/test_chassis_uart.c)
    target_link_libraries(test_chassis_uart chassis m)
    set_target_properties(test_chassis_uart PROPERTIES BUILD_RPATH "$ORIGIN")
    install(TARGETS test_chassis_uart
        RUNTIME DESTINATION bin
    )

    # Only build keyboard_control if it exists
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/keyboard_control.c")
        add_executable(keyboard_control test/keyboard_control.c)
        target_link_libraries(keyboard_control chassis m)
        set_target_properties(keyboard_control PROPERTIES BUILD_RPATH "$ORIGIN")
        install(TARGETS keyboard_control
            RUNTIME DESTINATION bin
        )
    endif()
endif()
