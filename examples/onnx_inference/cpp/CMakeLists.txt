cmake_minimum_required(VERSION 3.14)
project(act_onnx_inference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optional SpaceMIT ORT SDK path. Leave empty to use system packages
# (spacemit-onnxruntime / python3-spacemit-ort on K3).
set(SPACEMIT_ORT_DIR "" CACHE PATH "Optional path to SpaceMIT ORT SDK")

set(SPACEMIT_ORT_HINTS)
if(SPACEMIT_ORT_DIR)
    list(APPEND SPACEMIT_ORT_HINTS "${SPACEMIT_ORT_DIR}")
endif()

find_path(SPACEMIT_ORT_INCLUDE_DIR
    NAMES spacemit_ort_env.h
    HINTS ${SPACEMIT_ORT_HINTS}
    PATH_SUFFIXES include)

find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    HINTS ${SPACEMIT_ORT_HINTS}
    PATH_SUFFIXES lib lib64)
find_library(SPACEMIT_EP_LIB
    NAMES spacemit_ep
    HINTS ${SPACEMIT_ORT_HINTS}
    PATH_SUFFIXES lib lib64)

if(NOT SPACEMIT_ORT_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB OR NOT SPACEMIT_EP_LIB)
    message(FATAL_ERROR
        "Missing SpaceMIT ORT dependency. Install spacemit-onnxruntime on K3, "
        "or set SPACEMIT_ORT_DIR to an unpacked SDK containing include/ and lib/.")
endif()

add_library(spacemit_ort_deps INTERFACE)
target_include_directories(spacemit_ort_deps INTERFACE "${SPACEMIT_ORT_INCLUDE_DIR}")
target_link_libraries(spacemit_ort_deps INTERFACE
    "${ONNXRUNTIME_LIB}"
    "${SPACEMIT_EP_LIB}")

# ACT (Action Chunking Transformer) single whole-graph benchmark.
# Model path is passed as a CLI argument, so no compile definitions needed.
add_executable(act_benchmark act_benchmark.cpp)
target_link_libraries(act_benchmark spacemit_ort_deps)

# ACT real-robot control / test.
# Offline by default (no hardware deps): feed --images-npy/--state-npy.
# Real-robot mode: configure with -DACT_ROBOT_HW=ON on the K3 board, which
# links OpenCV (cameras) and the `motor` library (SO-101 servos).
option(ACT_ROBOT_HW "Build act_evaluate with OpenCV + motor library (real hardware)" OFF)
add_executable(act_evaluate act_evaluate.cpp)
target_link_libraries(act_evaluate spacemit_ort_deps)

if(ACT_ROBOT_HW)
    # motor library: components/peripherals/motor (libmotor.a, no ROS2).
    set(MOTOR_DIR "" CACHE PATH "Path to components/peripherals/motor")
    if(NOT MOTOR_DIR)
        foreach(MOTOR_DIR_CANDIDATE IN ITEMS
            "${CMAKE_CURRENT_LIST_DIR}/../../../../../peripherals/motor"
            "${CMAKE_CURRENT_LIST_DIR}/../../../../peripherals/motor"
            "${CMAKE_CURRENT_LIST_DIR}/../../../peripherals/motor")
            if(EXISTS "${MOTOR_DIR_CANDIDATE}/CMakeLists.txt")
                set(MOTOR_DIR "${MOTOR_DIR_CANDIDATE}" CACHE PATH
                    "Path to components/peripherals/motor" FORCE)
                break()
            endif()
        endforeach()
    endif()
    if(NOT MOTOR_DIR OR NOT EXISTS "${MOTOR_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "MOTOR_DIR was not found. Set -DMOTOR_DIR=/path/to/components/peripherals/motor.")
    endif()
    set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
        CACHE STRING "Enabled motor drivers (semicolon-separated)." FORCE)
    find_package(OpenCV REQUIRED)
    add_subdirectory(${MOTOR_DIR} motor_build)
    target_compile_definitions(act_evaluate PRIVATE ACT_ROBOT_HW)
    target_include_directories(act_evaluate PRIVATE ${MOTOR_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(act_evaluate
        -Wl,--whole-archive motor -Wl,--no-whole-archive
        ${OpenCV_LIBS})
endif()

# Install
install(TARGETS act_benchmark act_evaluate DESTINATION bin)
