cmake_minimum_required(VERSION 3.18)
project(smolvla_robot_pipeline LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(SPACEMIT_EP "EP204" CACHE STRING "EP backend: SYSTEM | EP204")
set_property(CACHE SPACEMIT_EP PROPERTY STRINGS SYSTEM EP204)
set(SPACEMIT_ORT_DIR "" CACHE PATH "Override ONNX Runtime / SpaceMIT EP SDK root")

set(_ORT_HINT_DIRS "")
set(_ORT_FIND_MODE)
if(SPACEMIT_ORT_DIR)
  list(APPEND _ORT_HINT_DIRS "${SPACEMIT_ORT_DIR}")
  list(APPEND _ORT_FIND_MODE NO_DEFAULT_PATH)
elseif(SPACEMIT_EP STREQUAL "EP204")
  list(APPEND _ORT_HINT_DIRS "$ENV{HOME}/spacemit-ort.riscv64.2.0.4_yyx")
  list(APPEND _ORT_FIND_MODE NO_DEFAULT_PATH)
elseif(SPACEMIT_EP STREQUAL "SYSTEM")
  message(STATUS "Using system ONNX Runtime")
else()
  message(FATAL_ERROR "Unknown SPACEMIT_EP=${SPACEMIT_EP}")
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${_ORT_HINT_DIRS}
  PATH_SUFFIXES include include/onnxruntime
  PATHS /usr/local/include/onnxruntime /usr/local/include /usr/include/onnxruntime /usr/include
  ${_ORT_FIND_MODE})

find_path(SPACEMIT_EP_INCLUDE_DIR spacemit_ort_env.h
  HINTS ${_ORT_HINT_DIRS}
  PATH_SUFFIXES include include/onnxruntime
  PATHS /usr/local/include/onnxruntime /usr/local/include /usr/include/onnxruntime /usr/include
  ${_ORT_FIND_MODE})

find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS ${_ORT_HINT_DIRS}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local/lib /usr/lib/riscv64-linux-gnu /usr/lib /lib/riscv64-linux-gnu /lib
  ${_ORT_FIND_MODE})

find_library(SPACEMIT_EP_LIB spacemit_ep
  HINTS ${_ORT_HINT_DIRS}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local/lib /usr/lib/riscv64-linux-gnu /usr/lib /lib/riscv64-linux-gnu /lib
  ${_ORT_FIND_MODE})

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT SPACEMIT_EP_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB OR NOT SPACEMIT_EP_LIB)
  message(FATAL_ERROR
    "onnxruntime / spacemit_ep not found\n"
    "  ONNXRUNTIME_INCLUDE_DIR=${ONNXRUNTIME_INCLUDE_DIR}\n"
    "  SPACEMIT_EP_INCLUDE_DIR=${SPACEMIT_EP_INCLUDE_DIR}\n"
    "  ONNXRUNTIME_LIB=${ONNXRUNTIME_LIB}\n"
    "  SPACEMIT_EP_LIB=${SPACEMIT_EP_LIB}")
endif()

find_package(OpenCV REQUIRED)

set(MOTOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../../peripherals/motor"
  CACHE PATH "Path to components/peripherals/motor")
set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech"
  CACHE STRING "Enabled motor drivers" FORCE)
add_subdirectory("${MOTOR_DIR}" motor_build)

add_executable(smolvla_robot_pipeline ../smolvla_robot_pipeline.cpp)
target_compile_definitions(smolvla_robot_pipeline PRIVATE ACT_ROBOT_HW)
target_include_directories(smolvla_robot_pipeline PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/.."
  "${ONNXRUNTIME_INCLUDE_DIR}"
  "${SPACEMIT_EP_INCLUDE_DIR}"
  "${MOTOR_DIR}/include"
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(smolvla_robot_pipeline PRIVATE
  "${ONNXRUNTIME_LIB}"
  -Wl,--whole-archive motor -Wl,--no-whole-archive
  ${OpenCV_LIBS}
  pthread dl)

get_filename_component(_ort_libdir "${ONNXRUNTIME_LIB}" DIRECTORY)
get_filename_component(_ep_libdir "${SPACEMIT_EP_LIB}" DIRECTORY)
set_target_properties(smolvla_robot_pipeline PROPERTIES
  BUILD_RPATH "${_ort_libdir};${_ep_libdir}"
  INSTALL_RPATH "${_ort_libdir};${_ep_libdir}")

message(STATUS "onnxruntime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "spacemit_ep include: ${SPACEMIT_EP_INCLUDE_DIR}")
message(STATUS "onnxruntime lib:     ${ONNXRUNTIME_LIB}")
message(STATUS "spacemit_ep lib:     ${SPACEMIT_EP_LIB}")
message(STATUS "motor dir:           ${MOTOR_DIR}")
