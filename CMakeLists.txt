cmake_minimum_required(VERSION 3.12)

# Standalone builds check the module skeleton only, not Pico firmware.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(pere_modbus_control LANGUAGES CXX)
endif()

add_library(pere_modbus_control OBJECT
    src/actuators/Fan.cpp
    src/actuators/Valve.cpp
    src/control/FanValveTask.cpp
)

target_compile_features(pere_modbus_control PUBLIC cxx_std_20)
target_include_directories(pere_modbus_control PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
