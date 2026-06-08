cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(admittance_control VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(rclcpp REQUIRED)

# 1. Controller library (Standard mc_rtc controller)
# Note: Using your state file as a source so the library isn't empty
add_library(AdmittanceController SHARED
  src/states/UpdateHoldTarget.cpp
)

target_compile_definitions(AdmittanceController PRIVATE MC_RTC_SHARED)

target_include_directories(AdmittanceController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/states>
)

target_link_libraries(AdmittanceController
  mc_rtc::mc_control_fsm
)

# 2. States library
add_library(AdmittanceStates SHARED
  src/states/UpdateHoldTarget.cpp
)

target_compile_definitions(AdmittanceStates PRIVATE MC_RTC_SHARED)

target_link_libraries(AdmittanceStates
  mc_rtc::mc_control_fsm
)

# 3. Install (Exactly like pick_and_place)
install(TARGETS AdmittanceController
  LIBRARY DESTINATION lib/mc_controller
)

install(TARGETS AdmittanceStates
  LIBRARY DESTINATION lib/mc_states
)

install(FILES etc/KinovaHandGuiding.yaml
  DESTINATION lib/mc_controller/etc
)

# 4. ROS Dependencies
ament_target_dependencies(AdmittanceController rclcpp)
ament_target_dependencies(AdmittanceStates rclcpp)

ament_package()
