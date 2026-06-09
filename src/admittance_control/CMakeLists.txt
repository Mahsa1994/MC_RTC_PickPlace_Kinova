cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(admittance_control VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Bridge executable
add_executable(kortex_mc_rtc_bridge_admittance
  src/kortex_mc_rtc_bridge_admittance.cpp
)
target_link_libraries(kortex_mc_rtc_bridge_admittance
  mc_rtc::mc_control
  mc_rtc::mc_rtc_utils
)
ament_target_dependencies(kortex_mc_rtc_bridge_admittance
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  std_msgs
)

# 2. Controller library
add_library(KinovaHandGuiding SHARED
  src/KinovaHandGuiding.cpp
)
target_compile_definitions(KinovaHandGuiding PRIVATE MC_RTC_SHARED)
target_link_libraries(KinovaHandGuiding
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)
ament_target_dependencies(KinovaHandGuiding rclcpp)

# 3. States — prefixed library names to avoid clashing with state names
add_library(KHG_UpdateHoldTarget SHARED
  src/states/UpdateHoldTarget.cpp
)
target_compile_definitions(KHG_UpdateHoldTarget PRIVATE MC_RTC_SHARED)
target_link_libraries(KHG_UpdateHoldTarget
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)
ament_target_dependencies(KHG_UpdateHoldTarget rclcpp)

add_library(KHG_HandGuideState SHARED
  src/states/HandGuideState.cpp
)
target_compile_definitions(KHG_HandGuideState PRIVATE MC_RTC_SHARED)
target_link_libraries(KHG_HandGuideState
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)
ament_target_dependencies(KHG_HandGuideState rclcpp)

# 4. Install
install(TARGETS kortex_mc_rtc_bridge_admittance
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS KinovaHandGuiding
  LIBRARY DESTINATION lib/mc_controller
)
install(TARGETS KHG_UpdateHoldTarget KHG_HandGuideState
  LIBRARY DESTINATION lib/mc_controller/KinovaHandGuiding/states
)
install(FILES etc/KinovaHandGuiding.yaml
  DESTINATION lib/mc_controller/etc
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
