cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(impedance_control VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------
# Dependencies (merged)
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(RBDyn REQUIRED)

# -----------------------------
# Impedance Controller
# -----------------------------
add_executable(kortex_mc_rtc_bridge_impedance
  src/kortex_mc_rtc_bridge_impedance.cpp
)

target_link_libraries(kortex_mc_rtc_bridge_impedance
  mc_rtc::mc_control
  mc_rtc::mc_rtc_utils
  RBDyn::RBDyn
)

ament_target_dependencies(kortex_mc_rtc_bridge_impedance
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  std_msgs
)

add_library(KinovaImpedance SHARED
  src/KinovaImpedance.cpp
)

target_compile_definitions(KinovaImpedance PRIVATE MC_RTC_SHARED)

target_link_libraries(KinovaImpedance
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)

ament_target_dependencies(KinovaImpedance rclcpp)

# -----------------------------
# Install rules (merged)
# -----------------------------
install(TARGETS
  kortex_mc_rtc_bridge_impedance
  KinovaImpedance
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  KinovaImpedance
  LIBRARY DESTINATION lib/mc_controller
)

install(FILES
  etc/KinovaImpedance.yaml
  DESTINATION lib/mc_controller/etc
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
