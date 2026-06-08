cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(admittance_control VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(rclcpp REQUIRED)

# 1. Controller library
add_library(KinovaHandGuiding SHARED
  src/KinovaHandGuiding.cpp
)
target_compile_definitions(KinovaHandGuiding PRIVATE MC_RTC_SHARED)
target_link_libraries(KinovaHandGuiding
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)
ament_target_dependencies(KinovaHandGuiding rclcpp)

# 2. States library
add_library(AdmittanceStates SHARED
  src/states/UpdateHoldTarget.cpp
)
target_compile_definitions(AdmittanceStates PRIVATE MC_RTC_SHARED)
target_link_libraries(AdmittanceStates
  mc_rtc::mc_control
  mc_rtc::mc_control_fsm
)
ament_target_dependencies(AdmittanceStates rclcpp)

# 3. Install
install(TARGETS KinovaHandGuiding
  LIBRARY DESTINATION lib/mc_controller
)

#install(TARGETS AdmittanceStates
#  LIBRARY DESTINATION lib/mc_controller/KinovaHandGuiding  # ← key fix
#)
install(TARGETS AdmittanceStates
  LIBRARY DESTINATION lib/mc_controller/KinovaHandGuiding/states
)

install(FILES etc/KinovaHandGuiding.yaml
  DESTINATION lib/mc_controller/etc
)

ament_package()
