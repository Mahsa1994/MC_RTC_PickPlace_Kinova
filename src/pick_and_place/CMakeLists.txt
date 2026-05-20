cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(pick_and_place VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED) 

# ── Controller library (no ROS dependency) ───────────────────────────────────
add_library(PickPlaceController SHARED
  src/PickPlaceController.cpp
)

target_compile_definitions(PickPlaceController PRIVATE MC_RTC_SHARED)

target_include_directories(PickPlaceController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(PickPlaceController
  mc_rtc::mc_control_fsm
)

# ── States library ────────────────────────────────────────────────────────────
add_library(PickPlaceStates SHARED
  src/states/PickPlaceStates.cpp
)

target_compile_definitions(PickPlaceStates PRIVATE MC_RTC_SHARED)

target_include_directories(PickPlaceStates PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(PickPlaceStates
  mc_rtc::mc_control_fsm
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS PickPlaceController
  LIBRARY DESTINATION lib/mc_controller
)

install(TARGETS PickPlaceStates
  LIBRARY DESTINATION lib/mc_controller/states
)

install(FILES etc/PickPlaceController.yaml
  DESTINATION lib/mc_controller/etc
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds/  DESTINATION share/${PROJECT_NAME}/worlds)



# ── RealRobotSensor GlobalPlugin ─────────────────────────────────────────────
add_library(RealRobotSensor SHARED src/plugins/RealRobotSensor.cpp)

# Link mc_rtc without PUBLIC so it matches ROS 2 ament_target_dependencies
target_link_libraries(RealRobotSensor mc_rtc::mc_control)
ament_target_dependencies(RealRobotSensor rclcpp sensor_msgs)

install(TARGETS RealRobotSensor
  LIBRARY DESTINATION lib/mc_plugins
)
install(FILES etc/RealRobotSensor.yaml
  DESTINATION lib/mc_plugins/etc
)

######################
add_executable(kortex_mc_rtc_bridge src/kortex_mc_rtc_bridge.cpp)
target_link_libraries(kortex_mc_rtc_bridge mc_rtc::mc_control)
ament_target_dependencies(kortex_mc_rtc_bridge rclcpp sensor_msgs trajectory_msgs)

install(TARGETS kortex_mc_rtc_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
ament_package()

