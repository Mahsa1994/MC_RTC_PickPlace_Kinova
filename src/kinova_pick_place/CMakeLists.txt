cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(kinova_pick_place VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ── Locate sva headers (transitive dep of mc_rtc) ────────────────────────────
find_path(SVA_INCLUDE_DIR sva/PTransformd.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}
)

if(NOT SVA_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find sva/PTransformd.h (SpaceVecAlg headers).")
endif()

# ── Controller library ─────────────────────────────────────────────────────
add_library(PickPlaceController SHARED
  src/PickPlaceController.cpp
)

target_compile_definitions(PickPlaceController PRIVATE MC_RTC_SHARED)

target_include_directories(PickPlaceController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SVA_INCLUDE_DIR}
)

target_link_libraries(PickPlaceController
  mc_rtc::mc_control_fsm
)

ament_target_dependencies(PickPlaceController
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# ── States library (single .so with all FSM states) ─────────────────────────
add_library(PickPlaceStates SHARED
  src/states/ResetState.cpp
  src/states/ApproachState.cpp
  src/states/PickState.cpp
  src/states/LiftState.cpp
  src/states/PlaceApproachState.cpp
  src/states/PlaceState.cpp
)

target_compile_definitions(PickPlaceStates PRIVATE MC_RTC_SHARED)

target_include_directories(PickPlaceStates PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${SVA_INCLUDE_DIR}
)

target_link_libraries(PickPlaceStates
  PickPlaceController
  mc_rtc::mc_control_fsm
)

ament_target_dependencies(PickPlaceStates
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS PickPlaceController
  LIBRARY DESTINATION lib/mc_controller
)

install(TARGETS PickPlaceStates
  LIBRARY DESTINATION lib/mc_controller/states
)

install(FILES etc/PickPlaceController.yaml
  DESTINATION lib/mc_controller/etc
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
