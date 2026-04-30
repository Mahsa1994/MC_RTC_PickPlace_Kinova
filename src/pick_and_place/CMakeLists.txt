cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(pick_and_place VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mc_rtc REQUIRED)

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

ament_package()
