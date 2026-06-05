# =============================================================================
#  CMakeLists.txt — KinovaGravityComp mc_rtc FSM controller
# =============================================================================
cmake_minimum_required(VERSION 3.10)
project(KinovaGravityComp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Find mc_rtc (provides mc_rtc::mc_control and mc_rtc::mc_tasks)
# ---------------------------------------------------------------------------
find_package(mc_rtc REQUIRED)

# ---------------------------------------------------------------------------
# The FSM controller itself.
#
# For a purely YAML-driven FSM, all you need is to install the YAML file.
# We still build a minimal shared library so mc_rtc can load it as a plugin.
# ---------------------------------------------------------------------------
add_library(KinovaGravityComp SHARED
  # No controller source files needed for a pure FSM — mc_rtc's built-in
  # FSMController handles everything declared in the YAML.
  # If you derive from FSMController add src/KinovaGravityComp.cpp here.
)
target_link_libraries(KinovaGravityComp PUBLIC mc_rtc::mc_control)

# ---------------------------------------------------------------------------
# Custom FSM state: UpdateHoldTarget
# ---------------------------------------------------------------------------
add_library(UpdateHoldTarget SHARED
  src/states/UpdateHoldTarget.cpp
)
target_include_directories(UpdateHoldTarget PRIVATE src/states)
target_link_libraries(UpdateHoldTarget PUBLIC mc_rtc::mc_control)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
# Install controller library to mc_rtc's controller search path
install(TARGETS KinovaGravityComp
  LIBRARY DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/KinovaGravityComp"
)

# Install the custom state library
install(TARGETS UpdateHoldTarget
  LIBRARY DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/KinovaGravityComp"
)

# Install YAML configuration
install(
  FILES etc/KinovaGravityComp.yaml
  DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/KinovaGravityComp/etc"
)

# ---------------------------------------------------------------------------
# Declare the FSM controller in mc_rtc's plugin manifest
# ---------------------------------------------------------------------------
set(CONTROLLER_NAME KinovaGravityComp)
set(CONTROLLER_FLAGS FSM)   # tells mc_rtc this uses the FSM base controller
mc_rtc_controller_macro(${CONTROLLER_NAME} ${CONTROLLER_FLAGS})
