cmake_minimum_required(VERSION 3.10)
project(MyKinovaController)

find_package(mc_rtc REQUIRED)

add_controller(MyKinovaController src/MyKinovaController.cpp)

install_controller_configuration(etc/MyKinovaController.yaml)
