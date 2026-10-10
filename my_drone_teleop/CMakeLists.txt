cmake_minimum_required(VERSION 3.8)
project(my_drone_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/arrow_teleop.py DESTINATION lib/${PROJECT_NAME} RENAME arrow_teleop)
install(PROGRAMS scripts/goto.py DESTINATION lib/${PROJECT_NAME} RENAME goto)

ament_package()
