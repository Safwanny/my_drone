cmake_minimum_required(VERSION 3.8)
project(my_drone_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sensor_realism.py DESTINATION lib/${PROJECT_NAME})

ament_package()
