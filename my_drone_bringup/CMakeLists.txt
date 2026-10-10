cmake_minimum_required(VERSION 3.8)
project(my_drone_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})

ament_package()
