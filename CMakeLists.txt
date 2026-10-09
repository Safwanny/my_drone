cmake_minimum_required(VERSION 3.8)
project(my_drone)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/arrow_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
