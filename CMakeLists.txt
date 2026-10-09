cmake_minimum_required(VERSION 3.8)
project(my_drone)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(flight_controller src/flight_controller.cpp)
target_compile_features(flight_controller PUBLIC cxx_std_17)
target_link_libraries(flight_controller tinyxml2::tinyxml2 Eigen3::Eigen)
ament_target_dependencies(flight_controller
  rclcpp actuator_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)

install(TARGETS flight_controller DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf meshes worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/arrow_teleop.py scripts/goto.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control Eigen3::Eigen)
endif()

ament_package()
