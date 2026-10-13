cmake_minimum_required(VERSION 3.8)
project(my_drone_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

# The control maths: header-only, no ROS.
add_library(control INTERFACE)
target_include_directories(control INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(control INTERFACE cxx_std_17)
target_link_libraries(control INTERFACE Eigen3::Eigen)

add_executable(flight_controller src/flight_controller.cpp)
target_link_libraries(flight_controller control tinyxml2::tinyxml2)
ament_target_dependencies(flight_controller
  rclcpp actuator_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)

add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator control)
ament_target_dependencies(state_estimator rclcpp nav_msgs sensor_msgs)

install(TARGETS control EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS flight_controller state_estimator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control control)
  ament_add_gtest(test_estimator test/test_estimator.cpp)
  target_link_libraries(test_estimator control)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3)
ament_package()
