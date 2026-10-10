cmake_minimum_required(VERSION 3.8)
project(my_drone_px4)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/offboard_adapter.py DESTINATION lib/${PROJECT_NAME} RENAME offboard_adapter)
install(DIRECTORY airframes DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_frames test/test_frames.py)
endif()

ament_package()
