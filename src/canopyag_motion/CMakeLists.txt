cmake_minimum_required(VERSION 3.8)
project(canopyag_motion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/motion_server.py scripts/move_to.py scripts/demo.py scripts/goal_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_motion test/test_motion.py)
endif()

ament_package()
