cmake_minimum_required(VERSION 3.8)
project(canopyag_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/import_solidworks_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Expands the xacro and checks the result, so a broken edit to
  # robot_parameters.yaml fails the build instead of failing at robot spawn.
  ament_add_pytest_test(test_urdf_parses test/test_urdf_parses.py)
endif()

ament_package()
