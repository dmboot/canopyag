cmake_minimum_required(VERSION 3.8)
project(canopyag_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Parses the xacro and runs check_urdf on the result, so a broken edit to
  # arm_parameters.yaml fails the build instead of failing at robot spawn.
  ament_add_pytest_test(test_urdf_parses test/test_urdf_parses.py)
endif()

ament_package()
