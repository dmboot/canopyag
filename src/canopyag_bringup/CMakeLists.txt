cmake_minimum_required(VERSION 3.8)
project(canopyag_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # hardware.launch.py + canopyag_hardware against tools/mks_sim.py on vcan0,
  # once per fault-injection flag. Skips with a notice when vcan0 is missing
  # (README: "Testing on vcan0").
  add_launch_test(test/test_vcan_launch.py TIMEOUT 900)
endif()

ament_package()
