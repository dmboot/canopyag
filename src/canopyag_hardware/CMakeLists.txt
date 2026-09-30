cmake_minimum_required(VERSION 3.8)
project(canopyag_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Driver layer: plain C++, no ROS types - what the gtests link against.
add_library(mks_driver STATIC
  src/mks_protocol.cpp
  src/can_socket.cpp
  src/mks_bus.cpp
  src/joint_config.cpp
)
target_include_directories(mks_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mks_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ros2_control plugin.
add_library(${PROJECT_NAME} SHARED src/canopyag_system.cpp)
target_link_libraries(${PROJECT_NAME} mks_driver)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface canopyag_hardware.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS tools/mks_sim.py scripts/ghost_state.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mks_protocol test/test_mks_protocol.cpp)
  target_link_libraries(test_mks_protocol mks_driver)
  ament_add_gtest(test_conversion test/test_conversion.cpp)
  target_link_libraries(test_conversion mks_driver)
  # The full-stack test against tools/mks_sim.py on vcan0 is in
  # canopyag_bringup/test, next to the launch file it starts.
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
