cmake_minimum_required(VERSION 3.8)
project(fr10_motion_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(controller_interface REQUIRED)
find_package(fr10_motion_core REQUIRED)
find_package(fr10_motion_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(fr10_motion_controller SHARED
  src/fr10_motion_controller.cpp
  src/trajectory_decoder.cpp
)
target_compile_features(fr10_motion_controller PUBLIC cxx_std_17)
target_include_directories(fr10_motion_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fr10_motion_controller
  controller_interface
  fr10_motion_core
  fr10_motion_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  trajectory_msgs
)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS fr10_motion_controller
  EXPORT export_fr10_motion_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_trajectory_decoder test/test_trajectory_decoder.cpp)
  target_link_libraries(test_trajectory_decoder fr10_motion_controller)
  ament_target_dependencies(test_trajectory_decoder
    fr10_motion_core fr10_motion_interfaces trajectory_msgs)
endif()

ament_export_targets(export_fr10_motion_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface fr10_motion_core fr10_motion_interfaces rclcpp)
ament_package()
