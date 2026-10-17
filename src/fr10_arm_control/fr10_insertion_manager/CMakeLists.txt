cmake_minimum_required(VERSION 3.8)
project(fr10_insertion_manager)

find_package(ament_cmake REQUIRED)
find_package(fr10_force_control REQUIRED)
find_package(fr10_motion_core REQUIRED)
find_package(fr10_motion_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(fr10_insertion_manager SHARED
  src/contact_interval_probe.cpp
  src/contact_reference.cpp
  src/keyed_yaw_probe.cpp
  src/force_session.cpp
  src/insertion_geometry_monitor.cpp
  src/insertion_state_machine.cpp
  src/nominal_insertion_provider.cpp
)
target_compile_features(fr10_insertion_manager PUBLIC cxx_std_17)
target_include_directories(fr10_insertion_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fr10_insertion_manager
  fr10_force_control fr10_motion_core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(fr10_insertion_manager PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

add_library(fr10_insertion_ros SHARED
  src/insertion_ros_adapter.cpp src/parking_ros_adapter.cpp)
target_compile_features(fr10_insertion_ros PUBLIC cxx_std_17)
target_include_directories(fr10_insertion_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fr10_insertion_ros fr10_insertion_manager)
ament_target_dependencies(fr10_insertion_ros
  fr10_force_control
  fr10_motion_core
  fr10_motion_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
)

if(BUILD_TESTING)
  add_executable(test_contact_interval_probe test/test_contact_interval_probe.cpp)
  target_compile_features(test_contact_interval_probe PUBLIC cxx_std_17)
  target_link_libraries(test_contact_interval_probe fr10_insertion_manager)
  add_test(NAME contact_interval_probe COMMAND test_contact_interval_probe)
  add_executable(test_completion_force_reserve test/test_completion_force_reserve.cpp)
  target_compile_features(test_completion_force_reserve PUBLIC cxx_std_17)
  target_link_libraries(test_completion_force_reserve fr10_insertion_manager)
  add_test(NAME completion_force_reserve COMMAND test_completion_force_reserve)
  add_executable(test_keyed_yaw_probe test/test_keyed_yaw_probe.cpp)
  target_compile_features(test_keyed_yaw_probe PUBLIC cxx_std_17)
  target_link_libraries(test_keyed_yaw_probe fr10_insertion_manager)
  add_test(NAME keyed_yaw_contact_probe COMMAND test_keyed_yaw_probe)
  add_executable(test_force_session_envelope test/test_force_session_envelope.cpp)
  target_compile_features(test_force_session_envelope PUBLIC cxx_std_17)
  target_link_libraries(test_force_session_envelope fr10_insertion_manager)
  add_test(NAME force_session_goal_envelope COMMAND test_force_session_envelope)
  add_executable(test_contact_reference test/test_contact_reference.cpp)
  target_compile_features(test_contact_reference PUBLIC cxx_std_17)
  target_link_libraries(test_contact_reference fr10_insertion_manager)
  add_test(NAME bounded_contact_reference COMMAND test_contact_reference)
  add_executable(test_wrench_source_status test/test_wrench_source_status.cpp)
  target_compile_features(test_wrench_source_status PUBLIC cxx_std_17)
  target_link_libraries(test_wrench_source_status fr10_insertion_ros)
  ament_target_dependencies(test_wrench_source_status rclcpp geometry_msgs fr10_motion_interfaces)
  add_test(NAME wrench_source_status COMMAND test_wrench_source_status)
  set_tests_properties(wrench_source_status PROPERTIES TIMEOUT 12
    ENVIRONMENT "ROS_DOMAIN_ID=229;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/fr10_source_status_test")
  add_executable(test_wrench_source_batch test/test_wrench_source_batch.cpp)
  target_compile_features(test_wrench_source_batch PUBLIC cxx_std_17)
  target_link_libraries(test_wrench_source_batch fr10_insertion_ros)
  ament_target_dependencies(test_wrench_source_batch rclcpp geometry_msgs fr10_motion_interfaces)
  add_test(NAME wrench_source_batch COMMAND test_wrench_source_batch)
  set_tests_properties(wrench_source_batch PROPERTIES TIMEOUT 12
    ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/fr10_wrench_batch_test")
  add_executable(test_hold_fault_reporting test/test_hold_fault_reporting.cpp)
  target_compile_features(test_hold_fault_reporting PUBLIC cxx_std_17)
  target_link_libraries(test_hold_fault_reporting fr10_insertion_ros)
  ament_target_dependencies(test_hold_fault_reporting rclcpp geometry_msgs fr10_motion_interfaces)
  add_test(NAME hold_fault_reporting COMMAND test_hold_fault_reporting)
  set_tests_properties(hold_fault_reporting PROPERTIES TIMEOUT 12
    ENVIRONMENT "ROS_DOMAIN_ID=231;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/fr10_hold_fault_reporting_test")
  add_executable(test_parking_feedback_transport test/test_parking_feedback_transport.cpp)
  target_compile_features(test_parking_feedback_transport PUBLIC cxx_std_17)
  target_link_libraries(test_parking_feedback_transport fr10_insertion_ros)
  ament_target_dependencies(test_parking_feedback_transport rclcpp nav_msgs fr10_motion_interfaces)
  add_test(NAME parking_feedback_transport COMMAND test_parking_feedback_transport)
  set_tests_properties(parking_feedback_transport PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_DOMAIN_ID=196;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/fr10_parking_feedback_test")
  add_executable(test_matched_force_transaction test/test_matched_force_transaction.cpp)
  target_compile_features(test_matched_force_transaction PUBLIC cxx_std_17)
  target_link_libraries(test_matched_force_transaction fr10_insertion_manager)
  add_test(NAME matched_force_transaction COMMAND test_matched_force_transaction)
  add_executable(test_base_hold_monitor test/test_base_hold_monitor.cpp)
  target_include_directories(test_base_hold_monitor PRIVATE include)
  target_compile_features(test_base_hold_monitor PUBLIC cxx_std_17)
  ament_target_dependencies(test_base_hold_monitor fr10_motion_core)
  add_test(NAME base_hold_monitor COMMAND test_base_hold_monitor)
  add_executable(test_matched_mating test/test_matched_mating.cpp)
  target_compile_features(test_matched_mating PUBLIC cxx_std_17)
  target_link_libraries(test_matched_mating fr10_insertion_manager)
  add_test(NAME matched_mating COMMAND test_matched_mating)
  add_executable(test_insertion_state_machine
    test/test_insertion_state_machine.cpp)
  target_compile_features(test_insertion_state_machine PUBLIC cxx_std_17)
  target_link_libraries(test_insertion_state_machine fr10_insertion_manager)
  add_test(NAME insertion_state_machine COMMAND test_insertion_state_machine)

  add_executable(test_fake_force_session test/test_fake_force_session.cpp)
  target_compile_features(test_fake_force_session PUBLIC cxx_std_17)
  target_link_libraries(test_fake_force_session fr10_insertion_manager)
  add_test(NAME fake_force_session COMMAND test_fake_force_session)

  add_executable(test_insertion_geometry_monitor test/test_insertion_geometry_monitor.cpp)
  target_compile_features(test_insertion_geometry_monitor PUBLIC cxx_std_17)
  target_link_libraries(test_insertion_geometry_monitor fr10_insertion_manager)
  add_test(NAME insertion_geometry_monitor COMMAND test_insertion_geometry_monitor)
  add_executable(test_force_analytical_pipeline test/test_force_analytical_pipeline.cpp)
  target_compile_features(test_force_analytical_pipeline PUBLIC cxx_std_17)
  target_link_libraries(test_force_analytical_pipeline fr10_insertion_manager)
  add_test(NAME force_analytical_pipeline COMMAND test_force_analytical_pipeline)
endif()

install(TARGETS fr10_insertion_manager fr10_insertion_ros
  EXPORT export_fr10_insertion_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_fr10_insertion_manager HAS_LIBRARY_TARGET)
ament_export_libraries(fr10_insertion_manager fr10_insertion_ros)
ament_export_include_directories(include)
ament_export_dependencies(
  fr10_force_control fr10_motion_core fr10_motion_interfaces
  geometry_msgs nav_msgs rclcpp rclcpp_action std_msgs tf2_msgs)
ament_package()
