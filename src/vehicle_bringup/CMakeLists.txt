cmake_minimum_required(VERSION 3.8)
project(vehicle_bringup)
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
add_library(vinuni_wheel_contact_surface SHARED src/wheel_contact_surface.cpp)
target_include_directories(vinuni_wheel_contact_surface PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(vinuni_wheel_contact_surface ${GAZEBO_LIBRARIES})
install(TARGETS vinuni_wheel_contact_surface LIBRARY DESTINATION lib)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.orig" EXCLUDE
  PATTERN "*.rej" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.orig" EXCLUDE
  PATTERN "*.rej" EXCLUDE)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(FILES README.md README_COMMANDS.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wait_for_gazebo_clock.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/force_fixture_publisher.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/matched_station_publisher.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fr10_force_status.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fr10_force_plot.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fr10_robot_plot.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/wait_for_navigation.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  add_executable(test_ode_pose_frames test/integration/test_ode_pose_frames.cpp)
  target_compile_features(test_ode_pose_frames PRIVATE cxx_std_17)
  target_include_directories(test_ode_pose_frames PRIVATE ${GAZEBO_INCLUDE_DIRS})
  target_link_libraries(test_ode_pose_frames ${GAZEBO_LIBRARIES})
  add_test(NAME independent_gazebo_world_frames COMMAND test_ode_pose_frames)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_readiness test/test_navigation_readiness.py)
  ament_add_pytest_test(test_launch_python_runtime test/test_launch_python_runtime.py)
  ament_add_pytest_test(test_backend_selection
    test/test_backend_selection.py)
  ament_add_pytest_test(test_charging_station test/test_charging_station.py)
  ament_add_pytest_test(test_simulation_station_sequence
    test/test_simulation_station_sequence.py)
  ament_add_pytest_test(test_wait_for_gazebo_clock
    test/test_wait_for_gazebo_clock.py)
  ament_add_pytest_test(test_fr10_robot_plot test/test_fr10_robot_plot.py)
  ament_add_pytest_test(test_simulation_controller_contract
    test/test_simulation_controller_contract.py)
  ament_add_pytest_test(test_shared_fr10_model
    test/test_shared_fr10_model.py)
  ament_add_pytest_test(test_vehicle_geometry_contract
    test/test_vehicle_geometry_contract.py)
  ament_add_pytest_test(test_rviz_profiles
    test/test_rviz_profiles.py)
endif()
ament_package()
