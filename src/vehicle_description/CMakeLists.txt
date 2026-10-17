cmake_minimum_required(VERSION 3.8)
project(vehicle_description)
find_package(ament_cmake REQUIRED)
# Install the canonical physical model. Retired sources are preserved in docs/archive.
install(FILES
  urdf/vehicle.urdf.xacro urdf/vehicle_base.xacro
  urdf/vehicle_dimensions.xacro urdf/vehicle_chassis.xacro urdf/swerve_module.xacro urdf/ros2_control.xacro
  urdf/fr10_arm.urdf urdf/fr10_wrist.xacro urdf/fr10_photo_wrist.xacro urdf/fr10_ros2_control.xacro urdf/body_sensors.xacro
  urdf/d435_simulation.xacro urdf/materials.xacro urdf/inertial_macros.xacro
  urdf/vehicle.gazebo.xacro
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(single_fr10_model_contract
    test/test_single_fr10_model_contract.py)
  ament_add_pytest_test(rb_v3_model_contract
    test/test_rb_v3_model_contract.py)
  ament_add_pytest_test(fr10_tool_datum_contract
    test/test_fr10_tool_datum.py)
  ament_add_pytest_test(fr10_photo_layout_contract test/test_fr10_photo_layout.py)
endif()
ament_package()
