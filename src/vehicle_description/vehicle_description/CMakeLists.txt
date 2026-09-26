cmake_minimum_required(VERSION 3.8)
project(vehicle_description)
find_package(ament_cmake REQUIRED)
# Only the authoritative physical model is installed; retained legacy copies
# await the user's single consolidated deletion review.
install(FILES
  urdf/complete_visual.xacro urdf/arm_only.xacro urdf/thanxe_new.xacro
  urdf/vehicle_dimensions.xacro urdf/vehicle_chassis.xacro urdf/swerve_module.xacro
  urdf/fr10_arm.urdf urdf/fr10_wrist.xacro urdf/body_sensors.xacro
  urdf/d435_simulation.xacro urdf/materials.xacro urdf/inertial_macros.xacro
  urdf/thanxe_new.gazebo
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
ament_package()
