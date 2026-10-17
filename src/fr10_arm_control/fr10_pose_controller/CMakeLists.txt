cmake_minimum_required(VERSION 3.8)
project(fr10_pose_controller)

cmake_policy(SET CMP0069 NEW)
include(CheckIPOSupported)
check_ipo_supported(RESULT FR10_IPO_SUPPORTED OUTPUT FR10_IPO_ERROR)
option(FR10_ENABLE_NATIVE_OPTIMIZATION
  "Enable deployment-host O3/native/LTO optimization" ON)

function(fr10_optimize_target target_name)
  if(FR10_ENABLE_NATIVE_OPTIMIZATION AND
     (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    target_compile_options(${target_name} PRIVATE -O3 -DNDEBUG -march=native)
    if(FR10_IPO_SUPPORTED)
      set_property(TARGET ${target_name} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
  endif()
endfunction()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fr10_motion_core REQUIRED)
find_package(fr10_motion_planning REQUIRED)
find_package(fr10_insertion_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig 0.9 REQUIRED)

add_library(fr10_backends SHARED
  src/ros_control_backend.cpp
)

target_compile_features(fr10_backends PUBLIC cxx_std_17)
target_include_directories(fr10_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fr10_backends
  rclcpp
  sensor_msgs
  std_msgs
  fr10_motion_core
)

target_link_libraries(fr10_backends
  ruckig::ruckig
)
fr10_optimize_target(fr10_backends)

add_executable(fr10_pose_controller_node
  src/fr10_pose_controller_node.cpp
  src/motion_adapter.cpp
  src/motion_runtime.cpp
  src/pose_planning_worker.cpp
  src/motion_command_router.cpp
  src/motion_diagnostics.cpp
  src/force_runtime.cpp
  src/force_corridor_planner.cpp
  src/force_runtime_config.cpp
  src/node_parameters.cpp
)
target_compile_features(fr10_pose_controller_node PUBLIC cxx_std_17)
target_link_libraries(fr10_pose_controller_node fr10_backends)
target_link_libraries(fr10_pose_controller_node Eigen3::Eigen)
target_compile_definitions(fr10_pose_controller_node PRIVATE
  EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MALLOC_ALREADY_ALIGNED=1
)
ament_target_dependencies(fr10_pose_controller_node
  rclcpp
  nav_msgs
  fr10_motion_core
  fr10_motion_planning
  fr10_insertion_manager
  geometry_msgs
  moveit_core
  moveit_ros_planning
  sensor_msgs
  std_msgs
)
# Do not apply -march=native to the MoveIt-facing adapter. MoveIt Humble is
# distributed with a 16-byte Eigen ABI; native AVX alignment across this shared
# library boundary can corrupt MatrixXd ownership. The ROS-free core remains
# native/LTO optimized.

add_executable(test_joint_retargeting
  test/test_joint_retargeting.cpp
)
target_compile_features(test_joint_retargeting PUBLIC cxx_std_17)
ament_target_dependencies(test_joint_retargeting fr10_motion_core)

add_executable(test_motion_controller
  test/test_motion_controller.cpp
)
target_compile_features(test_motion_controller PUBLIC cxx_std_17)
target_include_directories(test_motion_controller PRIVATE test/core)
ament_target_dependencies(test_motion_controller fr10_motion_core)

add_executable(fr10_kinematic_probe
  test/core/fr10_kinematic_probe.cpp
)
target_compile_features(fr10_kinematic_probe PUBLIC cxx_std_17)
target_include_directories(fr10_kinematic_probe PRIVATE test/core)
ament_target_dependencies(fr10_kinematic_probe fr10_motion_core)

add_executable(test_moveit_collision_provider
  test/core/test_moveit_collision_provider.cpp
)
target_compile_features(test_moveit_collision_provider PUBLIC cxx_std_17)
target_include_directories(test_moveit_collision_provider PRIVATE
  include
  test/core)
target_link_libraries(test_moveit_collision_provider Eigen3::Eigen)
target_compile_definitions(test_moveit_collision_provider PRIVATE
  EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MALLOC_ALREADY_ALIGNED=1
)
ament_target_dependencies(test_moveit_collision_provider
  geometry_msgs
  fr10_motion_core
  fr10_motion_planning
  moveit_core
  moveit_ros_planning
)

set(FR10_CORE_TESTS
  test_reference_lag_stop
  test_joint_otg_matrix
  test_cartesian_path
  test_trajectory_trace_validator
  test_cartesian_tracking
  test_joint_target_decoder
  test_insertion_sequence_state
  test_target_recovery_policy
  test_stop_reason_policy
  test_path_time_parameterizer
  test_invalid_commands
  test_command_arbitration
  test_library_consumer
  test_workspace
  test_analytical_ik
  test_insertion_planners
  test_realtime_buffer
  test_precision_insertion_extended
  test_kinematics_extended
  test_retarget_safety_extended
  test_diagnostic_display
)

foreach(test_target IN LISTS FR10_CORE_TESTS)
  add_executable(${test_target} test/core/${test_target}.cpp)
  target_compile_features(${test_target} PUBLIC cxx_std_17)
  target_include_directories(${test_target} PRIVATE test/core)
  ament_target_dependencies(${test_target} fr10_motion_core)
endforeach()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_node_tool_geometry
    test/core/test_node_tool_geometry.cpp src/node_parameters.cpp)
  target_compile_features(test_node_tool_geometry PRIVATE cxx_std_17)
  target_include_directories(test_node_tool_geometry PRIVATE include)
  ament_target_dependencies(test_node_tool_geometry rclcpp fr10_motion_core)
  add_test(NAME node_tool_geometry COMMAND test_node_tool_geometry)
  set_tests_properties(node_tool_geometry PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=209;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_log")
  add_executable(test_force_runtime_config
    test/core/test_force_runtime_config.cpp src/force_runtime_config.cpp)
  target_compile_features(test_force_runtime_config PRIVATE cxx_std_17)
  target_include_directories(test_force_runtime_config PRIVATE include)
  ament_target_dependencies(test_force_runtime_config rclcpp fr10_insertion_manager)
  add_test(NAME force_runtime_config COMMAND test_force_runtime_config)
  set_tests_properties(force_runtime_config PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=209;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_log")
  add_test(NAME reference_lag_stop COMMAND test_reference_lag_stop)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME delayed_joint_feedback_startup COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ros_control/test_feedback_startup.py)
  set_tests_properties(delayed_joint_feedback_startup PROPERTIES TIMEOUT 30)
  add_test(
    NAME fairino_v393_abi
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/abi/test_fairino_v393_abi.py
      --workspace ${CMAKE_CURRENT_SOURCE_DIR}/../../..
      --plugin
        ${CMAKE_INSTALL_PREFIX}/../fairino_hardware_v3_9_3/lib/fairino_hardware_v3_9_3/libfairino_hardware.so
      --backend
        ${CMAKE_INSTALL_PREFIX}/lib/libfr10_backends.so)
  add_test(NAME joint_otg_matrix COMMAND test_joint_otg_matrix)
  add_test(NAME cartesian_path COMMAND test_cartesian_path)
  add_test(NAME trajectory_trace_validator COMMAND test_trajectory_trace_validator)
  add_test(NAME cartesian_tracking COMMAND test_cartesian_tracking)
  add_test(NAME joint_target_decoder COMMAND test_joint_target_decoder)
  add_test(NAME insertion_sequence_state COMMAND test_insertion_sequence_state)
  add_test(NAME target_recovery_policy COMMAND test_target_recovery_policy)
  add_test(NAME stop_reason_policy COMMAND test_stop_reason_policy)
  add_test(NAME fake_setup_selection COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/simmachine/test_setup_selection.py)
  add_test(NAME fake_health_packet COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/simmachine/test_fake_health_packet.py)
  add_test(NAME wrist_radius_geometry COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/simmachine/test_wrist_radius_geometry.py)
  set_tests_properties(fake_setup_selection fake_health_packet
    wrist_radius_geometry PROPERTIES LABELS "fake_system_fixture")
  add_test(NAME path_time_parameterizer COMMAND test_path_time_parameterizer)
  add_test(NAME invalid_commands COMMAND test_invalid_commands)
  add_test(NAME command_arbitration COMMAND test_command_arbitration)
  add_test(NAME library_consumer COMMAND test_library_consumer)
  add_test(NAME workspace COMMAND test_workspace)
  add_test(NAME analytical_ik COMMAND test_analytical_ik)
  add_test(NAME insertion_planners COMMAND test_insertion_planners)
  add_test(NAME realtime_buffer COMMAND test_realtime_buffer)
  add_test(NAME diagnostic_display COMMAND test_diagnostic_display)
  add_test(NAME motion_executor COMMAND test_motion_controller)
  add_test(NAME moveit_collision_provider COMMAND test_moveit_collision_provider)

  set(FR10_PRECISION_INSERTION_CASES
    PI_BASE_POS_X
    PI_BASE_NEG_X
    PI_BASE_POS_Y
    PI_BASE_NEG_Y
    PI_BASE_POS_Z
    PI_BASE_NEG_Z
    PI_ROTATED_LOCAL_Z
    PI_ROTATED_LOCAL_X
    PI_PREINSERT_DISTANCE_MATRIX
    PI_ORIENTATION_PRESERVATION
    PI_WAYPOINT_DENSITY
    PI_NO_JOINT_SHORTCUT
  )
  foreach(case_id IN LISTS FR10_PRECISION_INSERTION_CASES)
    string(TOLOWER ${case_id} case_name)
    add_test(
      NAME extended_${case_name}
      COMMAND test_precision_insertion_extended --case ${case_id})
  endforeach()

  set(FR10_KINEMATICS_CASES
    K_IK_FK_CONSISTENCY
    K_CONTINUOUS_BRANCH
    K_WRIST_OUTSIDE
    K_WRIST_BOUNDARY
    K_WRIST_INSIDE
    K_ELBOW_OUTSIDE
    K_ELBOW_INSIDE
    K_RADIAL_BARRIER
    K_JACOBIAN_SIGMA_TREND
    K_JACOBIAN_CONDITION
    K_UNREACHABLE
    K_JOINT_LIMIT
    K_ANALYTICAL_BENCHMARK
    K_MOVEIT_NUMERICAL_NOT_ACTIVE
  )
  foreach(case_id IN LISTS FR10_KINEMATICS_CASES)
    string(TOLOWER ${case_id} case_name)
    add_test(
      NAME extended_${case_name}
      COMMAND test_kinematics_extended --case ${case_id})
    if(case_id STREQUAL "K_MOVEIT_NUMERICAL_NOT_ACTIVE")
      set_tests_properties(extended_${case_name}
        PROPERTIES SKIP_RETURN_CODE 77)
    endif()
  endforeach()

  set(FR10_RETARGET_SAFETY_CASES
    RT_STREAM_10HZ
    RT_STREAM_20HZ
    RT_STREAM_50HZ
    RT_STREAM_100HZ
    RT_DIRECTION_REVERSAL
    RT_RAPID_ALTERNATION
    RT_POSE_NOMINAL_TIMESTAMP_PLAYBACK
    RT_POSE_MISSED_CYCLE_PLAYBACK
    S_NAN_POSE
    S_INF_POSE
    S_INVALID_QUAT
    S_OWNERSHIP
    S_CONTROLLED_STOP_PROFILE
    S_PATH_FAULT_HYSTERESIS
    S_STALE_POSE_ACTIVATION
    S_POSE_FAULT_CONTROLLED_STOP
    S_FAULT_TRANSITION
    S_HARDWARE_ESTOP_INDICATION
    S_RECOVERY
    S_TARGET_REJECTED_AFTER_ESTOP_INDICATION
  )
  foreach(case_id IN LISTS FR10_RETARGET_SAFETY_CASES)
    string(TOLOWER ${case_id} case_name)
    add_test(
      NAME extended_${case_name}
      COMMAND test_retarget_safety_extended --case ${case_id})
  endforeach()

endif()

install(TARGETS fr10_backends
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS fr10_pose_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  include/fr10_pose_controller/ros_control_backend.hpp
  DESTINATION include/fr10_pose_controller
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  fr10_motion_core
  fr10_motion_planning
  ruckig
)
ament_export_include_directories(include)
# The standalone MuJoCo launch also loads this package's backend library.
# Export its lib directory through the normal ament runtime environment.
ament_export_libraries(fr10_backends)
ament_package()
