cmake_minimum_required(VERSION 3.8)
project(fr10_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fr10_motion_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(ompl REQUIRED)

add_library(fr10_motion_planning SHARED
  src/moveit_ik_provider.cpp
  src/joint_route_planner.cpp
  src/online_motion_validator.cpp
)
target_compile_features(fr10_motion_planning PUBLIC cxx_std_17)
target_include_directories(fr10_motion_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(fr10_motion_planning PRIVATE
  EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MALLOC_ALREADY_ALIGNED=1
)
target_link_libraries(fr10_motion_planning Eigen3::Eigen ompl::ompl)
ament_target_dependencies(fr10_motion_planning
  fr10_motion_core
  geometry_msgs
  moveit_core
  moveit_ros_planning
)

if(BUILD_TESTING)
  add_executable(test_joint_route_planner test/test_joint_route_planner.cpp)
  target_link_libraries(test_joint_route_planner fr10_motion_planning)
  add_test(NAME joint_route_planner COMMAND test_joint_route_planner)
  add_executable(test_online_motion_validator test/test_online_motion_validator.cpp)
  target_link_libraries(test_online_motion_validator fr10_motion_planning)
  add_test(NAME online_motion_validator COMMAND test_online_motion_validator)
endif()

install(TARGETS fr10_motion_planning
  EXPORT export_fr10_motion_planning
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_fr10_motion_planning HAS_LIBRARY_TARGET)
ament_export_libraries(fr10_motion_planning)
ament_export_include_directories(include)
ament_export_dependencies(fr10_motion_core geometry_msgs moveit_core moveit_ros_planning ompl)
ament_package()
