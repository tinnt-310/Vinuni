cmake_minimum_required(VERSION 3.8)
project(fr10_motion_core)

cmake_policy(SET CMP0069 NEW)
include(CheckIPOSupported)
check_ipo_supported(RESULT FR10_CORE_IPO_SUPPORTED OUTPUT FR10_CORE_IPO_ERROR)
option(FR10_ENABLE_NATIVE_OPTIMIZATION
  "Enable deployment-host O3/native/LTO optimization" ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig 0.9 REQUIRED)

add_library(fr10_motion_core SHARED
  src/joint_otg.cpp
  src/cartesian_path.cpp
  src/trajectory_trace_validator.cpp
  src/cartesian_route_planner.cpp
  src/cartesian_trajectory.cpp
  src/continuous_ik.cpp
  src/continuous_ik_stepper.cpp
  src/fr10_analytical_ik.cpp
  src/pre_insert_planner.cpp
  src/linear_insert_planner.cpp
  src/motion_controller.cpp
  src/motion_coordinator.cpp
)

target_compile_features(fr10_motion_core PUBLIC cxx_std_17)
target_include_directories(fr10_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fr10_motion_core ruckig::ruckig Eigen3::Eigen)

if(BUILD_TESTING)
  add_executable(test_reference_transaction test/test_reference_transaction.cpp)
  target_compile_features(test_reference_transaction PUBLIC cxx_std_17)
  target_link_libraries(test_reference_transaction fr10_motion_core)
  add_test(NAME reference_transaction COMMAND test_reference_transaction)
  add_executable(test_tick_clock test/test_tick_clock.cpp)
  target_compile_features(test_tick_clock PUBLIC cxx_std_17)
  target_link_libraries(test_tick_clock fr10_motion_core)
  add_test(NAME tick_clock COMMAND test_tick_clock)
  add_executable(test_motion_coordinator test/test_motion_coordinator.cpp)
  target_compile_features(test_motion_coordinator PUBLIC cxx_std_17)
  target_link_libraries(test_motion_coordinator fr10_motion_core)
  add_test(NAME motion_coordinator COMMAND test_motion_coordinator)
  add_executable(test_continuous_ik_stepper
    test/test_continuous_ik_stepper.cpp)
  target_compile_features(test_continuous_ik_stepper PUBLIC cxx_std_17)
  target_link_libraries(test_continuous_ik_stepper fr10_motion_core)
  add_test(NAME continuous_ik_stepper COMMAND test_continuous_ik_stepper)

  add_executable(test_online_motion_executor
    test/test_online_motion_executor.cpp)
  target_compile_features(test_online_motion_executor PUBLIC cxx_std_17)
  target_link_libraries(test_online_motion_executor fr10_motion_core)
  add_test(NAME online_motion_executor COMMAND test_online_motion_executor)
endif()

if(FR10_ENABLE_NATIVE_OPTIMIZATION AND
   (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  target_compile_options(fr10_motion_core PRIVATE
    -O3 -DNDEBUG -march=native -Wall -Wextra -Wpedantic)
  if(FR10_CORE_IPO_SUPPORTED)
    set_property(TARGET fr10_motion_core
      PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

install(TARGETS fr10_motion_core
  EXPORT export_fr10_motion_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_fr10_motion_core HAS_LIBRARY_TARGET)
ament_export_libraries(fr10_motion_core)
ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3 ruckig)
ament_package()
