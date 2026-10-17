cmake_minimum_required(VERSION 3.8)
project(fr10_force_control)

find_package(ament_cmake REQUIRED)
find_package(fr10_motion_core REQUIRED)

add_library(fr10_force_control SHARED
  src/admittance_envelope.cpp
  src/admittance_model.cpp
  src/contact_detector.cpp
  src/fake_wrench_source.cpp
  src/force_position_controller.cpp
  src/force_safety_monitor.cpp
  src/wrench_compensation.cpp
  src/wrench_filter.cpp
  src/wrench_transform.cpp
  src/wrench_tare.cpp
)
target_compile_features(fr10_force_control PUBLIC cxx_std_17)
ament_target_dependencies(fr10_force_control fr10_motion_core)
target_include_directories(fr10_force_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(fr10_force_control PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(test_admittance_envelope test/test_admittance_envelope.cpp)
  target_compile_features(test_admittance_envelope PUBLIC cxx_std_17)
  target_link_libraries(test_admittance_envelope fr10_force_control)
  add_test(NAME admittance_vector_envelope COMMAND test_admittance_envelope)
  add_executable(test_admittance_equilibrium test/test_admittance_equilibrium.cpp)
  target_compile_features(test_admittance_equilibrium PUBLIC cxx_std_17)
  target_link_libraries(test_admittance_equilibrium fr10_force_control)
  add_test(NAME admittance_equilibrium COMMAND test_admittance_equilibrium)
  add_executable(test_wrench_tare test/test_wrench_tare.cpp)
  target_compile_features(test_wrench_tare PUBLIC cxx_std_17)
  target_link_libraries(test_wrench_tare fr10_force_control)
  add_test(NAME qualified_no_contact_tare COMMAND test_wrench_tare)
  add_executable(test_force_control test/test_force_control.cpp)
  target_compile_features(test_force_control PUBLIC cxx_std_17)
  target_link_libraries(test_force_control fr10_force_control)
  add_test(NAME force_control_math COMMAND test_force_control)

  add_executable(test_force_conditioning test/test_force_conditioning.cpp)
  target_compile_features(test_force_conditioning PUBLIC cxx_std_17)
  target_link_libraries(test_force_conditioning fr10_force_control)
  add_test(NAME force_conditioning COMMAND test_force_conditioning)
endif()

install(TARGETS fr10_force_control
  EXPORT export_fr10_force_control
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_fr10_force_control HAS_LIBRARY_TARGET)
ament_export_libraries(fr10_force_control)
ament_export_include_directories(include)
ament_export_dependencies(fr10_motion_core)
ament_package()
