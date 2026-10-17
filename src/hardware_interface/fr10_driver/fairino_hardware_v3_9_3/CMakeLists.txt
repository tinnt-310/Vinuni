cmake_minimum_required(VERSION 3.8)
project(fairino_hardware_v3_9_3)

cmake_policy(SET CMP0069 NEW)
include(CheckIPOSupported)
check_ipo_supported(RESULT FAIRINO_IPO_SUPPORTED OUTPUT FAIRINO_IPO_ERROR)
option(FAIRINO_ENABLE_NATIVE_OPTIMIZATION
  "Enable deployment-host O3/native/LTO optimization" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fr10_motion_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

#compile
# file(CREATE_LINK
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# SYMBOLIC)

# file(CREATE_LINK
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2"
# "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so"
# SYMBOLIC)

add_library(
  fairino_hardware
  SHARED
  ./src/fairino_hardware_interface.cpp
)

if(FAIRINO_ENABLE_NATIVE_OPTIMIZATION AND
  (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  foreach(performance_target fairino_hardware)
    target_compile_options(${performance_target} PRIVATE
      -O3 -DNDEBUG -march=native)
    if(FAIRINO_IPO_SUPPORTED)
      set_property(TARGET ${performance_target} PROPERTY
        INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
  endforeach()
endif()

#target_compile_definitions(ros2_cmd_server PRIVATE CHN_VERSION)
#target_compile_definitions(ros2_cmd_server PRIVATE ENG_VERSION)


target_include_directories(fairino_hardware PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
$<INSTALL_INTERFACE:include/fairino_hardware>
)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so)
# target_link_libraries(ros2_cmd_server ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)
# target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.0)

# 链接库时添加rpath参数
target_link_libraries(fairino_hardware "-Wl,-rpath,$ORIGIN" ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.3)
target_link_options(fairino_hardware PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(fairino_hardware PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib"
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../../fr10_motion_interfaces/lib"
)

#add dependencies
ament_target_dependencies(fairino_hardware
  std_msgs
  geometry_msgs
  fr10_motion_interfaces
  realtime_tools
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

#install all files
#ros2_control hardware
install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Keep this versioned SDK beside the plugin. Installing it in the common
# prefix lib directory would let the 3.9.3 and 3.9.8 SDKs overwrite each
# other's libfairino.so.2 symlink in the same workspace.
install(
  FILES libfairino/lib/libfairino.so.2.3.3
  DESTINATION lib/${PROJECT_NAME}
)
install(CODE "
  file(CREATE_LINK
    libfairino.so.2.3.3
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so.2\"
    SYMBOLIC)
  file(CREATE_LINK
    libfairino.so.2
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so\"
    SYMBOLIC)
")


if(BUILD_TESTING)
  add_executable(test_force_sensor_packet test/test_force_sensor_packet.cpp)
  target_include_directories(test_force_sensor_packet PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME fairino_v393_force_packet COMMAND test_force_sensor_packet)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Preserve supplied SDK headers and unbuilt vendor command-server/examples.
  # Style checks cover the supported ros2_control plugin and its public header.
  file(GLOB_RECURSE FAIRINO_VENDOR_STYLE_FILES
    "examples/*.cpp" "examples/*.hpp" "libfairino/include/*.h"
    "include/fairino_hardware/*.hpp" "include/fairino_hardware/*.h" "src/*.cpp")
  list(REMOVE_ITEM FAIRINO_VENDOR_STYLE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/fairino_hardware_interface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/fairino_hardware/fairino_hardware_interface.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/fairino_hardware/version_control.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/fairino_hardware/visibility_control.h")
  set(ament_cmake_uncrustify_ADDITIONAL_EXCLUDE ${FAIRINO_VENDOR_STYLE_FILES})
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle
  geometry_msgs fr10_motion_interfaces realtime_tools)

ament_package()
