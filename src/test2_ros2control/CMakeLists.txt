cmake_minimum_required(VERSION 3.8)
project(test2_ros2control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  test2_ros2control
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(test2_ros2control PUBLIC cxx_std_17)
target_include_directories(test2_ros2control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/test2_ros2control>
)
ament_target_dependencies(
  test2_ros2control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "TEST2_ROS2CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface test2_ros2control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/test2_ros2control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/test2_ros2control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/test2_ros2control
)
install(TARGETS test2_ros2control
  EXPORT export_test2_ros2control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_test2_ros2control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

