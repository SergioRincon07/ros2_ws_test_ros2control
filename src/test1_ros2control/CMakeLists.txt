cmake_minimum_required(VERSION 3.8)
project(test1_ros2control)

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
  test1_ros2control
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(test1_ros2control PUBLIC cxx_std_17)
target_include_directories(test1_ros2control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/test1_ros2control>
)
ament_target_dependencies(
  test1_ros2control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "TEST1_ROS2CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface test1_ros2control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/test1_ros2control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/test1_ros2control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/test1_ros2control
)
install(TARGETS test1_ros2control
  EXPORT export_test1_ros2control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_test1_ros2control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
