cmake_minimum_required(VERSION 3.16)
project(robot_control_pkg LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
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
  robot_control_pkg
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(robot_control_pkg PUBLIC cxx_std_17)
target_include_directories(robot_control_pkg PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_control_pkg>
)
ament_target_dependencies(
  robot_control_pkg PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(robot_control_pkg PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOT_CONTROL_PKG_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_control_pkg.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_control_pkg
)
install(
  DIRECTORY description/urdf description/models description/ros2_control
  DESTINATION share/robot_control_pkg
)
install(
  DIRECTORY bringup/launch bringup/config description
  DESTINATION share/robot_control_pkg
)
install(TARGETS robot_control_pkg
  EXPORT export_robot_control_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_robot_control_pkg HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
