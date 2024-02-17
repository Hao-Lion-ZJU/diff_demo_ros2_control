cmake_minimum_required(VERSION 3.8)
project(diff_test_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  nav_msgs
  tf2
  tf2_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
include/diff_test_controller
)

# generate parameter listener hpp file
generate_parameter_library(diff_test_controller_parameters 
 src/diff_test_controller_parameter.yaml
)

# generate lib
add_library(diff_test_controller SHARED
  src/diff_test_controller.cpp
  src/odometry.cpp
)

target_compile_features(diff_test_controller PUBLIC cxx_std_17)

target_include_directories(diff_test_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_test_controller>
)

target_link_libraries(diff_test_controller PUBLIC diff_test_controller_parameters)
ament_target_dependencies(diff_test_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diff_test_controller PRIVATE "DIFF_TEST_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface diff_test_plugin.xml)

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

install(
  DIRECTORY include/
  DESTINATION include/diff_test_controller
)
install(TARGETS diff_test_controller diff_test_controller_parameters
  EXPORT export_diff_test_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 为了使下游文件可以访问
ament_export_targets(export_diff_test_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
