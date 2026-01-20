cmake_minimum_required(VERSION 3.8)
project(arpa_ethernet_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories for C++
include_directories(include)

# =============================================================================
# C++ Section
# =============================================================================

# ParkerCore library
add_library(parker_core SHARED
  src/parker_core.cpp
)
target_include_directories(parker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parker_core rclcpp)

# Install C++ library
install(TARGETS parker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export library for downstream packages
ament_export_targets(export_parker_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(TARGETS parker_core
  EXPORT export_parker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# =============================================================================
# ros2_control Hardware Interface
# =============================================================================

add_library(parker_hardware_interface SHARED
  src/parker_hardware_interface.cpp
)
target_include_directories(parker_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(parker_hardware_interface parker_core)
ament_target_dependencies(parker_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Install hardware interface library
install(TARGETS parker_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface parker_hardware_interface.xml)

# Install C++ header files
install(DIRECTORY include/
  DESTINATION include
)

# =============================================================================
# Python Section
# =============================================================================

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  arpa_ethernet_motor/scripts/EE_motor_node.py
  arpa_ethernet_motor/scripts/PARKER_motor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Testing
# =============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
