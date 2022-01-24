cmake_minimum_required(VERSION 3.8)
project(rcss3d_agent_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcss3d_agent REQUIRED)
find_package(rclcpp_components REQUIRED)

# Build
add_library(rcss3d_agent_basic_node SHARED
  src/rcss3d_agent_basic_node.cpp)
target_include_directories(rcss3d_agent_basic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rcss3d_agent_basic_node
  rcss3d_agent
  rclcpp_components)
rclcpp_components_register_node(rcss3d_agent_basic_node
  PLUGIN "rcss3d_agent_basic::Rcss3dAgentBasicNode"
  EXECUTABLE rcss3d_agent_basic)

# Install
install(
  TARGETS rcss3d_agent_basic_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
