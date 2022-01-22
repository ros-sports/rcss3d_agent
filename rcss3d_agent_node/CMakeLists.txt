cmake_minimum_required(VERSION 3.8)
project(rcss3d_agent_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcss3d_agent REQUIRED)

# Build
add_executable(rcss3d_agent_node
  src/rcss3d_agent_node.cpp
  src/main.cpp)
target_include_directories(rcss3d_agent_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rcss3d_agent_node rcss3d_agent)

# Install
install(TARGETS rcss3d_agent_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
