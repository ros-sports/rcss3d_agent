cmake_minimum_required(VERSION 3.5)
project(rcss3d_agent)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcss3d_agent_msgs REQUIRED)

add_library(rcss3d_agent
  src/rcss3d_agent.cpp
  src/rcss3d_joint_controller.cpp
  src/socket.cpp
  src/socketaddress.cpp
  src/connection.cpp
  src/sexp_creator.cpp
  src/sexp_parser.cpp
  external/sexpresso/sexpresso/sexpresso.cpp)

target_include_directories(rcss3d_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/sexpresso>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rcss3d_agent
  "rclcpp"
  "rosgraph_msgs"
  "sensor_msgs"
  "rcss3d_agent_msgs")

# target_compile_options(rcss3d_agent PRIVATE -Werror)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rcss3d_agent PRIVATE "RCSS3D_AGENT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rcss3d_agent
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(rcss3d_agent_node src/rcss3d_agent_node.cpp)
target_include_directories(rcss3d_agent_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rcss3d_agent_node rcss3d_agent)
# target_compile_options(rcss3d_agent_node PRIVATE -Werror)

add_executable(rcss3d_joint_controller_node src/rcss3d_joint_controller_node.cpp)
target_include_directories(rcss3d_joint_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rcss3d_joint_controller_node rcss3d_agent)
# target_compile_options(rcss3d_joint_controller_node PRIVATE -Werror)

install(TARGETS rcss3d_agent_node rcss3d_joint_controller_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  rcss3d_agent
)
ament_export_dependencies(
  sensor_msgs
  rcss3d_agent_msgs
)

ament_package()
