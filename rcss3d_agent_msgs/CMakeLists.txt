cmake_minimum_required(VERSION 3.5)
project(rcss3d_agent_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Accelerometer.msg"
  "msg/AgentState.msg"
  "msg/Ball.msg"
  "msg/Beam.msg"
  "msg/Effector.msg"
  "msg/FieldLine.msg"
  "msg/Flag.msg"
  "msg/ForceResistance.msg"
  "msg/GameState.msg"
  "msg/Goalpost.msg"
  "msg/GyroRate.msg"
  "msg/Hear.msg"
  "msg/HingeJointPos.msg"
  "msg/HingeJointVel.msg"
  "msg/Percept.msg"
  "msg/Player.msg"
  "msg/Say.msg"
  "msg/Spherical.msg"
  "msg/Synchronize.msg"
  "msg/UniversalJointPos.msg"
  "msg/UniversalJointVel.msg"
  "msg/Vision.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_package()
