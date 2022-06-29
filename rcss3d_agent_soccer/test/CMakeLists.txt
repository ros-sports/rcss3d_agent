# Build test_sim_to_soccer_vision_3d
ament_add_gtest(test_sim_to_soccer_vision_3d
  test_sim_to_soccer_vision_3d.cpp)

target_link_libraries(test_sim_to_soccer_vision_3d
  ${PROJECT_NAME})

# Build test_polar_to_point
ament_add_gtest(test_polar_to_point
  test_polar_to_point.cpp)

target_link_libraries(test_polar_to_point
  ${PROJECT_NAME})
