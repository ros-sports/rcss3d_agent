# Build test_conversion
ament_add_gtest(test_conversion
  test_conversion.cpp)

target_link_libraries(test_conversion
  ${PROJECT_NAME})

# Build test_polar_to_point
ament_add_gtest(test_polar_to_point
  test_polar_to_point.cpp)

target_link_libraries(test_polar_to_point
  ${PROJECT_NAME})
