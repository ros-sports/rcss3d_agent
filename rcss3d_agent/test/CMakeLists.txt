# Build test_sexp_parser
ament_add_gtest(test_sexp_parser
  test_sexp_parser.cpp)

target_link_libraries(test_sexp_parser
  rcss3d_agent
)

# Build test_sexp_creator
ament_add_gtest(test_sexp_creator
  test_sexp_creator.cpp)

target_link_libraries(test_sexp_creator
  rcss3d_agent
)

# Build test_sexp_vision
ament_add_gtest(test_sexp_vision
  test_sexp_vision.cpp)

target_link_libraries(test_sexp_vision
  rcss3d_agent
)
