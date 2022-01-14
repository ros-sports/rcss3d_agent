# Build test_sexp_parser
ament_add_gtest(test_sexp_parser
  test_sexp_parser.cpp)

target_link_libraries(test_sexp_parser
  rcss3d_agent
)
