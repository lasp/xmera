# The encoder .cpp is compiled only into the SWIG Python module (not a linkable
# C++ library), so pull the source in directly here.
add_executable(test_encoder
  ../encoder.cpp
  test_encoder.cpp
)
target_include_directories(test_encoder PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_encoder PRIVATE
  GTest::gtest_main
  Xmera::Core
  ArchitectureUtilities
)
gtest_discover_tests(test_encoder)
