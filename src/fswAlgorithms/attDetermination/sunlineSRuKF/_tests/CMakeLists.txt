# C++ unit tests for SunlineSRuKFAlgorithm. The algorithm is framework-agnostic
# (no xmera, no messaging, no SWIG) so the test exe compiles the algorithm
# source directly and links only filteringCore.

add_executable(test_sunlineSRuKFAlgorithm
  test_sunlineSRuKFAlgorithm.cpp
  ../sunlineSRuKFAlgorithm.cpp
)
target_include_directories(test_sunlineSRuKFAlgorithm PRIVATE ..)
target_link_libraries(test_sunlineSRuKFAlgorithm
  PRIVATE
    filteringCore
    GTest::gtest_main
)
gtest_discover_tests(test_sunlineSRuKFAlgorithm)
