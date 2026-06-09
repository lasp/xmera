# Unit tests for the framework-agnostic flyby OD SRUKF algorithm. Links the
# algorithm static library directly (no xmera, no messaging) and exercises it
# through its public C++ API.

add_executable(test_flybyODuKFAlgorithm
  test_flybyODuKFAlgorithm.cpp
)
target_link_libraries(test_flybyODuKFAlgorithm
  PRIVATE
    flybyODuKF_algorithm   # propagates filteringCore + Eigen include dirs
    GTest::gtest_main
)
gtest_discover_tests(test_flybyODuKFAlgorithm)
