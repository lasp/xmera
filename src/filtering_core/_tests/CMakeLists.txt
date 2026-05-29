# Unit tests for the framework-agnostic filtering_core infrastructure. Links the
# header-only INTERFACE library directly (which propagates Eigen + the include
# dirs); no xmera, no messaging.

add_executable(test_heterogeneous_measurements
  test_heterogeneous_measurements.cpp
)
target_link_libraries(test_heterogeneous_measurements
  PRIVATE
    filtering_core
    GTest::gtest_main
)
gtest_discover_tests(test_heterogeneous_measurements)
