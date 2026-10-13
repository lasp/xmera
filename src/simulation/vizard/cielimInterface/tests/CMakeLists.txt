add_executable(test_cielimInterface
  test_cielimInterface.cpp
)
target_link_libraries(test_cielimInterface PRIVATE
  GTest::gtest_main
  cielimInterfaceCore
)
gtest_discover_tests(test_cielimInterface)
