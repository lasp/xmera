add_executable(test_vizInterface
  test_vizInterface.cpp
)
target_link_libraries(test_vizInterface PRIVATE
  GTest::gtest_main
  vizInterfaceCore
)
gtest_discover_tests(test_vizInterface)
