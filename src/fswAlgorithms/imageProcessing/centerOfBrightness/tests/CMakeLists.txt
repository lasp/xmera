add_executable(test_centerOfBrightness
  ../centerOfBrightnessAlgorithm.cpp
  test_centerOfBrightness.cpp
)

target_include_directories(test_centerOfBrightness PRIVATE "../..")
target_include_directories(test_centerOfBrightness PRIVATE "..")
target_include_directories(test_centerOfBrightness PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_centerOfBrightness PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
)

gtest_discover_tests(test_centerOfBrightness)
