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

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_centerOfBrightness_fuzz
    ../centerOfBrightnessAlgorithm.cpp
    centerOfBrightnessTestHelpers.hpp
    test_centerOfBrightness_fuzz.cpp
  )

  target_include_directories(test_centerOfBrightness_fuzz PRIVATE "../..")
  target_include_directories(test_centerOfBrightness_fuzz PRIVATE "..")
  target_include_directories(test_centerOfBrightness_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_centerOfBrightness_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_centerOfBrightness_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()

find_package(OpenCV REQUIRED)

add_executable(test_imageReaderFromFile
  ../imageReader/imageReaderFromFile.cpp
  test_imageReaderFromFile.cpp
)
target_include_directories(test_imageReaderFromFile PRIVATE "../..")
target_include_directories(test_imageReaderFromFile PRIVATE "..")
target_include_directories(test_imageReaderFromFile PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_imageReaderFromFile PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
    opencv_imgcodecs
    opencv_imgproc
    opencv_core
)
# Suppress OpenCV warnings on clang due to its vcpkg distribution
# gcc doesn't emit such warnings
target_link_options(test_imageReaderFromFile PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-Wl,-no_warn_duplicate_libraries>"
)

gtest_discover_tests(test_imageReaderFromFile)
