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
gtest_discover_tests(test_imageReaderFromFile)
