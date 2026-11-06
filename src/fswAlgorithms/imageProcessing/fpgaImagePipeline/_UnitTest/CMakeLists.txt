# The SWIG Python module is the only target that compiles fpgaImagePipeline.cpp. The tests
# cannot link to it as a C++ library. Thus the tests compile fpgaImagePipeline.cpp directly.
add_executable(test_fpgaImagePipeline
  ../fpgaImagePipeline.cpp
  test_fpgaImagePipeline.cpp
)
target_include_directories(test_fpgaImagePipeline PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_fpgaImagePipeline PRIVATE
  GTest::gtest_main
  Xmera::Core
  ArchitectureUtilities
  opencv_imgcodecs
  opencv_imgproc
)
target_link_options(test_fpgaImagePipeline PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-Wl,-no_warn_duplicate_libraries>"
)
gtest_discover_tests(test_fpgaImagePipeline)
