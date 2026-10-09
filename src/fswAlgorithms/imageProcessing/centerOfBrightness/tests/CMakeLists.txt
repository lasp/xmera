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

  # fuzztest_setup_fuzzing_flags() adds AddressSanitizer, and it acts only when
  # FUZZTEST_FUZZING_MODE is on. UndefinedBehaviorSanitizer uses the same condition, thus
  # the nightly workflow gets it and pull request CI does not.
  #
  # -fno-sanitize-recover makes undefined behavior stop the process, thus the fuzzer records
  # a finding. Without it the sanitizer prints a message, continues, and the run stays green.
  # The vptr check is off because only a part of the program has the instrumentation, and
  # that check gives incorrect reports in that condition.
  #
  # The fuzz test source then removes the instrumentation again. That source includes
  # <fuzztest/fuzztest.h>, which includes abseil, and GCC cannot compile
  # absl/container/internal/hash_policy_traits.h with -fsanitize=undefined. GCC reports that
  # the comparison of a function address with nullptr is not a constant expression. The
  # algorithm source includes only Eigen and has no such problem, thus the code under test
  # keeps the instrumentation. Source options come after target options on the command line,
  # thus -fno-sanitize=undefined wins for this one file.
  if(FUZZTEST_FUZZING_MODE)
    target_compile_options(test_centerOfBrightness_fuzz PRIVATE
      -fsanitize=undefined -fno-sanitize=vptr -fno-sanitize-recover=undefined
    )
    set_source_files_properties(test_centerOfBrightness_fuzz.cpp PROPERTIES COMPILE_OPTIONS
      "-fno-sanitize=undefined"
    )
    target_link_options(test_centerOfBrightness_fuzz PRIVATE -fsanitize=undefined)
  endif()

  # The labels come from xmera_label_discovered_tests, not from PROPERTIES. Refer to the
  # note on that function in src/cmake/XmeraGoogleTest.cmake.
  gtest_discover_tests(test_centerOfBrightness_fuzz
    TEST_LIST cobFuzzTests
  )
  xmera_label_discovered_tests(test_centerOfBrightness_fuzz cobFuzzTests fuzz fuzz-smoke)
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
