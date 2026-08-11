add_executable(test_regionsOfInterest
  ../regionsOfInterestAlgorithm.cpp
  test_regionsOfInterest.cpp
)

target_include_directories(test_regionsOfInterest PRIVATE "../..")
target_include_directories(test_regionsOfInterest PRIVATE "..")
target_include_directories(test_regionsOfInterest PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_regionsOfInterest PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_regionsOfInterest)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_regionsOfInterest_fuzz
    ../regionsOfInterestAlgorithm.cpp
    test_regionsOfInterest_fuzz.cpp
  )

  target_include_directories(test_regionsOfInterest_fuzz PRIVATE "../..")
  target_include_directories(test_regionsOfInterest_fuzz PRIVATE "..")
  target_include_directories(test_regionsOfInterest_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_regionsOfInterest_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  # Refer to the note in centerOfBrightness/tests/CMakeLists.txt.
  if(FUZZTEST_FUZZING_MODE)
    target_compile_options(test_regionsOfInterest_fuzz PRIVATE
      -fsanitize=undefined -fno-sanitize=vptr -fno-sanitize-recover=undefined
    )
    set_source_files_properties(test_regionsOfInterest_fuzz.cpp PROPERTIES COMPILE_OPTIONS
      "-fno-sanitize=undefined"
    )
    target_link_options(test_regionsOfInterest_fuzz PRIVATE -fsanitize=undefined)
  endif()

  gtest_discover_tests(test_regionsOfInterest_fuzz
    TEST_LIST roiFuzzTests
  )
  xmera_label_discovered_tests(test_regionsOfInterest_fuzz roiFuzzTests fuzz fuzz-smoke)
endif()
