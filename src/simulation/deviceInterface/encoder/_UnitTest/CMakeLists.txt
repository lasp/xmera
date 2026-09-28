# The encoder .cpp is compiled only into the SWIG Python module (not a linkable
# C++ library), so pull the source in directly here.
add_executable(test_encoder
  ../encoder.cpp
  test_encoder.cpp
)
target_include_directories(test_encoder PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_encoder PRIVATE
  GTest::gtest_main
  Xmera::Core
  ArchitectureUtilities
)
gtest_discover_tests(test_encoder)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_encoder_fuzz
    ../encoder.cpp
    test_encoder_fuzz.cpp
  )
  target_include_directories(test_encoder_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_link_libraries(test_encoder_fuzz PRIVATE
    Xmera::Core
    ArchitectureUtilities
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_encoder_fuzz
    PROPERTIES
      LABELS "fuzz;fuzz-smoke"
  )
endif()
