# The reactionWheels .cpp is compiled only into the SWIG Python module (not a
# linkable C++ library), so pull the source in directly here, mirroring the
# architecture/utilities tests and the spacecraft benchmark target.
add_executable(test_reactionWheelStateEffector
  ../reactionWheelStateEffector.cpp
  test_reactionWheelStateEffector.cpp
)
target_include_directories(test_reactionWheelStateEffector PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_reactionWheelStateEffector PRIVATE
  GTest::gtest_main
  Xmera::Core
  ArchitectureUtilities
  dynamicsCommon
  Eigen3::Eigen
)
gtest_discover_tests(test_reactionWheelStateEffector)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  # Lightweight single-method invariants: saturation, friction, mass-props.
  add_executable(test_reactionWheelStateEffector_fuzz
    ../reactionWheelStateEffector.cpp
    test_reactionWheelStateEffector_fuzz.cpp
  )
  target_include_directories(test_reactionWheelStateEffector_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_link_libraries(test_reactionWheelStateEffector_fuzz PRIVATE
    Xmera::Core
    ArchitectureUtilities
    dynamicsCommon
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_reactionWheelStateEffector_fuzz
    PROPERTIES
      LABELS "fuzz;fuzz-smoke"
  )

  # Heavier conservation target: drives a full Spacecraft + integrator, so it
  # also pulls in spacecraft.cpp directly (same approach as the benchmark).
  add_executable(test_reactionWheelStateEffector_conservation_fuzz
    ../reactionWheelStateEffector.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../spacecraft/spacecraft.cpp"
    test_reactionWheelStateEffector_conservation_fuzz.cpp
  )
  target_include_directories(test_reactionWheelStateEffector_conservation_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_link_libraries(test_reactionWheelStateEffector_conservation_fuzz PRIVATE
    Xmera::Core
    ArchitectureUtilities
    dynamicsCommon
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_reactionWheelStateEffector_conservation_fuzz
    PROPERTIES
      LABELS "fuzz;fuzz-smoke"
  )
endif()
