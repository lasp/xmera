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
