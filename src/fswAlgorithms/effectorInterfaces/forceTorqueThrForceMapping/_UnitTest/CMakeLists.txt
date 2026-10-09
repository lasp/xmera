# The SWIG Python module is the only target that compiles the module sources. The tests
# cannot link to it as a C++ library. Thus the tests compile the sources directly.
add_executable(test_forceTorqueThrForceMapping
  ../forceTorqueThrForceMapping.cpp
  ../forceTorqueThrForceMappingAlgorithm.cpp
  test_forceTorqueThrForceMapping.cpp
)
target_include_directories(test_forceTorqueThrForceMapping PRIVATE ".." "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_forceTorqueThrForceMapping PRIVATE
  GTest::gtest_main
  Xmera::Core
  ArchitectureUtilities
  Eigen3::Eigen
)
gtest_discover_tests(test_forceTorqueThrForceMapping)
