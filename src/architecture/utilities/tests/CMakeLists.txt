add_executable(test_discretize test_discretize.cpp)
target_link_libraries(test_discretize GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_discretize PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_gaussMarkov test_gaussMarkov.cpp)
target_link_libraries(test_gaussMarkov GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_gaussMarkov PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_orbitalMotion test_orbitalMotion.cpp)
target_link_libraries(test_orbitalMotion GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_orbitalMotion PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_BSpline test_BSpline.cpp)
target_link_libraries(test_BSpline GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_BSpline PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_keplerianOrbit test_keplerianOrbit.cpp)
target_link_libraries(test_keplerianOrbit GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_keplerianOrbit PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_compare_orbitalMotion test_compare_orbitalMotion.cpp)
target_link_libraries(test_compare_orbitalMotion GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_compare_orbitalMotion PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_saturate test_saturate.cpp)
target_link_libraries(test_saturate GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_saturate PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_geodeticConversion test_geodeticConversion.cpp)
target_link_libraries(test_geodeticConversion GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_geodeticConversion PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_rigidBodyKinematics test_rigidBodyKinematics.cpp)
target_link_libraries(test_rigidBodyKinematics GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_rigidBodyKinematics PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_rigidBodyKinematicsC test_rigidBodyKinematicsC.cpp)
target_link_libraries(test_rigidBodyKinematicsC GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_rigidBodyKinematicsC PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_signalProcessing test_signalProcessing.cpp)
target_link_libraries(test_signalProcessing GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_signalProcessing PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_linearInterpolation test_linearInterpolation.cpp)
target_link_libraries(test_linearInterpolation GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_linearInterpolation PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_linearAlgebra test_linearAlgebra.cpp)
target_link_libraries(test_linearAlgebra GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_linearAlgebra PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_bilinearInterpolation test_bilinearInterpolation.cpp)
target_link_libraries(test_bilinearInterpolation GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_bilinearInterpolation PRIVATE "${CMAKE_SOURCE_DIR}")

add_executable(test_eigenSupport test_eigenSupport.cpp)
target_link_libraries(test_eigenSupport GTest::gtest_main Xmera::Core ArchitectureUtilities)
target_include_directories(test_eigenSupport PRIVATE "${CMAKE_SOURCE_DIR}")

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_discretize)
gtest_discover_tests(test_gaussMarkov)
gtest_discover_tests(test_orbitalMotion)
gtest_discover_tests(test_BSpline)
gtest_discover_tests(test_keplerianOrbit)
gtest_discover_tests(test_compare_orbitalMotion)
gtest_discover_tests(test_saturate)
gtest_discover_tests(test_geodeticConversion)
gtest_discover_tests(test_rigidBodyKinematics)
gtest_discover_tests(test_rigidBodyKinematicsC)
gtest_discover_tests(test_signalProcessing)
gtest_discover_tests(test_linearInterpolation)
gtest_discover_tests(test_linearAlgebra)
gtest_discover_tests(test_bilinearInterpolation)
gtest_discover_tests(test_eigenSupport)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_rigidBodyKinematics_fuzz test_rigidBodyKinematics_fuzz.cpp)
  target_include_directories(test_rigidBodyKinematics_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_link_libraries(test_rigidBodyKinematics_fuzz PRIVATE
    Xmera::Core
    ArchitectureUtilities
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main)

  gtest_discover_tests(test_rigidBodyKinematics_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke")
endif()
