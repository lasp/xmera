option(XMERA_ENABLE_FUZZTESTS "Fetch and build fuzz targets with Google FuzzTest" OFF)

if(XMERA_ENABLE_FUZZTESTS)
  include(FetchContent)

  set(XMERA_FUZZTEST_GIT_TAG "2025-08-05"
    CACHE STRING
    "Git tag or commit to use for the Google FuzzTest dependency"
  )

  FetchContent_Declare(
    fuzztest
    GIT_REPOSITORY https://github.com/google/fuzztest.git
    GIT_TAG main   # or a specific commit/tag
  )

  # Make fuzztest's own stuff quiet and out of 'all'
  set(FETCHCONTENT_FUZZTEST_EXCLUDE_FROM_ALL ON)

  # Turn off fuzztest's own tests / gtest download
  set(FUZZTEST_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(FUZZTEST_DOWNLOAD_GTEST OFF CACHE BOOL "" FORCE)
  set(ANTLR_BUILD_CPP_TESTS OFF CACHE BOOL "Disable building ANTLR4 tests" FORCE)
  set(ANTLR_BUILD_SHARED OFF CACHE BOOL "Disable building ANTLR4 shared library" FORCE)

  FetchContent_MakeAvailable(fuzztest)

  fuzztest_setup_fuzzing_flags()
endif()

find_package(GTest REQUIRED)
enable_testing()
