option(XMERA_ENABLE_FUZZTESTS "Fetch and build fuzz targets with Google FuzzTest" OFF)

if(XMERA_ENABLE_FUZZTESTS)
  include(FetchContent)

  set(XMERA_FUZZTEST_GIT_TAG "2026-02-18"
    CACHE STRING
    "Git tag or commit to use for the Google FuzzTest dependency"
  )

  FetchContent_Declare(
    fuzztest
    GIT_REPOSITORY https://github.com/google/fuzztest.git
    GIT_TAG ${XMERA_FUZZTEST_GIT_TAG}   # or a specific commit/tag
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

# Give each test found by gtest_discover_tests more than one CTest label.
#
# The PROPERTIES option of gtest_discover_tests cannot do this. It sends the properties
# through a command line and expands them again without quotation marks, thus a
# "fuzz;fuzz-smoke" pair becomes the two arguments "LABELS fuzz fuzz-smoke". That leaves an
# odd number of tokens, and the second label never becomes a label.
#
# gtest_discover_tests finds the test names only after the binary is built, thus the labels
# must be applied when ctest runs. CTest includes the files in the TEST_INCLUDE_FILES
# directory property in order, and gtest_discover_tests appends its generated file to that
# property. Thus the file written here is included after it, and the list of names is
# available.
#
# Give the gtest_discover_tests call a TEST_LIST, and call this function after it and in the
# same directory. The names of the labels come after the two first arguments.
function(xmera_label_discovered_tests TARGET LIST_VAR)
  set(_script "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_labels.cmake")
  # ${LIST_VAR} expands now, to the name of the variable. \${${LIST_VAR}} writes that name
  # into the file, thus ctest expands it later. set_tests_properties replaces the labels, it
  # does not add to them, thus this call must give all of them.
  file(WRITE "${_script}"
    "if(${LIST_VAR})\n"
    "  set_tests_properties(\${${LIST_VAR}} PROPERTIES LABELS \"${ARGN}\")\n"
    "endif()\n"
  )
  set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES "${_script}")
endfunction()
