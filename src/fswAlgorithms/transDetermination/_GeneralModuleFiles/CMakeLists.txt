add_library(fswAlgorithmsTransDeterminationCommon STATIC)

target_sources(fswAlgorithmsTransDeterminationCommon
  PRIVATE
    ephemerisUtilities.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    ephemerisUtilities.h
)

install(TARGETS fswAlgorithmsTransDeterminationCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
