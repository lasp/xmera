add_library(fswAlgorithmsAttGuidanceCommon STATIC)

target_sources(fswAlgorithmsAttGuidanceCommon
  PRIVATE
    constrainedAxisPointingLibrary.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    constrainedAxisPointingLibrary.h
)

target_link_libraries(fswAlgorithmsAttGuidanceCommon PUBLIC
  Xmera::Core
)

install(TARGETS fswAlgorithmsAttGuidanceCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
