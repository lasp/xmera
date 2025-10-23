add_library(fswAlgorithmsAttGuidanceCommon OBJECT
  constrainedAxisPointingLibrary.cpp
)

target_include_directories(fswAlgorithmsAttGuidanceCommon PRIVATE
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
)

target_link_libraries(fswAlgorithmsAttGuidanceCommon PUBLIC
  Xmera::Core
)
