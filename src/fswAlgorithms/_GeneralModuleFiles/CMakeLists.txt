add_library(fswAlgorithmsCommon OBJECT
  ekfInterface.cpp
  srukfInterface.cpp
  stateModels.cpp
)

target_include_directories(fswAlgorithmsCommon
  PRIVATE
    "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(fswAlgorithmsCommon PUBLIC
  Xmera::Core
  Eigen3::Eigen
)
