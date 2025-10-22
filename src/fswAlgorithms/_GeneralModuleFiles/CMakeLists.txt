add_library(fswAlgorithmsCommon STATIC)

target_sources(fswAlgorithmsCommon
  PRIVATE
    dynamicModels.cpp
    ekfInterface.cpp
    kalmanFilter.cpp
    measurementModels.cpp
    srukfInterface.cpp
    stateModels.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    dynamicModels.h
    ekfInterface.h
    filterInterfaceDefinitions.h
    kalmanFilter.h
    measurementModels.h
    srukfInterface.h
    stateModels.h
    ../fswUtilities/fswDefinitions.h
)

target_link_libraries(fswAlgorithmsCommon PUBLIC
  Xmera::Core
  Xmera::Messaging
  ArchitectureUtilities
  Eigen3::Eigen
)

install(TARGETS fswAlgorithmsCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
