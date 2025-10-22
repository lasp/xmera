add_library(onboardDataHandlingCommon STATIC)

target_sources(onboardDataHandlingCommon
  PRIVATE
    dataNodeBase.cpp
    dataStorageUnitBase.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    dataNodeBase.h
    dataStorageUnitBase.h
)

target_link_libraries(onboardDataHandlingCommon PUBLIC
  Xmera::Core
  Xmera::Messaging
  ArchitectureUtilities
  Eigen3::Eigen
)

install(TARGETS onboardDataHandlingCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
