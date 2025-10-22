add_library(powerCommon STATIC)

target_sources(powerCommon
  PRIVATE
    powerNodeBase.cpp
    powerStorageBase.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    powerNodeBase.h
    powerStorageBase.h
)

target_link_libraries(powerCommon PUBLIC
  Xmera::Core
  Xmera::Messaging
  ArchitectureUtilities
  Eigen3::Eigen
)

install(TARGETS powerCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
