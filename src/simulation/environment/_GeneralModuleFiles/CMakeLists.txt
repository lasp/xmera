add_library(environmentCommon STATIC)

target_sources(environmentCommon
  PRIVATE
    atmosphereBase.cpp
    magneticFieldBase.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    atmosphereBase.h
    magneticFieldBase.h
)

target_link_libraries(environmentCommon PUBLIC
  Xmera::Core
  Xmera::Messaging
  ArchitectureUtilities
  Eigen3::Eigen
)

install(TARGETS environmentCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
