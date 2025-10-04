add_library(environmentCommon OBJECT
  atmosphereBase.cpp
  magneticFieldBase.cpp
)

target_include_directories(environmentCommon PRIVATE
  "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(environmentCommon PUBLIC
  Xmera::Core
  Eigen3::Eigen
)
