add_library(onboardDataHandlingCommon OBJECT
  dataNodeBase.cpp
  dataStorageUnitBase.cpp
)

target_include_directories(onboardDataHandlingCommon PRIVATE
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
)

target_link_libraries(onboardDataHandlingCommon PUBLIC
  Xmera::Core
  Eigen3::Eigen
)
