add_library(powerCommon OBJECT
  powerNodeBase.cpp
  powerStorageBase.cpp
)

target_include_directories(powerCommon PRIVATE
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
)

target_link_libraries(powerCommon PUBLIC
  Xmera::Core
  Eigen3::Eigen
)
