add_library(dynamicsCommon OBJECT
  dynParamManager.cpp
  dynamicEffector.cpp
  dynamicObject.cpp
  extendedStateVector.cpp
  gravityEffector.cpp
  hubEffector.cpp
  pointMassGravityModel.cpp
  stateData.cpp
  stateVecIntegrator.cpp
  svIntegratorRK4.cpp
)

target_include_directories(dynamicsCommon PRIVATE
  # Project-wide includes
  "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(dynamicsCommon PUBLIC
  Eigen3::Eigen
)
