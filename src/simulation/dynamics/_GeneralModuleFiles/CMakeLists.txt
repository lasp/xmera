add_library(dynamicsCommon OBJECT
  dynParamManager.cpp
  dynamicObject.cpp
  gravityEffector.cpp
  hubEffector.cpp
  pointMassGravityModel.cpp
  stateData.cpp
)

target_include_directories(dynamicsCommon PRIVATE
  # Project-wide includes
  "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(dynamicsCommon PUBLIC
  Eigen3::Eigen
)


xmera_is_module_enabled("simulation.dynamics.svIntegrators" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("simulation.dynamics.svIntegrators")

target_link_libraries("simulation.dynamics.svIntegrators" PRIVATE
  ArchitectureUtilities
  dynamicsCommon
)
