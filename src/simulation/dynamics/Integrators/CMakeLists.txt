set(module "simulation.dynamics.svIntegrators")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  svIntegratorEuler.cpp
  svIntegratorRK2.cpp
  svIntegratorRKF45.cpp
  svIntegratorRKF78.cpp
)

target_link_libraries("${module}" PRIVATE
  ArchitectureUtilities
  dynamicsCommon
)
