set(module "simulation.onboardDataHandling.partitionedStorageUnit")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    partitionedStorageUnit.cpp
  )

  target_link_libraries("${module}" PRIVATE
    onboardDataHandlingCommon
  )
endif()

set(module "simulation.onboardDataHandling.simpleStorageUnit")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    simpleStorageUnit.cpp
  )

  target_link_libraries("${module}" PRIVATE
    onboardDataHandlingCommon
  )
endif()
