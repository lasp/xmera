set(module "fswAlgorithms.dvGuidance.dvGuidance")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  dvGuidance.cpp
)

target_link_libraries("${module}" PRIVATE
  ArchitectureUtilities
)
