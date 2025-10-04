set(module "simulation.environment.spiceInterface")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(cspice CONFIG REQUIRED)

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  spiceInterface.cpp
)

target_link_libraries("${module}" PRIVATE
  cspice::cspice
)
