set(module "fswAlgorithms.effectorInterfaces.dvAttEffect")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    dvAttEffect.cpp
  )
endif()

set(module "fswAlgorithms.effectorInterfaces.sunSafeACS")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    sunSafeACS.cpp
    dvAttEffect.cpp
  )
endif()
