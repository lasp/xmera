if(XMERA_ENABLE_INTERNAL)
  xmera_add_swig_module(architecture.swigDeprecatedCheck)

  xmera_add_swig_module(architecture.swigEigenCheck)
  target_link_libraries(architecture.swigEigenCheck PRIVATE Eigen3::Eigen)
endif()
