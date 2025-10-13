xmera_add_swig_module(architecture.sim_model)

target_link_libraries(architecture.sim_model PRIVATE
  Eigen3::Eigen
)
