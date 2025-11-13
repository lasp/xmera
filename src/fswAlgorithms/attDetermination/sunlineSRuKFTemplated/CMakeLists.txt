set(module "fswAlgorithms.attDetermination.sunlineSRuKFTemplated")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  sunlineSRuKFTemplated.cpp
  SRUnscentedKalmanFilter.hpp
  KalmanFilterBase.hpp
  StaticFilterTraits.hpp
  StaticProcessModel.hpp
  StaticMeasurementModel.hpp
  StaticKalmanBase.hpp
  GyroModel.hpp
  IMeasurement.hpp
  Measurement.hpp
  ObservationModel.hpp
  State.hpp
)

target_link_libraries("${module}" PRIVATE
  ArchitectureUtilities
  Eigen3::Eigen
)
