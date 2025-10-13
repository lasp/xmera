add_library(ArchitectureUtilities STATIC
  signalCondition.c
  ukfUtilities.c
  orbitalMotion.c
  linearAlgebra.c
  safeMath.c
  rigidBodyKinematics.c
  svd.c

  orbitalMotion.cpp
  saturate.cpp
  geodeticConversion.cpp
  signalProcessing.cpp
  discretize.cpp
  gauss_markov.cpp
  BSpline.cpp
  keplerianOrbit.cpp
)

target_link_libraries(ArchitectureUtilities PUBLIC
  Eigen3::Eigen
  Xmera::Core
)

target_include_directories(ArchitectureUtilities PRIVATE
  "${CMAKE_SOURCE_DIR}"
)

set_target_properties(ArchitectureUtilities PROPERTIES C_EXTENSIONS ON)

install(TARGETS ArchitectureUtilities DESTINATION lib)

xmera_add_swig_module(architecture.bskLogging)

xmera_add_swig_module(architecture.orbitalMotion)
target_sources(architecture.orbitalMotion PRIVATE orbitalMotion.cpp)

if(XMERA_ENABLE_INTERNAL)
  xmera_add_swig_module(architecture.BSpline)
  target_sources(architecture.BSpline PRIVATE BSpline.cpp)
endif()

add_subdirectory(tests)
