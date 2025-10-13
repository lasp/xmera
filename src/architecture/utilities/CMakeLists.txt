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
  Xmera::Core
  Eigen3::Eigen
)

target_include_directories(ArchitectureUtilities PRIVATE
  "${CMAKE_SOURCE_DIR}"
)

install(TARGETS ArchitectureUtilities DESTINATION lib)


xmera_add_swig_module(architecture.orbitalMotion)
target_sources(architecture.orbitalMotion PRIVATE orbitalMotion.cpp)
target_link_libraries(architecture.orbitalMotion PUBLIC Xmera::Core Eigen3::Eigen)

if(XMERA_ENABLE_INTERNAL)
  xmera_add_swig_module(architecture.BSpline)
  target_sources(architecture.BSpline PRIVATE BSpline.cpp)
  target_link_libraries(architecture.BSpline PUBLIC Eigen3::Eigen)
endif()

add_subdirectory(tests)
