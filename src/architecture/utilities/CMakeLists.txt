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
  bskLogging.cpp
  gauss_markov.cpp
  BSpline.cpp
  keplerianOrbit.cpp
)

target_link_libraries(ArchitectureUtilities PUBLIC
  Eigen3::Eigen
)

target_include_directories(ArchitectureUtilities PRIVATE
  "${CMAKE_SOURCE_DIR}"
)

set_target_properties(ArchitectureUtilities PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk"
)

install(TARGETS ArchitectureUtilities DESTINATION lib)


add_library(ModuleIdGenerator SHARED
  moduleIdGenerator/moduleIdGenerator.cpp
)

set_target_properties(ModuleIdGenerator PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk"
)

install(TARGETS ModuleIdGenerator DESTINATION lib)


xmera_add_swig_module(architecture.bskLogging)
xmera_add_swig_module(architecture.bskUtilities)

xmera_add_swig_module(architecture.orbitalMotion)
target_sources(architecture.orbitalMotion PRIVATE orbitalMotion.cpp)

xmera_add_swig_module(architecture.BSpline)
target_sources(architecture.BSpline PRIVATE BSpline.cpp)

add_subdirectory(tests)
