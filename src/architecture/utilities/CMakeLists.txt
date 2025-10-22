add_library(ArchitectureUtilities STATIC)

target_sources(ArchitectureUtilities
  PRIVATE
    linearAlgebra.c
    orbitalMotion.c
    rigidBodyKinematics.c
    safeMath.c
    signalCondition.c
    svd.c
    ukfUtilities.c

    BSpline.cpp
    discretize.cpp
    gauss_markov.cpp
    geodeticConversion.cpp
    keplerianOrbit.cpp
    orbitalMotion.cpp
    saturate.cpp
    signalProcessing.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    BSpline.h
    astroConstants.h
    bilinearInterpolation.hpp
    discretize.h
    eigenMRP.h
    eigenSupport.h
    gauss_markov.h
    geodeticConversion.h
    keplerianOrbit.h
    linearAlgebra.h
    linearInterpolation.hpp
    macroDefinitions.h
    orbitalMotion.h
    orbitalMotion.hpp
    rigidBodyKinematics.h
    rigidBodyKinematics.hpp
    safeMath.h
    saturate.h
    signalCondition.h
    signalProcessing.h
    simDefinitions.h
    svd.h
    ukfUtilities.h
)

target_link_libraries(ArchitectureUtilities PUBLIC
  Eigen3::Eigen
  Xmera::Core
)

set_target_properties(ArchitectureUtilities PROPERTIES C_EXTENSIONS ON)

install(TARGETS ArchitectureUtilities
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)

xmera_add_swig_module(architecture.orbitalMotion)
target_sources(architecture.orbitalMotion PRIVATE orbitalMotion.cpp)
target_link_libraries(architecture.orbitalMotion PUBLIC Eigen3::Eigen)

if(XMERA_ENABLE_INTERNAL)
  xmera_add_swig_module(architecture.BSpline)
  target_sources(architecture.BSpline PRIVATE BSpline.cpp)
  target_link_libraries(architecture.BSpline PUBLIC Eigen3::Eigen)
endif()

add_subdirectory(tests)
