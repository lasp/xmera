add_library(dynamicsCommon STATIC)

target_sources(dynamicsCommon
  PRIVATE
    dynParamManager.cpp
    dynamicEffector.cpp
    dynamicObject.cpp
    extendedStateVector.cpp
    gravityEffector.cpp
    hubEffector.cpp
    pointMassGravityModel.cpp
    stateData.cpp
    stateEffector.cpp
    stateVecIntegrator.cpp
    svIntegratorRK4.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    gravityModel.h
    THRSimConfig.h
    stateVecIntegrator.h
    stateData.h
    pointMassGravityModel.h
    svIntegratorRungeKutta.h
    THROperation.h
    BodyToHubInfo.h
    THRTimePair.h
    dynamicObject.h
    svIntegratorAdaptiveRungeKutta.h
    dynParamManager.h
    dynamicEffector.h
    hubEffector.h
    gravityEffector.h
    stateEffector.h
    svIntegratorRK4.h
    extendedStateVector.h
    fuelSlosh.h
)

target_link_libraries(dynamicsCommon PUBLIC
  Xmera::Messaging
  ArchitectureUtilities
  Eigen3::Eigen
)

install(TARGETS dynamicsCommon
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)
