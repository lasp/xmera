# Build a linkable core library for C++ modules
add_library(core SHARED)
add_library(Xmera::Core ALIAS core)

target_sources(core
  PRIVATE
    _GeneralModuleFiles/sys_model.cpp
    system_model/sim_model.cpp
    system_model/sys_model_task.cpp
    system_model/sys_process.cpp
    utilities/bskLogging.cpp

  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    _GeneralModuleFiles/sys_model.h
    messaging/messaging.h
    messaging/msgHeader.h
    system_model/bskSemaphore.h
    system_model/sim_model.h
    system_model/sys_model_task.h
    system_model/sys_process.h
    utilities/bskLogging.h
)

install(TARGETS core
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
)

install(
  FILES
    _GeneralModuleFiles/sys_model.i
    _GeneralModuleFiles/cSysModel.i
    _GeneralModuleFiles/swig_conly_data.i
    _GeneralModuleFiles/swig_eigen.i
    _GeneralModuleFiles/swig_deprecated.i
    _GeneralModuleFiles/swig_common_model.i
  DESTINATION include/architecture/_GeneralModuleFiles
)


add_subdirectory(utilities)
add_subdirectory(system_model)
add_subdirectory(messaging)

add_subdirectory(utilitiesSelfCheck)
