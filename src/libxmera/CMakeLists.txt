add_library(xmera_core SHARED)
add_library(Xmera::Core ALIAS xmera_core)

target_sources(xmera_core
  PRIVATE
    implementation/bskLogging.cpp
    implementation/sim_model.cpp
    implementation/sys_model.cpp
    implementation/sys_process.cpp
    implementation/sys_model_task.cpp

  PRIVATE FILE_SET internal_headers
    TYPE HEADERS
    BASE_DIRS implementation/

  PUBLIC FILE_SET interface_headers
    TYPE HEADERS
    BASE_DIRS interface/
    FILES
      interface/xmera/bskLogging.h
      interface/xmera/bskSemaphore.h
      interface/xmera/sim_model.h
      interface/xmera/sys_model.h
      interface/xmera/sys_model_task.h
      interface/xmera/sys_process.h
)

install(
  TARGETS xmera_core
    DESTINATION lib
  FILE_SET interface_headers
    DESTINATION include
)
install(DIRECTORY documentation/ DESTINATION doc)
install(DIRECTORY python/ DESTINATION Basilisk)


xmera_add_swig_module(architecture.sim_model
  INTERFACE swig/xmera/sim_model.i
  PROVIDES swig/
)

target_link_libraries(architecture.sim_model PRIVATE
  Xmera::Core
  Eigen3::Eigen
)
