add_subdirectory(utilities)
add_subdirectory(utilitiesSelfCheck)
add_subdirectory(msgPayloadDef)


# Build a linkable core library for C++ modules
add_library(xmera_core SHARED
  _GeneralModuleFiles/sys_model.cpp
  system_model/sim_model.cpp
  system_model/sys_model_task.cpp
  system_model/sys_process.cpp
  utilities/bskLogging.cpp
)

target_link_libraries(xmera_core PUBLIC
  Eigen3::Eigen
  ArchitectureUtilities
)

# TODO: Organize per-artifact includes in a sane way.
# _For now_, making each artifact be explicit abouts its in-tree includes
# keeps things explicit.
target_include_directories(xmera_core PRIVATE
  # Source includes
  # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
  "${CMAKE_CURRENT_SOURCE_DIR}"
  # Project-wide includes
  "${CMAKE_SOURCE_DIR}"
)

install(TARGETS xmera_core DESTINATION lib)

add_library(Xmera::Core ALIAS xmera_core)


add_subdirectory(system_model)
