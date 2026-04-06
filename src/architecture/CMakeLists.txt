# Build a linkable core library for C++ modules
add_library(xmera_core SHARED
  _GeneralModuleFiles/sys_model.cpp
  utilities/bskLogging.cpp
)

# On Windows, automatically export all symbols to generate the .lib import library
set_target_properties(xmera_core PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

# Generate export header for proper DLL symbol visibility on Windows
include(GenerateExportHeader)
generate_export_header(xmera_core
  BASE_NAME XMERA_CORE
  EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/xmera_core_export.h"
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

target_include_directories(xmera_core PUBLIC
  # Generated export header
  "${CMAKE_BINARY_DIR}"
)

install(TARGETS xmera_core
  RUNTIME DESTINATION xmera   # .dll on Windows
  LIBRARY DESTINATION lib     # .so on Linux/macOS
  ARCHIVE DESTINATION lib     # .lib/.a import libraries
)

add_library(Xmera::Core ALIAS xmera_core)


add_subdirectory(_GeneralModuleFiles)
add_subdirectory(utilities)
add_subdirectory(system_model)
add_subdirectory(msgPayloadDef)

add_subdirectory(utilitiesSelfCheck)
