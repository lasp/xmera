set(module "simulation.cielim.cielimInterface")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(cppzmq CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)

add_custom_command(
  COMMENT "Generating protobuf (C++/Python): cielimMessage"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/cielimMessage.proto"
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/cielimMessage.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/cielimMessage.pb.h"
    "${CMAKE_CURRENT_BINARY_DIR}/cielimMessage_pb2.py"
  COMMAND
    "${Protobuf_PROTOC_EXECUTABLE}"
    "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}"
    "--cpp_out=${CMAKE_CURRENT_BINARY_DIR}"
    "--python_out=${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cielimMessage.proto"
  VERBATIM
  DEPENDS_EXPLICIT_ONLY
)

add_custom_command(
  COMMENT "Generating protobuf (C++/Python): imageDiagnostics"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/imageDiagnostics.proto"
  OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/imageDiagnostics.pb.cc"
    "${CMAKE_CURRENT_BINARY_DIR}/imageDiagnostics.pb.h"
    "${CMAKE_CURRENT_BINARY_DIR}/imageDiagnostics_pb2.py"
  COMMAND
    "${Protobuf_PROTOC_EXECUTABLE}"
    "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}"
    "--cpp_out=${CMAKE_CURRENT_BINARY_DIR}"
    "--python_out=${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/imageDiagnostics.proto"
  VERBATIM
  DEPENDS_EXPLICIT_ONLY
)

# The SWIG Python module and the C++ tests both link this library.
add_library(cielimInterfaceCore STATIC
  "${CMAKE_CURRENT_BINARY_DIR}/cielimMessage.pb.cc"
  "${CMAKE_CURRENT_BINARY_DIR}/imageDiagnostics.pb.cc"
  cielimInterface.cpp
  zmqConnector.cpp
)

target_include_directories(cielimInterfaceCore PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
)

# The mission parameters directory comes first, as in xmera_add_swig_module().
target_include_directories(cielimInterfaceCore BEFORE PUBLIC
  "$<TARGET_PROPERTY:Xmera::MissionParameters,INTERFACE_INCLUDE_DIRECTORIES>"
)

target_link_libraries(cielimInterfaceCore PUBLIC
  Xmera::Core
  ArchitectureUtilities
  cppzmq
  protobuf::libprotobuf
)

xmera_add_swig_module("${module}")

target_link_libraries("${module}" PRIVATE
  cielimInterfaceCore
)

add_subdirectory(tests)

install(
  FILES cielimMessage.proto
  FILES imageDiagnostics.proto
  DESTINATION protobuf/
)
