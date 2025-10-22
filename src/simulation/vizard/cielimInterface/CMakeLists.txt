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

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/cielimMessage.pb.cc"
  cielimInterface.cpp
  zmqConnector.cpp
)

target_include_directories("${module}" PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
)

target_link_libraries("${module}" PRIVATE
  Xmera::Messaging
  cppzmq
  protobuf::libprotobuf
)

install(
  FILES cielimMessage.proto
  DESTINATION protobuf/
)
