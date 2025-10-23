# Root directories to add via `add_subdirectory()`
set(XMERA_MODULE_ROOTS
  "${CMAKE_SOURCE_DIR}/fswAlgorithms/"
  "${CMAKE_SOURCE_DIR}/simulation/"
  CACHE STRING
  "Semicolon-separated roots containing modules to add to the build. Each directory must contain a CMakeLists.txt."
)

set(XMERA_ENABLE_GROUPS ""
  CACHE STRING
  "Semicolon-separated group names or globs to enable."
)

set(XMERA_ENABLE_INTERNAL "NO"
  CACHE STRING
  "Whether to enable modules that are marked as INTERNAL (default NO)"
)

if(APPLE)
  set(XMERA_RPATH_ORIGIN "@loader_path")
else()
  set(XMERA_RPATH_ORIGIN "$ORIGIN")
endif()

define_property(GLOBAL PROPERTY XMERA_REGISTERED_MESSAGES BRIEF_DOCS
  "An accumulated list of messages registered via xmera_add_swig_message()"
)

function(_xmera_is_prefix prefix value out_var)
  set("${out_var}" OFF PARENT_SCOPE)

  string(LENGTH "${prefix}" _prefix_len)
  string(LENGTH "${value}" _value_len)
  if(_prefix_len LESS_EQUAL _value_len)
    string(SUBSTRING "${value}" 0 ${_prefix_len} _candidate)
    if(_candidate STREQUAL "${prefix}")
      set("${out_var}" ON PARENT_SCOPE)
    endif()
  endif()
endfunction()

function(xmera_is_module_enabled module_path out_var)
  cmake_parse_arguments(PARSE_ARGV 2 arg "INTERNAL" "" "")

  set("${out_var}" OFF PARENT_SCOPE)

  if((NOT arg_INTERNAL) OR XMERA_ENABLE_INTERNAL)
    foreach(_prefix IN LISTS XMERA_ENABLE_GROUPS)
      _xmera_is_prefix("${_prefix}." "${module_path}." _result)
      if(_result)
        set("${out_var}" "${_result}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
  endif()
endfunction()

function(xmera_add_swig_module module)
  # separate "a.b.c" into "a" and "c"
  #  ... yes, ignore intermediate packages. everything just collapses. :(
  # TODO: separate "a.b.c" into "a/b" and "c"
  string(REPLACE "." ";" _package_components "${module}")
  list(POP_BACK _package_components _module_basename)
  list(POP_FRONT _package_components _package_path)
  # list(JOIN _package_components "/" _package_path)

  set(_gen_target_includes "$<TARGET_PROPERTY:${module},INCLUDE_DIRECTORIES>")
  set(_gen_swig_include_flags "$<LIST:TRANSFORM,${_gen_target_includes},PREPEND,-I>")
  add_custom_command(
    COMMENT "Generating SWIG Python/C++ wrapper: ${module}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.cxx"
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}.py"
    COMMAND
      "${SWIG_EXECUTABLE}"
      -python
      -c++
      -MD
      -outcurrentdir
      "${_gen_swig_include_flags}"
      "${CMAKE_CURRENT_SOURCE_DIR}/${_module_basename}.i"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_BINARY_DIR}"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_SOURCE_DIR}/${_module_basename}.i"
    DEPFILE
      "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.d"
    VERBATIM
    COMMAND_EXPAND_LISTS
    DEPENDS_EXPLICIT_ONLY
  )

  add_library("${module}" MODULE
    "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}_wrap.cxx"
  )

  target_include_directories("${module}" PRIVATE
    # Source includes
    # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
    "${CMAKE_CURRENT_SOURCE_DIR}"
    # Project-wide includes
    "${CMAKE_SOURCE_DIR}"
    # Generated project-wide includes
    "${CMAKE_BINARY_DIR}"
    # @TODO add architecture/_GeneralModuleFiles to a global interface target or similar
    "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
  )

  target_link_libraries("${module}" PRIVATE
    Xmera::Core
    Python3::Module
  )

  # TODO: once we actually use the full package path to install modules,
  #   we need to use the following logic to ensure that the relative rpath
  #   is correct no matter how deeply nested the module is.
  #
  # # "a.b.c" -> "Basilisk/a/b/_c.so" -> "../../../lib"
  # string(REGEX MATCHALL [[\.]] _rpath "${module}")
  # # _rpath = ".;."
  # list(APPEND _rpath ".")
  # # _rpath = ".;.;."
  # list(TRANSFORM _rpath PREPEND ".")
  # # _rpath = "..;..;.."
  # list(JOIN _rpath "/" _rpath)
  # # _rpath = "../../.."
  # set(_rpath "${_rpath}/lib")
  # # _rpath = "../../../lib"
  set(_rpath "../../lib")

  set_target_properties("${module}" PROPERTIES
    PREFIX "_"
    OUTPUT_NAME "${_module_basename}"
    INSTALL_RPATH "${XMERA_RPATH_ORIGIN}/${_rpath}"
  )

  install(
    TARGETS "${module}"
    DESTINATION "Basilisk/${_package_path}"
  )
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${_module_basename}.py"
    DESTINATION "Basilisk/${_package_path}"
  )
endfunction()

function(xmera_add_swig_message message)
  set_property(GLOBAL APPEND PROPERTY XMERA_REGISTERED_MESSAGES "${message}")

  add_custom_command(
    COMMENT "Generating SWIG message interface: ${message}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/generateSWIGModules.py"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
      "${CMAKE_CURRENT_SOURCE_DIR}/${message}.h"
      "${message}"
      "${CMAKE_CURRENT_SOURCE_DIR}"
    WORKING_DIRECTORY
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_SOURCE_DIR}/${message}.h"
    DEPENDS
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/generateSWIGModules.py"
      "${CMAKE_SOURCE_DIR}/architecture/messaging/msgAutoSource/msgInterfacePy.i.in"
    VERBATIM
    DEPENDS_EXPLICIT_ONLY
  )

  set(_gen_target_includes "$<TARGET_PROPERTY:${message},INCLUDE_DIRECTORIES>")
  set(_gen_swig_include_flags "$<LIST:TRANSFORM,${_gen_target_includes},PREPEND,-I>")
  add_custom_command(
    COMMENT "Generating SWIG Python/C++ wrapper: ${message}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.cxx"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.py"
    COMMAND
      "${SWIG_EXECUTABLE}"
      -python
      -c++
      -MD
      -outcurrentdir
      "${_gen_swig_include_flags}"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_BINARY_DIR}"
    MAIN_DEPENDENCY
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    DEPFILE
      "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.d"
    VERBATIM
    COMMAND_EXPAND_LISTS
    DEPENDS_EXPLICIT_ONLY
  )

  add_library("${message}" MODULE
    "${CMAKE_CURRENT_BINARY_DIR}/${message}_wrap.cxx"
  )

  target_include_directories("${message}" PRIVATE
    # Source includes
    # @TODO: All local includes should ultimately be made relative to CMAKE_SOURCE_DIR.
    "${CMAKE_CURRENT_SOURCE_DIR}"
    # Project-wide includes
    "${CMAKE_SOURCE_DIR}"
  )

  target_link_libraries("${message}" PRIVATE
    Python3::Module
    Eigen3::Eigen
    Xmera::Core
  )

  set_target_properties("${message}" PROPERTIES
    PREFIX "_"
    OUTPUT_NAME ${message}
    INSTALL_RPATH "${XMERA_RPATH_ORIGIN}/../../../lib"
  )

  install(
    TARGETS "${message}"
    DESTINATION "Basilisk/architecture/messaging"
  )
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${message}.py"
    DESTINATION "Basilisk/architecture/messaging"
  )
endfunction()

function(xmera_generate_messaging_init)
  get_property(_file_contents GLOBAL PROPERTY XMERA_REGISTERED_MESSAGES)
  list(TRANSFORM _file_contents PREPEND "from Basilisk.architecture.messaging.")
  list(TRANSFORM _file_contents APPEND " import *")
  list(JOIN _file_contents "\n" _file_contents)

  file(GENERATE
    OUTPUT "${CMAKE_BINARY_DIR}/messaging__init__.py"
    CONTENT "${_file_contents}"
  )

  install(
    FILES "${CMAKE_BINARY_DIR}/messaging__init__.py"
    DESTINATION Basilisk/architecture/messaging/
    RENAME __init__.py
  )
endfunction()
