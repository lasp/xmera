cmake_policy(PUSH)
cmake_minimum_required(VERSION 3.27)

if(APPLE)
  set(XMERA_RPATH_ORIGIN "@loader_path")
else()
  set(XMERA_RPATH_ORIGIN "$ORIGIN")
endif()

define_property(SOURCE PROPERTY XMERA_SWIG_MESSAGE_TEMPLATE BRIEF_DOCS
  "The path to the templated .i file to use when generating a SWIG interface for this file"
)

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
    xmera::core
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

function(xmera_add_swig_message message header)
  get_property(_template SOURCE "${header}" PROPERTY XMERA_SWIG_MESSAGE_TEMPLATE)
  if(NOT (DEFINED _template))
    set(_template "${XMERA_DIR}/usr/share/xmera/msgInterfacePy.i.in")
  endif()

  add_custom_command(
    COMMENT "Generating SWIG message interface: ${message}"
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
    COMMAND
      "${XMERA_DIR}/bin/generateSWIGModules.py"
      "${CMAKE_CURRENT_BINARY_DIR}/${message}.i"
      "${_template}"
      "${message}"
      "${header}"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}"
    MAIN_DEPENDENCY
      "${header}"
    DEPENDS
      "${XMERA_DIR}/bin/generateSWIGModules.py"
      "${_template}"
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
    xmera::core
  )

  set_target_properties("${message}" PROPERTIES
    PREFIX "_"
    OUTPUT_NAME "${message}"
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

cmake_policy(POP)
