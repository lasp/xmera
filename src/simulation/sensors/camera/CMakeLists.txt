set(module "simulation.sensors.camera")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  camera.cpp
)

target_link_libraries("${module}" PRIVATE
  opencv_imgcodecs
)

target_link_options("${module}" PRIVATE
  # Suppress OpenCV warnings due to its vcpkg distribution
  "-Wl,-no_warn_duplicate_libraries"
)
