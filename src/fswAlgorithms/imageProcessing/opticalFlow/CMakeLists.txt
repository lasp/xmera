set(module "fswAlgorithms.imageProcessing.opticalFlow")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(OpenCV REQUIRED)

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  opticalFlow.cpp
)

target_link_libraries("${module}" PRIVATE
  ArchitectureUtilities
  opencv_imgcodecs
  opencv_video
)

target_link_options("${module}" PRIVATE
  # Suppress OpenCV warnings due to its vcpkg distribution
  "-Wl,-no_warn_duplicate_libraries"
)
