set(module "fswAlgorithms.imageProcessing.fpgaImagePipeline")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(OpenCV REQUIRED)

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  fpgaImagePipeline.cpp
)

target_link_libraries("${module}" PRIVATE
  opencv_imgcodecs
)

# Suppress duplicate library warnings from OpenCV's vcpkg distribution on macOS
target_link_options("${module}" PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-Wl,-no_warn_duplicate_libraries>"
)
