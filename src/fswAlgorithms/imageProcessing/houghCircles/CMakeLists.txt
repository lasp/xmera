set(module "fswAlgorithms.imageProcessing.houghCircles")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

find_package(OpenCV REQUIRED)

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  houghCircles.cpp
)

target_link_libraries("${module}" PRIVATE
  opencv_imgcodecs
)

# Suppress OpenCV warnings on clang due to its vcpkg distribution
# gcc doesn't emit such warnings
target_link_options("${module}" PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-Wl,-no_warn_duplicate_libraries>"
)
