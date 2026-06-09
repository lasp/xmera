# filteringCore — framework-agnostic Kalman filter infrastructure
#
# Header-only INTERFACE library. Depends only on Eigen and the C++ standard
# library — no `architecture/`, no `simulation/`, no msgPayload types. The
# boundary is intentional: code that links this library can be ported to any
# host (xmera, fp32-fsw-xmera + Adamant, etc.) without algorithm rewrites.

add_library(filteringCore INTERFACE)
add_library(xmera::filteringCore ALIAS filteringCore)

target_include_directories(filteringCore
  INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(filteringCore
  INTERFACE
    Eigen3::Eigen
)

target_compile_features(filteringCore
  INTERFACE
    cxx_std_23
)

add_subdirectory(_tests)
